#define _USE_MATH_DEFINES
#include "RelayCore.h"
#include "FeedbackParser.h"
#include "SafetyBoundary.h"
#include "../robot/RobotConnection.h"
#include "../robot/Kinematics.h"
#include "../core/AppState.h"
#include "../config/Config.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <windows.h>
#include "../safety/SafetyPredictor.h"
#include "../safety/RobotDiagnostics.h"
#include "../safety/SelfCollision.h"
#include "../force/ForcePipeline.h"
#include "../force/ForceCompensation.h"
#include "../force/ForceCalibration.h"
#include "../safety/SingularityAvoidance.h"

// ===== 姿态安全边界钳位 =====
static Vec3 clampOrientToBounds(const Vec3& target) {
    Vec3 clamped = target;
    bool warned = false;

    if (target.x < Config::SAFE_RX_MIN) { clamped.x = Config::SAFE_RX_MIN; warned = true; }
    if (target.x > Config::SAFE_RX_MAX) { clamped.x = Config::SAFE_RX_MAX; warned = true; }
    if (target.y < Config::SAFE_RY_MIN) { clamped.y = Config::SAFE_RY_MIN; warned = true; }
    if (target.y > Config::SAFE_RY_MAX) { clamped.y = Config::SAFE_RY_MAX; warned = true; }
    if (target.z < Config::SAFE_RZ_MIN) { clamped.z = Config::SAFE_RZ_MIN; warned = true; }
    if (target.z > Config::SAFE_RZ_MAX) { clamped.z = Config::SAFE_RZ_MAX; warned = true; }

    if (warned) {
        std::cerr << "[Safety] Orientation target out of bounds, clamped. Original: ("
                  << target.x << "," << target.y << "," << target.z << ")" << std::endl;
    }
    return clamped;
}

// ===== ForceReader 线程: 阻塞读取 30004 实时力数据 (125Hz) =====
static DWORD WINAPI forceReaderThread(LPVOID) {
    auto& app = appState;
    std::cout << "[Force] Reader thread started, connecting to port "
              << Config::FORCE_REALTIME_PORT << "..." << std::endl;

    while (!app.isClosing) {
        if (!robotConnectRealtime(Config::ROBOT_IP)) {
            std::cerr << "[Force] Realtime port connect failed, retrying in "
                      << Config::FORCE_RECONNECT_INTERVAL << "ms..." << std::endl;
            Sleep(Config::FORCE_RECONNECT_INTERVAL);
            continue;
        }

        std::cout << "[Force] Reader thread receiving at 125Hz..." << std::endl;
        char buf[1440];

        while (!app.isClosing) {
            if (!robotRecvRealtime(buf, sizeof(buf))) {
                std::cerr << "[Force] Realtime recv failed, reconnecting..." << std::endl;
                break;  // reconnect loop
            }

            // Parse ActualTCPForce at offset 576 (6 doubles, 48 bytes)
            double* forcePtr = reinterpret_cast<double*>(buf + 576);
            EnterCriticalSection(&app.forceDataMutex);
            for (int i = 0; i < 6; i++) {
                app.forceData.raw[i] = forcePtr[i];
            }
            app.forceData.lastUpdateMs = GetTickCount();
            app.forceData.isStale = false;
            LeaveCriticalSection(&app.forceDataMutex);

            // 看门狗兜底: 每 300ms 检查一次 (GLUT 可能已死)
            static DWORD lastWatchdogCheck = 0;
            DWORD now = GetTickCount();
            if (now - lastWatchdogCheck > 300) {
                lastWatchdogCheck = now;
                auto& relay = RelayCore::instance();
                DWORD lastHaptic = relay.lastHapticFrameMs();
                if (relay.isTransmitting() && lastHaptic > 0 &&
                    (now - lastHaptic) > (DWORD)(Config::WATCHDOG_TIMEOUT_MS * 2)) {
                    std::cerr << "[Safety] ForceReader WATCHDOG: GLUT appears dead ("
                              << (now - lastHaptic) << "ms since last haptic frame) — "
                              << "sending EmergencyStop" << std::endl;
                    robotSendEnable("DisableRobot()");
                    Sleep(100);
                }
            }
        }

        robotCloseRealtime();
        if (!app.isClosing) {
            Sleep(Config::FORCE_RECONNECT_INTERVAL);
        }
    }

    std::cout << "[Force] Reader thread exiting" << std::endl;
    return 0;
}

RelayCore& RelayCore::instance() {
    static RelayCore inst;
    return inst;
}

RelayCore::RelayCore() {
    InitializeCriticalSection(&m_basePointLock);
    InitializeCriticalSection(&m_relaySocketMutex);
}

RelayCore::~RelayCore() {
    shutdownForceReader();
    shutdownRelayReporting();
    DeleteCriticalSection(&m_basePointLock);
    DeleteCriticalSection(&m_relaySocketMutex);
}

// ===== 奇异脱困: 检测报警 → 拖拽模式 → 等待手动挪动 → 重新使能 =====
static bool escapeSingularity() {
    // 防止重入
    static bool s_escaping = false;
    if (s_escaping) {
        std::cout << "[脱困] 已在脱困流程中，跳过重复触发" << std::endl;
        return false;
    }
    s_escaping = true;
    std::cout << "[脱困] 检测到报警，开始诊断..." << std::endl;

    char fb[256];

    // ===== 读取当前状态 (GetPose + GetAngle) =====
    robotDrainEnable();
    robotSendEnable("GetPose()");
    Sleep(100);
    AppState::RobotPose curPose;
    if (robotRecvEnable(fb, sizeof(fb))) {
        FeedbackParser::parsePose(fb, curPose);
        std::cout << "[脱困] 末端位姿: X=" << curPose.x << " Y=" << curPose.y
                  << " Z=" << curPose.z << " Rx=" << curPose.rx
                  << " Ry=" << curPose.ry << " Rz=" << curPose.rz << std::endl;
    }

    robotDrainEnable();
    robotSendEnable("GetAngle()");
    Sleep(100);
    double curJoints[6] = {0};
    if (robotRecvEnable(fb, sizeof(fb))) {
        FeedbackParser::parseAngle(fb, curJoints);
        std::cout << "[脱困] 关节角: J1=" << curJoints[0] << " J2=" << curJoints[1]
                  << " J3=" << curJoints[2] << " J4=" << curJoints[3]
                  << " J5=" << curJoints[4] << " J6=" << curJoints[5] << std::endl;
    }

    // 检查哪个关节接近限位 (容差 5°)
    struct { int id; double val, minV, maxV; const char* name; } limits[6] = {
        {1, curJoints[0], -360, 360, "J1"},
        {2, curJoints[1], -360, 360, "J2"},
        {3, curJoints[2], -155, 155, "J3"},
        {4, curJoints[3], -360, 360, "J4"},
        {5, curJoints[4], -360, 360, "J5"},
        {6, curJoints[5], -360, 360, "J6"},
    };

    int stuckJoint = -1;
    const char* stuckName = "";
    bool isUpperLimit = false;
    for (int i = 0; i < 6; i++) {
        if (limits[i].val >= limits[i].maxV - 5.0) {
            stuckJoint = limits[i].id;
            stuckName = limits[i].name;
            isUpperLimit = true;
            break;
        }
        if (limits[i].val <= limits[i].minV + 5.0) {
            stuckJoint = limits[i].id;
            stuckName = limits[i].name;
            isUpperLimit = false;
            break;
        }
    }

    if (stuckJoint == -1) {
        std::cout << "[脱困] 未检测到关节限位, 可能是其他原因" << std::endl;
        s_escaping = false;
        return false;
    }

    std::cout << "[脱困] 检测到 " << stuckName << " "
              << (isUpperLimit ? "正向" : "负向") << "限位 (当前 "
              << limits[stuckJoint-1].val << "°)" << std::endl;

    // Step 1: 强制进入拖拽模式 + 单独松问题关节抱闸
    std::cout << "[脱困] 进入强制拖拽模式..." << std::endl;
    robotSendEnable("SetCollideDrag(1)");
    Sleep(300);
    robotDrainEnable();
    if (robotRecvEnable(fb, sizeof(fb))) {
        std::cout << "[脱困] SetCollideDrag(1) 原始: " << fb;
    }

    // 单独松开问题关节的抱闸 (双保险)
    char brakeCmd[32];
    snprintf(brakeCmd, sizeof(brakeCmd), "BrakeControl(%d,1)", stuckJoint);
    std::cout << "[脱困] 单独松 " << stuckName << " 抱闸: " << brakeCmd << std::endl;
    robotSendEnable(brakeCmd);
    Sleep(200);
    robotDrainEnable();
    if (robotRecvEnable(fb, sizeof(fb))) {
        std::cout << "[脱困] " << brakeCmd << " 原始: " << fb;
    }

    // Step 2: 提示用户只动问题关节
    std::cout << "\n========================================" << std::endl;
    std::cout << "[脱困] 请将 " << stuckName << " 向"
              << (isUpperLimit ? "负方向(反向)" : "正方向(正向)")
              << "转动 20~30°!" << std::endl;
    std::cout << "[脱困] (其他关节不需要动)" << std::endl;
    std::cout << "[脱困] 转动完成后按 Enter 继续" << std::endl;
    std::cout << "========================================" << std::endl;

    // 等待
    for (int i = 0; i < 120; i++) {
        if (GetAsyncKeyState(VK_RETURN) & 0x8000) {
            while (GetAsyncKeyState(VK_RETURN) & 0x8000) Sleep(10);
            break;
        }
        if (i % 5 == 0 && i > 0) {
            std::cout << "[脱困] 等待中... (" << (120-i) << "s) 按 Enter" << std::endl;
        }
        Sleep(1000);
    }

    // Step 3: 验证 (GetPose + GetAngle)
    robotDrainEnable();
    robotSendEnable("GetPose()");
    Sleep(100);
    AppState::RobotPose newPose;
    if (robotRecvEnable(fb, sizeof(fb))) {
        FeedbackParser::parsePose(fb, newPose);
        std::cout << "[脱困] 拖动后末端: X=" << newPose.x << " Y=" << newPose.y
                  << " Z=" << newPose.z << " Rx=" << newPose.rx
                  << " Ry=" << newPose.ry << " Rz=" << newPose.rz << std::endl;
    }

    robotDrainEnable();
    robotSendEnable("GetAngle()");
    Sleep(100);
    double newJoints[6] = {0};
    if (robotRecvEnable(fb, sizeof(fb))) {
        FeedbackParser::parseAngle(fb, newJoints);
        std::cout << "[脱困] GetAngle原始: " << fb;
        std::cout << "[脱困] 拖动后关节: J1=" << newJoints[0] << " J2=" << newJoints[1]
                  << " J3=" << newJoints[2] << " J4=" << newJoints[3]
                  << " J5=" << newJoints[4] << " J6=" << newJoints[5] << std::endl;
    }

    double newVal = newJoints[stuckJoint - 1];
    if (isUpperLimit && newVal < limits[stuckJoint-1].maxV - 5.0) {
        std::cout << "[脱困] " << stuckName << " 已离开上限" << std::endl;
    } else if (!isUpperLimit && newVal > limits[stuckJoint-1].minV + 5.0) {
        std::cout << "[脱困] " << stuckName << " 已离开下限" << std::endl;
    } else {
        std::cout << "[脱困] " << stuckName << " 仍接近限位 ("
                  << newVal << "°)" << std::endl;
    }

    // Step 4: 退出拖拽 + 锁回问题关节
    robotSendEnable("SetCollideDrag(0)");
    Sleep(300);
    robotDrainEnable();
    snprintf(brakeCmd, sizeof(brakeCmd), "BrakeControl(%d,0)", stuckJoint);
    robotSendEnable(brakeCmd);
    Sleep(200);
    robotDrainEnable();

    // Step 5: 先独立尝试 ClearError (不使能)
    std::cout << "[脱困] 尝试 ClearError..." << std::endl;
    robotSendEnable("ClearError()");
    Sleep(300);
    robotDrainEnable();

    robotSendEnable("RobotMode()");
    Sleep(100);
    int mode = -1;
    if (robotRecvEnable(fb, sizeof(fb))) {
        FeedbackParser::parseMode(fb, mode);
        std::cout << "[脱困] ClearError后 mode=" << mode;
    }

    if (mode != 9 && mode != -1) {
        std::cout << "[脱困] ClearError 直接清除成功!" << std::endl;
        s_escaping = false;
        return true;
    }

    // Step 6: ClearError 不够, 需要 EnableRobot
    std::cout << "[脱困] 尝试 EnableRobot..." << std::endl;
    robotSendEnable("EnableRobot(0.5,0,0,0)");
    Sleep(300);
    robotDrainEnable();

    robotSendEnable("RobotMode()");
    Sleep(100);
    if (robotRecvEnable(fb, sizeof(fb))) {
        FeedbackParser::parseMode(fb, mode);
        if (mode != 9 && mode != -1) {
            std::cout << "[脱困] EnableRobot后成功! mode=" << mode << std::endl;
            s_escaping = false;
            return true;
        }
        std::cout << "[脱困] EnableRobot后仍报警: " << fb;
    }

    s_escaping = false;
    return false;
}

bool RelayCore::init() {
    if (!robotConnect(Config::ROBOT_IP)) {
        std::cerr << "[Relay] 连接机械臂失败" << std::endl;
        return false;
    }

    m_stateMachine.onConnect();
    m_lastHeartbeatMs = GetTickCount();
    m_heartbeatLostReported = false;

    // Register FATAL callback: disable robot hardware on fatal error
    m_stateMachine.setFatalCallback([]() {
        robotSendEnable("DisableRobot()");
        Sleep(100);
        std::cerr << "[Safety] FATAL: robot disabled by state machine" << std::endl;
    });

    // 初始化序列：ClearError → 降灵敏度 → EnableRobot → (报警检测) → CP → GetPose
    Sleep(200);
    robotSendEnable("ClearError()");
    Sleep(300);

    // 使能前关闭所有可能误触发的灵敏度设置
    robotSendEnable("SetCollisionLevel(0)");   // 碰撞检测: 0=最不灵敏
    Sleep(50);
    robotSendEnable("SetSafeSkin(0)");          // 关闭电子皮肤
    Sleep(50);
    robotSendEnable("LoadSwitch(0)");           // 关闭负载自适应
    Sleep(50);

    if (!robotSendEnable("EnableRobot(0.5,0,0,0)")) {
        std::cerr << "[Relay] 使能失败" << std::endl;
        m_stateMachine.onEnableFail();
        return false;
    }
    std::cout << "[Relay] 机械臂使能成功" << std::endl;
    Sleep(200);

    // ===== 奇异检测: 使能后检查是否立即报警 (重试3次, 每次100ms) =====
    {
        int mode = -1;
        for (int retry = 0; retry < 3; retry++) {
            robotDrainEnable();
            robotSendEnable("RobotMode()");
            Sleep(100);
            char fb[128];
            if (robotRecvEnable(fb, sizeof(fb))) {
                if (FeedbackParser::parseMode(fb, mode)) {
                    break;
                }
            }
            std::cout << "[Relay] RobotMode retry " << (retry + 1) << "/3..." << std::endl;
        }

        if (mode == 9) {
            std::cout << "[Relay] 使能后检测到报警 (mode=9)，启动脱困流程..." << std::endl;
            if (!escapeSingularity()) {
                std::cerr << "[Relay] FATAL: 脱困失败" << std::endl;
                robotDisconnect();
                m_stateMachine.onEnableFail();
                return false;
            }
        } else if (mode == -1) {
            std::cout << "[Relay] 无法读取RobotMode (mode=-1)，继续初始化..." << std::endl;
        } else {
            std::cout << "[Relay] 机械臂状态正常 (mode=" << mode << ")" << std::endl;
        }
    }

    char cpBuf[64];
    snprintf(cpBuf, sizeof(cpBuf), "CP(%u)", Config::CP_SMOOTH_RATIO);
    robotSendEnable(cpBuf);
    Sleep(100);

    // 获取基准位姿 (重试 5 次，每次等待 100ms)
    bool gotBase = false;
    for (int retry = 0; retry < 5; retry++) {
        robotSendEnable("GetPose()");
        Sleep(100);
        char fb[1024];
        if (robotRecvEnable(fb, sizeof(fb))) {
            AppState::RobotPose pose;
            if (FeedbackParser::parsePose(fb, pose)) {
                auto& app = appState;
                EnterCriticalSection(&app.robotPoseMutex);
                app.robotBase.x = pose.x;
                app.robotBase.y = pose.y;
                app.robotBase.z = pose.z;
                app.robotBaseRx = pose.rx;
                app.robotBaseRy = pose.ry;
                app.robotBaseRz = pose.rz;
                app.robotActualPose = pose;
                app.robotTargetPose = pose;
                app.isRobotBaseSet = true;
                LeaveCriticalSection(&app.robotPoseMutex);
                std::cout << "[Relay] 基准位姿: (" << pose.x << "," << pose.y << "," << pose.z << ")" << std::endl;
                gotBase = true;
                break;
            }
        }
        std::cout << "[Relay] GetPose retry " << (retry + 1) << "/5..." << std::endl;
    }

    if (!gotBase) {
        std::cerr << "[Relay] FATAL: 无法获取机械臂当前位姿，拒绝运动控制" << std::endl;
        std::cerr << "[Relay] 请检查机械臂连接状态后重试" << std::endl;
        robotDisconnect();
        m_stateMachine.onEnableFail();
        return false;
    }

    m_stateMachine.onEnableSuccess();

    // 刷新心跳基准时间戳，避免 init() 耗时超过 HEARTBEAT_TIMEOUT_MS
    // 导致 queryPose() 首次触发时立即误判心跳超时 → FATAL
    m_lastHeartbeatMs = GetTickCount();

    return true;
}

void RelayCore::shutdown() {
    shutdownForceReader();
    robotSendEnable("DisableRobot()");
    Sleep(100);
    robotDisconnect();
    m_stateMachine.onDisconnect();
}

void RelayCore::sendPosition(const hduVector3Dd& devicePos) {
    if (!m_transmitting || !m_basePointSet || !isRobotConnected()) return;

    // ===== 安全守卫 =====
    if (!appState.isRobotBaseSet) return;

    // 更新触觉线程心跳时间戳
    m_lastHapticFrameMs = GetTickCount();

    // ===== ServoP 频率限制: 30Hz =====
    DWORD now = GetTickCount();
    if (now - m_lastServoTime < 33) return;
    m_lastServoTime = now;

    // ===== 增量式位移: 每帧计算 Touch 相对于上一帧的微小位移 =====
    Vec3 current = convertTouchToRobot(devicePos);

    EnterCriticalSection(&m_basePointLock);
    if (!m_lastTouchValid) {
        // 第一帧: 初始化参考点
        m_lastTouchPos = current;
        m_lastTouchValid = true;
        LeaveCriticalSection(&m_basePointLock);
        return;
    }

    // Compute position delta (for use outside orient mode only)
    double dx = current.x - m_lastTouchPos.x;
    double dy = current.y - m_lastTouchPos.y;
    double dz = current.z - m_lastTouchPos.z;
    m_lastTouchPos = current;  // always update touch reference

    Vec3 clamped = m_targetPos;  // default: fixed TCP (orientation-only mode)

    // Position delta: active when button1 is held (alone or combined with button2)
    if (appState.lastButtonState) {

        // NaN/Inf guard: 连续 3 帧异常 → FATAL
        if (std::isnan(dx) || std::isnan(dy) || std::isnan(dz) ||
            std::isinf(dx) || std::isinf(dy) || std::isinf(dz)) {
            m_nanFrameCount++;
            if (m_nanFrameCount >= 3) {
                RobotError error;
                error.code = RobotErrorCode::ERR_EMERGENCY_STOP;
                error.severity = Severity::FATAL;
                error.timestampMs = GetTickCount64();
                Vec3 zeroDelta = {0, 0, 0};
                m_stateMachine.onError(error, zeroDelta);
            }
            LeaveCriticalSection(&m_basePointLock);
            return;
        }
        m_nanFrameCount = 0;  // 正常帧清零

        // 跳过微小增量 (Touch 噪声)
        if (fabs(dx) < 0.05 && fabs(dy) < 0.05 && fabs(dz) < 0.05) {
            LeaveCriticalSection(&m_basePointLock);
            return;
        }

        // ===== 增量步长限制: 单步最大 3mm, 再乘以速度衰减因子 =====
        static double s_speedMul = 1.0;  // 跨帧持久, 由 SafetyPredictor 更新
        double len = sqrt(dx*dx + dy*dy + dz*dz);
        // Apply state machine speed factor ON TOP of safety verdict
        double effectiveSpeed = std::min(s_speedMul, m_stateMachine.speedFactor());
        double maxStep = 4.5 * effectiveSpeed;
        if (len > maxStep) {
            double scale = maxStep / len;
            dx *= scale; dy *= scale; dz *= scale;
        }

        // 计算候选位置 (先不更新 m_targetPos)
        Vec3 candidate;
        candidate.x = m_targetPos.x + dx;
        candidate.y = m_targetPos.y + dy;
        candidate.z = m_targetPos.z + dz;

        // 安全边界钳位
        clamped = SafetyBoundary::clampToBoundary(candidate);

        // ===== SafetyPredictor 预判 (先评估，后更新，防止边界漂移) =====
        SafetyVerdict verdict = SafetyPredictor::instance().evaluate(clamped);

        // State machine: escalate on warning
        if (verdict.errorCode != RobotErrorCode::OK
            && verdict.errorCode != RobotErrorCode::ERR_IK_NO_SOLUTION
            && verdict.errorCode != RobotErrorCode::ERR_JOINTLIMIT_WARN) {
            Vec3 deltaVec(dx, dy, dz);
            RobotError error = SafetyPredictor::instance().lastError();
            m_stateMachine.onError(error, deltaVec);
        } else {
            // Check for reverse-motion de-escalation (immediate recovery)
            Vec3 deltaVec(dx, dy, dz);
            auto& esc = m_stateMachine.escalation();
            if (esc.isEscalated() && esc.shouldDeescalate(deltaVec, esc.lastRejectDirection)) {
                m_stateMachine.onRecovery();
            }
            m_stateMachine.escalation().onClear();
        }

        // Check if state machine allows motion
        if (!m_stateMachine.canMove()) {
            LeaveCriticalSection(&m_basePointLock);
            return;
        }

        if (verdict.action == SafetyVerdict::REJECT) {
            std::cerr << "[Safety] REJECT: " << verdict.reason
                      << " — candidate=(" << clamped.x << "," << clamped.y << "," << clamped.z << ")"
                      << std::endl;
            LeaveCriticalSection(&m_basePointLock);
            return;  // 不更新 m_targetPos，下帧从同一位置重新计算
        }

        // 更新速度衰减因子 (用于下帧)
        s_speedMul = (verdict.action == SafetyVerdict::WARN_SLOW) ? verdict.speedFactor : 1.0;
    }

    // ===== 姿态计算 (优化或用户控制) =====
    auto& app = appState;
    double targetRx = app.robotBaseRx;
    double targetRy = app.robotBaseRy;
    double targetRz = app.robotBaseRz;

    // Mode 1: Position-only (button1, no button2) — optimize orientation
    if (appState.lastButtonState && !m_transmittingOrient) {
        double curJoints[6];
        {
            EnterCriticalSection(&app.robotPoseMutex);
            curJoints[0] = app.robotActualPose.j1;
            curJoints[1] = app.robotActualPose.j2;
            curJoints[2] = app.robotActualPose.j3;
            curJoints[3] = app.robotActualPose.j4;
            curJoints[4] = app.robotActualPose.j5;
            curJoints[5] = app.robotActualPose.j6;
            LeaveCriticalSection(&app.robotPoseMutex);
        }
        Vec3 optOrient = SingularityAvoidance::optimizeOrientation(clamped, curJoints);
        if (optOrient.x != 0.0 || optOrient.y != 0.0 || optOrient.z != 0.0) {
            targetRx = optOrient.x;
            targetRy = optOrient.y;
            targetRz = optOrient.z;
        }
        // else: IK failed, keep base orientation as fallback
    }

    Vec3 damped(0.0, 0.0, 0.0);  // orientation delta for cross-mode sharing

    if (m_transmittingOrient && m_orientValid) {
        // Read current stylus orientation (thread-safe)
        double stylusRx, stylusRy, stylusRz;
        EnterCriticalSection(&app.stylusOrientMutex);
        stylusRx = app.stylusOrient[0];
        stylusRy = app.stylusOrient[1];
        stylusRz = app.stylusOrient[2];
        LeaveCriticalSection(&app.stylusOrientMutex);

        Vec3 current(stylusRx, stylusRy, stylusRz);

        // Compute incremental delta from stylus rotation change
        double drx = current.x - m_lastStylusOrient.x;
        double dry = current.y - m_lastStylusOrient.y;
        double drz = current.z - m_lastStylusOrient.z;

        // Update reference for next frame
        m_lastStylusOrient = current;

        // NaN/Inf guard: skip this frame's orientation delta (don't increment nan counter)
        if (std::isnan(drx) || std::isnan(dry) || std::isnan(drz) ||
            std::isinf(drx) || std::isinf(dry) || std::isinf(drz)) {
            // orientation delta skipped for this frame only
        } else {
        // Deadzone filter
        if (fabs(drx) >= Config::ORIENT_DEADZONE_DEG ||
            fabs(dry) >= Config::ORIENT_DEADZONE_DEG ||
            fabs(drz) >= Config::ORIENT_DEADZONE_DEG) {

            // Apply gain
            drx *= Config::ORIENT_GAIN;
            dry *= Config::ORIENT_GAIN;
            drz *= Config::ORIENT_GAIN;

            // Step cap
            if (drx > Config::ORIENT_MAX_STEP_DEG) drx = Config::ORIENT_MAX_STEP_DEG;
            if (drx < -Config::ORIENT_MAX_STEP_DEG) drx = -Config::ORIENT_MAX_STEP_DEG;
            if (dry > Config::ORIENT_MAX_STEP_DEG) dry = Config::ORIENT_MAX_STEP_DEG;
            if (dry < -Config::ORIENT_MAX_STEP_DEG) dry = -Config::ORIENT_MAX_STEP_DEG;
            if (drz > Config::ORIENT_MAX_STEP_DEG) drz = Config::ORIENT_MAX_STEP_DEG;
            if (drz < -Config::ORIENT_MAX_STEP_DEG) drz = -Config::ORIENT_MAX_STEP_DEG;

            // ---- Axis remap: stylus frame → robot frame ----
            // Build 3×3 rotation that maps Touch rotation axes to robot rotation axes.
            // When calibration is enabled, use the calibrated rigid transform R.
            // Fallback: hardcoded axis mapping matching convertTouchToRobot():
            //   robot_X = touch_X   → [1, 0,  0]
            //   robot_Y = -touch_Z  → [0, 0, -1]
            //   robot_Z = touch_Y   → [0, 1,  0]
            double R00, R01, R02, R10, R11, R12, R20, R21, R22;
            if (Calibration::enabled) {
                R00 = Calibration::R[0]; R01 = Calibration::R[1]; R02 = Calibration::R[2];
                R10 = Calibration::R[3]; R11 = Calibration::R[4]; R12 = Calibration::R[5];
                R20 = Calibration::R[6]; R21 = Calibration::R[7]; R22 = Calibration::R[8];
            } else {
                R00 = 1.0; R01 = 0.0; R02 =  0.0;
                R10 = 0.0; R11 = 0.0; R12 = -1.0;
                R20 = 0.0; R21 = 1.0; R22 =  0.0;
            }
            double robot_dRx = R00*drx + R01*dry + R02*drz;
            double robot_dRy = R10*drx + R11*dry + R12*drz;
            double robot_dRz = R20*drx + R21*dry + R22*drz;

            Vec3 robotDelta(robot_dRx, robot_dRy, robot_dRz);

            // Get current joints for avoidance computation
            double curJoints[6];
            {
                EnterCriticalSection(&app.robotPoseMutex);
                curJoints[0] = app.robotActualPose.j1;
                curJoints[1] = app.robotActualPose.j2;
                curJoints[2] = app.robotActualPose.j3;
                curJoints[3] = app.robotActualPose.j4;
                curJoints[4] = app.robotActualPose.j5;
                curJoints[5] = app.robotActualPose.j6;
                LeaveCriticalSection(&app.robotPoseMutex);
            }

            Vec3 tcpAdj, repulsionOut;
            Vec3 currentTcp(clamped.x, clamped.y, clamped.z);

            damped = SingularityAvoidance::dampOrientationMotion(
                m_targetOrient, robotDelta, currentTcp, curJoints, tcpAdj, repulsionOut);

            // Apply damped orientation delta
            m_targetOrient.x += damped.x;
            m_targetOrient.y += damped.y;
            m_targetOrient.z += damped.z;
            m_targetOrient = clampOrientToBounds(m_targetOrient);

            // Apply TCP micro-adjust (position mode: locked; orient mode: micro-adjust)
            if (appState.lastButtonState && m_transmittingOrient) {
                // Combined mode: don't adjust position here (handled by dampFullCommand)
            } else if (m_transmittingOrient) {
                // Orientation-only: apply TCP micro-adjust
                clamped.x += tcpAdj.x;
                clamped.y += tcpAdj.y;
                clamped.z += tcpAdj.z;
            }

            // Phase 2: write directional repulsion force to AppState for haptic callback
            {
                EnterCriticalSection(&app.orientRepulsionMutex);
                app.orientRepulsionForce[0] = repulsionOut.x;
                app.orientRepulsionForce[1] = repulsionOut.y;
                app.orientRepulsionForce[2] = repulsionOut.z;
                app.hasOrientRepulsion = true;
                LeaveCriticalSection(&app.orientRepulsionMutex);
            }
        }

        targetRx = m_targetOrient.x;
        targetRy = m_targetOrient.y;
        targetRz = m_targetOrient.z;
        }  // end !NaN guard
    }

    // ===== Position update (only when NOT in orientation mode) =====
    // When m_transmittingOrient is true, the TCP position is frozen —
    // we compute it below from the fixed wrist center + rotated offset.
    if (!m_transmittingOrient) {
        // 通过安全检查后才更新目标位置
        m_targetPos = clamped;
    }
    LeaveCriticalSection(&m_basePointLock);

    // ===== Compute ServoP position =====
    // Position always comes from m_targetPos (via 'clamped'):
    //   - position mode (button1):     updated by Touch delta above
    //   - orientation-only (button2):  frozen at press-time TCP capture
    //   - combined (button1+2):        updated by Touch delta + orientation accumulation
    double servoCmdX = clamped.x;
    double servoCmdY = clamped.y;
    double servoCmdZ = clamped.z;

    // During orientation mode, validate the TCP position (no IK — position-only checks)
    if (m_transmittingOrient && m_orientValid) {
        Vec3 tcpCheck(servoCmdX, servoCmdY, servoCmdZ);
        SafetyVerdict ov = SafetyPredictor::instance().evaluatePositionOnly(tcpCheck);
        if (ov.action == SafetyVerdict::REJECT) {
            std::cerr << "[Safety] Orient TCP REJECT: " << ov.reason
                      << " — tcp=(" << servoCmdX << "," << servoCmdY << "," << servoCmdZ << ")"
                      << std::endl;
            return;
        }

        // Amplify singular constraint force in orientation mode
        double extraForce[3];
        ConstraintForce::computeSingularForce(
            Vec3(servoCmdX, servoCmdY, servoCmdZ),
            extraForce,
            Config::SINGAVOID_ORIENT_FORCE_AMP);
        EnterCriticalSection(&app.orientForceMutex);
        app.orientExtraForce[0] = extraForce[0];
        app.orientExtraForce[1] = extraForce[1];
        app.orientExtraForce[2] = extraForce[2];
        app.hasOrientExtraForce = true;
        LeaveCriticalSection(&app.orientForceMutex);
    }

    // Mode 3: Combined position+orientation — SVD-based selective damping
    if (appState.lastButtonState && m_transmittingOrient && m_orientValid) {
        // Both deltas were computed in this frame
        // Build the user's 6-DOF delta from what was actually applied
        Vec3 posDelta(
            clamped.x - m_targetPos.x,
            clamped.y - m_targetPos.y,
            clamped.z - m_targetPos.z
        );
        Vec3 orientDelta(damped.x, damped.y, damped.z);  // pre-damped by Mode 2

        double curJoints[6];
        {
            EnterCriticalSection(&app.robotPoseMutex);
            curJoints[0] = app.robotActualPose.j1;
            curJoints[1] = app.robotActualPose.j2;
            curJoints[2] = app.robotActualPose.j3;
            curJoints[3] = app.robotActualPose.j4;
            curJoints[4] = app.robotActualPose.j5;
            curJoints[5] = app.robotActualPose.j6;
            LeaveCriticalSection(&app.robotPoseMutex);
        }

        Vec3 dampedPos, dampedOrient;
        SingularityAvoidance::dampFullCommand(posDelta, orientDelta, curJoints,
                                              dampedPos, dampedOrient);

        // Reconstruct clamped position from damped delta
        clamped.x = m_targetPos.x + dampedPos.x;
        clamped.y = m_targetPos.y + dampedPos.y;
        clamped.z = m_targetPos.z + dampedPos.z;

        // Update servoCmd after Mode 3 modifies clamped (Bug 2 fix)
        servoCmdX = clamped.x;
        servoCmdY = clamped.y;
        servoCmdZ = clamped.z;

        // Bug 3 fix: undo Mode 2's orientation contribution, apply Mode 3's combined damping.
        // Mode 2 already added 'damped' to targetRx, so we subtract it before adding 'dampedOrient'.
        targetRx = targetRx - damped.x + dampedOrient.x;
        targetRy = targetRy - damped.y + dampedOrient.y;
        targetRz = targetRz - damped.z + dampedOrient.z;
    }

    // ===== 构造并发送 ServoP =====
    char cmd[256];
    snprintf(cmd, sizeof(cmd), "ServoP(%.2f,%.2f,%.2f,%.2f,%.2f,%.2f)",
        servoCmdX, servoCmdY, servoCmdZ,
        targetRx, targetRy, targetRz);

    bool sent = robotSendMotion(cmd);
    static int sendCount = 0, failCount = 0;
    sendCount++;
    if (!sent) failCount++;
    if (sendCount % 50 == 0) {
        std::cout << "[Relay] Motion sends: " << sendCount
                  << " ok, " << failCount << " fail"
                  << "  target=(" << servoCmdX << "," << servoCmdY << "," << servoCmdZ << ")"
                  << " orient=(" << targetRx << "," << targetRy << "," << targetRz << ")"
                  << std::endl;
    }

    // 上报到 MATLAB GUI
    reportCommand(cmd);

    // 记录到最后指令
    EnterCriticalSection(&app.lastCommandMutex);
    strncpy_s(app.lastCommandSent, cmd, sizeof(app.lastCommandSent) - 1);
    LeaveCriticalSection(&app.lastCommandMutex);

    // 记录到指令日志
    EnterCriticalSection(&app.commandLogMutex);
    strncpy_s(app.commandLog[app.commandLogIdx], cmd, sizeof(app.commandLog[0]) - 1);
    app.commandLogIdx = (app.commandLogIdx + 1) % AppState::LOG_SIZE;
    if (app.commandLogCount < AppState::LOG_SIZE) app.commandLogCount++;
    LeaveCriticalSection(&app.commandLogMutex);

    // 更新目标位姿
    EnterCriticalSection(&app.robotPoseMutex);
    app.robotTargetPose.x = servoCmdX;
    app.robotTargetPose.y = servoCmdY;
    app.robotTargetPose.z = servoCmdZ;
    app.robotTargetPose.rx = targetRx;
    app.robotTargetPose.ry = targetRy;
    app.robotTargetPose.rz = targetRz;
    LeaveCriticalSection(&app.robotPoseMutex);
}

void RelayCore::onButtonPress(const Vec3& robotPos) {
    m_stateMachine.onButtonPress();
    EnterCriticalSection(&m_basePointLock);
    // 以机器人当前实际位姿作为 target 起点 (钳位到安全边界内)
    {
        auto& app = appState;
        EnterCriticalSection(&app.robotPoseMutex);
        Vec3 rawPos(app.robotActualPose.x, app.robotActualPose.y, app.robotActualPose.z);
        LeaveCriticalSection(&app.robotPoseMutex);
        m_targetPos = SafetyBoundary::clampToBoundary(rawPos);
    }
    m_lastTouchPos = robotPos;
    m_lastTouchValid = true;
    LeaveCriticalSection(&m_basePointLock);
    m_basePointSet = true;
    m_transmitting = true;

    std::cout << "[Relay] Button PRESS  — target start=("
              << m_targetPos.x << "," << m_targetPos.y << "," << m_targetPos.z << ")"
              << std::endl;
    std::cout << "[Relay] Hold button + move Touch (incremental mode)" << std::endl;
}

void RelayCore::onButtonRelease() {
    // If orientation mode is still active (button2 held), keep transmitting
    // for orientation control. Only fully stop when both modes are done.
    if (!m_transmittingOrient) {
        m_transmitting = false;
        m_basePointSet = false;
        m_lastTouchValid = false;
        m_stateMachine.onButtonRelease();
    }
    std::cout << "[Relay] Button RELEASE"
              << (m_transmittingOrient ? " (orientation still active)" : " — motion stopped")
              << std::endl;
}

void RelayCore::onButton2Press(const Vec3& stylusOrient) {
    EnterCriticalSection(&m_basePointLock);
    // Capture stylus/robot reference snapshots at press moment.
    // These are stored for diagnostics/re-sync and logged on press — they are
    // NOT used in the per-frame delta computation. The per-frame tracking uses
    // m_lastStylusOrient (incremental delta) in sendPosition().
    m_orientRefStylus = stylusOrient;

    // Capture robot current orientation
    double curRx, curRy, curRz;
    {
        auto& app = appState;
        EnterCriticalSection(&app.robotPoseMutex);
        m_orientRefRobot = Vec3(app.robotActualPose.rx, app.robotActualPose.ry, app.robotActualPose.rz);
        curRx = app.robotActualPose.rx;
        curRy = app.robotActualPose.ry;
        curRz = app.robotActualPose.rz;
        LeaveCriticalSection(&app.robotPoseMutex);
    }

    // Initialize accumulated target and last-frame stylus orientation.
    // TCP stays at the current target position (fixed during orientation-only,
    // updated by position delta during combined button1+2 mode).
    m_targetOrient = Vec3(curRx, curRy, curRz);
    m_lastStylusOrient = stylusOrient;
    m_orientValid = true;
    m_transmittingOrient = true;

    // If position mode is not already active, start transmission
    if (!m_transmitting) {
        m_stateMachine.onButtonPress();
        // Seed position target from actual TCP pose (fixed during orientation-only,
        // updated by Touch delta during combined button1+2 mode).
        {
            auto& app = appState;
            EnterCriticalSection(&app.robotPoseMutex);
            Vec3 tcpPos(app.robotActualPose.x, app.robotActualPose.y, app.robotActualPose.z);
            LeaveCriticalSection(&app.robotPoseMutex);
            m_targetPos = SafetyBoundary::clampToBoundary(tcpPos);
        }
        m_transmitting = true;
        m_basePointSet = true;
    }
    LeaveCriticalSection(&m_basePointLock);

    std::cout << "[Relay] Button2 PRESS — orient ref=("
              << m_orientRefStylus.x << "," << m_orientRefStylus.y << "," << m_orientRefStylus.z << ")"
              << " robot ref=(" << m_orientRefRobot.x << "," << m_orientRefRobot.y << "," << m_orientRefRobot.z << ")"
              << std::endl;
}

void RelayCore::onButton2Release() {
    m_transmittingOrient = false;
    m_orientValid = false;

    // If button1 is NOT pressed (only button2 was active), stop all transmission.
    // When button1 IS still held, keep m_transmitting active for position control.
    if (!appState.lastButtonState.load()) {
        m_transmitting = false;
        m_basePointSet = false;
        m_lastTouchValid = false;
        m_stateMachine.onButtonRelease();
        std::cout << "[Relay] Button2 RELEASE — all motion stopped" << std::endl;
    } else {
        // Button1 still held — re-sync m_targetPos to current robot TCP.
        // During orientation mode, the TCP moved (rotated around wrist center),
        // so m_targetPos is stale. Re-seed from actual pose to avoid a position jump.
        EnterCriticalSection(&m_basePointLock);
        {
            auto& app = appState;
            EnterCriticalSection(&app.robotPoseMutex);
            Vec3 rawPos(app.robotActualPose.x, app.robotActualPose.y, app.robotActualPose.z);
            LeaveCriticalSection(&app.robotPoseMutex);
            m_targetPos = SafetyBoundary::clampToBoundary(rawPos);
        }
        LeaveCriticalSection(&m_basePointLock);
        std::cout << "[Relay] Button2 RELEASE — orientation control stopped (button1 still held)" << std::endl;
    }
}

static void logFeedback(const char* msg, const char* portLabel) {
    auto& app = appState;
    EnterCriticalSection(&app.feedbackLogMutex);
    int writeIdx = app.feedbackLogIdx;
    snprintf(app.feedbackLog[writeIdx], sizeof(app.feedbackLog[0]),
        "[%s] %s", portLabel, msg);
    app.feedbackLogIdx = (app.feedbackLogIdx + 1) % AppState::LOG_SIZE;
    if (app.feedbackLogCount < AppState::LOG_SIZE) app.feedbackLogCount++;
    // Relay to MATLAB GUI
    RelayCore::instance().reportFeedback(app.feedbackLog[writeIdx]);
    LeaveCriticalSection(&app.feedbackLogMutex);
}

void RelayCore::pollFeedback() {
    char buf[1024];
    // 读取运动端口反馈 (非阻塞)
    while (robotRecvMotionPoll(buf, sizeof(buf))) {
        RobotFeedback fb;
        strncpy_s(fb.raw, buf, sizeof(fb.raw) - 1);
        fb.fromPort = Config::MOTION_PORT;
        fb.errorId = (buf[0] == '0') ? 0 : -1;
        FeedbackParser::extractData(buf, fb.data, sizeof(fb.data));

        // Parse ServoP errors and report to state machine
        if (fb.raw[0] != '0') {
            int dobotCode = 0;
            if (FeedbackParser::extractErrorCode(fb.raw, dobotCode) && dobotCode != 0) {
                RobotErrorCode errCode = FeedbackParser::mapRobotErrorCode(dobotCode);
                RobotError error;
                error.code = errCode;
                error.severity = getSeverity(errCode);
                error.timestampMs = GetTickCount64();
                // Get current target from state
                error.targetPosition = m_targetPos;
                error.speedFactor = m_stateMachine.speedFactor();

                Vec3 zeroDelta = {0, 0, 0};  // no user delta for feedback errors
                m_stateMachine.onError(error, zeroDelta);

                double constraintMag = 0;  // feedback error has no constraint force
                RobotDiagnostics::instance().logError(error, constraintMag,
                    m_stateMachine.currentState());
            }
        }

        // 记录日志 (截断长字符串)
        char shortMsg[256];
        const char* src = fb.data[0] ? fb.data : fb.raw;
        snprintf(shortMsg, sizeof(shortMsg), "%.200s", src);
        logFeedback(shortMsg, "30003");

        for (auto* ext : m_extensions) {
            ext->onAfterFeedback(fb);
        }
    }

    // 注意: 不读取使能端口 (29999)
    // 使能端口采用"命令-响应"模式 (GetPose/GetAngle/RobotMode)，
    // 响应必须由发送命令的函数独享读取，避免 pollFeedback 偷走数据
    // 导致 robotActualPose 永远停留在 init 时的值。
}

void RelayCore::queryPose() {
    static int dbgCount = 0;
    if (++dbgCount <= 3 || dbgCount % 50 == 0) {
        std::cerr << "[DBG] queryPose #" << dbgCount << " connected=" << isRobotConnected()
                  << " hbAge=" << (GetTickCount() - m_lastHeartbeatMs) << "ms" << std::endl;
    }
    if (!isRobotConnected()) return;
    robotDrainEnable();  // 排空残留避免读到其他命令的响应
    robotSendEnable("GetPose()");
    Sleep(50);
    char fb[1024];
    if (robotRecvEnable(fb, sizeof(fb))) {
        auto& app = appState;
        EnterCriticalSection(&app.robotPoseMutex);
        FeedbackParser::parsePose(fb, app.robotActualPose);
        // 在锁内读取位姿，避免与 jointAngle 定时器竞态
        double px = app.robotActualPose.x;
        double py = app.robotActualPose.y;
        double pz = app.robotActualPose.z;
        double prx = app.robotActualPose.rx;
        double pry = app.robotActualPose.ry;
        double prz = app.robotActualPose.rz;
        LeaveCriticalSection(&app.robotPoseMutex);

        // 上报机器人实际位姿到 MATLAB GUI
        char buf[128];
        snprintf(buf, sizeof(buf), "RP|%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
            px, py, pz, prx, pry, prz);
        sendRelayUpdate(buf);
    }

    // PING/PONG latency measurement
    DWORD now = GetTickCount();
    if (now - m_lastPingMs > (DWORD)Config::PING_INTERVAL_MS) {
        m_lastPingMs = now;
        // Send PING to enable port
        if (isRobotConnected()) {
            char pingBuf[64];
            uint64_t sentMs = GetTickCount64();
            snprintf(pingBuf, sizeof(pingBuf), "PING|%llu", sentMs);
            robotSendEnable(pingBuf);
            // Read PONG response (non-blocking poll with short wait)
            Sleep(5);
            char pongBuf[128] = {};
            if (robotRecvEnablePoll(pongBuf, sizeof(pongBuf))) {
                if (strncmp(pongBuf, "PONG", 4) == 0) {
                    uint64_t now64 = GetTickCount64();
                    const char* pipe = strchr(pongBuf, '|');
                    if (pipe) {
                        uint64_t echoMs = _strtoui64(pipe + 1, nullptr, 10);
                        auto& app = appState;
                        app.latencyMs = (float)(now64 - echoMs);
                    }
                }
            }
        }
    }

    // Health check: skip during 10s grace period after startup
    if (now - m_heartbeatStartMs >= 10000) {
        if (now - m_lastHeartbeatMs > (DWORD)Config::HEARTBEAT_TIMEOUT_MS) {
            if (!m_heartbeatLostReported) {
                m_heartbeatLostReported = true;
                RobotError error;
                error.code = RobotErrorCode::ERR_HEARTBEAT_LOST;
                error.severity = Severity::FATAL;
                error.timestampMs = GetTickCount64();
                Vec3 zeroDelta = {0, 0, 0};
                m_stateMachine.onError(error, zeroDelta);
            }
        } else {
            m_heartbeatLostReported = false;
        }
    }

    // Heartbeat update
    m_lastHeartbeatMs = now;
}

void RelayCore::queryJointAngles() {
    static int dbgCount = 0;
    if (++dbgCount <= 3 || dbgCount % 50 == 0) {
        std::cerr << "[DBG] queryJointAngles #" << dbgCount << " connected=" << isRobotConnected() << std::endl;
    }
    if (!isRobotConnected()) return;
    robotDrainEnable();  // 排空残留
    robotSendEnable("GetAngle()");
    Sleep(50);
    char fb[1024];
    if (robotRecvEnable(fb, sizeof(fb))) {
        double angles[6] = {};
        if (FeedbackParser::parseAngle(fb, angles)) {
            auto& app = appState;
            EnterCriticalSection(&app.robotPoseMutex);
            app.robotActualPose.j1 = angles[0];
            app.robotActualPose.j2 = angles[1];
            app.robotActualPose.j3 = angles[2];
            app.robotActualPose.j4 = angles[3];
            app.robotActualPose.j5 = angles[4];
            app.robotActualPose.j6 = angles[5];
            LeaveCriticalSection(&app.robotPoseMutex);

            // 上报关节角度到 MATLAB GUI
            char buf[128];
            snprintf(buf, sizeof(buf), "J|%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
                angles[0], angles[1], angles[2], angles[3], angles[4], angles[5]);
            sendRelayUpdate(buf);
        }
    }

    // Health check: skip during 10s grace period after startup
    DWORD now = GetTickCount();
    if (now - m_heartbeatStartMs >= 10000) {
        if (now - m_lastHeartbeatMs > (DWORD)Config::HEARTBEAT_TIMEOUT_MS) {
            if (!m_heartbeatLostReported) {
                m_heartbeatLostReported = true;
                RobotError error;
                error.code = RobotErrorCode::ERR_HEARTBEAT_LOST;
                error.severity = Severity::FATAL;
                error.timestampMs = GetTickCount64();
                Vec3 zeroDelta = {0, 0, 0};
                m_stateMachine.onError(error, zeroDelta);
            }
        } else {
            m_heartbeatLostReported = false;
        }
    }
    m_lastHeartbeatMs = GetTickCount();
}

void RelayCore::checkHapticWatchdog() {
    if (!m_transmitting.load()) return;  // 未运动时不检查
    DWORD now = GetTickCount();
    DWORD lastFrame = m_lastHapticFrameMs.load();
    if (lastFrame > 0 && (now - lastFrame) > (DWORD)Config::WATCHDOG_TIMEOUT_MS) {
        if (!m_watchdogTripped) {
            m_watchdogTripped = true;
            std::cerr << "[Safety] WATCHDOG: haptic thread silent for "
                      << (now - lastFrame) << "ms — triggering FATAL" << std::endl;
            RobotError error;
            error.code = RobotErrorCode::ERR_EMERGENCY_STOP;
            error.severity = Severity::FATAL;
            error.timestampMs = GetTickCount64();
            Vec3 zeroDelta = {0, 0, 0};
            m_stateMachine.onError(error, zeroDelta);
            // FATAL callback (registered in init) will DisableRobot()
        }
    }
}

void RelayCore::pingRobot() {
    if (!isRobotConnected()) return;
    char pingBuf[64];
    snprintf(pingBuf, sizeof(pingBuf), "PING|%llu", GetTickCount64());
    robotSendEnable(pingBuf);
    // Response handled in pollFeedback (PONG echo)
}

void RelayCore::checkAlarm() {
    if (!isRobotConnected()) return;
    robotDrainEnable();  // 排空残留
    robotSendEnable("RobotMode()");
    Sleep(50);
    char fb[1024];
    if (robotRecvEnable(fb, sizeof(fb))) {
        int mode = -1;
        FeedbackParser::parseMode(fb, mode);
        auto& app = appState;
        bool wasAlarm = app.isRobotInAlarm.exchange(mode == 9);
        if (mode == 9 && !wasAlarm) {
            std::cout << "\n[Relay] !!! 检测到机械臂报警 (mode=9) !!!" << std::endl;

            // Report alarm to state machine
            RobotError error;
            error.code = RobotErrorCode::ERR_ALARM_MODE9;
            error.severity = Severity::FATAL;
            error.timestampMs = GetTickCount64();
            Vec3 zeroDelta = {0, 0, 0};
            m_stateMachine.onError(error, zeroDelta);

            // 立即获取当前位置并记录到 SafetyPredictor 黑名单
            queryPose();
            EnterCriticalSection(&app.robotPoseMutex);
            AppState::RobotPose alarmPose = app.robotActualPose;
            LeaveCriticalSection(&app.robotPoseMutex);
            SafetyPredictor::instance().addAlarmRecord(alarmPose);

            // 自动进入脱困流程
            std::cout << "[Relay] 自动启动脱困流程..." << std::endl;
            if (escapeSingularity()) {
                std::cout << "[Relay] 脱困成功，恢复正常操作" << std::endl;
                app.isRobotInAlarm = false;
                m_stateMachine.onRecovery();
            } else {
                std::cout << "[Relay] 脱困失败，按 'e' 重试或重启程序" << std::endl;
            }
        } else if (mode != 9 && wasAlarm) {
            // 报警已清除 (用户在机器人控制器上手动清除)
            std::cout << "[Relay] 报警已清除 (mode=" << mode << ")，尝试恢复..." << std::endl;
            app.isRobotInAlarm = false;
            // Re-enable robot since FATAL callback disabled it
            robotSendEnable("ClearError()");
            Sleep(200);
            robotSendEnable("EnableRobot(0.5,0,0,0)");
            Sleep(200);
            m_stateMachine.onRecovery();
        }
    }
}

bool RelayCore::triggerEscape() {
    if (!isRobotConnected()) {
        std::cerr << "[Relay] 机械臂未连接，无法脱困" << std::endl;
        return false;
    }
    return escapeSingularity();
}

void RelayCore::registerExtension(IExtension* ext) {
    m_extensions.push_back(ext);
}

// ===== MATLAB GUI 上报 =====

void RelayCore::initRelayReporting() {
    SOCKET sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock == INVALID_SOCKET) return;

    sockaddr_in addr;
    addr.sin_family = AF_INET;
    inet_pton(AF_INET, Config::RELAY_IP, &addr.sin_addr);
    addr.sin_port = htons(Config::RELAY_PORT);

    if (connect(sock, (SOCKADDR*)&addr, sizeof(addr)) == SOCKET_ERROR) {
        closesocket(sock);
        return;
    }

    int timeout = 100;
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, (char*)&timeout, sizeof(timeout));

    EnterCriticalSection(&m_relaySocketMutex);
    m_relaySocket = sock;
    LeaveCriticalSection(&m_relaySocketMutex);

    std::cout << "[Relay] GUI reporting connected to " << Config::RELAY_IP
              << ":" << Config::RELAY_PORT << std::endl;
}

void RelayCore::shutdownRelayReporting() {
    EnterCriticalSection(&m_relaySocketMutex);
    if (m_relaySocket != INVALID_SOCKET) {
        closesocket(m_relaySocket);
        m_relaySocket = INVALID_SOCKET;
    }
    LeaveCriticalSection(&m_relaySocketMutex);
}

int RelayCore::sendRelayUpdate(const char* msg) {
    EnterCriticalSection(&m_relaySocketMutex);
    SOCKET sock = m_relaySocket;
    LeaveCriticalSection(&m_relaySocketMutex);
    if (sock == INVALID_SOCKET) return -1;

    int n1 = send(sock, msg, (int)strlen(msg), 0);
    int n2 = send(sock, "\n", 1, 0);
    return n1 + n2;
}

void RelayCore::reportPosition() {
    DWORD now = GetTickCount();
    if (now - m_lastRelayUpdate < (DWORD)Config::RELAY_UPDATE_INTERVAL) return;
    m_lastRelayUpdate = now;

    auto& app = appState;
    char buf[256];
    hduVector3Dd pos;
    EnterCriticalSection(&app.devicePosMutex);
    pos = app.devicePos;
    LeaveCriticalSection(&app.devicePosMutex);

    double sx, sy, sz;
    {
        auto& appRef = appState;
        EnterCriticalSection(&appRef.stylusOrientMutex);
        sx = appRef.stylusOrient[0];
        sy = appRef.stylusOrient[1];
        sz = appRef.stylusOrient[2];
        LeaveCriticalSection(&appRef.stylusOrientMutex);
    }
    snprintf(buf, sizeof(buf), "P|%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
        pos[0], pos[1], pos[2], sx, sy, sz);

    int sent = sendRelayUpdate(buf);
    static int dbgCount = 0;
    if (++dbgCount % 100 == 0) {
        std::cout << "[Relay] Sent " << dbgCount << " position updates, last send=" << sent << "B" << std::endl;
    }
}

void RelayCore::reportCommand(const char* cmd) {
    char buf[384];
    snprintf(buf, sizeof(buf), "C|%s", cmd);
    sendRelayUpdate(buf);
}

void RelayCore::reportFeedback(const char* fbText) {
    char buf[384];
    snprintf(buf, sizeof(buf), "FB|%.350s", fbText);
    sendRelayUpdate(buf);
}

void RelayCore::sendSafetyStatus() {
    const auto& sm = m_stateMachine;
    char buf[128];
    snprintf(buf, sizeof(buf), "S|%d,%.2f,%d",
        static_cast<int>(sm.currentState()),
        sm.speedFactor(),
        SafetyPredictor::instance().alarmCount());
    sendRelayUpdate(buf);
}

void RelayCore::sendJointMargins() {
    static const double jlims[6][2] = {
        {-360,360},{-360,360},{-155,155},{-360,360},{-360,360},{-360,360}};
    auto& app = appState;
    EnterCriticalSection(&app.robotPoseMutex);
    double jv[6]={app.robotActualPose.j1,app.robotActualPose.j2,
        app.robotActualPose.j3,app.robotActualPose.j4,
        app.robotActualPose.j5,app.robotActualPose.j6};
    LeaveCriticalSection(&app.robotPoseMutex);
    char buf[128];
    snprintf(buf,sizeof(buf),"L|%.1f,%.1f,%.1f,%.1f,%.1f,%.1f",
        fmin(fabs(jv[0]-jlims[0][0]),fabs(jlims[0][1]-jv[0])),
        fmin(fabs(jv[1]-jlims[1][0]),fabs(jlims[1][1]-jv[1])),
        fmin(fabs(jv[2]-jlims[2][0]),fabs(jlims[2][1]-jv[2])),
        fmin(fabs(jv[3]-jlims[3][0]),fabs(jlims[3][1]-jv[3])),
        fmin(fabs(jv[4]-jlims[4][0]),fabs(jlims[4][1]-jv[4])),
        fmin(fabs(jv[5]-jlims[5][0]),fabs(jlims[5][1]-jv[5])));
    sendRelayUpdate(buf);
}

void RelayCore::sendSingularity() {
    auto& app = appState;
    EnterCriticalSection(&app.robotPoseMutex);
    double x=app.robotActualPose.x, y=app.robotActualPose.y;
    LeaveCriticalSection(&app.robotPoseMutex);
    double r_xy=sqrt(x*x+y*y);
    char buf[64];
    snprintf(buf,sizeof(buf),"G|%.1f,%d",r_xy,(r_xy<30.0)?1:0);
    sendRelayUpdate(buf);
}

void RelayCore::sendCalibStatus() {
    char buf[64];
    snprintf(buf,sizeof(buf),"B|%d,%.2f",
        Calibration::enabled?1:0, Calibration::enabled?Calibration::rmsError:-1.0);
    sendRelayUpdate(buf);
}

void RelayCore::sendConnectionHealth() {
    // Track process start time on first call
    if (m_processStartMs == 0) {
        m_processStartMs = GetTickCount();
    }

    auto& app = appState;
    bool enableOk = app.isRobotConnected.load();

    // Motion port: same socket lifecycle as enable
    bool motionOk = enableOk;

    // Force port: consider connected if data isn't stale
    bool forceOk = false;
    EnterCriticalSection(&app.forceDataMutex);
    forceOk = !app.forceData.isStale;
    LeaveCriticalSection(&app.forceDataMutex);

    float pingMs = app.latencyMs.load();
    DWORD uptimeS = (GetTickCount() - m_processStartMs) / 1000;

    char buf[128];
    snprintf(buf, sizeof(buf), "H|%d,%d,%d,%.1f,%u",
        enableOk ? 1 : 0,
        motionOk ? 1 : 0,
        forceOk ? 1 : 0,
        pingMs,
        uptimeS);
    sendRelayUpdate(buf);
}

void RelayCore::reportDiagnostic(int errorCode, double speedFactor, const char* reason) {
    char buf[256];
    snprintf(buf,sizeof(buf),"D|%d,%.2f,%.200s",errorCode,speedFactor,reason?reason:"");
    sendRelayUpdate(buf);
}

// ===== ForceReader 管理 =====

// Drag mode callback for ForceCalibration
static void calibDragMode(bool enable) {
    if (enable) {
        robotSendEnable("SetCollideDrag(1)");
        Sleep(100);
        robotDrainEnable();
        std::cout << "[Force] Drag mode ON — manually rotate end effector" << std::endl;
    } else {
        robotSendEnable("SetCollideDrag(0)");
        Sleep(100);
        robotDrainEnable();
        std::cout << "[Force] Drag mode OFF — position locked" << std::endl;
    }
}

bool RelayCore::initForceReader() {
    if (!isRobotConnected()) {
        std::cout << "[Force] Robot not connected, skipping ForceReader" << std::endl;
        return false;
    }
    ForcePipeline::init();
    ForceCompensation::init();
    ForceCalibration::setDragModeCallback(calibDragMode);
    m_forceThread = CreateThread(NULL, 0, forceReaderThread, NULL, 0, NULL);
    if (!m_forceThread) {
        std::cerr << "[Force] Failed to create ForceReader thread" << std::endl;
        return false;
    }
    return true;
}

void RelayCore::pollForce() {
    static DWORD lastPollMs = 0;
    DWORD now = GetTickCount();
    if (now - lastPollMs < 33) return;
    lastPollMs = now;

    auto& app = appState;

    // Read current pose for compensation
    double pose[6] = {0};
    EnterCriticalSection(&app.robotPoseMutex);
    pose[0] = app.robotActualPose.x;
    pose[1] = app.robotActualPose.y;
    pose[2] = app.robotActualPose.z;
    pose[3] = app.robotActualPose.rx;
    pose[4] = app.robotActualPose.ry;
    pose[5] = app.robotActualPose.rz;
    LeaveCriticalSection(&app.robotPoseMutex);

    EnterCriticalSection(&app.forceDataMutex);

    // Staleness check
    if (app.forceData.lastUpdateMs > 0 &&
        (now - app.forceData.lastUpdateMs) > static_cast<DWORD>(Config::FORCE_STALE_MS)) {
        app.forceData.isStale = true;
        for (int i = 0; i < 6; i++) app.forceData.filtered[i] = 0.0;
        for (int i = 0; i < 6; i++) app.forceData.compensated[i] = 0.0;
        for (int i = 0; i < 3; i++) app.forceData.hapticOut[i] = 0.0;
    }

    // Run calibration state machine if active (uses raw data directly)
    if (ForceCalibration::isRunning()) {
        ForceCalibration::update(0.033, app.forceData.raw, pose);
        if (ForceCalibration::isDone()) {
            // Apply results handled in idle() / keyboard callback
        }
    }

    // Run compensation (uses calibrated params if available)
    ForceCompensation::step(app.forceData, pose);

    // Run pipeline on compensated data
    ForcePipeline::step(app.forceData);

    // Build F| protocol message — send filtered[] with deadzone applied
    char buf[128];
    if (app.forceData.isStale) {
        snprintf(buf, sizeof(buf), "F|0.00,0.00,0.00,0.00,0.00,0.00,1");
    } else {
        double dz = Config::FORCE_RESIDUAL_DEADZONE_N;
        double fx = (fabs(app.forceData.filtered[0]) < dz) ? 0.0 : app.forceData.filtered[0];
        double fy = (fabs(app.forceData.filtered[1]) < dz) ? 0.0 : app.forceData.filtered[1];
        double fz = (fabs(app.forceData.filtered[2]) < dz) ? 0.0 : app.forceData.filtered[2];
        snprintf(buf, sizeof(buf), "F|%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d",
            fx, fy, fz,
            app.forceData.filtered[3], app.forceData.filtered[4],
            app.forceData.filtered[5],
            app.forceData.isStale ? 1 : 0);
    }
    LeaveCriticalSection(&app.forceDataMutex);

    sendRelayUpdate(buf);
}

void RelayCore::shutdownForceReader() {
    if (m_forceThread) {
        WaitForSingleObject(m_forceThread, 1000);
        CloseHandle(m_forceThread);
        m_forceThread = NULL;
    }
    robotCloseRealtime();
    ForcePipeline::shutdown();
    ForceCompensation::shutdown();
}

// ===== 力传感器标定控制 =====

bool RelayCore::startForceCalibration() {
    if (m_transmitting) {
        std::cout << "[Force] Cannot calibrate while transmitting — release button first" << std::endl;
        return false;
    }
    if (!isRobotConnected()) {
        std::cout << "[Force] Robot not connected, cannot calibrate" << std::endl;
        return false;
    }
    auto& app = appState;
    if (app.isRobotInAlarm.load()) {
        std::cout << "[Force] Robot in alarm, cannot calibrate" << std::endl;
        return false;
    }
    std::cout << "[Force] Starting calibration sweep..." << std::endl;
    return ForceCalibration::start();
}

void RelayCore::abortForceCalibration() {
    ForceCalibration::abort();
    std::cout << "[Force] Calibration aborted" << std::endl;
}

bool RelayCore::isForceCalibrating() const {
    return ForceCalibration::isRunning();
}

bool RelayCore::isForceCalibrationDone() const {
    return ForceCalibration::isDone();
}

const char* RelayCore::forceCalibStatus() const {
    return ForceCalibration::statusText();
}
