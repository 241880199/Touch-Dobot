#define _USE_MATH_DEFINES
#include "RelayCore.h"
#include "FeedbackParser.h"
#include "RelayCommandParser.h"
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
#include "../force/ForceLogger.h"
#include "../force/PayloadCalibration.h"
#include "../calibration/TcpCalibration.h"
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

            // 一次性回读: 机械臂实际上在用哪份负载参数 (下发成功 ≠ 机械臂采纳;
            // EnableRobot 的返回码只能说明语法对了)。
            // 30004 布局: Load @1168 (1×double), CenterX/Y/Z @1176~1199 (3×double)。
            // ⚠ 现在只报不判: 与本客户端的值对不上【不】当故障 —— 负载确实是生效的
            //   (连接时序里随 EnableRobot 下发, 2026-09-19 实机证实运行中改负载会让机械臂动),
            //   但早先的探针里 ActualTCPForce @576 没跟着 0.25 kg 的配置变化走, 为什么还不清楚。
            //   本回读仅供诊断, 不影响标定 —— 真正生效的是本地补偿。
            static bool loadEchoReported = false;
            if (!loadEchoReported) {
                loadEchoReported = true;
                const double* echo = reinterpret_cast<const double*>(buf + 1168);
                bool sane = echo[0] >= 0.0 && echo[0] <= 5.0
                         && fabs(echo[1]) <= 500.0 && fabs(echo[2]) <= 500.0
                         && fabs(echo[3]) <= 500.0;
                if (sane) {
                    double mWant, cWant[3];
                    PayloadCalibration::effective(mWant, cWant);
                    char msg[192];
                    snprintf(msg, sizeof(msg),
                             "[Relay] 机械臂实际负载: load=%.3f kg  center=(%.1f, %.1f, %.1f) mm",
                             echo[0], echo[1], echo[2], echo[3]);
                    if (fabs(echo[0] - mWant) > 0.01 || fabs(echo[3] - cWant[2]) > 1.0) {
                        std::cout << msg << "\n[Relay] · 与客户端的 "
                                  << mWant << " kg / (" << cWant[0] << "," << cWant[1] << ","
                                  << cWant[2] << ") mm 不一致 — 负载只在连接时下发, 运行中"
                                  << "【不】改 (改负载会让机械臂动), 所以对不上是常见情形"
                                  << std::endl;
                        std::cout << "[Relay]   重力/惯性补偿由 ForceCompensation 在本地做"
                                  << " (差值来源尚未查清, 见上方注释)" << std::endl;
                    } else {
                        std::cout << msg << std::endl;
                    }
                    // 落进 ForceData: 求解负载时的【基线】用 (main.cpp 的 solveAndApply)。
                    // 存机械臂自报的值而非我们下发的值 —— 要的是"它实际在用哪个"。
                    EnterCriticalSection(&app.forceDataMutex);
                    app.forceData.payloadEchoLoadKg = echo[0];
                    for (int i = 0; i < 3; i++) {
                        app.forceData.payloadEchoCenterMm[i] = echo[1 + i];
                    }
                    app.forceData.payloadEchoValid = true;
                    LeaveCriticalSection(&app.forceDataMutex);
                }
            }

            // Parse ActualTCPForce at offset 576 (6 doubles, 48 bytes)
            double* forcePtr = reinterpret_cast<double*>(buf + 576);
            // 同一帧里的 TCPForce @720 —— 与 @576 是两个不同的量 (见 AppState.h 的说明)。
            // 实测改 EnableRobot 的负载时 @576 不变, 所以两路都留着, 供以后对比/诊断。
            double* tcpForcePtr = reinterpret_cast<double*>(buf + 720);
            // 同一帧里的 ToolVectorActual @624 / TCPSpeedActual @672 —— 坐标系未确认
            // (见 AppState.h 的说明)。现在只镜像进 ForceData, 供运动探针并排打印对照。
            const double* tcpPosePtr = reinterpret_cast<const double*>(buf + 624);
            const double* tcpSpeedPtr = reinterpret_cast<const double*>(buf + 672);
            // 同一帧里的 SixForceValue @1304 = "当前六维力数据原始值" —— 与 @576 的派生量
            // 是【两个不同的量】, 见 AppState.h。六维力在线状态 @1037 (char, 单字节)。
            // 帧长 1440: 1304+48=1352 与 1037 都在范围内。
            double* sixForcePtr = reinterpret_cast<double*>(buf + 1304);
            const int sixForceOnline = static_cast<int>(static_cast<unsigned char>(buf[1037]));
            EnterCriticalSection(&app.forceDataMutex);
            for (int i = 0; i < 6; i++) {
                app.forceData.raw[i] = forcePtr[i];
                app.forceData.tcpForce[i] = tcpForcePtr[i];
                app.forceData.tcpPoseActual[i] = tcpPosePtr[i];
                app.forceData.tcpSpeedActual[i] = tcpSpeedPtr[i];
                app.forceData.sixForceRaw[i] = sixForcePtr[i];
            }
            app.forceData.sixForceOnline = sixForceOnline;
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

// ===== 使能机械臂 (带末端负载参数) =====
// EnableRobot(load, centerX, centerY, centerZ) — 机械臂内部按此做重力/惯性补偿。
// 负载设置不准 → 30004 力值随姿态漂移 / 碰撞检测误触发 / 拖拽失控。
// 负载值优先取实机标定结果 (PayloadCalibration / payload_calib.json),
// 未标定时回退 Config.h 的种子值。
//
// 连接时真正下发的那份负载 —— 机械臂在【整个会话】里用的就是这一份做重力/惯性补偿。
// 运行中的重新使能 (脱困 / 报警恢复) 必须原样回放它, 【不能】再去读 effective():
// 标定求解 (main.cpp 的 's') 会把内存生效值改成新值 (PayloadCalibration::applyResult),
// 而运行中改负载恰恰是能让机械臂猛地动起来的操作 (2026-09-19 实机证实: 改成 1.5 kg
// 后重启, 机械臂快速撞向关节限位)。它同时会让本地残余补偿多减一份, 即同一个误差减两次。
static bool s_sentPayloadValid = false;
static double s_sentPayloadMassKg = 0.0;
static double s_sentPayloadComMm[3] = {0.0, 0.0, 0.0};

// 参数化的发送器: 负载是入参, 不在这里隐式读全局生效值。
// note 仅供日志标注 (可为 nullptr), 不影响下发内容。
static bool sendEnableRobotWithPayload(double massKg, const double com[3],
                                       const char* note = nullptr) {
    char cmd[96];
    snprintf(cmd, sizeof(cmd), "EnableRobot(%.3f,%.1f,%.1f,%.1f)", massKg, com[0], com[1], com[2]);
    std::cout << "[Relay] 使能 " << cmd;
    if (note) std::cout << "  (" << note << ")";
    std::cout << std::endl;
    return robotSendEnable(cmd);
}

// 【只允许连接时序 (init) 调用】—— 它下发当前生效值, 并把下发成功的那份记成快照。
// 若在运行中调用, 快照就会被"新解出的"负载覆写, 那正是本文件要避免的事。
static bool enableRobotWithPayload() {
    double m, c[3];
    PayloadCalibration::effective(m, c);
    bool ok = sendEnableRobotWithPayload(
        m, c, PayloadCalibration::enabled ? "实机标定值" : "种子值, 未标定");
    if (ok) {
        // 成功之后才记: 直到进程重启, 机械臂用的就是这一份, 运行中的重新使能只回放它。
        s_sentPayloadMassKg = m;
        s_sentPayloadComMm[0] = c[0];
        s_sentPayloadComMm[1] = c[1];
        s_sentPayloadComMm[2] = c[2];
        s_sentPayloadValid = true;
    }
    return ok;
}

// 运行中的重新使能 (脱困 / 报警恢复) 专用: 回放连接时真正下发的那份负载。
static bool reenableRobotWithConnectPayload() {
    if (!s_sentPayloadValid) {
        // 连接时的使能没成功过, 快照不可用, 只能回退到当前生效值 —— 而它【未必】是机械臂
        // 此刻实际在用的负载, 所以这条路径必须把话说明白。
        double m, c[3];
        PayloadCalibration::effective(m, c);
        std::cout << "[Relay] 警告: 无连接时的负载快照 (使能未成功过), 回退到当前生效值 — "
                  << "它未必是机械臂此刻实际在用的负载" << std::endl;
        return sendEnableRobotWithPayload(m, c, "回退: 非连接时下发值, 未必是机械臂在用的负载");
    }
    return sendEnableRobotWithPayload(s_sentPayloadMassKg, s_sentPayloadComMm,
                                      "回放连接时的负载");
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
    // 【必须】回放连接时下发的那份负载, 不能用 effective(): 内存生效值可能已被本会话的
    // 标定求解改掉 (main.cpp 's' → PayloadCalibration::applyResult), 而带着新负载重新
    // 使能正是能让机械臂猛地动起来的操作 —— 脱困时操作员的手可能就在设备上。
    reenableRobotWithConnectPayload();
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

    if (!enableRobotWithPayload()) {
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
    // Default to robot's current actual orientation (not startup base),
    // so IK failure or mode switch doesn't snap back to a stale pose.
    double targetRx, targetRy, targetRz;
    {
        EnterCriticalSection(&app.robotPoseMutex);
        targetRx = app.robotActualPose.rx;
        targetRy = app.robotActualPose.ry;
        targetRz = app.robotActualPose.rz;
        LeaveCriticalSection(&app.robotPoseMutex);
    }

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
        // else: IK failed — keep current actual orientation (no snap-back)
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

            // Flip rotation sign: Touch stylus rotation direction → Dobot RPY convention.
            // Touch Euler angles (ZYX intrinsic) increase counter-clockwise looking
            // along the positive axis, but Dobot RPY follows the opposite convention.
            drx = -drx;
            dry = -dry;
            drz = -drz;

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
            // 【必须】回放连接时下发的那份负载, 不能用 effective(): 内存生效值可能已被本
            // 会话的标定求解改掉, 而运行中改负载会让机械臂动 (理由同 escapeSingularity)。
            robotSendEnable("ClearError()");
            Sleep(200);
            if (!reenableRobotWithConnectPayload()) {
                // 使能失败却照样 onRecovery() 会让上层以为手臂已可用 (实际还在下使能状态)
                std::cerr << "[Relay] 恢复失败: EnableRobot 未成功, 保持报警状态" << std::endl;
                app.isRobotInAlarm = true;
                return;
            }
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

void RelayCore::dispatchRelayCommand(const char* line) {
    using R = RelayCommandParser::Command;
    switch (RelayCommandParser::parse(line)) {
    case R::ForceFeedbackOn:
        appState.forceFeedbackEnabled = true;
        std::cout << "[Relay] Force feedback ENABLED (MATLAB command)" << std::endl;
        break;
    case R::ForceFeedbackOff:
        appState.forceFeedbackEnabled = false;
        std::cout << "[Relay] Force feedback DISABLED (MATLAB command)" << std::endl;
        break;
    case R::None:
    default:
        break;
    }
}

void RelayCore::pollRelayCommands() {
    EnterCriticalSection(&m_relaySocketMutex);
    SOCKET sock = m_relaySocket;
    LeaveCriticalSection(&m_relaySocketMutex);
    if (sock == INVALID_SOCKET) return;

    // 非阻塞检查可读数据
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(sock, &readfds);
    timeval tv = { 0, 0 };
    if (select(0, &readfds, nullptr, nullptr, &tv) <= 0) return;

    char tmp[256];
    int n = recv(sock, tmp, sizeof(tmp), 0);
    if (n <= 0) return;

    // 追加到接收缓冲, 逐行分发
    for (int i = 0; i < n; ++i) {
        char c = tmp[i];
        if (c == '\n') {
            m_relayRecvBuf[m_relayRecvLen] = '\0';
            dispatchRelayCommand(m_relayRecvBuf);
            m_relayRecvLen = 0;
        } else if (c != '\r' && m_relayRecvLen < (int)sizeof(m_relayRecvBuf) - 1) {
            m_relayRecvBuf[m_relayRecvLen++] = c;
        }
    }
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

// 拖拽模式 = Dobot 的 SetCollideDrag。开着时机械臂柔顺可手动拖动,
// 但姿态会漂, 所以采样/标定期间必须关掉。
bool RelayCore::setDragMode(bool enable) {
    if (!isRobotConnected()) {
        // 重连会重建连接, 拖拽状态随之归零 —— 别留一个「以为开着」的假状态,
        // 否则掉线时按 'd' 会永远切不回来。
        m_dragMode.store(false);
        return false;
    }
    if (m_dragMode.load() == enable) return true;   // 已是该状态, 不重复下发
    robotSendEnable(enable ? "SetCollideDrag(1)" : "SetCollideDrag(0)");
    Sleep(100);
    robotDrainEnable();
    m_dragMode.store(enable);
    std::cout << "[Relay] Drag mode " << (enable ? "ON — 可手动拖动机械臂" : "OFF — 位姿锁定")
              << std::endl;
    return true;
}

// Drag mode callback for ForceCalibration
static void calibDragMode(bool enable) {
    RelayCore::instance().setDragMode(enable);
}

bool RelayCore::initForceReader() {
    if (!isRobotConnected()) {
        std::cout << "[Force] Robot not connected, skipping ForceReader" << std::endl;
        return false;
    }
    ForcePipeline::init();
    ForceCompensation::init();
    ForceCalibration::setDragModeCallback(calibDragMode);
    if (!ForceLogger::open(Config::FORCE_LOG_PATH)) {
        std::cerr << "[Force] Failed to open force log " << Config::FORCE_LOG_PATH << std::endl;
    }
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

    // Run calibration state machine if active.
    // ⚠ 喂的是 @1304 (sixForceRaw) —— 调零定的零偏是【全量模型那个通道】的零偏
    //   (ForceCompensation::step 现在补偿 @1304, 见 Task 6)。继续喂 @576 会让 TARE
    //   平均出另一路量的零偏, 而两路的零偏不是一回事 (12:38 那份夹具上逐轴均值差
    //   19.8 / 1.6 / 1.7 N, 见 tests/fixtures/calib_poses_2026-09-19.txt):
    //   扣错以后读数依旧是个 N, 不报错。
    if (ForceCalibration::isRunning()) {
        ForceCalibration::update(0.033, app.forceData.sixForceRaw, pose);
        if (ForceCalibration::isDone()) {
            // Apply results handled in idle() / keyboard callback
        }
    }

    // Run compensation (uses calibrated params if available).
    // ⚠ 一致性闸门在这里面: 模型缺失或与 @576 对不上时, step() 会把 compensated[] 全置零
    //   (haptic / 约束力 / F| 一起断), 并把状态留在 ForceCompensation::guardState()。
    ForceCompensation::step(app.forceData, pose);
    const ForceCompensation::GuardState guardSt = ForceCompensation::guardState();

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

    // Copy filtered forces into a local before releasing the lock so the
    // log write (stdio buffering / disk) doesn't hold forceDataMutex across I/O.
    double filtered[6];
    for (int i = 0; i < 6; i++) filtered[i] = app.forceData.filtered[i];

    LeaveCriticalSection(&app.forceDataMutex);

    // 落盘到 CSV (演示对照实验用, 含 ff_enabled 标志列)。
    // TCP 偏移标定过之后记录笔尖世界坐标而不是法兰坐标 —— 演示要看的是笔尖
    // 在纸上的轨迹和受力, 法兰位姿差着笔长 + 夹持段 + 传感器高度 (约 100~200mm)。
    double logPose[6];
    if (TcpCalibration::enabled) {
        double tip[3];
        TcpCalibration::apply(pose, TcpCalibration::offset, tip);
        logPose[0] = tip[0]; logPose[1] = tip[1]; logPose[2] = tip[2];
        logPose[3] = pose[3]; logPose[4] = pose[4]; logPose[5] = pose[5]; // 姿态同法兰
    } else {
        for (int i = 0; i < 6; i++) logPose[i] = pose[i];
    }
    ForceLogger::log(now, filtered, logPose,
                     appState.forceFeedbackEnabled ? 1 : 0);

    sendRelayUpdate(buf);

    // ===== 一致性闸门的报错 (2026-09-19) =====
    // 走现成通道: RobotDiagnostics 记一条 (落 robot_diagnostics.log + 计数进会话报告),
    // 并经 reportDiagnostic 把 D| 帧发给 MATLAB GUI。data 侧已经由 step() 无条件置零,
    // 这里只管【把原因说清楚】。
    // 【两种原因用两个错误码】—— 处置一样 (都拒绝), 但操作员要做的事不同:
    //   ERR_FORCE_UNCALIBRATED -> 去按 'm'+'s' 重标模型;
    //   ERR_FORCE_INCONSISTENT -> 去查负载参数有没有真的发进机械臂 (Task 8)。
    // 合并成一个码会让这两件事在日志里长得一样, 而"该做什么"全靠这一位区分。
    // ⚠ 状态 -> 错误码的映射【只有一份实现】: ForceCompensation::guardErrorCode (见那里的
    //   说明)。这里从前是 static_cast<int>(guardState()) 比字面量 1 / 2 —— 一改枚举的
    //   数值就会把两条处置指引对调, 而且没有任何测试看得见。
    // 【只在状态变化时报, 不变的按 FORCE_GUARD_REPORT_MS 复报】—— 闸门每帧都判 (30Hz),
    // 每帧落一行会把诊断日志冲掉。
    {
        static int   lastGuardSt = -1;
        static DWORD lastGuardMs = 0;
        if (static_cast<int>(guardSt) != lastGuardSt ||
            (guardSt != ForceCompensation::GuardState::OK &&
             (now - lastGuardMs) > static_cast<DWORD>(Config::FORCE_GUARD_REPORT_MS))) {
            lastGuardSt = static_cast<int>(guardSt);
            lastGuardMs = now;
            if (guardSt != ForceCompensation::GuardState::OK) {
                RobotError err;
                err.code = ForceCompensation::guardErrorCode(guardSt);
                err.severity = getSeverity(err.code);
                err.timestampMs = GetTickCount64();
                EnterCriticalSection(&app.robotPoseMutex);
                err.targetPosition = Vec3(app.robotActualPose.x, app.robotActualPose.y,
                                          app.robotActualPose.z);
                LeaveCriticalSection(&app.robotPoseMutex);
                err.speedFactor = 0.0;   // 本帧的力数据未放行, 不参与任何速度调度
                RobotDiagnostics::instance().logError(err, 0.0, m_stateMachine.currentState());
            }
        }
    }
}

void RelayCore::shutdownForceReader() {
    if (m_forceThread) {
        WaitForSingleObject(m_forceThread, 1000);
        CloseHandle(m_forceThread);
        m_forceThread = NULL;
    }
    robotCloseRealtime();
    ForceLogger::close();
    ForcePipeline::shutdown();
    ForceCompensation::shutdown();
}

// ===== 力传感器标定控制 =====

// 力标定/调零的公共前置检查: 未传输中 + 已连接 + 未报警
static bool forceCalibPreconditions(bool transmitting, const char* what) {
    if (transmitting) {
        std::cout << "[Force] Cannot " << what
                  << " while transmitting — release button first" << std::endl;
        return false;
    }
    if (!isRobotConnected()) {
        std::cout << "[Force] Robot not connected, cannot " << what << std::endl;
        return false;
    }
    if (appState.isRobotInAlarm.load()) {
        std::cout << "[Force] Robot in alarm, cannot " << what << std::endl;
        return false;
    }
    return true;
}

bool RelayCore::startForceCalibration() {
    if (!forceCalibPreconditions(m_transmitting, "calibrate")) return false;
    std::cout << "[Force] Starting calibration sweep..." << std::endl;
    return ForceCalibration::start();
}

bool RelayCore::startForceZeroing() {
    if (!forceCalibPreconditions(m_transmitting, "zero")) return false;
    std::cout << "[Force] Starting zero (TARE only)..." << std::endl;
    return ForceCalibration::startZero();
}

void RelayCore::abortForceCalibration() {
    ForceCalibration::abort();
    std::cout << "[Force] Calibration aborted" << std::endl;
}

bool RelayCore::isForceCalibrating() const {
    return ForceCalibration::isRunning();
}

bool RelayCore::isForceZeroing() const {
    return ForceCalibration::isZeroing();
}

bool RelayCore::isForceCalibrationDone() const {
    return ForceCalibration::isDone();
}

const char* RelayCore::forceCalibStatus() const {
    return ForceCalibration::statusText();
}
