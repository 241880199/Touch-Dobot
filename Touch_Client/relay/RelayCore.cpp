#include "RelayCore.h"
#include "FeedbackParser.h"
#include "SafetyBoundary.h"
#include "../robot/RobotConnection.h"
#include "../core/AppState.h"
#include "../config/Config.h"
#include <iostream>
#include <windows.h>
#include "../safety/SafetyPredictor.h"

RelayCore& RelayCore::instance() {
    static RelayCore inst;
    return inst;
}

RelayCore::RelayCore() {
    InitializeCriticalSection(&m_basePointLock);
    InitializeCriticalSection(&m_relaySocketMutex);
}

RelayCore::~RelayCore() {
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
    std::cout << "[脱困] 检测到机械臂报警 (疑似奇异构型)" << std::endl;
    std::cout << "[脱困]   1. 把大臂/小臂折弯 (J2/J3 离开 0°)" << std::endl;
    std::cout << "[脱困]   2. 把手腕转一下 (J5 离开 0°)" << std::endl;
    std::cout << "[脱困]   3. 保持机械臂自然弯曲姿态" << std::endl;
    std::cout << "========================================" << std::endl;

    char fb[256];

    // Step 1: ClearError + EnableRobot
    robotSendEnable("ClearError()");
    Sleep(200);
    robotSendEnable("EnableRobot(0.5,0,0,0)");
    Sleep(300);

    // Step 2: BrakeControl 松 J2/J3/J5 抱闸 (不依赖 StartDrag)
    std::cout << "[脱困] 松开 J2/J3/J5 抱闸..." << std::endl;
    robotSendEnable("BrakeControl(2,1)"); Sleep(100); robotRecvEnable(fb, sizeof(fb));
    robotSendEnable("BrakeControl(3,1)"); Sleep(100); robotRecvEnable(fb, sizeof(fb));
    robotSendEnable("BrakeControl(5,1)"); Sleep(100); robotRecvEnable(fb, sizeof(fb));

    std::cout << "[脱困] 抱闸已松开，请手动拖动机械臂..." << std::endl;
    std::cout << "[脱困] 拖动完成后按 Enter 继续" << std::endl;

    // Step 3: 等待用户手动拖动
    for (int i = 0; i < 120; i++) {
        if (GetAsyncKeyState(VK_RETURN) & 0x8000) {
            std::cout << "[脱困] 用户按下 Enter" << std::endl;
            while (GetAsyncKeyState(VK_RETURN) & 0x8000) Sleep(10);
            break;
        }
        if (i % 5 == 0 && i > 0) {
            std::cout << "[脱困] 等待中... (" << (120 - i) << "s) 按 Enter 继续" << std::endl;
        }
        Sleep(1000);
    }

    // Step 4: 锁回抱闸
    std::cout << "[脱困] 锁回 J2/J3/J5 抱闸..." << std::endl;
    robotSendEnable("BrakeControl(2,0)"); Sleep(100); robotRecvEnable(fb, sizeof(fb));
    robotSendEnable("BrakeControl(3,0)"); Sleep(100); robotRecvEnable(fb, sizeof(fb));
    robotSendEnable("BrakeControl(5,0)"); Sleep(100); robotRecvEnable(fb, sizeof(fb));

    // Step 5: 重新使能
    robotSendEnable("ClearError()");
    Sleep(200);
    robotSendEnable("EnableRobot(0.5,0,0,0)");
    Sleep(300);

    // 再次检查 RobotMode
    robotSendEnable("RobotMode()");
    Sleep(50);
    if (robotRecvEnable(fb, sizeof(fb))) {
        int mode = -1;
        FeedbackParser::parseMode(fb, mode);
        if (mode != 9 && mode != -1) {
            std::cout << "[脱困] 重新使能成功 (mode=" << mode << ")" << std::endl;
            s_escaping = false;
            return true;
        }
    }

    std::cerr << "[脱困] 脱困失败，机械臂仍处于报警状态" << std::endl;
    s_escaping = false;
    return false;
}

bool RelayCore::init() {
    if (!robotConnect(Config::ROBOT_IP)) {
        std::cerr << "[Relay] 连接机械臂失败" << std::endl;
        return false;
    }

    // 初始化序列：ClearError → EnableRobot → (报警检测) → CP → GetPose
    Sleep(200);
    robotSendEnable("ClearError()");
    Sleep(300);

    if (!robotSendEnable("EnableRobot(0.5,0,0,0)")) {
        std::cerr << "[Relay] 使能失败" << std::endl;
        return false;
    }
    std::cout << "[Relay] 机械臂使能成功" << std::endl;
    Sleep(200);

    // ===== 奇异检测: 使能后检查是否立即报警 (重试3次, 每次100ms) =====
    {
        int mode = -1;
        for (int retry = 0; retry < 3; retry++) {
            robotSendEnable("RobotMode()");
            Sleep(100);
            char fb[128];
            if (robotRecvEnable(fb, sizeof(fb))) {
                // 过滤非JSON反馈 (movJ/servoP等响应)
                if (FeedbackParser::parseMode(fb, mode)) {
                    break;
                }
            }
            std::cout << "[Relay] RobotMode retry " << (retry + 1) << "/3..." << std::endl;
        }

        if (mode == 9) {
            std::cout << "[Relay] 使能后检测到报警 (mode=9)，启动脱困流程..." << std::endl;
            if (!escapeSingularity()) {
                std::cerr << "[Relay] FATAL: 脱困失败，请断电后手动将机械臂挪出奇异区再上电" << std::endl;
                robotSendEnable("DisableRobot()");
                Sleep(100);
                robotDisconnect();
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
        robotSendEnable("DisableRobot()");
        Sleep(100);
        robotDisconnect();
        return false;
    }

    return true;
}

void RelayCore::shutdown() {
    robotSendEnable("DisableRobot()");
    Sleep(100);
    robotDisconnect();
}

void RelayCore::sendPosition(const hduVector3Dd& devicePos) {
    if (!m_transmitting || !m_basePointSet || !isRobotConnected()) return;

    // ===== 安全守卫 =====
    if (!appState.isRobotBaseSet) return;

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

    // 计算增量
    double dx = current.x - m_lastTouchPos.x;
    double dy = current.y - m_lastTouchPos.y;
    double dz = current.z - m_lastTouchPos.z;

    // 更新 Touch 参考点
    m_lastTouchPos = current;

    // 跳过微小增量 (Touch 噪声)
    if (fabs(dx) < 0.05 && fabs(dy) < 0.05 && fabs(dz) < 0.05) {
        LeaveCriticalSection(&m_basePointLock);
        return;
    }

    // ===== 增量步长限制: 单步最大 3mm, 再乘以速度衰减因子 =====
    static double s_speedMul = 1.0;  // 跨帧持久, 由 SafetyPredictor 更新
    double len = sqrt(dx*dx + dy*dy + dz*dz);
    double maxStep = 3.0 * s_speedMul;
    if (len > maxStep) {
        double scale = maxStep / len;
        dx *= scale; dy *= scale; dz *= scale;
    }

    // 累加到目标位置
    m_targetPos.x += dx;
    m_targetPos.y += dy;
    m_targetPos.z += dz;

    // 安全边界钳位 (并同步 target 到钳位值，防止"卡边界")
    Vec3 clamped = SafetyBoundary::clampToBoundary(m_targetPos);
    m_targetPos = clamped;

    // ===== SafetyPredictor 预判 =====
    SafetyVerdict verdict = SafetyPredictor::instance().evaluate(clamped);

    if (verdict.action == SafetyVerdict::REJECT) {
        std::cerr << "[Safety] REJECT: " << verdict.reason
                  << " — target=(" << clamped.x << "," << clamped.y << "," << clamped.z << ")"
                  << std::endl;
        LeaveCriticalSection(&m_basePointLock);
        return;  // 不发送，目标位置回滚 (m_targetPos 已更新，但不下发)
    }

    // 更新速度衰减因子 (用于下帧)
    s_speedMul = (verdict.action == SafetyVerdict::WARN_SLOW) ? verdict.speedFactor : 1.0;
    LeaveCriticalSection(&m_basePointLock);

    // ===== 构造并发送 ServoP =====
    auto& app = appState;
    char cmd[256];
    snprintf(cmd, sizeof(cmd), "ServoP(%.2f,%.2f,%.2f,%.2f,%.2f,%.2f)",
        clamped.x, clamped.y, clamped.z,
        app.robotBaseRx, app.robotBaseRy, app.robotBaseRz);

    bool sent = robotSendMotion(cmd);
    static int sendCount = 0, failCount = 0;
    sendCount++;
    if (!sent) failCount++;
    if (sendCount % 50 == 0) {
        std::cout << "[Relay] Motion sends: " << sendCount
                  << " ok, " << failCount << " fail"
                  << "  target=(" << clamped.x << "," << clamped.y << "," << clamped.z << ")"
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
    app.robotTargetPose.x = clamped.x;
    app.robotTargetPose.y = clamped.y;
    app.robotTargetPose.z = clamped.z;
    LeaveCriticalSection(&app.robotPoseMutex);
}

void RelayCore::onButtonPress(const Vec3& robotPos) {
    EnterCriticalSection(&m_basePointLock);
    // 以机器人当前实际位姿作为 target 起点
    {
        auto& app = appState;
        EnterCriticalSection(&app.robotPoseMutex);
        m_targetPos.x = app.robotActualPose.x;
        m_targetPos.y = app.robotActualPose.y;
        m_targetPos.z = app.robotActualPose.z;
        LeaveCriticalSection(&app.robotPoseMutex);
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
    m_transmitting = false;
    m_basePointSet = false;
    m_lastTouchValid = false;
    std::cout << "[Relay] Button RELEASE — motion stopped" << std::endl;
}

static void logFeedback(const char* msg, const char* portLabel) {
    auto& app = appState;
    EnterCriticalSection(&app.feedbackLogMutex);
    snprintf(app.feedbackLog[app.feedbackLogIdx], sizeof(app.feedbackLog[0]),
        "[%s] %s", portLabel, msg);
    app.feedbackLogIdx = (app.feedbackLogIdx + 1) % AppState::LOG_SIZE;
    if (app.feedbackLogCount < AppState::LOG_SIZE) app.feedbackLogCount++;
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
}

void RelayCore::queryJointAngles() {
    if (!isRobotConnected()) return;
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
}

void RelayCore::checkAlarm() {
    if (!isRobotConnected()) return;
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
            } else {
                std::cout << "[Relay] 脱困失败，按 'e' 重试或重启程序" << std::endl;
            }
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

    snprintf(buf, sizeof(buf), "P|%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
        pos[0], pos[1], pos[2], 0.0, 0.0, 0.0);

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
