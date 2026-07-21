#include "RelayCore.h"
#include "FeedbackParser.h"
#include "SafetyBoundary.h"
#include "../robot/RobotConnection.h"
#include "../core/AppState.h"
#include "../config/Config.h"
#include <iostream>
#include <windows.h>

RelayCore& RelayCore::instance() {
    static RelayCore inst;
    return inst;
}

RelayCore::RelayCore() {
    InitializeCriticalSection(&m_basePointLock);
}

RelayCore::~RelayCore() {
    DeleteCriticalSection(&m_basePointLock);
}

bool RelayCore::init() {
    if (!robotConnect(Config::ROBOT_IP)) {
        std::cerr << "[Relay] 连接机械臂失败" << std::endl;
        return false;
    }

    // 初始化序列：ClearError → EnableRobot → CP → GetPose
    Sleep(200);
    robotSendEnable("ClearError()");
    Sleep(300);

    if (!robotSendEnable("EnableRobot(0.5,0,0,0)")) {
        std::cerr << "[Relay] 使能失败" << std::endl;
        return false;
    }
    std::cout << "[Relay] 机械臂使能成功" << std::endl;
    Sleep(200);

    char cpBuf[64];
    snprintf(cpBuf, sizeof(cpBuf), "CP(%u)", Config::CP_SMOOTH_RATIO);
    robotSendEnable(cpBuf);
    Sleep(100);

    // 获取基准位姿
    robotSendEnable("GetPose()");
    Sleep(200);
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
        }
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

    // 坐标转换
    Vec3 current = convertTouchToRobot(devicePos);
    EnterCriticalSection(&m_basePointLock);
    Vec3 basePoint = m_basePoint;
    LeaveCriticalSection(&m_basePointLock);
    Vec3 delta = computeDelta(current, basePoint);

    // 跳过微小移动
    if (delta.length() < Config::MIN_DELTA_THRESHOLD) return;

    auto& app = appState;
    Vec3 base(app.robotBase.x, app.robotBase.y, app.robotBase.z);
    Vec3 target = computeTarget(base, delta);

    // 安全边界
    target = SafetyBoundary::clampToBoundary(target);

    // 构造指令 (栈分配，无堆内存分配)
    char cmd[256];
    snprintf(cmd, sizeof(cmd), "ServoP(%.2f,%.2f,%.2f,%.2f,%.2f,%.2f)",
        target.x, target.y, target.z, app.robotBaseRx, app.robotBaseRy, app.robotBaseRz);

    // 给扩展插件修改指令的机会
    RobotCommand robotCmd;
    robotCmd.cmd = cmd;
    robotCmd.targetPort = Config::MOTION_PORT;
    for (auto* ext : m_extensions) {
        ext->onBeforeSend(robotCmd);
    }

    // 发送
    robotSendMotion(robotCmd.cmd.c_str());

    // 记录到最后指令
    EnterCriticalSection(&app.lastCommandMutex);
    strncpy_s(app.lastCommandSent, robotCmd.cmd.c_str(), sizeof(app.lastCommandSent) - 1);
    LeaveCriticalSection(&app.lastCommandMutex);

    // 记录到指令日志
    EnterCriticalSection(&app.commandLogMutex);
    strncpy_s(app.commandLog[app.commandLogIdx], robotCmd.cmd.c_str(), sizeof(app.commandLog[0]) - 1);
    app.commandLogIdx = (app.commandLogIdx + 1) % AppState::LOG_SIZE;
    if (app.commandLogCount < AppState::LOG_SIZE) app.commandLogCount++;
    LeaveCriticalSection(&app.commandLogMutex);

    // 更新目标位姿
    EnterCriticalSection(&app.robotPoseMutex);
    app.robotTargetPose.x = target.x;
    app.robotTargetPose.y = target.y;
    app.robotTargetPose.z = target.z;
    LeaveCriticalSection(&app.robotPoseMutex);
}

void RelayCore::onButtonPress(const Vec3& robotPos) {
    EnterCriticalSection(&m_basePointLock);
    m_basePoint = robotPos;
    LeaveCriticalSection(&m_basePointLock);
    m_basePointSet = true;
    m_transmitting = true;
}

void RelayCore::onButtonRelease() {
    m_transmitting = false;
    m_basePointSet = false;
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
    // 读取运动端口反馈
    while (robotRecvMotion(buf, sizeof(buf))) {
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

    // 读取使能端口反馈
    while (robotRecvEnable(buf, sizeof(buf))) {
        RobotFeedback fb;
        strncpy_s(fb.raw, buf, sizeof(fb.raw) - 1);
        fb.fromPort = Config::ENABLE_PORT;
        fb.errorId = (buf[0] == '0') ? 0 : -1;

        char shortMsg[256];
        snprintf(shortMsg, sizeof(shortMsg), "%.200s", fb.raw);
        logFeedback(shortMsg, "29999");

        for (auto* ext : m_extensions) {
            ext->onAfterFeedback(fb);
        }
    }
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
        LeaveCriticalSection(&app.robotPoseMutex);
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
            std::cout << "[Relay] 检测到机械臂报警 (mode=9)" << std::endl;
        }
    }
}

void RelayCore::registerExtension(IExtension* ext) {
    m_extensions.push_back(ext);
}
