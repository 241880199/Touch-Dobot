#include "RobotController.h"
#include "CommandBuilder.h"
#include "SafetyBoundary.h"
#include "../network/TcpClient.h"
#include "../core/AppState.h"
#include "../core/CoordinateTransform.h"
#include "../config/Config.h"
#include <iostream>
#include <cstring>
#include <string>

// ===== 核心：发送单条坐标指令（热路径，无阻塞） =====
bool sendCoordinate(double deltaX, double deltaY, double deltaZ) {
    auto& app = appState;

    if (app.isClosing || !app.isTcpConnected || !app.isRobotBaseSet) {
        return false;
    }

    // 1. 计算绝对目标位置
    Vec3 delta(deltaX, deltaY, deltaZ);
    Vec3 target = computeTarget(app.robotBase, delta);

    // 2. 安全边界钳位
    target = SafetyBoundary::clampToBoundary(target);

    // 3. 构造 ServoP 指令（固定姿态）
    std::string cmd = CommandBuilder::buildServoP(target, app.robotBaseRx, app.robotBaseRy, app.robotBaseRz);

    // 4. 异步发送（不等待反馈）
    return sendToRelay(Config::MOTION_PORT, cmd.c_str());
}

// ===== 初始化机械臂 =====
bool initRobot() {
    // 清除报警
    if (!sendToRelay(Config::ENABLE_PORT, CommandBuilder::buildClearError().c_str())) {
        std::cerr << "清除报警失败" << std::endl;
        return false;
    }
    Sleep(300);

    // 上使能
    if (!sendToRelay(Config::ENABLE_PORT, CommandBuilder::buildEnableRobot().c_str())) {
        std::cerr << "使能失败" << std::endl;
        return false;
    }
    std::cout << "机械臂使能成功" << std::endl;

    // 设置平滑过渡
    std::string cpCmd = CommandBuilder::buildCP(Config::CP_SMOOTH_RATIO);
    if (!sendToRelay(Config::ENABLE_PORT, cpCmd.c_str())) {
        std::cerr << "设置 CP 失败" << std::endl;
        return false;
    }

    // 获取基准位置
    if (!sendToRelay(Config::ENABLE_PORT, CommandBuilder::buildGetPose().c_str())) {
        std::cerr << "获取基准位置失败" << std::endl;
        return false;
    }

    char feedback[1024];
    if (readFeedback(feedback, sizeof(feedback), Config::FEEDBACK_TIMEOUT)) {
        char* s = strchr(feedback, '{');
        char* e = strchr(feedback, '}');
        if (s && e && s < e) {
            *e = '\0';
            if (sscanf_s(s + 1, "%lf,%lf,%lf,%lf,%lf,%lf",
                &appState.robotBase.x, &appState.robotBase.y, &appState.robotBase.z,
                &appState.robotBaseRx, &appState.robotBaseRy, &appState.robotBaseRz) == 6) {
                appState.isRobotBaseSet = true;
                std::cout << "机械臂基准位置: (" << appState.robotBase.x << ","
                          << appState.robotBase.y << "," << appState.robotBase.z << ")" << std::endl;
                return true;
            }
        }
    }
    std::cerr << "解析基准位置失败" << std::endl;
    return false;
}

// ===== 关闭机械臂 =====
void shutdownRobot() {
    sendToRelay(Config::ENABLE_PORT, CommandBuilder::buildDisableRobot().c_str());
    Sleep(100);
}

// ===== 报警巡检 =====
void checkAlarmPeriodically() {
    auto& app = appState;
    if (!app.isTcpConnected) return;

    sendToRelay(Config::ENABLE_PORT, CommandBuilder::buildRobotMode().c_str());

    char feedback[1024];
    if (readFeedback(feedback, sizeof(feedback), 1000)) {
        char* s = strchr(feedback, '{');
        if (s) {
            int mode = atoi(s + 1);
            bool wasAlarm = app.isRobotInAlarm.exchange(mode == 9);
            if (mode == 9 && !wasAlarm) {
                std::cout << "[Alarm] 检测到机械臂报警 (mode=9)" << std::endl;
            }
        }
    }
}

// ===== 清除报警 =====
bool clearAlarm() {
    if (!sendToRelay(Config::ENABLE_PORT, CommandBuilder::buildClearError().c_str())) {
        return false;
    }
    Sleep(300);
    // 重新使能
    return sendToRelay(Config::ENABLE_PORT, CommandBuilder::buildEnableRobot().c_str());
}
