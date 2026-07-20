#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#include <HD/hd.h>
#include <HDU/hduVector.h>
#include <iostream>
#include <conio.h>

#include "config/Config.h"
#include "core/AppState.h"
#include "core/SenderQueue.h"
#include "haptic/HapticDevice.h"
#include "network/TcpClient.h"
#include "robot/RobotController.h"

// ===== 报警巡检定时器线程 =====
DWORD WINAPI alarmCheckThreadProc(LPVOID param) {
    auto& app = appState;
    while (!app.isClosing) {
        Sleep(Config::ALARM_CHECK_INTERVAL);
        if (app.isClosing) break;
        checkAlarmPeriodically();
    }
    return 0;
}

// ===== TCP 状态输出 =====
void printStatus() {
    auto& app = appState;
    const char* conn = app.isTcpConnected ? "CONNECTED" : "DISCONNECTED";
    const char* trans = app.isTransmitting ? "TX" : "--";
    std::cout << "\r[" << conn << "] [" << trans << "] "
              << "q=quit  (Touch button1: move)   "
              << std::flush;
}

// ===== 主函数 =====
int main() {
    std::cout << "=== Touch-Dobot Remote Control System ===" << std::endl;
    std::cout << "Relay: " << Config::TCP_RELAY_IP << ":" << Config::RELAY_PORT << std::endl;
    std::cout << "Safety: X[" << Config::SAFE_X_MIN << "," << Config::SAFE_X_MAX
              << "] Y[" << Config::SAFE_Y_MIN << "," << Config::SAFE_Y_MAX
              << "] Z[" << Config::SAFE_Z_MIN << "," << Config::SAFE_Z_MAX << "]" << std::endl;

    // 1. 初始化 Touch 设备
    std::cout << "Initializing Touch device..." << std::endl;
    if (!initHapticDevice()) {
        std::cerr << "ERROR: Touch device init failed" << std::endl;
        return -1;
    }
    std::cout << "Touch device ready." << std::endl;

    // 2. 连接中继站
    std::cout << "Connecting to relay station..." << std::endl;
    startTcpClient();
    Sleep(2000); // 等待连接建立

    if (!appState.isTcpConnected) {
        std::cerr << "ERROR: Failed to connect to relay station" << std::endl;
        cleanupHapticDevice();
        return -1;
    }
    std::cout << "Connected to relay station." << std::endl;

    // 3. 初始化机械臂
    std::cout << "Initializing robot..." << std::endl;
    if (!initRobot()) {
        std::cerr << "ERROR: Robot init failed" << std::endl;
        shutdownRobot();
        stopTcpClient();
        cleanupHapticDevice();
        return -1;
    }
    std::cout << "Robot ready. Base position: ("
              << appState.robotBase.x << ", "
              << appState.robotBase.y << ", "
              << appState.robotBase.z << ")" << std::endl;

    // 4. 启动发送线程
    startSenderThread();

    // 5. 启动报警巡检线程
    HANDLE alarmThread = CreateThread(NULL, 0, alarmCheckThreadProc, NULL, 0, NULL);

    // 6. 主循环
    std::cout << "\nSystem ready. Press Touch button 1 to control robot." << std::endl;
    std::cout << "Type 'q' and press Enter to quit.\n" << std::endl;

    while (!appState.isClosing) {
        if (_kbhit()) {
            char c = _getch();
            if (c == 'q' || c == 'Q') {
                std::cout << "\nShutting down..." << std::endl;
                break;
            }
        }
        printStatus();
        Sleep(200);
    }

    // 7. 清理
    appState.isClosing = true;
    stopSenderThread();
    WaitForSingleObject(alarmThread, 2000);
    CloseHandle(alarmThread);
    shutdownRobot();
    stopTcpClient();
    cleanupHapticDevice();

    std::cout << "\nProgram exited normally." << std::endl;
    return 0;
}
