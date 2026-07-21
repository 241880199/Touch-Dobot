#include "SenderQueue.h"
#include "AppState.h"
#include "../config/Config.h"
#include <iostream>

// 前向声明（RobotController 中实现，编译时链接）
bool sendCoordinate(double deltaX, double deltaY, double deltaZ);

DWORD WINAPI senderThreadProc(LPVOID param) {
    std::cout << "发送线程启动" << std::endl;
    auto& app = appState;

    while (app.isSenderThreadRunning) {
        SendData data;
        bool hasData = false;

        {
            std::lock_guard<std::mutex> lock(app.queueMutex);
            if (!app.sendQueue.empty()) {
                data = app.sendQueue.front();
                app.sendQueue.pop();
                hasData = true;
            }
        }

        if (hasData) {
            sendCoordinate(data.deltaX, data.deltaY, data.deltaZ);
        } else {
            Sleep(Config::IDLE_SLEEP_MS);
        }
    }

    std::cout << "发送线程退出" << std::endl;
    return 0;
}

void startSenderThread() {
    appState.isSenderThreadRunning = true;
    appState.senderThread = CreateThread(NULL, 0, senderThreadProc, NULL, 0, NULL);
}

void stopSenderThread() {
    appState.isSenderThreadRunning = false;
    if (appState.senderThread) {
        WaitForSingleObject(appState.senderThread, 1000);
        CloseHandle(appState.senderThread);
        appState.senderThread = NULL;
    }
}
