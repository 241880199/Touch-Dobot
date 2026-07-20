#include "AppState.h"
#include <iostream>

AppState::AppState() {
    WSADATA wsaData;
    WSAStartup(MAKEWORD(2, 2), &wsaData);

    stopEvent = WSACreateEvent();
    if (stopEvent == WSA_INVALID_EVENT) {
        std::cerr << "创建 stopEvent 失败: " << WSAGetLastError() << std::endl;
    }

    InitializeCriticalSection(&devicePosMutex);
    InitializeCriticalSection(&adjustedPosMutex);
    InitializeCriticalSection(&adjustedPosTableMutex);
    InitializeCriticalSection(&relaySocketMutex);
    InitializeCriticalSection(&recvBufferMutex);
    InitializeCriticalSection(&statusMutex);
    InitializeCriticalSection(&basePointMutex);
    InitializeCriticalSection(&trailMutex);
}

AppState::~AppState() {
    if (stopEvent != WSA_INVALID_EVENT) {
        WSACloseEvent(stopEvent);
    }
    DeleteCriticalSection(&devicePosMutex);
    DeleteCriticalSection(&adjustedPosMutex);
    DeleteCriticalSection(&adjustedPosTableMutex);
    DeleteCriticalSection(&relaySocketMutex);
    DeleteCriticalSection(&recvBufferMutex);
    DeleteCriticalSection(&statusMutex);
    DeleteCriticalSection(&basePointMutex);
    DeleteCriticalSection(&trailMutex);
    WSACleanup();
}

AppState appState;  // 全局单例
