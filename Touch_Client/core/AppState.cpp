#include "AppState.h"
#include <iostream>

AppState::AppState() {
    WSADATA wsaData;
    WSAStartup(MAKEWORD(2, 2), &wsaData);

    InitializeCriticalSection(&devicePosMutex);
    InitializeCriticalSection(&adjustedPosMutex);
    InitializeCriticalSection(&adjustedPosTableMutex);
    InitializeCriticalSection(&robotSocketMutex);
    InitializeCriticalSection(&robotPoseMutex);
    InitializeCriticalSection(&statusMutex);
    InitializeCriticalSection(&basePointMutex);
    InitializeCriticalSection(&trailMutex);
    InitializeCriticalSection(&lastCommandMutex);
    InitializeCriticalSection(&commandLogMutex);
    InitializeCriticalSection(&feedbackLogMutex);
    InitializeCriticalSection(&forceMutex);

    // 大数组用 ZeroMemory 避免 in-class initializer 与 MSVC 的兼容问题
    ZeroMemory(commandLog, sizeof(commandLog));
    ZeroMemory(feedbackLog, sizeof(feedbackLog));
    ZeroMemory(forceRaw, sizeof(forceRaw));
    ZeroMemory(forceFiltered, sizeof(forceFiltered));
}

AppState::~AppState() {
    DeleteCriticalSection(&devicePosMutex);
    DeleteCriticalSection(&adjustedPosMutex);
    DeleteCriticalSection(&adjustedPosTableMutex);
    DeleteCriticalSection(&robotSocketMutex);
    DeleteCriticalSection(&robotPoseMutex);
    DeleteCriticalSection(&statusMutex);
    DeleteCriticalSection(&basePointMutex);
    DeleteCriticalSection(&trailMutex);
    DeleteCriticalSection(&lastCommandMutex);
    DeleteCriticalSection(&commandLogMutex);
    DeleteCriticalSection(&feedbackLogMutex);
    DeleteCriticalSection(&forceMutex);
    WSACleanup();
}

AppState appState;  // 全局单例
