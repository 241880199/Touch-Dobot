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
    WSACleanup();
}

AppState appState;  // 全局单例
