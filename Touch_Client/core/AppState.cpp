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
    InitializeCriticalSection(&forceDataMutex);
    InitializeCriticalSection(&stylusOrientMutex);
    InitializeCriticalSection(&orientForceMutex);
    orientExtraForce[0] = orientExtraForce[1] = orientExtraForce[2] = 0.0;
    hasOrientExtraForce = false;

    InitializeCriticalSection(&orientRepulsionMutex);
    orientRepulsionForce[0] = orientRepulsionForce[1] = orientRepulsionForce[2] = 0.0;
    hasOrientRepulsion = false;

    // 大数组用 ZeroMemory 避免 in-class initializer 与 MSVC 的兼容问题
    ZeroMemory(commandLog, sizeof(commandLog));
    ZeroMemory(feedbackLog, sizeof(feedbackLog));
    ZeroMemory(&forceData, sizeof(forceData));
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
    DeleteCriticalSection(&forceDataMutex);
    DeleteCriticalSection(&stylusOrientMutex);
    DeleteCriticalSection(&orientForceMutex);
    DeleteCriticalSection(&orientRepulsionMutex);
    WSACleanup();
}

AppState appState;  // 全局单例
