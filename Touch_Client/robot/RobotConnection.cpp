#include "RobotConnection.h"
#include "../config/Config.h"
#include "../core/AppState.h"
#include <iostream>
#include <cstring>

#pragma comment(lib, "ws2_32.lib")

static SOCKET connectPort(const char* ip, int port) {
    SOCKET sock = WSASocket(AF_INET, SOCK_STREAM, IPPROTO_TCP, NULL, 0, 0);
    if (sock == INVALID_SOCKET) return INVALID_SOCKET;

    sockaddr_in addr;
    addr.sin_family = AF_INET;
    inet_pton(AF_INET, ip, &addr.sin_addr);
    addr.sin_port = htons(port);

    // 非阻塞 connect (最多等 3 秒)
    u_long mode = 1;
    ioctlsocket(sock, FIONBIO, &mode);
    connect(sock, (SOCKADDR*)&addr, sizeof(addr));

    fd_set set;
    FD_ZERO(&set);
    FD_SET(sock, &set);
    timeval tv = { 3, 0 };
    if (select(0, NULL, &set, NULL, &tv) <= 0) {
        closesocket(sock);
        return INVALID_SOCKET;
    }

    // 恢复阻塞
    mode = 0;
    ioctlsocket(sock, FIONBIO, &mode);

    // 超时: 发送 100ms, 接收 200ms (保证 init 序列 GetPose 等可靠收到)
    int sndTimeout = 100;
    int rcvTimeout = 200;
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, (char*)&sndTimeout, sizeof(sndTimeout));
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (char*)&rcvTimeout, sizeof(rcvTimeout));

    return sock;
}

bool robotConnect(const char* ip) {
    auto& app = appState;

    EnterCriticalSection(&app.robotSocketMutex);

    app.robotEnableSocket = connectPort(ip, Config::ENABLE_PORT);
    if (app.robotEnableSocket == INVALID_SOCKET) {
        std::cerr << "连接使能端口 " << Config::ENABLE_PORT << " 失败" << std::endl;
        LeaveCriticalSection(&app.robotSocketMutex);
        return false;
    }
    std::cout << "已连接使能端口 " << Config::ENABLE_PORT << std::endl;

    app.robotMotionSocket = connectPort(ip, Config::MOTION_PORT);
    if (app.robotMotionSocket == INVALID_SOCKET) {
        std::cerr << "连接运动端口 " << Config::MOTION_PORT << " 失败" << std::endl;
        closesocket(app.robotEnableSocket);
        app.robotEnableSocket = INVALID_SOCKET;
        LeaveCriticalSection(&app.robotSocketMutex);
        return false;
    }
    std::cout << "已连接运动端口 " << Config::MOTION_PORT << std::endl;

    app.isRobotConnected = true;
    LeaveCriticalSection(&app.robotSocketMutex);
    return true;
}

void robotDisconnect() {
    auto& app = appState;
    EnterCriticalSection(&app.robotSocketMutex);
    if (app.robotEnableSocket != INVALID_SOCKET) {
        closesocket(app.robotEnableSocket);
        app.robotEnableSocket = INVALID_SOCKET;
    }
    if (app.robotMotionSocket != INVALID_SOCKET) {
        closesocket(app.robotMotionSocket);
        app.robotMotionSocket = INVALID_SOCKET;
    }
    app.isRobotConnected = false;
    LeaveCriticalSection(&app.robotSocketMutex);
}

static bool sendToSocket(SOCKET sock, const char* cmd) {
    if (sock == INVALID_SOCKET) return false;
    int len = (int)strlen(cmd);
    int sent = send(sock, cmd, len, 0);
    return sent == len;
}

// 阻塞读取 (init 序列使用)
static int recvFromSocket(SOCKET sock, char* buf, int len) {
    if (sock == INVALID_SOCKET) return -1;
    return recv(sock, buf, len, 0);
}

// 非阻塞读取 (pollFeedback 使用，不阻塞 GLUT 渲染循环)
static int recvFromSocketNoBlock(SOCKET sock, char* buf, int len) {
    if (sock == INVALID_SOCKET) return -1;
    fd_set set;
    FD_ZERO(&set);
    FD_SET(sock, &set);
    timeval tv = { 0, 0 };
    if (select(0, &set, NULL, NULL, &tv) <= 0) return -1;
    return recv(sock, buf, len, 0);
}

bool robotSendEnable(const char* cmd) {
    auto& app = appState;
    EnterCriticalSection(&app.robotSocketMutex);
    SOCKET sock = app.robotEnableSocket;
    LeaveCriticalSection(&app.robotSocketMutex);
    return sendToSocket(sock, cmd);
}

bool robotSendMotion(const char* cmd) {
    auto& app = appState;
    EnterCriticalSection(&app.robotSocketMutex);
    SOCKET sock = app.robotMotionSocket;
    LeaveCriticalSection(&app.robotSocketMutex);
    return sendToSocket(sock, cmd);
}

// 阻塞版本 (init 序列用)
bool robotRecvMotion(char* buf, int len) {
    auto& app = appState;
    EnterCriticalSection(&app.robotSocketMutex);
    SOCKET sock = app.robotMotionSocket;
    LeaveCriticalSection(&app.robotSocketMutex);
    int n = recvFromSocket(sock, buf, len - 1);
    if (n > 0) { buf[n] = '\0'; return true; }
    return false;
}

bool robotRecvEnable(char* buf, int len) {
    auto& app = appState;
    EnterCriticalSection(&app.robotSocketMutex);
    SOCKET sock = app.robotEnableSocket;
    LeaveCriticalSection(&app.robotSocketMutex);
    int n = recvFromSocket(sock, buf, len - 1);
    if (n > 0) { buf[n] = '\0'; return true; }
    return false;
}

// 非阻塞版本 (pollFeedback 用)
bool robotRecvMotionPoll(char* buf, int len) {
    auto& app = appState;
    EnterCriticalSection(&app.robotSocketMutex);
    SOCKET sock = app.robotMotionSocket;
    LeaveCriticalSection(&app.robotSocketMutex);
    int n = recvFromSocketNoBlock(sock, buf, len - 1);
    if (n > 0) { buf[n] = '\0'; return true; }
    return false;
}

bool robotRecvEnablePoll(char* buf, int len) {
    auto& app = appState;
    EnterCriticalSection(&app.robotSocketMutex);
    SOCKET sock = app.robotEnableSocket;
    LeaveCriticalSection(&app.robotSocketMutex);
    int n = recvFromSocketNoBlock(sock, buf, len - 1);
    if (n > 0) { buf[n] = '\0'; return true; }
    return false;
}

bool isRobotConnected() {
    return appState.isRobotConnected;
}

void robotDrainEnable() {
    auto& app = appState;
    EnterCriticalSection(&app.robotSocketMutex);
    SOCKET sock = app.robotEnableSocket;
    LeaveCriticalSection(&app.robotSocketMutex);

    if (sock == INVALID_SOCKET) return;

    char buf[256];
    // 非阻塞清空所有残留数据
    fd_set set;
    FD_ZERO(&set);
    FD_SET(sock, &set);
    timeval tv = {0, 0};
    while (select(0, &set, NULL, NULL, &tv) > 0) {
        recv(sock, buf, sizeof(buf), 0);
        FD_ZERO(&set);
        FD_SET(sock, &set);
    }
}
