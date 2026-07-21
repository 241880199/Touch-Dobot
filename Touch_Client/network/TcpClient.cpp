#include "TcpClient.h"
#include "RelayProtocol.h"
#include "PongHandler.h"
#include "../config/Config.h"
#include "../core/AppState.h"
#include <iostream>
#include <cstring>

#pragma comment(lib, "ws2_32.lib")

// ===== 连接中继站 =====
bool connectToRelay() {
    auto& app = appState;
    EnterCriticalSection(&app.relaySocketMutex);

    app.relaySocket = WSASocket(AF_INET, SOCK_STREAM, IPPROTO_TCP, NULL, 0, WSA_FLAG_OVERLAPPED);
    if (app.relaySocket == INVALID_SOCKET) {
        std::cerr << "创建套接字失败: " << WSAGetLastError() << std::endl;
        LeaveCriticalSection(&app.relaySocketMutex);
        return false;
    }

    sockaddr_in addr;
    addr.sin_family = AF_INET;
    inet_pton(AF_INET, Config::TCP_RELAY_IP, &addr.sin_addr);
    addr.sin_port = htons(Config::RELAY_PORT);

    if (connect(app.relaySocket, (SOCKADDR*)&addr, sizeof(addr)) == SOCKET_ERROR) {
        std::cerr << "连接中转站失败: " << WSAGetLastError() << std::endl;
        closesocket(app.relaySocket);
        app.relaySocket = INVALID_SOCKET;
        LeaveCriticalSection(&app.relaySocketMutex);
        return false;
    }

    app.isTcpConnected = true;
    std::cout << "已连接到中转站 " << Config::TCP_RELAY_IP << ":" << Config::RELAY_PORT << std::endl;
    LeaveCriticalSection(&app.relaySocketMutex);
    return true;
}

// ===== 断开连接 =====
void disconnectRelay() {
    auto& app = appState;
    EnterCriticalSection(&app.relaySocketMutex);
    if (app.relaySocket != INVALID_SOCKET) {
        closesocket(app.relaySocket);
        app.relaySocket = INVALID_SOCKET;
    }
    app.isTcpConnected = false;
    LeaveCriticalSection(&app.relaySocketMutex);
}

// ===== 发送指令（异步，不等待反馈） =====
bool sendToRelay(int targetPort, const char* cmd) {
    auto& app = appState;
    if (!app.isTcpConnected || app.isClosing) {
        snprintf(app.lastTransmissionDetail, sizeof(app.lastTransmissionDetail),
                 "未连接到中转站，无法发送");
        return false;
    }

    std::string msg = RelayProtocol::buildMessage(targetPort, cmd);

    EnterCriticalSection(&app.relaySocketMutex);
    SOCKET sock = app.relaySocket;
    if (sock == INVALID_SOCKET) {
        LeaveCriticalSection(&app.relaySocketMutex);
        return false;
    }

    int sent = send(sock, msg.c_str(), (int)msg.length(), 0);
    LeaveCriticalSection(&app.relaySocketMutex);

    if (sent == SOCKET_ERROR) {
        snprintf(app.lastTransmissionDetail, sizeof(app.lastTransmissionDetail),
                 "发送失败: %d", WSAGetLastError());
        return false;
    }
    return true;
}

// ===== 从缓冲区读取反馈 =====
bool readFeedback(char* outBuf, int outLen, DWORD timeoutMs) {
    auto& app = appState;
    DWORD start = GetTickCount();

    while (true) {
        EnterCriticalSection(&app.recvBufferMutex);
        bool hasData = (app.recvBufferLen > 0);
        LeaveCriticalSection(&app.recvBufferMutex);

        if (hasData) break;

        DWORD elapsed = GetTickCount() - start;
        if (elapsed >= timeoutMs) return false;
        Sleep(10);
    }

    EnterCriticalSection(&app.recvBufferMutex);
    int copyLen = min(app.recvBufferLen, outLen - 1);
    memcpy(outBuf, app.recvBuffer, copyLen);
    outBuf[copyLen] = '\0';
    app.recvBufferLen = 0;
    memset(app.recvBuffer, 0, sizeof(app.recvBuffer));
    LeaveCriticalSection(&app.recvBufferMutex);

    return true;
}

// ===== TCP 客户端事件循环线程 =====
DWORD WINAPI tcpClientThreadProc(LPVOID param) {
    auto& app = appState;

    if (!connectToRelay()) {
        app.clientRunning = false;
        return 1;
    }

    WSAEVENT hEvent = WSACreateEvent();
    WSAEventSelect(app.relaySocket, hEvent, FD_READ | FD_CLOSE);

    WSAEVENT events[2] = { hEvent, app.stopEvent };

    while (app.clientRunning && !app.isClosing) {
        DWORD result = WSAWaitForMultipleEvents(2, events, FALSE, 100, FALSE);
        if (result == WSA_WAIT_TIMEOUT) continue;
        if (result == WSA_WAIT_FAILED) break;

        DWORD idx = result - WSA_WAIT_EVENT_0;
        if (idx == 1) break; // stopEvent

        if (idx == 0) {
            WSANETWORKEVENTS netEvents;
            WSAEnumNetworkEvents(app.relaySocket, hEvent, &netEvents);

            if (netEvents.lNetworkEvents & FD_READ) {
                static std::vector<char> buf(Config::RECV_BUFFER_SIZE);
                int len = recv(app.relaySocket, buf.data(), (int)buf.size() - 1, 0);

                if (len > 0) {
                    buf[len] = '\0';

                    // 检查是否为 ROBOT_ARM_CLOSED
                    if (strcmp(buf.data(), Config::ROBOT_CLOSED_MSG) == 0) {
                        std::cout << "机械臂断开连接" << std::endl;
                        continue;
                    }

                    // 处理 PING
                    if (PongHandler::isPing(buf.data())) {
                        std::string pong = PongHandler::buildPong(buf.data());
                        EnterCriticalSection(&app.relaySocketMutex);
                        send(app.relaySocket, pong.c_str(), (int)pong.length(), 0);
                        LeaveCriticalSection(&app.relaySocketMutex);
                        continue;
                    }

                    // 存入接收缓冲区
                    EnterCriticalSection(&app.recvBufferMutex);
                    int space = (int)sizeof(app.recvBuffer) - app.recvBufferLen - 1;
                    int copyLen = min(len, space);
                    if (copyLen > 0) {
                        memcpy(app.recvBuffer + app.recvBufferLen, buf.data(), copyLen);
                        app.recvBufferLen += copyLen;
                        app.recvBuffer[app.recvBufferLen] = '\0';
                    }
                    LeaveCriticalSection(&app.recvBufferMutex);
                } else if (len == 0) {
                    std::cout << "中转站关闭连接" << std::endl;
                    break;
                }
            }

            if (netEvents.lNetworkEvents & FD_CLOSE) {
                std::cout << "中转站关闭连接 (FD_CLOSE)" << std::endl;
                break;
            }
        }
    }

    WSACloseEvent(hEvent);
    disconnectRelay();
    app.clientRunning = false;
    return 0;
}

// ===== 启动 TCP 客户端 =====
void startTcpClient() {
    auto& app = appState;
    if (app.clientRunning) return;

    app.clientRunning = true;
    app.clientThread = CreateThread(NULL, 0, tcpClientThreadProc, NULL, 0, NULL);
    if (!app.clientThread) {
        app.clientRunning = false;
        std::cerr << "创建 TCP 线程失败" << std::endl;
    }
}

// ===== 停止 TCP 客户端 =====
void stopTcpClient() {
    auto& app = appState;
    app.clientRunning = false;
    WSASetEvent(app.stopEvent);

    if (app.clientThread) {
        WaitForSingleObject(app.clientThread, 1000);
        CloseHandle(app.clientThread);
        app.clientThread = NULL;
    }
    disconnectRelay();
}
