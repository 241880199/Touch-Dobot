#include "RobotConnection.h"
#include "FrameLayout.h"
#include "../config/Config.h"
#include "../core/AppState.h"
#include <iostream>
#include <cstring>

static SOCKET g_realtimeSocket = INVALID_SOCKET;
static CRITICAL_SECTION g_realtimeSocketMutex;
static struct RealtimeSocketGuard {
    RealtimeSocketGuard() { InitializeCriticalSection(&g_realtimeSocketMutex); }
    ~RealtimeSocketGuard() { DeleteCriticalSection(&g_realtimeSocketMutex); }
} g_realtimeGuard;

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

    EnterCriticalSection(&g_realtimeSocketMutex);
    if (g_realtimeSocket != INVALID_SOCKET) {
        closesocket(g_realtimeSocket);
        g_realtimeSocket = INVALID_SOCKET;
    }
    LeaveCriticalSection(&g_realtimeSocketMutex);
}

bool robotConnectRealtime(const char* ip) {
    EnterCriticalSection(&g_realtimeSocketMutex);

    if (g_realtimeSocket != INVALID_SOCKET) {
        closesocket(g_realtimeSocket);
    }
    g_realtimeSocket = WSASocket(AF_INET, SOCK_STREAM, IPPROTO_TCP, NULL, 0, 0);
    if (g_realtimeSocket == INVALID_SOCKET) {
        LeaveCriticalSection(&g_realtimeSocketMutex);
        return false;
    }

    sockaddr_in addr;
    addr.sin_family = AF_INET;
    inet_pton(AF_INET, ip, &addr.sin_addr);
    addr.sin_port = htons(Config::FORCE_REALTIME_PORT);

    // Non-blocking connect with 3s timeout (same pattern as connectPort)
    u_long mode = 1;
    ioctlsocket(g_realtimeSocket, FIONBIO, &mode);
    connect(g_realtimeSocket, (SOCKADDR*)&addr, sizeof(addr));

    fd_set set;
    FD_ZERO(&set);
    FD_SET(g_realtimeSocket, &set);
    timeval tv = {3, 0};
    if (select(0, NULL, &set, NULL, &tv) <= 0) {
        closesocket(g_realtimeSocket);
        g_realtimeSocket = INVALID_SOCKET;
        LeaveCriticalSection(&g_realtimeSocketMutex);
        return false;
    }

    // Back to blocking mode, but with short recv timeout (8ms expected interval)
    mode = 0;
    ioctlsocket(g_realtimeSocket, FIONBIO, &mode);
    int timeout = 100;
    setsockopt(g_realtimeSocket, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof(timeout));

    LeaveCriticalSection(&g_realtimeSocketMutex);
    std::cout << "[Force] Connected to realtime port " << Config::FORCE_REALTIME_PORT << std::endl;
    return true;
}

bool robotRecvRealtime(char* buf, int len) {
    EnterCriticalSection(&g_realtimeSocketMutex);
    if (g_realtimeSocket == INVALID_SOCKET) {
        LeaveCriticalSection(&g_realtimeSocketMutex);
        return false;
    }
    int n = recv(g_realtimeSocket, buf, len, 0);
    if (n == len) {
        // ===== 帧自检 (2026-09-21 加) —— 理由/契约/档位见 robot/FrameLayout.h =====
        // ⚠ 与下面那条"宁断不错"的约定配起来看: 只有【恰好一整帧】才走到这里, 而那【不】
        //   证明这段字节落在帧边界上 —— 布局变了我们就会照旧读满 1440 再按错位的偏移解析。
        //   本检查是整条链路上【力数据那一路】唯一能发现那件事的地方。
        const FrameLayout::MagicOrder mo = FrameLayout::classifyMagic(
            reinterpret_cast<const unsigned char*>(buf), n);
        if (mo == FrameLayout::MagicOrder::None) {
            static int badCount = 0;
            badCount++;
            std::cerr << "[Force] !! 30004 帧自检失败 #" << badCount << ": offset "
                      << FrameLayout::MAGIC_OFFSET << " 的 TestValue 既不是 0x0123456789ABCDEF"
                         " (小端) 也不是它的大端排法。" << std::endl;
            std::cerr << "[Force]    ⇒ 机械臂侧的布局与我们假设的不符, 或流已错位。"
                         "这【不是】网络问题 —— 别去查网线。" << std::endl;
            if (Config::FORCE_FRAME_MAGIC_MODE >= 2) {
                // 判的档: 拒帧 + 断开重连 (与 n<=0 那一支同样处理, 见下)。
                std::cerr << "[Force]    本帧按【不使用】处理, 并断线重连以重新对齐。" << std::endl;
                closesocket(g_realtimeSocket);
                g_realtimeSocket = INVALID_SOCKET;
                LeaveCriticalSection(&g_realtimeSocketMutex);
                return false;
            }
            // 只报不判的档 (默认): 照旧收下这一帧。话要说明白 —— 否则下一个人会以为它在拦。
            std::cerr << "[Force]    【本档只报不判】⇒ 这一帧仍然被收下并按现有偏移解析。"
                         "要它真的拦下来, 把 Config::FORCE_FRAME_MAGIC_MODE 改成 2。" << std::endl;
            std::cerr << "[Force]    若确认机械臂侧【不填】这个字段 (或排法与文档不符), "
                         "把 Config::FORCE_FRAME_MAGIC_MODE 改成 0。" << std::endl;
        } else {
            // 首次通过时把【字节序】打出来。它是"机器人怎么序列化这个 uint64"的【实测答案】——
            // 从前只能按 x86 惯例假定小端, 那是推断不是验证 (见 FrameLayout.h 顶上那段)。
            static bool orderReported = false;
            if (!orderReported) {
                orderReported = true;
                std::cout << "[Force] 30004 帧自检通过: TestValue @"
                          << FrameLayout::MAGIC_OFFSET << " = 0x0123456789ABCDEF ("
                          << (mo == FrameLayout::MagicOrder::LittleEndian ? "小端" : "大端")
                          << "字节序) ⇒ 帧布局与我们的假设一致" << std::endl;
            }
        }
        LeaveCriticalSection(&g_realtimeSocketMutex);
        return true;  // must receive exactly 1440 bytes
    }
    if (n <= 0) {
        // Connection lost
        closesocket(g_realtimeSocket);
        g_realtimeSocket = INVALID_SOCKET;
    }
    LeaveCriticalSection(&g_realtimeSocketMutex);
    return false;
}

void robotCloseRealtime() {
    EnterCriticalSection(&g_realtimeSocketMutex);
    if (g_realtimeSocket != INVALID_SOCKET) {
        closesocket(g_realtimeSocket);
        g_realtimeSocket = INVALID_SOCKET;
        std::cout << "[Force] Realtime port disconnected" << std::endl;
    }
    LeaveCriticalSection(&g_realtimeSocketMutex);
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
