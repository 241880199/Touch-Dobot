# Touch_Client v3.0 单机数字孪生系统 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将 Touch_Client 从双进程架构（C++ + MATLAB 中继站）重构为单进程三层解耦架构，增加基于 STL 模型的 3D 数字孪生渲染。

**Architecture:** 三层解耦 —— Touch 层 (haptic/) 只读设备，Robot 层 (robot/) 只做 TCP 通信，Relay 层 (relay/) 编排所有业务逻辑。Render 层 (render/) 只读状态，纯展示。Touch 层和 Robot 层互不引用。

**Tech Stack:** C++17 / OpenHaptics SDK (HDAPI) / freeglut (GLUT 3.2, SDK 自带) / Winsock2 / Visual Studio 2022 Build Tools (MSBuild)

## Global Constraints

- 三层解耦：Touch 层不引用 Robot 层任何头文件，Robot 层不引用 Touch 层任何头文件
- Relay 层是唯一可以同时引用 Touch 和 Robot 两层的模块
- Render 层只读 AppState/RelayCore，不写任何状态
- MATLAB 中继站代码 (`Relay_Station/`) 保留不做删除
- 力反馈闭环不做（预留接口）
- 姿态控制不做（固定姿态）
- GLUT 使用 SDK 自带的 glut32.lib，不额外下载
- 所有渲染使用立即模式 OpenGL (glBegin/glEnd)，无 shader
- Windows 平台，MSBuild 命令行编译

---

### Task 1: 更新 Config.h —— 从 relay 模式切换到直连模式

**Files:**
- Modify: `Codes/Touch_Client/config/Config.h`

**Interfaces:**
- Consumes: nothing
- Produces: 删除 `TCP_RELAY_IP`、`RELAY_PORT`、`PING_PREFIX`、`PONG_PREFIX`、`ROBOT_CLOSED_MSG`，新增 `ROBOT_IP`、`POSE_QUERY_INTERVAL`

- [ ] **Step 1: 在 Config.h 中替换网络参数区域**

找到 `========== 网络参数 ==========` 区块，整体替换：

```cpp
// ========== 网络参数 ==========
const char* ROBOT_IP = "192.168.101.11";
const int ENABLE_PORT = 29999;
const int MOTION_PORT = 30003;
const int RECV_BUFFER_SIZE = 1024 * 64; // 64KB
```

- [ ] **Step 2: 删除 PING/PONG 参数区块**

找到 `========== PING/PONG 参数 ==========` 区块，整体删除以下行：
```cpp
const char* PING_PREFIX = "PING|";
const char* PONG_PREFIX = "PONG|";
const char* ROBOT_CLOSED_MSG = "ROBOT_ARM_CLOSED";
```

- [ ] **Step 3: 新增位姿查询间隔**

在 `ALARM_CHECK_INTERVAL` 下方添加：
```cpp
const int POSE_QUERY_INTERVAL = 100;    // 位姿查询间隔 (ms)，驱动 3D 模型更新
```

- [ ] **Step 4: 验证编译**

```bash
msbuild Codes\Touch_Client\Touch_Client.vcxproj /p:Configuration=Release /p:Platform=x64 /v:minimal
```

---

### Task 2: 移动 MathUtils 到 core/，删除 utils/ 目录

**Files:**
- Create: `Codes/Touch_Client/core/MathUtils.h`
- Delete: `Codes/Touch_Client/utils/MathUtils.h`
- Delete: `Codes/Touch_Client/utils/` (空目录)

**Interfaces:**
- Produces: `clamp(value, min, max)` 模板函数，路径改为 `core/MathUtils.h`

- [ ] **Step 1: 创建 core/MathUtils.h**

```cpp
#pragma once

template<typename T>
const T& clamp(const T& value, const T& min, const T& max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}
```

- [ ] **Step 2: 删除旧文件**

```bash
rm Codes/Touch_Client/utils/MathUtils.h
rmdir Codes/Touch_Client/utils
```

- [ ] **Step 3: 更新所有 `#include "../utils/MathUtils.h"` 引用**

在以下文件中将 `#include "../utils/MathUtils.h"` 改为 `#include "MathUtils.h"`：
- `Codes/Touch_Client/haptic/HapticCallback.cpp`
- `Codes/Touch_Client/robot/SafetyBoundary.h`

```bash
# HapticCallback.cpp: 路径变了 (haptic/ → core/ 是 ../core/)
sed -i 's|#include "../utils/MathUtils.h"|#include "../core/MathUtils.h"|' Codes/Touch_Client/haptic/HapticCallback.cpp
# SafetyBoundary.h: 路径变了 (robot/ → core/ 是 ../core/)
sed -i 's|#include "../utils/MathUtils.h"|#include "../core/MathUtils.h"|' Codes/Touch_Client/robot/SafetyBoundary.h
```

---

### Task 3: 更新 AppState —— 双连接状态 + 机械臂位姿

**Files:**
- Modify: `Codes/Touch_Client/core/AppState.h`
- Modify: `Codes/Touch_Client/core/AppState.cpp`

**Interfaces:**
- Consumes: nothing new
- Produces: 新增字段 `robotEnableSocket`、`robotMotionSocket`、`isRobotConnected`、`robotActualPose`、`robotTargetPose`、`latencyMs`、`lastCommandSent`；删除 `relaySocket`、`stopEvent`、`recvBuffer`、`recvBufferLen` 及对应的临界区

- [ ] **Step 1: 更新 AppState.h**

替换 TCP 网络区域和新增位姿区域。找到 `// ===== TCP 网络 =====` 区块，替换为：

```cpp
// ===== 机械臂 TCP 双端口 =====
SOCKET robotEnableSocket = INVALID_SOCKET;
SOCKET robotMotionSocket = INVALID_SOCKET;
CRITICAL_SECTION robotSocketMutex;
std::atomic<bool> isRobotConnected{ false };

// ===== 机械臂位姿（3D 模型驱动） =====
struct RobotPose {
    double x = 0, y = 0, z = 0;
    double rx = 0, ry = 0, rz = 0;
};
RobotPose robotActualPose;     // GetPose() 返回的实际位姿
RobotPose robotTargetPose;     // Touch 发送的目标位姿
CRITICAL_SECTION robotPoseMutex;
std::atomic<float> latencyMs{ 0.0f };      // 往返延迟 (ms)
char lastCommandSent[256] = "";            // 最后发送的指令文本
CRITICAL_SECTION lastCommandMutex;
```

删除以下行（不再需要 relay 概念）：
```
SOCKET relaySocket = INVALID_SOCKET;
CRITICAL_SECTION relaySocketMutex;
char recvBuffer[1024 * 64] = { 0 };
int recvBufferLen = 0;
CRITICAL_SECTION recvBufferMutex;
```

删除以下行（stopEvent 移到 RobotConnection 内部）：
```
WSAEVENT stopEvent = WSA_INVALID_EVENT;
```

删除以下行（不再需要 relay 状态字符串）：
```
char connectionStatus[128] = "TCP: not connected";
char serverInfo[64] = "IP: -";
```

- [ ] **Step 2: 更新 AppState.cpp 构造函数**

```cpp
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
```

- [ ] **Step 3: 更新 AppState.cpp 析构函数**

```cpp
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
```

---

### Task 4: RobotConnection —— 双端口 TCP 管理

**Files:**
- Create: `Codes/Touch_Client/robot/RobotConnection.h`
- Create: `Codes/Touch_Client/robot/RobotConnection.cpp`

**Interfaces:**
- Consumes: `Config::ROBOT_IP`, `Config::ENABLE_PORT`, `Config::MOTION_PORT`, `appState`
- Produces:
  - `bool robotConnect(const char* ip)` — 连接两个端口，返回成功/失败
  - `void robotDisconnect()` — 断开两个端口
  - `bool robotSendEnable(const char* cmd)` — 发送到使能端口 (29999)
  - `bool robotSendMotion(const char* cmd)` — 发送到运动端口 (30003)
  - `bool robotRecvMotion(char* buf, int len)` — 非阻塞读取运动端口反馈
  - `bool robotRecvEnable(char* buf, int len)` — 非阻塞读取使能端口反馈
  - `bool isRobotConnected()` — 两个端口是否都连接

- [ ] **Step 1: 编写 RobotConnection.h**

```cpp
#pragma once
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>

bool robotConnect(const char* ip);
void robotDisconnect();
bool robotSendEnable(const char* cmd);
bool robotSendMotion(const char* cmd);
bool robotRecvMotion(char* buf, int len);
bool robotRecvEnable(char* buf, int len);
bool isRobotConnected();
```

- [ ] **Step 2: 编写 RobotConnection.cpp**

```cpp
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

    // 设置非阻塞
    u_long mode = 1;
    ioctlsocket(sock, FIONBIO, &mode);

    // 非阻塞 connect
    connect(sock, (SOCKADDR*)&addr, sizeof(addr));

    // 等待连接完成 (最多 3 秒)
    fd_set set;
    FD_ZERO(&set);
    FD_SET(sock, &set);
    timeval tv = { 3, 0 };
    if (select(0, NULL, &set, NULL, &tv) <= 0) {
        closesocket(sock);
        return INVALID_SOCKET;
    }

    // 恢复阻塞模式
    mode = 0;
    ioctlsocket(sock, FIONBIO, &mode);

    int timeout = 1000;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof(timeout));
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, (char*)&timeout, sizeof(timeout));

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

static int recvFromSocket(SOCKET sock, char* buf, int len) {
    if (sock == INVALID_SOCKET) return -1;
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

bool isRobotConnected() {
    return appState.isRobotConnected;
}
```

---

### Task 5: IExtension.h —— 扩展插件接口

**Files:**
- Create: `Codes/Touch_Client/relay/IExtension.h`

**Interfaces:**
- Consumes: nothing
- Produces: `IExtension` 抽象基类，定义 `onBeforeSend` 和 `onAfterFeedback` 虚方法

- [ ] **Step 1: 编写 IExtension.h**

```cpp
#pragma once
#include <string>

// 机械臂指令（在发送前可被 Extension 修改）
struct RobotCommand {
    std::string cmd;
    int targetPort; // 29999 or 30003
};

// 机械臂原始反馈（在解析后可被 Extension 处理）
struct RobotFeedback {
    int errorId = -1;
    char data[512] = {}; // {data} 部分提取结果
    char raw[1024] = {}; // 完整原始字符串
    int fromPort = 0;    // 来自哪个端口
};

// 扩展插件接口 —— 预留力滤波、夹具控制、摄像头跟踪等后续模块
class IExtension {
public:
    virtual ~IExtension() = default;

    // 指令发送前的钩子：可修改或拦截指令
    virtual void onBeforeSend(RobotCommand& cmd) {}

    // 收到机械臂反馈后的钩子
    virtual void onAfterFeedback(const RobotFeedback& fb) {}
};
```

---

### Task 6: FeedbackParser —— 机械臂反馈解析

**Files:**
- Create: `Codes/Touch_Client/relay/FeedbackParser.h`
- Create: `Codes/Touch_Client/relay/FeedbackParser.cpp`

**Interfaces:**
- Consumes: `AppState::RobotPose`
- Produces:
  - `FeedbackParser` 命名空间
  - `bool parsePose(const char* feedback, RobotPose& out)` — 从 GetPose 反馈提取 {x,y,z,rx,ry,rz}
  - `bool parseMode(const char* feedback, int& out)` — 从 RobotMode 反馈提取模式码
  - `bool isSuccess(const char* feedback)` — 检查 ErrorID == 0
  - `bool extractData(const char* feedback, char* out, int len)` — 提取 {data} 部分

- [ ] **Step 1: 编写 FeedbackParser.h**

```cpp
#pragma once
#include "../core/AppState.h"

namespace FeedbackParser {
    // 解析 GetPose() 返回: 0,{x,y,z,rx,ry,rz},GetPose();
    bool parsePose(const char* feedback, AppState::RobotPose& out);

    // 解析 RobotMode() 返回: 0,{mode},RobotMode();
    bool parseMode(const char* feedback, int& out);

    // 检查反馈是否成功 (ErrorID == 0)
    bool isSuccess(const char* feedback);

    // 提取 { } 中的 data 内容
    bool extractData(const char* feedback, char* out, int len);
}
```

- [ ] **Step 2: 编写 FeedbackParser.cpp**

```cpp
#include "FeedbackParser.h"
#include <cstdio>
#include <cstring>
#include <cstdlib>

namespace FeedbackParser {

bool isSuccess(const char* feedback) {
    return feedback && feedback[0] == '0';
}

bool extractData(const char* feedback, char* out, int len) {
    if (!feedback || !out) return false;
    const char* start = strchr(feedback, '{');
    const char* end = strchr(feedback, '}');
    if (!start || !end || start >= end) return false;
    int n = (int)(end - start - 1);
    if (n <= 0 || n >= len) return false;
    memcpy(out, start + 1, n);
    out[n] = '\0';
    return true;
}

bool parsePose(const char* feedback, AppState::RobotPose& out) {
    char data[256];
    if (!extractData(feedback, data, sizeof(data))) return false;
    return sscanf_s(data, "%lf,%lf,%lf,%lf,%lf,%lf",
        &out.x, &out.y, &out.z, &out.rx, &out.ry, &out.rz) == 6;
}

bool parseMode(const char* feedback, int& out) {
    char data[64];
    if (!extractData(feedback, data, sizeof(data))) return false;
    out = atoi(data);
    return true;
}

} // namespace FeedbackParser
```

---

### Task 7: 迁移 CoordinateTransform + SafetyBoundary 到 relay/

**Files:**
- Move: `Codes/Touch_Client/core/CoordinateTransform.h` → `Codes/Touch_Client/relay/CoordinateTransform.h`
- Move: `Codes/Touch_Client/robot/SafetyBoundary.h` → `Codes/Touch_Client/relay/SafetyBoundary.h`

**Interfaces:**
- Consumes: nothing new (内部依赖路径调整)
- Produces: 所有坐标转换和安全边界逻辑集中到 relay/ 目录

- [ ] **Step 1: 移动文件**

```bash
mv Codes/Touch_Client/core/CoordinateTransform.h Codes/Touch_Client/relay/CoordinateTransform.h
mv Codes/Touch_Client/robot/SafetyBoundary.h Codes/Touch_Client/relay/SafetyBoundary.h
```

- [ ] **Step 2: 更新 SafetyBoundary.h 的内部 include 路径**

将文件头部的：
```cpp
#include "../config/Config.h"
#include "../core/CoordinateTransform.h"
#include "../utils/MathUtils.h"
```

改为：
```cpp
#include "../config/Config.h"
#include "CoordinateTransform.h"
#include "../core/MathUtils.h"
```

- [ ] **Step 3: 更新所有引用这些文件的源文件**

```bash
# HapticCallback.cpp 中引用了 CoordinateTransform.h
sed -i 's|#include "../core/CoordinateTransform.h"|#include "../relay/CoordinateTransform.h"|' Codes/Touch_Client/haptic/HapticCallback.cpp

# RobotController.cpp 中引用了 SafetyBoundary.h
sed -i 's|#include "SafetyBoundary.h"|#include "../relay/SafetyBoundary.h"|' Codes/Touch_Client/robot/RobotController.cpp
```

---

### Task 8: ProtocolAdapter —— Touch 语义 → Dobot 协议

**Files:**
- Create: `Codes/Touch_Client/relay/ProtocolAdapter.h`
- Create: `Codes/Touch_Client/relay/ProtocolAdapter.cpp`

**Interfaces:**
- Consumes: `Config::*`, `Vec3` from CoordinateTransform
- Produces: `ProtocolAdapter` 命名空间，纯函数将 Touch 层输出翻译为 Dobot TCP/IP 指令字符串

- [ ] **Step 1: 编写 ProtocolAdapter.h**

```cpp
#pragma once
#include <string>
#include "CoordinateTransform.h"

namespace ProtocolAdapter {
    // 运动控制指令
    std::string buildServoP(const Vec3& pos, double rx, double ry, double rz);

    // Dashboard 指令
    std::string buildEnableRobot();
    std::string buildDisableRobot();
    std::string buildClearError();
    std::string buildRobotMode();
    std::string buildGetPose();
    std::string buildCP(unsigned int ratio);
}
```

- [ ] **Step 2: 编写 ProtocolAdapter.cpp**

```cpp
#include "ProtocolAdapter.h"
#include "../config/Config.h"
#include <cstdio>

namespace ProtocolAdapter {

std::string buildServoP(const Vec3& pos, double rx, double ry, double rz) {
    char buf[256];
    snprintf(buf, sizeof(buf), "ServoP(%.2f,%.2f,%.2f,%.2f,%.2f,%.2f)",
        pos.x, pos.y, pos.z, rx, ry, rz);
    return std::string(buf);
}

std::string buildEnableRobot()  { return "EnableRobot(0.5,0,0,0)"; }
std::string buildDisableRobot() { return "DisableRobot()"; }
std::string buildClearError()   { return "ClearError()"; }
std::string buildRobotMode()    { return "RobotMode()"; }
std::string buildGetPose()      { return "GetPose()"; }

std::string buildCP(unsigned int ratio) {
    char buf[64];
    snprintf(buf, sizeof(buf), "CP(%u)", ratio);
    return std::string(buf);
}

} // namespace ProtocolAdapter
```

---

### Task 9: RelayCore —— 数据流编排中枢

**Files:**
- Create: `Codes/Touch_Client/relay/RelayCore.h`
- Create: `Codes/Touch_Client/relay/RelayCore.cpp`

**Interfaces:**
- Consumes: 所有 relay/ 模块、`RobotConnection`、`AppState`、`Config`
- Produces: `RelayCore` 单例
  - `static RelayCore& instance()`
  - `bool init()` — 连接机械臂 + 初始化序列
  - `void shutdown()` — 下使能 + 断开
  - `void sendPosition(const hduVector3Dd& devicePos)` — Touch 层调用的主入口
  - `void onButtonPress(const Vec3& robotPos)` — 按钮按下：记录基准点
  - `void onButtonRelease()` — 按钮松开：停止发送
  - `void pollFeedback()` — 每帧调用：读取反馈 + 更新状态
  - `void queryPose()` — 定时调用：GetPose 驱动 3D 模型
  - `void checkAlarm()` — 定时调用：报警巡检
  - `void registerExtension(IExtension* ext)` — 注册插件

- [ ] **Step 1: 编写 RelayCore.h**

```cpp
#pragma once
#include <vector>
#include <HD/hd.h>
#include <HDU/hduVector.h>
#include "CoordinateTransform.h"
#include "IExtension.h"

class RelayCore {
public:
    static RelayCore& instance();

    bool init();
    void shutdown();

    // Touch → Robot 正向数据流
    void sendPosition(const hduVector3Dd& devicePos);
    void onButtonPress(const Vec3& robotPos);
    void onButtonRelease();

    // Robot → Touch 反向数据流 (每帧调用)
    void pollFeedback();
    void queryPose();
    void checkAlarm();

    // 扩展
    void registerExtension(IExtension* ext);

    // 状态查询（供 Render 层读取）
    bool isTransmitting() const { return m_transmitting; }

private:
    RelayCore() = default;
    RelayCore(const RelayCore&) = delete;
    RelayCore& operator=(const RelayCore&) = delete;

    bool m_transmitting = false;
    bool m_basePointSet = false;
    Vec3 m_basePoint;
    std::vector<IExtension*> m_extensions;
};
```

- [ ] **Step 2: 编写 RelayCore.cpp**

```cpp
#include "RelayCore.h"
#include "ProtocolAdapter.h"
#include "FeedbackParser.h"
#include "SafetyBoundary.h"
#include "../robot/RobotConnection.h"
#include "../core/AppState.h"
#include "../config/Config.h"
#include <iostream>
#include <windows.h>

RelayCore& RelayCore::instance() {
    static RelayCore inst;
    return inst;
}

bool RelayCore::init() {
    if (!robotConnect(Config::ROBOT_IP)) {
        std::cerr << "[Relay] 连接机械臂失败" << std::endl;
        return false;
    }

    // 初始化序列：ClearError → EnableRobot → CP → GetPose
    Sleep(200);
    robotSendEnable(ProtocolAdapter::buildClearError().c_str());
    Sleep(300);

    if (!robotSendEnable(ProtocolAdapter::buildEnableRobot().c_str())) {
        std::cerr << "[Relay] 使能失败" << std::endl;
        return false;
    }
    std::cout << "[Relay] 机械臂使能成功" << std::endl;
    Sleep(200);

    robotSendEnable(ProtocolAdapter::buildCP(Config::CP_SMOOTH_RATIO).c_str());
    Sleep(100);

    // 获取基准位姿
    robotSendEnable(ProtocolAdapter::buildGetPose().c_str());
    Sleep(200);
    char fb[1024];
    if (robotRecvEnable(fb, sizeof(fb))) {
        AppState::RobotPose pose;
        if (FeedbackParser::parsePose(fb, pose)) {
            auto& app = appState;
            EnterCriticalSection(&app.robotPoseMutex);
            app.robotBase.x = pose.x;
            app.robotBase.y = pose.y;
            app.robotBase.z = pose.z;
            app.robotBaseRx = pose.rx;
            app.robotBaseRy = pose.ry;
            app.robotBaseRz = pose.rz;
            app.robotActualPose = pose;
            app.robotTargetPose = pose;
            app.isRobotBaseSet = true;
            LeaveCriticalSection(&app.robotPoseMutex);
            std::cout << "[Relay] 基准位姿: (" << pose.x << "," << pose.y << "," << pose.z << ")" << std::endl;
        }
    }

    return true;
}

void RelayCore::shutdown() {
    robotSendEnable(ProtocolAdapter::buildDisableRobot().c_str());
    Sleep(100);
    robotDisconnect();
}

void RelayCore::sendPosition(const hduVector3Dd& devicePos) {
    if (!m_transmitting || !m_basePointSet || !isRobotConnected()) return;

    // 坐标转换
    Vec3 current = convertTouchToRobot(devicePos);
    Vec3 delta = computeDelta(current, m_basePoint);

    // 跳过微小移动
    if (delta.length() < Config::MIN_DELTA_THRESHOLD) return;

    auto& app = appState;
    Vec3 base(app.robotBase.x, app.robotBase.y, app.robotBase.z);
    Vec3 target = computeTarget(base, delta);

    // 安全边界
    target = SafetyBoundary::clampToBoundary(target);

    // 构造指令
    std::string cmd = ProtocolAdapter::buildServoP(
        target, app.robotBaseRx, app.robotBaseRy, app.robotBaseRz);

    // 给扩展插件修改指令的机会
    RobotCommand robotCmd;
    robotCmd.cmd = cmd;
    robotCmd.targetPort = Config::MOTION_PORT;
    for (auto* ext : m_extensions) {
        ext->onBeforeSend(robotCmd);
    }

    // 发送
    robotSendMotion(robotCmd.cmd.c_str());

    // 记录
    EnterCriticalSection(&app.lastCommandMutex);
    strncpy_s(app.lastCommandSent, robotCmd.cmd.c_str(), sizeof(app.lastCommandSent) - 1);
    LeaveCriticalSection(&app.lastCommandMutex);

    // 更新目标位姿
    EnterCriticalSection(&app.robotPoseMutex);
    app.robotTargetPose.x = target.x;
    app.robotTargetPose.y = target.y;
    app.robotTargetPose.z = target.z;
    LeaveCriticalSection(&app.robotPoseMutex);
}

void RelayCore::onButtonPress(const Vec3& robotPos) {
    m_basePoint = robotPos;
    m_basePointSet = true;
    m_transmitting = true;
}

void RelayCore::onButtonRelease() {
    m_transmitting = false;
    m_basePointSet = false;
}

void RelayCore::pollFeedback() {
    char buf[1024];
    // 读取运动端口反馈
    while (robotRecvMotion(buf, sizeof(buf))) {
        RobotFeedback fb;
        strncpy_s(fb.raw, buf, sizeof(fb.raw) - 1);
        fb.fromPort = Config::MOTION_PORT;
        fb.errorId = (buf[0] == '0') ? 0 : -1;
        FeedbackParser::extractData(buf, fb.data, sizeof(fb.data));

        for (auto* ext : m_extensions) {
            ext->onAfterFeedback(fb);
        }
    }

    // 读取使能端口反馈
    while (robotRecvEnable(buf, sizeof(buf))) {
        RobotFeedback fb;
        strncpy_s(fb.raw, buf, sizeof(fb.raw) - 1);
        fb.fromPort = Config::ENABLE_PORT;
        fb.errorId = (buf[0] == '0') ? 0 : -1;

        for (auto* ext : m_extensions) {
            ext->onAfterFeedback(fb);
        }
    }
}

void RelayCore::queryPose() {
    if (!isRobotConnected()) return;
    robotSendEnable(ProtocolAdapter::buildGetPose().c_str());
    Sleep(50);
    char fb[1024];
    if (robotRecvEnable(fb, sizeof(fb))) {
        auto& app = appState;
        EnterCriticalSection(&app.robotPoseMutex);
        FeedbackParser::parsePose(fb, app.robotActualPose);
        LeaveCriticalSection(&app.robotPoseMutex);
    }
}

void RelayCore::checkAlarm() {
    if (!isRobotConnected()) return;
    robotSendEnable(ProtocolAdapter::buildRobotMode().c_str());
    Sleep(50);
    char fb[1024];
    if (robotRecvEnable(fb, sizeof(fb))) {
        int mode = -1;
        FeedbackParser::parseMode(fb, mode);
        auto& app = appState;
        bool wasAlarm = app.isRobotInAlarm.exchange(mode == 9);
        if (mode == 9 && !wasAlarm) {
            std::cout << "[Relay] 检测到机械臂报警 (mode=9)" << std::endl;
        }
    }
}

void RelayCore::registerExtension(IExtension* ext) {
    m_extensions.push_back(ext);
}
```

---

### Task 10: StlMesh + StlLoader —— STL 模型解析

**Files:**
- Create: `Codes/Touch_Client/render/StlMesh.h`
- Create: `Codes/Touch_Client/render/StlLoader.h`
- Create: `Codes/Touch_Client/render/StlLoader.cpp`

**Interfaces:**
- Consumes: nothing (无项目内依赖)
- Produces:
  - `struct StlTriangle` — 单个三角形（3 顶点 + 法向量）
  - `struct StlMesh` — 三角形集合 + 边界盒
  - `StlMesh loadStl(const char* path)` — 自动判断 Binary/ASCII 格式并加载

- [ ] **Step 1: 编写 StlMesh.h**

```cpp
#pragma once
#include <vector>

struct StlTriangle {
    float normal[3];
    float v1[3], v2[3], v3[3];
};

struct StlMesh {
    std::vector<StlTriangle> triangles;
    float bboxMin[3] = { 0, 0, 0 };
    float bboxMax[3] = { 0, 0, 0 };
    bool valid = false;

    void computeBBox();
    void draw() const; // 立即模式 OpenGL 渲染
};
```

- [ ] **Step 2: 编写 StlLoader.h**

```cpp
#pragma once
#include "StlMesh.h"

StlMesh loadStl(const char* path);

// 内部实现（在 StlLoader.cpp 中）
namespace StlLoader {
    StlMesh loadBinary(const char* path);
    StlMesh loadAscii(const char* path);
}
```

- [ ] **Step 3: 编写 StlLoader.cpp**

```cpp
#include "StlLoader.h"
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include <GL/glut.h>

// ===== BBox 计算 =====
void StlMesh::computeBBox() {
    if (triangles.empty()) return;
    bboxMin[0] = bboxMin[1] = bboxMin[2] = 1e10f;
    bboxMax[0] = bboxMax[1] = bboxMax[2] = -1e10f;
    for (auto& t : triangles) {
        for (int i = 0; i < 3; i++) {
            if (t.v1[i] < bboxMin[i]) bboxMin[i] = t.v1[i];
            if (t.v1[i] > bboxMax[i]) bboxMax[i] = t.v1[i];
            if (t.v2[i] < bboxMin[i]) bboxMin[i] = t.v2[i];
            if (t.v2[i] > bboxMax[i]) bboxMax[i] = t.v2[i];
            if (t.v3[i] < bboxMin[i]) bboxMin[i] = t.v3[i];
            if (t.v3[i] > bboxMax[i]) bboxMax[i] = t.v3[i];
        }
    }
}

// ===== 渲染 =====
void StlMesh::draw() const {
    if (!valid) return;
    glBegin(GL_TRIANGLES);
    for (auto& t : triangles) {
        glNormal3fv(t.normal);
        glVertex3fv(t.v1);
        glVertex3fv(t.v2);
        glVertex3fv(t.v3);
    }
    glEnd();
}

// ===== Binary STL 解析 =====
StlMesh StlLoader::loadBinary(const char* path) {
    StlMesh mesh;
    FILE* f = fopen(path, "rb");
    if (!f) { std::cerr << "[STL] 无法打开: " << path << std::endl; return mesh; }

    // 跳过 80 字节头
    fseek(f, 80, SEEK_SET);

    // 读三角形数量
    unsigned int count = 0;
    fread(&count, sizeof(unsigned int), 1, f);

    mesh.triangles.reserve(count);

    for (unsigned int i = 0; i < count; i++) {
        StlTriangle t;
        fread(t.normal, sizeof(float), 3, f);
        fread(t.v1, sizeof(float), 3, f);
        fread(t.v2, sizeof(float), 3, f);
        fread(t.v3, sizeof(float), 3, f);
        fseek(f, 2, SEEK_CUR); // attribute byte count
        mesh.triangles.push_back(t);
    }

    fclose(f);
    mesh.computeBBox();
    mesh.valid = !mesh.triangles.empty();
    std::cout << "[STL] Binary: " << count << " 三角形" << std::endl;
    return mesh;
}

// ===== ASCII STL 解析 =====
StlMesh StlLoader::loadAscii(const char* path) {
    StlMesh mesh;
    FILE* f = fopen(path, "r");
    if (!f) { std::cerr << "[STL] 无法打开: " << path << std::endl; return mesh; }

    char line[256];
    StlTriangle t;
    int vertexIdx = 0;

    while (fgets(line, sizeof(line), f)) {
        // 跳过 leading whitespace
        char* p = line;
        while (*p == ' ' || *p == '\t') p++;

        if (strncmp(p, "facet normal", 12) == 0) {
            sscanf_s(p, "facet normal %f %f %f", &t.normal[0], &t.normal[1], &t.normal[2]);
            vertexIdx = 0;
        }
        else if (strncmp(p, "vertex", 6) == 0) {
            float* v = (vertexIdx == 0) ? t.v1 : (vertexIdx == 1) ? t.v2 : t.v3;
            sscanf_s(p, "vertex %f %f %f", &v[0], &v[1], &v[2]);
            vertexIdx++;
            if (vertexIdx == 3) {
                mesh.triangles.push_back(t);
            }
        }
    }

    fclose(f);
    mesh.computeBBox();
    mesh.valid = !mesh.triangles.empty();
    std::cout << "[STL] ASCII: " << mesh.triangles.size() << " 三角形" << std::endl;
    return mesh;
}

// ===== 自动判断格式 =====
StlMesh loadStl(const char* path) {
    // 读前 5 字节判断：如果以 "solid" 开头则为 ASCII，否则 Binary
    FILE* f = fopen(path, "rb");
    if (!f) return StlMesh();

    char header[5] = {};
    fread(header, 1, 5, f);
    fclose(f);

    if (strncmp(header, "solid", 5) == 0) {
        return StlLoader::loadAscii(path);
    } else {
        return StlLoader::loadBinary(path);
    }
}
```

---

### Task 11: RobotModel —— 机械臂运动学 + 连杆渲染

**Files:**
- Create: `Codes/Touch_Client/render/RobotModel.h`
- Create: `Codes/Touch_Client/render/RobotModel.cpp`

**Interfaces:**
- Consumes: `StlMesh`, `AppState::RobotPose`
- Produces: `RobotModel` 类
  - `bool loadModels(const char* dir)` — 从目录加载 base + link1~link6 的 STL
  - `void draw(const RobotPose& joints, bool useFallback)` — 按运动链层级渲染
  - `void setFallbackMode()` — 启用几何体后备模式
  - `bool isLoaded() const`

- [ ] **Step 1: 编写 RobotModel.h**

```cpp
#pragma once
#include <GL/glut.h>
#include "StlMesh.h"
#include "../core/AppState.h"

class RobotModel {
public:
    bool loadModels(const char* directory);
    void draw(const AppState::RobotPose& joints);
    void setFallbackMode() { m_useFallback = true; }
    bool isLoaded() const { return m_loaded; }

private:
    void drawLink(const StlMesh& mesh);
    void drawFallbackLink(float w, float h, float d); // 长方体
    void drawFallbackBase();                           // 圆柱体

    StlMesh m_links[7];  // 0=base, 1~6=连杆
    bool m_linkLoaded[7] = {};
    bool m_loaded = false;
    bool m_useFallback = false;

    // CR3 几何参数 (mm) — 用于后备几何体和运动学
    static constexpr float BASE_HEIGHT = 60.0f;
    static constexpr float LINK1_Z = 100.0f;
    static constexpr float LINK2_LENGTH = 220.0f;
    static constexpr float LINK3_LENGTH = 210.0f;
    static constexpr float LINK4_Z = 50.0f;
    static constexpr float LINK5_Z = 50.0f;
    static constexpr float LINK6_Z = 40.0f;
};
```

- [ ] **Step 2: 编写 RobotModel.cpp**

```cpp
#include "RobotModel.h"
#include "StlLoader.h"
#include <cstdio>
#include <iostream>
#include <cmath>

bool RobotModel::loadModels(const char* dir) {
    const char* names[] = { "base", "link1", "link2", "link3", "link4", "link5", "link6" };
    char path[512];
    bool anyLoaded = false;

    for (int i = 0; i < 7; i++) {
        snprintf(path, sizeof(path), "%s/%s.stl", dir, names[i]);
        m_links[i] = loadStl(path);
        m_linkLoaded[i] = m_links[i].valid;
        if (m_linkLoaded[i]) anyLoaded = true;
    }

    m_loaded = anyLoaded;
    if (!m_loaded) {
        std::cout << "[RobotModel] STL 加载失败，启用几何体备用模式" << std::endl;
        m_useFallback = true;
    } else {
        std::cout << "[RobotModel] 加载完成" << std::endl;
    }
    return m_loaded || m_useFallback;
}

void RobotModel::drawLink(const StlMesh& mesh) {
    mesh.draw();
}

void RobotModel::drawFallbackLink(float w, float h, float d) {
    glPushMatrix();
    glScalef(w / 2.0f, h / 2.0f, d / 2.0f);
    glutSolidCube(2.0f);
    glPopMatrix();
}

void RobotModel::drawFallbackBase() {
    // 带法兰的底座
    glPushMatrix();
    glRotatef(-90, 1, 0, 0); // gluCylinder 默认沿 Z，旋转到沿 Y
    GLUquadric* q = gluNewQuadric();
    gluCylinder(q, 60, 70, BASE_HEIGHT, 16, 1);
    gluDeleteQuadric(q);
    glPopMatrix();
}

void RobotModel::draw(const AppState::RobotPose& joints) {
    if (!m_loaded && !m_useFallback) return;

    glColor3f(0.25f, 0.28f, 0.32f); // 深灰金属色

    glPushMatrix();

    // === 底座 (世界坐标原点，不旋转不移动) ===
    if (m_linkLoaded[0])        drawLink(m_links[0]);
    else if (m_useFallback)     drawFallbackBase();

    // === J1: 绕 Z 旋转 (腰部) ===
    glTranslatef(0, 0, BASE_HEIGHT + LINK1_Z * 0.5f);
    glRotatef((float)joints.rz, 0, 0, 1);
    glTranslatef(0, 0, -LINK1_Z * 0.5f);
    if (m_linkLoaded[1])        drawLink(m_links[1]);
    else if (m_useFallback)     drawFallbackLink(80, LINK1_Z, 80);

    // === J2: 绕 Y 旋转 (肩部) ===
    glTranslatef(0, 0, LINK1_Z);
    glRotatef((float)joints.ry, 0, 1, 0);
    glTranslatef(0, 0, LINK2_LENGTH * 0.3f);
    if (m_linkLoaded[2])        drawLink(m_links[2]);
    else if (m_useFallback)     drawFallbackLink(60, LINK2_LENGTH, 50);

    // === J3: 绕 Y 旋转 (肘部) ===
    glTranslatef(0, 0, LINK2_LENGTH * 0.7f);
    glRotatef((float)(joints.ry * 0.5), 0, 1, 0); // 近似：J3 与 J2 联动
    glTranslatef(0, 0, LINK3_LENGTH * 0.3f);
    if (m_linkLoaded[3])        drawLink(m_links[3]);
    else if (m_useFallback)     drawFallbackLink(45, LINK3_LENGTH, 40);

    // === J4: 绕 Z 旋转 (腕部1) ===
    glTranslatef(0, 0, LINK3_LENGTH * 0.7f);
    glRotatef((float)joints.rx, 0, 0, 1);
    glTranslatef(0, 0, LINK4_Z * 0.5f);
    if (m_linkLoaded[4])        drawLink(m_links[4]);
    else if (m_useFallback)     drawFallbackLink(30, LINK4_Z, 30);

    // === J5: 绕 Y 旋转 (腕部2) ===
    glTranslatef(0, 0, LINK4_Z);
    glRotatef((float)(joints.ry * 0.3), 0, 1, 0);
    glTranslatef(0, 0, LINK5_Z * 0.5f);
    if (m_linkLoaded[5])        drawLink(m_links[5]);
    else if (m_useFallback)     drawFallbackLink(25, LINK5_Z, 25);

    // === J6: 绕 Z 旋转 (末端) ===
    glTranslatef(0, 0, LINK5_Z);
    glRotatef((float)joints.rz, 0, 0, 1);
    glTranslatef(0, 0, LINK6_Z * 0.5f);
    if (m_linkLoaded[6])        drawLink(m_links[6]);
    else if (m_useFallback)     drawFallbackLink(20, LINK6_Z, 20);

    glPopMatrix();
}
```

---

### Task 12: HudOverlay —— 2D 文字叠加

**Files:**
- Create: `Codes/Touch_Client/render/HudOverlay.h`
- Create: `Codes/Touch_Client/render/HudOverlay.cpp`

**Interfaces:**
- Consumes: `AppState`, `Config::*`, `RelayCore`
- Produces: `HudOverlay` 命名空间
  - `void drawAll()` — 绘制所有 HUD 元素（坐标面板 + 状态栏 + 指令行）

- [ ] **Step 1: 编写 HudOverlay.h**

```cpp
#pragma once

namespace HudOverlay {
    void drawAll();
}
```

- [ ] **Step 2: 编写 HudOverlay.cpp**

```cpp
#include "HudOverlay.h"
#include "../core/AppState.h"
#include "../config/Config.h"
#include "../relay/RelayCore.h"
#include <GL/glut.h>
#include <cstdio>
#include <cstring>

namespace HudOverlay {

// 投影到 2D 并绘制字符串
static void text2D(int x, int y, const char* text, void* font = GLUT_BITMAP_9_BY_15) {
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0, Config::WINDOW_W, 0, Config::WINDOW_H);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    glRasterPos2i(x, y);
    for (const char* c = text; *c; c++) {
        glutBitmapCharacter(font, *c);
    }

    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
}

void drawAll() {
    auto& app = appState;
    auto& relay = RelayCore::instance();
    char buf[256];

    glDisable(GL_DEPTH_TEST);

    // === 左上：坐标面板 ===
    int panelX = 15;
    int panelY = Config::WINDOW_H - 20;
    int lineH = 20;

    // 背景
    glColor4f(0.10f, 0.13f, 0.17f, 0.82f);
    glRecti(panelX - 5, panelY - 140, panelX + 240, panelY + 5);

    auto drawLine = [&](int& y, const char* label, double a, double b, double c) {
        snprintf(buf, sizeof(buf), "%s  (%6.1f, %6.1f, %6.1f)", label, a, b, c);
        glColor3f(0.92f, 0.96f, 1.00f);
        text2D(panelX, y, buf);
        y -= lineH;
    };

    int y = panelY;
    hduVector3Dd raw;
    Vec3 mapped;
    AppState::RobotPose target, actual;
    {
        EnterCriticalSection(&app.devicePosMutex);
        raw = app.devicePos;
        LeaveCriticalSection(&app.devicePosMutex);
        EnterCriticalSection(&app.adjustedPosTableMutex);
        mapped = app.adjustedPosTable;
        LeaveCriticalSection(&app.adjustedPosTableMutex);
        EnterCriticalSection(&app.robotPoseMutex);
        target = app.robotTargetPose;
        actual = app.robotActualPose;
        LeaveCriticalSection(&app.robotPoseMutex);
    }

    drawLine(y, "Touch Raw:   ", raw[0], raw[1], raw[2]);
    drawLine(y, "Touch Mapped:", mapped.x, mapped.y, mapped.z);
    drawLine(y, "Robot Target:", target.x, target.y, target.z);
    drawLine(y, "Robot Actual:", actual.x, actual.y, actual.z);
    drawLine(y, "Delta:       ",
        actual.x - target.x, actual.y - target.y, actual.z - target.z);

    // 传输状态
    const char* state = relay.isTransmitting() ? "TX: ACTIVE" : "TX: IDLE";
    glColor3f(relay.isTransmitting() ? 0.35f : 0.62f,
              relay.isTransmitting() ? 0.90f : 0.68f,
              relay.isTransmitting() ? 0.50f : 0.78f);
    text2D(panelX, y, state);

    // === 右上：状态栏 ===
    int sx = Config::WINDOW_W - 240;
    int sy = Config::WINDOW_H - 20;

    const char* conn29999 = app.isRobotConnected ? "CONNECTED" : "DISCONNECTED";
    const char* conn30003 = app.isRobotConnected ? "CONNECTED" : "DISCONNECTED";
    const char* mode = app.isRobotInAlarm ? "ALARM" : "ENABLED";
    float lat = app.latencyMs;

    snprintf(buf, sizeof(buf), "TCP 29999: %s", conn29999);
    glColor3f(app.isRobotConnected ? 0.35f : 1.0f, app.isRobotConnected ? 0.90f : 0.35f, 0.50f);
    text2D(sx, sy, buf);

    snprintf(buf, sizeof(buf), "TCP 30003: %s", conn30003);
    text2D(sx, sy - lineH, buf);

    snprintf(buf, sizeof(buf), "Robot Mode: %s", mode);
    glColor3f(app.isRobotInAlarm ? 1.0f : 0.35f, app.isRobotInAlarm ? 0.35f : 0.90f, 0.50f);
    text2D(sx, sy - 2 * lineH, buf);

    snprintf(buf, sizeof(buf), "Latency: %.1f ms", lat);
    glColor3f(0.92f, 0.96f, 1.00f);
    text2D(sx, sy - 3 * lineH, buf);

    // === 底部：最后指令 ===
    char lastCmd[256];
    EnterCriticalSection(&app.lastCommandMutex);
    strncpy_s(lastCmd, app.lastCommandSent, sizeof(lastCmd) - 1);
    LeaveCriticalSection(&app.lastCommandMutex);

    int cx = Config::WINDOW_W / 2 - 150;
    snprintf(buf, sizeof(buf), "Last CMD: %s", lastCmd[0] ? lastCmd : "(none)");
    glColor3f(0.62f, 0.68f, 0.78f);
    text2D(cx, 25, buf);

    glEnable(GL_DEPTH_TEST);
}

} // namespace HudOverlay
```

---

### Task 13: SceneRenderer —— 3D 场景组装

**Files:**
- Create: `Codes/Touch_Client/render/SceneRenderer.h`
- Create: `Codes/Touch_Client/render/SceneRenderer.cpp`

**Interfaces:**
- Consumes: `AppState`, `Config::*`, `RobotModel`, `CoordinateTransform`
- Produces: `SceneRenderer` 命名空间
  - `void init()` — 一次性初始化（创建显示列表等）
  - `void draw3D()` — 每帧绘制 3D 场景
  - `RobotModel& getRobotModel()` — 获取机械臂模型对象

- [ ] **Step 1: 编写 SceneRenderer.h**

```cpp
#pragma once
#include "RobotModel.h"

namespace SceneRenderer {
    void init();
    void draw3D();
    RobotModel& getRobotModel();
}
```

- [ ] **Step 2: 编写 SceneRenderer.cpp**

```cpp
#include "SceneRenderer.h"
#include "../core/AppState.h"
#include "../config/Config.h"
#include "../relay/CoordinateTransform.h"
#include <GL/glut.h>

namespace SceneRenderer {

static RobotModel s_robotModel;

RobotModel& getRobotModel() { return s_robotModel; }

void init() {
    // 加载 STL 模型（如果 models/cr3/ 目录存在），否则自动 fallback 几何体
    if (!s_robotModel.loadModels("models/cr3")) {
        s_robotModel.setFallbackMode();
    }
}

void drawFloor() {
    glColor4f(0.22f, 0.25f, 0.30f, 0.40f);
    glLineWidth(1.0f);

    float size = 300.0f;
    float step = 50.0f;
    int lines = (int)(size / step);

    glBegin(GL_LINES);
    for (int i = -lines; i <= lines; i++) {
        float p = i * step;
        glVertex3f(p, -size, 0);
        glVertex3f(p, size, 0);
        glVertex3f(-size, p, 0);
        glVertex3f(size, p, 0);
    }
    glEnd();
}

void drawAxes() {
    glLineWidth(3.0f);
    glBegin(GL_LINES);
    // X (红)
    glColor3f(1.0f, 0.35f, 0.35f); glVertex3f(0, 0, 0); glVertex3f(50, 0, 0);
    // Y (绿)
    glColor3f(0.35f, 0.95f, 0.45f); glVertex3f(0, 0, 0); glVertex3f(0, 50, 0);
    // Z (蓝)
    glColor3f(0.35f, 0.55f, 1.0f); glVertex3f(0, 0, 0); glVertex3f(0, 0, 50);
    glEnd();
}

void drawBoundary() {
    auto& cfg = Config::SAFE_X_MIN; // use sym for brevity; actually reference each
    float xMin = (float)Config::SAFE_X_MIN, xMax = (float)Config::SAFE_X_MAX;
    float yMin = (float)Config::SAFE_Y_MIN, yMax = (float)Config::SAFE_Y_MAX;
    float zMin = (float)Config::SAFE_Z_MIN, zMax = (float)Config::SAFE_Z_MAX;

    glColor4f(1.0f, 0.78f, 0.28f, 0.40f);
    glLineWidth(1.0f);

    // 画安全边界线框
    glBegin(GL_LINE_LOOP);
    glVertex3f(xMin, yMin, zMin); glVertex3f(xMax, yMin, zMin);
    glVertex3f(xMax, yMax, zMin); glVertex3f(xMin, yMax, zMin);
    glEnd();
    glBegin(GL_LINE_LOOP);
    glVertex3f(xMin, yMin, zMax); glVertex3f(xMax, yMin, zMax);
    glVertex3f(xMax, yMax, zMax); glVertex3f(xMin, yMax, zMax);
    glEnd();
    glBegin(GL_LINES);
    glVertex3f(xMin, yMin, zMin); glVertex3f(xMin, yMin, zMax);
    glVertex3f(xMax, yMin, zMin); glVertex3f(xMax, yMin, zMax);
    glVertex3f(xMax, yMax, zMin); glVertex3f(xMax, yMax, zMax);
    glVertex3f(xMin, yMax, zMin); glVertex3f(xMin, yMax, zMax);
    glEnd();
}

void drawCursor(const Vec3& pos) {
    glColor4f(1.0f, 1.0f, 1.0f, 0.9f);

    glPushMatrix();
    glTranslatef((float)pos.x, (float)pos.y, (float)pos.z);
    glutSolidSphere(4.0, 16, 16);
    glPopMatrix();

    // 发光光环
    glColor4f(0.25f, 0.85f, 1.0f, 0.5f);
    glPushMatrix();
    glTranslatef((float)pos.x, (float)pos.y, (float)pos.z);
    glutWireSphere(8.0, 12, 12);
    glPopMatrix();
}

void drawTargetMarker(const Vec3& pos) {
    glColor4f(1.0f, 0.3f, 0.3f, 0.5f);
    glLineWidth(2.0f);
    glPushMatrix();
    glTranslatef((float)pos.x, (float)pos.y, (float)pos.z);
    glutWireCube(10.0);
    glPopMatrix();
}

void drawActualMarker(const Vec3& pos) {
    glColor4f(0.3f, 0.9f, 0.4f, 0.9f);
    glPushMatrix();
    glTranslatef((float)pos.x, (float)pos.y, (float)pos.z);
    glutSolidSphere(5.0, 12, 12);
    glPopMatrix();
}

void drawTrail() {
    auto& app = appState;
    EnterCriticalSection(&app.trailMutex);
    if (app.trailPoints.size() < 2) {
        LeaveCriticalSection(&app.trailMutex);
        return;
    }

    glColor4f(0.25f, 0.85f, 1.0f, 0.7f);
    glLineWidth(2.0f);
    glBegin(GL_LINE_STRIP);
    for (auto& p : app.trailPoints) {
        Vec3 v = convertTouchToRobot(p);
        glVertex3f((float)v.x, (float)v.y, (float)v.z);
    }
    glEnd();
    LeaveCriticalSection(&app.trailMutex);
}

void draw3D() {
    drawFloor();
    drawBoundary();
    drawAxes();

    // 机械臂模型
    AppState::RobotPose actual, target;
    {
        EnterCriticalSection(&appState.robotPoseMutex);
        actual = appState.robotActualPose;
        target = appState.robotTargetPose;
        LeaveCriticalSection(&appState.robotPoseMutex);
    }

    // 机械臂用实际位姿驱动
    s_robotModel.draw(actual);

    // 目标位置标记
    Vec3 targetPos(target.x, target.y, target.z);
    Vec3 actualPos(actual.x, actual.y, actual.z);
    drawTargetMarker(targetPos);
    drawActualMarker(actualPos);

    // Touch 光标
    Vec3 mapped;
    EnterCriticalSection(&appState.adjustedPosTableMutex);
    mapped = appState.adjustedPosTable;
    LeaveCriticalSection(&appState.adjustedPosTableMutex);
    drawCursor(mapped);

    // 轨迹
    drawTrail();
}

} // namespace SceneRenderer
```

---

### Task 14: 更新 HapticCallback —— 通过 RelayCore 通信

**Files:**
- Modify: `Codes/Touch_Client/haptic/HapticCallback.cpp`

**Interfaces:**
- Consumes: `RelayCore`
- 变更：不再发 TCP 数据，改为调 `RelayCore::sendPosition()` 和 `RelayCore::onButtonPress/Release()`

- [ ] **Step 1: 重写 HapticCallback.cpp**

```cpp
#include "HapticCallback.h"
#include "../core/AppState.h"
#include "../relay/CoordinateTransform.h"
#include "../relay/RelayCore.h"
#include "../core/MathUtils.h"
#include "../config/Config.h"
#include <HDU/hduVector.h>

HDCallbackCode HDCALLBACK hapticCallback(void* pUserData) {
    auto& app = appState;
    auto& relay = RelayCore::instance();

    if (app.isClosing) return HD_CALLBACK_DONE;

    hdBeginFrame(app.hHD);

    // ===== 1. 读取位置 =====
    hduVector3Dd newPos;
    hdGetDoublev(HD_CURRENT_POSITION, newPos);

    EnterCriticalSection(&app.devicePosMutex);
    app.devicePos = newPos;
    app.devicePos[0] = clamp(app.devicePos[0], Config::DEV_X_MIN, Config::DEV_X_MAX);
    app.devicePos[1] = clamp(app.devicePos[1], Config::DEV_Y_MIN, Config::DEV_Y_MAX);
    app.devicePos[2] = clamp(app.devicePos[2], Config::DEV_Z_MIN, Config::DEV_Z_MAX);
    hduVector3Dd localDevicePos = app.devicePos;
    LeaveCriticalSection(&app.devicePosMutex);

    // ===== 2. 坐标转换到 robot 系 =====
    Vec3 robotPos = convertTouchToRobot(localDevicePos);

    EnterCriticalSection(&app.adjustedPosTableMutex);
    app.adjustedPosTable = robotPos;
    LeaveCriticalSection(&app.adjustedPosTableMutex);

    EnterCriticalSection(&app.adjustedPosMutex);
    app.adjustedPos = localDevicePos;
    LeaveCriticalSection(&app.adjustedPosMutex);

    // ===== 3. 轨迹 =====
    EnterCriticalSection(&app.trailMutex);
    app.trailPoints.push_back(localDevicePos);
    while ((int)app.trailPoints.size() > Config::MAX_TRAIL) {
        app.trailPoints.pop_front();
    }
    LeaveCriticalSection(&app.trailMutex);

    // ===== 4. 按钮状态 =====
    int buttonState = 0;
    hdGetIntegerv(HD_CURRENT_BUTTONS, &buttonState);
    bool button1 = (buttonState & HD_DEVICE_BUTTON_1) != 0;
    bool button2 = (buttonState & HD_DEVICE_BUTTON_2) != 0;
    app.button2Pressed = button2;

    // ===== 5. 按钮 1 状态机 → RelayCore =====
    bool stateChanged = (button1 != app.lastButtonState);
    if (stateChanged) {
        app.lastButtonState = button1;
        if (button1) {
            relay.onButtonPress(robotPos);
        } else {
            relay.onButtonRelease();
        }
    }

    // ===== 6. 持续发送（按钮保持按下） =====
    if (relay.isTransmitting()) {
        relay.sendPosition(localDevicePos);
    }

    hdEndFrame(app.hHD);
    return HD_CALLBACK_CONTINUE;
}
```

- [ ] **Step 2: 移除 HapticCallback.cpp 中已不存在的 include**

确认以下 include 不再出现：`SenderQueue.h`（不再手动入队，Relay 内部处理）

---

### Task 15: 重写 main.cpp —— GLUT 主循环 + 三层组装

**Files:**
- Modify: `Codes/Touch_Client/main.cpp`

**Interfaces:**
- Consumes: 所有模块
- Produces: `int main()` — GLUT 窗口 + 三个定时器 + 主循环

- [ ] **Step 1: 重写 main.cpp**

```cpp
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#include <GL/glut.h>
#include <HD/hd.h>
#include <HDU/hduVector.h>
#include <iostream>
#include <cmath>
#include <windows.h>

#include "config/Config.h"
#include "core/AppState.h"
#include "haptic/HapticDevice.h"
#include "relay/RelayCore.h"
#include "render/SceneRenderer.h"
#include "render/HudOverlay.h"

// ===== GLUT 回调 =====
float g_rotateX = 15.0f, g_rotateY = 10.0f;
float g_camDist = 2.0f;
int g_lastX = 0, g_lastY = 0;
bool g_dragging = false;

void display() {
    if (appState.isClosing) return;

    glClearColor(0.1f, 0.12f, 0.18f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    // 3D 投影
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, (double)Config::WINDOW_W / Config::WINDOW_H, 10.0, 2000.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // 相机：看向工作区中心
    double cx = (Config::SAFE_X_MIN + Config::SAFE_X_MAX) / 2.0;
    double cy = (Config::SAFE_Y_MIN + Config::SAFE_Y_MAX) / 2.0;
    double cz = (Config::SAFE_Z_MIN + Config::SAFE_Z_MAX) / 2.0;
    double camDist = 600.0 * g_camDist;

    gluLookAt(cx, cy - camDist * 0.5, cz + camDist,  // 相机位置
              cx, cy, cz,                               // 看向中心
              0, 0, 1);                                 // 上方向 Z

    glRotatef(g_rotateX, 1, 0, 0);
    glRotatef(g_rotateY, 0, 0, 1);

    // 3D 场景
    SceneRenderer::draw3D();

    // 2D HUD
    HudOverlay::drawAll();

    glutSwapBuffers();

    // 非阻塞反馈处理
    RelayCore::instance().pollFeedback();
}

void idle() {
    if (!appState.isClosing) { glutPostRedisplay(); Sleep(1); }
}

void reshape(int w, int h) {
    glViewport(0, 0, w, h);
}

void mouse(int button, int state, int x, int y) {
    if (button == GLUT_LEFT_BUTTON) {
        g_dragging = (state == GLUT_DOWN);
        if (g_dragging) { g_lastX = x; g_lastY = y; }
    }
    else if (button == 3 && state == GLUT_DOWN) { // 滚轮上
        g_camDist *= 0.9f;
        if (g_camDist < 0.3f) g_camDist = 0.3f;
    }
    else if (button == 4 && state == GLUT_DOWN) { // 滚轮下
        g_camDist *= 1.1f;
        if (g_camDist > 5.0f) g_camDist = 5.0f;
    }
}

void motion(int x, int y) {
    if (!g_dragging) return;
    g_rotateY += (x - g_lastX) * 0.5f;
    g_rotateX -= (y - g_lastY) * 0.5f;
    if (g_rotateX < -60.0f) g_rotateX = -60.0f;
    if (g_rotateX > 60.0f) g_rotateX = 60.0f;
    g_lastX = x; g_lastY = y;
}

// ===== 定时器 =====
void poseQueryTimer(int) {
    RelayCore::instance().queryPose();
    if (!appState.isClosing) {
        glutTimerFunc(Config::POSE_QUERY_INTERVAL, poseQueryTimer, 0);
    }
}

void alarmCheckTimer(int) {
    RelayCore::instance().checkAlarm();
    if (!appState.isClosing) {
        glutTimerFunc(Config::ALARM_CHECK_INTERVAL, alarmCheckTimer, 0);
    }
}

void keyboard(unsigned char key, int, int) {
    if (key == 'q' || key == 'Q' || key == 27) { // q 或 ESC
        std::cout << "\nShutting down..." << std::endl;
        // 注意: 原始 GLUT 3.2 不支持 glutLeaveMainLoop()
        // 在主循环中直接做清理然后 exit
        RelayCore::instance().shutdown();
        cleanupHapticDevice();
        exit(0);
    }
}

// ===== 主函数 =====
int main(int argc, char* argv[]) {
    std::cout << "=== Touch-Dobot Digital Twin System v3.0 ===" << std::endl;
    std::cout << "Robot: " << Config::ROBOT_IP << std::endl;
    std::cout << "Safety: X[" << Config::SAFE_X_MIN << "," << Config::SAFE_X_MAX
              << "] Y[" << Config::SAFE_Y_MIN << "," << Config::SAFE_Y_MAX
              << "] Z[" << Config::SAFE_Z_MIN << "," << Config::SAFE_Z_MAX << "]" << std::endl;

    // 1. GLUT 初始化
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(Config::WINDOW_W, Config::WINDOW_H);
    glutInitWindowPosition(100, 100);
    glutCreateWindow("Touch-Dobot Digital Twin");

    glutDisplayFunc(display);
    glutIdleFunc(idle);
    glutReshapeFunc(reshape);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    glutKeyboardFunc(keyboard);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // 2. 初始化 Touch
    std::cout << "Initializing Touch device..." << std::endl;
    if (!initHapticDevice()) {
        std::cerr << "ERROR: Touch device init failed" << std::endl;
        return -1;
    }

    // 3. 通过 Relay 层连接机械臂
    std::cout << "Initializing robot via Relay..." << std::endl;
    if (!RelayCore::instance().init()) {
        std::cerr << "ERROR: Robot init failed" << std::endl;
        cleanupHapticDevice();
        return -1;
    }

    // 4. 初始化 3D 场景
    SceneRenderer::init();

    // 5. 启动定时器
    glutTimerFunc(Config::POSE_QUERY_INTERVAL, poseQueryTimer, 0);
    glutTimerFunc(Config::ALARM_CHECK_INTERVAL, alarmCheckTimer, 0);

    // 6. 进入主循环
    std::cout << "\nSystem ready." << std::endl;
    std::cout << "  Mouse drag: rotate view" << std::endl;
    std::cout << "  Mouse wheel: zoom" << std::endl;
    std::cout << "  q/ESC: quit" << std::endl;
    std::cout << "  Touch button 1: control robot\n" << std::endl;

    glutMainLoop(); // 原始 GLUT 3.2: 此函数不返回，退出通过键盘回调中的 exit(0)

    return 0; // unreachable
}
```

---

### Task 16: 更新 .vcxproj + build 脚本

**Files:**
- Modify: `Codes/Touch_Client/Touch_Client.vcxproj`
- Create: `Codes/Touch_Client/build.bat`

- [ ] **Step 1: 更新 .vcxproj 源文件列表**

在 vcxproj 的 `<ItemGroup>` 中，删除旧文件引用，添加新文件。

删除以下行：
```xml
    <ClInclude Include="network\PongHandler.h" />
    <ClCompile Include="network\PongHandler.cpp" />
    <ClInclude Include="network\RelayProtocol.h" />
    <ClCompile Include="network\RelayProtocol.cpp" />
    <ClInclude Include="network\TcpClient.h" />
    <ClCompile Include="network\TcpClient.cpp" />
    <ClInclude Include="robot\SafetyBoundary.h" />
    <ClInclude Include="core\CoordinateTransform.h" />
    <ClInclude Include="utils\MathUtils.h" />
```

添加以下行：
```xml
    <!-- core -->
    <ClInclude Include="core\MathUtils.h" />
    <!-- robot -->
    <ClInclude Include="robot\RobotConnection.h" />
    <ClCompile Include="robot\RobotConnection.cpp" />
    <!-- relay -->
    <ClInclude Include="relay\IExtension.h" />
    <ClInclude Include="relay\CoordinateTransform.h" />
    <ClInclude Include="relay\SafetyBoundary.h" />
    <ClInclude Include="relay\ProtocolAdapter.h" />
    <ClCompile Include="relay\ProtocolAdapter.cpp" />
    <ClInclude Include="relay\FeedbackParser.h" />
    <ClCompile Include="relay\FeedbackParser.cpp" />
    <ClInclude Include="relay\RelayCore.h" />
    <ClCompile Include="relay\RelayCore.cpp" />
    <!-- render -->
    <ClInclude Include="render\StlMesh.h" />
    <ClInclude Include="render\StlLoader.h" />
    <ClCompile Include="render\StlLoader.cpp" />
    <ClInclude Include="render\RobotModel.h" />
    <ClCompile Include="render\RobotModel.cpp" />
    <ClInclude Include="render\SceneRenderer.h" />
    <ClCompile Include="render\SceneRenderer.cpp" />
    <ClInclude Include="render\HudOverlay.h" />
    <ClCompile Include="render\HudOverlay.cpp" />
```

- [ ] **Step 2: 创建 build.bat**

```bat
@echo off
chcp 65001 >nul
setlocal

set "MSBUILD=D:\Program Files\Microsoft Visual Studio\2022\BuildTools\MSBuild\Current\Bin\MSBuild.exe"
set "PROJECT=%~dp0Touch_Client.vcxproj"
set "OUTDIR=%~dp0x64\Release"
set "OH_SDK=D:\Projects\Touch\OpenHaptics\Developer\3.5.0"

echo ================================================
echo   Touch_Client v3.0 Build
echo ================================================
echo.

echo [1/2] Building...
"%MSBUILD%" "%PROJECT%" /p:Configuration=Release /p:Platform=x64 /v:minimal
if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] Build failed!
    pause
    exit /b 1
)
echo   Build OK.

echo.
echo [2/2] Copying DLLs...
copy /Y "%OH_SDK%\lib\x64\Release\hd.dll" "%OUTDIR%\" >nul
copy /Y "%OH_SDK%\utilities\lib\x64\Release\hdu.dll" "%OUTDIR%\" >nul
copy /Y "%OH_SDK%\utilities\lib\x64\Release\glut32.dll" "%OUTDIR%\" >nul
echo   DLLs copied.

echo.
echo Build complete. Run: %OUTDIR%\Touch_Client.exe
endlocal
```

---

### Task 17: STL 模型获取脚本 + 后备几何体

**Files:**
- Create: `scripts/fetch_cr3_models.bat`

- [ ] **Step 1: 创建模型拉取脚本**

```bat
@echo off
chcp 65001 >nul
setlocal

set "MODEL_DIR=%~dp0..\Codes\Touch_Client\models\cr3"
set "TEMP_DIR=%TEMP%\cr3_models_clone"

echo ================================================
echo   Fetch Dobot CR3 STL Models
echo ================================================
echo.
echo Target: %MODEL_DIR%
echo.

if exist "%MODEL_DIR%\base.stl" (
    echo Models already exist. Skipping fetch.
    goto :done
)

echo Cloning movensys_manipulator_description (sparse, models only)...
echo This may take a minute...

git clone --depth 1 --filter=blob:none --sparse ^
    https://github.com/movensys/movensys_manipulator_description.git "%TEMP_DIR%" 2>&1
if %ERRORLEVEL% NEQ 0 (
    echo.
    echo [WARNING] git clone failed. Falling back to geometric models.
    echo The application will use geometric primitives (cylinders + boxes) instead of STL meshes.
    goto :done
)

cd /d "%TEMP_DIR%"
git sparse-checkout set meshes/cr3a

if not exist "meshes\cr3a\*.stl" (
    echo [WARNING] No STL files found in expected path. Falling back to geometric models.
    cd /d "%~dp0"
    rmdir /s /q "%TEMP_DIR%" 2>nul
    goto :done
)

echo Copying STL files...
if not exist "%MODEL_DIR%" mkdir "%MODEL_DIR%"

:: CR3 的 STL 文件名可能不同，批量拷贝
copy /Y "meshes\cr3a\*.stl" "%MODEL_DIR%\" >nul

:: 如果文件命名不符合预期(base/link1~6)，列出可用文件供手动调整
echo.
echo Available STL files:
dir /b "%MODEL_DIR%"

cd /d "%~dp0"
rmdir /s /q "%TEMP_DIR%" 2>nul
echo.
echo Models fetched successfully.

:done
endlocal
```

- [ ] **Step 2: 确认后备几何体已实现在 RobotModel 中**

RobotModel.cpp 的 `draw()` 方法已包含完整的 `m_useFallback` 分支（见 Task 11），无需额外代码。

---

### Task 18: 删除废弃文件

**Files:**
- Delete: `network/` 目录下所有文件 + 空目录
- Delete: `core/CoordinateTransform.h` (已迁移到 relay/)
- Delete: `robot/SafetyBoundary.h` (已迁移到 relay/)
- Delete: `robot/CommandBuilder.h` (被 ProtocolAdapter 替代)

- [ ] **Step 1: 清理**

```bash
rm -rf Codes/Touch_Client/network
rm -f Codes/Touch_Client/core/CoordinateTransform.h
rm -f Codes/Touch_Client/robot/SafetyBoundary.h
rm -f Codes/Touch_Client/robot/CommandBuilder.h
rm -f Codes/Touch_Client/robot/RobotController.h
rm -f Codes/Touch_Client/robot/RobotController.cpp
rm -f Codes/Touch_Client/core/SenderQueue.h
rm -f Codes/Touch_Client/core/SenderQueue.cpp
rm -f Codes/Touch_Client/glut_test.cpp
```

---

### Task 19: 编译验证 + 功能检查

- [ ] **Step 1: 编译**

```bash
cd Codes\Touch_Client
build.bat
```

预期：无编译错误。

- [ ] **Step 2: 验证文件清单**

```bash
find Codes/Touch_Client -name "*.h" -o -name "*.cpp" | sort
```

预期输出（22 个文件）：
```
Codes/Touch_Client/config/Config.h
Codes/Touch_Client/core/AppState.cpp
Codes/Touch_Client/core/AppState.h
Codes/Touch_Client/core/MathUtils.h
Codes/Touch_Client/haptic/HapticCallback.cpp
Codes/Touch_Client/haptic/HapticCallback.h
Codes/Touch_Client/haptic/HapticDevice.cpp
Codes/Touch_Client/haptic/HapticDevice.h
Codes/Touch_Client/main.cpp
Codes/Touch_Client/relay/CoordinateTransform.h
Codes/Touch_Client/relay/FeedbackParser.cpp
Codes/Touch_Client/relay/FeedbackParser.h
Codes/Touch_Client/relay/IExtension.h
Codes/Touch_Client/relay/ProtocolAdapter.cpp
Codes/Touch_Client/relay/ProtocolAdapter.h
Codes/Touch_Client/relay/RelayCore.cpp
Codes/Touch_Client/relay/RelayCore.h
Codes/Touch_Client/relay/SafetyBoundary.h
Codes/Touch_Client/render/HudOverlay.cpp
Codes/Touch_Client/render/HudOverlay.h
Codes/Touch_Client/render/RobotModel.cpp
Codes/Touch_Client/render/RobotModel.h
Codes/Touch_Client/render/SceneRenderer.cpp
Codes/Touch_Client/render/SceneRenderer.h
Codes/Touch_Client/render/StlLoader.cpp
Codes/Touch_Client/render/StlLoader.h
Codes/Touch_Client/render/StlMesh.h
Codes/Touch_Client/robot/RobotConnection.cpp
Codes/Touch_Client/robot/RobotConnection.h
```

- [ ] **Step 3: 确认旧代码已清理**

```bash
ls Codes/Touch_Client/network 2>&1    # 预期: No such file or directory
ls Codes/Touch_Client/utils 2>&1      # 预期: No such file or directory
```

- [ ] **Step 4: 确认三层解耦**

```bash
# Touch 层不应引用 Robot 层
grep -r "robot/" Codes/Touch_Client/haptic/ 2>&1  # 预期: 无匹配

# Robot 层不应引用 Touch 层
grep -r "haptic/" Codes/Touch_Client/robot/ 2>&1  # 预期: 无匹配

# Relay 层可以引用两者 — 这是允许的
grep -r "#include" Codes/Touch_Client/relay/*.cpp | grep -E "haptic/|robot/"
```

---
