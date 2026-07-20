# Touch-Dobot Touch 端代码重构 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将 test8.3 单体文件重构为模块化的 Touch_Client/ 项目，同时将跟随延迟从秒级优化到 <100ms。

**Architecture:** 分层模块化 C++ 项目，按依赖自底向上构建：Config → utils → core → network/robot → haptic → render → main。运动控制从 `MovL`（队列阻塞）切换为 `ServoP`（实时覆盖），发送与反馈解耦为异步管道。

**Tech Stack:** C++17 / OpenHaptics SDK (HDAPI) / freeglut (OpenGL) / Winsock2 / Visual Studio 2022

## Global Constraints

- 保持现有已验证的坐标映射关系不变
- 姿态固定（继承 GetPose 返回的 Rx/Ry/Rz），预留读取 HD_CURRENT_TRANSFORM 和 HD_DEVICE_BUTTON_2 的接口
- 中继站透明转发（假设），协议格式 `端口|指令`
- Windows 平台，Visual Studio 项目，无需 CMake
- 不实现力反馈闭环（后续迭代）

---

### Task 1: 清理旧代码并创建项目骨架

**Files:**
- Create: `Codes/Touch_Client/` 目录树
- Delete: `Codes/test1/` 至 `Codes/test7.3/`, `Codes/test8.1/` 至 `Codes/test8.3/`
- Delete: `Codes/Debug/`, `Codes/.vs/`
- Copy: `Codes/test8.3/pics/` → `Codes/Touch_Client/pics/`
- Copy: `Codes/test8.3/StbImage/stb_image.h` → `Codes/Touch_Client/vendor/stb_image.h`

**Interfaces:**
- Produces: 空目录结构供后续任务填充

- [ ] **Step 1: 创建目录结构**

```bash
mkdir -p Codes/Touch_Client/{config,core,haptic,render,network,robot,utils,vendor,pics}
```

- [ ] **Step 2: 迁移资源文件**

```bash
cp Codes/test8.3/pics/NINELAB.png Codes/Touch_Client/pics/
cp "Codes/test8.3/StbImage/stb_image.h" Codes/Touch_Client/vendor/
```

- [ ] **Step 3: 删除旧版本源代码目录**

```bash
rm -rf Codes/test1 "Codes/test2" "Codes/test3(Coordinate recording)" "Codes/test4(Coordinate recording visualization 2D)" "Codes/test4(Coordinate recording visualization)" "Codes/test5(Coordinate recording visualization 3D)" "Codes/test6(Touch_communication_client)" "Codes/test6(Touch_communication_server)" "Codes/test6(Touch_communication_with_computer)" "Codes/test6.1(Touch_realtime_coordinate_client)" "Codes/test6.1(Touch_realtime_coordinate_server)" "Codes/test6.2(Touch_relative_coordinate_client)" "Codes/test6.2(Touch_relative_coordinate_server)" "Codes/test7(Touch_RoboticArm_Commu_Client)" "Codes/test7(Touch_RoboticArm_Commu_Server)" "Codes/test7.1(control_DOBOT)" "Codes/test7.2(Touch_RoboticArm_relative_coordinate_Client)" "Codes/test7.3(Touch_RoboticArm_realtime_coordinate_Client)" "Codes/test8.1(Touch_RoboticArm_relative_coordinate_Client_Network)" "Codes/test8.2(Touch_RoboticArm_realtime_coordinate_Client_Network)" "Codes/test8.3"
```

- [ ] **Step 4: 删除编译产物和缓存**

```bash
rm -rf Codes/Debug Codes/.vs
```

- [ ] **Step 5: 验证清理结果**

```bash
ls Codes/
# 预期输出: Touch_Client/
ls Codes/Touch_Client/
# 预期输出: config/ core/ haptic/ render/ network/ robot/ utils/ vendor/ pics/
```

---

### Task 2: Config.h — 集中配置

**Files:**
- Create: `Codes/Touch_Client/config/Config.h`

**Interfaces:**
- Produces: `Config` 命名空间，所有模块只读引用
  - `Config::TCP_RELAY_IP`, `Config::RELAY_PORT`, `Config::ENABLE_PORT`, `Config::MOTION_PORT`
  - `Config::WINDOW_W/H`, `Config::DEV_X/Y/Z_MIN/MAX`, `Config::MAX_ABS`
  - `Config::SAFE_X/Y/Z_MIN/MAX`（安全边界）
  - `Config::SpeedL`, `Config::MIN_DELTA_THRESHOLD`, `Config::CP_SMOOTH_RATIO`
  - `Config::MAX_QUEUE_SIZE`, `Config::FEEDBACK_TIMEOUT`, `Config::ALARM_CHECK_INTERVAL`
  - 所有颜色常量、表格参数、Logo 参数

- [ ] **Step 1: 编写 Config.h**

```cpp
#pragma once

namespace Config {
    // ========== 窗口参数 ==========
    const int WINDOW_W = 1024;
    const int WINDOW_H = 768;

    // ========== Touch 设备坐标范围 ==========
    const double MAX_ABS = 150;
    const double DEV_X_MIN = -MAX_ABS, DEV_X_MAX = MAX_ABS;
    const double DEV_Y_MIN = -MAX_ABS, DEV_Y_MAX = MAX_ABS;
    const double DEV_Z_MIN = -MAX_ABS, DEV_Z_MAX = MAX_ABS;
    const double CENTER_X = (DEV_X_MIN + DEV_X_MAX) / 2.0;
    const double CENTER_Y = (DEV_Y_MIN + DEV_Y_MAX) / 2.0;
    const double CENTER_Z = (DEV_Z_MIN + DEV_Z_MAX) / 2.0;

    // ========== 安全边界（机械臂用户坐标系，单位mm） ==========
    // 初始值设为保守范围，根据实际环境调整
    const double SAFE_X_MIN = 180.0, SAFE_X_MAX = 420.0;
    const double SAFE_Y_MIN = -200.0, SAFE_Y_MAX = 200.0;
    const double SAFE_Z_MIN = 30.0,  SAFE_Z_MAX = 300.0;
    const double SAFE_BOUNDARY_BUFFER_RATIO = 0.2; // 20%边界缓冲区，线速度衰减

    // ========== 网络参数 ==========
    const char* TCP_RELAY_IP = "192.168.101.25";
    const int RELAY_PORT = 8888;
    const int ENABLE_PORT = 29999;
    const int MOTION_PORT = 30003;
    const int RECV_BUFFER_SIZE = 1024 * 64; // 64KB

    // ========== 机械臂运动参数 ==========
    const float SpeedL = 100;                // 运动速度比例 (1~100)
    const float MIN_DELTA_THRESHOLD = 1.0f;  // 最小位移阈值 (mm)
    const unsigned int CP_SMOOTH_RATIO = 100; // 平滑过渡比例 (0~100)

    // ========== 发送队列参数 ==========
    const int MAX_QUEUE_SIZE = 5;            // 队列容量上限（满时丢弃旧数据）
    const int TCP_SEND_INTERVAL = 10;        // 发送间隔 (ms)
    const int FEEDBACK_TIMEOUT = 2000;       // 反馈读取超时 (ms)
    const int ALARM_CHECK_INTERVAL = 300;    // 报警巡检间隔 (ms)
    const int IDLE_SLEEP_MS = 1;             // 发送线程空闲休眠 (ms)

    // ========== 3D 投影参数 ==========
    const float AXIS_LINE_WIDTH = 3.0f;
    const double BASE_CAM_X = 0.0, BASE_CAM_Y = 70.0, BASE_CAM_Z = 240.0;
    const double NEAR_CLIP = 1.0, FAR_CLIP = 800.0, FOV = 45.0;
    const float MIN_ZOOM = 0.3f, MAX_ZOOM = 5.0f, ZOOM_STEP = 1.5f;
    const float ROTATION_SPEED = 0.5f;

    // ========== 交互参数 ==========
    const int MAX_TRAIL = 300;

    // ========== Logo 参数 ==========
    const char* LOGO_PATH = "pics/NINELAB.png";
    const int LOGO_WIDTH = 150, LOGO_HEIGHT = 75;

    // ========== 颜色定义 ==========
    const float COLOR_FLOOR[4]       = { 0.22f, 0.25f, 0.30f, 0.55f };
    const float COLOR_BORDER[4]      = { 0.62f, 0.68f, 0.78f, 0.50f };
    const float COLOR_AXIS_X[4]      = { 1.00f, 0.35f, 0.35f, 0.95f };
    const float COLOR_AXIS_Y[4]      = { 0.35f, 0.95f, 0.45f, 0.95f };
    const float COLOR_AXIS_Z[4]      = { 0.35f, 0.55f, 1.00f, 0.95f };
    const float COLOR_CURSOR_DOT[4]  = { 1.00f, 1.00f, 1.00f, 0.95f };
    const float COLOR_TEXT[4]        = { 0.92f, 0.96f, 1.00f, 1.00f };
    const float COLOR_SUCCESS[4]     = { 0.35f, 0.90f, 0.50f, 1.00f };
    const float COLOR_ERROR[4]       = { 1.00f, 0.35f, 0.35f, 1.00f };
    const float COLOR_WARNING[4]     = { 1.00f, 0.78f, 0.28f, 1.00f };
    const float COLOR_TRAIL[4]       = { 0.25f, 0.85f, 1.00f, 0.90f };

    // ========== 坐标表格参数 ==========
    const int TABLE_LEFT = 15;
    const int TABLE_TOP = WINDOW_H - 100;
    const int TABLE_WIDTH = 230;
    const int TABLE_COL1_W = 70;
    const int TABLE_PADDING = 10;
    const int TABLE_TITLE_ROW_H = 34;
    const int TABLE_ROW_H = 26;
    const float TABLE_BG_COLOR[4]       = { 0.10f, 0.13f, 0.17f, 0.82f };
    const float TABLE_BORDER_COLOR[4]   = { 0.42f, 0.56f, 0.78f, 0.85f };
    const float TABLE_ALT_ROW_COLOR[4]  = { 0.12f, 0.16f, 0.21f, 0.82f };
    const float TABLE_CELL_TEXT_COLOR[4]= { 0.92f, 0.96f, 1.00f, 1.00f };
    const float TABLE_TITLE_BG[4]       = { 0.16f, 0.28f, 0.55f, 0.95f };
    const float TABLE_TITLE_TEXT_COLOR[4]={ 0.95f, 0.98f, 1.00f, 1.0f };
    const float TABLE_BORDER_WIDTH = 1.0f;

    // ========== 状态栏参数 ==========
    const int TCP_STATUS_BAR_HEIGHT = 30;

    // ========== PING/PONG 参数 ==========
    const char* PING_PREFIX = "PING|";
    const char* PONG_PREFIX = "PONG|";
    const char* ROBOT_CLOSED_MSG = "ROBOT_ARM_CLOSED";
}
```

- [ ] **Step 2: 验证编译**

无，Config.h 仅含常量，不单独编译。在后续 Task 中验证。

---

### Task 3: MathUtils.h — 数学工具

**Files:**
- Create: `Codes/Touch_Client/utils/MathUtils.h`

**Interfaces:**
- Produces: 模板函数 `clamp(value, min, max)` — 将值限制在 [min, max] 范围内

- [ ] **Step 1: 编写 MathUtils.h**

```cpp
#pragma once

template<typename T>
const T& clamp(const T& value, const T& min, const T& max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}
```

---

### Task 4: CoordinateTransform — 坐标转换

**Files:**
- Create: `Codes/Touch_Client/core/CoordinateTransform.h`

**Interfaces:**
- Produces:
  - `struct Vec3 { double x, y, z; }` — 三维向量
  - `Vec3 convertTouchToRobot(const double devicePos[3])` — Touch 原始坐标 → 机械臂右手系
  - `Vec3 computeDelta(const Vec3& current, const Vec3& base)` — 计算相对位移
  - `Vec3 computeTarget(const Vec3& robotBase, const Vec3& delta)` — 计算机械臂绝对目标

- [ ] **Step 1: 编写 CoordinateTransform.h**

```cpp
#pragma once
#include <cmath>

struct Vec3 {
    double x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    Vec3 operator-(const Vec3& other) const {
        return Vec3(x - other.x, y - other.y, z - other.z);
    }
    Vec3 operator+(const Vec3& other) const {
        return Vec3(x + other.x, y + other.y, z + other.z);
    }
    double length() const {
        return std::sqrt(x * x + y * y + z * z);
    }
};

// Touch 原始坐标 (devicePos[3]) → 机械臂右手系
inline Vec3 convertTouchToRobot(const double devicePos[3]) {
    return Vec3(
         devicePos[0],   // X → X
        -devicePos[2],   // Z(反转) → Y
         devicePos[1]    // Y → Z
    );
}

// 计算相对位移
inline Vec3 computeDelta(const Vec3& current, const Vec3& base) {
    return current - base;
}

// 计算机械臂绝对目标位置（用户坐标系）
inline Vec3 computeTarget(const Vec3& robotBase, const Vec3& delta) {
    return robotBase + delta;
}
```

---

### Task 5: AppState — 全局共享状态

**Files:**
- Create: `Codes/Touch_Client/core/AppState.h`
- Create: `Codes/Touch_Client/core/AppState.cpp`

**Interfaces:**
- Consumes: `Config::*`, `Vec3` from CoordinateTransform
- Produces: `AppState` 单例，包含所有线程安全的共享状态：
  - 触觉设备句柄：`hHD`, `hapticCallbackHandle`
  - TCP 套接字：`relaySocket`, 连接状态 `isTcpConnected`
  - 线程控制：`clientRunning`, `isClosing`, `stopEvent`
  - 坐标数据（带临界区保护）：`devicePos`, `adjustedPos`, `basePoint`, `robotBasePos`
  - 发送队列：`sendQueue`, `queueMutex`, `queueCV`
  - 按钮/传输状态：`isTransmitting`, `lastButtonState`
  - 接收缓冲：`recvBuffer[64KB]`, `recvBufferLen`
  - 状态显示字符串：`connectionStatus`, `transmissionState`, `lastTransmissionDetail`
  - 预留接口：`targetRx`, `targetRy`, `targetRz`, `button2Pressed`

- [ ] **Step 1: 编写 AppState.h**

```cpp
#pragma once
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#include <HD/hd.h>
#include <HDU/hduVector.h>
#include <atomic>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <vector>
#include "CoordinateTransform.h"

struct SendData {
    double deltaX, deltaY, deltaZ;
};

class AppState {
public:
    // ===== 触觉设备 =====
    HHD hHD = HD_INVALID_HANDLE;
    HDSchedulerHandle hapticCallbackHandle = HD_INVALID_HANDLE;
    bool deviceInitialized = false;

    // ===== 坐标数据 =====
    hduVector3Dd devicePos = { 0.0, 0.0, 0.0 };
    CRITICAL_SECTION devicePosMutex;
    hduVector3Dd adjustedPos = { 0.0, 0.0, 0.0 };
    CRITICAL_SECTION adjustedPosMutex;
    Vec3 adjustedPosTable = { 0.0, 0.0, 0.0 };
    CRITICAL_SECTION adjustedPosTableMutex;

    // ===== 姿态（预留接口） =====
    double targetRx = 0.0, targetRy = 0.0, targetRz = 0.0;
    double transformMatrix[16] = { 0 };  // HD_CURRENT_TRANSFORM 预留

    // ===== TCP 网络 =====
    SOCKET relaySocket = INVALID_SOCKET;
    CRITICAL_SECTION relaySocketMutex;
    std::atomic<bool> isTcpConnected{ false };
    char recvBuffer[1024 * 64] = { 0 };
    int recvBufferLen = 0;
    CRITICAL_SECTION recvBufferMutex;

    // ===== 机械臂基准位置 =====
    Vec3 robotBase = { 0.0, 0.0, 0.0 };
    double robotBaseRx = 0.0, robotBaseRy = 0.0, robotBaseRz = 0.0;
    std::atomic<bool> isRobotBaseSet{ false };
    std::atomic<bool> isRobotInAlarm{ false };

    // ===== 线程控制 =====
    std::atomic<bool> clientRunning{ false };
    std::atomic<bool> isClosing{ false };
    std::atomic<bool> isSenderThreadRunning{ false };
    HANDLE clientThread = NULL;
    HANDLE senderThread = NULL;
    WSAEVENT stopEvent = WSA_INVALID_EVENT;
    CRITICAL_SECTION statusMutex;

    // ===== 发送队列 =====
    std::queue<SendData> sendQueue;
    std::mutex queueMutex;
    std::condition_variable queueCV;

    // ===== 按钮与传输状态 =====
    std::atomic<bool> isTransmitting{ false };
    std::atomic<bool> lastButtonState{ false };
    bool isBasePointSet = false;
    Vec3 basePoint;
    CRITICAL_SECTION basePointMutex;

    // ===== 预留：第二个按钮 =====
    std::atomic<bool> button2Pressed{ false };

    // ===== 轨迹 =====
    CRITICAL_SECTION trailMutex;
    std::deque<hduVector3Dd> trailPoints;

    // ===== 交互与渲染状态 =====
    double rotateX = 15.0, rotateY = 10.0;
    float camDistance = 1.0f;
    int lastMouseX = 0, lastMouseY = 0;
    bool isDragging = false;
    GLuint logoTextureID = 0;
    bool logoLoaded = false;

    // ===== 状态显示字符串 =====
    char connectionStatus[128] = "TCP: not connected";
    char serverInfo[64] = "IP: -";
    char transmissionState[128] = "STATE: -";
    char lastTransmissionDetail[256] = "Waiting for transmission...";

    // ===== 构造函数 =====
    AppState();
    ~AppState();
};
```

- [ ] **Step 2: 编写 AppState.cpp**

```cpp
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
```

- [ ] **Step 3: 声明全局实例**

在 AppState.cpp 末尾添加：
```cpp
AppState appState;  // 全局单例
```

在 AppState.h 末尾添加：
```cpp
extern AppState appState;
```

---

### Task 6: SenderQueue — 发送队列与发送线程

**Files:**
- Create: `Codes/Touch_Client/core/SenderQueue.h`
- Create: `Codes/Touch_Client/core/SenderQueue.cpp`

**Interfaces:**
- Consumes: `appState` from AppState, `Config::*`, `Vec3` from CoordinateTransform
- Produces:
  - `void startSenderThread()` — 启动发送线程
  - `void stopSenderThread()` — 停止发送线程
  - `DWORD WINAPI senderThreadProc(LPVOID)` — 线程函数，从队列取数据 → 调用 sendCoordinate（在 RobotController 中定义）

- [ ] **Step 1: 编写 SenderQueue.h**

```cpp
#pragma once
#include <windows.h>

void startSenderThread();
void stopSenderThread();
DWORD WINAPI senderThreadProc(LPVOID param);
```

- [ ] **Step 2: 编写 SenderQueue.cpp**

```cpp
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
```

---

### Task 7: RelayProtocol — 中继站协议封装

**Files:**
- Create: `Codes/Touch_Client/network/RelayProtocol.h`
- Create: `Codes/Touch_Client/network/RelayProtocol.cpp`

**Interfaces:**
- Consumes: `Config::*`
- Produces:
  - `std::string buildRelayMessage(int targetPort, const char* cmd)` — 构造 `"端口|指令"` 格式消息
  - `bool parseFeedback(const char* raw, char* outData, int outLen)` — 解析反馈，提取 ErrorID 后的数据部分
  - `bool isSuccessFeedback(const char* raw)` — 判断 ErrorID 是否为 0

- [ ] **Step 1: 编写 RelayProtocol.h**

```cpp
#pragma once
#include <string>

namespace RelayProtocol {
    // 构造发往中继站的消息: "port|command"
    std::string buildMessage(int targetPort, const char* cmd);

    // 检查反馈是否成功 (ErrorID == 0)
    bool isSuccess(const char* feedback);

    // 从原始反馈中提取 {data} 部分
    // 反馈格式: "ErrorID,{data},CommandName();"
    bool extractData(const char* feedback, char* outData, int outLen);
}
```

- [ ] **Step 2: 编写 RelayProtocol.cpp**

```cpp
#include "RelayProtocol.h"
#include "../config/Config.h"
#include <cstdio>
#include <cstring>

namespace RelayProtocol {

std::string buildMessage(int targetPort, const char* cmd) {
    char buffer[512];
    snprintf(buffer, sizeof(buffer), "%d|%s", targetPort, cmd);
    return std::string(buffer);
}

bool isSuccess(const char* feedback) {
    return feedback && feedback[0] == '0';
}

bool extractData(const char* feedback, char* outData, int outLen) {
    if (!feedback || !outData) return false;
    const char* start = strchr(feedback, '{');
    const char* end = strchr(feedback, '}');
    if (!start || !end || start >= end) return false;
    int len = int(end - start - 1);
    if (len <= 0 || len >= outLen) return false;
    strncpy(outData, start + 1, len);
    outData[len] = '\0';
    return true;
}

} // namespace RelayProtocol
```

---

### Task 8: PongHandler — PING/PONG 时延测量

**Files:**
- Create: `Codes/Touch_Client/network/PongHandler.h`
- Create: `Codes/Touch_Client/network/PongHandler.cpp`

**Interfaces:**
- Consumes: `Config::PING_PREFIX`, `Config::PONG_PREFIX`
- Produces:
  - `bool isPingMessage(const char* msg)` — 判断是否为 PING 探测包
  - `std::string buildPongReply(const char* pingMsg)` — 构造 PONG 回复

- [ ] **Step 1: 编写 PongHandler.h**

```cpp
#pragma once
#include <string>

namespace PongHandler {
    bool isPing(const char* msg);
    std::string buildPong(const char* pingMsg);
}
```

- [ ] **Step 2: 编写 PongHandler.cpp**

```cpp
#include "PongHandler.h"
#include "../config/Config.h"
#include <cstring>
#include <cstdio>

namespace PongHandler {

bool isPing(const char* msg) {
    return msg && strncmp(msg, Config::PING_PREFIX, 5) == 0;
}

std::string buildPong(const char* pingMsg) {
    // pingMsg 格式: "PING|1234"
    const char* seq = pingMsg + 5; // 跳过 "PING|"
    char buf[128];
    snprintf(buf, sizeof(buf), "PONG|%s", seq);
    return std::string(buf);
}

} // namespace PongHandler
```

---

### Task 9: CommandBuilder — 机械臂指令构造器

**Files:**
- Create: `Codes/Touch_Client/robot/CommandBuilder.h`

**Interfaces:**
- Consumes: `Config::*`, `Vec3`
- Produces:
  - `std::string buildServoP(const Vec3& pos, double rx, double ry, double rz)` — 构造 ServoP 指令
  - `std::string buildEnableRobot()` — 构造 EnableRobot 指令
  - `std::string buildDisableRobot()` — 构造 DisableRobot 指令
  - `std::string buildClearError()` — 构造 ClearError 指令
  - `std::string buildRobotMode()` — 构造 RobotMode 查询指令
  - `std::string buildGetPose()` — 构造 GetPose 查询指令
  - `std::string buildCP(unsigned int ratio)` — 构造 CP 平滑指令

- [ ] **Step 1: 编写 CommandBuilder.h**

```cpp
#pragma once
#include <string>
#include "../core/CoordinateTransform.h"

namespace CommandBuilder {
    inline std::string buildServoP(const Vec3& pos, double rx, double ry, double rz) {
        char buf[256];
        snprintf(buf, sizeof(buf), "ServoP(%.2f,%.2f,%.2f,%.2f,%.2f,%.2f)",
            pos.x, pos.y, pos.z, rx, ry, rz);
        return std::string(buf);
    }

    inline std::string buildEnableRobot() {
        return "EnableRobot(0.5,0,0,0)";
    }

    inline std::string buildDisableRobot() {
        return "DisableRobot()";
    }

    inline std::string buildClearError() {
        return "ClearError()";
    }

    inline std::string buildRobotMode() {
        return "RobotMode()";
    }

    inline std::string buildGetPose() {
        return "GetPose()";
    }

    inline std::string buildCP(unsigned int ratio) {
        char buf[64];
        snprintf(buf, sizeof(buf), "CP(%u)", ratio);
        return std::string(buf);
    }
}
```

---

### Task 10: SafetyBoundary — 安全边界检查

**Files:**
- Create: `Codes/Touch_Client/robot/SafetyBoundary.h`

**Interfaces:**
- Consumes: `Config::SAFE_*`, `Vec3`
- Produces:
  - `Vec3 clampToBoundary(const Vec3& target)` — 将目标钳位到安全边界内
  - `double computeSpeedFactor(const Vec3& target)` — 根据距边界距离计算速度衰减系数

- [ ] **Step 1: 编写 SafetyBoundary.h**

```cpp
#pragma once
#include "../config/Config.h"
#include "../core/CoordinateTransform.h"
#include "../utils/MathUtils.h"
#include <algorithm>
#include <iostream>

namespace SafetyBoundary {

inline Vec3 clampToBoundary(const Vec3& target) {
    Vec3 clamped = target;
    bool warned = false;

    if (target.x < Config::SAFE_X_MIN) { clamped.x = Config::SAFE_X_MIN; warned = true; }
    if (target.x > Config::SAFE_X_MAX) { clamped.x = Config::SAFE_X_MAX; warned = true; }
    if (target.y < Config::SAFE_Y_MIN) { clamped.y = Config::SAFE_Y_MIN; warned = true; }
    if (target.y > Config::SAFE_Y_MAX) { clamped.y = Config::SAFE_Y_MAX; warned = true; }
    if (target.z < Config::SAFE_Z_MIN) { clamped.z = Config::SAFE_Z_MIN; warned = true; }
    if (target.z > Config::SAFE_Z_MAX) { clamped.z = Config::SAFE_Z_MAX; warned = true; }

    if (warned) {
        std::cerr << "[Safety] 目标超出安全边界，已钳位。原目标: ("
                  << target.x << "," << target.y << "," << target.z << ")" << std::endl;
    }
    return clamped;
}

inline double computeSpeedFactor(const Vec3& target) {
    // 计算距最近边界的距离（归一化到 [0, 1]）
    double rangeX = Config::SAFE_X_MAX - Config::SAFE_X_MIN;
    double rangeY = Config::SAFE_Y_MAX - Config::SAFE_Y_MIN;
    double rangeZ = Config::SAFE_Z_MAX - Config::SAFE_Z_MIN;

    double distX = std::min(target.x - Config::SAFE_X_MIN, Config::SAFE_X_MAX - target.x) / (rangeX * 0.5);
    double distY = std::min(target.y - Config::SAFE_Y_MIN, Config::SAFE_Y_MAX - target.y) / (rangeY * 0.5);
    double distZ = std::min(target.z - Config::SAFE_Z_MIN, Config::SAFE_Z_MAX - target.z) / (rangeZ * 0.5);
    double minDist = std::min({ distX, distY, distZ });

    if (minDist >= Config::SAFE_BOUNDARY_BUFFER_RATIO) return 1.0;
    if (minDist <= 0.0) return 0.1; // 已钳位到边界，最低 10% 速度
    return minDist / Config::SAFE_BOUNDARY_BUFFER_RATIO; // 线性衰减
}

} // namespace SafetyBoundary
```

---

### Task 11: TcpClient — TCP 连接管理

**Files:**
- Create: `Codes/Touch_Client/network/TcpClient.h`
- Create: `Codes/Touch_Client/network/TcpClient.cpp`

**Interfaces:**
- Consumes: `appState`, `Config::*`, `RelayProtocol`, `PongHandler`
- Produces:
  - `bool connectToRelay()` — 连接中继站
  - `void disconnectRelay()` — 断开连接
  - `bool sendToRelay(int targetPort, const char* cmd)` — 发送指令（异步，不等待反馈）
  - `bool readFeedback(char* outBuf, int outLen, DWORD timeoutMs)` — 从缓冲区读反馈
  - `DWORD WINAPI tcpClientThreadProc(LPVOID)` — TCP 事件循环线程

- [ ] **Step 1: 编写 TcpClient.h**

```cpp
#pragma once
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>

bool connectToRelay();
void disconnectRelay();
bool sendToRelay(int targetPort, const char* cmd);
bool readFeedback(char* outBuf, int outLen, DWORD timeoutMs);
DWORD WINAPI tcpClientThreadProc(LPVOID param);
void startTcpClient();
void stopTcpClient();
```

- [ ] **Step 2: 编写 TcpClient.cpp**

```cpp
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
```

---

### Task 12: RobotController — 机械臂控制入口

**Files:**
- Create: `Codes/Touch_Client/robot/RobotController.h`
- Create: `Codes/Touch_Client/robot/RobotController.cpp`

**Interfaces:**
- Consumes: `appState`, `Config::*`, `CommandBuilder`, `SafetyBoundary`, `TcpClient::sendToRelay/readFeedback`, `CoordinateTransform`
- Produces:
  - `bool sendCoordinate(double deltaX, double deltaY, double deltaZ)` — 核心：计算目标 → 安全钳位 → ServoP → 发送
  - `bool initRobot()` — 初始化序列：ClearAlarm → Enable → CP → GetPose
  - `bool shutdownRobot()` — 关闭序列：DisableRobot
  - `void checkAlarmPeriodically()` — 定时巡检报警状态
  - `bool clearAlarm()` — 清除报警并重新使能

- [ ] **Step 1: 编写 RobotController.h**

```cpp
#pragma once

bool sendCoordinate(double deltaX, double deltaY, double deltaZ);
bool initRobot();
void shutdownRobot();
void checkAlarmPeriodically();
bool clearAlarm();
```

- [ ] **Step 2: 编写 RobotController.cpp**

```cpp
#include "RobotController.h"
#include "CommandBuilder.h"
#include "SafetyBoundary.h"
#include "../network/TcpClient.h"
#include "../core/AppState.h"
#include "../core/CoordinateTransform.h"
#include "../config/Config.h"
#include <iostream>
#include <cstring>
#include <string>

// ===== 核心：发送单条坐标指令（热路径，无阻塞） =====
bool sendCoordinate(double deltaX, double deltaY, double deltaZ) {
    auto& app = appState;

    if (app.isClosing || !app.isTcpConnected || !app.isRobotBaseSet) {
        return false;
    }

    // 1. 计算绝对目标位置
    Vec3 delta(deltaX, deltaY, deltaZ);
    Vec3 target = computeTarget(app.robotBase, delta);

    // 2. 安全边界钳位
    target = SafetyBoundary::clampToBoundary(target);

    // 3. 构造 ServoP 指令（固定姿态）
    std::string cmd = CommandBuilder::buildServoP(target, app.robotBaseRx, app.robotBaseRy, app.robotBaseRz);

    // 4. 异步发送（不等待反馈）
    return sendToRelay(Config::MOTION_PORT, cmd.c_str());
}

// ===== 初始化机械臂 =====
bool initRobot() {
    // 清除报警
    if (!sendToRelay(Config::ENABLE_PORT, CommandBuilder::buildClearError().c_str())) {
        std::cerr << "清除报警失败" << std::endl;
        return false;
    }
    Sleep(300);

    // 上使能
    if (!sendToRelay(Config::ENABLE_PORT, CommandBuilder::buildEnableRobot().c_str())) {
        std::cerr << "使能失败" << std::endl;
        return false;
    }
    std::cout << "机械臂使能成功" << std::endl;

    // 设置平滑过渡
    std::string cpCmd = CommandBuilder::buildCP(Config::CP_SMOOTH_RATIO);
    if (!sendToRelay(Config::ENABLE_PORT, cpCmd.c_str())) {
        std::cerr << "设置 CP 失败" << std::endl;
        return false;
    }

    // 获取基准位置
    if (!sendToRelay(Config::ENABLE_PORT, CommandBuilder::buildGetPose().c_str())) {
        std::cerr << "获取基准位置失败" << std::endl;
        return false;
    }

    char feedback[1024];
    if (readFeedback(feedback, sizeof(feedback), Config::FEEDBACK_TIMEOUT)) {
        char* s = strchr(feedback, '{');
        char* e = strchr(feedback, '}');
        if (s && e && s < e) {
            *e = '\0';
            if (sscanf_s(s + 1, "%lf,%lf,%lf,%lf,%lf,%lf",
                &appState.robotBase.x, &appState.robotBase.y, &appState.robotBase.z,
                &appState.robotBaseRx, &appState.robotBaseRy, &appState.robotBaseRz) == 6) {
                appState.isRobotBaseSet = true;
                std::cout << "机械臂基准位置: (" << appState.robotBase.x << ","
                          << appState.robotBase.y << "," << appState.robotBase.z << ")" << std::endl;
                return true;
            }
        }
    }
    std::cerr << "解析基准位置失败" << std::endl;
    return false;
}

// ===== 关闭机械臂 =====
void shutdownRobot() {
    sendToRelay(Config::ENABLE_PORT, CommandBuilder::buildDisableRobot().c_str());
    Sleep(100);
}

// ===== 报警巡检 =====
void checkAlarmPeriodically() {
    auto& app = appState;
    if (!app.isTcpConnected) return;

    sendToRelay(Config::ENABLE_PORT, CommandBuilder::buildRobotMode().c_str());

    char feedback[1024];
    if (readFeedback(feedback, sizeof(feedback), 1000)) {
        char* s = strchr(feedback, '{');
        if (s) {
            int mode = atoi(s + 1);
            bool wasAlarm = app.isRobotInAlarm.exchange(mode == 9);
            if (mode == 9 && !wasAlarm) {
                std::cout << "[Alarm] 检测到机械臂报警 (mode=9)" << std::endl;
            }
        }
    }
}

// ===== 清除报警 =====
bool clearAlarm() {
    if (!sendToRelay(Config::ENABLE_PORT, CommandBuilder::buildClearError().c_str())) {
        return false;
    }
    Sleep(300);
    // 重新使能
    return sendToRelay(Config::ENABLE_PORT, CommandBuilder::buildEnableRobot().c_str());
}
```

---

### Task 13: HapticDevice — Touch 设备管理

**Files:**
- Create: `Codes/Touch_Client/haptic/HapticDevice.h`
- Create: `Codes/Touch_Client/haptic/HapticDevice.cpp`

**Interfaces:**
- Consumes: `appState`
- Produces:
  - `bool initHapticDevice()` — 初始化 Touch，注册回调，启动调度器
  - `void cleanupHapticDevice()` — 停止调度器，关闭设备

- [ ] **Step 1: 编写 HapticDevice.h**

```cpp
#pragma once

bool initHapticDevice();
void cleanupHapticDevice();
```

- [ ] **Step 2: 编写 HapticDevice.cpp**

```cpp
#include "HapticDevice.h"
#include "../core/AppState.h"
#include <iostream>
#include <HD/hd.h>

// 前向声明：HapticCallback 在 HapticCallback.cpp 中实现
HDCallbackCode HDCALLBACK hapticCallback(void* pUserData);

bool initHapticDevice() {
    auto& app = appState;

    app.hHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(hdGetError())) {
        std::cerr << "Touch 设备初始化失败" << std::endl;
        return false;
    }

    hdEnable(HD_FORCE_OUTPUT);
    app.hapticCallbackHandle = hdScheduleAsynchronous(
        hapticCallback, nullptr, HD_MAX_SCHEDULER_PRIORITY);

    if (HD_DEVICE_ERROR(hdGetError())) {
        std::cerr << "触觉回调注册失败" << std::endl;
        return false;
    }

    hdStartScheduler();
    if (HD_DEVICE_ERROR(hdGetError())) {
        std::cerr << "调度器启动失败" << std::endl;
        return false;
    }

    app.deviceInitialized = true;
    std::cout << "Touch 设备初始化成功" << std::endl;
    return true;
}

void cleanupHapticDevice() {
    auto& app = appState;
    if (app.deviceInitialized) {
        hdUnschedule(app.hapticCallbackHandle);
        hdStopScheduler();
        hdDisableDevice(app.hHD);
        app.hHD = HD_INVALID_HANDLE;
        app.deviceInitialized = false;
    }
}
```

---

### Task 14: HapticCallback — 1kHz 触觉回调

**Files:**
- Create: `Codes/Touch_Client/haptic/HapticCallback.h`
- Create: `Codes/Touch_Client/haptic/HapticCallback.cpp`

**Interfaces:**
- Consumes: `appState`, `Config::*`, `CoordinateTransform`, `Vec3`, `MathUtils::clamp`
- Produces: `HDCallbackCode HDCALLBACK hapticCallback(void*)` — 1kHz 回调，读取位置/按钮/姿态（预留），坐标转换，入队

- [ ] **Step 1: 编写 HapticCallback.h**

```cpp
#pragma once
#include <HD/hd.h>

HDCallbackCode HDCALLBACK hapticCallback(void* pUserData);
```

- [ ] **Step 2: 编写 HapticCallback.cpp**

```cpp
#include "HapticCallback.h"
#include "../core/AppState.h"
#include "../core/CoordinateTransform.h"
#include "../core/SenderQueue.h"
#include "../config/Config.h"
#include "../utils/MathUtils.h"
#include <HDU/hduVector.h>
#include <iostream>

HDCallbackCode HDCALLBACK hapticCallback(void* pUserData) {
    auto& app = appState;
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

    // ===== 2. 坐标转换 =====
    Vec3 robotPos = convertTouchToRobot(localDevicePos);

    EnterCriticalSection(&app.adjustedPosTableMutex);
    app.adjustedPosTable = robotPos;
    LeaveCriticalSection(&app.adjustedPosTableMutex);

    EnterCriticalSection(&app.adjustedPosMutex);
    app.adjustedPos = localDevicePos;
    hduVector3Dd localAdjustedPos = app.adjustedPos;
    LeaveCriticalSection(&app.adjustedPosMutex);

    // ===== 3. 预留：读取姿态 (HD_CURRENT_TRANSFORM) =====
    // hdGetDoublev(HD_CURRENT_TRANSFORM, app.transformMatrix);
    // 后续迭代：从 transformMatrix 提取 Rx/Ry/Rz 并增量映射

    // ===== 4. 轨迹记录 =====
    EnterCriticalSection(&app.trailMutex);
    app.trailPoints.push_back(localAdjustedPos);
    while ((int)app.trailPoints.size() > Config::MAX_TRAIL) {
        app.trailPoints.pop_front();
    }
    LeaveCriticalSection(&app.trailMutex);

    // ===== 5. 读取按钮状态 =====
    int buttonState = 0;
    hdGetIntegerv(HD_CURRENT_BUTTONS, &buttonState);
    bool button1 = (buttonState & HD_DEVICE_BUTTON_1) != 0;
    bool button2 = (buttonState & HD_DEVICE_BUTTON_2) != 0; // 预留
    app.button2Pressed = button2;

    // ===== 6. 按钮 1：运动控制 =====
    static hduVector3Dd startPos;
    bool stateChanged = (button1 != app.lastButtonState);

    if (stateChanged) {
        EnterCriticalSection(&app.basePointMutex);

        app.isTransmitting = button1;
        app.lastButtonState = button1;

        if (button1) {
            // 按下：记录基准点
            app.basePoint = robotPos;
            app.isBasePointSet = true;
            startPos = localDevicePos;
        } else {
            // 松开：停止发送
            app.isBasePointSet = false;
        }

        LeaveCriticalSection(&app.basePointMutex);

        // 更新 TCP 状态显示
        EnterCriticalSection(&app.statusMutex);
        if (button1) {
            snprintf(app.transmissionState, sizeof(app.transmissionState), "STATE: transmitting");
        } else {
            snprintf(app.transmissionState, sizeof(app.transmissionState), "STATE: waiting");
        }
        LeaveCriticalSection(&app.statusMutex);
    }

    // ===== 7. 持续发送（按钮保持按下） =====
    if (app.isTransmitting && app.isBasePointSet && app.isTcpConnected) {
        Vec3 base;
        EnterCriticalSection(&app.basePointMutex);
        base = app.basePoint;
        LeaveCriticalSection(&app.basePointMutex);

        Vec3 delta = computeDelta(robotPos, base);

        // 阈值过滤
        if (delta.length() >= Config::MIN_DELTA_THRESHOLD) {
            // 入队（带背压控制）
            {
                std::lock_guard<std::mutex> lock(app.queueMutex);
                while ((int)app.sendQueue.size() >= Config::MAX_QUEUE_SIZE) {
                    app.sendQueue.pop(); // 丢弃最旧数据
                }
                app.sendQueue.push({ delta.x, delta.y, delta.z });
            }
            app.queueCV.notify_one();
        }
    }

    hdEndFrame(app.hHD);
    return HD_CALLBACK_CONTINUE;
}
```

---

### Task 15: 渲染模块 (RenderUtils + LogoManager + SceneRenderer + CoordinateTable + StatusDisplay)

**Files:**
- Create: `Codes/Touch_Client/render/RenderUtils.h`
- Create: `Codes/Touch_Client/render/LogoManager.h`
- Create: `Codes/Touch_Client/render/SceneRenderer.h`
- Create: `Codes/Touch_Client/render/CoordinateTable.h`
- Create: `Codes/Touch_Client/render/StatusDisplay.h`

**Interfaces:**
- Consumes: `appState`, `Config::*`, OpenGL/freeglut
- Produces: 各渲染类的静态 draw 方法

由于渲染模块代码量较大（~700 行）且逻辑与 test8.3 基本一致（结构拆分，行为不改），此处列出接口和关键适配点：

- [ ] **Step 1: 编写 RenderUtils.h**

```cpp
#pragma once
#include <GL/freeglut.h>
#include "../config/Config.h"

namespace RenderUtils {
    inline void switchTo2D() {
        glPushAttrib(GL_PROJECTION_BIT | GL_MODELVIEW_BIT);
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluOrtho2D(0, Config::WINDOW_W, 0, Config::WINDOW_H);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
    }
    inline void switchTo3D() {
        glPopAttrib();
        glMatrixMode(GL_MODELVIEW);
    }
    void drawText(int x, int y, const char* text, const float* color, void* font);
    int getTextWidth(const char* text, void* font);
    void drawRectWithBorder(int left, int top, int w, int h,
                            const float* bg, const float* border, float borderW);
}
```

实现从 test8.3 的 `RenderUtils` 类直接迁移，去掉类包装改为命名空间函数。

- [ ] **Step 2: 编写 LogoManager.h**

从 test8.3 的 `LogoManager` 类迁移：
```cpp
#pragma once
#include <GL/freeglut.h>
namespace LogoManager {
    bool loadLogo(GLuint& textureID);
    void drawLogo(GLuint textureID);
}
```

- [ ] **Step 3: 编写 SceneRenderer.h**

从 test8.3 的 `SceneRenderer` 命名空间迁移：
```cpp
#pragma once
namespace SceneRenderer {
    void drawFloor();
    void drawCubeBorders();
    void draw3DAxis();
    void drawCursor();
    void drawTrail();
}
```

- [ ] **Step 4: 编写 CoordinateTable.h**

从 test8.3 的 `CoordinateTable` 类迁移：
```cpp
#pragma once
namespace CoordinateTable {
    void draw();
}
```

需适配：读取坐标从 `appState.adjustedPosTable`（Vec3 类型）替代原来的 `hduVector3Dd`。

- [ ] **Step 5: 编写 StatusDisplay.h**

从 test8.3 的 `StatusDisplay` 类迁移：
```cpp
#pragma once
namespace StatusDisplay {
    void draw();
}
```

---

### Task 16: main.cpp — 入口与 GLUT 主循环

**Files:**
- Create: `Codes/Touch_Client/main.cpp`

**Interfaces:**
- Consumes: 所有模块
- Produces: `int main(int argc, char* argv[])` — 组装所有模块，启动 GLUT 主循环

- [ ] **Step 1: 编写 main.cpp**

```cpp
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#include <GL/freeglut.h>
#include <HD/hd.h>
#include <HDU/hduVector.h>
#include <iostream>
#define STB_IMAGE_IMPLEMENTATION
#include "vendor/stb_image.h"

#include "config/Config.h"
#include "core/AppState.h"
#include "core/SenderQueue.h"
#include "haptic/HapticDevice.h"
#include "network/TcpClient.h"
#include "robot/RobotController.h"
#include "render/RenderUtils.h"
#include "render/SceneRenderer.h"
#include "render/CoordinateTable.h"
#include "render/StatusDisplay.h"
#include "render/LogoManager.h"
#include "utils/MathUtils.h"

// ===== GLUT 回调 =====
namespace Callbacks {
    void display() {
        if (appState.isClosing) return;

        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LEQUAL);

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluPerspective(Config::FOV, (double)Config::WINDOW_W / Config::WINDOW_H,
                       Config::NEAR_CLIP, Config::FAR_CLIP);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        double camZ = Config::BASE_CAM_Z * appState.camDistance;
        gluLookAt(Config::BASE_CAM_X, Config::BASE_CAM_Y, camZ,
                  Config::CENTER_X, Config::CENTER_Y, Config::CENTER_Z,
                  0.0, 1.0, 0.0);

        glRotatef((float)appState.rotateX, 1.0f, 0.0f, 0.0f);
        glRotatef((float)appState.rotateY, 0.0f, 1.0f, 0.0f);

        SceneRenderer::drawFloor();
        SceneRenderer::drawCubeBorders();
        SceneRenderer::draw3DAxis();
        SceneRenderer::drawTrail();
        SceneRenderer::drawCursor();
        StatusDisplay::draw();
        if (appState.logoLoaded) LogoManager::drawLogo(appState.logoTextureID);
        CoordinateTable::draw();

        glutSwapBuffers();
    }

    void idle() {
        if (!appState.isClosing) { glutPostRedisplay(); Sleep(1); }
    }

    void reshape(int w, int h) {
        glViewport(0, 0, w, h);
    }

    void mouse(int button, int state, int x, int y) {
        if (button == GLUT_LEFT_BUTTON) {
            appState.isDragging = (state == GLUT_DOWN);
            if (appState.isDragging) {
                appState.lastMouseX = x;
                appState.lastMouseY = y;
            }
        }
        else if ((button == 3 || button == 4) && state == GLUT_DOWN) { // 滚轮
            appState.camDistance *= (button == 3) ? (1.0f / Config::ZOOM_STEP) : Config::ZOOM_STEP;
            appState.camDistance = clamp(appState.camDistance, Config::MIN_ZOOM, Config::MAX_ZOOM);
        }
    }

    void motion(int x, int y) {
        if (!appState.isDragging) return;
        appState.rotateY += (x - appState.lastMouseX) * Config::ROTATION_SPEED;
        appState.rotateX -= (y - appState.lastMouseY) * Config::ROTATION_SPEED;
        appState.rotateX = clamp(appState.rotateX, -45.0, 60.0);
        appState.lastMouseX = x;
        appState.lastMouseY = y;
    }
}

// ===== 报警巡检定时器 =====
void alarmCheckTimer(int value) {
    checkAlarmPeriodically();
    if (!appState.isClosing) {
        glutTimerFunc(Config::ALARM_CHECK_INTERVAL, alarmCheckTimer, 0);
    }
}

// ===== 更新 TCP 状态显示 =====
void updateTcpStatus() {
    auto& app = appState;
    EnterCriticalSection(&app.statusMutex);
    if (app.isTcpConnected) {
        snprintf(app.connectionStatus, sizeof(app.connectionStatus), "TCP: connected to relay");
        snprintf(app.serverInfo, sizeof(app.serverInfo), "IP: %s", Config::TCP_RELAY_IP);
        snprintf(app.transmissionState, sizeof(app.transmissionState),
                 app.isTransmitting ? "STATE: transmitting" : "STATE: waiting");
    } else {
        snprintf(app.connectionStatus, sizeof(app.connectionStatus), "TCP: not connected");
        snprintf(app.serverInfo, sizeof(app.serverInfo), "IP: -");
        snprintf(app.transmissionState, sizeof(app.transmissionState), "STATE: -");
    }
    LeaveCriticalSection(&app.statusMutex);
}

// ===== 主函数 =====
int main(int argc, char* argv[]) {
    std::cout << "=== Touch-Dobot 远程控制系统 ===" << std::endl;

    // 1. GLUT 初始化
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(Config::WINDOW_W, Config::WINDOW_H);
    glutInitWindowPosition(100, 100);
    if (glutCreateWindow("Touch-Dobot Remote Control") == 0) {
        std::cerr << "窗口创建失败" << std::endl;
        return -1;
    }

    glutDisplayFunc(Callbacks::display);
    glutIdleFunc(Callbacks::idle);
    glutReshapeFunc(Callbacks::reshape);
    glutMouseFunc(Callbacks::mouse);
    glutMotionFunc(Callbacks::motion);

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glShadeModel(GL_SMOOTH);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // 2. 加载 Logo
    appState.logoLoaded = LogoManager::loadLogo(appState.logoTextureID);

    // 3. 初始化 Touch
    std::cout << "初始化 Touch 设备..." << std::endl;
    if (!initHapticDevice()) {
        std::cerr << "Touch 设备初始化失败，程序退出" << std::endl;
        return -1;
    }

    // 4. 启动 TCP 客户端
    std::cout << "连接中转站..." << std::endl;
    startTcpClient();
    Sleep(1000); // 等待连接建立

    if (!appState.isTcpConnected) {
        std::cerr << "中转站连接失败，程序退出" << std::endl;
        cleanupHapticDevice();
        return -1;
    }

    // 5. 机械臂初始化
    std::cout << "初始化机械臂..." << std::endl;
    if (!initRobot()) {
        std::cerr << "机械臂初始化失败，程序退出" << std::endl;
        shutdownRobot();
        stopTcpClient();
        cleanupHapticDevice();
        return -1;
    }

    // 6. 启动发送线程
    startSenderThread();

    // 7. 启动报警巡检定时器
    glutTimerFunc(Config::ALARM_CHECK_INTERVAL, alarmCheckTimer, 0);

    // 8. 输出操作说明
    std::cout << "\n操作说明:" << std::endl;
    std::cout << "  触控笔按钮1: 按住控制机械臂运动，松开停止" << std::endl;
    std::cout << "  鼠标左键拖动: 旋转视角" << std::endl;
    std::cout << "  鼠标滚轮: 缩放视图" << std::endl;
    std::cout << "  安全边界: X[" << Config::SAFE_X_MIN << "," << Config::SAFE_X_MAX
              << "] Y[" << Config::SAFE_Y_MIN << "," << Config::SAFE_Y_MAX
              << "] Z[" << Config::SAFE_Z_MIN << "," << Config::SAFE_Z_MAX << "]" << std::endl;

    // 9. 进入主循环
    std::cout << "\n系统就绪，进入主循环" << std::endl;
    glutMainLoop();

    // 10. 清理
    std::cout << "正在关闭..." << std::endl;
    stopSenderThread();
    shutdownRobot();
    stopTcpClient();
    cleanupHapticDevice();

    std::cout << "程序正常退出" << std::endl;
    return 0;
}
```

---

### Task 17: 通信协议文档

**Files:**
- Create: `Docs/protocol/Touch-Relay-Protocol.md`

- [ ] **Step 1: 编写协议文档**

```markdown
# Touch ↔ 中继站 通信协议规范 v1.0

## 概述

本协议定义 PC-A（Touch 端 C++ 程序）与 PC-B（中继站 MATLAB 程序）之间的 TCP 通信格式。
中继站负责将 Touch 端指令转发至 Dobot 机械臂对应端口，并将机械臂反馈透传回 Touch 端。

## 连接参数

| 参数 | 值 |
|------|-----|
| 传输协议 | TCP |
| 中继站 IP | 192.168.101.25 (可配置) |
| 端口 | 8888 |
| 编码 | ASCII 文本 |

## 消息格式

### Touch → 中继站

格式: `{目标端口}|{机械臂指令}`

| 场景 | 目标端口 | 示例 |
|------|---------|------|
| 运动控制 | 30003 | `30003\|ServoP(200.00,0.00,150.00,0.00,0.00,0.00)` |
| 使能控制 | 29999 | `29999\|EnableRobot(0.5,0,0,0)` |
| 下使能 | 29999 | `29999\|DisableRobot()` |
| 清除报警 | 29999 | `29999\|ClearError()` |
| 状态查询 | 29999 | `29999\|RobotMode()` |
| 位置查询 | 29999 | `29999\|GetPose()` |
| 平滑设置 | 29999 | `29999\|CP(100)` |

### 中继站 → Touch

透传机械臂原始反馈，格式由 Dobot TCP/IP 协议定义：

- 成功: `0,{data},CommandName();`
- 失败: `ErrorID,{data},CommandName();` (ErrorID ≠ 0)

### 时延探测

| 方向 | 格式 | 示例 |
|------|------|------|
| 中继站 → Touch | `PING\|{序号}` | `PING\|1234` |
| Touch → 中继站 | `PONG\|{序号}` | `PONG\|1234` |

### 异常通知

| 消息 | 含义 |
|------|------|
| `ROBOT_ARM_CLOSED` | 中继站与机械臂断开，中继站仍在线 |

## 机械臂指令参考

使用 Dobot TCP/IP 远程控制接口 V3 指令集：
- Dashboard 指令 (29999): EnableRobot, DisableRobot, ClearError, RobotMode, GetPose, CP
- 运动指令 (30003): ServoP — 笛卡尔空间动态跟随，直接覆盖执行，不排队
```

---

### Task 18: 最终验证与收尾

- [ ] **Step 1: 确认文件完整性**

```bash
find Codes/Touch_Client -name "*.h" -o -name "*.cpp" | sort
```

预期输出（19 个文件）：
```
Codes/Touch_Client/config/Config.h
Codes/Touch_Client/core/AppState.h
Codes/Touch_Client/core/AppState.cpp
Codes/Touch_Client/core/CoordinateTransform.h
Codes/Touch_Client/core/SenderQueue.h
Codes/Touch_Client/core/SenderQueue.cpp
Codes/Touch_Client/haptic/HapticDevice.h
Codes/Touch_Client/haptic/HapticDevice.cpp
Codes/Touch_Client/haptic/HapticCallback.h
Codes/Touch_Client/haptic/HapticCallback.cpp
Codes/Touch_Client/main.cpp
Codes/Touch_Client/network/PongHandler.h
Codes/Touch_Client/network/PongHandler.cpp
Codes/Touch_Client/network/RelayProtocol.h
Codes/Touch_Client/network/RelayProtocol.cpp
Codes/Touch_Client/network/TcpClient.h
Codes/Touch_Client/network/TcpClient.cpp
Codes/Touch_Client/render/CoordinateTable.h
Codes/Touch_Client/render/LogoManager.h
Codes/Touch_Client/render/RenderUtils.h
Codes/Touch_Client/render/SceneRenderer.h
Codes/Touch_Client/render/StatusDisplay.h
Codes/Touch_Client/robot/CommandBuilder.h
Codes/Touch_Client/robot/RobotController.h
Codes/Touch_Client/robot/RobotController.cpp
Codes/Touch_Client/robot/SafetyBoundary.h
Codes/Touch_Client/utils/MathUtils.h
```

- [ ] **Step 2: 确认旧代码已清理**

```bash
ls Codes/test* 2>&1
# 预期: No such file or directory
```

- [ ] **Step 3: 确认文档已生成**

```bash
ls Docs/protocol/Touch-Relay-Protocol.md
# 预期: 文件存在
```

---
