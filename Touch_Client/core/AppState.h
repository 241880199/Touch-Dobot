#pragma once
#include <winsock2.h>
#include <ws2tcpip.h>
#include <HD/hd.h>
#include <HDU/hduVector.h>
#include <atomic>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <vector>
#include "../relay/CoordinateTransform.h"

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

    // ===== 交互与渲染状态（UI 后续通过 open-design MCP 实现） =====
    double rotateX = 15.0, rotateY = 10.0;
    float camDistance = 1.0f;
    int lastMouseX = 0, lastMouseY = 0;
    bool isDragging = false;

    // ===== 状态显示字符串 =====
    char transmissionState[128] = "STATE: -";
    char lastTransmissionDetail[256] = "Waiting for transmission...";

    // ===== 构造函数 =====
    AppState();
    ~AppState();
};

extern AppState appState;
