#pragma once
#include <winsock2.h>
#include <ws2tcpip.h>
#include <vector>
#include <atomic>
#include <windows.h>
#include <HD/hd.h>
#include <HDU/hduVector.h>
#include "CoordinateTransform.h"
#include "IExtension.h"
#include "../safety/RobotStateMachine.h"

class RelayCore {
public:
    static RelayCore& instance();

    bool init();
    void shutdown();

    // Touch → Robot 正向数据流
    void sendPosition(const hduVector3Dd& devicePos);
    void onButtonPress(const Vec3& robotPos);
    void onButtonRelease();
    void onButton2Press(const Vec3& stylusOrient);
    void onButton2Release();

    // Robot → Touch 反向数据流 (每帧调用)
    void pollFeedback();
    void queryPose();
    void queryJointAngles();
    void checkAlarm();

    // 力传感器数据流
    bool initForceReader();
    void pollForce();
    void shutdownForceReader();

    // 力传感器标定
    bool startForceCalibration();
    // 仅调零: 静置采集零偏 → 直接应用+存盘 (不进 MOTION 相、不开拖拽模式)
    bool startForceZeroing();
    void abortForceCalibration();
    bool isForceCalibrating() const;
    bool isForceZeroing() const;
    bool isForceCalibrationDone() const;
    const char* forceCalibStatus() const;

    // 末端负载标定后重新下发 EnableRobot(load,cx,cy,cz)
    bool applyPayloadToRobot();

    // 用给定负载重新使能, 等稳定后返回当前姿态的残余力矩模长 (N·m)。
    // 符号裁决用: 同一静止姿态下依次探两个候选, 谁留下的力矩残余小谁对。
    // 调用方必须保证机械臂【不动】(两次探针在同一姿态比较才有意义)。
    // 读数取 30004 的 forceData.raw[] —— 不是 filtered[]。原因见 RelayCore.cpp 实现里
    // 的说明 (filtered 由主线程更新, 而本函数阻塞的就是主线程)。
    // 【返回值是个模长, 里面含一个姿态常数的传感器零偏】所以调用方判两个候选的【差值】,
    // 不要判胜者的绝对值 —— 理由见 Config.h 的 SIGN_PROBE_MIN_MARGIN_NM。
    // 返回 false 的四种原因: 机械臂未连接 (静默) / 候选下发失败 / 机械臂没采纳这次候选 /
    // 采样窗口内有效数据不足 —— 后三种都会打一行 [Probe] 提示。
    bool probePayloadResidual(double massKg, const double comMm[3], double& residualNm);

    // 奇异脱困 (可在运行中手动触发)
    bool triggerEscape();

    // 手动拖拽模式 (SetCollideDrag) —— 'm' 采集姿态时用 'd' 切换。
    // 拖拽中机械臂是柔顺的, 姿态会漂; 采样前必须关掉, 否则力数据是脏的。
    bool setDragMode(bool enable);
    bool isDragMode() const { return m_dragMode; }

    // 扩展
    void registerExtension(IExtension* ext);

    // MATLAB GUI 上报
    void initRelayReporting();
    void shutdownRelayReporting();
    int  sendRelayUpdate(const char* msg);
    void reportPosition();
    void reportCommand(const char* cmd);
    void reportFeedback(const char* fbText);

    void sendSafetyStatus();
    void sendJointMargins();
    void sendSingularity();
    void sendCalibStatus();
    void sendConnectionHealth();
    void reportDiagnostic(int errorCode, double speedFactor, const char* reason);

    // MATLAB → C++ 反向命令 (每帧调用, 非阻塞)
    void pollRelayCommands();

    // 状态查询（供 Render 层读取）
    bool isTransmitting() const { return m_transmitting; }

    // 看门狗状态查询
    DWORD lastHapticFrameMs() const { return m_lastHapticFrameMs.load(); }
    void checkHapticWatchdog();

    // 状态机 (供 HUD / 外部读取)
    RobotStateMachine& stateMachine() { return m_stateMachine; }
    const RobotStateMachine& stateMachine() const { return m_stateMachine; }

    // 心跳刷新（在所有启动初始化完成后调用，防止误判超时）
    void resetHeartbeat() { m_lastHeartbeatMs = GetTickCount(); m_heartbeatStartMs = GetTickCount(); }

    // PING/PONG 延迟测量
    void pingRobot();

private:
    RelayCore();
    ~RelayCore();
    RelayCore(const RelayCore&) = delete;
    RelayCore& operator=(const RelayCore&) = delete;

    std::atomic<bool> m_transmitting{false};
    std::atomic<bool> m_basePointSet{false};
    std::atomic<bool> m_dragMode{false};   // 手动/标定拖拽模式是否开着
    Vec3 m_targetPos;           // 累加式机器人目标位置
    Vec3 m_lastTouchPos;        // 上一帧 Touch 位置 (robot系), 用于增量计算
    bool   m_lastTouchValid = false;

    // ===== 姿态控制 (Button 2) =====
    // NOTE: These are accessed only from the haptic callback thread (1kHz).
    // m_basePointLock is used in onButton2Press for consistency but no cross-thread contention exists.
    Vec3 m_targetOrient;          // 累加式姿态目标 (Rx, Ry, Rz in degrees)
    Vec3 m_lastStylusOrient;      // 上一帧笔杆姿态, 用于增量计算
    Vec3 m_orientRefStylus;       // 按下瞬间的笔杆参考姿态
    Vec3 m_orientRefRobot;        // 按下瞬间的末端参考姿态
    bool  m_orientValid = false;
    bool  m_transmittingOrient = false;

    CRITICAL_SECTION m_basePointLock;
    std::vector<IExtension*> m_extensions;

    // MATLAB relay connection
    SOCKET m_relaySocket = INVALID_SOCKET;
    CRITICAL_SECTION m_relaySocketMutex;

    // 反向命令接收缓冲 (行式协议, 逐行切分)
    char m_relayRecvBuf[256];
    int  m_relayRecvLen = 0;
    void dispatchRelayCommand(const char* line);

    DWORD m_lastRelayUpdate = 0;
    DWORD m_lastServoTime = 0;      // ServoP 发送频率控制
    HANDLE m_forceThread = NULL;

    RobotStateMachine m_stateMachine;
    DWORD m_lastPingMs = 0;
    DWORD m_lastHeartbeatMs = 0;
    DWORD m_heartbeatStartMs = 0;    // 心跳检查开始时间 (宽限期后开启)
    DWORD m_processStartMs = 0;      // 进程启动时间戳 (用于计算 uptime)
    bool m_heartbeatLostReported = false;
    int m_nanFrameCount = 0;  // 连续 NaN 帧计数 (>=3 → FATAL)

    // 看门狗
    std::atomic<DWORD> m_lastHapticFrameMs{0};
    bool m_watchdogTripped = false;
};
