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
    void abortForceCalibration();
    bool isForceCalibrating() const;
    bool isForceCalibrationDone() const;
    const char* forceCalibStatus() const;

    // 奇异脱困 (可在运行中手动触发)
    bool triggerEscape();

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
    Vec3 m_wristCenterFixed;      // 按下瞬间的腕部中心 (J5 位置, 固定不动)
    Vec3 m_tcpOffsetFromWrist;    // 按下瞬间 TCP→J5 偏移向量 (robot 系)
    bool  m_orientValid = false;
    bool  m_transmittingOrient = false;

    CRITICAL_SECTION m_basePointLock;
    std::vector<IExtension*> m_extensions;

    // MATLAB relay connection
    SOCKET m_relaySocket = INVALID_SOCKET;
    CRITICAL_SECTION m_relaySocketMutex;
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
