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

    // 奇异脱困 (可在运行中手动触发)
    bool triggerEscape();

    // 手动拖拽模式 (SetCollideDrag) —— 'm' 采集姿态时用 'd' 切换。
    // 拖拽中机械臂是柔顺的, 姿态会漂; 采样前必须关掉, 否则力数据是脏的。
    bool setDragMode(bool enable);
    bool isDragMode() const { return m_dragMode; }

    // 运行时显式把负载参数下发给机械臂 (Task 8a): EnableRobot(m, cx, cy, cz) + LoadSwitch(1)。
    // comMm = 质心 (mm, 法兰系, 三个分量)。
    // 【只由用户显式触发】—— 绝不能被 init() 或任何"重新使能"路径调用 (约束与出处见 .cpp 里
    // sendPayloadCommands 顶上那段)。返回两条都成功才为 true。
    // ⚠ 运行中改负载会让机械臂动 (2026-09-19 实机证实: 1.5 kg 那次撞向关节限位) ——
    //   调用方必须先过两道闸、打安全规程提示, 并【取到操作员的明确确认】(main.cpp 的发送键
    //   'p' 摆出提示之后还要再按确认键才调到这里)。本函数只负责"发"。
    bool sendPayloadToRobot(double massKg, const double comMm[3]);

    // ===== 负载下发的命令文本 (Task 8a-3): 全程序【唯一】的拼法 =====
    // 发送侧 (RelayCore.cpp 的 sendPayloadCommands) 与【确认屏】(main.cpp 的发送键: 那一次
    // 要发什么的预览) 都取自这里。屏幕上摆出来给人确认的两条文本, 就是发送侧要写进 socket
    // 的那两条 —— "确认"确认的正是要发给真实机械臂的东西, 所以它不能是第二处拼法:
    // 格式串或参数顺序一改, 屏幕就会让人确认另一条命令。
    //   · 第 1 条 EnableRobot(m, cx, cy, cz) —— comMm 是质心 (mm, 法兰系)。
    //   · 第 2 条 LoadSwitch(1) —— 无参数 (文本里唯一的变数是那个 0/1)。
    // 出处与顺序见 RelayCore.cpp 里 sendPayloadCommands 顶上那段。
    static void formatPayloadEnableCommand(double massKg, const double comMm[3],
                                           char* out, int n);
    static const char* payloadLoadSwitchCommand();

    // 扩展
    void registerExtension(IExtension* ext);

    // MATLAB GUI 上报
    void initRelayReporting();
    void shutdownRelayReporting();
    int  sendRelayUpdate(const char* msg);
    void reportPosition();

    // 发一条 MATLAB 端的警告 (W| 协议) —— ★【这个线上格式的唯一一份定义】(2026-09-21)。
    //   level      : 1 = 警告, 2 = 严重 (relay_gui 用它给顶栏染色并选显示方式)
    //   type       : 单字符分类
    //   message / suggestion : 文本
    //   ⚠ 这两段文本【都不许含逗号】: 协议是逗号分隔的, 一个逗号会让后面所有字段整体错位。
    //     要分隔就写 '；' 或 '·'。
    //   ⚠ MATLAB 侧【每个刷新周期 (0.05s / 20Hz) 都会把 warn_max_level 清零】⇒ 调用方必须
    //     【重复发】它才会常亮 (见 Relay_Station/relay_gui.m 的 "Decay warnings" 与
    //     "Top-bar state override for warnings")。发一次就只能闪一下。
    void reportWarning(int level, const char* type, const char* message,
                       const char* suggestion, double param1, double param2);
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

    // 只刷新心跳时间戳, 【不】动启动宽限期 —— 供"故意阻塞主线程"的操作(如负载探针)
    // 在阻塞结束后声明自己还活着。
    // 为什么必须这么做: 心跳检查在 pollFeedback() 里, 而 pollFeedback() 和那些操作跑在
    // 同一个 GLUT 线程上 —— 阻塞期间它根本不会跑, 于是恢复后第一帧就撞见过期的心跳,
    // 误报 ERR_HEARTBEAT_LOST(FATAL, 会下使能)。实测: 探针阻塞 ~2s, 必然触发。
    // 不能用 resetHeartbeat(): 它会把启动宽限期一起重置, 每次探针都白送 10s 不检查心跳。
    void touchHeartbeat() { m_lastHeartbeatMs = GetTickCount(); }

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
