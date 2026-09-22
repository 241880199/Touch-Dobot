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

    // ===== 帧率噪声探针 (2026-09-21) =====
    // 【要回答的问题】原始 @1304 的噪声, 在【帧率】下把连续 k 个样本取平均, sd 掉多少?
    //   掉成 1/sqrt(k) ⇒ 宽带噪声 ⇒ 把力流水线挪到帧率上跑能白赚 √N;
    //   基本不掉       ⇒ 噪声落在更慢的频带上 ⇒ 换修法。判据的统计量在 force/NoiseProbe.h (有单测)。
    // 【为什么非要在这里取】流水线 (ForceCompensation/ForcePipeline) 跑在 pollForce 的 ~11 Hz 上,
    //   而帧以 8 ms 到达 —— 快的那一路只有【本线程】看得见。取慢的那一路来回答"平均有没有用",
    //   等于用被抽样过的数据回答抽样本身的问题。
    // 【每一帧都存, 不是"按键才开始"】省掉一个"忘了按"的状态; 读的是最近 CAJ_N 帧。
    // ⚠ 只读不动: 本探针【不参与】任何补偿/滤波/闸门计算, 加它不改变任何行为。
    struct ForceFrameSample {
        unsigned long long tickUs;   // steady_clock 微秒 (不能用 GetTickCount: 15.6 ms 粒度
                                     // 分辨不出 8 ms 的帧间隔 —— 那正是要量的东西)
        double f[3];                 // @1304 的 Fx,Fy,Fz (原始值, 未补偿)
    };
    // 取最近收到的 ≤maxN 帧, 按【从旧到新】写进 out。返回实际帧数。
    int copyRecentForceFrames(ForceFrameSample* out, int maxN);

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

    // ===== relay socket 的健壮化 (2026-09-21) =====
    // 【背景】这条 socket 从前【只在启动时连一次, 没有重试】, 而且 send 的返回值没人检查
    //   (两次 send 的返回值直接相加, -1 与 +1 互相抵消) ⇒ 连接事后死掉时, 客户端会一直往
    //   死 socket 里写而【完全无声】。现场表现: MATLAB 的孪生停在默认姿势, 从那一头根本
    //   分不出是"没收到数据"还是"显示坏了" —— 孪生的 3D 模型是拿 J|(关节角) 画的, 而
    //   J|/RP|/P| 全走这条 socket。2026-09-21 现场就在这一头绕了很久。
    //   ⇒ 三个入口各管一件事 (定义处有完整说明):
    bool connectRelaySocket();                  // 建+连+装上; 不打印 (启动与重连共用)
    bool ensureRelayConnected();                // 发之前保证连着; socket 无效时按秒重连
    void markRelayDisconnected(const char* why); // 作废 socket; 只在状态真变化时出声一次

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

    // ===== 力反射增益回读 (RG| 协议, 2026-09-22) =====
    // 【唯一真值通道】两个方向都用 RG|: MATLAB 发 RG|<值> 设, C++ 在连接 / 重连时、以及每收到
    //   一条良构的 RG| 之后, 只要【相对上次真的走进 send 的那一条变了】或【被拒】就回一条
    //     RG|<gain>,<min>,<max>,<ratio>,<deadN>,<satN>,<defGain>
    //   ⚠ 【不是】"不论接受还是拒绝都回", 也【不是】按"上一个生效值"判 (2026-09-22 修正)。
    //     条件①比的是【上次真的走进 send 的那一条的值】—— 这个区别有两个方向, 两个方向
    //     都【不是漏发】:
    //       · 【被接受、但值没变】的空操作仍可能回一条: 它被条件② (100ms 窗口) 挡下、
    //         由 pollRelayCommands 的补发送出去 ⇒ 那一条回读照样带着这个值到。挡它的是
    //         ②的【延迟】, 不是① —— 滑条松手时回显同一个值正是这种情形。
    //       · 反过来, 【被接受、值确实变了】也可能【没有】回读: 那一变被②挡进待发之后,
    //         又来一条被接受的命令把值设回【上次真的发过的那个值】⇒ ①命中 ⇒ 顺手清掉
    //         待发标志, 那条已经许出去的应答就取消了。这是【有意且无害】的: 此时净生效值
    //         恰等于 MATLAB 上次被告知的值 ⇒ 不产生分歧。
    //     (细则见 RelayCore.cpp 里那三个状态的定义与 ① 的清理分支。)
    //   ⇒ MATLAB 永远不需要"记住"自己设过什么, 它只显示这里说的数
    //   ⇒"GUI 显示的值 ≠ 实际生效的值"这个状态【在结构上无法存在】。
    // 报的是【目标值】, 不是斜坡的瞬时值 (见 ForceTuning.h 顶上那段)。
    //
    // ⚠ force 的含义【按调用点分两种】。从前这里只写了"连接 / 重连", 而【拒绝路径也是
    //   force=true】—— 按那句旧注释去"统一"成 false, 会让被拒的 RG| 彻底静默 (说明见下)。
    //   · force=true —— 无条件发, 不看下面那两条闸。三个调用点:
    //       ① initRelayReporting() 连上时
    //       ② ensureRelayConnected() 重连成功时
    //          (这两处 MATLAB 手里什么都没有, "值没变"没有意义)
    //       ③ dispatchRelayCommand() 的【拒绝】分支 —— 见 RelayCore.cpp 那一处的完整说明:
    //          被拒 ⇒ 生效值【按构造】没变 ⇒ 条件①【必然】命中 ⇒ 非无条件则一个字节都发不
    //          出去, 而 MATLAB 的滑条此刻【已经动了】、正等着被纠正回真值。
    //   · force=false —— 限频形态 (规格 §4): 两条【同时成立才发】:
    //       ① 目标值相对上次真的走进 send 的那一条变了 (没变就没可报的), 且 ② 距上次发送 ≥100ms。
    //       ⚠ 这里的"发出去 / 发送"指的是【走到 sendRelayUpdate 那一步】, 不是"确认送达" ——
    //         两个状态都落笔在 send 调用之前, 精确说法见 RelayCore.cpp 里那三个状态的定义。
    //     两个调用点: dispatchRelayCommand() 的【接受】分支 (拖动洪水, 防刷屏),
    //     以及 pollRelayCommands() 每帧的补发 (把被时间挡下的最后一条送出去)。
    //     任一条件不满足就只记下待发 (值没变则把待发也清掉), 由补发兜底 ——
    //     拖动滑条几十条/秒不会堆在 MATLAB 侧, 而"最后一条一定到"由补发保证。
    void sendReflectionGain(bool force);

    // 调零请求 (Z| 协议)。只置标志 —— 真正的处置在 main.cpp 的 requestForceZero(),
    // 因为 g_noRobot / cancelOtherCaptureModes 都是那个文件的 file-static, 这里拿不到。
    // 【为什么不让 RelayCore 自己判】复制一份"标定中/调零中"的守卫链就是本项目最忌讳的
    //   两份实现; 而两条入口 (键盘 'z' / MATLAB) 共用同一个函数, 守卫链就只有一份。
    // 返回 true 表示本次调用消费掉了一个待处理请求 (读到即清)。
    bool consumeForceZeroRequest();

    // 状态查询（供 Render 层读取）
    bool isTransmitting() const { return m_transmitting; }

    // 看门狗状态查询
    DWORD lastHapticFrameMs() const { return m_lastHapticFrameMs.load(); }
    // ★★ 2026-09-22: 由【触觉回调入口】无条件调用 —— 见 RelayCore.cpp 该函数的说明。
    // 从前这个时间戳是 `sendPosition` 刷的，而它只在 transmitting 时才跑 ⇒
    // 那个数表达的是"上一次下发"，不是"上一帧触觉回调" ⇒ 看门狗分不开
    // "回调停了"与"没在下发"两种状态。现场 2026-09-22 即栽在这里。
    void markHapticFrame();
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

    std::atomic<bool> m_forceZeroRequested{false};   // MATLAB 的 Zero 按钮 (由 dispatchRelayCommand 置)

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
