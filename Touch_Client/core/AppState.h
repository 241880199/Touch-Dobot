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
    AppState();
    ~AppState();

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
    double stylusOrient[3] = { 0.0, 0.0, 0.0 };  // 笔杆姿态 ZYX Euler (Rx,Ry,Rz in degrees)
    // Protects stylusOrient. Currently all access is single-threaded (haptic callback),
    // but CRITICAL_SECTION is retained for future multi-threaded use.
    CRITICAL_SECTION stylusOrientMutex;

    // ===== 机械臂 TCP 双端口 =====
    SOCKET robotEnableSocket = INVALID_SOCKET;
    SOCKET robotMotionSocket = INVALID_SOCKET;
    CRITICAL_SECTION robotSocketMutex;
    std::atomic<bool> isRobotConnected{ false };

    // ===== 机械臂位姿（3D 模型驱动） =====
    struct RobotPose {
        double x = 0, y = 0, z = 0;
        double rx = 0, ry = 0, rz = 0;
        double j1 = 0, j2 = 0, j3 = 0, j4 = 0, j5 = 0, j6 = 0;
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

    // ===== 指令日志 (环形缓冲区) =====
    static const int LOG_SIZE = 50;
    char commandLog[50][256];
    int commandLogIdx = 0;
    int commandLogCount = 0;
    CRITICAL_SECTION commandLogMutex;

    // ===== 反馈日志 (环形缓冲区) =====
    char feedbackLog[50][256];
    int feedbackLogIdx = 0;
    int feedbackLogCount = 0;
    CRITICAL_SECTION feedbackLogMutex;

    // ===== 力反馈开关 (MATLAB relay_gui 反向命令控制) =====
    // true=渲染力反馈, false=力归零(位置跟随保持)。
    // 由 RelayCore::pollRelayCommands 写入 (主线程), haptic 回调线程读取 (1kHz)。
    std::atomic<bool> forceFeedbackEnabled{ true };

    // ===== 力数据 =====
    struct ForceData {
        double raw[6] = {0};            // Fx,Fy,Fz,Mx,My,Mz (N, Nm) — 30004 原始数据
        double compensated[6] = {0};    // 重力+惯性+零偏补偿后 (N, Nm)
        double filtered[6] = {0};       // Butterworth 低通滤波输出
        double hapticOut[3] = {0};      // 已变换到 Touch 坐标系，haptic 线程直接读
        bool isStale = true;            // 超过 200ms 无新数据
        DWORD lastUpdateMs = 0;
        // 【曾删过 payloadEcho[4], 现以清晰命名重建 —— 是【新】用途, 不是旧字段复活】。
        // 当初那个字段唯一的使用者是符号探针 (RelayCore::probePayloadResidual) 的
        // "机械臂有没有采纳候选负载"核对, 探针已废除。
        //
        // 30004 帧 @1168 的负载回读: 机械臂【自报】它当前在用的负载。
        // 用途 (【活路径】上的消费者, 逐处核过, 都在 main.cpp 的 solveAndApply 里):
        //   · buildSendCandidate —— 回读拿不到 ⇒ 【没有候选】(CAND_NO_PAYLOAD_ECHO);
        //     拿到时回读的 cz 就是闸 1 用来判符号的那个外部锚点 cz_robot。
        //   · diffSendCandidate —— 拿它与候选逐分量比, 得出"这一次改了哪些"。
        //   · diagPayloadSection —— 写标定报告块里"机械臂自报负载"那一节。
        // 用机械臂自报的值而不是我们下发的值, 是因为我们要的是【它实际在用什么】,
        // 不是【我们以为它该用什么】; 实测过两者会不一致 (自报 0.4061 vs 下发 0.404)。
        // ⚠ 【不是】"负载求解的基线": "真值 − 基线"那套折算是【旧模型】的做法 (它的消费者
        //   在 main.cpp 的 #if 0 块里, 死块), 而活的求解器不吃基线参数 —— fitRaw/decompose
        //   的入参里没有它。所以别按"基线"去活路径里找这三个字段的消费者, 找不到。
        // ⚠ 【这三个字段是活的】: RelayCore 的实时读取线程每收到一帧合规的帧就刷新它们
        //   (见那个线程里那一段的两条硬要求)。所以它们说的是"【此刻】机械臂在用哪份",
        //   'p' 下发成功之后它们会跟着变 —— 不要把它读成"连接时的那一份"。
        //   读方一律持 forceDataMutex 读 (与写方同一把锁)。
        bool   payloadEchoValid = false;
        double payloadEchoLoadKg = 0.0;
        double payloadEchoCenterMm[3] = {0, 0, 0};

        // 30004 帧里的 TCPForce @720 = "TCP力值 (通过关节电流计算)" —— 与 raw[] 读的
        // ActualTCPForce @576 = "TCP传感器力值" 是【两个不同的量】。
        // @576 是传感器自己的读数 (物理量, 不受控制器怎么想的影响);
        // @720 是从关节力矩反推的, 应当反映控制器使用的负载模型。
        // ★ 2026-09-21: 本字段是【运行时一致性闸门的判据参考量】(由"对比确认"升为判据的
        //   依据: 实测改 EnableRobot 的负载时 @576 纹丝不动 ⇒ 旧判据在量一个与被配置负载
        //   无关的量)。判据参考量的【唯一一份定义】在 ForceCompensation.cpp 的
        //   guardReferenceValue —— 要换判据看哪一路就改那一处; 这里【不许】再记一份语义,
        //   也不许再抄通道号 (print 端同理)。raw[] (@576) 现在退为【只报不判】的诊断侧。
        double tcpForce[6] = {0};

        // 30004 帧里的 ToolVectorActual @624 = "TCP笛卡尔实际坐标值" (6×double)。
        // 与 robotActualPose (来自 100ms 一次的 GetPose() 仪表盘查询) 是【两个不同的来源】:
        // 这个在 125Hz 的帧里, 但我们还没用它。**坐标系未确认** —— 名字含 "Tool",
        // 而重力模型 (ForceCompensation / TcpCalibration::gravitySensorFrame) 假定的是
        // 【基座系】(与 GetPose 一致)。并排打印两者即可判定。
        double tcpPoseActual[6] = {0};
        // 30004 帧里的 TCPSpeedActual @672 = "TCP笛卡尔实际速度值" (6×double)。
        double tcpSpeedActual[6] = {0};

        // 30004 帧 @1304 SixForceValue[6] = "当前六维力数据原始值" (厂商文档 P76)。
        // 与 raw[] (@576 ActualTCPForce = "TCP传感器力值【通过六维力计算】") 是【两个不同的量】:
        // @576 是【派生量】, 这个是原始读数。
        // ⚠ 从前这里写着"我们一直只读 @576" —— 那句已过期: 全量模型的补偿吃的是【本字段】
        //   (@1304, 见 ForceCompensation::step)。@576 现在只作为诊断侧报出。
        double sixForceRaw[6] = {0};
        // 30004 帧 @1037 SixForceOnline = "六维力在线状态" (char)。原样存放 (帧里那个字节
        // 本身是无符号的, 取不到 -1)。
        // ★ 2026-09-21: 初值 -1 是【有含义的哨兵】, 不是"未知" —— 它表示"一帧都还没收到"
        //   (RelayCore 每收到一帧就写 buf[1037])。它还是【参考量可用性】判据的两个输入之一,
        //   且判据是【正向确认】: 只有 == 1 才算这一路在线 (0 = 机械臂自报不在线, -1 =
        //   根本没有帧 —— 两者都判"不可用", 于是不放行)。
        //   判据的【唯一一份定义】在 ForceCompensation.cpp 的 guardReferenceAvailable ——
        //   要改口径就改那一处, 这里不许再解释一遍。
        int sixForceOnline = -1;

        // 标定参数 (由 ForceCalibration 求解, ForceCompensation 读取)
        bool isCalibrated = false;
        double calibMassKg = 0.0;           // 末端等效质量 (kg)
        double calibComSensor[3] = {0};     // 质心在传感器坐标系 (m)
        double calibBiasForce[3] = {0};     // 力零偏 (N)
        double calibBiasTorque[3] = {0};    // 力矩零偏 (Nm)
    };
    ForceData forceData;
    CRITICAL_SECTION forceDataMutex;

    // Orient mode extra constraint force (Thread-safe: orientForceMutex)
    double orientExtraForce[3];
    bool   hasOrientExtraForce;
    CRITICAL_SECTION orientForceMutex;

    // Orient mode directional repulsion force — Phase 2 (Thread-safe: orientRepulsionMutex)
    // Written by RelayCore::sendPosition (30Hz), read by haptic callback (1kHz)
    double orientRepulsionForce[3];
    bool   hasOrientRepulsion;
    CRITICAL_SECTION orientRepulsionMutex;
};

extern AppState appState;
