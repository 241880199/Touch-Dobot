#define _USE_MATH_DEFINES
#include "RelayCore.h"
#include "Button2Joint.h"     // ★ Task 2: 按钮2 关节空间纯函数 (笔杆偏移 ⇒ J4/J5/J6 增量)
#include "Button2Mapping.h"
#include "FeedbackParser.h"
#include "GainReadbackPolicy.h"
#include "RelayCommandParser.h"
#include "SafetyBoundary.h"
#include "../robot/RobotConnection.h"
#include "../robot/Kinematics.h"
#include "../core/AppState.h"
#include "../config/Config.h"
#include <algorithm>
#include <chrono>      // 帧率噪声探针要真实微秒: GetTickCount 的 15.6 ms 粒度分辨不出 8 ms 的帧间隔
#include <cmath>
#include <iostream>
#include <windows.h>
#include "../safety/SafetyPredictor.h"
#include "../safety/RobotDiagnostics.h"
#include "../safety/SelfCollision.h"
#include "../force/ForcePipeline.h"
#include "../force/ForceTuning.h"
#include "../force/ForceCompensation.h"
#include "../force/ForceCalibration.h"
#include "../force/ForceLogger.h"
#include "../force/PayloadCalibration.h"
#include "../calibration/TcpCalibration.h"
#include "../safety/SingularityAvoidance.h"

// ===== 姿态安全边界钳位 =====
// ⚠★ 2026-09-22 更正: 这里【不是】死代码 —— 2026-09-21 曾记下"范围等于满量程 ⇒ 从来没夹到过
//   任何东西", 那句话是错的。边界确实等于角度满量程 (Config.h:600-602 的 ±180/±90/±180), 但
//   【被夹的值不是规范化的角度】: 它的参照是 m_orientRefRobot + R·(笔杆偏移) (:1051-1053),
//   按 ≤3°/frame 朝该参照收敛 (:1061-1066, 上限 ORIENT_MAX_STEP_DEG = 3.0), 初值取自按下按钮2
//   那一刻的实际 RPY (:1313), 参照本身也抄自那一刻的 robotActualPose (:1303)。
//   ⚠ 措辞更正 (核过 :1047): :1047 明确写着这个期望目标"【不是累加】"(参照在整个按住期间不变);
//     真正逐帧累加的是 m_targetOrient 自己, 而它收敛到那个参照 ⇒ 被夹的量的界就是参照。
//     要点在【从不 wrap】: 全仓没有任何一处把它规范化到 (−180,180] ⇒ ±180/±90 是【真】边界,
//     不是"等于满量程 ⇒ 形同虚设"。⇒ 本函数是唯一在约束它的东西。
//   【它确实能撞到边界】: force_demo_log.csv 实测 pose_rx ∈ −179.95 ~ +179.988
//   (9686 行在 [+179,+180]、4801 行在 [−180,−179]; 众数 179.670)
//   ⚠ 但那一列是【机器人实际姿态】(本文件 :2094 的 pose[3] = app.robotActualPose.rx),
//     【不是】本函数的输出 ⇒ 它是"参照就坐在边界上"的证据, 不是"本函数夹过"的直接证据。
//     机制正是: 参照抄自实际姿态 (:1303) ⇒ 参照 ≈ ±180 ⇒ 再加任何正向偏移就越界。
//     最贴近的一次 179.988 只差 0.012° ⇒ 往外多转 0.012° 就撞 SAFE_RX_MAX, 然后每帧都夹;
//     179.560 那一档 (185 行) 差 0.44° 是【更松】的一档 —— 拿它收尾会【低估】离活动钳位的距离。
//   ⚠ 上面那句是【可达性】不是【已发生】—— 钳位告警走 std::cerr, 而留档的 send_transcript.txt
//     里一次按钮都没按过 ⇒ 它【不可能】含这条告警, 拿它当反证不算数。
// ⚠ 它还【不受】SAFETY_BOUNDARY_CLAMP_ENABLED 管: clampToBoundaryActive 的 4 个调用点
//   (:871/:1262/:1328/:1363) 全是【位置】, 姿态这一路根本没走那个开关
//   ⇒ 关掉那个开关【不会】关掉本函数。Config.h:41 那处【原先】说钳位在"目标位置/姿态路径"
//   的 4 个下发点 —— 与代码不符, 已一并更正。
static Vec3 clampOrientToBounds(const Vec3& target) {
    Vec3 clamped = target;
    bool warned = false;

    if (target.x < Config::SAFE_RX_MIN) { clamped.x = Config::SAFE_RX_MIN; warned = true; }
    if (target.x > Config::SAFE_RX_MAX) { clamped.x = Config::SAFE_RX_MAX; warned = true; }
    if (target.y < Config::SAFE_RY_MIN) { clamped.y = Config::SAFE_RY_MIN; warned = true; }
    if (target.y > Config::SAFE_RY_MAX) { clamped.y = Config::SAFE_RY_MAX; warned = true; }
    if (target.z < Config::SAFE_RZ_MIN) { clamped.z = Config::SAFE_RZ_MIN; warned = true; }
    if (target.z > Config::SAFE_RZ_MAX) { clamped.z = Config::SAFE_RZ_MAX; warned = true; }

    if (warned) {
        std::cerr << "[Safety] Orientation target out of bounds, clamped. Original: ("
                  << target.x << "," << target.y << "," << target.z << ")" << std::endl;
    }
    return clamped;
}

// ★★ 2026-09-22 实验（见 Config::ORIENT_SEAM_FIX_ENABLED 那一大段）：把角度规范化到 [-180,180)。
// 为什么用 fmod 而不是 while 循环：while 在病态输入 (inf/nan) 下会挂住整个下发线程。
// 注意 180 会映射成 -180 —— 同一个朝向，且规范区间是半开，这是刻意的。
static double normalizeDeg180(double deg) {
    if (!(deg > -1e18 && deg < 1e18)) return 0.0;   // NaN/inf 兜底: 别把毒值发给机械臂
    double r = fmod(deg + 180.0, 360.0);
    if (r < 0.0) r += 360.0;
    return r - 180.0;
}

// ★★ 2026-09-22 实验：钳【相对参照的偏移】，而不是绝对值。
// 【为什么绝对值那条会抖】参照 `m_orientRefRobot` 抄自按下按钮2 那一刻的【实际姿态】
//   (:1303)，它可能就坐在 ±180 上（现场 `R=(+176.1,…)` 距 +180 只有 3.9°）⇒ 往外多转一点点
//   就撞墙 ⇒ 目标被钉死在该边界 ⇒ 手一抖就在墙上【逐帧来回】⇒ 指令以帧率往复 ⇒ 臂抖。
// 【为什么相对钳位才对】这个钳位想防的是"**相对起始姿态**转太多"，不是"别过 180 这个数"。
//   而且 ±180 只是 RPY 表示的接缝，J6 量程本来就是 ±360。
static Vec3 clampOrientOffset(const Vec3& target, const Vec3& ref) {
    const Vec3 raw(target.x - ref.x, target.y - ref.y, target.z - ref.z);
    Vec3 off = raw;
    bool warned = false;

    if (off.x >  Config::ORIENT_MAX_OFFSET_DEG) { off.x =  Config::ORIENT_MAX_OFFSET_DEG; warned = true; }
    if (off.x < -Config::ORIENT_MAX_OFFSET_DEG) { off.x = -Config::ORIENT_MAX_OFFSET_DEG; warned = true; }
    if (off.y >  Config::ORIENT_MAX_OFFSET_DEG) { off.y =  Config::ORIENT_MAX_OFFSET_DEG; warned = true; }
    if (off.y < -Config::ORIENT_MAX_OFFSET_DEG) { off.y = -Config::ORIENT_MAX_OFFSET_DEG; warned = true; }
    if (off.z >  Config::ORIENT_MAX_OFFSET_DEG) { off.z =  Config::ORIENT_MAX_OFFSET_DEG; warned = true; }
    if (off.z < -Config::ORIENT_MAX_OFFSET_DEG) { off.z = -Config::ORIENT_MAX_OFFSET_DEG; warned = true; }

    if (warned) {
        std::cerr << "[Safety] Orientation offset from press-time reference clamped to ±"
                  << Config::ORIENT_MAX_OFFSET_DEG << " deg. Original offset: ("
                  << raw.x << "," << raw.y << "," << raw.z << ")" << std::endl;
    }
    return Vec3(ref.x + off.x, ref.y + off.y, ref.z + off.z);
}

// ★★ 2026-09-22 实验之二（见 Config::ORIENT_STYLUS_LPF_ENABLED 那一大段）：
// 笔杆姿态【偏移】的一阶低通状态。放文件作用域而不是成员，是为了不动 RelayCore.h ——
// 生命周期与进程相同，而参照那一组是每次按下按钮2 重设的 ⇒ 必须配一个 reset（见 onButton2Press）。
static double s_stylusOffFilt[3] = { 0.0, 0.0, 0.0 };
static DWORD  s_stylusOffFiltLastMs = 0;
static bool   s_stylusOffFiltReady = false;

static void resetStylusOffsetFilter() {
    s_stylusOffFilt[0] = s_stylusOffFilt[1] = s_stylusOffFilt[2] = 0.0;
    s_stylusOffFiltLastMs = 0;
    s_stylusOffFiltReady = false;
}

// ★ 2026-09-23 (Task 3): "标定路径没接上"这件事只披露【一次】，别每帧刷屏。
// 见 Config::BTN2_ROTATION_COMPOSE_ENABLED 与 Button2Mapping.h 的调用约定那一节。
static bool s_warnedBtn2CalibIgnored = false;

// ===== 帧率噪声探针的【环形缓冲】=====
// 见 RelayCore.h 里那一段 (要回答什么问题) 与 force/NoiseProbe.h (统计量的判据)。
// 【为什么在这里】本线程是【唯一】看得到 8 ms 那一档的地方: 力流水线当时跑在 pollForce 的
//   ~11 Hz 上, 拿它来回答"帧率下平均有没有用", 等于用被抽样过的数据回答抽样本身的问题。
//   (★ 探针按出来之后, 同一天就把补偿+滤波【搬到了本线程】来跑 —— 见下面帧解析那一段。
//    本探针留在原处: 它量的是【原始 @1304】的结构, 与流水线搬没搬无关。)
// 【容量】2048 帧 = 文档速率 125 Hz 下 16.4 s。够长到能算 lag=32 的自相关, 又不至于占内存。
// ⚠ 加它【不改变任何行为】: 只存不改, 不进任何补偿/滤波/闸门路径。
static const int kNoiseCap = 2048;
static RelayCore::ForceFrameSample s_noiseBuf[kNoiseCap];
static int                      s_noiseWrite = 0;   // 下一个写入位置
static int                      s_noiseCount = 0;   // 已存帧数 (封顶到 kNoiseCap)
static CRITICAL_SECTION         s_noiseLock;
static bool                     s_noiseLockInit = false;

// 只在 ForceReader 线程调用。加锁的理由: 读它的是主线程 (按键), 【不是】因为写入慢 ——
// 125 Hz 下一次临界区可以忽略。与 forceDataMutex 分开, 免得探针的读把力数据的路径也拖住。
static void pushForceFrame(double fx, double fy, double fz) {
    if (!s_noiseLockInit) return;   // 初始化竞态里的最早期帧, 丢掉即可 (不改变任何判决)
    const unsigned long long us =
        (unsigned long long)std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    EnterCriticalSection(&s_noiseLock);
    s_noiseBuf[s_noiseWrite].tickUs = us;
    s_noiseBuf[s_noiseWrite].f[0] = fx;
    s_noiseBuf[s_noiseWrite].f[1] = fy;
    s_noiseBuf[s_noiseWrite].f[2] = fz;
    s_noiseWrite = (s_noiseWrite + 1) % kNoiseCap;
    if (s_noiseCount < kNoiseCap) s_noiseCount++;
    LeaveCriticalSection(&s_noiseLock);
}

// ===== ForceReader 线程: 阻塞读取 30004 实时力数据 (125Hz) =====
static DWORD WINAPI forceReaderThread(LPVOID) {
    auto& app = appState;
    std::cout << "[Force] Reader thread started, connecting to port "
              << Config::FORCE_REALTIME_PORT << "..." << std::endl;

    while (!app.isClosing) {
        if (!robotConnectRealtime(Config::ROBOT_IP)) {
            std::cerr << "[Force] Realtime port connect failed, retrying in "
                      << Config::FORCE_RECONNECT_INTERVAL << "ms..." << std::endl;
            Sleep(Config::FORCE_RECONNECT_INTERVAL);
            continue;
        }

        std::cout << "[Force] Reader thread receiving at 125Hz..." << std::endl;
        char buf[1440];

        while (!app.isClosing) {
            if (!robotRecvRealtime(buf, sizeof(buf))) {
                std::cerr << "[Force] Realtime recv failed, reconnecting..." << std::endl;
                break;  // reconnect loop
            }

            // ===== 机械臂自报的负载 (30004 @1168 Load / @1176~1199 CenterX/Y/Z) =====
            // 【这四个字段持续跟着最新一帧走】: 它们说的是"机械臂【此刻】在用哪份负载参数",
            //   而机械臂的负载会因为我们下发 ('p' 的 EnableRobot/LoadSwitch) 而变。只读一次
            //   就等于把发送【前】的值一直挂在快照里 —— 发送成功之后, 快照仍说旧值, 下一次
            //   's' 就会把一次【已经生效】的改动打印成"待发的改动", 并说出"只改 m"这类与
            //   事实相反的话 (本项目最忌讳的"安静地不一致"); 而 Task 10 的闭环正是反复比对
            //   发送前/后的仪器, 那种错在那里必然发生。所以: 每帧刷新 (与下面那批力数据同一
            //   把锁、同一个节拍, 不需要限流)。
            // 【诊断行仍旧只打一次】: 下面那段与客户端生效值的一致性比较是给人看的一次性提示,
            //   125 Hz 逐帧刷屏会把别的输出冲掉。刷新与打印是两件事, 这里分开。
            // ⚠ 【两个不许】(从前那个 bug 的根源): sanity 不过的帧【不许】写字段 (一帧坏数据
            //   会污染快照), 也【不许】把"已经打过诊断行"记下来 —— 从前的写法在 sanity 之前
            //   就 latch, 于是首帧不合理的会话里这四个字段【整个会话都是空的】。
            // 30004 布局: Load @1168 (1×double), CenterX/Y/Z @1176~1199 (3×double)。
            // ⚠ 现在只报不判: 与本客户端的值对不上【不】当故障 —— 负载确实是生效的
            //   (连接时序里随 EnableRobot 下发, 2026-09-19 实机证实运行中改负载会让机械臂动),
            //   但早先的探针里 ActualTCPForce @576 没跟着 0.25 kg 的配置变化走, 为什么还不清楚。
            //   本回读仅供诊断, 不影响标定 —— 真正生效的是本地补偿。
            {
                // 诊断行只打一次的开关。【只在 sanity 通过的那一帧置位】。
                static bool loadEchoDiagPrinted = false;
                const double* echo = reinterpret_cast<const double*>(buf + 1168);
                const bool sane = echo[0] >= 0.0 && echo[0] <= 5.0
                               && fabs(echo[1]) <= 500.0 && fabs(echo[2]) <= 500.0
                               && fabs(echo[3]) <= 500.0;
                if (sane) {
                    // 落进 ForceData: 【下发候选】用 —— main.cpp 的 solveAndApply 拿它当闸 1 的
                    // 外部锚点 (cz_robot), 并与它做"这一次改了哪些"的逐分量比对; 标定报告块
                    // (diagPayloadSection) 也读它。存机械臂自报的值而非我们下发的值 —— 要的是
                    // "它实际在用哪个"。读方持 forceDataMutex 读 (契约不变)。
                    EnterCriticalSection(&app.forceDataMutex);
                    app.forceData.payloadEchoLoadKg = echo[0];
                    for (int i = 0; i < 3; i++) {
                        app.forceData.payloadEchoCenterMm[i] = echo[1 + i];
                    }
                    app.forceData.payloadEchoValid = true;
                    LeaveCriticalSection(&app.forceDataMutex);

                    if (!loadEchoDiagPrinted) {
                        loadEchoDiagPrinted = true;
                        double mWant, cWant[3];
                        PayloadCalibration::effective(mWant, cWant);
                        // ⚠ 【本行是首个合格帧的快照, 不是实时值】: 上面那个 static 开关只放行
                        //   一帧, 之后不再重打; 而机械臂自报的负载会随 'p' 的下发变。报告块与候选
                        //   屏取的是【每帧刷新】的那组字段, 所以发送成功之后两处的数会不同 ——
                        //   那是"快照 vs 当前", 【不是】"发送没生效"。措辞里明写这一点。
                        char msg[192];
                        snprintf(msg, sizeof(msg),
                                 "[Relay] 机械臂实际负载 (【首个合格帧】的快照):"
                                 " load=%.3f kg  center=(%.1f, %.1f, %.1f) mm",
                                 echo[0], echo[1], echo[2], echo[3]);
                        if (fabs(echo[0] - mWant) > 0.01 || fabs(echo[3] - cWant[2]) > 1.0) {
                            std::cout << msg << "\n[Relay] · 与客户端的 "
                                      << mWant << " kg / (" << cWant[0] << "," << cWant[1] << ","
                                      << cWant[2] << ") mm 不一致"
                                      << " —— 本行只描述【那一帧】, 与此刻的差值无关:"
                                      << " 机械臂在用的负载会被 'p' 的下发改掉, 而这一行不会再打。"
                                      << std::endl;
                            std::cout << "[Relay]   要看【当前】的读数: 用每帧刷新的回读字段"
                                      << " (标定报告块与下发候选那一屏取的就是它)" << std::endl;
                            std::cout << "[Relay]   重力/惯性补偿由 ForceCompensation 在本地做"
                                      << " (差值来源尚未查清, 见上方注释)" << std::endl;
                        } else {
                            std::cout << msg << std::endl;
                        }
                    }
                }
            }

            // Parse ActualTCPForce at offset 576 (6 doubles, 48 bytes)
            double* forcePtr = reinterpret_cast<double*>(buf + 576);
            // 同一帧里的 TCPForce @720 —— 与 @576 是两个不同的量 (见 AppState.h 的说明)。
            // 实测改 EnableRobot 的负载时 @576 不变, 所以两路都留着, 供以后对比/诊断。
            double* tcpForcePtr = reinterpret_cast<double*>(buf + 720);
            // 同一帧里的 ToolVectorActual @624 / TCPSpeedActual @672。
            // ★ 2026-09-24: @624 的坐标系已确认 (= 基座系，与 GetPose 同约定) **且已被消费**
            //   —— 本函数下面取重力项姿态时优先用它 (恰好全 0 才退回仪表盘)。
            //   判定办法与证据写在 AppState.h 那个字段上。@672 **仍未确认**，仍然只镜像。
            const double* tcpPosePtr = reinterpret_cast<const double*>(buf + 624);
            const double* tcpSpeedPtr = reinterpret_cast<const double*>(buf + 672);
            // 同一帧里的 SixForceValue @1304 = "当前六维力数据原始值" —— 与 @576 的派生量
            // 是【两个不同的量】, 见 AppState.h。六维力在线状态 @1037 (char, 单字节)。
            // 帧长 1440: 1304+48=1352 与 1037 都在范围内。
            double* sixForcePtr = reinterpret_cast<double*>(buf + 1304);
            const int sixForceOnline = static_cast<int>(static_cast<unsigned char>(buf[1037]));

            // ★★ 2026-09-21: 补偿与滤波搬到这里 (帧率) 来跑 —— 从前它们在 pollForce 里,
            //    而 pollForce 的真实节拍是 ~11 Hz (实测间隔均值 92 ms, 见 force_demo_log.csv 的 t_ms)。
            // 【为什么非搬不可】噪声只能靠【过采样 + 平均】压; 一条跑在 11 Hz 上的链, 把 125 Hz 的帧
            //   用掉了 11 分之一, 而滤波器按 fs=120 Hz 算的系数撞上 11 Hz 的实际调用率
            //   ⇒ 实际截止掉到 ~2.75 Hz, 且没有任何 √N 可赚。实测账 (帧率噪声探针 'n'):
            //   原始 @1304 的 Fz sd 0.142 N, 连续 16 帧取平均可降到 0.022 N;
            //   而旧链路只把它削到 0.115 N (日志里的 filtered 列)。
            //   ⇒ 完整推导与取舍见 Config::FORCE_FILTER_CUTOFF / FORCE_FILTER_FS_HZ 那两段。
            // 【姿态为什么在这里读】与 pollForce 【同一来源、同一把锁、同一顺序】:
            //   robotPoseMutex 先取先放, 然后才取 forceDataMutex。反过来会与 pollForce 构成死锁。
            //   (HapticCallback 从不嵌套这两把锁 —— 144~172 行是串行取的。)
            //   ★★ 2026-09-24: **姿态改用【同一帧】的 ToolVectorActual @624**。上面那句
            //     "姿态仍只由 GetPose() 每 100 ms 刷新 ⇒ A·g 是阶梯，但重力项变化慢
            //      (0.1° ≈ 0.007 N)，代价可接受" —— **算术对，前提错**：
            //      0.1° 是"慢速工况"下的陈旧量；关节模式手腕能到 90°/s，而姿态实测每 ~200 ms
            //      才更新一次 ⇒ 每步陈旧 **9~21°** ⇒ `A·g` 残差可达 **1.55 N**。
            //      离线复算与 force_demo_log 同一时刻的实测 |F| 逐个对上：
            //        0.591/0.591 · 1.041/1.046 · 0.670/0.691 · 1.555/1.656（178 个姿态台阶，
            //        corr(|ΔFg|,|F|) = +0.727）；乘力反射增益 ≈1.98 ⇒ 手上 1~3 N 的"较大且抖的力"。
            //   【@624 的约定已实测确认，不再是"未确认"】位置那六位早就是同一个量
            //     （180 条采样逐位相同到 ±0.001 mm）；姿态那三位 2026-09-24 实测：静止 32 个样本的
            //     旋转矩阵差中位 0.000° / 最大 0.068°，运动样本的差正好= GetPose 自己的陈旧量
            //     ⇒ 与 GetPose **同一约定**（ZYX/度/基座系）。
            //   【为什么仍读一份仪表盘姿态】@624 在启动早期可能还没被填（全 0）⇒ 那一步退回
            //     GetPose（难看，但比把 0 当姿态喂进重力项好得多）。判据沿用本仓既有的
            //     "恰好全 0 = 从没写过"那条指纹（同 `isTrustworthyJointRef` 的子句一）。
            //   ⚠ 兜底那一读守的是【与从前完全相同】的锁行为：同一把锁、同一顺序、同一频率
            //     ⇒ 不引入任何新的锁序/争用（下面 forceDataMutex 那段注释仍然成立）。
            // 【★ 给下一个人的警告】这两个调用【全程序只能有这一处】。若在 pollForce 里再调一次,
            //   滤波器每帧被推两次 (11 Hz 那一路会把 123 Hz 的结果又滤一遍) ⇒ 相位与幅值全乱,
            //   而且没有任何报错。
            double dashPose[6] = {0};
            EnterCriticalSection(&app.robotPoseMutex);
            dashPose[0] = app.robotActualPose.x;  dashPose[1] = app.robotActualPose.y;
            dashPose[2] = app.robotActualPose.z;  dashPose[3] = app.robotActualPose.rx;
            dashPose[4] = app.robotActualPose.ry; dashPose[5] = app.robotActualPose.rz;
            LeaveCriticalSection(&app.robotPoseMutex);

            EnterCriticalSection(&app.forceDataMutex);
            for (int i = 0; i < 6; i++) {
                app.forceData.raw[i] = forcePtr[i];
                app.forceData.tcpForce[i] = tcpForcePtr[i];
                app.forceData.tcpPoseActual[i] = tcpPosePtr[i];
                app.forceData.tcpSpeedActual[i] = tcpSpeedPtr[i];
                app.forceData.sixForceRaw[i] = sixForcePtr[i];
            }
            app.forceData.sixForceOnline = sixForceOnline;
            app.forceData.lastUpdateMs = GetTickCount();
            app.forceData.isStale = false;

            // ★ 姿态：优先【同一帧】的 @624；全 0（启动早期没填过）则退回上面那份仪表盘姿态。
            //   ⚠ 判据是"恰好全 0"而不是"接近 0"：机械臂真的停在法兰 (0,0,0)/姿态全 0 是不可能同时
            //     成立的位姿，而"从没被写过"恰好就是这个指纹 —— 与 isTrustworthyJointRef 同一套。
            const double* p624 = app.forceData.tcpPoseActual;
            const bool have624 = !(p624[0] == 0.0 && p624[1] == 0.0 && p624[2] == 0.0 &&
                                   p624[3] == 0.0 && p624[4] == 0.0 && p624[5] == 0.0);
            double pose[6];
            for (int i = 0; i < 6; i++) pose[i] = have624 ? p624[i] : dashPose[i];

            // 一次性出声：把"这一刻用的是哪个源"变成可观测的。两种源的差别正是本节要消灭的东西
            // —— 而"读代码才知道用的哪一个"正是本项目最忌的那种状态。
            // ⚠ 只打一行、且只在这个线程里（forceDataMutex 持锁中 ⇒ 同一时刻只有一个写者）。
            static bool s_poseSrcReported = false;
            if (!s_poseSrcReported) {
                s_poseSrcReported = true;
                std::cout << "[Force] 重力项姿态源 = "
                          << (have624 ? "同帧 ToolVectorActual @624 (123 Hz)"
                                      : "⚠ GetPose 仪表盘 (~10 Hz 阶梯) —— @624 还没被填")
                          << std::endl;
            }

            // 顺序不能反: 先补偿 (ForceCompensation), 再滤波 + 映射 (ForcePipeline)。
            ForceCompensation::step(app.forceData, pose);
            ForcePipeline::step(app.forceData);

            LeaveCriticalSection(&app.forceDataMutex);

            // 帧率噪声探针: 存下这一帧的原始 @1304 三轴。
            // ⚠ 位置在【本帧解析完之后、且与本帧的赋值同源】—— 存的是刚读进来的 sixForcePtr,
            //   不是别的快照。存的这一列要拿来量"帧率下噪声的结构", 错一帧就白量。
            pushForceFrame(sixForcePtr[0], sixForcePtr[1], sixForcePtr[2]);

            // 看门狗兜底: 每 300ms 检查一次 (GLUT 可能已死)
            static DWORD lastWatchdogCheck = 0;
            DWORD now = GetTickCount();
            if (now - lastWatchdogCheck > 300) {
                lastWatchdogCheck = now;
                auto& relay = RelayCore::instance();
                DWORD lastHaptic = relay.lastHapticFrameMs();
                if (relay.isTransmitting() && lastHaptic > 0 &&
                    (now - lastHaptic) > (DWORD)(Config::WATCHDOG_TIMEOUT_MS * 2)) {
                    std::cerr << "[Safety] ForceReader WATCHDOG: GLUT appears dead ("
                              << (now - lastHaptic) << "ms since last haptic frame) — "
                              << "sending EmergencyStop" << std::endl;
                    robotSendEnable("DisableRobot()");
                    Sleep(100);
                }
            }
        }

        robotCloseRealtime();
        if (!app.isClosing) {
            Sleep(Config::FORCE_RECONNECT_INTERVAL);
        }
    }

    std::cout << "[Force] Reader thread exiting" << std::endl;
    return 0;
}

// ===== 使能机械臂 (带末端负载参数) =====
// EnableRobot(load, centerX, centerY, centerZ) — 机械臂内部按此做重力/惯性补偿。
// 负载设置不准 → 30004 力值随姿态漂移 / 碰撞检测误触发 / 拖拽失控。
// 负载值优先取实机标定结果 (PayloadCalibration / payload_calib.json),
// 未标定时回退 Config.h 的种子值。
//
// 连接时真正下发的那份负载 —— 机械臂在【整个会话】里用的就是这一份做重力/惯性补偿。
// 运行中的重新使能 (脱困 / 报警恢复) 必须原样回放它, 【不能】再去读 effective():
// 标定求解 (main.cpp 的 's') 会把内存生效值改成新值 (PayloadCalibration::applyResult),
// 而运行中改负载恰恰是能让机械臂猛地动起来的操作 (2026-09-19 实机证实: 改成 1.5 kg
// 后重启, 机械臂快速撞向关节限位)。它同时会让本地残余补偿多减一份, 即同一个误差减两次。
static bool s_sentPayloadValid = false;
static double s_sentPayloadMassKg = 0.0;
static double s_sentPayloadComMm[3] = {0.0, 0.0, 0.0};

// ===== 负载下发命令的文本: 全程序【唯一】的拼法 (Task 8a-3, 声明见 RelayCore.h) =====
// 【发送侧与确认屏都调这里】。从前这条格式串在本文件里有两份、main.cpp 的确认预览里还有
// 第三份 —— 三份逐字一致是【人工维持】的, 而"人确认的文本"与"发出去的字节"一旦分家,
// 屏幕就会让人确认另一条命令。抽成一处之后, 确认屏摆出来的就是发送侧要写进 socket 的那一份。
void RelayCore::formatPayloadEnableCommand(double massKg, const double comMm[3],
                                           char* out, int n) {
    snprintf(out, n, "EnableRobot(%.3f,%.1f,%.1f,%.1f)",
             massKg, comMm[0], comMm[1], comMm[2]);
}

const char* RelayCore::payloadLoadSwitchCommand() { return "LoadSwitch(1)"; }

// 参数化的发送器: 负载是入参, 不在这里隐式读全局生效值。
// note 仅供日志标注 (可为 nullptr), 不影响下发内容。
static bool sendEnableRobotWithPayload(double massKg, const double com[3],
                                       const char* note = nullptr) {
    char cmd[96];
    // 文本取自上面那个 formatter —— 连接时序与运行时下发拼的是同一条命令 (不再各拼一次)。
    RelayCore::formatPayloadEnableCommand(massKg, com, cmd, sizeof(cmd));
    std::cout << "[Relay] 使能 " << cmd;
    if (note) std::cout << "  (" << note << ")";
    std::cout << std::endl;
    return robotSendEnable(cmd);
}

// 【只允许连接时序 (init) 调用】—— 它下发当前生效值, 并把下发成功的那份记成快照。
// 若在运行中调用, 快照就会被"新解出的"负载覆写, 那正是本文件要避免的事。
static bool enableRobotWithPayload() {
    double m, c[3];
    PayloadCalibration::effective(m, c);
    bool ok = sendEnableRobotWithPayload(
        m, c, PayloadCalibration::enabled ? "实机标定值" : "种子值, 未标定");
    if (ok) {
        // 成功之后才记: 直到进程重启, 机械臂用的就是这一份, 运行中的重新使能只回放它。
        s_sentPayloadMassKg = m;
        s_sentPayloadComMm[0] = c[0];
        s_sentPayloadComMm[1] = c[1];
        s_sentPayloadComMm[2] = c[2];
        s_sentPayloadValid = true;
    }
    return ok;
}

// 运行中的重新使能 (脱困 / 报警恢复) 专用: 回放连接时真正下发的那份负载。
static bool reenableRobotWithConnectPayload() {
    if (!s_sentPayloadValid) {
        // 连接时的使能没成功过, 快照不可用, 只能回退到当前生效值 —— 而它【未必】是机械臂
        // 此刻实际在用的负载, 所以这条路径必须把话说明白。
        double m, c[3];
        PayloadCalibration::effective(m, c);
        std::cout << "[Relay] 警告: 无连接时的负载快照 (使能未成功过), 回退到当前生效值 — "
                  << "它未必是机械臂此刻实际在用的负载" << std::endl;
        return sendEnableRobotWithPayload(m, c, "回退: 非连接时下发值, 未必是机械臂在用的负载");
    }
    return sendEnableRobotWithPayload(s_sentPayloadMassKg, s_sentPayloadComMm,
                                      "回放连接时的负载");
}

// ===== 运行时显式下发负载 (Task 8a) —— 【只由用户显式触发, 不进连接时序】 =====
//
// 发两条, 就两条 (用户 2026-09-20 裁定; 计划文档里那份"三条一起发"写在拆分之前, 已被取代):
//   EnableRobot(m, cx, cy, cz)   质量 + 质心 (【唯一】能传质心的通道; centerX/Y/Z 单位 mm)
//   LoadSwitch(1)                负载设置开关 (0=关闭 / 1=开启)
// 出处 (逐条回源头核过; brief 里给的 "256-261" 只覆盖 LoadSwitch 那一条, 不是两条都在那里):
//   EnableRobot  Docs/机械臂资料/TCP_IP远程控制接口文档.md:116-125
//                (原型在 :117; centerX/Y/Z 单位 mm、范围 ±500 在 :121-123)
//   LoadSwitch   Docs/机械臂资料/TCP_IP远程控制接口文档.md:256-261
//                ("开启后可提高碰撞检测灵敏度" 在 :258; status 1 = 开启 在 :259)
//   两条一起列在 设计 §6b (Docs/superpowers/specs/2026-09-19-raw-channel-calibration-design.md:131-147)。
// ⚠ 顺序 (先 EnableRobot, 后 LoadSwitch(1)) **取自设计 §6b 的清单 (:144-146), 文档未说明其
//   必要性** —— 不替它编理由。
// ⚠ 【不发 PayLoad(m, I)】: 惯量要等惯量辨识 (Task 3) 出来, 补发归 8c。
//
// 【约束, 写死在这里, 改代码前先读这一条】
//  · 【不得】被 init() 或任何"重新使能"路径调用 —— enableRobotWithPayload /
//    reenableRobotWithConnectPayload 一行都不许碰它、也不许调它。它只由用户显式触发
//    (main.cpp 的发送键 'p')。理由与上面那条快照注释同源: 运行中改负载会让机械臂动。
//  · 【不】碰连接时序里那条 LoadSwitch(0) (init 里那三行"主动关掉灵敏度") —— 那是【独立
//    决定】(防误触发), 归 8b 之后的评估; 本函数只是在运行时再把负载设置打开。
//
// 两条都走【使能口】并【逐条读回执】, 照本文件 escapeSingularity 里既有的写法
// (drain → send → Sleep → recv)。控制台必须分得清三件事: 命令文本 / 机械臂回执 / 【哪一条失败】。
//
// 回执的判读: robotRecvEnable 只说明"收到了一行", 不说明机械臂【采纳了】—— Dobot 的返回以
// 首字符 '0' 表示 ErrorID == 0 (见 FeedbackParser::isSuccess)。所以成功 = 收到 且 ErrorID == 0,
// 否则把原始回执行照实打出来 (被拒的那一条, 回执里带着错误码)。
static bool sendPayloadCommands(double massKg, const double comMm[3]) {
    char cmd[96];
    char fb[256];

    // 文本取自 RelayCore::formatPayloadEnableCommand —— 全程序【唯一】的拼法 (见它的说明)。
    // 确认屏 (main.cpp 的发送键) 摆出来的也是它, 所以人确认的就是下面写进 socket 的这一份。
    RelayCore::formatPayloadEnableCommand(massKg, comMm, cmd, sizeof(cmd));
    std::cout << "[Relay] 负载下发 1/2: " << cmd << std::endl;
    robotDrainEnable();
    // ⚠ 【发送侧】的返回值也要看 (全文复审 Minor 4): robotSendEnable 返回 false = 这条命令
    //   没能【完整】写进 socket (部分写也算 false), 与"发出去了但机械臂没回执"是两件事, 而
    //   本函数的验收判据就是控制台必须【分得清哪一条失败】以及失败在哪一步。从前这里丢弃
    //   返回值, 于是发送侧的失败被报成"无回执, 超时" —— 那句话把人支去查机械臂, 而问题在链路。
    const bool sentEnable = robotSendEnable(cmd);
    bool gotEnable = false;
    fb[0] = '\0';
    if (sentEnable) {
        Sleep(100);
        gotEnable = robotRecvEnable(fb, sizeof(fb));
    }
    const bool okEnable = gotEnable && FeedbackParser::isSuccess(fb);
    std::cout << "[Relay]   回执: "
              << (!sentEnable ? "(发送失败 — 命令没能完整写进 socket)"
                              : (gotEnable ? fb : "(无回执, 超时)"))
              << (sentEnable && gotEnable && !okEnable ? "  ← 机械臂拒绝 (ErrorID != 0)" : "")
              << std::endl;

    std::cout << "[Relay] 负载下发 2/2: " << RelayCore::payloadLoadSwitchCommand() << std::endl;
    robotDrainEnable();
    const bool sentLoadSwitch = robotSendEnable(RelayCore::payloadLoadSwitchCommand());
    bool gotLoadSwitch = false;
    fb[0] = '\0';
    if (sentLoadSwitch) {
        Sleep(100);
        gotLoadSwitch = robotRecvEnable(fb, sizeof(fb));
    }
    const bool okLoadSwitch = gotLoadSwitch && FeedbackParser::isSuccess(fb);
    std::cout << "[Relay]   回执: "
              << (!sentLoadSwitch ? "(发送失败 — 命令没能完整写进 socket)"
                                  : (gotLoadSwitch ? fb : "(无回执, 超时)"))
              << (sentLoadSwitch && gotLoadSwitch && !okLoadSwitch
                      ? "  ← 机械臂拒绝 (ErrorID != 0)" : "")
              << std::endl;

    if (okEnable && okLoadSwitch) {
        std::cout << "[Relay] 负载已下发: EnableRobot + LoadSwitch(1) 两条都有成功回执" << std::endl;
    } else {
        std::cout << "[Relay] !! 负载下发【未完成】: 失败的条目 =";
        if (!okEnable)     std::cout << " EnableRobot";
        if (!okLoadSwitch) std::cout << " " << RelayCore::payloadLoadSwitchCommand();
        std::cout << " —— 两条只成了一部分, 机械臂此刻的负载参数【不要】当作已更新";
        // 再点一句【失败在哪一步】: 上面逐条的回执行已经说了, 这里汇总一句免得滚掉之后
        // 只剩"未完成"三个字 (发送侧的失败与机械臂拒绝的处置不同: 前者查链路, 后者查命令)。
        // ⚠ 措辞是"没能【完整】写进 socket": sendToSocket (robot/RobotConnection.cpp) 的
        //   判据是"send 的返回值 == strlen", 所以【部分写】同样返回 false —— 而那一次 send
        //   已经接受了前几个字节。说成"没写进"是说过头, 会把人支去按"什么都没发生"处置。
        if (!sentEnable || !sentLoadSwitch) std::cout << " (其中有【发送失败】的条目: 命令没能"
                                                         "完整写进 socket, 不是机械臂拒绝)";
        std::cout << std::endl;
    }
    return okEnable && okLoadSwitch;
}

RelayCore& RelayCore::instance() {
    static RelayCore inst;
    return inst;
}

RelayCore::RelayCore() {
    InitializeCriticalSection(&m_basePointLock);
    InitializeCriticalSection(&m_relaySocketMutex);
}

RelayCore::~RelayCore() {
    shutdownForceReader();
    shutdownRelayReporting();
    DeleteCriticalSection(&m_basePointLock);
    DeleteCriticalSection(&m_relaySocketMutex);
}

// ===== 奇异脱困: 检测报警 → 拖拽模式 → 等待手动挪动 → 重新使能 =====
static bool escapeSingularity() {
    // 防止重入
    static bool s_escaping = false;
    if (s_escaping) {
        std::cout << "[脱困] 已在脱困流程中，跳过重复触发" << std::endl;
        return false;
    }
    s_escaping = true;
    std::cout << "[脱困] 检测到报警，开始诊断..." << std::endl;

    char fb[256];

    // ===== 读取当前状态 (GetPose + GetAngle) =====
    robotDrainEnable();
    robotSendEnable("GetPose()");
    Sleep(100);
    AppState::RobotPose curPose;
    if (robotRecvEnable(fb, sizeof(fb))) {
        FeedbackParser::parsePose(fb, curPose);
        std::cout << "[脱困] 末端位姿: X=" << curPose.x << " Y=" << curPose.y
                  << " Z=" << curPose.z << " Rx=" << curPose.rx
                  << " Ry=" << curPose.ry << " Rz=" << curPose.rz << std::endl;
    }

    robotDrainEnable();
    robotSendEnable("GetAngle()");
    Sleep(100);
    double curJoints[6] = {0};
    if (robotRecvEnable(fb, sizeof(fb))) {
        FeedbackParser::parseAngle(fb, curJoints);
        std::cout << "[脱困] 关节角: J1=" << curJoints[0] << " J2=" << curJoints[1]
                  << " J3=" << curJoints[2] << " J4=" << curJoints[3]
                  << " J5=" << curJoints[4] << " J6=" << curJoints[5] << std::endl;
    }

    // 检查哪个关节接近限位 (容差 5°)
    struct { int id; double val, minV, maxV; const char* name; } limits[6] = {
        {1, curJoints[0], -360, 360, "J1"},
        {2, curJoints[1], -360, 360, "J2"},
        {3, curJoints[2], -155, 155, "J3"},
        {4, curJoints[3], -360, 360, "J4"},
        {5, curJoints[4], -360, 360, "J5"},
        {6, curJoints[5], -360, 360, "J6"},
    };

    int stuckJoint = -1;
    const char* stuckName = "";
    bool isUpperLimit = false;
    for (int i = 0; i < 6; i++) {
        if (limits[i].val >= limits[i].maxV - 5.0) {
            stuckJoint = limits[i].id;
            stuckName = limits[i].name;
            isUpperLimit = true;
            break;
        }
        if (limits[i].val <= limits[i].minV + 5.0) {
            stuckJoint = limits[i].id;
            stuckName = limits[i].name;
            isUpperLimit = false;
            break;
        }
    }

    if (stuckJoint == -1) {
        std::cout << "[脱困] 未检测到关节限位, 可能是其他原因" << std::endl;
        s_escaping = false;
        return false;
    }

    std::cout << "[脱困] 检测到 " << stuckName << " "
              << (isUpperLimit ? "正向" : "负向") << "限位 (当前 "
              << limits[stuckJoint-1].val << "°)" << std::endl;

    // Step 1: 强制进入拖拽模式 + 单独松问题关节抱闸
    std::cout << "[脱困] 进入强制拖拽模式..." << std::endl;
    robotSendEnable("SetCollideDrag(1)");
    Sleep(300);
    robotDrainEnable();
    if (robotRecvEnable(fb, sizeof(fb))) {
        std::cout << "[脱困] SetCollideDrag(1) 原始: " << fb;
    }

    // 单独松开问题关节的抱闸 (双保险)
    char brakeCmd[32];
    snprintf(brakeCmd, sizeof(brakeCmd), "BrakeControl(%d,1)", stuckJoint);
    std::cout << "[脱困] 单独松 " << stuckName << " 抱闸: " << brakeCmd << std::endl;
    robotSendEnable(brakeCmd);
    Sleep(200);
    robotDrainEnable();
    if (robotRecvEnable(fb, sizeof(fb))) {
        std::cout << "[脱困] " << brakeCmd << " 原始: " << fb;
    }

    // Step 2: 提示用户只动问题关节
    std::cout << "\n========================================" << std::endl;
    std::cout << "[脱困] 请将 " << stuckName << " 向"
              << (isUpperLimit ? "负方向(反向)" : "正方向(正向)")
              << "转动 20~30°!" << std::endl;
    std::cout << "[脱困] (其他关节不需要动)" << std::endl;
    std::cout << "[脱困] 转动完成后按 Enter 继续" << std::endl;
    std::cout << "========================================" << std::endl;

    // 等待
    for (int i = 0; i < 120; i++) {
        if (GetAsyncKeyState(VK_RETURN) & 0x8000) {
            while (GetAsyncKeyState(VK_RETURN) & 0x8000) Sleep(10);
            break;
        }
        if (i % 5 == 0 && i > 0) {
            std::cout << "[脱困] 等待中... (" << (120-i) << "s) 按 Enter" << std::endl;
        }
        Sleep(1000);
    }

    // Step 3: 验证 (GetPose + GetAngle)
    robotDrainEnable();
    robotSendEnable("GetPose()");
    Sleep(100);
    AppState::RobotPose newPose;
    if (robotRecvEnable(fb, sizeof(fb))) {
        FeedbackParser::parsePose(fb, newPose);
        std::cout << "[脱困] 拖动后末端: X=" << newPose.x << " Y=" << newPose.y
                  << " Z=" << newPose.z << " Rx=" << newPose.rx
                  << " Ry=" << newPose.ry << " Rz=" << newPose.rz << std::endl;
    }

    robotDrainEnable();
    robotSendEnable("GetAngle()");
    Sleep(100);
    double newJoints[6] = {0};
    if (robotRecvEnable(fb, sizeof(fb))) {
        FeedbackParser::parseAngle(fb, newJoints);
        std::cout << "[脱困] GetAngle原始: " << fb;
        std::cout << "[脱困] 拖动后关节: J1=" << newJoints[0] << " J2=" << newJoints[1]
                  << " J3=" << newJoints[2] << " J4=" << newJoints[3]
                  << " J5=" << newJoints[4] << " J6=" << newJoints[5] << std::endl;
    }

    double newVal = newJoints[stuckJoint - 1];
    if (isUpperLimit && newVal < limits[stuckJoint-1].maxV - 5.0) {
        std::cout << "[脱困] " << stuckName << " 已离开上限" << std::endl;
    } else if (!isUpperLimit && newVal > limits[stuckJoint-1].minV + 5.0) {
        std::cout << "[脱困] " << stuckName << " 已离开下限" << std::endl;
    } else {
        std::cout << "[脱困] " << stuckName << " 仍接近限位 ("
                  << newVal << "°)" << std::endl;
    }

    // Step 4: 退出拖拽 + 锁回问题关节
    robotSendEnable("SetCollideDrag(0)");
    Sleep(300);
    robotDrainEnable();
    snprintf(brakeCmd, sizeof(brakeCmd), "BrakeControl(%d,0)", stuckJoint);
    robotSendEnable(brakeCmd);
    Sleep(200);
    robotDrainEnable();

    // Step 5: 先独立尝试 ClearError (不使能)
    std::cout << "[脱困] 尝试 ClearError..." << std::endl;
    robotSendEnable("ClearError()");
    Sleep(300);
    robotDrainEnable();

    robotSendEnable("RobotMode()");
    Sleep(100);
    int mode = -1;
    if (robotRecvEnable(fb, sizeof(fb))) {
        FeedbackParser::parseMode(fb, mode);
        std::cout << "[脱困] ClearError后 mode=" << mode;
    }

    if (mode != 9 && mode != -1) {
        std::cout << "[脱困] ClearError 直接清除成功!" << std::endl;
        s_escaping = false;
        return true;
    }

    // Step 6: ClearError 不够, 需要 EnableRobot
    std::cout << "[脱困] 尝试 EnableRobot..." << std::endl;
    // 【必须】回放连接时下发的那份负载, 不能用 effective(): 内存生效值可能已被本会话的
    // 标定求解改掉 (main.cpp 's' → PayloadCalibration::applyResult), 而带着新负载重新
    // 使能正是能让机械臂猛地动起来的操作 —— 脱困时操作员的手可能就在设备上。
    reenableRobotWithConnectPayload();
    Sleep(300);
    robotDrainEnable();

    robotSendEnable("RobotMode()");
    Sleep(100);
    if (robotRecvEnable(fb, sizeof(fb))) {
        FeedbackParser::parseMode(fb, mode);
        if (mode != 9 && mode != -1) {
            std::cout << "[脱困] EnableRobot后成功! mode=" << mode << std::endl;
            s_escaping = false;
            return true;
        }
        std::cout << "[脱困] EnableRobot后仍报警: " << fb;
    }

    s_escaping = false;
    return false;
}

bool RelayCore::init() {
    if (!robotConnect(Config::ROBOT_IP)) {
        std::cerr << "[Relay] 连接机械臂失败" << std::endl;
        return false;
    }

    m_stateMachine.onConnect();
    m_lastHeartbeatMs = GetTickCount();
    m_heartbeatLostReported = false;

    // Register FATAL callback: disable robot hardware on fatal error
    m_stateMachine.setFatalCallback([]() {
        robotSendEnable("DisableRobot()");
        Sleep(100);
        std::cerr << "[Safety] FATAL: robot disabled by state machine" << std::endl;
    });

    // 初始化序列：ClearError → 降灵敏度 → EnableRobot → (报警检测) → CP → GetPose
    Sleep(200);
    robotSendEnable("ClearError()");
    Sleep(300);

    // 使能前关闭所有可能误触发的灵敏度设置
    robotSendEnable("SetCollisionLevel(0)");   // 碰撞检测: 0=最不灵敏
    Sleep(50);
    robotSendEnable("SetSafeSkin(0)");          // 关闭电子皮肤
    Sleep(50);
    robotSendEnable("LoadSwitch(0)");           // 关闭负载自适应
    Sleep(50);

    if (!enableRobotWithPayload()) {
        std::cerr << "[Relay] 使能失败" << std::endl;
        m_stateMachine.onEnableFail();
        return false;
    }
    std::cout << "[Relay] 机械臂使能成功" << std::endl;
    Sleep(200);

    // ===== 奇异检测: 使能后检查是否立即报警 (重试3次, 每次100ms) =====
    {
        int mode = -1;
        for (int retry = 0; retry < 3; retry++) {
            robotDrainEnable();
            robotSendEnable("RobotMode()");
            Sleep(100);
            char fb[128];
            if (robotRecvEnable(fb, sizeof(fb))) {
                if (FeedbackParser::parseMode(fb, mode)) {
                    break;
                }
            }
            std::cout << "[Relay] RobotMode retry " << (retry + 1) << "/3..." << std::endl;
        }

        if (mode == 9) {
            std::cout << "[Relay] 使能后检测到报警 (mode=9)，启动脱困流程..." << std::endl;
            if (!escapeSingularity()) {
                std::cerr << "[Relay] FATAL: 脱困失败" << std::endl;
                robotDisconnect();
                m_stateMachine.onEnableFail();
                return false;
            }
        } else if (mode == -1) {
            std::cout << "[Relay] 无法读取RobotMode (mode=-1)，继续初始化..." << std::endl;
        } else {
            std::cout << "[Relay] 机械臂状态正常 (mode=" << mode << ")" << std::endl;
        }
    }

    char cpBuf[64];
    snprintf(cpBuf, sizeof(cpBuf), "CP(%u)", Config::CP_SMOOTH_RATIO);
    robotSendEnable(cpBuf);
    Sleep(100);

    // 获取基准位姿 (重试 5 次，每次等待 100ms)
    bool gotBase = false;
    for (int retry = 0; retry < 5; retry++) {
        robotSendEnable("GetPose()");
        Sleep(100);
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
                gotBase = true;
                break;
            }
        }
        std::cout << "[Relay] GetPose retry " << (retry + 1) << "/5..." << std::endl;
    }

    if (!gotBase) {
        std::cerr << "[Relay] FATAL: 无法获取机械臂当前位姿，拒绝运动控制" << std::endl;
        std::cerr << "[Relay] 请检查机械臂连接状态后重试" << std::endl;
        robotDisconnect();
        m_stateMachine.onEnableFail();
        return false;
    }

    m_stateMachine.onEnableSuccess();

    // 刷新心跳基准时间戳，避免 init() 耗时超过 HEARTBEAT_TIMEOUT_MS
    // 导致 queryPose() 首次触发时立即误判心跳超时 → FATAL
    m_lastHeartbeatMs = GetTickCount();

    return true;
}

void RelayCore::shutdown() {
    shutdownForceReader();
    robotSendEnable("DisableRobot()");
    Sleep(100);
    robotDisconnect();
    m_stateMachine.onDisconnect();
}

void RelayCore::sendPosition(const hduVector3Dd& devicePos) {
    if (!m_transmitting || !m_basePointSet || !isRobotConnected()) return;

    // ===== 安全守卫 =====
    if (!appState.isRobotBaseSet) return;

    // ⛔ 2026-09-22 移走了: 这里从前写着 `m_lastHapticFrameMs = GetTickCount();`。
    //   【为什么它在这里是错的】本行在守卫 `:843` 之后，而本函数【只在 isTransmitting() 为真时
    //   才被调用】(HapticCallback.cpp:134) ⇒ 那个"心跳"实际是【正在下发】的心跳，
    //   **不是触觉线程的心跳** —— 名字与事实不符。
    //   【后果】一松手就停刷新 ⇒ 看门狗 (本文件 :309, 阈值 `WATCHDOG_TIMEOUT_MS*2`=400ms)
    //   只要求 isTransmitting() 为真 ⇒ 第二次按下时读到的是【上一次下发时】那个陈旧时间戳
    //   ⇒ 把"没有在下发"误判成"GLUT 死了" ⇒ EmergencyStop。
    //   ★ 它同时是"分不清真假"的原因: 真实停摆与"没在下发"在同一个时间戳上长得一模一样。
    //   ⇒ 心跳现在由 `markHapticFrame()` 在【触觉回调入口】无条件刷新 (见该函数)。

    // ===== ServoP 频率限制: 30Hz =====
    DWORD now = GetTickCount();
    if (now - m_lastServoTime < 33) return;
    m_lastServoTime = now;

    // ===== 增量式位移: 每帧计算 Touch 相对于上一帧的微小位移 =====
    Vec3 current = convertTouchToRobot(devicePos);

    EnterCriticalSection(&m_basePointLock);
    if (!m_lastTouchValid) {
        // 第一帧: 初始化参考点
        m_lastTouchPos = current;
        m_lastTouchValid = true;
        LeaveCriticalSection(&m_basePointLock);
        return;
    }

    // Compute position delta (for use outside orient mode only)
    double dx = current.x - m_lastTouchPos.x;
    double dy = current.y - m_lastTouchPos.y;
    double dz = current.z - m_lastTouchPos.z;
    m_lastTouchPos = current;  // always update touch reference

    Vec3 clamped = m_targetPos;  // default: fixed TCP (orientation-only mode)

    // Position delta: active when button1 is held (alone or combined with button2)
    if (appState.lastButtonState) {

        // NaN/Inf guard: 连续 3 帧异常 → FATAL
        if (std::isnan(dx) || std::isnan(dy) || std::isnan(dz) ||
            std::isinf(dx) || std::isinf(dy) || std::isinf(dz)) {
            m_nanFrameCount++;
            if (m_nanFrameCount >= 3) {
                RobotError error;
                error.code = RobotErrorCode::ERR_EMERGENCY_STOP;
                error.severity = Severity::FATAL;
                error.timestampMs = GetTickCount64();
                Vec3 zeroDelta = {0, 0, 0};
                m_stateMachine.onError(error, zeroDelta);
            }
            LeaveCriticalSection(&m_basePointLock);
            return;
        }
        m_nanFrameCount = 0;  // 正常帧清零

        // 跳过微小增量 (Touch 噪声)
        if (fabs(dx) < 0.05 && fabs(dy) < 0.05 && fabs(dz) < 0.05) {
            LeaveCriticalSection(&m_basePointLock);
            return;
        }

        // ===== 增量步长限制: 单步最大 3mm, 再乘以速度衰减因子 =====
        static double s_speedMul = 1.0;  // 跨帧持久, 由 SafetyPredictor 更新
        double len = sqrt(dx*dx + dy*dy + dz*dz);
        // Apply state machine speed factor ON TOP of safety verdict
        double effectiveSpeed = std::min(s_speedMul, m_stateMachine.speedFactor());
        double maxStep = 4.5 * effectiveSpeed;
        if (len > maxStep) {
            double scale = maxStep / len;
            dx *= scale; dy *= scale; dz *= scale;
        }

        // 计算候选位置 (先不更新 m_targetPos)
        Vec3 candidate;
        candidate.x = m_targetPos.x + dx;
        candidate.y = m_targetPos.y + dy;
        candidate.z = m_targetPos.z + dz;

        // 安全边界钳位
        clamped = SafetyBoundary::clampToBoundaryActive(candidate);

        // ===== SafetyPredictor 预判 (先评估，后更新，防止边界漂移) =====
        SafetyVerdict verdict = SafetyPredictor::instance().evaluate(clamped);

        // State machine: escalate on warning
        if (verdict.errorCode != RobotErrorCode::OK
            && verdict.errorCode != RobotErrorCode::ERR_IK_NO_SOLUTION
            && verdict.errorCode != RobotErrorCode::ERR_JOINTLIMIT_WARN) {
            Vec3 deltaVec(dx, dy, dz);
            RobotError error = SafetyPredictor::instance().lastError();
            m_stateMachine.onError(error, deltaVec);
        } else {
            // Check for reverse-motion de-escalation (immediate recovery)
            Vec3 deltaVec(dx, dy, dz);
            auto& esc = m_stateMachine.escalation();
            if (esc.isEscalated() && esc.shouldDeescalate(deltaVec, esc.lastRejectDirection)) {
                m_stateMachine.onRecovery();
            }
            m_stateMachine.escalation().onClear();
        }

        // Check if state machine allows motion
        if (!m_stateMachine.canMove()) {
            LeaveCriticalSection(&m_basePointLock);
            return;
        }

        if (verdict.action == SafetyVerdict::REJECT) {
            std::cerr << "[Safety] REJECT: " << verdict.reason
                      << " — candidate=(" << clamped.x << "," << clamped.y << "," << clamped.z << ")"
                      << std::endl;
            LeaveCriticalSection(&m_basePointLock);
            return;  // 不更新 m_targetPos，下帧从同一位置重新计算
        }

        // 更新速度衰减因子 (用于下帧)
        s_speedMul = (verdict.action == SafetyVerdict::WARN_SLOW) ? verdict.speedFactor : 1.0;
    }

    // ===== 姿态计算 (优化或用户控制) =====
    auto& app = appState;
    // Default to robot's current actual orientation (not startup base),
    // so IK failure or mode switch doesn't snap back to a stale pose.
    double targetRx, targetRy, targetRz;
    {
        EnterCriticalSection(&app.robotPoseMutex);
        targetRx = app.robotActualPose.rx;
        targetRy = app.robotActualPose.ry;
        targetRz = app.robotActualPose.rz;
        LeaveCriticalSection(&app.robotPoseMutex);
    }

    // Mode 1: Position-only (button1, no button2) — optimize orientation
    if (appState.lastButtonState && !m_transmittingOrient) {
        double curJoints[6];
        {
            EnterCriticalSection(&app.robotPoseMutex);
            curJoints[0] = app.robotActualPose.j1;
            curJoints[1] = app.robotActualPose.j2;
            curJoints[2] = app.robotActualPose.j3;
            curJoints[3] = app.robotActualPose.j4;
            curJoints[4] = app.robotActualPose.j5;
            curJoints[5] = app.robotActualPose.j6;
            LeaveCriticalSection(&app.robotPoseMutex);
        }
        Vec3 optOrient = SingularityAvoidance::optimizeOrientation(clamped, curJoints);
        if (optOrient.x != 0.0 || optOrient.y != 0.0 || optOrient.z != 0.0) {
            targetRx = optOrient.x;
            targetRy = optOrient.y;
            targetRz = optOrient.z;
        }
        // else: IK failed — keep current actual orientation (no snap-back)
    }

    Vec3 damped(0.0, 0.0, 0.0);  // orientation delta for cross-mode sharing

    if (m_transmittingOrient && m_orientValid) {
        // Read current stylus orientation (thread-safe)
        double stylusRx, stylusRy, stylusRz;
        EnterCriticalSection(&app.stylusOrientMutex);
        stylusRx = app.stylusOrient[0];
        stylusRy = app.stylusOrient[1];
        stylusRz = app.stylusOrient[2];
        LeaveCriticalSection(&app.stylusOrientMutex);

        Vec3 current(stylusRx, stylusRy, stylusRz);

        // ★★★ 2026-09-21: 从【累加式】改成【参照式】(相对按下点的绝对偏移)。
        //
        // 【为什么 —— 用户的话说得对】只要目标是"把逐帧增量累加起来", 手抖 (永远非零、而且
        //   因人而异) 就一定会被积进去 ⇒ **死区调多大都只是把漂移调慢, 治不了它**;
        //   而"去标定每个人的抖动"既不可靠也不该做。
        //   现场实测印证了量级问题: 移动手写笔时笔杆朝向只差 0.9°, 机器人姿态却转了 10.4° ——
        //   方向对得上轴重映射 (笔杆 Rz → 机器人 Ry, 见下面那段), 所以【映射没错】;
        //   错的是【量级】: 手一动笔杆就抖, 而抖动被"三轴一起放行 + 累加进目标"放大了。
        //   ⚠ 【2026-09-22 订正】此处原写"无界累加"—— "无界"是【错的】: 目标由下面的
        //     clampOrientToBounds 逐帧钳位, 积分到 ±180 就被钉住, 不是"越走越大"
        //     (钳位自 2026-07-26 e63c0b3 起就在, 早于本次改动)。"累加会把任何非零抖动
        //     积进去"这段仍是本改动的理由, 不受影响。详见 main.cpp "笔杆姿态"那段。
        //
        // 【现在】 目标 = 【按下按钮2时的机器人姿态】 + R×(K × (笔杆现在 − 笔杆按下时))
        //   · 抖动 ⇒ 目标只【颤动】(有界、自回), **与抖动大小无关 ⇒ 不漂** ✓
        //   · 手腕转到底 ⇒ 【松手再按 = 重新索引】 (onButton2Press 已经重设这组参照 ✓)
        //   · 死区从此只是"笔杆要转多少机器人才开始跟"的【响应门限】, 不再是防漂的关键参数
        //     ⇒ **不再需要按每个人的抖动去标定它** ✓
        //
        // ⚠ 这组参照 (m_orientRefStylus / m_orientRefRobot) 【本来就有】—— 它的原注释写着
        //   "stored for diagnostics/re-sync and logged on press — they are NOT used in the
        //    per-frame delta computation"。**本改动就是把它们从"只存着"变成"真的在用"。**
        double offx = current.x - m_orientRefStylus.x;
        double offy = current.y - m_orientRefStylus.y;
        double offz = current.z - m_orientRefStylus.z;

        // ★★ 2026-09-22 实验之二：偏移的一阶低通（理由见 Config::ORIENT_STYLUS_LPF_ENABLED）。
        // 【为什么滤的是"偏移"而不是"机器人目标"】抖动来自【输入】，在输入端压掉才不会
        //   在机器人侧留下任何痕迹；滤目标等于在输出端追着改，永远慢一拍。
        // ⚠ α 用【实测 dt】：本函数被 ServoP 的 30 Hz 节流着，但节流值会随改动漂，别写死 0.033。
        if (Config::ORIENT_STYLUS_LPF_ENABLED) {
            const DWORD nowMs = GetTickCount();
            if (!s_stylusOffFiltReady) {
                // 按下按钮2 后的第一帧：直接把滤值对齐到当前偏移（不从 0 慢慢爬）
                s_stylusOffFilt[0] = offx; s_stylusOffFilt[1] = offy; s_stylusOffFilt[2] = offz;
                s_stylusOffFiltLastMs = nowMs;
                s_stylusOffFiltReady = true;
            } else {
                const double dt = (double)(nowMs - s_stylusOffFiltLastMs) / 1000.0;
                s_stylusOffFiltLastMs = nowMs;
                if (dt <= 0.0) {
                    // 同一 tick 内重复调用（节流本该挡住）—— 不推进相位，避免 α=0 的除零/停滞
                } else if (dt > Config::ORIENT_STYLUS_LPF_MAX_GAP_S) {
                    // 隔了很久（卡顿/断连恢复）：重新对齐，别把陈旧姿态当增量补进来
                    s_stylusOffFilt[0] = offx; s_stylusOffFilt[1] = offy; s_stylusOffFilt[2] = offz;
                } else {
                    const double a = dt / (Config::ORIENT_STYLUS_LPF_TAU_S + dt);
                    s_stylusOffFilt[0] += a * (offx - s_stylusOffFilt[0]);
                    s_stylusOffFilt[1] += a * (offy - s_stylusOffFilt[1]);
                    s_stylusOffFilt[2] += a * (offz - s_stylusOffFilt[2]);
                }
            }
            offx = s_stylusOffFilt[0];
            offy = s_stylusOffFilt[1];
            offz = s_stylusOffFilt[2];
        }

        // 逐轴响应门限 (低于它本帧这一轴不动; 只影响颤动幅度, 不影响漂移)
        const double dz = Config::ORIENT_DEADZONE_DEG;
        auto axisGate = [dz](double d) -> double {
            return (fabs(d) >= dz) ? d : 0.0;
        };
        double drx = axisGate(offx);
        double dry = axisGate(offy);
        double drz = axisGate(offz);

        // ★★ 2026-09-23 (Task 2)：关节空间路径要的"**当前**笔杆姿态"就是这里 ——
        //   参照 + 【已过死区(drx/dry/drz)、已低通(上面的 s_stylusOffFilt)】的偏移。
        //   ⚠ 这正是下面 RPY 路径用的 `curFiltered`（同样的三个分量相加），**不是**另一个量。
        //     之所以不写在那一处：那一处在 `BTN2_ROTATION_COMPOSE_ENABLED` 分支【里面】
        //     ⇒ 哪天 RPY 开关翻回 false，**关节路径的入参也会跟着冻住** —— 那是静默耦合
        //     （关掉 A 却改了 B 的行为）。放这里 ⇒ 两条路径共用同一份"过门+低通"的中间量。
        //   ⚠ NaN：`dr*` 是 NaN 时这里会把 NaN 抄进去 —— **无害**，且不靠巧合：
        //     `button2JointTarget` 自己的守卫（Button2Joint.cpp 的 allFinite3）会让返回值
        //     **退回参照** ⇒ 关节不动、正解出的位置也有限。⇒ NaN 到不了 ServoJ，也到不了安全门。
        //   ⚠ 死区把三轴全归 0 时，这里写入的就是 `m_orientRefStylus` 本身 ⇒ 纯函数返回
        //     逐位等于 `m_jointRef` 的目标 ⇒ 机械臂**原地保持**（不是"这一帧不下发"）。
        m_btn2StylusFilt[0] = m_orientRefStylus.x + drx;
        m_btn2StylusFilt[1] = m_orientRefStylus.y + dry;
        m_btn2StylusFilt[2] = m_orientRefStylus.z + drz;

        // ⚠ m_lastStylusOrient 从本改动起【只用于诊断】(记录最近一次原始读到的笔杆姿态),
        //   控制回路不再读它 —— 别再把它当成"增量式参照"。
        m_lastStylusOrient = current;

        // NaN/Inf guard: skip this frame's orientation delta (don't increment nan counter)
        if (std::isnan(drx) || std::isnan(dry) || std::isnan(drz) ||
            std::isinf(drx) || std::isinf(dry) || std::isinf(drz)) {
            // orientation delta skipped for this frame only
        } else {
        // ★ 死区已经在上面【逐轴】判过了 (drx/dry/drz 不是 0 就是已过门限的值)。
        //   从前这里是一个【三轴 OR 的门】—— 那正是"只有一轴动、另两轴噪声也放行"的来源。
        //   ⇒ 现在只问一句: 这一帧有没有任何一轴真的用掉了增量?
        if (drx != 0.0 || dry != 0.0 || drz != 0.0) {

            // ★★★ 2026-09-23 (Task 3)：期望目标由【两条路径】之一给出 —— 开关见
            //   Config::BTN2_ROTATION_COMPOSE_ENABLED（默认 true ⇒ 新路径；翻 false = 回滚）。
            //   ⚠ 这一段【不被任何测试编译】：守住它的只有"编译 + 语法错误注入的负对照 + 上机"。
            // ⚠ 初值取【当前目标】而不是 `(0,0,0)`：`(0,0,0)` 是一个**合法但错**的目标姿态
            //   （机械臂真的会朝它转过去）。今天两条路径【都】无条件给 desired 赋值，所以这只
            //   是记账；但只要将来有人在上面加一条早退分支（比如某个开关下"这帧跳过"），
            //   初值就会**静默地**变成那个错目标 —— 而这一段**不被任何测试编译**
            //   （见 Config::BTN2_ROTATION_COMPOSE_ENABLED 那段：守它的只有编译 + 上机）。
            //   取 `m_targetOrient` ⇒ 早退 = "目标不动"，即 `wx=wy=wz=0`，是本帧的安全语义。
            Vec3 desired = m_targetOrient;
            if (Config::BTN2_ROTATION_COMPOSE_ENABLED) {
                // ---- 新路径：真旋转合成（纯函数，见 relay/Button2Mapping.{h,cpp}）----
                //   · 喂进去的是"当前笔杆姿态" = 参照 + 【已过门、已低通】的偏移（drx/dry/drz）。
                //     死区与低通仍作用在【笔杆偏移】上，语义不变 —— 它们【不】进纯函数
                //     （搬进去会改掉它们已被现场验证过的语义，还会把它们拖出可测范围）。
                //   · 偏移的限幅在【函数内部】按【旋转角】做完（≤ ORIENT_MAX_OFFSET_DEG）
                //     ⇒ 这条路径【不】再调 clampOrientOffset（逐分量那个，见下面的钳位段）。
                //   · 三个 Vec3 各自【显式拷贝】成局部 double[3]，不写 `&m_orientRefRobot.x`：
                //     那前提是"Vec3 的三个成员在内存里连续且按 x,y,z 排列"，而这个前提
                //     **没有任何东西保证**（编译器可以对成员各自安排）—— Task 1 已因同类
                //     前提被要求改写法。显式拷贝没有前提。
                Vec3 curFiltered(m_orientRefStylus.x + drx,
                                 m_orientRefStylus.y + dry,
                                 m_orientRefStylus.z + drz);
                double aRef[3] = { m_orientRefRobot.x,  m_orientRefRobot.y,  m_orientRefRobot.z  };
                double sRef[3] = { m_orientRefStylus.x, m_orientRefStylus.y, m_orientRefStylus.z };
                double sCur[3] = { curFiltered.x,       curFiltered.y,       curFiltered.z       };
                desired = button2OrientationTarget(aRef, sRef, sCur);

                // ⚠ 记账（Task 3 附加要求②，完整说明在 Button2Mapping.h 的调用约定里）：
                //   纯函数只用【兜底】那张表 M，**标定路径没接**。⇒ 一旦 calibration.json 存在
                //   （Calibration::enabled = true），平移路径走标定出来的 R/t，而姿态路径仍走
                //   兜底 M ⇒ 两条路【静默】不一致。本仓库最怕静默，所以这里至少让它响一次。
                if (Calibration::enabled && !s_warnedBtn2CalibIgnored) {
                    s_warnedBtn2CalibIgnored = true;
                    std::cerr << "[Orient] Calibration is enabled, but the button-2 pose mapping still "
                                 "uses the fallback axis table M (calibrated path NOT wired) -- "
                                 "translation and orientation will disagree. See relay/Button2Mapping.h."
                              << std::endl;
                }
            } else {
                // ---- 旧路径：Euler 角之差当旋转向量 + 逐分量加到参照上（**逐字保留**，只缩进）----
                // Apply gain
                drx *= Config::ORIENT_GAIN;
                dry *= Config::ORIENT_GAIN;
                drz *= Config::ORIENT_GAIN;

                // ★ 单帧限幅【不在这里】(2026-09-21 参照式改造)。
                //   参照式下 drx/dry/drz 是"相对按下按钮2那一点的偏移"——**可以很大, 而且那是对的**:
                //   操作员转了 30°, 就该给 30°。从前这里限的是【偏移】, 那是累加式时代的写法,
                //   会把大转动【永久截断】掉。
                //   ⇒ 限幅改到【目标姿态每帧的变化】上 (见下面 wx/wy/wz 那一段) —— 那才是"手一甩
                //     不让机器人跟着猛转"要限的量。

                // ★★ 逐轴符号 (2026-09-21 改)。从前这里是【无条件三轴取负】, 注释的理由是
                //   "Touch Euler (ZYX intrinsic) 沿正轴看逆时针增大, 而 Dobot RPY 相反"。
                //   那个理由与仓库里的两份实现都不符 —— Touch 侧的 Euler 提取与 Dobot 侧的
                //   TcpCalibration::rpyToMatrix **都是 Rz·Ry·Rx**; 而现场实测也是"转向反了"。
                //   ⇒ 默认改为【不翻转】。完整依据、以及"若只有某些轴反而是置换问题"的处置,
                //     见 Config::ORIENT_FLIP_RX 那一大段。
                drx *= Config::ORIENT_FLIP_RX;
                dry *= Config::ORIENT_FLIP_RY;
                drz *= Config::ORIENT_FLIP_RZ;

                // ---- Axis remap: stylus frame → robot frame ----
                // Build 3×3 rotation that maps Touch rotation axes to robot rotation axes.
                // When calibration is enabled, use the calibrated rigid transform R.
                // Fallback: hardcoded axis mapping matching convertTouchToRobot():
                //   robot_X = touch_X   → [1, 0,  0]
                //   robot_Y = -touch_Z  → [0, 0, -1]
                //   robot_Z = touch_Y   → [0, 1,  0]
                double R00, R01, R02, R10, R11, R12, R20, R21, R22;
                if (Calibration::enabled) {
                    R00 = Calibration::R[0]; R01 = Calibration::R[1]; R02 = Calibration::R[2];
                    R10 = Calibration::R[3]; R11 = Calibration::R[4]; R12 = Calibration::R[5];
                    R20 = Calibration::R[6]; R21 = Calibration::R[7]; R22 = Calibration::R[8];
                } else {
                    // 与平移路径同一张表（唯一一份定义）—— 见 CoordinateTransform::touchToRobotMatrix
                    // ⚠ 这里【故意】用显式局部数组 + 逐个赋值，而不是 touchToRobotMatrix(&R00):
                    //   "R00..R22 这 9 个 double 在内存里连续" 是一个【未验证的前提】(编译器可以对
                    //   局部标量各自安排位置), 而显式赋值不需要任何前提。
                    double M[9];
                    touchToRobotMatrix(M);
                    R00 = M[0]; R01 = M[1]; R02 = M[2];
                    R10 = M[3]; R11 = M[4]; R12 = M[5];
                    R20 = M[6]; R21 = M[7]; R22 = M[8];
                }
                double robot_dRx = R00*drx + R01*dry + R02*drz;
                double robot_dRy = R10*drx + R11*dry + R12*drz;
                double robot_dRz = R20*drx + R21*dry + R22*drz;

                // ★★★ 期望目标 = 【按下按钮2时的机器人姿态】+ 映射后的偏移 —— 【不是累加】。
                //   `m_orientRefRobot` 是 onButton2Press 时抓的那一份机器人姿态, 在整个按住期间不变。
                //   ⇒ 抖动只让 desired 在参照附近【颤动】, 手一回它自己就回去 ⇒【不漂】,
                //     而且与抖动多大无关 (这就是这套改法的全部意义)。
                desired = Vec3(m_orientRefRobot.x + robot_dRx,
                               m_orientRefRobot.y + robot_dRy,
                               m_orientRefRobot.z + robot_dRz);
            }

            // 本帧要走的量 = 期望 − 当前目标, 再【逐轴限幅】。
            // 保护的是"手一甩不会让机器人跟着猛转"; 因为目标是朝 desired 收敛的,
            // 限幅只会让它【慢慢跟上】, 不会像从前那样把量永久截断掉。
            double wx = desired.x - m_targetOrient.x;
            double wy = desired.y - m_targetOrient.y;
            double wz = desired.z - m_targetOrient.z;
            if (wx >  Config::ORIENT_MAX_STEP_DEG) wx =  Config::ORIENT_MAX_STEP_DEG;
            if (wx < -Config::ORIENT_MAX_STEP_DEG) wx = -Config::ORIENT_MAX_STEP_DEG;
            if (wy >  Config::ORIENT_MAX_STEP_DEG) wy =  Config::ORIENT_MAX_STEP_DEG;
            if (wy < -Config::ORIENT_MAX_STEP_DEG) wy = -Config::ORIENT_MAX_STEP_DEG;
            if (wz >  Config::ORIENT_MAX_STEP_DEG) wz =  Config::ORIENT_MAX_STEP_DEG;
            if (wz < -Config::ORIENT_MAX_STEP_DEG) wz = -Config::ORIENT_MAX_STEP_DEG;

            Vec3 robotDelta(wx, wy, wz);

            // Get current joints for avoidance computation
            double curJoints[6];
            {
                EnterCriticalSection(&app.robotPoseMutex);
                curJoints[0] = app.robotActualPose.j1;
                curJoints[1] = app.robotActualPose.j2;
                curJoints[2] = app.robotActualPose.j3;
                curJoints[3] = app.robotActualPose.j4;
                curJoints[4] = app.robotActualPose.j5;
                curJoints[5] = app.robotActualPose.j6;
                LeaveCriticalSection(&app.robotPoseMutex);
            }

            Vec3 tcpAdj, repulsionOut;
            Vec3 currentTcp(clamped.x, clamped.y, clamped.z);

            damped = SingularityAvoidance::dampOrientationMotion(
                m_targetOrient, robotDelta, currentTcp, curJoints, tcpAdj, repulsionOut);

            // Apply damped orientation delta
            m_targetOrient.x += damped.x;
            m_targetOrient.y += damped.y;
            m_targetOrient.z += damped.z;
            // ★★ 2026-09-22 实验：钳位改成比【相对参照的偏移】(理由见 clampOrientOffset)。
            //   翻回 false ⇒ 走原来的绝对值钳位 (那条会在参照贴着 ±180 时逐帧夹 ⇒ 手一抖就抖)。
            // ★ 2026-09-23 (Task 3)：逐分量那条钳位**只给旧路径** —— 新路径的偏移限幅已经在
            //   `button2OrientationTarget` 内部按【旋转角】做完（≤ ORIENT_MAX_OFFSET_DEG），
            //   再逐分量夹一次会与那个语义打架（把"合成的旋转"按表示量切掉一块）。
            //   `clampOrientToBounds`（绝对值 ±180 的安全钳位）两条路径都保留、语义不动。
            //   ⚠ 这条 `!BTN2_ROTATION_COMPOSE_ENABLED` 今天【不改变任何行为】：
            //     ORIENT_SEAM_FIX_ENABLED 是 false（那条修法 2026-09-22 现场证否，见 Config.h）。
            //     加它是因为 SEAM_FIX 的文档写着"翻回 true 即可重试" —— 不写这一句，那个回滚
            //     会在新路径上偷偷把逐分量限幅又接回来（本仓库最怕这种静默的组合）。
            if (Config::ORIENT_SEAM_FIX_ENABLED && !Config::BTN2_ROTATION_COMPOSE_ENABLED) {
                m_targetOrient = clampOrientOffset(m_targetOrient, m_orientRefRobot);
            } else {
                m_targetOrient = clampOrientToBounds(m_targetOrient);
            }

            // Apply TCP micro-adjust (position mode: locked; orient mode: micro-adjust)
            if (appState.lastButtonState && m_transmittingOrient) {
                // Combined mode: don't adjust position here (handled by dampFullCommand)
            } else if (m_transmittingOrient) {
                // Orientation-only: apply TCP micro-adjust
                clamped.x += tcpAdj.x;
                clamped.y += tcpAdj.y;
                clamped.z += tcpAdj.z;
            }

            // Phase 2: write directional repulsion force to AppState for haptic callback
            {
                EnterCriticalSection(&app.orientRepulsionMutex);
                app.orientRepulsionForce[0] = repulsionOut.x;
                app.orientRepulsionForce[1] = repulsionOut.y;
                app.orientRepulsionForce[2] = repulsionOut.z;
                app.hasOrientRepulsion = true;
                LeaveCriticalSection(&app.orientRepulsionMutex);
            }
        }

        targetRx = m_targetOrient.x;
        targetRy = m_targetOrient.y;
        targetRz = m_targetOrient.z;
        }  // end !NaN guard
    }

    // ===== Position update (only when NOT in orientation mode) =====
    // When m_transmittingOrient is true, the TCP position is frozen —
    // we compute it below from the fixed wrist center + rotated offset.
    if (!m_transmittingOrient) {
        // 通过安全检查后才更新目标位置
        m_targetPos = clamped;
    }
    LeaveCriticalSection(&m_basePointLock);

    // ===== Compute ServoP position =====
    // Position always comes from m_targetPos (via 'clamped'):
    //   - position mode (button1):     updated by Touch delta above
    //   - orientation-only (button2):  frozen at press-time TCP capture
    //   - combined (button1+2):        updated by Touch delta + orientation accumulation
    double servoCmdX = clamped.x;
    double servoCmdY = clamped.y;
    double servoCmdZ = clamped.z;

    // During orientation mode, validate the TCP position (no IK — position-only checks)
    if (m_transmittingOrient && m_orientValid) {
        Vec3 tcpCheck(servoCmdX, servoCmdY, servoCmdZ);
        SafetyVerdict ov = SafetyPredictor::instance().evaluatePositionOnly(tcpCheck);
        if (ov.action == SafetyVerdict::REJECT) {
            std::cerr << "[Safety] Orient TCP REJECT: " << ov.reason
                      << " — tcp=(" << servoCmdX << "," << servoCmdY << "," << servoCmdZ << ")"
                      << std::endl;
            return;
        }

        // Amplify singular constraint force in orientation mode
        double extraForce[3];
        ConstraintForce::computeSingularForce(
            Vec3(servoCmdX, servoCmdY, servoCmdZ),
            extraForce,
            Config::SINGAVOID_ORIENT_FORCE_AMP);
        EnterCriticalSection(&app.orientForceMutex);
        app.orientExtraForce[0] = extraForce[0];
        app.orientExtraForce[1] = extraForce[1];
        app.orientExtraForce[2] = extraForce[2];
        app.hasOrientExtraForce = true;
        LeaveCriticalSection(&app.orientForceMutex);
    }

    // Mode 3: Combined position+orientation — SVD-based selective damping
    if (appState.lastButtonState && m_transmittingOrient && m_orientValid) {
        // Both deltas were computed in this frame
        // Build the user's 6-DOF delta from what was actually applied
        Vec3 posDelta(
            clamped.x - m_targetPos.x,
            clamped.y - m_targetPos.y,
            clamped.z - m_targetPos.z
        );
        Vec3 orientDelta(damped.x, damped.y, damped.z);  // pre-damped by Mode 2

        double curJoints[6];
        {
            EnterCriticalSection(&app.robotPoseMutex);
            curJoints[0] = app.robotActualPose.j1;
            curJoints[1] = app.robotActualPose.j2;
            curJoints[2] = app.robotActualPose.j3;
            curJoints[3] = app.robotActualPose.j4;
            curJoints[4] = app.robotActualPose.j5;
            curJoints[5] = app.robotActualPose.j6;
            LeaveCriticalSection(&app.robotPoseMutex);
        }

        Vec3 dampedPos, dampedOrient;
        SingularityAvoidance::dampFullCommand(posDelta, orientDelta, curJoints,
                                              dampedPos, dampedOrient);

        // Reconstruct clamped position from damped delta
        clamped.x = m_targetPos.x + dampedPos.x;
        clamped.y = m_targetPos.y + dampedPos.y;
        clamped.z = m_targetPos.z + dampedPos.z;

        // Update servoCmd after Mode 3 modifies clamped (Bug 2 fix)
        servoCmdX = clamped.x;
        servoCmdY = clamped.y;
        servoCmdZ = clamped.z;

        // Bug 3 fix: undo Mode 2's orientation contribution, apply Mode 3's combined damping.
        // Mode 2 already added 'damped' to targetRx, so we subtract it before adding 'dampedOrient'.
        targetRx = targetRx - damped.x + dampedOrient.x;
        targetRy = targetRy - damped.y + dampedOrient.y;
        targetRz = targetRz - damped.z + dampedOrient.z;
    }

    // ===== 构造并发送 ServoP =====
    // ★★ 2026-09-22 实验：下发前把三个角规范化到 [-180,180)（见 Config::ORIENT_SEAM_FIX_ENABLED）。
    // 【为什么必须要这一步】上一步把钳位改成了"相对参照"，于是 `m_targetOrient` 现在是
    //   **连续累加、会越过 ±180** 的量（那是刻意的：工具确实在连续滚）—— 而它直接发给 ServoP
    //   就会送出 `183` 这种数。规范化是**等价的朝向**（183 ≡ −177），按矩阵做 IK 的控制器得到同解。
    // ⚠ 对 Mode 1/3 那几条路是**无操作**：它们的值来自 `robotActualPose` 或 IK 解，本就在范围内。
    // ⚠ 就地改 `targetRx` 而不是只用副本：下面 :1223 的日志行与 :1247 的 `robotTargetPose`
    //   都要跟着走 —— 否则界面会显示"目标 183 / 实际 −177"，看的人以为出错了。
    if (Config::ORIENT_SEAM_FIX_ENABLED) {
        targetRx = normalizeDeg180(targetRx);
        targetRy = normalizeDeg180(targetRy);
        targetRz = normalizeDeg180(targetRz);
    }

    char cmd[256];

    // ★★★ 2026-09-23 (Task 2)：按钮2 的两条下发路径在**这里**分叉 —— 唯一的开关是
    //   `Config::BTN2_JOINT_SPACE_ENABLED`（**回滚 = 翻 false**，下面 else 那一支就是【逐字】
    //   的旧路径，一字未改）。
    //
    // 【为什么分叉点选在下发前、而不是把上面那整块姿态计算换掉】
    //   开关只决定"**发出去的是什么**"。上面那一段（期望姿态 → 逐帧限幅 → 奇异阻尼 →
    //   `clampOrientToBounds` → `orientRepulsionForce`）**一行不动**，因为其中还挂着
    //   **触觉反馈**（`dampOrientationMotion` 写 `app.orientRepulsionForce`）与 TCP 微调；
    //   删掉它就等于**顺手关掉了奇异避让**，而那是明确要求"一条都不许删"的那一类。
    //   ⚠ 代价（**如实记账，不藏**）：关节空间路径下那一段仍然在跑，它的输出
    //     `targetRx/Ry/Rz` 只流向 `app.robotTargetPose`（`render/SceneRenderer.cpp:201` 只用来
    //     画目标位姿）—— 也就是说**那一处显示的仍是那条不再下发的 RPY 目标**。真正发出去的是
    //     什么，看 `cmd`（下面的 `reportCommand(cmd)` 与 `app.lastCommandSent` 存的都是 `cmd`
    //     本身 ⇒ 那两处是准的）。
    //   ★ 2026-09-23 复审 Important#1 补记：**同一句话对平移也成立，而且更隐蔽** ——
    //     按钮1 那条平移块写的是 `servoCmdX/Y/Z`（本函数上面 `if (appState.lastButtonState)`
    //     那一段，:891），它同样**照旧更新**、同样**一条都不发**（这一支发出的 `cmd` 是 `ServoJ`）。
    //     ⇒ 关节模式下有**两个**"只显示、不下发"的目标：`servoCmdX/Y/Z`（它又流向下面
    //       `[Relay] Motion sends` 的 `target=` 与 `app.robotTargetPose.x/y/z`）与
    //       `app.robotTargetPose`（画在 GUI 上）。对账的唯一权威仍然是 `cmd`。
    //     ⇒ 行为**一行未改**（锁存是刻意的，见下面关节分支的按钮1 提示）；改的是"让它出声"。
    //   ★ 2026-09-23 复审 I3：`[Relay] Motion sends` 那行**原来只打 target/orient**，在关节模式
    //     下等于**显示与实发不一致**（本项目最恨的那一类）⇒ 已在那行**补上真正发出去的 `cmd`**。
    //     二选一里选的是这一条（**不是**"关节模式下跳过 `robotTargetPose` 写入"）：
    //     后者会让目标位姿**冻结在上一次按下之前的值**（静止的假象仍会误导），而且会改动
    //     `SceneRenderer` 看到的状态 ⇒ 影响面更大。⇒ **残留（已知、可接受）**：GUI 里画的
    //     目标位姿在关节模式下仍是那条 RPY 目标；对账的唯一权威是 `cmd`。
    //   ⇒ 措辞订正（2026-09-23 复审 M1 —— 原文写的是"新路径**不调** `clampOrientToBounds`"，
    //     那是**假的**）：它**每帧仍在执行**（`:1268`，那个共享的 RPY 块在下面这个分叉【之前】）。
    //     关节空间里确实没有 RPY 表示、没有 ±180 接缝，所以"夹表示量"这件事对**下发值**
    //     没有意义；消失的只是它**对下发值的影响** —— 那一整段本身**一条没删**（旧路径要它，
    //     触觉反馈也挂在里面）。
    //
    // ⚠⚠ 守不住什么：`RelayCore.cpp` **不被任何测试编译** ⇒ 这个分叉**没有自动化用例**。
    //   能守它的只有三样：① MSBuild 零 error；② 负对照（往本文件注入语法错误 ⇒ 必须报
    //   `error C…`，证明这次构建确实在编它）；③ 上机（Task 3 的执行单）。纯函数那一半有单测
    //   （`tests/test_button2_joint.cpp`），但"**接线接对了没有**"只能靠上机。
    //   ⚠ 本条在 2026-09-23 修复轮又长了一块（I1 参照可信度 · I2 关节限位 · M2 逐帧步长限幅），
    //     同样**一条都没有自动化用例** ⇒ 只能靠人读 + 上机（见修复报告）。
    // ⚠ **只按按钮2**（不含按钮1+2 组合）：组合模式要**同时**做平移，而**平移在关节空间里不是
    //   一个关节量** ⇒ 要支持它就得回到 IK ⇒ 正是本方案要绕开的东西 ✗。
    //   ⇒ 组合模式**继续走旧的 RPY 路径**（= 保持现状，**不引入功能回退** ✓）；这是有理由的取舍，
    //     不是遗漏（2026-09-23 控制方裁定）。
    //
    // ★★★ 复审 C1 (Critical)：下面判的是**按下那一刻锁存的** `m_btn2JointMode`，
    //   **不是**每帧重算 `Config::BTN2_JOINT_SPACE_ENABLED && !appState.lastButtonState`。
    //   那个写法会在**按住中途换控制律**（真值随按钮1 变化而翻面）：
    //     · 松开按钮1 ⇒ 条件变真 ⇒ `ServoJ(m_jointRef + δ)` 把臂**拽回按下按钮2 时的位姿**
    //       （组合模式平移出去多远都白搭；那条 FK 位置门抓不到 —— 它校验的目标就是那个位姿）；
    //     · 先按2 再按1 ⇒ 反过来翻成 `ServoP` ⇒ 姿态突变。
    //   锁存点：`onButton2Press`（与 `m_jointRef` 同一个临界区）。
    if (m_btn2JointMode && m_transmittingOrient && m_orientValid) {
        // ================= 复审 Important#1：按钮1 在这个模式里被【静默忽略】=================
        // 【场景】先单按按钮2（锁存关节模式）⇒ **按住不放**再按按钮1。按钮1 那条平移块
        //   （本函数上面 `if (appState.lastButtonState)` 那一段，:891）**照旧在跑**：它更新
        //   `servoCmdX/Y/Z`、过安全门、推进速度衰减 —— 而本分支**看都不看它们**（下面发的是
        //   `ServoJ`）⇒ 平移**什么都没发生**，而且**一声不响**。
        //   ⚠ 这正是本项目最恨的"显示值 ≠ 实发值"：GUI 的目标位姿在动、`[Relay] Motion sends`
        //     那行的 `target=` 在动，只有臂不动。
        // 【为什么不改行为】锁存是**刻意的**（C1：模式必须是"这一次按下"的属性 —— 中途换控制律
        //   会把臂拽回按下时的位姿 / 姿态突变），而组合模式要**同时**做平移、平移在关节空间里
        //   又不是关节量 ⇒ 要支持它就得回到 IK ⇒ 正是本方案要绕开的东西（计划已裁定）。
        //   ⇒ 能改的只有**可观察性**与**书面记录**：让操作员当场知道"为什么不动"，
        //     并在下面分叉注释的"代价"那一段把平移也点名（原文只点了 RPY 目标）。
        // 【为什么必须一次性】按钮1 一直按着 ⇒ 这个条件**每帧都成立**；不设标记就是
        //   "按住多久刷多久"，而 `cout` 写在触觉回调线程上（见 `m_btn2JointBtn1Noticed` 的注释）。
        //   ⚠ 标记复位在 `onButton2Press` ⇒ 下一次按住会再报一次。
        if (appState.lastButtonState && !m_btn2JointBtn1Noticed) {
            m_btn2JointBtn1Noticed = true;
            std::cout << "[Relay] Btn2 JOINT hold: button1 pressed — TRANSLATION IGNORED until re-press. "
                      << "servoCmdX/Y/Z and robotTargetPose keep updating but this hold is latched to "
                      << "ServoJ (a joint target has no translation, so none of it is sent). "
                      << "Release and re-press button 2 to go back to combined (RPY) mode."
                      << std::endl;
        }

        // ================= 新路径：关节空间（厂商 ServoJ）=================
        // ① 关节目标：Task 1 的纯函数（**有单测**）——
        //      笔杆 Rx(前后摆)⇒J4 · Rz(左右摆)⇒J5 · Ry(自转)⇒J6；J1/J2/J3 无条件保持参照；
        //      死区/符号/限幅的语义都在函数内部（见 relay/Button2Joint.h 的「管线」与「单位变了」）。
        //    入参按位置读：`m_jointRef` 是 j1..j6，`m_btn2StylusFilt` 是笔杆的 **Rx,Ry,Rz**
        //    （即 `stylusOrient` 的次序）。⚠ 两个次序**不同**，且函数内部的映射是**错开**的
        //    （out[4]←Rz、out[5]←Ry）⇒ 这里写错次序不会报错，只会让"前后摆变成左右摆"。
        //    ⚠ 直传 `m_jointRef` / `m_btn2StylusFilt`：它们是**真的 double 数组**，退化成
        //      指针是标准行为。（对照：下面 RPY 路径要从 `Vec3` 取 double[3] 就必须**显式拷贝**
        //      —— "Vec3 的三个成员在内存里连续"是个没有依据的前提，本仓已因同类前提翻过车。）
        double j[6];
        double sRef[3];
        sRef[0] = m_orientRefStylus.x;
        sRef[1] = m_orientRefStylus.y;
        sRef[2] = m_orientRefStylus.z;
        button2JointTarget(m_jointRef, sRef, m_btn2StylusFilt, j);

        // ================= 复审 I1（Important）：参照不可信 ⇒ **拒发** =================
        // 【场景】`app.robotActualPose.j1..j6` 的初值**全 0**，唯一写入点是 `queryJointAngles`
        //   里 `FeedbackParser::parseAngle` 的**成功路径** ⇒ 若 `GetAngle()` **从未解析成功**，
        //   `m_jointRef` 就一直是全 0 ⇒ FK(0,…,0) = (0, −233.3, 756) **过得了下面所有位置门**
        //   ⇒ `ServoJ(0,0,0,0,0,0)` 会把臂**开向模型零位**（一个大而无门可挡的运动）。
        // 【判据】两个都查，任一成立即拒：
        //   · **六位恰好全 0** —— 真机上不可能（关节零点不会同时精确落在 0.0000）；
        //   · `!Kinematics::isWithinJointLimits(ref)` —— 现成函数、**有用例**（`Kinematics.cpp:457`，
        //     `tests/test_kinematics.cpp:322-337`），越限的参照一定不可信。
        // 【动作】`cerr` 出声 + `return`（**本帧不下发**）—— 与下面 `REJECT` 同款。
        // 【守什么】这是"**值**"上的判据，不是"解析成功过没有"那个标志：`m_jointRef` 是按下
        //   那一刻的快照，本分支每帧都在跑 ⇒ 判当前值才是"这一帧要用的那个参照"。
        //   ⚠ 不覆盖 NaN：NaN 在 `isWithinJointLimits` 里逐条比较全为 false ⇒ 它**返回 true**
        //     （放行）。那个情形由下面 ② 的 FK 门接住（FK(NaN) ⇒ NaN ⇒ 第一道守卫 REJECT）。
        // ⚠ 接线这一段（什么时候调、返回值怎么用）**仍然没有自动化用例**（本文件不被任何
        //   测试编译）⇒ 由人读 + 上机。判据**本身**已抽到 `relay/Button2Joint.cpp` 的
        //   `isTrustworthyJointRef`（那里有单测）—— 抽出的只有算术，`return` 留在这里。
        {
            if (!isTrustworthyJointRef(m_jointRef)) {
                // ★ Minor 2：这条**构造上非瞬时** —— 反馈在按下那一刻就坏了 ⇒ `m_jointRef` 是
                //   个坏快照 ⇒ 下面这个条件**整个按住期间恒成立** ⇒ 不压就是一整行、
                //   以 ~30 Hz 的发送速率刷到操作员松手为止。而 `cerr` 写在**触觉回调线程**上，
                //   控制台阻塞（QuickEdit）是本仓已记录的危害 ⇒ **每次按下只报一次**。
                //   ★ 2026-09-23 整支终审 (A) 订正：本段原文接着写的是"与上面/下面那些 `REJECT`
                //     路径不同：它们是**瞬时**的（下一帧状态一变就不报了）⇒ 它们不需要这一手"
                //     —— **那句前提是【假的】**，现已按它自己的理由把 I2 与 FK 那两道也压成一次：
                //       · I2（目标越关节限位）：偏移一直保持很大 ⇒ 目标**一直**越限；
                //       · FK 位置门：笔杆被顶在工作空间边缘 ⇒ FK 目标**一直**被拒。
                //     两条都**整个按住期间恒成立**，都以 ~30 Hz 刷在触觉回调线程上 ⇒
                //     正是让本条压成一次的那条链（控制台写 → QuickEdit 阻塞 → 看门狗 → EmergencyStop）。
                //   ★ (G) "当初为什么只给 I1 压、I2 没压"**不是**因"I2 瞬时"（已证否）——
                //     真正的差别在**可达性**：I1 的坏参照来自一次**冷** `GetAngle()`，
                //     操作员**什么都没做**它就可能成立（`app.robotActualPose.j1..j6` 初值全 0）；
                //     而 I2 的越限目标要求参照**已经坐在一个大关节角上**（`ref ± 150°` 才越界）
                //     ⇒ 那条路上有操作员的动作。次序上 I1 在前，是因为它**最便宜**
                //     （六次比较 + 一次限位查表，不算 FK）。这解释了**为什么 I1 排第一**，
                //     但**不构成**"I2 不必压"—— 两条理由各管各的。
                //   ⚠ 只压消息，**不压 `return`**：被拒的帧每一帧照样不下发（那是安全性质）。
                //   ⚠ 标记复位在 `onButton2Press`（与 `m_btn2JointMode` 同一个临界区）⇒ 下一次按下会再报。
                if (!m_btn2JointRefRejectLogged) {
                    m_btn2JointRefRejectLogged = true;
                    // ⚠ 这里的"理由"是**自己再问一次** `isWithinJointLimits`（判据只回 bool）。
                    //   两个分支**互斥且穷尽**：被拒 且 在限位内 ⇒ 只能是"六位恰好全 0"
                    //   ⇒ 打出来的字符串与抽出前那句 `allZero ? … : …` **逐字相同**。
                    //   ⚠ 2026-09-23 复审订正：上面这句成立**【只因为 0 落在每个关节的量程内】**
                    //     （`robot/Kinematics.h:15-20`：J3 是 ±155，其余五个是 ±360）—— 这是个
                    //     **未写明的前提**，把它写出来：若哪个 `J*_MIN` 被抬到 0 之上，全 0 的参照
                    //     就会落到**另一个**分支、打出 "outside joint limits"（旧代码那里打的是
                    //     "all six joints are exactly 0"）⇒ **只有这句标签不同**，
                    //     而**拒发行为完全一样**（判据只回 bool、`return` 不依赖这句话）。
                    //     ⇒ 即"互斥且穷尽"依赖限位常数，不是依赖 `isWithinJointLimits` 的实现。
                    //   代价：只在**被拒**的那一帧算一次（本来也要 return，可忽略）。
                    std::cerr << "[Safety] Btn2 joint REF UNTRUSTWORTHY ("
                              << (Kinematics::isWithinJointLimits(m_jointRef)
                                      ? "all six joints are exactly 0" : "outside joint limits")
                              << ") — NOT sending. Reported ONCE per press (every frame is still "
                              << "rejected). ref=("
                              << m_jointRef[0] << "," << m_jointRef[1] << "," << m_jointRef[2] << ","
                              << m_jointRef[3] << "," << m_jointRef[4] << "," << m_jointRef[5] << ")"
                              // ★ Minor 3：`m_jointRef` 是**按下那一刻的快照**，按住期间不会自己
                              //   恢复 ⇒ 哪怕 `GetAngle()` 中途开始成功，这个参照照旧是坏的、
                              //   整个按住都是死的。原文只说了"不发"，**没说怎么办** ⇒ 补上
                              //   唯一的那条恢复路径（否则操作员只能等，或者以为坏了）。
                              << ". RELEASE AND RE-PRESS BUTTON 2 — the reference is a press-time "
                              << "snapshot and cannot recover while this hold lasts. "
                              // ★ 2026-09-23 整支终审 (D)：**假拒的角落** —— 若机械臂的**物理零位**
                              //   真的让六个关节读数**恰好全是 0**（那是**合法位姿**，臂就该停在
                              //   那儿按着不动），本判据照样拒发，而上面那句"松开再按按钮2"
                              //   **帮不了**：重按读到的还是同一个全 0 快照 ⇒ 这是**无法自愈**的
                              //   假拒。判据**刻意不改**（放宽成容差会把"停在零位附近"也放行，
                              //   而"恰好全 0"正是 `isTrustworthyJointRef` 要抓的坏参照指纹，
                              //   见 `relay/Button2Joint.cpp:96-102`）⇒ 如实写进消息，
                              //   给出唯一有效的那条出路（把臂挪离零位再按）。
                              << "(If the arm is genuinely parked at its all-zero pose this is a "
                              << "false positive and re-pressing will read all-zero again — nudge "
                              << "one joint off zero first.)"
                              << std::endl;
                }
                return;   // 本帧不下发（下一帧从同一状态重算）
            }
        }

        // ================= 复审 I2（Important）：目标的**关节限位** =================
        // `ref ± 150°`（`Button2Joint.cpp` 的偏移限幅）可能超出 J4/J5/J6 的 ±360 ⇒ 从前只有
        // **控制器事后抱怨**。这里在下发前一行判**期望目标**，越限即 `cerr` + `return`。
        // ⚠ 判的是**期望目标**（纯函数给的那个），不是下面限幅后的值 —— 限幅只是把"走过去"
        //   这件事摊到多帧，它**不改变"目标本身合不合法"**。
        // ⚠ 与 I1 同一句：无自动化用例，由人读 + 上机。
        // ★ 2026-09-23 整支终审 (A)：这条 `cerr` 改成**每次按下只报一次**（标志复位在
        //   `onButton2Press`）—— 它**不是瞬时的**：只要笔杆偏移一直保持很大，目标就
        //   **每帧都越限**，不压就是 ~30 Hz 刷满整个按住期（触觉回调线程上的 `cerr`）。
        //   ⚠ 压的**只有消息**：`return` 仍在外面、**每帧照走**（本帧不下发是安全性质）。
        //   ⚠ 重复没有诊断价值：第一行已经带了**理由**与**位置**。
        //     ★ 2026-09-23 焦点复审 (Minor 1)：**重复的是理由与门，不是数值** —— `j` 是
        //       `ref + clamp(偏移, ±150°)` **每帧重算**的，只有那根轴顶在 ±150° 限幅上时数值才不动，
        //       未饱和的轴会逐帧变（FK 那条打印的 `fk` 同理）。所以"每行都一样"**不是**这条的理由，
        //       理由只有两条：**重复的是同一个门与同一个理由** + **体积**（~30 Hz × 整段按住）。
        if (!Kinematics::isWithinJointLimits(j)) {
            if (!m_btn2JointTargetRejectLogged) {
                m_btn2JointTargetRejectLogged = true;
                std::cerr << "[Safety] Btn2 joint TARGET outside joint limits — NOT sending. "
                          << "Reported ONCE per press (every frame is still rejected). j=("
                          << j[0] << "," << j[1] << "," << j[2] << ","
                          << j[3] << "," << j[4] << "," << j[5] << ")"
                          << std::endl;
            }
            return;   // 本帧不下发（每帧都走这里，与上面那行"只报一次"无关）
        }

        // ================= 复审 M2（Minor，但属**安全回退**）：每帧步长限幅 =================
        // 【为什么必须有】纯函数给的是 `参照 + 增量`，与**上一帧发了什么**无关 ⇒ 手一甩，
        //   哪怕笔杆偏移已经过死区与低通，本帧目标仍可能一步跨出 ~17.5°（≈30 Hz ⇒ 525°/s），
        //   远超厂商"寸动"用法。RPY 路径有 `ORIENT_MAX_STEP_DEG = 3°/帧` 挡着（它的注释就是
        //   "手一甩不甩飞机器人"），关节路径**原来没有** ⇒ 这里照它**同一形状**补上。
        // 【形状照抄】`期望 − 上一帧已下发` **逐轴**夹到 `Config::ORIENT_MAX_STEP_DEG`，再累加。
        //   ⚠ 累加（**不是**对 `m_jointRef` 直接夹）：直接夹 `j − ref` 会让大偏移被**永久截断**，
        //     而累加式下它只会**慢慢跟上** —— RPY 路径 2026-09-21 那条改造就是为这个
        //     （见 `Config.h` 的 `ORIENT_DEADZONE_DEG` 历史段与上面 `wx/wy/wz` 那一段的注释）。
        //   ⚠ 量级**不自己发明**：起点照用 `ORIENT_MAX_STEP_DEG`。两条路量的都是"度"，而 RPY
        //     那一段本身就同时在限制关节走速 ⇒ 同值起步有依据。Task 3 上机若觉得慢/快，
        //     改的是**这一个常数**（以及可能要给关节路径单开一个，届时再说）。
        // 【原地改写 `j`】⇒ 下面 ② 的 FK 门与 ③ 的下发用的都是**限幅后**的值（发出去的就是
        //   被这道闸放行的那一个，不存在"判一个、发另一个"）。
        // 【积分器的推进点】只在**真的走到下发**那一步（见 ③ 之后的一行）—— 被上面任一
        //   道门拒掉的帧**不参与积分** ⇒ 它记的是"发过什么"，不是"算过什么"。
        // ★ Task 2-fix2：这段算术已抽到 `relay/Button2Joint.cpp` 的 `clampJointStep`
        //   （**那里有单测**）—— 逐轴夹取、次序、原地改写 `j` 全部逐字照抄。
        //   ⚠ **积分器 `m_btn2JointCmd` 仍然由本函数持有**（它是"上一次发了什么"这个跨帧状态，
        //     不是算术）⇒ 抽出来的函数是无状态的、`prev` 靠入参传进去。
        //   ⚠ 接线这一段（传谁当 `prev`、什么时候推进）**仍然没有自动化用例**。
        clampJointStep(m_btn2JointCmd, j, Config::ORIENT_MAX_STEP_DEG, j);

        // ② 安全门：关节目标先**正解**出末端位置，再走**现有**的位置入口
        //    （`evaluatePositionOnly` —— **一条都没删、也没绕**）。
        //    ⚠ 判的对象是**法兰**，**不是笔尖**：`Kinematics::forwardPosition` 返回的是
        //      `positions[6]`（`robot/Kinematics.cpp:200-204` = 链末那个齐次矩阵的平移列），
        //      而 `Kinematics` 整条链里**没有工具偏移**（TCP 偏移的 apply 接线另有一笔待办）
        //      ⇒ 真实笔尖比这个对象多出**整个笔杆的长度**。
        //      ★ 2026-09-23 整支终审 (C)：关节模式下这个对象**是被【扫过】的**（swept），
        //        不再像"命令是一段**固定 TCP**"时那样是个**静止的点** —— J4/J5/J6 一动，
        //        法兰沿一段圆弧走过去，**工具偏移那部分误差是【被遍历的】**：工作空间半径
        //        620mm 与 Z 行程都按**法兰**算，而笔尖真实位置可能已经出界。
        //        ⚠ RPY 路径**没有**这条性质：它按住期间的 TCP 目标是冻结的（本门判的正是那个）。
        //    ★ 2026-09-23 整支终审 (B) 订正 —— 本入口**能产生 `REJECT` 的只有四种**
        //      （逐行核过 `safety/SafetyPredictor.cpp:240-339`）：
        //        · NaN/Inf（`:243`）· 工作空间半径 620mm（`:255`）· Z 行程 0~795（`:265`）
        //        · 安全边界"超出"（`:288`）—— ⚠ 这一条**今天恒为假**：它比的是
        //          `clampToBoundaryActive(target)` 有没有改动 `target`，而该函数在
        //          `Config::SAFETY_BOUNDARY_CLAMP_ENABLED == false`（**现值**，`Config.h:75`）时
        //          **退化成 identity**（`relay/SafetyBoundary.h:47-50`）⇒ 永远不会改动
        //          ⇒ 它是**死的**（不是"暂时还没撞上"；开关翻回来它就活）。
        //      ⇒ 原文把 **圆柱奇异 30/80mm** 与 **报警点黑名单** 也列在这里，**那是错的**：
        //        这两条**都返回 `WARN_SLOW`**（`:300` / `:307` / `:319` / `:326`），
        //        **从不 `REJECT`**。而调用方（下面那一行）**只对 `REJECT` 动作** ——
        //        `if (jv.action == SafetyVerdict::REJECT)` ⇒ 这两条在本门里**什么都不做**，
        //        与 RPY 路径那条 TCP 门**同款**（`:1320` 同样只判 `REJECT`；两条路一致，
        //        不是"降速在关节路径上漏接了"）。
        //        ⚠ 别再把它们写成"会拒"—— 那句话会让人以为奇异区/报警点在这里有一道闸，
        //          于是在上机时把"没被拒"读成"门坏了"。
        //    ⚠ `Kinematics::forwardPosition` 与 `SafetyPredictor` 吃的是**同一个基座系**
        //      （`app.robotActualPose` 那一套），所以这条比较才有意义。
        //    ⚠ `refJoints` 本身是 NaN（机器人状态已经坏了）时：FK 结果是 NaN ⇒ 撞上本入口的
        //      **第一道**守卫（NaN/Inf ⇒ REJECT）⇒ 本帧不下发。这是纯函数**刻意**不兜底的那个
        //      情形（见 Button2Joint.h 的契约段）在**接线层**被接住的证据。
        //    ⚠ 这条门判的是**关节目标**；上面姿态模式那条 TCP 门（`evaluatePositionOnly(tcpCheck)`，
        //      `tcpCheck` 来自 `servoCmdX/Y/Z`）判的是另一个对象。两条都在，**不是**重复。
        Vec3 fkPos = Kinematics::forwardPosition(j);
        SafetyVerdict jv = SafetyPredictor::instance().evaluatePositionOnly(fkPos);
        if (jv.action == SafetyVerdict::REJECT) {
            // ★ 2026-09-23 整支终审 (A)：本 `cerr` 改成**每次按下只报一次**（复位在
            //   `onButton2Press`）—— 它**不是瞬时的**：笔杆被顶在工作空间边缘/越界时，
            //   FK 目标**每帧都落在同一处**、每帧都被拒 ⇒ 不压就是 ~30 Hz 刷满整个按住期。
            //   ⚠ 压的**只有消息**：`return` 仍在外面、**每帧照走**（本帧不下发是安全性质）。
            if (!m_btn2JointFkRejectLogged) {
                m_btn2JointFkRejectLogged = true;
                std::cerr << "[Safety] Btn2 joint target REJECT: " << (jv.reason ? jv.reason : "?")
                          << " — Reported ONCE per press (every frame is still rejected). j=("
                          << j[0] << "," << j[1] << "," << j[2] << ","
                          << j[3] << "," << j[4] << "," << j[5] << ")"
                          << " fk=(" << fkPos.x << "," << fkPos.y << "," << fkPos.z << ")"
                          << std::endl;
            }
            return;   // 与上面那条 TCP REJECT 同款: 本帧不下发, 下一帧从同一状态重算
        }

        // ③ 下发：厂商 `ServoJ(J1..J6,t,lookahead_time,gain)`，参数照文档示例
        //    （t=0.1 / lookahead_time=50 / gain=500 —— 见 Docs/机械臂资料/TCP_IP远程控制接口文档.md 的 ServoJ 节）。
        //    ⚠ 30 Hz 节流在本函数**开头**（`if (now - m_lastServoTime < 33) return;`）——
        //      与厂商建议的 33 Hz 同量级，**那个节流没动**（两条路径共用）。
        snprintf(cmd, sizeof(cmd), "ServoJ(%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,0.1,50,500)",
                 j[0], j[1], j[2], j[3], j[4], j[5]);

        // M2：**走到这里才推进步长积分器**（上面任何一道门 `return` 掉的帧都不参与）。
        //   ⚠ 位置在 `robotSendMotion` **之前**：若那一次发送失败（链路问题，已有 `failCount`
        //     记账），积分器比"实际发出去的东西"**多走了一帧的量**。
        //   ★ 2026-09-23 整支终审 (F) **订正**：原文写的是"一帧的差，且下一帧照旧逐轴夹住
        //     ⇒ **不放大**" —— **对连续失败是错的**：逐轴夹逼只界住**每帧**的增量（≤3°），
        //     它**不**收回"积分器已经领先实际发送多少"。丢了 k 次则积分器领先 k×3°，
        //     恢复发送的第一帧下发的是 `领先量 + 3°` ⇒ **那一步是 k×3°，不是 1×3°**
        //     ⇒ 机械臂要一口气走完那 k 帧没走的路。等价说法：**每帧有界，连续丢包数无界**。
        //     ⚠ `failCount` 只**记账**、不参与任何控制律 ⇒ 今天**没有任何东西**按它回退积分器。
        //     ⚠ 量级参考：`ORIENT_MAX_STEP_DEG = 3°/帧` × 30 Hz ⇒ 那句"≈90°/s"**只对
        //       "逐帧"成立**，对"恢复的那一步"**不成立**（同一条订正；"≤90°/s"不是整条链的界）。
        //     之所以不放它后面：那要跨过下面那段两条路径共用的代码，反而更绕、更易错。
        for (int i = 0; i < 6; ++i) m_btn2JointCmd[i] = j[i];
    } else {
        // ================= 旧路径（**逐字**）=================
        snprintf(cmd, sizeof(cmd), "ServoP(%.2f,%.2f,%.2f,%.2f,%.2f,%.2f)",
            servoCmdX, servoCmdY, servoCmdZ,
            targetRx, targetRy, targetRz);
    }

    bool sent = robotSendMotion(cmd);
    static int sendCount = 0, failCount = 0;
    sendCount++;
    if (!sent) failCount++;
    // ★ 2026-09-23 复审 I3：**补上真正发出去的 `cmd`**（本项目最恨"显示与实发不一致"）。
    //   原来这行只打 `target=`/`orient=`（那条 **冻结的 TCP** 与**不再下发的 RPY 目标**），
    //   关节模式下等于在撒谎。二选一里选的是"补 cmd"而不是"关节模式下跳过 `robotTargetPose`
    //   写入"，理由与残留见上面的分叉注释（I3 那一段）。
    //   ⚠ 无条件加给两条路径：RPY 模式下 `cmd` 与 target/orient 重复，只是那一行变长；
    //     它**不改变任何下发内容**，也不改 `else` 那一支的语义。
    if (sendCount % 50 == 0) {
        std::cout << "[Relay] Motion sends: " << sendCount
                  << " ok, " << failCount << " fail"
                  << "  target=(" << servoCmdX << "," << servoCmdY << "," << servoCmdZ << ")"
                  << " orient=(" << targetRx << "," << targetRy << "," << targetRz << ")"
                  << "  cmd=" << cmd
                  << std::endl;
    }

    // 上报到 MATLAB GUI
    reportCommand(cmd);

    // 记录到最后指令
    EnterCriticalSection(&app.lastCommandMutex);
    strncpy_s(app.lastCommandSent, cmd, sizeof(app.lastCommandSent) - 1);
    LeaveCriticalSection(&app.lastCommandMutex);

    // 记录到指令日志
    EnterCriticalSection(&app.commandLogMutex);
    strncpy_s(app.commandLog[app.commandLogIdx], cmd, sizeof(app.commandLog[0]) - 1);
    app.commandLogIdx = (app.commandLogIdx + 1) % AppState::LOG_SIZE;
    if (app.commandLogCount < AppState::LOG_SIZE) app.commandLogCount++;
    LeaveCriticalSection(&app.commandLogMutex);

    // 更新目标位姿
    EnterCriticalSection(&app.robotPoseMutex);
    app.robotTargetPose.x = servoCmdX;
    app.robotTargetPose.y = servoCmdY;
    app.robotTargetPose.z = servoCmdZ;
    app.robotTargetPose.rx = targetRx;
    app.robotTargetPose.ry = targetRy;
    app.robotTargetPose.rz = targetRz;
    LeaveCriticalSection(&app.robotPoseMutex);
}

void RelayCore::onButtonPress(const Vec3& robotPos) {
    m_stateMachine.onButtonPress();
    EnterCriticalSection(&m_basePointLock);
    // 以机器人当前实际位姿作为 target 起点 (钳位到安全边界内)
    {
        auto& app = appState;
        EnterCriticalSection(&app.robotPoseMutex);
        Vec3 rawPos(app.robotActualPose.x, app.robotActualPose.y, app.robotActualPose.z);
        LeaveCriticalSection(&app.robotPoseMutex);
        m_targetPos = SafetyBoundary::clampToBoundaryActive(rawPos);
    }
    m_lastTouchPos = robotPos;
    m_lastTouchValid = true;
    LeaveCriticalSection(&m_basePointLock);
    m_basePointSet = true;
    m_transmitting = true;

    std::cout << "[Relay] Button PRESS  — target start=("
              << m_targetPos.x << "," << m_targetPos.y << "," << m_targetPos.z << ")"
              << std::endl;
    std::cout << "[Relay] Hold button + move Touch (incremental mode)" << std::endl;
}

void RelayCore::onButtonRelease() {
    // If orientation mode is still active (button2 held), keep transmitting
    // for orientation control. Only fully stop when both modes are done.
    if (!m_transmittingOrient) {
        m_transmitting = false;
        m_basePointSet = false;
        m_lastTouchValid = false;
        m_stateMachine.onButtonRelease();
    }
    std::cout << "[Relay] Button RELEASE"
              << (m_transmittingOrient ? " (orientation still active)" : " — motion stopped")
              << std::endl;
}

void RelayCore::onButton2Press(const Vec3& stylusOrient) {
    EnterCriticalSection(&m_basePointLock);
    // Capture stylus/robot reference snapshots at press moment.
    // These are stored for diagnostics/re-sync and logged on press — they are
    // NOT used in the per-frame delta computation. The per-frame tracking uses
    // m_lastStylusOrient (incremental delta) in sendPosition().
    m_orientRefStylus = stylusOrient;
    // ★★ 2026-09-22: 参照重设 ⇒ 那个低通【必须同时重置】（见 Config::ORIENT_STYLUS_LPF_ENABLED）。
    //   否则上一次按住期间的滤值会留到这一次，表现为"刚按下就有一小段残余姿态"。
    resetStylusOffsetFilter();

    // ★★ 2026-09-23 (Task 2)：关节空间路径的"当前笔杆姿态"也从按下这一刻起算。
    //   初值 = **参照本身** ⇒ 三个偏移全为 0 ⇒ `button2JointTarget` 返回的关节**逐位等于参照**
    //   （= 臂原地不动）。之后 `sendPosition` 的偏移段每帧覆盖它（见那里的 `m_btn2StylusFilt` 赋值）。
    //   ⚠ 写在这里（而不是靠 `sendPosition` 第一帧去填）：那会让第一帧读到**上一次按住**留下的
    //     陈旧笔杆姿态 ⇒ 按下瞬间跳一下。与姿态路径 `m_targetOrient = 当前实际姿态` 是同一个理由。
    m_btn2StylusFilt[0] = stylusOrient.x;
    m_btn2StylusFilt[1] = stylusOrient.y;
    m_btn2StylusFilt[2] = stylusOrient.z;

    // Capture robot current orientation
    double curRx, curRy, curRz;
    {
        auto& app = appState;
        EnterCriticalSection(&app.robotPoseMutex);
        m_orientRefRobot = Vec3(app.robotActualPose.rx, app.robotActualPose.ry, app.robotActualPose.rz);
        curRx = app.robotActualPose.rx;
        curRy = app.robotActualPose.ry;
        curRz = app.robotActualPose.rz;
        // ★★ 2026-09-23 (Task 2)：关节参照 —— **六个关节角**，抄自按下那一刻的实际位姿。
        //   字段是现成的（`AppState::RobotPose::j1..j6`），本文件 `sendPosition` 里已有两处
        //   同款逐字段拷贝（:991 与 :1220）。
        //   ⚠ 放在**同一个临界区**里：姿态参照与关节参照必须是同一次采样的结果，否则两者
        //     描述的不是同一个位姿（而纯函数拿 `m_jointRef` 当整个按住期间的定点）。
        //   ⚠ 次序是 `j1..j6`（**不是**笔杆的 Rx/Ry/Rz）—— `button2JointTarget` 按位置读，
        //     灌错次序不会报错、只会让"前后摆变成左右摆"，所以这里逐字段写、不写循环。
        m_jointRef[0] = app.robotActualPose.j1;
        m_jointRef[1] = app.robotActualPose.j2;
        m_jointRef[2] = app.robotActualPose.j3;
        m_jointRef[3] = app.robotActualPose.j4;
        m_jointRef[4] = app.robotActualPose.j5;
        m_jointRef[5] = app.robotActualPose.j6;

        // ★★ 2026-09-23 (Task 2 修复 / 复审 C1, Critical)：**锁存**这一次按住的模式。
        //   读的是"按下按钮2 的**这一刻**按钮1 是否已按下" ⇒ 组合模式（1+2）进来就锁成 false
        //   （整段按住走旧 RPY 路），**整个按住期间不再变化**。
        //   ⚠ 与 `m_jointRef` 在**同一个临界区**：模式与参照必须是同一次采样的产物
        //     （否则会出现"用 A 时刻的参照、按 B 时刻的模式下发"）。
        //   ⚠ 这一行**替代**了 `sendPosition` 里原来每帧重算的那个条件 —— 那个写法会在按住
        //     中途换控制律（松开按钮1 ⇒ 翻到 ServoJ 并回到按下时的位姿；先按2再按1 ⇒ 翻到 ServoP）。
        m_btn2JointMode = Config::BTN2_JOINT_SPACE_ENABLED && !appState.lastButtonState;

        // ⚠⚠ 2026-09-23 复审 Minor 5：上面这一行读到的 `appState.lastButtonState` **是本帧的值**，
        //   而这一点**靠的是调用方的小节次序** —— `HapticCallback.cpp` 必须让
        //   小节 5（按钮1 状态机，:114-123，它写 `app.lastButtonState = button1`）
        //   **跑在小节 5b 之前**（小节 5b 才是调 `onButton2Press` 的那一处，:125-142）。
        //   ⇒ 同帧内 1+2 一起按下时，这里读到的**已经是"按下"** ⇒ 锁成**组合模式**（走 RPY 旧路），
        //     正是我们要的语义（"组合模式继续走旧路，不引入功能回退"）。
        //   ⚠ 这条次序**没有任何自动化用例**：它跨文件、跨回调，两节调换顺序**不会报错、不会红**，
        //     只会**静默**改掉组合语义（同帧 1+2 会被锁成关节模式 ⇒ 按钮1 的平移被忽略，
        //     而那一路上还有一次性的 `cout` 提示）。**改 `HapticCallback.cpp` 的小节次序前先读这句。**
        //   ⚠ "按钮1 早已按着、之后再按按钮2"那种情形**与次序无关**（`lastButtonState` 早在上一帧
        //     就已经是 true）⇒ 受影响的只有**同帧**那一种，也就是最不容易被注意的那种。

        // ★ 2026-09-23 复审 Minor 2：一次性标记与"模式"同生共死 ⇒ **在这里复位**
        //   （同一个临界区、同一份"这一次按下"的快照），见 `RelayCore.h` 里那四个成员的注释。
        //   ★ 2026-09-23 整支终审 (A)：原先是**两条**（Minor 2 那轮），本轮把 I2 与 FK 那两道
        //   `cerr` 也压成"每次按下只报一次" ⇒ 变成**四条**，一起在这里复位。
        m_btn2JointBtn1Noticed = false;
        m_btn2JointRefRejectLogged = false;
        m_btn2JointTargetRejectLogged = false;
        m_btn2JointFkRejectLogged = false;

        // M2：步长限幅积分器的种子 = 参照本身 ⇒ 第一帧"本帧要走的量"为 0（与纯函数返回参照一致）。
        m_btn2JointCmd[0] = m_jointRef[0];
        m_btn2JointCmd[1] = m_jointRef[1];
        m_btn2JointCmd[2] = m_jointRef[2];
        m_btn2JointCmd[3] = m_jointRef[3];
        m_btn2JointCmd[4] = m_jointRef[4];
        m_btn2JointCmd[5] = m_jointRef[5];
        LeaveCriticalSection(&app.robotPoseMutex);
    }

    // Initialize accumulated target and last-frame stylus orientation.
    // TCP stays at the current target position (fixed during orientation-only,
    // updated by position delta during combined button1+2 mode).
    m_targetOrient = Vec3(curRx, curRy, curRz);
    m_lastStylusOrient = stylusOrient;
    m_orientValid = true;
    m_transmittingOrient = true;

    // If position mode is not already active, start transmission
    if (!m_transmitting) {
        m_stateMachine.onButtonPress();
        // Seed position target from actual TCP pose (fixed during orientation-only,
        // updated by Touch delta during combined button1+2 mode).
        {
            auto& app = appState;
            EnterCriticalSection(&app.robotPoseMutex);
            Vec3 tcpPos(app.robotActualPose.x, app.robotActualPose.y, app.robotActualPose.z);
            LeaveCriticalSection(&app.robotPoseMutex);
            m_targetPos = SafetyBoundary::clampToBoundaryActive(tcpPos);
        }
        m_transmitting = true;
        m_basePointSet = true;
    }
    LeaveCriticalSection(&m_basePointLock);

    std::cout << "[Relay] Button2 PRESS — orient ref=("
              << m_orientRefStylus.x << "," << m_orientRefStylus.y << "," << m_orientRefStylus.z << ")"
              << " robot ref=(" << m_orientRefRobot.x << "," << m_orientRefRobot.y << "," << m_orientRefRobot.z << ")"
              << std::endl;
    // ★ 2026-09-23 (Task 2)：关节参照也打出来。**这不是装饰** —— Task 3 上机判据里有一条
    //   "J1/J2/J3 在按住期间逐位不变 ⇒ 若有漂说明参照没抓对"，而"抓到了什么"这件事
    //   没有任何自动化用例覆盖（本文件不被测试编译）⇒ 至少让它可观察、可对账。
    std::cout << "[Relay] Button2 PRESS — joint ref=("
              << m_jointRef[0] << "," << m_jointRef[1] << "," << m_jointRef[2] << ","
              << m_jointRef[3] << "," << m_jointRef[4] << "," << m_jointRef[5] << ")"
              << (Config::BTN2_JOINT_SPACE_ENABLED ? "  [joint-space: ServoJ]" : "  [RPY: ServoP]")
              << std::endl;
    // ★ 2026-09-23 (Task 2 修复 / 复审 C1)：**锁存后的**模式也打出来 —— 它与上面那行的
    //   "开关状态"**不是一回事**：组合模式下开关是 true 而锁存后是 false（这一整段按住走 RPY）。
    //   ⚠ 上机对账时看的是**这一行**：`[joint-space: ServoJ]` 只说明开关开着，模式要看这里。
    std::cout << "[Relay] Button2 PRESS — latched mode: "
              << (m_btn2JointMode ? "JOINT-SPACE (ServoJ for the whole hold)"
                                  : "RPY (ServoP for the whole hold)")
              << std::endl;
}

void RelayCore::onButton2Release() {
    m_transmittingOrient = false;
    m_orientValid = false;
    // C1：模式跟着这一次按住一起结束（下一次按下的锁存在 `onButton2Press`）。
    m_btn2JointMode = false;

    // If button1 is NOT pressed (only button2 was active), stop all transmission.
    // When button1 IS still held, keep m_transmitting active for position control.
    if (!appState.lastButtonState.load()) {
        m_transmitting = false;
        m_basePointSet = false;
        m_lastTouchValid = false;
        m_stateMachine.onButtonRelease();
        std::cout << "[Relay] Button2 RELEASE — all motion stopped" << std::endl;
    } else {
        // Button1 still held — re-sync m_targetPos to current robot TCP.
        // During orientation mode, the TCP moved (rotated around wrist center),
        // so m_targetPos is stale. Re-seed from actual pose to avoid a position jump.
        EnterCriticalSection(&m_basePointLock);
        {
            auto& app = appState;
            EnterCriticalSection(&app.robotPoseMutex);
            Vec3 rawPos(app.robotActualPose.x, app.robotActualPose.y, app.robotActualPose.z);
            LeaveCriticalSection(&app.robotPoseMutex);
            m_targetPos = SafetyBoundary::clampToBoundaryActive(rawPos);
        }
        LeaveCriticalSection(&m_basePointLock);
        std::cout << "[Relay] Button2 RELEASE — orientation control stopped (button1 still held)" << std::endl;
    }
}

static void logFeedback(const char* msg, const char* portLabel) {
    auto& app = appState;
    EnterCriticalSection(&app.feedbackLogMutex);
    int writeIdx = app.feedbackLogIdx;
    snprintf(app.feedbackLog[writeIdx], sizeof(app.feedbackLog[0]),
        "[%s] %s", portLabel, msg);
    app.feedbackLogIdx = (app.feedbackLogIdx + 1) % AppState::LOG_SIZE;
    if (app.feedbackLogCount < AppState::LOG_SIZE) app.feedbackLogCount++;
    // Relay to MATLAB GUI
    RelayCore::instance().reportFeedback(app.feedbackLog[writeIdx]);
    LeaveCriticalSection(&app.feedbackLogMutex);
}

void RelayCore::pollFeedback() {
    char buf[1024];
    // 读取运动端口反馈 (非阻塞)
    while (robotRecvMotionPoll(buf, sizeof(buf))) {
        RobotFeedback fb;
        strncpy_s(fb.raw, buf, sizeof(fb.raw) - 1);
        fb.fromPort = Config::MOTION_PORT;
        fb.errorId = (buf[0] == '0') ? 0 : -1;
        FeedbackParser::extractData(buf, fb.data, sizeof(fb.data));

        // Parse ServoP errors and report to state machine
        if (fb.raw[0] != '0') {
            int dobotCode = 0;
            if (FeedbackParser::extractErrorCode(fb.raw, dobotCode) && dobotCode != 0) {
                RobotErrorCode errCode = FeedbackParser::mapRobotErrorCode(dobotCode);
                RobotError error;
                error.code = errCode;
                error.severity = getSeverity(errCode);
                error.timestampMs = GetTickCount64();
                // Get current target from state
                error.targetPosition = m_targetPos;
                error.speedFactor = m_stateMachine.speedFactor();

                Vec3 zeroDelta = {0, 0, 0};  // no user delta for feedback errors
                m_stateMachine.onError(error, zeroDelta);

                double constraintMag = 0;  // feedback error has no constraint force
                RobotDiagnostics::instance().logError(error, constraintMag,
                    m_stateMachine.currentState());
            }
        }

        // 记录日志 (截断长字符串)
        char shortMsg[256];
        const char* src = fb.data[0] ? fb.data : fb.raw;
        snprintf(shortMsg, sizeof(shortMsg), "%.200s", src);
        logFeedback(shortMsg, "30003");

        for (auto* ext : m_extensions) {
            ext->onAfterFeedback(fb);
        }
    }

    // 注意: 不读取使能端口 (29999)
    // 使能端口采用"命令-响应"模式 (GetPose/GetAngle/RobotMode)，
    // 响应必须由发送命令的函数独享读取，避免 pollFeedback 偷走数据
    // 导致 robotActualPose 永远停留在 init 时的值。
}

void RelayCore::queryPose() {
    if (!isRobotConnected()) return;
    robotDrainEnable();  // 排空残留避免读到其他命令的响应
    robotSendEnable("GetPose()");
    Sleep(50);
    char fb[1024];
    if (robotRecvEnable(fb, sizeof(fb))) {
        auto& app = appState;
        EnterCriticalSection(&app.robotPoseMutex);
        FeedbackParser::parsePose(fb, app.robotActualPose);
        // 在锁内读取位姿，避免与 jointAngle 定时器竞态
        double px = app.robotActualPose.x;
        double py = app.robotActualPose.y;
        double pz = app.robotActualPose.z;
        double prx = app.robotActualPose.rx;
        double pry = app.robotActualPose.ry;
        double prz = app.robotActualPose.rz;
        LeaveCriticalSection(&app.robotPoseMutex);

        // 上报机器人实际位姿到 MATLAB GUI
        char buf[128];
        snprintf(buf, sizeof(buf), "RP|%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
            px, py, pz, prx, pry, prz);
        sendRelayUpdate(buf);
    }

    // PING/PONG latency measurement
    DWORD now = GetTickCount();
    if (now - m_lastPingMs > (DWORD)Config::PING_INTERVAL_MS) {
        m_lastPingMs = now;
        // Send PING to enable port
        if (isRobotConnected()) {
            char pingBuf[64];
            uint64_t sentMs = GetTickCount64();
            snprintf(pingBuf, sizeof(pingBuf), "PING|%llu", sentMs);
            robotSendEnable(pingBuf);
            // Read PONG response (non-blocking poll with short wait)
            Sleep(5);
            char pongBuf[128] = {};
            if (robotRecvEnablePoll(pongBuf, sizeof(pongBuf))) {
                if (strncmp(pongBuf, "PONG", 4) == 0) {
                    uint64_t now64 = GetTickCount64();
                    const char* pipe = strchr(pongBuf, '|');
                    if (pipe) {
                        uint64_t echoMs = _strtoui64(pipe + 1, nullptr, 10);
                        auto& app = appState;
                        app.latencyMs = (float)(now64 - echoMs);
                    }
                }
            }
        }
    }

    // Health check: skip during 10s grace period after startup
    if (now - m_heartbeatStartMs >= 10000) {
        if (now - m_lastHeartbeatMs > (DWORD)Config::HEARTBEAT_TIMEOUT_MS) {
            if (!m_heartbeatLostReported) {
                m_heartbeatLostReported = true;
                RobotError error;
                error.code = RobotErrorCode::ERR_HEARTBEAT_LOST;
                error.severity = Severity::FATAL;
                error.timestampMs = GetTickCount64();
                Vec3 zeroDelta = {0, 0, 0};
                m_stateMachine.onError(error, zeroDelta);
            }
        } else {
            m_heartbeatLostReported = false;
        }
    }

    // Heartbeat update
    m_lastHeartbeatMs = now;
}

void RelayCore::queryJointAngles() {
    if (!isRobotConnected()) return;
    robotDrainEnable();  // 排空残留
    robotSendEnable("GetAngle()");
    Sleep(50);
    char fb[1024];
    if (robotRecvEnable(fb, sizeof(fb))) {
        double angles[6] = {};
        if (FeedbackParser::parseAngle(fb, angles)) {
            auto& app = appState;
            EnterCriticalSection(&app.robotPoseMutex);
            app.robotActualPose.j1 = angles[0];
            app.robotActualPose.j2 = angles[1];
            app.robotActualPose.j3 = angles[2];
            app.robotActualPose.j4 = angles[3];
            app.robotActualPose.j5 = angles[4];
            app.robotActualPose.j6 = angles[5];
            LeaveCriticalSection(&app.robotPoseMutex);

            // 上报关节角度到 MATLAB GUI
            char buf[128];
            snprintf(buf, sizeof(buf), "J|%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
                angles[0], angles[1], angles[2], angles[3], angles[4], angles[5]);
            sendRelayUpdate(buf);
        }
    }

    // Health check: skip during 10s grace period after startup
    DWORD now = GetTickCount();
    if (now - m_heartbeatStartMs >= 10000) {
        if (now - m_lastHeartbeatMs > (DWORD)Config::HEARTBEAT_TIMEOUT_MS) {
            if (!m_heartbeatLostReported) {
                m_heartbeatLostReported = true;
                RobotError error;
                error.code = RobotErrorCode::ERR_HEARTBEAT_LOST;
                error.severity = Severity::FATAL;
                error.timestampMs = GetTickCount64();
                Vec3 zeroDelta = {0, 0, 0};
                m_stateMachine.onError(error, zeroDelta);
            }
        } else {
            m_heartbeatLostReported = false;
        }
    }
    m_lastHeartbeatMs = GetTickCount();
}

// ★★ 2026-09-22 新增: 【触觉帧】心跳 —— 由触觉回调入口无条件调用 (HapticCallback.cpp)。
// 【为什么必须有这个函数，而不是让 sendPosition 去刷】
//   看门狗 (本文件 :309) 想查的是"**触觉回调**还活着吗"，可它读的那个时间戳从前是
//   `sendPosition` 刷的 —— 而 `sendPosition` 只在 `isTransmitting()` 为真时才被调用
//   (HapticCallback.cpp:134) ⇒ 那个数实际表达的是"**上一次下发**"，不是"上一帧回调"。
//   ⇒ 两种完全不同的状态 (回调真的停了 / 只是没在下发) 共用一个数 ⇒ **分不开**。
//   现场 2026-09-22 撞上的就是它: `ForceReader WATCHDOG ... 1078ms since last haptic frame`
//   —— 而操作员说空闲很短，两边对不上，正是因为那个数根本不是触觉帧的年龄。
//   ⇒ 放到这里以后: 再报 1078ms **一定**是回调真的停了 (那时查 GLUT / 控制台阻塞)。
// ⚠ 无条件刷新是对的: 本函数就是"回调跑过一帧"这一件事的记录，与按钮状态无关。
void RelayCore::markHapticFrame() {
    m_lastHapticFrameMs.store(GetTickCount());
}

void RelayCore::checkHapticWatchdog() {
    if (!m_transmitting.load()) return;  // 未运动时不检查
    DWORD now = GetTickCount();
    DWORD lastFrame = m_lastHapticFrameMs.load();
    if (lastFrame > 0 && (now - lastFrame) > (DWORD)Config::WATCHDOG_TIMEOUT_MS) {
        if (!m_watchdogTripped) {
            m_watchdogTripped = true;
            std::cerr << "[Safety] WATCHDOG: haptic thread silent for "
                      << (now - lastFrame) << "ms — triggering FATAL" << std::endl;
            RobotError error;
            error.code = RobotErrorCode::ERR_EMERGENCY_STOP;
            error.severity = Severity::FATAL;
            error.timestampMs = GetTickCount64();
            Vec3 zeroDelta = {0, 0, 0};
            m_stateMachine.onError(error, zeroDelta);
            // FATAL callback (registered in init) will DisableRobot()
        }
    }
}

void RelayCore::pingRobot() {
    if (!isRobotConnected()) return;
    char pingBuf[64];
    snprintf(pingBuf, sizeof(pingBuf), "PING|%llu", GetTickCount64());
    robotSendEnable(pingBuf);
    // Response handled in pollFeedback (PONG echo)
}

void RelayCore::checkAlarm() {
    if (!isRobotConnected()) return;
    robotDrainEnable();  // 排空残留
    robotSendEnable("RobotMode()");
    Sleep(50);
    char fb[1024];
    if (robotRecvEnable(fb, sizeof(fb))) {
        int mode = -1;
        FeedbackParser::parseMode(fb, mode);
        auto& app = appState;
        bool wasAlarm = app.isRobotInAlarm.exchange(mode == 9);
        if (mode == 9 && !wasAlarm) {
            std::cout << "\n[Relay] !!! 检测到机械臂报警 (mode=9) !!!" << std::endl;

            // Report alarm to state machine
            RobotError error;
            error.code = RobotErrorCode::ERR_ALARM_MODE9;
            error.severity = Severity::FATAL;
            error.timestampMs = GetTickCount64();
            Vec3 zeroDelta = {0, 0, 0};
            m_stateMachine.onError(error, zeroDelta);

            // 立即获取当前位置并记录到 SafetyPredictor 黑名单
            queryPose();
            EnterCriticalSection(&app.robotPoseMutex);
            AppState::RobotPose alarmPose = app.robotActualPose;
            LeaveCriticalSection(&app.robotPoseMutex);
            SafetyPredictor::instance().addAlarmRecord(alarmPose);

            // 自动进入脱困流程
            std::cout << "[Relay] 自动启动脱困流程..." << std::endl;
            if (escapeSingularity()) {
                std::cout << "[Relay] 脱困成功，恢复正常操作" << std::endl;
                app.isRobotInAlarm = false;
                m_stateMachine.onRecovery();
            } else {
                std::cout << "[Relay] 脱困失败，按 'e' 重试或重启程序" << std::endl;
            }
        } else if (mode != 9 && wasAlarm) {
            // 报警已清除 (用户在机器人控制器上手动清除)
            std::cout << "[Relay] 报警已清除 (mode=" << mode << ")，尝试恢复..." << std::endl;
            app.isRobotInAlarm = false;
            // Re-enable robot since FATAL callback disabled it
            // 【必须】回放连接时下发的那份负载, 不能用 effective(): 内存生效值可能已被本
            // 会话的标定求解改掉, 而运行中改负载会让机械臂动 (理由同 escapeSingularity)。
            robotSendEnable("ClearError()");
            Sleep(200);
            if (!reenableRobotWithConnectPayload()) {
                // 使能失败却照样 onRecovery() 会让上层以为手臂已可用 (实际还在下使能状态)
                std::cerr << "[Relay] 恢复失败: EnableRobot 未成功, 保持报警状态" << std::endl;
                app.isRobotInAlarm = true;
                return;
            }
            Sleep(200);
            m_stateMachine.onRecovery();
        }
    }
}

bool RelayCore::triggerEscape() {
    if (!isRobotConnected()) {
        std::cerr << "[Relay] 机械臂未连接，无法脱困" << std::endl;
        return false;
    }
    return escapeSingularity();
}

void RelayCore::registerExtension(IExtension* ext) {
    m_extensions.push_back(ext);
}

// ===== MATLAB GUI 上报 =====

// 建 + 连 + 装上 relay socket。**不打印任何东西** —— 打印由调用方决定:
//   启动那一次失败要把处置说全 (见 initRelayReporting), 而【事后重连】成功只需一句。
// 成功返回 true 且 m_relaySocket 已装好; 失败返回 false 且不留残余。
bool RelayCore::connectRelaySocket() {
    SOCKET sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock == INVALID_SOCKET) return false;

    sockaddr_in addr;
    addr.sin_family = AF_INET;
    inet_pton(AF_INET, Config::RELAY_IP, &addr.sin_addr);
    addr.sin_port = htons(Config::RELAY_PORT);

    if (connect(sock, (SOCKADDR*)&addr, sizeof(addr)) == SOCKET_ERROR) {
        closesocket(sock);
        return false;
    }

    int timeout = 100;
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, (char*)&timeout, sizeof(timeout));

    // ★★ 2026-09-24: 这条 socket 改成【非阻塞】。
    //   【为什么】它唯一的发送入口 `sendRelayUpdate` 跑在【触觉线程】里
    //     （`HapticCallback.cpp:146/150` → `sendPosition` / `reportPosition`），而现场实测：
    //     **MATLAB 开着、只是一时收得慢（背压）** ⇒ `send` 卡满 `SO_SNDTIMEO`(100ms) 超时，
    //     而一个触觉帧里有**两处**调用 ⇒ 触觉线程静默 **204ms** ⇒ 安全看门狗判 FATAL ⇒
    //     `ERR_EMERGENCY_STOP`、机械臂被禁用。
    //     ⇒ 一条【只给 MATLAB 画孪生】的遥测链路，把机械臂急停了 —— 那是设计上的越权，
    //       不是参数问题。MATLAB 在架构上是 display only（见 `start_system.bat` 的注释）。
    //   【为什么必须放在 connect 之后】`FIONBIO` 对 `connect` 也生效 ⇒ 放前面会把连接变成
    //     异步的，本函数下面"连不上就 closesocket 返回 false"那套语义就变了。
    //   【收包路径不受影响，且已被核过】它本来就**先 select 零超时再 recv**，且把 `n <= 0`
    //     一律当"没有数据"（本文件 `recv(sock, …)` 那一处，全文件只有那一处 recv）
    //     ⇒ 非阻塞下 `recv` 返回 `WSAEWOULDBLOCK` 走的正是同一条路。
    u_long nb = 1;
    ioctlsocket(sock, FIONBIO, &nb);

    EnterCriticalSection(&m_relaySocketMutex);
    m_relaySocket = sock;
    LeaveCriticalSection(&m_relaySocketMutex);
    return true;
}

void RelayCore::initRelayReporting() {
    if (connectRelaySocket()) {
        std::cout << "[Relay] GUI reporting connected to " << Config::RELAY_IP
                  << ":" << Config::RELAY_PORT << std::endl;
        // ★ 2026-09-22: 连上就回读一次增益。必须有 —— 否则 MATLAB 在 C++ 启动时看到的
        //   是滑条的初值 (它自己猜的), 而实际生效值可能来自 force_tuning.json。
        //   那正是本设计要消灭的"GUI 显示的值 ≠ 实际生效的值"。
        sendReflectionGain(true);
        return;
    }
    // ★ 失败必须出声 (2026-09-21)。从前这里是【静默 return】, 而整个程序里唯一会提到
    //   "GUI 没连上"的地方, 是 reportPosition() 里那句每 3.3 s 一次的 "last send=-1B" ——
    //   它把这件事写成一个不可读的数, 现场只会当噪音忽略。那句刷屏已删
    //   ⇒ 本句是【唯一】会说这件事的地方, 所以一次说全: 什么没在跑、影响哪几路、
    //     以及【它不是故障】—— 免得下一个人把它当异常去查。
    std::cout << "[Relay] GUI reporting 【未连接】—— " << Config::RELAY_IP << ":"
              << Config::RELAY_PORT << " 上没有在听 (MATLAB relay_gui 没在跑?)" << std::endl;
    std::cout << "[Relay]   影响: P| / J| / RP| / C| / FB| 全部发不出去 (位置 / 关节角 / 实际位姿 /"
                 " 命令回显 / 反馈回显)。" << std::endl;
    std::cout << "[Relay]   不影响: 机械臂控制、力数据采集 (@576/@720/@1304)、一致性闸门、"
                 "本地补偿 —— 它们都不走这条 socket。" << std::endl;
    std::cout << "[Relay]   要恢复: 在 MATLAB 那一侧起 relay_gui 即可 —— ★ 本程序会【自动按秒重连】,"
                 " 不必重启 (2026-09-21 起; 从前是只在启动时连一次, 所以那时确实要重启)。" << std::endl;
}

// 把 relay socket 作废, 并且【只在状态真的变化时】出声一次。
// 【为什么必须出声】这条 socket 一断, 上面那几路全哑 —— 而现场看到的现象是
//   【MATLAB 的孪生停在默认姿势】, 从那一头根本分不出是"没收到数据"还是"显示坏了"。
//   一次都不说的话, 排查会从显示那一侧开始 (2026-09-21 现场就是这么绕了一大圈)。
// ⚠ 这条路径从前【完全无声】: sendRelayUpdate 把两次 send 的返回值直接相加就 return,
//   SOCKET_ERROR (-1) 与换行那次 (+1) 会抵消成正数 ⇒ 连接死掉看起来和正常一样。
static bool s_relayDownReported = false;

// ★ 2026-09-24: 遥测【丢帧】的计数与限频上报 —— 与"断线"是两件事（见 sendRelayUpdate 里
//   那条语义分界）。丢帧是背压下的正常取舍：宁可丢孪生的一帧，也不让触觉线程等 100ms。
//   ⚠ 计数用 Interlocked：本路径可能被两个线程调用，普通 `++` 会丢计数。
//   ⚠ 上报那两句**仍然是竞态的，且刻意不修**：最坏是多打一行或少打一行；
//     为此再加一把锁，就是把触觉线程又往锁上挂一次 —— 那正是本次要消灭的东西。
//   ⚠ 计数【不】在重连时清零：它是整场会话的累计，"今天一共丢了多少帧"才是有用的数。
static volatile LONG s_relayDropFrames = 0;
static DWORD s_relayDropLastReportMs = 0;

static void noteRelayDrop() {
    const LONG total = InterlockedIncrement(&s_relayDropFrames);
    const DWORD now = GetTickCount();
    if (s_relayDropLastReportMs != 0 && (now - s_relayDropLastReportMs) < 5000) return;
    s_relayDropLastReportMs = now;
    std::cout << "[Relay] GUI 遥测丢帧 " << total
              << " 帧（对端一时收不过来 = 背压，【不是断线】；孪生会跳一段）" << std::endl;
}

void RelayCore::markRelayDisconnected(const char* why) {
    EnterCriticalSection(&m_relaySocketMutex);
    const bool had = (m_relaySocket != INVALID_SOCKET);
    if (had) {
        closesocket(m_relaySocket);
        m_relaySocket = INVALID_SOCKET;
    }
    LeaveCriticalSection(&m_relaySocketMutex);

    if (!had || s_relayDownReported) return;   // 没变化 / 已经说过 ⇒ 不出声 (不刷屏)
    s_relayDownReported = true;
    std::cout << "[Relay] GUI reporting 【连接已断】(" << why << ") —— "
                 "P| / J| / RP| / C| / FB| 全部发不出去了。" << std::endl;
    std::cout << "[Relay]   现场表现: MATLAB 的孪生会【停在最后一次收到的位置】;"
                 " 若从启动起一帧都没收到, 就是它的初值 (关节全 0 = 一个默认姿势)。" << std::endl;
    std::cout << "[Relay]   本程序会自动按秒重连; 重连上会再报一句。" << std::endl;
}

// 发之前保证连着: socket 无效时【按秒重连】。
// 【为什么】从前只在启动时连一次 (见 initRelayReporting 的旧文案): 启动那一刻的竞态、
//   或事后对端关掉, 都会让【整个会话】静默地发不出去 —— 而现场唯一能看到的只是"孪生不动"。
// 返回值: true = 现在可以发。
bool RelayCore::ensureRelayConnected() {
    EnterCriticalSection(&m_relaySocketMutex);
    const bool up = (m_relaySocket != INVALID_SOCKET);
    LeaveCriticalSection(&m_relaySocketMutex);
    if (up) return true;

    static DWORD lastTryMs = 0;
    const DWORD now = GetTickCount();
    if (lastTryMs != 0 && (now - lastTryMs) < 1000) return false;   // 每秒最多试一次, 别空转
    lastTryMs = now;

    if (!connectRelaySocket()) return false;
    s_relayDownReported = false;
    std::cout << "[Relay] GUI reporting 【已重连】到 " << Config::RELAY_IP << ":"
              << Config::RELAY_PORT << std::endl;
    // ★ 重连后同样要回读增益: MATLAB 可能是在 C++ 之后才起来的, 那条 RG| 从没送到过。
    //   sendRelayUpdate 不调本函数 (它在 socket 无效时只返回 -1), 所以这里【不会递归】。
    sendReflectionGain(true);
    return true;
}

void RelayCore::shutdownRelayReporting() {
    EnterCriticalSection(&m_relaySocketMutex);
    if (m_relaySocket != INVALID_SOCKET) {
        closesocket(m_relaySocket);
        m_relaySocket = INVALID_SOCKET;
    }
    LeaveCriticalSection(&m_relaySocketMutex);
}

int RelayCore::sendRelayUpdate(const char* msg) {
    EnterCriticalSection(&m_relaySocketMutex);
    SOCKET sock = m_relaySocket;
    LeaveCriticalSection(&m_relaySocketMutex);
    if (sock == INVALID_SOCKET) return -1;

    // ★★ 2026-09-24: 【先探可写性，不可写就丢这一帧】—— 与收包路径同一套办法（select 零超时）。
    //   起因见 `connectRelaySocket` 里 FIONBIO 那一段：现场一次 204ms 的触觉静默触发了急停。
    //   ============ 本函数最要紧的一处语义分界 ============
    //     · 不可写 / WSAEWOULDBLOCK / WSAETIMEDOUT ⇒ **背压** ⇒ 丢帧、【不断线】。
    //       从前把超时当断线 ⇒ 对端一次正常的慢就被 closesocket + 每秒重连 ——
    //       而"对端慢"与"对端死"要做的处置完全相反（一个该丢帧，一个该重连）。
    //     · 其它 send 错误（ECONNRESET / EPIPE …）⇒ 真的死了 ⇒ 才 `markRelayDisconnected`。
    //   返回值：>0 已发字节 · −1 socket 无效或真断线 · −2 丢帧（背压）。
    //   ⚠ 调用方【不消费】返回值（那是既有事实，见 `reportCommand` 那处的注释）——
    //     加一个 −2 不会改变任何调用点的行为，只是把"丢了"这件事变成可数的。
    fd_set wfds;
    FD_ZERO(&wfds);
    FD_SET(sock, &wfds);
    timeval tv = { 0, 0 };
    if (select(0, nullptr, &wfds, nullptr, &tv) <= 0) {
        noteRelayDrop();
        return -2;
    }

    // ★ 两次 send 合成一次（msg + '\n'）：
    //   ① 暴露时间减半（原来正文与换行各是一次独立 send，各能卡一个 SO_SNDTIMEO）；
    //   ② 顺手修掉那条老账 —— 两个线程同时发时正文与换行会交错、**粘行**，
    //      而粘行对 MATLAB 而言等于"第二条消息不存在"（静默丢）。
    //   ⚠ 截断就【整帧丢掉】，不发半条：半条会被对端当成一条畸形消息解析。
    char line[512];
    const int n = snprintf(line, sizeof(line), "%s\n", msg);
    if (n <= 0 || n >= (int)sizeof(line)) {
        noteRelayDrop();
        return -2;
    }

    const int sent = send(sock, line, n, 0);
    if (sent == SOCKET_ERROR) {
        const int e = WSAGetLastError();
        if (e == WSAEWOULDBLOCK || e == WSAETIMEDOUT) {   // 背压，不是死
            noteRelayDrop();
            return -2;
        }
        markRelayDisconnected("send() 失败");
        return -1;
    }
    return sent;
}

void RelayCore::reportPosition() {
    DWORD now = GetTickCount();
    if (now - m_lastRelayUpdate < (DWORD)Config::RELAY_UPDATE_INTERVAL) return;
    m_lastRelayUpdate = now;

    // ★ 2026-09-21: 发之前先保证连着 —— 从前 socket 只在启动时建立一次, 事后断开就哑掉
    //   整个会话, 而现场只看到"孪生不动"。本函数是 33ms 节流, 重连自己另有 1s 节流。
    if (!ensureRelayConnected()) return;

    auto& app = appState;
    char buf[256];
    hduVector3Dd pos;
    EnterCriticalSection(&app.devicePosMutex);
    pos = app.devicePos;
    LeaveCriticalSection(&app.devicePosMutex);

    double sx, sy, sz;
    {
        auto& appRef = appState;
        EnterCriticalSection(&appRef.stylusOrientMutex);
        sx = appRef.stylusOrient[0];
        sy = appRef.stylusOrient[1];
        sz = appRef.stylusOrient[2];
        LeaveCriticalSection(&appRef.stylusOrientMutex);
    }
    snprintf(buf, sizeof(buf), "P|%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
        pos[0], pos[1], pos[2], sx, sy, sz);

    // ⚠ 这里从前每 100 次 (RELAY_UPDATE_INTERVAL = 33ms ⇒ ≈3.3 s) 打一行
    //   "[Relay] Sent <N> position updates, last send=-1B"。2026-09-21 删掉，理由两条：
    //     · 那个 "-1" 是 sendRelayUpdate 在【socket 无效】时的返回 (见该函数), 也就是
    //       "MATLAB 那一侧没连上" —— 一个【整个会话恒定不变】的状态, 却被写成不可读的 "-1B",
    //       于是现场只能把它当噪音;
    //     · 它按 3.3 s 的节拍刷屏, 而本项目的控制台正是现场要读的东西 (闸门拒绝的原因/处置、
    //       逐通道表占十几行) —— 刷屏等于"看不见"。
    //   ⇒ 它想说的那件事改由 initRelayReporting() 在【连接失败那一次】一次性、可读地说全。
    //     【不是】把这件事变成静默: 那一句现在存在 (从前失败路径是静默 return, 靠这句刷屏
    //     隐晦地暗示 —— 那正是"安静地错"的形状)。
    //   返回值已无人消费 (reportCommand / reportFeedback 一直就忽略它), 所以不接。
    (void)sendRelayUpdate(buf);

    // ===== ★★ 触觉安全提示已关闭 —— 持续重发 MATLAB 警告 (2026-09-21) =====
    // 关掉的是哪三组力、为什么关、代价与补偿: 见 Config::FORCE_CONSTRAINT_FORCES_ENABLED。
    // 这里只负责【把代价兑现成看得见的东西】。本函数是 33ms 节流 (30Hz), 而 relay_gui 的刷新
    // 是 0.05s (20Hz) 且【每个周期把 warn_max_level 清零】⇒ 30Hz 重发足以让它常亮。
    // ⚠ 控制台的横幅【不在这里打】: 本函数跑在触觉实时线程上 (HapticCallback → 1kHz 节流到
    //   33ms), 往控制台写会干扰它。横幅在主线程打, 见 main.cpp 的 runConstraintDisableNotice()。
    if (!Config::FORCE_CONSTRAINT_FORCES_ENABLED) {
        reportWarning(2, "S",
                      "触觉安全提示已关闭",
                      "虚拟约束力(边界·奇异·工作空间·报警历史)已全部停用 ⇒ 靠近危险区不会有推力 请靠视觉与报警指示灯",
                      0.0, 0.0);
    }
}

// 线上格式的【唯一一份定义】—— 声明与约束 (不许含逗号 / 必须重复发) 见 RelayCore.h。
void RelayCore::reportWarning(int level, const char* type, const char* message,
                              const char* suggestion, double param1, double param2) {
    if (!type || !message || !suggestion) return;
    char wbuf[320];
    snprintf(wbuf, sizeof(wbuf), "W|%d,%s,%s,%s,%.1f,%.1f",
             level, type, message, suggestion, param1, param2);
    sendRelayUpdate(wbuf);
}

void RelayCore::reportCommand(const char* cmd) {
    char buf[384];
    snprintf(buf, sizeof(buf), "C|%s", cmd);
    sendRelayUpdate(buf);
}

void RelayCore::reportFeedback(const char* fbText) {
    char buf[384];
    snprintf(buf, sizeof(buf), "FB|%.350s", fbText);
    sendRelayUpdate(buf);
}

// 回读限频 (见 RelayCore.h 里的说明; 规格 §4 "回读限频")。
//
// ⚠ 【线程】(2026-09-22 修正): 从前这里写的是"只由 GLUT idle 线程调 ⇒ 不需要原子"。那句是
//   错的, 而且与本文件上方 reportPosition 那一段 (那里明写"本函数跑在触觉实时线程上") 直接
//   矛盾 —— 同一份文件里相隔三十来行, 两说的不是一回事。实际有【两个】线程调进来:
//     · GLUT idle 线程 —— 派发与补发: idle() → pollRelayCommands() → dispatchRelayCommand()
//       的【接受】/【拒绝】两个分支, 以及每帧那次 `if (s_gainReportPending)` 补发;
//     · 【触觉实时线程】—— 重连那一条: HapticCallback 的 reportPosition() →
//       ensureRelayConnected() → 重连成功时 sendReflectionGain(true)。
//   ★ 启动那一次【也不例外】(2026-09-22 再修正): 这里从前写的是"initRelayReporting() 由
//     main() 在 GLUT 主循环【之前】调, 那时还没有并发"—— 那句是【错的】, 而且错法与上面那句
//     "只由 GLUT 线程调"同源: 把"启动时"当成了"单线程"。实际次序是 initHapticDevice() →
//     hdStartScheduler() 【先把触觉线程起了起来】, initRelayReporting() 是之后才调的 ⇒
//     这中间 reportPosition() 已按 Config::RELAY_UPDATE_INTERVAL 在跑, 而那一刻 socket 还是
//     INVALID_SOCKET ⇒ ensureRelayConnected() 里 lastTryMs 初值为 0, 守卫
//     (lastTryMs != 0 && …) 对【首次】调用必然放行 ⇒ 连上就在【触觉线程上】调
//     sendReflectionGain(true), 与 main() 自己在 initRelayReporting() 里的那一次重叠。
//     ⇒ 这个调用点新加的状态同样要并发保护, 不能按"启动时是单线程"推断。
//     （前提: 触觉设备已启用。`--no-touch` 下 initHapticDevice() 不跑 ⇒ 那条启动路径确实是
//       单线程的; 上面的保守结论不受影响 —— 多一层原子没有代价。）
//   ⇒ 下面三个状态必须是 atomic: 两个线程不同步地读写同一个非原子对象就是数据竞争 (UB)。
//   残留的只是【次序】上的竞争, 而且无害 —— 这三个状态【不参与强制发送的决策】:
//     · force=true 一个判断都不从它们取 (只写) ⇒ 交错最坏 = 多回一条、或晚回一条;
//     · s_gainReportPending 丢一次更新是【自愈】的 —— 那条强制发送本身已经带上了当前值。
//
// force=false 时【两条都成立才发】(规格原文: "只在目标值真的变了、且距上次回读 ≥100ms"):
//   ① 目标值相对【上次真的发出去的那一条】变了 —— 没变就没有可报的东西
//   ② 距上次【发送】≥100ms —— 拖动滑条几十条/秒, 逐条回读会堆在 MATLAB 侧
// 被挡下的那一条在这里只置标志, 由 pollRelayCommands 每帧补发 ⇒ 最后一条一定到。
//
// ⚠ 【调用点不是随便挑的】(2026-09-22 修正): 上面条件①是一道【过滤被拒回读】的闸 ——
//   force=false 只能用于【接受】(拖动洪水) 与 pollRelayCommands 的补发;
//   dispatchRelayCommand 的【拒绝】分支必须走 force=true。理由:
//   被拒 = setGain 在 store 之前就返回 = 值按构造没变 ⇒ 条件①必然命中 ⇒ 一个字节都发不出去。
//   按调用点逐个说明见 RelayCore.h 的 sendReflectionGain 文档块。
//
// ⚠ ①②里的"上次真的发出去" = 【发送这一步】, 不是"确认送达": s_lastGainReportMs 与
//   s_lastSentGain 都落笔在 sendRelayUpdate 调用【之前】(见下面的赋值), 所以 socket 恰在那一
//   瞬间失效时, 这次算"发过了" —— 同一个值的重试会被条件①挡下。这是【已知且能收敛】的:
//   重连成功时的强制回读 (force=true) 不看这两条闸, 会把当前值原样再送一条 ⇒ 值最终一定到
//   MATLAB。⇒ 按这个定义读这两个名字, 别按"确认送达"读。
//
// s_lastSentGain 初值刻意选 0 —— 那是 setGain 不会接受的值 ⇒ 在第一次 force=true 之前
//   若有人用 force=false 进来, 它一定发得出去 (保守方向)。
static std::atomic<DWORD>  s_lastGainReportMs{0};
static std::atomic<double> s_lastSentGain{0.0};
static std::atomic<bool>   s_gainReportPending{false};

void RelayCore::sendReflectionGain(bool force) {
    const DWORD now = GetTickCount();
    const double g = ForceTuning::gain();
    // 判决【不在本文件里】—— 抽成了纯函数 (relay/GainReadbackPolicy.h), 理由与
    //   force 必须第一道的说明都写在那里, 并由 test_gain_readback_policy 钉住。
    //   本文件只负责: 取值、传参、按判决改这三样状态 (sent 值 / 上次发送时刻 / 待发标志)。
    switch (GainReadbackPolicy::gainReportDecision(
                force, g, s_lastSentGain.load(), now,
                s_lastGainReportMs.load(), GainReadbackPolicy::GAIN_REPORT_MIN_INTERVAL_MS)) {
    case GainReadbackPolicy::GainReport::SkipUnchanged:
        // ⚠ 必须【顺手清掉待发标志】: 一条被限频挡下的 A→B 之后值又变回 A, 此时"待发"已
        //   无事可做; 留着标志会让 pollRelayCommands 每帧都调进来、每帧都从这里返回 ⇒
        //   标志卡在 true 再也不动 (无害, 但那个标志从此失去意义)。
        s_gainReportPending.store(false);
        return;
    case GainReadbackPolicy::GainReport::SkipTooSoon:
        // 记下待发, 由 pollRelayCommands 补 —— 最后一条不丢。
        // ⚠ 这里【不】更新 s_lastGainReportMs: 它记的是"上次真的发出去"的时刻 (见头文件)。
        s_gainReportPending.store(true);
        return;
    case GainReadbackPolicy::GainReport::Send:
        break;   // 落到下面发送
    }
    s_gainReportPending.store(false);
    s_lastGainReportMs.store(now);
    s_lastSentGain.store(g);

    // ⚠ 载荷的 7 个字段【按位置】解析: RG| 是逗号分隔的定长字段 (C++→MATLAB 的其它线路
    //   都是这个形状), 规格 §4 只定义了【顺序】, 字段没有名字。顺序必须与 §4 的字段表
    //   逐字一致, 否则 MATLAB 侧会把每个数都读错位 —— 而本侧没有任何单测能发现这件事
    //   (回读的消费方在 MATLAB, Task 6)。
    //   依次是: 1 gain = 当前生效目标值       (ForceTuning::gain())
    //           2 min  = 可取范围下限         (ForceTuning::GAIN_MIN, 唯一一份定义)
    //           3 max  = 可取范围上限         (ForceTuning::GAIN_MAX, 唯一一份定义)
    //           4 ratio = 净比例 = 逐单位比例×gain (ForcePipeline::netRatioPerGainUnit)
    //           5 deadN = 死区                (Config::FORCE_RESIDUAL_DEADZONE_N, 唯一一份定义)
    //           6 satN  = 该轴打顶阈值        (ForcePipeline::saturationSensorN, 唯一一份定义)
    //           7 defGain = 出厂默认          (ForceTuning::defaultGain())
    //   上述四个 ratio/deadN/satN 全部取自各自【唯一一份定义】, 本处一个数字都不写。
    char buf[160];
    snprintf(buf, sizeof(buf), "RG|%.6g,%.6g,%.6g,%.4f,%.6g,%.6g,%.6g",
             g,
             ForceTuning::GAIN_MIN,
             ForceTuning::GAIN_MAX,
             ForcePipeline::netRatioPerGainUnit() * g,   // ratio
             Config::FORCE_RESIDUAL_DEADZONE_N,          // deadN
             ForcePipeline::saturationSensorN(g),        // satN
             ForceTuning::defaultGain());                // defGain
    sendRelayUpdate(buf);
}

bool RelayCore::consumeForceZeroRequest() {
    return m_forceZeroRequested.exchange(false);
}

void RelayCore::dispatchRelayCommand(const char* line) {
    using R = RelayCommandParser::Command;
    double value = 0.0;
    switch (RelayCommandParser::parse(line, &value)) {
    case R::ForceFeedbackOn:
        appState.forceFeedbackEnabled = true;
        std::cout << "[Relay] Force feedback ENABLED (MATLAB command)" << std::endl;
        break;
    case R::ForceFeedbackOff:
        appState.forceFeedbackEnabled = false;
        std::cout << "[Relay] Force feedback DISABLED (MATLAB command)" << std::endl;
        break;
    case R::SetReflectionGain:
        // 【不论接受还是拒绝都回读】—— 回的都是当前实际生效值 (规格 §4)。
        // 但两条路的形态【不同】, 而且必须不同 (2026-09-22 修正):
        if (ForceTuning::setGain(value)) {
            std::cout << "[Tuning] 力反射增益 → " << value << " (MATLAB command)" << std::endl;
            // 【接受】⇒ 目标值真的变了 ⇒ 限频形态 (force=false)。
            // 规格 §4 那两条条件 ("只在目标值真的变了、且距上次回读 ≥100ms") 写的正是
            // 这条拖动路径: 拖动滑条每秒几十条 RG|, 逐条回读会堆在 MATLAB 侧。
            // 被时间挡下的那条由 pollRelayCommands 补发, 最后一条一定到。
            sendReflectionGain(false);
        } else {
            // 拒收必须出声, 而且要说清范围 —— 范围取自 ForceTuning 那一份定义, 不另写数字。
            std::cout << "[Tuning] 增益 " << value << " 【被拒】: 可取范围 ["
                      << ForceTuning::GAIN_MIN << ", " << ForceTuning::GAIN_MAX
                      << "], 仍是 " << ForceTuning::gain() << std::endl;
            // 【拒绝】【必须无条件发】—— force=true。这就是 true 在本设计里的第三个用途
            // (连接、重连之外的第三个):
            //   · 被拒 ⇒ ForceTuning::setGain 在【任何 store 之前】就 return false
            //     ⇒ 生效值【按构造】没有变。
            //   · 而 force=false 的第一道闸就是 "g == s_lastSentGain ⇒ 不发"
            //     ⇒ 那道闸在拒绝路径上【必然】命中 ⇒ 一个字节都发不出去。
            //   · 偏偏这条恰恰是 MATLAB 最需要的一条: 它自己的滑条【已经动了】(到那个被拒的值),
            //     正等着被纠正回真值 —— 规格 §4 的中心例子就是它:
            //     "若它发了 500 被拒, 回读仍是 120, 滑条自己弹回 120"。
            //   ⚠ 【不要把它"优化"成 false】。值没变对"拖动洪水"是对的判据 (那是为了限频),
            //     但对"拒绝"是【反的】: 拒绝的定义就是值没变 ⇒ 拿值变没变当闸门, 恰好滤掉了
            //     唯一一条必须发出去的回读 ⇒ MATLAB 会永远显示一个假数 (规格 §4 末尾那段:
            //     滑条上限来自回读, GAIN_MAX 在 C++ 侧调低后 MATLAB 会一直发一个已被拒的值)。
            //     合上这条闸只省下一次 160 字节的发送, 代价是那个不变量失效。
            //   （判决本身已抽到 relay/GainReadbackPolicy.h, 由 test_gain_readback_policy 钉住。）
            sendReflectionGain(true);
        }
        break;
    case R::ForceZero:
        // 只置标志: 真正的处置在 main.cpp 的 requestForceZero() (与键盘 'z' 同一个函数)。
        // 为什么不在这个线程直接做 —— 见 RelayCore.h 里 consumeForceZeroRequest 的说明。
        m_forceZeroRequested.store(true);
        std::cout << "[Tuning] 收到 MATLAB 的调零请求" << std::endl;
        break;
    case R::None:
    default:
        // 【这是决定, 不是遗漏】(2026-09-22 拍板): 形状坏掉的 RG| 行 (如 "RG|abc") 在
        //   RelayCommandParser 里塌成 Command::None, 与未知命令无法区分 ⇒ 不回读。
        //   · 规格 §4 的"不论接受还是拒绝"指的是【被 setGain 按范围拒收】, 那条路走上面的
        //     SetReflectionGain 分支, 已覆盖; 解析器【故意】放行超范围的数值。
        //   · 规格真正的不变量是"MATLAB 显示的值不可能与实际生效值分叉"。畸形命令下
        //     MATLAB 自己也没改显示值 ⇒ 没有分叉 ⇒ 不变量成立。
        //   · sprintf('RG|%.4f', v) 配上 Limits 约束的数值框, 产不出畸形行。
        // ⇒ 【不要】在这里回头去判 "RG|" 前缀: 那是把协议知识搬回错误的层 (解析器已经在
        //   那一层认识它了), 会变成第二份实现。
        break;
    }
}

void RelayCore::pollRelayCommands() {
    // 力反射增益的两件家常事 (2026-09-22):
    //   tick()      —— 防抖落盘 (值变过且静默 ≥1s 才写盘)。借本循环当心跳, 不新起线程/定时器。
    //   补发待发的回读 —— 拖动滑条时被限频挡下的那一条, 在这里补上, 保证"最后一条一定到"。
    //   ⚠ 补发本身【仍然】受那两条条件管: 本循环每帧都跑, 若这一帧距上次发送仍不足 100ms,
    //     补发会被再次挡下 (标志保持 true), 再过几帧才真的发出去 —— 这是有意的, 不是漏发。
    // ⚠ 放在 socket 有效性检查【之前】: 这两件事与 relay socket 在不在无关 (tick 只管落盘),
    //   放在后面会在 GUI 没连上时整个停掉。
    ForceTuning::tick();
    if (s_gainReportPending.load()) sendReflectionGain(false);

    EnterCriticalSection(&m_relaySocketMutex);
    SOCKET sock = m_relaySocket;
    LeaveCriticalSection(&m_relaySocketMutex);
    if (sock == INVALID_SOCKET) return;

    // 非阻塞检查可读数据
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(sock, &readfds);
    timeval tv = { 0, 0 };
    if (select(0, &readfds, nullptr, nullptr, &tv) <= 0) return;

    char tmp[256];
    int n = recv(sock, tmp, sizeof(tmp), 0);
    if (n <= 0) return;

    // 追加到接收缓冲, 逐行分发
    for (int i = 0; i < n; ++i) {
        char c = tmp[i];
        if (c == '\n') {
            m_relayRecvBuf[m_relayRecvLen] = '\0';
            dispatchRelayCommand(m_relayRecvBuf);
            m_relayRecvLen = 0;
        } else if (c != '\r' && m_relayRecvLen < (int)sizeof(m_relayRecvBuf) - 1) {
            m_relayRecvBuf[m_relayRecvLen++] = c;
        }
    }
}

void RelayCore::sendSafetyStatus() {
    const auto& sm = m_stateMachine;
    char buf[128];
    snprintf(buf, sizeof(buf), "S|%d,%.2f,%d",
        static_cast<int>(sm.currentState()),
        sm.speedFactor(),
        SafetyPredictor::instance().alarmCount());
    sendRelayUpdate(buf);
}

void RelayCore::sendJointMargins() {
    static const double jlims[6][2] = {
        {-360,360},{-360,360},{-155,155},{-360,360},{-360,360},{-360,360}};
    auto& app = appState;
    EnterCriticalSection(&app.robotPoseMutex);
    double jv[6]={app.robotActualPose.j1,app.robotActualPose.j2,
        app.robotActualPose.j3,app.robotActualPose.j4,
        app.robotActualPose.j5,app.robotActualPose.j6};
    LeaveCriticalSection(&app.robotPoseMutex);
    char buf[128];
    snprintf(buf,sizeof(buf),"L|%.1f,%.1f,%.1f,%.1f,%.1f,%.1f",
        fmin(fabs(jv[0]-jlims[0][0]),fabs(jlims[0][1]-jv[0])),
        fmin(fabs(jv[1]-jlims[1][0]),fabs(jlims[1][1]-jv[1])),
        fmin(fabs(jv[2]-jlims[2][0]),fabs(jlims[2][1]-jv[2])),
        fmin(fabs(jv[3]-jlims[3][0]),fabs(jlims[3][1]-jv[3])),
        fmin(fabs(jv[4]-jlims[4][0]),fabs(jlims[4][1]-jv[4])),
        fmin(fabs(jv[5]-jlims[5][0]),fabs(jlims[5][1]-jv[5])));
    sendRelayUpdate(buf);
}

void RelayCore::sendSingularity() {
    auto& app = appState;
    EnterCriticalSection(&app.robotPoseMutex);
    double x=app.robotActualPose.x, y=app.robotActualPose.y;
    LeaveCriticalSection(&app.robotPoseMutex);
    double r_xy=sqrt(x*x+y*y);
    char buf[64];
    snprintf(buf,sizeof(buf),"G|%.1f,%d",r_xy,(r_xy<30.0)?1:0);
    sendRelayUpdate(buf);
}

void RelayCore::sendCalibStatus() {
    char buf[64];
    snprintf(buf,sizeof(buf),"B|%d,%.2f",
        Calibration::enabled?1:0, Calibration::enabled?Calibration::rmsError:-1.0);
    sendRelayUpdate(buf);
}

void RelayCore::sendConnectionHealth() {
    // Track process start time on first call
    if (m_processStartMs == 0) {
        m_processStartMs = GetTickCount();
    }

    auto& app = appState;
    bool enableOk = app.isRobotConnected.load();

    // Motion port: same socket lifecycle as enable
    bool motionOk = enableOk;

    // Force port: consider connected if data isn't stale
    bool forceOk = false;
    EnterCriticalSection(&app.forceDataMutex);
    forceOk = !app.forceData.isStale;
    LeaveCriticalSection(&app.forceDataMutex);

    float pingMs = app.latencyMs.load();
    DWORD uptimeS = (GetTickCount() - m_processStartMs) / 1000;

    char buf[128];
    snprintf(buf, sizeof(buf), "H|%d,%d,%d,%.1f,%u",
        enableOk ? 1 : 0,
        motionOk ? 1 : 0,
        forceOk ? 1 : 0,
        pingMs,
        uptimeS);
    sendRelayUpdate(buf);
}

void RelayCore::reportDiagnostic(int errorCode, double speedFactor, const char* reason) {
    char buf[256];
    snprintf(buf,sizeof(buf),"D|%d,%.2f,%.200s",errorCode,speedFactor,reason?reason:"");
    sendRelayUpdate(buf);
}

// ===== ForceReader 管理 =====

// 拖拽模式 = Dobot 的 SetCollideDrag。开着时机械臂柔顺可手动拖动,
// 但姿态会漂, 所以采样/标定期间必须关掉。
bool RelayCore::setDragMode(bool enable) {
    if (!isRobotConnected()) {
        // 重连会重建连接, 拖拽状态随之归零 —— 别留一个「以为开着」的假状态,
        // 否则掉线时按 'd' 会永远切不回来。
        m_dragMode.store(false);
        return false;
    }
    if (m_dragMode.load() == enable) return true;   // 已是该状态, 不重复下发
    robotSendEnable(enable ? "SetCollideDrag(1)" : "SetCollideDrag(0)");
    Sleep(100);
    robotDrainEnable();
    m_dragMode.store(enable);
    std::cout << "[Relay] Drag mode " << (enable ? "ON — 可手动拖动机械臂" : "OFF — 位姿锁定")
              << std::endl;
    return true;
}

// Drag mode callback for ForceCalibration
static void calibDragMode(bool enable) {
    RelayCore::instance().setDragMode(enable);
}

// ===== 把负载参数显式下发给机械臂 (Task 8a) =====
// 两条命令与它们的出处、以及"只由用户显式触发"这条约束, 都写在上面 sendPayloadCommands 顶上。
// 这里只做转发, 与 triggerEscape() / setDragMode() 同风格。
//
// 【为什么这里要 touchHeartbeat()】这两条命令是【逐条等回执】的 (每条: Sleep(100) + 最多
// 200ms 的 socket 接收超时 —— 使能口的 SO_RCVTIMEO 见 robot/RobotConnection.cpp), 两条加
// 起来的阻塞时间可以超过 Config::HEARTBEAT_TIMEOUT_MS (500ms)。而本函数跑在 GLUT 主线程上,
// 心跳检查在 pollFeedback() 里、也跑在同一个线程 —— 阻塞期间它根本不会跑, 恢复后第一帧就会
// 撞见过期的心跳, 误报 ERR_HEARTBEAT_LOST (FATAL, 会下使能)。touchHeartbeat() 正是为
// "故意阻塞主线程"的操作准备的 (见 RelayCore.h 里它的说明): 阻塞结束后声明自己还活着。
bool RelayCore::sendPayloadToRobot(double massKg, const double comMm[3]) {
    const bool ok = sendPayloadCommands(massKg, comMm);
    touchHeartbeat();
    return ok;
}

bool RelayCore::initForceReader() {
    if (!isRobotConnected()) {
        std::cout << "[Force] Robot not connected, skipping ForceReader" << std::endl;
        return false;
    }
    ForcePipeline::init();
    ForceCompensation::init();
    ForceCalibration::setDragModeCallback(calibDragMode);
    // 帧率噪声探针的锁 —— 【必须在线程起来之前】初始化: 线程一跑就会 pushForceFrame。
    // pushForceFrame 里还有一道 s_noiseLockInit 兜底 (最早期几帧字面丢弃), 因为
    // "先初始化再建线程"这件事本身没有编译期保证。
    if (!s_noiseLockInit) {
        InitializeCriticalSection(&s_noiseLock);
        s_noiseLockInit = true;
    }
    if (!ForceLogger::open(Config::FORCE_LOG_PATH)) {
        std::cerr << "[Force] Failed to open force log " << Config::FORCE_LOG_PATH << std::endl;
    }
    m_forceThread = CreateThread(NULL, 0, forceReaderThread, NULL, 0, NULL);
    if (!m_forceThread) {
        std::cerr << "[Force] Failed to create ForceReader thread" << std::endl;
        return false;
    }
    return true;
}

// 帧率噪声探针读出: 最近 ≤maxN 帧, 按【从旧到新】。见 RelayCore.h 里那一段。
// ⚠ 【从旧到新】不是随便定的: 块平均、自相关都要时间顺序; 反过来算出来的自相关是共轭的,
//   数值上看着一样但"哪个方向领先"就没了 —— 而正是那个方向说明"噪声是宽带还是慢漂"。
// 锁内拷贝 (≤64 KB memcpy, 几微秒): 写侧是 125 Hz, 等这几微秒没有影响。
int RelayCore::copyRecentForceFrames(ForceFrameSample* out, int maxN) {
    if (!out || maxN <= 0 || !s_noiseLockInit) return 0;
    EnterCriticalSection(&s_noiseLock);
    int n = (s_noiseCount < maxN) ? s_noiseCount : maxN;
    // 写指针指向"下一个待写" ⇒ 最旧的那一帧在 write − n 处 (模容量, 处理回绕)
    const int start = ((s_noiseWrite - n) % kNoiseCap + kNoiseCap) % kNoiseCap;
    for (int i = 0; i < n; i++) out[i] = s_noiseBuf[(start + i) % kNoiseCap];
    LeaveCriticalSection(&s_noiseLock);
    return n;
}

void RelayCore::pollForce() {
    static DWORD lastPollMs = 0;
    DWORD now = GetTickCount();
    // 节拍取自 Config::FORCE_POLL_INTERVAL_MS。
    // ★ 2026-09-21: 它的含义【变了】—— 从前它还是"力处理链的采样率"(补偿与滤波都在本函数里),
    //   现在那两样已经搬到 ForceReader 线程 (帧率) 上跑, 所以本函数的节拍【只决定】
    //   落盘 (ForceLogger) 与发给 MATLAB 的 F| 帧这两个输出口的更新率。
    //   ⇒ 本函数里【不许再出现】任何"按固定采样率换算"的东西 (这是它从前的老毛病:
    //     EMA 的 α 就曾按这个常数换算, 而实测节拍是 46~203 ms —— 见 ForceCompensation 的
    //     stepIntervalSec / feedMotionEstimator, 那两处现在用的是【实测耗时】)。
    if (now - lastPollMs < (DWORD)Config::FORCE_POLL_INTERVAL_MS) return;
    lastPollMs = now;

    auto& app = appState;

    // Read current pose for compensation
    double pose[6] = {0};
    EnterCriticalSection(&app.robotPoseMutex);
    pose[0] = app.robotActualPose.x;
    pose[1] = app.robotActualPose.y;
    pose[2] = app.robotActualPose.z;
    pose[3] = app.robotActualPose.rx;
    pose[4] = app.robotActualPose.ry;
    pose[5] = app.robotActualPose.rz;
    LeaveCriticalSection(&app.robotPoseMutex);

    EnterCriticalSection(&app.forceDataMutex);

    // Staleness check
    if (app.forceData.lastUpdateMs > 0 &&
        (now - app.forceData.lastUpdateMs) > static_cast<DWORD>(Config::FORCE_STALE_MS)) {
        app.forceData.isStale = true;
        for (int i = 0; i < 6; i++) app.forceData.filtered[i] = 0.0;
        for (int i = 0; i < 6; i++) app.forceData.compensated[i] = 0.0;
        for (int i = 0; i < 3; i++) app.forceData.hapticOut[i] = 0.0;
    }

    // Run calibration state machine if active.
    // ⚠ 喂的是 @1304 (sixForceRaw) —— 调零定的零偏是【全量模型那个通道】的零偏
    //   (ForceCompensation::step 现在补偿 @1304, 见 Task 6)。继续喂 @576 会让 TARE
    //   平均出另一路量的零偏, 而两路的零偏不是一回事 (12:38 那份夹具上逐轴均值差
    //   19.8 / 1.6 / 1.7 N, 见 tests/fixtures/calib_poses_2026-09-19.txt):
    //   扣错以后读数依旧是个 N, 不报错。
    // ★ 2026-09-22: 【去掉 isRunning() 的门, 改成无条件调用】。
    //   两件事一起改的, 不能只改一件:
    //   ① update() 自己测"距上次调用的实测耗时"当 dt (从前是调用方传常数 0.033,
    //      而真实节拍 46~203ms ⇒ 静默期被拉长 ~2.7 倍)。
    //   ② 那个计时器靠【每次轮询都被调用】保持新鲜。若继续用 isRunning() 门着,
    //      计时器会停在"上一次标定运行"那一刻 —— 下次按 'z' 时第一个 dt 就是那之间的
    //      全部时间 (几分钟), 一步跨过静默期与累计期, 而且【不会报任何错】。
    //   非运行态下 update() 自己早退 (只做一次 GetTickCount), 所以无条件调用无副作用。
    ForceCalibration::update(app.forceData.sixForceRaw, pose);

    // ★★ 2026-09-21: 补偿 (ForceCompensation::step) 与滤波/映射 (ForcePipeline::step) 已经
    //   【搬到 ForceReader 线程里跑】—— 见 forceReaderThread 里那一段的说明与
    //   Config::FORCE_FILTER_CUTOFF / FORCE_FILTER_FS_HZ 两段。本函数【不要再调它们】:
    //   每调一次就推进一次滤波器状态, 11 Hz 这一路再推一次会把 123 Hz 的结果又滤一遍,
    //   相位与幅值全乱 —— 而且不会有任何报错 (这里【只剩读】)。
    // ⚠ 一致性闸门的【判决】仍在 step() 里 (不通过的帧 compensated 被置零);
    //   下面这个 guardSt 是【最近一帧的】状态, 用于报错, 【不用于判决】。
    // ⚠ 【虚拟约束力不受影响】: 它在 HapticCallback.cpp:168 由【位置】现算
    //   (SafetyPredictor::computeConstraintForce), 与 compensated 无关 —— 别写成
    //   "触觉 / 约束力 / F| 一起断", 那会让人以为拒绝之后连安全边界的推手都没了。
    const ForceCompensation::GuardState guardSt = ForceCompensation::guardState();

    // Build F| protocol message — send filtered[] with deadzone applied
    char buf[128];
    if (app.forceData.isStale) {
        snprintf(buf, sizeof(buf), "F|0.00,0.00,0.00,0.00,0.00,0.00,1");
    } else {
        // ★★ 2026-09-21: 这里从前【又写了一份硬门】: `(fabs(x) < dz) ? 0.0 : x`。
        //   触觉那一路 (ForcePipeline::mapForceToTouch) 已经改成软门了, 而这一份没改
        //   ⇒ **MATLAB 显示的那条 F| 路上照旧阶跃** —— 现场看到的正是"MATLAB 上 FZ 在 0 与
        //   ±0.2 之间阶跃式跳"; 用户因此报"这个也需要改"。
        //   ⇒ 现在两处【都调 ForcePipeline::softDeadzone】—— 死区的唯一一份定义在 ForcePipeline.h。
        //   ⚠ 别再在这里写第三份: 同一个规则两份实现, 就是会改一份忘一份 (本项目第三笔了)。
        //   注: 力矩 (filtered[3..5]) 从前就【不过死区】, 保持原样 (不在本次讨论范围)。
        double fx = ForcePipeline::softDeadzone(app.forceData.filtered[0], Config::FORCE_RESIDUAL_DEADZONE_N);
        double fy = ForcePipeline::softDeadzone(app.forceData.filtered[1], Config::FORCE_RESIDUAL_DEADZONE_N);
        double fz = ForcePipeline::softDeadzone(app.forceData.filtered[2], Config::FORCE_RESIDUAL_DEADZONE_N);
        snprintf(buf, sizeof(buf), "F|%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d",
            fx, fy, fz,
            app.forceData.filtered[3], app.forceData.filtered[4],
            app.forceData.filtered[5],
            app.forceData.isStale ? 1 : 0);
    }

    // Copy filtered forces into a local before releasing the lock so the
    // log write (stdio buffering / disk) doesn't hold forceDataMutex across I/O.
    double filtered[6];
    for (int i = 0; i < 6; i++) filtered[i] = app.forceData.filtered[i];

    LeaveCriticalSection(&app.forceDataMutex);

    // 落盘到 CSV (演示对照实验用, 含 ff_enabled 标志列)。
    // TCP 偏移标定过之后记录笔尖世界坐标而不是法兰坐标 —— 演示要看的是笔尖
    // 在纸上的轨迹和受力, 法兰位姿差着笔长 + 夹持段 + 传感器高度 (约 100~200mm)。
    double logPose[6];
    if (TcpCalibration::enabled) {
        double tip[3];
        TcpCalibration::apply(pose, TcpCalibration::offset, tip);
        logPose[0] = tip[0]; logPose[1] = tip[1]; logPose[2] = tip[2];
        logPose[3] = pose[3]; logPose[4] = pose[4]; logPose[5] = pose[5]; // 姿态同法兰
    } else {
        for (int i = 0; i < 6; i++) logPose[i] = pose[i];
    }
    // 运动估计器的状态也落盘 (2026-09-21, 追加在行末两列/四字段)。理由见 ForceLogger.h 表头说明:
    // Fi = mass·acc 的运动时误差【只能靠外部反算】验证 —— 本文件的 pose 列二阶差分就是真加速度,
    // 与 acc 列一比就知道 step() 里 dt 那处修对了没有。两次手拖比 ΔFi 是不可比的, 必须同一运动自洽比较。
    // ⚠ 用【既有的】ForceCompensation::motionState(vel, acc) —— 它一次给出加速度与 isStill
    //   判定; 而它自己的注释里本来就写着"update() 的 dt 与 pollForce() 实际采样节奏不符",
    //   那正是 2026-09-21 修掉的那件事。本处【不另加访问器】(我一度加过 lastMotionAcc /
    //   lastMotionStill, 那两个与它功能重复, 已撤回)。
    double motionVel[3] = {0, 0, 0}, motionAcc[3] = {0, 0, 0};
    const bool motionStill = ForceCompensation::motionState(motionVel, motionAcc);
    ForceLogger::log(now, filtered, logPose, appState.forceFeedbackEnabled ? 1 : 0,
                     motionAcc, motionStill ? 1 : 0);

    sendRelayUpdate(buf);

    // ===== 一致性闸门的报错 (2026-09-19) =====
    // 走现成通道: RobotDiagnostics 记一条 (落 robot_diagnostics.log + 计数进会话报告),
    // 并经 reportDiagnostic 把 D| 帧发给 MATLAB GUI。data 侧已经由 step() 无条件置零,
    // 这里只管【把原因说清楚】。
    // 【三种原因用三个错误码】—— 处置一样 (都拒绝), 但操作员要做的事不同:
    //   ERR_FORCE_UNCALIBRATED -> 去按 'm'+'s' 重标模型;
    //   ERR_FORCE_INCONSISTENT -> 去查负载参数有没有真的发进机械臂 (Task 8);
    //   ERR_FORCE_REFERENCE_UNAVAILABLE (2026-09-21 Task 7) -> 去查参考量这一路为什么没有数据
    //     (30004 帧 / 六维力在线状态) —— 【不是】前两件事中的任何一件。
    // 合并成一个码会让这几件事在日志里长得一样, 而"该做什么"全靠这一位区分。
    // ⚠ 状态 -> 错误码的映射【只有一份实现】: ForceCompensation::guardErrorCode (见那里的
    //   说明)。这里从前是 static_cast<int>(guardState()) 比字面量 1 / 2 —— 一改枚举的
    //   数值就会把两条处置指引对调, 而且没有任何测试看得见。
    // 【只在状态变化时报, 不变的按 FORCE_GUARD_REPORT_MS 复报】—— 闸门每帧都判 (30Hz),
    // 每帧落一行会把诊断日志冲掉。
    {
        static int   lastGuardSt = -1;
        static DWORD lastGuardMs = 0;
        if (static_cast<int>(guardSt) != lastGuardSt ||
            (guardSt != ForceCompensation::GuardState::OK &&
             (now - lastGuardMs) > static_cast<DWORD>(Config::FORCE_GUARD_REPORT_MS))) {
            lastGuardSt = static_cast<int>(guardSt);
            lastGuardMs = now;
            if (guardSt != ForceCompensation::GuardState::OK) {
                RobotError err;
                err.code = ForceCompensation::guardErrorCode(guardSt);
                err.severity = getSeverity(err.code);
                err.timestampMs = GetTickCount64();
                EnterCriticalSection(&app.robotPoseMutex);
                err.targetPosition = Vec3(app.robotActualPose.x, app.robotActualPose.y,
                                          app.robotActualPose.z);
                LeaveCriticalSection(&app.robotPoseMutex);
                err.speedFactor = 0.0;   // 本帧的力数据未放行, 不参与任何速度调度
                RobotDiagnostics::instance().logError(err, 0.0, m_stateMachine.currentState());
            }
        }
    }
}

void RelayCore::shutdownForceReader() {
    if (m_forceThread) {
        WaitForSingleObject(m_forceThread, 1000);
        CloseHandle(m_forceThread);
        m_forceThread = NULL;
    }
    robotCloseRealtime();
    ForceLogger::close();
    ForcePipeline::shutdown();
    ForceCompensation::shutdown();
}

// ===== 力传感器标定控制 =====

// 力标定/调零的公共前置检查: 未传输中 + 已连接 + 未报警
static bool forceCalibPreconditions(bool transmitting, const char* what) {
    if (transmitting) {
        std::cout << "[Force] Cannot " << what
                  << " while transmitting — release button first" << std::endl;
        return false;
    }
    if (!isRobotConnected()) {
        std::cout << "[Force] Robot not connected, cannot " << what << std::endl;
        return false;
    }
    if (appState.isRobotInAlarm.load()) {
        std::cout << "[Force] Robot in alarm, cannot " << what << std::endl;
        return false;
    }
    return true;
}

bool RelayCore::startForceCalibration() {
    if (!forceCalibPreconditions(m_transmitting, "calibrate")) return false;
    std::cout << "[Force] Starting calibration sweep..." << std::endl;
    return ForceCalibration::start();
}

bool RelayCore::startForceZeroing() {
    if (!forceCalibPreconditions(m_transmitting, "zero")) return false;
    std::cout << "[Force] Starting zero (TARE only)..." << std::endl;
    return ForceCalibration::startZero();
}

void RelayCore::abortForceCalibration() {
    ForceCalibration::abort();
    std::cout << "[Force] Calibration aborted" << std::endl;
}

bool RelayCore::isForceCalibrating() const {
    return ForceCalibration::isRunning();
}

bool RelayCore::isForceZeroing() const {
    return ForceCalibration::isZeroing();
}

bool RelayCore::isForceCalibrationDone() const {
    return ForceCalibration::isDone();
}

const char* RelayCore::forceCalibStatus() const {
    return ForceCalibration::statusText();
}
