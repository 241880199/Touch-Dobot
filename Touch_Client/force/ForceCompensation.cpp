#define _USE_MATH_DEFINES
#include "ForceCompensation.h"
#include "../calibration/TcpCalibration.h"
#include "../config/Config.h"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <windows.h>

// ===== Internal state =====
static CRITICAL_SECTION g_calibMutex;
static bool g_mutexInit = false;
static bool g_isCalibrated = false;
static double g_A[9] = {0};            // 全量模型的 3×3 力响应 (kg, row-major)
static double g_comSensor[3] = {0};    // c_s (m, 传感器测量系)
static double g_biasForce[3] = {0};
static double g_biasTorque[3] = {0};
static MotionEstimator g_motion;

// 【本帧的姿态】—— 只为闸门那段打印服务 (2026-09-20 加)。
// 闸门报的那六个数是"残差对姿态的依赖"的读数, 而【没有姿态就没法解释它们】: 现场抄数的人
// 不把姿态一起抄下来, 事后就分不开"随姿态变"与"固定偏置", 也拟合不了 M = 残差/g 的各向同性
// (判 H1 还是 H2)。出处: Docs/superpowers/specs/2026-09-20-raw-channel-calibration-run-004.md §4.3。
// ⚠ 用【帧里那一份】姿态 (与算这六个数用的是同一次 30004 帧), 不在这里再问一次 GetPose ——
//   后者会在机械臂动过之后与那六个数对不上, 而"对不上"正是本项目最忌的那种安静地错。
// ⚠ 只在 step() 里写、只在 setGuardState() 里读 —— 两者同线程 (setGuardState 由 step 调),
//   所以不需要锁。
static double g_lastPose[6] = {0};   // {X_mm, Y_mm, Z_mm, Rx_deg, Ry_deg, Rz_deg}
// ⚠ 【有没有"本帧"】。setGuardState 不只在 step() 里被调 —— setCalibration 的拒收路径也调它,
//   而那时根本没有"本帧姿态"。没有这个标志就会打出一行全零, 而它会被读成"机械臂在原点姿态"
//   —— 那是凭空造了一个数。没有就照实说没有。
static bool g_lastPoseValid = false;

// ===== 运行时一致性闸门的状态 (2026-09-19) =====
// 全部由 ForceReader/pollForce 线程访问 (step() 是唯一入口), 与 g_A 那些用 g_calibMutex
// 保护的量不同 —— 这里不加锁, 与 g_motion 同理: 只有一个写者。
static double g_guardEma[6]  = {0};      // compensated − @576 的逐通道 EMA
// ===== 第二组 EMA: compensated − @720 (2026-09-20) —— 【只报不判】 =====
// 为什么加: 厂商接口文档把两个 TCP 力分得很清楚 ——
//     ActualTCPForce @576 = "TCP【传感器】力值"
//     TCPForce       @720 = "TCP力值 (【通过关节电流计算】)"
//   "通过关节电流算力"就必须知道负载 (重力矩 + 惯量矩) ⇒ @720 才反映控制器正在用的负载参数。
//   而一致性闸门比的却是 @576 —— 且项目自己早有一条实测记录
//   (relay/RelayCore.cpp): "改 EnableRobot 的负载, @576 纹丝不动, 为什么还不清楚"。
// ⇒ 把两组差【并排放出来】, 就能直接看出闸门的参考量该是谁:
//     若 (compensated − @720) 显著小于 (compensated − @576) ⇒ 该比 @720。
// ⚠ 【只报不判】: 放行/拒绝的逻辑一个字没动, 也不参与任何容差比较 ——
//   在看清它之前改判据, 就是拿一个没读过的数去改门。
static double g_guardEma720[6] = {0};    // compensated − @720 的逐通道 EMA (只报不判)
static bool   g_guardSeeded720 = false;  // 上面那一组的播种标志
// 逐通道容差。⚠ 【在静态初始化时就填好】, 不留"init() 没跑就是 0"的空档 ——
// 容差为 0 时 |EMA| > 0 都成立, 判决会退化, 而"退化"的方向必须是【拒绝】而不是放行。
static double g_guardTol[6]  = { Config::FORCE_GUARD_TOL_FORCE_N,  Config::FORCE_GUARD_TOL_FORCE_N,
                                 Config::FORCE_GUARD_TOL_FORCE_N,  Config::FORCE_GUARD_TOL_MOMENT_NM,
                                 Config::FORCE_GUARD_TOL_MOMENT_NM, Config::FORCE_GUARD_TOL_MOMENT_NM };
static bool   g_guardSeeded  = false;    // EMA 是否已用第一帧播种
static long   g_guardFrames  = 0;
static ForceCompensation::GuardState g_guardState = ForceCompensation::GuardState::UNCALIBRATED;
static DWORD  g_guardReportMs = 0;       // 上次打印/上报的时刻

// 【哪些通道参与判决】。Fz 【不投票】—— 理由写在报告与 .h 里, 一行摘要:
//   @576 的 z 通道响应实测秩 2 (奇异值 [0.212 0.201 0.008], 第三行比另两行小 8~15 倍,
//   Docs/superpowers/plans/2026-09-19-raw-channel-calibration.md:268-274), 它在【我们唯一
//   有的激励 (重力方向)】上不动。一个动不了的参考既证不了"一致", 也证不了"不一致"。
//   · 让它投票不会让闸门永远通过 (Fx/Fy 在, 现在是 1.7~2.2 N 量级的拒绝);
//   · 却会让闸门【永远拒绝】: 若它对外力也不响应, 则一旦有真实 z 接触, compensated_z
//     有值而 @576_z 恒 ~0, 差值直接超限 —— 那就是用户明确禁止的"永远不通过"。
//   所以处置是【每次都报出它的比较结果, 但不计票】, 不是"静默跳过"。
//   ⚠ 这一列的证据到此为止: "是 @576 报得坏, 还是机械臂 z 补偿太强" 目前【没有分开】
//     (计划书 :273-274 明说"不许猜")。分开之后应把它提升为投票通道。
//
// ⚠⚠ 【这一列不投票代价有多大 —— 给出数, 不要只说"少一道闸门"】(2026-09-19 复审要求)。
//   力矩门【理论上】能给 z 力当后盾: z 上的模型误差 ΔFz 会经 c_s 叉乘出一个力矩误差
//   Δc × ΔF, 其横向分量量级 = |c_s_横向| · ΔFz。但 c_s 的横向分量实测只有
//   【0.47 ~ 0.78 mm】(四份拟合的 sqrt(cs_x²+cs_y²), 由 test_runtime_consistency_guard_replay
//   现算打印), 而力矩容差是 tol_M = 0.03 N·m ⇒ 若 c_s 向量的横向分量恰好是对的那个方向,
//   ΔFz 要到 tol_M / |c_s_横向| ≈ 0.03 / 0.00078 ~ 0.03 / 0.00047 = 【38 ~ 64 N】量级
//   才能把力矩顶超限。也就是说: z 力方向的模型误差要靠力矩通道兜住, 得大到几十牛 ——
//   本闸门实际上【看不见 z 方向的力模型错误】。这是本次改动里【最大的一处已知漏洞】,
//   明确交给用户定夺 (是补一个 z 的独立判据, 还是接受这个洞), 不是可以靠调容差解决的。
//   (数字来源: 同一份测试打印的两列 —— 逐份 |c_s_横向| 与 tol_M/|c_s_横向|。)
static const bool g_guardVote[6] = { true, true, false, true, true, true };

// A 的可用性判据 —— 见头文件声明。|det| / ||A||_F³ 对"标量质量 × 正交"这一族恒为
// 1/(3√3) = 0.19245, 而秩亏时趋于 0。
// ⚠ 取 1e-3 是【很松】的一道: 它挡的是"几乎完全秩亏", 不是"一般病态"。
//   把门限翻译成条件数 (cond = σ1/σ3, 同一个 cond 下比值最大的形状是 σ1=σ2=σ3·cond):
//   比值 = 1/(2.828·cond) ⇒ 1e-3 对应的 cond ≈ 354。也就是说: 只有当 cond ≳ 354
//   才【必然】被拦下; cond 在 350 以内的矩阵里总有一些形状能过, 无论它多病态。
//   用户指令 3 真正点名的"全零 A"由上面那条 allZero 分支挡住 (与这个比值无关),
//   所以这里不必收紧; 但【消息里不许再说"至少一个力方向没有模型"】: 那句话描述的是秩亏,
//   而这条判据拦的是"离秩亏还差三个数量级"的东西。
static const double GUARD_MIN_DET_RATIO = 1e-3;

// 质量尺度 m = |det A|^(1/3)。A = m·S·Q (见 PayloadCalibration::decompose), 所以它的
// 三个奇异值的几何平均恰好是 m —— 而几何平均 = |det|^(1/3), 不需要 SVD。
// 就是 decompose() 报的 massScale, 这里现算一份的理由见头文件 currentMassKg。
static double massScaleOf(const double A[9]) {
    const double det = A[0] * (A[4] * A[8] - A[5] * A[7])
                     - A[1] * (A[3] * A[8] - A[5] * A[6])
                     + A[2] * (A[3] * A[7] - A[4] * A[6]);
    return cbrt(fabs(det));
}

// ===== Euler angles (deg) to rotation matrix =====
// ⚠ 本文件里【当前没有调用方】了: 重力的唯一去处已改成 TcpCalibration::gravitySensorFrameAtYaw
//   (约定只能有一份实现)。此函数与下面的 matTransposeMulVec 保留未删 —— 删不删由所有者定;
//   但【不要】再用它们在这里重新展开一遍重力, 那正是本文件以前和求解器各写一份的老毛病。
// R = Rz(rz_deg) * Ry(ry_deg) * Rx(rx_deg)
// Output: 3x3 row-major R[9]
static void eulerToRotation(double rx_deg, double ry_deg, double rz_deg, double R[9]) {
    double rx = rx_deg * M_PI / 180.0;
    double ry = ry_deg * M_PI / 180.0;
    double rz = rz_deg * M_PI / 180.0;

    double cx = cos(rx), sx = sin(rx);
    double cy = cos(ry), sy = sin(ry);
    double cz = cos(rz), sz = sin(rz);

    // Rz * Ry * Rx  (row-major)
    R[0] = cz * cy;
    R[1] = cz * sy * sx - sz * cx;
    R[2] = cz * sy * cx + sz * sx;
    R[3] = sz * cy;
    R[4] = sz * sy * sx + cz * cx;
    R[5] = sz * sy * cx - cz * sx;
    R[6] = -sy;
    R[7] = cy * sx;
    R[8] = cy * cx;
}

// Matrix-vector multiply: out = M^T * v  (3x3 row-major M, 3-vector v)
// ⚠ 同 eulerToRotation: 当前无调用方 (重力的唯一去处已改成共享函数), 保留未删。
static void matTransposeMulVec(const double M[9], const double v[3], double out[3]) {
    out[0] = M[0] * v[0] + M[3] * v[1] + M[6] * v[2];
    out[1] = M[1] * v[0] + M[4] * v[1] + M[7] * v[2];
    out[2] = M[2] * v[0] + M[5] * v[1] + M[8] * v[2];
}

// Cross product: out = a x b
static void cross(const double a[3], const double b[3], double out[3]) {
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

// ===== MotionEstimator implementation =====

// Butterworth2 LPF coefficient helper (same as ForcePipeline pattern)
static void calcLpfCoeffs(double fc, double fs,
    double& b0, double& b1, double& b2, double& a1, double& a2)
{
    double w0 = 2.0 * M_PI * fc / fs;
    double cos_w0 = cos(w0);
    double sin_w0 = sin(w0);
    double alpha = sin_w0 / sqrt(2.0);
    double a0 = 1.0 + alpha;
    b0 = ((1.0 - cos_w0) / 2.0) / a0;
    b1 = (1.0 - cos_w0) / a0;
    b2 = ((1.0 - cos_w0) / 2.0) / a0;
    a1 = (-2.0 * cos_w0) / a0;
    a2 = (1.0 - alpha) / a0;
}

MotionEstimator::MotionEstimator() : m_idx(0), m_count(0) {
    m_vel[0] = m_vel[1] = m_vel[2] = 0.0;
    m_accRaw[0] = m_accRaw[1] = m_accRaw[2] = 0.0;
    m_accFiltered[0] = m_accFiltered[1] = m_accFiltered[2] = 0.0;
    for (int i = 0; i < BUF_SIZE; i++) {
        m_posBuf[i][0] = m_posBuf[i][1] = m_posBuf[i][2] = 0.0;
    }
    for (int i = 0; i < 3; i++) {
        m_lpfX1[i] = m_lpfX2[i] = m_lpfY1[i] = m_lpfY2[i] = 0.0;
    }
    // 10Hz LPF at effective sample rate
    double fs = static_cast<double>(Config::FORCE_EFFECTIVE_SAMPLE_RATE);
    calcLpfCoeffs(Config::FORCE_ACC_FILTER_CUTOFF_HZ, fs,
        m_lpfB0, m_lpfB1, m_lpfB2, m_lpfA1, m_lpfA2);
}

void MotionEstimator::reset() {
    m_idx = 0; m_count = 0;
    m_vel[0] = m_vel[1] = m_vel[2] = 0.0;
    m_accRaw[0] = m_accRaw[1] = m_accRaw[2] = 0.0;
    m_accFiltered[0] = m_accFiltered[1] = m_accFiltered[2] = 0.0;
    for (int i = 0; i < BUF_SIZE; i++)
        m_posBuf[i][0] = m_posBuf[i][1] = m_posBuf[i][2] = 0.0;
    for (int i = 0; i < 3; i++)
        m_lpfX1[i] = m_lpfX2[i] = m_lpfY1[i] = m_lpfY2[i] = 0.0;
}

void MotionEstimator::update(double x, double y, double z, double dt) {
    // Store in ring buffer (unit: m)
    m_posBuf[m_idx][0] = x * 0.001;  // mm -> m
    m_posBuf[m_idx][1] = y * 0.001;
    m_posBuf[m_idx][2] = z * 0.001;
    m_idx = (m_idx + 1) % BUF_SIZE;
    if (m_count < BUF_SIZE) m_count++;

    if (m_count >= 3) {
        // Central difference velocity (using indices i and i-1)
        int i0 = (m_idx - 1 + BUF_SIZE) % BUF_SIZE;
        int i1 = (m_idx - 2 + BUF_SIZE) % BUF_SIZE;
        for (int k = 0; k < 3; k++) {
            m_vel[k] = (m_posBuf[i0][k] - m_posBuf[i1][k]) / dt;
        }
    }
    if (m_count >= 5) {
        // Central difference acceleration (3-point stencil)
        int i0 = (m_idx - 1 + BUF_SIZE) % BUF_SIZE;
        int i1 = (m_idx - 2 + BUF_SIZE) % BUF_SIZE;
        int i2 = (m_idx - 3 + BUF_SIZE) % BUF_SIZE;
        for (int k = 0; k < 3; k++) {
            m_accRaw[k] = (m_posBuf[i0][k] - 2.0 * m_posBuf[i1][k] + m_posBuf[i2][k]) / (dt * dt);
            // NaN guard
            if (std::isnan(m_accRaw[k]) || std::isinf(m_accRaw[k])) m_accRaw[k] = 0.0;
            // LPF: biquad step per channel
            double out = m_lpfB0 * m_accRaw[k] + m_lpfB1 * m_lpfX1[k] + m_lpfB2 * m_lpfX2[k]
                       - m_lpfA1 * m_lpfY1[k] - m_lpfA2 * m_lpfY2[k];
            m_lpfX2[k] = m_lpfX1[k]; m_lpfX1[k] = m_accRaw[k];
            m_lpfY2[k] = m_lpfY1[k]; m_lpfY1[k] = out;
            m_accFiltered[k] = out;
        }
    }
}

void MotionEstimator::getState(double vel[3], double acc[3]) const {
    for (int k = 0; k < 3; k++) {
        vel[k] = m_vel[k];
        acc[k] = m_accFiltered[k];
    }
}

bool MotionEstimator::isStill() const {
    double vsq = m_vel[0]*m_vel[0] + m_vel[1]*m_vel[1] + m_vel[2]*m_vel[2];
    double asq = m_accFiltered[0]*m_accFiltered[0] + m_accFiltered[1]*m_accFiltered[1] + m_accFiltered[2]*m_accFiltered[2];
    return (sqrt(vsq) < Config::FORCE_MOTION_VEL_THRESH_MS &&
            sqrt(asq) < Config::FORCE_MOTION_ACC_THRESH_MSS);
}

// ===== 闸门的报告 =====

// 闸门状态迁移 + 响亮地报出【逐通道】的比较结果。
// 只在【状态变化】时立刻打印; 状态不变时按 FORCE_GUARD_REPORT_MS 复报一次 ——
// 闸门每帧都判 (30Hz), 每帧都印会把控制台冲掉, 而"看不过来"与"没报"在操作上是一回事。
// ⚠ 复报【只对"拒绝"那一侧】: 放行是常态, 每 5 s 印一行"放行"同样是噪音
//   (而且会把真正要紧的那段挤出可视区)。放行只在它【刚刚恢复】时印一次。
static void setGuardState(ForceCompensation::GuardState st) {
    const DWORD now = GetTickCount();
    const bool changed = (st != g_guardState);
    g_guardState = st;

    if (st == ForceCompensation::GuardState::OK) {
        if (changed) {
            fprintf(stderr, "[Force] 一致性闸门: 放行 (本地全量模型与 @576 逐通道一致)\n");
            fflush(stderr);
        }
        g_guardReportMs = now;
        return;
    }
    const bool uncal = (st == ForceCompensation::GuardState::UNCALIBRATED);
    // 【UNCALIBRATED 不在复报之列】
    // 它是【配置态】, 不是【数据态】: "没有模型"这件事不会自己好, 也不随机械臂的动作变,
    // 所以复报出来的那 14 行与上一次【逐字相同】—— 唯一的效果是把别的输出挤出可视区,
    // 而这一屏本来是要在现场读的。实测 2026-09-20: 一次约 95 s 的会话里它出现过 19 次。
    // 启动那一路已经报过 ("无可用 force_calib.json — 按 'z' 调零"), 状态跃迁时这里再报
    // 一次, 就够了。
    // ⚠ INCONSISTENT 【仍然】复报: 它下面那张逐通道表的数据【会变】, 复报带的是新信息 ——
    //   那正是"复报"这个机制原本要服务的情形。
    if (!changed) {
        if (uncal) return;
        if ((now - g_guardReportMs) < static_cast<DWORD>(Config::FORCE_GUARD_REPORT_MS)) return;
    }
    g_guardReportMs = now;

    // 【复报只打一行】(2026-09-20)。全表只在【状态跃迁】时打。
    // 为什么: 拒绝是常态, 每 5 s 一次那 9 行解释 + 6 行表 + 姿态行会把控制台全冲掉 ——
    //   而现场要读的恰恰是【别的】输出: 's' 的那一屏、'p' 的确认提示、'y' 的逐条回执。
    //   实测代价 (2026-09-20 现场): 因为这条复报, 操作员【看不到 'p' 打了什么】, 于是
    //   无法判定"发送被拒"与"按键根本没收到" —— 一套诊断被彻底淹没。
    //   这与刚被取消的 UNCALIBRATED 复报是【同一个病】: 复报的内容与上次逐字相同。
    // 复报仍然出声 (拒绝没变这件事还得让人看见), 只是不再重抄整块; 哪几个通道超限直接
    //   写在那一行里 —— 那正是复报该带的唯一增量。
    // (走到这里且 !changed 只可能是 INCONSISTENT: !changed && uncal 在上面已经 return 了。)
    static const char* NM[6] = { "Fx(N)", "Fy(N)", "Fz(N)", "Mx(Nm)", "My(Nm)", "Mz(Nm)" };
    if (!changed) {
        // 【复报把六个通道【两个对照量】都报出来】(2026-09-20 修订)。
        // 原来只报超限的那几个通道的 @576 值 —— 但真正要看的是【@576 与 @720 哪个更接近 0】,
        // 而那只在【全表】里有, 全表又只在状态跃迁时打 ⇒ 操作员每次重启才看得到一次。
        // 现在这行就是一份【随时可读】的紧凑读数: 每个通道 "@576 / @720" 并排。
        // 一行约 130 字符、每 5 s 一次 —— 比原来那 14 行的块省得多, 而信息更全。
        char line[384];
        int off = snprintf(line, sizeof(line),
                           "[Force] !! (复报) 仍在拒绝  [@576 / @720]:");
        if (off < 0) off = 0;   // snprintf 可返回负值; 不管的话下面 (size_t)off 会回绕
        for (int i = 0; i < 6; i++) {
            if ((size_t)off + 40 >= sizeof(line)) break;   // 余量不足就停, 不越界
            const int w = snprintf(line + off, sizeof(line) - (size_t)off, " %s%+.3f/%+.4f",
                                   NM[i], g_guardEma[i], g_guardEma720[i]);
            if (w > 0) off += w;
        }
        if ((size_t)off < sizeof(line))
            snprintf(line + off, sizeof(line) - (size_t)off, "\n");
        fprintf(stderr, "%s", line);
        fflush(stderr);
        return;
    }

    // 输出一律走 stderr —— 与 ForceCalibration 的"响亮地说出来"同一条路; stdout 有缓冲,
    // 混着打会让这段在最需要它的时候缺半截。
    fprintf(stderr,
            "[Force] !! ============ 一致性闸门: 拒绝传递数据 ============\n"
            "[Force] !! 原因: %s\n"
            "[Force] !!   未标定 -> 去标定 ('m' 采多姿态 + 's' 解 A, 再 'z' 调零);\n"
            "[Force] !!   标定了但对不上 -> 去查负载参数有没有真的发进机械臂 (Task 8)。\n"
            "[Force] !!   【两者的处置一样 (都拒绝), 但要做的事不同, 所以原因必须分开报】。\n"
            "[Force] !! compensated[] 已【全 6 个分量置零】 —— 下游 ForcePipeline 由它推\n"
            "[Force] !!   filtered / hapticOut / F| 帧, 所以【传感器力那一条路】断了。\n"
            "[Force] !!   (虚拟约束力【不受影响】: 它在 HapticCallback.cpp:168 由位置现算,\n"
            "[Force] !!    与 compensated 无关 —— 安全边界的推手还在, 只是不再有传感器力。)\n",
            uncal ? "【没有可用模型】本地补偿未启用 —— 不是\"标定与机械臂不符\""
                  : "【有模型, 但与机械臂对不上】两边估计的不是同一个外力");
    // 【把本帧姿态一起打出来】(2026-09-20)。理由见 g_lastPose 的说明: 下面那六个数只能
    //   【连着姿态】才有意义 —— 现场抄数必须一起抄, 否则事后分不开"随姿态变"与"固定偏置",
    //   也拟合不了 M = 残差/g 的各向同性 (判 H1/H2)。出处 run-004 §4.3 的判别判据。
    // ⚠ 只在【拒绝】这一支打 (本函数走到这里的都非 OK): 放行是常态, 每次都给行姿态同样是噪音。
    if (g_lastPoseValid) {
        fprintf(stderr,
                "[Force] !! 本帧姿态: X=%+.3f Y=%+.3f Z=%+.3f  Rx=%+.3f Ry=%+.3f Rz=%+.3f"
                "  (mm / deg) —— 读下面那六个数要连着它一起抄\n",
                g_lastPose[0], g_lastPose[1], g_lastPose[2],
                g_lastPose[3], g_lastPose[4], g_lastPose[5]);
    } else {
        // 【不许打一行 0】: 没有"本帧"时打 0 会被读成"机械臂在原点姿态" —— 凭空造一个数。
        fprintf(stderr, "[Force] !! 本帧姿态: 【没有】—— 这一段不是由某一帧触发的"
                        " (例如装载/拒收路径), 所以没有姿态可报。\n");
    }
    if (uncal) {
        // 没有模型时【不打逐通道表】: 那时 compensated 恒为 0, 印出来会是"六个通道全在限内",
        // 而"在限内"在这里没有意义 —— 那是把"没比过"说成"比过了且没问题"。
        fprintf(stderr, "[Force] !! (没有模型可比较: 逐通道对比表不适用。上面那一段才是原因。)\n");
        fprintf(stderr, "[Force] !! 处理: 先按 'm' 采多姿态 -> 's' 解出 A, 再按 'z' 调零存盘。\n");
        fflush(stderr);
        return;
    }
    fprintf(stderr, "[Force] !! 逐通道结果 (EMA 差 = compensated − @576; 单位见各行标签):\n");
    // 【为什么每行末尾多一个 @720】(2026-09-20): 厂商文档 —— @576 = "TCP传感器力值",
    // @720 = "TCP力值 (通过关节电流计算)" ⇒ 后者才反映控制器用的负载参数, 而闸门比的是前者。
    // 两组并排, 一眼就能看出参考量该是谁。⚠ 它【只报不判】, 不参与任何容差比较。
    fprintf(stderr, "[Force] !!   行末的「与 @720」= compensated − @720 —— 只报不判,"
                    " 用来判闸门的参考量该是谁\n");
    for (int i = 0; i < 6; i++) {
        const bool ex = (g_guardTol[i] > 0.0) && (fabs(g_guardEma[i]) > g_guardTol[i]);
        if (!g_guardVote[i]) {
            fprintf(stderr, "[Force] !!   %-6s %+10.4f  容差 %.4f  【不投票】"
                            "@576 的 z 响应秩 2 (奇异值 0.212/0.201/0.008), 它动不了就证不了什么\n",
                    NM[i], g_guardEma[i], g_guardTol[i]);
        } else {
            fprintf(stderr, "[Force] !!   %-6s %+10.4f  容差 %.4f  %-16s └ 与 @720: %+9.4f"
                            " (只报不判)\n",
                    NM[i], g_guardEma[i], g_guardTol[i], ex ? "超限  <== 触发" : "在限内",
                    g_guardEma720[i]);
        }
    }
    // (走到这里只可能是 INCONSISTENT —— UNCALIBRATED 上面已经 return 了)
    fprintf(stderr, "[Force] !! 处理: 查负载参数有没有真的发进机械臂 (Task 8), 或重跑离线一致性检查。\n"
                    "[Force] !!   【不要】靠改容差把它压过去 —— 容差是由实测导出的。\n");
    fflush(stderr);
}

// 把 EMA 复位 —— 换模型/重启之后必须重新采证据, 不能拿旧模型的 EMA 去判新模型。
static void resetGuard() {
    for (int i = 0; i < 6; i++) g_guardEma[i] = 0.0;
    g_guardSeeded = false;
    for (int i = 0; i < 6; i++) g_guardEma720[i] = 0.0;
    g_guardSeeded720 = false;
    g_guardFrames = 0;
    g_guardState  = ForceCompensation::GuardState::UNCALIBRATED;
    g_guardReportMs = 0;
}

// ===== ForceCompensation namespace =====

namespace ForceCompensation {

void init() {
    if (!g_mutexInit) {
        InitializeCriticalSection(&g_calibMutex);
        g_mutexInit = true;
    }
    g_isCalibrated = false;
    for (int i = 0; i < 9; i++) g_A[i] = 0.0;
    for (int i = 0; i < 3; i++) {
        g_comSensor[i] = 0.0;
        g_biasForce[i] = 0.0;
        g_biasTorque[i] = 0.0;
    }
    g_motion.reset();

    // 闸门: 逐通道容差与状态。容差的【出处】写在 Config.h 的注释里 (实测导出, 不是猜)。
    for (int i = 0; i < 3; i++) {
        g_guardTol[i]     = Config::FORCE_GUARD_TOL_FORCE_N;
        g_guardTol[3 + i] = Config::FORCE_GUARD_TOL_MOMENT_NM;
    }
    resetGuard();
}

void setCalibration(const double A[9], const double biasForce[3],
                    const double biasTorque[3], const double comSensor[3])
{
    // ★ 用户指令 3「全零 A 拒绝传递数据并报错」的落点。
    //   在这里拒掉, 而不是等 step() 每帧去判 —— 那样"已标定"这个状态本身就带着一个
    //   没有重力项的模型, 而补偿后的读数依旧是 N, 不会有任何异常 (本项目最怕的那种
    //   "安静地错")。拒掉之后 g_isCalibrated 保持 false, step() 于是走"没有可用模型"
    //   那条路: 输出置零 + 报错 (ERR_FORCE_UNCALIBRATED)。
    //   ⚠ 现场确实会走到这里: 从未解过 A 时按 'z' 调零, ForceCalibration::update 会拿
    //     currentModel() 的空 A 回灌进来 (那时它自己也已经在报 WARNING)。
    // ⚠ 与装载路径 (ForceCalibration::loadFromFile) 【校验同一组东西】: 那边四个数组都判
    //   有限性, 这里从前只判 A —— 于是"从内存直接装一份带 inf 的零偏"这条不经过文件的路
    //   会静默收下一个 inf, 而 inf 会让 compensated 变 inf 再把闸门的 EMA 污染成 NaN。
    //   (NaN 在闸门里算不一致 -> 拒绝, 但那已经是"用一个坏模型报警", 不如根本不许装进来。)
    for (int i = 0; i < 3; i++) {
        if (!std::isfinite(biasForce[i]) || !std::isfinite(biasTorque[i]) ||
            !std::isfinite(comSensor[i])) {
            fprintf(stderr,
                    "[Force] !! setCalibration 【拒绝安装】: 第 %d 个分量不是有限数 "
                    "(b_F %.6g/%.6g/%.6g, b_M %.6g/%.6g/%.6g, c_s %.6g/%.6g/%.6g)。\n"
                    "[Force] !!   本地补偿保持【未启用】—— 输出置零并报 ERR_FORCE_UNCALIBRATED。\n"
                    "[Force] !!   判据与装载路径 (ForceCalibration::loadFromFile) 完全一致:\n"
                    "[Force] !!   同一个模型不该因为【来自文件】还是【来自内存】而一个收一个不收。\n",
                    i,
                    biasForce[0], biasForce[1], biasForce[2],
                    biasTorque[0], biasTorque[1], biasTorque[2],
                    comSensor[0], comSensor[1], comSensor[2]);
            fflush(stderr);
            EnterCriticalSection(&g_calibMutex);
            g_isCalibrated = false;      // 连旧的也一并作废: 拒绝安装的语义是"现在没有可用模型"
            LeaveCriticalSection(&g_calibMutex);
            resetGuard();
            setGuardState(GuardState::UNCALIBRATED);
            return;
        }
    }

    char why[192];
    if (!modelUsable(A, why, sizeof(why))) {
        fprintf(stderr,
                "[Force] !! setCalibration 【拒绝安装】: %s\n"
                "[Force] !!   本地补偿保持【未启用】—— 输出置零并报 ERR_FORCE_UNCALIBRATED。\n"
                "[Force] !!   一份没有重力项 (或秩亏) 的模型不会报错, 只会安静地算错, 所以不收。\n"
                "[Force] !!   处理: 按 'm' 采多姿态 -> 's' 解出 A (至少 4 个朝向不同的姿态), 再 'z' 调零。\n",
                why);
        fflush(stderr);
        EnterCriticalSection(&g_calibMutex);
        g_isCalibrated = false;      // 连旧的也一并作废: 拒绝安装的语义是"现在没有可用模型"
        LeaveCriticalSection(&g_calibMutex);
        resetGuard();
        setGuardState(GuardState::UNCALIBRATED);
        return;
    }

    EnterCriticalSection(&g_calibMutex);
    for (int i = 0; i < 9; i++) g_A[i] = A[i];
    for (int i = 0; i < 3; i++) {
        g_comSensor[i] = comSensor[i];
        g_biasForce[i] = biasForce[i];
        g_biasTorque[i] = biasTorque[i];
    }
    g_isCalibrated = true;
    LeaveCriticalSection(&g_calibMutex);

    // 换了模型就重新采证据: 拿旧模型的 EMA 去判新模型是错的。
    resetGuard();
}

void currentModel(double A[9], double comSensor[3]) {
    EnterCriticalSection(&g_calibMutex);
    for (int i = 0; i < 9; i++) A[i] = g_A[i];
    for (int i = 0; i < 3; i++) comSensor[i] = g_comSensor[i];
    LeaveCriticalSection(&g_calibMutex);
}

void currentBias(double biasForce[3], double biasTorque[3]) {
    EnterCriticalSection(&g_calibMutex);
    for (int i = 0; i < 3; i++) {
        biasForce[i]  = g_biasForce[i];
        biasTorque[i] = g_biasTorque[i];
    }
    LeaveCriticalSection(&g_calibMutex);
}

// 诊断用: 见头文件里为什么需要它。
// 有意【不】取 g_calibMutex: MotionEstimator 的状态不由它保护, step() 里读它
// (下面的惯性与 EMA 分支) 同样是无锁的, 这里跟着一致即可。
bool motionState(double vel[3], double acc[3]) {
    g_motion.getState(vel, acc);
    return g_motion.isStill();
}

bool isCalibrated() {
    return g_isCalibrated;
}

// A 能不能当【重力模型】用。三个毛病各报各的 —— 它们要做的事不一样:
//   · 非有限  -> 文件/内存被改坏了
//   · 全零    -> 从没解过 A (或调零时 A 是空的): 这是用户指令 3 点名的那一种
//   · 数值退化 -> "解出来了但秩亏": 姿态铺得太窄, 补偿里有两个方向根本没有模型
bool modelUsable(const double A[9], char* why, int whyLen) {
    if (why && whyLen > 0) why[0] = '\0';
    for (int i = 0; i < 9; i++) {
        if (!std::isfinite(A[i])) {
            if (why) snprintf(why, whyLen, "A 的第 %d 个元素不是有限数 (NaN/Inf)", i);
            return false;
        }
    }
    bool allZero = true;
    for (int i = 0; i < 9; i++) if (A[i] != 0.0) allZero = false;
    if (allZero) {
        if (why) snprintf(why, whyLen, "A 全为 0 (没有重力模型)");
        return false;
    }
    const double det = A[0] * (A[4] * A[8] - A[5] * A[7])
                     - A[1] * (A[3] * A[8] - A[5] * A[6])
                     + A[2] * (A[3] * A[7] - A[4] * A[6]);
    double fro2 = 0.0;
    for (int i = 0; i < 9; i++) fro2 += A[i] * A[i];
    const double fro = sqrt(fro2);
    if (!(fro > 0.0)) {
        if (why) snprintf(why, whyLen, "A 的 Frobenius 范数为 0");
        return false;
    }
    const double ratio = fabs(det) / (fro * fro * fro);
    if (!(ratio > GUARD_MIN_DET_RATIO)) {
        // ⚠ 措辞对得上判据 (2026-09-19 复审): 这条拦的是"接近完全秩亏", 不是"任何一个
        //   方向病态" —— 1e-3 换算成条件数约 354 (推导见 GUARD_MIN_DET_RATIO 处的注释),
        //   所以别再说"至少一个力方向没有模型"。
        if (why) snprintf(why, whyLen,
                          "A 数值退化: |det A| / ||A||^3 = %.3g <= %.3g "
                          "(接近完全秩亏: 该比值对'标量质量 x 正交'恒为 0.19245, "
                          "本门限约等于条件数 354)", ratio, GUARD_MIN_DET_RATIO);
        return false;
    }
    return true;
}

GuardState guardState() { return g_guardState; }

void guardReport(GuardReport& out) {
    out.state = g_guardState;
    out.frames = g_guardFrames;
    for (int i = 0; i < 6; i++) {
        out.ema[i]       = g_guardEma[i];
        out.tol[i]       = g_guardTol[i];
        out.voted[i]     = g_guardVote[i];
        out.exceeded[i]  = g_guardVote[i] && (g_guardTol[i] > 0.0)
                        && (fabs(g_guardEma[i]) > g_guardTol[i]);
    }
}

const char* guardStateName(GuardState s) {
    switch (s) {
        case GuardState::OK:            return "OK";
        case GuardState::UNCALIBRATED:  return "UNCALIBRATED";
        case GuardState::INCONSISTENT:  return "INCONSISTENT";
    }
    return "UNKNOWN";
}

// 闸门状态 -> 错误码。
// RelayCore 从前自己拿 static_cast<int>(guardState()) 去比字面量 1 和 2 —— 那是把
// "哪个状态配哪个码"存在【两个地方的巧合】里: 改一次枚举的数值, "去标定"与"去查负载
// 参数"这两条完全不同的处置指引就被对调, 而且没有任何测试会发现。现在这里是唯一的实现。
//
// 【"穷举"到什么程度, 说实话】: 这个 switch 没有 default, 但加了新的 GuardState 而忘了配
// 错误码时【编译器不会拦你】: 末尾那句 return 让缺失返回路径不存在 (没有 C4715), 而
// C4062 (unhandled enumerator) 默认关闭 —— 2026-09-19 用探针实测: /W1 /W3 /W4 都不报,
// 只有 /Wall 报, 本项目按 /W1 编译。所以真正把这张表钉住的是 test_force_compensation 的
// guard_error_code_mapping (三条映射逐条断言 + 与 errorCodeName 对上), 不是编译器。
// 末尾那句是"宁可返回 OK 也不掉出函数尾"的兜底。
RobotErrorCode guardErrorCode(GuardState s) {
    switch (s) {
        case GuardState::OK:            return RobotErrorCode::OK;
        case GuardState::UNCALIBRATED:  return RobotErrorCode::ERR_FORCE_UNCALIBRATED;
        case GuardState::INCONSISTENT:  return RobotErrorCode::ERR_FORCE_INCONSISTENT;
    }
    return RobotErrorCode::OK;
}

double currentMassKg() {
    EnterCriticalSection(&g_calibMutex);
    double A[9];
    for (int i = 0; i < 9; i++) A[i] = g_A[i];
    LeaveCriticalSection(&g_calibMutex);
    return massScaleOf(A);   // A 全 0 -> det 0 -> 报 0
}

void step(AppState::ForceData& fd, const double poseRxyz[6]) {
    // poseRxyz = {X_mm, Y_mm, Z_mm, Rx_deg, Ry_deg, Rz_deg}

    // 【记下本帧姿态】—— 闸门拒绝时那段打印要用它 (见 g_lastPose 的说明)。与算那六个数用的是
    // 同一次 30004 帧, 所以数与姿态天然对齐。
    for (int i = 0; i < 6; i++) g_lastPose[i] = poseRxyz[i];
    g_lastPoseValid = true;

    // 1. Update motion estimator
    double dt = 1.0 / static_cast<double>(Config::FORCE_EFFECTIVE_SAMPLE_RATE);
    g_motion.update(poseRxyz[0], poseRxyz[1], poseRxyz[2], dt);

    // 2. 默认输出 = 零。【fail closed 的落点】: 任何没走到"闸门放行"的路径都在这里留下 0。
    //    从前这里是"把 @1304 原样抄进 compensated" (透传) —— 那就是评审判定的 Critical:
    //    未标定时 @1304 的 x 通道在零外力下也报 −19.0 ~ −22.7 N
    //    (四份夹具 tests/fixtures/calib_poses_2026-09-19*.txt 的 F1304x 列, 36 个姿态实测),
    //    经下游 ForcePipeline 的映射 (Config::FORCE_MAX_TOUCH_N / FORCE_MAX_SENSOR_N = 3.3/200)
    //    与反射增益 5 之后以 21.9 N 计: 手上得到 ~1.8 N 的恒定推力 (21.9 × 3.3/200 × 5 = 1.81)。
    //    【数据侧一律置零与严重度无关】—— REJECT 只是"拒绝这一帧的运动", 而"不许传递
    //    数据"这件事由这一行无条件保证。
    for (int i = 0; i < 6; i++) fd.compensated[i] = 0.0;
    fd.isCalibrated = false;
    fd.calibMassKg = 0.0;
    for (int i = 0; i < 3; i++) {
        fd.calibComSensor[i] = 0.0;
        fd.calibBiasForce[i] = 0.0;
        fd.calibBiasTorque[i] = 0.0;
    }

    if (!g_isCalibrated) {
        // 指令 1/3: 没有可用模型 ⇒ 直接拒绝 + 报错, 不是透传。
        // (A 全零/退化 的情形在 setCalibration 就被挡下了, 所以这里报的是同一类原因。)
        setGuardState(GuardState::UNCALIBRATED);
        return;
    }

    // 3. Snapshot calibration globals under mutex
    EnterCriticalSection(&g_calibMutex);
    double A[9];
    for (int i = 0; i < 9; i++) A[i] = g_A[i];
    double com[3] = {g_comSensor[0], g_comSensor[1], g_comSensor[2]};
    double bF[3] = {g_biasForce[0], g_biasForce[1], g_biasForce[2]};
    double bM[3] = {g_biasTorque[0], g_biasTorque[1], g_biasTorque[2]};
    bool calib = g_isCalibrated;
    LeaveCriticalSection(&g_calibMutex);

    if (!calib) {   // setCalibration cleared calibration mid-flight
        setGuardState(GuardState::UNCALIBRATED);
        return;
    }

    // 4-5. Gravity in the SENSOR frame, then through the fitted response A.
    //    g = gravitySensorFrameAtYaw(pose, 0.0, g) —— 【psi 传 0】: 全量模型的 A 是自由 3×3,
    //    安装旋转/反射/非正交全被它吸收, 所以补偿式子里【没有 ψ】, 模块态不再参与。
    //    (残余模型走的是 gravitySensorFrame: 读模块态里的 ψ, 再乘一个【标量】质量。)
    //    ⚠ 走共享实现, 别在这里自己展开 Rᵀ(0,0,9.81): 模块自己实现过两次重力约定, 一份写
    //      成 R、一份写成 Rᵀ, 求解器因此安静地解错 —— 约定只能有一份实现。
    double gTool[3];
    TcpCalibration::gravitySensorFrameAtYaw(poseRxyz, 0.0, gTool);

    // Gravity force through the fitted response: Fg = A · g
    // (残余模型是标量质量乘 g —— A 的每一行都是一个"分量怎么随重力方向变"的响应。)
    double Fg[3];
    for (int a = 0; a < 3; a++) {
        Fg[a] = A[3 * a] * gTool[0] + A[3 * a + 1] * gTool[1] + A[3 * a + 2] * gTool[2];
    }

    // Gravity torque: c_s × (A·g) —— 与 Fg 同一个 w = A·g (叉乘结构, 不是独立的 3×3)。
    double Mg[3];
    cross(com, Fg, Mg);

    // 6. Inertia force (only if moving) —— 语义、时机、系数一字未改 (仍是 mass·a);
    //    mass 现在取自全量模型的质量尺度 |det A|^(1/3) (见 currentMassKg)。
    const double mass = massScaleOf(A);
    double Fi[3] = {0, 0, 0};
    if (!g_motion.isStill()) {
        double vel[3], acc[3];
        g_motion.getState(vel, acc);
        Fi[0] = mass * acc[0];
        Fi[1] = mass * acc[1];
        Fi[2] = mass * acc[2];
    }

    // 7. Compensate: compensated = sixForceRaw − bias − gravity − inertia
    //    ⚠ 先算进【局部变量】, 不直接写 fd —— 闸门要在数据出门之前判。
    double comp[6];
    comp[0] = fd.sixForceRaw[0] - bF[0] - Fg[0] - Fi[0];
    comp[1] = fd.sixForceRaw[1] - bF[1] - Fg[1] - Fi[1];
    comp[2] = fd.sixForceRaw[2] - bF[2] - Fg[2] - Fi[2];
    comp[3] = fd.sixForceRaw[3] - bM[0] - Mg[0];
    comp[4] = fd.sixForceRaw[4] - bM[1] - Mg[1];
    comp[5] = fd.sixForceRaw[5] - bM[2] - Mg[2];

    // ===== 7b. 运行时一致性闸门 (用户指令 1/2) =====
    // 判据与两个原因的分辨写在 .h 里; 这里只做: 更新逐通道 EMA -> 投票 -> 放行或拒绝。
    // 【EMA 无条件更新】(包括正在拒绝的时候): 否则闸门一旦拒绝就再也回不来, 而 Task 8
    //  "把负载发进去 -> 看它放行" 正是靠它回来的。
    for (int i = 0; i < 6; i++) {
        const double d = comp[i] - fd.raw[i];
        if (!g_guardSeeded) g_guardEma[i] = d;
        else g_guardEma[i] += Config::FORCE_GUARD_EMA_ALPHA * (d - g_guardEma[i]);
        // 第二组: 与 @720 的差。同一个 α、同一帧、同一次 comp —— 只换对照量。
        // 【只报不判】, 见 g_guardEma720 的说明: 它不参与任何容差比较。
        const double d720 = comp[i] - fd.tcpForce[i];
        if (!g_guardSeeded720) g_guardEma720[i] = d720;
        else g_guardEma720[i] += Config::FORCE_GUARD_EMA_ALPHA * (d720 - g_guardEma720[i]);
    }
    g_guardSeeded = true;
    g_guardSeeded720 = true;
    g_guardFrames++;

    bool inconsistent = false;
    for (int i = 0; i < 6; i++) {
        if (!g_guardVote[i]) continue;               // Fz 不投票, 但照报 (见上面的说明)
        if (!std::isfinite(g_guardEma[i])) { inconsistent = true; break; }  // NaN 也算不一致
        if (fabs(g_guardEma[i]) > g_guardTol[i]) { inconsistent = true; break; }
    }
    if (inconsistent) {
        // 拒绝: fd.compensated 保持第 2 步写下的全零 -> 下游由它推的 filtered / hapticOut /
        // F| 帧断开 (即【传感器力那一条路】)。⚠ 虚拟约束力【不断】—— 它在 HapticCallback.cpp:168
        // 由位置现算, 与 compensated 无关 (同 ForceCompensation.h 顶部与 RelayCore.cpp 那段)。
        setGuardState(GuardState::INCONSISTENT);
        return;
    }
    setGuardState(GuardState::OK);

    for (int i = 0; i < 6; i++) fd.compensated[i] = comp[i];

    // 8. Online EMA bias update (only when still) — 作用/时机/系数一字未改, 只有输入量
    //    跟着换成了 @1304。零偏是【这个通道】的零偏: 拿 @576 去更新它, 就是给另一路量的
    //    零偏做 EMA —— 两路的零偏不是一回事。实测出处: tests/fixtures/calib_poses_2026-09-19.txt
    //    7 个姿态的逐通道均值差 (@1304 − @576) = 19.8 / 1.6 / 1.7 N (x/y/z)。
    //    ⚠ 【新增】只在与 @576 一致时才更新。不一致时继续在线学零偏, 等于闸门一边拒它、
    //      一边把同样的数据学进 b_F (而 b_F 的 EMA 目标正是把 compensated 拉向 0) ——
    //      那会让不一致自我掩盖, 而"安静地学错"正是这条闸门要防的东西。
    //      代价: 拒绝期间零偏不再自跟踪; 闸门放行后自动恢复。
    if (g_motion.isStill()) {
        double alpha = Config::FORCE_BIAS_EMA_ALPHA;
        // Update local copy, then write back under mutex
        bF[0] += alpha * (fd.sixForceRaw[0] - Fg[0] - bF[0]);
        bF[1] += alpha * (fd.sixForceRaw[1] - Fg[1] - bF[1]);
        bF[2] += alpha * (fd.sixForceRaw[2] - Fg[2] - bF[2]);
        bM[0] += alpha * (fd.sixForceRaw[3] - Mg[0] - bM[0]);
        bM[1] += alpha * (fd.sixForceRaw[4] - Mg[1] - bM[1]);
        bM[2] += alpha * (fd.sixForceRaw[5] - Mg[2] - bM[2]);

        EnterCriticalSection(&g_calibMutex);
        g_biasForce[0] = bF[0];
        g_biasForce[1] = bF[1];
        g_biasForce[2] = bF[2];
        g_biasTorque[0] = bM[0];
        g_biasTorque[1] = bM[1];
        g_biasTorque[2] = bM[2];
        LeaveCriticalSection(&g_calibMutex);
    }

    // 9. Update calib params in ForceData for HUD display / MATLAB relay
    fd.isCalibrated = true;
    fd.calibMassKg = mass;              // 质量尺度 |det A|^(1/3), 不是调用方传进来的
    for (int i = 0; i < 3; i++) {
        fd.calibComSensor[i] = com[i];  // c_s (m, 传感器测量系)
        fd.calibBiasForce[i] = bF[i];
        fd.calibBiasTorque[i] = bM[i];
    }
}

void shutdown() {
    g_isCalibrated = false;
    g_motion.reset();
    resetGuard();
    if (g_mutexInit) {
        DeleteCriticalSection(&g_calibMutex);
        g_mutexInit = false;
    }
}

} // namespace ForceCompensation
