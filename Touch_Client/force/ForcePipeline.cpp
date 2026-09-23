#define _USE_MATH_DEFINES
#include "ForcePipeline.h"
#include "../config/Config.h"
#include "ForceTuning.h"
#include <cmath>
#include <algorithm>

// ===== Butterworth2 implementation =====

Butterworth2::Butterworth2() { reset(); }

void Butterworth2::reset() {
    x1 = x2 = y1 = y2 = 0.0;
}

// Calculate 2nd-order Butterworth lowpass coefficients at init time
// fc = cutoff frequency (Hz), fs = sample rate (Hz)
static void calcButterworthCoeffs(double fc, double fs,
    double& b0, double& b1, double& b2, double& a1, double& a2)
{
    double w0 = 2.0 * M_PI * fc / fs;
    double cos_w0 = cos(w0);
    double sin_w0 = sin(w0);
    double alpha = sin_w0 / sqrt(2.0);  // Q = 1/sqrt(2) for Butterworth

    double a0 = 1.0 + alpha;
    b0 = ((1.0 - cos_w0) / 2.0) / a0;
    b1 = (1.0 - cos_w0) / a0;
    b2 = ((1.0 - cos_w0) / 2.0) / a0;
    a1 = (-2.0 * cos_w0) / a0;
    a2 = (1.0 - alpha) / a0;
}

double Butterworth2::step(double input) {
    // NaN guard: reset state if input is invalid
    if (std::isnan(input) || std::isinf(input)) {
        reset();
        return 0.0;
    }
    double output = b0 * input + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
    x2 = x1;
    x1 = input;
    y2 = y1;
    y1 = output;
    return output;
}

// ===== ForcePipeline =====

static Butterworth2 g_filters[6];  // one per channel (Fx,Fy,Fz,Mx,My,Mz)
static double g_prevFiltered[6] = {0};  // for gradient limiting

// 增益斜坡的当前值 (向 ForceTuning::gain() 逼近)。
// 【为什么状态在这里而不在 ForceTuning】斜坡是【信号处理】(与 FORCE_GRADIENT_LIMIT 同类),
//   不是"参数" —— ForceTuning 只持有目标值。分开的直接好处: MATLAB 回读报的是目标值,
//   界面上的数字不会自己动。
static double g_gainRamp = 0.0;

// ===== 抖动诊断的两个累加器 (2026-09-23, 抖动计划 Task 2) —— 【只读、可删】=====
// 为什么是【两个】实例, 而不是一个:
//   判据 A 要的是 hapticOut (= 手感) 的逐轴 sd; 判据 B 要的是【残差越过死区】的帧占比 ——
//   这是【两个不同的量】, 中间隔着映射 (第 4 步那三行) 与增益 (第 5 步)。
//   若图省事用一个实例, 就只能把死区阈值用在 hapticOut 上 ⇒ 判据 B 量在了错的量上,
//   【而输出长得一模一样】—— 这正是本项目最怕的"安静地错"。
// 两个实例各自喂什么, 写在 step() 末尾那一段 (那里是唯一的喂入点)。
static JitterStats g_statsOut;        // ← hapticOut[0..2]   (判据 A: 输出/手感的 sd)
static JitterStats g_statsResidual;   // ← fd.filtered[0..2] (判据 B: 进死区的残差)

namespace ForcePipeline {

void init() {
    // ★ 2026-09-21 重定 —— 与 Config::FORCE_FILTER_FS_HZ / FORCE_FILTER_CUTOFF 那两段配套。
    //   从前这里写的是 `fs = FORCE_FILTER_CUTOFF * 4.0` —— 【两个量互相定义】, 于是它读起来
    //   像"设计成 fs/4 的 Butterworth", 而那个 fs 是猜的、从没实测过。
    //   实测之后真相是: 流水线当时跑在 ~11 Hz 上 ⇒ 同一个系数在那里意味着实际截止 ≈2.75 Hz,
    //   噪声只被削掉约 20% (原始 @1304 Fz sd 0.142 N ⇒ 日志里的 filtered Fz sd 0.115 N)。
    //   ⇒ 现在 fs 直接取【流水线的真实工作速率】这个常数 (RelayCore::forceReaderThread 把它
    //     挪到了帧率上; 实测 122.9 Hz vs 文档 125 Hz)。归一化频率 fc/fs 从此有意义。
    //   ⚠ 【别再把它反推回截止频率】: `fs = fc × 4` 那种写法会让两个常数互相定义, 于是
    //     "改截止"会【悄悄改采样率】, 而采样率是硬件事实, 不是设计自由度。
    double fs = static_cast<double>(Config::FORCE_FILTER_FS_HZ);
    double b0, b1, b2, a1, a2;
    calcButterworthCoeffs(static_cast<double>(Config::FORCE_FILTER_CUTOFF), fs, b0, b1, b2, a1, a2);
    for (int i = 0; i < 6; i++) {
        g_filters[i].b0 = b0; g_filters[i].b1 = b1; g_filters[i].b2 = b2;
        g_filters[i].a1 = a1; g_filters[i].a2 = a2;
        g_filters[i].reset();
        g_prevFiltered[i] = 0.0;
    }

    // 让斜坡直接就位 —— 否则启动时增益会从 0 爬到目标值 (那 0.25 秒里手上力是错的)。
    g_gainRamp = ForceTuning::gain();

    // ===== 抖动诊断的累加器 (2026-09-23) =====
    // 在这里清一次 + 【武装残差的越阈计数】。init() 是"每帧之前"的最后一道保险:
    //   · 它在 RelayCore::initForceReader 里、创建 ForceReader 线程【之前】被调 (唯一调用点),
    //     所以武装时一帧都还没喂过 —— 这是 JitterStats 要求的顺序 (Welford 不留样本,
    //     武装晚了那批帧的越阈数就补不回来了, 见 JitterStats.h 顶上那段)。
    //   · 阈值取 Config::FORCE_RESIDUAL_DEADZONE_N —— 死区的【唯一一份定义】,
    //     与 mapForceToTouch / RelayCore 的 F| 用的是同一个数。这里【不许】再写一个字面量:
    //     一旦两处不一致, 判据 B 会量在一个"不是死区"的阈值上, 而输出看不出来。
    g_statsOut.reset();
    g_statsResidual.reset();
    g_statsResidual.setCountThreshold(Config::FORCE_RESIDUAL_DEADZONE_N);
}

// ★ 死区: 【唯一一份定义在 ForcePipeline.h】(softDeadzone) —— 别再在本文件里另写一份。
//   它从前是硬门, 而且【RelayCore 构造 F| 消息时又写了一份硬门】⇒ 我只修了本文件那一份,
//   于是 MATLAB 显示的那一路照旧阶跃。抽到头文件就是为了让两处【必须】用同一个实现。

static inline double mapForceToTouch(double sensorForce) {
    // 软门 (见 softDeadzone 的说明 —— 硬门会在阈值处跳变, 现场表现为"读数在 0 与 0.2 之间跳")
    double v = softDeadzone(sensorForce, Config::FORCE_RESIDUAL_DEADZONE_N);
    // Linear mapping: 200N sensor -> 3.3N Touch
    // ★ 2026-09-22: 改用 netRatioPerGainUnit() —— 3.3/200 这个比率【唯一一份定义】在
    //   ForcePipeline.h (MATLAB 的 RG| 回读也要报它; 在这里再写一遍就是两份实现)。
    const double ratio = netRatioPerGainUnit();
    double out = v * ratio;
    // Hard clamp
    if (out > Config::FORCE_MAX_TOUCH_N)  out = Config::FORCE_MAX_TOUCH_N;
    if (out < -Config::FORCE_MAX_TOUCH_N) out = -Config::FORCE_MAX_TOUCH_N;
    return out;
}

void step(AppState::ForceData& fd) {
    // 1. Butterworth filter
    for (int i = 0; i < 6; i++) {
        fd.filtered[i] = g_filters[i].step(fd.compensated[i]);
    }

    // 2. Gradient limit (protect against sensor spike)
    for (int i = 0; i < 6; i++) {
        double delta = fd.filtered[i] - g_prevFiltered[i];
        if (delta > Config::FORCE_GRADIENT_LIMIT)
            fd.filtered[i] = g_prevFiltered[i] + Config::FORCE_GRADIENT_LIMIT;
        else if (delta < -Config::FORCE_GRADIENT_LIMIT)
            fd.filtered[i] = g_prevFiltered[i] - Config::FORCE_GRADIENT_LIMIT;
        g_prevFiltered[i] = fd.filtered[i];
    }

    // 3. Force mapping: sensor N -> Touch N (forces only, 3 axes)
    double fx = mapForceToTouch(fd.filtered[0]);
    double fy = mapForceToTouch(fd.filtered[1]);
    double fz = mapForceToTouch(fd.filtered[2]);

    // 4. Coordinate transform: Robot tool frame -> Touch device frame
    //    ★★★ 2026-09-21 定案: 垂直项【整轴关掉】(Config::FORCE_FEEDBACK_Z_SIGN = 0.0)。
    //      理由不是符号选错, 而是这一轴【不该承担反馈】: Touch Y 正是操作员用来落笔、维持
    //      入纸深度的那一轴 —— 往下推 = 帮忙往纸里按, 往上推 = 把笔抬起来根本落不了笔。
    //      两个方向都不行 ⇒ 关掉。**"阻力"由横向两路给** (hapticOut[0] fx→X, hapticOut[2] fy→Z)。
    //      ⚠ 完整依据 (含实测的 comp 符号、"若将来重开必须满足的三件事") 在
    //        Config::FORCE_FEEDBACK_Z_SIGN 那一大段 —— **动它之前先读那一段。**
    //    ⚠ 另两项本来就原样映射 (fx→X, fy→Z), 未动。
    //    ⚠ 第 4 步的【轴对应】本身尚未被独立验证 —— 若实测是"力出现在错的轴上", 那是另一件事。
    //    ★★ 2026-09-21 (同日第二次定案): 横向两路也带上了整体符号 −1。
    //      力映射应当是【位置映射的逆】(L = Mᵀ, M = convertTouchToRobot), 而代码里写死的轴对应
    //      等于 Mᵀ 再整体取负 ⇒ L = −Mᵀ。那个整体符号 −1 由【唯一有实测锚点的那一行】(垂直)
    //      定死, 再对三行一起成立。完整推导与"它为什么只是推导+现场描述"见
    //      Config::FORCE_FEEDBACK_LATERAL_SIGN 那一大段 —— **动符号之前先读它。**
    fd.hapticOut[0] =  fx * Config::FORCE_FEEDBACK_LATERAL_SIGN;  // Robot Fx -> Touch X
    fd.hapticOut[1] =  fz * Config::FORCE_FEEDBACK_Z_SIGN;        // Robot Fz -> Touch Y (0 = 关)
    fd.hapticOut[2] =  fy * Config::FORCE_FEEDBACK_LATERAL_SIGN;  // Robot Fy -> Touch Z

    // 5. Apply reflection gain (amplify for human perception)
    //    Typical contact forces (5-30N) → clearly perceptible (0.4-2.5N at Touch)
    //    Safety clamp at FORCE_MAX_TOUCH_N still applies in hapticCallback
    //
    //    ★ 2026-09-22: 增益来源从 Config 的编译期常量改成 ForceTuning 的运行时值,
    //      并加一道斜坡 (见 Config::FORCE_GAIN_SLEW_PER_S)。
    //      每帧最多动 (FORCE_GAIN_SLEW_PER_S / FORCE_FILTER_FS_HZ) 个增益单位。
    {
        const double target = ForceTuning::gain();
        const double maxStep = Config::FORCE_GAIN_SLEW_PER_S / (double)Config::FORCE_FILTER_FS_HZ;
        const double d = target - g_gainRamp;
        if (d > maxStep)       g_gainRamp += maxStep;
        else if (d < -maxStep) g_gainRamp -= maxStep;
        else                   g_gainRamp = target;

        for (int i = 0; i < 3; i++) {
            fd.hapticOut[i] *= g_gainRamp;
        }
    }

    // ===== 6. 抖动诊断: 只把【这一帧已经算完的两个量】喂给两个累加器 (2026-09-23) =====
    //   · g_statsOut      ← fd.hapticOut[0..2]  = 手感本身 (映射 + 增益之后), 判据 A。
    //   · g_statsResidual ← fd.filtered[0..2]   = 【进死区的那个量】(补偿 + 滤波之后,
    //     传感器 N, 还没过死区), 判据 B 的 |残差| >= 0.20 N 占比量的就是它。
    //     ⚠ 为什么是 filtered 而不是 compensated: 0.20 N 死区的两个消费者
    //       (mapForceToTouch 与 RelayCore 构造 F| 时) 吃的都是 filtered[] ⇒ 只有它
    //       "越过死区"这句话才成立; 用未滤波的 compensated[] 会量出一个【不经过死区】的
    //       占比 (它必然更高), 而两条路的输出长得一样。本项目记下的同口径历史值是
    //       "filtered 的 |Fz|>0.2 N 占 6.9%" (见 Config.h 的 FORCE_RESIDUAL_DEADZONE_N
    //       与 FORCE_FILTER_CUTOFF 两段、main.cpp 的 'n' 探针说明) —— 本次是它的复测。
    //   · 两个量【不可互换】: 中间隔着第 4 步的轴映射与第 5 步的增益。
    //
    // ⚠ 「这一帧」的判据 = 【step() 每帧被调一次】, 不数回调次数。
    //   依据 (读代码核过, 不是推断): RelayCore::forceReaderThread 是一个"阻塞 recv 一帧 →
    //   开 forceDataMutex → 补偿 → 本函数 → 关锁"的循环, 所以【一次调用 ⇔ 一帧 30004】;
    //   AppState 的 isStale 只由超时置位, 帧到达时置 false, 在本函数里恒为"这一帧刚到"。
    //   ⇒ 直接按调用次数喂即可。⛔ 绝【不】按 HapticCallback 的 1 kHz 回调节拍喂:
    //   同一帧会被重复累加几十次 (采样保持) ⇒ sd 与越阈占比被系统性压低 ⇒
    //   "机制不成立"这个结论会凭空出现 (本项目在这上面栽过一次)。
    //
    // 代价: O(1) —— 加减乘除 + 三个 double 的比较, 无分配、无 push_back (换成 Welford 的
    //   理由就是这条: 这是控制路径)。回滚 = 删掉下面这三行 (两个累加器不参与任何输出)。
    const double residual[3] = { fd.filtered[0], fd.filtered[1], fd.filtered[2] };
    g_statsOut.addFrame(fd.hapticOut);
    g_statsResidual.addFrame(residual);
}

// ===== 抖动诊断: 给 'n' 探针取一份值快照 / 清窗口 (见 ForcePipeline.h 的线程契约) =====
void copyJitterSnapshot(JitterSnapshot& out) {
    out.nOut = g_statsOut.n();
    out.nResidual = g_statsResidual.n();
    for (int a = 0; a < 3; a++) {
        out.meanOut[a] = g_statsOut.mean(a);
        out.sdOut[a] = g_statsOut.sd(a);
        out.meanResidual[a] = g_statsResidual.mean(a);
        out.sdResidual[a] = g_statsResidual.sd(a);
        // 死区的唯一一份定义 —— 与 init() 里武装的那个数同源 (见那里的说明)。
        // ⚠ 若这个阈值与武装时用的不一致, fracAbove 会返回 −1 (算不出来) 而不是一个
        //   假的 0%: 打印侧对它单独处理 (见 main.cpp)。
        out.fracResidual[a] = g_statsResidual.fracAbove(a, Config::FORCE_RESIDUAL_DEADZONE_N);
    }
}

void resetJitterStats() {
    g_statsOut.reset();
    g_statsResidual.reset();
}

void shutdown() {
    for (int i = 0; i < 6; i++) {
        g_filters[i].reset();
    }
    g_gainRamp = 0.0;
}

} // namespace ForcePipeline
