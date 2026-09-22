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
}

void shutdown() {
    for (int i = 0; i < 6; i++) {
        g_filters[i].reset();
    }
    g_gainRamp = 0.0;
}

} // namespace ForcePipeline
