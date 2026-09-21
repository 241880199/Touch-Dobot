#define _USE_MATH_DEFINES
#include "ForcePipeline.h"
#include "../config/Config.h"
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

namespace ForcePipeline {

void init() {
    // ⚠⚠ 【2026-09-21 记明, 未改系数】这个 fs 是【猜的】, 而且是从截止频率【反推】的
    //   (fs = 截止 × 4) —— 两个量互相定义, 所以它读起来像"设计成 fs/4 = 30Hz 的 Butterworth",
    //   而注释写的"effective sample rate ~120Hz"是我们【以为的】速率。
    //   真实速率: ForceCompensation::step 的节拍 = 1000/FORCE_POLL_INTERVAL_MS ≈ 【30Hz】
    //   (那一步的 dt 已按真实值改, 见 ForceCompensation.cpp; 本处系数【故意不动】)。
    //   滤波器算的是归一化频率 fc/fs, 所以同一系数在 30Hz 下运行, 实际截止 =
    //     30Hz × (30/120) ≈ 【7.5 Hz】 —— 比设计的钝 4 倍 (更平滑、也更滞后)。
    //   ⇒ 【为什么不改】: 同 MotionEstimator 那处 —— 更钝 = 噪声更低, 而噪声是死区 0.20 N
    //     的定标依据; "改对"会【增大】噪声。⇒ 那是设计取舍, 要跟着死区的重定一起做
    //     (run-005 §14 第 2 条)。此处只留真实数值, 免得下一个人按 30Hz 去推理。
    double fs = static_cast<double>(Config::FORCE_FILTER_CUTOFF) * 4.0; // 以为的 fs ~120Hz ⇒ 实际截止 ≈7.5Hz
    double b0, b1, b2, a1, a2;
    calcButterworthCoeffs(static_cast<double>(Config::FORCE_FILTER_CUTOFF), fs, b0, b1, b2, a1, a2);
    for (int i = 0; i < 6; i++) {
        g_filters[i].b0 = b0; g_filters[i].b1 = b1; g_filters[i].b2 = b2;
        g_filters[i].a1 = a1; g_filters[i].a2 = a2;
        g_filters[i].reset();
        g_prevFiltered[i] = 0.0;
    }
}

static inline double deadzone(double val, double threshold) {
    if (fabs(val) < threshold) return 0.0;
    return val;
}

static inline double mapForceToTouch(double sensorForce) {
    // Deadzone
    double v = deadzone(sensorForce, Config::FORCE_RESIDUAL_DEADZONE_N);
    // Linear mapping: 200N sensor -> 3.3N Touch
    double ratio = Config::FORCE_MAX_TOUCH_N / Config::FORCE_MAX_SENSOR_N;
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
    //    ★ 2026-09-21: 反馈应当是【阻力】—— 操作员压下去 ⇒ 工具受到向上的反作用 ⇒ 手上被往上推。
    //      ⇒ 垂直项改用 +fz (从前是 -fz, 那会【帮忙】往下推, 现场感受就是"斥力/被推开")。
    //      依据 (含"原来那行注释举的例子自相矛盾")见 Config::FORCE_FEEDBACK_Z_SIGN 那一大段。
    //    ⚠ 另两项本来就原样映射 (fx→X, fy→Z), 与同一个原则一致, 未动。
    //    ⚠ 第 4 步的【轴对应】本身尚未被独立验证 —— 若实测是"力出现在错的轴上", 那是另一件事。
    fd.hapticOut[0] =  fx;                                  // Robot Fx -> Touch X
    fd.hapticOut[1] =  fz * Config::FORCE_FEEDBACK_Z_SIGN;   // Robot Fz -> Touch Y (阻力, 见上)
    fd.hapticOut[2] =  fy;                                  // Robot Fy -> Touch Z

    // 5. Apply reflection gain (amplify for human perception)
    //    Typical contact forces (5-30N) → clearly perceptible (0.4-2.5N at Touch)
    //    Safety clamp at FORCE_MAX_TOUCH_N still applies in hapticCallback
    double gain = Config::FORCE_REFLECTION_GAIN;
    for (int i = 0; i < 3; i++) {
        fd.hapticOut[i] *= gain;
    }
}

void shutdown() {
    for (int i = 0; i < 6; i++) {
        g_filters[i].reset();
    }
}

} // namespace ForcePipeline
