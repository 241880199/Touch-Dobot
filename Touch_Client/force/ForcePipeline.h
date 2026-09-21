#pragma once
#include "../core/AppState.h"

// 2nd-order Butterworth lowpass filter (biquad form)
// One instance per channel, zero-phase initialization
class Butterworth2 {
public:
    Butterworth2();
    void reset();
    double step(double input);
    // coefficients — public so init() can set them
    double b0, b1, b2, a1, a2;
private:
    double x1, x2, y1, y2;       // delay states
};

namespace ForcePipeline {
    // ===== ★ 死区的【唯一一份定义】(2026-09-21) =====
    // 凡是"要按 FORCE_RESIDUAL_DEADZONE_N 抑制小信号"的地方, 【都必须调这一个】。
    //
    // 【为什么抽出来】这条规则曾经有【两份实现】, 而且两份都是硬门:
    //   · ForcePipeline::mapForceToTouch 里那份 (触觉那一路);
    //   · RelayCore 构造 F| 消息时又写了一遍 (MATLAB 显示的那一路)。
    //   我只修了前者, 于是 F| 那条路上的阶跃照旧 ⇒ 现场看到"MATLAB 上 FZ 在 0 与 ±0.2 之间
    //   阶跃式跳"。**同一个规则两份实现, 改一份忘一份** —— 本项目反复吃过这类账。
    //
    // 【为什么是软门而不是硬门】硬门 `if(|v| < dz) 0 else v` 在阈值处【不连续】: 静止残余
    //   恰好悬在 dz 附近时 (实测 'z' 之后 comp_z ≈ −0.14~−0.17, 而 dz = 0.20), 它在 0.20
    //   上下抖一点, 输出就在 0 与 ~0.2 之间【整段跳】。
    // 【软门】低于 dz 按 (|v|/dz)² 平滑衰减到 0; **高于 dz 仍是 v (1:1, 幅值不变)**。
    //   ⇒ 在 dz 处【值连续】(r=1 时 val·1² = val) ⇒ 没有跳变 ⇒ 不跳 ✓
    //   ⇒ 而笔压 (0.3~0.6 N) 远在 dz 之上 ⇒ 幅值完全不受影响 ✓
    // ⚠ 性质如实说: 它是 C⁰ (值连续、斜率有折点) —— 而"值不连续"正是跳变的来源, C⁰ 已足够。
    // 由 tests/test_force_pipeline.cpp 的 soft_deadzone_no_jump 用例钉住 (门限两侧之差 < 0.02)。
    inline double softDeadzone(double val, double threshold) {
        if (threshold <= 0.0) return val;
        const double a = val < 0.0 ? -val : val;
        if (a >= threshold) return val;      // 门限以上: 原样 (不改幅值)
        const double r = a / threshold;      // 0..1
        return val * r * r;                  // 门限以下: 三次律平滑到 0
    }

    // Call once: initialize filter coefficients
    void init();

    // Call at 30Hz: raw -> filtered -> hapticOut (writes into fd under caller's mutex)
    void step(AppState::ForceData& fd);

    // Call on shutdown
    void shutdown();
}
