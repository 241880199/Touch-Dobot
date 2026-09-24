#pragma once
#include "../core/AppState.h"
#include "../config/Config.h"
#include "JitterStats.h"   // 抖动诊断的两个累加器 (2026-09-23, 只读、可删 —— 见下面那一节)

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
    // ★ 2026-09-24: 粘性阻尼项（环路阻尼）—— damping = −b · v（v = 器件速度 mm/s）。
    //   用途/合规性（不改静态倍率）/"为什么不是降增益"：见 Config::TOUCH_VISC_DAMPING 那一大段。
    //   ⚠ 符号是【负】：阻尼必须【反对】运动方向 —— 用例①从符号上钉住它。
    inline void viscousDamping(const double velMmS[3], double b, double out[3]) {
        out[0] = -b * velMmS[0];
        out[1] = -b * velMmS[1];
        out[2] = -b * velMmS[2];
    }

    inline double softDeadzone(double val, double threshold) {
        if (threshold <= 0.0) return val;
        const double a = val < 0.0 ? -val : val;
        if (a >= threshold) return val;      // 门限以上: 原样 (不改幅值)
        const double r = a / threshold;      // 0..1
        return val * r * r;                  // 门限以下: 三次律平滑到 0
    }

    // ===== 净比例与打顶阈值 —— 【全程序唯一一份】=====
    // 净比例 = FORCE_MAX_TOUCH_N / FORCE_MAX_SENSOR_N (逐单位增益)。现值 3.3/200 = 0.0165。
    // 【为什么抽出来】MATLAB 的 RG| 回读要报 ratio 与 satN, 而这两个量都从 3.3/200 来。
    //   若在 RelayCore 里再写一遍 3.3 和 200, 就是"同一个规则两份实现" ——
    //   本项目有成文教训 (见上面 softDeadzone 那段)。
    inline double netRatioPerGainUnit() {
        return Config::FORCE_MAX_TOUCH_N / Config::FORCE_MAX_SENSOR_N;
    }

    // 该轴的打顶阈值 (传感器牛顿): 输出撞上 FORCE_MAX_TOUCH_N 那个夹子时的输入值。
    //   = FORCE_MAX_TOUCH_N / (netRatioPerGainUnit() × gain) = 200/gain (gain=120 ⇒ 1.67N)
    // ⚠ 两点前提, 界面上也要写:
    //   (1) 【该轴分量】—— HapticCallback.cpp:228-234 是三个轴各自夹, 不是夹合力。
    //   (2) 它依赖 FORCE_CONSTRAINT_FORCES_ENABLED = false。那个开关翻回 true
    //       ⇒ totalForce 变成叠加值 ⇒ 夹点提前 ⇒ 这个数当场作废。
    //       这是一句"当前状态"的结论, 不是恒等式。
    //   ⚠ 本处【故意不写 Config.h 的行号】: 加这个常数的同一次提交就把它推漂了 (708 之后)。
    //     要行号就现场 grep 符号名。本项目有成文教训: 常数与行号一样脆。
    inline double saturationSensorN(double gain) {
        const double ratio = netRatioPerGainUnit() * gain;
        if (ratio <= 0.0) return 0.0;
        return Config::FORCE_MAX_TOUCH_N / ratio;
    }

    // Call once: initialize filter coefficients
    void init();

    // Call at the FRAME rate (~123 Hz, from ForceReader/RelayCore::forceReaderThread, 2026-09-21):
    // raw -> filtered -> hapticOut (writes into fd under caller's mutex)
    // ⚠ 这个速率【是 Butterworth 系数的前提】(Config::FORCE_FILTER_FS_HZ) —— 换了调用率就必须
    //   同步改它, 否则归一化频率 fc/fs 跟着错位 (2026-09-21 之前就栽在这: 系数按 120 Hz 算、
    //   实际跑在 ~11 Hz ⇒ 实际截止 2.75 Hz, 噪声只削掉 20%)。
    // ⚠ 【全程序只有一个调用点】: 若在 pollForce 里也调一次, 滤波器每帧被推两次 ⇒ 相位乱。
    void step(AppState::ForceData& fd);

    // Call on shutdown
    void shutdown();

    // ===== 抖动诊断 (2026-09-23, 抖动计划 Task 2) —— 【只取证, 不改任何行为】=====
    // step() 每帧把【两个不同的量】喂给两个【不可互换】的累加器 (见 ForcePipeline.cpp 里的
    // 那一段): hapticOut (= 手感, 判据 A) 与 fd.filtered[0..2] (= 进死区的残差, 判据 B)。
    // 本节的函数【只读】, 不参与任何控制输出; 回滚 = 删掉 step() 末尾那两行 addFrame。
    //
    // ⚠ 【线程契约】: 两个累加器由 ForceReader 线程在 step() 里写, 而 'n' 探针在 GLUT 线程
    //   读 ⇒ 读方必须持 appState.forceDataMutex (step() 正是持着它跑的)。为了让这条契约
    //   是结构性的而不是口头的, 这里【不导出累加器本身】, 只导出下面这个"值快照" ——
    //   拷贝必须在锁内做, 打印在锁外 (I/O 不许占着 forceDataMutex, 与 pollForce 同一纪律)。
    struct JitterSnapshot {
        int nOut = 0;                 // 窗口帧数 (= 自上次 'n' 起; 与 nResidual 必相等)
        int nResidual = 0;
        double meanOut[3] = {0, 0, 0};        // hapticOut 逐轴均值 (Touch N)
        double sdOut[3] = {0, 0, 0};          // hapticOut 逐轴 sd   (判据 A)
        double meanResidual[3] = {0, 0, 0};   // 残差逐轴均值 (传感器 N)
        double sdResidual[3] = {0, 0, 0};     // 残差逐轴 sd
        double fracResidual[3] = {0, 0, 0};   // |残差| >= FORCE_RESIDUAL_DEADZONE_N 的帧占比
                                              // (判据 B; <0 ⇒ 该值算不出来, 见 JitterStats.h)
    };

    // 取一份快照。【调用方必须持 appState.forceDataMutex】(见上面那段线程契约)。
    void copyJitterSnapshot(JitterSnapshot& out);

    // 清空两个窗口 (自 'n' 起重新计数)。【调用方必须持 appState.forceDataMutex】。
    void resetJitterStats();
}
