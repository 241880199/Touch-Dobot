#pragma once
#include <cmath>

// 扫描的【段调度】: 给定从按下 'r' 起算的秒数, 返回这一段用哪个倍速、力反馈开不开、走到环上哪个相位。
// 纯函数、无状态、无时钟 —— 理由同 GainReadbackPolicy.h: 接线那半测不了, 能测的这半要厚。
//
// 排布: 前一半 FF 开, 后一半 FF 关; 两半内部倍速依次相同。
//   ⇒ 同一档的 FF 开与关【动作逐帧相同】—— 这正是 B' 那次栽掉的对照条件。
//
// ★ 2026-10-01：档数由 4 加到 5（加 6× ⇒ 约 2.19 Hz）。
//   【为什么是 6 而不是 8】偏移有一道 **4.5 mm/帧 @30Hz = 135 mm/s** 的增量限幅（见 RelayCore 的 G1）。
//   8 mm 幅度下最大波形斜率 = 2π·f·A：4× (1.46Hz) → 73 mm/s ✓；**8× (2.92Hz) → 147 mm/s ✗ 会被削**；
//   6× (2.19Hz) → **110 mm/s ✓ 不削**。⇒ 选 6× 既进得了 1~3 Hz 那扇门，又**不用去动那个刚被评审过的安全参数**。
//
// ⚠★ `f0Hz` 是【源轨迹的主频】, 由调用方传进来（实际传的是 `SweepWaveform::kF0Hz`）。
//   **作为入参、而不是本文件 include 进来**：本单元要保持与波形单元【无关】
//   （它只有 <cmath>, 单测也不需要链接任何波形数据）—— 与 `GainReadbackPolicy.h` 同一条边界。
//   · 段内实际频率 = `speed × f0Hz`（倍速就是"把源轨迹放快/放慢"的意思）。
//   · 相位 = 段内秒 × speed × f0Hz, 取模到 [0,1)（一个表周期 = 源轨迹的一个周期）。
namespace SweepPlan {

constexpr double kSegSec     = 10.0;
constexpr int    kSpeedCount = 5;                 // 倍速档数
constexpr int    kSegCount   = kSpeedCount * 2;   // 每档两次: 前一半 FF 开、后一半 FF 关
constexpr double kTotalSec   = kSegSec * kSegCount;
constexpr double kSpeed[kSpeedCount] = { 0.5, 1.0, 2.0, 4.0, 6.0 };

struct State { int seg; double speed; bool ffOn; double phase01; };

inline State sweepStateAt(double elapsedSec, double f0Hz) {
    double t = elapsedSec;
    if (!(t > 0.0)) t = 0.0;                    // 负 / NaN 都当 0
    int seg = (int)std::floor(t / kSegSec);
    double inSeg = t - seg * kSegSec;
    if (seg >= kSegCount) {                     // 跑完: 停在最后一段的末尾
        seg = kSegCount - 1;
        inSeg = kSegSec;
    }
    // ⚠ 两处都用 kSpeedCount/kSegCount 推出来, 不写死 4/5 —— 下次改档数不会再漏一处。
    const double speed = kSpeed[seg % kSpeedCount];
    const bool   ffOn  = (seg < kSegCount / 2);
    // ★ 2026-10-01 修复轮 F3：段内 t 秒走过的【圈数】= t × 倍速 × f0。
    //   ⛔ 从前这里写的是 `(inSeg / kSegSec) * speed` —— 那等于"一圈恒占 `kSegSec/speed` 秒"
    //     ⇒ 实际频率 = `speed / kSegSec` = 0.05 / 0.10 / 0.20 / 0.40 Hz，**与源频率 f0 无关**。
    //     后果是整轮只扫到 0.05~0.4 Hz（而本设计存在的理由就是够到 **1~3 Hz** 那一段），
    //     并且控制台会一边打 `speed=4.00x` 一边实际跑 0.40 Hz —— 正是"显示 ≠ 实际"。
    //   ⇒ 改成按 f0 参数化（**不是**把波形单元 include 进来，见文件顶那一段）。
    double phase = inSeg * speed * f0Hz;        // 段内秒 × 倍速 × f0 = 走了多少圈
    phase -= std::floor(phase);                 // 取模到 [0,1)
    return State{ seg, speed, ffOn, phase };
}

}  // namespace SweepPlan
