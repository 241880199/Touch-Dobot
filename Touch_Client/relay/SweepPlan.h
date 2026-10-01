#pragma once
#include <cmath>

// 扫描的【段调度】: 给定从按下 'r' 起算的秒数, 返回这一段用哪个倍速、力反馈开不开、走到环上哪个相位。
// 纯函数、无状态、无时钟 —— 理由同 GainReadbackPolicy.h: 接线那半测不了, 能测的这半要厚。
//
// 排布: 前 4 段 FF 开, 后 4 段 FF 关; 每段内部倍速依次 0.5 / 1 / 2 / 4。
//   ⇒ 同一段里 FF 开与关【动作逐帧相同】—— 这正是 B' 那次栽掉的对照条件。
namespace SweepPlan {

constexpr double kSegSec   = 10.0;
constexpr int    kSegCount = 8;                 // 4 倍速 × 2 个 FF 状态
constexpr double kTotalSec = kSegSec * kSegCount;
constexpr double kSpeed[4] = { 0.5, 1.0, 2.0, 4.0 };

struct State { int seg; double speed; bool ffOn; double phase01; };

inline State sweepStateAt(double elapsedSec) {
    double t = elapsedSec;
    if (!(t > 0.0)) t = 0.0;                    // 负 / NaN 都当 0
    int seg = (int)std::floor(t / kSegSec);
    double inSeg = t - seg * kSegSec;
    if (seg >= kSegCount) {                     // 跑完: 停在最后一段的末尾
        seg = kSegCount - 1;
        inSeg = kSegSec;
    }
    const double speed = kSpeed[seg % 4];
    const bool   ffOn  = (seg < 4);
    double phase = (inSeg / kSegSec) * speed;   // 段内进度 × 倍速 = 走了多少圈
    phase -= std::floor(phase);                 // 取模到 [0,1)
    return State{ seg, speed, ffOn, phase };
}

}  // namespace SweepPlan
