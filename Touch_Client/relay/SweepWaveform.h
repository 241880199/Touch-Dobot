#pragma once
#include <cmath>
#include "SweepWaveformData.h"   // 生成物: kF0Hz / kTableN / kTable[N][3]

// 扫描回放用的【单位峰值】位移轨迹: 一个 128 点的闭环, 环上任意相位线性插值。
//
// 【为什么是纯函数】与 GainReadbackPolicy.h 同款理由: 接线那半 (RelayCore.cpp)
//   不被任何测试编译, 所以能测的这半要尽量厚。这里没有时钟、没有状态。
//
// ⚠ 【环是 128 点闭合的】: 相位 1.0 与 0.0 是【同一点】, 不是表的最后一行。
//   ⇒ 查表索引要取模到 [0, N), 并且插值要**跨过末尾回到开头**(用 %N 拿下一个点)。
namespace SweepWaveform {

struct Vec3 { double x, y, z; };

inline Vec3 lookup(double phase01) {
    // 取模到 [0,1): 支持负数与多圈
    double p = phase01 - std::floor(phase01);
    const double scaled = p * (double)kTableN;
    int i0 = (int)std::floor(scaled);
    if (i0 >= kTableN) i0 = kTableN - 1;      // p 极接近 1 时的兜底
    const int i1 = (i0 + 1) % kTableN;        // ★ 环: 末点之后回到第 0 点
    const double f = scaled - (double)i0;
    const double a = 1.0 - f;
    return Vec3{
        kTable[i0][0] * a + kTable[i1][0] * f,
        kTable[i0][1] * a + kTable[i1][1] * f,
        kTable[i0][2] * a + kTable[i1][2] * f,
    };
}

}  // namespace SweepWaveform
