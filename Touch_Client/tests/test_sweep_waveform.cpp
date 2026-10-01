// 独立测试: SweepWaveform::lookup —— 单位峰值轨迹的【环上查表】
// Build: build_sweep_waveform_test.bat   Run: test_sweep_waveform.exe
// 【钉什么】① phase 取模(负值/超一圈); ② 线性插值的端点与中点; ③ 环闭合(相位 0 与 1 同点)。
// ⚠ 【不睡】: 纯函数, 无时钟。
#include <iostream>
#include <cmath>
#include "../relay/SweepWaveform.h"

using SweepWaveform::Vec3;      // 表与 lookup 都在 namespace SweepWaveform 里

static int g_passed = 0, g_failed = 0;
#define TEST(name) do { std::cout << "  " << #name << "... "; } while(0)
#define PASS() do { std::cout << "PASS" << std::endl; g_passed++; } while(0)
#define CHECK(cond) do { if (!(cond)) { std::cout << "FAIL: " << #cond << std::endl; g_failed++; return; } } while(0)

// 格 1: phase=0 就是表的第一行
static void test_phase_zero_is_first_row() {
    TEST(phase_zero_is_first_row);
    Vec3 v = SweepWaveform::lookup(0.0);
    CHECK(std::fabs(v.x - SweepWaveform::kTable[0][0]) < 1e-12);
    CHECK(std::fabs(v.y - SweepWaveform::kTable[0][1]) < 1e-12);
    CHECK(std::fabs(v.z - SweepWaveform::kTable[0][2]) < 1e-12);
    PASS();
}

// 格 2: 环闭合 —— phase=1 与 phase=0 是同一点（不是表的最后一行）
static void test_phase_one_wraps_to_zero() {
    TEST(phase_one_wraps_to_zero);
    Vec3 a = SweepWaveform::lookup(0.0);
    Vec3 b = SweepWaveform::lookup(1.0);
    CHECK(std::fabs(a.x - b.x) < 1e-12 && std::fabs(a.y - b.y) < 1e-12 && std::fabs(a.z - b.z) < 1e-12);
    PASS();
}

// 格 3: 取模 —— -0.25 与 0.75 同点
static void test_negative_phase_wraps() {
    TEST(negative_phase_wraps);
    Vec3 a = SweepWaveform::lookup(-0.25);
    Vec3 b = SweepWaveform::lookup(0.75);
    CHECK(std::fabs(a.x - b.x) < 1e-12 && std::fabs(a.y - b.y) < 1e-12 && std::fabs(a.z - b.z) < 1e-12);
    PASS();
}

// 格 4: 超一圈取模 —— 2.5 与 0.5 同点
static void test_multi_turn_wraps() {
    TEST(multi_turn_wraps);
    Vec3 a = SweepWaveform::lookup(2.5);
    Vec3 b = SweepWaveform::lookup(0.5);
    CHECK(std::fabs(a.x - b.x) < 1e-12 && std::fabs(a.y - b.y) < 1e-12 && std::fabs(a.z - b.z) < 1e-12);
    PASS();
}

// 格 5: 相邻表项的【中点】就是两者平均（线性插值）
static void test_midpoint_is_linear() {
    TEST(midpoint_is_linear);
    const double h = 1.0 / SweepWaveform::kTableN;
    Vec3 a = SweepWaveform::lookup(0.0);
    Vec3 b = SweepWaveform::lookup(h);
    Vec3 m = SweepWaveform::lookup(h * 0.5);
    CHECK(std::fabs(m.x - 0.5 * (a.x + b.x)) < 1e-12);
    CHECK(std::fabs(m.y - 0.5 * (a.y + b.y)) < 1e-12);
    CHECK(std::fabs(m.z - 0.5 * (a.z + b.z)) < 1e-12);
    PASS();
}

// 格 6: 表上每一点的模长都 <= 1（归一化到单位峰值, 不是"每点都等于 1"）
static void test_peak_is_one_and_never_exceeded() {
    TEST(peak_is_one_and_never_exceeded);
    double mx = 0.0;
    for (int j = 0; j < SweepWaveform::kTableN; j++) {
        Vec3 v = SweepWaveform::lookup((double)j / SweepWaveform::kTableN);
        double r = std::sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
        if (r > mx) mx = r;
    }
    CHECK(std::fabs(mx - 1.0) < 1e-9);          // 峰值恰为 1
    PASS();
}

// 格 7: 回绕那一格 —— 相位落在【最后一个表项与第 0 项之间】的中点上。
//   ⚠ 为什么必须单列一格: 格 6 用 j/N 采样, 每个点都【正好落在表行上】(插值权重 f=0)
//     ⇒ 末项 -> 第 0 项那个区间【从来没被真正走到】; 去掉环回绕 `% kTableN` 也照样全绿。
//     本格取中点(f=0.5), 越界读就会被乘进去 ⇒ 那才是能红的那一格。
static void test_wrap_interval_midpoint() {
    TEST(wrap_interval_midpoint);
    const double h = 1.0 / SweepWaveform::kTableN;
    Vec3 a = SweepWaveform::lookup(1.0 - h * 0.5);          // i0 = N-1, i1 = 0
    Vec3 b = SweepWaveform::lookup(0.0);                     // 第 0 项
    Vec3 c = SweepWaveform::lookup(1.0 - h);                 // 第 N-1 项
    CHECK(std::fabs(a.x - 0.5 * (b.x + c.x)) < 1e-12);
    CHECK(std::fabs(a.y - 0.5 * (b.y + c.y)) < 1e-12);
    CHECK(std::fabs(a.z - 0.5 * (b.z + c.z)) < 1e-12);
    PASS();
}

int main() {
    std::cout << "=== SweepWaveform Tests ===" << std::endl;
    test_phase_zero_is_first_row();
    test_phase_one_wraps_to_zero();
    test_negative_phase_wraps();
    test_multi_turn_wraps();
    test_midpoint_is_linear();
    test_peak_is_one_and_never_exceeded();
    test_wrap_interval_midpoint();
    std::cout << std::endl << g_passed << " passed, " << g_failed << " failed" << std::endl;
    return g_failed > 0 ? 1 : 0;
}
