// 独立测试: SweepPlan::sweepStateAt —— 8 段扫描的【段/倍速/FF/相位】调度
// Build: build_sweep_plan_test.bat   Run: test_sweep_plan.exe
// 【钉什么】段边界(10.0s 恰好落在哪一段) · 总时长之后的行为 · elapsed<0 · 倍速与 FF 的序列 · 相位。
// ⚠ 【不睡】: 时钟是入参。
//
// ★ 2026-10-01 修复轮 F3: `sweepStateAt` 的签名多了一个 `f0Hz`（源轨迹主频）。
//   相位从 `(段内秒/10s)×倍速` 改成 `段内秒 × 倍速 × f0`（取模 1）。
//   ⇒ 段内实际频率 = `speed × f0`，不再恒等于 `speed/10`。
//   本套件统一用 **f0 = 0.5 Hz**：段内 0.25 s 的相位 = `0.25×speed×0.5`，四个值
//   (0.0625 / 0.125 / 0.25 / 0.5) **在二进制里都精确**，读起来也是干净的 2 的幂。
#include <iostream>
#include <cmath>
#include "../relay/SweepPlan.h"

static int g_passed = 0, g_failed = 0;
#define TEST(name) do { std::cout << "  " << #name << "... "; } while(0)
#define PASS() do { std::cout << "PASS" << std::endl; g_passed++; } while(0)
#define CHECK(cond) do { if (!(cond)) { std::cout << "FAIL: " << #cond << std::endl; g_failed++; return; } } while(0)
#define NEAR(a,b) (std::fabs((a)-(b)) < 1e-9)

using SweepPlan::sweepStateAt;

// 本套件统一的源主频。⚠ 取 0.5 只为"钉出来的数字好读好核"，**不是**产品值
//   （产品传的是 `SweepWaveform::kF0Hz = 0.365`）。判据只依赖它是个**正的常数**。
static const double kF0 = 0.5;

// 格 1: t=0 是第 0 段、倍速 0.5、FF 开、相位 0
static void test_start_segment() {
    TEST(start_segment);
    auto s = sweepStateAt(0.0, kF0);
    CHECK(s.seg == 0);
    CHECK(NEAR(s.speed, 0.5));
    CHECK(s.ffOn);
    CHECK(NEAR(s.phase01, 0.0));
    PASS();
}

// 格 2: 段边界 —— t=10.0 恰好进入第 1 段（不是第 0 段）
static void test_boundary_is_half_open() {
    TEST(boundary_is_half_open);
    auto a = sweepStateAt(9.999999, kF0);
    auto b = sweepStateAt(10.0, kF0);
    CHECK(a.seg == 0);
    CHECK(b.seg == 1);
    PASS();
}

// 格 3: 倍速序列 = 0.5 / 1 / 2 / 4（四段一轮）
static void test_speed_sequence() {
    TEST(speed_sequence);
    CHECK(NEAR(sweepStateAt(0.0, kF0).speed, 0.5));
    CHECK(NEAR(sweepStateAt(10.0, kF0).speed, 1.0));
    CHECK(NEAR(sweepStateAt(20.0, kF0).speed, 2.0));
    CHECK(NEAR(sweepStateAt(30.0, kF0).speed, 4.0));
    // ★ 2026-10-01: 新增第 5 档 6×(≈2.19 Hz, 为够到 1~3 Hz 那扇门; 不取 8× 的理由见 SweepPlan.h)。
    CHECK(NEAR(sweepStateAt(40.0, kF0).speed, 6.0));
    PASS();
}

// 格 4: FF 在后四段关
static void test_ff_off_in_second_half() {
    TEST(ff_off_in_second_half);
    // ★ 2026-10-01: 档数 4→5 ⇒ 两半的边界由 40s 挪到 50s, 总时长 80s→100s。
    CHECK(sweepStateAt(49.0, kF0).ffOn);
    CHECK(!sweepStateAt(50.0, kF0).ffOn);
    CHECK(!sweepStateAt(99.9, kF0).ffOn);
    PASS();
}

// 格 5: 相位 = 【段内秒 × 倍速 × f0】, 取模到 [0,1)。
//   四段都取"段内 0.25 s" ⇒ 相位 = 0.25 × speed × 0.5 = 0.125 × speed，逐倍速翻倍。
static void test_phase_accumulates_with_speed() {
    TEST(phase_accumulates_with_speed);
    CHECK(NEAR(sweepStateAt(0.25,  kF0).phase01, 0.0625));   // 段0: 0.25×0.5×0.5
    CHECK(NEAR(sweepStateAt(10.25, kF0).phase01, 0.125));    // 段1: 0.25×1.0×0.5
    CHECK(NEAR(sweepStateAt(20.25, kF0).phase01, 0.25));     // 段2: 0.25×2.0×0.5
    CHECK(NEAR(sweepStateAt(30.25, kF0).phase01, 0.5));      // 段3: 0.25×4.0×0.5
    // ★ 取模这一条单独钉一格值: 段3(4x) 的段内 0.75 s ⇒ 0.75×4×0.5 = 1.5 圈 ⇒ 0.5
    //   ⚠ 简报原文此处写作 `1.0 % 1.0`, 那不是合法 C++（`%` 只对整型有定义, MSVC C2296/C2297）
    //     ⇒ 实现用的是 `phase -= std::floor(phase)`, 这里按它的语义钉。
    CHECK(NEAR(sweepStateAt(30.75, kF0).phase01, 0.5));
    PASS();
}

// 格 5b: 【f0 这一项本身】—— 同一 elapsed、不同 f0 ⇒ 不同相位。
//   ⛔ 这是 F3 的红对照。F3 之前的实现是 `(段内秒/kSegSec) × speed`, 它**根本不读 f0**；
//      若这个项再被丢掉（或 `f0Hz` 传进来没用上），本格三行会**同时**红：
//        · 两个 `NEAR` 的具体值对不上（退回旧式 = 0.0125）
//        · 最后那行 `!NEAR(a, b)` 直接失败（丢掉 f0 ⇒ 两个相位**相等**）
//      ⇒ 本格存在的唯一理由就是"让 f0 项被删这件事**必须**变红"。
static void test_phase_scales_with_f0() {
    TEST(phase_scales_with_f0);
    const double a = sweepStateAt(0.25, 0.5).phase01;    // 0.25×0.5×0.5
    const double b = sweepStateAt(0.25, 1.0).phase01;    // 0.25×0.5×1.0
    const double c = sweepStateAt(0.25, 0.25).phase01;   // 0.25×0.5×0.25
    CHECK(NEAR(a, 0.0625));
    CHECK(NEAR(b, 0.125));
    CHECK(NEAR(c, 0.03125));
    CHECK(!NEAR(a, b));   // ★ 分辨力: 丢掉 f0 ⇒ a == b ⇒ 本行红
    CHECK(!NEAR(b, c));
    PASS();
}

// 格 6: 跑完之后(>= 总时长) 停在【最后一段的末尾】而不是越界
static void test_after_end_clamps_to_last_segment() {
    TEST(after_end_clamps_to_last_segment);
    auto s = sweepStateAt(SweepPlan::kTotalSec + 5.0, kF0);
    CHECK(s.seg == SweepPlan::kSegCount - 1);
    CHECK(!s.ffOn);
    PASS();
}

// 格 7: elapsed < 0 当作 0（不崩、不越界）
static void test_negative_elapsed_is_zero() {
    TEST(negative_elapsed_is_zero);
    auto s = sweepStateAt(-3.0, kF0);
    CHECK(s.seg == 0);
    CHECK(NEAR(s.speed, 0.5));
    PASS();
}

// 格 8: elapsed = NaN 当作 0 —— 与格 7 同一条守卫 `if (!(t > 0.0))`。
//   ⚠ 为什么必须单列一格: NaN 与任何数比较【恒假】, 所以 `t > 0.0` 为假 ⇒ 走守卫；
//     若守卫被删, `(int)std::floor(NaN/10)` 是 UB ⇒ 格 7/8 是这条守卫唯一的红对照。
static void test_nan_elapsed_is_zero() {
    TEST(nan_elapsed_is_zero);
    auto s = sweepStateAt(std::nan(""), kF0);
    CHECK(s.seg == 0);
    CHECK(NEAR(s.speed, 0.5));
    CHECK(s.ffOn);
    CHECK(NEAR(s.phase01, 0.0));
    PASS();
}

int main() {
    std::cout << "=== SweepPlan Tests ===" << std::endl;
    test_start_segment();
    test_boundary_is_half_open();
    test_speed_sequence();
    test_ff_off_in_second_half();
    test_phase_accumulates_with_speed();
    test_phase_scales_with_f0();
    test_after_end_clamps_to_last_segment();
    test_negative_elapsed_is_zero();
    test_nan_elapsed_is_zero();
    std::cout << std::endl << g_passed << " passed, " << g_failed << " failed" << std::endl;
    return g_failed > 0 ? 1 : 0;
}
