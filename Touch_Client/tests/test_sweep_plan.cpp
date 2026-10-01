// 独立测试: SweepPlan::sweepStateAt —— 8 段扫描的【段/倍速/FF/相位】调度
// Build: build_sweep_plan_test.bat   Run: test_sweep_plan.exe
// 【钉什么】段边界(10.0s 恰好落在哪一段) · 总时长之后的行为 · elapsed<0 · 倍速与 FF 的序列 · 相位。
// ⚠ 【不睡】: 时钟是入参。
#include <iostream>
#include <cmath>
#include "../relay/SweepPlan.h"

static int g_passed = 0, g_failed = 0;
#define TEST(name) do { std::cout << "  " << #name << "... "; } while(0)
#define PASS() do { std::cout << "PASS" << std::endl; g_passed++; } while(0)
#define CHECK(cond) do { if (!(cond)) { std::cout << "FAIL: " << #cond << std::endl; g_failed++; return; } } while(0)
#define NEAR(a,b) (std::fabs((a)-(b)) < 1e-9)

using SweepPlan::sweepStateAt;

// 格 1: t=0 是第 0 段、倍速 0.5、FF 开、相位 0
static void test_start_segment() {
    TEST(start_segment);
    auto s = sweepStateAt(0.0);
    CHECK(s.seg == 0);
    CHECK(NEAR(s.speed, 0.5));
    CHECK(s.ffOn);
    CHECK(NEAR(s.phase01, 0.0));
    PASS();
}

// 格 2: 段边界 —— t=10.0 恰好进入第 1 段（不是第 0 段）
static void test_boundary_is_half_open() {
    TEST(boundary_is_half_open);
    auto a = sweepStateAt(9.999999);
    auto b = sweepStateAt(10.0);
    CHECK(a.seg == 0);
    CHECK(b.seg == 1);
    PASS();
}

// 格 3: 倍速序列 = 0.5 / 1 / 2 / 4（四段一轮）
static void test_speed_sequence() {
    TEST(speed_sequence);
    CHECK(NEAR(sweepStateAt(0.0).speed, 0.5));
    CHECK(NEAR(sweepStateAt(10.0).speed, 1.0));
    CHECK(NEAR(sweepStateAt(20.0).speed, 2.0));
    CHECK(NEAR(sweepStateAt(30.0).speed, 4.0));
    PASS();
}

// 格 4: FF 在后四段关
static void test_ff_off_in_second_half() {
    TEST(ff_off_in_second_half);
    CHECK(sweepStateAt(39.0).ffOn);
    CHECK(!sweepStateAt(40.0).ffOn);
    CHECK(!sweepStateAt(79.9).ffOn);
    PASS();
}

// 格 5: 相位 = (段内时间 / 10s) × 倍速, 取模到 [0,1) —— 倍速 2 的段里 1 秒走两圈
static void test_phase_accumulates_with_speed() {
    TEST(phase_accumulates_with_speed);
    CHECK(NEAR(sweepStateAt(2.5).phase01, 0.125));    // 段0: 0.5x, 2.5s/10s*0.5 = 0.125
    CHECK(NEAR(sweepStateAt(12.5).phase01, 0.25));    // 段1: 1x
    CHECK(NEAR(sweepStateAt(22.5).phase01, 0.5));     // 段2: 2x
    CHECK(NEAR(sweepStateAt(32.5).phase01, 0.0));     // 段3: 4x, 2.5/10*4 = 1.0 -> 取模 0
                                                       //   ⚠ 简报原文此处写作 `1.0 % 1.0`, 那不是合法 C++
                                                       //     (`%` 只对整型有定义, MSVC C2296/C2297) ⇒ 等价改成 0.0。
    PASS();
}

// 格 6: 跑完之后(>= 总时长) 停在【最后一段的末尾】而不是越界
static void test_after_end_clamps_to_last_segment() {
    TEST(after_end_clamps_to_last_segment);
    auto s = sweepStateAt(SweepPlan::kTotalSec + 5.0);
    CHECK(s.seg == SweepPlan::kSegCount - 1);
    CHECK(!s.ffOn);
    PASS();
}

// 格 7: elapsed < 0 当作 0（不崩、不越界）
static void test_negative_elapsed_is_zero() {
    TEST(negative_elapsed_is_zero);
    auto s = sweepStateAt(-3.0);
    CHECK(s.seg == 0);
    CHECK(NEAR(s.speed, 0.5));
    PASS();
}

// 格 8: elapsed = NaN 当作 0 —— 与格 7 同一条守卫 `if (!(t > 0.0))`。
//   ⚠ 为什么必须单列一格: NaN 与任何数比较【恒假】, 所以 `t > 0.0` 为假 ⇒ 走守卫；
//     若守卫被删, `(int)std::floor(NaN/10)` 是 UB ⇒ 格 7/8 是这条守卫唯一的红对照。
static void test_nan_elapsed_is_zero() {
    TEST(nan_elapsed_is_zero);
    auto s = sweepStateAt(std::nan(""));
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
    test_after_end_clamps_to_last_segment();
    test_negative_elapsed_is_zero();
    test_nan_elapsed_is_zero();
    std::cout << std::endl << g_passed << " passed, " << g_failed << " failed" << std::endl;
    return g_failed > 0 ? 1 : 0;
}
