// Standalone test: JitterStats — 抖动诊断用的纯统计累加器
// Build: build_jitter_stats_test.bat
// Run: test_jitter_stats.exe
//
// 【为什么这些用例值得写】这个累加器本身不算错什么, 但它是【两个判据的唯一来源】:
// 现场报的"阻力的方向一直在抖动"要靠它给出 ①hapticOut 在帧率上的逐轴 sd、
// ②残差越过死区 (Config::FORCE_RESIDUAL_DEADZONE_N = 0.20 N) 的帧占比。
// 判据 (见 Docs/superpowers/plans/2026-09-23-haptic-output-jitter-instrument.md) 是
// "占比 ≈ 15~25% ⇒ 机制成立", 所以 sd 或占比算错, 结论会【反过来】——
// 而这正是本项目最怕的"安静地错": 数照样印出来, 只是它是错的。
//
// 真值一律用【构造序列】从定义算出来 (见每个用例的注释), 不拿实现去对实现。
//
// ⚠ 断言宏: 本文件的 TEST 是【标签打印器】(只印名字, 不求值也不计失败), 真正求值并计
//   失败的是 CHECK —— 写成 TEST(<表达式>) 会让用例无论对错都绿 (本仓 2026-09-23 实测复现过)。

#include <iostream>
#include <cmath>

#include "../force/JitterStats.h"

static int g_passed = 0, g_failed = 0;

#define TEST(name) do { std::cout << "  " << #name << "... "; } while(0)
#define PASS() do { std::cout << "PASS" << std::endl; g_passed++; } while(0)
#define CHECK(cond) do { if (!(cond)) { std::cout << "FAIL: " << #cond << std::endl; g_failed++; return; } } while(0)

// 浮点比较一律带容差。这里是【断言容差】, 不是实现容差 —— 它只决定"算不算相等",
// 不参与任何计算。
static bool near(double a, double b, double eps = 1e-12) {
    return std::fabs(a - b) <= eps;
}

// ===== 常量序列 ⇒ sd 恒等于 0 =====
// 这【不是】顺带的一条: 它正是"两遍算法"相对 E[x²]-E[x]² 那个写法的价值所在 ——
// 逐项减去均值后偏差恰好是 0, 平方和也恰好是 0, 所以 sd 是【精确的 0】。
// 一遍写法要拿 Σx² 与 n·mean² 相减, 两个大数几乎相等, 相减会掉有效位, 常量序列都可能
// 得到一个非 0 (甚至因中间量转负而得到 NaN)。静止时的 sd 是判据的分母, 这里不能有假底噪。
static void test_constant_sequence_sd_is_exactly_zero() {
    TEST(constant_sequence_sd_is_exactly_zero);
    JitterStats s;
    const double v[3] = {2.5, -1.0, 0.0};
    for (int i = 0; i < 4; i++) s.addFrame(v);
    CHECK(s.n() == 4);
    for (int a = 0; a < 3; a++) {
        CHECK(s.mean(a) == v[a]);      // 常量序列的均值就是它自己, 可以精确比
        CHECK(s.sd(a) == 0.0);         // 【精确】0, 不是 0.0001
    }
    PASS();
}

// ===== 已知序列 {1,2,3,4} =====
// 真值从定义推: 均值 = (1+2+3+4)/4 = 2.5
//   偏差 = -1.5, -0.5, +0.5, +1.5 ⇒ 平方和 = 2.25+0.25+0.25+2.25 = 5
//   样本 sd (÷ n-1 = 3) = sqrt(5/3) ≈ 1.2909944487358056
static void test_known_sequence_mean_and_sd() {
    TEST(known_sequence_mean_and_sd);
    JitterStats s;
    for (int i = 1; i <= 4; i++) {
        const double v[3] = {(double)i, (double)i, (double)i};
        s.addFrame(v);
    }
    CHECK(s.n() == 4);
    for (int a = 0; a < 3; a++) {
        CHECK(near(s.mean(a), 2.5));
        // 与闭式解比, 不是与实现比
        CHECK(near(s.sd(a), std::sqrt(5.0 / 3.0), 1e-12));
        CHECK(near(s.sd(a), 1.2909944, 1e-6));   // 再对一个十进制字面量, 免得闭式解写错了还自洽
    }
    PASS();
}

// ===== fracAbove: 取【绝对值】, 且边界算"越" =====
// 序列 {0.1, 0.25, -0.3, 0.2}, thr = 0.2 ⇒ 逐帧判 |v| >= 0.2:
//   0.1  -> 0.1  < 0.2  不算
//   0.25 -> 0.25 >= 0.2 算
//   -0.3 -> 0.3  >= 0.2 算   ← 只看大小, 方向不分正负 (死区是两侧对称的)
//   0.2  -> 0.2  >= 0.2 算   ← 边界【算越】, 与"取 >= "这条约定一致
//   ⇒ 3/4 = 0.75
static void test_fracAbove_uses_magnitude_and_includes_boundary() {
    TEST(fracAbove_uses_magnitude_and_includes_boundary);
    JitterStats s;
    const double seq[4] = {0.1, 0.25, -0.3, 0.2};
    for (int i = 0; i < 4; i++) {
        const double v[3] = {seq[i], seq[i], seq[i]};
        s.addFrame(v);
    }
    CHECK(s.n() == 4);
    for (int a = 0; a < 3; a++) CHECK(near(s.fracAbove(a, 0.20), 0.75));
    PASS();
}

// 边界两侧各钉一次: 恰好等于阈值要算, 只差一点就不算。
// (分开写, 因为"边界算不算"与"绝对值用没用"是两个能各自改错的决定。)
static void test_fracAbove_boundary_just_below_is_excluded() {
    TEST(fracAbove_boundary_just_below_is_excluded);
    JitterStats s;
    const double v[3] = {0.2 - 1e-9, 0.2 - 1e-9, 0.2 - 1e-9};
    s.addFrame(v);
    CHECK(near(s.fracAbove(0, 0.2), 0.0));     // 差一点点 ⇒ 不算
    PASS();
}

// 全零序列 + thr = 0: |0| >= 0 也算越 —— 这一条钉的是"用的是 >=, 不是 >"。
// (真实调用不会传 thr=0; 写它是为了让 >= 这个约定有个不依赖 0.20 这个数的证据。)
static void test_fracAbove_zero_threshold_counts_zeros() {
    TEST(fracAbove_zero_threshold_counts_zeros);
    JitterStats s;
    const double v[3] = {0.0, 0.0, 0.0};
    s.addFrame(v);
    s.addFrame(v);
    CHECK(near(s.fracAbove(0, 0.0), 1.0));
    PASS();
}

// ===== n = 0 / n = 1: 不许除零, 也不许给一个"看着像结论"的数 =====
static void test_empty_returns_zero_no_division_by_zero() {
    TEST(empty_returns_zero_no_division_by_zero);
    JitterStats s;
    CHECK(s.n() == 0);
    for (int a = 0; a < 3; a++) {
        CHECK(s.mean(a) == 0.0);
        CHECK(s.sd(a) == 0.0);              // n < 2 ⇒ 0 (样本 sd 在 n=1 时无定义)
        CHECK(s.fracAbove(a, 0.20) == 0.0); // 0 帧里越过的帧占比无定义 ⇒ 0, 不是 NaN
    }
    PASS();
}

static void test_single_sample_sd_is_zero() {
    TEST(single_sample_sd_is_zero);
    JitterStats s;
    const double v[3] = {7.0, -7.0, 0.5};
    s.addFrame(v);
    CHECK(s.n() == 1);
    for (int a = 0; a < 3; a++) {
        CHECK(s.mean(a) == v[a]);   // 单样本的均值就是它自己
        CHECK(s.sd(a) == 0.0);      // ÷ (n-1) 在 n=1 时会除零 ⇒ 必须短路成 0
    }
    PASS();
}

// ===== 逐轴独立 =====
// 三根轴喂【不同】的序列。任何"三轴串味"的实现 (共用累加器 / 索引写错) 都会在这里露出来:
//   轴0 {1,2,3,4}      均值 2.5  sd = sqrt(5/3) ≈ 1.2909944
//   轴1 {0,0,0,0}      均值 0    sd = 0
//   轴2 {-1,1,-1,1}    均值 0    平方和 = 4 ⇒ sd = sqrt(4/3) ≈ 1.1547005
static void test_axes_are_independent() {
    TEST(axes_are_independent);
    JitterStats s;
    const double a0[4] = {1, 2, 3, 4};
    const double a1[4] = {0, 0, 0, 0};
    const double a2[4] = {-1, 1, -1, 1};
    for (int i = 0; i < 4; i++) {
        const double v[3] = {a0[i], a1[i], a2[i]};
        s.addFrame(v);
    }
    CHECK(s.n() == 4);
    CHECK(near(s.mean(0), 2.5));
    CHECK(near(s.sd(0), std::sqrt(5.0 / 3.0), 1e-12));
    CHECK(near(s.mean(1), 0.0));
    CHECK(s.sd(1) == 0.0);
    CHECK(near(s.mean(2), 0.0));
    CHECK(near(s.sd(2), std::sqrt(4.0 / 3.0), 1e-12));
    // 三轴的 sd 互不相同, 所以上面几条不是"恰好都对"
    CHECK(s.sd(0) != s.sd(2));
    PASS();
}

// ===== reset =====
// 'n' 探针是"自上次 'n' 起累计 N 帧"(见 Task 2), 所以 reset 之后必须是干净的空状态 ——
// 残留任何一个数都会让下一次的 sd 掺进上一次的数据, 而且【看不出来】。
static void test_reset_clears_everything() {
    TEST(reset_clears_everything);
    JitterStats s;
    for (int i = 1; i <= 4; i++) {
        const double v[3] = {(double)i, 100.0, -100.0};
        s.addFrame(v);
    }
    CHECK(s.n() == 4);
    s.reset();
    CHECK(s.n() == 0);
    for (int a = 0; a < 3; a++) {
        CHECK(s.mean(a) == 0.0);
        CHECK(s.sd(a) == 0.0);
        CHECK(s.fracAbove(a, 0.20) == 0.0);
    }
    // reset 之后还能继续用, 且只看得见新数据
    const double v[3] = {5.0, 5.0, 5.0};
    s.addFrame(v);
    CHECK(s.n() == 1);
    CHECK(near(s.mean(0), 5.0));
    PASS();
}

// ===== 越界轴 =====
// 调用侧是一个 for (a = 0; a < 3; a++) 的循环, 正常不会传越界值。这条守卫【只为避免 UB】:
// 越界索引直接读 samples_[axis] 是越界读, 在诊断仪器里表现为随机数或崩溃,
// 比返回 0 难查得多。返回 0 是"这里没有这个轴"的显式回答。
static void test_out_of_range_axis_is_guarded() {
    TEST(out_of_range_axis_is_guarded);
    JitterStats s;
    const double v[3] = {1.0, 2.0, 3.0};
    s.addFrame(v);
    CHECK(s.mean(-1) == 0.0);
    CHECK(s.mean(3) == 0.0);
    CHECK(s.sd(-1) == 0.0);
    CHECK(s.sd(3) == 0.0);
    CHECK(s.fracAbove(-1, 0.2) == 0.0);
    CHECK(s.fracAbove(3, 0.2) == 0.0);
    CHECK(s.n() == 1);          // 查越界轴不该改动任何状态
    PASS();
}

int main() {
    std::cout << "--- JitterStats (抖动诊断累加器) ---" << std::endl;
    test_constant_sequence_sd_is_exactly_zero();
    test_known_sequence_mean_and_sd();
    test_fracAbove_uses_magnitude_and_includes_boundary();
    test_fracAbove_boundary_just_below_is_excluded();
    test_fracAbove_zero_threshold_counts_zeros();
    test_empty_returns_zero_no_division_by_zero();
    test_single_sample_sd_is_zero();
    test_axes_are_independent();
    test_reset_clears_everything();
    test_out_of_range_axis_is_guarded();
    std::cout << "\nResults: " << g_passed << " passed, " << g_failed << " failed" << std::endl;
    return g_failed == 0 ? 0 : 1;
}
