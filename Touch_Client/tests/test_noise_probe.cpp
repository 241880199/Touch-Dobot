// Standalone test: NoiseProbe 的统计量 (块平均 / 自相关 / 均值-标准差)
// Build: build_noise_probe_test.bat   Run: test_noise_probe.exe
//
// 【这个测试为什么存在】见 force/NoiseProbe.h 顶上那一段。一句话:
//   块平均这条曲线是"该不该把流水线挪到帧率上跑"的【唯一判据】, 而它的错法很安静 ——
//   一个写错的块平均会给出"平均没用"的结论, 把人引到错误的修法上。
//   ⇒ 拿【构造信号】钉死两个端点: 白噪声必须按 1/sqrt(k) 掉; 慢带限信号必须不掉。

#define _USE_MATH_DEFINES          // MSVC 的 <cmath> 要这个宏才给 M_PI (与 ForcePipeline.cpp 同做法)
#include <iostream>
#include <cmath>
#include "../force/NoiseProbe.h"

static int g_passed = 0, g_failed = 0;

#define TEST(name) do { std::cout << "  " << #name << "... "; } while(0)
#define PASS() do { std::cout << "PASS" << std::endl; g_passed++; } while(0)
#define CHECK(cond) do { if (!(cond)) { std::cout << "FAIL: " << #cond << std::endl; g_failed++; return; } } while(0)

// ===== 确定性伪随机 (不依赖平台/库版本) =====
// LCG (Knuth 的 MMIX 常数): 只要常数不变, 每次跑出来的序列一模一样。
static unsigned long long g_seed = 20260921ULL;
static void seedReset(unsigned long long s) { g_seed = s; }
static double nextGaussish() {
    // 6 个均匀数之和 − 3: 均值 0、sd = sqrt(6/12) = 0.7071 (Irwin–Hall 近似正态)。
    // 用它而不是 Box-Muller: 没有 log/cos, 也没有 0 附近的对数奇点。
    g_seed = g_seed * 6364136223846793005ULL + 1442695040888963407ULL;
    double s = 0.0;
    for (int i = 0; i < 6; i++) {
        g_seed = g_seed * 6364136223846793005ULL + 1442695040888963407ULL;
        s += (double)((g_seed >> 11) & 0x1FFFFFFFFFFFFFULL) / (double)0x20000000000000ULL;
    }
    return s - 3.0;
}

// ===== 1. 均值/标准差: 已知答案 =====
static void test_mean_sd_known() {
    TEST(mean_sd_known);
    const double x[5] = {0.0, 1.0, 2.0, 3.0, 4.0};
    double m = 0.0, sd = 0.0;
    NoiseProbe::meanSd(x, 5, &m, &sd);
    CHECK(fabs(m - 2.0) < 1e-12);
    CHECK(fabs(sd - sqrt(2.5)) < 1e-12);      // 样本 sd: Σ(x−m)²/(n−1) = 10/4 = 2.5
    // n < 2 ⇒ 无方差可言, 报 0 而不是 NaN (调用方会拿它做除法/比值)
    NoiseProbe::meanSd(x, 1, &m, &sd);
    CHECK(sd == 0.0);
    NoiseProbe::meanSd(x, 0, &m, &sd);
    CHECK(m == 0.0 && sd == 0.0);
    PASS();
}

// ===== 2. 只用完整块 =====
// 100 个样本、k=16 ⇒ 6 个完整块 (96 个样本), 余 4 个必须【丢掉】。
// 若谁把尾块也算进去 (k 不足也平均), 那条块的方差会偏小 ⇒ 报出来的 sd 偏低 ⇒ 判决偏向"有用"。
static void test_complete_blocks_only() {
    TEST(complete_blocks_only);
    double x[100];
    for (int i = 0; i < 100; i++) x[i] = (double)(i % 7);
    int nb = -1;
    double sd = 0.0;
    NoiseProbe::blockSd(x, 100, 16, &sd, &nb);
    CHECK(nb == 6);
    // 块数 < 2 ⇒ sd 必须是 0 (2 个点也算不出方差), 而且不能除零
    NoiseProbe::blockSd(x, 20, 16, &sd, &nb);
    CHECK(nb == 1);
    CHECK(sd == 0.0);
    PASS();
}

// ===== 3. ★ 白噪声必须按 1/sqrt(k) 掉 =====
// 【这条是探针判决的"有用"那一侧】: 独立样本 ⇒ 平均 k 个把 sd 压到 1/sqrt(k)。
// 容差 15%: nb=512 个块, sd 的相对标准误约 1/sqrt(2·511) ≈ 3%, 15% 是宽裕的余量。
static void test_white_falls_as_inverse_sqrt_k() {
    TEST(white_falls_as_inverse_sqrt_k);
    const int N = 8192;
    static double x[N];
    seedReset(20260921ULL);
    for (int i = 0; i < N; i++) x[i] = nextGaussish();

    double m = 0.0, sdRaw = 0.0;
    NoiseProbe::meanSd(x, N, &m, &sdRaw);
    CHECK(fabs(m) < 0.05);                       // 构造信号的均值确实是 0
    CHECK(fabs(sdRaw - sqrt(0.5)) < 0.05);       // 且 sd 确实是 0.7071

    const int ks[4] = {2, 8, 16, 32};
    for (int j = 0; j < 4; j++) {
        const int k = ks[j];
        double sdBlock = 0.0;
        int nb = 0;
        NoiseProbe::blockSd(x, N, k, &sdBlock, &nb);
        CHECK(nb == N / k);
        const double ratio = sdBlock / sdRaw;
        const double ideal = NoiseProbe::idealBlockRatio(k);
        CHECK(fabs(ratio / ideal - 1.0) < 0.15);
    }
    PASS();
}

// ===== 4. ★ 慢带限信号必须【不】掉 =====
// 【这条是判决的"没用"那一侧】: 相关时间 ≫ k 个采样时, 块平均几乎不改变 sd。
// 若谁把 blockSd 写成"相邻样本作差"之类的花样, 这条会立刻红 (那种写法对慢信号会把它抹成 0)。
static void test_slow_band_limited_does_not_fall() {
    TEST(slow_band_limited_does_not_fall);
    const int N = 8192;
    static double x[N];
    // 周期 1000 个采样: 在 k=16 的尺度上几乎是常数 ⇒ 块均值 = 块中心的值 ⇒ 比值 ≈ 1
    for (int i = 0; i < N; i++) x[i] = 4.6 + 0.3 * sin(2.0 * M_PI * (double)i / 1000.0);

    double m = 0.0, sdRaw = 0.0;
    NoiseProbe::meanSd(x, N, &m, &sdRaw);
    CHECK(sdRaw > 0.2);                          // 确实有起伏

    double sdBlock = 0.0;
    int nb = 0;
    NoiseProbe::blockSd(x, N, 16, &sdBlock, &nb);
    const double ratio = sdBlock / sdRaw;
    // 块均值跨了 8192−16 个样本, 只比原始序列少最后 16 个 ⇒ 比值应当接近 1
    CHECK(ratio > 0.9);
    CHECK(ratio < 1.05);
    PASS();
}

// ===== 5. 自相关: 白噪声低、慢信号高 =====
static void test_autocorr_distinguishes() {
    TEST(autocorr_distinguishes);
    const int N = 4096;
    static double w[N], s[N];
    seedReset(777ULL);
    for (int i = 0; i < N; i++) w[i] = nextGaussish();
    for (int i = 0; i < N; i++) s[i] = sin(2.0 * M_PI * (double)i / 1000.0);

    // 白噪声 lag=1: 理论 0 (标准误 ~1/sqrt(N) = 1.6%, 0.1 是宽裕上界)
    CHECK(fabs(NoiseProbe::autocorr(w, N, 1)) < 0.1);
    // 慢信号 lag=1: 周期 1000 ⇒ r = cos(2π/1000) ≈ 0.99998
    CHECK(NoiseProbe::autocorr(s, N, 1) > 0.99);
    // 退化输入不许出 NaN (调用方会直接打印它)
    CHECK(NoiseProbe::autocorr(s, N, 0) == 0.0);
    CHECK(NoiseProbe::autocorr(s, N, N) == 0.0);
    CHECK(NoiseProbe::autocorr(s, 1, 1) == 0.0);
    PASS();
}

int main() {
    std::cout << "=== NoiseProbe Unit Tests ===" << std::endl;
    test_mean_sd_known();
    test_complete_blocks_only();
    test_white_falls_as_inverse_sqrt_k();
    test_slow_band_limited_does_not_fall();
    test_autocorr_distinguishes();
    std::cout << "\nResults: " << g_passed << " passed, " << g_failed << " failed" << std::endl;
    return g_failed ? 1 : 0;
}
