// Standalone test: PayloadCalibration — 由多姿态空载数据反解末端负载参数
// Build: build_payload_calibration_test.bat
// Run: test_payload_calibration.exe
//
// 思路: 用已知的 (真实负载, 配置负载, 姿态) 正向合成 raw 力/力矩, 再交给 solve() 反解,
//       检查能否把真实值还原回来。

#include <iostream>
#include <cmath>
#include <cstdio>
#include "../force/PayloadCalibration.h"
#include "../calibration/TcpCalibration.h"
#include "../config/Config.h"

static int g_passed = 0, g_failed = 0;

#define TEST(name) do { std::cout << "  " << #name << "... "; } while(0)
#define PASS() do { std::cout << "PASS" << std::endl; g_passed++; } while(0)
#define CHECK(cond) do { if (!(cond)) { std::cout << "FAIL: " << #cond << std::endl; g_failed++; return; } } while(0)

static const double G = 9.81;

// 姿态组: 位置固定 (绕腕转), 姿态互相差异大且含水平/倾斜姿态 (sinθ 足够大)
static const int NP = 6;
static const double g_poses[NP][6] = {
    {300, 100, 40,    0,   0,   0},
    {300, 100, 40,   40,   0,   0},
    {300, 100, 40,  -35,  15,   0},
    {300, 100, 40,    0,  60,  25},
    {300, 100, 40,   25, -50, -30},
    {300, 100, 40,  -20,  35,  55}
};

// 重力在工具系下的表示: g = Rᵀ·(0,0,G)。
//
// 【这里必须从定义展开写，不要"照抄"被测代码的下标。】
// 曾经两边都写成取 R 的第 3 列 (等价于 R·(0,0,G)，与 Rᵀ 差一个转置):
// 单测用错的约定造数据、求解器用同一错约定解回来，于是 10/10 全绿，
// 而实机上把 ~0.40 kg 的工具解成了 0.654 kg —— 转置后仍有相关性，
// 残差 0.68 N 不致命，所以它是"安静地解错"，不会报错。
// 约定必须与 ForceCompensation::step 的 matTransposeMulVec(R, (0,0,9.81)) 一致。
static void gravityTool(const double pose[6], double g[3]) {
    double R[9];
    TcpCalibration::rpyToMatrix(pose[3], pose[4], pose[5], R);
    const double v[3] = {0.0, 0.0, G};
    for (int i = 0; i < 3; i++) {
        g[i] = 0.0;
        // (Rᵀ·v)[i] = Σ_k R[k][i]·v[k]，row-major 下 R[k][i] 是 R[k*3+i]
        for (int k = 0; k < 3; k++) g[i] += R[k * 3 + i] * v[k];
    }
}

// 正向合成: 由真实负载 + 机械臂配置负载 生成 raw 力/力矩 (传感器零偏设 0)
static void synthesize(double mTrue, const double cTrue[3],
                       double mCfg, const double cCfg[3], double signZ,
                       double forces[NP][3], double moments[NP][3])
{
    double cEff[3] = {cCfg[0], cCfg[1], cCfg[2] * signZ};
    double pCfg[3]  = {mCfg * cEff[0] / 1000.0, mCfg * cEff[1] / 1000.0, mCfg * cEff[2] / 1000.0};
    double pTrue[3] = {mTrue * cTrue[0] / 1000.0, mTrue * cTrue[1] / 1000.0, mTrue * cTrue[2] / 1000.0};
    double dp[3] = {pTrue[0] - pCfg[0], pTrue[1] - pCfg[1], pTrue[2] - pCfg[2]};
    double dm = mTrue - mCfg;

    for (int i = 0; i < NP; i++) {
        double g[3];
        gravityTool(g_poses[i], g);
        for (int a = 0; a < 3; a++) forces[i][a] = dm * g[a];
        // M = Δp × g
        moments[i][0] = dp[1] * g[2] - dp[2] * g[1];
        moments[i][1] = dp[2] * g[0] - dp[0] * g[2];
        moments[i][2] = dp[0] * g[1] - dp[1] * g[0];
    }
}

// 基础: 从零配置出发, 能否还原出真实负载
static void test_recovers_true_payload() {
    TEST(recovers_true_payload);
    double cTrue[3] = {0.0, 0.0, 80.4};
    double cCfg[3]  = {0.0, 0.0, 0.0};
    double F[NP][3], M[NP][3];
    synthesize(0.657, cTrue, 0.5, cCfg, +1.0, F, M);

    PayloadCalibration::Result r;
    CHECK(PayloadCalibration::solve(g_poses, F, M, NP, 0.5, cCfg, +1.0, r));
    CHECK(fabs(r.massKg - 0.657) < 1e-6);
    CHECK(fabs(r.comMm[0] - 0.0) < 1e-6);
    CHECK(fabs(r.comMm[1] - 0.0) < 1e-6);
    CHECK(fabs(r.comMm[2] - 80.4) < 1e-6);
    CHECK(fabs(r.dm - 0.157) < 1e-6);
    // 精确合成数据 → 残差应到机器精度 (实测 ~1e-17)。
    // 注意: 残差是 |A·x − b|, 不是数据本身的量级 —— 别把后者当残差 (会掩盖错误解)。
    printf("[rmsF=%.2e rmsM=%.2e] ", r.rmsForceN, r.rmsMomentNm);
    CHECK(r.rmsForceN < 1e-12 && r.rmsMomentNm < 1e-12);
    PASS();
}

// 从非零配置出发 (真实与配置都非零) — 模拟"已经调过一轮, 再微调"
static void test_recovers_from_nonzero_config() {
    TEST(recovers_from_nonzero_config);
    double cTrue[3] = {0.0, 0.0, 80.4};
    double cCfg[3]  = {0.0, 0.0, 60.0};
    double F[NP][3], M[NP][3];
    synthesize(0.657, cTrue, 0.60, cCfg, +1.0, F, M);

    PayloadCalibration::Result r;
    CHECK(PayloadCalibration::solve(g_poses, F, M, NP, 0.60, cCfg, +1.0, r));
    CHECK(fabs(r.massKg - 0.657) < 1e-6);
    CHECK(fabs(r.comMm[2] - 80.4) < 1e-6);
    CHECK(fabs(r.dc[2] - 20.4) < 1e-6);
    PASS();
}

// 符号约定: signZ = -1 时, 仍应还原物理质心, 并把下发值折算回换算约定
static void test_sign_convention() {
    TEST(sign_convention);
    double cTrue[3] = {0.0, 0.0, 80.4};
    double cCfg[3]  = {0.0, 0.0, 80.4};   // 我们下发的值 (原样)
    double F[NP][3], M[NP][3];
    synthesize(0.657, cTrue, 0.657, cCfg, -1.0, F, M);   // 机械臂按 -1 解释

    PayloadCalibration::Result r;
    CHECK(PayloadCalibration::solve(g_poses, F, M, NP, 0.657, cCfg, -1.0, r));
    // 物理质心仍是 +80.4, 但下发时要折算: c_send = c_true / signZ = -80.4
    CHECK(fabs(r.massKg - 0.657) < 1e-6);
    CHECK(fabs(r.comMm[2] - (-80.4)) < 1e-6);
    PASS();
}

// 收敛性: 用解出的值再配置一遍, 残差应归零 (这是"复验"能过的前提)
static void test_second_pass_converges() {
    TEST(second_pass_converges);
    double cTrue[3] = {0.0, 0.0, 80.4};
    double cCfg[3]  = {0.0, 0.0, 0.0};
    double F[NP][3], M[NP][3];
    synthesize(0.657, cTrue, 0.5, cCfg, +1.0, F, M);

    PayloadCalibration::Result r1;
    CHECK(PayloadCalibration::solve(g_poses, F, M, NP, 0.5, cCfg, +1.0, r1));

    // 第二轮: 机械臂现在用 r1 的值, 重新采一轮
    double cNext[3] = {r1.comMm[0], r1.comMm[1], r1.comMm[2]};
    synthesize(0.657, cTrue, r1.massKg, cNext, +1.0, F, M);

    PayloadCalibration::Result r2;
    CHECK(PayloadCalibration::solve(g_poses, F, M, NP, r1.massKg, cNext, +1.0, r2));
    CHECK(fabs(r2.dm) < 1e-6);
    CHECK(fabs(r2.comMm[2] - cNext[2]) < 1e-6);
    PASS();
}

// 噪声: 加入 ~0.05 N / 0.005 N·m 噪声后仍应解到接近真值
static void test_noise_robustness() {
    TEST(noise_robustness);
    double cTrue[3] = {0.0, 0.0, 80.4};
    double cCfg[3]  = {0.0, 0.0, 0.0};
    double F[NP][3], M[NP][3];
    synthesize(0.657, cTrue, 0.5, cCfg, +1.0, F, M);

    unsigned seed = 12345u;
    for (int i = 0; i < NP; i++) {
        for (int a = 0; a < 3; a++) {
            seed = seed * 1103515245u + 12345u;
            double n1 = ((double)((seed >> 16) & 0x7fff) / 32767.0 - 0.5) * 0.10;  // ±0.05 N
            seed = seed * 1103515245u + 12345u;
            double n2 = ((double)((seed >> 16) & 0x7fff) / 32767.0 - 0.5) * 0.01;  // ±0.005 N·m
            F[i][a] += n1;
            M[i][a] += n2;
        }
    }

    PayloadCalibration::Result r;
    CHECK(PayloadCalibration::solve(g_poses, F, M, NP, 0.5, cCfg, +1.0, r));
    CHECK(fabs(r.massKg - 0.657) < 0.02);      // 质量 20 g 以内
    CHECK(fabs(r.comMm[2] - 80.4) < 15.0);     // 质心 15 mm 以内
    PASS();
}

// 姿态数不足
static void test_rejects_too_few_poses() {
    TEST(rejects_too_few_poses);
    double cCfg[3] = {0, 0, 0};
    double F[NP][3], M[NP][3];
    synthesize(0.657, cCfg, 0.5, cCfg, +1.0, F, M);
    PayloadCalibration::Result r;
    CHECK(!PayloadCalibration::solve(g_poses, F, M, 2, 0.5, cCfg, +1.0, r));
    PASS();
}

// 姿态退化: 全部同姿态 → g 全相同 → 差商为 0 → 秩亏
static void test_rejects_degenerate_poses() {
    TEST(rejects_degenerate_poses);
    double same[NP][6];
    for (int i = 0; i < NP; i++)
        for (int a = 0; a < 6; a++) same[i][a] = g_poses[0][a];

    double cTrue[3] = {0, 0, 80.4}, cCfg[3] = {0, 0, 0};
    double F[NP][3], M[NP][3];
    synthesize(0.657, cTrue, 0.5, cCfg, +1.0, F, M);
    PayloadCalibration::Result r;
    CHECK(!PayloadCalibration::solve(same, F, M, NP, 0.5, cCfg, +1.0, r));
    PASS();
}

// 非物理解: 反解出质量 ≤ 0 必须拒绝
static void test_rejects_nonphysical_mass() {
    TEST(rejects_nonphysical_mass);
    double cTrue[3] = {0, 0, 80.4}, cCfg[3] = {0, 0, 0};
    double F[NP][3], M[NP][3];
    // 真实负载质量 0 → 配置 0.5 → 解出 0 kg, 应判非物理
    synthesize(0.0, cTrue, 0.5, cCfg, +1.0, F, M);
    PayloadCalibration::Result r;
    CHECK(!PayloadCalibration::solve(g_poses, F, M, NP, 0.5, cCfg, +1.0, r));
    PASS();
}

// 落盘/读取往返 (含符号约定)
static void test_save_load_roundtrip() {
    TEST(save_load_roundtrip);
    const char* path = "_payload_test.json";
    PayloadCalibration::Result r;
    r.dm = 0.157; r.massKg = 0.657;
    r.dc[0] = 0; r.dc[1] = 0; r.dc[2] = 20.4;
    r.comMm[0] = 0.0; r.comMm[1] = 0.0; r.comMm[2] = 80.4;
    r.rmsForceN = 0.021; r.rmsMomentNm = 0.0013; r.poses = 6;

    PayloadCalibration::applyResult(r);
    PayloadCalibration::flipComSignZ();          // → -1
    CHECK(PayloadCalibration::save(path));

    // 清干净再读回
    PayloadCalibration::enabled = false;
    PayloadCalibration::massKg = 0.0;
    PayloadCalibration::comMm[2] = 0.0;
    PayloadCalibration::comSignZ = 1.0;
    PayloadCalibration::rmsForceN = 0.0;
    PayloadCalibration::poses = 0;

    CHECK(PayloadCalibration::load(path));
    CHECK(PayloadCalibration::enabled);
    CHECK(fabs(PayloadCalibration::massKg - 0.657) < 1e-9);
    CHECK(fabs(PayloadCalibration::comMm[2] - 80.4) < 1e-9);
    CHECK(PayloadCalibration::comSignZ < 0.0);
    CHECK(fabs(PayloadCalibration::rmsForceN - 0.021) < 1e-9);
    CHECK(PayloadCalibration::poses == 6);
    remove(path);
    PASS();
}

// 未标定时 effective() 回退种子值
static void test_effective_falls_back_to_seed() {
    TEST(effective_falls_back_to_seed);
    PayloadCalibration::enabled = false;
    double m, c[3];
    PayloadCalibration::effective(m, c);
    CHECK(fabs(m - Config::ROBOT_PAYLOAD_SEED_KG) < 1e-12);
    CHECK(fabs(c[2] - Config::ROBOT_PAYLOAD_SEED_CZ_MM) < 1e-12);
    PASS();
}

int main() {
    std::cout << "=== PayloadCalibration Tests ===" << std::endl;
    test_recovers_true_payload();
    test_recovers_from_nonzero_config();
    test_sign_convention();
    test_second_pass_converges();
    test_noise_robustness();
    test_rejects_too_few_poses();
    test_rejects_degenerate_poses();
    test_rejects_nonphysical_mass();
    test_save_load_roundtrip();
    test_effective_falls_back_to_seed();
    std::cout << "\n" << g_passed << " passed, " << g_failed << " failed" << std::endl;
    return g_failed ? 1 : 0;
}
