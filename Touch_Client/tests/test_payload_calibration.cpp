// Standalone test: PayloadCalibration — 由多姿态空载数据反解末端负载参数
// Build: build_payload_calibration_test.bat
// Run: test_payload_calibration.exe
//
// 思路: 用已知的 (真实负载, 配置负载, 姿态) 正向合成 raw 力/力矩, 再交给 solve() 反解,
//       检查能否把真实值还原回来。

#include <iostream>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdlib>
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

// 传感器相对法兰的安装偏转角 (度, 绕工具 z) —— 与 Config 里的同一个常量。
static const double PSI_DEG = Config::SENSOR_MOUNT_YAW_DEG;

// 重力在【法兰系】的表示: g0 = Rᵀ·(0,0,G)。
//
// 【这里必须从定义展开写，不要"照抄"被测代码的下标。】
// 曾经两边都写成取 R 的第 3 列 (等价于 R·(0,0,G)，与 Rᵀ 差一个转置):
// 单测用错的约定造数据、求解器用同一错约定解回来，于是 10/10 全绿，
// 而实机上把 ~0.40 kg 的工具解成了 0.654 kg —— 转置后仍有相关性，
// 残差 0.68 N 不致命，所以它是"安静地解错"，不会报错。
static void gravityFlange(const double pose[6], double g[3]) {
    double R[9];
    TcpCalibration::rpyToMatrix(pose[3], pose[4], pose[5], R);
    const double v[3] = {0.0, 0.0, G};
    for (int i = 0; i < 3; i++) {
        g[i] = 0.0;
        // (Rᵀ·v)[i] = Σ_k R[k][i]·v[k]，row-major 下 R[k][i] 是 R[k*3+i]
        for (int k = 0; k < 3; k++) g[i] += R[k * 3 + i] * v[k];
    }
}

// 重力在【传感器系】的表示 —— 【独立推导, 不调用被测函数】。
// 约定: 传感器系 = 法兰系绕 z 转 +psi ⇒ 同一矢量在传感器系里的坐标 = Rz(-psi)·(法兰系坐标):
//     Rz(-psi) = [[ cos, sin, 0], [-sin, cos, 0], [0, 0, 1]]
// 记法转置写反的话, 造出来的数据与求解器方向相反 —— 这个测试会立刻红 (残差 0.7 N 量级)。
static void gravitySensorRef(const double pose[6], double g[3]) {
    double g0[3];
    gravityFlange(pose, g0);
    const double D2R = 3.14159265358979323846 / 180.0;
    const double p = PSI_DEG * D2R;
    const double c = cos(p), s = sin(p);
    g[0] =  c * g0[0] + s * g0[1];
    g[1] = -s * g0[0] + c * g0[1];
    g[2] =  g0[2];
}

typedef void (*GravityFn)(const double pose[6], double g[3]);

// 正向合成: 由真实负载 + 机械臂配置负载 生成 raw 力/力矩 (传感器零偏设 0)。
// grav 决定用哪个重力模型造数据 —— 正向对照传 gravitySensorRef, 反向对照传 gravityFlange。
static void synthesizeWith(GravityFn grav, double mTrue, const double cTrue[3],
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
        grav(g_poses[i], g);
        for (int a = 0; a < 3; a++) forces[i][a] = dm * g[a];
        // M = Δp × g
        moments[i][0] = dp[1] * g[2] - dp[2] * g[1];
        moments[i][1] = dp[2] * g[0] - dp[0] * g[2];
        moments[i][2] = dp[0] * g[1] - dp[1] * g[0];
    }
}

// 默认 (物理真值) 合成: 传感器自己那一系的模型 —— 含安装偏转角。
static void synthesize(double mTrue, const double cTrue[3],
                       double mCfg, const double cCfg[3], double signZ,
                       double forces[NP][3], double moments[NP][3])
{
    synthesizeWith(gravitySensorRef, mTrue, cTrue, mCfg, cCfg, signZ, forces, moments);
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

// 符号的两种解释算出的【物理】质心横跨法兰平面: 一个在下方 (物理), 一个在上方 (非物理)。
// 这正是"数据定不了符号"的由来 —— 从前靠实机探针裁决, 探针已废除 (符号不再被任何判据或
// 下发依赖, 见 PayloadCalibration.h)。传入的 signZ 只影响 comMm 的折算, 左右不了 cTrueZ。
static void test_candidates_bracket_flange() {
    TEST(candidates_bracket_flange);
    // 配置 0.660kg / com +80.4mm; 真值 0.409kg / com +67.9mm
    double cTrue[3] = {0.3, 0.3, 67.9};
    double cCfg[3]  = {0.0, 0.0, 80.4};
    double F[NP][3], M[NP][3];
    synthesize(0.409, cTrue, 0.660, cCfg, +1.0, F, M);   // 机械臂按 +1 解释

    PayloadCalibration::Result r;
    // 故意传入【错误】的 signZ=-1: 解算器不据此纠正什么, 只按它折算 comMm
    CHECK(PayloadCalibration::solve(g_poses, F, M, NP, 0.660, cCfg, -1.0, r));
    CHECK(r.cTrueZ[0] > 0.0);      // +1 解释: 法兰下方, 物理
    CHECK(r.cTrueZ[1] < 0.0);      // -1 解释: 法兰上方, 非物理
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

    // comSignZ 是【持久化的显示约定】, 不再是求解结果的一部分: applyResult() 不碰它
    // (从前它被求解器/探针"定案", 现在没人定这个案了), 所以这里直接设生效值。
    PayloadCalibration::comSignZ = -1.0;
    PayloadCalibration::applyResult(r);
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

// 落盘必须带 saved_at_unix —— 没有这个字段的文件会被 CalibStore 判为过期。
static void test_save_includes_timestamp() {
    TEST(save_includes_timestamp);
    const char* path = "test_payload_ts.json";
    PayloadCalibration::enabled = true;
    PayloadCalibration::massKg = 0.5;
    PayloadCalibration::comMm[0] = 0.0;
    PayloadCalibration::comMm[1] = 0.0;
    PayloadCalibration::comMm[2] = 80.0;
    CHECK(PayloadCalibration::save(path));

    FILE* f = fopen(path, "r");
    CHECK(f != nullptr);
    char buf[2048];
    size_t n = fread(buf, 1, sizeof(buf) - 1, f);
    fclose(f);
    buf[n] = '\0';
    // 字段必须存在, 且是 plausible 的 Unix 秒 (> 2026-01-01)
    CHECK(strstr(buf, "\"saved_at_unix\"") != nullptr);
    const char* p = strstr(buf, "\"saved_at_unix\"");
    const char* colon = strchr(p, ':');
    double ts = strtod(colon + 1, nullptr);
    CHECK(ts > 1767225600.0);
    CHECK(strstr(buf, "\"version\": 2") != nullptr);
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

// 符号约定只影响报出的【绝对】读数, 不影响拟合结果 —— 这现在是本模块赖以成立的性质:
// 本地补偿只用 dm/dp, 而它们与传入的 signZ 无关 (符号不进 buildRows)。
// (取代了原来的 both_candidates_computed: 它测的是"两个候选的下发值", 候选机制已废除。)
static void test_fit_is_sign_independent() {
    TEST(fit_is_sign_independent);
    double cTrue[3] = {0.3, 0.3, 67.9};
    double cCfg[3]  = {0.0, 0.0, 80.4};
    double F[NP][3], M[NP][3];
    synthesize(0.409, cTrue, 0.660, cCfg, +1.0, F, M);

    PayloadCalibration::Result rA, rB;
    CHECK(PayloadCalibration::solve(g_poses, F, M, NP, 0.660, cCfg, +1.0, rA));
    CHECK(PayloadCalibration::solve(g_poses, F, M, NP, 0.660, cCfg, -1.0, rB));

    // 本地补偿要用的量: 两种约定下必须逐位相同
    CHECK(fabs(rA.dm - rB.dm) < 1e-12);
    CHECK(fabs(rA.massKg - rB.massKg) < 1e-12);
    CHECK(fabs(rA.dp[0] - rB.dp[0]) < 1e-12);
    CHECK(fabs(rA.dp[2] - rB.dp[2]) < 1e-12);
    CHECK(fabs(rA.rmsForceN - rB.rmsForceN) < 1e-12);
    // 只有【报出的绝对质心】跟着约定变: X/Y 与符号无关, Z 不同
    CHECK(fabs(rA.comMm[0] - rB.comMm[0]) < 1e-12);
    CHECK(fabs(rA.comMm[1] - rB.comMm[1]) < 1e-12);
    CHECK(fabs(rA.comMm[2] - rB.comMm[2]) > 1.0);
    PASS();
}

// 重力约定只能有一份, 且必须对得上【手算】—— 光"两条路径相等"不够, 它们可能一起错
// (历史上就一起错过: 两边都写成 R·(0,0,G), 10/10 全绿而实机解错)。
static void test_gravity_sensor_frame_convention() {
    TEST(gravity_sensor_frame_convention);

    // (a) 公共路径 (TcpCalibration::gravitySensorFrame) 与测试自己展开的模型逐姿态一致
    for (int i = 0; i < NP; i++) {
        double ref[3], got[3];
        gravitySensorRef(g_poses[i], ref);
        TcpCalibration::gravitySensorFrame(g_poses[i], got);
        CHECK(fabs(ref[0] - got[0]) < 1e-12);
        CHECK(fabs(ref[1] - got[1]) < 1e-12);
        CHECK(fabs(ref[2] - got[2]) < 1e-12);
    }

    // 纯旋转不改变模长 —— 任何姿态下都必须是 9.81
    double pose[6] = {300.0, 100.0, 40.0, 172.4, 65.0, 138.7};
    double got[3];
    TcpCalibration::gravitySensorFrame(pose, got);
    CHECK(fabs(sqrt(got[0]*got[0] + got[1]*got[1] + got[2]*got[2]) - G) < 1e-9);

    // (b) 手算参考 (仅对 psi = +90° 成立, 所以扫 psi 时自动跳过):
    //     RPY = (172.4, 65.0, 138.7)° ⇒ R = Rz·Ry·Rx, 只有第三行用得上:
    //       R[6] = -sin(ry)          = -0.90630779
    //       R[7] =  cos(ry)·sin(rx)  =  0.05589397
    //       R[8] =  cos(ry)·cos(rx)  = -0.41890579
    //     g0 (旧模型, 法兰系) = 9.81·(R[6],R[7],R[8])
    //                         = (-8.89087939, +0.54831980, -4.10946579)
    //     psi = +90° ⇒ Rz(-90°) 把 (x,y) 映成 (y,-x):
    //     g_sensor (新模型)    = (+0.54831980, +8.89087939, -4.10946579)
    if (fabs(PSI_DEG - 90.0) < 1e-9) {
        CHECK(fabs(got[0] - (+0.54831980)) < 1e-7);
        CHECK(fabs(got[1] - (+8.89087939)) < 1e-7);
        CHECK(fabs(got[2] - (-4.10946579)) < 1e-7);
    } else {
        std::cout << "(手算参考只针对 psi=90, 当前 psi=" << PSI_DEG << " 跳过) ";
    }
    PASS();
}

// 两条公共路径必须落在同一个重力约定上。
// 这里够得着的两条是: 负载求解 (PayloadCalibration::solve, 内部经由本文件的 gravityTool)
// 与共享函数 (TcpCalibration::gravitySensorFrame); ForceCompensation::step 需要 AppState
// 且未链进本可执行文件, 它那一路只由"step 已改成调用同一个函数"这一事实保证。
// 测法: 用【测试自己展开的】传感器系模型造数据 → 求解器若同约定, 残差应到机器精度。
static void test_solver_uses_shared_gravity_convention() {
    TEST(solver_uses_shared_gravity_convention);
    double cTrue[3] = {0.0, 0.0, 80.4};
    double cCfg[3]  = {0.0, 0.0, 0.0};
    double F[NP][3], M[NP][3];
    synthesize(0.657, cTrue, 0.5, cCfg, +1.0, F, M);   // 数据里已含 psi

    PayloadCalibration::Result r;
    CHECK(PayloadCalibration::solve(g_poses, F, M, NP, 0.5, cCfg, +1.0, r));
    CHECK(fabs(r.massKg - 0.657) < 1e-9);
    printf("[rmsF=%.2e rmsM=%.2e] ", r.rmsForceN, r.rmsMomentNm);
    CHECK(r.rmsForceN < 1e-9 && r.rmsMomentNm < 1e-9);
    PASS();
}

// 反向对照: 造数据时【故意不转】这个 yaw (即改之前的模型) → 带 yaw 的求解器就该明显解不上。
// 这证明 yaw 真的进了模型, 而不是被写成恒等 / 在别处被约掉。
// 历史教训: 单测用错约定造数据 + 求解器用同一错约定解回来 = 全绿而实机错。
static void test_stale_model_no_longer_fits() {
    TEST(stale_model_no_longer_fits);
    if (fabs(PSI_DEG) < 1.0) { std::cout << "SKIP(psi≈0, 无区分度) "; PASS(); return; }

    double cTrue[3] = {0.0, 0.0, 80.4};
    double cCfg[3]  = {0.0, 0.0, 0.0};
    double F[NP][3], M[NP][3];
    // 旧模型 (psi=0) 造数据 —— 实机 2026-09-18 那组数据的写照
    synthesizeWith(gravityFlange, 0.657, cTrue, 0.5, cCfg, +1.0, F, M);

    PayloadCalibration::Result r;
    bool ok = PayloadCalibration::solve(g_poses, F, M, NP, 0.5, cCfg, +1.0, r);
    if (ok) {
        printf("[旧模型数据 → mass=%.4f kg, rmsF=%.4f N] ", r.massKg, r.rmsForceN);
        CHECK(r.rmsForceN > 0.1);                  // 量级 0.70 N —— 与实机那 0.73 N 同源
        CHECK(fabs(r.massKg - 0.657) > 0.05);      // 质量也解错了, 不是"差不多能用"
    } else {
        std::cout << "[旧模型数据被直接拒绝] ";
    }
    PASS();
}

int main() {
    std::cout << "=== PayloadCalibration Tests ===" << std::endl;
    test_recovers_true_payload();
    test_recovers_from_nonzero_config();
    test_sign_convention();
    test_candidates_bracket_flange();
    test_second_pass_converges();
    test_noise_robustness();
    test_rejects_too_few_poses();
    test_rejects_degenerate_poses();
    test_rejects_nonphysical_mass();
    test_save_load_roundtrip();
    test_save_includes_timestamp();
    test_effective_falls_back_to_seed();
    test_fit_is_sign_independent();
    test_gravity_sensor_frame_convention();
    test_solver_uses_shared_gravity_convention();
    test_stale_model_no_longer_fits();
    std::cout << "\n" << g_passed << " passed, " << g_failed << " failed" << std::endl;
    return g_failed ? 1 : 0;
}
