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

// 传感器相对法兰的安装偏转角 (度, 绕工具 z)。
// ⚠ 【测试自己声明 ψ, 不再"因为 Config 写了这个数就用这个数"】: ψ 现在是求解器的输出,
//   若生成端与估计端都读同一个常量, 这个测试就退化成"求解器同意 Config"。
//   下面这个默认值只用于"历史行为"的回归用例; 每个需要说明 ψ 的用例都显式传自己的值。
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

// 重力在【传感器系】的表示 —— 【独立推导, 不调用被测函数】, ψ 由调用方给出。
// 约定: 传感器系 = 法兰系绕 z 转 +psi ⇒ 同一矢量在传感器系里的坐标 = Rz(-psi)·(法兰系坐标):
//     Rz(-psi) = [[ cos, sin, 0], [-sin, cos, 0], [0, 0, 1]]
// 记法转置写反的话, 造出来的数据与求解器方向相反 —— 这个测试会立刻红 (残差 0.7 N 量级)。
//
// 【生成器与估计器【不共用代码】, 这是刻意的】: 这一份是从定义手写展开的, 估计器那一份在
// TcpCalibration/PayloadCalibration 里。本仓库已经两次被"测试与实现同错"咬到 (两边都写
// 成取 R 第三列、10/10 全绿而实机解错) —— 谁把这里改成调用被测函数, 就是第三次。
static void gravitySensorRefAt(const double pose[6], double psiDeg, double g[3]) {
    double g0[3];
    gravityFlange(pose, g0);
    const double D2R = 3.14159265358979323846 / 180.0;
    const double p = psiDeg * D2R;
    const double c = cos(p), s = sin(p);
    g[0] =  c * g0[0] + s * g0[1];
    g[1] = -s * g0[0] + c * g0[1];
    g[2] =  g0[2];
}

// 旧模型 (psi 恒等): 只用于"反向对照" —— 造出一批【没有任何 psi 能解释】的数据。
static void gravityFlangeAt(const double pose[6], double psiDeg, double g[3]) {
    (void)psiDeg;                 // 旧模型不认识 psi
    gravityFlange(pose, g);
}

// 转置了的约定 (取 R 第三列 = R·(0,0,G)) —— 任何 ψ 都吸收不了这种错, 见下方反向对照用例。
static void gravityTransposedAt(const double pose[6], double psiDeg, double g[3]) {
    (void)psiDeg;
    double R[9];
    TcpCalibration::rpyToMatrix(pose[3], pose[4], pose[5], R);
    for (int i = 0; i < 3; i++) g[i] = R[i * 3 + 2] * G;   // 第三【列】, 即 R·(0,0,G)
}

typedef void (*GravityFnAt)(const double pose[6], double psiDeg, double g[3]);

// 正向合成: 由真实负载 + 机械臂配置负载 生成 raw 力/力矩 (传感器零偏设 0)。
// psiDeg 显式给出 —— "造数据时用了哪个安装角"必须写在调用点上, 不能藏在常量里。
// grav 决定用哪个重力模型造数据 —— 正向对照传 gravitySensorRefAt, 反向对照传
// gravityFlangeAt / gravityTransposedAt。
static void synthesizeWith(GravityFnAt grav, double psiDeg, double mTrue, const double cTrue[3],
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
        grav(g_poses[i], psiDeg, g);
        for (int a = 0; a < 3; a++) forces[i][a] = dm * g[a];
        // M = Δp × g
        moments[i][0] = dp[1] * g[2] - dp[2] * g[1];
        moments[i][1] = dp[2] * g[0] - dp[0] * g[2];
        moments[i][2] = dp[0] * g[1] - dp[1] * g[0];
    }
}

// 物理真值合成: 传感器自己那一系的模型 —— 含安装偏转角 psiDeg。
static void synthesize(double psiDeg, double mTrue, const double cTrue[3],
                       double mCfg, const double cCfg[3], double signZ,
                       double forces[NP][3], double moments[NP][3])
{
    synthesizeWith(gravitySensorRefAt, psiDeg, mTrue, cTrue, mCfg, cCfg, signZ, forces, moments);
}

// 基础: 从零配置出发, 能否还原出真实负载
static void test_recovers_true_payload() {
    TEST(recovers_true_payload);
    double cTrue[3] = {0.0, 0.0, 80.4};
    double cCfg[3]  = {0.0, 0.0, 0.0};
    double F[NP][3], M[NP][3];
    synthesize(PSI_DEG, 0.657, cTrue, 0.5, cCfg, +1.0, F, M);

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
    synthesize(PSI_DEG, 0.657, cTrue, 0.60, cCfg, +1.0, F, M);

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
    synthesize(PSI_DEG, 0.657, cTrue, 0.657, cCfg, -1.0, F, M);   // 机械臂按 -1 解释

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
    synthesize(PSI_DEG, 0.409, cTrue, 0.660, cCfg, +1.0, F, M);   // 机械臂按 +1 解释

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
    synthesize(PSI_DEG, 0.657, cTrue, 0.5, cCfg, +1.0, F, M);

    PayloadCalibration::Result r1;
    CHECK(PayloadCalibration::solve(g_poses, F, M, NP, 0.5, cCfg, +1.0, r1));

    // 第二轮: 机械臂现在用 r1 的值, 重新采一轮
    double cNext[3] = {r1.comMm[0], r1.comMm[1], r1.comMm[2]};
    synthesize(PSI_DEG, 0.657, cTrue, r1.massKg, cNext, +1.0, F, M);

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
    synthesize(PSI_DEG, 0.657, cTrue, 0.5, cCfg, +1.0, F, M);

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
    synthesize(PSI_DEG, 0.657, cCfg, 0.5, cCfg, +1.0, F, M);
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
    synthesize(PSI_DEG, 0.657, cTrue, 0.5, cCfg, +1.0, F, M);
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
    synthesize(PSI_DEG, 0.0, cTrue, 0.5, cCfg, +1.0, F, M);
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
    r.sensorYawDeg = 79.5;      // 解出的 psi 必须随文件往返 —— 它是模型的一半

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
    PayloadCalibration::sensorYawDeg = -123.0;

    CHECK(PayloadCalibration::load(path));
    CHECK(PayloadCalibration::enabled);
    CHECK(fabs(PayloadCalibration::massKg - 0.657) < 1e-9);
    CHECK(fabs(PayloadCalibration::comMm[2] - 80.4) < 1e-9);
    CHECK(PayloadCalibration::comSignZ < 0.0);
    CHECK(fabs(PayloadCalibration::rmsForceN - 0.021) < 1e-9);
    CHECK(PayloadCalibration::poses == 6);
    CHECK(fabs(PayloadCalibration::sensorYawDeg - 79.5) < 1e-9);
    remove(path);
    PASS();
}

// 旧文件 (本字段出现之前存的 payload_calib.json) 【必须仍能加载】, psi 回退 Config 种子。
// 这是升级路径的硬要求: 若缺字段就拒载, 用户现存的标定会集体作废、机器人拿不到负载参数。
static void test_load_old_file_without_yaw() {
    TEST(load_old_file_without_yaw);
    const char* path = "test_payload_old.json";
    FILE* f = fopen(path, "w");
    CHECK(f != nullptr);
    // 逐字写出"旧版本"的文件内容 (没有 sensor_yaw_deg 这一行)
    fprintf(f, "{\n  \"version\": 2,\n  \"saved_at_unix\": 1767225601,\n"
               "  \"mass_kg\": 0.657,\n  \"com_mm\": [0, 0, 80.4],\n"
               "  \"rms_force_n\": 0.021,\n  \"rms_moment_nm\": 0.0013,\n"
               "  \"poses\": 6,\n  \"com_sign_z\": 1.0\n}\n");
    fclose(f);

    PayloadCalibration::sensorYawDeg = -123.0;   // 先弄脏, 确认它是真的被读/被回退
    CHECK(PayloadCalibration::load(path));
    CHECK(fabs(PayloadCalibration::sensorYawDeg - Config::SENSOR_MOUNT_YAW_DEG) < 1e-12);
    CHECK(fabs(PayloadCalibration::massKg - 0.657) < 1e-9);   // 其余字段照常读进来
    CHECK(PayloadCalibration::enabled);
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
    synthesize(PSI_DEG, 0.409, cTrue, 0.660, cCfg, +1.0, F, M);

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

    // (a) 公共路径 (TcpCalibration::gravitySensorFrame, 走【模块状态】里的 psi) 与测试自己
    //     展开的模型 (psi 显式给 PSI_DEG) 逐姿态一致。
    //     这一步同时断言了: 没有哪个用例把模块状态留在了别的角度上 (setSensorYawDeg 的用例
    //     必须还原) —— 否则这里会红, 而不是让后面的用例悄悄跑在错的 psi 下。
    CHECK(fabs(TcpCalibration::sensorYawDeg() - PSI_DEG) < 1e-12);
    for (int i = 0; i < NP; i++) {
        double ref[3], got[3];
        gravitySensorRefAt(g_poses[i], PSI_DEG, ref);
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

// psi 的模块状态: setter/getter 生效, 且 gravitySensorFrame 真的按它转 (与"显式给 psi"的
// 路径逐位相同)。标定后的 psi 就是经这条路进重力模型的 —— 对不上等于标定白做。
static void test_sensor_yaw_state() {
    TEST(sensor_yaw_state);
    CHECK(fabs(TcpCalibration::sensorYawDeg() - PSI_DEG) < 1e-12);   // 未标定时 = Config 种子

    TcpCalibration::setSensorYawDeg(80.0);                           // 模拟标定解出 80°
    CHECK(fabs(TcpCalibration::sensorYawDeg() - 80.0) < 1e-12);
    for (int i = 0; i < NP; i++) {
        double viaState[3], explicitPsi[3], handRef[3];
        TcpCalibration::gravitySensorFrame(g_poses[i], viaState);            // 走模块状态
        TcpCalibration::gravitySensorFrameAtYaw(g_poses[i], 80.0, explicitPsi);
        gravitySensorRefAt(g_poses[i], 80.0, handRef);                       // 测试自己的推导
        CHECK(fabs(viaState[0] - explicitPsi[0]) < 1e-12);
        CHECK(fabs(viaState[0] - handRef[0]) < 1e-12);
        CHECK(fabs(viaState[1] - explicitPsi[1]) < 1e-12);
        CHECK(fabs(viaState[1] - handRef[1]) < 1e-12);
        CHECK(fabs(viaState[2] - explicitPsi[2]) < 1e-12);
        CHECK(fabs(viaState[2] - handRef[2]) < 1e-12);
    }
    // 还回种子值 —— 模块状态是全局的, 留着 80° 会让别的用例跑在另一个模型上。
    TcpCalibration::setSensorYawDeg(PSI_DEG);
    CHECK(fabs(TcpCalibration::sensorYawDeg() - PSI_DEG) < 1e-12);
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
    synthesize(PSI_DEG, 0.657, cTrue, 0.5, cCfg, +1.0, F, M);   // 数据里已含 psi

    PayloadCalibration::Result r;
    CHECK(PayloadCalibration::solve(g_poses, F, M, NP, 0.5, cCfg, +1.0, r));
    CHECK(fabs(r.massKg - 0.657) < 1e-9);
    printf("[rmsF=%.2e rmsM=%.2e] ", r.rmsForceN, r.rmsMomentNm);
    CHECK(r.rmsForceN < 1e-9 && r.rmsMomentNm < 1e-9);
    PASS();
}

// ψ 的可解性 —— 【这就是"扫描真的在估 ψ"的证明】。没有它, 扫描是未验证的。
// 数据用【已知的】ψ 合成 (生成器是上面那份独立展开的推导, 不调用被测函数), 求解器应把它
// 解回来: 误差不超过扫描分辨率的一半 (0.25°), 残差落到机器精度, 质量/质心也跟着解对。
// 同一个函数跑两个不同的 ψ —— 只测一个值说明不了"不是被调成那个答案的"。
static void test_recovers_sensor_yaw(double psiTrue) {
    std::cout << "  recovers_sensor_yaw(psi_true=" << psiTrue << "deg)... ";
    double cTrue[3] = {0.0, 0.0, 80.4};
    double cCfg[3]  = {0.0, 0.0, 0.0};
    double F[NP][3], M[NP][3];
    synthesize(psiTrue, 0.657, cTrue, 0.5, cCfg, +1.0, F, M);

    PayloadCalibration::Result r;
    CHECK(PayloadCalibration::solve(g_poses, F, M, NP, 0.5, cCfg, +1.0, r));
    printf("[psi=%.1f (err=%.2f), mass=%.5f (err=%.1e), comZ=%.4f, rmsF=%.2e] ",
           r.sensorYawDeg, fabs(r.sensorYawDeg - psiTrue), r.massKg,
           fabs(r.massKg - 0.657), r.comMm[2], r.rmsForceN);
    // 网格是 0.5°, 所以真值到最近网格点最多差 0.25° —— 这就是"扫描分辨率内"的严格写法。
    CHECK(fabs(r.sensorYawDeg - psiTrue) <= 0.25 + 1e-9);
    // 残差: 真值正落在网格上时是机器精度 (实测 ~2e-16); 落在网格【之间】时最优网格点差半个
    // 步长, 模型对不上, 残差抬到 ~3e-3 N 量级 (差 0.25° 的一阶效应)。两者都远在 0.10 N 的
    // "好拟合"线和 0.30 N 的门限之下 —— 一个阈值就能同时覆盖, 不必按用例分档。
    CHECK(r.rmsForceN < 0.05);
    // 质量/质心: 网格上的真值精确还原 (误差 0); 网格之间的真值差半个步长, 误差抬到
    // ~1e-6 kg / ~2e-3 mm —— 仍然远小于噪声用例的容差 (20 g / 15 mm)。
    CHECK(fabs(r.massKg - 0.657) < 1e-4);
    CHECK(fabs(r.comMm[2] - 80.4) < 0.1);
    PASS();
}

// 反向对照: 造数据时用【转置了的】重力约定 (取 R 的第三列 = R·(0,0,G), 历史上就错在这里)。
// 这不是任何一个 ψ 能修好的错 —— ψ 只能吸收绕 z 的偏转, 吸收不了转置。扫描必须照样解不上,
// 否则说明"扫描"变成了万能挡箭牌: 什么错的数据都能被它拟合到门限以内。
// 历史教训: 单测用错约定造数据 + 求解器用同一错约定解回来 = 全绿而实机错。
static void test_scan_cannot_fix_transposed_gravity() {
    TEST(scan_cannot_fix_transposed_gravity);
    double cTrue[3] = {0.0, 0.0, 80.4};
    double cCfg[3]  = {0.0, 0.0, 0.0};
    double F[NP][3], M[NP][3];
    synthesizeWith(gravityTransposedAt, 0.0, 0.657, cTrue, 0.5, cCfg, +1.0, F, M);

    PayloadCalibration::Result r;
    bool ok = PayloadCalibration::solve(g_poses, F, M, NP, 0.5, cCfg, +1.0, r);
    if (ok) {
        printf("[转置数据 → psi=%+.1f, mass=%.4f kg, rmsF=%.4f N] ",
               r.sensorYawDeg, r.massKg, r.rmsForceN);
        // ψ 扫描能把转置误差从 0.70 N (固定 ψ=90 时) 压到 ~0.28 N —— 但压不到噪声本底,
        // 差着十几个数量级。这就是本用例要守的: 扫描不是万能挡箭牌。
        // ⚠ 0.28 N 已经【贴着】0.30 N 的验收门限 (见报告): 转置这种错现在只勉强拦得住。
        CHECK(r.rmsForceN > 0.1);
    } else {
        std::cout << "[转置数据被直接拒绝] ";
    }
    PASS();
}

// =====================================================================================
// 原始力通道 (无 psi 的线性模型) 的用例
//
// 模型:   F_i = b_F + A·g_i          A: 任意 3×3
//         M_i = b_M + c_s × (A·g_i)  c_s: 质心
//         g_i = 重力在法兰系的表示 (psi = 0, 数据里【不存在】安装角)
// 参考数 (控制器 2026-09-19 由 7 个实机姿态离线算出, 见 task-1-brief):
//         A / b_F / m / parity / sv / isotropyRatio / c_s / 两个 rms
// =====================================================================================

// 3×3 行列式 (测试侧独立实现)
static double det3(const double M[9]) {
    return M[0] * (M[4] * M[8] - M[5] * M[7])
         - M[1] * (M[3] * M[8] - M[5] * M[6])
         + M[2] * (M[3] * M[7] - M[4] * M[6]);
}

// A 的 9 个分量里最大的那个 1σ —— 各向同性门限的尺子。测试侧独立算一遍 (不调被测函数)。
static double maxSigmaA(const PayloadCalibration::RawFit& f) {
    double s = 0.0;
    for (int i = 0; i < 9; i++) if (f.paramSigma[i] > s) s = f.paramSigma[i];
    return s;
}

// 合成器: 由【任意 3×3 A】造原始力/力矩。
// 【刻意不复用上面的 synthesize()】: 那一份是按"psi 纯偏航"模型造的 (solve() 用), 它的入参里
// 根本没有 A 这个概念。这一份拿任意 A 造数, 重力只用 psi = 0 —— 数据里不存在任何安装角,
// 这正是新模型要能拟合的东西。(生成器与估计器不共用代码, 见 gravitySensorRefAt 的说明。)
//
// grav 决定【造数据时】用哪个重力约定 —— 正向对照传 gravitySensorRefAt (psi 给 0), 反向对照
// 传 gravityTransposedAt (历史上那个转置 bug)。生成器永远不调用被测函数。
static void synthRawWith(GravityFnAt grav, double psiDeg, const double A[9], const double bF[3],
                         const double cs[3], const double bM[3], const double poses[][6], int n,
                         double forces[][3], double moments[][3])
{
    for (int i = 0; i < n; i++) {
        double g[3];
        grav(poses[i], psiDeg, g);
        double w[3];
        for (int a = 0; a < 3; a++)
            w[a] = A[a * 3 + 0] * g[0] + A[a * 3 + 1] * g[1] + A[a * 3 + 2] * g[2];
        for (int a = 0; a < 3; a++) forces[i][a] = bF[a] + w[a];
        // M = b_M + c_s × w
        moments[i][0] = bM[0] + cs[1] * w[2] - cs[2] * w[1];
        moments[i][1] = bM[1] + cs[2] * w[0] - cs[0] * w[2];
        moments[i][2] = bM[2] + cs[0] * w[1] - cs[1] * w[0];
    }
}

static void synthRaw(const double A[9], const double bF[3], const double cs[3],
                     const double bM[3], const double poses[][6], int n,
                     double forces[][3], double moments[][3])
{
    synthRawWith(gravitySensorRefAt, 0.0, A, bF, cs, bM, poses, n, forces, moments);
}

// 逐姿态的噪声申报 —— 测试【自己知道】它往均值里加了多少噪声 (addRawNoise 的 sigF/sigM),
// 所以它把加进去的量如实申报成"这个输入值的 1σ"。N = 1 = 没有做平均 (噪声是直接加在均值上
// 的), var = σ² -> σ_mean = σ。
// 【这不是"把答案喂给判据"】: 判据要的是"输入有多准"这个事实, 而这件事在生产路径上来自
// 采集时的样本方差 (BiasCheck), 在测试里只能来自"我知道我加了什么"。申报值与被测代码
// 如何拟合无关 —— 生成器与估计器仍然不共用代码。
static void declareNoise(int n, double sigF, double sigM, PayloadCalibration::PoseNoise* nz) {
    for (int i = 0; i < n; i++) {
        nz[i].n = 1;
        for (int a = 0; a < 3; a++) {
            nz[i].varF[a] = sigF * sigF;
            nz[i].varM[a] = sigM * sigM;
        }
    }
}

// A = m · diag(1,1,parity) · Rz(theta) · P   (P 由调用方给)
static void buildAWithP(double m, double parity, double thetaDeg, const double P[9], double A[9]) {
    const double D2R = 3.14159265358979323846 / 180.0;
    const double t = thetaDeg * D2R, c = cos(t), s = sin(t);
    const double R[9] = { c, -s, 0.0,  s, c, 0.0,  0.0, 0.0, 1.0 };
    double RP[9];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            double acc = 0.0;
            for (int q = 0; q < 3; q++) acc += R[i * 3 + q] * P[q * 3 + j];
            RP[i * 3 + j] = acc;
        }
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            A[i * 3 + j] = m * ((i == 2) ? parity : 1.0) * RP[i * 3 + j];
}

// P = I + k·u·vᵀ: k = 0 时 A 恰好是"质量尺度 × 含手系的旋转"; k ≠ 0 时 A 非正交。
static void buildA(double m, double parity, double thetaDeg, double k,
                   const double u[3], const double v[3], double A[9]) {
    double P[9];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) P[i * 3 + j] = (i == j ? 1.0 : 0.0) + k * u[i] * v[j];
    buildAWithP(m, parity, thetaDeg, P, A);
}

// 加噪声 (与 test_noise_robustness 同一套 LCG, 免得两处的"噪声"含义不同)
static void addRawNoise(double forces[][3], double moments[][3], int n,
                        double sigF, double sigM, unsigned& seed) {
    for (int i = 0; i < n; i++)
        for (int a = 0; a < 3; a++) {
            seed = seed * 1103515245u + 12345u;
            forces[i][a] += ((double)((seed >> 16) & 0x7fff) / 32767.0 - 0.5) * 2.0 * sigF;
            seed = seed * 1103515245u + 12345u;
            moments[i][a] += ((double)((seed >> 16) & 0x7fff) / 32767.0 - 0.5) * 2.0 * sigM;
        }
}

static const double ARB_U[3] = {0.5, 0.8, 0.2};
static const double ARB_V[3] = {0.6, -0.3, 0.45};

// 任意 3×3 (含【反射】与【非正交】) 的逐元素复原。
// 这一条走 fitRawLinear —— 它【不做物理自检】。这样安排是刻意的: 见 isotropy 那两条用例。
static void test_rawfit_recovers_arbitrary_A() {
    TEST(rawfit_recovers_arbitrary_A);
    double A[9];
    buildA(0.51, -1.0, 32.0, 0.12, ARB_U, ARB_V, A);     // 反射 + 非正交 (奇异值比 ≈ 1.10)
    const double bF[3] = {1.7, -0.9, 2.3};
    const double cs[3] = {0.021, -0.034, 0.118};
    const double bM[3] = {0.05, -0.02, 0.011};
    double F[NP][3], M[NP][3];
    synthRaw(A, bF, cs, bM, g_poses, NP, F, M);

    PayloadCalibration::RawFit fit;
    CHECK(PayloadCalibration::fitRawLinear(g_poses, F, M, NP, fit));
    for (int i = 0; i < 9; i++) CHECK(fabs(fit.A[i] - A[i]) < 1e-12);
    for (int i = 0; i < 3; i++) {
        CHECK(fabs(fit.bF[i] - bF[i]) < 1e-12);
        CHECK(fabs(fit.cS[i] - cs[i]) < 1e-12);
        CHECK(fabs(fit.bM[i] - bM[i]) < 1e-12);
    }
    printf("[noiseless: rmsF=%.1e rmsM=%.1e cond=%.1f] ", fit.rmsForceN, fit.rmsMomentNm, fit.cond);
    CHECK(fit.rmsForceN < 1e-12 && fit.rmsMomentNm < 1e-12);

    // 加噪声: 容差【按 paramSigma】—— 9 个分量各按自己的 4σ 判, 不是拍一个绝对数。
    unsigned seed = 20260919u;
    addRawNoise(F, M, NP, 0.02, 0.001, seed);
    CHECK(PayloadCalibration::fitRawLinear(g_poses, F, M, NP, fit));
    int worst = 0;
    for (int i = 0; i < 9; i++) {
        const double tol = 4.0 * fit.paramSigma[i];
        CHECK(tol > 1e-9);                                  // paramSigma 得是真估出来的
        if (fabs(fit.A[i] - A[i]) > fabs(fit.A[worst] - A[worst])) worst = i;
        CHECK(fabs(fit.A[i] - A[i]) < tol);
    }
    for (int i = 0; i < 3; i++) {
        CHECK(fabs(fit.cS[i] - cs[i]) < 4.0 * fit.paramSigma[12 + i]);
        CHECK(fabs(fit.bM[i] - bM[i]) < 4.0 * fit.paramSigma[15 + i]);
        CHECK(fabs(fit.bF[i] - bF[i]) < 4.0 * fit.paramSigma[9 + i]);
    }
    printf("[noisy: A[%d] err=%.1e (4sig=%.1e), rmsF=%.4f] ",
           worst, fabs(fit.A[worst] - A[worst]), 4.0 * fit.paramSigma[worst], fit.rmsForceN);
    // 尺度: 质量尺度也该复原 (它是 A 的奇异值几何平均, 不是单独拟合的参数)
    PayloadCalibration::Decomp d;
    CHECK(PayloadCalibration::decompose(fit.A, d));
    CHECK(fabs(d.m - 0.51304) < 0.02);
    PASS();
}

// 分解: 已知角度 + 已知手系 → m / parity / theta 都要能读回来。
static void test_decompose_recovers_rotation_and_parity() {
    TEST(decompose_recovers_rotation_and_parity);
    const double m = 0.657;
    const double thetas[3] = {0.0, 32.0, -141.5};
    const double parities[2] = {+1.0, -1.0};
    for (int pi = 0; pi < 2; pi++) {
        for (int ti = 0; ti < 3; ti++) {
            double A[9];
            buildA(m, parities[pi], thetas[ti], 0.0, ARB_U, ARB_V, A);   // 恰好 m·S·Rz(theta)
            PayloadCalibration::Decomp d;
            CHECK(PayloadCalibration::decompose(A, d));
            CHECK(fabs(d.m - m) < 1e-12);
            CHECK(fabs(d.parity - parities[pi]) < 1e-12);
            for (int k = 0; k < 3; k++) CHECK(fabs(d.sv[k] - m) < 1e-12);
            CHECK(fabs(d.isotropyRatio - 1.0) < 1e-12);
            // Q 必须【就是】Rz(theta): Q = S·A/m = S·S·Rz = Rz。约定被钉死在这一条上 ——
            // 谁改了约定 (比如换成 U·diag(1,1,parity)·Vᵀ), 这里立刻红。
            const double D2R = 3.14159265358979323846 / 180.0;
            const double t = thetas[ti] * D2R;
            const double R[9] = { cos(t), -sin(t), 0.0,  sin(t), cos(t), 0.0,  0.0, 0.0, 1.0 };
            for (int i = 0; i < 9; i++) CHECK(fabs(d.Q[i] - R[i]) < 1e-12);
            CHECK(fabs(det3(d.Q) - 1.0) < 1e-12);   // Q 恒为旋转 (det = +1), 手系在 parity 里
        }
    }
    printf("[12 组合 (m, parity, theta) 全部复原] ");
    PASS();
}

// round-trip: 由 (m, S=diag(1,1,parity), Q) 重建 A, 与输入【逐位】相等。
// 这条对【任意可逆 A】都成立 (不只是正交的 A) —— 这是约定良定义的证明。
static void test_decompose_roundtrip_exact() {
    TEST(decompose_roundtrip_exact);
    double cases[3][9];
    buildA(0.51, -1.0, 32.0, 0.12, ARB_U, ARB_V, cases[0]);       // 非正交 + 反射
    buildA(0.83, +1.0, -77.0, -0.20, ARB_V, ARB_U, cases[1]);     // 非正交, 手系为正
    // 参考 A (task-1-brief 里控制器给的那一份, 5 位小数)
    const double ref[9] = { 0.36456, 0.21055, -0.00003,
                           -0.21828, 0.37331,  0.00298,
                           -0.01155, -0.01493, -0.41390 };
    for (int i = 0; i < 9; i++) cases[2][i] = ref[i];

    for (int c = 0; c < 3; c++) {
        PayloadCalibration::Decomp d;
        CHECK(PayloadCalibration::decompose(cases[c], d));
        // A_rebuilt = m · diag(1,1,parity) · Q
        double rebuilt[9];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                rebuilt[i * 3 + j] = d.m * ((i == 2) ? d.parity : 1.0) * d.Q[i * 3 + j];
        for (int i = 0; i < 9; i++) CHECK(fabs(rebuilt[i] - cases[c][i]) < 1e-12);
    }
    PASS();
}

// 分解还必须能对上控制器给的参考数 (task-1-brief): 这是"与实机离线结果一致"的静态锚点。
static void test_decompose_matches_reference_numbers() {
    TEST(decompose_matches_reference_numbers);
    const double ref[9] = { 0.36456, 0.21055, -0.00003,
                           -0.21828, 0.37331,  0.00298,
                           -0.01155, -0.01493, -0.41390 };
    PayloadCalibration::Decomp d;
    CHECK(PayloadCalibration::decompose(ref, d));
    printf("[m=%.5f parity=%+.0f sv=[%.5f %.5f %.5f] iso=%.5f] ",
           d.m, d.parity, d.sv[0], d.sv[1], d.sv[2], d.isotropyRatio);
    CHECK(fabs(d.m - 0.42236) < 1e-4);
    CHECK(fabs(d.parity - (-1.0)) < 1e-12);
    CHECK(fabs(d.sv[0] - 0.43386) < 1e-4);
    CHECK(fabs(d.sv[1] - 0.42647) < 1e-4);
    CHECK(fabs(d.sv[2] - 0.40720) < 1e-4);
    CHECK(fabs(d.isotropyRatio - 1.0655) < 1e-3);
    CHECK(fabs(det3(ref) - (-0.075342)) < 1e-4);
    PASS();
}

// 各向同性比是【报告量, 不是门限】: 明显各向异性的 A (奇异值比 = 4) 照样被接受,
// 但因为数据里本来就是这个响应 —— A 的 9 个元素全自由, "非正交"是被测量出来的事实。
// (取代 isotropy_gate_rejects_nonorthogonal: 它当年断言"4:1 必须被拒"。那个断言的
//  根据是自指的 (σ ← 残差), 且方向本身是错的 —— 力的 A 没有任何约束, 非正交不是罪证。)
static void test_isotropy_ratio_is_reported_not_gated() {
    TEST(isotropy_ratio_is_reported_not_gated);
    const double D[9] = {1.0, 0.0, 0.0,  0.0, 1.0, 0.0,  0.0, 0.0, 0.25};
    double A[9];
    buildAWithP(0.45, -1.0, 10.0, D, A);          // 奇异值 (0.45, 0.45, 0.1125), 比 = 4
    const double bF[3] = {0.4, 0.3, -0.2};
    const double cs[3] = {0.01, 0.02, 0.09};
    const double bM[3] = {0.01, -0.01, 0.005};
    double F[NP][3], M[NP][3];
    synthRaw(A, bF, cs, bM, g_poses, NP, F, M);
    unsigned seed = 7u;
    addRawNoise(F, M, NP, 0.01, 0.0005, seed);

    PayloadCalibration::PoseNoise nz[NP];
    declareNoise(NP, 0.01, 0.0005, nz);

    // 接受 —— 且是【过了模型形式检验】才接受的 (不是"没做检验")。
    PayloadCalibration::RawFit fit;
    CHECK(PayloadCalibration::fitRaw(g_poses, F, M, NP, fit, nz));
    CHECK(fit.modelFormChecked);
    printf("[iso=%.4f 报告 (m=%.4f kg), 残差 %.4f N vs 噪声 %.4f N -> 过] ",
           fit.isotropyRatio, fit.massScale, fit.rmsForceN, fit.noiseForceN);
    CHECK(fit.isotropyRatio > 3.5);              // 展布照实报出来 (4:1)
    CHECK(fit.chi2ForceRatio < 1.0 + 3.0 * sqrt(2.0 / fit.chi2DofForce));
    // 报告量与 decompose 是同一个数 (不是另算一份口径)
    PayloadCalibration::Decomp d;
    CHECK(PayloadCalibration::decompose(fit.A, d));
    CHECK(fabs(d.isotropyRatio - fit.isotropyRatio) < 1e-12);
    CHECK(fabs(d.m - fit.massScale) < 1e-12);
    PASS();
}

// ★ 评审的反例 (本次修复要拦住的正是它): 真值【完全各向同性】, 数据用历史上的【转置回归量】
//   生成 (就是那个让实机安静地解错两次的 bug), 0.02 N 噪声。
//   旧的各向同性门限拿它没办法: 拟合 iso≈2.01, 而门限 1+3σ/m 随残差一起涨到 ≈2.59
//   → 【接受】, 返回 m≈0.4159 (真值 0.42)。这正是"安静地给出错答案"。
//   新判据必须拒: 残差 ~0.47 N 对上实测噪声 0.02 N, χ²/dof 差着几个数量级。
static void test_modelform_rejects_transposed_convention() {
    TEST(modelform_rejects_transposed_convention);
    double A[9];
    buildA(0.42, -1.0, 30.0, 0.0, ARB_U, ARB_V, A);   // 真值 = 恰好 0.42·diag(1,1,-1)·Rz(30°)
    const double bF[3] = {0.0, 0.0, 0.0};
    const double cs[3] = {0.0, 0.0, 0.08};
    const double bM[3] = {0.0, 0.0, 0.0};
    double F[NP][3], M[NP][3];
    synthRawWith(gravityTransposedAt, 0.0, A, bF, cs, bM, g_poses, NP, F, M);
    unsigned seed = 20260919u;
    addRawNoise(F, M, NP, 0.02, 0.001, seed);

    PayloadCalibration::PoseNoise nz[NP];
    declareNoise(NP, 0.02, 0.001, nz);

    // 线性层照样解得出来 —— 被拒的是模型形式, 不是拟合。
    PayloadCalibration::RawFit fit;
    CHECK(PayloadCalibration::fitRawLinear(g_poses, F, M, NP, fit, nz));
    PayloadCalibration::Decomp d;
    CHECK(PayloadCalibration::decompose(fit.A, d));
    // 旧门限的自指关系照实算出来 (供对照, 不作断言依据): 残差涨 -> σ 涨 -> 门限涨得比 iso 快。
    const double oldLimit = 1.0 + 3.0 * maxSigmaA(fit) / d.m;
    printf("[转置数据: rmsF=%.4f N, 噪声=%.4f N, iso=%.3f, 旧门限 1+3s/m=%.3f -> 旧行为接受] ",
           fit.rmsForceN, fit.noiseForceN, d.isotropyRatio, oldLimit);
    CHECK(d.isotropyRatio < oldLimit);            // 旧门限确实放它过去 (这就是那个漏洞)
    CHECK(fit.rmsForceN > 0.1);                   // 残差远大于噪声 —— 数据与模型形式不符

    // 新判据: 拒。
    CHECK(!PayloadCalibration::fitRaw(g_poses, F, M, NP, fit, nz));
    printf("[χ²/dof=%.1f (dof=%d, 限=%.2f) -> 拒] ",
           fit.chi2ForceRatio, fit.chi2DofForce,
           1.0 + 3.0 * sqrt(2.0 / fit.chi2DofForce));
    CHECK(fit.chi2ForceRatio > 1.0 + 3.0 * sqrt(2.0 / fit.chi2DofForce));
    PASS();
}

// 正确模型 + 与实机同量级的噪声 -> 必须接受, 且是【过了检验】才接受的。
// 与上一条配对: 一个拒、一个过, 而两条的差别只在"数据是不是真按模型生成的"。
static void test_modelform_accepts_correct_fit_with_noise() {
    TEST(modelform_accepts_correct_fit_with_noise);
    double A[9];
    buildA(0.42, -1.0, 30.0, 0.0, ARB_U, ARB_V, A);   // 与反例【同一个 A】, 但约定是对的
    const double bF[3] = {1.2, -0.4, 0.3};
    const double cs[3] = {0.005, -0.008, 0.061};
    const double bM[3] = {0.01, -0.01, 0.005};
    double F[NP][3], M[NP][3];
    synthRaw(A, bF, cs, bM, g_poses, NP, F, M);
    unsigned seed = 20260919u;
    addRawNoise(F, M, NP, 0.02, 0.001, seed);

    PayloadCalibration::PoseNoise nz[NP];
    declareNoise(NP, 0.02, 0.001, nz);

    PayloadCalibration::RawFit fit;
    CHECK(PayloadCalibration::fitRawLinear(g_poses, F, M, NP, fit, nz));
    printf("[正确约定: rmsF=%.4f N, 噪声=%.4f N, χ²/dof=%.2f (dof=%d), 力矩失拟=%.2f (dof=%d)] ",
           fit.rmsForceN, fit.noiseForceN, fit.chi2ForceRatio, fit.chi2DofForce,
           fit.lackOfFitMomentRatio, fit.lackOfFitMomentDof);
    CHECK(PayloadCalibration::fitRaw(g_poses, F, M, NP, fit, nz));
    CHECK(fit.modelFormChecked);
    // 正确模型下 χ²/dof 应落在 1 附近 —— 门限 1+3·sqrt(2/dof) 之内, 且不该小得离谱
    // (太小说明噪声被报大了, 那会让判据失去分辨力)。
    CHECK(fit.chi2ForceRatio < 1.0 + 3.0 * sqrt(2.0 / fit.chi2DofForce));
    CHECK(fit.chi2ForceRatio > 0.2);
    printf("[A[0] err=%.1e] ", fabs(fit.A[0] - A[0]));
    CHECK(fabs(fit.A[0] - A[0]) < 4.0 * fit.paramSigma[0]);
    PASS();
}

// 门限【随实测噪声走】: 同一批数据 (同一个模型形式错), 只把【申报的】噪声改掉 —— 判决跟着翻。
// 这条是"阈值来自实测噪声, 不是固定的 N 数"的直接证明:
//   (a) 申报 0.02 N: 残差 0.47 N 解释不了 -> 拒;
//   (b) 申报 0.60 N: 0.47 N 的残差与这么吵的测量并不矛盾 -> 接受。
// (取代 isotropy_gate_tracks_param_sigma: 它当年用【同一批数据 + 两个噪声水平】证明"门限随
//  paramSigma 走", 而 paramSigma 来自残差 —— 噪声大残差也大, 那个"随"是自指的。)
static void test_modelform_gate_tracks_measured_noise() {
    TEST(modelform_gate_tracks_measured_noise);
    double A[9];
    buildA(0.42, -1.0, 30.0, 0.0, ARB_U, ARB_V, A);
    const double bF[3] = {0.0, 0.0, 0.0};
    const double cs[3] = {0.0, 0.0, 0.08};
    const double bM[3] = {0.0, 0.0, 0.0};
    double F[NP][3], M[NP][3];
    synthRawWith(gravityTransposedAt, 0.0, A, bF, cs, bM, g_poses, NP, F, M);
    unsigned seed = 20260919u;
    addRawNoise(F, M, NP, 0.02, 0.001, seed);      // 真实噪声: 0.02 N

    PayloadCalibration::RawFit fit;
    PayloadCalibration::PoseNoise nz[NP];

    // (a) 如实申报 0.02 N -> 残差与之不符 -> 拒
    declareNoise(NP, 0.02, 0.001, nz);
    CHECK(PayloadCalibration::fitRawLinear(g_poses, F, M, NP, fit, nz));
    printf("[申报 0.02N: χ²/dof=%.1f -> 拒] ", fit.chi2ForceRatio);
    CHECK(!PayloadCalibration::fitRaw(g_poses, F, M, NP, fit, nz));

    // (b) 同一个拟合, 只把噪声申报成 0.60 N -> 同一个残差落进门限 -> 接受
    declareNoise(NP, 0.60, 0.03, nz);
    CHECK(PayloadCalibration::fitRawLinear(g_poses, F, M, NP, fit, nz));
    printf("[申报 0.60N: χ²/dof=%.2f (限 %.2f), 力矩失拟=%.2f -> 过] ",
           fit.chi2ForceRatio, 1.0 + 3.0 * sqrt(2.0 / fit.chi2DofForce),
           fit.lackOfFitMomentRatio);
    CHECK(PayloadCalibration::fitRaw(g_poses, F, M, NP, fit, nz));
    CHECK(fit.chi2ForceRatio < 1.0 + 3.0 * sqrt(2.0 / fit.chi2DofForce));
    PASS();
}

// 力矩通道的【失拟】门: 它是受约束的 (c_s × (A·g), 3 参数 vs 自由的 9), 所以那里问的是
// "叉乘结构成不成立"。这里造一批力通道完全正确、而力矩【不是】叉乘结构的数据
// (M = N·w, N 含对称分量) —— 力通道的 χ² 照样合格, 只有力矩失拟能把这种错挑出来。
// 【这条用例存在的理由】: 力矩判据是新增的代码路径, 没有它的话"门能不能开"无从验证
// (本实现第一次就把它写反了方向 (ssFree − ssM ≤ 0 恒成立), 全靠这里的对照才发现)。
static void test_moment_lack_of_fit_rejects_non_cross_product() {
    TEST(moment_lack_of_fit_rejects_non_cross_product);
    double A[9];
    buildA(0.42, -1.0, 30.0, 0.0, ARB_U, ARB_V, A);
    const double bF[3] = {1.0, -0.5, 0.2};
    const double cs[3] = {0.005, -0.008, 0.061};
    const double bM[3] = {0.01, -0.01, 0.005};
    // 力矩的"响应矩阵": 反对称部分 (≈ c_s×) 之外再加一个对称部分, 量级 ~4 mm (远超噪声)
    const double N[9] = { 0.001, 0.004, 0.000,
                          0.004, 0.002, 0.000,
                          0.000, 0.000, 0.003 };
    double F[NP][3], M[NP][3];
    synthRaw(A, bF, cs, bM, g_poses, NP, F, M);
    for (int i = 0; i < NP; i++) {
        double g[3], w[3];
        gravitySensorRefAt(g_poses[i], 0.0, g);          // 测试侧自己算, 不调被测函数
        for (int a = 0; a < 3; a++)
            w[a] = A[a * 3 + 0] * g[0] + A[a * 3 + 1] * g[1] + A[a * 3 + 2] * g[2];
        for (int a = 0; a < 3; a++) {
            // 保留原来的 c_s × w, 再叠加非反对称的那一份
            const double cross = (a == 0) ? (cs[1] * w[2] - cs[2] * w[1])
                               : (a == 1) ? (cs[2] * w[0] - cs[0] * w[2])
                                          : (cs[0] * w[1] - cs[1] * w[0]);
            M[i][a] = bM[a] + cross + N[a * 3 + 0] * w[0] + N[a * 3 + 1] * w[1]
                                                                + N[a * 3 + 2] * w[2];
        }
    }
    unsigned seed = 4242u;
    addRawNoise(F, M, NP, 0.02, 0.001, seed);
    PayloadCalibration::PoseNoise nz[NP];
    declareNoise(NP, 0.02, 0.001, nz);

    PayloadCalibration::RawFit fit;
    CHECK(PayloadCalibration::fitRawLinear(g_poses, F, M, NP, fit, nz));
    printf("[力通道 χ²/dof=%.2f (合格), 力矩失拟=%.2f (dof=%d, 限=%.2f) -> 拒] ",
           fit.chi2ForceRatio, fit.lackOfFitMomentRatio, fit.lackOfFitMomentDof,
           1.0 + 3.0 * sqrt(2.0 / fit.lackOfFitMomentDof));
    // 力通道没问题 (它是自由拟合, 数据也确实符合) —— 拒的理由必须来自力矩那一条
    CHECK(fit.chi2ForceRatio < 1.0 + 3.0 * sqrt(2.0 / fit.chi2DofForce));
    CHECK(fit.lackOfFitMomentRatio > 1.0 + 3.0 * sqrt(2.0 / fit.lackOfFitMomentDof));
    CHECK(!PayloadCalibration::fitRaw(g_poses, F, M, NP, fit, nz));
    PASS();
}

// 实机那条正确的解 (参考 A, iso = 1.0655, 6.5% 的物理非正交) 必须过 —— 而且现在是靠
// "残差 vs 实测噪声"过的, 不再靠"iso 撞上一条随残差放松的门限"。
// 用参考 A 的形状 + 与实机同量级的噪声造数据。
static void test_realistic_spread_accepted_with_measured_noise() {
    TEST(realistic_spread_accepted_with_measured_noise);
    const double ref[9] = { 0.36456, 0.21055, -0.00003,
                           -0.21828, 0.37331,  0.00298,
                           -0.01155, -0.01493, -0.41390 };
    const double bF[3] = {-18.55, -2.44, 0.82};
    const double cs[3] = {0.0006, -0.0005, 0.0545};
    const double bM[3] = {-0.16, 0.57, -0.02};
    double F[NP][3], M[NP][3];
    synthRaw(ref, bF, cs, bM, g_poses, NP, F, M);
    unsigned seed = 1304u;
    addRawNoise(F, M, NP, 0.15, 0.008, seed);

    PayloadCalibration::PoseNoise nz[NP];
    declareNoise(NP, 0.15, 0.008, nz);

    PayloadCalibration::RawFit fit;
    CHECK(PayloadCalibration::fitRaw(g_poses, F, M, NP, fit, nz));   // 必须放行
    CHECK(fit.modelFormChecked);
    printf("[iso=%.4f (报告), rmsF=%.4f N, 噪声=%.4f N, χ²/dof=%.2f, 力矩失拟=%.2f,"
           " m=%.4f kg, parity=%+.0f] ",
           fit.isotropyRatio, fit.rmsForceN, fit.noiseForceN, fit.chi2ForceRatio,
           fit.lackOfFitMomentRatio, fit.massScale, fit.parity);
    CHECK(fit.isotropyRatio > 1.02);               // 确实带着实机那种量级的展布 (不是碰巧正交)
    CHECK(fabs(fit.parity - (-1.0)) < 1e-12);      // 参考 A 的手系是负的
    CHECK(fabs(fit.massScale - 0.4224) < 0.06);
    PASS();
}

// 没有噪声估计 = 【不做】模型形式检验, 而不是"通过": 同一个反例数据, 不传 noise 时
// 线性层与自检都放行 —— 这个"洞"必须【看得见】 (modelFormChecked = false), 而不是被当成绿灯。
// 同时钉住: 生产路径不传 noise 就等于关掉了模型形式检验 —— 所以生产路径必须传。
static void test_modelform_not_checked_without_noise() {
    TEST(modelform_not_checked_without_noise);
    double A[9];
    buildA(0.42, -1.0, 30.0, 0.0, ARB_U, ARB_V, A);
    const double bF[3] = {0.0, 0.0, 0.0};
    const double cs[3] = {0.0, 0.0, 0.08};
    const double bM[3] = {0.0, 0.0, 0.0};
    double F[NP][3], M[NP][3];
    synthRawWith(gravityTransposedAt, 0.0, A, bF, cs, bM, g_poses, NP, F, M);
    unsigned seed = 20260919u;
    addRawNoise(F, M, NP, 0.02, 0.001, seed);

    PayloadCalibration::RawFit fit;
    CHECK(PayloadCalibration::fitRaw(g_poses, F, M, NP, fit));    // 5 参数旧形式
    CHECK(!fit.modelFormChecked);                                // 明说"没验过"
    CHECK(fit.chi2ForceRatio == 0.0 && fit.chi2DofForce == 0);   // 检验用的字段全空

    // 不可用的噪声估计 (方差为 0 = 通道冻住 / 没采到) 同样【不做】检验, 而不是"通过"。
    PayloadCalibration::PoseNoise nz[NP];
    declareNoise(NP, 0.02, 0.001, nz);
    nz[2].varF[1] = 0.0;
    CHECK(PayloadCalibration::fitRawLinear(g_poses, F, M, NP, fit, nz));
    CHECK(!fit.modelFormChecked);
    // 与"没给"完全同路: 不做检验, 也【不】因此拒绝 (拒绝必须由某条判据给出理由, 不是"缺数据
    // 就毙掉")。调用方要区分这两种情形, 读的就是 modelFormChecked。
    CHECK(PayloadCalibration::fitRaw(g_poses, F, M, NP, fit, nz));
    CHECK(!fit.modelFormChecked);
    PASS();
}

// 拟合器【不读 psi】: 这个模型里根本没有 psi。
// (a) 含反射的数据也能拟合到机器精度 —— 纯偏航模型做不到 (它当年靠翻质量符号去凑);
// (b) 把 psi 的模块状态改成任何值, 结果【逐位】不变。
static void test_rawfit_uses_no_psi() {
    TEST(rawfit_uses_no_psi);
    double A[9];
    buildA(0.657, -1.0, 24.0, 0.0, ARB_U, ARB_V, A);     // 恰好是 m·S·Rz(24°): 含反射
    const double bF[3] = {0.31, -0.22, 0.17};
    const double cs[3] = {0.004, -0.007, 0.081};
    const double bM[3] = {0.003, -0.004, 0.002};
    double F[NP][3], M[NP][3];
    synthRaw(A, bF, cs, bM, g_poses, NP, F, M);

    PayloadCalibration::RawFit withYaw, withoutYaw;
    TcpCalibration::setSensorYawDeg(137.5);              // 一个绝不可能被"扫"到的角
    CHECK(PayloadCalibration::fitRawLinear(g_poses, F, M, NP, withYaw));
    TcpCalibration::setSensorYawDeg(PSI_DEG);            // 模块状态还原 (别的用例依赖它)
    CHECK(PayloadCalibration::fitRawLinear(g_poses, F, M, NP, withoutYaw));

    for (int i = 0; i < 9; i++) CHECK(withYaw.A[i] == withoutYaw.A[i]);   // 逐位
    for (int i = 0; i < 3; i++) {
        CHECK(withYaw.bF[i] == withoutYaw.bF[i]);
        CHECK(withYaw.cS[i] == withoutYaw.cS[i]);
        CHECK(withYaw.bM[i] == withoutYaw.bM[i]);
    }
    printf("[反射数据 psi=137.5deg 与 psi=%.0fdeg 结果逐位相同, rmsF=%.1e] ",
           PSI_DEG, withoutYaw.rmsForceN);
    CHECK(withoutYaw.rmsForceN < 1e-12 && withoutYaw.rmsMomentNm < 1e-12);
    for (int i = 0; i < 9; i++) CHECK(fabs(withoutYaw.A[i] - A[i]) < 1e-12);
    // 模块状态确实还原了 (否则后面的用例会跑在另一个重力模型上)
    CHECK(fabs(TcpCalibration::sensorYawDeg() - PSI_DEG) < 1e-12);
    PASS();
}

// 姿态数不足: 12 个未知量, 3 个姿态只给 9 个方程 -> 解不出
static void test_rawfit_rejects_too_few_poses() {
    TEST(rawfit_rejects_too_few_poses);
    double A[9];
    buildA(0.51, -1.0, 32.0, 0.0, ARB_U, ARB_V, A);
    const double bF[3] = {0, 0, 0}, cs[3] = {0, 0, 0.08}, bM[3] = {0, 0, 0};
    double F[NP][3], M[NP][3];
    synthRaw(A, bF, cs, bM, g_poses, NP, F, M);
    PayloadCalibration::RawFit fit;
    CHECK(!PayloadCalibration::fitRawLinear(g_poses, F, M, 3, fit));
    CHECK(!PayloadCalibration::fitRaw(g_poses, F, M, 3, fit));
    CHECK(PayloadCalibration::fitRawLinear(g_poses, F, M, 4, fit));   // 4 个正好够
    PASS();
}

// 姿态退化: 全部同姿态 -> J 的 A 列与常数项共线 -> 秩亏
static void test_rawfit_rejects_degenerate_poses() {
    TEST(rawfit_rejects_degenerate_poses);
    double same[NP][6];
    for (int i = 0; i < NP; i++)
        for (int a = 0; a < 6; a++) same[i][a] = g_poses[0][a];
    double A[9];
    buildA(0.51, -1.0, 32.0, 0.0, ARB_U, ARB_V, A);
    const double bF[3] = {0, 0, 0}, cs[3] = {0, 0, 0.08}, bM[3] = {0, 0, 0};
    double F[NP][3], M[NP][3];
    synthRaw(A, bF, cs, bM, same, NP, F, M);
    PayloadCalibration::RawFit fit;
    CHECK(!PayloadCalibration::fitRawLinear(same, F, M, NP, fit));
    CHECK(!PayloadCalibration::fitRaw(same, F, M, NP, fit));
    PASS();
}

// 质量尺度自检: 解出的尺度必须落在 CR3 的负载量程里 (EnableRobot 的量程)
static void test_rawfit_rejects_bad_mass_scale() {
    TEST(rawfit_rejects_bad_mass_scale);
    const double bF[3] = {0, 0, 0}, cs[3] = {0, 0, 0.08}, bM[3] = {0, 0, 0};
    double A[9], F[NP][3], M[NP][3];
    PayloadCalibration::RawFit fit;

    buildA(0.001, -1.0, 32.0, 0.0, ARB_U, ARB_V, A);      // 1 g —— 不是工具, 是模型形式错
    synthRaw(A, bF, cs, bM, g_poses, NP, F, M);
    CHECK(PayloadCalibration::fitRawLinear(g_poses, F, M, NP, fit));   // 线性层解得出来
    CHECK(!PayloadCalibration::fitRaw(g_poses, F, M, NP, fit));        // 门拒掉

    buildA(12.0, -1.0, 32.0, 0.0, ARB_U, ARB_V, A);       // 12 kg —— 超过 CR3 的 3 kg
    synthRaw(A, bF, cs, bM, g_poses, NP, F, M);
    CHECK(PayloadCalibration::fitRawLinear(g_poses, F, M, NP, fit));
    CHECK(!PayloadCalibration::fitRaw(g_poses, F, M, NP, fit));
    PASS();
}

// 分解的退化解: 奇异 A 定不出手系与尺度
static void test_decompose_rejects_singular_A() {
    TEST(decompose_rejects_singular_A);
    const double zero[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    const double flat[9] = {1, 2, 3, 2, 4, 6, 0, 0, 0};    // 奇异值 (0, 0, ~8.1)
    PayloadCalibration::Decomp d;
    CHECK(!PayloadCalibration::decompose(zero, d));
    CHECK(!PayloadCalibration::decompose(flat, d));
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
    test_load_old_file_without_yaw();
    test_effective_falls_back_to_seed();
    test_fit_is_sign_independent();
    test_gravity_sensor_frame_convention();
    test_sensor_yaw_state();
    test_solver_uses_shared_gravity_convention();
    // psi 估计: 两个不同的真值各来一遍 (只测一个说明不了"不是被调成那个答案的")
    test_recovers_sensor_yaw(80.0);
    test_recovers_sensor_yaw(35.0);
    test_recovers_sensor_yaw(80.25);   // 故意落在网格【之间】—— 验证容差 (半个步长) 不是摆设
    test_scan_cannot_fix_transposed_gravity();

    // 原始力通道 (线性模型 + 分解 + 自检) —— 新模型, 不含 psi
    std::cout << "--- raw channel (linear model, no psi) ---" << std::endl;
    test_decompose_recovers_rotation_and_parity();
    test_decompose_roundtrip_exact();
    test_decompose_matches_reference_numbers();
    test_decompose_rejects_singular_A();
    test_rawfit_recovers_arbitrary_A();
    test_rawfit_uses_no_psi();
    // 各向同性比: 【报告量】, 不进判决 (它由力通道的 A 自由性决定, 非正交是物理属性)
    test_isotropy_ratio_is_reported_not_gated();
    // 模型形式检验: 残差 vs 【实测】噪声 (χ² 式)。判据的尺度全来自数据。
    test_modelform_rejects_transposed_convention();   // ★ 评审的反例 (旧门限放它过去)
    test_modelform_accepts_correct_fit_with_noise();
    test_modelform_gate_tracks_measured_noise();
    test_moment_lack_of_fit_rejects_non_cross_product();
    test_modelform_not_checked_without_noise();
    test_realistic_spread_accepted_with_measured_noise();
    test_rawfit_rejects_too_few_poses();
    test_rawfit_rejects_degenerate_poses();
    test_rawfit_rejects_bad_mass_scale();

    std::cout << "\n" << g_passed << " passed, " << g_failed << " failed" << std::endl;
    return g_failed ? 1 : 0;
}
