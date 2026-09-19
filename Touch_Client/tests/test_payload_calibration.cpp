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
    std::cout << "\n" << g_passed << " passed, " << g_failed << " failed" << std::endl;
    return g_failed ? 1 : 0;
}
