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
#include <ctime>
#include <string>        // Task 8a-2: 措辞用例要按子串断言 (标签 / 结论行)
#ifdef _WIN32
#include <io.h>          // _dup / _dup2 / _close —— 蒙特卡洛要把 stderr 静音
#include <fcntl.h>       // _O_WRONLY
#endif
#include "../force/PayloadCalibration.h"
#include "../force/RepeatPairRegistry.h"
#include "../force/ForceCompensation.h"     // ★ Task 6 验收: 跑生产补偿代码本身
#include "../core/AppState.h"
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
// 与共享函数 (TcpCalibration::gravitySensorFrame)。
// (2026-09-19 Task 6 起 ForceCompensation.cpp 也链进了本可执行文件 —— 见文件末尾的验收
//  用例 test_runtime_compensation_pose_independence, 它跑的就是生产补偿代码本身。)
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

// ===== 重复姿态对 (模型形式检验的尺子) =====
// 采集协议: 摆完所有姿态后【回到第 1 个姿态】(位置和姿态都回来) 再采一次, 中间有真实运动。
// 合成数据里就是"把第 1 行那个姿态再放一遍, 给它独立的一份噪声"。
// 【为什么必须新加一行, 而不是原地复制一遍】: 那一行要有【自己的】噪声与自己的偏差 —— 尺子
// 量的正是"同一位姿再来一次会差多少", 复制一份只会量出 0。
static const int NQ = NP + 1;
static const PayloadCalibration::RepeatPair REP_PAIR = {0, NP};

static void buildRepeatPoses(double out[NQ][6]) {
    for (int i = 0; i < NP; i++)
        for (int a = 0; a < 6; a++) out[i][a] = g_poses[i][a];
    for (int a = 0; a < 6; a++) out[NP][a] = g_poses[0][a];   // 第二次访问: 同一个位姿
}

// ===== 多对重复访问 (尺子的自由度) =====
// 采集端按 'r' 是【追加】一对 (不覆盖), 求解端把它们【池化】成一把尺子, 尺子的自由度 = 对数。
// 这里造最多 MAXR 对: 第 1 个姿态被反复访问。
static const int MAXR = 6;
static const int NQR = NP + MAXR;
static const PayloadCalibration::RepeatPair REP_PAIRS[MAXR] = {
    {0, NP + 0}, {0, NP + 1}, {0, NP + 2}, {0, NP + 3}, {0, NP + 4}, {0, NP + 5}
};

static void buildRepeatPosesR(double out[NQR][6], int pairs) {
    for (int i = 0; i < NP; i++)
        for (int a = 0; a < 6; a++) out[i][a] = g_poses[i][a];
    for (int j = 0; j < pairs; j++)
        for (int a = 0; a < 6; a++) out[NP + j][a] = g_poses[0][a];
}

// 姿态间复现性 (sigma_sys) 的合成: 【每一次访问】各自抽一份偏移, 行与行之间独立。
// 这是"回到同一个位姿再来一次, 读数能差多少"的物理模型 —— 模型形式仍然是对的 (它只是一份
// 与位姿无关的额外噪声), 而【残差】与【重复对的差值】被它同时抬高: 这正是
// r = sigma_sys^2 / sigma_in^2 这个比值。实机那批数据就落在 r = 3 附近。
// 与 addRawNoise 同一套 LCG, 免得两处的"噪声"含义不同。
static void addVisitScatter(double forces[][3], double moments[][3], int n,
                            double sF, double sM, unsigned& seed) {
    for (int i = 0; i < n; i++)
        for (int a = 0; a < 3; a++) {
            seed = seed * 1103515245u + 12345u;
            forces[i][a] += ((double)((seed >> 16) & 0x7fff) / 32767.0 - 0.5) * 2.0 * sF;
            seed = seed * 1103515245u + 12345u;
            moments[i][a] += ((double)((seed >> 16) & 0x7fff) / 32767.0 - 0.5) * 2.0 * sM;
        }
}

// 尺子两个分量的比值 r = SUM_a sigma_sys,a^2 / SUM_a floor,a^2 —— 门限的放宽幅度由它决定
// (见 PayloadCalibration::modelFormLimit)。测试侧独立算一遍, 好把 r 打进日志。
static double rhoFromFit(const PayloadCalibration::RawFit& f) {
    double s = 0.0, fl = 0.0;
    for (int a = 0; a < 3; a++) { s += f.repeatSysF[a]; fl += f.repeatFloorF[a]; }
    return (fl > 0.0) ? s / fl : 0.0;
}

// 逐姿态的噪声申报 —— 测试【自己知道】它往均值里加了多少噪声 (addRawNoise 的 sigF/sigM),
// 所以它把加进去的量申报成"这个输入值的噪声尺度"。N = 1 = 没有做平均 (噪声是直接加在均值上
// 的), var = 申报值² -> σ_mean = 申报值。
//
// ⚠ 【申报的 floor² 是真实姿态内方差的 3 倍, 读 r 时必须知道】: addRawNoise 加的是
//   【均匀分布】uniform(±sigF), 它的【标准差是 sigF/√3】, 方差只有 sigF²/3; 而这里申报的
//   varF = sigF² —— 也就是申报值是真实方差的 3 倍, 而申报值的开方 (sigF) 并不是加进去那份
//   噪声的标准差。后果只有一个: 门限读到的 r̂ = Σσ_sys²/Σfloor² 比物理比值【小 3 倍】。
//   具体到 modelform_accepts_at_real_operating_point: 那里打印出来的 r≈3.02【不是物理比值】,
//   物理比值 ≈ 9 (那个用例给每一次访问加的离散是半宽 3·sigF 的均匀分布, sd = √3·sigF)。
//   【结论一个都不变】: 统计量与门限各自除以同一个申报值, 比值自洽, 判决不受影响; 但别拿
//   打印出来的 r 去对物理直觉 —— 它一律偏小 3 倍。
//
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

// 姿态级尺子 (力) —— 测试侧独立算一遍, 与 poseResidualRatioF 的分母同口径。
// 不去读那个比值再反推: 反推会把"分母算错"这类 bug 一起消掉。
static double yardFromFit(const PayloadCalibration::RawFit& f) {
    return sqrt((f.repeatSigmaF[0] * f.repeatSigmaF[0]
               + f.repeatSigmaF[1] * f.repeatSigmaF[1]
               + f.repeatSigmaF[2] * f.repeatSigmaF[2]) / 3.0);
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
    double poses[NQ][6], F[NQ][3], M[NQ][3];
    buildRepeatPoses(poses);
    synthRaw(A, bF, cs, bM, poses, NQ, F, M);
    unsigned seed = 7u;
    addRawNoise(F, M, NQ, 0.01, 0.0005, seed);

    PayloadCalibration::PoseNoise nz[NQ];
    declareNoise(NQ, 0.01, 0.0005, nz);

    // 接受 —— 且是【过了模型形式检验】才接受的 (不是"没做检验")。
    PayloadCalibration::RawFit fit;
    CHECK(PayloadCalibration::fitRaw(poses, F, M, NQ, fit, nz, &REP_PAIR, 1));
    CHECK(fit.modelFormChecked);
    CHECK(fit.modelFormStatus == PayloadCalibration::MODEL_FORM_OK);
    printf("[iso=%.4f 报告 (m=%.4f kg), 残差 %.4f N vs 尺子 %.4f N (姿态内噪声 %.4f N) -> 过] ",
           fit.isotropyRatio, fit.massScale, fit.rmsForceN, yardFromFit(fit), fit.noiseForceN);
    CHECK(fit.isotropyRatio > 3.5);              // 展布照实报出来 (4:1)
    CHECK(fit.chi2RepForceRatio < fit.chi2RepForceLimit);
    // 报告量与 decompose 是同一个数 (不是另算一份口径)
    PayloadCalibration::Decomp d;
    CHECK(PayloadCalibration::decompose(fit.A, d));
    CHECK(fabs(d.isotropyRatio - fit.isotropyRatio) < 1e-12);
    CHECK(fabs(d.m - fit.massScale) < 1e-12);
    PASS();
}

// ★ 评审的反例 (两轮修复都要拦住的正是它): 真值【完全各向同性】, 数据用历史上的【转置回归量】
//   生成 (就是那个让实机安静地解错两次的 bug), 0.02 N 噪声。
//   · 第一版的各向同性门限拿它没办法: 拟合 iso≈2.01, 而门限 1+3σ/m 随残差一起涨到 ≈2.59
//     → 【接受】, 返回 m≈0.4159 (真值 0.42)。这正是"安静地给出错答案"。
//   · 第二版的"残差 vs 姿态内噪声"χ² 能拦它, 但同一把尺子会错杀实机那条正确的解 (见下一条
//     用例): 姿态内噪声量不出姿态间的系统差。
//   本轮的判据 (残差 vs 【姿态间复现性】) 照样拒: 重复访问这一对只差姿态内噪声, 尺子很细,
//   而残差 0.46 N 差着 20 倍以上。
static void test_modelform_rejects_transposed_convention() {
    TEST(modelform_rejects_transposed_convention);
    double A[9];
    buildA(0.42, -1.0, 30.0, 0.0, ARB_U, ARB_V, A);   // 真值 = 恰好 0.42·diag(1,1,-1)·Rz(30°)
    const double bF[3] = {0.0, 0.0, 0.0};
    const double cs[3] = {0.0, 0.0, 0.08};
    const double bM[3] = {0.0, 0.0, 0.0};
    double poses[NQ][6], F[NQ][3], M[NQ][3];
    buildRepeatPoses(poses);
    synthRawWith(gravityTransposedAt, 0.0, A, bF, cs, bM, poses, NQ, F, M);
    unsigned seed = 20260919u;
    addRawNoise(F, M, NQ, 0.02, 0.001, seed);

    PayloadCalibration::PoseNoise nz[NQ];
    declareNoise(NQ, 0.02, 0.001, nz);

    // 线性层照样解得出来 —— 被拒的是模型形式, 不是拟合。
    PayloadCalibration::RawFit fit;
    CHECK(PayloadCalibration::fitRawLinear(poses, F, M, NQ, fit, nz, &REP_PAIR, 1));
    PayloadCalibration::Decomp d;
    CHECK(PayloadCalibration::decompose(fit.A, d));
    // 旧各向同性门限的自指关系照实算出来 (供对照, 不作断言依据): 残差涨 -> σ 涨 -> 门限涨得比 iso 快。
    const double oldLimit = 1.0 + 3.0 * maxSigmaA(fit) / d.m;
    printf("[转置数据: rmsF=%.4f N, 尺子=%.4f N, iso=%.3f, 旧门限 1+3s/m=%.3f -> 旧行为接受] ",
           fit.rmsForceN, yardFromFit(fit), d.isotropyRatio, oldLimit);
    CHECK(d.isotropyRatio < oldLimit);            // 旧门限确实放它过去 (这就是那个漏洞)
    CHECK(fit.rmsForceN > 0.1);                   // 残差远大于尺子 —— 数据与模型形式不符

    // 判据 (残差 vs 姿态间复现性): 拒。
    CHECK(!PayloadCalibration::fitRaw(poses, F, M, NQ, fit, nz, &REP_PAIR, 1));
    printf("[χ²rep/dof=%.1f (dof=%d, 限=%.2f) -> 拒; 最差 pose %d] ",
           fit.chi2RepForceRatio, fit.chi2DofForce,
           fit.chi2RepForceLimit, fit.worstPoseF + 1);
    CHECK(fit.modelFormStatus == PayloadCalibration::MODEL_FORM_OK);   // 尺子是齐的, 是判决拒的
    CHECK(!fit.modelFormChecked);                                      // 拒了就不算"验过"
    CHECK(fit.chi2RepForceRatio > fit.chi2RepForceLimit);
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
    double poses[NQ][6], F[NQ][3], M[NQ][3];
    buildRepeatPoses(poses);
    synthRaw(A, bF, cs, bM, poses, NQ, F, M);
    unsigned seed = 20260919u;
    addRawNoise(F, M, NQ, 0.02, 0.001, seed);

    PayloadCalibration::PoseNoise nz[NQ];
    declareNoise(NQ, 0.02, 0.001, nz);

    PayloadCalibration::RawFit fit;
    CHECK(PayloadCalibration::fitRawLinear(poses, F, M, NQ, fit, nz, &REP_PAIR, 1));
    printf("[正确约定: rmsF=%.4f N, 尺子=%.4f N, χ²rep/dof=%.2f (dof=%d), 力矩失拟=%.2f (dof=%d)] ",
           fit.rmsForceN, yardFromFit(fit), fit.chi2RepForceRatio, fit.chi2DofForce,
           fit.lackOfFitMomentRatio, fit.lackOfFitMomentDof);
    CHECK(PayloadCalibration::fitRaw(poses, F, M, NQ, fit, nz, &REP_PAIR, 1));
    CHECK(fit.modelFormChecked);
    CHECK(fit.momentFormChecked);            // 力矩那一半也真的验了 (不是悄悄跳过)
    // 正确模型下 χ²rep/dof 应落在 1 附近 —— 门限 1+3·sqrt(2/dof) 之内, 且不该小得离谱
    // (太小说明尺子被报大了, 那会让判据失去分辨力)。
    CHECK(fit.chi2RepForceRatio < fit.chi2RepForceLimit);
    CHECK(fit.chi2RepForceRatio > 0.2);
    printf("[A[0] err=%.1e] ", fabs(fit.A[0] - A[0]));
    CHECK(fabs(fit.A[0] - A[0]) < 4.0 * fit.paramSigma[0]);
    PASS();
}

// ★ 本轮修复的核心性质 (brief 点名要的那条): 【判决跟着姿态间复现性走, 不跟着姿态内噪声走】。
//   同一批数据、同一个重复姿态, 只把【第二次访问】的读数改掉:
//     (a) 第二次访问与第一次一致 (复现性好) -> 尺子只有姿态内噪声那么细 -> 残差远超尺子 -> 拒;
//     (b) 第二次访问漂了一截 (现场复现性就这么差) -> 尺子涨到与残差同量级 -> 过。
//   【两例里的姿态内噪声逐位相同、申报值也相同】, 所以这一对直接证否了"拿姿态内噪声当尺子":
//   那把尺子对两例给出同一个判决, 而"残差有没有超出复现性"这个问题的答案两例不同。
//   (取代 modelform_gate_tracks_measured_noise: 那条证明的是"门限随实测噪声走", 但它量的
//    噪声是【姿态内】的 —— 尺度确实来自数据, 只是量错了对象。)
static void test_modelform_gate_tracks_repeat_reproducibility() {
    TEST(modelform_gate_tracks_repeat_reproducibility);
    double A[9];
    buildA(0.42, -1.0, 30.0, 0.0, ARB_U, ARB_V, A);
    const double bF[3] = {1.0, -0.5, 0.2};
    const double cs[3] = {0.005, -0.008, 0.061};
    const double bM[3] = {0.01, -0.01, 0.005};
    double poses[NQ][6], F[NQ][3], M[NQ][3];
    buildRepeatPoses(poses);
    synthRaw(A, bF, cs, bM, poses, NQ, F, M);
    // 姿态相关的模型误差: g 的二次项, 线性模型 (b + A·g) 【吸收不掉】—— 余下的就是"形式错"。
    // 【关键】它是位姿的函数: 同一个姿态两次访问, 这一项一模一样, 所以它【不抬高尺子】——
    // 姿态相关的误差正是靠这一点与"复现性差"区分开的。
    for (int i = 0; i < NQ; i++) {
        double g[3];
        gravitySensorRefAt(poses[i], 0.0, g);
        const double u = (g[0] * g[1]) / (G * G);
        F[i][0] += 1.40 * u; F[i][1] += -1.12 * u; F[i][2] += 1.68 * u;
    }
    unsigned seed = 20260919u;
    addRawNoise(F, M, NQ, 0.02, 0.001, seed);
    PayloadCalibration::PoseNoise nz[NQ];
    declareNoise(NQ, 0.02, 0.001, nz);

    PayloadCalibration::RawFit fit;

    // (a) 复现性好: 第二次访问与第一次一致 (只差各自那份姿态内噪声) -> 尺子细 -> 拒
    CHECK(PayloadCalibration::fitRawLinear(poses, F, M, NQ, fit, nz, &REP_PAIR, 1));
    printf("[(a) 尺子=%.4f N, 残差=%.4f N, χ²rep/dof=%.2f (限 %.2f) -> 拒] ",
           yardFromFit(fit), fit.rmsForceN, fit.chi2RepForceRatio,
           fit.chi2RepForceLimit);
    CHECK(fit.chi2RepForceRatio > fit.chi2RepForceLimit);
    CHECK(!PayloadCalibration::fitRaw(poses, F, M, NQ, fit, nz, &REP_PAIR, 1));
    CHECK(fit.modelFormStatus == PayloadCalibration::MODEL_FORM_OK);   // 尺子齐, 是判决拒的
    CHECK(!fit.modelFormChecked);
    const double residualKept = fit.rmsForceN;

    // (b) 复现性差: 只是第二次访问漂了一截 (力与力矩一起漂 = 现场级的漂移, 不是单通道坏)
    for (int a = 0; a < 3; a++) {
        F[NP][a] += 0.50;
        M[NP][a] += 0.020;
    }
    CHECK(PayloadCalibration::fitRawLinear(poses, F, M, NQ, fit, nz, &REP_PAIR, 1));
    printf("[(b) 尺子=%.4f N, 残差=%.4f N, χ²rep/dof=%.2f (限 %.2f) -> 过] ",
           yardFromFit(fit), fit.rmsForceN, fit.chi2RepForceRatio,
           fit.chi2RepForceLimit);
    CHECK(fit.chi2RepForceRatio < fit.chi2RepForceLimit);
    CHECK(PayloadCalibration::fitRaw(poses, F, M, NQ, fit, nz, &REP_PAIR, 1));
    CHECK(fit.modelFormChecked);
    // 【同一个残差量级, 判决相反】—— 差别只在尺子。这一行是这条用例的立身之本。
    printf("[残差量级 (a)=%.4f N / (b)=%.4f N, 判决 拒 -> 过] ", residualKept, fit.rmsForceN);
    CHECK(fit.rmsForceN > 0.5 * residualKept);
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
    double poses[NQ][6], F[NQ][3], M[NQ][3];
    buildRepeatPoses(poses);
    synthRaw(A, bF, cs, bM, poses, NQ, F, M);
    for (int i = 0; i < NQ; i++) {
        double g[3], w[3];
        gravitySensorRefAt(poses[i], 0.0, g);            // 测试侧自己算, 不调被测函数
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
    addRawNoise(F, M, NQ, 0.02, 0.001, seed);
    PayloadCalibration::PoseNoise nz[NQ];
    declareNoise(NQ, 0.02, 0.001, nz);

    PayloadCalibration::RawFit fit;
    CHECK(PayloadCalibration::fitRawLinear(poses, F, M, NQ, fit, nz, &REP_PAIR, 1));
    // ★ 2026-09-19 门限重标定【放宽了】力矩那一刀: 这条错模型的余量从 26.27x 掉到 9.09x
    //   (改动前的实测: 155.53 / 5.922; 改动后: 155.53 / 17.11)。⚠ 9.09x 已经【低于】模块
    //   头文件里那条 ~10x 的底线 —— 这是本次改动真实的代价, 记在这里与本轮的验收报告里,
    //   不藏。断言只钉到 5x: 它拦的是"下一轮再把门放宽一倍多", 而不是把观察值当门限
    //   (把断言的数贴着观察值定, 等于把它变成第二次冻结金标)。
    const double margin = fit.lackOfFitMomentRatio / fit.lackOfFitMomentLimit;
    printf("[力通道 χ²rep/dof=%.2f (合格), 力矩失拟=%.2f (dof=%d, 限=%.2f, 余量 %.2fx) -> 拒] ",
           fit.chi2RepForceRatio, fit.lackOfFitMomentRatio, fit.lackOfFitMomentDof,
           fit.lackOfFitMomentLimit, margin);
    // 力通道没问题 (它是自由拟合, 数据也确实符合) —— 拒的理由必须来自力矩那一条
    CHECK(fit.chi2RepForceRatio < fit.chi2RepForceLimit);
    CHECK(fit.lackOfFitMomentRatio > fit.lackOfFitMomentLimit);
    CHECK(margin > 5.0);          // 放宽不等于放过去 (见上: 观察值 9.09x, 底线 ~10x 在报告里记着)
    CHECK(!PayloadCalibration::fitRaw(poses, F, M, NQ, fit, nz, &REP_PAIR, 1));

    // ===== 2026-09-20: MODEL_FORM_FORCE_ONLY —— 同一批数据, 力矩没过但【整体放行】 =====
    //
    // 必须用【上面同一批数据】: 上面那个 CHECK 证明旧策略拒, 这里证明新策略收 —— 两者的差别
    // 就只能来自策略本身, 不可能来自数据。
    PayloadCalibration::RawFit fitFO;
    CHECK(PayloadCalibration::fitRaw(poses, F, M, NQ, fitFO, nz, &REP_PAIR, 1,
                                     PayloadCalibration::MODEL_FORM_FORCE_ONLY));
    // 力通道确实过了 —— 否则这条用例证明的不是"力矩不拦", 而是"力通道也放水了"。
    CHECK(fitFO.modelFormChecked);
    // ⚠ 【最要紧的一条】: 力矩那一半没过, 所以 momentFormChecked 必须【保持 false】。
    //   它的语义是"做了【且通过】"。若这里为真, 下游 (main.cpp 的通过分支) 会把"力矩没验过"
    //   读成"验过了" —— 那正是这个标志当初被引入时要消灭的那种假通过。
    CHECK(!fitFO.momentFormChecked);
    // 而"检验【做了】"是另一件事: dof > 0 说明它真的跑了, 只是没过 —— 不是被跳过。
    CHECK(fitFO.lackOfFitMomentDof > 0);
    CHECK(fitFO.lackOfFitMomentRatio > fitFO.lackOfFitMomentLimit);
    // 力通道的判据一个字没动: 新策略下它的统计量与门限与旧策略逐位相同。
    CHECK(fitFO.chi2RepForceRatio == fit.chi2RepForceRatio);
    CHECK(fitFO.chi2RepForceLimit == fit.chi2RepForceLimit);
    PASS();
}

// 实机那条正确的解 (参考 A, iso = 1.0655, 6.5% 的物理非正交) 必须过 —— 而且现在是靠
// "残差 vs 姿态间复现性"过的, 不再靠"iso 撞上一条随残差放松的门限", 也不再靠姿态内噪声。
// 用参考 A 的形状 + 与实机同量级的噪声造数据。
static void test_realistic_spread_accepted_with_measured_noise() {
    TEST(realistic_spread_accepted_with_measured_noise);
    const double ref[9] = { 0.36456, 0.21055, -0.00003,
                           -0.21828, 0.37331,  0.00298,
                           -0.01155, -0.01493, -0.41390 };
    const double bF[3] = {-18.55, -2.44, 0.82};
    const double cs[3] = {0.0006, -0.0005, 0.0545};
    const double bM[3] = {-0.16, 0.57, -0.02};
    double poses[NQ][6], F[NQ][3], M[NQ][3];
    buildRepeatPoses(poses);
    synthRaw(ref, bF, cs, bM, poses, NQ, F, M);
    unsigned seed = 1304u;
    addRawNoise(F, M, NQ, 0.15, 0.008, seed);

    PayloadCalibration::PoseNoise nz[NQ];
    declareNoise(NQ, 0.15, 0.008, nz);

    PayloadCalibration::RawFit fit;
    CHECK(PayloadCalibration::fitRaw(poses, F, M, NQ, fit, nz, &REP_PAIR, 1));   // 必须放行
    CHECK(fit.modelFormChecked);
    printf("[iso=%.4f (报告), rmsF=%.4f N, 尺子=%.4f N, χ²rep/dof=%.2f, 力矩失拟=%.2f,"
           " m=%.4f kg, parity=%+.0f] ",
           fit.isotropyRatio, fit.rmsForceN, yardFromFit(fit), fit.chi2RepForceRatio,
           fit.lackOfFitMomentRatio, fit.massScale, fit.parity);
    CHECK(fit.isotropyRatio > 1.02);               // 确实带着实机那种量级的展布 (不是碰巧正交)
    CHECK(fabs(fit.parity - (-1.0)) < 1e-12);      // 参考 A 的手系是负的
    CHECK(fabs(fit.massScale - 0.4224) < 0.06);
    PASS();
}

// ★★★ 第三次修复的【核心用例】(brief I4 点名要的那一条): 实机工况点。
// 前面那些"接受"用例全都落在 r ≈ 0 —— 那里【没有】额外的姿态间离散, 而申报的 floor² 又比
// 真实姿态内方差大 3 倍 (见 declareNoise), 重复的那两行只带自己那份姿态内噪声, 于是
// sigma_sys 被夹到 0、尺子取到它的下限 (floor), 统计量就是残差/floor。那种条件下【过是必定的】,
// 门根本不用判 —— 它验不了任何东西。
// 这一条把数据挪到实机真正所在的工况: 每一次【访问】另有一份姿态间离散 s, 取 s = 根号3·sigma_in
// (即申报口径 r̂ ≈ 3), 残差与重复对的差值【同时】被抬高。模型形式仍然是对的 —— 必须被接受。
// 【旧门限下这一条是红的】: 旧门限 1 + 3·sqrt(2/dof) 压在零分布的中位数附近, 这个统计量
// 越过去是常事。下面把这一点直接写成断言 —— 那段历史只有钉在这儿才不会重演。
//
// ⚠ 【打印出来的 r 不是物理比值】: 这里是【申报口径】的 r̂ = Σσ_sys²/Σfloor², 而 declareNoise
//   申报的 floor² 是真实姿态内方差的 3 倍, 所以物理比值约是它的 3 倍 —— 这一段 r̂≈3.02 对应的
//   物理比值 ≈ 9 (3.0·sigF 那份均匀离散 sd = √3·sigF, 姿态内噪声 sd = sigF/√3)。
//   断言用的一律是 r̂; 报告里那张"实测冤枉率"表列的也是 r̂。两者别混着读。
static void test_modelform_accepts_at_real_operating_point() {
    TEST(modelform_accepts_at_real_operating_point);
    double A[9];
    buildA(0.42, -1.0, 30.0, 0.0, ARB_U, ARB_V, A);
    const double bF[3] = {1.0, -0.5, 0.2};
    const double cs[3] = {0.005, -0.008, 0.061};
    const double bM[3] = {0.01, -0.01, 0.005};
    const double sigF = 0.02, sigM = 0.001;
    double poses[NQ][6], F[NQ][3], M[NQ][3];
    buildRepeatPoses(poses);
    synthRaw(A, bF, cs, bM, poses, NQ, F, M);
    // 种子不是随便挑的: 这一条要同时满足两件事 —— r 落在实机那个量级, 且【统计量越过了旧门限】
    // (否则它就退回成"在极端的另一端"那条空用例了)。扫过一批种子之后取 r 最接近 3 的那个。
    unsigned seed = 6074282u;
    addRawNoise(F, M, NQ, sigF, sigM, seed);
    addVisitScatter(F, M, NQ, 3.0 * sigF, 3.0 * sigM, seed);   // 均匀分布: 半宽 3σ -> sd = 根号3·σ

    PayloadCalibration::PoseNoise nz[NQ];
    declareNoise(NQ, sigF, sigM, nz);

    PayloadCalibration::RawFit fit;
    CHECK(PayloadCalibration::fitRawLinear(poses, F, M, NQ, fit, nz, &REP_PAIR, 1));
    const double oldLimit = 1.0 + 3.0 * sqrt(2.0 / fit.chi2DofForce);
    printf("[实机工况 r=%.2f: 残差 %.4f N, 尺子 %.4f N, 姿态内噪声 %.4f N,"
           " 统计量 %.3f (旧门限 %.3f, 新门限 %.3f, dof=%d) -> 过] ",
           rhoFromFit(fit), fit.rmsForceN, yardFromFit(fit), fit.noiseForceN,
           fit.chi2RepForceRatio, oldLimit, fit.chi2RepForceLimit, fit.chi2DofForce);
    CHECK(rhoFromFit(fit) > 1.5);                            // 确实带着姿态间离散 (不是 r≈0)
    CHECK(fit.chi2RepForceRatio > oldLimit);                 // ★ 旧门限下【红】—— 本用例的由来
    CHECK(fit.chi2RepForceRatio < fit.chi2RepForceLimit);    // 新门限下过
    CHECK(PayloadCalibration::fitRaw(poses, F, M, NQ, fit, nz, &REP_PAIR, 1));
    CHECK(fit.modelFormChecked);
    CHECK(fit.momentFormChecked);
    PASS();
}

// ★ 尺子可以被【追加】(brief I1): 按 'r' 多采几对, 求解侧把它们【池化】成一把尺子, 尺子的
// 自由度 = 对数, 门限的"自由度折扣"跟着收紧。这条钉两件事:
//   (a) 池化真的用到了每一对 —— 与测试侧独立算出来的逐对平均逐位对上;
//   (b) 对数真的进了门限 (对数越多门限越紧, 但永不紧过 base)。
// (从前按 'r' 是【覆盖】, 于是"判决贴着线时补一对"这个补救是空的: 补进来还是同一个 1 自由度
//  估量, 尺子的离散一点没降。)
// 采集侧的【重复对登记规则】(RepeatPairRegistry) —— 协议: 原地复采, 一对 = (上一笔, 这一笔)。
// 这是本次协议变更里唯一有逻辑的一处, 所以钉在这里。上层的 BiasCheck 是胶水 + 打印, 不单测。
static void test_repeat_pair_registration() {
    TEST(repeat_pair_registration);

    // (a) 正常: 上一笔 3, 这一笔 4 -> 一对 (3, 4)。first 不再是常量 0 (旧协议"恒为 pose 1")。
    int f = -99, s = -99;
    CHECK(RepeatPairRegistry::registerPair(3, 4, 0, 8, &f, &s) == RepeatPairRegistry::OK);
    CHECK(f == 3);
    CHECK(s == 4);

    // (b) 最后一格可用 (n = max-1); 满了就拒, 且【不动】输出 —— 否则调用方会存下一个假下标。
    f = -99; s = -99;
    CHECK(RepeatPairRegistry::registerPair(6, 7, 7, 8, &f, &s) == RepeatPairRegistry::OK);
    CHECK(f == 6 && s == 7);
    f = -99; s = -99;
    CHECK(RepeatPairRegistry::registerPair(6, 7, 8, 8, &f, &s) == RepeatPairRegistry::LIMIT);
    CHECK(f == -99 && s == -99);

    // (c) ★ 没有上一笔: 'r' 在任何 SPACE 之前按下 (prev = -1) -> 明确拒绝, 不登记 ——
    //     留一对假尺子比少一对坏得多。
    f = -99; s = -99;
    CHECK(RepeatPairRegistry::registerPair(-1, 0, 0, 8, &f, &s) == RepeatPairRegistry::NO_PREVIOUS);
    CHECK(f == -99 && s == -99);

    // (d) 同一笔与自己配对 (prev >= cur) 走不到, 但同样归 NO_PREVIOUS ——
    //     这一对会给出 d = 0 的假尺子, 比没有尺子更能骗过门限。
    f = -99; s = -99;
    CHECK(RepeatPairRegistry::registerPair(4, 4, 0, 8, &f, &s) == RepeatPairRegistry::NO_PREVIOUS);
    CHECK(f == -99 && s == -99);

    // (e) 连采三笔原地不动 -> 两对【首尾相接】(3,4) 与 (4,5): 允许 (换姿态前连着按两次 'r'
    //     就是这个形状)。这里钉的是"不把它误判成同一笔而拒掉"。
    f = -99; s = -99;
    CHECK(RepeatPairRegistry::registerPair(4, 5, 1, 8, &f, &s) == RepeatPairRegistry::OK);
    CHECK(f == 4 && s == 5);

    PASS();
}

static void test_repeat_pairs_pool_and_carry_their_dof() {
    TEST(repeat_pairs_pool_and_carry_their_dof);
    static const int PAIRS = 3;
    double A[9];
    buildA(0.42, -1.0, 30.0, 0.0, ARB_U, ARB_V, A);
    const double bF[3] = {1.0, -0.5, 0.2};
    const double cs[3] = {0.005, -0.008, 0.061};
    const double bM[3] = {0.01, -0.01, 0.005};
    const double sigF = 0.02, sigM = 0.001;
    double poses[NQR][6], F[NQR][3], M[NQR][3];
    buildRepeatPosesR(poses, PAIRS);
    synthRaw(A, bF, cs, bM, poses, NQR, F, M);
    unsigned seed = 777u;
    addRawNoise(F, M, NQR, sigF, sigM, seed);
    addVisitScatter(F, M, NQR, 2.0 * sigF, 2.0 * sigM, seed);

    PayloadCalibration::PoseNoise nz[NQR];
    declareNoise(NQR, sigF, sigM, nz);

    PayloadCalibration::RawFit fit;
    CHECK(PayloadCalibration::fitRawLinear(poses, F, M, NQR, fit, nz, REP_PAIRS, PAIRS));
    CHECK(fit.repeatPairCount == PAIRS);
    // (a) 逐对独立复算 sigma_sys^2, 再取平均 —— 必须与实现池化出来的值一致。
    //     (v = 申报的均值方差; excess = max((d^2 - 2v)/2, 0); sigma_sys^2 = mean excess)
    const double v = sigF * sigF;
    for (int a = 0; a < 3; a++) {
        double acc = 0.0;
        for (int j = 0; j < PAIRS; j++) {
            const double d = F[REP_PAIRS[j].second][a] - F[REP_PAIRS[j].first][a];
            const double ex = 0.5 * (d * d - 2.0 * v);
            if (ex > 0.0) acc += ex;
        }
        CHECK(fabs(fit.repeatSysF[a] - acc / PAIRS) < 1e-12);
        CHECK(fabs(fit.repeatFloorF[a] - v) < 1e-12);
    }
    printf("[%d 对池化: 尺子 %.4f N (r=%.2f), 门限 %.4f (dof=%d); 1 对时门限 %.4f] ",
           PAIRS, yardFromFit(fit), rhoFromFit(fit), fit.chi2RepForceLimit, fit.chi2DofForce,
           PayloadCalibration::modelFormLimit(fit.chi2DofForce, 1, fit.repeatSysF, fit.repeatFloorF));
    // (b) 同一批数据、同一个 r, 只改【对数】: 对数多的门限更紧 (自由度真的用上了),
    //     但永不紧过"尺子完全可信"的 base。
    const double limBase = PayloadCalibration::modelFormLimit(fit.chi2DofForce, 0,
                                                              fit.repeatSysF, fit.repeatFloorF);
    const double lim1 = PayloadCalibration::modelFormLimit(fit.chi2DofForce, 1,
                                                           fit.repeatSysF, fit.repeatFloorF);
    const double lim3 = PayloadCalibration::modelFormLimit(fit.chi2DofForce, 3,
                                                           fit.repeatSysF, fit.repeatFloorF);
    CHECK(lim1 > lim3);
    CHECK(lim3 >= limBase);
    // base = chi2(dof,0.9999)/dof (本用例 dof=9 -> 3.747), 比旧式 1 + 3·sqrt(2/dof) (dof 9 时
    // 2.414) 高一截 —— 置信水平从 0.997 调到 0.9999 的结果, 不是回归。1.9 这条线只钉"base
    // 仍然是个 O(1~4) 的数", 别把它读成"与旧式等值"。
    CHECK(limBase > 1.9);
    // 尺子有 3 个自由度时判决照样通过 (门限放宽的幅度小了, 但仍然容得下正常散布)
    CHECK(PayloadCalibration::fitRaw(poses, F, M, NQR, fit, nz, REP_PAIRS, PAIRS));
    CHECK(fit.modelFormChecked);
    PASS();
}

// ★ 门限本身: 直接按定义算出来的数 (与 chi2 分布表可查的值逐位对照)。
// 本仓库栽过的两类都在这条上钉住了:
//   · 门限随【残差】一起放松 (自指) —— 现在门限只由 dof 与尺子的两个分量决定, 与残差无关;
//   · 门限压在零分布【中位数】上 (分母只有 1 自由度时的病) —— 现在分母按自己的自由度取
//     单侧置信下界: 对数是 1 时折扣最大, 对数越多折扣越小, 但永远 >= 1 (只会更宽, 不会更紧)。
static void test_modelform_limit_is_a_chi2_quantile_times_a_yardstick_discount() {
    TEST(modelform_limit_is_a_chi2_quantile_times_a_yardstick_discount);
    const double sys2[3]   = {1.0, 1.0, 1.0};     // sigma_sys^2 = floor^2 -> r = 1
    const double floor2[3] = {1.0, 1.0, 1.0};
    // base = chi2(dof, 0.9999)/dof, 参考值 (chi2 分布表)。⚠ 置信水平从 0.997 调到 0.9999 之后
    // 这些数【整体变大】—— 方向别搞反: 两个 chi2 分位数都随 α 单调增, **调大 α 才是放宽门限**。
    CHECK(fabs(PayloadCalibration::modelFormLimit(6.0, 0, sys2, floor2)  - 4.6427235393) < 1e-6);
    CHECK(fabs(PayloadCalibration::modelFormLimit(9.0, 0, sys2, floor2)  - 3.7466609377) < 1e-6);
    CHECK(fabs(PayloadCalibration::modelFormLimit(12.0, 0, sys2, floor2) - 3.2612003235) < 1e-6);
    CHECK(fabs(PayloadCalibration::modelFormLimit(18.0, 0, sys2, floor2) - 2.7327441373) < 1e-6);
    // r = 1, 对数 = 1: 折扣 = (1+1)/(1 + (1/chi2(1,0.9999))·1), chi2(1,0.9999) = 15.1379
    CHECK(fabs(PayloadCalibration::modelFormLimit(12.0, 1, sys2, floor2) - 6.1182040929) < 1e-6);
    // r = 1, 对数 = 4: chi2(4,0.9999) = 23.5127
    CHECK(fabs(PayloadCalibration::modelFormLimit(12.0, 4, sys2, floor2) - 5.5741272191) < 1e-6);
    // 【尺子说复现性差多少, 门限就放宽多少】: r 越大折扣越大
    const double s2big[3] = {9.0, 9.0, 9.0};
    CHECK(PayloadCalibration::modelFormLimit(12.0, 1, s2big, floor2)
          > PayloadCalibration::modelFormLimit(12.0, 1, sys2, floor2));
    // 没有量到复现性差 -> 不打折, 就是 base
    const double zero[3] = {0.0, 0.0, 0.0};
    CHECK(fabs(PayloadCalibration::modelFormLimit(12.0, 1, zero, floor2) - 3.2612003235) < 1e-6);
    printf("[门限 = chi2(dof,0.9999)/dof × (1+r)/(1+(R/chi2(R,0.9999))·r): dof=12 base=%.4f,"
           " r=1 时 1 对 %.4f / 4 对 %.4f] ",
           PayloadCalibration::modelFormLimit(12.0, 0, sys2, floor2),
           PayloadCalibration::modelFormLimit(12.0, 1, sys2, floor2),
           PayloadCalibration::modelFormLimit(12.0, 4, sys2, floor2));
    PASS();
}

// ★ 六个通道【全】冻住: 从前会掉进"没有逐姿态噪声估计"那一支, 于是给出一条【错的】建议
// ("把采集的样本方差传进来") —— 而调用方本来就传了。它和"某一条通道冻住"是同一种病
// (传感器/接线/取数), 该走同一条消息: 查硬件, 重采没用。
// (从前那条用例只钉了【部分】冻住: 力 z 通道冻住而力矩通道活着。)
static void test_dead_channel_covers_all_six_frozen() {
    TEST(dead_channel_covers_all_six_frozen);
    double A[9];
    buildA(0.42, -1.0, 30.0, 0.0, ARB_U, ARB_V, A);
    const double bF[3] = {0.0, 0.0, 0.0};
    const double cs[3] = {0.0, 0.0, 0.08};
    const double bM[3] = {0.0, 0.0, 0.0};
    double poses[NQ][6], F[NQ][3], M[NQ][3];
    buildRepeatPoses(poses);
    synthRawWith(gravityTransposedAt, 0.0, A, bF, cs, bM, poses, NQ, F, M);
    unsigned seed = 20260919u;
    addRawNoise(F, M, NQ, 0.02, 0.001, seed);

    PayloadCalibration::PoseNoise allDead[NQ];
    declareNoise(NQ, 0.02, 0.001, allDead);
    for (int i = 0; i < NQ; i++)
        for (int a = 0; a < 3; a++) { allDead[i].varF[a] = 0.0; allDead[i].varM[a] = 0.0; }

    PayloadCalibration::RawFit fit;
    CHECK(!PayloadCalibration::fitRaw(poses, F, M, NQ, fit, allDead, &REP_PAIR, 1));
    // 【是"通道冻住", 不是"没有噪声估计"】—— 这条就是本用例的全部意义。
    CHECK(fit.modelFormStatus == PayloadCalibration::MODEL_FORM_DEAD_CHANNEL);
    printf("[六个通道全冻住 -> DEAD_CHANNEL (不是 NO_NOISE); 建议查硬件而不是重采] ");
    PASS();
}

// ★ 本轮修复的第二条契约 (brief 点名): 【模型形式没被检验过, 就不给参数】。
// 上一版的行为是"没验过也照给, 只把 modelFormChecked 置成 false" —— 调用方只要忘了读那个
// 标志, 参数就落在一个【从未被检验过形式】的模型上, 而模型形式错正是本项目栽得最惨的那一件
// 事 (安静地解错)。现在改成: 尺子不齐 -> 返回 false。想要参数只能逐字写出那个刺眼的令牌。
// 同时钉住两种"尺子不齐"是【分开报】的: 缺噪声 / 缺重复对 / 通道冻住。
static void test_modelform_unverified_refused_not_accepted() {
    TEST(modelform_unverified_refused_not_accepted);
    double A[9];
    buildA(0.42, -1.0, 30.0, 0.0, ARB_U, ARB_V, A);
    const double bF[3] = {0.0, 0.0, 0.0};
    const double cs[3] = {0.0, 0.0, 0.08};
    const double bM[3] = {0.0, 0.0, 0.0};
    double poses[NQ][6], F[NQ][3], M[NQ][3];
    buildRepeatPoses(poses);
    synthRawWith(gravityTransposedAt, 0.0, A, bF, cs, bM, poses, NQ, F, M);
    unsigned seed = 20260919u;
    addRawNoise(F, M, NQ, 0.02, 0.001, seed);

    PayloadCalibration::RawFit fit;
    PayloadCalibration::PoseNoise nz[NQ];
    declareNoise(NQ, 0.02, 0.001, nz);

    // (a) 什么都没给 (旧的 5 参形式): 线性层照解, 但【拒绝给参数】。
    CHECK(PayloadCalibration::fitRawLinear(poses, F, M, NQ, fit));
    CHECK(fit.modelFormStatus == PayloadCalibration::MODEL_FORM_NO_NOISE);
    CHECK(!fit.modelFormChecked);
    CHECK(!PayloadCalibration::fitRaw(poses, F, M, NQ, fit));
    CHECK(fit.poseResidualCount == NQ);            // 逐姿态残差照算照报 (false 之后仍可诊断)

    // (b) 有噪声但【没有重复姿态对】—— 与 (a) 是不同的原因, 必须分开报。
    CHECK(!PayloadCalibration::fitRaw(poses, F, M, NQ, fit, nz));
    CHECK(fit.modelFormStatus == PayloadCalibration::MODEL_FORM_NO_REPEAT);
    CHECK(fit.chi2RepForceRatio == 0.0 && fit.repeatFirst < 0);

    // (b2) 【个别一笔】方差为 0 (只有 pose 3 的 y 通道) —— 通道没死, 是那一笔没采到。
    //      这与 (c) 的"整条通道死了"是两回事, 必须分开报。
    PayloadCalibration::PoseNoise hole[NQ];
    declareNoise(NQ, 0.02, 0.001, hole);
    hole[2].varF[1] = 0.0;
    CHECK(!PayloadCalibration::fitRaw(poses, F, M, NQ, fit, hole, &REP_PAIR, 1));
    CHECK(fit.modelFormStatus == PayloadCalibration::MODEL_FORM_NOISE_HOLES);
    CHECK(fit.chi2ForceRatio == 0.0);     // 尺子不完整 -> 那个统计量【不算】(0 做分母会出 inf/NaN)

    // (c) 力通道【整批冻住】(方差恒为 0) 而力矩通道是活的 —— 本机的真实故障模式, 又是另一类。
    PayloadCalibration::PoseNoise dead[NQ];
    declareNoise(NQ, 0.02, 0.001, dead);
    for (int i = 0; i < NQ; i++) dead[i].varF[2] = 0.0;      // z 通道冻住
    CHECK(!PayloadCalibration::fitRaw(poses, F, M, NQ, fit, dead, &REP_PAIR, 1));
    CHECK(fit.modelFormStatus == PayloadCalibration::MODEL_FORM_DEAD_CHANNEL);

    // (d) 只有逐字写出那个令牌, 才拿得到参数 —— 而且 modelFormChecked 仍然是 false,
    //     【不会】被伪装成"验过了"。
    CHECK(PayloadCalibration::fitRaw(poses, F, M, NQ, fit, dead, &REP_PAIR, 1,
                                     PayloadCalibration::I_ACCEPT_UNVERIFIED_MODEL_FORM));
    CHECK(!fit.modelFormChecked);
    CHECK(fit.modelFormStatus == PayloadCalibration::MODEL_FORM_DEAD_CHANNEL);
    printf("[无尺子一律拒给参数: 无噪声/无重复对/个别笔没采到/通道冻住 四类分开报; "
           "只有显式令牌才给, 且 modelFormChecked 仍为 false] ");
    PASS();
}

// 逐姿态残差 (spec §3 表格第 4 行): 坏了【哪一个】姿态要指得出来 —— 这是"一个坏姿态"与
// "整体形式错"唯一的区分手段, 而两者的处置完全不同。
//
// 【为什么这条用 9 个姿态 + 重复访问, 而不是上面那 6 个】: 每通道的力模型是 4 个参数
// (1, gx, gy, gz), 7 行时杠杆 h ≈ 4/7 = 0.57 —— 单个坏姿态的偏差只有 43% 留在它自己身上,
// 其余被最小二乘摊到别的姿态上, 最差姿态会指到【别人】身上 (实测: 坏的是第 4 个, 指出来的是
// 第 6 个)。这是最小二乘的性质, 不是实现的错; 姿态一多 (9 个) 杠杆降到 0.44, 指向就准了。
// 生产路径采 6~8 个姿态, 所以这条限制要照实写在这里, 别让表看起来比它实际能做到的更可靠。
static void test_pose_residuals_mark_the_worst_pose() {
    TEST(pose_residuals_mark_the_worst_pose);
    static const int NR = 9;
    static const int NBAD = 3;
    const double posesR[NR][6] = {
        {300, 100, 40,    0,   0,   0},
        {300, 100, 40,   40,   0,   0},
        {300, 100, 40,  -35,  15,   0},
        {300, 100, 40,    0,  60,  25},
        {300, 100, 40,   25, -50, -30},
        {300, 100, 40,  -20,  35,  55},
        {300, 100, 40,   70,   0, 120},
        {300, 100, 40,  -60,  45, -75},
        {300, 100, 40,   35, -70, 160}
    };
    const int N = NR + 1;
    double poses[NR + 1][6], F[NR + 1][3], M[NR + 1][3];
    for (int i = 0; i < NR; i++)
        for (int a = 0; a < 6; a++) poses[i][a] = posesR[i][a];
    for (int a = 0; a < 6; a++) poses[NR][a] = posesR[0][a];      // 重复访问

    double A[9];
    buildA(0.42, -1.0, 30.0, 0.0, ARB_U, ARB_V, A);
    const double bF[3] = {1.0, -0.5, 0.2};
    const double cs[3] = {0.005, -0.008, 0.061};
    const double bM[3] = {0.01, -0.01, 0.005};
    synthRaw(A, bF, cs, bM, poses, N, F, M);
    unsigned seed = 20260919u;
    addRawNoise(F, M, N, 0.02, 0.001, seed);
    // 只把第 NBAD 个姿态弄坏 (三个力分量一起偏) —— 其余姿态与两次重复访问都干净
    for (int a = 0; a < 3; a++) F[NBAD][a] += 0.60;
    const PayloadCalibration::RepeatPair rep = {0, NR};

    PayloadCalibration::PoseNoise nz[NR + 1];
    declareNoise(N, 0.02, 0.001, nz);

    PayloadCalibration::RawFit fit;
    CHECK(PayloadCalibration::fitRawLinear(poses, F, M, N, fit, nz, &rep, 1));
    double restMax = 0.0;
    for (int i = 0; i < fit.poseResidualCount; i++)
        if (i != fit.worstPoseF && fit.poseResidualF[i] > restMax) restMax = fit.poseResidualF[i];
    printf("[%d 个姿态, 第 %d 个坏: 最差 = pose %d (残差 %.4f N = 尺子的 %.2f 倍);"
           " 其余姿态最大 %.4f N] ",
           N, NBAD + 1, fit.worstPoseF + 1, fit.poseResidualF[fit.worstPoseF],
           fit.poseResidualRatioF[fit.worstPoseF], restMax);
    CHECK(fit.worstPoseF == NBAD);                    // 指得出是哪一个
    CHECK(fit.poseResidualCount == N);
    // 与其余姿态拉开量级。门限取 1.8 是【照着实测写的】, 不是"应该有多大": 单个坏姿态的偏差
    // 有一部分被最小二乘摊到别的姿态上 (杠杆 h = 参数数/方程数), 所以分离度只有 2 倍上下,
    // 姿态越少越糊 (7 行时最差会指错人, 见上面那段说明)。把它当"最突出的那个"用, 别当铁证。
    CHECK(fit.poseResidualF[NBAD] > 1.8 * restMax);
    // 一个坏姿态足以让整体判决拒 —— 表里能看到"只有它高", 这就是可行动的信息。
    CHECK(!PayloadCalibration::fitRaw(poses, F, M, N, fit, nz, &rep, 1));
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

// =====================================================================================
// ★ 实机回归 (Task 2): 用 7 个【真实姿态】的采集文件重放, 钉住整条求解链
//   (fitRaw 的线性解 + decompose 的物理量)。
//
// 【为什么需要它】: 本文件其余每一条用例的数据都是【合成】的 —— 生成器与估计器虽然刻意不
//   共用代码, 却终究出自同一套约定。夹具 fixtures/calib_poses_2026-09-19.txt 是实机采集的
//   原件 (main.cpp 的 BiasCheck 落盘后【冻结进仓库】的只读副本), 它【早于】本轮的模型形式
//   协议存在 (没有重复姿态对, 也没有逐姿态方差), 是手上唯一一份"我们没有参与制造"的数据。
//   把它拟合出的 12 + 6 个参数钉死, 是"实现真的在解实机上那件事"的硬证据。
//
// ⚠ 【这些数是金标, 不是快照】: 它们由控制器用 Python 独立算出、经两轮独立复算确认
//   (task-2-brief 的表)。**不得为了迁就将来某次改动而修改** —— 对不上就是实现回归了,
//   要查的是实现, 不是这张表。谁在这里改数, 这条用例就死了 (从此对什么都不敏感)。
//   每段尾注是控制器的表: 那边给 5~6 位小数, 这里是同一批数据的全精度值 (逐位核对过)。
//
// ⚠ 【夹具是【已入库】的只读副本, 不是运行期产物】: 它在 tests/fixtures/ 下, 随仓库分发,
//   任何一次干净检出上都【必然存在】—— 所以这条用例【不需要】SKIP 就能跑, 真跳过了就是
//   检出坏了 (文件缺失), 不是"这台机器上没采过"。别改回去读 calib/: 那是运行期目录, 会被
//   下一次采集覆盖或整个清掉, 守卫的绿/红就成了"这台机器上碰巧有没有那个文件"的函数 ——
//   那种绿不携带任何信息 (本项目已被这类绿色的空跑咬过多次)。
//   文件在、但姿态数不是 7 => 那是另一次采集, 金标对它不成立: 【大声失败】, 而不是拿新数据
//   去对旧金标 (那会退化成"什么都能过")。
//
// 列布局 (文件头自己写着, 已逐行核对, 18 列):
//   rx,ry,rz,x,y,z,F576*,M576*,F1304*,M1304*
// 拟合吃 @1304 那一份 (原始通道); @576 不进拟合, 忽略。
// ⚠ 用的是【原始未镜像】的列: 参考 A 的第三行整体为负、而它的 2×2 块 det = +0.182, 正是
//   z 镜像还没叠上去的样子 (镜像版会得到 A[2] = +[0.0115 0.0149 0.4139], det = +0.0753)。
//   main.cpp 只在喂给实时求解器的那一份拷贝上做镜像; 落盘与离线分析用的都是没动过的原件。
// =====================================================================================

// 金标只对 2026-09-19 12:38:19 那次采集 (7 姿态) 成立 —— 就是下面这份冻结夹具里的那一批。
static const int REF_POSES = 7;
static const char* REF_FIXTURE = "calib_poses_2026-09-19.txt";

// 参考值 (全精度; 括号里是控制器表上的 5~6 位小数版本)
static const double REF_M     = 0.4223567251;          // 0.422357 kg
static const double REF_A[9]  = {  0.3645611253,  0.2105461118, -0.0000294763,   // 0.36456  0.21055 -0.00003
                                  -0.2182776660,  0.3733052719,  0.0029763987,   // -0.21828 0.37331  0.00298
                                  -0.0115452228, -0.0149328140, -0.4139021828 }; // -0.01155 -0.01493 -0.41390
static const double REF_SV[3] = { 0.4338551651, 0.4264690060, 0.4071983364 };   // 0.43386 0.42647 0.40720
static const double REF_ISO   = 1.0654639921;          // 1.06546
static const double REF_DET   = -0.0753421902;         // -0.075342 -> parity -1
static const double REF_RMSF  = 0.0223861767;          // 0.022386 N
static const double REF_RMSM  = 0.0013974296;          // 0.001397 N·m
static const double REF_CSMM[3] = { 0.5981153164, -0.5015476115, 54.5494358039 };  // (0.60, -0.50, 54.55) mm
static const double REF_SIGA  = 0.0141280725;          // 0.014128 —— A 的 9 个分量里最大的 1σ
static const double REF_COND  = 75.8194558;            // 75.82

// A 的 9 个元素【共用矩阵量级】作尺子 (见下)。
static const double REF_A_SCALE = 0.4139021828;

// 【容差】= 该量自身参考量级的 1e-5 (A 的 9 个元素例外, 见下)。
// 两头的余量都量过:
//   · 数值可复现性: 参考值来自 Python/numpy 的 SVD 最小二乘, 本仓库走正规方程 + Jacobi
//     特征分解 —— 两套不同实现、不同 libm, 同一个模型。实测吻合到 ~1e-12 相对; 而
//     sin/cos 的 1 ulp 差异经 cond ≈ 76 放大也只有 ~1e-14 相对。1e-5 宽出 7~9 个数量级,
//     换编译器/换数学库都不会误伤。
//   · 回归灵敏度: 本用例要拦的每一类错 (姿态列序搞反、重力误用带 psi 的那一支、力矩不走
//     叉乘、分解换约定、paramSigma 的自由度算错) 都会让这些数动 >= 1e-2 相对 —— 比容差
//     大三个数量级。
// ⇔ 松到不会被编译器差异误伤, 紧到任何真实回归都躲不过。
// A 是唯一的例外: 它的 9 个元素跨 4 个数量级 (-2.9e-5 到 -0.4139), 逐元素取相对容差会让
// 近零那一项的容差小到 3e-10 —— 那不是判断力, 是在测浮点噪声。所以 9 个元素共用矩阵量级。
static const double RTOL = 1e-5;

// 金标回归专用断言: 【不 return】—— 一次把所有对不上的量都打出来, 免得修一个冒一个。
static void nearRef(int& bad, const char* what, double got, double ref, double tol) {
    const double d = fabs(got - ref);
    if (!(d <= tol)) {
        std::cout << "\n    !! " << what << " = " << got << " (参考 " << ref
                  << ", 差了 " << d << " > 容差 " << tol << ")" << std::endl;
        bad++;
    }
}

// 采集文件里姿态是 [rx,ry,rz,x,y,z], 求解器要 [x,y,z,rx,ry,rz] —— 与 main.cpp 的实时路径
// 逐字相同的重排。**这个置换必须写死、必须注明**: 排错的话求解器会把【位置】当成角度去算
// 重力, 而得到的 A 依旧长得像一个合法的响应矩阵 (只是错的) —— 又一次"安静地解错"。
static void repackPoseRow(const double src[6], double dst[6]) {
    dst[0] = src[3]; dst[1] = src[4]; dst[2] = src[5];    // x,y,z    <- 文件的第 4..6 列
    dst[3] = src[0]; dst[4] = src[1]; dst[5] = src[2];    // rx,ry,rz <- 文件的第 1..3 列
}

static void test_replay_real_capture() {
    std::cout << "  replay_real_capture... ";

    // 候选路径: 夹具【已入库】, 本 exe 从 tests\ 跑 (构建脚本就会 cd 到那里), 或从
    // Touch_Client\ / 仓库根跑。刻意【不列 calib/ 下的运行期文件】—— 见上面的说明。
    static const char* CAND[] = {
        "fixtures/calib_poses_2026-09-19.txt",
        "tests/fixtures/calib_poses_2026-09-19.txt",
        "Touch_Client/tests/fixtures/calib_poses_2026-09-19.txt",
        "../../Touch_Client/tests/fixtures/calib_poses_2026-09-19.txt"
    };
    FILE* fp = nullptr;
    const char* used = nullptr;
    for (size_t i = 0; i < sizeof(CAND) / sizeof(CAND[0]) && !fp; i++) {
        fp = fopen(CAND[i], "r");
        if (fp) used = CAND[i];
    }
    if (!fp) {
        // 夹具随仓库分发, 本该【必然存在】—— 四个候选路径都找不到 = 这次检出是坏的
        // (不是"这台机器没采过")。
        // 【记失败, 不是跳过】: 夹具入库后 SKIP 的理由已经不存在了, 再记 pass 就成了一次
        // 什么都没验的绿跑 —— 那正是本项目被咬过多次的"不携带任何信息的绿色"。
        std::cout << "FAIL: 四个候选路径都没有 " << REF_FIXTURE << " —— 它是【已入库】的只读"
                     "副本, 缺了说明这次检出是坏的。本次【没有】校验实机拟合。" << std::endl;
        g_failed++;
        return;
    }
    std::cout << "[" << used << "] ";

    // 与 RawFit 的逐姿态残差表同尺寸: 超过它就没法逐姿态看残差, 也就没法诊断。
    static const int MAXN = PayloadCalibration::RAW_POSE_REPORT_MAX;
    double rawRows[MAXN][6];          // 文件给的 [rx,ry,rz,x,y,z]
    double F[MAXN][3], M[MAXN][3];
    int n = 0;
    char line[512];
    while (fgets(line, sizeof(line), fp)) {
        const char* q = line;
        while (*q == ' ' || *q == '\t') q++;
        if (*q == '\0' || *q == '\r' || *q == '\n' || *q == '#') continue;   // 空行与注释头
        if (n >= MAXN) { n = -1; break; }                                    // 行数超上限
        double c[18];
        // 【不静默跳行】: 列数对不上 = 布局变了, 拿它去对金标只会得出一个假的结论。
        if (sscanf(q, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
                   &c[0], &c[1], &c[2], &c[3], &c[4], &c[5], &c[6], &c[7], &c[8],
                   &c[9], &c[10], &c[11], &c[12], &c[13], &c[14], &c[15], &c[16], &c[17]) != 18) {
            n = -2;
            break;
        }
        for (int a = 0; a < 6; a++) rawRows[n][a] = c[a];
        // @1304 = 第 13..18 列 (下标 12..17): 拟合吃的那一份原始通道
        for (int a = 0; a < 3; a++) { F[n][a] = c[12 + a]; M[n][a] = c[15 + a]; }
        n++;
    }
    fclose(fp);

    if (n < 0) {
        std::cout << "FAIL: 采集文件读不动 (行数超上限 或 有一行不是 18 列浮点) ——"
                     " 布局变了就别拿它去对金标。" << std::endl;
        g_failed++;
        return;
    }
    if (n != REF_POSES) {
        std::cout << "FAIL: 文件里有 " << n << " 个姿态, 而金标只对 " << REF_POSES
                  << " 个姿态那一批 (2026-09-19 12:38:19) 成立。" << std::endl
                  << "      这是【另一次采集】, 不是实现回归 —— 要么换回那份冻结夹具,"
                     " 要么【新增】一份夹具并【显式】重新导出金标、在报告里说明来由;"
                     " 不要就地改数。" << std::endl;
        g_failed++;
        return;
    }

    double poses[MAXN][6];
    for (int i = 0; i < n; i++) repackPoseRow(rawRows[i], poses[i]);

    PayloadCalibration::RawFit fit;

    // (a) 不带令牌: 这批数据是旧协议采集的 (没有重复姿态对, 也没有逐姿态方差), 尺子不齐
    //     -> 新契约【拒给参数】。这不是"实现坏了", 正是门在按设计工作 —— 先把它钉住,
    //     免得将来有人用"放宽门限"把这条回放变成生产采纳路径。
    CHECK(!PayloadCalibration::fitRaw(poses, F, M, n, fit));
    CHECK(fit.modelFormStatus == PayloadCalibration::MODEL_FORM_NO_NOISE);
    CHECK(!fit.modelFormChecked);

    // (b) 逐字写出那个令牌: 离线重放 (手上本来就没有采集现场、拿不到重复姿态对) 才拿得到
    //     参数 —— 而它【仍然】不会被伪装成"验过了"。
    CHECK(PayloadCalibration::fitRaw(poses, F, M, n, fit, nullptr, nullptr, 1,
                                     PayloadCalibration::I_ACCEPT_UNVERIFIED_MODEL_FORM));
    CHECK(!fit.modelFormChecked);
    CHECK(fit.modelFormStatus == PayloadCalibration::MODEL_FORM_NO_NOISE);

    // ---- 金标: 12 + 6 个参数与派生的物理量, 逐项对 ----
    int bad = 0;
    for (int i = 0; i < 9; i++) {
        char what[24];
        snprintf(what, sizeof(what), "A[%d][%d]", i / 3, i % 3);
        nearRef(bad, what, fit.A[i], REF_A[i], RTOL * REF_A_SCALE);
    }
    nearRef(bad, "massScale (kg)",   fit.massScale,       REF_M,      RTOL * fabs(REF_M));
    nearRef(bad, "parity",           fit.parity,         -1.0,        1e-12);
    nearRef(bad, "isotropyRatio",    fit.isotropyRatio,   REF_ISO,    RTOL * fabs(REF_ISO));
    nearRef(bad, "rmsForceN",        fit.rmsForceN,       REF_RMSF,   RTOL * fabs(REF_RMSF));
    nearRef(bad, "rmsMomentNm",      fit.rmsMomentNm,     REF_RMSM,   RTOL * fabs(REF_RMSM));
    nearRef(bad, "cS[0] (mm)",       fit.cS[0] * 1000.0,  REF_CSMM[0], RTOL * fabs(REF_CSMM[0]));
    nearRef(bad, "cS[1] (mm)",       fit.cS[1] * 1000.0,  REF_CSMM[1], RTOL * fabs(REF_CSMM[1]));
    nearRef(bad, "cS[2] (mm)",       fit.cS[2] * 1000.0,  REF_CSMM[2], RTOL * fabs(REF_CSMM[2]));
    nearRef(bad, "max paramSigma(A)", maxSigmaA(fit),     REF_SIGA,   RTOL * fabs(REF_SIGA));
    nearRef(bad, "cond",             fit.cond,            REF_COND,   RTOL * fabs(REF_COND));

    // 分解那一半: 奇异值 / 行列式 (行列式用测试自己的 det3, 与 decompose 的 parity 互为对照)。
    // RawFit 只报 m/parity/isotropyRatio, 奇异值要走 decompose —— 两条路必须落在同一批数上。
    PayloadCalibration::Decomp d;
    CHECK(PayloadCalibration::decompose(fit.A, d));
    for (int k = 0; k < 3; k++) {
        char what[24];
        snprintf(what, sizeof(what), "sv[%d]", k);
        nearRef(bad, what, d.sv[k], REF_SV[k], RTOL * fabs(REF_SV[k]));
    }
    nearRef(bad, "decompose.m",      d.m,             REF_M,   RTOL * fabs(REF_M));
    nearRef(bad, "decompose.parity", d.parity,       -1.0,     1e-12);
    nearRef(bad, "decompose.iso",    d.isotropyRatio, REF_ISO, RTOL * fabs(REF_ISO));
    nearRef(bad, "det(A)",           det3(fit.A),     REF_DET, RTOL * fabs(REF_DET));

    printf("[7 姿态实机: m=%.6f kg, parity=%+.0f, iso=%.5f, rmsF=%.6f N, rmsM=%.6f N·m,"
           " cS=(%.4f, %.4f, %.4f) mm, cond=%.4f, maxSigmaA=%.7f] ",
           fit.massScale, fit.parity, fit.isotropyRatio, fit.rmsForceN, fit.rmsMomentNm,
           fit.cS[0] * 1000.0, fit.cS[1] * 1000.0, fit.cS[2] * 1000.0, fit.cond, maxSigmaA(fit));

    if (bad != 0) { std::cout << "FAIL (" << bad << " 项对不上金标)" << std::endl; g_failed++; return; }
    PASS();
}

// =====================================================================================
// ★ 力矩门限的实机标定 (2026-09-19 三次采集: 15:25 / 15:30 / 15:33)
//
// 【问题】三次采集的【力通道全过】, 只有【力矩失拟】一次比一次大 (5.36 / 20.5 / 61.2), 而
//   门限落在 5~8 —— 且姿态铺得越开 (cond 207 -> 44 -> 18) 它越大。两种读法的处置相反:
//     (i)  力矩模型 M = b_M + c_s × (A·g) 真的不完备 —— 自由 12 参数模型确实找到了叉乘
//          结构解释不了的那一份结构;
//     (ii) 门限对"受约束那一侧少 6 个参数"这件事标定得不对 —— 正确模型在这个统计量上本来
//          就跑这么高, 门限把统计量的零分布切错了位置。
//
// 【怎么把它们分开】把三份采集的姿态 / 重复对 / 逐姿态样本数 / 逐姿态 sd 原样冻进夹具
//   (fixtures/calib_poses_2026-09-19_15*.txt), 再用【生产代码 fitRaw 本身】跑零假设蒙特卡洛:
//   用该次采集【自己拟合出来的】A / c_s / b_M 造力与力矩 —— 于是"叉乘模型为真"是【构造】
//   出来的, 不是假设的 —— 加实测尺度的噪声, 再看 fitRaw 判不判。实测值落在零分布尾巴里
//   -> (i); 零分布自己就有一大片超过实测值 -> (ii)。
//
// ⚠ 【必须驱动生产代码, 不得重推公式】: 控制器曾离线重写一份统计量, 得到 4.95 / 13.36 /
//   11.06, 与实机打印的 5.36 / 20.5 / 61.2 差 4~5 倍 (而它算的 σ_rep,M 与打印值一致) ——
//   那份重写件不可信, 不得作为任何结论的依据。所以这里【只调用】PayloadCalibration::fitRaw,
//   一个字都不重推它的公式。
//
// ⚠ 【本用例只测量, 不改】: 不动任何门限 / 模型 / 判决逻辑。金标是【实机的记录】——
//   谁改了统计量或门限, 这里必须【变红】, 而不是"顺手把数改成新的"。
// =====================================================================================

static const int MG_MAXN   = PayloadCalibration::RAW_POSE_REPORT_MAX;
static const int MG_MAXREP = 8;

// 三次采集的金标。
//
// ⚠ 【必须先说的是: 这张表【不是】控制台当时打印的那几个数】。
//   实机控制台 (runs 001-003 的原始输出, Docs/superpowers/specs/2026-09-19-raw-channel-calibration-*)
//   打印的是 5.359 / 20.51 / 61.22, 而【从冻结夹具重放得不到它们】, 得到的是
//   6.973 / 18.36 / 33.74。原因不是实现回归, 是【夹具存不下那几个数】:
//     · 夹具是 calib_poses.txt 的原样副本, 而它把姿态写成 %.1f/%.3f、把 @1304 的力与力矩
//       写成 %.3f —— 力矩的量化台阶 0.001 N·m 比【重复姿态对的真实差值】(~0.0002 N·m)
//       还大, 于是从夹具算出的 σ_sys,M 被量化噪声顶上去 (0.00071 vs 控制台的 0.0005),
//       尺子变粗, 失拟统计量被除以一个更大的分母 —— 这三个数就是这么变小的;
//     · 控制台读的是内存里的【全精度 double】, 夹具存的是它的 3 位小数截断。
//   实测 (用夹具的量化台阶做还原抽样, 400 次, 用生产 fitRaw): 只把"四舍五入丢掉的那
//   一点"按均匀分布补回去, 统计量就在 2.26~19.96 / 6.97~45.15 / 14.18~63.16 之间跑,
//   门限在 5.32~11.48 / 5.08~7.86 / 5.27~8.65 之间跑 —— 控制台的三个数【全都落在这个
//   区间里】。所以: 夹具重放与控制台打印【本来就是两个数】, 差多少由量化决定。
//   ⇒ 下面 refRatio/refLimit 钉的是【夹具重放】的值 (可复现、可回归); 控制台那六个数
//     另存一列 (conRatio/conLimit/consoleCond), 【只打印、不断言】—— 断言它们等于夹具
//     的值是错的, 断言它们等于控制台的值也是做不到的。详见
//     Docs/superpowers/evidence/moment-gate-calibration-report.md。
//
// 容差见 RTOL_MG。refPass = 力通道与力矩通道【两个门都过】才为 true (fitRaw 的返回值) ——
// 这一列【夹具重放与控制台完全一致】(通过 / 拒绝 / 拒绝), 也是本表里唯一可以直接对控制台的那一列。
struct MomentCapture {
    const char* tag;
    const char* fixture;
    int         poses;
    int         refPairs;
    double      refRatio;      // 夹具重放的 lackOfFitMomentRatio (金标, 断言)
    double      refLimit;      // 夹具重放的 lackOfFitMomentLimit (金标, 断言)
    bool        refPass;       // 夹具重放的判决 (金标, 断言)
    double      conRatio;      // 控制台当时打印的失拟 (记录, 不断言)
    double      conLimit;      // 控制台当时打印的门限 (记录, 不断言)
    double      conCond;       // 控制台当时打印的 cond  (记录, 不断言)
    // ★ 2026-09-19 (b) 重标定的力矩门限需要两个【不是从本实现读出来的】锚:
    //   refProdLimit = 重标定之前的门限 (= modelFormLimit(6,R,σ_sysM²,floor_M²) 自己) ——
    //                  §6.1 之前那一版的 refLimit, 现在【只用来独立复算新门限】, 不再断言相等。
    //                  ⚠ 它没变: 变的是门外面的那个乘数, 不是这个函数。
    //   refExcess    = e = δA 引起的期望多余量。【这个数不是本实现算出来的】—— 它是离线报告
    //                  (Docs/superpowers/evidence/limit-recalibration-report.md §1.3 的 e(A) 列,
    //                  独立实现、独立 harness) 印出来的五位小数。断言它是【口径自校】:
    //                  口径一旦漂移 (Σ_A 用 paramSigma、漏掉 p* 重优化项、少除那个 6),
    //                  这里立刻变红。容差见 REF_EXCESS_TOL。
    double      refProdLimit;
    double      refExcess;
};

// ★★ 这张表在 2026-09-19 被【显式重导】过一次(refLimit 一列)。规矩与来由:
//
// 【为什么必须动】力矩门的门限从 modelFormLimit(6,R,σ_sysM²,floor_M²) 换成了
//     c0(α,R)·modelFormLimit(6,R,σ_sysM²,floor_M²) + κ(α,R)·e
// (把那一刀从零分布的 99% 附近挪到它本来就该在的 99.99% 分位; 推导见 PayloadCalibration.cpp
// 的 momentFormLimit 上面)。门限换了, refLimit 必然变 —— 这是本表自己的规矩要求的"变红",
// 不是"顺手把数改成新的"。
//
// 【怎么重导的 —— 三块输入, 加一句必须说清楚的话】
//   ⚠ 【字面量本身是【生产代码在全精度 e 下的输出】(钉到 RTOL_MG = 1e-6), 不是"由这三块
//     独立重算出来的"】: 拿引用的【五位小数】e 代进去, 15:25 得 14.68331183 —— 与冻结的
//     14.68333151 差 1.97e-5, 比 RTOL_MG·refLimit = 1.47e-5 还宽; 15:30 同理 (10.50520464
//     对 10.50518393, 差 2.07e-5 > 1.05e-5); 15:33 差 2.9e-6, 恰好落在它自己的容差 (1.10e-5)
//     之内。那点差就是 e 被印成五位小数时丢掉的那几位 (κ·Δe 最大到 2.6e-5 的量级: κ=5.225,
//     舍入 ±5e-6)。⇒ 五位小数的 e 只够把 e 钉到 1e-4 (见下面的 REF_EXCESS_TOL), 【不够把
//     refLimit 钉到 1e-6】—— 要逐位可复现, e 得带足位数。下面那条"当场重算"用的是实现自己的
//     全精度字段, 所以它断的是【结构 + 1e-6 的一致性】, 不是"外部锚能独立复现这个字面量"。
//   (1) refProdLimit: 未变, 仍是 7.402061054 / 6.050025293 / 6.440865571 (本实现之前就钉住的
//       金标, 复现过 5.1e-11 相对)。
//   (2) e: 离线报告的 e(A) 列 (0.55771 / 0.48289 / 0.48102), 独立实现独立 harness 印出来的
//       五位小数。本实现给出 0.557713 / 0.482887 / 0.481020 (实现的全精度字段), 五位小数逐位
//       相同 (1e-4 容差)。⚠ 本实现自己的 %.6f 打印是 0.557714 / 0.482886 / 0.481021 —— 出处
//       D:\tmp\limitprod\final.txt:275 (moment_gate_limit_is_c0_times_prod_plus_kappa_times_e
//       的 PASS 行)。六位数与全精度值不必逐位吻合 (中间那个 0.482886 对全精度 0.4828857229
//       既非舍入也非截断), 所以这里只拿【五位小数】当外部锚。
//   (3) c0 / κ: 离线报告的标定表 (α=0.9999 档, 路线 2, R=1/3/5):
//       c0 = (2.160, 1.590, 1.350),  κ = (4.589, 5.225, 4.841)。
//   ⇒ 三个数代进去 (用实现自己的全精度值) 就是新的 refLimit。⚠ 【变红的范围要说准, 不是"都会红"】:
//     下面 (a) 那条结构断言【不读任何冻结字面量】—— 门限与 c0·LIMIT_prod + κ·e 两侧都由实现
//     现算, 所以改字面量它不动, e (Σ_A) 漂了它两侧一起动也不动。真正会红的只有两处:
//     nearRef(重算 vs 字面量) 对【字面量被改】红, 以及 (b) 那条 e 外部锚对【口径漂
//     (Σ_A / p* 重优化 / 那个 6)】红 —— 口径也正是由 (b) 那条独立锚钉住的。
//
// 【没变的】refRatio 三列、refPass 三列、以及控制台那三列(记录)。refPass 保住 (过/拒/拒) 是
// 本表的硬要求: 18.36 与 33.74 对新门限 10.51 / 11.02 仍然超 (余量 1.75× / 3.06×)。
static const MomentCapture MG_CAPS[3] = {
    { "15:25", "calib_poses_2026-09-19_1525.txt",  9, 3,
      6.973380689, 14.68333151, true,   5.359, 7.998, 206.606,
      7.402061054, 0.55771 },
    { "15:30", "calib_poses_2026-09-19_1530.txt", 10, 5,
      18.36001184, 10.50518393, false,  20.51,  6.321,  43.9034,
      6.050025293, 0.48289 },
    { "15:33", "calib_poses_2026-09-19_1533.txt", 10, 5,
      33.74121269, 11.02378921, false,  61.22,  5.239,  18.2353,
      6.440865571, 0.48102 }
};

// refExcess 的容差: 离线报告印的是【五位小数】, 所以真值落在 ±5e-6 之内; 本实现在 FP 累加
// 顺序上与离线探针有别 (~1e-12 相对), 两头加起来留 1e-4 足够宽。口径错一点 (比如 Σ_A 换成
// paramSigma) 动的是 40% 以上 —— 这个容差拦得住。
static const double REF_EXCESS_TOL = 1e-4;

// ★ 力通道门限 —— 2026-09-19 重标定【之前】夹具重放出来的逐位值 (改动前那次运行的实测记录)。
// 本任务只动力矩分支, 这三个数必须【逐位不变】; 下面的测试把这句话写成两条断言。
static const double REF_FORCE_LIMIT[3] = { 4.164824768, 4.372012792, 2.924170995 };

// 【容差】= 该量自身的 1e-6 (先按 1e-3 跑一遍读出全精度值, 再收紧到这里)。
//   · 数值可复现性: 同一份夹具、同一个二进制走同一条确定性算术, 逐位一致; 换编译器/libm 的
//     差异经 12 参数正规方程放大也只有 ~1e-12 相对。1e-6 宽出 6 个数量级。
//   · 回归灵敏度: 本用例要拦的每一类错 (统计量的分母口径、失拟的自由度、σ_rep 的池化、
//     门限的 χ² 分位/折扣) 都会让这两个数动 >= 1e-2 相对 —— 比容差大四个数量级。
//     (夹具那 3 位小数的量化本身就会让统计量动 >= 2 倍, 但它已经冻在夹具里了, 不是变量。)
static const double RTOL_MG = 1e-6;

// 夹具读出来的一份采集: 姿态 (求解器序) / 力 / 力矩 / 逐姿态样本数与 sd / 重复对。
struct MomentCaptureData {
    int    n;
    double poses[MG_MAXN][6];              // [x,y,z,rx,ry,rz] —— 与 main.cpp 的重排逐字相同
    double F[MG_MAXN][3], M[MG_MAXN][3];   // @1304 原始未镜像
    double sdF[MG_MAXN][3], sdM[MG_MAXN][3];
    int    nsamp[MG_MAXN];
    PayloadCalibration::RepeatPair reps[MG_MAXREP];
    int    repCount;
};

// 夹具的四个候选路径 —— 与 REF 那条同一套 (夹具已入库: 四个都找不到 = 检出坏了, 记 FAIL)。
static char g_mgFixturePath[512];

static bool mgOpenFixture(const char* name, FILE** out) {
    static const char* DIRS[4] = { "fixtures/", "tests/fixtures/",
                                   "Touch_Client/tests/fixtures/",
                                   "../../Touch_Client/tests/fixtures/" };
    for (int i = 0; i < 4; i++) {
        snprintf(g_mgFixturePath, sizeof(g_mgFixturePath), "%s%s", DIRS[i], name);
        FILE* f = fopen(g_mgFixturePath, "r");
        if (f) { *out = f; return true; }
    }
    g_mgFixturePath[0] = '\0';
    return false;
}

// "# repeat: first=1,3,5 seconds=2,4,6  (共 3 对...)" -> 0 基下标对。返回解析出的对数。
// "# repeat: none" -> 0。列号是 1 基的行号, 与 main.cpp 落盘时逐字一致。
static int mgParseRepeat(const char* line, PayloadCalibration::RepeatPair* out, int maxOut) {
    int firsts[16], seconds[16], nf = 0, ns = 0;
    const char* p = strstr(line, "first=");
    if (!p) return 0;
    p += 6;
    while (*p && *p != ' ' && nf < 16) {
        char* e = nullptr;
        const long v = strtol(p, &e, 10);
        if (e == p) break;
        firsts[nf++] = (int)v;
        p = e;
        if (*p == ',') p++;
    }
    p = strstr(line, "seconds=");
    if (!p) return 0;
    p += 8;
    while (*p && *p != ' ' && ns < 16) {
        char* e = nullptr;
        const long v = strtol(p, &e, 10);
        if (e == p) break;
        seconds[ns++] = (int)v;
        p = e;
        if (*p == ',') p++;
    }
    int cnt = (nf < ns) ? nf : ns;
    if (cnt > maxOut) cnt = maxOut;
    for (int i = 0; i < cnt; i++) { out[i].first = firsts[i] - 1; out[i].second = seconds[i] - 1; }
    return cnt;
}

// 逗号分隔的一行 -> 逐列。【数一下列数】: 列数不是 25 = 布局变了, 不静默跳行 ——
// 拿一个列序读歪的表去对金标, 只会得出一个假的结论。
static int mgSplitRow(const char* q, double* out, int maxOut) {
    int k = 0;
    while (*q && k < maxOut) {
        char* e = nullptr;
        const double v = strtod(q, &e);
        if (e == q) break;
        out[k++] = v;
        q = e;
        if (*q == ',') q++; else break;
    }
    return k;
}

// 夹具 -> MomentCaptureData。25 列 = 18 均值 + N1304 + 6 个 sd (见夹具的 # 头)。
static bool mgLoad(const char* fixture, MomentCaptureData& d, const char*& pathUsed) {
    FILE* fp = nullptr;
    if (!mgOpenFixture(fixture, &fp)) return false;
    pathUsed = g_mgFixturePath;
    d.n = 0; d.repCount = 0;
    char line[1024];
    while (fgets(line, sizeof(line), fp)) {
        if (strstr(line, "# repeat:") != nullptr) {
            d.repCount = mgParseRepeat(line, d.reps, MG_MAXREP);
            continue;
        }
        const char* q = line;
        while (*q == ' ' || *q == '\t') q++;
        if (*q == '\0' || *q == '\r' || *q == '\n' || *q == '#') continue;
        if (d.n >= MG_MAXN) { fclose(fp); return false; }
        double c[25];
        if (mgSplitRow(q, c, 25) != 25) { fclose(fp); return false; }
        // 列序 (main.cpp 落盘时写明): rx,ry,rz,x,y,z, F576*, M576*, F1304*, M1304*, N1304, sd*
        double src[6];
        for (int a = 0; a < 6; a++) src[a] = c[a];
        repackPoseRow(src, d.poses[d.n]);      // 与实机路径【逐字相同】的重排
        for (int a = 0; a < 3; a++) {
            d.F[d.n][a]     = c[12 + a];
            d.M[d.n][a]     = c[15 + a];
            d.sdF[d.n][a]   = c[19 + a];
            d.sdM[d.n][a]   = c[22 + a];
        }
        d.nsamp[d.n] = (int)(c[18] + 0.5);
        if (d.nsamp[d.n] < 1) { fclose(fp); return false; }
        d.n++;
    }
    fclose(fp);
    if (d.n <= 0) return false;
    for (int i = 0; i < d.n; i++)
        for (int a = 0; a < 3; a++)
            if (!(d.sdF[i][a] > 0.0) || !(d.sdM[i][a] > 0.0)) return false;
    return true;
}

// 夹具的 (N, sd) -> fitRaw 要的 PoseNoise: var = sd², n = N1304。
// 【口径必须说清】夹具的 sd 是【单个样本】的标准差, 而 fitRaw 内部把它折算成【均值】的方差
// (var/N)。所以零假设里给"均值"加的抖动必须按 sd/√N 抽 —— 这样申报的 floor² 与真实抖动
// 【逐位一致】; 若直接按 sd 抽, 合成数据会比实机噪 √N ≈ 4.5 倍, 那是另一场实验。
static void mgDeclareNoise(const MomentCaptureData& d, PayloadCalibration::PoseNoise* nz) {
    for (int i = 0; i < d.n; i++) {
        nz[i].n = d.nsamp[i];
        for (int a = 0; a < 3; a++) {
            nz[i].varF[a] = d.sdF[i][a] * d.sdF[i][a];
            nz[i].varM[a] = d.sdM[i][a] * d.sdM[i][a];
        }
    }
}

// 蒙特卡洛会成千上万次调用 fitRaw, 每次拒绝都往 stderr 写几行 —— 要的是统计量, 不是几万行
// 日志。把 fd 2 接到 NUL, 跑完接回来 (不碰 stdout)。
static int g_savedStderrFd = -1;

static void mgMuteStderr() {
#ifdef _WIN32
    fflush(stderr);
    g_savedStderrFd = _dup(2);
    if (g_savedStderrFd >= 0) {
        const int nul = _open("NUL", _O_WRONLY);
        if (nul >= 0) { _dup2(nul, 2); _close(nul); }
    }
#endif
}

static void mgUnmuteStderr() {
#ifdef _WIN32
    fflush(stderr);
    if (g_savedStderrFd >= 0) {
        _dup2(g_savedStderrFd, 2);
        _close(g_savedStderrFd);
        g_savedStderrFd = -1;
    }
#endif
}

// 确定性 RNG (固定种子 -> 整个蒙特卡洛可复现): 64 位 LCG + Box-Muller。
struct MgRng {
    unsigned long long s;
    void seed(unsigned long long v) { s = v ? v : 1ULL; }
    double uni() {
        s = s * 6364136223846793005ULL + 1442695040888963407ULL;
        return (double)((s >> 11) & 0x1FFFFFFFFFFFFFULL) / 9007199254740992.0;
    }
    double norm() {
        double u1 = uni();
        if (!(u1 > 0.0)) u1 = 1e-300;
        const double u2 = uni();
        return sqrt(-2.0 * log(u1)) * cos(6.283185307179586476925286766559 * u2);
    }
};

// -------------------------------------------------------------------------------------
// 1) 金标: 三份冻结夹具各跑一次生产 fitRaw, 断言统计量 / 门限 / 判决。
// -------------------------------------------------------------------------------------
static void test_moment_gate_real_captures_golden() {
    std::cout << "  moment_gate_real_captures_golden..." << std::endl;
    int bad = 0;
    mgMuteStderr();
    for (int k = 0; k < 3; k++) {
        MomentCaptureData d;
        const char* used = nullptr;
        if (!mgLoad(MG_CAPS[k].fixture, d, used)) {
            mgUnmuteStderr();
            std::cout << "    FAIL: 读不到夹具 " << MG_CAPS[k].fixture
                      << " —— 它【已入库】的只读副本, 缺了说明这次检出是坏的。" << std::endl;
            g_failed++;
            return;
        }
        if (d.n != MG_CAPS[k].poses || d.repCount != MG_CAPS[k].refPairs) {
            mgUnmuteStderr();
            std::cout << "    FAIL: 夹具 " << MG_CAPS[k].fixture << " 里是 " << d.n << " 个姿态 / "
                      << d.repCount << " 对, 而金标只对 " << MG_CAPS[k].poses << " 个 / "
                      << MG_CAPS[k].refPairs << " 对那一次采集成立 —— 这是【另一次采集】,"
                         " 不是实现回归: 请【新增】夹具并【显式】重导金标, 不要就地改数。"
                      << std::endl;
            g_failed++;
            return;
        }
        PayloadCalibration::PoseNoise nz[MG_MAXN];
        mgDeclareNoise(d, nz);
        PayloadCalibration::RawFit fit;
        const bool ok = PayloadCalibration::fitRaw(d.poses, d.F, d.M, d.n, fit, nz,
                                                   d.reps, d.repCount,
                                                   PayloadCalibration::MODEL_FORM_REQUIRED);
        mgUnmuteStderr();

        printf("    [%s] %s  n=%d pairs=%d cond=%.6g rmsF=%.6g N rmsM=%.6g N·m\n",
               MG_CAPS[k].tag, used, d.n, d.repCount, fit.cond, fit.rmsForceN, fit.rmsMomentNm);
        printf("         力通道  χ²_rep/dof = %.10g  门限 = %.10g  -> %s\n",
               fit.chi2RepForceRatio, fit.chi2RepForceLimit,
               (fit.chi2RepForceRatio < fit.chi2RepForceLimit) ? "过" : "拒");
        printf("         力矩失拟 ratio   = %.10g  门限 = %.10g  dof=%d  -> %s\n",
               fit.lackOfFitMomentRatio, fit.lackOfFitMomentLimit, fit.lackOfFitMomentDof,
               (fit.lackOfFitMomentRatio < fit.lackOfFitMomentLimit) ? "过" : "拒");
        // 门限的两块: LIMIT_prod (未变) 与 e (新加的修正项)。两个都印全精度 —— 门限是这两个
        // 数的函数, 判决是门限的函数, 所以"离门限多远"要能一路追到这里。
        printf("         门限分解 c0·LIMIT_prod + κ·e:  LIMIT_prod = %.10g (未变),  e = %.10g\n",
               PayloadCalibration::modelFormLimit(6.0, fit.repeatPairCount, fit.repeatSysM,
                                                  fit.repeatFloorM),
               fit.lackOfFitMomentExcess);
        printf("         尺子 σ_rep,M = (%.6g, %.6g, %.6g) N·m;  σ_rep,F = (%.6g, %.6g, %.6g) N\n",
               fit.repeatSigmaM[0], fit.repeatSigmaM[1], fit.repeatSigmaM[2],
               fit.repeatSigmaF[0], fit.repeatSigmaF[1], fit.repeatSigmaF[2]);
        printf("         判决 fitRaw = %s (modelFormChecked=%d, momentFormChecked=%d)\n",
               ok ? "通过" : "拒绝", (int)fit.modelFormChecked, (int)fit.momentFormChecked);
        // 控制台当时打印的 (记录, 不断言) —— 与夹具重放的差就是【夹具存不下的那点精度】。
        printf("         对照 控制台: cond=%.6g 失拟=%.4g 门限=%.4g  [夹具/控制台 失拟比 = %.3f,"
               " 门限比 = %.3f, cond 比 = %.6f]\n",
               MG_CAPS[k].conCond, MG_CAPS[k].conRatio, MG_CAPS[k].conLimit,
               fit.lackOfFitMomentRatio / MG_CAPS[k].conRatio,
               fit.lackOfFitMomentLimit / MG_CAPS[k].conLimit,
               fit.cond / MG_CAPS[k].conCond);

        nearRef(bad, "lackOfFitMomentRatio", fit.lackOfFitMomentRatio,
                MG_CAPS[k].refRatio, RTOL_MG * fabs(MG_CAPS[k].refRatio));
        nearRef(bad, "lackOfFitMomentLimit", fit.lackOfFitMomentLimit,
                MG_CAPS[k].refLimit, RTOL_MG * fabs(MG_CAPS[k].refLimit));
        if (ok != MG_CAPS[k].refPass) {
            std::cout << "    !! [" << MG_CAPS[k].tag << "] 判决 = " << (ok ? "通过" : "拒绝")
                      << " (金标 " << (MG_CAPS[k].refPass ? "通过" : "拒绝") << ")" << std::endl;
            bad++;
        }
    }
    if (bad != 0) {
        std::cout << "FAIL (" << bad << " 项对不上金标 —— 这些数是实机的记录, 要查的是实现)" << std::endl;
        g_failed++;
        return;
    }
    PASS();
}

// -------------------------------------------------------------------------------------
// 1b) ★ 2026-09-19 重标定的两条断言 (改动本身的两条, 与上面那张金标表互补)。
//
// 为什么单独写两条, 而不是塞进金标表: 它们断言的是【结构】而不是数值 ——
//   · 力通道门限必须【逐位】等于未改动的那个函数算出来的东西 (改的是力矩那一支,
//     如果谁顺手把力那一支也改了, 金标表动的是第三位小数, 而这里动的是"是不是同一个式子");
//   · 力矩门限必须【等于 c0·LIMIT_prod + κ·e】, 而 e 必须等于【离线报告】的数 (外部锚)。
// 两者一起才把"这次改动是什么"钉住: 一个是"没动什么", 一个是"动了什么、按哪个口径动的"。
// -------------------------------------------------------------------------------------

// 力通道门限的逐位不变: 三份夹具上,
//   (1) 它 == modelFormLimit(chi2DofForce, R, repeatSysF, repeatFloorF) —— 同一个函数、同一组
//       输入 ⇒ 必须【逐位】(==) 相同; 这一条与数值无关, 是"力那一条支路没被碰过"的证明;
//   (2) 它 == 改动【之前】那一次运行的实测记录 (REF_FORCE_LIMIT), 容差 RTOL_MG。
static void moment_gate_force_limit_is_bit_identical_to_the_old_formula() {
    TEST(moment_gate_force_limit_is_bit_identical_to_the_old_formula);
    int bad = 0;
    mgMuteStderr();
    for (int k = 0; k < 3; k++) {
        MomentCaptureData d;
        const char* used = nullptr;
        if (!mgLoad(MG_CAPS[k].fixture, d, used)) {
            mgUnmuteStderr();
            std::cout << "    FAIL: 读不到夹具 " << MG_CAPS[k].fixture << std::endl;
            g_failed++;
            return;
        }
        PayloadCalibration::PoseNoise nz[MG_MAXN];
        mgDeclareNoise(d, nz);
        PayloadCalibration::RawFit fit;
        PayloadCalibration::fitRaw(d.poses, d.F, d.M, d.n, fit, nz, d.reps, d.repCount,
                                   PayloadCalibration::MODEL_FORM_REQUIRED);
        // (1) 逐位 == 旧的式子
        const double recomputed = PayloadCalibration::modelFormLimit(
            (double)fit.chi2DofForce, fit.repeatPairCount, fit.repeatSysF, fit.repeatFloorF);
        if (!(fit.chi2RepForceLimit == recomputed)) {
            std::cout << "    !! [" << MG_CAPS[k].tag << "] 力门限不再逐位等于"
                      << " modelFormLimit(dofF,R,sysF,floorF): " << fit.chi2RepForceLimit
                      << " vs " << recomputed << " (差 "
                      << (fit.chi2RepForceLimit - recomputed) << ")" << std::endl;
            bad++;
        }
        // (2) == 改动【之前】那一次运行的实测记录 (这一列是记录下来的十进制值, 不是 double 的
        //     精确二进制展开, 所以这里比的是"没有可观察的位变", 容差 = 记录值本身的 1e-6;
        //     真正的逐位证明是上面那条 (1))。
        nearRef(bad, "chi2RepForceLimit", fit.chi2RepForceLimit,
                REF_FORCE_LIMIT[k], RTOL_MG * fabs(REF_FORCE_LIMIT[k]));
        printf("[%s 力门限 %.10g (改动前 %.10g, 差 %.2e)] ", MG_CAPS[k].tag,
               fit.chi2RepForceLimit, REF_FORCE_LIMIT[k],
               fit.chi2RepForceLimit - REF_FORCE_LIMIT[k]);
    }
    mgUnmuteStderr();
    if (bad != 0) {
        std::cout << "FAIL (" << bad << " 项 —— 力通道的门限必须逐位不变, 本任务只动力矩分支)"
                  << std::endl;
        g_failed++;
        return;
    }
    printf("[力门限逐位不变: 4.164824768 / 4.372012792 / 2.924170995, == 未改动的式子] ");
    PASS();
}

// ★ 力矩门限的结构 × 口径:
//   (a) 门限 == c0(R)·LIMIT_prod + κ(R)·e, 其中 LIMIT_prod 与 e 都是 fit 自己的字段
//       —— 三个数一起重算一遍, 与冻结的 refLimit 是同一件事写两遍;
//   (b) e == 离线报告印的五位小数 (外部锚, 见 MG_CAPS 的 refExcess);
//   (c) R ∉ {1,3,5} 的处理: 区间内线性插值、区间外平夹 —— 三个标定点逐位落在表上。
static const double MG_C0[3]  = { 2.160, 1.590, 1.350 };   // 离线报告 α=0.9999 / 路线 2
static const double MG_KAP[3] = { 4.589, 5.225, 4.841 };

static void mgExpectCoeffs(int R, double& c0, double& kap) {
    if (R <= 1)      { c0 = MG_C0[0];  kap = MG_KAP[0]; }
    else if (R >= 5) { c0 = MG_C0[2];  kap = MG_KAP[2]; }
    else if (R < 3)  { const double t = (R - 1.0) / 2.0;
                       c0 = MG_C0[0] + t * (MG_C0[1] - MG_C0[0]);
                       kap = MG_KAP[0] + t * (MG_KAP[1] - MG_KAP[0]); }
    else if (R > 3)  { const double t = (R - 3.0) / 2.0;
                       c0 = MG_C0[1] + t * (MG_C0[2] - MG_C0[1]);
                       kap = MG_KAP[1] + t * (MG_KAP[2] - MG_KAP[1]); }
    else             { c0 = MG_C0[1];  kap = MG_KAP[1]; }
}

static void moment_gate_limit_is_c0_times_prod_plus_kappa_times_e() {
    TEST(moment_gate_limit_is_c0_times_prod_plus_kappa_times_e);
    int bad = 0;
    mgMuteStderr();
    for (int k = 0; k < 3; k++) {
        MomentCaptureData d;
        const char* used = nullptr;
        if (!mgLoad(MG_CAPS[k].fixture, d, used)) {
            mgUnmuteStderr();
            std::cout << "    FAIL: 读不到夹具 " << MG_CAPS[k].fixture << std::endl;
            g_failed++;
            return;
        }
        PayloadCalibration::PoseNoise nz[MG_MAXN];
        mgDeclareNoise(d, nz);
        PayloadCalibration::RawFit fit;
        PayloadCalibration::fitRaw(d.poses, d.F, d.M, d.n, fit, nz, d.reps, d.repCount,
                                   PayloadCalibration::MODEL_FORM_REQUIRED);
        const int R = fit.repeatPairCount;
        double c0 = 1.0, kap = 0.0;
        mgExpectCoeffs(R, c0, kap);
        // (a) 门限 = c0·LIMIT_prod + κ·e
        const double prod = PayloadCalibration::modelFormLimit(6.0, R, fit.repeatSysM,
                                                              fit.repeatFloorM);
        const double expect = c0 * prod + kap * fit.lackOfFitMomentExcess;
        const double tol = 1e-12 * (fabs(expect) + 1e-300);
        if (!(fabs(fit.lackOfFitMomentLimit - expect) <= tol)) {
            std::cout << "    !! [" << MG_CAPS[k].tag << "] 门限 != c0·LIMIT_prod + κ·e: "
                      << fit.lackOfFitMomentLimit << " vs " << expect << std::endl;
            bad++;
        }
        // 冻结的 refLimit 与上面现算的必须是同一个数 (近似到 RTOL_MG)。
        // ⚠ 上面那个 expect 用的是【实现自己的全精度】LIMIT_prod 与 e —— 引用的五位小数 e 只
        // 能把 refLimit 复现到 ~2e-5, 比这里的 1e-6 宽 (见 MG_CAPS 上方的说明)。所以这一条钉的
        // 是【结构 + 1e-6 的一致性】(字面量与实现同步), 不是"从外部锚独立复现"; e 的口径由下面
        // (b) 那条独立锚 (离线报告的五位小数, 容差 1e-4) 钉住。
        nearRef(bad, "refLimit(重算)", expect, MG_CAPS[k].refLimit,
                RTOL_MG * fabs(MG_CAPS[k].refLimit));
        // (b) e == 离线报告 (外部锚)
        if (!(fabs(fit.lackOfFitMomentExcess - MG_CAPS[k].refExcess) <= REF_EXCESS_TOL)) {
            std::cout << "    !! [" << MG_CAPS[k].tag << "] e 与离线报告的 e(A) 对不上: 本实现 "
                      << fit.lackOfFitMomentExcess << ", 离线报告 " << MG_CAPS[k].refExcess
                      << " (容差 " << REF_EXCESS_TOL << ") —— 口径漂了 (Σ_A? p* 重优化? 那个 6?)"
                      << std::endl;
            bad++;
        }
        printf("[%s R=%d: e=%.6f (离线 %.5f), LIMIT_prod=%.6f, 门限=%.6f] ",
               MG_CAPS[k].tag, R, fit.lackOfFitMomentExcess, MG_CAPS[k].refExcess,
               prod, fit.lackOfFitMomentLimit);
    }
    mgUnmuteStderr();

    // (c) R ∉ {1,3,5} 的规则: 黑箱把系数抠出来再断。
    //     门限 = c0(R)·P(R) + κ(R)·e 对 e 是【线性】的 (斜率 κ, 截距 c0·P), 所以取 e=0 与
    //     e=1 两点的值就能把 (c0, κ) 解出来 —— 不需要把内部函数暴露出来, 也不是"读实现自己的
    //     输出": 断的是抠出来的数与【离线报告那张表 + 声明过的插值规则】的关系。
    {
        const double sys2[3]   = { 4.0e-8, 9.0e-8, 2.5e-7 };
        const double floor2[3] = { 1.0e-8, 2.0e-8, 5.0e-8 };
        double c0v[8], kav[8];
        for (int R = 1; R <= 8; R++) {
            const double at0 = PayloadCalibration::momentFormLimit(0.0, R, sys2, floor2);
            const double at1 = PayloadCalibration::momentFormLimit(1.0, R, sys2, floor2);
            const double P   = PayloadCalibration::modelFormLimit(6.0, R, sys2, floor2);
            if (!(P > 0.0)) { std::cout << "    !! modelFormLimit 在 R=" << R << " 退化" << std::endl;
                              bad++; continue; }
            c0v[R - 1] = at0 / P;          // 截距 / P(R)
            kav[R - 1] = at1 - at0;        // 斜率
        }
        const double rel = 1e-12;
        // 三个标定点: 系数必须【恰是】离线报告表上的值
        const int RTAB[3] = { 1, 3, 5 };
        for (int t = 0; t < 3; t++) {
            const int R = RTAB[t];
            if (!(fabs(c0v[R - 1] - MG_C0[t]) <= rel * MG_C0[t])) {
                std::cout << "    !! c0(R=" << R << ") = " << c0v[R - 1] << " != 表上的 "
                          << MG_C0[t] << std::endl; bad++;
            }
            if (!(fabs(kav[R - 1] - MG_KAP[t]) <= rel * MG_KAP[t])) {
                std::cout << "    !! kappa(R=" << R << ") = " << kav[R - 1] << " != 表上的 "
                          << MG_KAP[t] << std::endl; bad++;
            }
        }
        // R=2 (夹在 1..3) 与 R=4 (夹在 3..5): 线性插值 = 两端中点
        if (!(fabs(c0v[1] - 0.5 * (MG_C0[0] + MG_C0[1])) <= rel * MG_C0[1])) {
            std::cout << "    !! R=2 的 c0 不是 R=1..3 的中点" << std::endl; bad++;
        }
        if (!(fabs(kav[1] - 0.5 * (MG_KAP[0] + MG_KAP[1])) <= rel * MG_KAP[1])) {
            std::cout << "    !! R=2 的 kappa 不是 R=1..3 的中点" << std::endl; bad++;
        }
        if (!(fabs(c0v[3] - 0.5 * (MG_C0[1] + MG_C0[2])) <= rel * MG_C0[1])) {
            std::cout << "    !! R=4 的 c0 不是 R=3..5 的中点" << std::endl; bad++;
        }
        if (!(fabs(kav[3] - 0.5 * (MG_KAP[1] + MG_KAP[2])) <= rel * MG_KAP[1])) {
            std::cout << "    !! R=4 的 kappa 不是 R=3..5 的中点" << std::endl; bad++;
        }
        // R = 6/7/8: 系数【平夹到 R=5】(不外推; 门限自己还是会随 P(R) 变, 变的是系数)。
        // 比的是"同一个 double"到 1e-12 相对 —— 抠系数这一步本身有 1~2 ulp 的除法/减法噪声
        // ((c0·P)/P 不保证逐位回到 c0), 所以这里不能用 ==。
        for (int R = 6; R <= 8; R++) {
            if (!(fabs(c0v[R - 1] - c0v[4]) <= rel * MG_C0[2])
             || !(fabs(kav[R - 1] - kav[4]) <= rel * MG_KAP[2])) {
                std::cout << "    !! R=" << R << " 的系数没有被平夹到 R=5 的系数 (c0 "
                          << c0v[R - 1] << " vs " << c0v[4] << ", kappa "
                          << kav[R - 1] << " vs " << kav[4] << ")" << std::endl;
                bad++;
            }
        }
        // 方位检查: 插值只能落在两端之间; c0 随 R 单调不增 (表上如此, 插值不会把它翻过来)
        for (int R = 2; R <= 4; R++) {
            const double lo = fmin(c0v[R - 2], c0v[R]), hi = fmax(c0v[R - 2], c0v[R]);
            if (!(c0v[R - 1] >= lo - rel && c0v[R - 1] <= hi + rel)) {
                std::cout << "    !! R=" << R << " 的 c0 插到区间外" << std::endl; bad++;
            }
        }
        printf("[R 规则: 表点 (2.160/1.590/1.350, 4.589/5.225/4.841) 命中, R=2/4 取中点,"
               " R>5 平夹] ");
    }

    if (bad != 0) {
        std::cout << "FAIL (" << bad << " 项)" << std::endl;
        g_failed++;
        return;
    }
    PASS();
}

// -------------------------------------------------------------------------------------
// 2) 零假设蒙特卡洛: 用生产 fitRaw 量【叉乘模型为真】时被拒的概率 (力/力矩两条门分开报)。
//
// 两套噪声模型, 【都报】—— 免得结论挂在一套噪声模型上:
//   A "brief 口径": 只有姿态内噪声。逐姿态均值按 N(0, sd²/N) 抖 (sd 与 N 都照夹具),
//     申报给 fitRaw 的也正是 (n = N1304, var = sd²) —— 于是门限读到的 floor² 与真实抖动
//     【逐位一致】。这一套里"回到同一姿态再采一次"只差噪声, 尺子最细, 是最保守的一套。
//   B "同复现性口径": A 之外, 【每一次访问】再各自加一份 σ_sys 偏移 (逐通道), 大小取该次
//     采集【自己测出来的】repeatSys[]。实机上"回到同一个位姿"的离散是真有的 (重复对差值
//     表里就看得见), 不把它放进零假设等于让尺子比现实细 —— 那会做出一个假的冤枉率。
//   两套的差就是"零假设的另一个自由度", 报出来才知道结论稳不稳。
// -------------------------------------------------------------------------------------

static const int MOMENT_MC_ITERS = 2000;

static double g_mcNull[3][2][MOMENT_MC_ITERS];   // [采集][噪声模型][第几次] -> 失拟统计量 (-1 = 没算出来)

struct McTally {
    int iters;
    int anyReject;       // fitRaw 返回 false (任何一种理由)
    int statusBad;       // 尺子不齐 (通道冻住/有洞/没有对/自由度 0)
    int forceReject;     // 尺子齐备、力通道判据被拒
    int momentReached;   // 走到力矩判据的次数 (力通道已过)
    int momentReject;    // 力矩判据被拒
    int otherReject;     // 两条门都过了, 仍被 fitRaw 的其它自检拒 (质量尺度/cond/秩亏)
};

static McTally g_mcTally[3][2];
static double  g_mcObs[3];

static void mgRunNull(int capIdx, int variant, const MomentCaptureData& d,
                      const PayloadCalibration::PoseNoise* nz,
                      const double A[9], const double bF[3], const double cS[3], const double bM[3],
                      const double sysF[3], const double sysM[3],
                      McTally& t, double* statOut)
{
    for (int it = 0; it < MOMENT_MC_ITERS; it++) statOut[it] = -1.0;
    t.iters = MOMENT_MC_ITERS;
    t.anyReject = t.statusBad = t.forceReject = 0;
    t.momentReached = t.momentReject = t.otherReject = 0;

    MgRng rng;
    rng.seed(20260919ULL + 1000ULL * (unsigned long long)capIdx
                          + 10ULL * (unsigned long long)variant);
    PayloadCalibration::PoseNoise nzSim[MG_MAXN];
    for (int i = 0; i < d.n; i++) nzSim[i] = nz[i];

    double F[MG_MAXN][3], M[MG_MAXN][3];
    for (int it = 0; it < MOMENT_MC_ITERS; it++) {
        // (a) 从【该次采集自己拟合出来的】A / b_F / c_s / b_M 造"真值" —— 叉乘模型为真
        for (int i = 0; i < d.n; i++) {
            double g[3];
            TcpCalibration::gravitySensorFrameAtYaw(d.poses[i], 0.0, g);
            double w[3];
            for (int a = 0; a < 3; a++)
                w[a] = A[a * 3 + 0] * g[0] + A[a * 3 + 1] * g[1] + A[a * 3 + 2] * g[2];
            const double muF[3] = { bF[0] + w[0], bF[1] + w[1], bF[2] + w[2] };
            const double muM[3] = { bM[0] + cS[1] * w[2] - cS[2] * w[1],
                                    bM[1] + cS[2] * w[0] - cS[0] * w[2],
                                    bM[2] + cS[0] * w[1] - cS[1] * w[0] };
            for (int a = 0; a < 3; a++) {
                // 姿态内噪声: 均值的 1σ = sd/√N (即门限读到的 floor)
                const double sF = d.sdF[i][a] / sqrt((double)d.nsamp[i]);
                const double sM = d.sdM[i][a] / sqrt((double)d.nsamp[i]);
                double eF = sF * rng.norm();
                double eM = sM * rng.norm();
                if (variant == 1) {          // 每一次访问各一份 σ_sys 偏移 (逐通道)
                    eF += sqrt(sysF[a]) * rng.norm();
                    eM += sqrt(sysM[a]) * rng.norm();
                }
                F[i][a] = muF[a] + eF;
                M[i][a] = muM[a] + eM;
            }
        }
        // (b) 生产判决 (与实机路径逐字同参: 同一个 nz、同一串重复对、同一个 policy)
        PayloadCalibration::RawFit fit;
        const bool ok = PayloadCalibration::fitRaw(d.poses, F, M, d.n, fit, nzSim,
                                                   d.reps, d.repCount,
                                                   PayloadCalibration::MODEL_FORM_REQUIRED);
        if (!ok) t.anyReject++;
        if (fit.modelFormStatus != PayloadCalibration::MODEL_FORM_OK) { t.statusBad++; continue; }
        if (!(fit.chi2RepForceRatio < fit.chi2RepForceLimit)) { t.forceReject++; continue; }
        if (fit.lackOfFitMomentDof > 0) {
            t.momentReached++;
            statOut[it] = fit.lackOfFitMomentRatio;
            // 力矩门拒了就【不再往"其它自检"里数】—— 两条出路只能占一条。
            if (!(fit.lackOfFitMomentRatio < fit.lackOfFitMomentLimit)) { t.momentReject++; continue; }
        }
        if (!ok) t.otherReject++;
    }
}

// 观测值在零分布里的百分位 (严格小的算 1, 相等的算 1/2)。无效样本 (统计量没算出来) 不计。
static double mgPercentile(const double* v, int n, double x) {
    int below = 0, equal = 0, valid = 0;
    for (int i = 0; i < n; i++) {
        if (!(v[i] >= 0.0)) continue;
        valid++;
        if (v[i] < x) below++;
        else if (v[i] == x) equal++;
    }
    return (valid > 0) ? ((double)below + 0.5 * (double)equal) / (double)valid : -1.0;
}

static double mgMean(const double* v, int n) {
    double s = 0.0; int c = 0;
    for (int i = 0; i < n; i++) if (v[i] >= 0.0) { s += v[i]; c++; }
    return c ? s / (double)c : -1.0;
}

static int mgCmpDouble(const void* a, const void* b) {
    const double x = *(const double*)a, y = *(const double*)b;
    return (x < y) ? -1 : ((x > y) ? 1 : 0);
}

// 零分布的分位数 (最近秩法, 只在有效样本上) —— 纯报告量。
static double mgQuantile(const double* v, int n, double p) {
    static double buf[MOMENT_MC_ITERS];
    int c = 0;
    for (int i = 0; i < n; i++) if (v[i] >= 0.0) buf[c++] = v[i];
    if (c <= 0) return -1.0;
    qsort(buf, (size_t)c, sizeof(double), mgCmpDouble);
    int idx = (int)(p * (double)(c - 1) + 0.5);
    if (idx < 0) idx = 0;
    if (idx >= c) idx = c - 1;
    return buf[idx];
}

static void test_moment_gate_null_false_reject_rate() {
    std::cout << "  moment_gate_null_false_reject_rate..." << std::endl;
    static const char* VARNAME[2] = { "A(只有逐姿态噪声)", "B(+每次访问 sigma_sys)" };

    const clock_t t0 = clock();
    mgMuteStderr();
    for (int k = 0; k < 3; k++) {
        MomentCaptureData d;
        const char* used = nullptr;
        if (!mgLoad(MG_CAPS[k].fixture, d, used)) {
            mgUnmuteStderr();
            std::cout << "    FAIL: 读不到夹具 " << MG_CAPS[k].fixture << std::endl;
            g_failed++;
            return;
        }
        PayloadCalibration::PoseNoise nz[MG_MAXN];
        mgDeclareNoise(d, nz);
        // 实机那一次的拟合 —— 被拒也要 A / b_F / c_s / b_M (线性解在, 头文件的契约如此):
        // 蒙特卡洛的"真值"就是从这几个数来的。
        PayloadCalibration::RawFit real;
        PayloadCalibration::fitRaw(d.poses, d.F, d.M, d.n, real, nz, d.reps, d.repCount,
                                   PayloadCalibration::MODEL_FORM_REQUIRED);
        const double obs = real.lackOfFitMomentRatio;
        mgUnmuteStderr();

        printf("    [%s] n=%d pairs=%d  实测 lackOfFitMomentRatio = %.6g (门限 %.6g, %s)\n",
               MG_CAPS[k].tag, d.n, d.repCount, obs, real.lackOfFitMomentLimit,
               (obs < real.lackOfFitMomentLimit) ? "过" : "拒");
        printf("         σ_sys,M = (%.3g, %.3g, %.3g);  σ_rep,M = (%.4g, %.4g, %.4g) N·m\n",
               sqrt(real.repeatSysM[0]), sqrt(real.repeatSysM[1]), sqrt(real.repeatSysM[2]),
               real.repeatSigmaM[0], real.repeatSigmaM[1], real.repeatSigmaM[2]);
        // 门限自己读的那个比值 (三通道"和的比", 与 modelFormLimit 同一口径) —— 门限的宽窄由它定,
        // 所以它必须与判决一起报。力矩那一半与力那一半分开算。
        static double rM = 0.0, rF = 0.0;
        {
            double sM = 0, fM = 0, sF = 0, fF = 0;
            for (int a = 0; a < 3; a++) {
                sM += real.repeatSysM[a]; fM += real.repeatFloorM[a];
                sF += real.repeatSysF[a]; fF += real.repeatFloorF[a];
            }
            rM = (fM > 0.0) ? sM / fM : 0.0;
            rF = (fF > 0.0) ? sF / fF : 0.0;
        }
        printf("         门限读到的 r = Σσ_sys²/Σfloor²:  力矩 %.4g,  力 %.4g\n", rM, rF);

        for (int v = 0; v < 2; v++) {
            McTally t;
            mgMuteStderr();
            mgRunNull(k, v, d, nz, real.A, real.bF, real.cS, real.bM,
                      real.repeatSysF, real.repeatSysM, t, g_mcNull[k][v]);
            mgUnmuteStderr();
            g_mcTally[k][v] = t;
            const double pct = mgPercentile(g_mcNull[k][v], MOMENT_MC_ITERS, obs);
            printf("         零假设 %s: %d 次; 任何理由被拒 %d (%.3f%%);  尺子不齐 %d;"
                   "  力通道拒 %d (%.3f%%);  走到力矩门 %d 次, 被拒 %d (%.3f%%);  其它自检拒 %d\n",
                   VARNAME[v], t.iters, t.anyReject, 100.0 * t.anyReject / t.iters,
                   t.statusBad, t.forceReject, 100.0 * t.forceReject / t.iters,
                   t.momentReached, t.momentReject,
                   100.0 * (t.momentReached ? (double)t.momentReject / (double)t.momentReached : 0.0),
                   t.otherReject);
            printf("                  零分布统计量: 均值 %.4g, 中位 %.4g, 90%% %.4g, 99%% %.4g,"
                   " 最大 %.4g  ->  夹具值 %.6g 落在 %.2f 百分位;  控制台值 %.4g 落在 %.2f 百分位\n",
                   mgMean(g_mcNull[k][v], MOMENT_MC_ITERS),
                   mgQuantile(g_mcNull[k][v], MOMENT_MC_ITERS, 0.50),
                   mgQuantile(g_mcNull[k][v], MOMENT_MC_ITERS, 0.90),
                   mgQuantile(g_mcNull[k][v], MOMENT_MC_ITERS, 0.99),
                   mgQuantile(g_mcNull[k][v], MOMENT_MC_ITERS, 1.0), obs, 100.0 * pct,
                   MG_CAPS[k].conRatio,
                   100.0 * mgPercentile(g_mcNull[k][v], MOMENT_MC_ITERS, MG_CAPS[k].conRatio));
        }
        g_mcObs[k] = obs;
    }
    // 性能与规模【照实报】, 否则"2000 次"只是个没有代价的数字。
    const double secs = (double)(clock() - t0) / (double)CLOCKS_PER_SEC;
    printf("    蒙特卡洛: 3 采集 x 2 噪声模型 x %d 次 = %d 次生产 fitRaw, 用时 %.1f s\n",
           MOMENT_MC_ITERS, 3 * 2 * MOMENT_MC_ITERS, secs);

    // ---- 冻结的实测值 (确定性种子 -> 逐次可复现; 容差见下) ----
    // 力矩分支的冤枉率 (分子是【走到力矩门】的次数, 分母是走到的次数)。力分支一次都没拒
    // (唯一的例外: 15:25 的 B 里 2 次, 记在下面), 所以两条门的分母几乎相同。
    //
    // ★ 2026-09-19 【力矩那一列被显式重导过】(127/438/33/90/20/18 -> 1/68/0/9/0/1) —— 门限从
    //   modelFormLimit 换成了 c0·LIMIT_prod + κ·e, 零假设的统计量【一个比特都没动】(它只由
    //   数据与尺子决定), 只有切的那一刀挪了位。这就是"统计量/门限被改过就会是这样"那句注解
    //   要求的变红方式; 重导的合法性由两件事保证:
    //     · 门限本身有两个独立的锚 (新加的 moment_gate_limit_is_c0_times_prod_plus_kappa_times_e
    //       把 e 钉在离线报告的五位小数上, 把门限钉在 c0·LIMIT_prod + κ·e 上; 金标表把门限的
    //       数值钉在 1e-6);
    //     · 零分布那一边一动不动: REF_REACHED 与上面印出来的零分布分位数 (均值/中位/99%/max)
    //       全部逐位未变 —— 只有"被拒的次数"这一列动了。
    //   力通道那一列 (0/2/0/0/0/0) 与 REF_REACHED 一样【逐位未变】: 本任务没碰力那一条支路。
    //
    // 【容差 ±5 次】: 只容"边界上几次抽样翻转"(换编译器/libm 时 FP 抖动会翻掉几个边缘抽样),
    // 不是给实现改动留的余地。⚠ 重导之后力矩那一列变成了个位数, 所以这个 ±5 对【小格】而言
    // 比从前松 (0 与 5 都在容差里)。紧的那道闸在别处: 门限的数值由 refLimit 与上面那条
    // c0·LIMIT_prod + κ·e 的断言钉到 1e-6 / 1e-12, 统计量的分布由 (i)/(ii) 那条分界
    // (零分布 max vs 实测, 两套噪声模型都判) 钉住。这里这一列因此是【粗验】。
    static const int REF_MOMENT_REJECT[3][2] = { { 1, 68 }, { 0, 9 }, { 0, 1 } };
    static const int REF_FORCE_REJECT[3][2]  = { {   0,   2 }, {  0,  0 }, {  0,  0 } };
    static const int REF_REACHED[3][2]       = { {2000,1998 }, {2000,2000 }, {2000,2000 } };
    static const int MC_TOL = 5;
    for (int k = 0; k < 3; k++) {
        for (int v = 0; v < 2; v++) {
            const McTally& t = g_mcTally[k][v];
            if (t.iters != MOMENT_MC_ITERS) {
                std::cout << "    FAIL: [" << MG_CAPS[k].tag << "/" << VARNAME[v] << "] 只跑了 "
                          << t.iters << " 次" << std::endl;
                g_failed++;
                return;
            }
            // 计数自洽 (四条出路必须把每一次都分完, 且只能占一条)
            if (t.anyReject != t.statusBad + t.forceReject + t.momentReject + t.otherReject
             || t.statusBad + t.forceReject + t.momentReached != t.iters) {
                std::cout << "    FAIL: [" << MG_CAPS[k].tag << "/" << VARNAME[v]
                          << "] 计数不自洽 (拒 " << t.anyReject << " != 尺子不齐 " << t.statusBad
                          << " + 力 " << t.forceReject << " + 力矩 " << t.momentReject
                          << " + 其它 " << t.otherReject << ";  走到力矩门 " << t.momentReached
                          << " + 力拒 " << t.forceReject << " + 尺子不齐 " << t.statusBad
                          << " != " << t.iters << ")" << std::endl;
                g_failed++;
                return;
            }
            const int dM = t.momentReject - REF_MOMENT_REJECT[k][v];
            const int dF = t.forceReject  - REF_FORCE_REJECT[k][v];
            const int dR = t.momentReached - REF_REACHED[k][v];
            if (dM > MC_TOL || dM < -MC_TOL || dF > MC_TOL || dF < -MC_TOL
             || dR > MC_TOL || dR < -MC_TOL) {
                std::cout << "    FAIL: [" << MG_CAPS[k].tag << "/" << VARNAME[v]
                          << "] 零假设下的计数变了: 力矩拒 " << t.momentReject << " (冻结 "
                          << REF_MOMENT_REJECT[k][v] << "), 力拒 " << t.forceReject << " (冻结 "
                          << REF_FORCE_REJECT[k][v] << "), 走到力矩门 " << t.momentReached
                          << " (冻结 " << REF_REACHED[k][v] << ") —— 容差 ±" << MC_TOL
                          << " 次。统计量/门限被改过就会是这样。" << std::endl;
                g_failed++;
                return;
            }
        }
    }

    // ---- (i)/(ii) 的分界, 钉成不变量 ----
    // 分界线 = 【零分布的最大值】: 实测值在它【之内】-> 这套装置本身就产出这么高的统计量,
    // 门限切错了 (ii); 在它【之外】-> 自由模型找到了零假设里根本没有的结构 (i)。
    // 两套噪声模型都各判一次, 结论必须一致 (否则说明结论挂在噪声模型上, 不能报)。
    int verdict_i = 0, verdict_ii = 0;
    for (int k = 0; k < 3; k++) {
        for (int v = 0; v < 2; v++) {
            const double mx = mgQuantile(g_mcNull[k][v], MOMENT_MC_ITERS, 1.0);
            if (g_mcObs[k] > mx) verdict_i++;
            else                  verdict_ii++;
            printf("    [%s/%s] 零分布最大 %.4g  vs  实测 %.6g  ->  %s\n",
                   MG_CAPS[k].tag, VARNAME[v], mx, g_mcObs[k],
                   (g_mcObs[k] > mx) ? "(i) 零假设解释不了, 力矩模型真的少了结构"
                                     : "(ii) 落在零分布之内, 门限自己切错了");
        }
    }
    // 冻结: 15:25 两套都落在零分布【之内】(ii); 15:30、15:33 两套都在【之外】(i)。
    // 这条断言是本次测量的结论本身 —— 谁改统计量/门限、或换夹具, 它必须变红。
    if (verdict_i != 4 || verdict_ii != 2) {
        std::cout << "    FAIL: (i)/(ii) 的分界变了: 落在零分布之外的次数 = " << verdict_i
                  << " (冻结 4), 落在之内 = " << verdict_ii << " (冻结 2)" << std::endl;
        g_failed++;
        return;
    }
    PASS();
}

// =====================================================================================
// ★★★ Task 6 验收 —— 本地补偿的【姿态无关性】: 全量 (新) vs 残余 (旧), 逐通道, 四份采集
// =====================================================================================
//
// 【为什么是这条判据, 而不是"门过不过"】
//   本地补偿的定义就是"补偿后的读数应当与姿态无关"(负载正确时, 残余读数与姿态无关)。
//   门 (chi2Rep / lackOfFit) 问的是"模型形式与数据的复现性一致吗", 它与"补偿后还剩多少
//   姿态相关"不是同一个量 —— 用门来判这次切换, 是拿另一件事的尺子量这件事。
//
// 【量的是什么】
//   对每一份采集: 用【该次采集解出的】参数做全量补偿, 再量补偿后读数在姿态之间的散布。
//     dep = sqrt( (1/(3n)) Σ_a Σ_i ( c[a,i] − mean_i c[a,i] )² )     [N] 或 [N·m]
//   力三轴合并成一个数、力矩三轴合并成一个数 —— 【不合成一个总账】: 力矩通道的模型本项目
//   自己记录为不完整 (docs: moment-gate-diagnosis-report.md), 合成会把它的缺点摊到力上,
//   读起来像"两个通道一起好/一起坏"。
//   两侧都用【同一个 dep】—— 口径一致才有可比性。
//
// 【新 (全量) 这一侧跑的是生产代码本身】
//   ForceCompensation::setCalibration(fit.A, fit.bF, fit.bM, fit.cS) + step() —— 不是把公式
//   抄一遍。每调用一次前重做 init()+setCalibration(), 于是读到的是【纯模型】:
//   step() 的输出在第 7 步就算完, 在线 EMA 与惯性项都在其后 (第 8 步), 且新初始化的运动
//   估计器判"静止"(vel=acc=0)⇒ Fi=0。所以这个读数逐位等于 sixForceRaw − b_F − A·g
//   (力矩 − b_M − c_s×(A·g)), 不含任何在线漂移。
//
// 【旧 (残余) 这一侧是"它最好的样子", 不是稻草人】
//   旧模型: compensated = fd.raw(@576) − b_F − Δm·g_ψ ; 力矩 − b_M − Δp × g_ψ。
//   现场那次标定把 Δm/Δp/ψ 连同 TARE 零偏一起写下, 而离线只有这份采集, 所以这里在
//   【同一份采集】上按最小二乘把 (ψ, Δm, Δp, b_F, b_M) 全部重新定一遍:
//     · ψ 扫 [-180,180]/0.5° 取力+力矩残差平方和最小者 —— 与生产 old-solver 的判据同式
//       (PayloadCalibration::solve 的 fitAtYaw); 给它最优的 ψ 只会让"新模型更好"更难成立;
//     · b_F / b_M 也按 LS 定 (现场是 TARE 采的, 离线没有那一次采样——只能给最有利的值)。
//   因为 b 是截距, LS 残差的均值恒为 0 ⇒ dep_old 就是 LS 残差的 RMS, 与 dep_new 同口径。
//
// 【第二个口径: 留一交叉验证 (LOO)】
//   有人会问"全量模型 12 个自由参数对旧模型 4 个, 在样本内当然拟合得更好"。这条质疑是
//   对的, 所以再加一列: 每次留出一个姿态、在其余姿态上重新定【两侧的参数】, 再量【被留出
//   那个姿态】的补偿后读数 (的散布)。样本内拟合优度不能靠"多几个参数"赢下这一列。
//   (旧模型的 ψ 在 LOO 里固定为整份采集扫出来的那个 —— ψ 是模型形式参数, 每折重扫 721 次
//   既无必要也慢; 留出姿态对 ψ 的影响本来就只有半个步长量级。)
//
// ⚠ 两侧吃的是【不同通道】(@576 对 @1304) —— 这正是本次切换的内容, 不是不公平:
//   问题是"哪一条补偿路径留下的读数更与姿态无关", 而两条路径各自的输入就是它们各自的输入。

static const int T6_MAXN = 16;

struct T6Capture {
    const char* label;
    const char* file;
    int    n;
    double poses[T6_MAXN][6];                       // [x,y,z,rx,ry,rz] (求解器序)
    double F576[T6_MAXN][3],  M576[T6_MAXN][3];     // 旧模型的输入 (@576)
    double F1304[T6_MAXN][3], M1304[T6_MAXN][3];    // 新模型的输入 (@1304)
};

// 读一份采集。列布局由【列数】判: 18 列 (12:38 那批) 或 25 列 (15:xx 那批, 多了 N 与 sd)。
// 两种布局的前 18 列逐列相同 —— 这也是为什么可以共存 (夹具头部自己写着列名)。
static bool t6Load(T6Capture& cap) {
    static const char* DIRS[4] = { "fixtures/", "tests/fixtures/",
                                   "Touch_Client/tests/fixtures/",
                                   "../../Touch_Client/tests/fixtures/" };
    FILE* fp = nullptr;
    for (int i = 0; i < 4 && !fp; i++) {
        char path[512];
        snprintf(path, sizeof(path), "%s%s", DIRS[i], cap.file);
        fp = fopen(path, "r");
    }
    if (!fp) return false;

    cap.n = 0;
    char line[1024];
    while (fgets(line, sizeof(line), fp)) {
        const char* q = line;
        while (*q == ' ' || *q == '\t') q++;
        if (*q == '\0' || *q == '\r' || *q == '\n' || *q == '#') continue;
        if (cap.n >= T6_MAXN) { fclose(fp); return false; }
        double c[25];
        const int nc = mgSplitRow(q, c, 25);
        // 【不静默跳行】: 列数不是这两种 = 布局变了, 拿它去作结论只会得出一个假的
        if (nc != 18 && nc != 25) { fclose(fp); return false; }
        double src[6];
        for (int a = 0; a < 6; a++) src[a] = c[a];
        repackPoseRow(src, cap.poses[cap.n]);
        for (int a = 0; a < 3; a++) {
            cap.F576[cap.n][a]  = c[6 + a];
            cap.M576[cap.n][a]  = c[9 + a];
            cap.F1304[cap.n][a] = c[12 + a];
            cap.M1304[cap.n][a] = c[15 + a];
        }
        cap.n++;
    }
    fclose(fp);
    return cap.n >= 4;   // 少于 4 个姿态连 12 个参数都定不下来
}

// K×K 正规方程 (Gauss 消元, 部分主元)。退化返回 false。K ≤ 6。
static bool t6SolveNormal(int K, double AtA[6][6], const double Atb[6], double x[6]) {
    double M[6][7];
    for (int r = 0; r < K; r++) {
        for (int c = 0; c < K; c++) M[r][c] = AtA[r][c];
        M[r][K] = Atb[r];
    }
    for (int col = 0; col < K; col++) {
        int piv = col;
        for (int r = col + 1; r < K; r++) if (fabs(M[r][col]) > fabs(M[piv][col])) piv = r;
        if (fabs(M[piv][col]) < 1e-12) return false;
        if (piv != col) for (int c = col; c <= K; c++) { double t = M[col][c]; M[col][c] = M[piv][c]; M[piv][c] = t; }
        const double d = M[col][col];
        for (int c = col; c <= K; c++) M[col][c] /= d;
        for (int r = 0; r < K; r++) {
            if (r == col) continue;
            const double f = M[r][col];
            for (int c = col; c <= K; c++) M[r][c] -= f * M[col][c];
        }
    }
    for (int i = 0; i < K; i++) x[i] = M[i][K];
    return true;
}

// 旧 (残余) 模型在给定 ψ 下的最小二乘。
//   F_i = b_F + Δm·g_i(ψ)          -> 未知 [bFx,bFy,bFz,Δm]
//   M_i = b_M + Δp × g_i(ψ)        -> 未知 [bMx,bMy,bMz,dp0,dp1,dp2]
// 力矩那三个方程按叉乘展开 (cross(dp,g)): Mx = bMx + dp1·g2 − dp2·g1, 等等。
// skip = 要在拟合中【排除】的姿态下标 (-1 = 不排除), 供 LOO 用。
// 输出 ssF/ssM = 拟合残差平方和 (只在【参与拟合的姿态】上累计)。
// ⚠ dmOut 必须被 LOO 那条路用起来 (2026-09-19 复审): 旧模型是
//      compensated = @576 − b_F − Δm·g
//   只减 b_F 会漏掉 Δm·g 那一项, 把 LOO 那一列的旧模型残差抬高 —— 于是"旧 vs 新"的
//   对比里旧的那一侧被冤枉, 而这一列是打印出来给人看的。
static bool t6FitOldAtYaw(const T6Capture& cap, double psiDeg, int skip,
                          double bF[3], double bM[3], double dp[3],
                          double& ssF, double& ssM, double* dmOut = nullptr) {
    double A4[6][6] = {{0}}; double b4[6] = {0};
    double A6[6][6] = {{0}}; double b6[6] = {0};
    for (int i = 0; i < cap.n; i++) {
        if (i == skip) continue;
        double g[3];
        TcpCalibration::gravitySensorFrameAtYaw(cap.poses[i], psiDeg, g);
        for (int a = 0; a < 3; a++) {
            double row[4] = {0, 0, 0, 0};
            row[a] = 1.0; row[3] = g[a];
            for (int r = 0; r < 4; r++) {
                for (int c = 0; c < 4; c++) A4[r][c] += row[r] * row[c];
                b4[r] += row[r] * cap.F576[i][a];
            }
        }
        const double rows[3][6] = {
            { 1.0, 0.0, 0.0,   0.0,   g[2], -g[1] },
            { 0.0, 1.0, 0.0,  -g[2],  0.0,   g[0] },
            { 0.0, 0.0, 1.0,   g[1], -g[0],  0.0  }
        };
        for (int r = 0; r < 3; r++) {
            for (int c1 = 0; c1 < 6; c1++) {
                for (int c2 = 0; c2 < 6; c2++) A6[c1][c2] += rows[r][c1] * rows[r][c2];
                b6[c1] += rows[r][c1] * cap.M576[i][r];
            }
        }
    }
    double x4[6], x6[6];
    if (!t6SolveNormal(4, A4, b4, x4)) return false;
    if (!t6SolveNormal(6, A6, b6, x6)) return false;
    for (int a = 0; a < 3; a++) { bF[a] = x4[a]; bM[a] = x6[a]; }
    dp[0] = x6[3]; dp[1] = x6[4]; dp[2] = x6[5];
    const double dm = x4[3];
    if (dmOut) *dmOut = dm;

    ssF = 0.0; ssM = 0.0;
    for (int i = 0; i < cap.n; i++) {
        if (i == skip) continue;
        double g[3];
        TcpCalibration::gravitySensorFrameAtYaw(cap.poses[i], psiDeg, g);
        for (int a = 0; a < 3; a++) {
            const double e = bF[a] + dm * g[a] - cap.F576[i][a];
            ssF += e * e;
        }
        const double mx = bM[0] + dp[1] * g[2] - dp[2] * g[1];
        const double my = bM[1] + dp[2] * g[0] - dp[0] * g[2];
        const double mz = bM[2] + dp[0] * g[1] - dp[1] * g[0];
        const double e0 = mx - cap.M576[i][0];
        const double e1 = my - cap.M576[i][1];
        const double e2 = mz - cap.M576[i][2];
        ssM += e0 * e0 + e1 * e1 + e2 * e2;
    }
    return true;
}

// 扫 ψ 找最优 (判据 = 力 + 力矩残差平方和最小, 与生产旧求解器同式)。
static bool t6ScanYaw(const T6Capture& cap, int skip, double& psiBest,
                      double bF[3], double bM[3], double dp[3],
                      double& ssF, double& ssM) {
    double best[3], bbF[3], bbM[3], bdp[3];
    bool have = false;
    double bestSum = 0.0;
    for (int s = 0; s <= 720; s++) {
        const double psi = -180.0 + s * 0.5;
        double fF[3], fM[3], fdp[3], sF = 0.0, sM = 0.0;
        if (!t6FitOldAtYaw(cap, psi, skip, fF, fM, fdp, sF, sM)) continue;
        if (!have || sF + sM < bestSum) {
            have = true; bestSum = sF + sM; psiBest = psi;
            for (int a = 0; a < 3; a++) { bbF[a] = fF[a]; bbM[a] = fM[a]; bdp[a] = fdp[a]; }
        }
    }
    if (!have) return false;
    // 用最优 ψ 重跑一次, 把残差与参数一并取出 (扫描时存的是 bestSum, 这里要的是分量)
    if (!t6FitOldAtYaw(cap, psiBest, skip, bbF, bbM, bdp, ssF, ssM)) return false;
    for (int a = 0; a < 3; a++) { bF[a] = bbF[a]; bM[a] = bbM[a]; dp[a] = bdp[a]; }
    return true;
}

// 散布 (姿态无关性的反面): 补偿后读数对姿态均值的 RMS, 三轴合并。
static double t6Spread(const double c[T6_MAXN][6], int n, int i0) {
    if (n <= 1) return 0.0;
    double mean[3] = {0, 0, 0};
    for (int i = 0; i < n; i++) for (int a = 0; a < 3; a++) mean[a] += c[i][i0 + a] / n;
    double ss = 0.0;
    for (int i = 0; i < n; i++)
        for (int a = 0; a < 3; a++) { const double d = c[i][i0 + a] - mean[a]; ss += d * d; }
    return sqrt(ss / (3.0 * n));
}

// 本地全量模型【应该】算出什么 —— 只给夹具重放用。
//
// 【为什么这里可以照写一遍公式】: 运行时一致性闸门 (2026-09-19) 要求【判据参考量】与
// 本地模型算出来的东西一致才放行, 所以"想量模型输出"就必须先在参考量那一侧造一个一致的
// 值。而闸门拒绝时 compensated 会被置零 —— 那量到的就不是模型输出, 而是 0。
// ⚠ 参考量是哪一路由 ForceCompensation.cpp 的 guardReferenceValue 一处定义; 下面凡是要
//   "让闸门放行"的地方, 喂的都必须【是那一侧】(现在的实现是 fd.tcpForce)。喂错边会让这些
//   用例【照旧绿】但测的东西变成 0 与 0 相等 —— 空洞的绿, 比红更坏。
//
// 两处约定都【不在这里另立】:
//   · 重力 → 共享实现 TcpCalibration::gravitySensorFrameAtYaw(pose, 0.0, g) (ψ 传 0,
//     因为 A 是自由 3×3, 安装旋转已被它吸收);
//   · 公式 compensated = six − b − A·g / c_s×(A·g) 由 test_force_compensation 逐条钉住
//     (comp_gravity_goes_through_A / comp_moment_is_cross_of_Ag)。
// 惯性项不在这里出现: 重放里每个姿态都是静态单帧 (估计器 vel/acc 恒为 0), Fi ≡ 0。
//
// ⚠⚠ 但"照写一遍公式"这件事的【代价】必须写在旁边 (复审 Important 4): 用本函数造出来的
//   【参考量那一侧的值】(fd.tcpForce, 见上面 guardReferenceValue 那一条) 与本地模型输出
//   【恒等】(d ≡ 0 是代数结论, 不是测量结论)。所以任何拿它当反面对照
//   的用例, 量到的都是"闸门放行这条路通不通", 【量不到容差松紧】—— 容差放到 3 倍还是
//   36/36 放行。反面对照只配当一个【分支存在性】的证据。
static void t6LocalModel(const PayloadCalibration::RawFit& fit, const double pose[6],
                         const double six[6], double out[6]) {
    double g[3];
    TcpCalibration::gravitySensorFrameAtYaw(pose, 0.0, g);
    double Fg[3];
    for (int a = 0; a < 3; a++)
        Fg[a] = fit.A[3 * a] * g[0] + fit.A[3 * a + 1] * g[1] + fit.A[3 * a + 2] * g[2];
    const double Mg[3] = { fit.cS[1] * Fg[2] - fit.cS[2] * Fg[1],
                           fit.cS[2] * Fg[0] - fit.cS[0] * Fg[2],
                           fit.cS[0] * Fg[1] - fit.cS[1] * Fg[0] };
    for (int a = 0; a < 3; a++) {
        out[a]     = six[a]     - fit.bF[a] - Fg[a];
        out[3 + a] = six[3 + a] - fit.bM[a] - Mg[a];
    }
}

static void test_runtime_compensation_pose_independence() {
    TEST(runtime_compensation_pose_independence);

    static const char* FILES[4] = {
        "calib_poses_2026-09-19.txt",       // 12:38:19, 7 姿态, 18 列
        "calib_poses_2026-09-19_1525.txt",  // 15:25:23, 9 姿态, 25 列 (含重复姿态对)
        "calib_poses_2026-09-19_1530.txt",  // 15:30:xx, 10 姿态
        "calib_poses_2026-09-19_1533.txt"   // 15:33:xx, 10 姿态
    };
    static const char* LABELS[4] = { "12:38", "15:25", "15:30", "15:33" };

    int worseF = 0, worseM = 0;
    std::cout << std::endl;
    for (int k = 0; k < 4; k++) {
        T6Capture cap;
        cap.label = LABELS[k];
        cap.file  = FILES[k];
        if (!t6Load(cap)) {
            std::cout << "    FAIL: 夹具 " << FILES[k] << " 读不到 (它是已入库的只读副本;"
                         " 缺了说明检出坏了, 不是这台机器没采过)" << std::endl;
            g_failed++;
            return;
        }

        // ---------- 新 (全量) ----------
        PayloadCalibration::RawFit fit;
        // fitRawLinear: 只做线性拟合, 不做任何自检 —— 采样本地的量测就该用它
        // (自检是"能不能采纳"的门, 与本验收无关; 用 fitRaw 会被门拦住而量不成)。
        if (!PayloadCalibration::fitRawLinear(cap.poses, cap.F1304, cap.M1304, cap.n, fit)) {
            std::cout << "    FAIL: " << cap.label << " 的 @1304 线性拟合失败 (秩亏)" << std::endl;
            g_failed++;
            return;
        }
        static double compNew[T6_MAXN][6];
        for (int i = 0; i < cap.n; i++) {
            AppState::ForceData fd;
            for (int a = 0; a < 3; a++) {
                fd.sixForceRaw[a]     = cap.F1304[i][a];   // 力 x,y,z
                fd.sixForceRaw[3 + a] = cap.M1304[i][a];   // 力矩 x,y,z
            }
            // ⚠ 【一致性闸门要求【参考量】与本地模型一致才放行】(2026-09-19)。本用例量的是
            // 【模型输出】, 所以把【参考量那一侧】喂成"模型说多少就是多少" (t6LocalModel) ——
            // 喂错边会让闸门读到 0, 那时量到的是 0 而不是模型输出, 而它会以"口径自校不过"
            // 的样子红掉, 看的却不是它要测的东西。
            // (夹具只有 @576 / @1304 两列, 没有参考量那一路的列 —— 所以这里只能现算。)
            double mdl[6];
            for (int a = 0; a < 3; a++) {
                mdl[a]     = cap.F1304[i][a];
                mdl[3 + a] = cap.M1304[i][a];
            }
            t6LocalModel(fit, cap.poses[i], mdl, mdl);
            for (int a = 0; a < 6; a++) fd.tcpForce[a] = mdl[a];
            // ★ 2026-09-21 (Task 7): 还要声明【参考量可用】(帧新鲜 + 机械臂自报在线), 否则
            //   闸门判【参考量不可用】并拒绝 —— 那时量到的 compensated 是闸门置的 0, 于是
            //   下面那条"口径自校"会红, 而它红得【看不懂】(它量的是模型输出, 不是数据可用性)。
            //   本用例喂的参考量是现算的、一定有值 ⇒ 这一帧就是"机械臂报了读数"的那一帧;
            //   生产里这几件事由 RelayCore 在同一帧里一起做好。
            fd.isStale = false;
            fd.sixForceOnline = 1;
            ForceCompensation::init();                       // 干净的估计器状态 (见文件头说明)
            ForceCompensation::setCalibration(fit.A, fit.bF, fit.bM, fit.cS);
            ForceCompensation::step(fd, cap.poses[i]);
            for (int a = 0; a < 6; a++) compNew[i][a] = fd.compensated[a];
        }
        const double depNewF = t6Spread(compNew, cap.n, 0);
        const double depNewM = t6Spread(compNew, cap.n, 3);
        // 口径自校: b_F / b_M 都是截距 ⇒ 拟合残差的均值恒为 0 ⇒ 上面的散布应该逐位等于
        // fitRawLinear 报的 rms (残差 = 补偿后读数)。对不上就说明"量的东西"与"拟合的东西"
        // 不是同一件事 —— 那这条验收的整个读数都不可信。
        if (!(fabs(depNewF - fit.rmsForceN) < 1e-9 && fabs(depNewM - fit.rmsMomentNm) < 1e-9)) {
            std::cout << "    FAIL: " << cap.label << " 口径自校不过: 散布 ("
                      << depNewF << ", " << depNewM << ") vs fitRawLinear 的 rms ("
                      << fit.rmsForceN << ", " << fit.rmsMomentNm << ") —— 量错了东西"
                      << std::endl;
            g_failed++;
            return;
        }

        // ---------- 旧 (残余) ----------
        double psi = 0.0, ssF = 0.0, ssM = 0.0, bF[3], bM[3], dp[3];
        if (!t6ScanYaw(cap, -1, psi, bF, bM, dp, ssF, ssM)) {
            std::cout << "    FAIL: " << cap.label << " 的旧 (残余) 模型拟合失败" << std::endl;
            g_failed++;
            return;
        }
        const double depOldF = sqrt(ssF / (3.0 * cap.n));
        const double depOldM = sqrt(ssM / (3.0 * cap.n));

        // ---------- 留一交叉验证 ----------
        static double looNew[T6_MAXN][6], looOld[T6_MAXN][6];
        int looN = 0;
        for (int i = 0; i < cap.n; i++) {
            // 新: 在其余姿态上重解 A / b / c_s
            PayloadCalibration::RawFit f2;
            static double p2[T6_MAXN][6], F2[T6_MAXN][3], M2[T6_MAXN][3];
            int m = 0;
            for (int j = 0; j < cap.n; j++) {
                if (j == i) continue;
                for (int a = 0; a < 6; a++) p2[m][a] = cap.poses[j][a];
                for (int a = 0; a < 3; a++) { F2[m][a] = cap.F1304[j][a]; M2[m][a] = cap.M1304[j][a]; }
                m++;
            }
            if (!PayloadCalibration::fitRawLinear(p2, F2, M2, m, f2)) continue;
            AppState::ForceData fd;
            for (int a = 0; a < 3; a++) {
                fd.sixForceRaw[a]     = cap.F1304[i][a];
                fd.sixForceRaw[3 + a] = cap.M1304[i][a];
            }
            {   // 闸门参考量那一侧的值: 由【留一那一次】的模型现算 (见 test_runtime_compensation_*)
                double mdl[6];
                for (int a = 0; a < 3; a++) {
                    mdl[a]     = cap.F1304[i][a];
                    mdl[3 + a] = cap.M1304[i][a];
                }
                t6LocalModel(f2, cap.poses[i], mdl, mdl);
                for (int a = 0; a < 6; a++) fd.tcpForce[a] = mdl[a];
            }
            // ★ 2026-09-21 (Task 7): 同上一处 —— 喂了参考量就得声明【参考量可用】
            //   (帧新鲜 + 机械臂自报在线), 否则闸门判"不可用"、compensated 被置零,
            //   量出来的就不是模型输出了。
            fd.isStale = false;
            fd.sixForceOnline = 1;
            ForceCompensation::init();
            ForceCompensation::setCalibration(f2.A, f2.bF, f2.bM, f2.cS);
            ForceCompensation::step(fd, cap.poses[i]);
            for (int a = 0; a < 6; a++) looNew[looN][a] = fd.compensated[a];

            // 旧: ψ 固定为整份采集扫出来的那个, 只重定零偏与 Δm/Δp
            double fF[3], fM[3], fdp[3], sF = 0.0, sM = 0.0, fdm = 0.0;
            if (!t6FitOldAtYaw(cap, psi, i, fF, fM, fdp, sF, sM, &fdm)) continue;
            double g[3];
            TcpCalibration::gravitySensorFrameAtYaw(cap.poses[i], psi, g);
            // ⚠ 旧模型是 @576 − b_F − Δm·g —— 【两项都要减】。只减 b_F 会漏掉重力那一项,
            //   把这一列(旧模型)的残差抬高, 对比就变成"新模型赢在一个被冤枉的对手上"。
            for (int a = 0; a < 3; a++) looOld[looN][a] = cap.F576[i][a] - fF[a] - fdm * g[a];
            looOld[looN][3] = cap.M576[i][0] - (fM[0] + fdp[1] * g[2] - fdp[2] * g[1]);
            looOld[looN][4] = cap.M576[i][1] - (fM[1] + fdp[2] * g[0] - fdp[0] * g[2]);
            looOld[looN][5] = cap.M576[i][2] - (fM[2] + fdp[0] * g[1] - fdp[1] * g[0]);
            looN++;
        }
        const double looNewF = t6Spread(looNew, looN, 0);
        const double looNewM = t6Spread(looNew, looN, 3);
        const double looOldF = t6Spread(looOld, looN, 0);
        const double looOldM = t6Spread(looOld, looN, 3);

        printf("    %s (n=%2d, psi_old=%+7.1f deg)  力: 旧 %.4f -> 新 %.4f N   (%.2fx)   "
               "力矩: 旧 %.4f -> 新 %.4f N·m (%.2fx)\n",
               cap.label, cap.n, psi, depOldF, depNewF, depOldF / depNewF,
               depOldM, depNewM, depOldM / depNewM);
        printf("    %s  LOO                     力: 旧 %.4f -> 新 %.4f N   (%.2fx)   "
               "力矩: 旧 %.4f -> 新 %.4f N·m (%.2fx)\n",
               cap.label, looOldF, looNewF, looOldF / looNewF,
               looOldM, looNewM, looOldM / looNewM);

        if (!(depNewF < depOldF)) worseF++;
        if (!(depNewM < depOldM)) worseM++;
    }
    std::cout << "    (x 倍 = 旧/新, 越大越好; 力与力矩分开报, 不合账)" << std::endl;

    // 断言【两个通道都变好】。四份采集上量出来的就是两个都变好 (逐份的数字见上面的表,
    // 报告里也逐份列了), 所以这里两条都断言 —— 不是"力过了就算数"。
    // 若将来某一份的力矩那一侧翻转: 这条会红, 而表就在上面。先读表, 再决定是改模型还是
    // 改断言 —— 【不许】把力矩那条断言删掉换成一句打印: 那就是把"没变好"藏进一个绿的里面。
    CHECK(worseF == 0);
    CHECK(worseM == 0);
    PASS();
}

// ★★ 运行时一致性闸门在【四份实机采集】上怎么判 (2026-09-19, 用户指令 1/2)。
//
// 判据 (ForceCompensation::step 的 7b): 逐通道比较【本地全量模型输出的外力】与
// 【机械臂自报的参考量】(现为【通过关节电流计算】的那一路 —— 哪一路由 ForceCompensation.cpp
// 的 guardReferenceValue 一处定义, 那里也写着"为什么是那一路")。
// 两者都对时估计的是同一个量, 所以应当一致。
//
// 这一份是离线重放, 每个姿态喂一帧 (夹具的每一行本来就是该姿态的均值, 所以"一帧"就是
// 那个姿态的平均读数 —— 与实机上静置时 EMA 收敛到的东西是同一个数)。
//
// 【要证明的两件事】
//   (甲) 当前状态 (机械臂里存的负载是旧的) 下, 闸门【拒绝】—— 逐通道超限倍数印出来;
//        ⚠ 这半边现在【量不到】(见下面 (甲) 那一段里的 ⚠, 它【不许】被改绿);
//   (乙) 它【不是永远拒绝】: 把【参考量那一侧】换成与本地模型一致的值,
//        同样的姿态、同样的模型必须放行 ——
//        这条是反面对照, 没有它"拒绝"可能只是因为闸门坏了。
//   至于"发送正确负载之后应当放行" —— 那是实机上的事 (Task 8), 离线【量不到】,
//   所以这里不做任何"它会通过"的断言, 只把容差与当前的超限倍数摆出来。
// A 的三个奇异值 (降序)。测试侧独立算一遍 (不调被测函数): 对 AᵀA 做 Jacobi 求特征值。
// 用途: 容差的【量级】里有一项是"机械臂那一侧的模型类差" —— 它的力模型是"标量质量 ×
// 旋转"(3 个自由度), 而本地的 A 是自由 3×3; 这份差距的下界就是 A 的三个奇异值相对其
// 均值的最大偏离, 乘 g。见 runtime-guard-report.md 的容差推导。
static void t6SigmaA(const double A[9], double sg[3]) {
    double M[3][3];
    for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) {
        double s = 0.0;
        for (int k = 0; k < 3; k++) s += A[k * 3 + i] * A[k * 3 + j];
        M[i][j] = s;
    }
    for (int sweep = 0; sweep < 60; sweep++) {
        if (fabs(M[0][1]) + fabs(M[0][2]) + fabs(M[1][2]) < 1e-18) break;
        for (int p = 0; p < 2; p++) for (int q = p + 1; q < 3; q++) {
            if (fabs(M[p][q]) < 1e-18) continue;
            const double theta = (M[q][q] - M[p][p]) / (2.0 * M[p][q]);
            const double t = (theta >= 0 ? 1.0 : -1.0) / (fabs(theta) + sqrt(theta * theta + 1.0));
            const double c = 1.0 / sqrt(t * t + 1.0), s = t * c;
            for (int k = 0; k < 3; k++) {
                const double kp = M[k][p], kq = M[k][q];
                M[k][p] = c * kp - s * kq;  M[k][q] = s * kp + c * kq;
            }
            for (int k = 0; k < 3; k++) {
                const double pk = M[p][k], qk = M[q][k];
                M[p][k] = c * pk - s * qk;  M[q][k] = s * pk + c * qk;
            }
        }
    }
    for (int i = 0; i < 3; i++) sg[i] = sqrt(M[i][i] > 0 ? M[i][i] : 0.0);
    for (int i = 0; i < 2; i++) for (int j = i + 1; j < 3; j++)
        if (sg[j] > sg[i]) { const double t = sg[i]; sg[i] = sg[j]; sg[j] = t; }
}

static void test_runtime_consistency_guard_replay() {
    TEST(runtime_consistency_guard_replay);

    static const char* FILES[4] = {
        "calib_poses_2026-09-19.txt",
        "calib_poses_2026-09-19_1525.txt",
        "calib_poses_2026-09-19_1530.txt",
        "calib_poses_2026-09-19_1533.txt"
    };
    static const char* LABELS[4] = { "12:38", "15:25", "15:30", "15:33" };
    static const char* NM[6] = { "Fx", "Fy", "Fz", "Mx", "My", "Mz" };

    int refused = 0, passedCtrl = 0, poses = 0;
    double csLatMin = 1e9, csLatMax = -1e9;   // |c_s_横向| 的四份范围 (z 力漏洞的尺寸)

    const double tolF = Config::FORCE_GUARD_TOL_FORCE_N;
    const double tolM = Config::FORCE_GUARD_TOL_MOMENT_NM;
    // ⚠ 【掩码与逐通道容差都不在这份测试里再写一份】—— 从生产 API 读生效值。
    //   从前这里是两份字面量 (掩码 {true,true,false,true,true,true} 与一份 tol 映射), 于是
    //   生产里改了掩码而这里没改时, 这份回放会【安静地继续按旧闸门建模】: 它照样全绿,
    //   报出来的每一列却与生产对不上 —— 而这份回放的全部价值就是"报告得与生产一致"。
    //   guardReport() 填的就是生产里那份 g_guardVote / g_guardTol (掩码的唯一一份实现),
    //   所以读它不可能漂; 引用绑定 (加 &) 是为了【不复制】, 复制就又成了第二份实现。
    ForceCompensation::GuardReport maskRep;
    ForceCompensation::guardReport(maskRep);
    const bool   (&vote)[6] = maskRep.voted;
    const double (&tol)[6]  = maskRep.tol;

    std::cout << std::endl;
    std::cout << "    容差: 力 " << tolF << " N / 力矩 " << tolM
              << " N·m (由实测导出, 见 Config.h 与 runtime-guard-report.md)" << std::endl;
    std::cout << "    Fz 【不投票】: @576 的 z 响应实测秩 2 (奇异值 0.212/0.201/0.008)"
                 " —— 它的比较结果照样报出来。"
                 " (⚠ 这条秩 2 是在【旧判据通道】上实测的, 换到参考量之后【未重测】)"
              << std::endl;
    std::cout << "    力矩 Mx/My/Mz 【同样不投票】(2026-09-21 起): 换参考量后力矩仍差"
                 " Mx +0.061 / My +1.147 / Mz +1.316, 而其差距约 90% 是【姿态无关且在漂】"
                 " 的偏置 ⇒ 与参考量之间不存在\"一致\"态。它们照算照报 (理由与代价见"
                 " ForceCompensation.cpp 的 g_guardVote 段)。" << std::endl;
    std::cout << "    ⇒ 生效掩码由生产 API 读出 (guardReport().voted), 本用例【不抄它】;"
                 " 本行以下凡写\"投票通道\"处, 指的就是那份掩码选中的通道。" << std::endl;

    // ===== 容差的量级依据: 【四份夹具的前置遍历】(2026-09-21 复审 Important 1) =====
    //   eps_F = rmsForceN(本地模型自己的失拟) + dist_to_scalar·9.81(机械臂那一侧的模型类差)
    //   eps_M = rmsMomentNm + |c_s|·dist_to_scalar·9.81
    // 9.81 = 标准重力 (与 TcpCalibration 的重力约定同一个常数)。
    //
    // ⚠ 为什么单独开这一遍, 而不是留在下面那个逐份循环里: 下面那一半在【第一份夹具的第一个
    //   姿态】就按设计判失败并 return (它自 2026-09-21 起是红的, 见那里的 ⚠⚠)。留在一起的话,
    //   这段打印与"容差 > eps"那条断言就【只跑第一份】—— 而第一份恰好是四份里要求最低的那份
    //   (eps_F 0.153138 N), 于是"容差被设到最坏一份之下"这件事再也发现不了 (最坏 0.287098 N)。
    //   前置遍历让它四份全跑, 并且按【最坏的一份】断言。这一遍只读夹具、只算 eps、只打印、
    //   最后断言一次 —— 不改任何判据, 也不碰下面那条故意红着的断言。
    //
    // ⚠ dist_to_scalar = (σ1−σ3)/2 是【精确的】那一个 (2026-09-19 复审 Important 3 改的):
    //   机械臂那一侧的力模型是"标量质量 × 正交"(m·Q), 而 A 到最近的 m·Q 的算子范数距离
    //   恰好是 (σ1−σ3)/2 (在 m = (σ1+σ3)/2 处取到)。从前的写法是 max|σ−σ̄|, 那个量
    //   【不是下界而是上界】(它恒 ≥ (σ1−σ3)/2), 所以用它当依据会把"容差是合法差的几倍"
    //   说小, 而原文还称它"fail-closed 的方向"—— 方向正好说反了 (取大了容差只会更松)。
    //   现在用的既然是精确距离, 余量就只有那个倍数本身。【那次】改口径时容差的数值
    //   一个都没动 —— 但力通道的容差【后来】在 2026-09-21 上调过 (依据见 Config.h,
    //   上调的是另一个误差项: 参考量自带的力偏置及其跨轮漂移); 力矩容差仍是 0.03 N·m。
    //   两个量都在下面打出来, 好在改口径时一眼看出差了多少 (实测 0~15%)。
    double epsFMax = 0.0, epsMMax = 0.0;      // 四份里的最坏值 —— 断言按它们判
    const char* epsFMaxLabel = "";
    const char* epsMMaxLabel = "";
    for (int k = 0; k < 4; k++) {
        T6Capture cap;
        cap.label = LABELS[k];
        cap.file  = FILES[k];
        if (!t6Load(cap)) {
            std::cout << "    FAIL: 夹具 " << FILES[k] << " 读不到" << std::endl;
            g_failed++;
            return;
        }
        PayloadCalibration::RawFit fit;
        if (!PayloadCalibration::fitRawLinear(cap.poses, cap.F1304, cap.M1304, cap.n, fit)) {
            std::cout << "    FAIL: " << cap.label << " 的 @1304 线性拟合失败" << std::endl;
            g_failed++;
            return;
        }
        double csLat = 0.0;      // |c_s| 的横向分量 (供 z 力漏洞那一条用, 见下)
        double sg[3], sbar = 0.0, dev = 0.0, cs = 0.0;
        t6SigmaA(fit.A, sg);
        for (int a = 0; a < 3; a++) sbar += sg[a] / 3.0;
        for (int a = 0; a < 3; a++) if (fabs(sg[a] - sbar) > dev) dev = fabs(sg[a] - sbar);
        for (int a = 0; a < 3; a++) cs += fit.cS[a] * fit.cS[a];
        cs = sqrt(cs);
        csLat = sqrt(fit.cS[0] * fit.cS[0] + fit.cS[1] * fit.cS[1]);
        if (csLat < csLatMin) csLatMin = csLat;
        if (csLat > csLatMax) csLatMax = csLat;
        const double distScalar = 0.5 * (sg[0] - sg[2]);   // σ 已降序 (t6SigmaA 最后排过)
        const double epsClassF = distScalar * 9.81;
        const double epsF = fit.rmsForceN + epsClassF;
        const double epsM = fit.rmsMomentNm + cs * epsClassF;
        std::cout << "    ---- " << cap.label << " 容差依据 ----" << std::endl;
        std::cout << "      σ(A)=" << sg[0] << " " << sg[1] << " " << sg[2]
                      << " kg, σ̄=" << sbar
                      << "  (参考: max|σ−σ̄|=" << dev << " kg —— 这是【上界】, 只用于对照)"
                      << std::endl;
        std::cout << "      (σ1−σ3)/2=" << distScalar << " kg -> 类差 " << epsClassF
                      << " N;  rms_力=" << fit.rmsForceN << " N, rms_力矩=" << fit.rmsMomentNm
                      << " N·m" << std::endl;
        std::cout << "      eps_F=" << epsF << " N,  eps_M=" << epsM << " N·m"
                      << "   [容差/eps: 力 " << tolF / epsF << "x, 力矩 " << tolM / epsM << "x]"
                      << std::endl;
        // |c_s| 与它的横向分量 —— 【z 力漏洞的尺寸】就出在这两个数上 (复审 Important 2)。
        std::cout << "      |c_s|=" << cs << " m (轴向 " << fit.cS[2] << "), |c_s_横向|="
                      << csLat << " m  ⇒ 力矩通道能看见的 z 力误差下限 ~ tol_M/|c_s_横向| = "
                      << (csLat > 0 ? tolM / csLat : 0.0) << " N" << std::endl;
        if (epsF > epsFMax) { epsFMax = epsF; epsFMaxLabel = cap.label; }
        if (epsM > epsMMax) { epsMMax = epsM; epsMMaxLabel = cap.label; }
    }
    std::cout << "    => 四份夹具里最坏: 力 eps_F=" << epsFMax << " N (" << epsFMaxLabel
              << "), 力矩 eps_M=" << epsMMax << " N·m (" << epsMMaxLabel << ")"
              << std::endl;
    std::cout << "       容差/最坏 eps: 力 " << tolF / epsFMax << "x, 力矩 " << tolM / epsMMax
              << "x" << std::endl;
    // 容差【不许】落在实测导出的量级之下 —— 落下去就是"永远拒绝"，
    // 而这条断言是那个决定唯一能被机器检查的地方。
    // ⚠ 按【四份里最坏的一份】判 (2026-09-21): 只按第一份判的话, 第一份恰好最小, 容差被改到
    //   最坏那一份之下也发现不了。这一条是【唯一】的容差断言 —— 下面那个逐份循环里不再重复,
    //   否则重复的那一份必然是死代码 (前置遍历已经先把四份都判过了)。
    if (!(tolF > epsFMax && tolM > epsMMax)) {
        std::cout << "    FAIL: 容差低于实测导出的量级 (最坏 eps: 力 " << epsFMax << " N / 力矩 "
                  << epsMMax << " N·m)" << std::endl;
        g_failed++;
        return;
    }

    for (int k = 0; k < 4; k++) {
        T6Capture cap;
        cap.label = LABELS[k];
        cap.file  = FILES[k];
        if (!t6Load(cap)) {
            std::cout << "    FAIL: 夹具 " << FILES[k] << " 读不到" << std::endl;
            g_failed++;
            return;
        }
        PayloadCalibration::RawFit fit;
        if (!PayloadCalibration::fitRawLinear(cap.poses, cap.F1304, cap.M1304, cap.n, fit)) {
            std::cout << "    FAIL: " << cap.label << " 的 @1304 线性拟合失败" << std::endl;
            g_failed++;
            return;
        }

        // 容差的量级依据 (eps_F / eps_M 的现算、打印与"容差 > 最坏 eps"那条断言) 已挪到
        // 本用例开头的【前置遍历】—— 那里四份夹具全跑; 留在这里只会跑第一份, 见那里的 ⚠。

        double worst[6] = {0, 0, 0, 0, 0, 0};   // 逐通道 |EMA|/容差 的最大值
        double sum[6]   = {0, 0, 0, 0, 0, 0};   // 逐通道 EMA 的均值 (本份采集内)
        std::cout << "    ---- " << cap.label << " (n=" << cap.n << ") ----" << std::endl;

        for (int i = 0; i < cap.n; i++) {
            double six[6];
            for (int a = 0; a < 3; a++) {
                six[a]     = cap.F1304[i][a];
                six[3 + a] = cap.M1304[i][a];
            }

            // (甲) 真实夹具: @576 用夹具里的那一列
            //
            // ⚠⚠ 【这一半仍然红着, 但红的【原因】自 2026-09-21 Task 7 起换了一个 —— 换的是
            //   描述, 不是结论。结论不变: 【这份数据回答不了"真实夹具上判据会不会拒绝"】,
            //   而且它【不许】被算成原断言得到满足。】
            //
            //   改动【前】的现状 (2026-09-21 当天实测): 判据参考量换到另一路之后, 判据那一侧
            //   喂进去的是 0 (夹具没有那一路的列), 而本地模型的残差 (~0.02 N / ~0.001 N·m)
            //   远在容差之下 ⇒ 闸门【放行】, 于是"每一帧都必须拒绝"失败, 消息是
            //   "竟然放行了 (state=OK)"。
            //   改动【后】的现状: 判据多了一道【参考量可用性】判据 (Task 7,
            //   ForceCompensation.cpp 的 guardReferenceAvailable)。这份 ForceData 的初值
            //   (isStale=true / sixForceOnline=-1) 表达的正是【这一路没有数据】⇒ 闸门判
            //   【参考量不可用】, 拒绝, 于是"每一帧都必须拒绝"【同样不成立】——
            //   它【只是不再靠"放行"来不成立】。
            //   ⚠ 为什么两版都算不上"通过": 那条断言要的是"闸门在【比过】之后拒绝",
            //     而这里【根本没有比过】—— 无论闸门当时是"放行"还是"不可用", 这份数据都
            //     没有回答过它。把"参考量不可用"记成"原断言通过", 就是把一件未验证的事
            //     变成绿的 (用户明确否决过)。
            //   ⚠ 这份 ForceData 的默认值【不是】在声明"那台机械臂当时不在线": 夹具文件头
            //     记的是 sixForceOnline=1。这里声明的是【这份数据里没有参考量那一路的值】——
            //     它没有那一列, 所以喂进去的 0 是【缺列】, 不是读数。
            //   ⚠ 保留它【红着】是刻意的: 它是"这份数据不再支持旧结论"的唯一机器可见的记录。
            //   ⇒ 要它重新有判别力, 只有两条路 (都需要【新的采集】, 不在本计划内):
            //     ① 重采一份【带参考量那一路的列】的夹具, 用真值喂 (甲);
            //     ② 由所有者决定这一半改成测什么 (但【不许】把它删成一个绿的空壳)。
            AppState::ForceData fd;
            for (int a = 0; a < 6; a++) fd.sixForceRaw[a] = six[a];
            fd.raw[0] = cap.F576[i][0]; fd.raw[1] = cap.F576[i][1]; fd.raw[2] = cap.F576[i][2];
            fd.raw[3] = cap.M576[i][0]; fd.raw[4] = cap.M576[i][1]; fd.raw[5] = cap.M576[i][2];
            ForceCompensation::init();
            ForceCompensation::setCalibration(fit.A, fit.bF, fit.bM, fit.cS);
            // 连喂 8 帧同样的读数: 闸门逐帧都判, 这里要的是"持续"那一侧的语义
            // (EMA 由第 1 帧播种, 8 帧同值 ⇒ EMA 恒等于该姿态的均值差)。
            for (int f = 0; f < 8; f++) ForceCompensation::step(fd, cap.poses[i]);

            ForceCompensation::GuardReport rep;
            ForceCompensation::guardReport(rep);
            if (rep.state == ForceCompensation::GuardState::INCONSISTENT) refused++;
            if (i == 0) {
                // ★ 2026-09-21 (Task 7): 只有【比过】才有逐通道结果可打。参考量不可用时
                //   那六个数不是任何一次比较的结果 (被减数那一侧没有数据) —— 打出来就是把
                //   "没比过"说成"比过了"。
                if (rep.state != ForceCompensation::GuardState::INCONSISTENT) {
                    std::cout << "      pose 1 逐通道: 【没有比过】—— 闸门状态 "
                              << ForceCompensation::guardStateName(rep.state)
                              << ", 所以没有 EMA 差可打 (不是\"在限内\", 是\"没比过\")。"
                              << std::endl;
                } else {
                    char line[512];
                    int off = snprintf(line, sizeof(line), "      pose 1 逐通道 (EMA 差, 超限倍数):");
                    for (int a = 0; a < 6; a++) {
                        if (!vote[a]) {
                            // 不投票的原因【不在这里再抄一遍】(Fz 是"秩 2 依据未复测", 力矩是
                            // "与参考量之间没有一致态" —— 两件事, 混成一句就是另一份会漂的文字)。
                            off += snprintf(line + off, sizeof(line) - off, "  %s=不投票", NM[a]);
                        } else {
                            off += snprintf(line + off, sizeof(line) - off, "  %s %+.3f(%.1fx)",
                                            NM[a], rep.ema[a], fabs(rep.ema[a]) / tol[a]);
                        }
                    }
                    std::cout << line << std::endl;
                }
            }
            for (int a = 0; a < 6; a++) {
                sum[a] += rep.ema[a] / cap.n;
                if (!vote[a]) continue;
                const double r = fabs(rep.ema[a]) / tol[a];
                if (r > worst[a]) worst[a] = r;
            }
            poses++;
            // 每一帧都必须拒绝 —— 【而且必须是"比过之后拒绝"】。
            // ⚠ 2026-09-21 (Task 7): 这里【仍然算失败、仍然是 1 条】, 只是把"为什么不是
            //   INCONSISTENT"说清楚。不许把"参考量不可用"算成这条断言通过 —— 它要的是
            //   "闸门比过之后拒绝", 而没有参考量时【根本没有比过】。
            if (rep.state != ForceCompensation::GuardState::INCONSISTENT) {
                std::cout << "    FAIL: " << cap.label << " pose " << (i + 1)
                          << " 不是 INCONSISTENT (state="
                          << ForceCompensation::guardStateName(rep.state) << ")" << std::endl;
                if (rep.state == ForceCompensation::GuardState::REFERENCE_UNAVAILABLE) {
                    std::cout << "      ⇒ 【参考量不可用】: 这份夹具没有【参考量那一路】的列,"
                                 " 所以判据那一侧没有数据 (Task 7 的存在性守卫把它择出来了)。"
                              << std::endl;
                    std::cout << "      ⇒ 【原断言仍然没有被验证】: 它问的是\"两边【比过】之后"
                                 " 会不会拒绝\", 而这里两边【没有比过】。这条【照旧算失败】——"
                                 " 把它记成通过, 就是把一件未验证的事变绿。" << std::endl;
                    std::cout << "      ⇒ 要它重新有判别力只能靠【重采一份带参考量那一路的列"
                                 "的夹具】(它是本计划的收口必做项), 不是靠改这里。" << std::endl;
                }
                g_failed++;
                return;
            }

            // (乙) 反面对照: 把【参考量那一侧】换成与本地模型一致的值 -> 必须放行。
            // (夹具没有参考量那一路的列 —— 所以用 t6LocalModel 现算, 理由同上面那条 ⚠。)
            // ⚠⚠ 【这条能证明什么、不能证明什么 —— 如实说 (复审 Important 4/§0)】:
            //   下面 mdl 是用 t6LocalModel 算的, 而 t6LocalModel 是 ForceCompensation::step()
            //   那条公式的【逐字副本】。所以"参考量 == 本地模型输出"这件事是【代数上恒真】的:
            //   d ≡ 0, 与容差是多少【完全无关】。因此这半边
            //     · 证明了: "放行"这条路是通的 (不是坏掉的闸门, 也不是一个恒 return false
            //       的桩就能让整个用例全绿);
            //     · 证明不了: 它【分辨不了】力通道容差的具体数值 —— 把容差乘 10 或除以 10,
            //       这里都是 36/36 放行。换句话说这是【分支存在性】检查, 不是【判别力】检查。
            //   容差本身的量级由上面那段"容差 > 实测导出的 eps"来守; 而"容差会不会太松"
            //   这一问【本轮没有机器可检查的答案】(见报告 §7)。不要把这一行读成
            //   "容差被验证过了"。
            double mdl[6];
            for (int a = 0; a < 6; a++) mdl[a] = six[a];
            t6LocalModel(fit, cap.poses[i], mdl, mdl);
            AppState::ForceData fd2;
            for (int a = 0; a < 6; a++) { fd2.sixForceRaw[a] = six[a]; fd2.tcpForce[a] = mdl[a]; }
            // ★ 2026-09-21 (Task 7): 这一半喂的是一个【真的读到了值】的参考量 (由本地模型
            //   现算), 所以必须同时声明"这一帧是新鲜的、机械臂自报在线" —— 否则闸门判的是
            //   【参考量不可用】, 这一半就不再是"放行那条路通不通"的对照 (它的全部意义就是
            //   当那个对照)。生产里这几件事由 RelayCore 在同一帧里一起做好。
            fd2.isStale = false;
            fd2.sixForceOnline = 1;
            ForceCompensation::init();
            ForceCompensation::setCalibration(fit.A, fit.bF, fit.bM, fit.cS);
            for (int f = 0; f < 8; f++) ForceCompensation::step(fd2, cap.poses[i]);
            if (ForceCompensation::guardState() == ForceCompensation::GuardState::OK) passedCtrl++;
        }
        std::cout << "      本份的逐通道 EMA 均值 (N / N·m):";
        for (int a = 0; a < 6; a++) std::cout << "  " << NM[a] << " " << sum[a];
        std::cout << std::endl;
        std::cout << "      最大超限倍数 (逐通道, 只算投票通道):";
        for (int a = 0; a < 6; a++) {
            if (!vote[a]) continue;
            std::cout << "  " << NM[a] << " " << worst[a] << "x";
        }
        std::cout << std::endl;
    }

    std::cout << "    => 真实夹具: " << refused << " / " << poses << " 个姿态【拒绝】"
              << "    反面对照 (参考量与本地一致): " << passedCtrl << " / " << poses << " 个姿态放行"
              << std::endl;
    // ⚠ 【z 力漏洞的尺寸】(复审 Important 2): Fz 不投票, 而【力矩通道原本是它唯一可能的
    //   替补】—— 那个替补的门槛 = tol_M / |c_s_横向|。四份实测的 |c_s_横向| 范围决定
    //   "力矩门若在"这个洞有多大; 下面这行把它印出来, 免得"少一道闸门"这种话盖住一个几十
    //   牛的孔。
    //   ⚠ 2026-09-21: 力矩三个分量【已经不再投票】(见上面那段), 所以下面这个门槛现在算的是
    //     【一道不存在的门】—— 数字一个没变 (它量的是"要多大 z 差才顶得动 tol_M"), 变的是
    //     现在【连这道门都没有】。这个洞因此比原来更大一点, 不是更小。
    std::cout << "    => z 力方向【没有闸门】: |c_s_横向| 四份范围 [" << csLatMin << ", " << csLatMax
              << "] m ⇒ 力矩通道【若投票】要看见 z 力模型误差, 它得大到 "
              << tolM / csLatMax << " ~ " << tolM / csLatMin << " N (即几十牛);"
              << " 而力矩【也不投票】⇒ 本闸门对这个方向没有判据。" << std::endl;
    std::cout << "       Fz 不投票这一点本身由复审判定可接受, 但这个洞的大小必须写明"
                 " —— 见报告里的未决项与 Config.h 的开放项 C。" << std::endl;

    // 四份采集、【每一个】姿态都必须拒绝。若某一份里有一个姿态放行, 说明闸门在那个姿态上
    // 看不见差异 —— 那是"闸门有洞", 必须先查清楚再放行, 不能把断言放宽。
    CHECK(refused == poses);
    // 反面对照也必须【全部】放行。这一条防的是"闸门永远拒绝"那半边: 只断言拒绝的话,
    // 一个 return false 的桩也能全绿。
    CHECK(passedCtrl == poses);
    PASS();
}

// =====================================================================================
// ★ Task 8a: 下发前的两道闸 (上机操作单 §6) —— 纯逻辑, 无 socket, 所以在这里测。
//
// 用例的输入数字取自实机那一对真值 (上机操作单 §6 闸1):
//   |c_s_z| = 55.556 mm (2026-09-19 四次采集 c_s 模长 54.55~56.06 之一)
//   cz_robot = 68.700 mm (@1176 CenterZ 上次读到的值)
// 约定一 = c_s_z 与工具轴同向, 约定二 = 反向; 两支差 2·c_s_z。
// =====================================================================================
// 本节的数都是【定义出来的小量】, 不是解出来的 —— 比的就是那几步算术, 所以容差取机器精度级。
static bool nearRefAbs(double got, double ref, double tol = 1e-9) {
    return fabs(got - ref) < tol;
}

static const double SG_CS_Z   = 55.556;    // 本次解出的 |c_s_z| (mm), 沿工具轴
static const double SG_CZ_ROB = 68.700;    // 机械臂自报 CenterZ (mm)
// 候选负载: 用机械臂自报的那三个数当量级合理的候选 (CenterX/Y/Z = 0.3, -0.1, 68.7)
static const double SG_COM[3] = {0.3, -0.1, 68.7};

static void test_send_gate_convention_one_wins_and_signs_cz() {
    TEST(send_gate_convention_one_wins_and_signs_cz);
    // c_s_z = +55.556 -> d同向 = 68.700 − 55.556 =  13.144 (在范围内)
    //                     d反向 = 68.700 + 55.556 = 124.256 (在范围外) -> 约定一胜出
    const double csZ = +SG_CS_Z;
    PayloadCalibration::SendGate g =
        PayloadCalibration::evaluateSendGate(0.42, SG_COM, &csZ, &SG_CZ_ROB, PayloadCalibration::MASS_SOURCE_MEASURED);
    CHECK(g.verdict == PayloadCalibration::SEND_OK);
    CHECK(g.convention == 1);
    // 选中的 cz 符号 = 选中的那支 c_s_z 的符号 = +1
    CHECK(g.czSign == +1.0);
    CHECK(g.dSameIn && !g.dFlipIn);
    // 候选的 cz 按选中的符号定号 —— 输入故意给负的 cz, 输出必须是正的 |cz|
    const double comNeg[3] = {0.3, -0.1, -68.7};
    PayloadCalibration::SendGate g2 =
        PayloadCalibration::evaluateSendGate(0.42, comNeg, &csZ, &SG_CZ_ROB, PayloadCalibration::MASS_SOURCE_MEASURED);
    CHECK(g2.verdict == PayloadCalibration::SEND_OK);
    CHECK(nearRefAbs(g2.comMm[2], +68.7));
    CHECK(nearRefAbs(g2.comMm[0], +0.3));   // 横向分量【不动】—— 只有 cz 定号
    CHECK(nearRefAbs(g2.comMm[1], -0.1));
    PASS();
}

static void test_send_gate_convention_two_wins_and_signs_cz() {
    TEST(send_gate_convention_two_wins_and_signs_cz);
    // 解出的 c_s_z 若是【负的】, 两支的大小关系跟着翻 (约定二 = 约定一 + 2·c_s_z):
    //   d同向 = 68.700 − (−55.556) = 124.256 (在范围外)
    //   d反向 = 68.700 + (−55.556) =  13.144 (在范围内)  -> 约定二胜出
    // 而两支的【物理含义】不变: 选中的那支 c_s_z 仍是 +55.556, 所以 cz 仍是正的。
    const double csZ = -SG_CS_Z;
    PayloadCalibration::SendGate g =
        PayloadCalibration::evaluateSendGate(0.42, SG_COM, &csZ, &SG_CZ_ROB, PayloadCalibration::MASS_SOURCE_MEASURED);
    CHECK(g.verdict == PayloadCalibration::SEND_OK);
    CHECK(g.convention == 2);
    CHECK(g.czSign == +1.0);
    CHECK(!g.dSameIn && g.dFlipIn);
    CHECK(nearRefAbs(g.comMm[2], +68.7));
    PASS();
}

static void test_send_gate_two_conventions_in_range_is_ambiguous() {
    TEST(send_gate_two_conventions_in_range_is_ambiguous);
    // 两支都落在 (0, 31.5): cz_robot = 15.75, c_s_z = 1.0
    //   d同向 = 14.75 (在内) / d反向 = 16.75 (在内) —— 数据定不了符号, 不许二选一猜。
    const double csZ = 1.0;
    const double czRob = 15.75;
    PayloadCalibration::SendGate g =
        PayloadCalibration::evaluateSendGate(0.42, SG_COM, &csZ, &czRob, PayloadCalibration::MASS_SOURCE_MEASURED);
    CHECK(g.verdict == PayloadCalibration::SEND_SIGN_AMBIGUOUS);
    CHECK(g.dSameIn && g.dFlipIn);
    CHECK(g.convention == 0);   // 没定下约定 -> 也就没定下 cz 的号
    CHECK(g.czSign == 0.0);
    PASS();
}

static void test_send_gate_no_convention_in_range_is_refused() {
    TEST(send_gate_no_convention_in_range_is_refused);
    // 两支都在范围外: cz_robot = 200 mm -> d同向 = 144.444 / d反向 = 255.556
    const double csZ = +SG_CS_Z;
    const double czRob = 200.0;
    PayloadCalibration::SendGate g =
        PayloadCalibration::evaluateSendGate(0.42, SG_COM, &csZ, &czRob, PayloadCalibration::MASS_SOURCE_MEASURED);
    CHECK(g.verdict == PayloadCalibration::SEND_SIGN_NONE_IN_RANGE);
    CHECK(!g.dSameIn && !g.dFlipIn);
    CHECK(g.convention == 0);
    CHECK(g.czSign == 0.0);
    PASS();
}

static void test_send_gate_interval_is_open_at_both_ends() {
    TEST(send_gate_interval_is_open_at_both_ends);
    const double csZ = +SG_CS_Z;
    // d 恰好 = 0: cz_robot = c_s_z = 55.556 -> d同向 = 0 (不在开区间内), d反向 = 111.112 (也在外)
    const double czAtZero = 55.556;
    PayloadCalibration::SendGate g0 =
        PayloadCalibration::evaluateSendGate(0.42, SG_COM, &csZ, &czAtZero, PayloadCalibration::MASS_SOURCE_MEASURED);
    CHECK(g0.verdict == PayloadCalibration::SEND_SIGN_NONE_IN_RANGE);
    CHECK(nearRefAbs(g0.dSameDir, 0.0));
    // d 恰好 = 31.5: cz_robot = 55.556 + 31.5 = 87.056 -> d同向 = 31.5 (不在开区间内)
    const double czAtEdge = 87.056;
    PayloadCalibration::SendGate g1 =
        PayloadCalibration::evaluateSendGate(0.42, SG_COM, &csZ, &czAtEdge, PayloadCalibration::MASS_SOURCE_MEASURED);
    CHECK(g1.verdict == PayloadCalibration::SEND_SIGN_NONE_IN_RANGE);
    CHECK(nearRefAbs(g1.dSameDir, 31.5));
    PASS();
}

static void test_send_gate_mass_bounds_are_inclusive() {
    TEST(send_gate_mass_bounds_are_inclusive);
    const double csZ = +SG_CS_Z;
    const double m[2] = {0.2, 1.5};       // 闸2 的量级判据: m ∈ [0.2, 1.5] kg, 两端【含】
    for (int i = 0; i < 2; i++) {
        PayloadCalibration::SendGate g =
            PayloadCalibration::evaluateSendGate(m[i], SG_COM, &csZ, &SG_CZ_ROB, PayloadCalibration::MASS_SOURCE_MEASURED);
        CHECK(g.verdict == PayloadCalibration::SEND_OK);
        CHECK(g.massOk);
    }
    PASS();
}

static void test_send_gate_mass_outside_bounds_is_refused() {
    TEST(send_gate_mass_outside_bounds_is_refused);
    const double csZ = +SG_CS_Z;
    const double m[2] = {0.19, 1.51};
    for (int i = 0; i < 2; i++) {
        PayloadCalibration::SendGate g =
            PayloadCalibration::evaluateSendGate(m[i], SG_COM, &csZ, &SG_CZ_ROB, PayloadCalibration::MASS_SOURCE_MEASURED);
        CHECK(g.verdict == PayloadCalibration::SEND_MASS_OUT_OF_RANGE);
        CHECK(!g.massOk);
    }
    PASS();
}

static void test_send_gate_com_magnitude_bound_is_exclusive_at_500() {
    TEST(send_gate_com_magnitude_bound_is_exclusive_at_500);
    // 【口径以规格为准 (用户 2026-09-20 裁定)】: on-machine-checklist.md §6 闸2 的原文是
    // "|c| < 500 mm" —— 【不含】500。从前这条用例断言的是"含"(照 8a 简报的用例表),
    // 裁定后改成严格小于: 500 必须拒, 而 499.999 必须放行 (否则就是整条闸被关掉了)。
    const double csZ = +SG_CS_Z;
    const double at500[3] = {500.0, 0.0, 0.0};
    PayloadCalibration::SendGate gAt500 =
        PayloadCalibration::evaluateSendGate(0.42, at500, &csZ, &SG_CZ_ROB,
                                            PayloadCalibration::MASS_SOURCE_MEASURED);
    CHECK(gAt500.verdict == PayloadCalibration::SEND_COM_OUT_OF_RANGE);   // |c| = 500 不含
    CHECK(!gAt500.comOk);
    const double justUnder[3] = {499.999, 0.0, 0.0};
    PayloadCalibration::SendGate gJustUnder =
        PayloadCalibration::evaluateSendGate(0.42, justUnder, &csZ, &SG_CZ_ROB,
                                            PayloadCalibration::MASS_SOURCE_MEASURED);
    CHECK(gJustUnder.verdict == PayloadCalibration::SEND_OK);             // 499.999 放行
    CHECK(gJustUnder.comOk);
    const double at501[3] = {501.0, 0.0, 0.0};
    PayloadCalibration::SendGate gBad =
        PayloadCalibration::evaluateSendGate(0.42, at501, &csZ, &SG_CZ_ROB,
                                            PayloadCalibration::MASS_SOURCE_MEASURED);
    CHECK(gBad.verdict == PayloadCalibration::SEND_COM_OUT_OF_RANGE);
    CHECK(!gBad.comOk);
    PASS();
}

static void test_send_gate_refuses_without_cs_and_says_so() {
    TEST(send_gate_refuses_without_cs_and_says_so);
    // c_s 不可用 (本次线性层就没解出来) -> 拒因必须是【没有 c_s】, 不是【符号不对】:
    // 前者说"这次没数据", 后者说"数据在但定不了号", 处置完全不同。
    const double czRob = SG_CZ_ROB;
    PayloadCalibration::SendGate g =
        PayloadCalibration::evaluateSendGate(0.42, SG_COM, nullptr, &czRob,
                                            PayloadCalibration::MASS_SOURCE_MEASURED);
    CHECK(g.verdict == PayloadCalibration::SEND_NO_CS);
    CHECK(g.verdict != PayloadCalibration::SEND_SIGN_NONE_IN_RANGE);
    CHECK(g.verdict != PayloadCalibration::SEND_SIGN_AMBIGUOUS);
    PASS();
}

static void test_send_gate_refuses_without_cz_robot_and_says_so() {
    TEST(send_gate_refuses_without_cz_robot_and_says_so);
    // cz_robot (@1176) 不可用 -> 【不许退回自己下发的值】当参照 (那条路带着 centerZ 折叠歧义),
    // 只能照实报"不可用"并不放行。拒因必须是【没有 cz_robot】, 不是别的。
    const double csZ = +SG_CS_Z;
    PayloadCalibration::SendGate g =
        PayloadCalibration::evaluateSendGate(0.42, SG_COM, &csZ, nullptr,
                                            PayloadCalibration::MASS_SOURCE_MEASURED);
    CHECK(g.verdict == PayloadCalibration::SEND_NO_CZ_ROBOT);
    CHECK(g.verdict != PayloadCalibration::SEND_SIGN_NONE_IN_RANGE);
    CHECK(g.convention == 0);
    PASS();
}

// =====================================================================================
// ★ Task 8a-2: 候选构造 + "未实测 ⇒ 拒发" + 换帧措辞 + 与机械臂当前值的逐分量比对。
//
// 为什么这三件事也要在这里钉住: 它们是【纯逻辑】(没有 socket、不读全局状态), 而 8a 交付时
// 它们一个都不存在 —— 'p' 发的是 effective() (= 上次落盘值/种子), 发不出本次标定的结果。
// 本节的用例把"候选只能来自本次实测"这条规矩变成可执行的断言, 而不是注释里的承诺。
// =====================================================================================

// 【未实测 ⇒ 拒发】—— 判据必须【独立于】量级闸: "数看着合理但来路不对"与"数不合理"是
// 两件事, 处置也不同 (前者去查标定为什么没跑, 后者去查装夹)。
static void test_send_gate_refuses_unmeasured_mass_and_says_so() {
    TEST(send_gate_refuses_unmeasured_mass_and_says_so);
    const double csZ = +SG_CS_Z;
    // 0.66 kg 正是 Config 的种子值 (CAD 猜的, 本项目已判定不可用)。它【量级上过闸2】——
    // 下面那一条断言就是钉住这一点: 拒的理由只能是【来源】, 不许是量级。
    PayloadCalibration::SendGate g =
        PayloadCalibration::evaluateSendGate(0.66, SG_COM, &csZ, &SG_CZ_ROB,
                                            PayloadCalibration::MASS_SOURCE_SEED);
    CHECK(g.verdict == PayloadCalibration::SEND_NOT_MEASURED);
    CHECK(g.verdict != PayloadCalibration::SEND_MASS_OUT_OF_RANGE);   // 不许并进量级闸
    CHECK(g.massOk);                                                  // 量级确实是过的
    CHECK(g.verdict != PayloadCalibration::SEND_OK);                  // 更不许放行
    // ★ 这一支返回时闸1 的两个 d 与 convention 还是结构体的初值 0 —— 0 在这里是【没有算过】,
    //   不是"d = 0"、"没选中约定"。打印端 (main.cpp 的候选块) 正是靠这条约定决定"要不要把
    //   这两个数打出去": 拿 0 当 d 打印, 屏幕上就会出现一个从"没有数据"算出来的、看着像真数
    //   的东西。这条约定从前只写在头文件里, 没有用例 (二次复审 Minor 6) ⇒ 在这里钉住它。
    CHECK(g.dSameDir == 0.0 && g.dFlipDir == 0.0);
    CHECK(!g.dSameIn && !g.dFlipIn);
    CHECK(g.convention == 0 && g.czSign == 0.0);
    PASS();
}

// 【约定二 + cz_robot 为负】—— 这条路径以前【没有用例走过】(两支都在内的模糊用例与
// 都在外的用例都停在 convention = 0, 从不选约定二且 czSign = −1)。
//   c_s_z = +55.556 (正), cz_robot = −40 (负):
//     d同向 = −40 − 55.556 = −95.556 (在范围外)
//     d反向 = −40 + 55.556 = +15.556 (在范围内)  -> 约定二胜出
//   选中的那支 c_s_z = −55.556 -> czSign = −1 -> 候选的 cz 【是负的】。
static void test_send_gate_convention_two_wins_with_negative_cz_robot() {
    TEST(send_gate_convention_two_wins_with_negative_cz_robot);
    const double csZ = +SG_CS_Z;
    const double czRobNeg = -40.0;
    const double comNegZ[3] = {0.3, -0.1, -40.0};    // 与 cz_robot 一致 (生产路径就是这么传的)
    PayloadCalibration::SendGate g =
        PayloadCalibration::evaluateSendGate(0.42, comNegZ, &csZ, &czRobNeg,
                                            PayloadCalibration::MASS_SOURCE_MEASURED);
    CHECK(g.verdict == PayloadCalibration::SEND_OK);
    CHECK(g.convention == 2);
    CHECK(g.czSign == -1.0);
    CHECK(!g.dSameIn && g.dFlipIn);
    CHECK(g.comMm[2] < 0.0);                          // cz 是负的
    CHECK(nearRefAbs(g.comMm[2], -40.0));
    CHECK(nearRefAbs(g.comMm[0], +0.3));              // 横向分量仍【不动】
    CHECK(nearRefAbs(g.comMm[1], -0.1));
    PASS();
}

// ★ 【全项目唯一会改 c 的那条路】: |cz_robot| < 31.5 时, cz 的号会被闸1翻过来。
//   推导: 胜出的那支 d 满足 selected_c_s_z = cz_robot − d (d ∈ (0, 31.5)), 而候选的
//   cz = sign(选中的 c_s_z)·|cz_robot|。所以 |cz_robot| > 31.5 时号【必然不变】;
//   要翻号, 必须有 cz_robot ∈ (0, 31.5) 且胜出的 d > cz_robot。
//   例: cz_robot = +10, c_s_z = −10 -> d同向 = 20 (在内) / d反向 = 0 (开区间, 不在内)
//       -> 约定一胜出, 选中的 c_s_z = −10 -> czSign = −1 -> 候选 cz 由 +10 翻成 −10。
static void test_send_gate_small_positive_cz_robot_flips_the_sign() {
    TEST(send_gate_small_positive_cz_robot_flips_the_sign);
    const double csZ = -10.0;
    const double czRob = 10.0;
    const double comPosZ[3] = {0.3, -0.1, 10.0};
    PayloadCalibration::SendGate g =
        PayloadCalibration::evaluateSendGate(0.42, comPosZ, &csZ, &czRob,
                                            PayloadCalibration::MASS_SOURCE_MEASURED);
    CHECK(g.verdict == PayloadCalibration::SEND_OK);
    CHECK(g.convention == 1);
    CHECK(g.czSign == -1.0);
    CHECK(g.dSameIn && !g.dFlipIn);
    CHECK(nearRefAbs(g.dSameDir, 20.0));
    CHECK(nearRefAbs(g.comMm[2], -10.0));    // ★ 翻号: 机械臂当前是 +10, 发出的是 −10
    CHECK(g.comMm[2] * czRob < 0.0);         // 与当前值异号 = 号被翻了
    PASS();
}

// ---- 候选构造 (buildSendCandidate): "有没有候选"与"候选能不能发"是两层 ----

static void test_build_send_candidate_normal_path() {
    TEST(build_send_candidate_normal_path);
    // 实机真值: 质量尺度取 2026-09-19 四次采集的 |c_s| 之一 (55.556 mm),
    // 机械臂自报 (CenterX/Y/Z) = (0.3, −0.1, 68.700) -> d = 13.144 ∈ (0, 31.5) ✓
    const double measuredM = 0.420847;   // Decomp::m (实机解出的质量尺度)
    const double csZ = +55.556;
    const double echoCenter[3] = {0.3, -0.1, 68.700};
    PayloadCalibration::SendCandidate c = PayloadCalibration::buildSendCandidate(
        &measuredM, &csZ, echoCenter);
    CHECK(c.present);
    CHECK(c.absent == PayloadCalibration::CAND_PRESENT);
    CHECK(nearRefAbs(c.massKg, measuredM));              // m 就是【本次实测的】那一个
    CHECK(c.gate.verdict == PayloadCalibration::SEND_OK);
    CHECK(c.gate.convention == 1);
    // c 三个分量都取自机械臂自报, 且 cz 按闸1 定的号 (这里 = 不翻)
    CHECK(nearRefAbs(c.comMm[0], +0.3));
    CHECK(nearRefAbs(c.comMm[1], -0.1));
    CHECK(nearRefAbs(c.comMm[2], +68.700));
    PASS();
}

static void test_build_send_candidate_without_measured_mass_has_none() {
    TEST(build_send_candidate_without_measured_mass_has_none);
    // 本次没解出质量尺度 (Decomp::m 不可用) -> 【没有候选】, 不是"候选 = 0"、更不是回退种子。
    // ★ 这一条就是 8a 那个洞的封口: 从前这里会退回 effective() (= 种子值)。
    const double csZ = +55.556;
    const double echoCenter[3] = {0.3, -0.1, 68.700};
    PayloadCalibration::SendCandidate c =
        PayloadCalibration::buildSendCandidate(nullptr, &csZ, echoCenter);
    CHECK(!c.present);
    CHECK(c.absent == PayloadCalibration::CAND_NO_MEASURED_MASS);
    CHECK(c.absent != PayloadCalibration::CAND_NO_PAYLOAD_ECHO);   // 归因不许糊
    PASS();
}

static void test_build_send_candidate_without_echo_has_none() {
    TEST(build_send_candidate_without_echo_has_none);
    // 没回读到机械臂自报的 @1168/@1176 -> 【没有候选】。
    // 【不许】退回本客户端自己下发的值当参照: 那条路带着 centerZ 折叠歧义。
    const double measuredM = 0.420847;
    const double csZ = +55.556;
    PayloadCalibration::SendCandidate c =
        PayloadCalibration::buildSendCandidate(&measuredM, &csZ, nullptr);
    CHECK(!c.present);
    CHECK(c.absent == PayloadCalibration::CAND_NO_PAYLOAD_ECHO);
    CHECK(c.absent != PayloadCalibration::CAND_NO_MEASURED_MASS);
    PASS();
}

// ---- 措辞 (§2): m 的标签【必须】说明它是"测量原点以下"的量, 且【紧接着】写明换帧差 ----
static void test_send_candidate_mass_label_matches_brief_wording() {
    TEST(send_candidate_mass_label_matches_brief_wording);
    char buf[512];
    PayloadCalibration::formatSendCandidateMassText(0.4208, buf, sizeof(buf));
    const std::string t(buf);
    // ① 标签逐字: "本次实测的质量尺度" + "传感器测量原点以下"
    CHECK(t.find("本次实测的质量尺度") != std::string::npos);
    CHECK(t.find("传感器测量原点以下") != std::string::npos);
    // ② 【紧接着】要写明它与 EnableRobot 要的整条链【差了传感器机器人侧那一段, 量未定】
    CHECK(t.find("整条链") != std::string::npos);
    CHECK(t.find("量未定") != std::string::npos);
    CHECK(t.find("0.4208") != std::string::npos);        // 数是填进去的, 不是写死的
    // ③ 禁止措辞 (§2 明文): 它们会把这个量读成【已经换好帧】的
    CHECK(t.find("标定结果") == std::string::npos);
    CHECK(t.find("绝对负载") == std::string::npos);
    CHECK(t.find("整条链质量") == std::string::npos);
    PASS();
}

// ---- 候选块里 c 那一行的方括号标签 (M2): 措辞只此一份, 两种情形各说各的 ----
// 它是那一屏里唯一没有任何测试钉住的操作员可见字符串 (其余措辞都有)。判据是闸1【真的】
// 定下了号没有: SEND_SIGN_AMBIGUOUS / SEND_SIGN_NONE_IN_RANGE 在赋值之前就返回, 那时候选的
// cz 就是机械臂自报的原样 —— 标签若说"号已定", 屏幕上就同时出现"闸1 无法判定"与"号已定"
// 两句互相打架的话。
static void test_send_candidate_center_label_follows_the_verdict() {
    TEST(send_candidate_center_label_follows_the_verdict);
    // ① 号被定下来了 (实机那组: cz_robot = 68.7, c_s_z = +55.556 -> 约定一, 不翻号)
    {
        const double measuredM = 0.420847;
        const double csZ = +55.556;
        const double echoCenter[3] = {0.3, -0.1, 68.700};
        PayloadCalibration::SendCandidate c = PayloadCalibration::buildSendCandidate(
            &measuredM, &csZ, echoCenter);
        CHECK(c.gate.convention == 1);
        char buf[64];
        PayloadCalibration::formatSendCandidateCenterLabel(c.gate, buf, sizeof(buf));
        const std::string t(buf);
        CHECK(t == "cz 已按闸1 定的号");                     // ★ 逐字钉住
        CHECK(t.find("未定号") == std::string::npos);
    }
    // ② 号【没】定下来 (两种约定都落在 (0, 31.5) 内 = SEND_SIGN_AMBIGUOUS)
    {
        const double measuredM = 0.420847;
        const double csZ = 1.0;
        const double echoCenter[3] = {0.3, -0.1, 15.75};
        PayloadCalibration::SendCandidate c = PayloadCalibration::buildSendCandidate(
            &measuredM, &csZ, echoCenter);
        CHECK(c.gate.convention == 0);
        char buf[64];
        PayloadCalibration::formatSendCandidateCenterLabel(c.gate, buf, sizeof(buf));
        const std::string t(buf);
        CHECK(t == "闸1 未定号, cz 即自报原样");             // ★ 逐字钉住
        CHECK(t.find("已按闸1 定的号") == std::string::npos);   // 不许出现"号已定"
    }
    PASS();
}

// ---- 与机械臂【当前值】的逐分量比对 (§3.5): 这一次到底改了什么 ----
// 用 buildSendCandidate 造候选 (这样 diff 拿到的是【真会发出去的那一份】, 含 cz 的号)。
static void test_send_candidate_diff_reports_unchanged_c_on_the_real_machine_case() {
    TEST(send_candidate_diff_reports_unchanged_c_on_the_real_machine_case);
    // 实机配置: 机械臂当前 @1168 Load = 0.404 kg, @1176 = (0.3, −0.1, 68.7), cz_robot = 68.7。
    // |cz_robot| = 68.7 > 31.5 ⇒ 号翻不了 ⇒ 发出值与当前值【逐位相同】, 只改 m。
    const double measuredM = 0.420847;
    const double csZ = +55.556;
    const double echoCenter[3] = {0.3, -0.1, 68.700};
    PayloadCalibration::SendCandidate c =
        PayloadCalibration::buildSendCandidate(&measuredM, &csZ, echoCenter);
    CHECK(c.present);
    PayloadCalibration::SendCandidateDiff df =
        PayloadCalibration::diffSendCandidate(c, 0.404, echoCenter);
    CHECK(df.cUnchanged);                       // c 未变
    CHECK(!df.czSignFlipped);                   // 没翻号 -> 不是高危
    CHECK(df.sendable);                         // 过闸了 -> "只改 m"这句是【能发出去】的候选才配说的

    CHECK(nearRefAbs(df.dc[0], 0.0) && nearRefAbs(df.dc[1], 0.0) && nearRefAbs(df.dc[2], 0.0));
    CHECK(nearRefAbs(df.dm, measuredM - 0.404));  // 只改 m
    char buf[512];
    PayloadCalibration::formatSendCandidateDiffConclusion(df, buf, sizeof(buf));
    const std::string t(buf);
    CHECK(t.find("c 未变") != std::string::npos);
    CHECK(t.find("高危") == std::string::npos);   // 没翻号就不许喊高危
    PASS();
}

static void test_send_candidate_diff_flags_flipped_cz_as_high_risk() {
    TEST(send_candidate_diff_flags_flipped_cz_as_high_risk);
    // 翻号那一支 (见上面 test_send_gate_small_positive_cz_robot_flips_the_sign 的构造):
    // 机械臂当前 CenterZ = +10, 闸1 把发出去的 cz 翻成 −10。量级上这是一次【巨大】的改动,
    // 不是"只改 m", 所以结论行必须标【高危】。
    const double measuredM = 0.420847;
    const double csZ = -10.0;
    const double echoCenter[3] = {0.3, -0.1, 10.0};
    PayloadCalibration::SendCandidate c =
        PayloadCalibration::buildSendCandidate(&measuredM, &csZ, echoCenter);
    CHECK(c.present);
    CHECK(nearRefAbs(c.comMm[2], -10.0));       // 发出的是 −10 (当前是 +10)
    PayloadCalibration::SendCandidateDiff df =
        PayloadCalibration::diffSendCandidate(c, 0.404, echoCenter);
    CHECK(df.czSignFlipped);                    // ★ 号被翻
    CHECK(!df.cUnchanged);                      // c 已变
    CHECK(nearRefAbs(df.dc[2], -20.0));         // 逐分量差 = −10 − (+10)
    char buf[512];
    PayloadCalibration::formatSendCandidateDiffConclusion(df, buf, sizeof(buf));
    const std::string t(buf);
    CHECK(t.find("高危") != std::string::npos);
    CHECK(t.find("c 已变") != std::string::npos);
    // ★ 翻号时【不许】再打"只改 m"这类措辞 (§3.5 第 2 条)
    CHECK(t.find("只改 m") == std::string::npos);
    // ★ M6: 这一支【发得出去】⇒ "再决定发不发"是真存在的选项, 该说。
    CHECK(df.sendable);
    CHECK(t.find("决定发不发") != std::string::npos);
    PASS();
}

// ★ 二次复审 Minor 1: 【发不出去的候选没有"改动"可言】。
//   闸1 定不了号 (SEND_SIGN_AMBIGUOUS) 时 convention / czSign / comMm[2] 都还没赋值, 候选的 cz
//   就是机械臂自报的原样 ⇒ 逐位比较必然"c 未变"。老措辞于是打出"c 未变 —— 本次只改 m",
//   而操作员读到的是"按 'p' 会做一次最小改动" —— 事实是按 'p' 会被拒、一个字节都发不出去。
//   这一条把"发不出去的候选不许打 只改 m"钉住 (判据与 'p' 那一支同一个: verdict == SEND_OK)。
//   构造与 test_send_gate_two_conventions_in_range_is_ambiguous 同一组: cz_robot = 15.75,
//   c_s_z = 1.0 -> d同向 = 14.75 / d反向 = 16.75, 两支都在 (0, 31.5) 内。
static void test_send_candidate_diff_conclusion_never_says_only_m_when_unsendable() {
    TEST(send_candidate_diff_conclusion_never_says_only_m_when_unsendable);
    const double measuredM = 0.420847;
    const double csZ = 1.0;
    const double echoCenter[3] = {0.3, -0.1, 15.75};
    PayloadCalibration::SendCandidate c =
        PayloadCalibration::buildSendCandidate(&measuredM, &csZ, echoCenter);
    CHECK(c.present);                                            // 候选【有】...
    CHECK(c.gate.verdict == PayloadCalibration::SEND_SIGN_AMBIGUOUS);   // ...但发不出去
    CHECK(c.gate.convention == 0);                               // 号没定下来
    CHECK(nearRefAbs(c.comMm[2], 15.75));                        // ⇒ cz 就是自报原样 (没定号)
    PayloadCalibration::SendCandidateDiff df =
        PayloadCalibration::diffSendCandidate(c, 0.404, echoCenter);
    CHECK(!df.sendable);                                         // 判据与 'p' 那一支同一个
    CHECK(df.cUnchanged);            // 数值上确实没变 —— 但这【不】等于"本次只改 m"
    char buf[512];
    PayloadCalibration::formatSendCandidateDiffConclusion(df, buf, sizeof(buf));
    const std::string t(buf);
    CHECK(t.find("候选不可发送") != std::string::npos);           // ★ 照实说发不出去 (与闸的结论行同词)
    CHECK(t.find("只改 m") == std::string::npos);                 // ★ 不许说成一次最小改动
    CHECK(t.find("高危") == std::string::npos);                   // 也没翻号, 不喊高危

    // ★ 另一半: 【翻号 + 发不出去】时"高危"照样要喊 (§3.5 第 2 条是硬要求, 不许被"发不出去"顶掉)。
    //   构造同 test_send_gate_small_positive_cz_robot_flips_the_sign (cz_robot = +10, c_s_z = −10
    //   -> 约定一胜出 -> cz 由 +10 翻成 −10), 但 m = 2.0 超出闸2 的 [0.2, 1.5] ⇒ 发不出去。
    //   ⇒ 两句话都该出现: 闸的结论行说"不可发送", 结论行说"高危"。
    const double heavyM = 2.0;
    const double csZNeg = -10.0;
    const double czRobPos[3] = {0.3, -0.1, 10.0};
    PayloadCalibration::SendCandidate cf =
        PayloadCalibration::buildSendCandidate(&heavyM, &csZNeg, czRobPos);
    CHECK(cf.present);
    CHECK(cf.gate.verdict == PayloadCalibration::SEND_MASS_OUT_OF_RANGE);   // 发不出去 ...
    CHECK(cf.gate.czSign == -1.0);                               // ... 但号【定了】, 且是翻的
    CHECK(nearRefAbs(cf.comMm[2], -10.0));
    PayloadCalibration::SendCandidateDiff dff =
        PayloadCalibration::diffSendCandidate(cf, 0.404, czRobPos);
    CHECK(!dff.sendable);
    CHECK(dff.czSignFlipped);                                    // ★ 号被翻
    PayloadCalibration::formatSendCandidateDiffConclusion(dff, buf, sizeof(buf));
    const std::string tf(buf);
    CHECK(tf.find("高危") != std::string::npos);                  // ★ 翻号必须标高危 — 优先于"发不出去"
    CHECK(tf.find("只改 m") == std::string::npos);
    // ★ M6: 【翻号 + 发不出去】时"要不要发"这个选项【不存在】—— 按 'p' 会被确定性地拒掉,
    //   所以那句话不许出现; 该说的是"没有发不发可决定"。
    CHECK(tf.find("决定发不发") == std::string::npos);
    CHECK(tf.find("候选不可发送") != std::string::npos);
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
    // 模型形式检验: 残差 vs 【姿态间复现性】(重复姿态对测出来的尺子)。判据的尺度全来自数据。
    test_modelform_rejects_transposed_convention();   // ★ 评审的反例 (旧门限放它过去)
    test_modelform_accepts_correct_fit_with_noise();
    test_modelform_gate_tracks_repeat_reproducibility();   // ★ 判决跟着尺子走, 不跟姿态内噪声走
    test_moment_lack_of_fit_rejects_non_cross_product();
    test_modelform_unverified_refused_not_accepted();    // ★ 第三次修复: 门限必须承认【尺子自己也是估量】(分母的自由度与单侧置信下界)
    test_modelform_limit_is_a_chi2_quantile_times_a_yardstick_discount();
    test_modelform_accepts_at_real_operating_point();
    test_repeat_pairs_pool_and_carry_their_dof();
    test_repeat_pair_registration();   // ★ 采集侧: 'r' 配的是【上一笔】(原地复采), 不是 pose 1
    test_dead_channel_covers_all_six_frozen();
      // ★ 没验过 -> 不给参数
    test_pose_residuals_mark_the_worst_pose();             // spec §3 表格第 4 行
    test_realistic_spread_accepted_with_measured_noise();
    test_rawfit_rejects_too_few_poses();
    test_rawfit_rejects_degenerate_poses();
    test_rawfit_rejects_bad_mass_scale();
    // ★ 2026-09-19 (b): 力矩门限重标定 —— 一条钉"没动什么"(力门限逐位不变), 一条钉
    //   "动了什么"(门限 = c0·LIMIT_prod + κ·e, 且 e 与离线报告的五位小数相同)。
    moment_gate_force_limit_is_bit_identical_to_the_old_formula();   // ★ 力那一条支路没被碰过
    moment_gate_limit_is_c0_times_prod_plus_kappa_times_e();         // ★ 力矩门限的结构与口径

    // ★ 实机回归: 7 个真实姿态的【冻结夹具】重放, 逐项对金标 (夹具已入库, 本该必然跑;
    //   真 SKIP 了 = 检出缺文件)。
    // 【放在最后】: 它会把 [Payload] 的逐姿态残差表与"接受未检验模型形式"的告警打到
    // stderr, 排在最后免得那些输出插在别的用例中间。
    test_replay_real_capture();

    // ★ 力矩门限的实机标定 (2026-09-19 三次采集: 15:25 / 15:30 / 15:33):
    //   金标 + 零假设蒙特卡洛 —— 用【生产 fitRaw】量"叉乘模型为真时被拒的概率", 以分开
    //   (i) 力矩模型真不完备 与 (ii) 门限对"少 6 个参数"标偏。只测量, 不改任何门限。
    std::cout << "--- moment gate calibration (real captures, production fitRaw) ---" << std::endl;
    test_moment_gate_real_captures_golden();
    test_moment_gate_null_false_reject_rate();

    // ★★★ Task 6 验收: 本地补偿的姿态无关性 (全量 vs 残余, 逐通道, 四份采集)。
    // 【放在最后】: 它要跑生产补偿代码 (ForceCompensation::step), 并且会打一张表。
    std::cout << "--- Task 6 acceptance: pose-independence of local compensation ---" << std::endl;
    test_runtime_compensation_pose_independence();

    // ★★ 运行时一致性闸门 (2026-09-19, 用户指令 1/2) 在同样四份夹具上的离线重放。
    std::cout << "--- runtime consistency guard (replay on the four captures) ---" << std::endl;
    test_runtime_consistency_guard_replay();

    // ⚠ TODO (Task 8a-3) —— 覆盖缺口, 只记录, 本次不动 runner:
    //   tests\run_tests.bat 【不跑本文件】。它跑的是另外 12 个用例程序 (7 个预构建 exe +
    //   force_compensation / relay_command_parser / force_logger / tcp_calibration /
    //   session_report), 里面没有 test_payload_calibration。所以那句"12 个用例程序全 0 failed"
    //   【不覆盖】下面 Task 8a / 8a-2 的回归钉子 —— 它们只有单独构建并运行本文件才会跑到
    //   (tests\build_payload_calibration_test.bat)。runner 是受保护的, 不在这里改。

    // ★★★ Task 8a: 下发负载前的两道闸 (上机操作单 §6)。纯逻辑、无 socket —— 所以能在这里钉住,
    //   而它是【唯一】会在真正下发之前拒绝的防线 (发送键 'p' 直接消费它的判决)。
    std::cout << "--- Task 8a: pre-send gates (sign convention + magnitude) ---" << std::endl;
    test_send_gate_convention_one_wins_and_signs_cz();
    test_send_gate_convention_two_wins_and_signs_cz();
    test_send_gate_two_conventions_in_range_is_ambiguous();
    test_send_gate_no_convention_in_range_is_refused();
    test_send_gate_interval_is_open_at_both_ends();
    test_send_gate_mass_bounds_are_inclusive();
    test_send_gate_mass_outside_bounds_is_refused();
    test_send_gate_com_magnitude_bound_is_exclusive_at_500();
    test_send_gate_refuses_without_cs_and_says_so();
    test_send_gate_refuses_without_cz_robot_and_says_so();

    // ★★ Task 8a-2: 候选构造 / 来源判据 / 措辞 / 与当前值的比对。8a 交付时 'p' 发的是
    //   effective() (= 上次落盘值或种子), 发不出本次标定的结果 —— 这一节就是那条断路。
    std::cout << "--- Task 8a-2: candidate construction + provenance + wording ---" << std::endl;
    test_send_gate_refuses_unmeasured_mass_and_says_so();
    test_send_gate_convention_two_wins_with_negative_cz_robot();
    test_send_gate_small_positive_cz_robot_flips_the_sign();
    test_build_send_candidate_normal_path();
    test_build_send_candidate_without_measured_mass_has_none();
    test_build_send_candidate_without_echo_has_none();
    test_send_candidate_mass_label_matches_brief_wording();
    test_send_candidate_center_label_follows_the_verdict();
    test_send_candidate_diff_reports_unchanged_c_on_the_real_machine_case();
    test_send_candidate_diff_flags_flipped_cz_as_high_risk();
    test_send_candidate_diff_conclusion_never_says_only_m_when_unsendable();

    std::cout << "\n" << g_passed << " passed, " << g_failed << " failed" << std::endl;
    return g_failed ? 1 : 0;
}
