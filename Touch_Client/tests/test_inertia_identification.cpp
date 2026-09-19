// Standalone test: InertiaIdentification — 从设计激励下的原始力旋量反解工具链惯量张量
// Build: build_inertia_identification_test.bat
// Run: test_inertia_identification.exe
//
// 【生成端必须独立】—— 本项目在这上面栽过两次 (历史教训写在 build_payload_calibration_test
// 的注释里: 单测用错的约定造数据、求解器用同一错约定解回来, 10/10 全绿, 而实机上解错)。
// 所以本文件:
//   · 自己写 RPY->R (从定义展开, 不调 TcpCalibration::rpyToMatrix)
//   · 自己写基座系重力在传感器系里的表示 (Rᵀ·(0,0,G), 不调 gravitySensorFrameAtYaw)
//   · 自己写解析轨迹与解析导数 —— 【这是关键的独立性】: 被测代码要从【拟合】里把运动学
//     反推出来, 而这里直接给出真值。拟合若有偏差, 差异立刻显形。
//   · 自己写力旋量模型 (显式公式), 不调 identifyInertia 的任何东西。
// 唯一的"共享"是物理约定本身 —— 那是被测对象的前提, 不是实现细节; 它由零运动一致性
// 用例钉死 (那条用例把两边接在静力学模型上)。

#include <iostream>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include "../force/InertiaIdentification.h"

static int g_passed = 0, g_failed = 0;

#define TEST(name) do { std::cout << "  " << #name << "... " << std::flush; } while(0)
#define PASS() do { std::cout << "PASS" << std::endl; g_passed++; } while(0)
#define FAILMSG(msg) do { std::cout << "FAIL: " << (msg) << std::endl; g_failed++; return; } while(0)
#define CHECK(cond) do { if (!(cond)) { std::cout << "FAIL: " << #cond << std::endl; g_failed++; return; } } while(0)
#define CHECK_NEAR(a, b, tol) do { double _d = fabs((a)-(b)); if (!(_d <= (tol))) { \
    std::cout << "FAIL: |" << #a << " - " << #b << "| = " << _d << " > " << (tol) << std::endl; \
    g_failed++; return; } } while(0)

static const double G_ACC = 9.81;
static const double PI_T  = 3.14159265358979323846;
static const double D2R   = PI_T / 180.0;

// ===== 独立实现 1: RPY -> R, 单位【度】, R = Rz·Ry·Rx (row-major) =====
// 与 TcpCalibration::rpyToMatrix 同约定, 但由本文件从定义展开写 —— 不是为了"换个写法",
// 是为了让"约定漂移"能在编译期之外被抓到: 两边一旦不一致, 这个文件里的用例会红。
static void tRpyToR(double rxDeg, double ryDeg, double rzDeg, double R[9]) {
    const double rx = rxDeg * D2R, ry = ryDeg * D2R, rz = rzDeg * D2R;
    const double crx = cos(rx), srx = sin(rx);
    const double cry = cos(ry), sry = sin(ry);
    const double crz = cos(rz), srz = sin(rz);
    R[0] = crz * cry;  R[1] = crz * sry * srx - srz * crx;  R[2] = crz * sry * crx + srz * srx;
    R[3] = srz * cry;  R[4] = srz * sry * srx + crz * crx;  R[5] = srz * sry * crx - crz * srx;
    R[6] = -sry;       R[7] = cry * srx;                     R[8] = cry * crx;
}

// ===== 独立实现 2: 重力在传感器(位姿)系的表示 =====
// g_s[i] = (Rᵀ·(0,0,G))[i] = Σ_k R[k][i]·v[k], 只剩 k=2 一项 -> 取 R 的第三【行】。
// ⚠ 【不是第三列】: 取第三列等于 R·(0,0,G), 与 Rᵀ 差一个转置。历史上就错在这里, 转置后
//   仍与真值相关、残差不会爆掉, 只会安静地解错 —— 所以这里从 Σ 展开写。
static void tGravitySensor(const double pose[6], double g[3]) {
    double R[9];
    tRpyToR(pose[3], pose[4], pose[5], R);
    const double v[3] = {0.0, 0.0, G_ACC};
    for (int i = 0; i < 3; i++) {
        g[i] = 0.0;
        for (int k = 0; k < 3; k++) g[i] += R[k * 3 + i] * v[k];
    }
}

static void tMatVec(const double M[9], const double v[3], double y[3]) {
    for (int a = 0; a < 3; a++)
        y[a] = M[a * 3 + 0] * v[0] + M[a * 3 + 1] * v[1] + M[a * 3 + 2] * v[2];
}
static void tMatTVec(const double M[9], const double v[3], double y[3]) {
    for (int i = 0; i < 3; i++)
        y[i] = M[0 * 3 + i] * v[0] + M[1 * 3 + i] * v[1] + M[2 * 3 + i] * v[2];
}
static void tCross(const double a[3], const double b[3], double c[3]) {
    c[0] = a[1] * b[2] - a[2] * b[1];
    c[1] = a[2] * b[0] - a[0] * b[2];
    c[2] = a[0] * b[1] - a[1] * b[0];
}
static void tMatMul(const double A[9], const double B[9], double C[9]) {
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++) {
            double acc = 0.0;
            for (int k = 0; k < 3; k++) acc += A[r * 3 + k] * B[k * 3 + c];
            C[r * 3 + c] = acc;
        }
}

// ===== 独立实现 3: 解析轨迹 (混合基波 + 二次谐波, 各轴带相位) =====
// 二次谐波是【故意的】: 它让"谐波阶数由 BIC 从数据里定"这件事有东西可定。
struct Traj {
    double freqHz;
    double p0[3];       // mm
    double Ap[3];       // mm 幅度
    double phP[3];      // 相位 (rad)
    double kP[3];       // 每轴落在第几次谐波 (1 或 2)
    double Ar[3];       // 度 幅度
    double phR[3];
    double kR[3];

    void eval(double t, double pos[3], double rpy[3], double vel[3], double acc[3],
              double rdot[3], double rddot[3]) const
    {
        const double W = 2.0 * PI_T * freqHz;
        for (int i = 0; i < 3; i++) {
            const double w = W * kP[i], ph = w * t + phP[i];
            pos[i] = p0[i] + Ap[i] * sin(ph);
            vel[i] = Ap[i] * w * cos(ph);                  // mm/s
            acc[i] = -Ap[i] * w * w * sin(ph);             // mm/s^2
        }
        for (int i = 0; i < 3; i++) {
            const double w = W * kR[i], ph = w * t + phR[i];
            rpy[i]   = Ar[i] * sin(ph);                    // 度
            rdot[i]  = Ar[i] * w * cos(ph) * D2R;          // rad/s
            rddot[i] = -Ar[i] * w * w * sin(ph) * D2R;     // rad/s^2
        }
    }
};

// 轨迹的角速度与角加速度 —— 从定义独立展开:
//   ω_B = ẑ·ṙz + Rz·(ŷ·ṙy) + Rz·Ry·(x̂·ṙx)
//   ω̇_B = ẑ·r̈z + ṙz(ẑ×Rzŷ)·ṙy + Rzŷ·r̈y + ṙz(ẑ×RzRy x̂)·ṙx + ṙy·Rz(ŷ×(Ry x̂))·ṙx + RzRy x̂·r̈x
// 代回: ω_S = Rᵀω_B, α_S = Rᵀω̇_B (推导: Ṙ = [ω_B]ₓR ⟹ Ṙᵀω_B = 0, 只剩 Rᵀω̇_B)
static void tAngular(const double rpy[3], const double rdot[3], const double rddot[3],
                     double omegaS[3], double alphaS[3])
{
    double R[9], Rz[9], Ry[9], RzRy[9];
    tRpyToR(rpy[0], rpy[1], rpy[2], R);
    tRpyToR(0.0, 0.0, rpy[2], Rz);
    tRpyToR(0.0, rpy[1], 0.0, Ry);
    tMatMul(Rz, Ry, RzRy);

    const double c1[3] = {0.0, 0.0, 1.0};
    const double c2[3] = {Rz[1], Rz[4], Rz[7]};            // Rz·ŷ  (第二列)
    const double c3[3] = {RzRy[0], RzRy[3], RzRy[6]};      // Rz·Ry·x̂ (第一列)
    double wB[3];
    for (int i = 0; i < 3; i++)
        wB[i] = c1[i] * rdot[2] + c2[i] * rdot[1] + c3[i] * rdot[0];

    double RyX[3];
    tMatTVec(Rz, c3, RyX);                                  // Ry·x̂ = Rzᵀ·(Rz Ry x̂)
    const double zhat[3] = {0.0, 0.0, 1.0}, yhat[3] = {0.0, 1.0, 0.0};
    double zc2[3], zc3[3], yy[3], RzYy[3];
    tCross(zhat, c2, zc2);
    tCross(zhat, c3, zc3);
    tCross(yhat, RyX, yy);
    tMatVec(Rz, yy, RzYy);

    double wDotB[3];
    for (int i = 0; i < 3; i++)
        wDotB[i] = c1[i] * rddot[2]
                 + zc2[i] * rdot[2] * rdot[1] + c2[i] * rddot[1]
                 + (zc3[i] + RzYy[i]) * rdot[2] * rdot[0] + c3[i] * rddot[0];

    tMatTVec(R, wB, omegaS);
    tMatTVec(R, wDotB, alphaS);
}

// ===== 独立实现 4: 力旋量模型 (显式公式) =====
//   F = b_F + A·g_s + m·a_com^ch
//   M = b_M + c_s × (A·g_s) + c_s × (m·a_O^ch) + I·α^ch + ω^ch × (I·ω^ch)
// 通道坐标 = (A/m) 作用在位姿系坐标上 (与静力学模型的 A 同一个约定)。
struct Model {
    double m;
    double cs[3];          // m
    double A[9];
    double bF[3], bM[3];
    double I[9];
};

static void buildWrench(const Model& md, const Traj& tr, double t,
                        double pose[6], double speed[6], double wrench[6])
{
    double pos[3], rpy[3], vel[3], acc[3], rdot[3], rddot[3];
    tr.eval(t, pos, rpy, vel, acc, rdot, rddot);
    for (int i = 0; i < 3; i++) {
        pose[i] = pos[i]; pose[3 + i] = rpy[i];
        speed[i] = vel[i];
        speed[3 + i] = rdot[i] / D2R;      // 度/s
    }
    double R[9];
    tRpyToR(rpy[0], rpy[1], rpy[2], R);

    double omegaS[3], alphaS[3];
    tAngular(rpy, rdot, rddot, omegaS, alphaS);
    double aOS[3];
    tMatTVec(R, acc, aOS);                 // 位姿系, mm/s^2
    for (int i = 0; i < 3; i++) aOS[i] /= 1000.0;   // -> m/s^2

    // 通道坐标: v^ch = (A/m)·v^S
    const double invm = 1.0 / md.m;
    double omC[3], alC[3], aOC[3];
    for (int i = 0; i < 3; i++) {
        omC[i] = (md.A[i * 3 + 0] * omegaS[0] + md.A[i * 3 + 1] * omegaS[1]
                  + md.A[i * 3 + 2] * omegaS[2]) * invm;
        alC[i] = (md.A[i * 3 + 0] * alphaS[0] + md.A[i * 3 + 1] * alphaS[1]
                  + md.A[i * 3 + 2] * alphaS[2]) * invm;
        aOC[i] = (md.A[i * 3 + 0] * aOS[0] + md.A[i * 3 + 1] * aOS[1]
                  + md.A[i * 3 + 2] * aOS[2]) * invm;
    }
    double t1[3], t2[3], t3[3], acomC[3];
    tCross(alC, md.cs, t1);
    tCross(omC, md.cs, t2);
    tCross(omC, t2, t3);
    for (int i = 0; i < 3; i++) acomC[i] = aOC[i] + t1[i] + t3[i];

    double gs[3];
    tGravitySensor(pose, gs);
    double Ag[3];
    tMatVec(md.A, gs, Ag);

    double cg[3], ca[3], m_aOC[3];
    for (int i = 0; i < 3; i++) m_aOC[i] = md.m * aOC[i];
    tCross(md.cs, Ag, cg);
    tCross(md.cs, m_aOC, ca);

    double Iom[3];
    tMatVec(md.I, omC, Iom);
    double Ial[3];
    tMatVec(md.I, alC, Ial);
    double wIw[3];
    tCross(omC, Iom, wIw);

    for (int a = 0; a < 3; a++) {
        wrench[a]     = md.bF[a] + Ag[a] + md.m * acomC[a];
        wrench[3 + a] = md.bM[a] + cg[a] + ca[a] + Ial[a] + wIw[a];
    }
}

// ===== 场景组装 =====
struct Scene {
    Model md;
    Traj  tr;
    int   n;
};

static void defaultModel(Model& md, double m, double parity)
{
    md.m = m;
    md.cs[0] = 0.0030; md.cs[1] = -0.0020; md.cs[2] = 0.05455;   // m (spec §6c: c_s = 54.55 mm)
    // A = m·S·Q, S = diag(1,1,parity), Q 为正交装夹姿态 -> 与静力学标定的分解形式同构
    double Q[9];
    tRpyToR(12.0, -5.0, 8.0, Q);
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            md.A[r * 3 + c] = m * ((r == 2) ? parity : 1.0) * Q[r * 3 + c];
    md.bF[0] = 0.42; md.bF[1] = -0.31; md.bF[2] = 1.07;
    md.bM[0] = -0.011; md.bM[1] = 0.004; md.bM[2] = 0.002;

    // 工具链量级: 与 spec §6c 的 CAD 参考同量级 (10^-3 ~ 10^-4 kg·m^2)。
    // ⚠ 【必须是一个真的惯量张量】: 绕任一点的惯量张量正定且满足 I1+I2 >= I3。
    //    spec §6c 给的那对 CAD 值 (Ixx=Iyy≈8.5e-3, Izz≈4.1e-4) 就【不满足】后者
    //    (8.5e-3+8.5e-3 ≪ 17e-3), 所以这里不能照抄 —— 照抄造出来的是非物理数据。
    //    这里取一个"扁盘 + 小交叉项"的真张量: 8.5 + 14.5 >= 15.2 有余量。
    md.I[0] = 8.5e-3;  md.I[1] = 2.0e-4;  md.I[2] = -1.0e-4;
    md.I[3] = 2.0e-4;  md.I[4] = 14.5e-3;  md.I[5] = 3.0e-4;
    md.I[6] = -1.0e-4; md.I[7] = 3.0e-4;   md.I[8] = 15.2e-3;
}

static void defaultTraj(Traj& tr, double freqHz)
{
    tr.freqHz = freqHz;
    tr.p0[0] = -110.7; tr.p0[1] = -58.6; tr.p0[2] = 497.1;     // spec §6c 的实测信封中心
    tr.Ap[0] = 18.0; tr.Ap[1] = 12.0; tr.Ap[2] = 7.0;          // mm
    tr.phP[0] = 0.0; tr.phP[1] = 0.7; tr.phP[2] = 0.3;
    tr.kP[0] = 1; tr.kP[1] = 1; tr.kP[2] = 2;
    tr.Ar[0] = 11.0; tr.Ar[1] = 9.0; tr.Ar[2] = 7.0;           // 度 (腕转 ±20° 之内)
    tr.phR[0] = 0.2; tr.phR[1] = -0.5; tr.phR[2] = 0.9;
    tr.kR[0] = 1; tr.kR[1] = 1; tr.kR[2] = 2;
}

// 简单可复现的伪随机 (不用 rand(): 用例要能逐位复现)
static double lcg(uint32_t& st) {
    st = st * 1664525u + 1013904223u;
    return ((double)(st >> 8) / 16777216.0) * 2.0 - 1.0;       // [-1, 1)
}

static const int MAXS = 4096;

// 造一批采样点。poseNoise / wrenchNoise 为 0 时是无噪声重放。
static int makeSamples(const Scene& sc, InertiaSample* out, double t0, double dt,
                       double poseNoise, double wrenchNoise, uint32_t seed = 12345u)
{
    uint32_t st = seed;
    for (int i = 0; i < sc.n; i++) {
        const double t = t0 + i * dt;
        double pose[6], speed[6], wrench[6];
        buildWrench(sc.md, sc.tr, t, pose, speed, wrench);
        InertiaSample& s = out[i];
        s.t = t;
        for (int k = 0; k < 6; k++) {
            s.pose[k]   = pose[k]   + (poseNoise   > 0.0 ? poseNoise * lcg(st) : 0.0);
            s.speed[k]  = speed[k];
            s.wrench[k] = wrench[k] + (wrenchNoise > 0.0 ? wrenchNoise * lcg(st) : 0.0);
        }
    }
    return sc.n;
}

static InertiaFit runFit(const Scene& sc, const InertiaSample* s, int n, double freqHz)
{
    InertiaFit f;
    identifyInertia(s, n, freqHz, sc.md.m, sc.md.cs, sc.md.A, sc.md.bF, sc.md.bM, f);
    return f;
}

// =====================================================================================
// 用例
// =====================================================================================

// ★ 零运动一致性 —— 本模块最容易静默出错的地方就是这里 (坐标系约定)。
// 姿态恒定、ω = α = 0 时, 辨识模型必须【精确】退化成静力学模型:
//   残差与静力学残差逐位相同, 且 I_O 不被拉向任何地方。
static void test_zero_motion_reduces_to_static()
{
    TEST(test_zero_motion_reduces_to_static);
    Model md; defaultModel(md, 0.42, -1.0);
    Traj tr; defaultTraj(tr, 0.5);
    // 常数轨迹: 所有谐波幅度为 0
    for (int i = 0; i < 3; i++) { tr.Ap[i] = 0.0; tr.Ar[i] = 0.0; }
    Scene sc; sc.md = md; sc.tr = tr; sc.n = 400;

    static InertiaSample s[MAXS];
    // 手写常数位姿 + 静力学模型的 wrench (不调用 buildWrench 的动力学部分)
    const double pose[6] = {-110.7, -58.6, 497.1, 15.0, -8.0, 22.0};
    double gs[3]; tGravitySensor(pose, gs);
    double Ag[3]; tMatVec(md.A, gs, Ag);
    double cg[3]; tCross(md.cs, Ag, cg);
    for (int i = 0; i < 400; i++) {
        s[i].t = i / 125.0;
        for (int k = 0; k < 6; k++) { s[i].pose[k] = pose[k]; s[i].speed[k] = 0.0; }
        for (int a = 0; a < 3; a++) {
            s[i].wrench[a]     = md.bF[a] + Ag[a];
            s[i].wrench[3 + a] = md.bM[a] + cg[a];
        }
    }
    InertiaFit f = runFit(sc, s, 400, 0.5);

    // 静力学模型的残差 (本文件独立算一遍)
    double ssF = 0.0, ssM = 0.0;
    for (int i = 0; i < 400; i++) {
        for (int a = 0; a < 3; a++) {
            const double eF = (md.bF[a] + Ag[a]) - s[i].wrench[a];
            const double eM = (md.bM[a] + cg[a]) - s[i].wrench[3 + a];
            ssF += eF * eF; ssM += eM * eM;
        }
    }
    const double staticRmsF = sqrt(ssF / (3.0 * 400));
    const double staticRmsM = sqrt(ssM / (3.0 * 400));

    // 【判据 1】残差与静力学残差逐位相同 (零运动下惯性项必须整项消失)
    CHECK_NEAR(f.rmsForceN, staticRmsF, 1e-12);
    CHECK_NEAR(f.rmsMomentNm, staticRmsM, 1e-12);

    // 【判据 2】I_O 不被拉向任何地方 —— 没有运动就没有信息, 必须【拒绝】而不是给一个数。
    // (零激励下 JᵀJ = 0, 秩亏; determinacyOk 也为假。)
    CHECK(!f.ok);
    CHECK(!f.determinacyOk);
    for (int i = 0; i < 9; i++) CHECK_NEAR(f.I[i], 0.0, 1e-12);

    // 【判据 3】有静力学残差时同样成立 —— 塞一个常数偏移进去, 辨识的残差必须【等于】
    // 那个偏移的 rms (惯性项仍然整项消失, 一点都不能被"吸收"进 I_O)。
    const double off = 0.037;
    for (int i = 0; i < 400; i++) {
        s[i].wrench[0] += off;
        s[i].wrench[3 + 1] -= off * 0.01;
    }
    InertiaFit f2 = runFit(sc, s, 400, 0.5);
    const double expectF = sqrt((ssF + 400.0 * off * off) / (3.0 * 400));
    const double expectM = sqrt((ssM + 400.0 * (off * 0.01) * (off * 0.01)) / (3.0 * 400));
    CHECK_NEAR(f2.rmsForceN, expectF, 1e-12);
    CHECK_NEAR(f2.rmsMomentNm, expectM, 1e-12);
    CHECK(!f2.ok);
    PASS();
}

// ★ 合成恢复 + 报告的 sigma 有意义。两个 parity 各来一遍 —— 约定不该依赖手系。
static void test_recovers_inertia_within_sigma(double parity)
{
    Model md; defaultModel(md, 0.42, parity);
    Traj tr; defaultTraj(tr, 0.5);
    Scene sc; sc.md = md; sc.tr = tr; sc.n = 750;              // 6 s @ 125 Hz = 3 个周期

    static InertiaSample s[MAXS];
    makeSamples(sc, s, 0.0, 1.0 / 125.0, 0.01, 0.004);
    InertiaFit f = runFit(sc, s, sc.n, tr.freqHz);

    CHECK(f.ok);
    CHECK(f.harmonicOrder >= 2);            // 轨迹含二次谐波, BIC 必须看得见
    CHECK(f.cond < 1.0e3);                  // 多轴激励应当把六个分量分开

    const double Iref[6] = { md.I[0], md.I[4], md.I[8], md.I[1], md.I[2], md.I[5] };
    const int idx[6] = {0, 4, 8, 1, 2, 5};
    for (int k = 0; k < 6; k++) {
        const double d = fabs(f.I[idx[k]] - Iref[k]);
        if (!(d <= 5.0 * f.sigma[k] + 1e-12)) {
            char buf[256];
            snprintf(buf, sizeof(buf), "分量 %d: |%.6g - %.6g| = %.3g > 5σ = %.3g",
                     k, f.I[idx[k]], Iref[k], d, 5.0 * f.sigma[k]);
            FAILMSG(buf);
        }
    }
    PASS();
}

static void test_recovers_inertia_parity_plus()  { TEST(test_recovers_inertia_parity_plus);  test_recovers_inertia_within_sigma(+1.0); }
static void test_recovers_inertia_parity_minus() { TEST(test_recovers_inertia_parity_minus); test_recovers_inertia_within_sigma(-1.0); }

// ★ 力矩方程对 I_O 严格线性 -> 无噪声下应当恢复到【接近机器精度】。
// 同一个运动, 换一个 I_O 再来一次: 两次的解都必须精确。(单项通过不能说明线性。)
static void test_moment_equation_is_exactly_linear()
{
    TEST(test_moment_equation_is_exactly_linear);
    Model md; defaultModel(md, 0.42, -1.0);
    Traj tr; defaultTraj(tr, 0.5);
    Scene sc; sc.md = md; sc.tr = tr; sc.n = 500;

    static InertiaSample s[MAXS];
    const int idx[6] = {0, 4, 8, 1, 2, 5};
    const double scales[2] = {1.0, 3.7};      // 第二个张量是第一个的 3.7 倍 (线性才有意义)
    for (int trial = 0; trial < 2; trial++) {
        for (int i = 0; i < 9; i++) sc.md.I[i] = md.I[i] * scales[trial];
        makeSamples(sc, s, 0.0, 1.0 / 125.0, 0.0, 0.0);
        InertiaFit f = runFit(sc, s, sc.n, tr.freqHz);
        CHECK(f.ok);
        for (int k = 0; k < 6; k++) {
            const double want = md.I[idx[k]] * scales[trial];
            const double rel = fabs(f.I[idx[k]] - want) / fabs(want);
            if (!(rel < 1e-9)) {
                char buf[256];
                snprintf(buf, sizeof(buf), "trial %d 分量 %d: 相对误差 %.3g (要 < 1e-9)",
                         trial, k, rel);
                FAILMSG(buf);
            }
        }
    }
    PASS();
}

// ★ 运动学检查必须能拒绝 —— 激励不是命令的那个频率
static void test_rejects_wrong_excitation_frequency()
{
    TEST(test_rejects_wrong_excitation_frequency);
    Model md; defaultModel(md, 0.42, -1.0);
    Traj tr; defaultTraj(tr, 0.5);
    Scene sc; sc.md = md; sc.tr = tr; sc.n = 750;

    static InertiaSample s[MAXS];
    makeSamples(sc, s, 0.0, 1.0 / 125.0, 0.0, 0.0);

    // 数据真的在 0.5 Hz, 却告诉它 1.25 Hz (2.5 倍)
    InertiaFit f = runFit(sc, s, sc.n, 1.25);
    CHECK(!f.ok);
    CHECK(!f.kinematicsOk);
    // 同样的数据, 报对频率 -> 通过 (证明拒绝的理由是频率, 不是数据本身)
    InertiaFit g = runFit(sc, s, sc.n, 0.5);
    CHECK(g.ok);
    PASS();
}

// ★ 独立来源对不上: TCPSpeedActual 通道被污染
static void test_rejects_corrupted_speed_channel()
{
    TEST(test_rejects_corrupted_speed_channel);
    Model md; defaultModel(md, 0.42, -1.0);
    Traj tr; defaultTraj(tr, 0.5);
    Scene sc; sc.md = md; sc.tr = tr; sc.n = 750;

    static InertiaSample s[MAXS];
    makeSamples(sc, s, 0.0, 1.0 / 125.0, 0.0, 0.0);
    for (int i = 0; i < sc.n; i++) s[i].speed[0] += 400.0;      // mm/s 的假速
    InertiaFit f = runFit(sc, s, sc.n, 0.5);
    CHECK(!f.ok);
    CHECK(!f.kinematicsOk);
    // 假速是加在【一个轴】上的 400 mm/s, 三分量的 rms 差 = 400/sqrt(3) ≈ 231
    CHECK(f.speedCheckRms > 200.0);
    PASS();
}

// ★ 单轴往复【不足以保证可辨识】—— 恒有 M = (αE + ω[n]ₓ)(I·n), 只依赖 I·n 三个量,
//   秩最多 3 < 6。必须拒绝, 而不是给一个"看着合理"的张量。
static void test_rejects_single_axis_excitation()
{
    TEST(test_rejects_single_axis_excitation);
    Model md; defaultModel(md, 0.42, -1.0);
    Traj tr; defaultTraj(tr, 0.5);
    tr.Ap[0] = tr.Ap[1] = tr.Ap[2] = 0.0;      // 不平移
    tr.Ar[0] = tr.Ar[1] = 0.0;                 // 只绕一个轴 (z) 往复
    tr.Ar[2] = 12.0; tr.kR[2] = 1;
    Scene sc; sc.md = md; sc.tr = tr; sc.n = 600;

    static InertiaSample s[MAXS];
    makeSamples(sc, s, 0.0, 1.0 / 125.0, 0.0, 0.0);
    InertiaFit f = runFit(sc, s, sc.n, tr.freqHz);
    CHECK(!f.ok);
    CHECK(!f.determinacyOk);
    PASS();
}

// ★ 符号约定的护栏: 用力矩通道的符号【反】的模型造数据, 解出来是 −I_O。
//   它必须被物理门限 (绕一点的惯量张量必正定) 拦住 —— 而不是当成一个"合理"的负数发出去。
static void test_rejects_nonphysical_negative_inertia()
{
    TEST(test_rejects_nonphysical_negative_inertia);
    Model md; defaultModel(md, 0.42, -1.0);
    for (int i = 0; i < 9; i++) md.I[i] = -md.I[i];            // 非物理 (负定)
    Traj tr; defaultTraj(tr, 0.5);
    Scene sc; sc.md = md; sc.tr = tr; sc.n = 600;

    static InertiaSample s[MAXS];
    makeSamples(sc, s, 0.0, 1.0 / 125.0, 0.0, 0.0);
    InertiaFit f = runFit(sc, s, sc.n, tr.freqHz);
    CHECK(!f.ok);
    CHECK(!f.physicalOk);
    CHECK(f.eig[2] < 0.0);
    PASS();
}

// 报告的 sigma 是否标定得对: 多次噪声实现下, 归一化误差 ΔI/σ 的 rms 应当 ≈ 1。
// (只测一次恢复不到"sigma 对不对"; 尺子自己也要被量一次。)
static void test_sigma_is_calibrated()
{
    TEST(test_sigma_is_calibrated);
    Model md; defaultModel(md, 0.42, -1.0);
    Traj tr; defaultTraj(tr, 0.5);
    Scene sc; sc.md = md; sc.tr = tr; sc.n = 750;

    static InertiaSample s[MAXS];
    const int idx[6] = {0, 4, 8, 1, 2, 5};
    double z2 = 0.0, zmax = 0.0;
    int cnt = 0;
    for (uint32_t seed = 1; seed <= 16; seed++) {
        makeSamples(sc, s, 0.0, 1.0 / 125.0, 0.01, 0.004, seed * 7919u);
        InertiaFit f = runFit(sc, s, sc.n, tr.freqHz);
        CHECK(f.ok);
        for (int k = 0; k < 6; k++) {
            const double sig = f.sigma[k];
            if (!(sig > 0.0)) continue;
            const double z = (f.I[idx[k]] - md.I[idx[k]]) / sig;
            z2 += z * z; cnt++;
            if (fabs(z) > zmax) zmax = fabs(z);
        }
    }
    CHECK(cnt > 0);
    const double zrms = sqrt(z2 / cnt);
    if (!(zrms > 0.4 && zrms < 2.5)) {
        char buf[128];
        snprintf(buf, sizeof(buf), "归一化误差 rms = %.3g (期望 ≈ 1), max|z| = %.3g", zrms, zmax);
        FAILMSG(buf);
    }
    if (!(zmax < 6.0)) FAILMSG("max|z| >= 6, sigma 偏小");
    std::cout << "(归一化误差 rms = " << zrms << ", max|z| = " << zmax << ") ";
    PASS();
}

int main()
{
    std::cout << "InertiaIdentification tests" << std::endl;

    std::cout << "--- 坐标系约定 / 零运动一致性 ---" << std::endl;
    test_zero_motion_reduces_to_static();

    std::cout << "--- 合成恢复 ---" << std::endl;
    test_recovers_inertia_parity_plus();
    test_recovers_inertia_parity_minus();
    test_moment_equation_is_exactly_linear();
    test_sigma_is_calibrated();

    std::cout << "--- 拒绝 ---" << std::endl;
    test_rejects_wrong_excitation_frequency();
    test_rejects_corrupted_speed_channel();
    test_rejects_single_axis_excitation();
    test_rejects_nonphysical_negative_inertia();

    std::cout << "\n" << g_passed << " passed, " << g_failed << " failed" << std::endl;
    return g_failed ? 1 : 0;
}
