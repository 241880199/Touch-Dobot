// Standalone test: ForceCompensation + ForceCalibration core logic
//
// 2026-09-19 (Task 6): 补偿模型从【残余】换成【全量】——
//   旧: compensated = fd.raw(@576) − b_F − mass·g(ψ)          mass = 可带符号的残余质量
//   新: compensated = fd.sixForceRaw(@1304) − b_F − A·g − Fi  A = 自由 3×3 (kg)
//       compensated_M = fd.sixForceRaw − b_M − c_s × (A·g)
// 所以下面凡是设参数的地方都改成 setCalibration(A, b_F, b_M, c_s), 凡是喂读数的地方
// 都改成 fd.sixForceRaw。断言里凡与模型形式有关的, 都重算过 (不是把旧的期望值搬过来)。
#include <iostream>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <windows.h>
#ifdef _WIN32
#include <io.h>          // _dup / _dup2 / _close —— "第一次拒绝"那段打印要抓进来看
#include <fcntl.h>       // _O_CREAT / _O_TRUNC / _O_BINARY
#include <sys/stat.h>    // _S_IREAD / _S_IWRITE
#endif
#include "../force/ForceCompensation.h"
#include "../force/ForceCalibration.h"
#include "../force/ZeroDriftCheck.h"
#include "../calibration/TcpCalibration.h"
#include "../config/Config.h"

// 3×3 对角 A = m·I: 这是【残余模型】在新模型里的等价物 (各向同性、无安装旋转), 用来
// 把"换通道/换写法"与"换模型本身"分开 —— 用它能逐位复算出旧测试的期望值。
static void diagA(double m, double A[9]) {
    for (int i = 0; i < 9; i++) A[i] = 0.0;
    A[0] = A[4] = A[8] = m;
}

// ===== "闸门看得见的一帧" (2026-09-21, Task 7) =====
// 闸门要求【参考量可用】: 帧新鲜 (fd.isStale == false) 且机械臂自报六维力在线
// (fd.sixForceOnline == 1)。生产里这两件事与 tcpForce 是【同一帧、同一把锁】里一起做好的
// (RelayCore 的 ForceReader 每收到一帧就置 isStale=false 并写 @1037), 所以凡是"给闸门喂一个
// 参考读数"的用例, 都得显式声明这一帧是新鲜的、在线 —— 否则闸门判的是【参考量不可用】
// (Task 7), 而用例名说的是另一件事。
// ⚠ 这不是把夹具放宽: 它补的是原来缺的那部分夹具事实 —— AppState::ForceData 的初值是
//   isStale=true / sixForceOnline=-1, 语义是"一帧都还没收到"。
static AppState::ForceData gateVisibleFrame() {
    AppState::ForceData fd;
    fd.isStale = false;
    fd.sixForceOnline = 1;
    return fd;
}

static int g_passed = 0, g_failed = 0;

#define TEST(name) do { std::cout << "  " << #name << "... "; } while(0)
#define PASS() do { std::cout << "PASS" << std::endl; g_passed++; } while(0)
#define CHECK(cond) do { if (!(cond)) { std::cout << "FAIL: " << #cond << std::endl; g_failed++; return; } } while(0)

// ===== MotionEstimator tests =====

static void test_motion_still() {
    TEST(motion_still);
    MotionEstimator est;
    // Feed same position 10 times
    for (int i = 0; i < 10; i++) {
        est.update(100.0, 200.0, 300.0, 0.008); // 8ms timestep
    }
    CHECK(est.isStill());
    double vel[3], acc[3];
    est.getState(vel, acc);
    CHECK(fabs(vel[0]) < 0.01);
    CHECK(fabs(vel[1]) < 0.01);
    CHECK(fabs(vel[2]) < 0.01);
    PASS();
}

static void test_motion_moving() {
    TEST(motion_moving);
    MotionEstimator est;
    // Moving at 0.1 m/s in X (100mm/s)
    double dt = 0.008;
    for (int i = 0; i < 10; i++) {
        double x = 100.0 + 0.1 * 1000.0 * i * dt; // mm
        est.update(x, 200.0, 300.0, dt);
    }
    double vel[3], acc[3];
    est.getState(vel, acc);
    CHECK(fabs(vel[0] - 0.1) < 0.02);  // ~0.1 m/s
    PASS();
}

// ===== ForceCompensation tests =====

// ★ 2026-09-19: 未标定【不再透传 @1304】, 改为【拒绝 + 报错】(用户指令 1)。
//   旧行为是评审判定的 Critical: @1304 带着 ~21.9 N 的零偏, 经 ForcePipeline 的
//   几何映射 (3.3/200) 与反射增益 5 之后, 手上得到 ~1.8 N 的恒定推力。
//   现在 compensated 全 6 个分量为 0, 闸门状态 = UNCALIBRATED, isCalibrated = false。
static void test_comp_uncalibrated_refuses() {
    TEST(comp_uncalibrated_refuses);
    ForceCompensation::init();
    AppState::ForceData fd = gateVisibleFrame();
    fd.sixForceRaw[0] = -20.65; fd.sixForceRaw[1] = -2.07; fd.sixForceRaw[2] = 3.055;
    fd.sixForceRaw[3] = -0.27;  fd.sixForceRaw[4] = 0.42;  fd.sixForceRaw[5] = -0.025;
    fd.raw[0] = -0.65; fd.raw[1] = -1.07; fd.raw[2] = 0.055;
    fd.raw[3] = -0.02; fd.raw[4] = 0.02;  fd.raw[5] = 0.005;

    double pose[6] = {0, 0, 0, 0, 0, 0};
    ForceCompensation::step(fd, pose);

    // 一个分量都不许漏 —— 漏掉的那一路就是透传。
    for (int i = 0; i < 6; i++) CHECK(fabs(fd.compensated[i]) < 1e-12);
    CHECK(fd.isCalibrated == false);
    // 原因必须是【没有模型】, 不能与"有模型但对不上"混成一个。
    CHECK(ForceCompensation::guardState() == ForceCompensation::GuardState::UNCALIBRATED);
    CHECK(fd.calibMassKg == 0.0);
    PASS();
}

static void test_comp_gravity_only() {
    TEST(comp_gravity_only);
    ForceCompensation::init();
    // A = 1·I (质量尺度 1 kg), c_s 在原点, 零偏全 0 —— 新模型里它退化成"标量质量 × g",
    // 与残余模型同形, 所以期望值可以直接与旧测试对照。
    double A[9]; diagA(1.0, A);
    double com[3] = {0, 0, 0};
    double biasF[3] = {0, 0, 0};
    double biasM[3] = {0, 0, 0};
    ForceCompensation::setCalibration(A, biasF, biasM, com);

    AppState::ForceData fd = gateVisibleFrame();
    // pose 0 (R=I) -> g = Rᵀ(0,0,9.81) = (0, 0, +9.81) —— 传感器吊着工具时读到的是 +Z 支撑力。
    // 期望: Fz 读 +9.81, 重力项把它减掉 -> 0。
    fd.sixForceRaw[0] = 0.0; fd.sixForceRaw[1] = 0.0; fd.sixForceRaw[2] = 9.81;
    fd.sixForceRaw[3] = 0.0; fd.sixForceRaw[4] = 0.0; fd.sixForceRaw[5] = 0.0;

    double pose[6] = {0, 0, 0, 0, 0, 0};
    ForceCompensation::step(fd, pose);
    CHECK(fabs(fd.compensated[2]) < 0.1); // gravity compensated away
    CHECK(fd.isCalibrated == true);
    // 质量尺度就是 |det A|^(1/3) —— 参数表里没有标量质量, 这个数由 A 现算。
    CHECK(fabs(ForceCompensation::currentMassKg() - 1.0) < 1e-12);
    PASS();
}

// ★ 重力必须【过 A】, 不是过标量 —— 这是本任务换掉的那件事本身。
// A 取置换阵 [[0,0,1],[0,1,0],[1,0,0]]: 它把 g 的 z 分量映到力的 x 分量, 把 g 的 x 分量
// 映到力的 z 分量 —— Fg = A·g 与 g 不平行, 标量模型 (Fg ∥ g) 无论如何做不到。
// 姿态取 R=I 与一个绕 z 转过的姿态, 两次都要减干净。
// ⚠ 2026-09-19: 这里【曾经】是"只有 A[2] 一个非零元"的单元素矩阵。那个 A 是秩 1 的,
//   而用户指令 3 要求"全零/退化的 A 拒绝传递数据", 所以 setCalibration 现在会拒收它
//   (见 test_setcalib_rejects_zero_and_degenerate_A)。换成置换阵之后行列式 = −1,
//   非退化, 而"过 A 不过标量"这件事一点没变: Fg = (g_z, g_y, g_x) ≠ 标量·g。
static void test_comp_gravity_goes_through_A() {
    TEST(comp_gravity_goes_through_A);
    ForceCompensation::init();
    double A[9] = { 0, 0, 1,
                    0, 1, 0,
                    1, 0, 0 };
    double com[3] = {0, 0, 0};
    double bF[3] = {0, 0, 0}, bM[3] = {0, 0, 0};
    ForceCompensation::setCalibration(A, bF, bM, com);
    CHECK(ForceCompensation::isCalibrated());        // 非退化 -> 收下了

    AppState::ForceData fd = gateVisibleFrame();
    double pose[6] = {0, 0, 0, 0, 0, 0};
    // pose 0: g = (0,0,9.81) -> Fg = A·g = (9.81, 0, 0)。读数正是它 -> 补偿到 0。
    fd.sixForceRaw[0] = 9.81; fd.sixForceRaw[1] = 0.0; fd.sixForceRaw[2] = 0.0;
    fd.sixForceRaw[3] = 0.0;  fd.sixForceRaw[4] = 0.0; fd.sixForceRaw[5] = 0.0;
    ForceCompensation::step(fd, pose);
    CHECK(fabs(fd.compensated[0]) < 1e-9);
    CHECK(fabs(fd.compensated[1]) < 1e-9);
    CHECK(fabs(fd.compensated[2]) < 1e-9);

    // 绕 z 转任意角: g 沿基座 z, Rz 不动它 -> 补偿结果必须一模一样。
    // (这一条同时钉住"ψ 不在补偿路径里": 若还有人拿模块态 ψ 去转重力, Rz 一改结果就变。)
    double pose2[6] = {0, 0, 0, 0, 0, 137.0};
    AppState::ForceData fd2 = gateVisibleFrame();
    fd2.sixForceRaw[0] = 9.81;
    ForceCompensation::step(fd2, pose2);
    CHECK(fabs(fd2.compensated[0]) < 1e-9);

    // 绕 y 转 90°: 期望值【由共享的重力函数现算】, 不在这里另写一份约定
    // (重力的约定只能有一份实现 —— 测试里再展开一遍正是历史上安静解错的样子)。
    double pose3[6] = {0, 0, 0, 0, 90.0, 0};
    double g[3];
    TcpCalibration::gravitySensorFrameAtYaw(pose3, 0.0, g);
    CHECK(fabs(g[2]) < 1e-9);          // Ry(90) 把 g 转到水平面内 -> g_z = 0
    AppState::ForceData fd3 = gateVisibleFrame();
    // 把 Fg = A·g 全三个分量都填上 = 一个"模型完全对"的读数, 于是补偿结果必须是 0。
    fd3.sixForceRaw[0] = g[2];         // Fg_x = A(0,:)·g = g_z
    fd3.sixForceRaw[1] = g[1];
    fd3.sixForceRaw[2] = g[0];
    ForceCompensation::step(fd3, pose3);
    CHECK(fabs(fd3.compensated[0]) < 1e-9);
    CHECK(fabs(fd3.compensated[1]) < 1e-9);
    CHECK(fabs(fd3.compensated[2]) < 1e-9);
    PASS();
}

// 力矩: Mg = c_s × (A·g) —— 与 Fg 【同一个 w = A·g】, 不是独立的 3×3, 也不是 dp×g。
static void test_comp_moment_is_cross_of_Ag() {
    TEST(comp_moment_is_cross_of_Ag);
    ForceCompensation::init();
    double A[9]; diagA(1.0, A);        // w = A·g = g
    double bF[3] = {0, 0, 0}, bM[3] = {0, 0, 0};
    double pose[6] = {0, 0, 0, 0, 0, 0};   // g = (0, 0, 9.81) -> w = (0, 0, 9.81)

    // (a) c_s 沿 z (与 w 平行): 叉乘为 0 -> 力矩补偿量恒 0。读数原样留到 compensated。
    double cS_z[3] = {0.0, 0.0, 0.05};     // 单位【米】= 50 mm
    ForceCompensation::setCalibration(A, bF, bM, cS_z);
    AppState::ForceData fd = gateVisibleFrame();
    fd.sixForceRaw[4] = 0.4905;
    // 一致性闸门要求【参考量】与本地模型给的是同一个数 —— 这一帧给它们相等,
    // 于是闸门放行, 下面量的才是【补偿结果】本身, 而不是被闸门置零的 0。
    // (喂到参考量那一侧: fd.tcpForce —— 见本节顶上的说明。)
    fd.tcpForce[4] = 0.4905;
    ForceCompensation::step(fd, pose);
    CHECK(fabs(fd.compensated[4] - 0.4905) < 1e-9);

    // (b) c_s 沿 x: c_s × w = (0.05,0,0) × (0,0,9.81) = (0, −0.4905, 0)
    //     读数正好是这个力矩 -> 被减干净。这一条同时钉住【叉乘的顺序/符号】:
    //     写成 w × c_s 会得到 +0.4905, 于是补偿后的读数变成 −0.981 而不是 0。
    double cS_x[3] = {0.05, 0.0, 0.0};
    ForceCompensation::setCalibration(A, bF, bM, cS_x);
    AppState::ForceData fd2 = gateVisibleFrame();
    fd2.sixForceRaw[4] = -0.4905;
    ForceCompensation::step(fd2, pose);
    CHECK(fabs(fd2.compensated[4]) < 1e-9);
    CHECK(fabs(fd2.compensated[3]) < 1e-9);
    CHECK(fabs(fd2.compensated[5]) < 1e-9);
    PASS();
}

// ===== 运行时一致性闸门 (2026-09-19, 用户指令 1/2/3) =====
//
// 判据: 本地全量模型的输出 compensated 与机械臂自报的【参考量】逐通道比较。
// 两个模型都对时它们估计的是同一个量 (外力), 所以应当一致; 不一致 ⇒ 至少一个错 ⇒
// compensated 全 6 个分量置零 + 报错。未标定/模型不可用 ⇒ 同一处置, 但报的是另一个原因。
// ⚠ 【参考量是哪一路】不在这份测试里记: 它由 ForceCompensation.cpp 的 guardReferenceValue
//   一处定义。下面凡是要"让闸门放行"或"让闸门拒绝"的地方, 喂的都是【参考量那一侧】
//   (fd.tcpForce), 而 fd.raw (@576) 是与它并排报出的诊断侧 —— 放错边会得到一个【空洞
//   的】用例: 它仍然绿, 但测的不再是它名字说的那件事。

// 判据参考量必须是机械臂【通过关节电流算】的那一路 (@720)，不是传感器侧 (@576)。
// 构造: 让 @720 与本地一致, 而 @576 严重不符 -> 必须【放行】。
static void test_guard_reference_is_the_current_derived_channel() {
    TEST(guard_reference_is_the_current_derived_channel);
    ForceCompensation::init();
    double A[9]; diagA(1.0, A);
    double com[3] = {0, 0, 0};
    double bF[3] = {0, 0, 0}, bM[3] = {0, 0, 0};
    ForceCompensation::setCalibration(A, bF, bM, com);

    double pose[6] = {0, 0, 0, 0, 0, 0};
    AppState::ForceData fd = gateVisibleFrame();
    fd.sixForceRaw[2] = 9.81;   // 本地算出 compensated = (0,0,0)
    fd.tcpForce[0]    = 0.05;   // @720 说 x 上几乎没外力      <- 判据该看这个
    // @576 说 x 上有 2.0 N                                <- 不该再看这个
    // ⚠ 这个数必须【大于力通道容差】, 否则这条用例会退化成恒真: 容差 0.50 N 时 0.90 N 就够,
    //   而容差 2026-09-21 上调到 1.2464 N 之后, 0.90 N 无论判据看哪一路都放行 —— 那样
    //   "判据看的是哪一路"这件事就【没有被测到】(两条分支给同样的结果)。⇒ 取 2.0 N。
    fd.raw[0]         = 2.0;
    // ⚠ 上面那个 ⚠ 说的"必须大于容差"这件事【在这里被机器检查】: 容差若哪天被抬到 2.0 N
    //   以上, 这一条会先红, 而不是让整条用例安静地退化成恒真 (绿着什么都没测)。
    CHECK(2.0 > Config::FORCE_GUARD_TOL_FORCE_N);
    ForceCompensation::step(fd, pose);

    CHECK(ForceCompensation::guardState() == ForceCompensation::GuardState::OK);
    PASS();
}

// 一致 -> 放行: compensated 就是模型算出来的值 (不被置零), 状态 OK。
static void test_guard_passes_when_consistent() {
    TEST(guard_passes_when_consistent);
    ForceCompensation::init();
    double A[9]; diagA(1.0, A);
    double com[3] = {0, 0, 0};
    double bF[3] = {0, 0, 0}, bM[3] = {0, 0, 0};
    ForceCompensation::setCalibration(A, bF, bM, com);

    double pose[6] = {0, 0, 0, 0, 0, 0};       // g = (0,0,9.81) -> Fg = (0,0,9.81)
    AppState::ForceData fd = gateVisibleFrame();
    fd.sixForceRaw[2] = 9.81;                  // 读数 = 重力 -> compensated 应为 0
    fd.tcpForce[2] = 0.0;                      // 参考量也说是 0 (两边一致)
    fd.sixForceRaw[0] = 1.25;                  // 再叠一个真实外力: 两边都必须看到它
    fd.tcpForce[0] = 1.25;
    ForceCompensation::step(fd, pose);

    CHECK(ForceCompensation::guardState() == ForceCompensation::GuardState::OK);
    CHECK(fd.isCalibrated == true);
    CHECK(fabs(fd.compensated[0] - 1.25) < 1e-9);   // 数据真的过去了, 不是被置零
    CHECK(fabs(fd.compensated[2]) < 1e-9);
    for (int i = 3; i < 6; i++) CHECK(fabs(fd.compensated[i]) < 1e-9);
    PASS();
}

// 不一致 -> 拒绝: compensated 全 6 个分量为 0, 且报的是 INCONSISTENT (不是 UNCALIBRATED)。
static void test_guard_refuses_when_inconsistent() {
    TEST(guard_refuses_when_inconsistent);
    ForceCompensation::init();
    double A[9]; diagA(1.0, A);
    double com[3] = {0, 0, 0};
    double bF[3] = {0, 0, 0}, bM[3] = {0, 0, 0};
    ForceCompensation::setCalibration(A, bF, bM, com);

    double pose[6] = {0, 0, 0, 0, 0, 0};
    AppState::ForceData fd = gateVisibleFrame();
    fd.sixForceRaw[2] = 9.81;                  // 本地算出 compensated = (0,0,0)
    // 参考量说 x 上有 2.0 N 的外力, 而本地算出 0 ⇒ 差 2.0 N。
    // ⚠ 这个数【跟着容差上调过】(2026-09-21): 力通道容差从 0.50 N 换成 1.2464 N (依据见
    //   Config.h), 原来的 0.90 N 已经落在新容差【之内】—— 那一版这里会放行, 于是这条用例
    //   会因为容差变大而假红。取 2.0 N (= 新容差的 1.6 倍) 而不是"刚好越过": 让它明显
    //   超过, 免得下次微调容差时又变成一条卡在边界上的用例。它钉的是【拒绝这条路走不走得通】,
    //   不是容差的数值 —— 容差的数值由 test_runtime_consistency_guard_replay 那半边守。
    fd.tcpForce[0] = 2.0;
    fd.tcpForce[1] = 0.05;                     // y 在限内
    ForceCompensation::step(fd, pose);

    CHECK(ForceCompensation::guardState() == ForceCompensation::GuardState::INCONSISTENT);
    for (int i = 0; i < 6; i++) CHECK(fabs(fd.compensated[i]) < 1e-12);   // 一个分量都不许漏
    CHECK(fd.isCalibrated == false);
    CHECK(fd.calibMassKg == 0.0);
    // 逐通道的结果要能被读出来 —— 上层 (RelayCore) 拿它去报 RobotDiagnostics。
    ForceCompensation::GuardReport rep;
    ForceCompensation::guardReport(rep);
    CHECK(rep.state == ForceCompensation::GuardState::INCONSISTENT);
    CHECK(fabs(rep.ema[0] - (-2.0)) < 1e-9);
    CHECK(rep.exceeded[0] == true);
    CHECK(rep.exceeded[1] == false);
    CHECK(rep.voted[0] == true);
    CHECK(rep.tol[0] == Config::FORCE_GUARD_TOL_FORCE_N);
    CHECK(rep.tol[3] == Config::FORCE_GUARD_TOL_MOMENT_NM);
    PASS();
}

// ★ 拒绝的原因必须分得开 (简报硬要求): 处置一样 (都拒绝), 但操作员要做的事不同 ——
//   "去标定"、"去查负载参数有没有发进去"、"去查参考量这一路为什么没有数据" 是三件事。
//   合成一句话就会让人乱猜。
//   ⚠ 用例名里的 "two" 是历史 (2026-09-19 那版只有两种原因), 2026-09-21 Task 7 起是三种。
//   本用例【三种都钉】; (甲) 与 (乙) 是三件里的前两件, (丙) 是第三件。
static void test_guard_two_causes_are_distinguishable() {
    TEST(guard_two_causes_are_distinguishable);

    // (甲) 没有可用模型
    ForceCompensation::init();
    AppState::ForceData fd = gateVisibleFrame();
    double pose[6] = {0, 0, 0, 0, 0, 0};
    ForceCompensation::step(fd, pose);
    const ForceCompensation::GuardState stA = ForceCompensation::guardState();

    // (乙) 有模型但对不上
    double A[9]; diagA(1.0, A);
    double com[3] = {0, 0, 0};
    double bF[3] = {0, 0, 0}, bM[3] = {0, 0, 0};
    ForceCompensation::setCalibration(A, bF, bM, com);
    AppState::ForceData fd2 = gateVisibleFrame();
    fd2.tcpForce[0] = 5.0;   // 参考量那一侧严重不符 -> INCONSISTENT
    ForceCompensation::step(fd2, pose);
    const ForceCompensation::GuardState stB = ForceCompensation::guardState();

    // (丙) 有模型, 但参考量这一路没有数据 (Task 7)
    AppState::ForceData fd3 = gateVisibleFrame();
    fd3.sixForceOnline = 0;      // 机械臂自报六维力【不】在线 -> 参考量不可用
    fd3.tcpForce[0] = 5.0;      // ★ 就算参考量【像是】有个大偏差, 也不许报成"对不上"
    ForceCompensation::step(fd3, pose);
    const ForceCompensation::GuardState stC = ForceCompensation::guardState();

    CHECK(stA == ForceCompensation::GuardState::UNCALIBRATED);
    CHECK(stB == ForceCompensation::GuardState::INCONSISTENT);
    CHECK(stC == ForceCompensation::GuardState::REFERENCE_UNAVAILABLE);
    CHECK(stA != stB);
    CHECK(stA != stC);
    CHECK(stB != stC);
    // 名字也要分得开 —— 日志里落的是名字。
    CHECK(std::string(ForceCompensation::guardStateName(stA)) == "UNCALIBRATED");
    CHECK(std::string(ForceCompensation::guardStateName(stB)) == "INCONSISTENT");
    CHECK(std::string(ForceCompensation::guardStateName(stC)) == "REFERENCE_UNAVAILABLE");
    PASS();
}

// ★★★ Task 7 (2026-09-21, 用户指令追加): 参考量【不可用】时 fail-closed, 且必须与
//   "不一致" 分得开。本用例【同时钉住这两件事】。
//
// 要关的那个洞: 判据是 compensated − 参考量。参考量读到 ~0 时判据退化成"本地输出是否在
//   自己的容差内" —— 而按构造它总是在 ⇒ 闸门在【一个不携带信息的通道上放行】。
//   "零"既可能是"真的没有外力", 也可能是"这一路没有数据 / 已失效", 两者从前【不可区分】。
//
// ⚠ 【这是防紧, 不是修一个正在发生的 bug】: 生产链路上 RelayCore 在【同一帧、同一把锁】里
//   一起填 raw[] / tcpForce[] / sixForceRaw[] / sixForceOnline ⇒ "通道其实有数但读数为零"
//   在【实机目前不可达】。它只在【回放 / 夹具】路径出现 (四份夹具没有参考量那一路的列,
//   见 test_payload_calibration 的 runtime_consistency_guard_replay)。
//
// 怎么在没有机械臂的情况下驱动 —— 依据只用【既有信号】, 不新造门限:
//   · fd.sixForceOnline (30004 帧 @1037): 实机实测值 1 = 在线; 初值 −1 = 一帧都没收到;
//   · fd.isStale: 既有超时常量 Config::FORCE_STALE_MS 的落点 (RelayCore::pollForce 里用
//     lastUpdateMs 算出这个标志, 就在 step() 之前、同一把锁内 —— 所以这里是"越过那个
//     窗口"的等价驱动方式)。
static void test_guard_reference_unavailable_does_not_pass() {
    TEST(guard_reference_unavailable_does_not_pass);
    double pose[6] = {0, 0, 0, 0, 0, 0};

    // ---- (甲) 参考量【不可用】, 而"判据差"按构造是 0 —— 就是那个退化的放行场景 ----
    struct Case { const char* why; int online; bool stale; };
    const Case cases[3] = {
        { "sixForceOnline=0 (机械臂自报不在线)", 0,  false },
        { "sixForceOnline=-1 (一帧都没收到)",   -1, false },
        { "越过 FORCE_STALE_MS (帧已陈旧)",      1,  true  },
    };
    for (int k = 0; k < 3; k++) {
        ForceCompensation::init();
        double A[9]; diagA(1.0, A);
        double com[3] = {0, 0, 0};
        double bF[3] = {0, 0, 0}, bM[3] = {0, 0, 0};
        ForceCompensation::setCalibration(A, bF, bM, com);

        AppState::ForceData fd = gateVisibleFrame();
        fd.isStale        = cases[k].stale;
        fd.sixForceOnline = cases[k].online;
        fd.sixForceRaw[2] = 9.81;   // 本地算出 compensated = (0,0,0)
        fd.tcpForce[0]    = 0.0;    // 参考量: 零 —— 但这一路【没有数据】
        ForceCompensation::step(fd, pose);

        const ForceCompensation::GuardState st = ForceCompensation::guardState();
        const std::string nm(ForceCompensation::guardStateName(st));
        if (st == ForceCompensation::GuardState::OK) {
            std::cout << std::endl << "    FAIL (" << cases[k].why << "): 参考量不可用"
                      << "【竟然放行了】—— 判据退化成了\"本地输出是否在自己的容差内\","
                      << " 那个比较不携带任何信息。" << std::endl;
            g_failed++;
            return;
        }
        // ① 状态本身: 必须是那个【独立】的状态, 不许是"不一致", 也不许是"没有模型"。
        if (nm != "REFERENCE_UNAVAILABLE") {
            std::cout << std::endl << "    FAIL (" << cases[k].why << "): 状态是 " << nm
                      << " —— 参考量不可用必须有自己的状态名 (日志里落的是名字)。"
                      << std::endl;
            g_failed++;
            return;
        }
        if (st == ForceCompensation::GuardState::INCONSISTENT ||
            st == ForceCompensation::GuardState::UNCALIBRATED) {
            std::cout << std::endl << "    FAIL (" << cases[k].why << "): \"不可用\"与"
                      << "\"不一致/没有模型\"混成了同一个状态。" << std::endl;
            g_failed++;
            return;
        }
        // ② 拒绝的落地: compensated 全 6 个分量置零 + 下游不许被标成"已标定"。
        for (int i = 0; i < 6; i++) CHECK(fabs(fd.compensated[i]) < 1e-12);
        CHECK(fd.isCalibrated == false);
        // ③ 错误码对得上这个状态 (报错是给操作员看的"该做什么")。
        CHECK(std::string(errorCodeName(ForceCompensation::guardErrorCode(st))) ==
              "ERR_FORCE_REFERENCE_UNAVAILABLE");
        CHECK(ForceCompensation::guardErrorCode(st) != RobotErrorCode::ERR_FORCE_INCONSISTENT);
        CHECK(ForceCompensation::guardErrorCode(st) != RobotErrorCode::ERR_FORCE_UNCALIBRATED);
        CHECK(getSeverity(ForceCompensation::guardErrorCode(st)) == Severity::REJECT);
    }

    // ---- (乙) 正面对照: 参考量【可用】且一致 -> 必须放行 ----
    // 没有这一半, "一律拒绝"的桩也能让上面全绿 —— 那时上面钉的就不是"可用性"了。
    {
        ForceCompensation::init();
        double A[9]; diagA(1.0, A);
        double com[3] = {0, 0, 0};
        double bF[3] = {0, 0, 0}, bM[3] = {0, 0, 0};
        ForceCompensation::setCalibration(A, bF, bM, com);

        AppState::ForceData fd = gateVisibleFrame();
        fd.isStale = false;             // 帧新鲜
        fd.sixForceOnline = 1;          // 机械臂自报在线 (实机实测值就是 1)
        fd.sixForceRaw[2] = 9.81;       // compensated = (0,0,0)
        fd.tcpForce[0] = 1.25;          // 参考量真的读到 1.25 N, 本地也看到 1.25 N
        fd.sixForceRaw[0] = 1.25;
        ForceCompensation::step(fd, pose);

        CHECK(ForceCompensation::guardState() == ForceCompensation::GuardState::OK);
        CHECK(fabs(fd.compensated[0] - 1.25) < 1e-9);   // 数据真的过去了
    }

    // ---- (丙) 反面对照: 参考量【可用】但两边对不上 -> INCONSISTENT, 且与 (甲) 分得开 ----
    // 这一半钉的是"可区分": 同一个 0 读数, 有数据时是"不一致", 没数据时是"不可用"。
    {
        ForceCompensation::init();
        double A[9]; diagA(1.0, A);
        double com[3] = {0, 0, 0};
        double bF[3] = {0, 0, 0}, bM[3] = {0, 0, 0};
        ForceCompensation::setCalibration(A, bF, bM, com);

        AppState::ForceData fd = gateVisibleFrame();
        fd.isStale = false;
        fd.sixForceOnline = 1;
        fd.sixForceRaw[2] = 9.81;       // 本地算出 0
        fd.tcpForce[0] = 2.0;           // 参考量读到 2.0 N (远超力通道容差) -> 对不上
        ForceCompensation::step(fd, pose);

        const ForceCompensation::GuardState stInc = ForceCompensation::guardState();
        CHECK(stInc == ForceCompensation::GuardState::INCONSISTENT);
        CHECK(std::string(ForceCompensation::guardStateName(stInc)) !=
              std::string("REFERENCE_UNAVAILABLE"));
        CHECK(ForceCompensation::guardErrorCode(stInc) !=
              ForceCompensation::guardErrorCode(ForceCompensation::GuardState::REFERENCE_UNAVAILABLE));
    }
    PASS();
}

// ===== stderr 捕获窗口 (2026-09-21 复审, 服务下面那条用例) =====
// 同源做法见 test_payload_calibration 的 mgMuteStderr (把 fd 2 接走, 跑完接回来), 区别只在
// 这里接的是【临时文件】而不是 NUL —— 要读回来断言。
// ⚠ 【窗口里不许出现 CHECK】: CHECK 失败会 return, 那时 fd 2 还指着临时文件, 后面所有用例的
//   输出都会丢 (包括那条"结果计数"行)。所以窗口只包住一次 step() 调用, 断言全部在窗口之外。
static int g_capSavedFd = -1;

static bool capBegin(const char* path) {
    fflush(stderr);
    g_capSavedFd = _dup(2);
    if (g_capSavedFd < 0) return false;
    const int f = _open(path, _O_CREAT | _O_WRONLY | _O_TRUNC | _O_BINARY,
                        _S_IREAD | _S_IWRITE);
    if (f < 0) { _close(g_capSavedFd); g_capSavedFd = -1; return false; }
    _dup2(f, 2);
    _close(f);
    return true;
}

static void capEnd() {
    fflush(stderr);
    if (g_capSavedFd >= 0) {
        _dup2(g_capSavedFd, 2);
        _close(g_capSavedFd);
        g_capSavedFd = -1;
    }
}

static std::string capRead(const char* path) {
    std::string s;
    FILE* f = fopen(path, "rb");
    if (!f) return s;
    char buf[4096];
    size_t n = 0;
    while ((n = fread(buf, 1, sizeof(buf), f)) > 0) s.append(buf, n);
    fclose(f);
    return s;
}

// "这一段是整块"的判据 —— 整块独有的抬头句。两条用例都用它。
// ⚠ 它是生产代码 (ForceCompensation.cpp 的 setGuardState) 里那一句的副本: 那边改了措辞,
//   这里就找不到 -> 用例【响亮地红】。这与"捕获里没有某某字面量"那种写法【方向相反】——
//   后者的判据一旦被改名就【恒真】(见下面 ③ 那条注释里的账)。
static const char* kGuardBlockHeader = "============ 一致性闸门: 拒绝传递数据 ============";

// 跑一帧并把这一次的 stderr 收回来。下面那条用例要抓五次, 所以提成助手。
// ⚠ 窗口里【不许】出现 CHECK (见上面那条规矩): 本助手只回答"窗口有没有搭起来", 断言全在
//   调用处 —— 失败时 return 也只会发生在窗口之外。
static bool capStepForPrint(const char* path, AppState::ForceData& fd, const double pose[6],
                            std::string& out) {
    if (!capBegin(path)) return false;
    ForceCompensation::step(fd, pose);
    capEnd();
    out = capRead(path);
    remove(path);
    return true;
}

// ★★ 2026-09-21 复审 (Important 1): 参考量不可用【第一次】被拒绝时, 那一段打印必须把
//   两个可用性读数【当场说出来】。上面那条用例钉的是状态与错误码, 钉不住这一段文字。
//
// 为什么非要有这条: 那两个数从前【只】出现在 5 s 复报那一行里, 而那一行没有用例覆盖
//   (见任务报告的遗留清单); 于是最常见的第一次拒绝里, "根本没有帧"与"帧到了、但机械臂
//   自报不在线"在操作员眼里【分不开】—— 而这两件事要做的处置不同。这条用例就是钉住
//   "第一眼能拿到诊断": 三个 (online, stale) 组合各自印出【自己的】值。
//
// ⚠ 断言故意按子串找那两个数 (与复报行共用的写法 "@1037 = %d" / "帧陈旧 = %d"), 而不是逐字
//   比对整段: 整段比对会在任何一句措辞调整时红掉 —— 那时它测的是措辞不是诊断能力。这两个
//   子串是"值有没有被印出来"的最小证据, 也是复审点名的那件事。
static void test_guard_unavailable_first_refusal_prints_the_two_values() {
    TEST(guard_unavailable_first_refusal_prints_the_two_values);
    const char* capPath = "gate_print_capture.tmp";
    double pose[6] = {0, 0, 0, 0, 0, 0};

    struct Case { const char* why; int online; bool stale; };
    const Case cases[3] = {
        { "sixForceOnline=0 (机械臂自报不在线)", 0,  false },
        { "sixForceOnline=-1 (一帧都没收到)",   -1, false },
        { "越过 FORCE_STALE_MS (帧已陈旧)",      1,  true  },
    };
    for (int k = 0; k < 3; k++) {
        ForceCompensation::init();
        double A[9]; diagA(1.0, A);
        double com[3] = {0, 0, 0};
        double bF[3] = {0, 0, 0}, bM[3] = {0, 0, 0};
        ForceCompensation::setCalibration(A, bF, bM, com);

        AppState::ForceData fd = gateVisibleFrame();
        fd.isStale        = cases[k].stale;
        fd.sixForceOnline = cases[k].online;
        fd.sixForceRaw[2] = 9.81;   // 本地算出 compensated = (0,0,0)
        fd.tcpForce[0]    = 0.0;    // 参考量: 零 —— 但这一路【没有数据】

        if (!capBegin(capPath)) {
            // 窗口没搭起来 (capBegin 失败时自己还原过 fd 2) —— 这里说实话: 断言没做成。
            std::cout << std::endl << "    FAIL (捕获窗口没搭起来): 这条打印没有被钉住。"
                      << std::endl;
            g_failed++;
            return;
        }
        // ↓↓ 窗口: 只有这一句 (上面说过窗口里不许 CHECK)
        ForceCompensation::step(fd, pose);   // 状态跃迁 -> 第一次拒绝那一段
        // ↑↑ 窗口到此为止
        capEnd();
        const std::string txt = capRead(capPath);
        remove(capPath);

        // ① 这一段确实是"参考量不可用"那一次跃迁的打印, 不是别的状态的。
        if (txt.find("参考量不可用") == std::string::npos) {
            std::cout << std::endl << "    FAIL (" << cases[k].why << "): 第一次拒绝那段里"
                      << "没有说到【参考量不可用】。" << std::endl;
            g_failed++;
            return;
        }
        // ② ★ 本帧这两个读数在【跃迁这一次】打印里就有了 —— 本条用例存在的唯一理由。
        char wantOnline[64], wantStale[64];
        snprintf(wantOnline, sizeof(wantOnline), "@1037 = %d", cases[k].online);
        snprintf(wantStale,  sizeof(wantStale),  "帧陈旧 = %d", cases[k].stale ? 1 : 0);
        if (txt.find(wantOnline) == std::string::npos) {
            std::cout << std::endl << "    FAIL (" << cases[k].why << "): 第一次拒绝那段里"
                      << "没有印出六维力在线状态 (找不到 \"" << wantOnline
                      << "\") —— 这一路没有数据的两种来源在操作员眼里分不开。" << std::endl;
            g_failed++;
            return;
        }
        if (txt.find(wantStale) == std::string::npos) {
            std::cout << std::endl << "    FAIL (" << cases[k].why << "): 第一次拒绝那段里"
                      << "没有印出帧陈旧 (找不到 \"" << wantStale << "\")。" << std::endl;
            g_failed++;
            return;
        }
        // ③ 这两个数必须来自【跃迁】那一段, 不是 5 s 复报那一行。只喂了一帧、状态刚变,
        //    所以捕获窗口里必须是【整块】—— 是复报行就说明值是从那条没覆盖的路径来的。
        //    ⚠ 判据【换了写法】(2026-09-21 收口 Fix 2): 原来写的是"捕获里没有字面量
        //      `(复报)`" —— 而本波把那处文字改成了变量 (一行式的抬头按跃迁/复报分岔),
        //      那个字面量于是【只活在这个文件里】: 这条断言从此【恒真】, 查什么都能过。
        //      ⇒ 改成【正着查"整块在不在"】: 整块那一段有它自己的抬头句 (kGuardBlockHeader),
        //        节流复报那一行【从不】带它。整块一旦不再打, 这里就红 (见 kGuardBlockHeader
        //        处关于"正查/反查"的说明)。
        if (txt.find(kGuardBlockHeader) == std::string::npos) {
            std::cout << std::endl << "    FAIL (" << cases[k].why << "): 第一次拒绝那一段里"
                      << "找不到整块的抬头句 —— 捕获到的不是状态跃迁那一段, 那么上面两个数"
                      << "仍然只出现在别处的打印里。" << std::endl;
            g_failed++;
            return;
        }
    }
    PASS();
}

// ★★ 状态 -> 错误码的映射 (2026-09-19 复审 Important 4)。
//   在此之前 RelayCore 自己拿 static_cast<int>(guardState()) 去比字面量 1 和 2, 而这条
//   映射【没有任何测试】: 给 GuardState 换个顺序, ERR_FORCE_UNCALIBRATED 与
//   ERR_FORCE_INCONSISTENT 就悄悄对调, 而用户指令 1 的全部意义就是告诉操作员
//   【该去标定还是该去查负载参数】—— 报错报反了比不报还坏。
//   现在唯一的实现是 ForceCompensation::guardErrorCode, 本用例把它的【四个】输入逐条钉住
//   (2026-09-21 Task 7 加第四个), 并顺手钉住"错误码 -> 名字 -> 严重度"这条下游链
//   (报告里说的"严重度现在是咨询性的"也在这条链上: 这里只断言它是 REJECT, 断言不了
//   它有没有被消费)。
static void test_guard_error_code_mapping() {
    TEST(guard_error_code_mapping);

    using ForceCompensation::GuardState;
    const RobotErrorCode cOk   = ForceCompensation::guardErrorCode(GuardState::OK);
    const RobotErrorCode cUnc  = ForceCompensation::guardErrorCode(GuardState::UNCALIBRATED);
    const RobotErrorCode cInc  = ForceCompensation::guardErrorCode(GuardState::INCONSISTENT);
    // ★ 2026-09-21 (Task 7): 第四个状态。它的码【不是】复用 INCONSISTENT —— 参考量不可用时
    //   【没有比过】, 报成"对不上"就是让日志里出现一个没发生过的事实。
    const RobotErrorCode cRef =
        ForceCompensation::guardErrorCode(GuardState::REFERENCE_UNAVAILABLE);

    CHECK(cOk == RobotErrorCode::OK);                                  // 放行 -> 不上报
    CHECK(cUnc == RobotErrorCode::ERR_FORCE_UNCALIBRATED);
    CHECK(cInc == RobotErrorCode::ERR_FORCE_INCONSISTENT);
    CHECK(cRef == RobotErrorCode::ERR_FORCE_REFERENCE_UNAVAILABLE);
    // 三个码必须互不相同 —— 否则"三种原因分开报"这件事在日志里根本不成立。
    CHECK(cUnc != cInc);
    CHECK(cRef != cInc);
    CHECK(cRef != cUnc);
    // 与枚举的数值索引【无关】: 这几个码在 RobotErrorCode 里的位置本来就与 GuardState 不同,
    // 所以"按 static_cast<int> 对上"这种巧合不许再被依赖。
    CHECK(static_cast<int>(cUnc) != static_cast<int>(GuardState::UNCALIBRATED));
    CHECK(static_cast<int>(cInc) != static_cast<int>(GuardState::INCONSISTENT));
    CHECK(static_cast<int>(cRef) != static_cast<int>(GuardState::REFERENCE_UNAVAILABLE));

    // 名字 (进 robot_diagnostics.log 与 D| 帧的那一个) 也必须对得上, 且分得开。
    CHECK(std::string(errorCodeName(cUnc)) == "ERR_FORCE_UNCALIBRATED");
    CHECK(std::string(errorCodeName(cInc)) == "ERR_FORCE_INCONSISTENT");
    CHECK(std::string(errorCodeName(cRef)) == "ERR_FORCE_REFERENCE_UNAVAILABLE");

    // 严重度: 三个码都是 REJECT —— 而且三者一致 (给操作员看的档位不该因原因而不同)。
    CHECK(getSeverity(cUnc) == Severity::REJECT);
    CHECK(getSeverity(cInc) == Severity::REJECT);
    CHECK(getSeverity(cRef) == Severity::REJECT);
    CHECK(getSeverity(cUnc) == getSeverity(cInc));
    CHECK(getSeverity(cRef) == getSeverity(cInc));

    // 名字函数: 四个状态都要能读出来, 且互不相同。
    CHECK(std::string(ForceCompensation::guardStateName(GuardState::OK)) == "OK");
    CHECK(std::string(ForceCompensation::guardStateName(GuardState::UNCALIBRATED)) !=
          std::string(ForceCompensation::guardStateName(GuardState::INCONSISTENT)));
    CHECK(std::string(ForceCompensation::guardStateName(GuardState::REFERENCE_UNAVAILABLE)) ==
          "REFERENCE_UNAVAILABLE");
    CHECK(std::string(ForceCompensation::guardStateName(GuardState::REFERENCE_UNAVAILABLE)) !=
          std::string(ForceCompensation::guardStateName(GuardState::INCONSISTENT)));
    CHECK(std::string(ForceCompensation::guardStateName(GuardState::REFERENCE_UNAVAILABLE)) !=
          std::string(ForceCompensation::guardStateName(GuardState::UNCALIBRATED)));
    CHECK(std::string(ForceCompensation::guardStateName(GuardState::REFERENCE_UNAVAILABLE)) !=
          std::string(ForceCompensation::guardStateName(GuardState::OK)));
    PASS();
}

// ★ Fz 【不投票】但【照报】—— 这条取舍的依据写在 ForceCompensation.cpp 的 g_guardVote 段
//   (它是【参考量还是 @576 的时候】定下的; 换成新参考量之后那条依据没有跟着复测, 掩码
//   本身的取舍不在本次改动内)。本用例钉住两件事: (a) 巨大 z 差不会让闸门拒绝
//   (不然就是"永远不通过"); (b) 它的比较结果仍然读得出来 (不然就是"静默"跳过, 明令禁止)。
//   ⚠ 所以 z 差【必须】喂到参考量那一侧: 喂到 @576 (诊断侧) 它就退化成一条恒真的空用例
//     —— 无论判据看哪一路都绿, 却什么都没钉住。
static void test_guard_fz_reported_but_not_voted() {
    TEST(guard_fz_reported_but_not_voted);
    ForceCompensation::init();
    double A[9]; diagA(1.0, A);
    double com[3] = {0, 0, 0};
    double bF[3] = {0, 0, 0}, bM[3] = {0, 0, 0};
    ForceCompensation::setCalibration(A, bF, bM, com);

    double pose[6] = {0, 0, 0, 0, 0, 0};
    AppState::ForceData fd = gateVisibleFrame();
    fd.sixForceRaw[2] = 9.81;                 // compensated = (0,0,0)
    fd.tcpForce[2] = 3.0;                     // 参考量的 z 报 3 N —— 远超力通道容差 1.2464
    ForceCompensation::step(fd, pose);

    CHECK(ForceCompensation::guardState() == ForceCompensation::GuardState::OK);  // (a)
    ForceCompensation::GuardReport rep;
    ForceCompensation::guardReport(rep);
    CHECK(rep.voted[0] == true && rep.voted[1] == true);
    CHECK(rep.voted[2] == false);                                             // Fz 不投票
    // ⚠ 力矩三个通道【也不投票】(2026-09-21, 理由见 .cpp 的 g_guardVote 段) —— 这一条与
    //   下面两条一起把【生效掩码】钉成 Fx/Fy 两个 true、其余四个 false。名字里只说 Fz 是
    //   因为本用例喂的是 z 那一侧的差; 掩码本身的完整形状由这三条 CHECK 负责。
    CHECK(rep.voted[3] == false && rep.voted[4] == false && rep.voted[5] == false);
    CHECK(fabs(rep.ema[2] - (-3.0)) < 1e-9);                                  // (b) 照报
    CHECK(rep.exceeded[2] == false);          // 不投票的通道不算"超限"
    PASS();
}

// ★ 力矩通道【不投票】(2026-09-21): 力矩严重不符也【必须放行】—— 但那一半仍要照实报出来。
//   理由与实测出处写在 ForceCompensation.cpp 的 g_guardVote 段 (力矩与参考量之间不存在
//   "一致"态: 约 90% 是姿态无关、且在漂的偏置)。本用例钉住两件事:
//     (a) 力矩差得再多也不拦 (否则闸门永远拒绝 -> 判决是全或无 -> 力通道一起断);
//     (b) 那一半仍然被算出来、被报出来 (「不投票」不等于「不检查、不显示」)。
//   ⚠ 所以力矩差【必须】喂到参考量那一侧 (fd.tcpForce): 喂到 fd.raw (@576) 那一侧它就
//     退化成一条恒真的空用例 —— 无论掩码怎么变都绿, 却什么都没钉住。
static void test_guard_moment_channel_reports_but_does_not_vote() {
    TEST(guard_moment_channel_reports_but_does_not_vote);
    ForceCompensation::init();
    double A[9]; diagA(1.0, A);
    double com[3] = {0, 0, 0};
    double bF[3] = {0, 0, 0}, bM[3] = {0, 0, 0};
    ForceCompensation::setCalibration(A, bF, bM, com);

    double pose[6] = {0, 0, 0, 0, 0, 0};
    AppState::ForceData fd = gateVisibleFrame();
    fd.sixForceRaw[2] = 9.81;      // 本地算出 compensated_z = 0 (重力被减掉)
    fd.sixForceRaw[0] = 1.05;      // 一个【真实】外力 —— 用来证明力通道的数据真的出门了
    fd.tcpForce[0] = 1.10;         // 力: 参考量只差 0.05, 在限内 (< 力通道容差)
    fd.tcpForce[1] = 0.05;
    fd.tcpForce[3] = 1.20;         // 力矩: 远超声明的力矩容差 (1.20 >> 0.03)
    // ⚠ "在限内 / 超限"这两个说法【在这里被机器检查】: 容差若被抬到 0.05 以上 (或力矩容差
    //   被抬到 1.20 以上), 本条用例会先红, 而不是安静地退化成"两个方向都放行"的空壳。
    CHECK(0.05 < Config::FORCE_GUARD_TOL_FORCE_N);
    CHECK(1.20 > Config::FORCE_GUARD_TOL_MOMENT_NM);
    ForceCompensation::step(fd, pose);

    // (a) 不投票 -> 放行, 且【力通道的数据真的过去了】(被拒的话 compensated 是全 0,
    //     所以这一条能分辨"放行"与"拒绝": 拒绝时它必定是 0)。
    CHECK(ForceCompensation::guardState() == ForceCompensation::GuardState::OK);
    CHECK(fabs(fd.compensated[0] - 1.05) < 1e-9);
    // (b) 但必须照实报 —— "不投票"不等于"不检查、不显示"
    ForceCompensation::GuardReport rep;
    ForceCompensation::guardReport(rep);
    CHECK(rep.voted[3] == false);
    CHECK(fabs(rep.ema[0] - (-0.05)) < 1e-9);   // 力通道也在照实报
    CHECK(fabs(rep.ema[3] - (-1.20)) < 1e-9);   // 力矩那一半照样算出来、报出来
    CHECK(rep.exceeded[3] == false);            // 不投票的通道不算"超限"
    PASS();
}

// 力通道必须【仍然】投票 —— 防止掩码被改过头 (全 false 就再也没有闸门了)。
static void test_guard_force_channels_still_vote() {
    TEST(guard_force_channels_still_vote);
    ForceCompensation::init();
    double A[9]; diagA(1.0, A);
    double com[3] = {0, 0, 0};
    double bF[3] = {0, 0, 0}, bM[3] = {0, 0, 0};
    ForceCompensation::setCalibration(A, bF, bM, com);

    double pose[6] = {0, 0, 0, 0, 0, 0};
    AppState::ForceData fd = gateVisibleFrame();
    fd.sixForceRaw[2] = 9.81;
    fd.tcpForce[0] = 5.00;         // 力严重不符 (5.00 > 力通道容差)
    CHECK(5.00 > Config::FORCE_GUARD_TOL_FORCE_N);
    ForceCompensation::step(fd, pose);

    CHECK(ForceCompensation::guardState() == ForceCompensation::GuardState::INCONSISTENT);
    PASS();
}

// ★ 非有限值【先判, 再问投不投票】(2026-09-21 复审 Important)。
//   "不投票"说的是"这一路的差【不参与容差比较】", 不是"这一路可以是 NaN"。NaN/Inf 不是
//   "差多少"的问题, 而是"这个数根本不是个读数"的问题 —— 没有任何容差能容纳它, 所以它必须
//   拒绝, 且与投票与否无关。
//   ⚠ 这条钉的是【次序】: 判决循环里 isfinite 原来排在
//     `if (!g_guardVote[i]) continue;` 之下, 于是不投票通道上的非有限值直接放行 ——
//     非有限值既没有大小, 也就没有任何容差能容纳它, 它不许被当成"差在限内"报出去。
//     (它【不是】在说数值会漏到下游: ForcePipeline 的第一道是 Butterworth, 那道滤波器
//     自己就检出并拒收非有限输入、返回 0 (见 ForcePipeline.cpp 的 Butterworth2::step),
//     所以梯度限幅器根本见不到非有限值。)
//     "力矩不再投票"这次改动把这条次序缺口扩到了【所有】不投票的通道 —— 补上之后
//     Fz (它在力矩之前就已经不投票) 那个【既有】的次序缺口也一并关上。
//   ⚠ 本用例必须【同时】喂"极大但有限"的同一个通道, 否则它会退化成恒真: 若哪天该通道被
//     改成投票通道, 只喂 NaN 时两条分支都拒绝, 用例绿着什么都没测到。下面 (a) 是这个对照 ——
//     它证明该通道【确实不投票】(极大的有限差照样放行), 于是 (b) 的拒绝只可能来自"非有限"。
static void test_guard_nonfinite_refuses_even_on_nonvoting_channel() {
    TEST(guard_nonfinite_refuses_even_on_nonvoting_channel);
    const double nan = std::numeric_limits<double>::quiet_NaN();
    double pose[6] = {0, 0, 0, 0, 0, 0};

    // (a) 对照: 同一个通道上一个【极大但有限】的差 -> 不投票 => 放行, 且数据真的过去了。
    {
        ForceCompensation::init();
        double A[9]; diagA(1.0, A);
        double com[3] = {0, 0, 0};
        double bF[3] = {0, 0, 0}, bM[3] = {0, 0, 0};
        ForceCompensation::setCalibration(A, bF, bM, com);

        AppState::ForceData fd = gateVisibleFrame();
        fd.sixForceRaw[2] = 9.81;      // compensated = (0,0,0) (重力被减掉)
        fd.sixForceRaw[0] = 0.5;       // 一个真实外力 —— 分辨"放行"与"拒绝" (拒绝时它必是 0)
        fd.tcpForce[0] = 0.5;          // 力通道一致
        fd.tcpForce[3] = 1.20;         // 力矩通道: 远超声明的力矩容差
        CHECK(1.20 > Config::FORCE_GUARD_TOL_MOMENT_NM);
        ForceCompensation::step(fd, pose);

        ForceCompensation::GuardReport rep;
        ForceCompensation::guardReport(rep);
        CHECK(rep.voted[3] == false);  // 掩码从生产 API 读, 不在这里抄一份字面量
        CHECK(ForceCompensation::guardState() == ForceCompensation::GuardState::OK);
        CHECK(fabs(fd.compensated[0] - 0.5) < 1e-9);   // 没被置零 = 真的放行了
    }

    // (b) 同一个通道换成 NaN -> 必须拒绝, 哪怕它不投票。
    {
        ForceCompensation::init();
        double A[9]; diagA(1.0, A);
        double com[3] = {0, 0, 0};
        double bF[3] = {0, 0, 0}, bM[3] = {0, 0, 0};
        ForceCompensation::setCalibration(A, bF, bM, com);

        AppState::ForceData fd = gateVisibleFrame();
        fd.sixForceRaw[2] = 9.81;
        fd.sixForceRaw[0] = 0.5;
        fd.tcpForce[0] = 0.5;          // 投票通道完全正常 —— 拒绝只可能来自那个 NaN
        fd.tcpForce[3] = nan;
        ForceCompensation::step(fd, pose);

        ForceCompensation::GuardReport rep;
        ForceCompensation::guardReport(rep);
        CHECK(rep.voted[3] == false);  // 它仍然不投票: 拒绝不是因为"它把票投出来了"
        CHECK(ForceCompensation::guardState() == ForceCompensation::GuardState::INCONSISTENT);
        CHECK(fd.compensated[0] == 0.0);   // 判决全或无: 力通道跟着一起断
    }

    // (c) 同上, 换成 Fz —— 它在力矩之前就已经不投票, 所以这是那个【既有】缺口。
    //     与 (b) 同一个形状: 投票通道正常, 只把非有限值喂到不投票的 z 那一侧。
    {
        ForceCompensation::init();
        double A[9]; diagA(1.0, A);
        double com[3] = {0, 0, 0};
        double bF[3] = {0, 0, 0}, bM[3] = {0, 0, 0};
        ForceCompensation::setCalibration(A, bF, bM, com);

        AppState::ForceData fd = gateVisibleFrame();
        fd.sixForceRaw[2] = 9.81;
        fd.sixForceRaw[0] = 0.5;
        fd.tcpForce[0] = 0.5;
        fd.tcpForce[2] = nan;
        ForceCompensation::step(fd, pose);

        ForceCompensation::GuardReport rep;
        ForceCompensation::guardReport(rep);
        CHECK(rep.voted[2] == false);
        CHECK(ForceCompensation::guardState() == ForceCompensation::GuardState::INCONSISTENT);
        CHECK(fd.compensated[0] == 0.0);
    }
    PASS();
}

// ★ 非有限值【不再永久掐死力路】(2026-09-21 收口, 最终复审 2a)。
//   EMA 那条递推式自己留不住非有限值: `NaN + α·(有限 − NaN)` 恒为 NaN ⇒ 一次坏读数会让
//   这个槽位【永久】非有限, 于是闸门从此每帧都拒 —— 一条坏帧把"传感器力那一条路"掐到
//   有人重新标定为止。fail-closed 是对的, 【永久】不是。
//   本用例钉三件事: (a) 坏那一帧照旧拒; (b) 紧接着的好帧必须【恢复】(不是继续拒);
//   (c) 源头一直坏则【每帧都拒】—— 恢复不等于放行一个一直坏的源。
static void test_guard_recovers_after_a_nonfinite_frame() {
    TEST(guard_recovers_after_a_nonfinite_frame);
    const double nan = std::numeric_limits<double>::quiet_NaN();
    double pose[6] = {0, 0, 0, 0, 0, 0};

    ForceCompensation::init();
    double A[9]; diagA(1.0, A);
    double com[3] = {0, 0, 0};
    double bF[3] = {0, 0, 0}, bM[3] = {0, 0, 0};
    ForceCompensation::setCalibration(A, bF, bM, com);

    // 一帧共同的构造: 本地算出 compensated = (0.5, 0, 0), 参考量前三通道对齐。
    // 单把 refNow[ch] 改成坏值 —— 于是"拒/放行"只可能来自那一帧的参考量。
    const int CH = 1;                 // 用【投票】通道, 让 (a) 的拒绝与投票无关地也成立
    AppState::ForceData fd = gateVisibleFrame();
    fd.sixForceRaw[2] = 9.81;
    fd.sixForceRaw[0] = 0.5;
    fd.tcpForce[0] = 0.5;
    fd.tcpForce[CH] = nan;            // (a) 坏帧
    ForceCompensation::step(fd, pose);
    CHECK(ForceCompensation::guardState() == ForceCompensation::GuardState::INCONSISTENT);
    CHECK(fd.compensated[0] == 0.0);  // 全或无: 力通道跟着断

    // (b) 下一帧恢复正常 -> 必须【同一帧就回到正常比较】, 不是"再拒几帧"、更不是"永远拒"
    AppState::ForceData good = gateVisibleFrame();
    good.sixForceRaw[2] = 9.81;
    good.sixForceRaw[0] = 0.5;
    good.tcpForce[0] = 0.5;           // tcpForce[CH] 保持初值 0
    ForceCompensation::step(good, pose);
    CHECK(ForceCompensation::guardState() == ForceCompensation::GuardState::OK);
    CHECK(fabs(good.compensated[0] - 0.5) < 1e-9);   // 数据真的又过去了

    // (c) 源头一直坏 -> 每帧都拒 (恢复的判据是【本帧的差】, 不是"曾经坏过/曾经好过")
    for (int f = 0; f < 3; f++) {
        AppState::ForceData bad = gateVisibleFrame();
        bad.sixForceRaw[2] = 9.81;
        bad.sixForceRaw[0] = 0.5;
        bad.tcpForce[0] = 0.5;
        bad.tcpForce[CH] = nan;
        ForceCompensation::step(bad, pose);
        CHECK(ForceCompensation::guardState() == ForceCompensation::GuardState::INCONSISTENT);
        CHECK(bad.compensated[0] == 0.0);
    }
    PASS();
}

// ★★ 状态跃迁【一律】打整块; 节流【只管复报】(2026-09-21 收口 Fix 1)。
//   前一版把跃迁也按同一个间隔节流掉, 并在注释里承诺"整块到点补出" —— 那个承诺【没有兑现】
//   (补出需要有"还欠着一块"的状态, 代码里没有它)。后果: 【在节流窗口里进入的状态】只剩一行
//   紧凑读数, 原因/处置、本帧姿态、逐通道的"超限 <== 触发"标记全都看不到 —— 而现场抄数
//   要抄的恰恰是这几样。本用例按【次序】钉四件事:
//     ① 启动后第一次跃迁: 打整块;
//     ② 同一个复报间隔内【再一次跃迁】: 仍然打整块 (前一版在这里只出一行 —— 被修掉的那条);
//     ③ 复位 (setCalibration -> resetGuard) 之后第一次跃迁: 仍然打整块;
//     ④ 复报【才】被节流: 状态不变 -> 一个字符都不打; 过一个间隔 -> 只出那一行紧凑读数。
//   ⚠ ④ 要等一个【真实的】FORCE_GUARD_REPORT_MS (5 s): 判据用的是 GetTickCount, 没有注入
//     时钟的口子, 而"间隔到点才复报"只能这么验。本用例因此比其他用例慢 5 s 出头。
static void test_guard_transition_always_prints_block_repeat_is_throttled() {
    TEST(guard_transition_always_prints_block_repeat_is_throttled);
    const char* capPath = "gate_transition_capture.tmp";
    double pose[6] = {0, 0, 0, 0, 0, 0};

    ForceCompensation::init();
    double A[9]; diagA(1.0, A);
    double com[3] = {0, 0, 0};
    double bF[3] = {0, 0, 0}, bM[3] = {0, 0, 0};
    ForceCompensation::setCalibration(A, bF, bM, com);

    // 一帧"参考量不可用"(六维力自报不在线), 与一帧"一切正常"—— 交替它们就能制造跃迁。
    AppState::ForceData bad = gateVisibleFrame();
    bad.sixForceOnline = 0;
    AppState::ForceData good = gateVisibleFrame();

    std::string t;

    // ---- ① 启动后第一次跃迁: 整块 ----
    if (!capStepForPrint(capPath, bad, pose, t)) {
        std::cout << std::endl << "    FAIL (捕获窗口没搭起来): 本条打印没有被钉住。" << std::endl;
        g_failed++;
        return;
    }
    if (t.find(kGuardBlockHeader) == std::string::npos) {
        std::cout << std::endl << "    FAIL: 启动后第一次拒绝没有打整块 (找不到整块的抬头句)。"
                  << std::endl;
        g_failed++;
        return;
    }
    CHECK(ForceCompensation::guardState() == ForceCompensation::GuardState::REFERENCE_UNAVAILABLE);

    // ---- ② 回到放行, 再跃迁 (同一个复报间隔内): 仍然整块 ----
    //   前一版在这里只出一行紧凑读数 —— 那正是被修掉的那条。
    ForceCompensation::step(good, pose);
    CHECK(ForceCompensation::guardState() == ForceCompensation::GuardState::OK);
    if (!capStepForPrint(capPath, bad, pose, t)) {
        std::cout << std::endl << "    FAIL (捕获窗口没搭起来): 本条打印没有被钉住。" << std::endl;
        g_failed++;
        return;
    }
    if (t.find(kGuardBlockHeader) == std::string::npos) {
        std::cout << std::endl << "    FAIL: 复报间隔内【第二次】跃迁没有打整块 —— 在节流窗口里"
                  << "进入的那个状态丢了原因/处置/本帧姿态/逐通道标记 (本波修的就是这一条)。"
                  << std::endl;
        g_failed++;
        return;
    }

    // ---- ③ 复位之后第一次跃迁: 仍然整块 (整块不节流 ⇒ 不依赖任何时刻量) ----
    ForceCompensation::setCalibration(A, bF, bM, com);   // -> resetGuard
    if (!capStepForPrint(capPath, bad, pose, t)) {
        std::cout << std::endl << "    FAIL (捕获窗口没搭起来): 本条打印没有被钉住。" << std::endl;
        g_failed++;
        return;
    }
    if (t.find(kGuardBlockHeader) == std::string::npos) {
        std::cout << std::endl << "    FAIL: 复位之后第一次跃迁没有打整块 —— 那会让操作员"
                  << "看不到新一套判据状态的原因与处置。" << std::endl;
        g_failed++;
        return;
    }

    // ---- ④(a) 状态不变、间隔没到: 一个字符都不打 (复报的节流就在这一条上) ----
    if (!capStepForPrint(capPath, bad, pose, t)) {
        std::cout << std::endl << "    FAIL (捕获窗口没搭起来): 本条打印没有被钉住。" << std::endl;
        g_failed++;
        return;
    }
    if (!t.empty()) {
        std::cout << std::endl << "    FAIL: 状态没变、间隔也没到, 却又出声了 (" << t.size()
                  << " 字节) —— 拒绝是常态, 这会把现场要读的别的输出冲掉。" << std::endl;
        g_failed++;
        return;
    }
    CHECK(ForceCompensation::guardState() == ForceCompensation::GuardState::REFERENCE_UNAVAILABLE);

    // ---- ④(b) 过一个 FORCE_GUARD_REPORT_MS 之后: 只出那一行紧凑读数, 【不】出整块 ----
    Sleep(static_cast<DWORD>(Config::FORCE_GUARD_REPORT_MS) + 200);
    if (!capStepForPrint(capPath, bad, pose, t)) {
        std::cout << std::endl << "    FAIL (捕获窗口没搭起来): 本条打印没有被钉住。" << std::endl;
        g_failed++;
        return;
    }
    // 正查: 间隔到点【必须】有一句话 (复报不是静默)。
    if (t.find("仍在拒绝") == std::string::npos) {
        std::cout << std::endl << "    FAIL: 复报间隔到点之后一个字的拒绝读数都没有 —— 那才是"
                  << "静默。整块只在跃迁时打, 所以这里该出现的是一行紧凑读数。" << std::endl;
        g_failed++;
        return;
    }
    // 反查: 复报【不许】把整块重抄一遍 (这里配着上面那条正查用, 不会像"没有 (复报)"那样恒真 ——
    // 抬头句一旦改名, ①②③ 会先红)。
    if (t.find(kGuardBlockHeader) != std::string::npos) {
        std::cout << std::endl << "    FAIL: 复报打出了整块 —— 全表只在跃迁时打 (每 5 s 一次"
                  << "那几行解释 + 6 行表 + 姿态行会把现场要读的别的输出全冲掉)。" << std::endl;
        g_failed++;
        return;
    }
    PASS();
}

// 默认构造的 GuardReport 【不许】声称任何一份掩码: 类型自己的初值无法引用 .cpp 里的
// static 掩码, 任何抄在那里的字面量都会漂 —— 于是"改一处忘一处"会让一份默认构造的报告
// 对外报出与实际生效不同的掩码。填真值由 guardReport() 负责; 初值一律 false = "尚未填充"。
static void test_guard_default_report_claims_no_mask() {
    TEST(guard_default_report_claims_no_mask);
    ForceCompensation::GuardReport rep;
    for (int i = 0; i < 6; i++) CHECK(rep.voted[i] == false);
    PASS();
}

// EMA 的语义: 【第一帧按瞬时差判】(fail closed), 之后按指数滑动平均 —— 所以一个持续
// 的不一致会在若干帧之后才触发, 而不是立刻。本用例把这条时间常数钉住, 免得将来有人
// 把 EMA 改没了 (那样逐帧噪声就会直接进判决) 或改成"永远不触发"。
static void test_guard_ema_needs_sustained_mismatch() {
    TEST(guard_ema_needs_sustained_mismatch);
    ForceCompensation::init();
    double A[9]; diagA(1.0, A);
    double com[3] = {0, 0, 0};
    double bF[3] = {0, 0, 0}, bM[3] = {0, 0, 0};
    ForceCompensation::setCalibration(A, bF, bM, com);

    double pose[6] = {0, 0, 0, 0, 0, 0};      // g = (0,0,9.81), com = 0 -> compensated 全是 0
    AppState::ForceData fd = gateVisibleFrame();
    fd.sixForceRaw[2] = 9.81;

    // 第 1 帧: 瞬时差 0.30 N, 在容差 1.2464 之内 -> 放行 (EMA 由第 1 帧播种)。
    fd.tcpForce[0] = -0.30;
    ForceCompensation::step(fd, pose);
    CHECK(ForceCompensation::guardState() == ForceCompensation::GuardState::OK);

    // 之后持续 2.5 N: EMA 从 0.30 爬向 2.5, 迟早越过力通道容差 1.2464 N。
    // 目标值必须【明显】超过容差, 否则"哪一帧越线"就落在噪声上了, 而这条钉的正是那个时间。
    // ⚠ 从 0.90 改成 2.5 (2026-09-21): 容差 0.50 -> 1.2464 之后, 0.90 N 永远越不过去
    //   (EMA 的稳态值就是 0.90), 这条会假红。
    fd.tcpForce[0] = -2.5;

    // ★★ 2026-09-21: 这条用例的【时间】现在由用例自己给 —— 不再靠帧数隐含采样率。
    // 【为什么必须改】闸门的 α 从前是每帧固定的 0.02 (隐含 30Hz), 当天改成按实测 dt 换算
    //   (α = dt/τ, τ = FORCE_GUARD_EMA_TAU_S = 3.3 s)。而紧循环里 GetTickCount 的 dt ≈ 0
    //   ⇒ EMA 冻住 ⇒ 原来"第 41 帧越线"的断言必然红, 而且【红得没有信息】(它测的是墙钟,
    //   不是那条规则)。⇒ 用 setStepDtForTest 把时间钉成确定的输入。
    // ⚠ 这也顺手消掉了墙钟依赖 —— 本项目已有一条用例因为用 Sleep 凑时间而间歇性假红 (A19)。
    const double DT = 0.008;                       // 与生产实际节拍同量级 (125Hz)
    ForceCompensation::setStepDtForTest(DT);

    // 算清楚"多久才该越线"(这是本条断言的全部内容):
    //   τ = 3.3 s, dt = 8 ms ⇒ α = dt/τ = 2.4242e-3
    //   EMA(n) = 2.5 − (2.5−0.30)·(1−α)^n
    //   越线条件 EMA(n) > 1.2464  ⇒  (1−α)^n < 0.56982  ⇒  n > 232  (≈1.86 s)
    // ⇒ 取两侧都有余量的两个点: n=100 (≈0.24τ, EMA≈0.77) 与 n=400 (≈0.97τ, EMA≈1.67)。
    for (int k = 0; k < 4; k++) ForceCompensation::step(fd, pose);
    CHECK(ForceCompensation::guardState() == ForceCompensation::GuardState::OK);

    for (int k = 0; k < 96; k++) ForceCompensation::step(fd, pose);   // 累计 n=100 (0.8 s)
    CHECK(ForceCompensation::guardState() == ForceCompensation::GuardState::OK);

    for (int k = 0; k < 300; k++) ForceCompensation::step(fd, pose);  // 累计 n=400 (3.2 s)
    CHECK(ForceCompensation::guardState() == ForceCompensation::GuardState::INCONSISTENT);

    ForceCompensation::GuardReport rep;
    ForceCompensation::guardReport(rep);
    CHECK(rep.ema[0] > Config::FORCE_GUARD_TOL_FORCE_N);
    CHECK(rep.frames == 401);
    // ⚠ 【必须还原】: 这是进程级全局, 不还原会漏给后面每一条用例 (本项目那份"测试之间
    //   互相污染"的账就是这么来的 —— 见 guard 那条旧红)。
    ForceCompensation::setStepDtForTest(-1.0);
    PASS();
}

// ===== 调零 (TARE only) 测试 =====

static int g_dragOnCalls = 0;   // 拖拽模式被「开启」的次数 — 调零流程必须为 0

static void countDrag(bool enable) {
    if (enable) g_dragOnCalls++;
}

// 调零: 静置采集 → 直接应用零偏 + 存盘; 不开拖拽、不进 MOTION、【A / c_s 原样保留】
static void test_zero_only_no_motion() {
    TEST(zero_only_no_motion);
    ForceCompensation::init();
    double A[9]; diagA(0.42, A);                            // 预置全量模型 (质量尺度 0.42 kg)
    double cS[3] = {0.0, 0.0, 0.03};
    double bF[3] = {0, 0, 0}, bM[3] = {0, 0, 0};
    ForceCompensation::setCalibration(A, bF, bM, cS);

    g_dragOnCalls = 0;
    ForceCalibration::setDragModeCallback(countDrag);

    CHECK(ForceCalibration::startZero());
    CHECK(ForceCalibration::isZeroing());

    // 静置 2s: 每次 0.5s, 采集满 FORCE_CALIB_STILL_COLLECT_S 后自动定稿
    double raw[6] = {-0.48, -1.35, -0.02, 0.010, -0.020, 0.005};
    double pose[6] = {0, 0, 0, 0, 0, 0};
    for (int i = 0; i < 5 && !ForceCalibration::isDone(); i++) {
        ForceCalibration::update(0.5, raw, pose);
    }

    CHECK(ForceCalibration::isDone());
    CHECK(!ForceCalibration::isZeroing());
    CHECK(!ForceCalibration::isRunning());
    CHECK(g_dragOnCalls == 0);                                      // 未开拖拽模式
    CHECK(fabs(ForceCompensation::currentMassKg() - 0.42) < 1e-9);  // 模型保留 (质量尺度)

    // ★ 调零【只】动零偏: A 与 c_s 必须逐位不变 (调零前把 A/c_s 弄脏一点再比)。
    double A2[9], cS2[3];
    ForceCompensation::currentModel(A2, cS2);
    for (int i = 0; i < 9; i++) CHECK(fabs(A2[i] - A[i]) < 1e-15);
    for (int i = 0; i < 3; i++) CHECK(fabs(cS2[i] - cS[i]) < 1e-15);

    // 零偏已生效。pose 全 0 → 重力项 A·g 只在 Z 轴 (A 是对角), X/Y 纯看零偏。
    // 参考量那一侧 (fd.tcpForce) 填成"机械臂也给出同一个外力"的样子 —— 闸门放行才量得到
    // 补偿结果。(@576 那一路现在是诊断侧, 闸门不看它, 所以这里不必再喂。)
    // 期望值 (A = 0.42·I, g = (0,0,9.81) -> Fg = (0,0,4.1202), c_s 沿 z -> Mg = 0):
    //   x: 5 − (−0.48)          = 5.48
    //   y: 5 − (−1.35)          = 6.35
    //   z: 5 − (−0.02) − 4.1202 = 0.8998
    //   M: 5 − (0.010, −0.020, 0.005)
    AppState::ForceData fd = gateVisibleFrame();
    for (int i = 0; i < 6; i++) fd.sixForceRaw[i] = 5.0;
    fd.tcpForce[0] = 5.48; fd.tcpForce[1] = 6.35; fd.tcpForce[2] = 0.90;
    fd.tcpForce[3] = 4.99; fd.tcpForce[4] = 5.02; fd.tcpForce[5] = 4.995;
    ForceCompensation::step(fd, pose);
    CHECK(ForceCompensation::guardState() == ForceCompensation::GuardState::OK);
    CHECK(fabs(fd.compensated[0] - (5.0 - (-0.48))) < 0.02);
    CHECK(fabs(fd.compensated[1] - (5.0 - (-1.35))) < 0.02);
    // Z: 5.0 - (-0.02) - 0.42*9.81 ≈ 0.90 — 同时证明保留的 A 确实进了 setCalibration
    CHECK(fabs(fd.compensated[2] - 0.90) < 0.05);
    // 力矩零偏也已应用 (Mx 零偏 0.010); c_s 沿 z 与 A·g 平行 → 重力力矩为 0
    CHECK(fabs(fd.compensated[3] - (5.0 - 0.010)) < 0.02);

    ForceCalibration::setDragModeCallback(nullptr);
    PASS();
}

// 调零中止: 未采集满就中止 → 零偏不应用、不存盘
// ⚠ 2026-09-19: 这里【曾经】用 A = 全 0 当"空模型"。全零 A 现在被 setCalibration 拒收
//   (用户指令 3), 于是"零偏没被应用"这件事会被闸门置零掩盖掉, 测不出东西来。
//   改成装一个【正常的】模型 + 一组[已知的]旧零偏, 再中止一次调零 —— 这样断言的
//   就是"新零偏没进去、旧零偏还在", 与闸门放不放行无关。
static void test_zero_abort_not_applied() {
    TEST(zero_abort_not_applied);
    ForceCompensation::init();
    double A[9]; diagA(0.42, A);
    double cS[3] = {0, 0, 0.03};
    double bF[3] = {1.0, 2.0, 3.0}, bM[3] = {0.1, 0.2, 0.3};   // 已知旧零偏
    ForceCompensation::setCalibration(A, bF, bM, cS);
    CHECK(ForceCompensation::isCalibrated());

    CHECK(ForceCalibration::startZero());
    double raw[6] = {-0.48, -1.35, -0.02, 0.010, -0.020, 0.005};
    double pose[6] = {0, 0, 0, 0, 0, 0};
    ForceCalibration::update(0.5, raw, pose);   // 只采 0.5s, 未达阈值
    ForceCalibration::abort();

    CHECK(!ForceCalibration::isRunning());
    CHECK(!ForceCalibration::isZeroing());

    // 零偏【逐位】没动 —— 这是这条用例真正要钉的东西。
    double bF2[3], bM2[3];
    ForceCompensation::currentBias(bF2, bM2);
    for (int i = 0; i < 3; i++) {
        CHECK(fabs(bF2[i] - bF[i]) < 1e-15);
        CHECK(fabs(bM2[i] - bM[i]) < 1e-15);
    }

    // 补偿结果里用的仍是旧零偏: x = 5 − 1.0 = 4.0 (不是 5 − (−0.48) = 5.48)。
    AppState::ForceData fd = gateVisibleFrame();
    for (int i = 0; i < 6; i++) fd.sixForceRaw[i] = 5.0;
    fd.tcpForce[0] = 4.0; fd.tcpForce[1] = 3.0; fd.tcpForce[2] = 5.0 - 3.0 - 4.1202;
    fd.tcpForce[3] = 4.9; fd.tcpForce[4] = 4.8; fd.tcpForce[5] = 4.7;
    ForceCompensation::step(fd, pose);
    CHECK(ForceCompensation::guardState() == ForceCompensation::GuardState::OK);
    CHECK(fabs(fd.compensated[0] - 4.0) < 0.02);   // 旧零偏 (1.0), 不是新的 (−0.48)
    PASS();
}

// 回归: 全流程标定仍然进 MOTION 相并开启拖拽模式
static void test_sweep_still_enters_motion() {
    TEST(sweep_still_enters_motion);
    ForceCompensation::init();
    g_dragOnCalls = 0;
    ForceCalibration::setDragModeCallback(countDrag);

    CHECK(ForceCalibration::start());
    CHECK(!ForceCalibration::isZeroing());          // 全流程不是调零

    double raw[6] = {0, 0, 0, 0, 0, 0};
    double pose[6] = {0, 0, 0, 0, 0, 0};
    for (int i = 0; i < 5 && ForceCalibration::currentState() == ForceCalibration::State::TARE; i++) {
        ForceCalibration::update(0.5, raw, pose);
    }

    CHECK(ForceCalibration::currentState() == ForceCalibration::State::MOTION);
    CHECK(ForceCalibration::isRunning());
    CHECK(g_dragOnCalls == 1);                      // 全流程开拖拽
    ForceCalibration::abort();                      // 收尾, 避免影响后续
    CHECK(g_dragOnCalls == 1);                      // abort 只关不开
    CHECK(!ForceCalibration::isRunning());

    ForceCalibration::setDragModeCallback(nullptr);
    PASS();
}

// 可重复调零: 上一次结束后能再次启动 (不做进程重启)
static void test_zero_restartable() {
    TEST(zero_restartable);
    ForceCompensation::init();
    double A[9]; diagA(0.42, A);                 // 正常模型 (全零 A 会被 setCalibration 拒收)
    double cS[3] = {0, 0, 0.03};
    double bF[3] = {0, 0, 0}, bM[3] = {0, 0, 0};
    ForceCompensation::setCalibration(A, bF, bM, cS);

    double raw[6] = {0.1, 0.2, 0.3, 0, 0, 0};
    double pose[6] = {0, 0, 0, 0, 0, 0};

    CHECK(ForceCalibration::startZero());
    for (int i = 0; i < 5 && !ForceCalibration::isDone(); i++) {
        ForceCalibration::update(0.5, raw, pose);
    }
    CHECK(ForceCalibration::isDone());

    CHECK(ForceCalibration::startZero());           // 第二次调零 (前置状态是 DONE)
    CHECK(ForceCalibration::isZeroing());
    ForceCalibration::abort();

    // 全流程标定同样可重启 (前置状态 ABORTED)
    CHECK(ForceCalibration::start());
    CHECK(!ForceCalibration::isZeroing());
    CHECK(ForceCalibration::currentState() == ForceCalibration::State::TARE);
    ForceCalibration::abort();
    CHECK(!ForceCalibration::isRunning());
    PASS();
}

// ===== force_calib.json 的格式 (Task 6: version 2 -> 3) =====
//
// 文件写在【当前目录】(测试 exe 从 tests/ 跑), 跑完即删 —— .gitignore 里已经有一条
// force_calib.json, 所以即使中途崩了也不会污染仓库。

static const char* TMP_NEW = "force_calib.json";        // 与 .gitignore 里那一条同名
static const char* TMP_OLD = "force_calib.json";        // 旧格式也写同一个名字 (先覆盖再读)

// ===== "安装 → 落盘 → 重启装载" 这条链 (2026-09-20) =====
//
// 【为什么单独立一条】: 上面的 calib_file_roundtrip 只做 save → load, 起点已经是内存里装好的
// 模型。而【进程重启】实际发生的是: init() 把一切清空 → loadFromFile → setCalibration。
// 's' 求解成功后走的是这条链的另一半 (setCalibration → saveToFile)。
// 两半合起来才是"这次标定能活过重启"; 断哪一半, 现场看到的现象都是同一句:
// 闸门报"没有可用模型"、控制台每 5 秒说一次"未标定" —— 而根因完全不同, 所以两半都要钉。
//
// ⚠ 本用例会调 ForceCompensation::init(), 所以它【注册在 main() 的最后】: 免得给后面的
//   用例留下一台"刚开机"的机器。
static void test_install_then_reload_roundtrip() {
    TEST(install_then_reload_roundtrip);
    double A[9]  = { 0.3645611253, 0.2105461118, -0.0000294763,
                    -0.2182776660, 0.3733052719,  0.0029763987,
                    -0.0115452228, -0.0149328140, -0.4139021828 };
    double bF[3] = { -21.9, -1.4, 2.6 };
    double bM[3] = { -0.18, 0.38, -0.025 };
    double cS[3] = { 0.0005981153, -0.0005015476, 0.0545494358 };

    // 1) 's' 的那一半: 装进本会话内存。
    //    setCalibration 返回 void, 拒收与否只能靠 isCalibrated() 事后问 —— 这正是
    //    solveAndApply 里那条判断的写法, 所以这里也照那个写法钉。
    ForceCompensation::setCalibration(A, bF, bM, cS);
    CHECK(ForceCompensation::isCalibrated());

    // 2) 同一份再落盘。
    CHECK(ForceCalibration::saveToFile(TMP_NEW, A, bF, bM, cS));

    // 3) 模拟进程重启: init() 把模型清光 (这一步是 calib_file_roundtrip 没有的那一段)。
    ForceCompensation::init();
    CHECK(!ForceCompensation::isCalibrated());

    // 4) 启动装载的那一半。
    double A2[9], bF2[3], bM2[3], cS2[3];
    CHECK(ForceCalibration::loadFromFile(TMP_NEW, A2, bF2, bM2, cS2));
    ForceCompensation::setCalibration(A2, bF2, bM2, cS2);
    CHECK(ForceCompensation::isCalibrated());

    // 5) 装回去的必须与当初那一份一致 (容差与 calib_file_roundtrip 同口径:
    //    A 用 %.9g、c_s 用 %.9g 落盘)。
    double A3[9], cS3[3];
    ForceCompensation::currentModel(A3, cS3);
    for (int i = 0; i < 9; i++) CHECK(fabs(A3[i] - A[i]) < 1e-8);
    for (int i = 0; i < 3; i++) CHECK(fabs(cS3[i] - cS[i]) < 1e-9);

    remove(TMP_NEW);
    PASS();
}

// ===== 为什么 solveAndApply 里那个"装上了才落盘"的门是真的在挡东西 =====
//
// 那条路径门控在 isCalibrated() 之后而不是无条件落盘。这条用例钉住它的理由:
//   saveToFile 【不做任何校验】—— 给它一份 setCalibration 会拒收的 A, 它照样写得出来。
// 所以"先装后落"不是多余的谨慎: 反过来做, 一次拒收就会用一份【装载时必然被拒】的模型
// 覆盖掉盘上可能好用的那一份 —— 而 saveToFile 是 "w" 打开, 覆盖即截断, 没有备份。
//
// ⚠ 中间那个 fopen 不能省: loadFromFile 对"文件不存在"也返回 false, 少了它这条断言谁都能
//   满足 —— 那时它证的是"没写出文件", 而不是"写出的文件被拒"。
//   (同型说明见 test_calib_file_rejects_old_format 末尾。)
static void test_save_writes_unusable_model_but_load_rejects() {
    TEST(save_writes_unusable_model_but_load_rejects);
    double zeroA[9] = {0};
    double bF[3] = { -21.9, -1.4, 2.6 };
    double bM[3] = { -0.18, 0.38, -0.025 };
    double cS[3] = { 0.0005981153, -0.0005015476, 0.0545494358 };

    // 全零 A 装不进去 (与 test_setcalib_rejects_zero_and_degenerate_A 共用同一条判据)。
    ForceCompensation::setCalibration(zeroA, bF, bM, cS);
    CHECK(!ForceCompensation::isCalibrated());

    // 但 saveToFile 不检查 —— 它照写。
    CHECK(ForceCalibration::saveToFile(TMP_NEW, zeroA, bF, bM, cS));

    // 先确认文件【真的在】。
    FILE* f = fopen(TMP_NEW, "r");
    CHECK(f != nullptr);
    if (f) fclose(f);

    // 装载端拒它 —— 这就是"装不上就不该落盘"所指的那件事。
    double A[9], bF2[3], bM2[3], cS2[3];
    CHECK(!ForceCalibration::loadFromFile(TMP_NEW, A, bF2, bM2, cS2));

    remove(TMP_NEW);
    PASS();
}

static void test_calib_file_roundtrip() {
    TEST(calib_file_roundtrip);
    double A[9]  = { 0.3645611253, 0.2105461118, -0.0000294763,
                    -0.2182776660, 0.3733052719,  0.0029763987,
                    -0.0115452228, -0.0149328140, -0.4139021828 };
    double bF[3] = { -21.9, -1.4, 2.6 };
    double bM[3] = { -0.18, 0.38, -0.025 };
    double cS[3] = { 0.0005981153, -0.0005015476, 0.0545494358 };

    CHECK(ForceCalibration::saveToFile(TMP_NEW, A, bF, bM, cS));

    double A2[9], bF2[3], bM2[3], cS2[3];
    CHECK(ForceCalibration::loadFromFile(TMP_NEW, A2, bF2, bM2, cS2));

    // A 用 %.9g 落盘 -> 相对误差 ~1e-9, 取 1e-8 相对量级作容差。
    for (int i = 0; i < 9; i++) CHECK(fabs(A2[i] - A[i]) < 1e-8);
    for (int i = 0; i < 3; i++) {
        CHECK(fabs(bF2[i] - bF[i]) < 1e-6);
        CHECK(fabs(bM2[i] - bM[i]) < 1e-6);
        CHECK(fabs(cS2[i] - cS[i]) < 1e-9);
    }
    remove(TMP_NEW);
    PASS();
}

// 旧文件 (version 2: mass_kg + 零偏) 必须【被拒】。
// 这是有意的: 旧文件里没有 A, 拿新版读会安静地得到一份【没有重力项】的模型 ——
// 补偿后的读数依旧是 N, 不会报错。所以这条用例断言的是"拒", 不是"兼容"。
static void test_calib_file_rejects_old_format() {
    TEST(calib_file_rejects_old_format);
    static const char* V2 =
        "{\n"
        "  \"version\": 2,\n"
        "  \"mass_kg\": 0.25,\n"
        "  \"bias_force_n\": [-0.48, -1.35, -0.02],\n"
        "  \"bias_torque_nm\": [0.01, -0.02, 0.005]\n"
        "}\n";
    FILE* f = fopen(TMP_OLD, "w");
    CHECK(f != nullptr);
    fputs(V2, f);
    fclose(f);

    double A[9], bF[3], bM[3], cS[3];
    // 上面的拒绝消息会打到 stderr —— 但【不是静默】就够了, 这里断言的是返回值。
    bool ok = ForceCalibration::loadFromFile(TMP_OLD, A, bF, bM, cS);
    remove(TMP_OLD);
    CHECK(!ok);

    // 同一件事的另一半: 文件【根本不存在】时也返回 false, 但那是正常路径 (还没标定过),
    // 不是格式不兼容 —— 两者的返回值相同, 区别只在 stderr 上那一段。这里顺手钉住返回值,
    // 免得将来有人把"文件不存在"也改成大声报错而淹没真正的格式错。
    CHECK(!ForceCalibration::loadFromFile("no_such_force_calib_file.json", A, bF, bM, cS));
    PASS();
}

// version 字段缺失 (更老的文件) 同样被拒。
static void test_calib_file_rejects_no_version() {
    TEST(calib_file_rejects_no_version);
    static const char* V1 =
        "{ \"bias_force_n\": [0, 0, 0], \"bias_torque_nm\": [0, 0, 0] }\n";
    FILE* f = fopen(TMP_NEW, "w");
    CHECK(f != nullptr);
    fputs(V1, f);
    fclose(f);

    double A[9], bF[3], bM[3], cS[3];
    CHECK(!ForceCalibration::loadFromFile(TMP_NEW, A, bF, bM, cS));
    remove(TMP_NEW);
    PASS();
}

// ===== 用户指令 3: 全零 A / 退化的 A 必须被拒 =====
//
// 两处都要拒, 而且是【同一条判据】:
//   · setCalibration —— 让"全零 A"根本进不了"已标定"这个状态 (现场真的会走到:
//     从未解过 A 时按 'z' 调零, ForceCalibration::update 会拿空 A 回灌进来);
//   · loadFromFile —— 文件里写着一个没有重力项的模型 (version 仍是 3)。
// 为什么必须拒: 那种文件【读得进来】, 补偿后的读数依旧是个 N, 不报错, 只是整个重力项
// 没有 —— 读数随姿态漂而所有检查都绿。这是本项目栽过多次的"安静地错"。
static void test_setcalib_rejects_zero_and_degenerate_A() {
    TEST(setcalib_rejects_zero_and_degenerate_A);
    double com[3] = {0, 0, 0.03};
    double bF[3] = {0, 0, 0}, bM[3] = {0, 0, 0};

    // (a) 全零 A
    ForceCompensation::init();
    double A0[9] = {0};
    ForceCompensation::setCalibration(A0, bF, bM, com);
    CHECK(ForceCompensation::isCalibrated() == false);

    // (b) 秩 1 的 A (只有 A[2] = 1): 有非零元, 但两个力方向没有模型。
    double A1[9] = {0};
    A1[2] = 1.0;
    ForceCompensation::setCalibration(A1, bF, bM, com);
    CHECK(ForceCompensation::isCalibrated() == false);

    // (c) 非有限
    double An[9]; diagA(1.0, An);
    An[4] = std::nan("");
    ForceCompensation::setCalibration(An, bF, bM, com);
    CHECK(ForceCompensation::isCalibrated() == false);

    // (d) 好的 A 照样收 (免得上面三条是靠"永远拒绝"过的)
    double Ag[9]; diagA(0.42, Ag);
    ForceCompensation::setCalibration(Ag, bF, bM, com);
    CHECK(ForceCompensation::isCalibrated() == true);

    // (e) 【拒绝安装 = 现在没有可用模型】: 装着一个好模型时又灌一个全零 A,
    //     旧模型也必须作废 —— 否则"拒绝安装"会退化成"继续用旧的", 而调用方以为换过了。
    ForceCompensation::setCalibration(A0, bF, bM, com);
    CHECK(ForceCompensation::isCalibrated() == false);

    // (f) 拒收之后闸门状态是"没有可用模型" (不是"不一致")。
    AppState::ForceData fd = gateVisibleFrame();
    double pose[6] = {0, 0, 0, 0, 0, 0};
    ForceCompensation::step(fd, pose);
    CHECK(ForceCompensation::guardState() == ForceCompensation::GuardState::UNCALIBRATED);
    PASS();
}

// 装载路径: version=3 但 A 全零 / 退化 / 非有限 -> 一律拒, 且不更新调用方的数组之外的任何状态。
static void test_calib_file_rejects_unusable_A() {
    TEST(calib_file_rejects_unusable_A);
    static const char* ZERO_A =
        "{ \"version\": 3,"
        "  \"a_matrix\": [0,0,0, 0,0,0, 0,0,0],"
        "  \"bias_force_n\": [-0.48, -1.35, -0.02],"
        "  \"bias_torque_nm\": [0.01, -0.02, 0.005],"
        "  \"com_sensor_m\": [0, 0, 0.055] }\n";
    static const char* RANK1_A =
        "{ \"version\": 3,"
        "  \"a_matrix\": [0,0,1, 0,0,0, 0,0,0],"
        "  \"bias_force_n\": [0,0,0],"
        "  \"bias_torque_nm\": [0,0,0],"
        "  \"com_sensor_m\": [0,0,0.055] }\n";
    static const char* NAN_A =
        "{ \"version\": 3,"
        "  \"a_matrix\": [1,0,0, 0,nan,0, 0,0,1],"
        "  \"bias_force_n\": [0,0,0],"
        "  \"bias_torque_nm\": [0,0,0],"
        "  \"com_sensor_m\": [0,0,0.055] }\n";
    static const char* NAN_BIAS =
        "{ \"version\": 3,"
        "  \"a_matrix\": [1,0,0, 0,1,0, 0,0,1],"
        "  \"bias_force_n\": [0,inf,0],"
        "  \"bias_torque_nm\": [0,0,0],"
        "  \"com_sensor_m\": [0,0,0.055] }\n";
    // 正常的一份 (对照组): 同一批字段、非退化的 A —— 它必须【读得进来】,
    // 否则上面四条拒的是"文件"而不是"A"。
    static const char* GOOD_A =
        "{ \"version\": 3,"
        "  \"a_matrix\": [0.42,0,0, 0,0.42,0, 0,0,0.42],"
        "  \"bias_force_n\": [-0.48, -1.35, -0.02],"
        "  \"bias_torque_nm\": [0.01, -0.02, 0.005],"
        "  \"com_sensor_m\": [0, 0, 0.055] }\n";

    struct Case { const char* text; bool want; const char* label; };
    const Case cases[] = {
        { ZERO_A,   false, "A 全零" },
        { RANK1_A,  false, "A 秩 1 (退化)" },
        { NAN_A,    false, "A 里有 NaN" },
        { NAN_BIAS, false, "bias_force_n 里有 inf" },
        { GOOD_A,   true,  "对照组: 非退化的 A" },
    };
    for (int k = 0; k < 5; k++) {
        FILE* f = fopen(TMP_NEW, "w");
        CHECK(f != nullptr);
        fputs(cases[k].text, f);
        fclose(f);
        double A[9], bF[3], bM[3], cS[3];
        const bool ok = ForceCalibration::loadFromFile(TMP_NEW, A, bF, bM, cS);
        if (ok != cases[k].want) {
            std::cout << "FAIL: " << cases[k].label << " 期望 "
                      << (cases[k].want ? "接受" : "拒绝") << std::endl;
            remove(TMP_NEW);
            g_failed++;
            return;
        }
    }
    remove(TMP_NEW);
    PASS();
}

// version 写着 3 但内容是半截的 -> 也不能被当成"读成功了"。
static void test_calib_file_rejects_truncated() {
    TEST(calib_file_rejects_truncated);
    static const char* V3_BAD =
        "{ \"version\": 3, \"a_matrix\": [1,0,0], \"bias_force_n\": [0,0,0] }\n";
    FILE* f = fopen(TMP_NEW, "w");
    CHECK(f != nullptr);
    fputs(V3_BAD, f);
    fclose(f);

    double A[9], bF[3], bM[3], cS[3];
    CHECK(!ForceCalibration::loadFromFile(TMP_NEW, A, bF, bM, cS));
    remove(TMP_NEW);
    PASS();
}

// ===== 启动零偏漂移检查: 闸门【放行】之后的那几支 (2026-09-21, Task 5) =====
//
// 这段判定原先整个长在 main.cpp 的 runZeroDriftCheck() 里: 由主循环用真实时钟
// (GetTickCount) 驱动、直接读 appState 与 ForceCompensation::guardState(), 【没有可注入点】
// ⇒ 一个分支都测不到。而它只在闸门放行后才做事, 闸门此前一直拒绝, 所以它至今跑的全是
// 【未做】那一支 —— 放行后正常 / 放行后超阈这两条路【从来没在现场跑过】。
// 判定已抽成 force/ZeroDriftCheck.h 里的纯函数。下面调的是【那个真实实现】(头文件内联的
// 唯一一份定义, main.cpp 与这里编译的是同一份), 不是测试里另写一份复制品。
static void driftInput(ZeroDriftCheck::Input& in, double mx, double my, double mz) {
    in.guard = ForceCompensation::GuardState::OK;
    in.mean[0] = mx; in.mean[1] = my; in.mean[2] = mz;
    in.refuseElapsedMs = 0;
    in.sampleCount = 40;
    in.thresholdN = Config::FORCE_ZERO_DRIFT_WARN_N;
    in.waitMs = 60000;
    // ⚠ 取【生产的那一个常量】, 不在这里写一个字面量: 从前这里是硬编码的 10, 于是
    // 常量改了这边不会红, 而那几条边界断言 (9 与 10 之差) 会【静默地换含义】。
    // 值的本身由下面那条用例显式钉住。
    in.minSamples = Config::FORCE_ZERO_DRIFT_MIN_SAMPLES;
}

// 文字里必须出现【函数自己算出来的】那个数 —— 不是测试塞进去的任何一个分量。
static bool textHasNumber(const std::string& text, double v) {
    char buf[64];
    snprintf(buf, sizeof(buf), "%g", v);
    return text.find(buf) != std::string::npos;
}

static void test_zero_drift_normal_is_a_conclusion() {
    TEST(zero_drift_normal_is_a_conclusion);
    ZeroDriftCheck::Input in;
    driftInput(in, 0.2, 0.2, 0.2);
    const ZeroDriftCheck::Decision d = ZeroDriftCheck::decide(in);
    // 闸门放行 -> 必须真的产出结论, 【不是】"未做"
    CHECK(d.outcome == ZeroDriftCheck::Outcome::Normal);
    CHECK(d.outcome != ZeroDriftCheck::Outcome::NotDone);
    // 报的数是它自己算的三轴模, 不是任何一个分量
    const double want = sqrt(0.2 * 0.2 + 0.2 * 0.2 + 0.2 * 0.2);
    CHECK(fabs(d.driftN - want) < 1e-12);
    CHECK(d.driftN != 0.2);
    CHECK(textHasNumber(d.text, want));
    // 正常那一支只说结论, 不许出现"未做"
    CHECK(d.text.find("未做") == std::string::npos);
    PASS();
}

static void test_zero_drift_over_threshold_is_a_conclusion() {
    TEST(zero_drift_over_threshold_is_a_conclusion);
    ZeroDriftCheck::Input in;
    // 0.8 N 静偏: 闸门容差 (1.2464 N) 之内 -> 闸门会放行, 但本检查的 0.5 N 要报。
    // 这正是本检查【比闸门紧】的地方 —— 两个数各有各的理由, 不许"对齐"成一个。
    driftInput(in, 0.8, 0.0, 0.0);
    const ZeroDriftCheck::Decision d = ZeroDriftCheck::decide(in);
    CHECK(d.outcome == ZeroDriftCheck::Outcome::OverThreshold);
    CHECK(d.outcome != ZeroDriftCheck::Outcome::NotDone);
    CHECK(fabs(d.driftN - 0.8) < 1e-12);
    CHECK(textHasNumber(d.text, 0.8));
    CHECK(textHasNumber(d.text, in.thresholdN));   // 阈值也要写进话里
    CHECK(d.text.find("未做") == std::string::npos);
    PASS();
}

// 边界: 恰好等于阈值【不报】—— 判据是严格大于 (与旧代码一致, 抽出判定时原样保留)。
// 取 0.5 N 单轴: 0.5 是二进制精确值, sqrt(0.5²)=0.5 精确, 所以这条边界不是浮点碰运气。
static void test_zero_drift_exactly_at_threshold_is_normal() {
    TEST(zero_drift_exactly_at_threshold_is_normal);
    ZeroDriftCheck::Input in;
    driftInput(in, 0.0, 0.0, 0.0);
    in.mean[0] = in.thresholdN;   // 必须在 driftInput 之后取 (它才是设阈值的那个)
    const ZeroDriftCheck::Decision d = ZeroDriftCheck::decide(in);
    CHECK(d.driftN == in.thresholdN);
    CHECK(d.outcome == ZeroDriftCheck::Outcome::Normal);
    CHECK(d.text.find("未做") == std::string::npos);
    PASS();
}

static void test_zero_drift_not_done_after_full_wait() {
    TEST(zero_drift_not_done_after_full_wait);
    // 【三种】拒绝原因走同一支: 结论都是【未做】、都不给漂移数 (2026-09-21 Task 7 起了
    // 第三种【参考量不可用】—— 它同样"没有比过", 所以同样给不出漂移数)。
    // ⚠ 它们共用一个 outcome, 但【该说的话不一样】: decide() 按状态给出不同的
    //   "为什么没查"那一段 (去标定 / 去查下发 / 去查这一路的数据是【三件不同的事】)。
    const ForceCompensation::GuardState refusals[] = {
        ForceCompensation::GuardState::UNCALIBRATED,
        ForceCompensation::GuardState::INCONSISTENT,
        ForceCompensation::GuardState::REFERENCE_UNAVAILABLE,
    };
    for (int k = 0; k < 3; k++) {
        ZeroDriftCheck::Input in;
        driftInput(in, 0.0, 0.0, 0.0);
        in.guard = refusals[k];
        in.refuseElapsedMs = in.waitMs;   // 从第一次被拒起算满等待期
        in.sampleCount = 0;
        const ZeroDriftCheck::Decision d = ZeroDriftCheck::decide(in);
        CHECK(d.outcome == ZeroDriftCheck::Outcome::NotDone);
        CHECK(d.text.find("未做") != std::string::npos);
        // 拒绝期间 compensated 是被闸门置的 0, 拿它算出来的"漂移"恒为 0 -> 不许报成读数
        CHECK(d.text.find("补偿后读数") == std::string::npos);
        CHECK(d.driftN == 0.0);
        CHECK(d.text.find("60 s") != std::string::npos);   // 等了多久要说出来
    }
    PASS();
}

// 还没等满 -> 不是结论, 一声不吭地继续等 (旧行为如此, 保持)。
static void test_zero_drift_waiting_stays_silent() {
    TEST(zero_drift_waiting_stays_silent);
    ZeroDriftCheck::Input in;
    driftInput(in, 0.0, 0.0, 0.0);
    in.guard = ForceCompensation::GuardState::INCONSISTENT;
    in.refuseElapsedMs = in.waitMs - 1;   // 差 1 ms
    const ZeroDriftCheck::Decision d = ZeroDriftCheck::decide(in);
    CHECK(d.outcome == ZeroDriftCheck::Outcome::Waiting);
    CHECK(d.outcome != ZeroDriftCheck::Outcome::NotDone);
    CHECK(d.text.empty());
    PASS();
}

// ★ 样本不足这一支曾经是【静默】的: 设完 done 就 return, 既不报结论、也不报"没查" ——
// 于是"查了、没发现问题"与"根本没查"在输出上分不开, 与该文件自己写的原则
// ("『没查』必须有句话")冲突。用户指令: 改成明说, 并用这条钉住。
static void test_zero_drift_insufficient_samples_speaks() {
    TEST(zero_drift_insufficient_samples_speaks);
    // ★ 钉住最少样本数的【值】(2026-09-21 收口): 本用例下面那两条断言 (话里要出现 9 与 10)
    //   是按 10 写的。常量若被改掉, 它们不会红, 会【静默地换成另一条边界】⇒ 这里让它响亮地红。
    CHECK(Config::FORCE_ZERO_DRIFT_MIN_SAMPLES == 10);
    ZeroDriftCheck::Input in;
    driftInput(in, 0.0, 0.0, 0.0);
    in.sampleCount = in.minSamples - 1;
    const ZeroDriftCheck::Decision d = ZeroDriftCheck::decide(in);
    CHECK(d.outcome == ZeroDriftCheck::Outcome::InsufficientSamples);
    CHECK(d.outcome != ZeroDriftCheck::Outcome::NotDone);
    CHECK(!d.text.empty());                                  // 不许静默
    CHECK(d.text.find("样本不足") != std::string::npos);
    CHECK(d.text.find("不作结论") != std::string::npos);
    // 也不许借机编一个结论出来
    CHECK(d.text.find("补偿后读数") == std::string::npos);
    CHECK(d.driftN == 0.0);
    // 差多少要说清楚: 采到几个 / 至少几个
    CHECK(d.text.find("9") != std::string::npos);
    CHECK(d.text.find("10") != std::string::npos);
    // 边界: 刚好够就必须给结论 (9 与 10 之差)
    in.sampleCount = in.minSamples;
    in.mean[0] = 0.2; in.mean[1] = 0.2; in.mean[2] = 0.2;
    const ZeroDriftCheck::Decision d2 = ZeroDriftCheck::decide(in);
    CHECK(d2.outcome == ZeroDriftCheck::Outcome::Normal);
    CHECK(!d2.text.empty());
    PASS();
}

int main() {
    std::cout << "=== ForceCompensation + Calibration Tests ===" << std::endl;
    test_motion_still();
    test_motion_moving();
    test_comp_uncalibrated_refuses();
    test_comp_gravity_only();
    test_comp_gravity_goes_through_A();
    test_comp_moment_is_cross_of_Ag();
    test_guard_reference_is_the_current_derived_channel();
    test_guard_passes_when_consistent();
    test_guard_refuses_when_inconsistent();
    test_guard_two_causes_are_distinguishable();
    // ★ Task 7: 参考量不可用时 fail-closed, 且与"不一致"分得开 (一条用例钉两件事)
    test_guard_reference_unavailable_does_not_pass();
    test_guard_unavailable_first_refusal_prints_the_two_values();
    test_guard_error_code_mapping();
    test_guard_fz_reported_but_not_voted();
    test_guard_moment_channel_reports_but_does_not_vote();
    test_guard_force_channels_still_vote();
    test_guard_nonfinite_refuses_even_on_nonvoting_channel();
    test_guard_recovers_after_a_nonfinite_frame();
    test_guard_transition_always_prints_block_repeat_is_throttled();
    test_guard_default_report_claims_no_mask();
    test_guard_ema_needs_sustained_mismatch();
    test_zero_only_no_motion();
    test_zero_abort_not_applied();
    test_sweep_still_enters_motion();
    test_zero_restartable();
    test_setcalib_rejects_zero_and_degenerate_A();
    test_calib_file_roundtrip();
    test_calib_file_rejects_old_format();
    test_calib_file_rejects_no_version();
    test_calib_file_rejects_truncated();
    test_calib_file_rejects_unusable_A();
    test_save_writes_unusable_model_but_load_rejects();
    // 启动零偏漂移检查在闸门【放行】后的分支 (此前一个分支都测不到, 见定义处注释)
    test_zero_drift_normal_is_a_conclusion();
    test_zero_drift_over_threshold_is_a_conclusion();
    test_zero_drift_exactly_at_threshold_is_normal();
    test_zero_drift_not_done_after_full_wait();
    test_zero_drift_waiting_stays_silent();
    test_zero_drift_insufficient_samples_speaks();
    // ⚠ 最后一条: 它会调 ForceCompensation::init() (模拟重启), 别让它影响上面任何用例。
    test_install_then_reload_roundtrip();
    std::cout << "\nResults: " << g_passed << " passed, " << g_failed << " failed" << std::endl;
    return g_failed ? 1 : 0;
}
