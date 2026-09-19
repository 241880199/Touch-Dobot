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
#include <string>
#include <windows.h>
#include "../force/ForceCompensation.h"
#include "../force/ForceCalibration.h"
#include "../calibration/TcpCalibration.h"
#include "../config/Config.h"

// 3×3 对角 A = m·I: 这是【残余模型】在新模型里的等价物 (各向同性、无安装旋转), 用来
// 把"换通道/换写法"与"换模型本身"分开 —— 用它能逐位复算出旧测试的期望值。
static void diagA(double m, double A[9]) {
    for (int i = 0; i < 9; i++) A[i] = 0.0;
    A[0] = A[4] = A[8] = m;
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
    AppState::ForceData fd;
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

    AppState::ForceData fd;
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

    AppState::ForceData fd;
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
    AppState::ForceData fd2;
    fd2.sixForceRaw[0] = 9.81;
    ForceCompensation::step(fd2, pose2);
    CHECK(fabs(fd2.compensated[0]) < 1e-9);

    // 绕 y 转 90°: 期望值【由共享的重力函数现算】, 不在这里另写一份约定
    // (重力的约定只能有一份实现 —— 测试里再展开一遍正是历史上安静解错的样子)。
    double pose3[6] = {0, 0, 0, 0, 90.0, 0};
    double g[3];
    TcpCalibration::gravitySensorFrameAtYaw(pose3, 0.0, g);
    CHECK(fabs(g[2]) < 1e-9);          // Ry(90) 把 g 转到水平面内 -> g_z = 0
    AppState::ForceData fd3;
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
    AppState::ForceData fd;
    fd.sixForceRaw[4] = 0.4905;
    // 一致性闸门要求 fd.raw (@576) 与本地模型给的是同一个数 —— 这一帧给它们相等,
    // 于是闸门放行, 下面量的才是【补偿结果】本身, 而不是被闸门置零的 0。
    fd.raw[4] = 0.4905;
    ForceCompensation::step(fd, pose);
    CHECK(fabs(fd.compensated[4] - 0.4905) < 1e-9);

    // (b) c_s 沿 x: c_s × w = (0.05,0,0) × (0,0,9.81) = (0, −0.4905, 0)
    //     读数正好是这个力矩 -> 被减干净。这一条同时钉住【叉乘的顺序/符号】:
    //     写成 w × c_s 会得到 +0.4905, 于是补偿后的读数变成 −0.981 而不是 0。
    double cS_x[3] = {0.05, 0.0, 0.0};
    ForceCompensation::setCalibration(A, bF, bM, cS_x);
    AppState::ForceData fd2;
    fd2.sixForceRaw[4] = -0.4905;
    ForceCompensation::step(fd2, pose);
    CHECK(fabs(fd2.compensated[4]) < 1e-9);
    CHECK(fabs(fd2.compensated[3]) < 1e-9);
    CHECK(fabs(fd2.compensated[5]) < 1e-9);
    PASS();
}

// ===== 运行时一致性闸门 (2026-09-19, 用户指令 1/2/3) =====
//
// 判据: 本地全量模型的输出 compensated 与机械臂自报的 @576 (fd.raw) 逐通道比较。
// 两个模型都对时它们估计的是同一个量 (外力), 所以应当一致; 不一致 ⇒ 至少一个错 ⇒
// compensated 全 6 个分量置零 + 报错。未标定/模型不可用 ⇒ 同一处置, 但报的是另一个原因。

// 一致 -> 放行: compensated 就是模型算出来的值 (不被置零), 状态 OK。
static void test_guard_passes_when_consistent() {
    TEST(guard_passes_when_consistent);
    ForceCompensation::init();
    double A[9]; diagA(1.0, A);
    double com[3] = {0, 0, 0};
    double bF[3] = {0, 0, 0}, bM[3] = {0, 0, 0};
    ForceCompensation::setCalibration(A, bF, bM, com);

    double pose[6] = {0, 0, 0, 0, 0, 0};       // g = (0,0,9.81) -> Fg = (0,0,9.81)
    AppState::ForceData fd;
    fd.sixForceRaw[2] = 9.81;                  // 读数 = 重力 -> compensated 应为 0
    fd.raw[2] = 0.0;                           // @576 也说是 0 (两边一致)
    fd.sixForceRaw[0] = 1.25;                  // 再叠一个真实外力: 两边都必须看到它
    fd.raw[0] = 1.25;
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
    AppState::ForceData fd;
    fd.sixForceRaw[2] = 9.81;                  // 本地算出 compensated = (0,0,0)
    fd.raw[0] = 0.90;                          // @576 说 x 上有 0.9 N 的外力 (容差 0.5)
    fd.raw[1] = 0.05;                          // y 在限内
    ForceCompensation::step(fd, pose);

    CHECK(ForceCompensation::guardState() == ForceCompensation::GuardState::INCONSISTENT);
    for (int i = 0; i < 6; i++) CHECK(fabs(fd.compensated[i]) < 1e-12);   // 一个分量都不许漏
    CHECK(fd.isCalibrated == false);
    CHECK(fd.calibMassKg == 0.0);
    // 逐通道的结果要能被读出来 —— 上层 (RelayCore) 拿它去报 RobotDiagnostics。
    ForceCompensation::GuardReport rep;
    ForceCompensation::guardReport(rep);
    CHECK(rep.state == ForceCompensation::GuardState::INCONSISTENT);
    CHECK(fabs(rep.ema[0] - (-0.90)) < 1e-9);
    CHECK(rep.exceeded[0] == true);
    CHECK(rep.exceeded[1] == false);
    CHECK(rep.voted[0] == true);
    CHECK(rep.tol[0] == Config::FORCE_GUARD_TOL_FORCE_N);
    CHECK(rep.tol[3] == Config::FORCE_GUARD_TOL_MOMENT_NM);
    PASS();
}

// ★ 两种原因必须分得开 (简报硬要求): 处置一样 (都拒绝), 但操作员要做的事不同 ——
//   一个是"去标定", 一个是"去查负载参数有没有发进去"。合成一句话就会让人乱猜。
static void test_guard_two_causes_are_distinguishable() {
    TEST(guard_two_causes_are_distinguishable);

    // (甲) 没有可用模型
    ForceCompensation::init();
    AppState::ForceData fd;
    double pose[6] = {0, 0, 0, 0, 0, 0};
    ForceCompensation::step(fd, pose);
    const ForceCompensation::GuardState stA = ForceCompensation::guardState();

    // (乙) 有模型但对不上
    double A[9]; diagA(1.0, A);
    double com[3] = {0, 0, 0};
    double bF[3] = {0, 0, 0}, bM[3] = {0, 0, 0};
    ForceCompensation::setCalibration(A, bF, bM, com);
    AppState::ForceData fd2;
    fd2.raw[0] = 5.0;
    ForceCompensation::step(fd2, pose);
    const ForceCompensation::GuardState stB = ForceCompensation::guardState();

    CHECK(stA == ForceCompensation::GuardState::UNCALIBRATED);
    CHECK(stB == ForceCompensation::GuardState::INCONSISTENT);
    CHECK(stA != stB);
    // 名字也要分得开 —— 日志里落的是名字。
    CHECK(std::string(ForceCompensation::guardStateName(stA)) == "UNCALIBRATED");
    CHECK(std::string(ForceCompensation::guardStateName(stB)) == "INCONSISTENT");
    PASS();
}

// ★★ 状态 -> 错误码的映射 (2026-09-19 复审 Important 4)。
//   在此之前 RelayCore 自己拿 static_cast<int>(guardState()) 去比字面量 1 和 2, 而这条
//   映射【没有任何测试】: 给 GuardState 换个顺序, ERR_FORCE_UNCALIBRATED 与
//   ERR_FORCE_INCONSISTENT 就悄悄对调, 而用户指令 1 的全部意义就是告诉操作员
//   【该去标定还是该去查负载参数】—— 报错报反了比不报还坏。
//   现在唯一的实现是 ForceCompensation::guardErrorCode, 本用例把它的三个输入逐条钉住,
//   并顺手钉住"错误码 -> 名字 -> 严重度"这条下游链 (报告里说的"严重度现在是咨询性的"
//   也在这条链上: 这里只断言它是 REJECT, 断言不了它有没有被消费)。
static void test_guard_error_code_mapping() {
    TEST(guard_error_code_mapping);

    using ForceCompensation::GuardState;
    const RobotErrorCode cOk   = ForceCompensation::guardErrorCode(GuardState::OK);
    const RobotErrorCode cUnc  = ForceCompensation::guardErrorCode(GuardState::UNCALIBRATED);
    const RobotErrorCode cInc  = ForceCompensation::guardErrorCode(GuardState::INCONSISTENT);

    CHECK(cOk == RobotErrorCode::OK);                                  // 放行 -> 不上报
    CHECK(cUnc == RobotErrorCode::ERR_FORCE_UNCALIBRATED);
    CHECK(cInc == RobotErrorCode::ERR_FORCE_INCONSISTENT);
    // 两个码必须不同 —— 否则"两种原因分开报"这件事在日志里根本不成立。
    CHECK(cUnc != cInc);
    // 与枚举的数值索引【无关】: 这两个码在 RobotErrorCode 里的位置本来就与 GuardState 不同,
    // 所以"按 static_cast<int> 对上"这种巧合不许再被依赖。
    CHECK(static_cast<int>(cUnc) != static_cast<int>(GuardState::UNCALIBRATED));
    CHECK(static_cast<int>(cInc) != static_cast<int>(GuardState::INCONSISTENT));

    // 名字 (进 robot_diagnostics.log 与 D| 帧的那一个) 也必须对得上, 且分得开。
    CHECK(std::string(errorCodeName(cUnc)) == "ERR_FORCE_UNCALIBRATED");
    CHECK(std::string(errorCodeName(cInc)) == "ERR_FORCE_INCONSISTENT");

    // 严重度: 两个码都是 REJECT —— 而且二者一致 (给操作员看的档位不该因原因而不同)。
    CHECK(getSeverity(cUnc) == Severity::REJECT);
    CHECK(getSeverity(cInc) == Severity::REJECT);
    CHECK(getSeverity(cUnc) == getSeverity(cInc));

    // 名字函数: 三个状态都要能读出来, 且互不相同。
    CHECK(std::string(ForceCompensation::guardStateName(GuardState::OK)) == "OK");
    CHECK(std::string(ForceCompensation::guardStateName(GuardState::UNCALIBRATED)) !=
          std::string(ForceCompensation::guardStateName(GuardState::INCONSISTENT)));
    PASS();
}

// ★ Fz 【不投票】但【照报】—— @576 的 z 响应实测秩 2, 它动不了就证不了"一致"。
//   本用例钉住两件事: (a) 巨大 z 差不会让闸门拒绝 (不然就是"永远不通过");
//   (b) 它的比较结果仍然读得出来 (不然就是"静默"跳过, 简报明令禁止)。
static void test_guard_fz_reported_but_not_voted() {
    TEST(guard_fz_reported_but_not_voted);
    ForceCompensation::init();
    double A[9]; diagA(1.0, A);
    double com[3] = {0, 0, 0};
    double bF[3] = {0, 0, 0}, bM[3] = {0, 0, 0};
    ForceCompensation::setCalibration(A, bF, bM, com);

    double pose[6] = {0, 0, 0, 0, 0, 0};
    AppState::ForceData fd;
    fd.sixForceRaw[2] = 9.81;                 // compensated = (0,0,0)
    fd.raw[2] = 3.0;                          // @576 的 z 报 3 N —— 远超容差 0.5
    ForceCompensation::step(fd, pose);

    CHECK(ForceCompensation::guardState() == ForceCompensation::GuardState::OK);  // (a)
    ForceCompensation::GuardReport rep;
    ForceCompensation::guardReport(rep);
    CHECK(rep.voted[0] == true && rep.voted[1] == true);
    CHECK(rep.voted[2] == false);                                             // Fz 不投票
    CHECK(rep.voted[3] == true && rep.voted[4] == true && rep.voted[5] == true);
    CHECK(fabs(rep.ema[2] - (-3.0)) < 1e-9);                                  // (b) 照报
    CHECK(rep.exceeded[2] == false);          // 不投票的通道不算"超限"
    PASS();
}

// 力矩通道同样投票: 力矩对不上 -> 拒绝。
static void test_guard_moment_channel_votes() {
    TEST(guard_moment_channel_votes);
    ForceCompensation::init();
    double A[9]; diagA(1.0, A);
    double com[3] = {0, 0, 0};
    double bF[3] = {0, 0, 0}, bM[3] = {0, 0, 0};
    ForceCompensation::setCalibration(A, bF, bM, com);

    double pose[6] = {0, 0, 0, 0, 0, 0};
    AppState::ForceData fd;
    fd.sixForceRaw[2] = 9.81;
    fd.sixForceRaw[3] = 0.20;                 // 本地 compensated[3] = 0.20
    fd.raw[3] = 0.20;                         // 力矩在限内 -> 先放行
    ForceCompensation::step(fd, pose);
    CHECK(ForceCompensation::guardState() == ForceCompensation::GuardState::OK);

    // 重新装一次模型 = 清掉上一帧的 EMA (否则判决里混着上一段的平均值)。
    ForceCompensation::init();
    ForceCompensation::setCalibration(A, bF, bM, com);
    AppState::ForceData fd2;
    fd2.sixForceRaw[2] = 9.81;
    fd2.sixForceRaw[3] = 0.20;
    fd2.raw[3] = 0.20 - 0.12;                 // 差 0.12 > 容差 0.03 -> 拒绝
    ForceCompensation::step(fd2, pose);
    CHECK(ForceCompensation::guardState() == ForceCompensation::GuardState::INCONSISTENT);
    for (int i = 0; i < 6; i++) CHECK(fabs(fd2.compensated[i]) < 1e-12);
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
    AppState::ForceData fd;
    fd.sixForceRaw[2] = 9.81;

    // 第 1 帧: 瞬时差 0.30 N, 在容差 0.50 之内 -> 放行 (EMA 由第 1 帧播种)。
    fd.raw[0] = -0.30;
    ForceCompensation::step(fd, pose);
    CHECK(ForceCompensation::guardState() == ForceCompensation::GuardState::OK);

    // 之后持续 0.90 N: EMA 从 0.30 爬向 0.90, alpha = 0.02 —— 第 5 帧还不够, 第 40 帧够了。
    fd.raw[0] = -0.90;
    for (int k = 0; k < 4; k++) ForceCompensation::step(fd, pose);
    CHECK(ForceCompensation::guardState() == ForceCompensation::GuardState::OK);
    for (int k = 0; k < 36; k++) ForceCompensation::step(fd, pose);
    CHECK(ForceCompensation::guardState() == ForceCompensation::GuardState::INCONSISTENT);

    ForceCompensation::GuardReport rep;
    ForceCompensation::guardReport(rep);
    CHECK(rep.ema[0] > Config::FORCE_GUARD_TOL_FORCE_N);
    CHECK(rep.frames == 41);
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
    // fd.raw (@576) 填成"机械臂也给出同一个外力"的样子 —— 闸门放行才量得到补偿结果。
    // 期望值 (A = 0.42·I, g = (0,0,9.81) -> Fg = (0,0,4.1202), c_s 沿 z -> Mg = 0):
    //   x: 5 − (−0.48)          = 5.48
    //   y: 5 − (−1.35)          = 6.35
    //   z: 5 − (−0.02) − 4.1202 = 0.8998
    //   M: 5 − (0.010, −0.020, 0.005)
    AppState::ForceData fd;
    for (int i = 0; i < 6; i++) fd.sixForceRaw[i] = 5.0;
    fd.raw[0] = 5.48; fd.raw[1] = 6.35; fd.raw[2] = 0.90;
    fd.raw[3] = 4.99; fd.raw[4] = 5.02; fd.raw[5] = 4.995;
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
    AppState::ForceData fd;
    for (int i = 0; i < 6; i++) fd.sixForceRaw[i] = 5.0;
    fd.raw[0] = 4.0; fd.raw[1] = 3.0; fd.raw[2] = 5.0 - 3.0 - 4.1202;
    fd.raw[3] = 4.9; fd.raw[4] = 4.8; fd.raw[5] = 4.7;
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
    AppState::ForceData fd;
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

int main() {
    std::cout << "=== ForceCompensation + Calibration Tests ===" << std::endl;
    test_motion_still();
    test_motion_moving();
    test_comp_uncalibrated_refuses();
    test_comp_gravity_only();
    test_comp_gravity_goes_through_A();
    test_comp_moment_is_cross_of_Ag();
    test_guard_passes_when_consistent();
    test_guard_refuses_when_inconsistent();
    test_guard_two_causes_are_distinguishable();
    test_guard_error_code_mapping();
    test_guard_fz_reported_but_not_voted();
    test_guard_moment_channel_votes();
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
    std::cout << "\nResults: " << g_passed << " passed, " << g_failed << " failed" << std::endl;
    return g_failed ? 1 : 0;
}
