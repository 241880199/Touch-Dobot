// Standalone test: ForceCompensation + ForceCalibration core logic
#include <iostream>
#include <cassert>
#include <cmath>
#include <windows.h>
#include "../force/ForceCompensation.h"
#include "../force/ForceCalibration.h"
#include "../config/Config.h"

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

static void test_comp_no_calib() {
    TEST(comp_no_calib);
    ForceCompensation::init();
    AppState::ForceData fd;
    fd.raw[0] = -0.65; fd.raw[1] = -1.07; fd.raw[2] = 0.055;
    fd.raw[3] = -0.02; fd.raw[4] = 0.02;  fd.raw[5] = 0.005;

    // No calibration -> compensated should mirror raw
    double pose[6] = {0, 0, 0, 0, 0, 0};
    ForceCompensation::step(fd, pose);
    CHECK(fabs(fd.compensated[0] - (-0.65)) < 0.01);
    CHECK(fabs(fd.compensated[1] - (-1.07)) < 0.01);
    CHECK(fd.isCalibrated == false);
    PASS();
}

static void test_comp_gravity_only() {
    TEST(comp_gravity_only);
    ForceCompensation::init();
    // Calibrate: mass=1kg, com at origin, zero bias
    double mass = 1.0;
    double com[3] = {0, 0, 0};
    double biasF[3] = {0, 0, 0};
    double biasM[3] = {0, 0, 0};
    ForceCompensation::setCalibration(mass, com, biasF, biasM);

    AppState::ForceData fd;
    // Tool pointing straight down: Rx=0, Ry=0, Rz=0 -> g_tool = (0, 0, -9.81)
    // Expected: Fz sensor reads +9.81 (supporting weight), gravity comp subtracts it -> 0
    fd.raw[0] = 0.0; fd.raw[1] = 0.0; fd.raw[2] = 9.81;
    fd.raw[3] = 0.0; fd.raw[4] = 0.0; fd.raw[5] = 0.0;

    double pose[6] = {0, 0, 0, 0, 0, 0};
    ForceCompensation::step(fd, pose);
    CHECK(fabs(fd.compensated[2]) < 0.1); // gravity compensated away
    CHECK(fd.isCalibrated == true);
    PASS();
}

// ===== 调零 (TARE only) 测试 =====

static int g_dragOnCalls = 0;   // 拖拽模式被「开启」的次数 — 调零流程必须为 0

static void countDrag(bool enable) {
    if (enable) g_dragOnCalls++;
}

// 调零: 静置采集 → 直接应用零偏 + 存盘; 不开拖拽、不进 MOTION、保留惯性质量
static void test_zero_only_no_motion() {
    TEST(zero_only_no_motion);
    ForceCompensation::init();
    double com[3] = {0, 0, 0};
    double bF[3] = {0, 0, 0}, bM[3] = {0, 0, 0};
    ForceCompensation::setCalibration(0.42, com, bF, bM);   // 预置惯性质量

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
    CHECK(fabs(ForceCompensation::currentMassKg() - 0.42) < 1e-9);  // 质量保留

    // 零偏已生效。pose 全 0 → 重力项 = m*g 只在 Z 轴, X/Y 纯看零偏。
    AppState::ForceData fd;
    for (int i = 0; i < 6; i++) fd.raw[i] = 5.0;
    ForceCompensation::step(fd, pose);
    CHECK(fabs(fd.compensated[0] - (5.0 - (-0.48))) < 0.02);
    CHECK(fabs(fd.compensated[1] - (5.0 - (-1.35))) < 0.02);
    // Z: 5.0 - (-0.02) - 0.42*9.81 ≈ 0.90 — 同时证明保留的质量确实进了 setCalibration
    CHECK(fabs(fd.compensated[2] - 0.90) < 0.05);
    // 力矩零偏也已应用 (Mx 零偏 0.010); com=0 → 重力力矩为 0
    CHECK(fabs(fd.compensated[3] - (5.0 - 0.010)) < 0.02);

    ForceCalibration::setDragModeCallback(nullptr);
    PASS();
}

// 调零中止: 未采集满就中止 → 零偏不应用、不存盘
static void test_zero_abort_not_applied() {
    TEST(zero_abort_not_applied);
    ForceCompensation::init();
    double com[3] = {0, 0, 0};
    double bF[3] = {0, 0, 0}, bM[3] = {0, 0, 0};
    ForceCompensation::setCalibration(0.0, com, bF, bM);   // 质量 0 → 无重力项

    CHECK(ForceCalibration::startZero());
    double raw[6] = {-0.48, -1.35, -0.02, 0.010, -0.020, 0.005};
    double pose[6] = {0, 0, 0, 0, 0, 0};
    ForceCalibration::update(0.5, raw, pose);   // 只采 0.5s, 未达阈值
    ForceCalibration::abort();

    CHECK(!ForceCalibration::isRunning());
    CHECK(!ForceCalibration::isZeroing());

    AppState::ForceData fd;
    for (int i = 0; i < 6; i++) fd.raw[i] = 5.0;
    ForceCompensation::step(fd, pose);
    CHECK(fabs(fd.compensated[0] - 5.0) < 0.02);   // 零偏未被应用
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
    double com[3] = {0, 0, 0};
    double bF[3] = {0, 0, 0}, bM[3] = {0, 0, 0};
    ForceCompensation::setCalibration(0.0, com, bF, bM);

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

int main() {
    std::cout << "=== ForceCompensation + Calibration Tests ===" << std::endl;
    test_motion_still();
    test_motion_moving();
    test_comp_no_calib();
    test_comp_gravity_only();
    test_zero_only_no_motion();
    test_zero_abort_not_applied();
    test_sweep_still_enters_motion();
    test_zero_restartable();
    std::cout << "\nResults: " << g_passed << " passed, " << g_failed << " failed" << std::endl;
    return g_failed ? 1 : 0;
}
