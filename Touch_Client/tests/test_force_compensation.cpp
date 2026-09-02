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

int main() {
    std::cout << "=== ForceCompensation + Calibration Tests ===" << std::endl;
    test_motion_still();
    test_motion_moving();
    test_comp_no_calib();
    test_comp_gravity_only();
    std::cout << "\nResults: " << g_passed << " passed, " << g_failed << " failed" << std::endl;
    return g_failed ? 1 : 0;
}
