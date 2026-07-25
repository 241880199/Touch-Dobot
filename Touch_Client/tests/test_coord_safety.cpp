// Standalone test: CoordinateTransform + SafetyBoundary
// Build: cl /EHsc /std:c++17 /I"D:\Projects\Touch\OpenHaptics\Developer\3.5.0\include"
//        /I"D:\Projects\Touch\OpenHaptics\Developer\3.5.0\utilities\include"
//        /DWIN32 /DNOMINMAX /D_CRT_SECURE_NO_WARNINGS
//        test_coord_safety.cpp /Fe:test_coord_safety.exe
// Run: test_coord_safety.exe

#include <iostream>
#include <cmath>
#include <cstdio>
#include <HDU/hduVector.h>

// Pull in real implementations (CoordTransform is header-only, SafetyBoundary too)
#include "../config/Config.h"
#include "../core/MathUtils.h"
#include "../relay/CoordinateTransform.h"
#include "../relay/SafetyBoundary.h"

static int g_passed = 0, g_failed = 0;

#define TEST(name) do { std::cout << "  " << #name << "... "; } while(0)
#define PASS() do { std::cout << "PASS" << std::endl; g_passed++; } while(0)
#define CHECK(cond) do { if (!(cond)) { std::cout << "FAIL: " << #cond << std::endl; g_failed++; return; } } while(0)
#define CHECK_CLOSE(a, b, tol) do { if (fabs((a)-(b)) > (tol)) { std::cout << "FAIL: |" << #a << "-" << #b << "| = " << fabs((a)-(b)) << " > " << tol << std::endl; g_failed++; return; } } while(0)

// ============================================================
// CoordinateTransform tests
// ============================================================

// --- convertTouchToRobot (double[3]) ---

static void test_ct_identity_origin() {
    TEST(ct_identity_origin);
    double dev[3] = {0.0, 0.0, 0.0};
    Vec3 r = convertTouchToRobot(dev);
    CHECK_CLOSE(r.x, 0.0, 1e-9);
    CHECK_CLOSE(r.y, 0.0, 1e-9);
    CHECK_CLOSE(r.z, 0.0, 1e-9);
    PASS();
}

static void test_ct_basic_mapping() {
    TEST(ct_basic_mapping);
    // Touch(X,Y,Z) → Robot(X, -Z, Y)
    double dev[3] = {10.0, 20.0, 30.0};
    Vec3 r = convertTouchToRobot(dev);
    CHECK_CLOSE(r.x, 10.0, 1e-9);   // X → X
    CHECK_CLOSE(r.y, -30.0, 1e-9);  // -Z → Y
    CHECK_CLOSE(r.z, 20.0, 1e-9);   // Y → Z
    PASS();
}

static void test_ct_all_negative() {
    TEST(ct_all_negative);
    double dev[3] = {-5.0, -15.0, -25.0};
    Vec3 r = convertTouchToRobot(dev);
    CHECK_CLOSE(r.x, -5.0, 1e-9);
    CHECK_CLOSE(r.y, 25.0, 1e-9);   // -(-25) = 25
    CHECK_CLOSE(r.z, -15.0, 1e-9);
    PASS();
}

static void test_ct_mixed_signs() {
    TEST(ct_mixed_signs);
    double dev[3] = {100.0, -50.0, 75.0};
    Vec3 r = convertTouchToRobot(dev);
    CHECK_CLOSE(r.x, 100.0, 1e-9);
    CHECK_CLOSE(r.y, -75.0, 1e-9);  // -75
    CHECK_CLOSE(r.z, -50.0, 1e-9);
    PASS();
}

// --- convertTouchToRobot (hduVector3Dd) ---

static void test_ct_hdu_vector() {
    TEST(ct_hdu_vector);
    hduVector3Dd dev(1.0, 2.0, 3.0);
    Vec3 r = convertTouchToRobot(dev);
    CHECK_CLOSE(r.x, 1.0, 1e-9);
    CHECK_CLOSE(r.y, -3.0, 1e-9);
    CHECK_CLOSE(r.z, 2.0, 1e-9);
    PASS();
}

// --- computeDelta ---

static void test_delta_positive() {
    TEST(delta_positive);
    Vec3 cur(50, 60, 70);
    Vec3 base(10, 20, 30);
    Vec3 d = computeDelta(cur, base);
    CHECK_CLOSE(d.x, 40.0, 1e-9);
    CHECK_CLOSE(d.y, 40.0, 1e-9);
    CHECK_CLOSE(d.z, 40.0, 1e-9);
    PASS();
}

static void test_delta_negative() {
    TEST(delta_negative);
    Vec3 cur(5, 5, 5);
    Vec3 base(10, 10, 10);
    Vec3 d = computeDelta(cur, base);
    CHECK_CLOSE(d.x, -5.0, 1e-9);
    CHECK_CLOSE(d.y, -5.0, 1e-9);
    CHECK_CLOSE(d.z, -5.0, 1e-9);
    PASS();
}

static void test_delta_zero() {
    TEST(delta_zero);
    Vec3 cur(42, 42, 42);
    Vec3 d = computeDelta(cur, cur);
    CHECK_CLOSE(d.x, 0.0, 1e-9);
    CHECK_CLOSE(d.y, 0.0, 1e-9);
    CHECK_CLOSE(d.z, 0.0, 1e-9);
    PASS();
}

// --- computeTarget ---

static void test_target_basic() {
    TEST(target_basic);
    Vec3 base(100, 200, 300);
    Vec3 delta(10, -20, 30);
    Vec3 t = computeTarget(base, delta);
    CHECK_CLOSE(t.x, 110.0, 1e-9);
    CHECK_CLOSE(t.y, 180.0, 1e-9);
    CHECK_CLOSE(t.z, 330.0, 1e-9);
    PASS();
}

static void test_target_zero_delta() {
    TEST(target_zero_delta);
    Vec3 base(-100, 0, 400);
    Vec3 delta(0, 0, 0);
    Vec3 t = computeTarget(base, delta);
    CHECK_CLOSE(t.x, base.x, 1e-9);
    CHECK_CLOSE(t.y, base.y, 1e-9);
    CHECK_CLOSE(t.z, base.z, 1e-9);
    PASS();
}

// --- roundtrip ---

static void test_ct_roundtrip() {
    TEST(ct_roundtrip);
    // Simulate real pipeline:
    // 1. Read Touch device coords → 2. Convert to robot coords
    // 3. Compute delta from touch base point → 4. Compute target = robotBase + delta
    double touchPos[3] = {20.0, 15.0, -30.0};
    double touchBase[3] = {10.0, 5.0, -20.0};

    Vec3 robotPos = convertTouchToRobot(touchPos);       // (20, 30, 15)
    Vec3 robotBaseTouch = convertTouchToRobot(touchBase); // (10, 20, 5)

    Vec3 delta = computeDelta(robotPos, robotBaseTouch);  // (10, 10, 10)

    Vec3 robotBaseActual(150, -153, 381);
    Vec3 target = computeTarget(robotBaseActual, delta);  // (160, -143, 391)

    CHECK_CLOSE(target.x, 160.0, 1e-9);
    CHECK_CLOSE(target.y, -143.0, 1e-9);
    CHECK_CLOSE(target.z, 391.0, 1e-9);
    PASS();
}

// ============================================================
// SafetyBoundary tests
// ============================================================

// --- clampToBoundary ---

static void test_sb_inside_no_clamp() {
    TEST(sb_inside_no_clamp);
    Vec3 target(0, 0, 300);  // well within bounds
    Vec3 clamped = SafetyBoundary::clampToBoundary(target);
    CHECK_CLOSE(clamped.x, target.x, 1e-9);
    CHECK_CLOSE(clamped.y, target.y, 1e-9);
    CHECK_CLOSE(clamped.z, target.z, 1e-9);
    // No clamping = identity
    CHECK(clamped.x == target.x && clamped.y == target.y && clamped.z == target.z);
    PASS();
}

static void test_sb_x_exceeds_max() {
    TEST(sb_x_exceeds_max);
    Vec3 target(999, 0, 300);
    Vec3 clamped = SafetyBoundary::clampToBoundary(target);
    CHECK_CLOSE(clamped.x, Config::SAFE_X_MAX, 1e-9);
    CHECK_CLOSE(clamped.y, target.y, 1e-9);
    CHECK_CLOSE(clamped.z, target.z, 1e-9);
    PASS();
}

static void test_sb_x_below_min() {
    TEST(sb_x_below_min);
    Vec3 target(-999, 0, 300);
    Vec3 clamped = SafetyBoundary::clampToBoundary(target);
    CHECK_CLOSE(clamped.x, Config::SAFE_X_MIN, 1e-9);
    PASS();
}

static void test_sb_y_exceeds_max() {
    TEST(sb_y_exceeds_max);
    Vec3 target(0, 999, 300);
    Vec3 clamped = SafetyBoundary::clampToBoundary(target);
    CHECK_CLOSE(clamped.y, Config::SAFE_Y_MAX, 1e-9);
    PASS();
}

static void test_sb_y_below_min() {
    TEST(sb_y_below_min);
    Vec3 target(0, -999, 300);
    Vec3 clamped = SafetyBoundary::clampToBoundary(target);
    CHECK_CLOSE(clamped.y, Config::SAFE_Y_MIN, 1e-9);
    PASS();
}

static void test_sb_z_exceeds_max() {
    TEST(sb_z_exceeds_max);
    Vec3 target(0, 0, 999);
    Vec3 clamped = SafetyBoundary::clampToBoundary(target);
    CHECK_CLOSE(clamped.z, Config::SAFE_Z_MAX, 1e-9);
    PASS();
}

static void test_sb_z_below_min() {
    TEST(sb_z_below_min);
    Vec3 target(0, 0, -999);
    Vec3 clamped = SafetyBoundary::clampToBoundary(target);
    CHECK_CLOSE(clamped.z, Config::SAFE_Z_MIN, 1e-9);
    PASS();
}

static void test_sb_multiple_axes_clamped() {
    TEST(sb_multiple_axes_clamped);
    Vec3 target(999, -999, 999);  // all three axes out of bounds
    Vec3 clamped = SafetyBoundary::clampToBoundary(target);
    CHECK_CLOSE(clamped.x, Config::SAFE_X_MAX, 1e-9);
    CHECK_CLOSE(clamped.y, Config::SAFE_Y_MIN, 1e-9);
    CHECK_CLOSE(clamped.z, Config::SAFE_Z_MAX, 1e-9);
    PASS();
}

static void test_sb_at_exact_boundary() {
    TEST(sb_at_exact_boundary);
    Vec3 target(Config::SAFE_X_MAX, Config::SAFE_Y_MIN, Config::SAFE_Z_MAX);
    Vec3 clamped = SafetyBoundary::clampToBoundary(target);
    // Exactly at boundary should not be clamped (no strict inequality violation)
    CHECK_CLOSE(clamped.x, target.x, 1e-9);
    CHECK_CLOSE(clamped.y, target.y, 1e-9);
    CHECK_CLOSE(clamped.z, target.z, 1e-9);
    PASS();
}

// --- computeSpeedFactor ---

static void test_sf_center_full_speed() {
    TEST(sf_center_full_speed);
    // Center of safety bounds
    double cx = (Config::SAFE_X_MIN + Config::SAFE_X_MAX) / 2.0;
    double cy = (Config::SAFE_Y_MIN + Config::SAFE_Y_MAX) / 2.0;
    double cz = (Config::SAFE_Z_MIN + Config::SAFE_Z_MAX) / 2.0;
    Vec3 target(cx, cy, cz);
    double sf = SafetyBoundary::computeSpeedFactor(target);
    CHECK_CLOSE(sf, 1.0, 1e-9);
    PASS();
}

static void test_sf_near_xmax_boundary() {
    TEST(sf_near_xmax_boundary);
    // Place target within buffer zone: 10% of half-range from X_MAX
    // half-range = (250 - (-300)) / 2 = 275
    // buffer zone starts at 20% of half-range = 55mm from boundary
    // 10% of half-range = 27.5mm → well within buffer zone
    double halfRange = (Config::SAFE_X_MAX - Config::SAFE_X_MIN) / 2.0;
    double cy = (Config::SAFE_Y_MIN + Config::SAFE_Y_MAX) / 2.0;
    double cz = (Config::SAFE_Z_MIN + Config::SAFE_Z_MAX) / 2.0;
    Vec3 target(Config::SAFE_X_MAX - halfRange * 0.10, cy, cz);
    double sf = SafetyBoundary::computeSpeedFactor(target);
    // distX = 0.10, bufferRatio = 0.20, so sf = 0.10/0.20 = 0.5
    CHECK(sf > 0.0 && sf < 1.0);
    CHECK_CLOSE(sf, 0.5, 0.01);
    PASS();
}

static void test_sf_at_boundary_min_speed() {
    TEST(sf_at_boundary_min_speed);
    // Exactly at boundary → minDist = 0 → speedFactor = 0.1
    Vec3 target(Config::SAFE_X_MAX,
                (Config::SAFE_Y_MIN + Config::SAFE_Y_MAX) / 2.0,
                (Config::SAFE_Z_MIN + Config::SAFE_Z_MAX) / 2.0);
    double sf = SafetyBoundary::computeSpeedFactor(target);
    CHECK_CLOSE(sf, 0.1, 1e-9);
    PASS();
}

static void test_sf_beyond_boundary_min_speed() {
    TEST(sf_beyond_boundary_min_speed);
    // Beyond boundary (should have been clamped, but computeSpeedFactor still works)
    Vec3 target(9999, 0, 300);
    double sf = SafetyBoundary::computeSpeedFactor(target);
    // minDist ≤ 0 → returns 0.1
    CHECK_CLOSE(sf, 0.1, 1e-9);
    PASS();
}

static void test_sf_half_buffer() {
    TEST(sf_half_buffer);
    // At 50% of buffer zone → speedFactor = 0.5
    double rangeX = Config::SAFE_X_MAX - Config::SAFE_X_MIN;
    double halfBuf = Config::SAFE_BOUNDARY_BUFFER_RATIO * 0.5 * rangeX * 0.5;
    Vec3 target(Config::SAFE_X_MAX - halfBuf,
                (Config::SAFE_Y_MIN + Config::SAFE_Y_MAX) / 2.0,
                (Config::SAFE_Z_MIN + Config::SAFE_Z_MAX) / 2.0);
    double sf = SafetyBoundary::computeSpeedFactor(target);
    // minDist / bufferRatio ≈ 0.5
    CHECK_CLOSE(sf, 0.5, 0.05);  // 5% tolerance due to integer coords
    PASS();
}

// ============================================================
// Vec3 operations (exercised by above, but test edge cases)
// ============================================================

static void test_vec3_length() {
    TEST(vec3_length);
    Vec3 v(3, 4, 0);
    CHECK_CLOSE(v.length(), 5.0, 1e-9);
    Vec3 v2(1, 2, 3);
    CHECK_CLOSE(v2.length(), sqrt(14.0), 1e-9);
    PASS();
}

static void test_vec3_operators() {
    TEST(vec3_operators);
    Vec3 a(1, 2, 3);
    Vec3 b(4, 5, 6);
    Vec3 sum = a + b;
    CHECK_CLOSE(sum.x, 5.0, 1e-9);
    CHECK_CLOSE(sum.y, 7.0, 1e-9);
    CHECK_CLOSE(sum.z, 9.0, 1e-9);
    Vec3 diff = a - b;
    CHECK_CLOSE(diff.x, -3.0, 1e-9);
    CHECK_CLOSE(diff.y, -3.0, 1e-9);
    CHECK_CLOSE(diff.z, -3.0, 1e-9);
    PASS();
}

// ============================================================
int main() {
    std::cout << "=== CoordinateTransform + SafetyBoundary Tests ===" << std::endl;

    test_ct_identity_origin();
    test_ct_basic_mapping();
    test_ct_all_negative();
    test_ct_mixed_signs();
    test_ct_hdu_vector();
    test_delta_positive();
    test_delta_negative();
    test_delta_zero();
    test_target_basic();
    test_target_zero_delta();
    test_ct_roundtrip();

    test_sb_inside_no_clamp();
    test_sb_x_exceeds_max();
    test_sb_x_below_min();
    test_sb_y_exceeds_max();
    test_sb_y_below_min();
    test_sb_z_exceeds_max();
    test_sb_z_below_min();
    test_sb_multiple_axes_clamped();
    test_sb_at_exact_boundary();
    test_sf_center_full_speed();
    test_sf_near_xmax_boundary();
    test_sf_at_boundary_min_speed();
    test_sf_beyond_boundary_min_speed();
    test_sf_half_buffer();

    test_vec3_length();
    test_vec3_operators();

    std::cout << std::endl;
    std::cout << "Results: " << g_passed << " passed, " << g_failed << " failed" << std::endl;
    return g_failed > 0 ? 1 : 0;
}
