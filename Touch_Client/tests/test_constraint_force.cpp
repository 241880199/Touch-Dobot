// Standalone test: ConstraintForce virtual force fields
// Build: cl /EHsc /std:c++17 test_constraint_force.cpp ../safety/ConstraintForce.cpp
//        /I"..\..\OpenHaptics\Developer\3.5.0\include"
//        /I"..\..\OpenHaptics\Developer\3.5.0\utilities\include"
//        /Fe:test_constraint_force.exe
// Run: test_constraint_force.exe

#include <iostream>
#include <cassert>
#include <cmath>

#define M_PI 3.14159265358979323846

// Include project headers
#include "../safety/ConstraintForce.h"
#include "../safety/SafetyPredictor.h"
#include "../config/Config.h"

static int g_passed = 0, g_failed = 0;

#define TEST(name) do { std::cout << "  " << #name << "... "; } while(0)
#define PASS() do { std::cout << "PASS" << std::endl; g_passed++; } while(0)
#define CHECK(cond) do { if (!(cond)) { std::cout << "FAIL: " << #cond << std::endl; g_failed++; return; } } while(0)

// ===== Test 1: Boundary force = 0 at center =====
static void test_boundary_force_zero_at_center() {
    TEST(boundary_zero_at_center);
    double f[3];
    Vec3 target = {0, 0, 300};  // well within all boundaries
    ConstraintForce::computeBoundaryForce(target, f);
    CHECK(fabs(f[0]) < 0.01 && fabs(f[1]) < 0.01 && fabs(f[2]) < 0.01);
    PASS();
}

// ===== Test 2: Boundary force pushes inward near X_MAX =====
static void test_boundary_force_near_xmax() {
    TEST(boundary_near_xmax);
    double f[3];
    double xNear = Config::SAFE_X_MAX - 10.0;  // 10mm from X_MAX, well within 50mm range
    Vec3 target = { xNear, 0, 300 };
    ConstraintForce::computeBoundaryForce(target, f);
    // Force should push toward -X (away from boundary)
    CHECK(f[0] < -0.1);  // negative X force
    CHECK(fabs(f[1]) < 0.01);
    CHECK(fabs(f[2]) < 0.01);
    PASS();
}

// ===== Test 3: Boundary force pushes inward near X_MIN =====
static void test_boundary_force_near_xmin() {
    TEST(boundary_near_xmin);
    double f[3];
    double xNear = Config::SAFE_X_MIN + 10.0;
    Vec3 target = { xNear, 0, 300 };
    ConstraintForce::computeBoundaryForce(target, f);
    // Force should push toward +X (away from boundary)
    CHECK(f[0] > 0.1);  // positive X force
    PASS();
}

// ===== Test 4: Singular force points radially outward =====
static void test_singular_force_radial() {
    TEST(singular_radial);
    double f[3];
    Vec3 target = { 10, 0, 300 };  // r_xy=10mm < 80mm range
    ConstraintForce::computeSingularForce(target, f);
    // Force should point radially outward: +X direction
    CHECK(f[0] > 0.1);   // radial outward is +X
    CHECK(fabs(f[1]) < 0.01);
    CHECK(fabs(f[2]) < 0.01);
    PASS();
}

// ===== Test 5: Singular force = 0 far from Z axis =====
static void test_singular_force_zero_far() {
    TEST(singular_zero_far);
    double f[3];
    Vec3 target = { 200, 0, 300 };  // r_xy=200mm > 80mm range
    ConstraintForce::computeSingularForce(target, f);
    CHECK(fabs(f[0]) < 0.01 && fabs(f[1]) < 0.01 && fabs(f[2]) < 0.01);
    PASS();
}

// ===== Test 6: Total force clamped to 3.3N =====
static void test_total_force_clamped() {
    TEST(total_clamped);
    double f[3];
    // Position that triggers all force fields simultaneously
    Vec3 target = { 580, 0, 5 };  // near workspace edge + low Z
    ConstraintForce::computeTotalForce(target, {}, f);
    double mag = sqrt(f[0]*f[0] + f[1]*f[1] + f[2]*f[2]);
    CHECK(mag <= Config::FORCE_MAX_TOUCH_N + 0.01);
    PASS();
}

// ===== Test 7: Total force = 0 in safe zone =====
static void test_total_force_zero_safe() {
    TEST(total_zero_safe);
    double f[3];
    Vec3 target = { 0, 0, 300 };  // center of workspace
    ConstraintForce::computeTotalForce(target, {}, f);
    double mag = sqrt(f[0]*f[0] + f[1]*f[1] + f[2]*f[2]);
    CHECK(mag < 0.01);
    PASS();
}

int main() {
    std::cout << "=== ConstraintForce Unit Tests ===" << std::endl;
    test_boundary_force_zero_at_center();
    test_boundary_force_near_xmax();
    test_boundary_force_near_xmin();
    test_singular_force_radial();
    test_singular_force_zero_far();
    test_total_force_clamped();
    test_total_force_zero_safe();
    std::cout << "\nResults: " << g_passed << " passed, " << g_failed << " failed" << std::endl;
    return g_failed ? 1 : 0;
}
