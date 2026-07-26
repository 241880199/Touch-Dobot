// Standalone test: Kabsch-Umeyama solver
// No robot, no Touch, no OpenHaptics needed — pure math verification
// Build: tests\build_calibration_test.bat
// Run: test_calibration.exe

#include <iostream>
#include <cmath>
#include <vector>
#include <utility>
#include "../calibration/CalibrationSolver.h"

static int g_passed = 0, g_failed = 0;

#define TEST(name) do { std::cout << "  " << #name << "... "; } while(0)
#define PASS() do { std::cout << "PASS" << std::endl; g_passed++; } while(0)
#define CHECK(cond) do { if (!(cond)) { std::cout << "FAIL: " << #cond << std::endl; g_failed++; return; } } while(0)
#define CHECK_CLOSE(a, b, tol) do { \
    double _a = (a), _b = (b); \
    if (fabs(_a - _b) > (tol)) { \
        std::cout << "FAIL: |" << #a << "-" << #b << "| = " << fabs(_a-_b) << " > " << tol << std::endl; \
        g_failed++; return; \
    } \
} while(0)

// ===== Helper: apply rigid transform to point =====
static Vec3 applyTransform(const double R[9], const double t[3], const Vec3& p) {
    return Vec3(
        R[0]*p.x + R[1]*p.y + R[2]*p.z + t[0],
        R[3]*p.x + R[4]*p.y + R[5]*p.z + t[1],
        R[6]*p.x + R[7]*p.y + R[8]*p.z + t[2]
    );
}

// ===== Helper: create ground-truth rotation around Z =====
static void rotZ(double R[9], double angleDeg) {
    double rad = angleDeg * 3.14159265358979323846 / 180.0;
    double c = cos(rad), s = sin(rad);
    R[0] = c;  R[1] = -s; R[2] = 0;
    R[3] = s;  R[4] =  c; R[5] = 0;
    R[6] = 0;  R[7] =  0; R[8] = 1;
}

// ===== Helper: generate random 3D points within a bounding box =====
static Vec3 randomPoint(int seed) {
    // Deterministic pseudo-random for reproducibility
    unsigned int s = (unsigned int)(seed + 1) * 1103515245u + 12345u;
    auto randf = [&s]() -> double {
        s = s * 1103515245u + 12345u;
        return (double)((s >> 16) & 0x7FFF) / 32767.0;
    };
    return Vec3(
        (randf() - 0.5) * 200.0,  // -100..100 mm
        (randf() - 0.5) * 200.0,
        (randf() - 0.5) * 200.0
    );
}

// ================================================================
// Test 1: Identity — same Touch and Robot points → R=I, t=0
// ================================================================
static void test_identity() {
    TEST(identity);

    std::vector<std::pair<Vec3, Vec3>> pairs;
    for (int i = 0; i < 5; i++) {
        Vec3 p = randomPoint(100 + i);
        pairs.push_back({p, p});  // identical
    }

    KabschResult r = solveKabsch(pairs);
    CHECK(r.valid);
    CHECK_CLOSE(r.rmsError, 0.0, 1e-6);

    // R should be identity
    CHECK_CLOSE(r.R[0], 1.0, 1e-9);
    CHECK_CLOSE(r.R[4], 1.0, 1e-9);
    CHECK_CLOSE(r.R[8], 1.0, 1e-9);
    CHECK_CLOSE(r.R[1], 0.0, 1e-9);

    // t should be zero
    CHECK_CLOSE(r.t[0], 0.0, 1e-9);
    CHECK_CLOSE(r.t[1], 0.0, 1e-9);
    CHECK_CLOSE(r.t[2], 0.0, 1e-9);

    PASS();
}

// ================================================================
// Test 2: Translation only → R=I, t matches
// ================================================================
static void test_translation() {
    TEST(translation);

    double tTrue[3] = { 100.0, -50.0, 200.0 };
    double Rtrue[9] = {1,0,0, 0,1,0, 0,0,1};

    std::vector<std::pair<Vec3, Vec3>> pairs;
    for (int i = 0; i < 5; i++) {
        Vec3 touch = randomPoint(200 + i);
        Vec3 robot = applyTransform(Rtrue, tTrue, touch);
        pairs.push_back({touch, robot});
    }

    KabschResult r = solveKabsch(pairs);
    CHECK(r.valid);
    CHECK_CLOSE(r.rmsError, 0.0, 1e-6);

    CHECK_CLOSE(r.t[0], tTrue[0], 1e-6);
    CHECK_CLOSE(r.t[1], tTrue[1], 1e-6);
    CHECK_CLOSE(r.t[2], tTrue[2], 1e-6);

    PASS();
}

// ================================================================
// Test 3: 90° rotation around Z axis
// ================================================================
static void test_rotation_z90() {
    TEST(rotation_z90);

    double Rtrue[9];
    rotZ(Rtrue, 90.0);  // (X,Y) → (-Y, X)
    double tTrue[3] = {0, 0, 0};

    std::vector<std::pair<Vec3, Vec3>> pairs;
    for (int i = 0; i < 5; i++) {
        Vec3 touch = randomPoint(300 + i);
        Vec3 robot = applyTransform(Rtrue, tTrue, touch);
        pairs.push_back({touch, robot});
    }

    KabschResult r = solveKabsch(pairs);
    CHECK(r.valid);
    CHECK_CLOSE(r.rmsError, 0.0, 1e-9);

    // Verify R is close to 90° Z-rotation
    CHECK_CLOSE(r.R[0],  0.0, 1e-6);
    CHECK_CLOSE(r.R[1], -1.0, 1e-6);
    CHECK_CLOSE(r.R[3],  1.0, 1e-6);
    CHECK_CLOSE(r.R[4],  0.0, 1e-6);

    PASS();
}

// ================================================================
// Test 4: Full rigid transform (rotation + translation)
// ================================================================
static void test_combined_transform() {
    TEST(combined_transform);

    double Rtrue[9];
    rotZ(Rtrue, 30.0);
    double tTrue[3] = { 15.0, -42.0, 88.0 };

    std::vector<std::pair<Vec3, Vec3>> pairs;
    for (int i = 0; i < 6; i++) {
        Vec3 touch = randomPoint(400 + i);
        Vec3 robot = applyTransform(Rtrue, tTrue, touch);
        pairs.push_back({touch, robot});
    }

    KabschResult r = solveKabsch(pairs);
    CHECK(r.valid);
    CHECK_CLOSE(r.rmsError, 0.0, 1e-9);

    // Verify R
    CHECK_CLOSE(r.R[0], Rtrue[0], 1e-6);
    CHECK_CLOSE(r.R[1], Rtrue[1], 1e-6);
    CHECK_CLOSE(r.R[3], Rtrue[3], 1e-6);
    CHECK_CLOSE(r.R[4], Rtrue[4], 1e-6);

    // Verify t
    CHECK_CLOSE(r.t[0], tTrue[0], 1e-6);
    CHECK_CLOSE(r.t[1], tTrue[1], 1e-6);
    CHECK_CLOSE(r.t[2], tTrue[2], 1e-6);

    PASS();
}

// ================================================================
// Test 5: Solver corrects improper rotation (det < 0)
// ================================================================
static void test_reflection_correction() {
    TEST(reflection_correction);

    // Create a reflection matrix (det = -1): flip X
    double Href[9] = {-1,0,0, 0,1,0, 0,0,1};
    double tRef[3] = {10, 20, 30};

    std::vector<std::pair<Vec3, Vec3>> pairs;
    for (int i = 0; i < 5; i++) {
        Vec3 touch = randomPoint(500 + i);
        Vec3 robot = applyTransform(Href, tRef, touch);
        pairs.push_back({touch, robot});
    }

    KabschResult r = solveKabsch(pairs);
    CHECK(r.valid);

    // RMS error should be near zero (the solver found the best proper rotation)
    // With a reflection as input, the RMS won't be perfectly zero
    // but the solver corrects it to a proper rotation
    // R should have det ≈ 1
    double det = r.R[0]*(r.R[4]*r.R[8] - r.R[5]*r.R[7])
               - r.R[1]*(r.R[3]*r.R[8] - r.R[5]*r.R[6])
               + r.R[2]*(r.R[3]*r.R[7] - r.R[4]*r.R[6]);
    CHECK(det > 0.0);  // must be proper rotation

    PASS();
}

// ================================================================
// Test 6: Noisy data — RMS error reflects noise level
// ================================================================
static void test_noisy_data() {
    TEST(noisy_data);

    double Rtrue[9] = {1,0,0, 0,1,0, 0,0,1};
    double tTrue[3] = {0, 0, 0};

    std::vector<std::pair<Vec3, Vec3>> pairs;
    for (int i = 0; i < 10; i++) {
        Vec3 touch = randomPoint(600 + i);
        Vec3 robot = applyTransform(Rtrue, tTrue, touch);
        // Add ~2mm noise
        Vec3 noisyRobot(
            robot.x + randomPoint(700 + i*3).x * 0.02,
            robot.y + randomPoint(700 + i*3+1).y * 0.02,
            robot.z + randomPoint(700 + i*3+2).z * 0.02
        );
        pairs.push_back({touch, noisyRobot});
    }

    KabschResult r = solveKabsch(pairs);
    CHECK(r.valid);

    // RMS error should be non-zero but small (< 5mm for 2mm noise with 10 points)
    CHECK(r.rmsError > 0.0);
    CHECK(r.rmsError < 5.0);

    // R should still be close to identity
    CHECK_CLOSE(r.R[0], 1.0, 0.05);

    PASS();
}

// ================================================================
// Test 7: Exactly 3 non-collinear points
// ================================================================
static void test_minimal_3points() {
    TEST(minimal_3points);

    // 3 well-separated points (non-collinear)
    Vec3 t1(10, 0, 0), t2(0, 10, 0), t3(0, 0, 10);
    Vec3 r1(110, -50, 200);  // translated
    Vec3 r2(100, -40, 200);
    Vec3 r3(100, -50, 210);

    std::vector<std::pair<Vec3, Vec3>> pairs = {
        {t1, r1}, {t2, r2}, {t3, r3}
    };

    KabschResult result = solveKabsch(pairs);
    CHECK(result.valid);
    CHECK(result.rmsError >= 0.0);

    // Should reconstruct r1 from t1 with zero error (3 points = deterministic)
    Vec3 reconstructed = applyTransform(result.R, result.t, t1);
    CHECK_CLOSE(reconstructed.x, r1.x, 1e-6);
    CHECK_CLOSE(reconstructed.y, r1.y, 1e-6);
    CHECK_CLOSE(reconstructed.z, r1.z, 1e-6);

    PASS();
}

// ================================================================
// Test 8: Too few points returns invalid
// ================================================================
static void test_too_few_points() {
    TEST(too_few_points);

    std::vector<std::pair<Vec3, Vec3>> pairs = {
        { Vec3(0,0,0), Vec3(1,1,1) },
        { Vec3(1,1,1), Vec3(2,2,2) }
    };

    KabschResult r = solveKabsch(pairs);
    CHECK(!r.valid);

    PASS();
}

// ================================================================
int main() {
    std::cout << "=== Kabsch-Umeyama Solver Tests ===" << std::endl;

    test_identity();
    test_translation();
    test_rotation_z90();
    test_combined_transform();
    test_reflection_correction();
    test_noisy_data();
    test_minimal_3points();
    test_too_few_points();

    std::cout << std::endl;
    std::cout << "Results: " << g_passed << " passed, " << g_failed << " failed" << std::endl;
    return g_failed > 0 ? 1 : 0;
}
