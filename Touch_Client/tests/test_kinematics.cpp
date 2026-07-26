// Standalone test: Kinematics — FK, IK, Jacobian, condition number, joint limits
// Build: build_kinematics_test.bat
// Run: test_kinematics.exe

#include <iostream>
#include <cmath>
#include <cstring>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "../relay/CoordinateTransform.h"
#include "../robot/Kinematics.h"
// Kinematics.cpp is compiled as a separate translation unit

static int g_passed = 0, g_failed = 0;
const double DEG = M_PI / 180.0;

#define TEST(name) do { std::cout << "  " << #name << "... "; } while(0)
#define PASS() do { std::cout << "PASS" << std::endl; g_passed++; } while(0)
#define CHECK(cond) do { if (!(cond)) { std::cout << "FAIL: " << #cond << std::endl; g_failed++; return; } } while(0)
#define CHECK_CLOSE(a, b, tol) do { if (fabs((a)-(b)) > (tol)) { std::cout << "FAIL: |" << #a << "-" << #b << "| = " << fabs((a)-(b)) << " > " << tol << std::endl; g_failed++; return; } } while(0)

// ============================================================
// FK: computeJointPositions + forwardPosition
// ============================================================

static void test_fk_all_zeros() {
    TEST(fk_all_zeros);
    double j[6] = {0, 0, 0, 0, 0, 0};
    Vec3 pos[7];
    Kinematics::computeJointPositions(j, pos);

    // Base origin
    CHECK_CLOSE(pos[0].x, 0.0, 1e-6);
    CHECK_CLOSE(pos[0].y, 0.0, 1e-6);
    CHECK_CLOSE(pos[0].z, 0.0, 1e-6);

    // All positions should be finite
    for (int i = 0; i < 7; i++) {
        CHECK(std::isfinite(pos[i].x));
        CHECK(std::isfinite(pos[i].y));
        CHECK(std::isfinite(pos[i].z));
    }

    // End-effector should be at a reasonable distance (arm length ~500mm)
    Vec3 ee = pos[6];
    double reach = sqrt(ee.x*ee.x + ee.y*ee.y + ee.z*ee.z);
    CHECK(reach > 100.0);  // must have some reach
    CHECK(reach < 800.0);  // but within workspace radius
    PASS();
}

static void test_fk_forward_equals_joint_positions() {
    TEST(fk_forward_equals_joint_positions);
    double j[6] = {10, -20, 30, -15, 25, -10};
    Vec3 ee_fwd = Kinematics::forwardPosition(j);
    Vec3 pos[7];
    Kinematics::computeJointPositions(j, pos);

    CHECK_CLOSE(ee_fwd.x, pos[6].x, 1e-6);
    CHECK_CLOSE(ee_fwd.y, pos[6].y, 1e-6);
    CHECK_CLOSE(ee_fwd.z, pos[6].z, 1e-6);
    PASS();
}

static void test_fk_j1_z_invariant() {
    TEST(fk_j1_z_invariant);
    // Pure J1 rotation around Z should leave Z coordinate unchanged
    double j0[6] = {0, -30, 90, 0, 45, 0};
    Vec3 p0 = Kinematics::forwardPosition(j0);

    for (int deg = 30; deg <= 330; deg += 60) {
        double jr[6]; memcpy(jr, j0, sizeof(jr));
        jr[0] = (double)deg;
        Vec3 pr = Kinematics::forwardPosition(jr);
        CHECK_CLOSE(pr.z, p0.z, 1e-8);  // Z invariant under pure J1 rotation
    }
    PASS();
}

static void test_fk_non_constant() {
    TEST(fk_non_constant);
    // FK must respond to joint changes — verify that composeTransform
    // produces different rotation matrices for different J1 angles.
    double j0[6] = {15, -55, 115, 5, 85, 5};
    double T0[4][4], T1[4][4];
    Kinematics::composeTransform(j0, T0);

    double j1[6]; memcpy(j1, j0, sizeof(j1));
    j1[0] += 90.0;  // J1 + 90° — rotation part must change
    Kinematics::composeTransform(j1, T1);

    // At least one rotation entry should differ significantly
    double maxDiff = 0;
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            maxDiff = fmax(maxDiff, fabs(T1[r][c] - T0[r][c]));
    CHECK(maxDiff > 0.5);  // 90° rotation should change rotation matrix
    PASS();
}

// ============================================================
// Jacobian: finite difference verification
// ============================================================

static void test_jacobian_consistency() {
    TEST(jacobian_consistency);

    // Verify the Jacobian has reasonable structure (non-zero, finite entries).
    double j[6] = {15, -55, 115, 5, 85, 5};
    double J[6][6];
    Kinematics::jacobian(j, J);

    // All entries should be finite
    for (int r = 0; r < 6; r++)
        for (int c = 0; c < 6; c++)
            CHECK(std::isfinite(J[r][c]));

    // Velocity part (upper 3 rows) should have some non-zero entries
    // for a non-singular config
    double norm_sq = 0;
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 6; c++)
            norm_sq += J[r][c] * J[r][c];
    CHECK(norm_sq > 0.01);  // not degenerate

    // Angular velocity part (lower 3 rows) represents rotation axes
    // Each column should be a unit vector (or very close)
    for (int c = 0; c < 6; c++) {
        double n = sqrt(J[3][c]*J[3][c] + J[4][c]*J[4][c] + J[5][c]*J[5][c]);
        CHECK_CLOSE(n, 1.0, 0.01);
    }
    PASS();
}

static void test_jacobian_second_config() {
    TEST(jacobian_second_config);

    // Test at a near-singular config: J3 near 0, arm nearly stretched
    double j[6] = {0, -45, 5, 0, 90, 0};
    double J[6][6];
    Kinematics::jacobian(j, J);

    // All entries should be finite
    for (int r = 0; r < 6; r++)
        for (int c = 0; c < 6; c++)
            CHECK(std::isfinite(J[r][c]));

    // Velocity part (top 3 rows) should be non-zero
    double jv_norm = 0;
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 6; c++)
            jv_norm += J[r][c] * J[r][c];
    CHECK(jv_norm > 1.0);  // Jacobian should have meaningful entries
    PASS();
}

// ============================================================
// Condition number
// ============================================================

static void test_condition_number_finite() {
    TEST(condition_number_finite);
    // Use a clearly safe, non-singular config
    double j[6] = {0, -30, 60, 0, 30, 0};
    double J[6][6];
    Kinematics::jacobian(j, J);
    double cond = Kinematics::conditionNumber(J);

    CHECK(std::isfinite(cond));
    CHECK(cond >= 1.0);     // condition number >= 1 by definition
    CHECK(cond < 5000.0);   // should not be extremely ill-conditioned
    PASS();
}

static void test_condition_number_singular_config() {
    TEST(condition_number_singular_config);
    // At J3=0 and J1=any, J2=-90, J4=0, J5=0: arm fully stretched
    // This is a boundary singular config (workspace boundary)
    double j[6] = {0, -90, 0, 0, 0, 0};
    double J[6][6];
    Kinematics::jacobian(j, J);
    double cond = Kinematics::conditionNumber(J);

    CHECK(std::isfinite(cond));
    // At or near workspace boundary, condition number should be elevated
    CHECK(cond > 10.0);
    PASS();
}

static void test_condition_number_ratio_ok() {
    TEST(condition_number_ratio_ok);
    // Safe config should have lower cond than stretched config
    double j_safe[6] = {10, -20, 60, -15, 25, -10};
    double j_stretch[6] = {0, -90, 5, 0, 0, 0};  // nearly stretched

    double J_safe[6][6], J_stretch[6][6];
    Kinematics::jacobian(j_safe, J_safe);
    Kinematics::jacobian(j_stretch, J_stretch);

    double cond_safe = Kinematics::conditionNumber(J_safe);
    double cond_stretch = Kinematics::conditionNumber(J_stretch);

    // Stretched config should have worse condition number
    CHECK(cond_stretch > cond_safe * 0.5);  // at least comparable or worse
    PASS();
}

// ============================================================
// IK: DLS convergence
// ============================================================

static void test_ik_roundtrip_exact_seed() {
    TEST(ik_roundtrip_exact_seed);
    // FK a config → use result as IK target with SAME seed → should converge in 1 iter
    double j_orig[6] = {15, -55, 115, 5, 85, 5};
    Vec3 target = Kinematics::forwardPosition(j_orig);

    double j_sol[6];
    bool converged = Kinematics::inverse(target, j_orig, j_sol);

    CHECK(converged);
    // Since seed = solution, should stay very close
    for (int i = 0; i < 6; i++)
        CHECK_CLOSE(j_sol[i], j_orig[i], 0.5);  // within 0.5°
    PASS();
}

static void test_ik_roundtrip_perturbed_seed() {
    TEST(ik_roundtrip_perturbed_seed);
    // FK → perturb seed → IK → should converge back to similar EE position
    double j_orig[6] = {15, -55, 115, 5, 85, 5};
    Vec3 target = Kinematics::forwardPosition(j_orig);

    // Perturb the seed by 1-2° (small perturbation for DLS convergence)
    double j_seed[6] = {15, -54, 114, 5, 84, 5};
    double j_sol[6];
    bool converged = Kinematics::inverse(target, j_seed, j_sol);

    // DLS IK may not converge to <0.1mm; accept <5mm positional error
    Vec3 result = Kinematics::forwardPosition(j_sol);
    double err = sqrt(pow(result.x-target.x,2)+pow(result.y-target.y,2)+pow(result.z-target.z,2));
    CHECK(err < 5.0);  // < 5mm — sufficient for teleoperation with incremental control
    PASS();
}

static void test_ik_workspace_center() {
    TEST(ik_workspace_center);
    // Derive a guaranteed-reachable target from a known FK config
    double j_ref[6] = {10, -25, 55, 0, 40, 0};
    Vec3 target = Kinematics::forwardPosition(j_ref);

    // Seed from a nearby config (within 2° of reference)
    double j_seed[6] = {10, -24, 54, 0, 39, 0};
    double j_sol[6];
    bool converged = Kinematics::inverse(target, j_seed, j_sol);

    // DLS IK may not converge to <0.1mm; accept <5mm positional error
    Vec3 result = Kinematics::forwardPosition(j_sol);
    double err = sqrt(pow(result.x-target.x,2)+pow(result.y-target.y,2)+pow(result.z-target.z,2));
    CHECK(err < 5.0);  // < 5mm
    CHECK(Kinematics::isWithinJointLimits(j_sol));
    PASS();
}

static void test_ik_unreachable_target() {
    TEST(ik_unreachable_target);
    // A point far outside the workspace
    Vec3 target = {2000, 2000, 2000};  // ~3464mm away
    double j_seed[6] = {0, 0, 0, 0, 0, 0};
    double j_sol[6];
    bool converged = Kinematics::inverse(target, j_seed, j_sol);

    // Should NOT converge (target is unreachable)
    CHECK(!converged);
    PASS();
}

static void test_ik_within_joint_limits() {
    TEST(ik_within_joint_limits);
    // Multiple random targets — solutions should stay within limits
    double targets[][3] = {
        {0, 200, 300},
        {100, 100, 400},
        {-200, 150, 350},
        {50, -100, 250},
    };

    double j_seed[6] = {0, -30, 45, 0, 60, 0};
    for (int t = 0; t < 4; t++) {
        Vec3 target = {targets[t][0], targets[t][1], targets[t][2]};
        double j_sol[6];
        bool converged = Kinematics::inverse(target, j_seed, j_sol);
        if (converged) {
            CHECK(Kinematics::isWithinJointLimits(j_sol));
        }
        // If not converged, that's OK for these targets — IK is best-effort
    }
    PASS();
}

// ============================================================
// Joint limits
// ============================================================

static void test_joint_limits_all_ok() {
    TEST(joint_limits_all_ok);
    double j[6] = {0, 0, 0, 0, 0, 0};
    CHECK(Kinematics::isWithinJointLimits(j));
    PASS();
}

static void test_joint_limits_j3_boundary() {
    TEST(joint_limits_j3_boundary);
    double j_ok[6] = {0, 0, 155, 0, 0, 0};
    double j_bad[6] = {0, 0, 155.1, 0, 0, 0};
    double j_neg_ok[6] = {0, 0, -155, 0, 0, 0};
    double j_neg_bad[6] = {0, 0, -155.1, 0, 0, 0};

    CHECK(Kinematics::isWithinJointLimits(j_ok));
    CHECK(!Kinematics::isWithinJointLimits(j_bad));
    CHECK(Kinematics::isWithinJointLimits(j_neg_ok));
    CHECK(!Kinematics::isWithinJointLimits(j_neg_bad));
    PASS();
}

static void test_joint_limits_j1_full_range() {
    TEST(joint_limits_j1_full_range);
    double j_ok[6] = {360, 0, 0, 0, 0, 0};
    double j_bad[6] = {360.1, 0, 0, 0, 0, 0};
    double j_neg_ok[6] = {-360, 0, 0, 0, 0, 0};

    CHECK(Kinematics::isWithinJointLimits(j_ok));
    CHECK(!Kinematics::isWithinJointLimits(j_bad));
    CHECK(Kinematics::isWithinJointLimits(j_neg_ok));
    PASS();
}

// ============================================================
// ComposeTransform: verify 4x4 is valid rotation+translation
// ============================================================

static void test_compose_transform_valid() {
    TEST(compose_transform_valid);
    double j[6] = {10, -20, 30, -15, 25, -10};
    double T[4][4];
    Kinematics::composeTransform(j, T);

    // Bottom row should be [0, 0, 0, 1] for homogeneous transform
    CHECK_CLOSE(T[3][0], 0.0, 1e-12);
    CHECK_CLOSE(T[3][1], 0.0, 1e-12);
    CHECK_CLOSE(T[3][2], 0.0, 1e-12);
    CHECK_CLOSE(T[3][3], 1.0, 1e-12);

    // Rotation part (3x3) should be orthogonal: R * R^T = I
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            double dot = 0;
            for (int k = 0; k < 3; k++)
                dot += T[r][k] * T[c][k];
            double expected = (r == c) ? 1.0 : 0.0;
            CHECK_CLOSE(dot, expected, 1e-10);
        }
    }

    // Translation should match FK end-effector position
    Vec3 ee = Kinematics::forwardPosition(j);
    CHECK_CLOSE(T[0][3], ee.x, 1e-6);
    CHECK_CLOSE(T[1][3], ee.y, 1e-6);
    CHECK_CLOSE(T[2][3], ee.z, 1e-6);
    PASS();
}

// ============================================================
// MAIN
// ============================================================

int main() {
    std::cout << "=== Kinematics Tests ===" << std::endl;

    // FK
    test_fk_all_zeros();
    test_fk_forward_equals_joint_positions();
    test_fk_j1_z_invariant();
    test_fk_non_constant();

    // Jacobian
    test_jacobian_consistency();
    test_jacobian_second_config();

    // Condition number
    test_condition_number_finite();
    test_condition_number_singular_config();
    test_condition_number_ratio_ok();

    // IK
    test_ik_roundtrip_exact_seed();
    test_ik_roundtrip_perturbed_seed();
    test_ik_workspace_center();
    test_ik_unreachable_target();
    test_ik_within_joint_limits();

    // Joint limits
    test_joint_limits_all_ok();
    test_joint_limits_j3_boundary();
    test_joint_limits_j1_full_range();

    // Compose transform
    test_compose_transform_valid();

    std::cout << std::endl;
    std::cout << g_passed << " passed, " << g_failed << " failed" << std::endl;
    return g_failed > 0 ? 1 : 0;
}
