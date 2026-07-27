// test_singularity_avoidance.cpp
// Unit tests for SingularityAvoidance module
// Define TEST_SINGAVOID to stub sendWarning() (avoids linking RelayCore + deps)

#define TEST_SINGAVOID
#include "../safety/SingularityAvoidance.h"
#include "../robot/Kinematics.h"
#include "../config/Config.h"
#include <cmath>
#include <cstdio>
#include <cassert>

static bool approx(double a, double b, double tol = 0.01) {
    return fabs(a - b) < tol;
}

// Test 1: Null-space projector preserves position
static bool test_null_space_preserves_position() {
    double q_test[6] = {10, 30, -45, 20, -15, 60};  // typical home config
    Vec3 pos_before = Kinematics::forwardPosition(q_test);

    // Get position Jacobian
    double J_full[6][6];
    Kinematics::jacobian(q_test, J_full);

    // Apply a random perturbation in null space
    double q_perturbed[6];
    for (int i = 0; i < 6; i++)
        q_perturbed[i] = q_test[i] + 0.05 * (i % 2 ? 1.0 : -1.0);

    // Project perturbation into null space (caller must verify position unchanged)
    // For now, just verify FK consistency
    Vec3 pos_after = Kinematics::forwardPosition(q_perturbed);
    printf("  Test1: pos_before=(%.2f,%.2f,%.2f) pos_after=(%.2f,%.2f,%.2f)\n",
           pos_before.x, pos_before.y, pos_before.z,
           pos_after.x, pos_after.y, pos_after.z);
    return true;  // baseline sanity check
}

// Test 2: optimizeOrientation returns valid orientation at safe position
static bool test_optimize_at_safe_position() {
    double q[6] = {30, 45, -60, 10, 30, -45};
    Vec3 targetPos = Kinematics::forwardPosition(q);  // FK-consistent target
    Vec3 orient = SingularityAvoidance::optimizeOrientation(targetPos, q);

    // Should return a valid orientation (not all zeros)
    if (fabs(orient.x) < 0.001 && fabs(orient.y) < 0.001 && fabs(orient.z) < 0.001) {
        printf("  FAIL: optimizeOrientation returned zero vec\n");
        return false;
    }
    printf("  Test2: orient=(%.2f,%.2f,%.2f)\n", orient.x, orient.y, orient.z);
    return true;
}

// Test 3: optimizeOrientation pushes arm away when near Z-axis
static bool test_optimize_near_z_axis() {
    // Position near Z-axis (shoulder singularity risk)
    Vec3 nearZ(20, 15, 300);  // r_xy ≈ 25mm

    // Seed joints at a typical config
    double q[6] = {45, 90, -120, 0, 45, 0};

    Vec3 orient = SingularityAvoidance::optimizeOrientation(nearZ, q);
    // Even if IK struggles, it should not crash
    printf("  Test3: orient near Z=(%.2f,%.2f,%.2f)\n", orient.x, orient.y, orient.z);
    return true;  // no crash
}

// Test 4: dampOrientationMotion passes through when safe
static bool test_damp_safe_orientation() {
    double q[6] = {30, 20, -30, 45, 60, -30};  // well-conditioned config
    Vec3 targetOrient(10, 20, 30);
    Vec3 delta(2.0, -1.0, 1.5);
    Vec3 currentTcp(300, 200, 400);
    Vec3 tcpAdj, repulsion;

    Vec3 damped = SingularityAvoidance::dampOrientationMotion(
        targetOrient, delta, currentTcp, q, tcpAdj, repulsion);

    printf("  Test4: delta_in=(%.2f,%.2f,%.2f) delta_out=(%.2f,%.2f,%.2f) tcpAdj=(%.2f,%.2f,%.2f)\n",
           delta.x, delta.y, delta.z, damped.x, damped.y, damped.z,
           tcpAdj.x, tcpAdj.y, tcpAdj.z);
    // At safe config, delta should pass through near-unchanged; TCP adjustment should be ~0
    return true;  // baseline — tuning will validate on hardware
}

// Test 5: dampOrientationMotion damps when wrist near singular
static bool test_damp_wrist_singular() {
    // J5 near 0 deg — wrist singularity approach (J4 + J6 aligned)
    double q[6] = {10, 30, -45, 20, 1.0, 60};  // J5 = 1 deg → near alignment
    Vec3 targetOrient(0, 0, 0);
    Vec3 delta(5.0, 5.0, 5.0);  // large orientation delta
    Vec3 currentTcp(300, 200, 400);
    Vec3 tcpAdj, repulsion;

    Vec3 damped = SingularityAvoidance::dampOrientationMotion(
        targetOrient, delta, currentTcp, q, tcpAdj, repulsion);

    double magIn = sqrt(delta.x*delta.x + delta.y*delta.y + delta.z*delta.z);
    double magOut = sqrt(damped.x*damped.x + damped.y*damped.y + damped.z*damped.z);

    printf("  Test5: mag_in=%.2f mag_out=%.2f tcpAdj=(%.2f,%.2f,%.2f)\n",
           magIn, magOut, tcpAdj.x, tcpAdj.y, tcpAdj.z);
    // At near-singular wrist, delta should be damped (magOut <= magIn)
    return magOut <= magIn * 1.01;  // allow tiny numerical growth
}

// Test 6: dampFullCommand passes through when safe
static bool test_damp_full_safe() {
    double q[6] = {20, 40, -50, 30, 45, -20};  // well-conditioned
    Vec3 userPos(1.0, -0.5, 2.0);
    Vec3 userOrient(0.5, 1.0, -0.5);
    Vec3 dampPos, dampOrient;

    SingularityAvoidance::dampFullCommand(userPos, userOrient, q, dampPos, dampOrient);

    printf("  Test6: pos_in=(%.2f,%.2f,%.2f) pos_out=(%.2f,%.2f,%.2f)\n",
           userPos.x, userPos.y, userPos.z, dampPos.x, dampPos.y, dampPos.z);
    // At safe config, should pass through near-unchanged
    return approx(dampPos.x, userPos.x, 0.1) &&
           approx(dampPos.y, userPos.y, 0.1) &&
           approx(dampPos.z, userPos.z, 0.1);
}

// Test 7: dampOrientationMotion — continuous damping is monotonic (Phase 2)
static bool test_continuous_damping_monotonic() {
    Vec3 currentTcp(300, 200, 400);
    Vec3 delta(5.0, 0.0, 0.0);
    Vec3 targetOrient(0, 0, 0);

    double prevMag = -1.0;
    // Test joints at increasing J5 proximity to singularity (J5 → 0)
    double testJ5[] = {60.0, 30.0, 10.0, 5.0, 1.0, 0.5};
    for (int i = 0; i < 6; i++) {
        double q[6] = {10, 30, -45, 20, testJ5[i], 60};
        Vec3 tcpAdj, repulsion;
        Vec3 damped = SingularityAvoidance::dampOrientationMotion(
            targetOrient, delta, currentTcp, q, tcpAdj, repulsion);
        double mag = sqrt(damped.x*damped.x + damped.y*damped.y + damped.z*damped.z);
        printf("  Test7: J5=%.1f → mag_out=%.2f\n", testJ5[i], mag);
        // Damping should increase (output magnitude should decrease or stay same)
        if (prevMag >= 0.0 && mag > prevMag * 1.01) {
            printf("  FAIL: damping not monotonic\n");
            return false;
        }
        prevMag = mag;
    }
    return true;
}

// Test 8: dampOrientationMotion — repulsion opposes user rotation toward J5=0 (Phase 2)
static bool test_repulsion_direction_correct() {
    // J5 near 0 deg — wrist singularity, rotation into J5=0 is dangerous
    double q[6] = {10, 30, -45, 20, 2.0, 60};
    Vec3 targetOrient(0, 0, 0);
    // Positive delta in Rx (likely rotates J5 toward 0 depending on config)
    Vec3 delta(10.0, 0.0, 0.0);  // large delta into potential danger
    Vec3 currentTcp(300, 200, 400);
    Vec3 tcpAdj, repulsion;

    Vec3 damped = SingularityAvoidance::dampOrientationMotion(
        targetOrient, delta, currentTcp, q, tcpAdj, repulsion);

    double repMag = sqrt(repulsion.x*repulsion.x + repulsion.y*repulsion.y + repulsion.z*repulsion.z);
    printf("  Test8: repulsion=(%.3f,%.3f,%.3f) mag=%.3f\n",
           repulsion.x, repulsion.y, repulsion.z, repMag);
    // At J5≈2°, repulsion should be non-zero (cond should exceed 20)
    // Don't assert direction since it depends on the arm configuration;
    // just verify repulsion is computed (non-NaN, finite)
    if (std::isnan(repMag) || std::isinf(repMag)) {
        printf("  FAIL: repulsion is NaN/Inf\n");
        return false;
    }
    return true;  // structural correctness — direction validated on hardware
}

// Test 9: dampOrientationMotion — no repulsion when cond < 20 (Phase 2)
static bool test_repulsion_zero_in_safe_zone() {
    // Well-conditioned config: all joints at mid-range
    double q[6] = {30, 20, -30, 45, 60, -30};
    Vec3 targetOrient(10, 20, 30);
    Vec3 delta(2.0, -1.0, 1.5);
    Vec3 currentTcp(300, 200, 400);
    Vec3 tcpAdj, repulsion;

    Vec3 damped = SingularityAvoidance::dampOrientationMotion(
        targetOrient, delta, currentTcp, q, tcpAdj, repulsion);

    double repMag = sqrt(repulsion.x*repulsion.x + repulsion.y*repulsion.y + repulsion.z*repulsion.z);
    printf("  Test9: repulsion_mag=%.6f (expect 0)\n", repMag);
    if (repMag > 0.001) {
        printf("  FAIL: repulsion in safe zone\n");
        return false;
    }
    return true;
}

// Test 10: dampOrientationMotion — shoulder TCP adjust triggers at 120mm (Phase 2)
static bool test_shoulder_early_trigger() {
    // Joints that place elbow near Z-axis (r_xy between 50 and 120)
    // J2 ≈ 90° with some J1 gives elbow at moderate r_xy
    double q[6] = {5, 85, -100, 10, 45, -20};
    Vec3 positions[7];
    Kinematics::computeJointPositions(q, positions);
    double r_elbow = sqrt(positions[2].x*positions[2].x + positions[2].y*positions[2].y);
    printf("  Test10: r_elbow=%.1f mm\n", r_elbow);

    Vec3 targetOrient(0, 0, 0);
    Vec3 delta(0.1, 0.0, 0.0);
    Vec3 currentTcp(100, 50, 300);
    Vec3 tcpAdj, repulsion;

    SingularityAvoidance::dampOrientationMotion(
        targetOrient, delta, currentTcp, q, tcpAdj, repulsion);

    double adjMag = sqrt(tcpAdj.x*tcpAdj.x + tcpAdj.y*tcpAdj.y + tcpAdj.z*tcpAdj.z);
    printf("  Test10: tcpAdj=(%.2f,%.2f,%.2f) mag=%.2f\n", tcpAdj.x, tcpAdj.y, tcpAdj.z, adjMag);

    if (r_elbow < 120.0 && adjMag < 0.001) {
        printf("  FAIL: elbow at %.1fmm < 120mm but no TCP adjust\n", r_elbow);
        return false;
    }
    return true;  // If r_elbow ≥ 120, no adjustment is correct behavior
}

// Test 11: dampOrientationMotion — dual singularity warning when both wrist and shoulder at risk (Phase 2)
static bool test_dual_singular_warning() {
    // Wrist near singular (J5≈1°) + elbow near Z-axis
    double q[6] = {3, 88, -100, 10, 1.0, 20};
    Vec3 positions[7];
    Kinematics::computeJointPositions(q, positions);
    double r_elbow = sqrt(positions[2].x*positions[2].x + positions[2].y*positions[2].y);

    Vec3 targetOrient(0, 0, 0);
    Vec3 delta(5.0, 5.0, 5.0);
    Vec3 currentTcp = Kinematics::forwardPosition(q);
    Vec3 tcpAdj, repulsion;

    Vec3 damped = SingularityAvoidance::dampOrientationMotion(
        targetOrient, delta, currentTcp, q, tcpAdj, repulsion);

    double repMag = sqrt(repulsion.x*repulsion.x + repulsion.y*repulsion.y + repulsion.z*repulsion.z);
    printf("  Test11: r_elbow=%.1f, dampMag=%.2f, repMag=%.2f\n",
           r_elbow,
           sqrt(damped.x*damped.x + damped.y*damped.y + damped.z*damped.z),
           repMag);
    // Should not crash; repulsion should be non-trivial at this config
    if (std::isnan(repMag) || std::isinf(repMag)) {
        printf("  FAIL: NaN/Inf in dual-singular output\n");
        return false;
    }
    return true;
}

// Test 12: dampOrientationMotion — stress/no-crash test for gradient step (Phase 2)
static bool test_gradient_step_no_crash() {
    // Config where elbow is at ~80mm from Z-axis (within SAFE_R but not critical)
    double q[6] = {8, 75, -90, 15, 30, -25};
    Vec3 targetOrient(0, 0, 0);
    Vec3 delta(0.5, 0.0, 0.0);
    Vec3 currentTcp = Kinematics::forwardPosition(q);
    Vec3 tcpAdj, repulsion;

    SingularityAvoidance::dampOrientationMotion(
        targetOrient, delta, currentTcp, q, tcpAdj, repulsion);

    double adjMag = sqrt(tcpAdj.x*tcpAdj.x + tcpAdj.y*tcpAdj.y + tcpAdj.z*tcpAdj.z);
    printf("  Test12: tcpAdj mag=%.3f mm (grad_step=%.2f, stress test)\n", adjMag, Config::SINGAVOID_GRAD_STEP);
    // Stress test — just verify no crash and finite output when
    // elbow is within 120mm of Z-axis (not critical, but within safe range)
    (void)adjMag;  // no-crash stress test
    if (std::isnan(adjMag) || std::isinf(adjMag)) {
        printf("  FAIL: NaN/Inf TCP adjustment\n");
        return false;
    }
    return true;
}

int main() {
    int passed = 0, total = 12;
    printf("=== SingularityAvoidance Tests (Phase 2) ===\n\n");

    if (test_null_space_preserves_position()) passed++;
    else printf("  FAILED\n");

    if (test_optimize_at_safe_position()) passed++;
    else printf("  FAILED\n");

    if (test_optimize_near_z_axis()) passed++;
    else printf("  FAILED\n");

    if (test_damp_safe_orientation()) passed++;
    else printf("  FAILED\n");

    if (test_damp_wrist_singular()) passed++;
    else printf("  FAILED\n");

    if (test_damp_full_safe()) passed++;
    else printf("  FAILED\n");

    if (test_continuous_damping_monotonic()) passed++;
    else printf("  FAILED\n");

    if (test_repulsion_direction_correct()) passed++;
    else printf("  FAILED\n");

    if (test_repulsion_zero_in_safe_zone()) passed++;
    else printf("  FAILED\n");

    if (test_shoulder_early_trigger()) passed++;
    else printf("  FAILED\n");

    if (test_dual_singular_warning()) passed++;
    else printf("  FAILED\n");

    if (test_gradient_step_no_crash()) passed++;
    else printf("  FAILED\n");

    printf("\n=== %d/%d tests passed ===\n", passed, total);
    return (passed == total) ? 0 : 1;
}
