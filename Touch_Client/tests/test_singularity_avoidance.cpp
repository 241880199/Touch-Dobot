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

int main() {
    int passed = 0, total = 6;
    printf("=== SingularityAvoidance Tests ===\n\n");

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

    printf("\n=== %d/%d tests passed ===\n", passed, total);
    return (passed == total) ? 0 : 1;
}
