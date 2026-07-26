#define _USE_MATH_DEFINES
#include <cstdio>
#include <cmath>
#include <cassert>

#include "../relay/CoordinateTransform.h"
#include "../robot/Kinematics.h"
#include "../safety/SelfCollision.h"

static int passed = 0, failed = 0;
#define TEST(name) do { printf("  %s... ", name); } while(0)
#define PASS() do { printf("PASS\n"); passed++; } while(0)
#define FAIL(msg) do { printf("FAIL: %s\n", msg); failed++; } while(0)
#define CHECK(cond, msg) do { if (!(cond)) { FAIL(msg); return; } } while(0)

// ===== Test helpers =====

static void test_safe_config() {
    TEST("safe_config_no_warning");
    // Typical CR3 pose: all joints near zero (arm stretched forward-ish)
    double joints[6] = {0, -30, -60, 0, 30, 0};
    Vec3 positions[7];
    Kinematics::computeJointPositions(joints, positions);
    SelfCollision::Result r = SelfCollision::check(positions);
    CHECK(!r.warning && !r.reject,
          "safe config should not trigger warning");
    printf(" (dist=%.1fmm) ", r.minDistMm);
    PASS();
}

static void test_wrist_near_base() {
    TEST("wrist_near_base_warns");
    // CR3 kinematics: the wrist assembly stays well-separated from the upper arm
    // (>150mm even in max-folded configs). The practical self-collision risk is
    // the wrist or TCP dipping near the base column. Use positional setup:
    Vec3 positions[7];
    positions[0] = Vec3(0, 0, 0);       // base
    positions[1] = Vec3(0, 0, 136);     // J1 (base column top)
    positions[2] = Vec3(0, 0, 400);     // J2 (shoulder)
    positions[3] = Vec3(200, 0, 400);   // J3 (elbow)
    positions[4] = Vec3(200, 0, 200);   // J4
    positions[5] = Vec3(50, 0, 180);    // J5 (wrist near J1 base column, ~55mm)
    positions[6] = Vec3(80, 0, 100);    // J6 (TCP)
    SelfCollision::Result r = SelfCollision::check(positions);
    CHECK(r.warning,
          "wrist near base column should trigger warning");
    CHECK(r.minDistMm < SelfCollision::WARN_DIST_MM,
          "distance should be below warn threshold");
    printf(" (dist=%.1fmm, pair=J%d/J%d) ", r.minDistMm, r.pairA, r.pairB);
    PASS();
}

static void test_reject_too_close() {
    TEST("reject_when_extremely_close");
    // Manually place wrist right next to elbow link to test reject threshold
    Vec3 positions[7];
    // Set up a scenario with J5 extremely close to J2-J3 segment
    positions[0] = Vec3(0, 0, 0);       // base
    positions[1] = Vec3(0, 0, 136);     // J1 (base top)
    positions[2] = Vec3(0, 0, 400);     // J2 (shoulder, high up)
    positions[3] = Vec3(300, 0, 400);   // J3 (elbow, stretched out)
    positions[4] = Vec3(500, 0, 200);   // J4
    positions[5] = Vec3(280, 0, 410);   // J5 — only 30mm from J2-J3 segment!
    positions[6] = Vec3(380, 0, 410);   // J6
    SelfCollision::Result r = SelfCollision::check(positions);
    CHECK(r.reject,
          "J5 at 30mm from elbow link should trigger reject");
    CHECK(r.minDistMm < SelfCollision::REJECT_DIST_MM,
          "distance should be below reject threshold");
    printf(" (dist=%.1fmm) ", r.minDistMm);
    PASS();
}

static void test_tcp_near_base() {
    TEST("tcp_near_base_warns");
    Vec3 positions[7];
    // TCP dipping very low, close to base origin
    positions[0] = Vec3(0, 0, 0);       // base
    positions[1] = Vec3(0, 0, 136);     // J1
    positions[2] = Vec3(0, 0, 300);     // J2
    positions[3] = Vec3(200, 0, 300);   // J3
    positions[4] = Vec3(200, 0, 100);   // J4
    positions[5] = Vec3(150, 0, 50);    // J5
    positions[6] = Vec3(50, 0, 30);     // J6 — close to base (0,0,0)
    SelfCollision::Result r = SelfCollision::check(positions);
    CHECK(r.warning,
          "TCP near base should trigger warning");
    printf(" (dist=%.1fmm, pair=J%d/J%d) ", r.minDistMm, r.pairA, r.pairB);
    PASS();
}

static void test_min_dist_tracking() {
    TEST("min_dist_tracks_closest_pair");
    Vec3 positions[7];
    positions[0] = Vec3(0, 0, 0);
    positions[1] = Vec3(0, 0, 136);
    positions[2] = Vec3(0, 0, 400);
    positions[3] = Vec3(300, 0, 400);
    positions[4] = Vec3(500, 0, 200);
    positions[5] = Vec3(500, 0, 180);   // wrist
    positions[6] = Vec3(400, 0, 380);   // TCP close to J2-J3 line
    SelfCollision::Result r = SelfCollision::check(positions);
    CHECK(r.pairA >= 0 && r.pairB >= 0,
          "should identify closest pair");
    // r.pairA should be 6 (TCP) and r.pairB should be 2 (J2-J3 segment)
    CHECK(r.pairA == 6 || r.pairB == 6,
          "TCP should be part of closest pair in this config");
    printf(" (dist=%.1fmm, J%d/J%d) ", r.minDistMm, r.pairA, r.pairB);
    PASS();
}

static void test_orient_roundtrip() {
    TEST("orientation_roundtrip_safe");
    // Simulate an orientation change: J1-J3 fixed, J4-J6 rotate
    // Start from a safe config, rotate through orientations, verify safe
    double baseJoints[6] = {10, -40, -90, 0, 45, 0};
    Vec3 basePos[7];
    Kinematics::computeJointPositions(baseJoints, basePos);

    // Capture wrist center and TCP offset (simulating button2 press)
    Vec3 wristFixed = basePos[5];
    Vec3 tcpOffset(
        basePos[6].x - wristFixed.x,
        basePos[6].y - wristFixed.y,
        basePos[6].z - wristFixed.z
    );

    // Simulate rotating through different orientations
    double testOrients[][3] = {
        {0, 0, 0}, {15, 0, 0}, {-15, 0, 0},
        {0, 15, 0}, {0, -15, 0}, {0, 0, 30},
        {10, -10, 20}, {-20, 15, -10}
    };

    int maxWarnings = 0;
    for (int i = 0; i < 8; i++) {
        double rx = testOrients[i][0] * M_PI / 180.0;
        double ry = testOrients[i][1] * M_PI / 180.0;
        double rz = testOrients[i][2] * M_PI / 180.0;

        // R = Rz * Ry * Rx
        double crx = cos(rx), srx = sin(rx);
        double cry = cos(ry), sry = sin(ry);
        double crz = cos(rz), srz = sin(rz);
        double R00 = crz*cry;
        double R01 = crz*sry*srx - srz*crx;
        double R02 = crz*sry*crx + srz*srx;
        double R10 = srz*cry;
        double R11 = srz*sry*srx + crz*crx;
        double R12 = srz*sry*crx - crz*srx;
        double R20 = -sry;
        double R21 = cry*srx;
        double R22 = cry*crx;

        Vec3 rotOffset(
            R00*tcpOffset.x + R01*tcpOffset.y + R02*tcpOffset.z,
            R10*tcpOffset.x + R11*tcpOffset.y + R12*tcpOffset.z,
            R20*tcpOffset.x + R21*tcpOffset.y + R22*tcpOffset.z
        );
        Vec3 tcp(wristFixed.x + rotOffset.x,
                 wristFixed.y + rotOffset.y,
                 wristFixed.z + rotOffset.z);

        // Build positions: keep J1-J5 from base, override J6 with computed TCP
        Vec3 checkPos[7];
        for (int j = 0; j < 7; j++) checkPos[j] = basePos[j];
        checkPos[5] = wristFixed;  // J5 fixed
        checkPos[6] = tcp;         // J6 rotated

        SelfCollision::Result r = SelfCollision::check(checkPos);
        if (r.warning) maxWarnings++;
        CHECK(!r.reject,
              "orientation roundtrip should not reject in safe base config");
    }
    printf(" (max warnings across 8 orientations: %d, dist range OK) ", maxWarnings);
    PASS();
}

int main() {
    printf("=== SelfCollision Unit Tests ===\n");
    test_safe_config();
    test_wrist_near_base();
    test_reject_too_close();
    test_tcp_near_base();
    test_min_dist_tracking();
    test_orient_roundtrip();
    printf("\nResults: %d passed, %d failed\n", passed, failed);
    return failed ? 1 : 0;
}
