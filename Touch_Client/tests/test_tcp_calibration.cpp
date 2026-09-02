#define _USE_MATH_DEFINES
// Standalone test: TcpCalibration — RPY→矩阵 / 应用偏移 / N 点最小二乘求解
// Build: build_tcp_calibration_test.bat
// Run: test_tcp_calibration.exe

#include <iostream>
#include <cmath>
#include "../calibration/TcpCalibration.h"

static int g_passed = 0, g_failed = 0;

#define TEST(name) do { std::cout << "  " << #name << "... "; } while(0)
#define PASS() do { std::cout << "PASS" << std::endl; g_passed++; } while(0)
#define CHECK(cond) do { if (!(cond)) { std::cout << "FAIL: " << #cond << std::endl; g_failed++; return; } } while(0)

static void test_rpy_identity() {
    TEST(rpy_identity);
    double R[9];
    TcpCalibration::rpyToMatrix(0, 0, 0, R);
    CHECK(fabs(R[0]-1)<1e-9 && fabs(R[4]-1)<1e-9 && fabs(R[8]-1)<1e-9);
    CHECK(fabs(R[1])<1e-9 && fabs(R[3])<1e-9 && fabs(R[6])<1e-9);
    PASS();
}

static void test_rpy_rz90() {
    TEST(rpy_rz90);
    double R[9];
    TcpCalibration::rpyToMatrix(0, 0, M_PI/2, R);
    // Rz(90°): [[0,-1,0],[1,0,0],[0,0,1]]
    CHECK(fabs(R[0]-0)<1e-9 && fabs(R[1]+1)<1e-9);
    CHECK(fabs(R[3]-1)<1e-9 && fabs(R[4]-0)<1e-9);
    CHECK(fabs(R[8]-1)<1e-9);
    PASS();
}

static void test_apply_zero_offset() {
    TEST(apply_zero_offset);
    double pose[6] = {10, 20, 30, 0.1, 0.2, 0.3};
    double off[3] = {0,0,0};
    double tip[3];
    TcpCalibration::apply(pose, off, tip);
    CHECK(tip[0]==10 && tip[1]==20 && tip[2]==30);
    PASS();
}

static void test_apply_rotated_offset() {
    TEST(apply_rotated_offset);
    double pose[6] = {0,0,0, 0,0, M_PI/2};  // Rz 90°
    double off[3] = {1,0,0};
    double tip[3];
    TcpCalibration::apply(pose, off, tip);
    CHECK(fabs(tip[0]-0)<1e-9 && fabs(tip[1]-1)<1e-9 && fabs(tip[2]-0)<1e-9);
    PASS();
}

static void test_solve_recovers_offset() {
    TEST(solve_recovers_offset);
    double tTrue[3] = {5.0, -8.0, -150.0};
    double pTip[3] = {300.0, 100.0, 40.0};
    double rpy[4][3] = {
        {0.1, 0.2, 0.3},
        {-0.15, 0.05, 0.5},
        {0.25, -0.1, -0.2},
        {0.0, 0.3, -0.4}
    };
    double poses[4][6];
    for (int k = 0; k < 4; k++) {
        double R[9];
        TcpCalibration::rpyToMatrix(rpy[k][0], rpy[k][1], rpy[k][2], R);
        poses[k][0] = pTip[0] - (R[0]*tTrue[0]+R[1]*tTrue[1]+R[2]*tTrue[2]);
        poses[k][1] = pTip[1] - (R[3]*tTrue[0]+R[4]*tTrue[1]+R[5]*tTrue[2]);
        poses[k][2] = pTip[2] - (R[6]*tTrue[0]+R[7]*tTrue[1]+R[8]*tTrue[2]);
        poses[k][3] = rpy[k][0]; poses[k][4] = rpy[k][1]; poses[k][5] = rpy[k][2];
    }
    double off[3]; double rms;
    CHECK(TcpCalibration::solve(poses, 4, off, rms));
    CHECK(fabs(off[0]-tTrue[0]) < 1e-6);
    CHECK(fabs(off[1]-tTrue[1]) < 1e-6);
    CHECK(fabs(off[2]-tTrue[2]) < 1e-6);
    CHECK(rms < 1e-6);
    PASS();
}

static void test_solve_insufficient() {
    TEST(solve_insufficient);
    double poses[2][6] = {{0}};
    double off[3]; double rms;
    CHECK(!TcpCalibration::solve(poses, 2, off, rms));
    PASS();
}

static void test_solve_degenerate_same_orientation() {
    TEST(solve_degenerate_same_orientation);
    // 相同姿态 (R 全部相同) → dR=0 → 退化
    double poses[3][6] = {
        {0,0,0, 0,0,0},
        {10,0,0, 0,0,0},
        {20,0,0, 0,0,0}
    };
    double off[3]; double rms;
    CHECK(!TcpCalibration::solve(poses, 3, off, rms));
    PASS();
}

int main() {
    std::cout << "=== TcpCalibration Tests ===" << std::endl;
    test_rpy_identity();
    test_rpy_rz90();
    test_apply_zero_offset();
    test_apply_rotated_offset();
    test_solve_recovers_offset();
    test_solve_insufficient();
    test_solve_degenerate_same_orientation();
    std::cout << std::endl;
    std::cout << g_passed << " passed, " << g_failed << " failed" << std::endl;
    return g_failed > 0 ? 1 : 0;
}
