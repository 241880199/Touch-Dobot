#include "Kinematics.h"
#include <cmath>
#include <cstring>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace Kinematics {

// ===== 内部辅助: 4x4 矩阵运算 =====

static void mat4_identity(double M[4][4]) {
    memset(M, 0, 16 * sizeof(double));
    M[0][0] = M[1][1] = M[2][2] = M[3][3] = 1.0;
}

// T = T * translate(x, y, z)
static void mat4_translate(double T[4][4], double x, double y, double z) {
    T[0][3] += x;
    T[1][3] += y;
    T[2][3] += z;
}

// T = T * rotz(angle_rad)
static void mat4_rotz(double T[4][4], double a) {
    double c = cos(a), s = sin(a);
    double R[4][4] = {
        {c, -s, 0, 0},
        {s,  c, 0, 0},
        {0,  0, 1, 0},
        {0,  0, 0, 1}
    };
    // T = T * R
    double tmp[4][4];
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++) {
            tmp[i][j] = 0;
            for (int k = 0; k < 4; k++)
                tmp[i][j] += T[i][k] * R[k][j];
        }
    memcpy(T, tmp, 16 * sizeof(double));
}

// T = T * rotx(angle_rad)
static void mat4_rotx(double T[4][4], double a) {
    double c = cos(a), s = sin(a);
    double R[4][4] = {
        {1, 0,  0, 0},
        {0, c, -s, 0},
        {0, s,  c, 0},
        {0, 0,  0, 1}
    };
    double tmp[4][4];
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++) {
            tmp[i][j] = 0;
            for (int k = 0; k < 4; k++)
                tmp[i][j] += T[i][k] * R[k][j];
        }
    memcpy(T, tmp, 16 * sizeof(double));
}

// T = T * roty(angle_rad)
static void mat4_roty(double T[4][4], double a) {
    double c = cos(a), s = sin(a);
    double R[4][4] = {
        { c, 0, s, 0},
        { 0, 1, 0, 0},
        {-s, 0, c, 0},
        { 0, 0, 0, 1}
    };
    double tmp[4][4];
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++) {
            tmp[i][j] = 0;
            for (int k = 0; k < 4; k++)
                tmp[i][j] += T[i][k] * R[k][j];
        }
    memcpy(T, tmp, 16 * sizeof(double));
}

// ===== composeTransform: 计算末端变换矩阵 (对齐 MATLAB computeFK) =====

void composeTransform(const double joints[6], double T[4][4]) {
    double d2r = M_PI / 180.0;
    double j1 = joints[0] * d2r;
    double j2 = joints[1] * d2r;
    double j3 = joints[2] * d2r;
    double j4 = joints[3] * d2r;
    double j5 = joints[4] * d2r;
    double j6 = joints[5] * d2r;

    // 固定旋转角 (弧度) — 对齐 MATLAB
    double j2_ry = M_PI / 2.0;
    double j2_rx = M_PI / 2.0;
    double j4_rz = -M_PI / 2.0;
    double j5_rx = M_PI / 2.0;
    double j6_rx = -M_PI / 2.0;

    mat4_identity(T);

    // J1: base → Link1
    mat4_translate(T, 0, 0, J1_Z);
    mat4_rotz(T, j1);

    // J2: Link1 → Link2
    mat4_roty(T, j2_ry);
    mat4_rotx(T, j2_rx);
    mat4_rotz(T, j2);

    // J3: Link2 → Link3
    mat4_translate(T, J3_X, 0, 0);
    mat4_rotz(T, j3);

    // J4: Link3 → Link4
    mat4_translate(T, J4_X, 0, J4_Z);
    mat4_rotz(T, j4_rz);
    mat4_rotz(T, j4);

    // J5: Link4 → Link5
    mat4_translate(T, 0, J5_Y, 0);
    mat4_rotx(T, j5_rx);
    mat4_rotz(T, j5);

    // J6: Link5 → Link6
    mat4_translate(T, 0, J6_Y, 0);
    mat4_rotx(T, j6_rx);
    mat4_rotz(T, j6);
}

// ===== computeJointPositions: 计算各关节世界坐标 =====

void computeJointPositions(const double joints[6], Vec3 outPositions[7]) {
    double d2r = M_PI / 180.0;
    double j1 = joints[0] * d2r;
    double j2 = joints[1] * d2r;
    double j3 = joints[2] * d2r;
    double j4 = joints[3] * d2r;
    double j5 = joints[4] * d2r;
    double j6 = joints[5] * d2r;

    double j2_ry = M_PI / 2.0, j2_rx = M_PI / 2.0;
    double j4_rz = -M_PI / 2.0;
    double j5_rx = M_PI / 2.0;
    double j6_rx = -M_PI / 2.0;

    double T[4][4];
    mat4_identity(T);

    // Base origin
    outPositions[0] = Vec3(0, 0, 0);

    // J1
    mat4_translate(T, 0, 0, J1_Z);
    mat4_rotz(T, j1);
    outPositions[1] = Vec3(T[0][3], T[1][3], T[2][3]);

    // J2
    mat4_roty(T, j2_ry);
    mat4_rotx(T, j2_rx);
    mat4_rotz(T, j2);
    outPositions[2] = Vec3(T[0][3], T[1][3], T[2][3]);

    // J3
    mat4_translate(T, J3_X, 0, 0);
    mat4_rotz(T, j3);
    outPositions[3] = Vec3(T[0][3], T[1][3], T[2][3]);

    // J4
    mat4_translate(T, J4_X, 0, J4_Z);
    mat4_rotz(T, j4_rz);
    mat4_rotz(T, j4);
    outPositions[4] = Vec3(T[0][3], T[1][3], T[2][3]);

    // J5
    mat4_translate(T, 0, J5_Y, 0);
    mat4_rotx(T, j5_rx);
    mat4_rotz(T, j5);
    outPositions[5] = Vec3(T[0][3], T[1][3], T[2][3]);

    // J6 (末端)
    mat4_translate(T, 0, J6_Y, 0);
    mat4_rotx(T, j6_rx);
    mat4_rotz(T, j6);
    outPositions[6] = Vec3(T[0][3], T[1][3], T[2][3]);
}

Vec3 forwardPosition(const double joints[6]) {
    Vec3 positions[7];
    computeJointPositions(joints, positions);
    return positions[6];
}

} // namespace Kinematics
