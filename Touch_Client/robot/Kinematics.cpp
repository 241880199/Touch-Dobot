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

// T = T * translate(x, y, z)  — local-frame (correct)
static void mat4_translate(double T[4][4], double x, double y, double z) {
    double Trans[4][4] = {
        {1, 0, 0, x},
        {0, 1, 0, y},
        {0, 0, 1, z},
        {0, 0, 0, 1}
    };
    double tmp[4][4];
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++) {
            tmp[i][j] = 0;
            for (int k = 0; k < 4; k++)
                tmp[i][j] += T[i][k] * Trans[k][j];
        }
    memcpy(T, tmp, 16 * sizeof(double));
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

// ===== 几何雅可比矩阵 (6x6) =====

void jacobian(const double joints[6], double J[6][6]) {
    Vec3 positions[7];
    computeJointPositions(joints, positions);

    // 计算各关节在世界坐标系中的旋转轴 (z轴)
    Vec3 z_axes[6];
    double d2r = M_PI / 180.0;
    double j2_ry = M_PI / 2.0, j2_rx = M_PI / 2.0;
    double j4_rz = -M_PI / 2.0;
    double j5_rx = M_PI / 2.0;
    double j6_rx = -M_PI / 2.0;

    double T[4][4];
    mat4_identity(T);

    // J1 axis: world Z
    z_axes[0] = Vec3(0, 0, 1);

    // J2 axis
    mat4_translate(T, 0, 0, J1_Z);
    mat4_rotz(T, joints[0] * d2r);
    mat4_roty(T, j2_ry);
    mat4_rotx(T, j2_rx);
    z_axes[1] = Vec3(T[0][2], T[1][2], T[2][2]);

    // J3 axis
    mat4_rotz(T, joints[1] * d2r);
    mat4_translate(T, J3_X, 0, 0);
    z_axes[2] = Vec3(T[0][2], T[1][2], T[2][2]);

    // J4 axis
    mat4_rotz(T, joints[2] * d2r);
    mat4_translate(T, J4_X, 0, J4_Z);
    mat4_rotz(T, j4_rz);
    z_axes[3] = Vec3(T[0][2], T[1][2], T[2][2]);

    // J5 axis
    mat4_rotz(T, joints[3] * d2r);
    mat4_translate(T, 0, J5_Y, 0);
    mat4_rotx(T, j5_rx);
    z_axes[4] = Vec3(T[0][2], T[1][2], T[2][2]);

    // J6 axis
    mat4_rotz(T, joints[4] * d2r);
    mat4_translate(T, 0, J6_Y, 0);
    mat4_rotx(T, j6_rx);
    z_axes[5] = Vec3(T[0][2], T[1][2], T[2][2]);

    // 构建雅可比
    Vec3 p_ee = positions[6];
    memset(J, 0, 6 * 6 * sizeof(double));
    for (int i = 0; i < 6; i++) {
        Vec3 p_i = positions[i + 1]; // 关节i的位置 = positions[i+1]
        Vec3 d = {
            p_ee.x - p_i.x,
            p_ee.y - p_i.y,
            p_ee.z - p_i.z
        };
        // Jv_i = z_i × (p_ee - p_i)
        double jv_x = z_axes[i].y * d.z - z_axes[i].z * d.y;
        double jv_y = z_axes[i].z * d.x - z_axes[i].x * d.z;
        double jv_z = z_axes[i].x * d.y - z_axes[i].y * d.x;
        J[0][i] = jv_x; J[1][i] = jv_y; J[2][i] = jv_z;
        // Jw_i = z_i
        J[3][i] = z_axes[i].x; J[4][i] = z_axes[i].y; J[5][i] = z_axes[i].z;
    }
}

// ===== 6x6 对称矩阵 Jacobi 特征值分解 =====
// 输入: 对称矩阵 A[6][6], 输出: eigenvalues[6]
// 使用经典 Jacobi 旋转方法

static void jacobiEigenvalues(double A[6][6], double eigenvalues[6]) {
    double V[6][6];  // 特征向量 (累积旋转)
    memset(V, 0, sizeof(V));
    for (int i = 0; i < 6; i++) V[i][i] = 1.0;

    const int MAX_ITER = 50;
    const double TOL = 1e-12;

    for (int iter = 0; iter < MAX_ITER; iter++) {
        // 找到最大的非对角线元素
        int p = 0, q = 1;
        double maxOffDiag = fabs(A[0][1]);
        for (int i = 0; i < 6; i++)
            for (int j = i + 1; j < 6; j++)
                if (fabs(A[i][j]) > maxOffDiag) {
                    maxOffDiag = fabs(A[i][j]);
                    p = i; q = j;
                }

        if (maxOffDiag < TOL) break;

        // 计算旋转角
        double theta;
        if (fabs(A[p][p] - A[q][q]) < 1e-15) {
            theta = (A[p][q] > 0) ? M_PI / 4.0 : -M_PI / 4.0;
        } else {
            theta = 0.5 * atan2(2.0 * A[p][q], A[p][p] - A[q][q]);
        }

        double c = cos(theta), s = sin(theta);

        // 应用旋转: A = R^T * A * R
        // 更新行/列 p, q
        double old_pk[6], old_qk[6];
        for (int k = 0; k < 6; k++) {
            old_pk[k] = A[p][k];
            old_qk[k] = A[q][k];
        }
        for (int k = 0; k < 6; k++) {
            A[p][k] =  c * old_pk[k] + s * old_qk[k];
            A[q][k] = -s * old_pk[k] + c * old_qk[k];
            A[k][p] = A[p][k];
            A[k][q] = A[q][k];
        }
        A[p][p] = c * c * old_pk[p] + 2 * s * c * old_pk[q] + s * s * old_qk[q];
        A[q][q] = s * s * old_pk[p] - 2 * s * c * old_pk[q] + c * c * old_qk[q];
        A[p][q] = 0.0;
        A[q][p] = 0.0;
    }

    // 对角线即特征值
    for (int i = 0; i < 6; i++)
        eigenvalues[i] = fabs(A[i][i]);
}

double conditionNumber(double A[6][6]) {
    // 计算 A^T * A (6x6 对称)
    double ATA[6][6];
    memset(ATA, 0, sizeof(ATA));
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            for (int k = 0; k < 6; k++)
                ATA[i][j] += A[k][i] * A[k][j];

    double eigenvalues[6];
    jacobiEigenvalues(ATA, eigenvalues);

    double sigma_max = sqrt(eigenvalues[0]);
    double sigma_min = sqrt(eigenvalues[0]);
    for (int i = 1; i < 6; i++) {
        double s = sqrt(eigenvalues[i]);
        if (s > sigma_max) sigma_max = s;
        if (s < sigma_min) sigma_min = s;
    }

    if (sigma_min < 1e-12) return 1e12;  // 奇异, 返回极大值
    return sigma_max / sigma_min;
}

// ===== 6x6 线性方程组求解 (高斯消元) =====
static bool solveLinear6(double A[6][6], const double b[6], double x[6]) {
    double aug[6][7];
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) aug[i][j] = A[i][j];
        aug[i][6] = b[i];
    }
    // 前向消元
    for (int col = 0; col < 6; col++) {
        // 选主元
        int best = col;
        double bestVal = fabs(aug[col][col]);
        for (int row = col + 1; row < 6; row++)
            if (fabs(aug[row][col]) > bestVal) {
                bestVal = fabs(aug[row][col]);
                best = row;
            }
        if (bestVal < 1e-15) return false;
        if (best != col)
            for (int j = col; j < 7; j++) {
                double tmp = aug[col][j];
                aug[col][j] = aug[best][j];
                aug[best][j] = tmp;
            }
        // 消元
        double pivot = aug[col][col];
        for (int j = col; j < 7; j++) aug[col][j] /= pivot;
        for (int row = 0; row < 6; row++) {
            if (row == col) continue;
            double factor = aug[row][col];
            for (int j = col; j < 7; j++)
                aug[row][j] -= factor * aug[col][j];
        }
    }
    for (int i = 0; i < 6; i++) x[i] = aug[i][6];
    return true;
}

bool inverse(const Vec3& target, const double seed[6], double out[6]) {
    memcpy(out, seed, 6 * sizeof(double));

    double lambda = 0.1;
    double prevError = 1e12;

    for (int iter = 0; iter < 50; iter++) {
        Vec3 current = forwardPosition(out);
        double ex = target.x - current.x;
        double ey = target.y - current.y;
        double ez = target.z - current.z;
        double error = sqrt(ex*ex + ey*ey + ez*ez);

        if (error < 0.1) return true;  // 收敛: <0.1mm

        // 自适应阻尼
        if (error < prevError) lambda *= 0.5;
        else lambda *= 2.0;
        if (lambda < 0.001) lambda = 0.001;
        if (lambda > 10.0) lambda = 10.0;
        prevError = error;

        // 雅可比
        double J[6][6];
        jacobian(out, J);

        // J^T * J + λ*I
        double JTJ[6][6];
        memset(JTJ, 0, sizeof(JTJ));
        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 6; j++) {
                for (int k = 0; k < 6; k++)
                    JTJ[i][j] += J[k][i] * J[k][j];  // J^T * J
                if (i == j) JTJ[i][j] += lambda;
            }

        // J^T * error (只取前3行，因为只有位置误差)
        double JTe[6] = {0};
        for (int i = 0; i < 6; i++)
            JTe[i] = J[0][i] * ex + J[1][i] * ey + J[2][i] * ez;

        double dq[6];
        if (!solveLinear6(JTJ, JTe, dq)) return false;

        // 应用步长并钳位关节限位
        for (int i = 0; i < 6; i++) {
            out[i] += dq[i];
            // 钳位
            double lims[6][2] = {
                {J1_MIN, J1_MAX}, {J2_MIN, J2_MAX}, {J3_MIN, J3_MAX},
                {J4_MIN, J4_MAX}, {J5_MIN, J5_MAX}, {J6_MIN, J6_MAX}
            };
            if (out[i] < lims[i][0]) out[i] = lims[i][0];
            if (out[i] > lims[i][1]) out[i] = lims[i][1];
        }
    }

    return false;  // 未收敛
}

bool isWithinJointLimits(const double joints[6]) {
    if (joints[0] < J1_MIN || joints[0] > J1_MAX) return false;
    if (joints[1] < J2_MIN || joints[1] > J2_MAX) return false;
    if (joints[2] < J3_MIN || joints[2] > J3_MAX) return false;
    if (joints[3] < J4_MIN || joints[3] > J4_MAX) return false;
    if (joints[4] < J5_MIN || joints[4] > J5_MAX) return false;
    if (joints[5] < J6_MIN || joints[5] > J6_MAX) return false;
    return true;
}

} // namespace Kinematics
