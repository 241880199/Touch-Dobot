# SafetyPredictor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 C++ 端实现分层安全预判系统，包括 FK/IK/雅可比运动学模块、硬边界+软边界检查、报警历史黑名单，集成到 sendPosition 流程。

**Architecture:** 新增 `robot/Kinematics` (FK+IK+Jacobian+SVD) 和 `safety/SafetyPredictor` (分层校验+黑名单) 两个模块。`sendPosition()` 发送 ServoP 前经过 SafetyPredictor 四层检查。`checkAlarm()` 检测到报警时记录位姿到黑名单。

**Tech Stack:** C++17, GLUT, WinSock2, CR3 机械臂 TCP 协议

## Global Constraints

- FK 参数必须与 MATLAB `relay_gui.m:computeFK()` 完全对齐
- 关节限位: J1±360°, J2±360°, J3±155°, J4±360°, J5±360°, J6±360°
- 工作半径: 620mm, Z 上限: 795mm
- 奇异条件数阈值: WARN=100, REJECT=500
- 使用现有 `Config::` 常量命名风格 (`const` 而非 `constexpr static`)
- 所有数学运算使用 `double` 精度
- 编译目标: MSBuild Release x64

---

### Task 1: Kinematics.h — 运动学模块头文件

**Files:**
- Create: `Touch_Client/robot/Kinematics.h`

**Interfaces:**
- Produces: `Kinematics::forwardPosition()`, `Kinematics::inverse()`, `Kinematics::jacobian()`, `Kinematics::conditionNumber()`, `Kinematics::isWithinJointLimits()`, joint position helper `Kinematics::computeJointPositions()`, constants `J1_Z`, `J3_X`, `J4_X`, `J4_Z`, `J5_Y`, `J6_Y`, joint limits

- [ ] **Step 1: Create Kinematics.h**

```cpp
#pragma once
#include "../relay/CoordinateTransform.h"

namespace Kinematics {

    // ===== CR3 URDF 参数 (mm) — 对齐 MATLAB computeFK =====
    const double J1_Z  = 128.3;   // 底座 → J2
    const double J3_X  = -274.0;  // J2 → J3 (大臂长)
    const double J4_X  = -230.0;  // J3 → J4 (小臂长)
    const double J4_Z  = 128.3;   // J4 Z 偏移
    const double J5_Y  = -116.0;  // J4 → J5
    const double J6_Y  = 105.0;   // J5 → J6

    // ===== 关节限位 (度) =====
    const double J1_MIN = -360.0, J1_MAX = 360.0;
    const double J2_MIN = -360.0, J2_MAX = 360.0;
    const double J3_MIN = -155.0, J3_MAX = 155.0;
    const double J4_MIN = -360.0, J4_MAX = 360.0;
    const double J5_MIN = -360.0, J5_MAX = 360.0;
    const double J6_MIN = -360.0, J6_MAX = 360.0;

    // ===== FK: 关节角(度) → 各关节世界坐标 (7×Vec3, 索引0=底座原点, 1~6=J1~J6位置) =====
    void computeJointPositions(const double joints[6], Vec3 outPositions[7]);

    // FK: 末端位置 (mm)
    Vec3 forwardPosition(const double joints[6]);

    // ===== 数值 IK: 目标 Cartesian → 关节角, DLS 方法 =====
    // seed: 初始猜测(度), out: 解(度), 返回是否收敛 (<0.1mm)
    bool inverse(const Vec3& target, const double seed[6], double out[6]);

    // ===== 6x6 几何雅可比矩阵 =====
    void jacobian(const double joints[6], double J[6][6]);

    // ===== 条件数 (基于 6x6 雅可比) =====
    double conditionNumber(double A[6][6]);

    // ===== 关节限位检查 =====
    bool isWithinJointLimits(const double joints[6]);

    // ===== 内部: 4x4 齐次变换 =====
    void composeTransform(const double joints[6], double T[4][4]);
}
```

### Task 2: Kinematics.cpp — FK 实现

**Files:**
- Create: `Touch_Client/robot/Kinematics.cpp`

**Interfaces:**
- Consumes: `Kinematics.h`, `CoordinateTransform.h`
- Produces: `computeJointPositions()`, `forwardPosition()`, `composeTransform()`

- [ ] **Step 1: Create Kinematics.cpp with FK implementation**

```cpp
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
```

- [ ] **Step 2: Commit**

```bash
git add Touch_Client/robot/Kinematics.h Touch_Client/robot/Kinematics.cpp
git commit -m "feat: add Kinematics module with FK implementation"
```

---

### Task 3: Kinematics.cpp — IK + Jacobian + SVD + 关节限位

**Files:**
- Modify: `Touch_Client/robot/Kinematics.cpp` (append new functions)

**Interfaces:**
- Consumes: Kinematics FK functions from Task 2
- Produces: `inverse()`, `jacobian()`, `conditionNumber()`, `isWithinJointLimits()`

- [ ] **Step 1: Append Jacobian implementation**

```cpp
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
```

- [ ] **Step 2: Append condition number (Jacobi eigenvalue on A^T*A)**

```cpp
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
```

- [ ] **Step 3: Append numerical IK (DLS method)**

```cpp
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
```

- [ ] **Step 4: Append joint limit check**

```cpp
bool isWithinJointLimits(const double joints[6]) {
    if (joints[0] < J1_MIN || joints[0] > J1_MAX) return false;
    if (joints[1] < J2_MIN || joints[1] > J2_MAX) return false;
    if (joints[2] < J3_MIN || joints[2] > J3_MAX) return false;
    if (joints[3] < J4_MIN || joints[3] > J4_MAX) return false;
    if (joints[4] < J5_MIN || joints[4] > J5_MAX) return false;
    if (joints[5] < J6_MIN || joints[5] > J6_MAX) return false;
    return true;
}
```

- [ ] **Step 5: Commit**

```bash
git add Touch_Client/robot/Kinematics.cpp
git commit -m "feat: add IK, Jacobian, SVD condition number, joint limits"
```

---

### Task 4: SafetyPredictor.h — 安全预判模块头文件

**Files:**
- Create: `Touch_Client/safety/SafetyPredictor.h`

**Interfaces:**
- Produces: `SafetyPredictor::instance()`, `evaluate()`, `addAlarmRecord()`, `nearestAlarmDistance()`, `loadAlarmLog()`, `saveAlarmLog()`, `alarmCount()`

- [ ] **Step 1: Create directory and SafetyPredictor.h**

```cpp
#pragma once
#include <vector>
#include <ctime>
#include "../relay/CoordinateTransform.h"
#include "../core/AppState.h"

struct AlarmRecord {
    double x, y, z;
    double j1, j2, j3, j4, j5, j6;
    time_t timestamp;
};

struct SafetyVerdict {
    enum Action { ALLOW = 0, WARN_SLOW = 1, REJECT = 2 };
    Action action;
    const char* reason;
    double speedFactor;  // 1.0 = 全速, 0.0 = 停止
};

class SafetyPredictor {
public:
    static SafetyPredictor& instance();

    // 主入口: 评估目标位姿是否安全 (每帧 ServoP 前调用)
    SafetyVerdict evaluate(const Vec3& target);

    // 报警黑名单管理
    void addAlarmRecord(const AppState::RobotPose& pose);
    double nearestAlarmDistance(const Vec3& target) const;
    int alarmCount() const { return (int)m_alarmList.size(); }

    // 获取最近一次 verdict (给 HUD 显示)
    SafetyVerdict lastVerdict() const { return m_lastVerdict; }

    // 持久化
    void loadAlarmLog(const char* path);
    void saveAlarmLog(const char* path) const;

    // 获取 IK 种子 (上次成功求解的关节角)
    const double* lastJoints() const { return m_lastJoints; }

private:
    SafetyPredictor() {
        for (int i = 0; i < 6; i++) m_lastJoints[i] = 0.0;
    }

    double m_lastJoints[6];
    std::vector<AlarmRecord> m_alarmList;
    SafetyVerdict m_lastVerdict = {SafetyVerdict::ALLOW, nullptr, 1.0};

    // 内部阈值
    static const double WORKSPACE_RADIUS;      // 620 mm
    static const double MAX_Z;                 // 795 mm
    static const double SINGULARITY_WARN;     // 100
    static const double SINGULARITY_REJECT;   // 500
    static const double ALARM_DANGER_R;       // 30 mm
    static const double ALARM_WARN_R;         // 80 mm
    static const double SINGULARITY_SPEED;    // 0.3
    static const double ALARM_DANGER_SPEED;   // 0.3
    static const double ALARM_WARN_SPEED;     // 0.5
};
```

- [ ] **Step 2: Commit**

```bash
git add Touch_Client/safety/SafetyPredictor.h
git commit -m "feat: add SafetyPredictor header"
```

---

### Task 5: SafetyPredictor.cpp — 实现

**Files:**
- Create: `Touch_Client/safety/SafetyPredictor.cpp`

**Interfaces:**
- Consumes: `SafetyPredictor.h`, `Kinematics.h`, `SafetyBoundary.h`, `AppState.h`
- Produces: `evaluate()`, `addAlarmRecord()`, `nearestAlarmDistance()`, `load/saveAlarmLog()`

- [ ] **Step 1: Create SafetyPredictor.cpp**

```cpp
#include "SafetyPredictor.h"
#include "../robot/Kinematics.h"
#include "../relay/SafetyBoundary.h"
#include "../config/Config.h"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <iostream>

// ===== 静态常量定义 =====
const double SafetyPredictor::WORKSPACE_RADIUS    = 620.0;
const double SafetyPredictor::MAX_Z               = 795.0;
const double SafetyPredictor::SINGULARITY_WARN    = 100.0;
const double SafetyPredictor::SINGULARITY_REJECT  = 500.0;
const double SafetyPredictor::ALARM_DANGER_R      = 30.0;
const double SafetyPredictor::ALARM_WARN_R        = 80.0;
const double SafetyPredictor::SINGULARITY_SPEED   = 0.3;
const double SafetyPredictor::ALARM_DANGER_SPEED  = 0.3;
const double SafetyPredictor::ALARM_WARN_SPEED    = 0.5;

SafetyPredictor& SafetyPredictor::instance() {
    static SafetyPredictor inst;
    return inst;
}

// ===== 主入口 =====

SafetyVerdict SafetyPredictor::evaluate(const Vec3& target) {
    // ===== 第1层: 硬边界 (O(1) 计算) =====

    // 1a. 工作半径
    double dist = sqrt(target.x * target.x + target.y * target.y);
    if (dist > WORKSPACE_RADIUS) {
        m_lastVerdict = {SafetyVerdict::REJECT, "超出工作半径 (620mm)", 0.0};
        return m_lastVerdict;
    }

    // 1b. Z 轴范围
    if (target.z < 0 || target.z > MAX_Z) {
        m_lastVerdict = {SafetyVerdict::REJECT, "Z轴超限 (0~795mm)", 0.0};
        return m_lastVerdict;
    }

    // 1c. 安全边界 (复用现有 SafetyBoundary)
    Vec3 clamped = SafetyBoundary::clampToBoundary(target);
    if (clamped.x != target.x || clamped.y != target.y || clamped.z != target.z) {
        m_lastVerdict = {SafetyVerdict::REJECT, "超出安全边界", 0.0};
        return m_lastVerdict;
    }

    // ===== 第2层: 运动学检查 =====

    // 2a. 数值 IK
    double joints[6];
    bool converged = Kinematics::inverse(target, m_lastJoints, joints);
    if (!converged) {
        m_lastVerdict = {SafetyVerdict::REJECT, "IK无解 (不可达)", 0.0};
        return m_lastVerdict;
    }

    // 2b. 关节限位
    if (!Kinematics::isWithinJointLimits(joints)) {
        m_lastVerdict = {SafetyVerdict::REJECT, "关节超出限位", 0.0};
        return m_lastVerdict;
    }

    // ===== 第3层: 奇异检测 (软边界) =====

    double J[6][6];
    Kinematics::jacobian(joints, J);
    double cond = Kinematics::conditionNumber(J);

    if (cond > SINGULARITY_REJECT) {
        m_lastVerdict = {SafetyVerdict::REJECT, "进入奇异构型 (cond>>500)", 0.0};
        return m_lastVerdict;
    }

    if (cond > SINGULARITY_WARN) {
        m_lastVerdict = {SafetyVerdict::WARN_SLOW, "接近奇异区域", SINGULARITY_SPEED};
        // 仍然缓存 joints (减速但不阻止)
        memcpy(m_lastJoints, joints, 6 * sizeof(double));
        return m_lastVerdict;
    }

    // ===== 第4层: 报警历史黑名单 =====

    double minDist = nearestAlarmDistance(target);
    if (minDist < ALARM_DANGER_R) {
        m_lastVerdict = {SafetyVerdict::WARN_SLOW, "靠近历史报警点 (<30mm)", ALARM_DANGER_SPEED};
        memcpy(m_lastJoints, joints, 6 * sizeof(double));
        return m_lastVerdict;
    }
    if (minDist < ALARM_WARN_R) {
        m_lastVerdict = {SafetyVerdict::WARN_SLOW, "接近历史报警区域 (<80mm)", ALARM_WARN_SPEED};
        memcpy(m_lastJoints, joints, 6 * sizeof(double));
        return m_lastVerdict;
    }

    // ===== 通过 =====
    memcpy(m_lastJoints, joints, 6 * sizeof(double));
    m_lastVerdict = {SafetyVerdict::ALLOW, nullptr, 1.0};
    return m_lastVerdict;
}

// ===== 报警黑名单 =====

void SafetyPredictor::addAlarmRecord(const AppState::RobotPose& pose) {
    AlarmRecord rec;
    rec.x = pose.x;
    rec.y = pose.y;
    rec.z = pose.z;
    rec.j1 = pose.j1;
    rec.j2 = pose.j2;
    rec.j3 = pose.j3;
    rec.j4 = pose.j4;
    rec.j5 = pose.j5;
    rec.j6 = pose.j6;
    rec.timestamp = time(nullptr);
    m_alarmList.push_back(rec);

    std::cout << "[Safety] Alarm recorded at (" << rec.x << "," << rec.y << "," << rec.z
              << ") — total alarms: " << m_alarmList.size() << std::endl;

    // 自动保存
    saveAlarmLog("alarms.log");
}

double SafetyPredictor::nearestAlarmDistance(const Vec3& target) const {
    double minDist = 1e12;
    for (const auto& a : m_alarmList) {
        double dx = target.x - a.x;
        double dy = target.y - a.y;
        double dz = target.z - a.z;
        double d = sqrt(dx*dx + dy*dy + dz*dz);
        if (d < minDist) minDist = d;
    }
    return minDist;
}

// ===== 持久化 =====

void SafetyPredictor::loadAlarmLog(const char* path) {
    FILE* f = nullptr;
    if (fopen_s(&f, path, "r") != 0 || !f) {
        std::cout << "[Safety] No existing alarm log at " << path << std::endl;
        return;
    }

    m_alarmList.clear();
    char line[512];
    while (fgets(line, sizeof(line), f)) {
        AlarmRecord rec;
        // 格式: YYYY-MM-DD HH:MM:SS, x=..., y=..., z=..., J1=..., ...
        char date[16], tim[16];
        if (sscanf_s(line, "%s %s x=%lf y=%lf z=%lf J1=%lf J2=%lf J3=%lf J4=%lf J5=%lf J6=%lf",
            date, (unsigned)sizeof(date), tim, (unsigned)sizeof(tim),
            &rec.x, &rec.y, &rec.z, &rec.j1, &rec.j2, &rec.j3, &rec.j4, &rec.j5, &rec.j6) == 11) {
            rec.timestamp = time(nullptr);
            m_alarmList.push_back(rec);
        }
    }
    fclose(f);
    std::cout << "[Safety] Loaded " << m_alarmList.size() << " alarm records from " << path << std::endl;
}

void SafetyPredictor::saveAlarmLog(const char* path) const {
    if (m_alarmList.empty()) return;

    FILE* f = nullptr;
    if (fopen_s(&f, path, "a") != 0 || !f) return;

    const auto& a = m_alarmList.back();  // 只追加最后一条
    char timeBuf[32];
    struct tm tmInfo;
    localtime_s(&tmInfo, &a.timestamp);
    strftime(timeBuf, sizeof(timeBuf), "%Y-%m-%d %H:%M:%S", &tmInfo);

    fprintf(f, "%s, x=%.2f, y=%.2f, z=%.2f, J1=%.2f, J2=%.2f, J3=%.2f, J4=%.2f, J5=%.2f, J6=%.2f\n",
        timeBuf, a.x, a.y, a.z, a.j1, a.j2, a.j3, a.j4, a.j5, a.j6);
    fclose(f);
}
```

- [ ] **Step 2: Commit**

```bash
git add Touch_Client/safety/SafetyPredictor.cpp
git commit -m "feat: implement SafetyPredictor with 4-layer evaluation and alarm blacklist"
```

---

### Task 6: Config.h — 添加安全常量

**Files:**
- Modify: `Touch_Client/config/Config.h`

**Interfaces:**
- Consumes: 现有 Config.h 结构
- Produces: 安全相关常量供 SafetyPredictor 和 HudOverlay 使用

- [ ] **Step 1: Add safety constants after existing SAFE_BOUNDARY_BUFFER_RATIO**

在 `Config.h` 的 `SAFE_BOUNDARY_BUFFER_RATIO` 行 (line 23) 之后添加:

```cpp
    // ========== SafetyPredictor 安全预判参数 ==========
    const double WORKSPACE_RADIUS         = 620.0;   // CR3 最大工作半径 (mm)
    const double ROBOT_MAX_Z              = 795.0;   // CR3 总高度 (mm)
    const double SINGULARITY_COND_WARN    = 100.0;   // 雅可比条件数: 警告阈值
    const double SINGULARITY_COND_REJECT  = 500.0;   // 雅可比条件数: 拒绝阈值
    const double ALARM_DANGER_RADIUS      = 30.0;    // 历史报警点: 危险半径 (mm)
    const double ALARM_WARN_RADIUS        = 80.0;    // 历史报警点: 警告半径 (mm)
```

- [ ] **Step 2: Commit**

```bash
git add Touch_Client/config/Config.h
git commit -m "feat: add safety predictor constants to Config.h"
```

---

### Task 7: RelayCore.cpp — 集成 SafetyPredictor

**Files:**
- Modify: `Touch_Client/relay/RelayCore.cpp`

**Interfaces:**
- Consumes: `SafetyPredictor.h`, 现有 `RelayCore.cpp` 中的 `sendPosition()` 和 `checkAlarm()`
- Produces: 集成 SafetyPredictor 的 sendPosition 和增强的 checkAlarm

- [ ] **Step 1: Add include at top of RelayCore.cpp**

在现有 includes 之后添加:

```cpp
#include "../safety/SafetyPredictor.h"
```

- [ ] **Step 2: Modify sendPosition() — 在发送 ServoP 前插入 SafetyPredictor 检查**

找到 `sendPosition()` 中 `clamped = SafetyBoundary::clampToBoundary(m_targetPos)` 之后、构造 ServoP 之前的位置 (约 line 146), 插入:

```cpp
    // ===== SafetyPredictor 预判 =====
    SafetyVerdict verdict = SafetyPredictor::instance().evaluate(clamped);

    if (verdict.action == SafetyVerdict::REJECT) {
        std::cerr << "[Safety] REJECT: " << verdict.reason
                  << " — target=(" << clamped.x << "," << clamped.y << "," << clamped.z << ")"
                  << std::endl;
        LeaveCriticalSection(&m_basePointLock);
        return;  // 不发送，目标位置回滚 (m_targetPos 已更新，但不下发)
    }

    // 速度衰减因子
    double speedMul = (verdict.action == SafetyVerdict::WARN_SLOW) ? verdict.speedFactor : 1.0;
```

- [ ] **Step 3: 在增量步长限制中应用速度衰减**

找到 `sendPosition()` 中的增量步长限制代码:
```cpp
    if (len > 3.0) {
        double scale = 3.0 / len;
        dx *= scale; dy *= scale; dz *= scale;
    }
```

修改为:
```cpp
    // 单步最大 3mm, 再乘以速度衰减因子
    double maxStep = 3.0 * speedMul;
    if (len > maxStep) {
        double scale = maxStep / len;
        dx *= scale; dy *= scale; dz *= scale;
    }
```

- [ ] **Step 4: Modify checkAlarm() — 检测到报警时记录位姿**

找到 `checkAlarm()` 中 `if (mode == 9 && !wasAlarm)` 块 (约 line 320):

修改为:
```cpp
        if (mode == 9 && !wasAlarm) {
            std::cout << "[Relay] 检测到机械臂报警 (mode=9)" << std::endl;

            // 立即获取当前位置并记录到 SafetyPredictor 黑名单
            queryPose();
            auto& app = appState;
            EnterCriticalSection(&app.robotPoseMutex);
            AppState::RobotPose alarmPose = app.robotActualPose;
            LeaveCriticalSection(&app.robotPoseMutex);

            SafetyPredictor::instance().addAlarmRecord(alarmPose);
        }
```

- [ ] **Step 5: Commit**

```bash
git add Touch_Client/relay/RelayCore.cpp
git commit -m "feat: integrate SafetyPredictor into sendPosition and checkAlarm"
```

---

### Task 8: HudOverlay.cpp — 显示安全状态

**Files:**
- Modify: `Touch_Client/render/HudOverlay.cpp`

**Interfaces:**
- Consumes: `SafetyPredictor.h`, 现有 `drawCoordPanel()` 的布局参数
- Produces: 在 Robot State 面板最后追加安全状态行

- [ ] **Step 1: Add include at top of HudOverlay.cpp**

```cpp
#include "../safety/SafetyPredictor.h"
```

- [ ] **Step 2: Append safety status lines at end of drawCoordPanel()**

在 `drawCoordPanel()` 函数末尾、右花括号之前 (在 TX 状态行之后), 添加:

```cpp
    // ===== SafetyPredictor 状态 =====
    ty -= 6;
    drawSeparatorLine(x + 4, x + w - 4, ty + 2);
    ty -= 4;

    SafetyVerdict v = SafetyPredictor::instance().lastVerdict();
    int alarmCount = SafetyPredictor::instance().alarmCount();

    // 安全状态灯
    ty -= lineH;
    switch (v.action) {
        case SafetyVerdict::ALLOW:
            glColor3f(0.35f, 0.90f, 0.50f);
            snprintf(buf, sizeof(buf), "Safety: OK");
            break;
        case SafetyVerdict::WARN_SLOW:
            glColor3f(1.0f, 0.78f, 0.28f);
            snprintf(buf, sizeof(buf), "Safety: WARN — %s (x%.0f%%)", v.reason, v.speedFactor * 100);
            break;
        case SafetyVerdict::REJECT:
            glColor3f(1.0f, 0.35f, 0.35f);
            snprintf(buf, sizeof(buf), "Safety: REJECT — %s", v.reason);
            break;
    }
    text2D(x + 6, ty, buf, GLUT_BITMAP_8_BY_13);

    // 报警历史计数
    ty -= lineH;
    if (alarmCount > 0) {
        glColor3f(1.0f, 0.55f, 0.25f);
        snprintf(buf, sizeof(buf), "Alarms: %d recorded", alarmCount);
    } else {
        glColor3f(0.45f, 0.50f, 0.55f);
        snprintf(buf, sizeof(buf), "Alarms: 0");
    }
    text2D(x + 6, ty, buf, GLUT_BITMAP_8_BY_13);
```

- [ ] **Step 2: Commit**

```bash
git add Touch_Client/render/HudOverlay.cpp
git commit -m "feat: display safety status and alarm count in HUD"
```

---

### Task 9: vcxproj — 编译集成

**Files:**
- Modify: `Touch_Client/Touch_Client.vcxproj`

- [ ] **Step 1: Add Kinematics and SafetyPredictor to vcxproj**

在 `<ItemGroup>` 中, 在 `<!-- robot -->` 区域添加 Kinematics, 新增 `<!-- safety -->` 区域:

```xml
    <!-- robot -->
    <ClInclude Include="robot\RobotConnection.h" />
    <ClCompile Include="robot\RobotConnection.cpp" />
    <ClInclude Include="robot\Kinematics.h" />
    <ClCompile Include="robot\Kinematics.cpp" />
    <!-- safety -->
    <ClInclude Include="safety\SafetyPredictor.h" />
    <ClCompile Include="safety\SafetyPredictor.cpp" />
```

精确插入位置: 在 `RobotConnection.cpp` (line 83) 之后, `<!-- relay -->` (line 84) 之前。

- [ ] **Step 2: Commit**

```bash
git add Touch_Client/Touch_Client.vcxproj
git commit -m "build: add Kinematics and SafetyPredictor to vcxproj"
```

---

### Task 10: 编译验证

- [ ] **Step 1: Build**

```bash
cmd.exe //c 'D:\Projects\Touch\Touch_Client\build.bat'
```

Expected: Build OK, 无错误。

- [ ] **Step 2: Verify output files exist**

```bash
ls -la D:/Projects/Touch/Touch_Client/x64/Release/Touch_Client.exe
```

Expected: 文件存在, 时间戳最新。

- [ ] **Step 3: Commit if any build-related changes**

```bash
git status
# 如果有 vcxproj 过滤器文件变更, 一并提交
```
