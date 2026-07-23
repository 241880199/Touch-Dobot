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
