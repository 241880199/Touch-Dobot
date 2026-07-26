#pragma once
#include <vector>
#include <utility>
#include "../relay/CoordinateTransform.h"

// ===== Kabsch-Umeyama 刚体变换求解器 =====
// 输入: N 对 (Touch原始坐标, Robot实际位姿) 对应点
// 输出: 3×3 旋转矩阵 R, 平移向量 t, RMS残差

struct KabschResult {
    double R[9];        // 3×3 旋转矩阵 (row-major)
    double t[3];        // 平移向量 (mm)
    double rmsError;    // RMS 残差 (mm)
    bool valid;         // 求解成功标志
};

// pairs: 每对 (touch_raw, robot_actual), N >= 3
// touch 是 Touch 设备原始坐标 (未变换)
// robot 是 GetPose 返回的机械臂实际位置
KabschResult solveKabsch(const std::vector<std::pair<Vec3, Vec3>>& pairs);
