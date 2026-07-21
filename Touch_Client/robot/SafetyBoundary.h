#pragma once
#include "../config/Config.h"
#include "../core/CoordinateTransform.h"
#include "../utils/MathUtils.h"
#include <algorithm>
#include <iostream>

namespace SafetyBoundary {

inline Vec3 clampToBoundary(const Vec3& target) {
    Vec3 clamped = target;
    bool warned = false;

    if (target.x < Config::SAFE_X_MIN) { clamped.x = Config::SAFE_X_MIN; warned = true; }
    if (target.x > Config::SAFE_X_MAX) { clamped.x = Config::SAFE_X_MAX; warned = true; }
    if (target.y < Config::SAFE_Y_MIN) { clamped.y = Config::SAFE_Y_MIN; warned = true; }
    if (target.y > Config::SAFE_Y_MAX) { clamped.y = Config::SAFE_Y_MAX; warned = true; }
    if (target.z < Config::SAFE_Z_MIN) { clamped.z = Config::SAFE_Z_MIN; warned = true; }
    if (target.z > Config::SAFE_Z_MAX) { clamped.z = Config::SAFE_Z_MAX; warned = true; }

    if (warned) {
        std::cerr << "[Safety] 目标超出安全边界，已钳位。原目标: ("
                  << target.x << "," << target.y << "," << target.z << ")" << std::endl;
    }
    return clamped;
}

inline double computeSpeedFactor(const Vec3& target) {
    // 计算距最近边界的距离（归一化到 [0, 1]）
    double rangeX = Config::SAFE_X_MAX - Config::SAFE_X_MIN;
    double rangeY = Config::SAFE_Y_MAX - Config::SAFE_Y_MIN;
    double rangeZ = Config::SAFE_Z_MAX - Config::SAFE_Z_MIN;

    double distX = std::min(target.x - Config::SAFE_X_MIN, Config::SAFE_X_MAX - target.x) / (rangeX * 0.5);
    double distY = std::min(target.y - Config::SAFE_Y_MIN, Config::SAFE_Y_MAX - target.y) / (rangeY * 0.5);
    double distZ = std::min(target.z - Config::SAFE_Z_MIN, Config::SAFE_Z_MAX - target.z) / (rangeZ * 0.5);
    double minDist = std::min({ distX, distY, distZ });

    if (minDist >= Config::SAFE_BOUNDARY_BUFFER_RATIO) return 1.0;
    if (minDist <= 0.0) return 0.1; // 已钳位到边界，最低 10% 速度
    return minDist / Config::SAFE_BOUNDARY_BUFFER_RATIO; // 线性衰减
}

} // namespace SafetyBoundary
