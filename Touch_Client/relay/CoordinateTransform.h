#pragma once
#include <cmath>
#include <HDU/hduVector.h>

struct Vec3 {
    double x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    Vec3 operator-(const Vec3& other) const {
        return Vec3(x - other.x, y - other.y, z - other.z);
    }
    Vec3 operator+(const Vec3& other) const {
        return Vec3(x + other.x, y + other.y, z + other.z);
    }
    double length() const {
        return std::sqrt(x * x + y * y + z * z);
    }
};

// Touch 原始坐标 (devicePos[3]) → 机械臂右手系
inline Vec3 convertTouchToRobot(const double devicePos[3]) {
    return Vec3(
         devicePos[0],   // X → X
        -devicePos[2],   // Z(反转) → Y
         devicePos[1]    // Y → Z
    );
}

// Touch 原始坐标 (hduVector3Dd) → 机械臂右手系
inline Vec3 convertTouchToRobot(const hduVector3Dd& devicePos) {
    return Vec3(
         devicePos[0],   // X → X
        -devicePos[2],   // Z(反转) → Y
         devicePos[1]    // Y → Z
    );
}

// 计算相对位移
inline Vec3 computeDelta(const Vec3& current, const Vec3& base) {
    return current - base;
}

// 计算机械臂绝对目标位置（用户坐标系）
inline Vec3 computeTarget(const Vec3& robotBase, const Vec3& delta) {
    return robotBase + delta;
}
