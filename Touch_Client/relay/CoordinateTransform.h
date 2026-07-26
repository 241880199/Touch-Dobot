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

// Calibration state — loaded from calibration.json at startup
// When enabled, convertTouchToRobot uses the calibrated rigid transform.
// When disabled, falls back to hardcoded axis mapping.
namespace Calibration {
    extern bool enabled;       // true when valid calibration is loaded
    extern double R[9];        // 3×3 rotation matrix (row-major)
    extern double t[3];        // translation vector (mm)
    extern double rmsError;    // RMS residual (mm)

    // Session-only data collection state
    extern bool collectMode;
    extern int  collectCount;  // number of recorded point pairs (max 50)

    // Stored point pairs: raw Touch device coords → robot actual position
    static const int MAX_COLLECT_POINTS = 50;
    extern double collectTouch[MAX_COLLECT_POINTS][3];  // raw device coords
    extern double collectRobot[MAX_COLLECT_POINTS][3];  // GetPose actual position

    bool load(const char* filepath);   // returns true on success
    bool save(const char* filepath);   // writes calibration.json
    void startCollect();
    void cancelCollect();
}

// Touch 原始坐标 (devicePos[3]) → 机械臂右手系
inline Vec3 convertTouchToRobot(const double devicePos[3]) {
    if (Calibration::enabled) {
        double x = devicePos[0], y = devicePos[1], z = devicePos[2];
        double* R_ = Calibration::R;
        double* T_ = Calibration::t;
        return Vec3(
            R_[0]*x + R_[1]*y + R_[2]*z + T_[0],
            R_[3]*x + R_[4]*y + R_[5]*z + T_[1],
            R_[6]*x + R_[7]*y + R_[8]*z + T_[2]
        );
    }
    // Fallback: hardcoded axis mapping
    return Vec3(
         devicePos[0],   // X → X
        -devicePos[2],   // Z(反转) → Y
         devicePos[1]    // Y → Z
    );
}

// Touch 原始坐标 (hduVector3Dd) → 机械臂右手系
inline Vec3 convertTouchToRobot(const hduVector3Dd& devicePos) {
    double arr[3] = { devicePos[0], devicePos[1], devicePos[2] };
    return convertTouchToRobot(arr);  // delegate to double[3] overload
}

// 计算相对位移
inline Vec3 computeDelta(const Vec3& current, const Vec3& base) {
    return current - base;
}

// 计算机械臂绝对目标位置（用户坐标系）
inline Vec3 computeTarget(const Vec3& robotBase, const Vec3& delta) {
    return robotBase + delta;
}
