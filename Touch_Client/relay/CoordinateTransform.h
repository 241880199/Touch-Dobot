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

// ★★ 2026-09-23: 器件系 → 基座系 的【唯一一份】3×3（行主序），robot = M · touch。
// 【为什么要抽出来】平移路径(convertTouchToRobot)与姿态路径(RelayCore 的 robot_dR)各自写了一遍
//   同一张表，而姿态那一份的注释写着"从未被验证过" ⇒ 两边【可能】已经漂开，且漂开时没有任何东西会报。
//   抽成一处之后，"它们是不是同一张表"变成一条【可断言的】性质（见 test_frame_layout 的两条用例）。
// ⚠ Calibration::enabled 为真时，平移路径走的是标定出来的 R/t（含平移项）—— 那不是一张纯 3×3，
//   本函数【只描述兜底那一份】；调用者要按 Calibration::enabled 自己分支（与现在一致）。
inline void touchToRobotMatrix(double M[9]) {
    M[0] = 1.0; M[1] = 0.0; M[2] =  0.0;
    M[3] = 0.0; M[4] = 0.0; M[5] = -1.0;
    M[6] = 0.0; M[7] = 1.0; M[8] =  0.0;
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
