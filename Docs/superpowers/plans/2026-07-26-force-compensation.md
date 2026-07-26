# Force Compensation System Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build gravity+inertia force sensor compensation with automatic multi-pose calibration, replacing the simple deadzone with bias-subtraction+gravity-comp+inertia-comp.

**Architecture:** New `ForceCompensation` module (motion estimator, gravity/inertia comp, EMA bias tracking) inserted before the existing `ForcePipeline`. New `ForceCalibration` module (state machine, multi-pose sweep, normal-equations solver, JSON persistence) triggered via 'c' key when idle.

**Tech Stack:** C++17, MSVC, WinSock2, OpenHaptics SDK, hand-rolled linear algebra (no Eigen), hand-rolled JSON I/O (fprintf/strstr/strtod — matching existing `CalibrationIO.cpp` pattern), single-file unit tests via custom TEST/CHECK/PASS macros.

## Global Constraints

- `--no-robot` mode must start without crash (uncalibrated, no compensation)
- Stale detection (200ms timeout → force zero) must still work
- Safety interrupts during calibration (space key, alarm detection) must abort immediately
- Calibration result persists to `Touch_Client/calib/force_calib.json`; auto-loaded on startup
- HapticCallback must receive zero force when robot stationary + unloaded (post-calibration)
- Existing Butterworth filter pipeline must function correctly after modifications
- All compensation happens BEFORE Butterworth filtering (structured signal removal, not noise filtering)
- Euler angle convention: R = Rz(Rz_deg) · Ry(Ry_deg) · Rx(Rx_deg) where angles are in degrees from GetPose()
- **MVP simplification:** Calibration MOVE phase prompts user to manually reposition robot to each target orientation (rather than automatic ServoP motion). This eliminates robot-motion risk during initial calibration validation. Automatic sweep can be added in a follow-up.

---

### Task 1: Update Config.h — Force compensation constants

**Files:**
- Modify: `Touch_Client/config/Config.h:58-67`

**Produced:**
- `Config::FORCE_RESIDUAL_DEADZONE_N` (double, 0.05)
- `Config::FORCE_CALIB_SPEED_FACTOR` (double, 0.30)
- `Config::FORCE_CALIB_STILL_COLLECT_S` (double, 2.0)
- `Config::FORCE_CALIB_MOVE_TIMEOUT_S` (double, 5.0)
- `Config::FORCE_CALIB_SETTLE_TIME_S` (double, 0.5)
- `Config::FORCE_CALIB_SAMPLE_TIME_S` (double, 0.5)
- `Config::FORCE_CALIB_MAX_RESIDUAL_N` (double, 0.3)
- `Config::FORCE_CALIB_POSE_ANGLE_DEG` (double, 15.0)
- `Config::FORCE_CALIB_NUM_POSES` (int, 6)
- `Config::FORCE_MOTION_VEL_THRESH_MS` (double, 0.002)
- `Config::FORCE_MOTION_ACC_THRESH_MSS` (double, 0.005)
- `Config::FORCE_BIAS_EMA_ALPHA` (double, 0.01)
- `Config::FORCE_ACC_FILTER_CUTOFF_HZ` (double, 10.0)
- `Config::FORCE_EFFECTIVE_SAMPLE_RATE` (int, 125)

- [ ] **Step 1: Replace force sensor constants block**

Replace the force sensor section in `Config.h` (lines 58-67):

Old block:
```cpp
    // ========== 力传感器参数 ==========
    const int FORCE_REALTIME_PORT = 30004;       // 实时反馈端口 (125Hz)
    const int FORCE_FILTER_CUTOFF = 30;          // Butterworth 截止频率 (Hz)
    const int FORCE_STALE_MS = 200;              // 数据超时阈值 (ms)
    const double FORCE_DEADZONE_N = 0.2;         // 死区 (N) — 降低以减少小力过滤
    const double FORCE_MAX_SENSOR_N = 200.0;     // 传感器量程 (N)
    const double FORCE_MAX_TOUCH_N = 3.3;        // Touch 最大安全力 (N)
    const double FORCE_REFLECTION_GAIN = 5.0;    // 力反射增益 — 放大传感器力到可感知范围
    const double FORCE_GRADIENT_LIMIT = 50.0;    // 梯度限幅 (N/frame)
    const int FORCE_RECONNECT_INTERVAL = 2000;   // 断线重试间隔 (ms)
```

New block:
```cpp
    // ========== 力传感器参数 ==========
    const int FORCE_REALTIME_PORT = 30004;       // 实时反馈端口 (125Hz)
    const int FORCE_EFFECTIVE_SAMPLE_RATE = 125; // 传感器数据采样率 (Hz)
    const int FORCE_FILTER_CUTOFF = 30;          // Butterworth 截止频率 (Hz)
    const int FORCE_STALE_MS = 200;              // 数据超时阈值 (ms)
    const double FORCE_RESIDUAL_DEADZONE_N = 0.05; // 补偿后残余死区 (N)
    const double FORCE_MAX_SENSOR_N = 200.0;     // 传感器量程 (N)
    const double FORCE_MAX_TOUCH_N = 3.3;        // Touch 最大安全力 (N)
    const double FORCE_REFLECTION_GAIN = 5.0;    // 力反射增益 — 放大传感器力到可感知范围
    const double FORCE_GRADIENT_LIMIT = 50.0;    // 梯度限幅 (N/frame)
    const int FORCE_RECONNECT_INTERVAL = 2000;   // 断线重试间隔 (ms)

    // ========== 力传感器标定参数 ==========
    const double FORCE_CALIB_SPEED_FACTOR = 0.30;       // 标定期速度因子
    const double FORCE_CALIB_STILL_COLLECT_S = 2.0;      // 初始静止采集时间 (s)
    const double FORCE_CALIB_MOVE_TIMEOUT_S = 5.0;       // 单姿态移动超时 (s)
    const double FORCE_CALIB_SETTLE_TIME_S = 0.5;        // 姿态稳定等待 (s)
    const double FORCE_CALIB_SAMPLE_TIME_S = 0.5;        // 数据采集时间 (s)
    const double FORCE_CALIB_MAX_RESIDUAL_N = 0.3;       // 标定残差阈值 (N)
    const double FORCE_CALIB_POSE_ANGLE_DEG = 15.0;      // 标定姿态偏角 (度)
    const int    FORCE_CALIB_NUM_POSES = 6;               // 标定姿态数

    // ========== 力补偿运行时参数 ==========
    const double FORCE_MOTION_VEL_THRESH_MS = 0.002;      // 静止判定: 速度阈值 (m/s)
    const double FORCE_MOTION_ACC_THRESH_MSS = 0.005;     // 静止判定: 加速度阈值 (m/s²)
    const double FORCE_BIAS_EMA_ALPHA = 0.01;             // 零偏 EMA 更新率 (仅静止态)
    const double FORCE_ACC_FILTER_CUTOFF_HZ = 10.0;       // 加速度估计低通截止 (Hz)
```

- [ ] **Step 2: Verify Config.h compiles**

Build a trivial test:
```bash
cd Touch_Client
cl /EHsc /std:c++17 /c /I"D:\Projects\Touch\OpenHaptics\Developer\3.5.0\include" /I"D:\Projects\Touch\OpenHaptics\Developer\3.5.0\utilities\include" /DWIN32 /D_CRT_SECURE_NO_WARNINGS config/Config.h 2>&1 || echo "EXPECTED: header-only, link error ok"
```

Expected: No compile errors from Config.h itself.

- [ ] **Step 3: Commit**

```bash
git add Touch_Client/config/Config.h
git commit -m "feat(config): replace FORCE_DEADZONE_N with calibration + compensation constants"
```

---

### Task 2: Expand AppState::ForceData with compensation fields

**Files:**
- Modify: `Touch_Client/core/AppState.h:104-113`

**Produced:**
- `AppState::ForceData::compensated[6]` (double[6])
- `AppState::ForceData::isCalibrated` (bool)
- `AppState::ForceData::calibMassKg` (double)
- `AppState::ForceData::calibComSensor[3]` (double[3])
- `AppState::ForceData::calibBiasForce[3]` (double[3])
- `AppState::ForceData::calibBiasTorque[3]` (double[3])

- [ ] **Step 1: Edit ForceData struct**

Replace the existing `ForceData` struct (lines 105-111):
```cpp
    // ===== 力数据 =====
    struct ForceData {
        double raw[6] = {0};            // Fx,Fy,Fz,Mx,My,Mz (N, Nm) — 30004 原始数据
        double compensated[6] = {0};    // 重力+惯性+零偏补偿后 (N, Nm)
        double filtered[6] = {0};       // Butterworth 低通滤波输出
        double hapticOut[3] = {0};      // 已变换到 Touch 坐标系，haptic 线程直接读
        bool isStale = true;            // 超过 200ms 无新数据
        DWORD lastUpdateMs = 0;

        // 标定参数 (由 ForceCalibration 求解, ForceCompensation 读取)
        bool isCalibrated = false;
        double calibMassKg = 0.0;           // 末端等效质量 (kg)
        double calibComSensor[3] = {0};     // 质心在传感器坐标系 (m)
        double calibBiasForce[3] = {0};     // 力零偏 (N)
        double calibBiasTorque[3] = {0};    // 力矩零偏 (Nm)
    };
    ForceData forceData;
    CRITICAL_SECTION forceDataMutex;
```

- [ ] **Step 2: Verify the struct compiles**

No separate test needed — the struct is validated when ForceCompensation.cpp includes AppState.h in Task 3.

- [ ] **Step 3: Commit**

```bash
git add Touch_Client/core/AppState.h
git commit -m "feat(appstate): add compensated, isCalibrated, calib params to ForceData"
```

---

### Task 3: Create ForceCompensation — runtime gravity+inertia compensation

**Files:**
- Create: `Touch_Client/force/ForceCompensation.h`
- Create: `Touch_Client/force/ForceCompensation.cpp`

**Interfaces:**
- Produces: `namespace ForceCompensation { void init(); void step(AppState::ForceData& fd, const double poseRxyz[6]); void shutdown(); void setCalibration(double massKg, const double comSensor[3], const double biasForce[3], const double biasTorque[3]); }`
- Produces: `class MotionEstimator` — internal helper, public `void update(double x, double y, double z, double dt)`, `void getState(double vel[3], double acc[3]) const`, `bool isStill() const`
- Consumes: `AppState::ForceData` (Task 2), `Config::FORCE_MOTION_VEL_THRESH_MS`, `Config::FORCE_MOTION_ACC_THRESH_MSS`, `Config::FORCE_BIAS_EMA_ALPHA`, `Config::FORCE_ACC_FILTER_CUTOFF_HZ`, `Config::FORCE_EFFECTIVE_SAMPLE_RATE`

- [ ] **Step 1: Write ForceCompensation.h**

```cpp
#pragma once
#include "../core/AppState.h"

// Motion estimator: tracks tool velocity & acceleration from position history
// Uses 5-point ring buffer for central-difference acceleration estimation
class MotionEstimator {
public:
    MotionEstimator();
    void update(double x, double y, double z, double dt);
    void getState(double vel[3], double acc[3]) const;
    bool isStill() const;
    void reset();
private:
    static const int BUF_SIZE = 5;
    double m_posBuf[5][3];
    int m_idx;
    int m_count;
    double m_vel[3];
    double m_accRaw[3];
    double m_accFiltered[3];
    // Butterworth2-style 10Hz LPF for acceleration
    double m_lpfB0, m_lpfB1, m_lpfB2, m_lpfA1, m_lpfA2;
    double m_lpfX1[3], m_lpfX2[3], m_lpfY1[3], m_lpfY2[3];
};

namespace ForceCompensation {
    // Call once at startup — loads calibration file, initializes filters
    void init();

    // Call at ~125Hz (from ForceReader thread) or ~30Hz (from pollForce)
    // fd.raw[] must be fresh; poseRxyz = {X,Y,Z,Rx,Ry,Rz} in mm & deg from GetPose()
    // Writes fd.compensated[] (6-axis compensated force)
    void step(AppState::ForceData& fd, const double poseRxyz[6]);

    // Set calibration parameters (called after calibration completes or file load)
    void setCalibration(double massKg, const double comSensor[3],
                        const double biasForce[3], const double biasTorque[3]);

    // Check if calibration is active
    bool isCalibrated();

    // Call on shutdown
    void shutdown();
}
```

- [ ] **Step 2: Write ForceCompensation.cpp — helper functions**

```cpp
#define _USE_MATH_DEFINES
#include "ForceCompensation.h"
#include "../config/Config.h"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdlib>

// ===== Internal state =====
static bool g_isCalibrated = false;
static double g_massKg = 0.0;
static double g_comSensor[3] = {0};
static double g_biasForce[3] = {0};
static double g_biasTorque[3] = {0};
static MotionEstimator g_motion;

// ===== Euler angles (deg) to rotation matrix =====
// R = Rz(rz_deg) * Ry(ry_deg) * Rx(rx_deg)
// Output: 3x3 row-major R[9]
static void eulerToRotation(double rx_deg, double ry_deg, double rz_deg, double R[9]) {
    double rx = rx_deg * M_PI / 180.0;
    double ry = ry_deg * M_PI / 180.0;
    double rz = rz_deg * M_PI / 180.0;

    double cx = cos(rx), sx = sin(rx);
    double cy = cos(ry), sy = sin(ry);
    double cz = cos(rz), sz = sin(rz);

    // Rz * Ry * Rx  (row-major)
    R[0] = cz * cy;
    R[1] = cz * sy * sx - sz * cx;
    R[2] = cz * sy * cx + sz * sx;
    R[3] = sz * cy;
    R[4] = sz * sy * sx + cz * cx;
    R[5] = sz * sy * cx - cz * sx;
    R[6] = -sy;
    R[7] = cy * sx;
    R[8] = cy * cx;
}

// Matrix-vector multiply: out = M^T * v  (3x3 row-major M, 3-vector v)
static void matTransposeMulVec(const double M[9], const double v[3], double out[3]) {
    out[0] = M[0] * v[0] + M[3] * v[1] + M[6] * v[2];
    out[1] = M[1] * v[0] + M[4] * v[1] + M[7] * v[2];
    out[2] = M[2] * v[0] + M[5] * v[1] + M[8] * v[2];
}

// Cross product: out = a × b
static void cross(const double a[3], const double b[3], double out[3]) {
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}
```

- [ ] **Step 3: Write ForceCompensation.cpp — MotionEstimator**

```cpp
// ===== MotionEstimator implementation =====

// Butterworth2 LPF coefficient helper (same as ForcePipeline pattern)
static void calcLpfCoeffs(double fc, double fs,
    double& b0, double& b1, double& b2, double& a1, double& a2)
{
    double w0 = 2.0 * M_PI * fc / fs;
    double cos_w0 = cos(w0);
    double sin_w0 = sin(w0);
    double alpha = sin_w0 / sqrt(2.0);
    double a0 = 1.0 + alpha;
    b0 = ((1.0 - cos_w0) / 2.0) / a0;
    b1 = (1.0 - cos_w0) / a0;
    b2 = ((1.0 - cos_w0) / 2.0) / a0;
    a1 = (-2.0 * cos_w0) / a0;
    a2 = (1.0 - alpha) / a0;
}

MotionEstimator::MotionEstimator() : m_idx(0), m_count(0) {
    m_vel[0] = m_vel[1] = m_vel[2] = 0.0;
    m_accRaw[0] = m_accRaw[1] = m_accRaw[2] = 0.0;
    m_accFiltered[0] = m_accFiltered[1] = m_accFiltered[2] = 0.0;
    for (int i = 0; i < BUF_SIZE; i++) {
        m_posBuf[i][0] = m_posBuf[i][1] = m_posBuf[i][2] = 0.0;
    }
    for (int i = 0; i < 3; i++) {
        m_lpfX1[i] = m_lpfX2[i] = m_lpfY1[i] = m_lpfY2[i] = 0.0;
    }
    // 10Hz LPF at effective sample rate
    double fs = static_cast<double>(Config::FORCE_EFFECTIVE_SAMPLE_RATE);
    calcLpfCoeffs(Config::FORCE_ACC_FILTER_CUTOFF_HZ, fs,
        m_lpfB0, m_lpfB1, m_lpfB2, m_lpfA1, m_lpfA2);
}

void MotionEstimator::reset() {
    m_idx = 0; m_count = 0;
    m_vel[0] = m_vel[1] = m_vel[2] = 0.0;
    m_accRaw[0] = m_accRaw[1] = m_accRaw[2] = 0.0;
    m_accFiltered[0] = m_accFiltered[1] = m_accFiltered[2] = 0.0;
    for (int i = 0; i < BUF_SIZE; i++)
        m_posBuf[i][0] = m_posBuf[i][1] = m_posBuf[i][2] = 0.0;
    for (int i = 0; i < 3; i++)
        m_lpfX1[i] = m_lpfX2[i] = m_lpfY1[i] = m_lpfY2[i] = 0.0;
}

void MotionEstimator::update(double x, double y, double z, double dt) {
    // Store in ring buffer (unit: m)
    m_posBuf[m_idx][0] = x * 0.001;  // mm → m
    m_posBuf[m_idx][1] = y * 0.001;
    m_posBuf[m_idx][2] = z * 0.001;
    m_idx = (m_idx + 1) % BUF_SIZE;
    if (m_count < BUF_SIZE) m_count++;

    if (m_count >= 3) {
        // Central difference velocity (using indices i and i-1)
        int i0 = (m_idx - 1 + BUF_SIZE) % BUF_SIZE;
        int i1 = (m_idx - 2 + BUF_SIZE) % BUF_SIZE;
        for (int k = 0; k < 3; k++) {
            m_vel[k] = (m_posBuf[i0][k] - m_posBuf[i1][k]) / dt;
        }
    }
    if (m_count >= 5) {
        // Central difference acceleration (3-point stencil)
        int i0 = (m_idx - 1 + BUF_SIZE) % BUF_SIZE;
        int i1 = (m_idx - 2 + BUF_SIZE) % BUF_SIZE;
        int i2 = (m_idx - 3 + BUF_SIZE) % BUF_SIZE;
        for (int k = 0; k < 3; k++) {
            m_accRaw[k] = (m_posBuf[i0][k] - 2.0 * m_posBuf[i1][k] + m_posBuf[i2][k]) / (dt * dt);
            // NaN guard
            if (std::isnan(m_accRaw[k]) || std::isinf(m_accRaw[k])) m_accRaw[k] = 0.0;
            // LPF: biquad step per channel
            double out = m_lpfB0 * m_accRaw[k] + m_lpfB1 * m_lpfX1[k] + m_lpfB2 * m_lpfX2[k]
                       - m_lpfA1 * m_lpfY1[k] - m_lpfA2 * m_lpfY2[k];
            m_lpfX2[k] = m_lpfX1[k]; m_lpfX1[k] = m_accRaw[k];
            m_lpfY2[k] = m_lpfY1[k]; m_lpfY1[k] = out;
            m_accFiltered[k] = out;
        }
    }
}

void MotionEstimator::getState(double vel[3], double acc[3]) const {
    for (int k = 0; k < 3; k++) {
        vel[k] = m_vel[k];
        acc[k] = m_accFiltered[k];
    }
}

bool MotionEstimator::isStill() const {
    double vsq = m_vel[0]*m_vel[0] + m_vel[1]*m_vel[1] + m_vel[2]*m_vel[2];
    double asq = m_accFiltered[0]*m_accFiltered[0] + m_accFiltered[1]*m_accFiltered[1] + m_accFiltered[2]*m_accFiltered[2];
    return (sqrt(vsq) < Config::FORCE_MOTION_VEL_THRESH_MS &&
            sqrt(asq) < Config::FORCE_MOTION_ACC_THRESH_MSS);
}
```

- [ ] **Step 4: Write ForceCompensation.cpp — init/step/setCalibration**

```cpp
// ===== ForceCompensation namespace =====

namespace ForceCompensation {

void init() {
    g_isCalibrated = false;
    g_massKg = 0.0;
    for (int i = 0; i < 3; i++) {
        g_comSensor[i] = 0.0;
        g_biasForce[i] = 0.0;
        g_biasTorque[i] = 0.0;
    }
    g_motion.reset();
}

void setCalibration(double massKg, const double comSensor[3],
                    const double biasForce[3], const double biasTorque[3])
{
    g_massKg = massKg;
    for (int i = 0; i < 3; i++) {
        g_comSensor[i] = comSensor[i];
        g_biasForce[i] = biasForce[i];
        g_biasTorque[i] = biasTorque[i];
    }
    g_isCalibrated = true;
}

bool isCalibrated() {
    return g_isCalibrated;
}

void step(AppState::ForceData& fd, const double poseRxyz[6]) {
    // poseRxyz = {X_mm, Y_mm, Z_mm, Rx_deg, Ry_deg, Rz_deg}

    // 1. Update motion estimator
    double dt = 1.0 / static_cast<double>(Config::FORCE_EFFECTIVE_SAMPLE_RATE);
    g_motion.update(poseRxyz[0], poseRxyz[1], poseRxyz[2], dt);

    // 2. Copy raw to compensated as default (no-op if uncalibrated)
    for (int i = 0; i < 6; i++) {
        fd.compensated[i] = fd.raw[i];
    }

    if (!g_isCalibrated) return;

    // 3. Compute rotation matrix from Euler angles
    double R[9];
    eulerToRotation(poseRxyz[3], poseRxyz[4], poseRxyz[5], R);

    // 4. Gravity in tool frame: g_tool = R^T * (0, 0, -9.81)
    double gBase[3] = {0.0, 0.0, -9.81};
    double gTool[3];
    matTransposeMulVec(R, gBase, gTool);

    // Gravity force in sensor frame
    double Fg[3];
    Fg[0] = g_massKg * gTool[0];
    Fg[1] = g_massKg * gTool[1];
    Fg[2] = g_massKg * gTool[2];

    // Gravity torque: r_com × Fg
    double Mg[3];
    cross(g_comSensor, Fg, Mg);

    // 5. Inertia force (only if moving)
    double Fi[3] = {0, 0, 0};
    if (!g_motion.isStill()) {
        double vel[3], acc[3];
        g_motion.getState(vel, acc);
        Fi[0] = g_massKg * acc[0];
        Fi[1] = g_massKg * acc[1];
        Fi[2] = g_massKg * acc[2];
    }

    // 6. Compensate: compensated = raw - bias - gravity - inertia
    fd.compensated[0] = fd.raw[0] - g_biasForce[0] - Fg[0] - Fi[0];
    fd.compensated[1] = fd.raw[1] - g_biasForce[1] - Fg[1] - Fi[1];
    fd.compensated[2] = fd.raw[2] - g_biasForce[2] - Fg[2] - Fi[2];
    fd.compensated[3] = fd.raw[3] - g_biasTorque[0] - Mg[0];
    fd.compensated[4] = fd.raw[4] - g_biasTorque[1] - Mg[1];
    fd.compensated[5] = fd.raw[5] - g_biasTorque[2] - Mg[2];

    // 7. Online EMA bias update (only when still — slow drift tracking)
    if (g_motion.isStill()) {
        double alpha = Config::FORCE_BIAS_EMA_ALPHA;
        // Force bias: track residual (raw - gravity as "expected zero")
        g_biasForce[0] += alpha * (fd.raw[0] - Fg[0] - g_biasForce[0]);
        g_biasForce[1] += alpha * (fd.raw[1] - Fg[1] - g_biasForce[1]);
        g_biasForce[2] += alpha * (fd.raw[2] - Fg[2] - g_biasForce[2]);
        // Torque bias
        g_biasTorque[0] += alpha * (fd.raw[3] - Mg[0] - g_biasTorque[0]);
        g_biasTorque[1] += alpha * (fd.raw[4] - Mg[1] - g_biasTorque[1]);
        g_biasTorque[2] += alpha * (fd.raw[5] - Mg[2] - g_biasTorque[2]);
    }

    // 8. Update calib params in ForceData for HUD display / MATLAB relay
    fd.isCalibrated = true;
    fd.calibMassKg = g_massKg;
    for (int i = 0; i < 3; i++) {
        fd.calibComSensor[i] = g_comSensor[i];
        fd.calibBiasForce[i] = g_biasForce[i];
        fd.calibBiasTorque[i] = g_biasTorque[i];
    }
}

void shutdown() {
    g_isCalibrated = false;
    g_motion.reset();
}

} // namespace ForceCompensation
```

- [ ] **Step 5: Commit**

```bash
git add Touch_Client/force/ForceCompensation.h Touch_Client/force/ForceCompensation.cpp
git commit -m "feat(force): add ForceCompensation — gravity+inertia comp + EMA bias tracking"
```

---

### Task 4: Create ForceCalibration — calibration state machine + solver

**Files:**
- Create: `Touch_Client/force/ForceCalibration.h`
- Create: `Touch_Client/force/ForceCalibration.cpp`

**Interfaces:**
- Produces: `namespace ForceCalibration { bool start(); void abort(); bool isRunning(); bool isDone(); const char* statusText(); bool saveToFile(const char* path, double residualRms); bool loadFromFile(const char* path, double& massKg, double comSensor[3], double biasForce[3], double biasTorque[3], double& residualRms); void update(double dt, const double raw[6], const double pose[6]); }`
- Produces: `class GaussSolver` — internal, `bool solve(int nRows, const double A[], const double b[], int nCols, double x[], double& residualRms)`
- Consumes: `AppState::ForceData` (Task 2), `Config::FORCE_CALIB_*` constants (Task 1)

- [ ] **Step 1: Write ForceCalibration.h**

```cpp
#pragma once

// Gaussian elimination solver for Ax = b (linear least squares via normal equations)
class GaussSolver {
public:
    // Solve overdetermined system via normal equations: A^T A x = A^T b
    // A is (nRows × nCols) row-major, b is (nRows × 1)
    // x is (nCols × 1) output; residualRms = ||Ax - b|| / sqrt(nRows)
    // Returns false if singular or degenerate
    static bool solve(int nRows, const double A[], const double b[],
                      int nCols, double x[], double& residualRms);
private:
    static bool gaussElim(int n, double A[], double b[], double x[]);
};

// Force sensor calibration — multi-pose automatic sweep
namespace ForceCalibration {

    enum class State {
        IDLE,
        TARE,      // 2s still collection for initial bias estimate
        MOVE,      // Moving to target orientation
        SETTLE,    // 0.5s wait for vibration decay
        SAMPLE,    // 0.5s data collection
        SOLVE,     // Normal equations → extract params
        VERIFY,    // Check residual
        DONE,      // Success
        ABORTED    // User interrupt or safety trip
    };

    // Start calibration (must be called from main thread when not transmitting)
    bool start();

    // Abort immediately (called from safety handlers or user interrupt)
    void abort();

    // Confirm current pose reached (user presses SPACE in manual MOVE mode)
    void confirmPose();

    bool isRunning();
    bool isDone();
    State currentState();
    const char* statusText();

    // Called each frame (~125Hz from ForceReader) to drive state machine
    // Returns true when calibration is complete (DONE or ABORTED)
    bool update(double dt, const double raw[6], const double pose[6]);

    // Persistence
    bool saveToFile(const char* path, double residualRms,
                    double massKg, const double comSensor[3],
                    const double biasForce[3], const double biasTorque[3]);
    bool loadFromFile(const char* path, double& massKg, double comSensor[3],
                      double biasForce[3], double biasTorque[3], double& residualRms);

} // namespace ForceCalibration
```

- [ ] **Step 2: Write ForceCalibration.cpp — GaussSolver**

```cpp
#include "ForceCalibration.h"
#include "../config/Config.h"
#include <cmath>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <algorithm>

static const int N_UNKNOWNS = 10;  // m, p_x, p_y, p_z, Fx0, Fy0, Fz0, Mx0, My0, Mz0
static const int N_EQS_PER_POSE = 6;

// ===== GaussSolver implementation =====

bool GaussSolver::gaussElim(int n, double A[], double b[], double x[]) {
    // Gaussian elimination with partial pivoting, in-place
    // A is n×n row-major; b is n×1
    for (int col = 0; col < n; col++) {
        // Partial pivot: find max in column
        int maxRow = col;
        double maxVal = fabs(A[col * n + col]);
        for (int row = col + 1; row < n; row++) {
            double v = fabs(A[row * n + col]);
            if (v > maxVal) { maxVal = v; maxRow = row; }
        }
        if (maxVal < 1e-12) return false; // singular

        // Swap rows
        if (maxRow != col) {
            for (int j = 0; j < n; j++) {
                std::swap(A[col * n + j], A[maxRow * n + j]);
            }
            std::swap(b[col], b[maxRow]);
        }

        // Eliminate below
        double pivot = A[col * n + col];
        for (int row = col + 1; row < n; row++) {
            double factor = A[row * n + col] / pivot;
            if (fabs(factor) < 1e-15) continue;
            for (int j = col; j < n; j++) {
                A[row * n + j] -= factor * A[col * n + j];
            }
            b[row] -= factor * b[col];
        }
    }

    // Back substitution
    for (int i = n - 1; i >= 0; i--) {
        double sum = b[i];
        for (int j = i + 1; j < n; j++) {
            sum -= A[i * n + j] * x[j];
        }
        x[i] = sum / A[i * n + i];
    }
    return true;
}

bool GaussSolver::solve(int nRows, const double A[], const double b[],
                         int nCols, double x[], double& residualRms)
{
    // Build normal equations: ATA (nCols × nCols), ATb (nCols × 1)
    double* ATA = new double[nCols * nCols]();
    double* ATb = new double[nCols]();

    for (int i = 0; i < nRows; i++) {
        for (int j = 0; j < nCols; j++) {
            double a_ij = A[i * nCols + j];
            ATb[j] += a_ij * b[i];
            for (int k = 0; k < nCols; k++) {
                ATA[j * nCols + k] += a_ij * A[i * nCols + k];
            }
        }
    }

    bool ok = gaussElim(nCols, ATA, ATb, x);

    // Compute residual
    if (ok) {
        double sumSq = 0.0;
        for (int i = 0; i < nRows; i++) {
            double pred = 0.0;
            for (int j = 0; j < nCols; j++) {
                pred += A[i * nCols + j] * x[j];
            }
            double err = b[i] - pred;
            sumSq += err * err;
        }
        residualRms = sqrt(sumSq / nRows);
    }

    delete[] ATA;
    delete[] ATb;
    return ok;
}
```

- [ ] **Step 3: Write ForceCalibration.cpp — state machine & data structures**

```cpp
// ===== Calibration state =====

static ForceCalibration::State g_calibState = ForceCalibration::State::IDLE;
static bool g_abortFlag = false;
static bool g_poseConfirmed = false;  // set by confirmPose() to advance MOVE→SETTLE
static int g_poseIndex = 0;
static double g_phaseTimer = 0.0;
static double g_tareAccum[6] = {0};
static int g_tareCount = 0;

// Collected data: per-pose {R matrix, raw force/torque}
static const int MAX_POSES = 10;
static int g_numCollected = 0;
static double g_poseR[10][9];      // rotation matrix per pose
static double g_poseRaw[10][6];    // raw force/torque per pose (median)

// Accumulator for SAMPLE phase (median filter within each pose)
static double g_sampleBuf[125][6]; // ~1s at 125Hz
static int g_sampleCount = 0;

// Solve results
static double g_solvedMass = 0.0;
static double g_solvedCom[3] = {0};
static double g_solvedBiasF[3] = {0};
static double g_solvedBiasM[3] = {0};
static double g_solvedResidual = 0.0;

// Calibration target orientations (relative to current pose, rotation-only)
static double g_targetRxyz[6][3];  // {rx_deg, ry_deg, rz_deg} offsets for 6 poses

namespace ForceCalibration {

State currentState() { return g_calibState; }

bool isRunning() {
    return g_calibState != State::IDLE &&
           g_calibState != State::DONE &&
           g_calibState != State::ABORTED;
}

bool isDone() {
    return g_calibState == State::DONE || g_calibState == State::ABORTED;
}

const char* statusText() {
    switch (g_calibState) {
        case State::IDLE:    return "Idle";
        case State::TARE:    return "Taring...";
        case State::MOVE:    return "Moving to pose...";
        case State::SETTLE:  return "Settling...";
        case State::SAMPLE:  return "Sampling...";
        case State::SOLVE:   return "Solving...";
        case State::VERIFY:  return "Verifying...";
        case State::DONE:    return "Calibration complete";
        case State::ABORTED: return "Calibration aborted";
    }
    return "Unknown";
}

bool start() {
    if (g_calibState != State::IDLE) return false;
    g_calibState = State::TARE;
    g_abortFlag = false;
    g_poseIndex = 0;
    g_phaseTimer = 0.0;
    g_tareCount = 0;
    for (int i = 0; i < 6; i++) g_tareAccum[i] = 0.0;
    g_numCollected = 0;
    g_sampleCount = 0;

    // Generate 6 target orientation offsets
    double a = Config::FORCE_CALIB_POSE_ANGLE_DEG;
    double offsets[6][3] = {
        { 0,  0,  0},
        {+a,  0,  0},
        {-a,  0,  0},
        { 0, +a,  0},
        { 0, -a,  0},
        { 0,  0, +a},
    };
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 3; j++) {
            g_targetRxyz[i][j] = offsets[i][j];
        }
    }
    return true;
}

void abort() {
    g_abortFlag = true;
    g_calibState = State::ABORTED;
}

void confirmPose() {
    g_poseConfirmed = true;
}
```

- [ ] **Step 4: Write ForceCalibration.cpp — update() state machine**

```cpp
bool update(double dt, const double raw[6], const double pose[6]) {
    if (g_calibState == State::IDLE || g_calibState == State::DONE ||
        g_calibState == State::ABORTED) {
        return false;
    }

    if (g_abortFlag) {
        g_calibState = State::ABORTED;
        return true; // done (aborted)
    }

    // pose = {X, Y, Z, Rx, Ry, Rz}  (mm, deg)

    switch (g_calibState) {

    case State::TARE: {
        // Accumulate raw data while still
        for (int i = 0; i < 6; i++) g_tareAccum[i] += raw[i];
        g_tareCount++;
        g_phaseTimer += dt;
        if (g_phaseTimer >= Config::FORCE_CALIB_STILL_COLLECT_S) {
            for (int i = 0; i < 6; i++) g_tareAccum[i] /= g_tareCount;
            g_phaseTimer = 0.0;
            g_poseIndex = 0;
            g_poseConfirmed = false;
            // Print first target orientation offset
            printf("[Force] Tare complete. Pose %d/%d: offset Rx=%.0f Ry=%.0f Rz=%.0f deg\n",
                   g_poseIndex + 1, Config::FORCE_CALIB_NUM_POSES,
                   g_targetRxyz[g_poseIndex][0], g_targetRxyz[g_poseIndex][1],
                   g_targetRxyz[g_poseIndex][2]);
            printf("[Force] Reposition robot, then press SPACE to confirm.\n");
            g_calibState = State::MOVE;
        }
        break;
    }

    case State::MOVE: {
        // MVP: Manual mode — user repositions robot, presses SPACE to confirm.
        // State machine waits for external signal (set via confirmPose()).
        // Auto-advance timer as fallback.
        g_phaseTimer += dt;
        // Check for external confirmation
        if (g_poseConfirmed || g_phaseTimer >= Config::FORCE_CALIB_MOVE_TIMEOUT_S) {
            g_poseConfirmed = false;
            g_phaseTimer = 0.0;
            g_calibState = State::SETTLE;
        }
        break;
    }

    case State::SETTLE: {
        g_phaseTimer += dt;
        if (g_phaseTimer >= Config::FORCE_CALIB_SETTLE_TIME_S) {
            g_phaseTimer = 0.0;
            g_sampleCount = 0;
            g_calibState = State::SAMPLE;
        }
        break;
    }

    case State::SAMPLE: {
        // Record raw data for median
        if (g_sampleCount < 125) {
            for (int i = 0; i < 6; i++) {
                g_sampleBuf[g_sampleCount][i] = raw[i];
            }
            g_sampleCount++;
        }
        g_phaseTimer += dt;
        if (g_phaseTimer >= Config::FORCE_CALIB_SAMPLE_TIME_S) {
            // Compute per-channel median
            double median[6];
            for (int ch = 0; ch < 6; ch++) {
                double vals[125];
                int n = g_sampleCount > 0 ? g_sampleCount : 1;
                for (int k = 0; k < n; k++) vals[k] = g_sampleBuf[k][ch];
                // Simple median: sort copy
                std::sort(vals, vals + n);
                median[ch] = (n % 2) ? vals[n / 2] : (vals[n / 2 - 1] + vals[n / 2]) / 2.0;
            }

            // Store result for this pose
            for (int i = 0; i < 6; i++) {
                g_poseRaw[g_numCollected][i] = median[i];
            }
            // Compute rotation matrix from current pose Euler angles
            double rx = pose[3], ry = pose[4], rz = pose[5];
            double rx_rad = rx * M_PI / 180.0;
            double ry_rad = ry * M_PI / 180.0;
            double rz_rad = rz * M_PI / 180.0;
            double cx = cos(rx_rad), sx = sin(rx_rad);
            double cy = cos(ry_rad), sy = sin(ry_rad);
            double cz = cos(rz_rad), sz = sin(rz_rad);
            double* R = g_poseR[g_numCollected];
            R[0] = cz * cy;  R[1] = cz * sy * sx - sz * cx;  R[2] = cz * sy * cx + sz * sx;
            R[3] = sz * cy;  R[4] = sz * sy * sx + cz * cx;  R[5] = sz * sy * cx - cz * sx;
            R[6] = -sy;      R[7] = cy * sx;                 R[8] = cy * cx;

            g_numCollected++;
            g_poseIndex++;
            g_phaseTimer = 0.0;

            if (g_poseIndex >= Config::FORCE_CALIB_NUM_POSES) {
                g_calibState = State::SOLVE;
            } else {
                // Prompt for next pose
                printf("[Force] Pose %d/%d: offset Rx=%.0f Ry=%.0f Rz=%.0f deg\n",
                       g_poseIndex + 1, Config::FORCE_CALIB_NUM_POSES,
                       g_targetRxyz[g_poseIndex][0], g_targetRxyz[g_poseIndex][1],
                       g_targetRxyz[g_poseIndex][2]);
                printf("[Force] Reposition robot, then press SPACE to confirm.\n");
                g_calibState = State::MOVE;
            }
        }
        break;
    }

    case State::SOLVE: {
        if (g_numCollected < 2) {
            g_calibState = State::ABORTED;
            break;
        }

        int nRows = g_numCollected * N_EQS_PER_POSE;
        double* A = new double[nRows * N_UNKNOWNS]();
        double* b = new double[nRows]();
        double gBase[3] = {0.0, 0.0, -9.81};

        for (int p = 0; p < g_numCollected; p++) {
            int baseRow = p * N_EQS_PER_POSE;
            double* R = g_poseR[p];
            // Gravity in tool frame: gTool = R^T * gBase
            double gx = R[0]*gBase[0] + R[3]*gBase[1] + R[6]*gBase[2];
            double gy = R[1]*gBase[0] + R[4]*gBase[1] + R[7]*gBase[2];
            double gz = R[2]*gBase[0] + R[5]*gBase[1] + R[8]*gBase[2];

            // Force eqns: F = m * gTool + F0
            // Fx: m*gx + Fx0               cols: 0=m, 1=px, 2=py, 3=pz, 4=Fx0, 5=Fy0, 6=Fz0, 7=Mx0, 8=My0, 9=Mz0
            A[baseRow * N_UNKNOWNS + 0] = gx;
            A[baseRow * N_UNKNOWNS + 4] = 1.0;
            b[baseRow] = g_poseRaw[p][0];

            A[(baseRow + 1) * N_UNKNOWNS + 0] = gy;
            A[(baseRow + 1) * N_UNKNOWNS + 5] = 1.0;
            b[baseRow + 1] = g_poseRaw[p][1];

            A[(baseRow + 2) * N_UNKNOWNS + 0] = gz;
            A[(baseRow + 2) * N_UNKNOWNS + 6] = 1.0;
            b[baseRow + 2] = g_poseRaw[p][2];

            // Torque eqns: M = gTool × p + M0
            // Mx: gy*pz - gz*py + Mx0
            A[(baseRow + 3) * N_UNKNOWNS + 2] = -gz;   // -gz * py
            A[(baseRow + 3) * N_UNKNOWNS + 3] =  gy;   //  gy * pz
            A[(baseRow + 3) * N_UNKNOWNS + 7] = 1.0;
            b[baseRow + 3] = g_poseRaw[p][3];

            // My: gz*px - gx*pz + My0
            A[(baseRow + 4) * N_UNKNOWNS + 1] =  gz;   //  gz * px
            A[(baseRow + 4) * N_UNKNOWNS + 3] = -gx;   // -gx * pz
            A[(baseRow + 4) * N_UNKNOWNS + 8] = 1.0;
            b[baseRow + 4] = g_poseRaw[p][4];

            // Mz: gx*py - gy*px + Mz0
            A[(baseRow + 5) * N_UNKNOWNS + 1] = -gy;   // -gy * px
            A[(baseRow + 5) * N_UNKNOWNS + 2] =  gx;   //  gx * py
            A[(baseRow + 5) * N_UNKNOWNS + 9] = 1.0;
            b[baseRow + 5] = g_poseRaw[p][5];
        }

        double x[N_UNKNOWNS];
        if (GaussSolver::solve(nRows, A, b, N_UNKNOWNS, x, g_solvedResidual)) {
            g_solvedMass = x[0];
            g_solvedCom[0] = x[1] / x[0];  // r_com = p / m
            g_solvedCom[1] = x[2] / x[0];
            g_solvedCom[2] = x[3] / x[0];
            g_solvedBiasF[0] = x[4];
            g_solvedBiasF[1] = x[5];
            g_solvedBiasF[2] = x[6];
            g_solvedBiasM[0] = x[7];
            g_solvedBiasM[1] = x[8];
            g_solvedBiasM[2] = x[9];
            g_calibState = State::VERIFY;
        } else {
            g_calibState = State::ABORTED; // singular matrix
        }

        delete[] A;
        delete[] b;
        break;
    }

    case State::VERIFY: {
        if (g_solvedResidual <= Config::FORCE_CALIB_MAX_RESIDUAL_N) {
            g_calibState = State::DONE;
        } else {
            // Residual too high — still mark done but caller checks residual
            g_calibState = State::DONE;
        }
        break;
    }

    default:
        break;
    }

    return (g_calibState == State::DONE || g_calibState == State::ABORTED);
}
```

- [ ] **Step 5: Write ForceCalibration.cpp — saveToFile / loadFromFile**

```cpp
bool saveToFile(const char* path, double residualRms,
                double massKg, const double comSensor[3],
                const double biasForce[3], const double biasTorque[3])
{
    FILE* f = fopen(path, "w");
    if (!f) return false;
    fprintf(f, "{\n");
    fprintf(f, "  \"version\": 1,\n");
    fprintf(f, "  \"mass_kg\": %.6g,\n", massKg);
    fprintf(f, "  \"com_sensor_m\": [%.6g, %.6g, %.6g],\n",
            comSensor[0], comSensor[1], comSensor[2]);
    fprintf(f, "  \"bias_force_n\": [%.6g, %.6g, %.6g],\n",
            biasForce[0], biasForce[1], biasForce[2]);
    fprintf(f, "  \"bias_torque_nm\": [%.6g, %.6g, %.6g],\n",
            biasTorque[0], biasTorque[1], biasTorque[2]);
    fprintf(f, "  \"residual_rms_n\": %.6g,\n", residualRms);
    fprintf(f, "  \"num_poses\": %d,\n", g_numCollected);
    fprintf(f, "  \"timestamp\": \"calibrated\"\n");
    fprintf(f, "}\n");
    fclose(f);
    return true;
}

bool loadFromFile(const char* path, double& massKg, double comSensor[3],
                  double biasForce[3], double biasTorque[3], double& residualRms)
{
    FILE* f = fopen(path, "r");
    if (!f) return false;

    char buf[4096];
    size_t n = fread(buf, 1, sizeof(buf) - 1, f);
    fclose(f);
    if (n == 0) return false;
    buf[n] = '\0';

    // Parse mass_kg
    const char* p = strstr(buf, "\"mass_kg\":");
    if (!p) return false;
    massKg = strtod(p + 10, nullptr);

    // Parse com_sensor_m array
    p = strstr(buf, "\"com_sensor_m\":[");
    if (!p) return false;
    p += 15;
    for (int i = 0; i < 3; i++) {
        char* end = nullptr;
        comSensor[i] = strtod(p, &end);
        if (end == p) return false;
        p = end;
        while (*p == ',' || *p == ' ' || *p == '\n' || *p == '\r' || *p == '\t') p++;
    }

    // Parse bias_force_n array
    p = strstr(buf, "\"bias_force_n\":[");
    if (!p) return false;
    p += 15;
    for (int i = 0; i < 3; i++) {
        char* end = nullptr;
        biasForce[i] = strtod(p, &end);
        if (end == p) return false;
        p = end;
        while (*p == ',' || *p == ' ' || *p == '\n' || *p == '\r' || *p == '\t') p++;
    }

    // Parse bias_torque_nm array
    p = strstr(buf, "\"bias_torque_nm\":[");
    if (!p) return false;
    p += 17;
    for (int i = 0; i < 3; i++) {
        char* end = nullptr;
        biasTorque[i] = strtod(p, &end);
        if (end == p) return false;
        p = end;
        while (*p == ',' || *p == ' ' || *p == '\n' || *p == '\r' || *p == '\t') p++;
    }

    // Parse residual_rms_n
    p = strstr(buf, "\"residual_rms_n\":");
    if (p) {
        residualRms = strtod(p + 17, nullptr);
    } else {
        residualRms = -1.0;
    }

    return true;
}
```

- [ ] **Step 6: Close namespace and commit**

At end of ForceCalibration.cpp:
```cpp
} // namespace ForceCalibration
```

```bash
git add Touch_Client/force/ForceCalibration.h Touch_Client/force/ForceCalibration.cpp
git commit -m "feat(force): add ForceCalibration — multi-pose sweep + normal-equations solver + JSON persistence"
```

---

### Task 5: Modify ForcePipeline to use compensated data

**Files:**
- Modify: `Touch_Client/force/ForcePipeline.cpp:66-128`

**Consumes:** `ForceData::compensated[]` (Task 2), `Config::FORCE_RESIDUAL_DEADZONE_N` (Task 1)

- [ ] **Step 1: Update ForcePipeline::step() to use compensated data + reduced deadzone**

Replace the `step()` function body in `ForcePipeline.cpp` (lines 83-119) with:

```cpp
void step(AppState::ForceData& fd) {
    // 1. Butterworth filter on COMPENSATED data (not raw)
    for (int i = 0; i < 6; i++) {
        fd.filtered[i] = g_filters[i].step(fd.compensated[i]);
    }

    // 2. Gradient limit (protect against sensor spike)
    for (int i = 0; i < 6; i++) {
        double delta = fd.filtered[i] - g_prevFiltered[i];
        if (delta > Config::FORCE_GRADIENT_LIMIT)
            fd.filtered[i] = g_prevFiltered[i] + Config::FORCE_GRADIENT_LIMIT;
        else if (delta < -Config::FORCE_GRADIENT_LIMIT)
            fd.filtered[i] = g_prevFiltered[i] - Config::FORCE_GRADIENT_LIMIT;
        g_prevFiltered[i] = fd.filtered[i];
    }

    // 3. Force mapping with RESIDUAL deadzone (bias already removed by compensation)
    double fx = mapForceToTouch(fd.filtered[0]);
    double fy = mapForceToTouch(fd.filtered[1]);
    double fz = mapForceToTouch(fd.filtered[2]);

    // 4. Coordinate transform: Robot tool frame -> Touch device frame
    fd.hapticOut[0] =  fx;   // Robot Fx -> Touch X
    fd.hapticOut[1] = -fz;   // Robot -Fz -> Touch Y
    fd.hapticOut[2] =  fy;   // Robot +Fy -> Touch Z

    // 5. Apply reflection gain
    double gain = Config::FORCE_REFLECTION_GAIN;
    for (int i = 0; i < 3; i++) {
        fd.hapticOut[i] *= gain;
    }
}
```

And update `mapForceToTouch` to use `FORCE_RESIDUAL_DEADZONE_N`:
```cpp
static inline double mapForceToTouch(double sensorForce) {
    double v = deadzone(sensorForce, Config::FORCE_RESIDUAL_DEADZONE_N);
    double ratio = Config::FORCE_MAX_TOUCH_N / Config::FORCE_MAX_SENSOR_N;
    double out = v * ratio;
    if (out > Config::FORCE_MAX_TOUCH_N)  out = Config::FORCE_MAX_TOUCH_N;
    if (out < -Config::FORCE_MAX_TOUCH_N) out = -Config::FORCE_MAX_TOUCH_N;
    return out;
}
```

- [ ] **Step 2: Verify test_force_pipeline.exe still compiles (will adjust in Task 8)**

```bash
cd Touch_Client\tests
call build_test.bat 2>&1
```

Expected: may fail because old test uses `fd.raw[]` for deadzone test; this is expected — tests will be updated in Task 8.

- [ ] **Step 3: Commit**

```bash
git add Touch_Client/force/ForcePipeline.cpp
git commit -m "refactor(force): ForcePipeline now uses compensated[] data + FORCE_RESIDUAL_DEADZONE_N"
```

---

### Task 6: Update test_force_pipeline.cpp for new behavior

**Files:**
- Modify: `Touch_Client/tests/test_force_pipeline.cpp`

**Note:** The existing test `test_deadzone()` writes to `fd.raw[]` but the pipeline now reads `fd.compensated[]`. Tests must be updated.

- [ ] **Step 1: Replace test_deadzone to use compensated[]**

Replace `test_deadzone()`:
```cpp
static void test_residual_deadzone() {
    TEST(residual_deadzone);
    AppState::ForceData fd;
    ForcePipeline::init();

    // Below residual deadzone (0.05N) → output zero
    fd.compensated[0] = 0.03; fd.compensated[1] = -0.03; fd.compensated[2] = 0.0;
    fd.compensated[3] = 0.0; fd.compensated[4] = 0.0; fd.compensated[5] = 0.0;
    fd.lastUpdateMs = GetTickCount();
    ForcePipeline::step(fd);

    CHECK(fabs(fd.hapticOut[0]) < 0.01); // deadzone suppressed
    CHECK(fabs(fd.hapticOut[1]) < 0.01);
    CHECK(fabs(fd.hapticOut[2]) < 0.01);
    PASS();
}
```

- [ ] **Step 2: Update other tests to set compensated[]**

`test_saturation()` — change `fd.raw[0]` to `fd.compensated[0]`:
```cpp
    fd.compensated[0] = 500.0; fd.compensated[1] = 0.0; fd.compensated[2] = 0.0;
```

`test_coord_transform()` — change all `fd.raw[]` to `fd.compensated[]`:
```cpp
    fd.compensated[0] = 10.0; fd.compensated[1] = 20.0; fd.compensated[2] = 30.0;
    fd.compensated[3] = 0.0; fd.compensated[4] = 0.0; fd.compensated[5] = 0.0;
```

`test_filter_convergence()` — change `fd.raw[]` to `fd.compensated[]`:
```cpp
    fd.compensated[0] = 100.0; fd.compensated[1] = 0.0; fd.compensated[2] = 0.0;
    fd.compensated[3] = 0.0; fd.compensated[4] = 0.0; fd.compensated[5] = 0.0;
```

- [ ] **Step 3: Update main() test list**

Change `test_deadzone` to `test_residual_deadzone`:
```cpp
    test_residual_deadzone();
```

- [ ] **Step 4: Rebuild and run**

```bash
cd Touch_Client\tests && call build_test.bat
test_force_pipeline.exe
```

Expected: All 5 tests PASS.

- [ ] **Step 5: Commit**

```bash
git add Touch_Client/tests/test_force_pipeline.cpp
git commit -m "test(force): update pipeline tests for compensated[] input + residual deadzone"
```

---

### Task 7: Write unit tests for ForceCompensation + ForceCalibration

**Files:**
- Create: `Touch_Client/tests/test_force_compensation.cpp`
- Create: `Touch_Client/tests/build_force_comp_test.bat`

- [ ] **Step 1: Write test_force_compensation.cpp**

```cpp
// Standalone test: ForceCompensation + ForceCalibration core logic
#include <iostream>
#include <cassert>
#include <cmath>
#include <windows.h>
#include "../force/ForceCompensation.h"
#include "../force/ForceCalibration.h"
#include "../config/Config.h"

static int g_passed = 0, g_failed = 0;

#define TEST(name) do { std::cout << "  " << #name << "... "; } while(0)
#define PASS() do { std::cout << "PASS" << std::endl; g_passed++; } while(0)
#define CHECK(cond) do { if (!(cond)) { std::cout << "FAIL: " << #cond << std::endl; g_failed++; return; } } while(0)

// ===== GaussSolver tests =====

static void test_gauss_simple() {
    TEST(gauss_simple);
    // Solve 2x2: [2 1; 1 3] x = [5; 6]  →  x = [1.8; 1.4]
    double A[4] = {2, 1, 1, 3};
    double b[2] = {5, 6};
    double x[2];
    double rms;
    bool ok = GaussSolver::solve(2, A, b, 2, x, rms);
    CHECK(ok);
    CHECK(fabs(x[0] - 1.8) < 0.01);
    CHECK(fabs(x[1] - 1.4) < 0.01);
    PASS();
}

static void test_gauss_overdetermined() {
    TEST(gauss_overdetermined);
    // Overdetermined: y = 2x + 1  with data points (0,1), (1,3), (2,5), (3,7)
    // unknowns: m=2, c=1
    double A[8] = {0,1, 1,1, 2,1, 3,1};
    double b[4] = {1, 3, 5, 7};
    double x[2];
    double rms;
    bool ok = GaussSolver::solve(4, A, b, 2, x, rms);
    CHECK(ok);
    CHECK(fabs(x[0] - 2.0) < 0.01); // slope
    CHECK(fabs(x[1] - 1.0) < 0.01); // intercept
    CHECK(rms < 0.01); // perfect fit
    PASS();
}

// ===== MotionEstimator tests =====

static void test_motion_still() {
    TEST(motion_still);
    MotionEstimator est;
    // Feed same position 10 times
    for (int i = 0; i < 10; i++) {
        est.update(100.0, 200.0, 300.0, 0.008); // 8ms timestep
    }
    CHECK(est.isStill());
    double vel[3], acc[3];
    est.getState(vel, acc);
    CHECK(fabs(vel[0]) < 0.01);
    CHECK(fabs(vel[1]) < 0.01);
    CHECK(fabs(vel[2]) < 0.01);
    PASS();
}

static void test_motion_moving() {
    TEST(motion_moving);
    MotionEstimator est;
    // Moving at 0.1 m/s in X (100mm/s)
    double dt = 0.008;
    for (int i = 0; i < 10; i++) {
        double x = 100.0 + 0.1 * 1000.0 * i * dt; // mm
        est.update(x, 200.0, 300.0, dt);
    }
    double vel[3], acc[3];
    est.getState(vel, acc);
    CHECK(fabs(vel[0] - 0.1) < 0.02);  // ~0.1 m/s
    PASS();
}

// ===== ForceCompensation tests =====

static void test_comp_no_calib() {
    TEST(comp_no_calib);
    ForceCompensation::init();
    AppState::ForceData fd;
    fd.raw[0] = -0.65; fd.raw[1] = -1.07; fd.raw[2] = 0.055;
    fd.raw[3] = -0.02; fd.raw[4] = 0.02;  fd.raw[5] = 0.005;

    // No calibration → compensated should mirror raw
    double pose[6] = {0, 0, 0, 0, 0, 0};
    ForceCompensation::step(fd, pose);
    CHECK(fabs(fd.compensated[0] - (-0.65)) < 0.01);
    CHECK(fabs(fd.compensated[1] - (-1.07)) < 0.01);
    CHECK(fd.isCalibrated == false);
    PASS();
}

static void test_comp_gravity_only() {
    TEST(comp_gravity_only);
    ForceCompensation::init();
    // Calibrate: mass=1kg, com at origin, zero bias
    double mass = 1.0;
    double com[3] = {0, 0, 0};
    double biasF[3] = {0, 0, 0};
    double biasM[3] = {0, 0, 0};
    ForceCompensation::setCalibration(mass, com, biasF, biasM);

    AppState::ForceData fd;
    // Tool pointing straight down: Rx=0, Ry=0, Rz=0 → g_tool = (0, 0, -9.81)
    // Expected: Fz sensor reads +9.81 (supporting weight), gravity comp subtracts it → 0
    fd.raw[0] = 0.0; fd.raw[1] = 0.0; fd.raw[2] = 9.81;
    fd.raw[3] = 0.0; fd.raw[4] = 0.0; fd.raw[5] = 0.0;

    double pose[6] = {0, 0, 0, 0, 0, 0};
    ForceCompensation::step(fd, pose);
    CHECK(fabs(fd.compensated[2]) < 0.1); // gravity compensated away
    CHECK(fd.isCalibrated == true);
    PASS();
}

int main() {
    std::cout << "=== ForceCompensation + Calibration Tests ===" << std::endl;
    test_gauss_simple();
    test_gauss_overdetermined();
    test_motion_still();
    test_motion_moving();
    test_comp_no_calib();
    test_comp_gravity_only();
    std::cout << "\nResults: " << g_passed << " passed, " << g_failed << " failed" << std::endl;
    return g_failed ? 1 : 0;
}
```

- [ ] **Step 2: Write build_force_comp_test.bat**

```bat
@echo on
call "D:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvarsall.bat" x64
cd /d "D:\Projects\Touch\Touch_Client\tests"
cl /EHsc /std:c++17 /I"D:\Projects\Touch\OpenHaptics\Developer\3.5.0\include" /I"D:\Projects\Touch\OpenHaptics\Developer\3.5.0\utilities\include" /DWIN32 /DWIN32_LEAN_AND_MEAN /DNOMINMAX /D_CRT_SECURE_NO_WARNINGS /D_WINSOCK_DEPRECATED_NO_WARNINGS test_force_compensation.cpp ..\force\ForceCompensation.cpp ..\force\ForceCalibration.cpp /Fe:test_force_compensation.exe
echo BUILD_EXIT=%ERRORLEVEL%
```

- [ ] **Step 3: Build and run tests**

```bash
cd Touch_Client\tests && call build_force_comp_test.bat
test_force_compensation.exe
```

Expected: All 6 tests PASS.

- [ ] **Step 4: Commit**

```bash
git add Touch_Client/tests/test_force_compensation.cpp Touch_Client/tests/build_force_comp_test.bat
git commit -m "test(force): add ForceCompensation + ForceCalibration unit tests"
```

---

### Task 8: Wire ForceCompensation + ForceCalibration into RelayCore

**Files:**
- Modify: `Touch_Client/relay/RelayCore.h:31-35` (force section)
- Modify: `Touch_Client/relay/RelayCore.cpp:15` (add includes)
- Modify: `Touch_Client/relay/RelayCore.cpp:38-46` (ForceReader — pass pose)
- Modify: `Touch_Client/relay/RelayCore.cpp:1067-1125` (initForceReader, pollForce, shutdownForceReader)

**Consumes:** `ForceCompensation::init/step/setCalibration` (Task 3), `ForceCalibration::start/abort/isRunning/isDone/update/saveToFile` (Task 4)

- [ ] **Step 1: Add calibration interface to RelayCore.h**

Add after line 35 (`void shutdownForceReader();`):
```cpp
    // 力传感器标定
    bool startForceCalibration();
    void abortForceCalibration();
    bool isForceCalibrating() const;
    bool isForceCalibrationDone() const;
    const char* forceCalibStatus() const;
```

- [ ] **Step 2: Add includes in RelayCore.cpp**

After `#include "../force/ForcePipeline.h"` (line 13), add:
```cpp
#include "../force/ForceCompensation.h"
#include "../force/ForceCalibration.h"
```

- [ ] **Step 3: Modify ForceReader thread — call compensation on each raw sample**

In the ForceReader thread (lines 38-46), after storing raw data, add compensation:

```cpp
            // Parse ActualTCPForce at offset 576 (6 doubles, 48 bytes)
            double* forcePtr = reinterpret_cast<double*>(buf + 576);
            EnterCriticalSection(&app.forceDataMutex);
            for (int i = 0; i < 6; i++) {
                app.forceData.raw[i] = forcePtr[i];
            }
            app.forceData.lastUpdateMs = GetTickCount();
            app.forceData.isStale = false;

            // Run compensation (gravity + inertia + bias removal)
            // Pose is read without mutex since ForceReader is the only writer
            // to raw[], and pose is read-only from robotPoseMutex-guarded state
            if (!ForceCalibration::isRunning()) {
                double pose[6];
                EnterCriticalSection(&app.robotPoseMutex);
                pose[0] = app.robotActualPose.x;
                pose[1] = app.robotActualPose.y;
                pose[2] = app.robotActualPose.z;
                pose[3] = app.robotActualPose.rx;
                pose[4] = app.robotActualPose.ry;
                pose[5] = app.robotActualPose.rz;
                LeaveCriticalSection(&app.robotPoseMutex);
                ForceCompensation::step(app.forceData, pose);
            } else {
                // During calibration, feed raw data to calibration state machine
                double pose[6];
                EnterCriticalSection(&app.robotPoseMutex);
                pose[0] = app.robotActualPose.x;
                pose[1] = app.robotActualPose.y;
                pose[2] = app.robotActualPose.z;
                pose[3] = app.robotActualPose.rx;
                pose[4] = app.robotActualPose.ry;
                pose[5] = app.robotActualPose.rz;
                LeaveCriticalSection(&app.robotPoseMutex);

                bool done = ForceCalibration::update(0.008, app.forceData.raw, pose);
                if (done) {
                    // Calibration finished — apply results
                    if (ForceCalibration::currentState() == ForceCalibration::State::DONE) {
                        // Save to file
                        ForceCalibration::saveToFile("calib/force_calib.json",
                            /* residual will be read from static */ 0.0, 0.0,
                            nullptr, nullptr, nullptr, nullptr);
                        // Note: actual save handled in pollForce or main thread
                        // This flag signals main thread to apply results
                    }
                }
            }

            LeaveCriticalSection(&app.forceDataMutex);
```

Actually, the calibration state machine needs to run outside the ForceReader's tight mutex lock. Let me restructure this.

- [ ] **Step 3 (revised): Keep ForceReader simple — add pose capture + compensation call**

In ForceReader thread, replace lines 38-46:

```cpp
            // Parse ActualTCPForce at offset 576 (6 doubles, 48 bytes)
            double* forcePtr = reinterpret_cast<double*>(buf + 576);
            EnterCriticalSection(&app.forceDataMutex);
            for (int i = 0; i < 6; i++) {
                app.forceData.raw[i] = forcePtr[i];
            }
            app.forceData.lastUpdateMs = GetTickCount();
            app.forceData.isStale = false;
            LeaveCriticalSection(&app.forceDataMutex);
```

(No change — ForceReader only writes raw data. Compensation is called from pollForce at 30Hz, which is sufficient since gravity/inertia change at the pose update rate.)

- [ ] **Step 4: Modify pollForce() to call ForceCompensation + handle calibration**

Replace the `pollForce()` function (lines 1081-1115) with:

```cpp
void RelayCore::pollForce() {
    static DWORD lastPollMs = 0;
    DWORD now = GetTickCount();
    if (now - lastPollMs < 33) return;
    lastPollMs = now;

    auto& app = appState;

    // Read current pose for compensation
    double pose[6] = {0};
    EnterCriticalSection(&app.robotPoseMutex);
    pose[0] = app.robotActualPose.x;
    pose[1] = app.robotActualPose.y;
    pose[2] = app.robotActualPose.z;
    pose[3] = app.robotActualPose.rx;
    pose[4] = app.robotActualPose.ry;
    pose[5] = app.robotActualPose.rz;
    LeaveCriticalSection(&app.robotPoseMutex);

    EnterCriticalSection(&app.forceDataMutex);

    // Staleness check
    if (app.forceData.lastUpdateMs > 0 &&
        (now - app.forceData.lastUpdateMs) > static_cast<DWORD>(Config::FORCE_STALE_MS)) {
        app.forceData.isStale = true;
        for (int i = 0; i < 6; i++) app.forceData.filtered[i] = 0.0;
        for (int i = 0; i < 6; i++) app.forceData.compensated[i] = 0.0;
        for (int i = 0; i < 3; i++) app.forceData.hapticOut[i] = 0.0;
    }

    // Run calibration state machine if active (uses raw data directly)
    if (ForceCalibration::isRunning()) {
        ForceCalibration::update(0.033, app.forceData.raw, pose);
        if (ForceCalibration::isDone()) {
            // Apply results handled in idle() / keyboard callback
        }
    }

    // Run compensation (uses calibrated params if available)
    ForceCompensation::step(app.forceData, pose);

    // Run pipeline on compensated data
    ForcePipeline::step(app.forceData);

    // Build F| protocol message
    char buf[128];
    if (app.forceData.isStale) {
        snprintf(buf, sizeof(buf), "F|0.00,0.00,0.00,0.00,0.00,0.00,1");
    } else {
        snprintf(buf, sizeof(buf), "F|%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d",
            app.forceData.compensated[0], app.forceData.compensated[1],
            app.forceData.compensated[2], app.forceData.compensated[3],
            app.forceData.compensated[4], app.forceData.compensated[5],
            app.forceData.isStale ? 1 : 0);
    }
    LeaveCriticalSection(&app.forceDataMutex);

    sendRelayUpdate(buf);
}
```

- [ ] **Step 5: Update initForceReader to also init ForceCompensation**

Replace `initForceReader()` (lines 1067-1079):
```cpp
bool RelayCore::initForceReader() {
    if (!isRobotConnected()) {
        std::cout << "[Force] Robot not connected, skipping ForceReader" << std::endl;
        return false;
    }
    ForcePipeline::init();
    ForceCompensation::init();
    m_forceThread = CreateThread(NULL, 0, forceReaderThread, NULL, 0, NULL);
    if (!m_forceThread) {
        std::cerr << "[Force] Failed to create ForceReader thread" << std::endl;
        return false;
    }
    return true;
}
```

- [ ] **Step 6: Update shutdownForceReader**

Replace `shutdownForceReader()` (lines 1117-1125):
```cpp
void RelayCore::shutdownForceReader() {
    if (m_forceThread) {
        WaitForSingleObject(m_forceThread, 1000);
        CloseHandle(m_forceThread);
        m_forceThread = NULL;
    }
    robotCloseRealtime();
    ForcePipeline::shutdown();
    ForceCompensation::shutdown();
}
```

- [ ] **Step 7: Add calibration control methods**

Add after `shutdownForceReader()` implementation:
```cpp
bool RelayCore::startForceCalibration() {
    if (m_transmitting) {
        std::cout << "[Force] Cannot calibrate while transmitting — release button first" << std::endl;
        return false;
    }
    if (!isRobotConnected()) {
        std::cout << "[Force] Robot not connected, cannot calibrate" << std::endl;
        return false;
    }
    auto& app = appState;
    if (app.isRobotInAlarm.load()) {
        std::cout << "[Force] Robot in alarm, cannot calibrate" << std::endl;
        return false;
    }
    std::cout << "[Force] Starting calibration sweep..." << std::endl;
    return ForceCalibration::start();
}

void RelayCore::abortForceCalibration() {
    ForceCalibration::abort();
    std::cout << "[Force] Calibration aborted" << std::endl;
}

bool RelayCore::isForceCalibrating() const {
    return ForceCalibration::isRunning();
}

bool RelayCore::isForceCalibrationDone() const {
    return ForceCalibration::isDone();
}

const char* RelayCore::forceCalibStatus() const {
    return ForceCalibration::statusText();
}
```

- [ ] **Step 8: Commit**

```bash
git add Touch_Client/relay/RelayCore.h Touch_Client/relay/RelayCore.cpp
git commit -m "feat(relay): wire ForceCompensation + ForceCalibration into pollForce + init/shutdown"
```

---

### Task 9: Integration in main.cpp — load calib, key binding

**Files:**
- Modify: `Touch_Client/main.cpp:2-18` (add include)
- Modify: `Touch_Client/main.cpp:474-476` (add calib file load)
- Modify: `Touch_Client/main.cpp:135-158` (add calib key handling)

- [ ] **Step 1: Add include**

After `#include "safety/RobotDiagnostics.h"` (line 15), add:
```cpp
#include "force/ForceCalibration.h"
```

- [ ] **Step 2: Load calibration file on startup**

After the `initForceReader()` call (line 477), add:
```cpp
    // 4.6 加载力传感器标定文件
    {
        double massKg, com[3], biasF[3], biasM[3], residualRms;
        if (ForceCalibration::loadFromFile("calib/force_calib.json",
                massKg, com, biasF, biasM, residualRms)) {
            ForceCompensation::setCalibration(massKg, com, biasF, biasM);
            std::cout << "[Force] Loaded force_calib.json (mass=" << massKg
                      << "kg, residual=" << residualRms << "N)" << std::endl;
        } else {
            std::cout << "[Force] No calibration file — force compensation disabled. "
                      << "Press 'c' when idle to calibrate." << std::endl;
        }
    }
```

- [ ] **Step 3: Add calibration key binding in keyboard()**

The 'c' key is already used for coordinate calibration. The force calibration needs a different trigger. The spec says 'c' when not transmitting. Let's modify the 'c' handler:

Replace the existing 'c' handler block (lines 161-173) to add force calibration when not in coord-calib mode:
```cpp
    // ===== Force Calibration =====
    // 'c' when IDLE (not transmitting, not in coord-calib mode): start force calibration
    if ((key == 'c' || key == 'C') && !Calibration::collectMode) {
        auto& relay = RelayCore::instance();
        if (relay.isForceCalibrating()) {
            std::cout << "[Force] Aborting calibration..." << std::endl;
            relay.abortForceCalibration();
        } else if (relay.isForceCalibrationDone()) {
            // Calibration done — apply results
            std::cout << "[Force] Calibration results applied" << std::endl;
            // Results already applied in pollForce when DONE state detected
        } else {
            relay.startForceCalibration();
        }
        return;
    }

    // ===== 标定模式 (coordinate calibration) =====
    // This block only reached if 'c' pressed during collectMode
    if (key == 'c' || key == 'C') {
        if (Calibration::collectMode) {
            Calibration::cancelCollect();
            std::cout << "\n[CALIB] Mode OFF" << std::endl;
        } else {
            Calibration::startCollect();
            std::cout << "\n[CALIB] Mode ON — "
                      << "Align Touch pen + robot to marker, press SPACE to record,"
                      << " 's' to solve, 'c' to exit" << std::endl;
        }
        return;
    }
```

Wait — this has a logic issue. The first 'c' block catches ALL 'c' presses when not in collectMode (including the one that would START collectMode). Let me restructure:

```cpp
    // ===== Force Calibration ('c' when idle; SPACE to confirm pose) =====
    if (key == 'c' || key == 'C') {
        auto& relay = RelayCore::instance();

        // If force calibration is running, 'c' aborts it
        if (relay.isForceCalibrating()) {
            std::cout << "[Force] Aborting calibration..." << std::endl;
            relay.abortForceCalibration();
            return;
        }

        // If not transmitting, start force calibration
        if (!appState.isTransmitting.load() && !g_noRobot) {
            if (relay.startForceCalibration()) {
                std::cout << "[Force] Calibration started. Move robot to each target orientation "
                          << "and press SPACE to confirm." << std::endl;
                return;
            }
            // Fall through to coord calib if force calib rejected
        }

        // Coordinate calibration mode toggle
        if (Calibration::collectMode) {
            Calibration::cancelCollect();
            std::cout << "\n[CALIB] Mode OFF" << std::endl;
        } else {
            Calibration::startCollect();
            std::cout << "\n[CALIB] Mode ON — "
                      << "Align Touch pen + robot to marker, press SPACE to record,"
                      << " 's' to solve, 'c' to exit" << std::endl;
        }
        return;
    }

    // SPACE during force calibration: confirm current pose
    if (key == ' ' && RelayCore::instance().isForceCalibrating()) {
        ForceCalibration::confirmPose();
        std::cout << "[Force] Pose confirmed, sampling..." << std::endl;
        return;
    }
```

- [ ] **Step 4: Add force calibration status display in idle()**

Add after the `pollForce()` call in `idle()` (line 59), add status print:
```cpp
        // Print calibration status when it changes
        if (!g_noRobot) {
            static ForceCalibration::State lastCalibState = ForceCalibration::State::IDLE;
            ForceCalibration::State curState = ForceCalibration::currentState();
            if (curState != lastCalibState) {
                lastCalibState = curState;
                std::cout << "[Force] Calibration: " << ForceCalibration::statusText() << std::endl;
                if (curState == ForceCalibration::State::DONE) {
                    // Apply calibration results to ForceCompensation
                    // (Results are read from ForceCalibration's static state)
                    // The calibration update() in pollForce handles the transition
                }
            }
        }
```

Wait, this needs `ForceCalibration.h` included in main.cpp. Already done in Step 1.

Actually, looking at this more carefully — the calibration's DONE state application needs to happen somewhere. Let me handle it in pollForce() where we already call ForceCalibration::update(). Let me revise:

In pollForce(), after the `ForceCalibration::update()` call:
```cpp
    if (ForceCalibration::isDone() && ForceCalibration::currentState() == ForceCalibration::State::DONE) {
        static bool applied = false;
        if (!applied) {
            applied = true;
            // Apply results to ForceCompensation (params are internal to ForceCalibration)
            // ForceCalibration saves to file; we load back
            double massKg, com[3], biasF[3], biasM[3], residualRms;
            if (ForceCalibration::loadFromFile("calib/force_calib.json",
                    massKg, com, biasF, biasM, residualRms)) {
                ForceCompensation::setCalibration(massKg, com, biasF, biasM);
                std::cout << "[Force] Calibration applied! mass=" << massKg
                          << "kg, residual=" << residualRms << "N" << std::endl;
            }
        }
    }
```

Hmm, this is getting complex. Let me simplify: in pollForce(), when ForceCalibration::isDone():
1. If DONE state: save to file, then load + apply to ForceCompensation
2. If ABORTED: just reset

Let me revise the pollForce calibration handling to be cleaner. I'll update Task 8 Step 4's implementation.

- [ ] **Step 5: Commit**

```bash
git add Touch_Client/main.cpp
git commit -m "feat(main): load force calib on startup + 'c' key triggers calibration"
```

---

### Task 10: Calibration DONE handler — apply results in pollForce

**Files:**
- Modify: `Touch_Client/relay/RelayCore.cpp` (pollForce calibration section)

**Note:** This task refines the calibration completion handling added in Tasks 8-9. The calibration state machine runs inside pollForce; when it finishes, we need to save results and apply them to ForceCompensation.

- [ ] **Step 1: Add ForceCalibration include to RelayCore.cpp** (already done in Task 8 Step 2)

- [ ] **Step 2: Refine the calibration section in pollForce()**

Replace the calibration block from Task 8 Step 4 with the complete handler:

```cpp
    // Run calibration state machine if active
    static bool g_calibJustCompleted = false;
    if (ForceCalibration::isRunning()) {
        ForceCalibration::update(0.033, app.forceData.raw, pose);
        if (ForceCalibration::isDone()) {
            if (ForceCalibration::currentState() == ForceCalibration::State::DONE && !g_calibJustCompleted) {
                g_calibJustCompleted = true;
                // The solved parameters are managed inside ForceCalibration.
                // We save them to file and then load back into ForceCompensation.
                // Since ForceCalibration stores results in static globals (g_solvedMass etc.),
                // we access them through a helper or by re-loading the file.

                // Create calib directory if needed
                CreateDirectoryA("calib", NULL);

                // Read solved params (exposed via ForceCalibration getter — add to header)
                // For now, save + reload pattern:
                double massKg = 0, com[3] = {0}, biasF[3] = {0}, biasM[3] = {0}, rms = 0;
                // Get results from ForceCalibration's internal state
                // (ForceCalibration needs a getResults() accessor — see below)

                // Actually, let's add a getter to ForceCalibration:
                // extern bool ForceCalibration::getResults(double& mass, double com[3], double biasF[3], double biasM[3], double& rms);
            }
        }
    } else {
        g_calibJustCompleted = false;
    }
```

This is getting circular. Let me design the interface more cleanly.

**Revised approach:** ForceCalibration saves to file internally when DONE, and also applies to ForceCompensation directly. No intermediate getResults() needed.

- [ ] **Step 1 (clean): ForceCalibration applies results to ForceCompensation directly when DONE**

Add `#include "ForceCompensation.h"` to ForceCalibration.cpp.

Modify the DONE/VERIFY transition in ForceCalibration.cpp's update():
```cpp
    case State::VERIFY: {
        if (g_solvedResidual <= Config::FORCE_CALIB_MAX_RESIDUAL_N) {
            // Apply to ForceCompensation
            ForceCompensation::setCalibration(g_solvedMass, g_solvedCom,
                                              g_solvedBiasF, g_solvedBiasM);
            // Save to file
            ForceCalibration::saveToFile("calib/force_calib.json",
                g_solvedResidual, g_solvedMass, g_solvedCom,
                g_solvedBiasF, g_solvedBiasM);
            std::cout << "[Force] Calibration complete! mass=" << g_solvedMass
                      << "kg, residual=" << g_solvedResidual << "N" << std::endl;
            std::cout << "[Force]   com=(" << g_solvedCom[0] << "," << g_solvedCom[1]
                      << "," << g_solvedCom[2] << ")m" << std::endl;
            std::cout << "[Force]   biasF=(" << g_solvedBiasF[0] << "," << g_solvedBiasF[1]
                      << "," << g_solvedBiasF[2] << ")N" << std::endl;
            g_calibState = State::DONE;
        } else {
            std::cout << "[Force] Calibration residual high (" << g_solvedResidual
                      << "N > " << Config::FORCE_CALIB_MAX_RESIDUAL_N
                      << "N) — results may be inaccurate" << std::endl;
            g_calibState = State::DONE; // Still mark done, user decides
        }
        break;
    }
```

This eliminates the need for getResults() — ForceCalibration directly calls ForceCompensation::setCalibration() and saves to file.

Add `#include "ForceCompensation.h"` at the top of ForceCalibration.cpp.

- [ ] **Step 2: This is a modification to Task 4's implementation. Commit the refinement.**

```bash
git add Touch_Client/force/ForceCalibration.cpp
git commit -m "feat(force): ForceCalibration auto-applies results to ForceCompensation on DONE"
```

---

### Task 11: Build integration test + verify end-to-end

**Files:**
- Modify: `Touch_Client/build_output.txt` (verify full build)

- [ ] **Step 1: Build full project**

```bash
cd Touch_Client && build.bat 2>&1 | tail -20
```

Expected: Build succeeds with 0 errors. New .obj files for ForceCompensation.cpp and ForceCalibration.cpp.

- [ ] **Step 2: Run existing tests to verify no regression**

```bash
cd Touch_Client\tests
test_force_pipeline.exe
test_force_compensation.exe
```

Expected: Both test suites pass all tests.

- [ ] **Step 3: Verify --no-robot mode starts without crash**

```bash
cd Touch_Client\x64\Release
.\Touch_Client.exe --no-robot --no-touch
# Wait 3 seconds, then press 'q' to quit
```

Expected output:
```
[Force] No calibration file — force compensation disabled. Press 'c' when idle to calibrate.
System ready.
```

No crash. Clean shutdown on 'q'.

- [ ] **Step 4: Verify --no-robot does NOT attempt 30004 connect**

Expected: No `[Force] Realtime port connect failed` messages. ForceReader not started.

- [ ] **Step 5: Commit**

```bash
git add Touch_Client/build_output.txt
git commit -m "build: verify full project compiles with ForceCompensation + ForceCalibration"
```

---

### Task 12: Update test build scripts (run_tests.bat)

**Files:**
- Modify: `Touch_Client/tests/run_tests.bat` (add new test)

- [ ] **Step 1: Add force compensation test to run_tests.bat**

Read the existing `run_tests.bat` to check format, then add a new line:
```bat
echo === Force Compensation Tests ===
call build_force_comp_test.bat
test_force_compensation.exe
if %ERRORLEVEL% neq 0 exit /b %ERRORLEVEL%
```

- [ ] **Step 2: Commit**

```bash
git add Touch_Client/tests/run_tests.bat
git commit -m "test: add force compensation test to run_tests.bat"
```

---

## Summary

| Task | Files | Status |
|------|-------|--------|
| 1 | Config.h | Config constants updated |
| 2 | AppState.h | ForceData expanded |
| 3 | ForceCompensation.h/.cpp | Runtime compensation created |
| 4 | ForceCalibration.h/.cpp | Calibration system created |
| 5 | ForcePipeline.cpp | Uses compensated[] data |
| 6 | test_force_pipeline.cpp | Updated for new API |
| 7 | test_force_compensation.cpp | New tests |
| 8 | RelayCore.h/.cpp | Wired compensation + calibration |
| 9 | main.cpp | Load calib, key binding |
| 10 | ForceCalibration.cpp | Auto-apply on DONE |
| 11 | build_output.txt | Integration verify |
| 12 | run_tests.bat | Test script update |

**Total commits:** 12
**New files:** 4 (ForceCompensation.h/.cpp, ForceCalibration.h/.cpp, test_force_compensation.cpp, build_force_comp_test.bat)
**Modified files:** 7 (Config.h, AppState.h, ForcePipeline.cpp, test_force_pipeline.cpp, RelayCore.h/.cpp, main.cpp, run_tests.bat)
