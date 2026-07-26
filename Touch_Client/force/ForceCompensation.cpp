#define _USE_MATH_DEFINES
#include "ForceCompensation.h"
#include "../config/Config.h"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <windows.h>

// ===== Internal state =====
static CRITICAL_SECTION g_calibMutex;
static bool g_mutexInit = false;
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

// Cross product: out = a x b
static void cross(const double a[3], const double b[3], double out[3]) {
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

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
    m_posBuf[m_idx][0] = x * 0.001;  // mm -> m
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

// ===== ForceCompensation namespace =====

namespace ForceCompensation {

void init() {
    if (!g_mutexInit) {
        InitializeCriticalSection(&g_calibMutex);
        g_mutexInit = true;
    }
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
    EnterCriticalSection(&g_calibMutex);
    g_massKg = massKg;
    for (int i = 0; i < 3; i++) {
        g_comSensor[i] = comSensor[i];
        g_biasForce[i] = biasForce[i];
        g_biasTorque[i] = biasTorque[i];
    }
    g_isCalibrated = true;
    LeaveCriticalSection(&g_calibMutex);
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

    // 3. Snapshot calibration globals under mutex
    EnterCriticalSection(&g_calibMutex);
    double mass = g_massKg;
    double com[3] = {g_comSensor[0], g_comSensor[1], g_comSensor[2]};
    double bF[3] = {g_biasForce[0], g_biasForce[1], g_biasForce[2]};
    double bM[3] = {g_biasTorque[0], g_biasTorque[1], g_biasTorque[2]};
    bool calib = g_isCalibrated;
    LeaveCriticalSection(&g_calibMutex);

    if (!calib) return; // setCalibration cleared calibration mid-flight

    // 4. Compute rotation matrix from Euler angles
    double R[9];
    eulerToRotation(poseRxyz[3], poseRxyz[4], poseRxyz[5], R);

    // 5. Gravity in tool frame: g_tool = R^T * (0, 0, +9.81)
    //    (sensor sees positive Z when supporting a hanging tool)
    double gBase[3] = {0.0, 0.0, 9.81};
    double gTool[3];
    matTransposeMulVec(R, gBase, gTool);

    // Gravity force: sensor reads +m*g support force when tool hangs
    // (same direction as gTool — not a reaction force)
    double Fg[3];
    Fg[0] = mass * gTool[0];
    Fg[1] = mass * gTool[1];
    Fg[2] = mass * gTool[2];

    // Gravity torque: r_com x Fg
    double Mg[3];
    cross(com, Fg, Mg);

    // 6. Inertia force (only if moving)
    double Fi[3] = {0, 0, 0};
    if (!g_motion.isStill()) {
        double vel[3], acc[3];
        g_motion.getState(vel, acc);
        Fi[0] = mass * acc[0];
        Fi[1] = mass * acc[1];
        Fi[2] = mass * acc[2];
    }

    // 7. Compensate: compensated = raw - bias - gravity - inertia
    fd.compensated[0] = fd.raw[0] - bF[0] - Fg[0] - Fi[0];
    fd.compensated[1] = fd.raw[1] - bF[1] - Fg[1] - Fi[1];
    fd.compensated[2] = fd.raw[2] - bF[2] - Fg[2] - Fi[2];
    fd.compensated[3] = fd.raw[3] - bM[0] - Mg[0];
    fd.compensated[4] = fd.raw[4] - bM[1] - Mg[1];
    fd.compensated[5] = fd.raw[5] - bM[2] - Mg[2];

    // 8. Online EMA bias update (only when still — slow drift tracking)
    if (g_motion.isStill()) {
        double alpha = Config::FORCE_BIAS_EMA_ALPHA;
        // Update local copy, then write back under mutex
        bF[0] += alpha * (fd.raw[0] - Fg[0] - bF[0]);
        bF[1] += alpha * (fd.raw[1] - Fg[1] - bF[1]);
        bF[2] += alpha * (fd.raw[2] - Fg[2] - bF[2]);
        bM[0] += alpha * (fd.raw[3] - Mg[0] - bM[0]);
        bM[1] += alpha * (fd.raw[4] - Mg[1] - bM[1]);
        bM[2] += alpha * (fd.raw[5] - Mg[2] - bM[2]);

        EnterCriticalSection(&g_calibMutex);
        g_biasForce[0] = bF[0];
        g_biasForce[1] = bF[1];
        g_biasForce[2] = bF[2];
        g_biasTorque[0] = bM[0];
        g_biasTorque[1] = bM[1];
        g_biasTorque[2] = bM[2];
        LeaveCriticalSection(&g_calibMutex);
    }

    // 9. Update calib params in ForceData for HUD display / MATLAB relay
    fd.isCalibrated = true;
    fd.calibMassKg = mass;
    for (int i = 0; i < 3; i++) {
        fd.calibComSensor[i] = com[i];
        fd.calibBiasForce[i] = bF[i];
        fd.calibBiasTorque[i] = bM[i];
    }
}

void shutdown() {
    g_isCalibrated = false;
    g_motion.reset();
    if (g_mutexInit) {
        DeleteCriticalSection(&g_calibMutex);
        g_mutexInit = false;
    }
}

} // namespace ForceCompensation
