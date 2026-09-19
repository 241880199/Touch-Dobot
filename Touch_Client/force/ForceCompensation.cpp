#define _USE_MATH_DEFINES
#include "ForceCompensation.h"
#include "../calibration/TcpCalibration.h"
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
static double g_A[9] = {0};            // 全量模型的 3×3 力响应 (kg, row-major)
static double g_comSensor[3] = {0};    // c_s (m, 传感器测量系)
static double g_biasForce[3] = {0};
static double g_biasTorque[3] = {0};
static MotionEstimator g_motion;

// 质量尺度 m = |det A|^(1/3)。A = m·S·Q (见 PayloadCalibration::decompose), 所以它的
// 三个奇异值的几何平均恰好是 m —— 而几何平均 = |det|^(1/3), 不需要 SVD。
// 就是 decompose() 报的 massScale, 这里现算一份的理由见头文件 currentMassKg。
static double massScaleOf(const double A[9]) {
    const double det = A[0] * (A[4] * A[8] - A[5] * A[7])
                     - A[1] * (A[3] * A[8] - A[5] * A[6])
                     + A[2] * (A[3] * A[7] - A[4] * A[6]);
    return cbrt(fabs(det));
}

// ===== Euler angles (deg) to rotation matrix =====
// ⚠ 本文件里【当前没有调用方】了: 重力的唯一去处已改成 TcpCalibration::gravitySensorFrameAtYaw
//   (约定只能有一份实现)。此函数与下面的 matTransposeMulVec 保留未删 —— 删不删由所有者定;
//   但【不要】再用它们在这里重新展开一遍重力, 那正是本文件以前和求解器各写一份的老毛病。
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
// ⚠ 同 eulerToRotation: 当前无调用方 (重力的唯一去处已改成共享函数), 保留未删。
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
    for (int i = 0; i < 9; i++) g_A[i] = 0.0;
    for (int i = 0; i < 3; i++) {
        g_comSensor[i] = 0.0;
        g_biasForce[i] = 0.0;
        g_biasTorque[i] = 0.0;
    }
    g_motion.reset();
}

void setCalibration(const double A[9], const double biasForce[3],
                    const double biasTorque[3], const double comSensor[3])
{
    EnterCriticalSection(&g_calibMutex);
    for (int i = 0; i < 9; i++) g_A[i] = A[i];
    for (int i = 0; i < 3; i++) {
        g_comSensor[i] = comSensor[i];
        g_biasForce[i] = biasForce[i];
        g_biasTorque[i] = biasTorque[i];
    }
    g_isCalibrated = true;
    LeaveCriticalSection(&g_calibMutex);
}

void currentModel(double A[9], double comSensor[3]) {
    EnterCriticalSection(&g_calibMutex);
    for (int i = 0; i < 9; i++) A[i] = g_A[i];
    for (int i = 0; i < 3; i++) comSensor[i] = g_comSensor[i];
    LeaveCriticalSection(&g_calibMutex);
}

void currentBias(double biasForce[3], double biasTorque[3]) {
    EnterCriticalSection(&g_calibMutex);
    for (int i = 0; i < 3; i++) {
        biasForce[i]  = g_biasForce[i];
        biasTorque[i] = g_biasTorque[i];
    }
    LeaveCriticalSection(&g_calibMutex);
}

// 诊断用: 见头文件里为什么需要它。
// 有意【不】取 g_calibMutex: MotionEstimator 的状态不由它保护, step() 里读它
// (下面的惯性与 EMA 分支) 同样是无锁的, 这里跟着一致即可。
bool motionState(double vel[3], double acc[3]) {
    g_motion.getState(vel, acc);
    return g_motion.isStill();
}

bool isCalibrated() {
    return g_isCalibrated;
}

double currentMassKg() {
    EnterCriticalSection(&g_calibMutex);
    double A[9];
    for (int i = 0; i < 9; i++) A[i] = g_A[i];
    LeaveCriticalSection(&g_calibMutex);
    return massScaleOf(A);   // A 全 0 -> det 0 -> 报 0
}

void step(AppState::ForceData& fd, const double poseRxyz[6]) {
    // poseRxyz = {X_mm, Y_mm, Z_mm, Rx_deg, Ry_deg, Rz_deg}

    // 1. Update motion estimator
    double dt = 1.0 / static_cast<double>(Config::FORCE_EFFECTIVE_SAMPLE_RATE);
    g_motion.update(poseRxyz[0], poseRxyz[1], poseRxyz[2], dt);

    // 2. Copy the raw channel to compensated as default (no-op if uncalibrated)
    //    默认值取 @1304 (与下面标定后的公式【同一个通道】)。从前这里是 fd.raw (@576) ——
    //    切换通道时漏掉这一行, 未标定时的读数就会来自另一个物理量, 而"标定前后读到的
    //    不是同一件事"是查不出来的 (两边的量纲都是 N)。
    for (int i = 0; i < 6; i++) {
        fd.compensated[i] = fd.sixForceRaw[i];
    }

    if (!g_isCalibrated) return;

    // 3. Snapshot calibration globals under mutex
    EnterCriticalSection(&g_calibMutex);
    double A[9];
    for (int i = 0; i < 9; i++) A[i] = g_A[i];
    double com[3] = {g_comSensor[0], g_comSensor[1], g_comSensor[2]};
    double bF[3] = {g_biasForce[0], g_biasForce[1], g_biasForce[2]};
    double bM[3] = {g_biasTorque[0], g_biasTorque[1], g_biasTorque[2]};
    bool calib = g_isCalibrated;
    LeaveCriticalSection(&g_calibMutex);

    if (!calib) return; // setCalibration cleared calibration mid-flight

    // 4-5. Gravity in the SENSOR frame, then through the fitted response A.
    //    g = gravitySensorFrameAtYaw(pose, 0.0, g) —— 【psi 传 0】: 全量模型的 A 是自由 3×3,
    //    安装旋转/反射/非正交全被它吸收, 所以补偿式子里【没有 ψ】, 模块态不再参与。
    //    (残余模型走的是 gravitySensorFrame: 读模块态里的 ψ, 再乘一个【标量】质量。)
    //    ⚠ 走共享实现, 别在这里自己展开 Rᵀ(0,0,9.81): 模块自己实现过两次重力约定, 一份写
    //      成 R、一份写成 Rᵀ, 求解器因此安静地解错 —— 约定只能有一份实现。
    double gTool[3];
    TcpCalibration::gravitySensorFrameAtYaw(poseRxyz, 0.0, gTool);

    // Gravity force through the fitted response: Fg = A · g
    // (残余模型是标量质量乘 g —— A 的每一行都是一个"分量怎么随重力方向变"的响应。)
    double Fg[3];
    for (int a = 0; a < 3; a++) {
        Fg[a] = A[3 * a] * gTool[0] + A[3 * a + 1] * gTool[1] + A[3 * a + 2] * gTool[2];
    }

    // Gravity torque: c_s × (A·g) —— 与 Fg 同一个 w = A·g (叉乘结构, 不是独立的 3×3)。
    double Mg[3];
    cross(com, Fg, Mg);

    // 6. Inertia force (only if moving) —— 语义、时机、系数一字未改 (仍是 mass·a);
    //    mass 现在取自全量模型的质量尺度 |det A|^(1/3) (见 currentMassKg)。
    const double mass = massScaleOf(A);
    double Fi[3] = {0, 0, 0};
    if (!g_motion.isStill()) {
        double vel[3], acc[3];
        g_motion.getState(vel, acc);
        Fi[0] = mass * acc[0];
        Fi[1] = mass * acc[1];
        Fi[2] = mass * acc[2];
    }

    // 7. Compensate: compensated = sixForceRaw − bias − gravity − inertia
    fd.compensated[0] = fd.sixForceRaw[0] - bF[0] - Fg[0] - Fi[0];
    fd.compensated[1] = fd.sixForceRaw[1] - bF[1] - Fg[1] - Fi[1];
    fd.compensated[2] = fd.sixForceRaw[2] - bF[2] - Fg[2] - Fi[2];
    fd.compensated[3] = fd.sixForceRaw[3] - bM[0] - Mg[0];
    fd.compensated[4] = fd.sixForceRaw[4] - bM[1] - Mg[1];
    fd.compensated[5] = fd.sixForceRaw[5] - bM[2] - Mg[2];

    // 8. Online EMA bias update (only when still) — 作用/时机/系数一字未改, 只有输入量
    //    跟着换成了 @1304。零偏是【这个通道】的零偏: 拿 @576 去更新它, 就是给另一路量的
    //    零偏做 EMA —— 两路的零偏不是一回事。实测出处: tests/fixtures/calib_poses_2026-09-19.txt
    //    7 个姿态的逐通道均值差 (@1304 − @576) = 19.8 / 1.6 / 1.7 N (x/y/z)。
    if (g_motion.isStill()) {
        double alpha = Config::FORCE_BIAS_EMA_ALPHA;
        // Update local copy, then write back under mutex
        bF[0] += alpha * (fd.sixForceRaw[0] - Fg[0] - bF[0]);
        bF[1] += alpha * (fd.sixForceRaw[1] - Fg[1] - bF[1]);
        bF[2] += alpha * (fd.sixForceRaw[2] - Fg[2] - bF[2]);
        bM[0] += alpha * (fd.sixForceRaw[3] - Mg[0] - bM[0]);
        bM[1] += alpha * (fd.sixForceRaw[4] - Mg[1] - bM[1]);
        bM[2] += alpha * (fd.sixForceRaw[5] - Mg[2] - bM[2]);

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
    fd.calibMassKg = mass;              // 质量尺度 |det A|^(1/3), 不是调用方传进来的
    for (int i = 0; i < 3; i++) {
        fd.calibComSensor[i] = com[i];  // c_s (m, 传感器测量系)
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
