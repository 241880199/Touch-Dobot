#define _USE_MATH_DEFINES
#include "ForceCalibration.h"
#include "ForceCompensation.h"
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
    // A is n x n row-major; b is n x 1
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
    // Build normal equations: ATA (nCols x nCols), ATb (nCols x 1)
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

// ===== Calibration state =====

static ForceCalibration::State g_calibState = ForceCalibration::State::IDLE;
static bool g_abortFlag = false;
static bool g_poseConfirmed = false;  // set by confirmPose() to advance MOVE->SETTLE
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
            R[6] = -sy;      R[7] = cy * sx;                   R[8] = cy * cx;

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

            // Torque eqns: M = gTool x p + M0
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
        // Load solved parameters into ForceCompensation
        ForceCompensation::setCalibration(g_solvedMass, g_solvedCom,
                                          g_solvedBiasF, g_solvedBiasM);
        // Persist to file
        ForceCalibration::saveToFile("force_calib.json");

        if (g_solvedResidual <= Config::FORCE_CALIB_MAX_RESIDUAL_N) {
            printf("[Force] Calibration OK: mass=%.4f kg, residual=%.4f N\n",
                   g_solvedMass, g_solvedResidual);
        } else {
            printf("[Force] Calibration WARNING: high residual=%.4f N (threshold %.4f N)\n",
                   g_solvedResidual, Config::FORCE_CALIB_MAX_RESIDUAL_N);
        }
        g_calibState = State::DONE;
        break;
    }

    default:
        break;
    }

    return (g_calibState == State::DONE || g_calibState == State::ABORTED);
}

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

bool saveToFile(const char* path) {
    return saveToFile(path, g_solvedResidual, g_solvedMass,
                      g_solvedCom, g_solvedBiasF, g_solvedBiasM);
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

} // namespace ForceCalibration
