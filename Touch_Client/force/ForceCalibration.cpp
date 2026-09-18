#define _USE_MATH_DEFINES
#include "ForceCalibration.h"
#include "ForceCompensation.h"
#include "../config/Config.h"
#include "../core/CalibStore.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <algorithm>

// ===== Internal state =====

static ForceCalibration::State g_state = ForceCalibration::State::IDLE;
static bool g_tareOnly = false;   // true = 仅调零流程: TARE 定稿后直接应用+存盘, 不进 MOTION

// TARE
static double g_phaseTimer = 0.0;
static double g_tareAccum[6] = {0};
static int    g_tareCount = 0;

// MOTION — record F vs a during user movement
static const int MAX_MOTION_SAMPLES = 300;  // ~10s at 30Hz
static int    g_motionCount = 0;
static double g_motionRaw[300][6];   // raw force at each frame
static double g_motionPos[300][3];   // position (mm) at each frame
static double g_motionRxyz[300][3];  // orientation Rx,Ry,Rz (deg) at each frame

// Results
static double g_massKg = 0.0;
static double g_biasForce[3] = {0};
static double g_biasTorque[3] = {0};

// Drag callback
static void (*g_dragCb)(bool) = nullptr;

// ===== MotionEstimator for MOTION phase =====
// Lightweight: 5-point pos buffer → velocity + acceleration

namespace ForceCalibration {

void setDragModeCallback(void (*cb)(bool)) { g_dragCb = cb; }

State currentState() { return g_state; }

bool isRunning() {
    return g_state != State::IDLE && g_state != State::DONE && g_state != State::ABORTED;
}

bool isZeroing() {
    return g_tareOnly && isRunning();
}

bool isDone() {
    return g_state == State::DONE || g_state == State::ABORTED;
}

const char* statusText() {
    switch (g_state) {
        case State::IDLE:    return "Idle";
        case State::TARE:    return g_tareOnly ? "Zeroing... (keep still)"
                                               : "Taring... (keep still)";
        case State::MOTION:  return "Motion cal... (move robot, SPACE to stop)";
        case State::SOLVE:   return "Solving...";
        case State::DONE:    return "Calibration complete";
        case State::ABORTED: return "Calibration aborted";
    }
    return "Unknown";
}

// 由采集缓冲定稿零偏 (力 + 力矩)
static void finalizeBias() {
    int n = (g_tareCount > 0) ? g_tareCount : 1;
    for (int i = 0; i < 6; i++) g_tareAccum[i] /= n;
    for (int i = 0; i < 3; i++) {
        g_biasForce[i]  = g_tareAccum[i];
        g_biasTorque[i] = g_tareAccum[i + 3];
    }
}

// 可启动: 空闲, 或上一次已结束 (DONE/ABORTED) — 允许同一进程内重新调零/重标
static bool canStart() {
    return g_state == ForceCalibration::State::IDLE
        || g_state == ForceCalibration::State::DONE
        || g_state == ForceCalibration::State::ABORTED;
}

bool start() {
    if (!canStart()) return false;

    g_state = State::TARE;
    g_tareOnly = false;
    g_phaseTimer = 0.0;
    g_tareCount = 0;
    for (int i = 0; i < 6; i++) g_tareAccum[i] = 0.0;

    printf("[Force] Calibration started — TARE phase (keep robot still for 2s)...\n");
    return true;
}

bool startZero() {
    if (!canStart()) return false;

    g_state = State::TARE;
    g_tareOnly = true;
    g_phaseTimer = 0.0;
    g_tareCount = 0;
    for (int i = 0; i < 6; i++) g_tareAccum[i] = 0.0;

    printf("[Force] ZERO started — keep robot still for %.0fs "
           "(TARE only: no motion phase, drag mode NOT enabled)...\n",
           Config::FORCE_CALIB_STILL_COLLECT_S);
    return true;
}

void abort() {
    printf("[Force] Calibration ABORTED (was: %s)\n", statusText());
    if (g_dragCb) g_dragCb(false);
    g_state = State::ABORTED;
    g_tareOnly = false;   // 中止即放弃, 不做任何应用/存盘
}

void confirmPose() {
    if (g_state == State::TARE) {
        // TARE 由 update() 采集满 FORCE_CALIB_STILL_COLLECT_S 后自动定稿 (定时器递增,
        // 跨过阈值的同一次 update() 就切换状态), 因此这里只报告进度、不需要按键。
        printf("[Force] TARE in progress (%.1f/%.0fs) — auto-completes, no key needed\n",
               g_phaseTimer, Config::FORCE_CALIB_STILL_COLLECT_S);
        return;
    }

    if (g_state == State::MOTION) {
        // SPACE to stop motion recording (min 2s)
        if (g_phaseTimer < 2.0) {
            printf("[Force] Min 2s motion needed (%.1fs elapsed), keep moving...\n", g_phaseTimer);
            return;
        }
        printf("[Force] Motion recording stopped (%d samples). Fitting mass...\n", g_motionCount);
        if (g_dragCb) g_dragCb(false);
        g_state = State::SOLVE;
        return;
    }
}

bool update(double dt, const double raw[6], const double pose[6]) {
    if (g_state == State::IDLE || g_state == State::DONE || g_state == State::ABORTED) {
        return (g_state == State::DONE || g_state == State::ABORTED);
    }

    switch (g_state) {

    // ===== TARE: accumulate static bias =====
    case State::TARE: {
        for (int i = 0; i < 6; i++) g_tareAccum[i] += raw[i];
        g_tareCount++;
        g_phaseTimer += dt;
        if (g_phaseTimer >= Config::FORCE_CALIB_STILL_COLLECT_S) {
            // Auto-complete tare after collection time
            finalizeBias();
            printf("[Force] TARE done: biasF=(%+.3f, %+.3f, %+.3f) N  biasM=(%+.4f, %+.4f, %+.4f) Nm\n",
                   g_biasForce[0], g_biasForce[1], g_biasForce[2],
                   g_biasTorque[0], g_biasTorque[1], g_biasTorque[2]);

            // ===== 仅调零: 直接应用+存盘, 保留现有惯性补偿质量, 不进 MOTION、不开拖拽 =====
            if (g_tareOnly) {
                double mass = ForceCompensation::currentMassKg();
                // CR3 内部已补偿重力/惯性, 正常情况这里的质量应 ≈0 (见 force-compensation 设计)。
                // 若 force_calib.json 里带着一个非零质量 (上一次 'k' 全流程, 或换装工具前留下的),
                // 保留它会让 compensated = raw - 零偏 - m·g_tool 随姿态漂移,
                // 而且这个错误零偏会被写进文件。这时该做的是 'k' 重标, 不是 'z'。
                if (mass > 0.05) {
                    printf("[Force] WARNING: 保留的惯性补偿质量 %.3f kg 非零 —— "
                           "残余 %.2f N 的姿态相关误差会被当成零偏存盘。\n"
                           "        换装工具后应先用 'k' 全流程重标质量, 再用 'z' 调零。\n",
                           mass, mass * 9.81);
                }
                double comZero[3] = {0};
                ForceCompensation::setCalibration(mass, comZero, g_biasForce, g_biasTorque);
                ForceCalibration::saveToFile(CalibStore::fileFor("force_calib.json"), mass, g_biasForce, g_biasTorque);
                printf("[Force] ZERO complete (mass %.4f kg kept): "
                       "force bias=(%+.3f,%+.3f,%+.3f) N, torque bias=(%+.4f,%+.4f,%+.4f) Nm\n",
                       mass, g_biasForce[0], g_biasForce[1], g_biasForce[2],
                       g_biasTorque[0], g_biasTorque[1], g_biasTorque[2]);
                printf("[Force] Saved force_calib.json\n");
                g_state = State::DONE;
                g_tareOnly = false;
                break;
            }

            g_state = State::MOTION;
            g_motionCount = 0;
            g_phaseTimer = 0.0;
            if (g_dragCb) g_dragCb(true);
            printf("[Force] MOTION phase: move robot with varying speed + direction, then press SPACE\n");
        }
        break;
    }

    // ===== MOTION: record raw force + position during movement =====
    case State::MOTION: {
        g_phaseTimer += dt;
        // Record sample
        if (g_motionCount < MAX_MOTION_SAMPLES) {
            for (int i = 0; i < 6; i++) g_motionRaw[g_motionCount][i] = raw[i];
            g_motionPos[g_motionCount][0] = pose[0];
            g_motionPos[g_motionCount][1] = pose[1];
            g_motionPos[g_motionCount][2] = pose[2];
            g_motionRxyz[g_motionCount][0] = pose[3];
            g_motionRxyz[g_motionCount][1] = pose[4];
            g_motionRxyz[g_motionCount][2] = pose[5];
            g_motionCount++;
        }
        break;
    }

    // ===== SOLVE: fit m from F = m*a =====
    case State::SOLVE: {
        if (g_motionCount < 5) {
            printf("[Force] Not enough motion data (%d samples), aborting\n", g_motionCount);
            g_state = State::ABORTED;
            break;
        }

        // Compute acceleration from position (2nd-order central diff, mm→m)
        // CRITICAL: acceleration is in base frame, force is in sensor frame.
        // Rotate acceleration into sensor frame before fitting F = m*a.
        double sumFa = 0.0, sumA2 = 0.0;
        int used = 0;

        // 5-point position + orientation history
        double px[5] = {0}, py[5] = {0}, pz[5] = {0};
        double rx[5] = {0}, ry[5] = {0}, rz[5] = {0};
        int pi = 0, pn = 0;

        for (int k = 0; k < g_motionCount; k++) {
            px[pi] = g_motionPos[k][0];
            py[pi] = g_motionPos[k][1];
            pz[pi] = g_motionPos[k][2];
            rx[pi] = g_motionRxyz[k][0];
            ry[pi] = g_motionRxyz[k][1];
            rz[pi] = g_motionRxyz[k][2];
            pi = (pi + 1) % 5;
            if (pn < 5) pn++;

            if (pn < 5) continue;

            int i0 = (pi - 1 + 5) % 5;  // newest
            int i1 = (pi - 2 + 5) % 5;  // middle
            int i2 = (pi - 3 + 5) % 5;

            // 3-point stencil acceleration in BASE frame (m/s²)
            double ax_b = (px[i0] - 2.0 * px[i1] + px[i2]) / (dt * dt) * 0.001;
            double ay_b = (py[i0] - 2.0 * py[i1] + py[i2]) / (dt * dt) * 0.001;
            double az_b = (pz[i0] - 2.0 * pz[i1] + pz[i2]) / (dt * dt) * 0.001;

            // Rotate acceleration: a_sensor = R_base→sensor * a_base
            // R_base→sensor = (Rz*Ry*Rx)^T  for the MIDDLE sample's orientation
            double rxx = rx[i1] * M_PI / 180.0;
            double ryy = ry[i1] * M_PI / 180.0;
            double rzz = rz[i1] * M_PI / 180.0;
            double cx = cos(rxx), sx = sin(rxx);
            double cy = cos(ryy), sy = sin(ryy);
            double cz = cos(rzz), sz = sin(rzz);
            // R = Rz*Ry*Rx
            double R00 = cz*cy, R01 = cz*sy*sx - sz*cx, R02 = cz*sy*cx + sz*sx;
            double R10 = sz*cy, R11 = sz*sy*sx + cz*cx, R12 = sz*sy*cx - cz*sx;
            double R20 = -sy,   R21 = cy*sx,              R22 = cy*cx;
            // a_sensor = R^T * a_base
            double ax = R00*ax_b + R10*ay_b + R20*az_b;
            double ay = R01*ax_b + R11*ay_b + R21*az_b;
            double az = R02*ax_b + R12*ay_b + R22*az_b;

            double aMag = sqrt(ax*ax + ay*ay + az*az);
            if (aMag < 0.05) continue;  // skip near-static

            // Force in sensor frame (raw - bias)
            double fx = g_motionRaw[i1][0] - g_biasForce[0];
            double fy = g_motionRaw[i1][1] - g_biasForce[1];
            double fz = g_motionRaw[i1][2] - g_biasForce[2];
            double fMag = sqrt(fx*fx + fy*fy + fz*fz);

            // F = m*a → m ≈ Σ(|F|·|a|) / Σ(|a|²)
            sumFa += fMag * aMag;
            sumA2 += aMag * aMag;
            used++;
        }

        printf("[Force] Motion fit: %d valid samples (a > 0.05 m/s²)\n", used);

        if (used < 10 || sumA2 < 0.01) {
            printf("[Force] WARNING: insufficient motion data, using m=0\n");
            g_massKg = 0.0;
        } else {
            g_massKg = sumFa / sumA2;
            // Sanity check
            if (g_massKg < 0.0) {
                printf("[Force] WARNING: negative mass (%.3f kg), using 0\n", g_massKg);
                g_massKg = 0.0;
            } else if (g_massKg > 50.0) {
                printf("[Force] WARNING: mass too large (%.1f kg), capping at 5.0\n", g_massKg);
                g_massKg = 5.0;
            }
        }

        printf("[Force] SOLVE: mass=%.4f kg\n", g_massKg);

        // Apply results
        double comZero[3] = {0};
        ForceCompensation::setCalibration(g_massKg, comZero, g_biasForce, g_biasTorque);
        ForceCalibration::saveToFile(CalibStore::fileFor("force_calib.json"), g_massKg, g_biasForce, g_biasTorque);

        printf("[Force] Calibration complete! bias=(%+.3f,%+.3f,%+.3f)N  mass=%.3f kg\n",
               g_biasForce[0], g_biasForce[1], g_biasForce[2], g_massKg);
        g_state = State::DONE;
        break;
    }

    default:
        break;
    }

    return (g_state == State::DONE || g_state == State::ABORTED);
}

// ===== Persistence =====

bool saveToFile(const char* path, double massKg,
                const double biasForce[3], const double biasTorque[3])
{
    FILE* f = fopen(path, "w");
    if (!f) return false;
    fprintf(f, "{\n");
    fprintf(f, "  \"version\": 3,\n");
    // CalibStore 按这个字段判有效期; 缺了它整份标定会被判过期。
    fprintf(f, "  \"saved_at_unix\": %ld,\n", (long)time(NULL));
    fprintf(f, "  \"mass_kg\": %.6g,\n", massKg);
    fprintf(f, "  \"bias_force_n\": [%.6g, %.6g, %.6g],\n",
            biasForce[0], biasForce[1], biasForce[2]);
    fprintf(f, "  \"bias_torque_nm\": [%.6g, %.6g, %.6g]\n",
            biasTorque[0], biasTorque[1], biasTorque[2]);
    fprintf(f, "}\n");
    fclose(f);
    return true;
}

// Helper: find key in JSON buf, return pointer to first char after ':' (skipping whitespace)
static const char* jsonFind(const char* buf, const char* key) {
    const char* p = strstr(buf, key);
    if (!p) return nullptr;
    p += strlen(key);
    while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r') p++;
    if (*p == ':') p++;
    while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r') p++;
    return p;  // points to '[' or first digit/'-'
}

bool loadFromFile(const char* path, double& massKg,
                  double biasForce[3], double biasTorque[3])
{
    FILE* f = fopen(path, "r");
    if (!f) return false;

    char buf[4096];
    size_t n = fread(buf, 1, sizeof(buf) - 1, f);
    fclose(f);
    if (n == 0) return false;
    buf[n] = '\0';

    const char* p = jsonFind(buf, "\"mass_kg\"");
    if (!p) return false;
    massKg = strtod(p, nullptr);

    p = jsonFind(buf, "\"bias_force_n\"");
    if (!p) return false;
    if (*p == '[') p++;  // skip opening bracket
    for (int i = 0; i < 3; i++) {
        char* end = nullptr;
        biasForce[i] = strtod(p, &end);
        if (end == p) return false;
        p = end;
        while (*p == ',' || *p == ' ' || *p == '\n' || *p == '\r' || *p == '\t' || *p == ']') p++;
    }

    p = jsonFind(buf, "\"bias_torque_nm\"");
    if (!p) return false;
    if (*p == '[') p++;
    for (int i = 0; i < 3; i++) {
        char* end = nullptr;
        biasTorque[i] = strtod(p, &end);
        if (end == p) return false;
        p = end;
        while (*p == ',' || *p == ' ' || *p == '\n' || *p == '\r' || *p == '\t' || *p == ']') p++;
    }

    return true;
}

} // namespace ForceCalibration
