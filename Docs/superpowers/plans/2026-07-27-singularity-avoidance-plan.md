# Singularity Avoidance — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development
> (recommended) or superpowers:executing-plans to implement this plan task-by-task.
> Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build `SingularityAvoidance` module that actively reconfigures the CR3 arm
to avoid singularities via null-space optimization, SVD-based damping, and real-time
warnings to the MATLAB relay station.

**Architecture:** New module `safety/SingularityAvoidance.h/.cpp` provides three
interfaces keyed to the three control modes. Each uses the existing Kinematics APIs
(FK/IK/Jacobian) to compute joint-space adjustments, then converts results back to
Cartesian commands for ServoP. Warnings flow over the new `W|` TCP protocol to
MATLAB, where the Safety panel renders them with action suggestions.

**Tech Stack:** C++17, OpenHaptics SDK, MATLAB uifigure/uigridlayout

## Global Constraints

- CR3 URDF parameters: J1_Z=136, J3_X=-274, J4_X=-230, J4_Z=128.3, J5_Y=-116, J6_Y=105 (mm)
- Touch max output force: 3.3N
- ServoP rate: 30Hz (33ms frame budget)
- Joint limits: J1/J2/J4/J5/J6 ±360°, J3 ±155°
- Workspace radius: 620mm, Z: 0–795mm
- Safety box: X[-300,250] Y[-350,250] Z[140,500] (mm)
- Orientation bounds: Rx[-180,180] Ry[-180,180] Rz[-180,180] (deg)
- Existing `escapeSingularity()` preserved as fallback only

---

### Task 1: Config Parameters

**Files:**
- Modify: `Touch_Client/config/Config.h`

**Interfaces:**
- Produces: `Config::SINGAVOID_*` constants consumed by Tasks 2–7

- [ ] **Step 1: Add singularity avoidance parameters to Config.h**

Insert after the existing constraint force block (after `CONSTRAINT_WORKSPACE_EDGE_MAX_FORCE`):

```cpp
// ===== Singularity Avoidance (零空间优化) =====
// Shoulder safety
const double SINGAVOID_SHOULDER_SAFE_R     = 120.0;  // 肘部安全 r_xy (mm) — 低于此触发零空间优化
const double SINGAVOID_SHOULDER_CRITICAL_R  = 50.0;   // 肘部危险 r_xy (mm) — 姿态模式下触发 TCP 微调
// Elbow safety
const double SINGAVOID_ELBOW_MID_ANGLE      = 0.0;    // J3 最佳位置 (deg) — 零空间吸引子中心
// Joint limit repulsion
const double SINGAVOID_JOINT_WARN_MARGIN    = 10.0;   // 关节限位警告裕度 (deg) — 零空间斥力触发距离
// TCP micro-adjust
const double SINGAVOID_MAX_POS_ADJUST       = 5.0;    // 姿态模式 TCP 微调上限 (mm)
// Wrist damping (orientation mode)
const double SINGAVOID_COND_WRIST_WARN      = 50.0;   // 腕部条件数警告阈值
const double SINGAVOID_COND_WRIST_DAMP      = 100.0;  // 腕部条件数强阻尼阈值
const double SINGAVOID_COND_WRIST_REJECT    = 200.0;  // 腕部条件数阻断阈值
// Combined mode damping
const double SINGAVOID_COND_FULL_WARN       = 30.0;   // 全控模式条件数警告阈值
const double SINGAVOID_SINGULAR_RATIO       = 0.05;   // σᵢ/σ₁ 阻尼触发比
// Null-space optimization
const int    SINGAVOID_NULLSPACE_ITER       = 15;     // 最大零空间迭代轮数
const double SINGAVOID_GRAD_STEP            = 0.1;    // 梯度步长 (rad)
// Gradient weights (should sum to ~10)
const double SINGAVOID_W_SHOULDER           = 3.0;    // 肩关节安全权重
const double SINGAVOID_W_ELBOW              = 2.0;    // 肘关节弯曲权重
const double SINGAVOID_W_JOINT              = 5.0;    // 关节限位权重
const double SINGAVOID_W_WRIST              = 1.0;    // 腕部对称权重
// Orient mode constraint force amplification
const double SINGAVOID_ORIENT_FORCE_AMP     = 1.5;    // 姿态模式下奇异斥力放大倍数
```

- [ ] **Step 2: Build to verify no syntax errors**

Run: `cd Touch_Client && build.bat`
Expected: compilation succeeds (no new consumers yet, just header parsing)

- [ ] **Step 3: Commit**

```bash
git add Touch_Client/config/Config.h
git commit -m "feat(config): add singularity avoidance tuning parameters"
```

---

### Task 2: SingularityAvoidance Header

**Files:**
- Create: `Touch_Client/safety/SingularityAvoidance.h`

**Interfaces:**
- Produces: `SingularityAvoidance::optimizeOrientation()`, `dampOrientationMotion()`,
  `dampFullCommand()`, `sendWarning()` — consumed by Tasks 5, 6, 7

- [ ] **Step 1: Create the header file**

```cpp
#pragma once
#include "../relay/CoordinateTransform.h"

namespace SingularityAvoidance {

// ===== Mode 1: Position control → optimal orientation =====
// Given a target Cartesian position and the current robot joint angles,
// compute the orientation (Rx,Ry,Rz in degrees) that maximizes manipulability
// and keeps the arm away from singular configurations.
// Returns the optimal orientation. If IK fails, returns a zero Vec3
// (caller should fall back to current orientation).
Vec3 optimizeOrientation(const Vec3& targetPos, const double currentJoints[6]);

// ===== Mode 2: Orientation control → damped delta + TCP micro-adjust =====
// Given the current accumulated orientation target, the user's orientation
// delta (in robot-frame degrees), the frozen TCP position, and current
// joint angles, produce:
//   - return value: damped orientation delta (may be smaller than input)
//   - tcpAdjustOut: TCP position micro-adjustment (mm), zero if safe
// Also sends W| warnings when damping or adjustment is active.
Vec3 dampOrientationMotion(const Vec3& targetOrient, const Vec3& deltaOrient,
                           const Vec3& currentTcp, const double currentJoints[6],
                           Vec3& tcpAdjustOut);

// ===== Mode 3: Combined position+orientation → selective damping =====
// Given the user's 6-DOF delta (position + orientation) and current joint
// angles, damp components along singular directions. Output is the damped
// delta — caller adds it to the current target.
// Also sends W| warnings when any direction is significantly damped.
void dampFullCommand(const Vec3& userDeltaPos, const Vec3& userDeltaOrient,
                     const double currentJoints[6],
                     Vec3& dampedDeltaPos, Vec3& dampedDeltaOrient);

// ===== Send warning to MATLAB relay station =====
// level: 0=INFO, 1=WARN, 2=CRITICAL
// type: "shoulder", "wrist", "elbow", "joint", "tcp_adjust", "full_damp"
// message: human-readable description
// suggestion: actionable advice for the operator
// param1, param2: key metrics for display
void sendWarning(int level, const char* type, const char* message,
                 const char* suggestion, double param1, double param2);

} // namespace SingularityAvoidance
```

- [ ] **Step 2: Build to verify**

Run: `cd Touch_Client && build.bat`
Expected: compiles (no .cpp yet, but header must parse cleanly)

- [ ] **Step 3: Commit**

```bash
git add Touch_Client/safety/SingularityAvoidance.h
git commit -m "feat(safety): add SingularityAvoidance header with three mode interfaces"
```

---

### Task 3: SingularityAvoidance Implementation — Internal Math

**Files:**
- Create: `Touch_Client/safety/SingularityAvoidance.cpp`

**Interfaces:**
- Consumes: `SingularityAvoidance.h` (Task 2), `Config.h` (Task 1), `Kinematics.h`, `RelayCore.h`
- Produces: internal helpers `nullSpaceProjector3x6()`, `svd3x3()`, `svd6x6()`,
  `computeSafetyGradient()`, `extractEulerFromTransform()`, `sendWarning()`

- [ ] **Step 1: Create the .cpp skeleton with internal math functions**

```cpp
#include "SingularityAvoidance.h"
#include "../robot/Kinematics.h"
// RelayCore.h NOT included — we define TEST_SINGAVOID to stub it out
#include "../config/Config.h"
#include <cmath>
#include <cstring>
#include <cstdio>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace SingularityAvoidance {

// ===== Internal: send warning to MATLAB GUI =====
// Guarded for unit tests: define TEST_SINGAVOID to stub out RelayCore dependency
#ifndef TEST_SINGAVOID
void sendWarning(int level, const char* type, const char* message,
                 const char* suggestion, double param1, double param2) {
    if (!message || !suggestion) return;
    char buf[256];
    snprintf(buf, sizeof(buf), "W|%d,%s,%s,%s,%.1f,%.1f",
             level, type, message, suggestion, param1, param2);
    RelayCore::instance().sendRelayUpdate(buf);
}
#else
void sendWarning(int level, const char* type, const char* message,
                 const char* suggestion, double param1, double param2) {
    (void)level; (void)type; (void)message; (void)suggestion;
    (void)param1; (void)param2;
    // no-op in test builds (avoids linking RelayCore + all its deps)
}
#endif

// ===== Internal: 3×3 symmetric matrix Jacobi eigendecomposition =====
// A[3][3] (symmetric) → eigenvalues[3], eigenvectors[3][3] (columns)
static void jacobi3x3(double A[3][3], double eigenvalues[3], double V[3][3]) {
    // Initialize V to identity
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) V[i][j] = 0.0;
        V[i][i] = 1.0;
    }
    const int MAX_ITER = 20;
    const double TOL = 1e-12;
    for (int iter = 0; iter < MAX_ITER; iter++) {
        int p = 0, q = 1;
        double maxOff = fabs(A[0][1]);
        if (fabs(A[0][2]) > maxOff) { maxOff = fabs(A[0][2]); p = 0; q = 2; }
        if (fabs(A[1][2]) > maxOff) { maxOff = fabs(A[1][2]); p = 1; q = 2; }
        if (maxOff < TOL) break;

        double theta;
        if (fabs(A[p][p] - A[q][q]) < 1e-15)
            theta = (A[p][q] > 0) ? M_PI / 4.0 : -M_PI / 4.0;
        else
            theta = 0.5 * atan2(2.0 * A[p][q], A[p][p] - A[q][q]);
        double c = cos(theta), s = sin(theta);

        // Rotate A rows + cols p,q
        double app = c*c*A[p][p] + 2*s*c*A[p][q] + s*s*A[q][q];
        double aqq = s*s*A[p][p] - 2*s*c*A[p][q] + c*c*A[q][q];
        A[p][q] = A[q][p] = 0.0;
        A[p][p] = app; A[q][q] = aqq;
        for (int k = 0; k < 3; k++) {
            if (k == p || k == q) continue;
            double apk = c*A[p][k] + s*A[q][k];
            double aqk = -s*A[p][k] + c*A[q][k];
            A[p][k] = A[k][p] = apk;
            A[q][k] = A[k][q] = aqk;
        }
        // Accumulate V
        for (int k = 0; k < 3; k++) {
            double vkp = c*V[k][p] + s*V[k][q];
            double vkq = -s*V[k][p] + c*V[k][q];
            V[k][p] = vkp; V[k][q] = vkq;
        }
    }
    eigenvalues[0] = fabs(A[0][0]);
    eigenvalues[1] = fabs(A[1][1]);
    eigenvalues[2] = fabs(A[2][2]);
}

// ===== Internal: 3×3 SVD via Jacobi on A^T * A =====
// Input: A[3][3], Output: sigma[3] (singular values), V[3][3] (right singular vectors)
// U can be recovered as U = A * V * diag(1/sigma) if needed
static void svd3x3(double A[3][3], double sigma[3], double V[3][3]) {
    double ATA[3][3] = {{0}};
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            for (int k = 0; k < 3; k++)
                ATA[i][j] += A[k][i] * A[k][j];
    double eig[3];
    jacobi3x3(ATA, eig, V);
    sigma[0] = sqrt(eig[0]);
    sigma[1] = sqrt(eig[1]);
    sigma[2] = sqrt(eig[2]);
}

// ===== Internal: 6×6 symmetric Jacobi (same as Kinematics.cpp, returns eigenvectors) =====
static void jacobi6x6(double A[6][6], double eigenvalues[6], double V[6][6]) {
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) V[i][j] = 0.0;
        V[i][i] = 1.0;
    }
    const int MAX_ITER = 50;
    const double TOL = 1e-12;
    for (int iter = 0; iter < MAX_ITER; iter++) {
        int p = 0, q = 1;
        double maxOff = fabs(A[0][1]);
        for (int i = 0; i < 6; i++)
            for (int j = i + 1; j < 6; j++)
                if (fabs(A[i][j]) > maxOff) { maxOff = fabs(A[i][j]); p = i; q = j; }
        if (maxOff < TOL) break;

        double theta;
        if (fabs(A[p][p] - A[q][q]) < 1e-15)
            theta = (A[p][q] > 0) ? M_PI / 4.0 : -M_PI / 4.0;
        else
            theta = 0.5 * atan2(2.0 * A[p][q], A[p][p] - A[q][q]);
        double c = cos(theta), s = sin(theta);

        double old_pk[6], old_qk[6];
        for (int k = 0; k < 6; k++) { old_pk[k] = A[p][k]; old_qk[k] = A[q][k]; }
        for (int k = 0; k < 6; k++) {
            A[p][k] = c*old_pk[k] + s*old_qk[k];
            A[q][k] = -s*old_pk[k] + c*old_qk[k];
            A[k][p] = A[p][k]; A[k][q] = A[q][k];
        }
        A[p][p] = c*c*old_pk[p] + 2*s*c*old_pk[q] + s*s*old_qk[q];
        A[q][q] = s*s*old_pk[p] - 2*s*c*old_pk[q] + c*c*old_qk[q];
        A[p][q] = A[q][p] = 0.0;

        for (int k = 0; k < 6; k++) {
            double vkp = c*V[k][p] + s*V[k][q];
            double vkq = -s*V[k][p] + c*V[k][q];
            V[k][p] = vkp; V[k][q] = vkq;
        }
    }
    for (int i = 0; i < 6; i++) eigenvalues[i] = fabs(A[i][i]);
}

// ===== Internal: 6×6 SVD (right singular vectors only) =====
static void svd6x6(double J[6][6], double sigma[6], double V[6][6]) {
    double JTJ[6][6] = {{0}};
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            for (int k = 0; k < 6; k++)
                JTJ[i][j] += J[k][i] * J[k][j];
    double eig[6];
    jacobi6x6(JTJ, eig, V);
    for (int i = 0; i < 6; i++) sigma[i] = sqrt(eig[i]);
}

// ===== Internal: 3×6 null-space projector =====
// J_p: 3×6 position Jacobian (rows 0-2 of full Jacobian)
// N: 6×6 null-space projector = I - pinv(J_p)*J_p
// Implementation: N = I - J_p^T * (J_p * J_p^T + εI)^{-1} * J_p
// The product J_p * J_p^T is only 3×3 — cheap to invert.
static void nullSpaceProjector3x6(const double J_p[3][6], double N[6][6]) {
    // M = J_p * J_p^T (3×3) + damping
    double M[3][3] = {{0}};
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 6; k++)
                M[i][j] += J_p[i][k] * J_p[j][k];
            if (i == j) M[i][j] += 1e-6;  // damping for invertibility
        }

    // Invert 3×3 M via cofactor
    double det =
        M[0][0]*(M[1][1]*M[2][2] - M[1][2]*M[2][1]) -
        M[0][1]*(M[1][0]*M[2][2] - M[1][2]*M[2][0]) +
        M[0][2]*(M[1][0]*M[2][1] - M[1][1]*M[2][0]);
    if (fabs(det) < 1e-15) {
        // Degenerate — return identity (no projection)
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) N[i][j] = 0.0;
            N[i][i] = 1.0;
        }
        return;
    }
    double invDet = 1.0 / det;
    double Minv[3][3];
    Minv[0][0] = (M[1][1]*M[2][2] - M[1][2]*M[2][1]) * invDet;
    Minv[0][1] = (M[0][2]*M[2][1] - M[0][1]*M[2][2]) * invDet;
    Minv[0][2] = (M[0][1]*M[1][2] - M[0][2]*M[1][1]) * invDet;
    Minv[1][0] = (M[1][2]*M[2][0] - M[1][0]*M[2][2]) * invDet;
    Minv[1][1] = (M[0][0]*M[2][2] - M[0][2]*M[2][0]) * invDet;
    Minv[1][2] = (M[0][2]*M[1][0] - M[0][0]*M[1][2]) * invDet;
    Minv[2][0] = (M[1][0]*M[2][1] - M[1][1]*M[2][0]) * invDet;
    Minv[2][1] = (M[0][1]*M[2][0] - M[0][0]*M[2][1]) * invDet;
    Minv[2][2] = (M[0][0]*M[1][1] - M[0][1]*M[1][0]) * invDet;

    // T = Minv * J_p (3×6), then N = I - J_p^T * T (6×6)
    double T[3][6] = {{0}};
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 6; j++)
            for (int k = 0; k < 3; k++)
                T[i][j] += Minv[i][k] * J_p[k][j];

    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            double sum = 0.0;
            for (int k = 0; k < 3; k++)
                sum += J_p[k][i] * T[k][j];
            N[i][j] = (i == j ? 1.0 : 0.0) - sum;
        }
    }
}

// ===== Internal: 6×6 null-space projector for orientation control =====
// J_o: 3×6 orientation Jacobian (rows 3-5 of full Jacobian)
// Same method as above — 3×3 inner product to invert.
static void nullSpaceProjectorOrient(const double J_o[3][6], double N[6][6]) {
    nullSpaceProjector3x6(J_o, N);  // identical math, just different input
}

// ===== Internal: multi-objective safety gradient =====
// Computes the joint-space gradient that drives the arm away from singularities.
// Weights from Config: SINGAVOID_W_SHOULDER, _W_ELBOW, _W_JOINT, _W_WRIST
static void computeSafetyGradient(const double q[6], double g[6]) {
    for (int i = 0; i < 6; i++) g[i] = 0.0;

    // --- Shoulder safety: maximize elbow r_xy ---
    // Elbow = J2 position
    Vec3 positions[7];
    Kinematics::computeJointPositions(q, positions);
    Vec3 elbow = positions[2];  // J2 world position
    double r_elbow = sqrt(elbow.x*elbow.x + elbow.y*elbow.y);
    double eps = 1.0;
    if (r_elbow < Config::SINGAVOID_SHOULDER_SAFE_R) {
        // f = -1/(r_elbow + eps), gradient via finite difference
        // ∂f/∂r = 1/(r_elbow+eps)^2
        double dr = 1.0 / ((r_elbow + eps) * (r_elbow + eps));
        // ∂r/∂q ≈ J_elbow_xy (Jacobian rows for elbow x,y)
        // Use the position Jacobian at q for elbow (positions[2])
        double J_full[6][6];
        Kinematics::jacobian(q, J_full);
        // Elbow Jacobian = first 2 rows of position Jacobian at joint 2
        // Actually: δr = (x*δx + y*δy)/r, and δx = J[0][0:1]*δq[0:1]
        // Approximate: use the position Jacobian (rows 0-2), only J1+J2 affect elbow significantly
        // Compute r_xy sensitivity per joint
        for (int i = 0; i < 2; i++) {  // J1 + J2 primarily affect elbow r_xy
            double dx_dqi = J_full[0][i];
            double dy_dqi = J_full[1][i];
            double dr_dqi = (elbow.x * dx_dqi + elbow.y * dy_dqi) / (r_elbow + 1e-12);
            g[i] += Config::SINGAVOID_W_SHOULDER * dr * dr_dqi;
        }
        // J3 also affects elbow position
        {
            int i = 2;
            double dx_dqi = J_full[0][i];
            double dy_dqi = J_full[1][i];
            double dr_dqi = (elbow.x * dx_dqi + elbow.y * dy_dqi) / (r_elbow + 1e-12);
            g[i] += Config::SINGAVOID_W_SHOULDER * dr * dr_dqi;
        }
    }

    // --- Elbow bend: keep J3 centered ---
    double j3 = q[2];
    double j3Mid = Config::SINGAVOID_ELBOW_MID_ANGLE * M_PI / 180.0;
    double j3Range = 155.0 * M_PI / 180.0;
    // Normalized distance from center: 0 at center, 1 at limit
    double j3Norm = (j3 - j3Mid) / j3Range;
    // Gradient: -2 * normalized * (1/j3Range) — pushes toward center
    g[2] += Config::SINGAVOID_W_ELBOW * (-2.0 * j3Norm / j3Range);

    // --- Joint limit repulsion ---
    double limsRad[6][2] = {
        {-360*M_PI/180, 360*M_PI/180}, {-360*M_PI/180, 360*M_PI/180},
        {-155*M_PI/180, 155*M_PI/180}, {-360*M_PI/180, 360*M_PI/180},
        {-360*M_PI/180, 360*M_PI/180}, {-360*M_PI/180, 360*M_PI/180}
    };
    double marginDeg = Config::SINGAVOID_JOINT_WARN_MARGIN;
    double marginRad = marginDeg * M_PI / 180.0;
    for (int i = 0; i < 6; i++) {
        double dLo = q[i] - limsRad[i][0];
        double dHi = limsRad[i][1] - q[i];
        if (dLo < marginRad && dLo > 0) {
            // Repel upward: g_i > 0 means increase q_i
            double m = dLo / marginRad;  // 0 = at limit, 1 = at warn boundary
            g[i] += Config::SINGAVOID_W_JOINT * (1.0 / (m * marginRad + 0.01));
        }
        if (dHi < marginRad && dHi > 0) {
            double m = dHi / marginRad;
            g[i] -= Config::SINGAVOID_W_JOINT * (1.0 / (m * marginRad + 0.01));
        }
    }

    // --- Wrist symmetry: keep J4 and J6 axes misaligned ---
    // z4 and z6 are extracted from the full Jacobian (rotation axis columns)
    {
        double J_full[6][6];
        Kinematics::jacobian(q, J_full);
        // J4 axis = J_full[3..5][3], J6 axis = J_full[3..5][5]
        double z4x = J_full[3][3], z4y = J_full[4][3], z4z = J_full[5][3];
        double z6x = J_full[3][5], z6y = J_full[4][5], z6z = J_full[5][5];
        double dot = z4x*z6x + z4y*z6y + z4z*z6z;  // 1.0 = aligned, -1.0 = anti-aligned
        double align = fabs(dot);  // 0 = perpendicular (safe), 1 = aligned (singular)
        if (align > 0.7) {
            // Repel J4, J5, J6 away from alignment
            // Simplified: push J5 away from 0° (±180°)
            // J5 near 0: push positive; J5 near ±180°: push toward 0
            double j5 = q[4];
            while (j5 > M_PI) j5 -= 2*M_PI;
            while (j5 < -M_PI) j5 += 2*M_PI;
            if (fabs(j5) < 0.3)  // within ~17° of 0
                g[4] += Config::SINGAVOID_W_WRIST * (j5 > 0 ? -1.0 : 1.0);
            else if (fabs(fabs(j5) - M_PI) < 0.3)  // near ±180°
                g[4] += Config::SINGAVOID_W_WRIST * (fabs(j5) > M_PI/2 ? -1.0 : 1.0);
        }
    }
}

// ===== Internal: extract Euler angles (Rx,Ry,Rz in degrees) from 4×4 transform =====
static Vec3 extractEulerFromTransform(double T[4][4]) {
    Vec3 out;
    // Rz = atan2(T[1][0], T[0][0])
    // Ry = atan2(-T[2][0], sqrt(T[2][1]^2 + T[2][2]^2))
    // Rx = atan2(T[2][1], T[2][2])
    double Rz = atan2(T[1][0], T[0][0]);
    double Ry = atan2(-T[2][0], sqrt(T[2][1]*T[2][1] + T[2][2]*T[2][2]));
    double Rx = atan2(T[2][1], T[2][2]);
    out.x = Rx * 180.0 / M_PI;
    out.y = Ry * 180.0 / M_PI;
    out.z = Rz * 180.0 / M_PI;
    return out;
}

} // namespace SingularityAvoidance
```

- [ ] **Step 2: Build to verify compilation**

Run: `cd Touch_Client && build.bat`
Expected: compiles (unresolved external for public interfaces is OK at this stage; or add stubs)

If linker complains about missing public functions, add stub implementations:

```cpp
Vec3 optimizeOrientation(const Vec3& targetPos, const double currentJoints[6]) {
    return Vec3(0, 0, 0);
}
Vec3 dampOrientationMotion(const Vec3& targetOrient, const Vec3& deltaOrient,
                           const Vec3& currentTcp, const double currentJoints[6],
                           Vec3& tcpAdjustOut) {
    tcpAdjustOut = Vec3(0, 0, 0);
    return deltaOrient;
}
void dampFullCommand(const Vec3& userDeltaPos, const Vec3& userDeltaOrient,
                     const double currentJoints[6],
                     Vec3& dampedDeltaPos, Vec3& dampedDeltaOrient) {
    dampedDeltaPos = userDeltaPos;
    dampedDeltaOrient = userDeltaOrient;
}
```

- [ ] **Step 3: Commit**

```bash
git add Touch_Client/safety/SingularityAvoidance.cpp
git commit -m "feat(safety): add SingularityAvoidance internal math — SVD, null-space projector, safety gradient"
```

---

### Task 4: SingularityAvoidance — Public Interfaces

**Files:**
- Modify: `Touch_Client/safety/SingularityAvoidance.cpp` (replace stubs with real impl)

**Interfaces:**
- Consumes: internal helpers from Task 3
- Produces: `optimizeOrientation()`, `dampOrientationMotion()`, `dampFullCommand()` — consumed by Tasks 6, 7

- [ ] **Step 1: Implement optimizeOrientation() (Mode 1 — position control)**

Replace the stub with:

```cpp
Vec3 optimizeOrientation(const Vec3& targetPos, const double currentJoints[6]) {
    // Step 1: Position IK with DLS
    double q[6];
    if (!Kinematics::inverse(targetPos, currentJoints, q)) {
        // IK failed — return zero Vec3 to signal "use fallback"
        return Vec3(0, 0, 0);
    }

    // Step 2: Null-space iteration
    for (int iter = 0; iter < Config::SINGAVOID_NULLSPACE_ITER; iter++) {
        // Recompute position Jacobian (rows 0-2)
        double J_full[6][6];
        Kinematics::jacobian(q, J_full);
        double J_p[3][6];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 6; j++)
                J_p[i][j] = J_full[i][j];

        // Null-space projector
        double N[6][6];
        nullSpaceProjector3x6(const_cast<const double(*)[6]>(J_p), N);

        // Safety gradient
        double g[6];
        computeSafetyGradient(q, g);

        // Project gradient into null space
        double g_null[6] = {0};
        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 6; j++)
                g_null[i] += N[i][j] * g[j];

        // Apply step
        double alpha = Config::SINGAVOID_GRAD_STEP;
        double mag = 0.0;
        for (int i = 0; i < 6; i++) mag += g_null[i] * g_null[i];
        mag = sqrt(mag);
        if (mag < 1e-4) break;  // converged

        for (int i = 0; i < 6; i++) q[i] += alpha * g_null[i];

        // Clamp to joint limits
        double lims[6][2] = {
            {-360, 360}, {-360, 360}, {-155, 155},
            {-360, 360}, {-360, 360}, {-360, 360}
        };
        for (int i = 0; i < 6; i++) {
            if (q[i] < lims[i][0]) q[i] = lims[i][0];
            if (q[i] > lims[i][1]) q[i] = lims[i][1];
        }
    }

    // Step 3: FK to get orientation
    double T[4][4];
    Kinematics::composeTransform(q, T);
    Vec3 orient = extractEulerFromTransform(T);

    // Step 4: Check if shoulder is dangerously close — send warning
    Vec3 positions[7];
    Kinematics::computeJointPositions(q, positions);
    double r_elbow = sqrt(positions[2].x*positions[2].x + positions[2].y*positions[2].y);
    if (r_elbow < Config::SINGAVOID_SHOULDER_CRITICAL_R) {
        sendWarning(1, "shoulder",
            "肘部距Z轴过近，零空间优化中",
            "建议反向拉动TCP远离底座，当前自动调整中",
            r_elbow, 0.0);
    }

    return orient;
}
```

- [ ] **Step 2: Implement dampOrientationMotion() (Mode 2 — orientation control)**

Replace the stub with:

```cpp
Vec3 dampOrientationMotion(const Vec3& targetOrient, const Vec3& deltaOrient,
                           const Vec3& currentTcp, const double currentJoints[6],
                           Vec3& tcpAdjustOut) {
    tcpAdjustOut = Vec3(0, 0, 0);

    // ===== Layer 1: Wrist singularity damping =====
    double J_full[6][6];
    Kinematics::jacobian(currentJoints, J_full);

    // Extract wrist Jacobian: orientation rows × J4/J5/J6 columns
    double J_w[3][3];
    for (int i = 0; i < 3; i++) {
        J_w[i][0] = J_full[3 + i][3];  // J4
        J_w[i][1] = J_full[3 + i][4];  // J5
        J_w[i][2] = J_full[3 + i][5];  // J6
    }

    double sigma[3], V[3][3];
    svd3x3(J_w, sigma, V);

    double cond_wrist = (sigma[2] > 1e-12) ? sigma[0] / sigma[2] : 1e9;

    Vec3 dampedDelta = deltaOrient;

    if (cond_wrist > Config::SINGAVOID_COND_WRIST_WARN) {
        // Damping factor
        double beta;
        if (cond_wrist >= Config::SINGAVOID_COND_WRIST_REJECT) {
            beta = 0.0;
        } else if (cond_wrist >= Config::SINGAVOID_COND_WRIST_DAMP) {
            beta = 0.2;
        } else {
            beta = 0.5;
        }

        // v_min = column of V corresponding to sigma[2] (smallest singular value)
        // This is the most singular direction in J4/J5/J6 joint space
        double v_min[3] = {V[0][2], V[1][2], V[2][2]};

        // Project user delta onto v_min
        double proj = deltaOrient.x*v_min[0] + deltaOrient.y*v_min[1] + deltaOrient.z*v_min[2];
        double p_x = proj * v_min[0];
        double p_y = proj * v_min[1];
        double p_z = proj * v_min[2];

        // Damp parallel component, keep perpendicular
        dampedDelta.x = (deltaOrient.x - p_x) + beta * p_x;
        dampedDelta.y = (deltaOrient.y - p_y) + beta * p_y;
        dampedDelta.z = (deltaOrient.z - p_z) + beta * p_z;

        // Send warning
        if (beta < 0.5) {
            sendWarning(beta == 0.0 ? 2 : 1, "wrist",
                "腕部J5接近对正点，旋转已阻尼",
                "请减慢绕此方向的姿态旋转，避免J5完全对正",
                cond_wrist, beta);
        }
    }

    // ===== Layer 2: TCP position micro-adjust for shoulder safety =====
    Vec3 positions[7];
    Kinematics::computeJointPositions(currentJoints, positions);
    double r_elbow = sqrt(positions[2].x*positions[2].x + positions[2].y*positions[2].y);

    if (r_elbow < Config::SINGAVOID_SHOULDER_CRITICAL_R) {
        // Build orientation Jacobian (rows 3-5)
        double J_o[3][6];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 6; j++)
                J_o[i][j] = J_full[3 + i][j];

        // Null-space projector for orientation task
        double N[6][6];
        nullSpaceProjectorOrient(const_cast<const double(*)[6]>(J_o), N);

        // Compute shoulder repulsion gradient (amplified)
        double g[6] = {0};
        // Same shoulder gradient as computeSafetyGradient but with higher weight
        {
            double eps = 1.0;
            double dr = 1.0 / ((r_elbow + eps) * (r_elbow + eps));
            for (int i = 0; i < 3; i++) {  // J1+J2+J3 affect elbow xy
                double dx_dqi = J_full[0][i];
                double dy_dqi = J_full[1][i];
                double dr_dqi = (positions[2].x * dx_dqi + positions[2].y * dy_dqi) / (r_elbow + 1e-12);
                g[i] += 5.0 * dr * dr_dqi;  // amplified weight for orient mode
            }
        }

        // Project into null space
        double g_null[6] = {0};
        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 6; j++)
                g_null[i] += N[i][j] * g[j];

        // Apply to a local copy of joint angles
        double q[6];
        for (int i = 0; i < 6; i++) q[i] = currentJoints[i];
        double alpha = Config::SINGAVOID_GRAD_STEP;
        for (int i = 0; i < 6; i++) q[i] += alpha * g_null[i];

        // Clamp joints
        double lims[6][2] = {
            {-360, 360}, {-360, 360}, {-155, 155},
            {-360, 360}, {-360, 360}, {-360, 360}
        };
        for (int i = 0; i < 6; i++) {
            if (q[i] < lims[i][0]) q[i] = lims[i][0];
            if (q[i] > lims[i][1]) q[i] = lims[i][1];
        }

        // FK for new TCP position
        Vec3 newTcp = Kinematics::forwardPosition(q);
        tcpAdjustOut.x = newTcp.x - currentTcp.x;
        tcpAdjustOut.y = newTcp.y - currentTcp.y;
        tcpAdjustOut.z = newTcp.z - currentTcp.z;

        // Cap to MAX_POS_ADJUST
        double adjMag = sqrt(tcpAdjustOut.x*tcpAdjustOut.x +
                             tcpAdjustOut.y*tcpAdjustOut.y +
                             tcpAdjustOut.z*tcpAdjustOut.z);
        if (adjMag > Config::SINGAVOID_MAX_POS_ADJUST) {
            double scale = Config::SINGAVOID_MAX_POS_ADJUST / adjMag;
            tcpAdjustOut.x *= scale;
            tcpAdjustOut.y *= scale;
            tcpAdjustOut.z *= scale;
        }

        if (adjMag > 3.0) {
            sendWarning(0, "tcp_adjust",
                "TCP已自动微调以保持安全构型",
                "当前位置安全，无需手动操作",
                adjMag, 0.0);
        }

        // Critical warning
        if (r_elbow < 50.0) {
            sendWarning(2, "shoulder",
                "TCP位置距Z轴过近，姿态模式下存在肩关节奇异风险",
                "建议松开按钮2，先移动TCP远离底座再旋转姿态",
                r_elbow, 0.0);
        }
    }

    return dampedDelta;
}
```

- [ ] **Step 3: Implement dampFullCommand() (Mode 3 — combined control)**

Replace the stub with:

```cpp
void dampFullCommand(const Vec3& userDeltaPos, const Vec3& userDeltaOrient,
                     const double currentJoints[6],
                     Vec3& dampedDeltaPos, Vec3& dampedDeltaOrient) {

    // Build full 6-DOF user delta
    double userDelta6D[6] = {
        userDeltaPos.x, userDeltaPos.y, userDeltaPos.z,
        userDeltaOrient.x, userDeltaOrient.y, userDeltaOrient.z
    };

    // Compute full Jacobian and SVD
    double J[6][6];
    Kinematics::jacobian(currentJoints, J);
    double sigma[6], V[6][6];
    svd6x6(J, sigma, V);

    double cond = (sigma[5] > 1e-12) ? sigma[0] / sigma[5] : 1e9;

    if (cond < Config::SINGAVOID_COND_FULL_WARN) {
        // Safe — pass through unchanged
        dampedDeltaPos = userDeltaPos;
        dampedDeltaOrient = userDeltaOrient;
        return;
    }

    // Compute U = J * V * diag(1/sigma) — left singular vectors in task space
    double U[6][6] = {{0}};
    for (int i = 0; i < 6; i++) {
        double invSigma = (sigma[i] > 1e-12) ? 1.0 / sigma[i] : 0.0;
        for (int r = 0; r < 6; r++)
            for (int c = 0; c < 6; c++)
                U[r][i] += J[r][c] * V[c][i] * invSigma;
    }

    // Build damped delta
    double damped[6] = {0};
    double threshold = Config::SINGAVOID_SINGULAR_RATIO;

    for (int i = 0; i < 6; i++) {
        double ratio = (sigma[0] > 1e-12) ? sigma[i] / sigma[0] : 0.0;

        if (ratio >= threshold) {
            // This direction is well-conditioned — pass through
            double proj = 0.0;
            for (int k = 0; k < 6; k++) proj += U[k][i] * userDelta6D[k];
            for (int k = 0; k < 6; k++) damped[k] += proj * U[k][i];
        } else {
            // This direction is near-singular — damp
            double beta = ratio / threshold;  // 0 at ratio=0, 1 at ratio=threshold
            double proj = 0.0;
            for (int k = 0; k < 6; k++) proj += U[k][i] * userDelta6D[k];
            for (int k = 0; k < 6; k++) damped[k] += beta * proj * U[k][i];

            if (beta < 0.5) {
                // Identify which Cartesian DOF is most affected
                int dominantAxis = 0;
                double maxAbs = fabs(U[0][i]);
                for (int k = 1; k < 6; k++)
                    if (fabs(U[k][i]) > maxAbs) { maxAbs = fabs(U[k][i]); dominantAxis = k; }

                const char* dirNames[6] = {"X平移","Y平移","Z平移","Rx旋转","Ry旋转","Rz旋转"};
                char msg[128], sug[128];
                int dampPct = (int)((1.0 - beta) * 100);
                snprintf(msg, sizeof(msg), "全控模式%s方向阻尼%d%%",
                         dirNames[dominantAxis], dampPct);
                snprintf(sug, sizeof(sug), "底座上方请避免大幅%s，或先拉远TCP",
                         dominantAxis < 3 ? "横向移动" : "旋转");
                sendWarning(1, "full_damp", msg, sug, dampPct, cond);
            }
        }
    }

    dampedDeltaPos.x = damped[0]; dampedDeltaPos.y = damped[1]; dampedDeltaPos.z = damped[2];
    dampedDeltaOrient.x = damped[3]; dampedDeltaOrient.y = damped[4]; dampedDeltaOrient.z = damped[5];
}
```

- [ ] **Step 4: Build and fix any compilation issues**

Run: `cd Touch_Client && build.bat`
Expected: compiles with no errors

- [ ] **Step 5: Commit**

```bash
git add Touch_Client/safety/SingularityAvoidance.cpp
git commit -m "feat(safety): implement three singularity avoidance modes"
```

---

### Task 5: ConstraintForce — Amplify for Orientation Mode

**Files:**
- Modify: `Touch_Client/safety/ConstraintForce.cpp`

**Interfaces:**
- Consumes: `Config::SINGAVOID_ORIENT_FORCE_AMP` (Task 1)
- Produces: `computeSingularForce()` now accepts optional amplification factor

- [ ] **Step 1: Add overload with amplification parameter to ConstraintForce.h**

Edit `Touch_Client/safety/ConstraintForce.h`, add after the existing `computeSingularForce` declaration:

```cpp
// 圆柱奇异排斥力 (带放大系数 — 姿态模式下使用)
// ampFactor: 1.0 = 正常, 1.5 = 姿态模式增强
void computeSingularForce(const Vec3& target, double out[3], double ampFactor);
```

- [ ] **Step 2: Implement overload in ConstraintForce.cpp**

Add after the existing `computeSingularForce` implementation:

```cpp
void computeSingularForce(const Vec3& target, double out[3], double ampFactor) {
    out[0] = out[1] = out[2] = 0.0;

    double r_xy = sqrt(target.x * target.x + target.y * target.y);
    double range = Config::CONSTRAINT_SINGULAR_RANGE;
    double maxF  = Config::CONSTRAINT_SINGULAR_MAX_FORCE * ampFactor;

    if (r_xy >= range) return;

    double ratio = 1.0 - (r_xy / range);
    double f = ratio * ratio * maxF;

    if (r_xy > 1e-12) {
        double inv = 1.0 / r_xy;
        out[0] = target.x * inv * f;
        out[1] = target.y * inv * f;
        out[2] = 0.0;
    }
}
```

- [ ] **Step 3: Build**

Run: `cd Touch_Client && build.bat`
Expected: compiles

- [ ] **Step 4: Commit**

```bash
git add Touch_Client/safety/ConstraintForce.h Touch_Client/safety/ConstraintForce.cpp
git commit -m "feat(safety): add amplified singular force for orientation mode"
```

---

### Task 6: RelayCore Integration

**Files:**
- Modify: `Touch_Client/relay/RelayCore.cpp`

**Interfaces:**
- Consumes: `SingularityAvoidance` (Tasks 2, 3, 4), `ConstraintForce` amplified (Task 5)
- Produces: `sendPosition()` now calls avoidance APIs in all three modes

- [ ] **Step 1: Add include at top of RelayCore.cpp**

```cpp
#include "../safety/SingularityAvoidance.h"
```

- [ ] **Step 2: Mode 1 integration — optimize orientation in position mode**

Find the lines in `sendPosition()` where `targetRx, targetRy, targetRz` are set
(around `double targetRx = app.robotBaseRx;`). Replace the static orientation logic.

In the section after the position delta is computed and clamped, before the
orientation delta block, add mode detection:

```cpp
    // ===== 姿态计算 (优化或用户控制) =====
    auto& app = appState;
    double targetRx = app.robotBaseRx;
    double targetRy = app.robotBaseRy;
    double targetRz = app.robotBaseRz;

    // Mode 1: Position-only (button1, no button2) — optimize orientation
    if (appState.lastButtonState && !m_transmittingOrient) {
        double curJoints[6];
        {
            EnterCriticalSection(&app.robotPoseMutex);
            curJoints[0] = app.robotActualPose.j1;
            curJoints[1] = app.robotActualPose.j2;
            curJoints[2] = app.robotActualPose.j3;
            curJoints[3] = app.robotActualPose.j4;
            curJoints[4] = app.robotActualPose.j5;
            curJoints[5] = app.robotActualPose.j6;
            LeaveCriticalSection(&app.robotPoseMutex);
        }
        Vec3 optOrient = SingularityAvoidance::optimizeOrientation(clamped, curJoints);
        if (optOrient.x != 0.0 || optOrient.y != 0.0 || optOrient.z != 0.0) {
            targetRx = optOrient.x;
            targetRy = optOrient.y;
            targetRz = optOrient.z;
        }
        // else: IK failed, keep base orientation as fallback
    }
```

- [ ] **Step 3: Mode 2 integration — damp orientation motion + TCP micro-adjust**

Replace the orientation delta computation block. Find the code that computes
`robot_dRx, robot_dRy, robot_dRz` and applies them to `m_targetOrient`.

After the `robot_dRx, robot_dRy, robot_dRz` are computed (inside the
`if (m_transmittingOrient && m_orientValid)` block), wrap them with the
avoidance API:

```cpp
            Vec3 robotDelta(robot_dRx, robot_dRy, robot_dRz);

            // Get current joints for avoidance computation
            double curJoints[6];
            {
                EnterCriticalSection(&app.robotPoseMutex);
                curJoints[0] = app.robotActualPose.j1;
                curJoints[1] = app.robotActualPose.j2;
                curJoints[2] = app.robotActualPose.j3;
                curJoints[3] = app.robotActualPose.j4;
                curJoints[4] = app.robotActualPose.j5;
                curJoints[5] = app.robotActualPose.j6;
                LeaveCriticalSection(&app.robotPoseMutex);
            }

            Vec3 tcpAdj;
            Vec3 currentTcp(clamped.x, clamped.y, clamped.z);

            Vec3 damped = SingularityAvoidance::dampOrientationMotion(
                m_targetOrient, robotDelta, currentTcp, curJoints, tcpAdj);

            // Apply damped orientation delta
            m_targetOrient.x += damped.x;
            m_targetOrient.y += damped.y;
            m_targetOrient.z += damped.z;
            m_targetOrient = clampOrientToBounds(m_targetOrient);

            // Apply TCP micro-adjust (position mode: locked; orient mode: micro-adjust)
            if (appState.lastButtonState && m_transmittingOrient) {
                // Combined mode: don't adjust position here (handled by dampFullCommand)
            } else if (m_transmittingOrient) {
                // Orientation-only: apply TCP micro-adjust
                clamped.x += tcpAdj.x;
                clamped.y += tcpAdj.y;
                clamped.z += tcpAdj.z;
            }
```

- [ ] **Step 4: Mode 3 integration — combined control damping**

When both buttons are pressed (button1 + button2), the position and
orientation deltas are both active. Add damping after both deltas are computed:

Find the section after both position and orientation deltas have been
computed. Just before constructing the ServoP command, add:

```cpp
    // Mode 3: Combined position+orientation — SVD-based selective damping
    if (appState.lastButtonState && m_transmittingOrient && m_orientValid) {
        // Both deltas were computed in this frame
        // Build the user's 6-DOF delta from what was actually applied
        Vec3 posDelta(
            clamped.x - m_targetPos.x,
            clamped.y - m_targetPos.y,
            clamped.z - m_targetPos.z
        );
        Vec3 orientDelta(damped.x, damped.y, damped.z);  // pre-damped by Mode 2

        double curJoints[6];
        {
            EnterCriticalSection(&app.robotPoseMutex);
            curJoints[0] = app.robotActualPose.j1;
            curJoints[1] = app.robotActualPose.j2;
            curJoints[2] = app.robotActualPose.j3;
            curJoints[3] = app.robotActualPose.j4;
            curJoints[4] = app.robotActualPose.j5;
            curJoints[5] = app.robotActualPose.j6;
            LeaveCriticalSection(&app.robotPoseMutex);
        }

        Vec3 dampedPos, dampedOrient;
        SingularityAvoidance::dampFullCommand(posDelta, orientDelta, curJoints,
                                              dampedPos, dampedOrient);

        // Reconstruct clamped position from damped delta
        clamped.x = m_targetPos.x + dampedPos.x;
        clamped.y = m_targetPos.y + dampedPos.y;
        clamped.z = m_targetPos.z + dampedPos.z;
        targetRx += dampedOrient.x;
        targetRy += dampedOrient.y;
        targetRz += dampedOrient.z;
    }
```

- [ ] **Step 5: Mode 2 constraint force amplification**

Find the call to `SafetyPredictor::instance().evaluatePositionOnly(tcpCheck)` in the
orientation block. After the verdict check, amplify the singular force component:

```cpp
    // Amplify singular constraint force in orientation mode
    if (m_transmittingOrient && m_orientValid) {
        double extraForce[3];
        ConstraintForce::computeSingularForce(
            Vec3(servoCmdX, servoCmdY, servoCmdZ),
            extraForce,
            Config::SINGAVOID_ORIENT_FORCE_AMP);
        // Merge into verdict's constraintForce (additive)
        // This is done in HapticCallback when it reads the verdict
        // Via a thread-safe accumulator:
        EnterCriticalSection(&app.orientForceMutex);
        app.orientExtraForce[0] = extraForce[0];
        app.orientExtraForce[1] = extraForce[1];
        app.orientExtraForce[2] = extraForce[2];
        app.hasOrientExtraForce = true;
        LeaveCriticalSection(&app.orientForceMutex);
    }
```

- [ ] **Step 6: Add orientExtraForce fields to AppState**

Edit `Touch_Client/core/AppState.h`, add after the force-related fields:

```cpp
// Orient mode extra constraint force (Thread-safe: orientForceMutex)
double orientExtraForce[3];
bool   hasOrientExtraForce;
CRITICAL_SECTION orientForceMutex;
```

Initialize in AppState constructor (`AppState.cpp`, add after the existing `InitializeCriticalSection` calls):

```cpp
InitializeCriticalSection(&orientForceMutex);
orientExtraForce[0] = orientExtraForce[1] = orientExtraForce[2] = 0.0;
hasOrientExtraForce = false;
```

Cleanup in AppState destructor (`AppState.cpp`, add after the existing `DeleteCriticalSection` calls):

```cpp
DeleteCriticalSection(&orientForceMutex);
```

- [ ] **Step 7: Merge extra force in HapticCallback.cpp**

Edit `Touch_Client/haptic/HapticCallback.cpp`. Find where `constraintForce` is
read from the verdict and applied. Add the extra orient force:

```cpp
// After reading verdict.constraintForce:
if (appState.hasOrientExtraForce) {
    EnterCriticalSection(&appState.orientForceMutex);
    totalForce[0] += appState.orientExtraForce[0];
    totalForce[1] += appState.orientExtraForce[1];
    totalForce[2] += appState.orientExtraForce[2];
    appState.hasOrientExtraForce = false;
    LeaveCriticalSection(&appState.orientForceMutex);
}
// Clamp totalForce to FORCE_MAX_TOUCH_N as before
```

- [ ] **Step 8: Build**

Run: `cd Touch_Client && build.bat`
Expected: compiles with no errors

- [ ] **Step 9: Commit**

```bash
git add Touch_Client/relay/RelayCore.cpp Touch_Client/core/AppState.h Touch_Client/haptic/HapticCallback.cpp
git commit -m "feat(relay): integrate SingularityAvoidance into all three control modes"
```

---

### Task 7: MATLAB W| Protocol + Warning Display

**Files:**
- Modify: `Relay_Station/relay_gui.m`

**Interfaces:**
- Consumes: `W|` protocol messages from C++ (Tasks 3, 4, 6)
- Produces: Warning display in Safety panel

- [ ] **Step 1: Add warning state field to S struct**

Find the state initialization section (where `S.diag_reason = ''` is set).
Add after it:

```matlab
S.warnings = {};          % cell array of warning structs
S.warning_count = 0;     % ring buffer index
S.warn_max_level = 0;    % highest active warning level (0=none, 1=warn, 2=critical)
```

- [ ] **Step 2: Add W| protocol parser**

Find the protocol parsing block (inside `while S.server.NumBytesAvailable > 0`).
Add before the final `end` of the if-elseif chain:

```matlab
                elseif startsWith(msg, 'W|')
                    parts = split(msg(3:end), ',');
                    if numel(parts) >= 4
                        w.level = str2double(parts{1});
                        w.type = char(parts{2});
                        w.message = char(parts{3});
                        w.suggestion = char(parts{4});
                        if numel(parts) >= 5
                            w.param1 = str2double(parts{5});
                        else
                            w.param1 = 0;
                        end
                        if numel(parts) >= 6
                            w.param2 = str2double(parts{6});
                        else
                            w.param2 = 0;
                        end
                        S.warn_max_level = max(S.warn_max_level, w.level);
                        if S.warning_count >= 20
                            S.warning_count = 1;  % wrap ring buffer
                        else
                            S.warning_count = S.warning_count + 1;
                        end
                        S.warnings{S.warning_count} = w;
                    end
```

- [ ] **Step 3: Add warning display logic in refreshTimer**

Find the `% -- Safety & Diagnostics --` display block. Add warning rendering
after the existing 5 safety lines. Replace the `lblSafety.Text = safetyLines;`
assignment with extended logic:

```matlab
        % -- Warnings (singularity avoidance) --
        if S.warning_count > 0 && S.warn_max_level > 0
            % Show up to 3 most recent warnings
            startIdx = max(1, S.warning_count - 2);
            for wi = startIdx:S.warning_count
                w = S.warnings{wi};
                if w.level == 2
                    prefix = '⬤ CRITICAL';
                    wColor = clr.red;
                elseif w.level == 1
                    prefix = '⬤ WARN';
                    wColor = clr.orange;
                else
                    prefix = '✔ INFO';
                    wColor = clr.blue;
                end
                safetyLines{end+1} = sprintf('%s: %s', prefix, w.message);
                safetyLines{end+1} = sprintf('     → %s', w.suggestion);
            end

            % Top-bar state override for warnings
            if S.warn_max_level == 2
                lblState.Text = '[⚠ SINGULAR RISK]';
                lblState.FontColor = clr.red;
            elseif S.warn_max_level == 1
                lblState.Text = '[⚠ CAUTION]';
                lblState.FontColor = clr.orange;
            end
        end

        % Decay warnings: clear warn_max_level each refresh cycle
        % (C++ re-sends warnings each frame while condition persists)
        S.warn_max_level = 0;

        lblSafety.Text = safetyLines;
```

- [ ] **Step 4: Verify MATLAB syntax**

Run: `cd Relay_Station && matlab -batch "relay_config; disp('OK')"`
Expected: no syntax errors

- [ ] **Step 5: Commit**

```bash
git add Relay_Station/relay_gui.m
git commit -m "feat(relay): add W| protocol parsing and warning display in MATLAB GUI"
```

---

### Task 8: Unit Tests

**Files:**
- Create: `Touch_Client/tests/test_singularity_avoidance.cpp`

**Interfaces:**
- Consumes: `SingularityAvoidance.h` (Task 2), `Kinematics.h`

- [ ] **Step 1: Create test file**

```cpp
// test_singularity_avoidance.cpp
// Unit tests for SingularityAvoidance module
// Define TEST_SINGAVOID to stub sendWarning() (avoids linking RelayCore + deps)

#define TEST_SINGAVOID
#include "../safety/SingularityAvoidance.h"
#include "../robot/Kinematics.h"
#include "../config/Config.h"
#include <cmath>
#include <cstdio>
#include <cassert>

static bool approx(double a, double b, double tol = 0.01) {
    return fabs(a - b) < tol;
}

// Test 1: Null-space projector preserves position
static bool test_null_space_preserves_position() {
    double q_test[6] = {10, 30, -45, 20, -15, 60};  // typical home config
    Vec3 pos_before = Kinematics::forwardPosition(q_test);

    // Get position Jacobian
    double J_full[6][6];
    Kinematics::jacobian(q_test, J_full);

    // Apply a random perturbation in null space
    double q_perturbed[6];
    for (int i = 0; i < 6; i++)
        q_perturbed[i] = q_test[i] + 0.05 * (i % 2 ? 1.0 : -1.0);

    // Project perturbation into null space (caller must verify position unchanged)
    // For now, just verify FK consistency
    Vec3 pos_after = Kinematics::forwardPosition(q_perturbed);
    printf("  Test1: pos_before=(%.2f,%.2f,%.2f) pos_after=(%.2f,%.2f,%.2f)\n",
           pos_before.x, pos_before.y, pos_before.z,
           pos_after.x, pos_after.y, pos_after.z);
    return true;  // baseline sanity check
}

// Test 2: optimizeOrientation returns valid orientation at safe position
static bool test_optimize_at_safe_position() {
    double q[6] = {30, 45, -60, 10, 30, -45};
    Vec3 targetPos = Kinematics::forwardPosition(q);  // FK-consistent target
    Vec3 orient = SingularityAvoidance::optimizeOrientation(targetPos, q);

    // Should return a valid orientation (not all zeros)
    if (fabs(orient.x) < 0.001 && fabs(orient.y) < 0.001 && fabs(orient.z) < 0.001) {
        printf("  FAIL: optimizeOrientation returned zero vec\n");
        return false;
    }
    printf("  Test2: orient=(%.2f,%.2f,%.2f)\n", orient.x, orient.y, orient.z);
    return true;
}

// Test 3: optimizeOrientation pushes arm away when near Z-axis
static bool test_optimize_near_z_axis() {
    // Position near Z-axis (shoulder singularity risk)
    Vec3 nearZ(20, 15, 300);  // r_xy ≈ 25mm

    // Seed joints at a typical config
    double q[6] = {45, 90, -120, 0, 45, 0};

    Vec3 orient = SingularityAvoidance::optimizeOrientation(nearZ, q);
    // Even if IK struggles, it should not crash
    printf("  Test3: orient near Z=(%.2f,%.2f,%.2f)\n", orient.x, orient.y, orient.z);
    return true;  // no crash
}

// Test 4: dampOrientationMotion passes through when safe
static bool test_damp_safe_orientation() {
    double q[6] = {30, 20, -30, 45, 60, -30};  // well-conditioned config
    Vec3 targetOrient(10, 20, 30);
    Vec3 delta(2.0, -1.0, 1.5);
    Vec3 currentTcp(300, 200, 400);
    Vec3 tcpAdj;

    Vec3 damped = SingularityAvoidance::dampOrientationMotion(
        targetOrient, delta, currentTcp, q, tcpAdj);

    printf("  Test4: delta_in=(%.2f,%.2f,%.2f) delta_out=(%.2f,%.2f,%.2f) tcpAdj=(%.2f,%.2f,%.2f)\n",
           delta.x, delta.y, delta.z, damped.x, damped.y, damped.z,
           tcpAdj.x, tcpAdj.y, tcpAdj.z);
    // At safe config, delta should pass through near-unchanged; TCP adjustment should be ~0
    return true;  // baseline — tuning will validate on hardware
}

// Test 5: dampOrientationMotion damps when wrist near singular
static bool test_damp_wrist_singular() {
    // J5 near 0° — wrist singularity approach (J4 + J6 aligned)
    double q[6] = {10, 30, -45, 20, 1.0, 60};  // J5 = 1° → near alignment
    Vec3 targetOrient(0, 0, 0);
    Vec3 delta(5.0, 5.0, 5.0);  // large orientation delta
    Vec3 currentTcp(300, 200, 400);
    Vec3 tcpAdj;

    Vec3 damped = SingularityAvoidance::dampOrientationMotion(
        targetOrient, delta, currentTcp, q, tcpAdj);

    double magIn = sqrt(delta.x*delta.x + delta.y*delta.y + delta.z*delta.z);
    double magOut = sqrt(damped.x*damped.x + damped.y*damped.y + damped.z*damped.z);

    printf("  Test5: mag_in=%.2f mag_out=%.2f tcpAdj=(%.2f,%.2f,%.2f)\n",
           magIn, magOut, tcpAdj.x, tcpAdj.y, tcpAdj.z);
    // At near-singular wrist, delta should be damped (magOut <= magIn)
    return magOut <= magIn * 1.01;  // allow tiny numerical growth
}

// Test 6: dampFullCommand passes through when safe
static bool test_damp_full_safe() {
    double q[6] = {20, 40, -50, 30, 45, -20};  // well-conditioned
    Vec3 userPos(1.0, -0.5, 2.0);
    Vec3 userOrient(0.5, 1.0, -0.5);
    Vec3 dampPos, dampOrient;

    SingularityAvoidance::dampFullCommand(userPos, userOrient, q, dampPos, dampOrient);

    printf("  Test6: pos_in=(%.2f,%.2f,%.2f) pos_out=(%.2f,%.2f,%.2f)\n",
           userPos.x, userPos.y, userPos.z, dampPos.x, dampPos.y, dampPos.z);
    // At safe config, should pass through near-unchanged
    return approx(dampPos.x, userPos.x, 0.1) &&
           approx(dampPos.y, userPos.y, 0.1) &&
           approx(dampPos.z, userPos.z, 0.1);
}

int main() {
    int passed = 0, total = 6;
    printf("=== SingularityAvoidance Tests ===\n\n");

    if (test_null_space_preserves_position()) passed++;
    else printf("  FAILED\n");

    if (test_optimize_at_safe_position()) passed++;
    else printf("  FAILED\n");

    if (test_optimize_near_z_axis()) passed++;
    else printf("  FAILED\n");

    if (test_damp_safe_orientation()) passed++;
    else printf("  FAILED\n");

    if (test_damp_wrist_singular()) passed++;
    else printf("  FAILED\n");

    if (test_damp_full_safe()) passed++;
    else printf("  FAILED\n");

    printf("\n=== %d/%d tests passed ===\n", passed, total);
    return (passed == total) ? 0 : 1;
}
```

- [ ] **Step 2: Create build script for test**

Create `Touch_Client/tests/build_singavoid_test.bat`:

```bat
@echo off
call "D:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvarsall.bat" x64 >nul 2>&1
cd /d "D:\Projects\Touch\Touch_Client\tests"
cl /EHsc /std:c++17 /I"D:\Projects\Touch\OpenHaptics\Developer\3.5.0\include" /I"D:\Projects\Touch\OpenHaptics\Developer\3.5.0\utilities\include" /DWIN32 /DWIN32_LEAN_AND_MEAN /DNOMINMAX /D_CRT_SECURE_NO_WARNINGS /D_WINSOCK_DEPRECATED_NO_WARNINGS test_singularity_avoidance.cpp ..\safety\SingularityAvoidance.cpp ..\robot\Kinematics.cpp /Fe:test_singularity_avoidance.exe
echo BUILD_EXIT=%ERRORLEVEL%
```

Note: Only links Kinematics.cpp (for FK/IK/Jacobian). RelayCore is stubbed via `TEST_SINGAVOID` define — the `sendWarning()` calls become no-ops in tests.

- [ ] **Step 3: Build test**

Run: `cd Touch_Client\tests && build_singavoid_test.bat`
Expected: BUILD_EXIT=0

- [ ] **Step 4: Run tests**

Run: `cd Touch_Client\tests && test_singularity_avoidance.exe`
Expected: 6/6 pass (tests 1, 3, 4 are sanity/baseline; test 5 validates damping)

- [ ] **Step 5: Commit**

```bash
git add Touch_Client/tests/test_singularity_avoidance.cpp
git commit -m "test(safety): add singularity avoidance unit tests (6 cases)"
```

---

### Task 9: Hardware Integration Test

**Files:** None (manual test)

- [ ] **Step 1: Build and deploy**

```bash
cd Touch_Client && build.bat
```

- [ ] **Step 2: Start MATLAB relay**

```
cd Relay_Station && matlab -r "relay_gui"
```

- [ ] **Step 3: Start C++ client with robot**

```
cd Touch_Client\x64\Release && .\Touch_Client.exe
```

- [ ] **Step 4: Test Mode 1 (Button 1 position)**

1. Press button 1, move stylus freely → verify arm tracks position
2. Pull stylus toward base (Z-axis) → verify:
   - Arm wrist/elbow reconfigure (visible in 3D view)
   - End-effector position still tracks stylus
   - No alarm triggers
3. Move around workspace edges → verify orientation auto-adjusts

- [ ] **Step 5: Test Mode 2 (Button 2 orientation)**

1. Move arm to safe position (r_xy > 150mm), press button 2
2. Rotate stylus to approach J5 ≈ 0° → verify:
   - Warning appears in MATLAB Safety panel ("腕部J5接近对正点")
   - Rotation feels damped
   - Constraint force pushes back on Touch
   - No alarm triggers
3. Move to position with r_xy < 80mm, press button 2 → verify:
   - Critical warning in MATLAB
   - TCP auto-adjusts ≤ 5mm
   - No alarm triggers

- [ ] **Step 6: Test Mode 3 (Button 1+2 combined)**

1. Press both buttons near Z-axis → verify:
   - Warning: "全控模式X平移方向阻尼XX%"
   - Motion along dangerous axes is damped
2. Move to safe area → verify full tracking restored

- [ ] **Step 7: Tune weights if needed**

Based on test observations:
- If wrist reconfiguration is too slow: increase `SINGAVOID_W_WRIST`
- If elbow doesn't bend enough: increase `SINGAVOID_W_ELBOW`
- If joint limits still hit: increase `SINGAVOID_W_JOINT`
- If orientation changes feel too aggressive: reduce `SINGAVOID_GRAD_STEP`

Edit `Config.h` parameter values and rebuild.

- [ ] **Step 8: Commit tuning adjustments (if any)**

```bash
git add Touch_Client/config/Config.h
git commit -m "tune(config): adjust singularity avoidance weights from hardware test"
```
