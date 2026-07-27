#include "SingularityAvoidance.h"
#include "../robot/Kinematics.h"
// RelayCore.h included for sendWarning — TEST_SINGAVOID builds stub it out
#include "../relay/RelayCore.h"
#include "../config/Config.h"
#include <cmath>
#include <cstdio>
#include <utility>

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

// ===== Internal: 3x3 symmetric matrix Jacobi eigendecomposition =====
// A[3][3] (symmetric) -> eigenvalues[3], eigenvectors[3][3] (columns)
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

// ===== Internal: 3x3 SVD via Jacobi on A^T * A =====
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

    // Sort descending (bubble sort, 3 elements) — Jacobi does not guarantee order
    for (int i = 0; i < 2; i++) {
        for (int j = i + 1; j < 3; j++) {
            if (sigma[i] < sigma[j]) {
                std::swap(sigma[i], sigma[j]);
                for (int k = 0; k < 3; k++) std::swap(V[k][i], V[k][j]);
            }
        }
    }
}

// ===== Internal: 6x6 symmetric Jacobi (same as Kinematics.cpp, returns eigenvectors) =====
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

// ===== Internal: 6x6 SVD (right singular vectors only) =====
static void svd6x6(double J[6][6], double sigma[6], double V[6][6]) {
    double JTJ[6][6] = {{0}};
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            for (int k = 0; k < 6; k++)
                JTJ[i][j] += J[k][i] * J[k][j];
    double eig[6];
    jacobi6x6(JTJ, eig, V);
    for (int i = 0; i < 6; i++) sigma[i] = sqrt(eig[i]);

    // Sort descending (bubble sort, 6 elements) — Jacobi does not guarantee order
    for (int i = 0; i < 5; i++) {
        for (int j = i + 1; j < 6; j++) {
            if (sigma[i] < sigma[j]) {
                std::swap(sigma[i], sigma[j]);
                for (int k = 0; k < 6; k++) std::swap(V[k][i], V[k][j]);
            }
        }
    }
}

// ===== Internal: 3x6 null-space projector =====
// J_p: 3x6 position Jacobian (rows 0-2 of full Jacobian)
// N: 6x6 null-space projector = I - pinv(J_p)*J_p
// Implementation: N = I - J_p^T * (J_p * J_p^T + epsilon*I)^{-1} * J_p
// The product J_p * J_p^T is only 3x3 -- cheap to invert.
static void nullSpaceProjector3x6(const double J_p[3][6], double N[6][6]) {
    // M = J_p * J_p^T (3x3) + damping
    double M[3][3] = {{0}};
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 6; k++)
                M[i][j] += J_p[i][k] * J_p[j][k];
            if (i == j) M[i][j] += 1e-6;  // damping for invertibility
        }

    // Invert 3x3 M via cofactor
    double det =
        M[0][0]*(M[1][1]*M[2][2] - M[1][2]*M[2][1]) -
        M[0][1]*(M[1][0]*M[2][2] - M[1][2]*M[2][0]) +
        M[0][2]*(M[1][0]*M[2][1] - M[1][1]*M[2][0]);
    if (fabs(det) < 1e-15) {
        // Degenerate -- return identity (no projection)
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

    // T = Minv * J_p (3x6), then N = I - J_p^T * T (6x6)
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

// ===== Internal: 6x6 null-space projector for orientation control =====
// J_o: 3x6 orientation Jacobian (rows 3-5 of full Jacobian)
// Same method as above -- 3x3 inner product to invert.
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
        // df/dr = 1/(r_elbow+eps)^2
        double dr = 1.0 / ((r_elbow + eps) * (r_elbow + eps));
        // dr/dq approx J_elbow_xy (Jacobian rows for elbow x,y)
        // Use the position Jacobian at q for elbow (positions[2])
        double J_full[6][6];
        Kinematics::jacobian(q, J_full);
        // Elbow Jacobian = first 2 rows of position Jacobian at joint 2
        // Actually: delta_r = (x*delta_x + y*delta_y)/r, and delta_x = J[0][0:1]*delta_q[0:1]
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
    double j3Mid = Config::SINGAVOID_ELBOW_MID_ANGLE;  // 0.0 degrees
    double j3Range = 155.0;                             // 155 degrees (J3 limit)
    // Normalized distance from center: 0 at center, 1 at limit
    double j3Norm = (j3 - j3Mid) / j3Range;             // e.g., -60/155 = -0.39
    // Gradient: -2 * normalized * (1/j3Range) -- pushes toward center
    g[2] += Config::SINGAVOID_W_ELBOW * (-2.0 * j3Norm / j3Range);

    // --- Joint limit repulsion ---
    double limsDeg[6][2] = {
        {-360.0, 360.0}, {-360.0, 360.0}, {-155.0, 155.0},
        {-360.0, 360.0}, {-360.0, 360.0}, {-360.0, 360.0}
    };
    double marginDeg = Config::SINGAVOID_JOINT_WARN_MARGIN;  // 10.0 degrees
    for (int i = 0; i < 6; i++) {
        double dLo = q[i] - limsDeg[i][0];
        double dHi = limsDeg[i][1] - q[i];
        if (dLo < marginDeg && dLo > 0) {
            // Repel upward: g_i > 0 means increase q_i
            double m = dLo / marginDeg;  // 0 = at limit, 1 = at warn boundary
            g[i] += Config::SINGAVOID_W_JOINT * (1.0 / (m * marginDeg + 0.01));
        }
        if (dHi < marginDeg && dHi > 0) {
            double m = dHi / marginDeg;
            g[i] -= Config::SINGAVOID_W_JOINT * (1.0 / (m * marginDeg + 0.01));
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
            // Simplified: push J5 away from 0deg (+-180deg)
            // J5 near 0: push positive; J5 near +-180deg: push toward 0
            double j5 = q[4];
            // Normalize to [-180, 180] degrees
            while (j5 > 180.0) j5 -= 360.0;
            while (j5 < -180.0) j5 += 360.0;
            if (fabs(j5) < 17.0)  // 17 degrees from 0 = wrist near singular
                g[4] += Config::SINGAVOID_W_WRIST * (j5 > 0 ? -1.0 : 1.0);
            else if (fabs(fabs(j5) - 180.0) < 17.0)  // near ±180°
                g[4] += Config::SINGAVOID_W_WRIST * (fabs(j5) > 90.0 ? -1.0 : 1.0);
        }
    }
}

// ===== Internal: extract Euler angles (Rx,Ry,Rz in degrees) from 4x4 transform =====
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

// ===== Public interfaces =====

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

Vec3 dampOrientationMotion(const Vec3& targetOrient, const Vec3& deltaOrient,
                           const Vec3& currentTcp, const double currentJoints[6],
                           Vec3& tcpAdjustOut, Vec3& repulsionOut) {
    tcpAdjustOut = Vec3(0, 0, 0);
    repulsionOut = Vec3(0, 0, 0);

    // ===== Layer 1: Wrist singularity damping (Phase 2: continuous) =====
    double J_full[6][6];
    Kinematics::jacobian(currentJoints, J_full);

    // Extract wrist Jacobian: orientation rows x J4/J5/J6 columns
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

    if (cond_wrist >= Config::SINGAVOID_COND_WRIST_EARLY) {
        // Continuous damping: t = normalized proximity to block zone
        double t = (cond_wrist - Config::SINGAVOID_COND_WRIST_EARLY)
                 / (Config::SINGAVOID_COND_WRIST_BLOCK - Config::SINGAVOID_COND_WRIST_EARLY);
        if (t > 1.0) t = 1.0;
        double beta = 1.0 - t * t;  // quadratic: gentle onset, rapid near danger

        // v_min = column of V corresponding to sigma[2] (smallest singular value)
        // This is the most singular direction in J4/J5/J6 joint space
        double v_min[3] = {V[0][2], V[1][2], V[2][2]};

        // Map singular direction to task space: u_min = J_w * v_min (3D rotation vector)
        double u_min[3];
        u_min[0] = J_w[0][0]*v_min[0] + J_w[0][1]*v_min[1] + J_w[0][2]*v_min[2];
        u_min[1] = J_w[1][0]*v_min[0] + J_w[1][1]*v_min[1] + J_w[1][2]*v_min[2];
        u_min[2] = J_w[2][0]*v_min[0] + J_w[2][1]*v_min[1] + J_w[2][2]*v_min[2];

        // Normalize u_min
        double uMag = sqrt(u_min[0]*u_min[0] + u_min[1]*u_min[1] + u_min[2]*u_min[2]);
        if (uMag > 1e-12) {
            u_min[0] /= uMag; u_min[1] /= uMag; u_min[2] /= uMag;
        }

        // Project user delta onto v_min (joint-space singular direction)
        double proj = deltaOrient.x*v_min[0] + deltaOrient.y*v_min[1] + deltaOrient.z*v_min[2];
        double p_x = proj * v_min[0];
        double p_y = proj * v_min[1];
        double p_z = proj * v_min[2];

        // Damp parallel component, keep perpendicular
        dampedDelta.x = (deltaOrient.x - p_x) + beta * p_x;
        dampedDelta.y = (deltaOrient.y - p_y) + beta * p_y;
        dampedDelta.z = (deltaOrient.z - p_z) + beta * p_z;

        // Directional repulsion: oppose user's rotation into danger
        // Project user delta onto u_min (task-space direction of the singular rotation)
        double projU = deltaOrient.x*u_min[0] + deltaOrient.y*u_min[1] + deltaOrient.z*u_min[2];
        if (projU > 0.0) {
            // User is rotating toward danger — apply opposing force
            double speedFactor = (fabs(projU) < 5.0) ? fabs(projU) / 5.0 : 1.0;
            double repMag = (1.0 - beta) * Config::SINGAVOID_SINGULAR_FORCE_MAX_N * speedFactor;
            repulsionOut.x = -u_min[0] * repMag;
            repulsionOut.y = -u_min[1] * repMag;
            repulsionOut.z = -u_min[2] * repMag;
        }

        // Send warning
        if (beta < 0.9) {
            int level = (beta < 0.2) ? 2 : 1;
            int dampPct = (int)((1.0 - beta) * 100);
            char msg[128], sug[128];
            snprintf(msg, sizeof(msg), "腕部接近奇异位姿，旋转已阻尼%d%%", dampPct);
            snprintf(sug, sizeof(sug), "请减速绕此方向旋转，避免J5对正");
            sendWarning(level, "wrist", msg, sug, cond_wrist, beta * 100.0);
        }
    }

    // ===== Layer 2: Direct Cartesian TCP push for shoulder safety (Phase 3) =====
    // Bypass null-space projection — push TCP radially outward in XY plane.
    // This directly increases elbow r_xy, unlike the old joint-gradient approach
    // where ~80% of the gradient was filtered out by the orientation null-space projector.
    Vec3 positions[7];
    Kinematics::computeJointPositions(currentJoints, positions);
    double r_elbow = sqrt(positions[2].x*positions[2].x + positions[2].y*positions[2].y);

    if (r_elbow < Config::SINGAVOID_SHOULDER_SAFE_R) {  // 120mm
        // Radial outward direction (XY plane) — pushes elbow away from Z-axis
        double pushDirX = (r_elbow > 0.01) ? positions[2].x / r_elbow : 1.0;
        double pushDirY = (r_elbow > 0.01) ? positions[2].y / r_elbow : 0.0;

        double pushMag, forceMag;

        if (r_elbow >= Config::SINGAVOID_DUAL_SING_ELBOW_THR) {
            // Yellow zone [80, 120): gentle push + light Touch repulsion
            double t = (Config::SINGAVOID_SHOULDER_SAFE_R - r_elbow)
                     / (Config::SINGAVOID_SHOULDER_SAFE_R - Config::SINGAVOID_DUAL_SING_ELBOW_THR);
            pushMag  = t * 3.0;   // 0 → 3 mm/frame
            forceMag = t * 1.5;   // 0 → 1.5 N
        } else if (r_elbow >= Config::SINGAVOID_SHOULDER_CRITICAL_R) {
            // Orange zone [50, 80): active push + strong Touch repulsion
            double t = (Config::SINGAVOID_DUAL_SING_ELBOW_THR - r_elbow)
                     / (Config::SINGAVOID_DUAL_SING_ELBOW_THR - Config::SINGAVOID_SHOULDER_CRITICAL_R);
            pushMag  = 3.0 + t * 2.0;   // 3 → 5 mm/frame
            forceMag = 1.5 + t * 1.5;   // 1.5 → 3.0 N
        } else {
            // Red zone < 50mm: maximum intervention
            pushMag  = Config::SINGAVOID_MAX_POS_ADJUST;        // 5mm/frame
            forceMag = Config::SINGAVOID_SINGULAR_FORCE_MAX_N;  // 3.0 N
        }

        // Apply TCP push — direct Cartesian, no null-space projection
        tcpAdjustOut.x = pushDirX * pushMag;
        tcpAdjustOut.y = pushDirY * pushMag;
        tcpAdjustOut.z = 0.0;

        // Touch radial repulsion (adds to wrist repulsion from Layer 1)
        repulsionOut.x += pushDirX * forceMag;
        repulsionOut.y += pushDirY * forceMag;

        // Dual-singularity: amplify by 1.5× when both wrist and shoulder at risk
        if (cond_wrist > Config::SINGAVOID_DUAL_SING_COND_THR &&
            r_elbow < Config::SINGAVOID_DUAL_SING_ELBOW_THR) {
            tcpAdjustOut.x *= 1.5;
            tcpAdjustOut.y *= 1.5;
            repulsionOut.x *= 1.5;
            repulsionOut.y *= 1.5;
            // Re-cap TCP adjust
            double newMag = sqrt(tcpAdjustOut.x*tcpAdjustOut.x +
                                tcpAdjustOut.y*tcpAdjustOut.y);
            if (newMag > Config::SINGAVOID_MAX_POS_ADJUST) {
                double scale = Config::SINGAVOID_MAX_POS_ADJUST / newMag;
                tcpAdjustOut.x *= scale;
                tcpAdjustOut.y *= scale;
            }
            sendWarning(2, "dual_singular",
                "腕部+肩部双重奇异风险，推力已增强",
                "建议松开按钮2，先移动TCP远离底座，再旋转",
                cond_wrist, r_elbow);
        }

        // Warnings
        if (r_elbow < Config::SINGAVOID_SHOULDER_CRITICAL_R) {
            sendWarning(2, "shoulder",
                "TCP距Z轴过近，已最大力度外推",
                "建议松开按钮2，先移动TCP远离底座再旋转",
                r_elbow, pushMag);
        } else if (r_elbow < Config::SINGAVOID_DUAL_SING_ELBOW_THR) {
            sendWarning(1, "shoulder",
                "肘部接近Z轴，TCP正在主动外推",
                "请配合向外移动TCP，避免继续靠近底座",
                r_elbow, pushMag);
        }
    }

    return dampedDelta;
}

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
            double beta = (ratio * ratio) / (threshold * threshold);  // Phase 2: quadratic for smoother onset
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

    // ===== Shoulder safety for combined mode (Phase 3) =====
    // Same radial-outward Cartesian push as dampOrientationMotion Layer 2.
    // Without this, button 1+2 combined mode has no shoulder protection at all.
    {
        Vec3 positions[7];
        Kinematics::computeJointPositions(currentJoints, positions);
        double r_elbow = sqrt(positions[2].x*positions[2].x + positions[2].y*positions[2].y);

        if (r_elbow < Config::SINGAVOID_SHOULDER_SAFE_R) {
            double pushDirX = (r_elbow > 0.01) ? positions[2].x / r_elbow : 1.0;
            double pushDirY = (r_elbow > 0.01) ? positions[2].y / r_elbow : 0.0;

            double pushMag;
            if (r_elbow >= Config::SINGAVOID_DUAL_SING_ELBOW_THR) {
                double t = (Config::SINGAVOID_SHOULDER_SAFE_R - r_elbow)
                         / (Config::SINGAVOID_SHOULDER_SAFE_R - Config::SINGAVOID_DUAL_SING_ELBOW_THR);
                pushMag = t * 3.0;
            } else if (r_elbow >= Config::SINGAVOID_SHOULDER_CRITICAL_R) {
                double t = (Config::SINGAVOID_DUAL_SING_ELBOW_THR - r_elbow)
                         / (Config::SINGAVOID_DUAL_SING_ELBOW_THR - Config::SINGAVOID_SHOULDER_CRITICAL_R);
                pushMag = 3.0 + t * 2.0;
            } else {
                pushMag = Config::SINGAVOID_MAX_POS_ADJUST;
            }

            // Add outward push to position delta (counteracts inward movement)
            dampedDeltaPos.x += pushDirX * pushMag;
            dampedDeltaPos.y += pushDirY * pushMag;

            if (r_elbow < Config::SINGAVOID_SHOULDER_CRITICAL_R) {
                sendWarning(2, "shoulder",
                    "全控模式：TCP距Z轴过近，已外推",
                    "建议先拉远TCP再继续操作",
                    r_elbow, pushMag);
            }
        }
    }

} // dampFullCommand

} // namespace SingularityAvoidance
