#include "SingularityAvoidance.h"
#include "../robot/Kinematics.h"
// RelayCore.h NOT included — we define TEST_SINGAVOID to stub it out
// (included here for the non-test sendWarning path; TEST_SINGAVOID builds skip RelayCore linkage)
#include "../relay/RelayCore.h"
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
    double j3Mid = Config::SINGAVOID_ELBOW_MID_ANGLE * M_PI / 180.0;
    double j3Range = 155.0 * M_PI / 180.0;
    // Normalized distance from center: 0 at center, 1 at limit
    double j3Norm = (j3 - j3Mid) / j3Range;
    // Gradient: -2 * normalized * (1/j3Range) -- pushes toward center
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
            // Simplified: push J5 away from 0deg (+-180deg)
            // J5 near 0: push positive; J5 near +-180deg: push toward 0
            double j5 = q[4];
            while (j5 > M_PI) j5 -= 2*M_PI;
            while (j5 < -M_PI) j5 += 2*M_PI;
            if (fabs(j5) < 0.3)  // within ~17deg of 0
                g[4] += Config::SINGAVOID_W_WRIST * (j5 > 0 ? -1.0 : 1.0);
            else if (fabs(fabs(j5) - M_PI) < 0.3)  // near +-180deg
                g[4] += Config::SINGAVOID_W_WRIST * (fabs(j5) > M_PI/2 ? -1.0 : 1.0);
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

// ===== Public stub implementations =====
// Full implementations will be added in Tasks 4, 5, 6

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

} // namespace SingularityAvoidance
