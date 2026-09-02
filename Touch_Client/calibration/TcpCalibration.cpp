#include "TcpCalibration.h"
#include <cmath>

namespace TcpCalibration {
    void rpyToMatrix(double rx, double ry, double rz, double R[9]) {
        double crx = cos(rx), srx = sin(rx);
        double cry = cos(ry), sry = sin(ry);
        double crz = cos(rz), srz = sin(rz);
        // R = Rz * Ry * Rx (row-major)
        R[0] = crz*cry;
        R[1] = crz*sry*srx - srz*crx;
        R[2] = crz*sry*crx + srz*srx;
        R[3] = srz*cry;
        R[4] = srz*sry*srx + crz*crx;
        R[5] = srz*sry*crx - crz*srx;
        R[6] = -sry;
        R[7] = cry*srx;
        R[8] = cry*crx;
    }

    void apply(const double pose[6], const double toolOffset[3], double tipOut[3]) {
        double R[9];
        rpyToMatrix(pose[3], pose[4], pose[5], R);
        tipOut[0] = pose[0] + R[0]*toolOffset[0] + R[1]*toolOffset[1] + R[2]*toolOffset[2];
        tipOut[1] = pose[1] + R[3]*toolOffset[0] + R[4]*toolOffset[1] + R[5]*toolOffset[2];
        tipOut[2] = pose[2] + R[6]*toolOffset[0] + R[7]*toolOffset[1] + R[8]*toolOffset[2];
    }

    bool solve(const double poses[][6], int n, double offsetOut[3], double& rmsOut) {
        if (n < 3) return false;

        double R0[9];
        rpyToMatrix(poses[0][3], poses[0][4], poses[0][5], R0);

        double AtA[9] = {0};
        double Atb[3] = {0};

        for (int k = 1; k < n; k++) {
            double Rk[9];
            rpyToMatrix(poses[k][3], poses[k][4], poses[k][5], Rk);
            double dR[9];
            for (int i = 0; i < 9; i++) dR[i] = Rk[i] - R0[i];
            double dp[3] = {
                poses[0][0] - poses[k][0],
                poses[0][1] - poses[k][1],
                poses[0][2] - poses[k][2]
            };
            // A^T A += dR^T dR ; A^T b += dR^T dp
            for (int r = 0; r < 3; r++) {
                for (int c = 0; c < 3; c++) {
                    double s = 0;
                    for (int i = 0; i < 3; i++) s += dR[i*3 + r] * dR[i*3 + c];
                    AtA[r*3 + c] += s;
                }
                double s = 0;
                for (int i = 0; i < 3; i++) s += dR[i*3 + r] * dp[i];
                Atb[r] += s;
            }
        }

        // 3×3 Gauss 消元 (部分主元)
        double A[3][3] = {
            {AtA[0], AtA[1], AtA[2]},
            {AtA[3], AtA[4], AtA[5]},
            {AtA[6], AtA[7], AtA[8]}
        };
        double b[3] = {Atb[0], Atb[1], Atb[2]};

        for (int col = 0; col < 3; col++) {
            int piv = col;
            for (int r = col + 1; r < 3; r++)
                if (fabs(A[r][col]) > fabs(A[piv][col])) piv = r;
            if (fabs(A[piv][col]) < 1e-12) return false;
            if (piv != col) {
                for (int c = 0; c < 3; c++) { double t = A[col][c]; A[col][c] = A[piv][c]; A[piv][c] = t; }
                double t = b[col]; b[col] = b[piv]; b[piv] = t;
            }
            double d = A[col][col];
            for (int c = col; c < 3; c++) A[col][c] /= d;
            b[col] /= d;
            for (int r = 0; r < 3; r++) {
                if (r == col) continue;
                double f = A[r][col];
                for (int c = col; c < 3; c++) A[r][c] -= f * A[col][c];
                b[r] -= f * b[col];
            }
        }

        offsetOut[0] = b[0];
        offsetOut[1] = b[1];
        offsetOut[2] = b[2];

        double sumSq = 0;
        for (int k = 1; k < n; k++) {
            double Rk[9];
            rpyToMatrix(poses[k][3], poses[k][4], poses[k][5], Rk);
            for (int i = 0; i < 3; i++) {
                double rv = 0;
                for (int c = 0; c < 3; c++) rv += (Rk[i*3 + c] - R0[i*3 + c]) * offsetOut[c];
                double e = rv - (poses[0][i] - poses[k][i]);
                sumSq += e * e;
            }
        }
        rmsOut = sqrt(sumSq / (n - 1));
        return true;
    }
}
