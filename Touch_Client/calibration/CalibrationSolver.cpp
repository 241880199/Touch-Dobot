#include "CalibrationSolver.h"
#include <cmath>
#include <algorithm>
#include <cstring>

// ===== 3×3 矩阵工具 =====

static void mat3Identity(double M[9]) {
    M[0]=1; M[1]=0; M[2]=0;
    M[3]=0; M[4]=1; M[5]=0;
    M[6]=0; M[7]=0; M[8]=1;
}

static void mat3Mul(double C[9], const double A[9], const double B[9]) {
    // C = A * B (row-major)
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            C[r*3 + c] = A[r*3]*B[c] + A[r*3+1]*B[3+c] + A[r*3+2]*B[6+c];
        }
    }
}

static void mat3Transpose(double AT[9], const double A[9]) {
    AT[0]=A[0]; AT[1]=A[3]; AT[2]=A[6];
    AT[3]=A[1]; AT[4]=A[4]; AT[5]=A[7];
    AT[6]=A[2]; AT[7]=A[5]; AT[8]=A[8];
}

static double mat3Det(const double A[9]) {
    return A[0]*(A[4]*A[8] - A[5]*A[7])
         - A[1]*(A[3]*A[8] - A[5]*A[6])
         + A[2]*(A[3]*A[7] - A[4]*A[6]);
}

static void mat3VecMul(double out[3], const double M[9], const double v[3]) {
    out[0] = M[0]*v[0] + M[1]*v[1] + M[2]*v[2];
    out[1] = M[3]*v[0] + M[4]*v[1] + M[5]*v[2];
    out[2] = M[6]*v[0] + M[7]*v[1] + M[8]*v[2];
}

// ===== 3×3 Jacobi 特征值分解 (对称矩阵) =====
// 输入: S[9] — 3×3 对称矩阵 (row-major)
// 输出: eigenval[3] — 特征值 (降序), eigenvec[9] — 特征向量 (列主序, V[col*3+row])
// 返回: 迭代次数 (0 = 失败)

static int jacobiEigen3(const double S[9], double eigenval[3], double eigenvec[9]) {
    // 复制到工作矩阵
    double A[9];
    std::memcpy(A, S, 9 * sizeof(double));

    // V = I
    mat3Identity(eigenvec);

    const int MAX_SWEEPS = 50;
    const double TOL = 1e-15;

    for (int sweep = 0; sweep < MAX_SWEEPS; sweep++) {
        // 找到最大非对角元
        double maxOff = 0.0;
        int p = 0, q = 1;
        for (int i = 0; i < 3; i++) {
            for (int j = i + 1; j < 3; j++) {
                double val = fabs(A[i*3 + j]);
                if (val > maxOff) {
                    maxOff = val;
                    p = i; q = j;
                }
            }
        }

        if (maxOff < TOL) break;

        // Jacobi rotation for A[p][q]
        double app = A[p*3 + p];
        double aqq = A[q*3 + q];
        double apq = A[p*3 + q];

        double theta = 0.5 * atan2(2.0 * apq, aqq - app);
        double c = cos(theta);
        double s = sin(theta);

        // Apply rotation: A = J^T * A * J
        // Columns p and q
        for (int i = 0; i < 3; i++) {
            double aip = A[i*3 + p];
            double aiq = A[i*3 + q];
            A[i*3 + p] =  c * aip - s * aiq;
            A[i*3 + q] =  s * aip + c * aiq;
        }
        // Rows p and q
        for (int i = 0; i < 3; i++) {
            double api = A[p*3 + i];
            double aqi = A[q*3 + i];
            A[p*3 + i] =  c * api - s * aqi;
            A[q*3 + i] =  s * api + c * aqi;
        }

        // Accumulate V = V * J
        for (int i = 0; i < 3; i++) {
            double vip = eigenvec[i*3 + p];
            double viq = eigenvec[i*3 + q];
            eigenvec[i*3 + p] = c * vip - s * viq;
            eigenvec[i*3 + q] = s * vip + c * viq;
        }
    }

    // 提取特征值
    eigenval[0] = A[0];
    eigenval[1] = A[4];
    eigenval[2] = A[8];

    // 按降序排列
    // Bubble sort on eigenval, swap columns of eigenvec correspondingly
    int order[3] = {0, 1, 2};
    if (fabs(eigenval[order[1]]) > fabs(eigenval[order[0]])) std::swap(order[0], order[1]);
    if (fabs(eigenval[order[2]]) > fabs(eigenval[order[0]])) std::swap(order[0], order[2]);
    if (fabs(eigenval[order[2]]) > fabs(eigenval[order[1]])) std::swap(order[1], order[2]);

    double sortedVal[3] = {eigenval[order[0]], eigenval[order[1]], eigenval[order[2]]};
    double sortedVec[9];
    for (int i = 0; i < 3; i++) {
        int srcCol = order[i];
        sortedVec[0*3 + i] = eigenvec[0*3 + srcCol];
        sortedVec[1*3 + i] = eigenvec[1*3 + srcCol];
        sortedVec[2*3 + i] = eigenvec[2*3 + srcCol];
    }

    std::memcpy(eigenval, sortedVal, 3 * sizeof(double));
    std::memcpy(eigenvec, sortedVec, 9 * sizeof(double));

    return 1;
}

// ===== 3×3 SVD via Jacobi (H = U * S * V^T) =====
// H[9]: 输入矩阵 (row-major)
// U[9], S[3], V[9]: 输出
// 返回 true 成功

static bool svd3(const double H[9], double U[9], double S[3], double V[9]) {
    // M = H^T * H (3×3 对称)
    double M[9];
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            M[r*3 + c] = 0.0;
            for (int k = 0; k < 3; k++) {
                M[r*3 + c] += H[k*3 + r] * H[k*3 + c];  // H^T * H
            }
        }
    }

    // Jacobi eigen: M * v = lambda * v
    double eigenval[3];
    jacobiEigen3(M, eigenval, V);  // V columns are eigenvectors of M

    // 奇异值 = sqrt(max(0, lambda))
    S[0] = sqrt(std::max(0.0, eigenval[0]));
    S[1] = sqrt(std::max(0.0, eigenval[1]));
    S[2] = sqrt(std::max(0.0, eigenval[2]));

    // U 列 = H * V_col / sigma_i (处理零奇异值)
    for (int col = 0; col < 3; col++) {
        double Hv[3] = {0, 0, 0};
        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 3; c++) {
                Hv[r] += H[r*3 + c] * V[c*3 + col];
            }
        }

        if (S[col] > 1e-12) {
            double invS = 1.0 / S[col];
            U[0*3 + col] = Hv[0] * invS;
            U[1*3 + col] = Hv[1] * invS;
            U[2*3 + col] = Hv[2] * invS;
        } else {
            // 零奇异值: Gram-Schmidt 补全
            U[0*3 + col] = (col == 0) ? 1.0 : 0.0;
            U[1*3 + col] = (col == 1) ? 1.0 : 0.0;
            U[2*3 + col] = (col == 2) ? 1.0 : 0.0;
            // 正交化
            for (int j = 0; j < col; j++) {
                double dot = 0.0;
                for (int r = 0; r < 3; r++) dot += U[r*3 + col] * U[r*3 + j];
                for (int r = 0; r < 3; r++) U[r*3 + col] -= dot * U[r*3 + j];
            }
            double norm = 0.0;
            for (int r = 0; r < 3; r++) norm += U[r*3 + col] * U[r*3 + col];
            if (norm > 1e-12) {
                double invN = 1.0 / sqrt(norm);
                for (int r = 0; r < 3; r++) U[r*3 + col] *= invN;
            }
        }
    }

    return true;
}

// ===== Kabsch-Umeyama 算法 =====

KabschResult solveKabsch(const std::vector<std::pair<Vec3, Vec3>>& pairs) {
    KabschResult result;
    result.valid = false;
    result.rmsError = 0.0;
    mat3Identity(result.R);
    result.t[0] = result.t[1] = result.t[2] = 0.0;

    int N = (int)pairs.size();
    if (N < 3) return result;

    // 1. 计算质心
    double cA[3] = {0, 0, 0};  // Touch
    double cB[3] = {0, 0, 0};  // Robot
    for (int i = 0; i < N; i++) {
        cA[0] += pairs[i].first.x;
        cA[1] += pairs[i].first.y;
        cA[2] += pairs[i].first.z;
        cB[0] += pairs[i].second.x;
        cB[1] += pairs[i].second.y;
        cB[2] += pairs[i].second.z;
    }
    double invN = 1.0 / N;
    cA[0] *= invN; cA[1] *= invN; cA[2] *= invN;
    cB[0] *= invN; cB[1] *= invN; cB[2] *= invN;

    // 2. 交叉协方差矩阵 H = sum(centered_B * centered_A^T)
    double H[9] = {0};
    for (int i = 0; i < N; i++) {
        double a[3] = {
            pairs[i].first.x - cA[0],
            pairs[i].first.y - cA[1],
            pairs[i].first.z - cA[2]
        };
        double b[3] = {
            pairs[i].second.x - cB[0],
            pairs[i].second.y - cB[1],
            pairs[i].second.z - cB[2]
        };
        // H += b * a^T (outer product)
        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 3; c++) {
                H[r*3 + c] += b[r] * a[c];
            }
        }
    }

    // 3. SVD: H = U * S * V^T
    double U[9], S[3], V[9];
    if (!svd3(H, U, S, V)) return result;

    // 4. R = U * V^T  (因为 H = B*A^T 而非 A^T*B 的约定)
    double VT[9];
    mat3Transpose(VT, V);
    mat3Mul(result.R, U, VT);

    // 5. 处理反射 (det(R) < 0)
    if (mat3Det(result.R) < 0.0) {
        // 翻转 U 的最后一列 (对应标准 Kabsch 翻转 V 的最后一列)
        U[0*3 + 2] = -U[0*3 + 2];
        U[1*3 + 2] = -U[1*3 + 2];
        U[2*3 + 2] = -U[2*3 + 2];
        mat3Mul(result.R, U, VT);
    }

    // 6. t = cB - R * cA
    double RcA[3];
    mat3VecMul(RcA, result.R, cA);
    result.t[0] = cB[0] - RcA[0];
    result.t[1] = cB[1] - RcA[1];
    result.t[2] = cB[2] - RcA[2];

    // 7. RMS 残差
    double sumSq = 0.0;
    for (int i = 0; i < N; i++) {
        double src[3] = { pairs[i].first.x, pairs[i].first.y, pairs[i].first.z };
        double transformed[3];
        mat3VecMul(transformed, result.R, src);
        transformed[0] += result.t[0];
        transformed[1] += result.t[1];
        transformed[2] += result.t[2];

        double dx = transformed[0] - pairs[i].second.x;
        double dy = transformed[1] - pairs[i].second.y;
        double dz = transformed[2] - pairs[i].second.z;
        sumSq += dx*dx + dy*dy + dz*dz;
    }
    result.rmsError = sqrt(sumSq / N);
    result.valid = true;

    return result;
}
