#include "PayloadCalibration.h"
#include "../calibration/TcpCalibration.h"
#include "../config/Config.h"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdlib>

namespace PayloadCalibration {

    // ===== 生效状态 =====
    bool   enabled = false;
    double massKg = 0.0;
    double comMm[3] = {0.0, 0.0, 0.0};
    double rmsForceN = 0.0;
    double rmsMomentNm = 0.0;
    int    poses = 0;
    double comSignZ = 1.0;
    // 生效的传感器安装偏转角 (度)。种子 = Config (未标定/旧文件时的回退值)。
    double sensorYawDeg = Config::SENSOR_MOUNT_YAW_DEG;

    static const int    MIN_POSES = 3;   // 4 个未知量, 每个姿态贡献 6 个方程; 3 个起解

    // 重力在【传感器系】下的表示 —— 就一句话: 转给 TcpCalibration, 本文件不自己算。
    // 本文件从前自己算过一遍 Rᵀ·(0,0,G), 与 ForceCompensation::step 里的那一份并存过 ——
    // 两份约定一旦漂移 (历史上就是差了个转置), 求解器会【安静地解错】: 残差仍与真值相关,
    // 不会报错。约定只能有一份实现, 就在这里转出去。
    // psi 显式给出 (psiDeg) 而不是读模块状态: 扫描要遍历 721 个角, 反复 setSensorYawDeg
    // 既慢又会在中途失败时给运行时留下一个被污染的安装角。数学仍只有一份 ——
    // gravitySensorFrameAtYaw 与 gravitySensorFrame 共用同一个实体 (见 TcpCalibration.cpp)。
    static void gravityTool(const double pose[6], double psiDeg, double g[3]) {
        TcpCalibration::gravitySensorFrameAtYaw(pose, psiDeg, g);
    }

    // 构造姿态 k (相对姿态 0 差商后) 的 6 行方程。
    // 未知量 x = [Δm, Δp_x, Δp_y, Δp_z] (Δm: kg; Δp: kg·m)。
    // 差商同时消掉了传感器零偏 b_F / b_M, 所以方程右端不含常数项。
    // psiDeg: 构造 g_k 时用的安装偏转角 —— 扫描就是这个参数在动。
    static void buildRows(const double posesIn[][6], const double forces[][3],
                          const double moments[][3], const double g0[3], double psiDeg, int k,
                          double rows[6][4], double rhs[6])
    {
        double gk[3];
        gravityTool(posesIn[k], psiDeg, gk);
        double dg[3] = {gk[0] - g0[0], gk[1] - g0[1], gk[2] - g0[2]};   // m/s²

        double dF[3] = {forces[k][0] - forces[0][0],
                        forces[k][1] - forces[0][1],
                        forces[k][2] - forces[0][2]};
        double dM[3] = {moments[k][0] - moments[0][0],
                        moments[k][1] - moments[0][1],
                        moments[k][2] - moments[0][2]};

        // 力: dF[a] = Δm · dg[a]
        for (int a = 0; a < 3; a++) {
            for (int c = 0; c < 4; c++) rows[a][c] = 0.0;
            rows[a][0] = dg[a];
            rhs[a] = dF[a];
        }
        // 力矩: dM = Δp × dg
        rows[3][0] = 0.0; rows[3][1] = 0.0;    rows[3][2] = dg[2];  rows[3][3] = -dg[1];
        rows[4][0] = 0.0; rows[4][1] = -dg[2]; rows[4][2] = 0.0;    rows[4][3] = dg[0];
        rows[5][0] = 0.0; rows[5][1] = dg[1];  rows[5][2] = -dg[0]; rows[5][3] = 0.0;
        rhs[3] = dM[0]; rhs[4] = dM[1]; rhs[5] = dM[2];
    }

    // ===== ψ 扫描的范围与步长 =====
    // 90° 只是安装孔 45° 分度给出的种子; 三组独立实机数据都指向 ≈79–81.5°, 而 90° 与 80°
    // 之差正好跨过 0.30 N 的验收门限 (90° → 0.3192 N 拒 / ≈80° → 0.215 N 过) —— ψ 是
    // "能标不能猜"的量, 所以让数据自己找。
    // [-180, 180] 覆盖整个圆周; 0.5° 步长 = 721 次 4×4 解, 相对采集耗时可忽略。
    // ⚠ 【不钳位】: 最优落在区间边缘是信号 (数据/装夹/模型有问题), 不是该被抹掉的噪声。
    static const double PSI_MIN_DEG  = -180.0;
    static const double PSI_MAX_DEG  =  180.0;
    static const double PSI_STEP_DEG =    0.5;
    static const int    PSI_N_SCAN   = 721;   // (180 − (−180)) / 0.5 + 1

    // 在给定 ψ 下做一次完整的最小二乘。这是本文件【唯一的拟合主体】—— 扫描与最终解算都走
    // 它, 所以扫描在最小化的量就是 solve() 报出、门限判的那个量, 两条路径不可能各算各的。
    // 返回 false = 姿态退化 (4×4 主元过小, 与从前 solve() 的判据相同)。
    // 输出 x[4] = {Δm, Δp_x, Δp_y, Δp_z}; sumSqF / sumSqM = 两通道的 |A·x − b|²。
    static bool fitAtYaw(const double posesIn[][6], const double forces[][3],
                         const double moments[][3], int n, double psiDeg,
                         double x[4], double& sumSqF, double& sumSqM)
    {
        if (n < MIN_POSES) return false;

        double AtA[4][4] = {{0}};
        double Atb[4] = {0};

        double g0[3];
        gravityTool(posesIn[0], psiDeg, g0);

        double rows[6][4], rhs[6];
        for (int k = 1; k < n; k++) {
            buildRows(posesIn, forces, moments, g0, psiDeg, k, rows, rhs);
            for (int r = 0; r < 6; r++) {
                for (int a = 0; a < 4; a++) {
                    for (int b = 0; b < 4; b++) AtA[a][b] += rows[r][a] * rows[r][b];
                    Atb[a] += rows[r][a] * rhs[r];
                }
            }
        }

        // 4×4 Gauss 消元 (部分主元)
        double A[4][5];
        for (int r = 0; r < 4; r++) {
            for (int c = 0; c < 4; c++) A[r][c] = AtA[r][c];
            A[r][4] = Atb[r];
        }
        for (int col = 0; col < 4; col++) {
            int piv = col;
            for (int r = col + 1; r < 4; r++)
                if (fabs(A[r][col]) > fabs(A[piv][col])) piv = r;
            // 阈值按量级取: dg ~ 10 m/s², Δp 系数同量级; 太小说明姿态退化
            if (fabs(A[piv][col]) < 1e-6) return false;
            if (piv != col)
                for (int c = col; c < 5; c++) { double t = A[col][c]; A[col][c] = A[piv][c]; A[piv][c] = t; }
            double d = A[col][col];
            for (int c = col; c < 5; c++) A[col][c] /= d;
            for (int r = 0; r < 4; r++) {
                if (r == col) continue;
                double f = A[r][col];
                for (int c = col; c < 5; c++) A[r][c] -= f * A[col][c];
            }
        }
        for (int i = 0; i < 4; i++) x[i] = A[i][4];

        // ===== 拟合残差 |A·x − b| (不是数据本身的量级) =====
        sumSqF = 0.0;
        sumSqM = 0.0;
        for (int k = 1; k < n; k++) {
            buildRows(posesIn, forces, moments, g0, psiDeg, k, rows, rhs);
            for (int r = 0; r < 6; r++) {
                double pred = 0.0;
                for (int a = 0; a < 4; a++) pred += rows[r][a] * x[a];
                double e = pred - rhs[r];
                if (r < 3) sumSqF += e * e;
                else       sumSqM += e * e;
            }
        }
        return true;
    }

    bool solve(const double posesIn[][6], const double forces[][3], const double moments[][3],
               int n, double mCfg, const double comCfg[3], double signZ, Result& out)
    {
        if (n < MIN_POSES) return false;

        // ===== 1) 先扫 ψ: 让数据自己定传感器安装偏转角 =====
        double bestPsi = 0.0, bestSumSq = 0.0;
        bool haveFit = false;
        for (int s = 0; s < PSI_N_SCAN; s++) {
            const double psi = PSI_MIN_DEG + s * PSI_STEP_DEG;
            double xs[4], ssF = 0.0, ssM = 0.0;
            if (!fitAtYaw(posesIn, forces, moments, n, psi, xs, ssF, ssM)) continue;
            // 判据 = 拟合残差 |A·x − b|² 的【全部 6 行】(力 3 + 力矩 3) —— 与 solve() 报出、
            // 门限判的那个残差是同一个表达式的同一个量, 只是不按通道拆开看。
            // 【两个通道都要算进去】: 力矩方程同样含 ψ (dM = Δp × dg)。只看力通道会在
            // Δm ≈ 0 时退化 (力方程全是 0 → 每个 ψ 的残差都是 0 → argmin 由扫描起点决定),
            // 而那种情况下 ψ 恰好是由力矩通道定的。
            if (!haveFit || ssF + ssM < bestSumSq) {
                haveFit = true;
                bestSumSq = ssF + ssM;
                bestPsi = psi;
            }
        }
        if (!haveFit) return false;   // 每个 ψ 下都秩亏 → 退化的是姿态本身, 不是 ψ
        // 全零数据 (dF 与 dM 恒为 0: 当前配置与真值一模一样) → 每个 ψ 的残差都是 0, argmin
        // 没有意义。此时保持种子不动, 而不是报一个"由扫描起点决定"的角 —— 那个角会被当成
        // 标定结果持久化, 进而把运行时的重力模型整个转歪。
        if (bestSumSq <= 0.0) bestPsi = Config::SENSOR_MOUNT_YAW_DEG;
        out.sensorYawDeg = bestPsi;

        // 最优落在扫描边界 = 信号。照实报出 (不钳位), 另外说一句 —— 免得它被当成正常结果。
        if (fabs(bestPsi - PSI_MIN_DEG) < 1e-9 || fabs(bestPsi - PSI_MAX_DEG) < 1e-9) {
            fprintf(stderr, "[Payload] !! psi 最优落在扫描边界 %.1f deg — 未钳位, 但这是异常信号:"
                            " 多半是姿态覆盖不足 / 装夹松动 / 数据里有直线运动, 不是真装了 180°。\n",
                    bestPsi);
        }

        // ===== 2) 用最优 ψ 跑最终那一次拟合 (同一个 fitAtYaw, 不是另一条路径) =====
        double x[4], sumSqF = 0.0, sumSqM = 0.0;
        if (!fitAtYaw(posesIn, forces, moments, n, bestPsi, x, sumSqF, sumSqM)) return false;

        double dm = x[0];
        double dp[3] = {x[1], x[2], x[3]};   // kg·m

        // ===== 换算绝对值 =====
        // signZ 不进 buildRows, 所以两种符号的拟合残差【完全相同】—— 数据本身区分不了符号
        // (见 Result::cTrueZ)。它只影响这里折算出来的绝对质心 Z, 而绝对质心现在只是记录/
        // 显示用 (本地补偿用 dm/dp), 所以不必也不该在解算器里"选边"。
        double mTrue = mCfg + dm;
        if (!(mTrue > 0.01) || mTrue > 5.0) return false;   // 非物理

        double cTrueZ[2];
        for (int k = 0; k < 2; k++) {
            const double s = (k == 0) ? 1.0 : -1.0;
            const double pCfgZ = mCfg * (comCfg[2] * s) / 1000.0;
            cTrueZ[k] = (pCfgZ + dp[2]) / mTrue * 1000.0;
        }

        double cEff[3] = {comCfg[0], comCfg[1], comCfg[2] * signZ};
        double pCfg[3] = {mCfg * cEff[0] / 1000.0,
                          mCfg * cEff[1] / 1000.0,
                          mCfg * cEff[2] / 1000.0};         // kg·m
        double pTrue[3] = {pCfg[0] + dp[0], pCfg[1] + dp[1], pCfg[2] + dp[2]};

        out.dm = dm;
        out.massKg = mTrue;
        out.dp[0] = dp[0];
        out.dp[1] = dp[1];
        out.dp[2] = dp[2];
        out.cTrueZ[0] = cTrueZ[0];
        out.cTrueZ[1] = cTrueZ[1];
        for (int i = 0; i < 3; i++) {
            double cTrue = pTrue[i] / mTrue * 1000.0;       // 物理质心 (mm)
            // 按传入的约定折算: 使机械臂的 c_eff 恰好等于 cTrue。
            double cSend = (i == 2 && signZ != 0.0) ? cTrue / signZ : cTrue;
            // 量程只查这个真正会被报出/写盘的值 —— 从前是逐候考查的 (那时会下发), 现在
            // 只有一个值, 也就只该由它决定成败。
            if (fabs(cSend) > 500.0) return false;          // 超出 EnableRobot 的 ±500 量程
            out.comMm[i] = cSend;
            out.dc[i] = cSend - comCfg[i];
        }
        // ===== 拟合残差 |A·x − b| (不是数据本身的量级) =====
        // 逐分量均方根: 每姿态 3 个力方程、3 个力矩方程 → 分母各 3·(n−1)。
        // 这两个值就是扫描在上一步最小化的同一个量的开方 (见 fitAtYaw)。
        const double eqF = 3.0 * (n - 1);
        out.rmsForceN   = sqrt(sumSqF / eqF);
        out.rmsMomentNm = sqrt(sumSqM / eqF);
        out.poses = n;
        return true;
    }

    // =================================================================================
    // 原始力通道的线性模型 (2026-09-19 重做; 取代上面的 psi 扫描)
    //
    //   F_i = b_F + A · g_i            A: 3×3, 9 个元素全自由
    //   M_i = b_M + c_s × (A · g_i)    c_s: 3, 质心 (m, 传感器测量系)
    //   g_i = gravitySensorFrameAtYaw(pose_i, 0.0, g)      <- 【psi = 0, 没有 psi】
    //
    // 力通道 12 个未知 (b_F 3 + A 9), 每个姿态 3 个方程; 力矩通道 6 个 (b_M 3 + c_s 3),
    // 给定 A 之后仍线性。两段各自一次最小二乘, 不扫描、不预设。
    // =================================================================================
    static const int    RAW_MIN_POSES   = 4;      // 12 个未知量 -> 至少要 4 个姿态 (3n >= 12)
    static const double RAW_MAX_COND    = 1.0e3;  // 力通道设计矩阵的条件数上限 (姿态激发)
    static const double RAW_SINGULAR_REL= 1.0e12; // cond 超过它 = 数值上奇异, 线性层直接拒
    static const double RAW_MASS_MIN_KG = 0.05;   // 量程下限: 工具链不可能轻于此
    static const double RAW_MASS_MAX_KG = 3.0;    // 量程上限: CR3 额定负载 3 kg
                                                  // (Docs/机械臂资料/Dobot CR3机械臂参数文档.md)
    // χ² 式模型形式检验的置信倍数, 推导见 fitRaw 的自检 3。它【不是】阈值本身 ——
    // 阈值 = 1 + K·sqrt(2/dof) 是【这个统计量自己的分布】给出的, 尺度完全来自实测噪声。
    static const double RAW_CHI2_SIGMA_K = 3.0;

    // 重力在【传感器系】的表示, psi 恒为 0 —— 新模型里 A 吸收了一切, 不存在安装角。
    // 【这里必须走 AtYaw 而不是 gravitySensorFrame】: 后者读 TcpCalibration 的模块状态,
    // 那样拟合结果就会随一个与数据无关的全局变量变 —— 那正是 psi 时代的问题。
    static void gravityNoYaw(const double pose[6], double g[3]) {
        TcpCalibration::gravitySensorFrameAtYaw(pose, 0.0, g);
    }

    // 对称矩阵的 Jacobi 特征分解 (n×n, row-major; M 被就地破坏)。
    // 只用得到特征值, 所以不累积特征向量。n ≤ 12, 几十次扫描的代价可以忽略。
    // 用途: A 的奇异值 (AᵀA 的特征值开方) 与 cond(J) (JᵀJ 的特征值之比开方)。
    static void jacobiEigenSym(double* M, int n, double* evalOut) {
        for (int sweep = 0; sweep < 60; sweep++) {
            double off = 0.0, diag2 = 0.0;
            for (int i = 0; i < n; i++) {
                diag2 += M[i * n + i] * M[i * n + i];
                for (int j = i + 1; j < n; j++) off += M[i * n + j] * M[i * n + j];
            }
            if (off <= 1e-28 * (diag2 + 1e-300)) break;      // 相对判据: 尺度和量级无关
            for (int p = 0; p < n; p++) {
                for (int q = p + 1; q < n; q++) {
                    const double apq = M[p * n + q];
                    if (fabs(apq) <= 1e-300) continue;
                    const double theta = (M[q * n + q] - M[p * n + p]) / (2.0 * apq);
                    const double sgn = (theta >= 0.0) ? 1.0 : -1.0;
                    const double t = sgn / (fabs(theta) + sqrt(theta * theta + 1.0));
                    const double c = 1.0 / sqrt(t * t + 1.0), s = t * c;
                    for (int k = 0; k < n; k++) {            // 列变换
                        const double mkp = M[k * n + p], mkq = M[k * n + q];
                        M[k * n + p] = c * mkp - s * mkq;
                        M[k * n + q] = s * mkp + c * mkq;
                    }
                    for (int k = 0; k < n; k++) {            // 行变换
                        const double mpk = M[p * n + k], mqk = M[q * n + k];
                        M[p * n + k] = c * mpk - s * mqk;
                        M[q * n + k] = s * mpk + c * mqk;
                    }
                    M[p * n + q] = 0.0;                      // 消干净, 免得留下舍入残渣
                    M[q * n + p] = 0.0;
                }
            }
        }
        for (int i = 0; i < n; i++) evalOut[i] = M[i * n + i];
    }

    // Gauss-Jordan 求逆 + 解方程: 输入 AtA (p×p, 对称正定) 与 Atb (p), 输出 x = (AtA)⁻¹·Atb
    // 与 C = (AtA)⁻¹ (paramSigma 要的就是它的对角)。返回 false = 主元塌了 (秩亏)。
    // 阈值取【相对的】(对 AtA 的最大对角元): AtA 的量级随姿态数/重力模长变 (几十~几千),
    // 绝对阈值在两种尺度下不可能同时对。
    static bool solveNormal(const double* AtA, const double* Atb, int p, double* x, double* C) {
        double scale = 0.0;
        for (int i = 0; i < p; i++) if (AtA[i * p + i] > scale) scale = AtA[i * p + i];
        if (!(scale > 0.0)) return false;

        double M[12][24];        // p <= 12
        for (int r = 0; r < p; r++) {
            for (int c = 0; c < p; c++) M[r][c] = AtA[r * p + c];
            for (int c = 0; c < p; c++) M[r][p + c] = (r == c) ? 1.0 : 0.0;
        }
        for (int col = 0; col < p; col++) {
            int piv = col;
            for (int r = col + 1; r < p; r++)
                if (fabs(M[r][col]) > fabs(M[piv][col])) piv = r;
            if (!(fabs(M[piv][col]) > 1e-10 * scale)) return false;
            if (piv != col)
                for (int c = 0; c < 2 * p; c++) {
                    const double t = M[col][c]; M[col][c] = M[piv][c]; M[piv][c] = t;
                }
            const double d = M[col][col];
            for (int c = 0; c < 2 * p; c++) M[col][c] /= d;
            for (int r = 0; r < p; r++) {
                if (r == col) continue;
                const double f = M[r][col];
                if (f == 0.0) continue;
                for (int c = 0; c < 2 * p; c++) M[r][c] -= f * M[col][c];
            }
        }
        for (int r = 0; r < p; r++) {
            for (int c = 0; c < p; c++) C[r * p + c] = M[r][p + c];
            double acc = 0.0;
            for (int c = 0; c < p; c++) acc += M[r][p + c] * Atb[c];
            x[r] = acc;
        }
        return true;
    }

    // 对称矩阵的 min/max 特征值 (只给 cond 用; 矩阵被就地破坏)
    static bool symExtremes(double* M, int n, double& lo, double& hi) {
        double ev[12];
        jacobiEigenSym(M, n, ev);
        lo = ev[0]; hi = ev[0];
        for (int i = 1; i < n; i++) {
            if (ev[i] < lo) lo = ev[i];
            if (ev[i] > hi) hi = ev[i];
        }
        return lo > 0.0;
    }

    // 该姿态该通道【均值】的 1σ² : σ² = var / N。方差来自采集时同一批样本的样本方差,
    // 与模型、与重力约定都无关 —— 它就是"这个输入值本身有多准"。
    static double meanVar(const PoseNoise& nz, int a, bool moment) {
        return (moment ? nz.varM[a] : nz.varF[a]) / (double)nz.n;
    }

    // 整批噪声估计是否可用。任一姿态的方差 ≤ 0 (通道没采到 / 读数冻住 / 没填) 或样本数
    // < 1 就判【整批不可用】—— 模型形式检验是拿这把尺子去量残差的, 半批坏尺子会让判据
    // 忽紧忽松。宁可明确地说"没做检验"(modelFormChecked = false), 也不拿它判生死。
    // 注意 N = 1 是【合法】的: 它表示"没有做平均, 但输入值的方差是已知的" (调用方从别处
    // 知道), 此时 σ_mean = sqrt(var/1)。生产路径上 N 是采集窗口里的样本数 (通常 ~30)。
    static bool poseNoiseUsable(const PoseNoise* noise, int n) {
        if (!noise) return false;
        for (int i = 0; i < n; i++) {
            if (noise[i].n < 1) return false;
            for (int a = 0; a < 3; a++)
                if (!(noise[i].varF[a] > 0.0) || !(noise[i].varM[a] > 0.0)) return false;
        }
        return true;
    }

    // 自由力矩模型 M = b_M + N·w (N: 3×3 全自由, 12 参数) —— 【只用于失拟比较】, 不产出参数。
    // 受约束的参数模型是它的【子模型】: c_s × w = [c_s]ₓ·w 是一个反对称阵, 所以
    // "自由模型能解释而受约束模型解释不了"的那一份, 就是叉乘结构本身不成立的部分。
    // 自由度 = 12 − 6 = 6。
    // 回归量 w = A·g 用【已经解出的 A】算 —— 与受约束那一份用的是同一个 w, 这样两边的残差
    // 才可比 (A 的误差同等地进两边, 在差值里大部分相消; 若用真值 w 反而不可比)。
    // 返回 false = 方程数不足以养自由模型 (3n ≤ 12, 自由度 0), 或设计矩阵秩亏/数值奇异。
    static bool fitMomentFree(const double posesIn[][6], const double A[9],
                              const double moments[][3], int n, double& ssFree)
    {
        const int PMF = 12;
        if (3 * n <= PMF) return false;

        double AtA[144] = {0}, Atb[12] = {0};
        for (int i = 0; i < n; i++) {
            double g[3];
            gravityNoYaw(posesIn[i], g);
            double w[3];
            for (int a = 0; a < 3; a++)
                w[a] = A[a * 3 + 0] * g[0] + A[a * 3 + 1] * g[1] + A[a * 3 + 2] * g[2];
            for (int a = 0; a < 3; a++) {
                double row[12] = {0};
                row[a] = 1.0;
                for (int c = 0; c < 3; c++) row[3 + 3 * a + c] = w[c];
                for (int c = 0; c < PMF; c++) {
                    for (int e = 0; e < PMF; e++) AtA[c * PMF + e] += row[c] * row[e];
                    Atb[c] += row[c] * moments[i][a];
                }
            }
        }
        double AtAcopy[144];
        for (int i = 0; i < PMF * PMF; i++) AtAcopy[i] = AtA[i];
        double lo = 0.0, hi = 0.0;
        if (!symExtremes(AtAcopy, PMF, lo, hi)) return false;
        if (!(sqrt(hi / lo) < RAW_SINGULAR_REL)) return false;

        double x[12], C[144];
        if (!solveNormal(AtA, Atb, PMF, x, C)) return false;

        ssFree = 0.0;
        for (int i = 0; i < n; i++) {
            double g[3];
            gravityNoYaw(posesIn[i], g);
            double w[3];
            for (int a = 0; a < 3; a++)
                w[a] = A[a * 3 + 0] * g[0] + A[a * 3 + 1] * g[1] + A[a * 3 + 2] * g[2];
            for (int a = 0; a < 3; a++) {
                double pred = x[a];
                for (int c = 0; c < 3; c++) pred += x[3 + 3 * a + c] * w[c];
                const double e = pred - moments[i][a];
                ssFree += e * e;
            }
        }
        return true;
    }

    bool fitRawLinear(const double posesIn[][6], const double forces[][3],
                      const double moments[][3], int n, RawFit& out, const PoseNoise* noiseIn)
    {
        if (n < RAW_MIN_POSES) return false;

        // 噪声估计可用才进入检验; 不可用一律当"没有" —— 字段留 0, modelFormChecked = false。
        const PoseNoise* noise = poseNoiseUsable(noiseIn, n) ? noiseIn : nullptr;
        for (int i = 0; i < 18; i++) out.paramSigma[i] = 0.0;
        out.massScale = 0.0; out.parity = 0.0; out.isotropyRatio = 0.0;
        out.noiseForceN = 0.0; out.noiseMomentNm = 0.0;
        out.chi2ForceRatio = 0.0; out.chi2DofForce = 0;
        out.chi2MomentRatio = 0.0; out.chi2DofMoment = 0;
        out.lackOfFitMomentRatio = 0.0; out.lackOfFitMomentDof = 0;
        out.modelFormChecked = false;

        // ===== 力通道: x = [b_F(3), A(9)] (A row-major) =====
        const int PF = 12;
        double AtA[144] = {0}, Atb[12] = {0};
        for (int i = 0; i < n; i++) {
            double g[3];
            gravityNoYaw(posesIn[i], g);
            for (int a = 0; a < 3; a++) {
                double row[12] = {0};
                row[a] = 1.0;
                for (int c = 0; c < 3; c++) row[3 + 3 * a + c] = g[c];
                for (int c = 0; c < PF; c++) {
                    for (int e = 0; e < PF; e++) AtA[c * PF + e] += row[c] * row[e];
                    Atb[c] += row[c] * forces[i][a];
                }
            }
        }
        // cond(J) = sqrt(λmax/λmin) of AtA = JᵀJ —— 奇异值之比与是否正交无关, 直接可读。
        double AtAcopy[144];
        for (int i = 0; i < PF * PF; i++) AtAcopy[i] = AtA[i];
        double lamMin = 0.0, lamMax = 0.0;
        if (!symExtremes(AtAcopy, PF, lamMin, lamMax)) return false;   // λ ≤ 0 -> 秩亏
        const double condJ = sqrt(lamMax / lamMin);
        if (!(condJ < RAW_SINGULAR_REL)) return false;                 // 数值上奇异

        double xF[12], CF[144];
        if (!solveNormal(AtA, Atb, PF, xF, CF)) return false;

        double bF[3] = {xF[0], xF[1], xF[2]};
        double A[9];
        for (int i = 0; i < 9; i++) A[i] = xF[3 + i];

        // 力通道残差与参数不确定度。
        // 同一个循环里顺手累加 χ² = Σ(e/σ)² (σ = 该姿态该通道【均值】的实测 1σ) ——
        // 这是模型形式检验的统计量, 判据在 fitRaw。ssF 的累加【顺序与式子和从前完全一样】,
        // 所以 rmsForceN / paramSigma 一个比特都不会变。
        double ssF = 0.0, chi2F = 0.0, sig2SumF = 0.0;
        for (int i = 0; i < n; i++) {
            double g[3];
            gravityNoYaw(posesIn[i], g);
            for (int a = 0; a < 3; a++) {
                double pred = bF[a];
                for (int c = 0; c < 3; c++) pred += A[a * 3 + c] * g[c];
                const double e = pred - forces[i][a];
                ssF += e * e;
                if (noise) {
                    const double s2 = meanVar(noise[i], a, false);
                    chi2F += e * e / s2;
                    sig2SumF += s2;
                }
            }
        }
        const int dofF = 3 * n - PF;
        // 零自由度 (n = 4) 时估不出噪声方差 —— 报 0, 自检的门限随之退化成"必须严格正交"。
        const double s2F = (dofF > 0) ? ssF / (double)dofF : 0.0;

        // ===== 力矩通道: 给定 A 后 y = [b_M(3), c_s(3)], 每个姿态 3 个方程 =====
        const int PM = 6;
        double MtM[36] = {0}, Mtb[6] = {0};
        for (int i = 0; i < n; i++) {
            double g[3];
            gravityNoYaw(posesIn[i], g);
            double w[3];                                  // w = A·g (传感器系的重力响应)
            for (int a = 0; a < 3; a++)
                w[a] = A[a * 3 + 0] * g[0] + A[a * 3 + 1] * g[1] + A[a * 3 + 2] * g[2];
            for (int a = 0; a < 3; a++) {
                double row[6] = {0};
                row[a] = 1.0;
                // (c_s × w) 的第 a 个分量 —— 叉乘结构写成分量, 不是独立的 3×3
                if (a == 0) { row[3 + 1] =  w[2]; row[3 + 2] = -w[1]; }
                if (a == 1) { row[3 + 2] =  w[0]; row[3 + 0] = -w[2]; }
                if (a == 2) { row[3 + 0] =  w[1]; row[3 + 1] = -w[0]; }
                for (int c = 0; c < PM; c++) {
                    for (int e = 0; e < PM; e++) MtM[c * PM + e] += row[c] * row[e];
                    Mtb[c] += row[c] * moments[i][a];
                }
            }
        }
        {   // 力矩通道同样要能判秩亏 (给定 A 后设计矩阵只由姿态与 A 定)
            double MtMcopy[36];
            for (int i = 0; i < PM * PM; i++) MtMcopy[i] = MtM[i];
            double lo = 0.0, hi = 0.0;
            if (!symExtremes(MtMcopy, PM, lo, hi)) return false;
            if (!(sqrt(hi / lo) < RAW_SINGULAR_REL)) return false;
        }
        double xM[6], CM[36];
        if (!solveNormal(MtM, Mtb, PM, xM, CM)) return false;

        double bM[3] = {xM[0], xM[1], xM[2]};
        double cS[3] = {xM[3], xM[4], xM[5]};

        double ssM = 0.0, chi2M = 0.0, sig2SumM = 0.0;
        for (int i = 0; i < n; i++) {
            double g[3];
            gravityNoYaw(posesIn[i], g);
            double w[3];
            for (int a = 0; a < 3; a++)
                w[a] = A[a * 3 + 0] * g[0] + A[a * 3 + 1] * g[1] + A[a * 3 + 2] * g[2];
            const double pred[3] = { bM[0] + cS[1] * w[2] - cS[2] * w[1],
                                     bM[1] + cS[2] * w[0] - cS[0] * w[2],
                                     bM[2] + cS[0] * w[1] - cS[1] * w[0] };
            for (int a = 0; a < 3; a++) {
                const double e = pred[a] - moments[i][a];
                ssM += e * e;
                if (noise) {
                    const double s2 = meanVar(noise[i], a, true);
                    chi2M += e * e / s2;
                    sig2SumM += s2;
                }
            }
        }
        const int dofM = 3 * n - PM;
        const double s2M = (dofM > 0) ? ssM / (double)dofM : 0.0;

        // 力矩通道的【失拟】: 自由 12 参数模型与受约束 6 参数模型的残差之差, 按实测噪声折算。
        // 为什么力矩问的是失拟而不是"残差 vs 噪声": 见 fitRaw 的自检 3 与 RawFit 的说明。
        double ssFreeM = 0.0;
        const bool haveFreeM = noise && fitMomentFree(posesIn, A, moments, n, ssFreeM);

        // ===== 汇总 =====
        for (int i = 0; i < 9; i++) out.A[i] = A[i];
        for (int i = 0; i < 3; i++) { out.bF[i] = bF[i]; out.cS[i] = cS[i]; out.bM[i] = bM[i]; }
        out.rmsForceN   = sqrt(ssF / (3.0 * n));
        out.rmsMomentNm = sqrt(ssM / (3.0 * n));
        out.cond        = condJ;
        // 布局与 RawFit 的字段同序: [0..8] = A, [9..11] = bF, [12..14] = cS, [15..17] = bM
        for (int i = 0; i < 9; i++) out.paramSigma[i] = sqrt(CF[(3 + i) * PF + (3 + i)] * s2F);
        for (int i = 0; i < 3; i++) out.paramSigma[9 + i]  = sqrt(CF[i * PF + i] * s2F);
        for (int i = 0; i < 3; i++) out.paramSigma[12 + i] = sqrt(CM[(3 + i) * PM + (3 + i)] * s2M);
        for (int i = 0; i < 3; i++) out.paramSigma[15 + i] = sqrt(CM[i * PM + i] * s2M);

        // ===== 检验用的报告字段 (只在不传 noise 时为 0) =====
        if (noise) {
            // 实测噪声的合并尺度: 各方程 σ² 的均值再开方 (与 rms 同量纲, 好直接对比)。
            out.noiseForceN   = sqrt(sig2SumF / (3.0 * n));
            out.noiseMomentNm = sqrt(sig2SumM / (3.0 * n));
            out.chi2ForceRatio = (dofF > 0) ? chi2F / (double)dofF : 0.0;
            out.chi2DofForce   = dofF;
            out.chi2MomentRatio = (dofM > 0) ? chi2M / (double)dofM : 0.0;
            out.chi2DofMoment   = dofM;
            if (haveFreeM) {
                // ⚠ 方向: 自由模型【包含】受约束模型 (c_s × w = [c_s]ₓ·w 是 N 的一个特例),
                // 所以恒有 ssFree ≤ ssM。"多出来的 6 个参数少解释的那一份"是 ssM − ssFree:
                // 受约束模型对得上时它只是噪声, 期望 6σ_M²; 叉乘结构不成立时它远大于此。
                // 自由度 6 = 自由 12 参数 − 受约束 6 参数。按【合并的】噪声尺度折算
                // (逐姿态 σ 略有差异时这是个近似: 差值统计量对权重的敏感度是二阶的)。
                const double diff = (ssM > ssFreeM) ? (ssM - ssFreeM) : 0.0;   // 舍入可致微负
                const double sig2Bar = sig2SumM / (3.0 * n);
                out.lackOfFitMomentRatio = diff / sig2Bar / 6.0;
                out.lackOfFitMomentDof   = 6;
            }
            // 3n > 12 (即 n ≥ 5) 才有残差自由度可言; n = 4 时自由度 0, 检验【做不了】。
            out.modelFormChecked = (dofF > 0);
        }
        return true;
    }

    bool decompose(const double Ain[9], Decomp& out) {
        // AᵀA 的特征值开方 = A 的奇异值 (3×3 对称, Jacobi 足够且稳)
        double M[9], ev[3];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) {
                double acc = 0.0;
                for (int k = 0; k < 3; k++) acc += Ain[k * 3 + i] * Ain[k * 3 + j];
                M[i * 3 + j] = acc;
            }
        jacobiEigenSym(M, 3, ev);
        double sv[3] = {sqrt(fabs(ev[0])), sqrt(fabs(ev[1])), sqrt(fabs(ev[2]))};
        for (int a = 0; a < 2; a++)                       // 降序 (σ1 >= σ2 >= σ3)
            for (int b = a + 1; b < 3; b++)
                if (sv[b] > sv[a]) { const double t = sv[a]; sv[a] = sv[b]; sv[b] = t; }

        const double detA = Ain[0] * (Ain[4] * Ain[8] - Ain[5] * Ain[7])
                          - Ain[1] * (Ain[3] * Ain[8] - Ain[5] * Ain[6])
                          + Ain[2] * (Ain[3] * Ain[7] - Ain[4] * Ain[6]);
        // 奇异 A: 定不出手系也定不出尺度 (σ3/σ1 太小 = 数值上秩亏)
        if (!(sv[0] > 0.0) || !(sv[2] > 1e-12 * sv[0])) return false;

        // m = (σ1σ2σ3)^(1/3) (> 0) ; parity = sign(det A) ; S = diag(1,1,parity) ; Q = S·A/m
        // 于是 A = m·S·Q 恒【精确】成立 (Q 的正交性不在恒等式里, 它只在 A 恰好是正交尺度阵时
        // 才正交 —— 那正是自检在查的事)。
        const double m = pow(sv[0] * sv[1] * sv[2], 1.0 / 3.0);
        const double parity = (detA < 0.0) ? -1.0 : 1.0;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                out.Q[i * 3 + j] = ((i == 2) ? parity : 1.0) * Ain[i * 3 + j] / m;
        out.m = m;
        out.parity = parity;
        for (int i = 0; i < 3; i++) out.sv[i] = sv[i];
        out.isotropyRatio = sv[0] / sv[2];
        return true;
    }

    bool fitRaw(const double posesIn[][6], const double forces[][3], const double moments[][3],
                int n, RawFit& out, const PoseNoise* noise)
    {
        if (!fitRawLinear(posesIn, forces, moments, n, out, noise)) return false;
        const bool noiseUsable = poseNoiseUsable(noise, n);

        Decomp d;
        if (!decompose(out.A, d)) {
            fprintf(stderr, "[Payload] 自检拒绝: A 奇异 (σ3/σ1 过小), 定不出安装姿态与质量尺度。\n");
            return false;
        }
        out.massScale = d.m;
        out.parity = d.parity;
        out.isotropyRatio = d.isotropyRatio;

        // ---- 自检 1: 条件数 (12 个参数定不定得下来) ----
        // 姿态激发不足 -> A 的每个分量都在大误差里, 后面两条判据也就没了意义, 所以先判它。
        if (!(out.cond < RAW_MAX_COND)) {
            fprintf(stderr, "[Payload] 自检拒绝: 力通道 cond=%.3g >= %.3g —— 姿态激发不足,"
                            " 12 个参数定不下来 (姿态要够散, 不能只在小角度里晃)。\n",
                    out.cond, RAW_MAX_COND);
            return false;
        }

        // ---- 自检 2: 质量尺度必须落在 EnableRobot 的负载量程里 ----
        // CR3 额定 3 kg (Docs/机械臂资料/Dobot CR3机械臂参数文档.md §最大负载);
        // 下限 0.05 kg: 工具链(传感器+笔夹+笔)不可能轻于此, 解到更小说明解出的不是工具重量。
        if (!(d.m > RAW_MASS_MIN_KG) || d.m > RAW_MASS_MAX_KG) {
            fprintf(stderr, "[Payload] 自检拒绝: 质量尺度 m=%.4g kg 超出量程 (%.2f, %.2f] kg"
                            " —— 这不是工具链的重量。\n",
                    d.m, RAW_MASS_MIN_KG, RAW_MASS_MAX_KG);
            return false;
        }

        // ---- 自检 3: 模型形式 —— 残差 vs 【实测】噪声, χ² 式 (取代从前的各向同性门限) ----
        //
        // 【为什么各向同性比不再是判据】
        // 力通道的模型是 F = b_F + A·g, A 的 9 个元素【全部自由】—— 数据对它没有任何约束。
        // 于是 σ1/σ3 > 1 只能说明"传感器的实际响应不是各向同性的", 这是对传感器本身的
        // 【测量结果】, 不是"模型形式与数据不符"的证据。实机那批数据里 A 本就 6.5% 非正交
        // (isotropyRatio = 1.06546, 物理属性)。把它当门限有两个方向都错的后果:
        //   · 反方向 (错杀): 用拟合残差估出来的 σ 去定门限, 而残差里混着"形式错"那一份 ——
        //     无噪声重放真实数据时 σ → 0, 门限压到 1.0, 【正确的解被拒】;
        //   · 正方向 (放行错解): 形式错 → 残差涨 → σ 涨 → 门限放松得比 iso 涨得更快。
        //     评审实跑的反例: 真值完全各向同性 (A = 0.42·diag(1,1,−1)·Rz(30°)), 数据用
        //     【历史上的转置回归量】生成 (就是那个让实机安静地解错两次的 bug) + 0.02 N 噪声 →
        //     拟合 iso = 2.01, 门限 1+3σ/m = 2.59 → 【接受】, 返回 m = 0.4159 (真值 0.42)。
        //     门对着自己放水, 正解错解一起放。
        // 所以 isotropyRatio 现在【只报告】(out.isotropyRatio / 下面那行打印), 不进判决。
        //
        // 【判据改用什么】拟合残差 vs 【与模型无关】的噪声估计: 每个姿态是 ~1 s (~30 个样本)
        // 的平均, 样本方差量的是测量噪声本身 —— 它不经过模型、不经过 A、不经过重力约定。
        // 该姿态均值的不确定度 σ = sqrt(var/N) (标准误), 于是
        //     χ² = Σ_i Σ_a (pred_ia − F_ia)² / σ²_ia ,  dof = 方程数 − 参数数 (= 3n − 12)
        // 模型形式对时 χ²/dof 的期望是 1、标准差 √(2/dof) (χ² 分布的一阶矩与二阶矩), 门限就取
        // 它自己的分布:
        //     拒绝 ⇔ χ²/dof > 1 + K·sqrt(2/dof) ,  K = 3 (3σ 置信水平)
        // 【没有绝对 N 阈值】: 判据的尺度全部来自实测的 var/N —— 传感器吵, 门限按比例放宽;
        // 传感器安静, 门限按比例收紧。K 只是"我愿意认几倍标准差", 量纲与量级都由数据给。
        // 实机数据上 rmsForceN 与这把尺子的关系会照实打印出来 (含两者之比), 供人核对。
        //
        // 力矩通道【另论】: 它确实是受约束的 (c_s × (A·g), 3 参数 vs 自由的 9), 但它的回归量
        // w = A·g 里带着 A 的估计误差, 所以"力矩残差 vs 噪声"不是纯噪声统计量 (会系统性偏大),
        // 拿它当判据会错杀。力矩问的是【失拟】: 自由模型 (b_M + N·w, 12 参数) 与受约束模型
        // (b_M + c_s × w, 6 参数) 的残差之差, 按同一个实测噪声折算, 自由度 6。
        // 两边都用同一个 w, A 的误差在差值里大部分相消。
        if (!noiseUsable) {
            // 【说清楚"没做"与"通过"的区别】: 没有逐姿态噪声估计时, 残差是唯一能看的量,
            // 而它自己的尺度里就有模型形式错那一份 —— 拿它判等于自指, 所以这里【不判】。
            fprintf(stderr, "[Payload] 注意: 未提供(或不可用)逐姿态噪声估计 -> 【不做】模型形式"
                            "检验 (modelFormChecked=false)。各向同性比 σ1/σ3=%.5g 仅作报告。\n",
                    d.isotropyRatio);
        } else if (!out.modelFormChecked) {
            // 有噪声但没残差自由度 (n = 4: 3n = 12 = 参数数) —— 检验同样做不了, 照实说。
            fprintf(stderr, "[Payload] 注意: %d 个姿态下力通道自由度 = 0 (3n − 12), 模型形式"
                            "检验【做不了】(不是通过)。\n", n);
        } else {
            if (out.chi2DofForce > 0) {
                const double limF = 1.0 + RAW_CHI2_SIGMA_K * sqrt(2.0 / (double)out.chi2DofForce);
                if (!(out.chi2ForceRatio < limF)) {
                    fprintf(stderr, "[Payload] 自检拒绝(模型形式): 力通道 χ²/dof=%.4g 超过门限"
                                    " %.4g (= 1 + %.0f·sqrt(2/%d), 3σ 置信) —— 残差 %.4g N 与实测"
                                    " 噪声 %.4g N (sd/sqrt(N)) 不一致, 模型形式解释不了这批数据。\n",
                            out.chi2ForceRatio, limF, RAW_CHI2_SIGMA_K, out.chi2DofForce,
                            out.rmsForceN, out.noiseForceN);
                    return false;
                }
            }
            if (out.lackOfFitMomentDof > 0) {
                const double limM = 1.0 + RAW_CHI2_SIGMA_K
                                        * sqrt(2.0 / (double)out.lackOfFitMomentDof);
                if (!(out.lackOfFitMomentRatio < limM)) {
                    fprintf(stderr, "[Payload] 自检拒绝(模型形式): 力矩通道失拟 %.4g 超过门限 %.4g"
                                    " (= 1 + %.0f·sqrt(2/%d)) —— 自由模型比 c_s × (A·g) 显著地"
                                    " 解释得更好, 叉乘结构不成立。\n",
                            out.lackOfFitMomentRatio, limM, RAW_CHI2_SIGMA_K,
                            out.lackOfFitMomentDof);
                    return false;
                }
            }
        }
        // 报告量: 各向同性比与它的构成 (奇异值)。【不是判决】, 理由见上。
        // 换工具/重装传感器/换一支笔, 这个数会变 —— 它是"此刻这只传感器响应有多正"的读数。
        fprintf(stderr, "[Payload] 各向同性比 σ1/σ3 = %.5f (报告量, 不作判据): σ=[%.5f %.5f %.5f]"
                        " kg, m = %.5f kg, parity = %+.0f\n",
                d.isotropyRatio, d.sv[0], d.sv[1], d.sv[2], d.m, d.parity);
        // 通过时也要把"离门限多远"照实报出来 —— 拒绝时的那些数只有被拒才看得到, 而
        // "贴着线过"与"富余着过"是两回事 (从前 `K=2` 那条门限就是被这个数救回来的)。
        if (out.modelFormChecked) {
            fprintf(stderr, "[Payload] 模型形式: rmsF=%.4g N / 实测噪声 %.4g N -> χ²/dof=%.4g"
                            " (门限 %.4g, dof=%d); 力矩失拟=%.4g (门限 %.4g, dof=%d)\n",
                    out.rmsForceN, out.noiseForceN, out.chi2ForceRatio,
                    1.0 + RAW_CHI2_SIGMA_K * sqrt(2.0 / (double)out.chi2DofForce), out.chi2DofForce,
                    out.lackOfFitMomentRatio,
                    out.lackOfFitMomentDof > 0
                        ? 1.0 + RAW_CHI2_SIGMA_K * sqrt(2.0 / (double)out.lackOfFitMomentDof) : 0.0,
                    out.lackOfFitMomentDof);
        }
        return true;
    }

    void effective(double& massKgOut, double comMmOut[3]) {
        if (enabled) {
            massKgOut = massKg;
            for (int i = 0; i < 3; i++) comMmOut[i] = comMm[i];
        } else {
            massKgOut = Config::ROBOT_PAYLOAD_SEED_KG;
            comMmOut[0] = Config::ROBOT_PAYLOAD_SEED_CX_MM;
            comMmOut[1] = Config::ROBOT_PAYLOAD_SEED_CY_MM;
            comMmOut[2] = Config::ROBOT_PAYLOAD_SEED_CZ_MM;
        }
    }

    // 把求解结果记成生效值 (仅内存)。【不碰 comSignZ】—— 它是持久化的显示约定, 不是
    // 求解器的输出: 数据定不了符号, 也没有探针去实测它, 谁都不该"定"这个案。
    void applyResult(const Result& r) {
        enabled = true;
        massKg = r.massKg;
        for (int i = 0; i < 3; i++) comMm[i] = r.comMm[i];
        rmsForceN = r.rmsForceN;
        rmsMomentNm = r.rmsMomentNm;
        poses = r.poses;
        // ψ 也一并记下 (它随文件持久化), 但【不】替调用方去动 TcpCalibration 的模块状态 ——
        // 那样做等于让一次"记账"悄悄改掉全局重力约定。装 psi 是 main.cpp 的显式一步。
        sensorYawDeg = r.sensorYawDeg;
    }

    // ===== 持久化 =====
    // ⚠ com_sign_z 现在【只是信息性的】, 仍然写出来只为两件事:
    //   (1) 随文件记住 com_mm 的 Z 是按哪种解释折算的, 免得重启后语义漂移;
    //   (2) load()/旧文件格式不用动。
    // 它的取值不再是"实测裁决出来的符号": 数据定不了符号 (两种解释残差完全相同), 实机探针
    // 也已废除 —— 没有任何判据、下发或本地补偿依赖它。别拿这个字段当结论读。
    bool save(const char* filepath) {
        FILE* f = fopen(filepath, "w");
        if (!f) return false;
        fprintf(f, "{\n");
        fprintf(f, "  \"version\": 1,\n");
        fprintf(f, "  \"mass_kg\": %.6g,\n", massKg);
        fprintf(f, "  \"com_mm\": [%.6g, %.6g, %.6g],\n", comMm[0], comMm[1], comMm[2]);
        fprintf(f, "  \"rms_force_n\": %.6g,\n", rmsForceN);
        fprintf(f, "  \"rms_moment_nm\": %.6g,\n", rmsMomentNm);
        fprintf(f, "  \"poses\": %d,\n", poses);
        // 传感器安装偏转角: 【是结论, 不是信息性字段】—— 重力模型的一半靠它, 装错整个
        // 力补偿都会跟着歪。写 6 位有效数字: 扫描分辨率 0.5°, 存回精度远高于此。
        fprintf(f, "  \"sensor_yaw_deg\": %.6g,\n", sensorYawDeg);
        fprintf(f, "  \"com_sign_z\": %.1f\n", comSignZ);   // 信息性, 见上 — 不是结论
        fprintf(f, "}\n");
        fclose(f);
        return true;
    }

    static const char* jsonFind(const char* buf, const char* key) {
        const char* p = strstr(buf, key);
        if (!p) return nullptr;
        p += strlen(key);
        while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r') p++;
        if (*p == ':') p++;
        while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r') p++;
        return p;
    }

    // 只负责解析 —— 文件不存在或读不出就直接返回 false。
    bool load(const char* filepath) {
        FILE* f = fopen(filepath, "r");
        if (!f) return false;
        char buf[2048];
        size_t n = fread(buf, 1, sizeof(buf) - 1, f);
        fclose(f);
        if (n == 0) return false;
        buf[n] = '\0';

        const char* p = jsonFind(buf, "\"mass_kg\"");
        if (!p) return false;
        double m = strtod(p, nullptr);
        if (!(m > 0.0)) return false;

        p = jsonFind(buf, "\"com_mm\"");
        if (!p) return false;
        if (*p == '[') p++;
        double c[3];
        for (int i = 0; i < 3; i++) {
            char* end = nullptr;
            c[i] = strtod(p, &end);
            if (end == p) return false;
            p = end;
            while (*p == ',' || *p == ' ' || *p == '\t' || *p == '\n' || *p == '\r') p++;
        }

        p = jsonFind(buf, "\"rms_force_n\"");
        rmsForceN = p ? strtod(p, nullptr) : 0.0;
        p = jsonFind(buf, "\"rms_moment_nm\"");
        rmsMomentNm = p ? strtod(p, nullptr) : 0.0;
        p = jsonFind(buf, "\"poses\"");
        poses = p ? atoi(p) : 0;
        // 信息性字段 (见 save 的说明): 只为让 com_mm 的 Z 有一个确定的解释, 缺了也无妨。
        p = jsonFind(buf, "\"com_sign_z\"");
        comSignZ = (p && strtod(p, nullptr) < 0.0) ? -1.0 : 1.0;
        // ψ: 【有则用, 无则回退 Config 种子】—— 旧文件 (本字段出现之前存的) 没有它,
        // 必须仍能加载, 否则用户的标定会在升级后集体作废。回退不是"缺失即 90"的猜测:
        // Config::SENSOR_MOUNT_YAW_DEG 就是本字段存在之前一直生效的那个值。
        p = jsonFind(buf, "\"sensor_yaw_deg\"");
        sensorYawDeg = p ? strtod(p, nullptr) : Config::SENSOR_MOUNT_YAW_DEG;

        massKg = m;
        for (int i = 0; i < 3; i++) comMm[i] = c[i];
        enabled = true;
        return true;
    }

} // namespace PayloadCalibration
