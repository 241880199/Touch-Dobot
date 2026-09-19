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
    // 模型形式检验的【显著性水平】(不是物理阈值, 是统计惯例) —— 推导与实测的冤枉率
    // 见下面的 modelFormLimit。
    static const double RAW_MODEL_FORM_ALPHA = 0.997;

    // ===== 统计函数 (判决门限要用) =====
    // 只为实现 χ² 分位数; 都是纯函数, 与标定语义无关。
    static double logGammaFn(double x) {
        // Lanczos (g=7, n=9) —— 双精度下相对误差 ~1e-15, 足够。
        static const double c[9] = {
            0.99999999999980993,  676.5203681218851,   -1259.1392167224028,
            771.32342877765313,  -176.61502916214059,    12.507343278686905,
            -0.13857109526572012,   9.9843695780195716e-6, 1.5056327351493116e-7 };
        if (x < 0.5) {
            // 反射公式: Γ(x)Γ(1−x) = π/sin(πx)
            return log(3.14159265358979323846 / fabs(sin(3.14159265358979323846 * x)))
                 - logGammaFn(1.0 - x);
        }
        const double z = x - 1.0;
        double acc = c[0];
        for (int i = 1; i < 9; i++) acc += c[i] / (z + (double)i);
        const double t = z + 7.5;
        return 0.5 * log(2.0 * 3.14159265358979323846) + (z + 0.5) * log(t) - t + log(acc);
    }

    // 正则化下不完全 gamma P(a, x) = γ(a,x)/Γ(a) —— χ²(c) 的 CDF 就是 P(c/2, x/2)。
    // 级数 (x < a+1) / 连分式 (否则), 标准做法, 相对误差 ~1e-14。
    static double gammaIncP(double a, double x) {
        if (!(x > 0.0)) return 0.0;
        const double gln = logGammaFn(a);
        if (x < a + 1.0) {
            double ap = a, sum = 1.0 / a, del = sum;
            for (int n = 1; n <= 500; n++) {
                ap += 1.0; del *= x / ap; sum += del;
                if (fabs(del) < fabs(sum) * 1e-16) break;
            }
            return sum * exp(-x + a * log(x) - gln);
        }
        // 连分式求 Q(a,x), 再取 1−Q
        const double tiny = 1e-300;
        double b = x + 1.0 - a, c = 1.0 / tiny, d = 1.0 / b, h = d;
        for (int i = 1; i <= 500; i++) {
            const double an = -(double)i * ((double)i - a);
            b += 2.0;
            d = an * d + b; if (fabs(d) < tiny) d = tiny;
            c = b + an / c; if (fabs(c) < tiny) c = tiny;
            d = 1.0 / d;
            const double del = d * c;
            h *= del;
            if (fabs(del - 1.0) < 1e-16) break;
        }
        return 1.0 - exp(-x + a * log(x) - gln) * h;
    }

    // χ²(dof) 的 p 分位数。二分法 (P 单调): 不依赖初值猜测, 收敛到机器精度。
    // dof ≤ 0 或 p 不在 (0,1) 内 -> 返回 0 (= 门限退化成 0, 判决必然拒绝; 调用方会先挡掉)。
    static double chi2Quantile(double dof, double p) {
        if (!(dof > 0.0) || !(p > 0.0) || !(p < 1.0)) return 0.0;
        const double a = 0.5 * dof;
        double hi = dof + 1.0;
        for (int i = 0; i < 200 && gammaIncP(a, 0.5 * hi) < p; i++) hi *= 2.0;
        double lo = 0.0;
        for (int i = 0; i < 200; i++) {
            const double mid = 0.5 * (lo + hi);
            if (mid <= lo || mid >= hi) break;          // 已到浮点分辨率
            if (gammaIncP(a, 0.5 * mid) < p) lo = mid; else hi = mid;
        }
        return 0.5 * (lo + hi);
    }

    // ===== 模型形式检验的判决门限 =====
    //
    // 统计量 (力通道) : Z = Σ_i Σ_a e²_ia / σ_rep,a² / dof_fit,  dof_fit = 3n − 12
    // 零假设 (模型形式对) 下 Z 的期望是 1, 但【它是两个估计量之比】, 只拿 "1 + 3·sqrt(2/dof)"
    // 当门限是错的 —— 分母自己也是估量, 而且自由度很小。第三次评审的算式 (一通道, U ~ χ²₁):
    //
    //     σ_rep² ≈ σ²·U/normalizer ,  E[e²] ≈ σ²   =>   Z ≈ (1/3)·Σ_a (1/U_a)
    //
    // E[1/χ²₁] = ∞, 中位数 1/0.4549 = 2.198 —— 也就是旧门限 (dof 9 时 2.414) 正好压在零分布
    // 的【中位数】上, 而不是 99.7% 分位数上。正确模型在实机工况下有一半概率被拒, 而且是拿
    // 一个假的理由 (残差超尺子) 拒的。这一版把门限改成【分别承认分子与分母的自由度】:
    //
    //   拒绝 ⇔ Z > [ χ²(dof_fit, α) / dof_fit ] · factor
    //   factor = σ_rep² / σ_rep,lower²      (分母的单侧置信下界, 见下)
    //
    // · 分子: 正确模型下 S = Σe² ~ σ²·χ²(dof_fit), 所以 S/dof_fit 的 α 分位数就是
    //   χ²(dof_fit,α)/dof_fit。旧式 "1 + 3·sqrt(2/dof)" 是它的正态近似 —— dof 9 时给 2.41,
    //   而真值是 3.10, 即旧式本身还偏低 25%。现在直接用分位数, 不再近似。
    // · 分母: σ_rep² 是【估量】。它由两部分相加而成:
    //       σ_rep² = σ_sys² + floor²
    //   floor² (姿态内噪声) 是调用方申报的、已知的量 (生产路径来自同一姿态 N 个样本的样本
    //   方差); σ_sys² 是【量出来的】, 自由度 = 池化进来的重复对数 R: σ_sys² ≈ σ_sys,真²·U/R,
    //   U ~ χ²(R)。R 越小它越不可靠, 而它进的是【分母】(倒数) —— 分母偏小会把 Z 顶上去,
    //   于是正确模型被判错。所以把分母换成它的【单侧置信下界】(同为置信水平 α, 自由度 R):
    //       σ_sys,lower² = σ_sys² · R/χ²(R, α)     (floor² 不估, 不缩)
    //       σ_rep,lower² = σ_sys,lower² + floor²
    //   则 factor = σ_rep²/σ_rep,lower² = (1+r)/(1 + (R/χ²(R,α))·r)  (r = σ_sys²/floor²)。
    //   因为 χ²(R,α) ≥ R, 系数 R/χ²(R,α) ∈ (0,1] —— 下界永远在估计值【下面】, 所以
    //   factor ≥ 1 恒成立: 门限只会比"尺子完全可信"时【更宽】, 绝不会更紧。R 越大系数越接近
    //   1、factor 越接近 1 (门限越紧、分辨力越高); R = 1、α = 0.997 时系数 = 1/8.9, 门限被
    //   放宽到约 (1+r)/(1+0.11r) 倍 —— 这也正是自洽性要求: 零假设下 E[残差²] ≈ σ_rep²,
    //   统计量的期望本来就是 1, 尺子粗的时候必须允许它更大。
    //
    // 【仍然没有任何物理量阈值】: 尺度全部来自实测 (σ_sys² 与 floor² 都是数据/申报),
    // α 只是统计惯例 (见 RAW_MODEL_FORM_ALPHA), dof 全部来自数据形状 (姿态数、对数)。
    // 【不是自指】: σ_sys² 只从【重复对的差值】与【逐姿态样本方差】来, 从不读拟合残差。
    //
    // 实测的冤枉率 P(拒绝 | 模型形式正确) —— 按本函数的算式做蒙特卡洛 (12 参数最小二乘,
    // 正确模型 + 每姿态独立的姿态间离散 s + 姿态内噪声 e, r = s²/e²), n=8 (dof=12), 2.5 万次:
    //     r = 0 (合成数据常落在这里):      R=1 0.09%  R=2 0.04%  R=4 0.00%
    //     r ≈ 3.1 (报告里推出来的实机工况): R=1  12%  R=2 4.9%  R=3 2.5%  R=4 1.5%  R=8 0.5%
    //     r = 10  (复现性更差):            R=1  15%  R=2 5.9%  R=3 3.1%  R=4 2.0%  R=8 0.6%
    //   旧门限 (1 + 3·sqrt(2/dof), 等于压在零分布的中位数上) 在同一组模拟下:
    //     r=3.1 时 R=1 拒 36%; r=10 时 R=1 拒 61% —— 病就在那里 (而 n=7/dof=9 时更差)。
    //   R=1 剩下的那一截冤枉率是【信息量】的限制, 不是门限没调好: 一对只给 1 个自由度,
    //   有时两次访问凑巧对得很齐, 数据就真的说"复现性很好"; 再多池化几对 (按 'r' 可追加)
    //   才压得下去 —— 所以"贴着线过时补一对"这件事现在是真的有用。求解时会报 R。
    double modelFormLimit(double dofFit, int repPairs,
                          const double sys2[3], const double floor2[3]) {
        if (!(dofFit > 0.0)) return 0.0;
        const double base = chi2Quantile(dofFit, RAW_MODEL_FORM_ALPHA) / dofFit;
        // 池化后的 r = σ_sys²/floor² (三通道取"和的比", 与 poseLevelYardstick 同一口径:
        // 逐姿态残差也是把三通道的 σ² 相加再开方, 两边必须同口径才好比)。
        double s = 0.0, f = 0.0;
        for (int a = 0; a < 3; a++) { s += sys2[a]; f += floor2[a]; }
        if (!(f > 0.0) || !(s > 0.0)) return base;     // 没有测到复现性差 -> 不打折
        const double r = s / f;
        double shrink = 1.0;                           // R/χ²(R,α) ∈ (0,1]
        if (repPairs >= 1) {
            const double q = chi2Quantile((double)repPairs, RAW_MODEL_FORM_ALPHA);
            if (q > 0.0) shrink = (double)repPairs / q;
        }
        return base * (1.0 + r) / (1.0 + shrink * r);
    }

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

    // 逐姿态噪声估计【本身】是否可用 (与"哪个通道死了"分开判): 给了, 且每个姿态都有样本数。
    // 注意 N = 1 是【合法】的: 它表示"没有做平均, 但输入值的方差是已知的" (调用方从别处
    // 知道), 此时 σ_mean = sqrt(var/1)。生产路径上 N 是采集窗口里的样本数 (通常 ~30)。
    static bool poseNoiseGiven(const PoseNoise* noise, int n) {
        if (!noise) return false;
        for (int i = 0; i < n; i++) if (noise[i].n < 1) return false;
        return true;
    }

    // 逐点检查: 任一方差 ≤ 0 (那个姿态那个通道没采到 / 读数冻住 / 没填) 就不是一把完整的尺子。
    // 【为什么不能只看"整通道是否死"】: 尺子的每一项都要做分母 (e²/σ²), 一个 0 就够把统计量
    // 变成 inf/NaN —— 那样的"判决"不是判决。宁可说"尺子不完整", 也不拿它算。
    static bool allVariancesLive(const PoseNoise* noise, int n) {
        for (int i = 0; i < n; i++)
            for (int a = 0; a < 3; a++)
                if (!(noise[i].varF[a] > 0.0) || !(noise[i].varM[a] > 0.0)) return false;
        return true;
    }

    // 【整批恒为 0】的通道 —— 本机的一个真实故障模式 (读数冻住 / 通道没接上 / 没采到)。
    // 返回位掩码 (bit a = 第 a 个通道), 0 = 三个通道都有活数据。
    // 判据只看"整批恒等于 0": 这不是一个门限, 而是"这个数根本没有分布"这个【事实】。
    // 【为什么要单独认它】: 从前把"任一方差 ≤ 0"一律当成"没有噪声估计", 于是【通道坏了】
    // 与【协议没走完】被报成同一句话, 操作员会去重采一轮又一轮, 而该修的是硬件。
    static int deadChannelMask(const PoseNoise* noise, int n, bool moment) {
        if (!noise) return 0;
        int mask = 0;
        for (int a = 0; a < 3; a++) {
            bool anyLive = false;
            for (int i = 0; i < n; i++) {
                const double v = moment ? noise[i].varM[a] : noise[i].varF[a];
                if (v > 0.0) { anyLive = true; break; }
            }
            if (!anyLive) mask |= (1 << a);
        }
        return mask;
    }

    // 【一串】重复姿态对 -> 逐通道的【姿态间复现性】σ_rep —— 模型形式检验的尺子。
    // 逐对 (d = 两次访问的读数之差, v0²/v1² = 两次访问各自的【均值】方差 var/N):
    //   Var(d)  = 2·σ_sys² + v0² + v1²   (两次访问独立; σ_sys = "回到这个位姿再来一次"的离散)
    //   => σ_sys² = (d² − v0² − v1²)/2   (逐对夹到 0: 单次观测, 可能为负)
    //   σ_rep²  = σ_sys² + (v0² + v1²)/2
    // 池化 (R 对) :
    //   σ_sys,a² = mean_j max((d_j² − v0_j² − v1_j²)/2, 0)     <- 自由度 R (判决门限要它)
    //   floor_a² = mean_j (v0_j² + v1_j²)/2                    <- 申报的已知量, 不估
    //   σ_rep,a² = σ_sys,a² + floor_a²
    // 【最后那一项为什么加回来】尺子不可能比"单次测量本身有多准"更细: 两次访问凑巧对得很齐
    // (d≈0) 是运气, 不是复现性好。不加回, 一次幸运的重复会把门压到 0 —— 那是把一个
    // false-reject 换成另一个 false-reject, 正是本轮要修的病。
    // 【它自洽吗】正确模型下残差方差 ≈ σ_sys² + 姿态内噪声, 与 σ_rep² 同量级, 所以统计量
    // χ²_rep/dof 在零假设下的期望就是 1。
    // 【为什么池化而不是只认最后一对】: 尺子自己也是估量, R 对把它的离散按 1/sqrt(R) 压下去,
    // 门限里的 factor (见 modelFormLimit) 才收得紧 —— 判决的分辨力就是这么来的。
    // diff[] 报的是池化后各通道 |d| 的 rms (尺子的原始观测, 照实报)。
    // 返回实际池化进来的对数 (不合法的对不算), 0 = 没有尺子。
    static int yardstickPooled(const double forces[][3],
                               const double moments[][3], int n, const PoseNoise* nz,
                               const RepeatPair* reps, int repCount,
                               double sigF[3], double sigM[3],
                               double sysF[3], double sysM[3],
                               double floorF[3], double floorM[3],
                               double diffF[3], double diffM[3], bool* firstPairOk,
                               int* firstA, int* firstB) {
        double accSysF[3] = {0}, accSysM[3] = {0};
        double accFloorF[3] = {0}, accFloorM[3] = {0};
        double accSqF[3] = {0}, accSqM[3] = {0};
        int used = 0;
        *firstPairOk = false;
        for (int j = 0; j < repCount; j++) {
            const RepeatPair& rp = reps[j];
            if (rp.first < 0 || rp.first >= n) continue;
            if (rp.second < 0 || rp.second >= n) continue;
            if (rp.first == rp.second) continue;
            if (!*firstPairOk) { *firstPairOk = true; *firstA = rp.first; *firstB = rp.second; }
            for (int a = 0; a < 3; a++) {
                const double f0 = meanVar(nz[rp.first], a, false);
                const double f1 = meanVar(nz[rp.second], a, false);
                const double m0 = meanVar(nz[rp.first], a, true);
                const double m1 = meanVar(nz[rp.second], a, true);
                const double dF = forces[rp.second][a] - forces[rp.first][a];
                const double dM = moments[rp.second][a] - moments[rp.first][a];
                double exF = 0.5 * (dF * dF - f0 - f1); if (!(exF > 0.0)) exF = 0.0;
                double exM = 0.5 * (dM * dM - m0 - m1); if (!(exM > 0.0)) exM = 0.0;
                accSysF[a] += exF;   accSysM[a] += exM;
                accFloorF[a] += 0.5 * (f0 + f1);
                accFloorM[a] += 0.5 * (m0 + m1);
                accSqF[a] += dF * dF; accSqM[a] += dM * dM;
            }
            used++;
        }
        if (used <= 0) {
            for (int a = 0; a < 3; a++) {
                sigF[a] = sigM[a] = 0.0;
                sysF[a] = sysM[a] = floorF[a] = floorM[a] = 0.0;
                diffF[a] = diffM[a] = 0.0;
            }
            *firstA = -1; *firstB = -1;
            return 0;
        }
        for (int a = 0; a < 3; a++) {
            sysF[a]   = accSysF[a] / used;
            sysM[a]   = accSysM[a] / used;
            floorF[a] = accFloorF[a] / used;
            floorM[a] = accFloorM[a] / used;
            sigF[a]   = sqrt(sysF[a] + floorF[a]);
            sigM[a]   = sqrt(sysM[a] + floorM[a]);
            diffF[a]  = sqrt(accSqF[a] / used);
            diffM[a]  = sqrt(accSqM[a] / used);
        }
        return used;
    }

    // RawFit 的零填充 —— 【任何提前返回之前都必须过这里】。
    // 理由: 头文件明确鼓励调用方在 false 之后读诊断字段 (逐姿态残差、尺子状态都在 out 里),
    // 而从前 n < RAW_MIN_POSES 那条提前返回会把 out 留成调用方栈上的旧内容 —— 读到的就是垃圾。
    static void rawFitZero(RawFit& out) {
        for (int i = 0; i < 9; i++) out.A[i] = 0.0;
        for (int i = 0; i < 3; i++) { out.bF[i] = 0.0; out.cS[i] = 0.0; out.bM[i] = 0.0; }
        out.rmsForceN = 0.0; out.rmsMomentNm = 0.0; out.cond = 0.0;
        for (int i = 0; i < 18; i++) out.paramSigma[i] = 0.0;
        out.massScale = 0.0; out.parity = 0.0; out.isotropyRatio = 0.0;
        out.noiseForceN = 0.0; out.noiseMomentNm = 0.0;
        out.chi2ForceRatio = 0.0; out.chi2DofForce = 0;
        out.chi2MomentRatio = 0.0; out.chi2DofMoment = 0;
        out.repeatFirst = -1; out.repeatSecond = -1; out.repeatPairCount = 0;
        for (int i = 0; i < 3; i++) {
            out.repeatDiffF[i] = 0.0; out.repeatDiffM[i] = 0.0;
            out.repeatSigmaF[i] = 0.0; out.repeatSigmaM[i] = 0.0;
            out.repeatSysF[i] = 0.0; out.repeatSysM[i] = 0.0;
            out.repeatFloorF[i] = 0.0; out.repeatFloorM[i] = 0.0;
        }
        out.chi2RepForceRatio = 0.0; out.chi2RepForceLimit = 0.0;
        out.lackOfFitMomentRatio = 0.0; out.lackOfFitMomentLimit = 0.0;
        out.lackOfFitMomentDof = 0;
        for (int i = 0; i < RAW_POSE_REPORT_MAX; i++) {
            out.poseResidualF[i] = 0.0; out.poseResidualM[i] = 0.0;
            out.poseResidualRatioF[i] = 0.0;
        }
        out.poseResidualCount = 0; out.worstPoseF = -1;
        out.modelFormStatus = MODEL_FORM_NO_DOF;   // 姿态数不足时的诚实默认
        out.modelFormChecked = false;
        out.momentFormChecked = false;
    }

    // 自由力矩模型 M = b_M + N·w (N: 3×3 全自由, 12 参数) —— 【只用于失拟比较】, 不产出参数。
    // 受约束的参数模型是它的【子模型】: c_s × w = [c_s]ₓ·w 是一个反对称阵, 所以
    // "自由模型能解释而受约束模型解释不了"的那一份, 就是叉乘结构本身不成立的部分。
    // 自由度 = 12 − 6 = 6。
    // 回归量 w = A·g 用【已经解出的 A】算 —— 与受约束那一份用的是同一个 w, 这样两边的残差
    // 才可比 (A 的误差同等地进两边, 在差值里大部分相消; 若用真值 w 反而不可比)。
    // 返回 false = 方程数不足以养自由模型 (3n ≤ 12, 自由度 0), 或设计矩阵秩亏/数值奇异。
    // ssFree 按【通道】返回 (ssFree[3]) —— 失拟要逐通道除以该通道自己的 σ_rep,M²:
    // 三个力矩通道的复现性不必相同, 用通道平均的尺子去折算等于给它们强行等权。
    static bool fitMomentFree(const double posesIn[][6], const double A[9],
                              const double moments[][3], int n, double ssFree[3])
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

        for (int a = 0; a < 3; a++) ssFree[a] = 0.0;
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
                ssFree[a] += e * e;
            }
        }
        return true;
    }

    bool fitRawLinear(const double posesIn[][6], const double forces[][3],
                      const double moments[][3], int n, RawFit& out,
                      const PoseNoise* noiseIn, const RepeatPair* repeatsIn, int repeatCount)
    {
        rawFitZero(out);
        if (n < RAW_MIN_POSES) return false;

        // ===== 尺子: 姿态间复现性 (由重复姿态对测出) =====
        // 它【只依赖原始数据与逐姿态噪声】, 不依赖任何拟合结果 —— 所以可以在求解之前先算出来,
        // 而这一点正是它比"尺子取自残差"强的地方: 它不可能被拟合结果反向放松。
        // 逐通道的 σ_rep 要等噪声可用性判完才谈得上; 这里先摆好"有没有对"。
        const bool noiseGiven = poseNoiseGiven(noiseIn, n);
        const int  deadF = noiseGiven ? deadChannelMask(noiseIn, n, false) : 7;
        const int  deadM = noiseGiven ? deadChannelMask(noiseIn, n, true)  : 7;
        const bool allLive = noiseGiven && allVariancesLive(noiseIn, n);
        const bool noiseUsable = allLive && (deadF == 0) && (deadM == 0);
        // 可用的重复对: 下标合法且不是同一行。【池化】—— 每一对都并进同一把尺子, 尺子的
        // 自由度 = 并进来的对数 (见 yardstickPooled 与 modelFormLimit)。
        const bool haveReps = (repeatsIn != nullptr) && (repeatCount > 0);

        if (noiseUsable && haveReps) {
            bool firstOk = false;
            int a0 = -1, b0 = -1;
            out.repeatPairCount = yardstickPooled(forces, moments, n, noiseIn,
                                                  repeatsIn, repeatCount,
                                                  out.repeatSigmaF, out.repeatSigmaM,
                                                  out.repeatSysF, out.repeatSysM,
                                                  out.repeatFloorF, out.repeatFloorM,
                                                  out.repeatDiffF, out.repeatDiffM,
                                                  &firstOk, &a0, &b0);
            if (out.repeatPairCount > 0) {
                out.repeatFirst  = a0;
                out.repeatSecond = b0;
            }
        }
        const bool repeatOk = (out.repeatPairCount > 0);

        // 姿态级尺子 (逐姿态残差要拿它做分母): 三通道 σ_rep² 的均值再开方 ——
        // 与"该姿态的力残差 sqrt(Σ_a e²/3)"同一条口径。
        double repBarF2 = 0.0, repBarM2 = 0.0;
        if (noiseUsable && repeatOk) {
            for (int a = 0; a < 3; a++) {
                repBarF2 += out.repeatSigmaF[a] * out.repeatSigmaF[a];
                repBarM2 += out.repeatSigmaM[a] * out.repeatSigmaM[a];
            }
            repBarF2 /= 3.0; repBarM2 /= 3.0;
        }

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

        // 力通道残差与参数不确定度。同一个循环里顺手累加三样东西:
        //   · chi2F     = Σ(e/σ_in)², σ_in = 该姿态该通道【均值】的实测 1σ  -> 【报告量】
        //   · chi2RepF  = Σ(e/σ_rep)², σ_rep = 姿态间复现性 (重复对测出来的)  -> 【判据】
        //   · poseSSF[i]= 该姿态的 Σ_a e²                                    -> 逐姿态残差报告
        // ssF 的累加【顺序与式子和从前完全一样】, 所以 rmsForceN / paramSigma 一个比特都不会变。
        double ssF = 0.0, chi2F = 0.0, sig2SumF = 0.0, chi2RepF = 0.0;
        double poseSSF[RAW_POSE_REPORT_MAX] = {0.0};
        for (int i = 0; i < n; i++) {
            double g[3];
            gravityNoYaw(posesIn[i], g);
            for (int a = 0; a < 3; a++) {
                double pred = bF[a];
                for (int c = 0; c < 3; c++) pred += A[a * 3 + c] * g[c];
                const double e = pred - forces[i][a];
                ssF += e * e;
                if (i < RAW_POSE_REPORT_MAX) poseSSF[i] += e * e;
                if (noiseUsable) {          // 尺子有一处为 0 就不算这个统计量 (0 会算出 inf/NaN)
                    chi2F += e * e / meanVar(noiseIn[i], a, false);
                    sig2SumF += meanVar(noiseIn[i], a, false);
                }
                if (repBarF2 > 0.0) chi2RepF += e * e / (out.repeatSigmaF[a] * out.repeatSigmaF[a]);
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
        double ssMCh[3] = {0.0, 0.0, 0.0};       // 逐通道的力矩残差平方和 (失拟要逐通道折算)
        double poseSSM[RAW_POSE_REPORT_MAX] = {0.0};
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
                ssMCh[a] += e * e;
                if (i < RAW_POSE_REPORT_MAX) poseSSM[i] += e * e;
                if (noiseUsable) {          // 同上
                    chi2M += e * e / meanVar(noiseIn[i], a, true);
                    sig2SumM += meanVar(noiseIn[i], a, true);
                }
            }
        }
        const int dofM = 3 * n - PM;
        const double s2M = (dofM > 0) ? ssM / (double)dofM : 0.0;

        // 力矩通道的【失拟】: 自由 12 参数模型与受约束 6 参数模型的残差之差, 按尺子折算。
        // 为什么力矩问的是失拟而不是"残差 vs 尺子": 见 fitRaw 的自检 3 与 RawFit 的说明。
        // 【逐通道折算】: 每一项除以【该通道自己的】σ_rep,M,a² —— 三个力矩通道的复现性
        // 不必相同 (见 fitMomentFree 的说明)。统计量 = Σ_a(ssM_a − ssFree_a)/σ_rep,M,a²。
        double ssFreeM[3] = {0.0, 0.0, 0.0};
        const bool haveFreeM = repBarM2 > 0.0 && fitMomentFree(posesIn, A, moments, n, ssFreeM);

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

        // ===== 尺子 1 (报告量): 姿态内噪声 =====
        if (noiseUsable) {
            // 实测噪声的合并尺度: 各方程 σ² 的均值再开方 (与 rms 同量纲, 好直接对比)。
            out.noiseForceN   = sqrt(sig2SumF / (3.0 * n));
            out.noiseMomentNm = sqrt(sig2SumM / (3.0 * n));
            out.chi2ForceRatio = (dofF > 0) ? chi2F / (double)dofF : 0.0;
            out.chi2DofForce   = dofF;
            out.chi2MomentRatio = (dofM > 0) ? chi2M / (double)dofM : 0.0;
            out.chi2DofMoment   = dofM;
        }

        // ===== 逐姿态残差 (spec §3 表格第 4 行) =====
        // 每个姿态的 rms, 以及它与【尺子】的比 —— 只有这两个数摆在一起, 读的人才能分开
        // "某一个姿态坏了" (比值只有那一个高) 和"模型整体不对" (比值个个都高)。
        // 它的分母是【姿态级】尺子 (三通道 σ_rep² 的均值再开方), 与分子的口径一致。
        out.poseResidualCount = (n < RAW_POSE_REPORT_MAX) ? n : RAW_POSE_REPORT_MAX;
        for (int i = 0; i < out.poseResidualCount; i++) {
            out.poseResidualF[i] = sqrt(poseSSF[i] / 3.0);
            out.poseResidualM[i] = sqrt(poseSSM[i] / 3.0);
            out.poseResidualRatioF[i] = (repBarF2 > 0.0)
                                      ? out.poseResidualF[i] / sqrt(repBarF2) : 0.0;
            if (out.worstPoseF < 0 || out.poseResidualF[i] > out.poseResidualF[out.worstPoseF])
                out.worstPoseF = i;
        }

        // ===== 尺子 2 (判据): 残差 vs 姿态间复现性 =====
        if (repBarF2 > 0.0) {
            out.chi2RepForceRatio = (dofF > 0) ? chi2RepF / (double)dofF : 0.0;
            // 力矩失拟按【同样的尺子】折算 (从前按姿态内 σ̄_M² —— 那把尺子量不了姿态间系统差):
            //   diff = ssM(受约束 6 参数) − ssFree(自由 12 参数), 正确模型下期望 6·σ_rep,M²
            // ⚠ 方向: 自由模型【包含】受约束模型 (c_s × w = [c_s]ₓ·w 是 N 的一个特例),
            // 所以恒有 ssFree ≤ ssM。"多出来的 6 个参数少解释的那一份"就是 ssM − ssFree,
            // 叉乘结构不成立时它远大于 6σ²。
            if (haveFreeM) {
                double loF = 0.0;
                for (int a = 0; a < 3; a++) {
                    const double d = ssMCh[a] - ssFreeM[a];
                    loF += ((d > 0.0) ? d : 0.0)   // 舍入可致微负
                         / (out.repeatSigmaM[a] * out.repeatSigmaM[a]);
                }
                out.lackOfFitMomentRatio = loF / 6.0;
                out.lackOfFitMomentDof   = 6;
                out.lackOfFitMomentLimit = modelFormLimit(6.0, out.repeatPairCount,
                                                          out.repeatSysM, out.repeatFloorM);
            }
        }
        out.chi2RepForceLimit = (out.chi2DofForce > 0)
                              ? modelFormLimit((double)out.chi2DofForce, out.repeatPairCount,
                                               out.repeatSysF, out.repeatFloorF)
                              : 0.0;

        // ===== 尺子状态 —— "没验过"必须与"验过、通过了"分得开, 且"为什么没验"要分开报 =====
        // 3n > 12 (即 n ≥ 5) 才有残差自由度可言; n = 4 时自由度 0, 检验【做不了】。
        if (!noiseGiven)            out.modelFormStatus = MODEL_FORM_NO_NOISE;
        else if (noiseUsable) {
            if (!repeatOk)          out.modelFormStatus = MODEL_FORM_NO_REPEAT;
            else if (dofF <= 0)     out.modelFormStatus = MODEL_FORM_NO_DOF;
            else                    out.modelFormStatus = MODEL_FORM_OK;
        }
        // 【整批为 0 的通道 = 通道冻住】—— 【包括六个通道【全】冻住】那种情形。
        // (从前先判 "有没有活数据": 六个全冻住时会被报成"没有逐姿态噪声估计", 于是给出
        //  "把采集的样本方差传进来" 这条【错的】建议 —— 调用方本来就传了, 该做的是查硬件。)
        else if ((deadF | deadM) != 0) out.modelFormStatus = MODEL_FORM_DEAD_CHANNEL;
        else                        out.modelFormStatus = MODEL_FORM_NOISE_HOLES;
        // 【modelFormChecked / momentFormChecked 只在真正判决通过时才置 true (在 fitRaw 里)】——
        // 这里是求解层, 它只负责报"尺子发生了什么"。
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

    // 姿态级尺子: 三通道 σ_rep² 的均值再开方 —— 与逐姿态残差比值的分母是同一个量。
    static double poseLevelYardstick(const double sig[3]) {
        return sqrt((sig[0] * sig[0] + sig[1] * sig[1] + sig[2] * sig[2]) / 3.0);
    }

    // 逐姿态残差表 (spec §3 表格第 4 行)。【打印在判决之前】—— 拒绝的时候比通过的时候更
    // 需要它: 只有这一张表能把"某一个姿态坏了" (只有一个比值高) 与"模型整体不对" (个个都高)
    // 分开, 而这两种情形的处置完全不同 (重采那一个姿态 vs 换模型/查装夹)。
    static void printPoseResiduals(const RawFit& f) {
        if (f.poseResidualCount <= 0) return;
        fprintf(stderr, "[Payload] 逐姿态残差 (力 N / 力矩 N·m / 力残差÷尺子):\n");
        for (int i = 0; i < f.poseResidualCount; i++) {
            const bool worst = (i == f.worstPoseF);
            fprintf(stderr, "         %s pose %2d: %8.4f  %8.4f   %.2f%s\n",
                    worst ? "->" : "  ", i + 1,
                    f.poseResidualF[i], f.poseResidualM[i], f.poseResidualRatioF[i],
                    worst ? "   <- 力残差最大" : "");
        }
        if (f.repeatPairCount > 0) {
            // 尺子 (池化): 报出处、对数 (= 自由度)、两个分量与合成值 —— 判决门限就是从
            // σ_sys²/floor² 与对数来的 (见 modelFormLimit), 所以这几个数要能一眼看到。
            fprintf(stderr, "         尺子 = 姿态间复现性 (pose %d 与 pose %d 是同一姿态的两次"
                            "访问, 中间有真实运动; 共池化 %d 对 = 自由度 %d):\n",
                    f.repeatFirst + 1, f.repeatSecond + 1, f.repeatPairCount, f.repeatPairCount);
            fprintf(stderr, "                力 σ_rep=%.4f/%.4f/%.4f N"
                            " (其中【复位姿的离散】σ_sys=%.4f/%.4f/%.4f N),"
                            " 力矩 σ_rep=%.4f/%.4f/%.4f N·m\n",
                    f.repeatSigmaF[0], f.repeatSigmaF[1], f.repeatSigmaF[2],
                    sqrt(f.repeatSysF[0]), sqrt(f.repeatSysF[1]), sqrt(f.repeatSysF[2]),
                    f.repeatSigmaM[0], f.repeatSigmaM[1], f.repeatSigmaM[2]);
        } else {
            fprintf(stderr, "         没有重复姿态对 -> 最后一列 (残差÷尺子) 无意义, 一律 0\n");
        }
    }

    bool fitRaw(const double posesIn[][6], const double forces[][3], const double moments[][3],
                int n, RawFit& out, const PoseNoise* noise, const RepeatPair* repeats,
                int repeatCount, ModelFormPolicy policy)
    {
        if (!fitRawLinear(posesIn, forces, moments, n, out, noise, repeats, repeatCount)) return false;

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

        // 逐姿态残差表 (报告, 不判) —— 判决之前打, 拒绝时才有据可查。
        // 【必须在质量尺度门之前】: 质量尺度被拒时这一张表最有用 (它能把"某一个坏姿态"与
        // "整体形式错"分开, 而这两种情形下质量尺度都可能解成非物理的值) —— 从前它排在那道
        // 门之后, 于是最需要它的时候恰恰看不到。
        printPoseResiduals(out);

        // ---- 自检 2: 质量尺度必须落在 EnableRobot 的负载量程里 ----
        // CR3 额定 3 kg (Docs/机械臂资料/Dobot CR3机械臂参数文档.md §最大负载);
        // 下限 0.05 kg: 工具链(传感器+笔夹+笔)不可能轻于此, 解到更小说明解出的不是工具重量。
        if (!(d.m > RAW_MASS_MIN_KG) || d.m > RAW_MASS_MAX_KG) {
            fprintf(stderr, "[Payload] 自检拒绝: 质量尺度 m=%.4g kg 超出量程 (%.2f, %.2f] kg"
                            " —— 这不是工具链的重量。\n",
                    d.m, RAW_MASS_MIN_KG, RAW_MASS_MAX_KG);
            return false;
        }

        // ---- 自检 3: 模型形式 —— 残差 vs 【姿态间复现性】(重复姿态对测出来的尺子) ----
        //
        // 【为什么尺子必须是重复姿态对, 而不是姿态内噪声 —— 两轮评审的病都在这里】
        // 姿态内噪声量的是"同一个姿态多采几秒会有多稳"; 而模型形式错是姿态【之间】的系统差。
        // 前者对后者是【盲的】。拿姿态内噪声当尺子, 两次都错:
        //   (1) 第一版各向同性门限 (σ ← 残差) 自指: 评审实跑反例 —— 用历史上的转置回归量
        //       生成的数据拟合出 iso=2.01 < 门限 2.59 -> 【错解被接受】, m=0.4159 (真值 0.42);
        //   (2) 第二版改成"残差 vs 姿态内噪声"的 χ² 门限, 仍是错的, 而且方向相反: 实机
        //       7 姿态 rmsF=0.022386 N, 而姿态内 σ̄ 只有 ~0.011 N (force_demo_log.csv 同一
        //       姿态 76 个样本, 逐轴 sd 0.0985/0.0191/0.0098 N) —— 那个门限实际在要求
        //       "姿态间系统差 ≤ 0.78σ̄", 即要求模型与 4.1 N 的重力信号一致到 0.5%, 而传感器
        //       自己的底噪就有 0.2%。【正确的解会被拒】, 正是设计要避免的那一面。
        // 病根: 姿态相关的模型误差 (比如重力约定搞错) 在姿态内散布里【完全看不见】。
        // 换成"回到同一个姿态再采一次"测出的离散之后, 判据问的才是正确的问题:
        //     "模型的失配, 有没有超出这台设备复现同一个姿态的能力?"
        // 那两次访问之间差的是装夹/位姿复现/迟滞/漂移 —— 与模型形式错【同一条路径、同一个
        // 量纲】, 而且它仍然不是预设: 它是量出来的 (RepeatPair)。
        // 统计量沿用 χ² 的形状 (零假设下期望 1, 与 σ 的来处无关)。
        // 【门限 —— 第三次修复: 门限必须承认【尺子自己也是估量】】
        // 前两轮的门限都是 "1 + K·sqrt(2/dof_fit)" (dof_fit 是【分子】的自由度)。那是错的:
        // σ_rep² 是由【重复对】估出来的, 一对只有 1 个自由度, 而它进的是【分母】。一通道
        // (U ~ χ²₁): σ_rep² ≈ σ²·U/normalizer, E[e²] ≈ σ² ⇒ Z ≈ (1/3)Σ_a(1/U_a);
        // E[1/χ²₁] = ∞、中位数 2.198, 而旧门限在 dof 9~15 是 2.41~2.10 —— 正好压在零分布的
        // 【中位数】上, 于是正确模型有大约一半的概率被拒, 理由还是假的。
        // 现在的门限 (见 modelFormLimit, 推导与实测冤枉率都写在那里):
        //     拒绝 ⇔ χ²_rep/dof_fit > [χ²(dof_fit,α)/dof_fit] · σ_rep²/σ_rep,lower²
        // 分子用残差自己的 χ² 分位数 (不再用正态近似), 分母用它自己的单侧置信下界 ——
        // 两侧都按自己的自由度说话。【没有绝对 N/N·m 阈值】: 尺度全部来自那把实测的尺子,
        // α 只是统计惯例 (RAW_MODEL_FORM_ALPHA)。
        //
        // 力矩通道【另论】: 它确实是受约束的 (c_s × (A·g), 3 参数 vs 自由的 9), 但它的回归量
        // w = A·g 里带着 A 的估计误差, 所以"力矩残差 vs 尺子"不是纯噪声统计量 (会系统性偏大),
        // 拿它当判据会错杀。力矩问的是【失拟】: 自由模型 (b_M + N·w, 12 参数) 与受约束模型
        // (b_M + c_s × w, 6 参数) 的残差之差, 按【同一把尺子】折算, 自由度 6。
        // 两边都用同一个 w, A 的误差在差值里大部分相消。
        if (out.modelFormStatus != MODEL_FORM_OK) {
            // ===== 尺子不齐: 说清楚【是哪种不齐】, 然后【不给参数】 =====
            switch (out.modelFormStatus) {
            case MODEL_FORM_DEAD_CHANNEL: {
                const int df = deadChannelMask(noise, n, false);
                const int dm = deadChannelMask(noise, n, true);
                // 【六个通道全冻住也走这里】: 从前先判"有没有活数据", 全冻住会被报成
                // "没有逐姿态噪声估计", 于是给出一条【错的】建议 ("把采集的样本方差传进来")
                // —— 调用方本来就传了, 该做的是查硬件/接线/取数。
                const bool allDead = ((df | dm) == 7);
                fprintf(stderr, "[Payload] !! 【通道冻住】@1304 有通道整批方差恒为 0 (力 0x%x /"
                                " 力矩 0x%x, bit0=x bit1=y bit2=z)%s。\n"
                                "         这不是「采集没做好」, 是传感器/接线/取数的问题 ——"
                                " 重采不会有改善, 先修通道。\n",
                        df, dm, allDead ? " —— 【六个通道全冻住】" : ", 而别的通道是活的");
                break;
            }
            case MODEL_FORM_NO_REPEAT:
                fprintf(stderr, "[Payload] !! 【没有重复姿态对】: 同一个姿态采两次是模型形式"
                                "唯一的尺子, 少一次就无从判。\n"
                                "         采集时: 摆完所有姿态后回到【第 1 个姿态】(位置和姿态"
                                "都回到第一次那个位姿), 按 'r' 再采一次。\n");
                break;
            case MODEL_FORM_NOISE_HOLES:
                fprintf(stderr, "[Payload] !! 【有的姿态/通道方差为 0】: 通道没死 (别的姿态"
                                "采到了), 是那一笔数据没采到 —— 尺子不完整。\n");
                if (noise) {
                    int shown = 0;
                    for (int i = 0; i < n && shown < 6; i++)
                        for (int a = 0; a < 3 && shown < 6; a++) {
                            if (!(noise[i].varF[a] > 0.0)) {
                                fprintf(stderr, "         pose %d 的力 %c 通道方差 = 0\n",
                                        i + 1, "xyz"[a]); shown++;
                            }
                            if (!(noise[i].varM[a] > 0.0)) {
                                fprintf(stderr, "         pose %d 的力矩 %c 通道方差 = 0\n",
                                        i + 1, "xyz"[a]); shown++;
                            }
                        }
                    if (shown == 0) fprintf(stderr, "         (样本数 < 2 的姿态也会走到这里)\n");
                }
                break;
            case MODEL_FORM_NO_NOISE:
                fprintf(stderr, "[Payload] !! 【没有逐姿态噪声估计】: 尺子要从重复对的差值里"
                                "扣掉姿态内噪声那一份, 没有它就量不出复现性。\n"
                                "         生产路径请传采集时的样本方差 (BiasCheck 每次都采)。\n");
                break;
            case MODEL_FORM_NO_DOF:
            default:
                fprintf(stderr, "[Payload] !! 【自由度不足】: %d 个姿态下力通道残差自由度"
                                " = 3n − 12 = %d —— 检验做不了 (不是通过)。\n",
                        n, out.chi2DofForce);
                break;
            }
            if (policy == MODEL_FORM_REQUIRED) {
                fprintf(stderr, "[Payload] 自检拒绝(模型形式): 模型形式【没有被检验过】——"
                                " 不给参数 (要参数请显式传 I_ACCEPT_UNVERIFIED_MODEL_FORM)。\n");
                return false;
            }
            // 显式放弃: 照给参数, 但【不把 modelFormChecked 置成 true】—— 它一直是 false。
            fprintf(stderr, "[Payload] !! 调用方显式接受了【未检验的模型形式】"
                            " (I_ACCEPT_UNVERIFIED_MODEL_FORM): 参数照给, modelFormChecked"
                            " 仍为 false —— 别把它当「验过了」读。\n");
        } else {
            const double yardF = poseLevelYardstick(out.repeatSigmaF);
            // 力通道: 残差 vs 姿态间复现性。门限含【尺子自由度的折扣】(见 modelFormLimit)。
            const double limF = out.chi2RepForceLimit;
            if (!(out.chi2RepForceRatio < limF)) {
                fprintf(stderr, "[Payload] 自检拒绝(模型形式): 力通道 残差÷尺子 χ²/dof=%.4g 超过"
                                " 门限 %.4g (= χ²(%d,%.3g)/%d = %.4g, 再乘尺子的自由度折扣 %.3g)"
                                " —— 残差 %.4g N 远超【姿态间复现性】%.4g N (同一姿态两次访问的"
                                "水平)。\n",
                        out.chi2RepForceRatio, limF, out.chi2DofForce, RAW_MODEL_FORM_ALPHA,
                        out.chi2DofForce,
                        chi2Quantile((double)out.chi2DofForce, RAW_MODEL_FORM_ALPHA)
                            / (double)out.chi2DofForce,
                        (limF > 0.0)
                            ? limF / (chi2Quantile((double)out.chi2DofForce, RAW_MODEL_FORM_ALPHA)
                                      / (double)out.chi2DofForce)
                            : 1.0,
                        out.rmsForceN, yardF);
                if (out.worstPoseF >= 0)
                    fprintf(stderr, "         最差姿态 = pose %d (残差 %.4f N = 尺子的 %.2f 倍)"
                                    " —— 只有它一个高就先重采它; 个个都高才是模型形式错。\n",
                            out.worstPoseF + 1, out.poseResidualF[out.worstPoseF],
                            out.poseResidualRatioF[out.worstPoseF]);
                if (out.repeatPairCount < 2)
                    fprintf(stderr, "         尺子只有 %d 对 (自由度 %d) —— 尺子自己不够稳时门限"
                                    "必然放宽, 分辨力也就低。回到第 1 个姿态再按 'r' 补几对。\n",
                            out.repeatPairCount, out.repeatPairCount);
                return false;
            }
            // 力矩通道: 失拟 (自由模型比叉乘结构是否显著地解释得更好)
            if (out.lackOfFitMomentDof > 0) {
                const double limM = out.lackOfFitMomentLimit;
                if (!(out.lackOfFitMomentRatio < limM)) {
                    fprintf(stderr, "[Payload] 自检拒绝(模型形式): 力矩通道失拟 %.4g 超过门限 %.4g"
                                    " (= χ²(6,%.3g)/6 = %.4g, 再乘尺子的自由度折扣)"
                                    " —— 自由模型比 c_s × (A·g) 显著地解释得更好,"
                                    " 叉乘结构不成立。\n",
                            out.lackOfFitMomentRatio, limM, RAW_MODEL_FORM_ALPHA,
                            chi2Quantile(6.0, RAW_MODEL_FORM_ALPHA) / 6.0);
                    return false;
                }
            } else if (3 * n > 12) {
                // 【力矩那一半没验成, 不能悄悄过去】: 3n > 12 说明自由度是够的, 那么自由模型
                // 解不出来只能是设计矩阵秩亏/数值奇异 (姿态都挤在一小块里)。此时力矩的失拟
                // 检验【根本没做】, 而力通道的判据是独立的 —— 照给参数, 但要把话说出来。
                fprintf(stderr, "[Payload] 注意: 力矩通道的失拟检验【没做成】(自由 12 参数模型"
                                "在这批姿态上秩亏/数值奇异) —— 叉乘结构这一半【没有被检验】。\n"
                                "         力通道的模型形式检验是各自独立的, 上面的结论不受影响。\n");
            }
            // 走到这里才是【验过且通过】—— modelFormChecked 与 momentFormChecked 唯一被置
            // true 的地方。后者从前在求解层就置了 true, 于是"力通道被判错、整体拒绝"时它
            // 照样亮着, 读起来像"至少力矩那半边是好的" —— 现在两个标志同进同退。
            out.modelFormChecked = true;
            out.momentFormChecked = (out.lackOfFitMomentDof > 0);
        }
        // 报告量: 各向同性比与它的构成 (奇异值)。【不是判决】, 理由见上。
        // 换工具/重装传感器/换一支笔, 这个数会变 —— 它是"此刻这只传感器响应有多正"的读数。
        fprintf(stderr, "[Payload] 各向同性比 σ1/σ3 = %.5f (报告量, 不作判据): σ=[%.5f %.5f %.5f]"
                        " kg, m = %.5f kg, parity = %+.0f\n",
                d.isotropyRatio, d.sv[0], d.sv[1], d.sv[2], d.m, d.parity);
        // 通过时也要把"离门限多远"照实报出来 —— 拒绝时的那些数只有被拒才看得到, 而
        // "贴着线过"与"富余着过"是两回事 (从前 `K=2` 那条门限就是被这个数救回来的)。
        // 两把尺子一起报: 姿态内噪声是【诊断】(它明显小于残差, 说明现场有姿态间的系统差),
        // 姿态间复现性才是【判据】。两个数摆在一起, 读的人才知道该去查什么。
        if (out.modelFormChecked) {
            fprintf(stderr, "[Payload] 模型形式(判据 = 尺子 2, 姿态间复现性; 尺子 %d 对"
                            " -> 门限自带自由度折扣): rmsF=%.4g N / 尺子 %.4g N -> χ²/dof=%.4g"
                            " (门限 %.4g, dof=%d); 力矩失拟=%.4g (尺子 %.4g N·m, 门限 %.4g, dof=%d)\n",
                    out.repeatPairCount,
                    out.rmsForceN, poseLevelYardstick(out.repeatSigmaF),
                    out.chi2RepForceRatio, out.chi2RepForceLimit, out.chi2DofForce,
                    out.lackOfFitMomentRatio, poseLevelYardstick(out.repeatSigmaM),
                    out.lackOfFitMomentLimit, out.lackOfFitMomentDof);
            fprintf(stderr, "[Payload] 对照(尺子 1, 姿态内噪声, 只报告不判): 力 %.4g N / 力矩"
                            " %.4g N·m; χ²/dof(姿态内) 力 %.4g / 力矩 %.4g —— 残差若明显大于这一组"
                            "而与小的一致的尺子相符, 就是现场复现性差, 不是模型错。\n",
                    out.noiseForceN, out.noiseMomentNm, out.chi2ForceRatio, out.chi2MomentRatio);
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
