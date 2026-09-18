#include "PayloadCalibration.h"
#include "../calibration/TcpCalibration.h"
#include "../config/Config.h"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <ctime>

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
        fprintf(f, "  \"version\": 2,\n");
        fprintf(f, "  \"saved_at_unix\": %ld,\n", (long)time(NULL));
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

    // 只负责解析, 不判有效期 —— 调用方必须先过 CalibStore::resolve(),
    // 由它挡掉缺 saved_at_unix 或已过期的文件。
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
