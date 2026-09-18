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

    static const double G = 9.81;     // m/s²
    static const int    MIN_POSES = 3;   // 4 个未知量, 每个姿态贡献 6 个方程; 3 个起解

    // 重力在工具系下的表示: g_tool = Rᵀ · (0,0,G)
    static void gravityTool(const double pose[6], double g[3]) {
        double R[9];
        TcpCalibration::rpyToMatrix(pose[3], pose[4], pose[5], R);
        // R 为 row-major (工具→世界); 重力在工具系的表示 = Rᵀ·(0,0,G)。
        // (Rᵀ·v)[i] = Σ_k R[k*3+i]·v[k], 对 v=(0,0,G) 只剩 k=2 一项 → 取 R 的【第 2 行】。
        // 约定必须与 ForceCompensation::step 一致 (那里是 matTransposeMulVec(R,·)),
        // 否则解出的 Δm 是错的 —— 转置后仍与真值相关, 残差不会爆掉, 只会安静地解错。
        g[0] = R[6] * G;
        g[1] = R[7] * G;
        g[2] = R[8] * G;
    }

    // 构造姿态 k (相对姿态 0 差商后) 的 6 行方程。
    // 未知量 x = [Δm, Δp_x, Δp_y, Δp_z] (Δm: kg; Δp: kg·m)。
    // 差商同时消掉了传感器零偏 b_F / b_M, 所以方程右端不含常数项。
    static void buildRows(const double posesIn[][6], const double forces[][3],
                          const double moments[][3], const double g0[3], int k,
                          double rows[6][4], double rhs[6])
    {
        double gk[3];
        gravityTool(posesIn[k], gk);
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

    bool solve(const double posesIn[][6], const double forces[][3], const double moments[][3],
               int n, double mCfg, const double comCfg[3], double signZ, Result& out)
    {
        if (n < MIN_POSES) return false;

        double AtA[4][4] = {{0}};
        double Atb[4] = {0};

        double g0[3];
        gravityTool(posesIn[0], g0);

        double rows[6][4], rhs[6];
        for (int k = 1; k < n; k++) {
            buildRows(posesIn, forces, moments, g0, k, rows, rhs);
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
        double dm = A[0][4];
        double dp[3] = {A[1][4], A[2][4], A[3][4]};   // kg·m

        // ===== 换算绝对值 =====
        // signZ 不进 buildRows, 所以两种符号的拟合残差【完全相同】——
        // 数据本身区分不了符号, 只能把两种解释都给全 (下面的 comCand)。
        // 这里仍把两个候选的【物理】质心 Z 算出来供人复核 (判据必须看物理值,
        // 不能看下发值 —— cSend = cTrue/signZ 在两种符号下都可能是正的)。
        double mTrue = mCfg + dm;
        if (!(mTrue > 0.01) || mTrue > 5.0) return false;   // 非物理

        double cTrueZ[2];
        for (int k = 0; k < 2; k++) {
            const double s = (k == 0) ? 1.0 : -1.0;
            const double pCfgZ = mCfg * (comCfg[2] * s) / 1000.0;
            cTrueZ[k] = (pCfgZ + dp[2]) / mTrue * 1000.0;
        }
        // ===== 两个候选都算出来, 不在解算器里选 =====
        // 符号不在数据里 (它只进入 p_cfg 的换算, 在 Δp 的差值里被消掉), 所以解算器
        // 只能把两种解释都给全, 由调用方在实机上各下发一次、看谁留下的力矩残余小。
        double sChosen = signZ;              // 临时值; 由调用方定案
        bool   ambiguous = true;             // 实测前恒为"未定案"

        for (int k = 0; k < 2; k++) {
            const double s = (k == 0) ? 1.0 : -1.0;
            double cEffK[3] = {comCfg[0], comCfg[1], comCfg[2] * s};
            double pCfgK[3] = {mCfg * cEffK[0] / 1000.0, mCfg * cEffK[1] / 1000.0,
                               mCfg * cEffK[2] / 1000.0};
            for (int i = 0; i < 3; i++) {
                double cSend = (pCfgK[i] + dp[i]) / mTrue * 1000.0;
                if (i == 2 && s != 0.0) cSend /= s;
                // 量程按【每个候选】逐项查: 真正下发哪个候选要等实机探针裁决之后才知道,
                // 只看占位值会误杀一个根本不会用的候选、也会放行一个超量程的定案候选。
                if (fabs(cSend) > 500.0) return false;      // 超出 EnableRobot 的 ±500 量程
                out.comCand[k][i] = cSend;
            }
        }

        double cEff[3] = {comCfg[0], comCfg[1], comCfg[2] * sChosen};
        double pCfg[3] = {mCfg * cEff[0] / 1000.0,
                          mCfg * cEff[1] / 1000.0,
                          mCfg * cEff[2] / 1000.0};         // kg·m
        double pTrue[3] = {pCfg[0] + dp[0], pCfg[1] + dp[1], pCfg[2] + dp[2]};

        out.dm = dm;
        out.massKg = mTrue;
        out.signZ = sChosen;
        out.cTrueZ[0] = cTrueZ[0];
        out.cTrueZ[1] = cTrueZ[1];
        out.signAmbiguous = ambiguous;
        for (int i = 0; i < 3; i++) {
            double cTrue = pTrue[i] / mTrue * 1000.0;       // 物理质心 (mm)
            // 占位值: 按传入的 signZ 折算, 使机械臂的 c_eff 恰好等于 cTrue。真下发的值由
            // 实机探针选出的候选决定, 所以这里【不】是结论 —— 定案后也没有人会来覆写它。
            double cSend = (i == 2 && sChosen != 0.0) ? cTrue / sChosen : cTrue;
            out.comMm[i] = cSend;
            out.dc[i] = cSend - comCfg[i];
            // 量程不在这里查: comMm 现在只是占位值 (按传入的 signZ 折算), 真正会被下发的
            // 是 comCand[chosen][*], 已在上面的候选循环里逐项查过。
        }
        // ===== 拟合残差 |A·x − b| (不是数据本身的量级) =====
        double x[4] = {A[0][4], A[1][4], A[2][4], A[3][4]};
        double sumSqF = 0.0, sumSqM = 0.0;
        int eqF = 0, eqM = 0;
        for (int k = 1; k < n; k++) {
            buildRows(posesIn, forces, moments, g0, k, rows, rhs);
            for (int r = 0; r < 6; r++) {
                double pred = 0.0;
                for (int a = 0; a < 4; a++) pred += rows[r][a] * x[a];
                double e = pred - rhs[r];
                if (r < 3) { sumSqF += e * e; eqF++; }
                else       { sumSqM += e * e; eqM++; }
            }
        }
        out.rmsForceN = eqF > 0 ? sqrt(sumSqF / (double)eqF) : 0.0;
        out.rmsMomentNm = eqM > 0 ? sqrt(sumSqM / (double)eqM) : 0.0;
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

    void applyResult(const Result& r) {
        enabled = true;
        massKg = r.massKg;
        for (int i = 0; i < 3; i++) comMm[i] = r.comMm[i];
        comSignZ = r.signZ;
        rmsForceN = r.rmsForceN;
        rmsMomentNm = r.rmsMomentNm;
        poses = r.poses;
    }

    // 实机探针选定候选后定案: 0 = 符号约定 +1, 1 = 符号约定 -1。
    // comCand[chosen] 本身就是该符号下的下发值, 所以直接生效, 不需要再折算。
    // 【不回写 Result】(曾经 const_cast 写过 comMm/signZ/signAmbiguous): Result 是求解器的
    // 输出, 定案是调用方的事, 写回去既要用 const_cast 强改调用方的对象, 又没法把 dc 一起改对
    // (dc = 下发值 − 原配置值, 而原配置值只有 solve 收得到) —— 只改 comMm 不改 dc 的 Result
    // 自相矛盾。调用方要报定案值, 直接读它自己选中的 r.comCand[chosen]。
    void applyResult(const Result& r, int chosen) {
        if (chosen != 0 && chosen != 1) return;      // 无效选择: 不动生效值
        const double sChosen = (chosen == 0) ? 1.0 : -1.0;

        enabled = true;
        massKg = r.massKg;
        for (int i = 0; i < 3; i++) comMm[i] = r.comCand[chosen][i];
        comSignZ = sChosen;
        rmsForceN = r.rmsForceN;
        rmsMomentNm = r.rmsMomentNm;
        poses = r.poses;
    }

    // ===== 持久化 =====
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
        fprintf(f, "  \"com_sign_z\": %.1f\n", comSignZ);
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
        p = jsonFind(buf, "\"com_sign_z\"");
        comSignZ = (p && strtod(p, nullptr) < 0.0) ? -1.0 : 1.0;

        massKg = m;
        for (int i = 0; i < 3; i++) comMm[i] = c[i];
        enabled = true;
        return true;
    }

} // namespace PayloadCalibration
