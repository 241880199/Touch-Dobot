#include "TcpCalibration.h"
#include "../config/Config.h"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdlib>

namespace TcpCalibration {
    static const double D2R = 3.14159265358979323846 / 180.0;
    static const double G   = 9.81;

    // ===== 全局标定状态 =====
    bool enabled = false;
    double offset[3] = {0, 0, 0};
    double rmsError = 0.0;

    bool collectMode = false;
    int  collectCount = 0;
    double collectPose[MAX_COLLECT_POSES][6] = {{0}};

    // ===== 传感器安装偏转角 psi (模块状态) =====
    // 种子 = Config::SENSOR_MOUNT_YAW_DEG (编译期常量即可初始化, 无静态初始化顺序问题)。
    // 真值由 PayloadCalibration::solve 从数据里扫出来, main.cpp 用 setSensorYawDeg 装上。
    static double s_sensorYawDeg = Config::SENSOR_MOUNT_YAW_DEG;
    // cos/sin 缓存: gravitySensorFrame 在运行时路径上 (~125 Hz), 不值得每次都调三角函数。
    // 用"有效位 + 懒重算"而不是在 setSensorYawDeg 里直接算 —— 否则任何一个早于 setter 的
    // 调用都会读到未初始化的 0, g 变成全 0 而【不报错】(本文件最怕的就是这种安静的错误)。
    static double s_cosYaw = 0.0, s_sinYaw = 0.0;
    static bool   s_yawCacheValid = false;

    static void refreshYawCache() {
        const double psi = s_sensorYawDeg * D2R;
        s_cosYaw = cos(psi);
        s_sinYaw = sin(psi);
        s_yawCacheValid = true;
    }

    void setSensorYawDeg(double deg) {
        s_sensorYawDeg = deg;
        s_yawCacheValid = false;
    }

    double sensorYawDeg() {
        return s_sensorYawDeg;
    }

    // 输入为【度】(与 GetPose 返回的 Rx/Ry/Rz、Config 安全限位、ServoP 一致)。
    // 注: 早先此函数直接对入参做 cos/sin (即按弧度解释), 而唯一真实调用方
    //     main.cpp 传的是 GetPose 的度值 → 旋转矩阵是错的。单位统一到"度"。
    void rpyToMatrix(double rx_deg, double ry_deg, double rz_deg, double R[9]) {
        double rx = rx_deg * D2R, ry = ry_deg * D2R, rz = rz_deg * D2R;
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

    // 重力在【法兰系】的表示 g0 = Rᵀ·(0,0,G)。
    // (Rᵀ·v)[i] = Σ_k R[k*3+i]·v[k], 对 v=(0,0,G) 只剩 k=2 一项 → 取 R 的第三行 R[6..8]。
    // ⚠ 取 R[2],R[5],R[8] (第三列) 等于 R·(0,0,G), 与 Rᵀ 差一个转置 —— 历史上就错在这里,
    //    转置后仍与真值相关、残差不会爆掉, 只会安静地解错, 所以别凭"看着像"改。
    static void flangeGravity(const double pose[6], double g0[3]) {
        double R[9];
        rpyToMatrix(pose[3], pose[4], pose[5], R);
        g0[0] = R[6] * G;
        g0[1] = R[7] * G;
        g0[2] = R[8] * G;
    }

    // Rz(-psi) 作用在【法兰系】重力上 —— 这是本文件唯一的数学实体, 上面两个入口都走它,
    // 免得"带 psi 的扫描版"和"用模块状态的运行时版"各写一份而漂移。
    // Rz(-psi) = [[ cos, sin, 0], [-sin, cos, 0], [0, 0, 1]]  (psi > 0 = 传感器系相对法兰
    // 系逆时针偏转这么多)。z 分量不受绕 z 偏转影响 —— 这正是"零第三行/列"的特征。
    static void rotateGravityByYaw(const double g0[3], double cpsi, double spsi, double g[3]) {
        g[0] =  cpsi * g0[0] + spsi * g0[1];
        g[1] = -spsi * g0[0] + cpsi * g0[1];
        g[2] =  g0[2];
    }

    // 重力在【传感器系】下的表示 —— 力补偿与负载求解【共用】的唯一一份实现。
    // 推导与符号约定见头文件; 一句话: 参考系是法兰系 (GetPose 的 RPY), 传感器系 = 法兰系
    // 绕 z 转 +psi, 所以同一矢量在传感器系里的坐标 = Rz(-psi)·(法兰系坐标)。
    void gravitySensorFrame(const double pose[6], double g[3]) {
        double g0[3];
        flangeGravity(pose, g0);
        if (!s_yawCacheValid) refreshYawCache();   // 一次可预测分支 —— psi 只在标定后变
        rotateGravityByYaw(g0, s_cosYaw, s_sinYaw, g);
    }

    // psi 显式给出的版本 (扫描用, 不碰模块状态)。数学与上面逐位相同。
    void gravitySensorFrameAtYaw(const double pose[6], double psiDeg, double g[3]) {
        double g0[3];
        flangeGravity(pose, g0);
        const double psi = psiDeg * D2R;
        rotateGravityByYaw(g0, cos(psi), sin(psi), g);
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

    bool load(const char* filepath) {
        FILE* f = fopen(filepath, "r");
        if (!f) return false;
        char buf[1024];
        size_t n = fread(buf, 1, sizeof(buf) - 1, f);
        fclose(f);
        if (n == 0) return false;
        buf[n] = '\0';

        const char* p = strstr(buf, "\"offset\"");
        if (!p) return false;
        p = strchr(p, '[');
        if (!p) return false;
        p++;
        for (int i = 0; i < 3; i++) {
            char* end = nullptr;
            offset[i] = strtod(p, &end);
            if (end == p) return false;
            p = end;
            while (*p == ',' || *p == ' ' || *p == '\n' || *p == '\r' || *p == '\t') p++;
        }
        p = strstr(buf, "\"rmsError\"");
        if (p) {
            p = strchr(p, ':');
            if (p) rmsError = strtod(p + 1, nullptr);
        }
        enabled = true;
        return true;
    }

    bool save(const char* filepath) {
        FILE* f = fopen(filepath, "w");
        if (!f) return false;
        fprintf(f, "{\n");
        fprintf(f, "  \"offset\": [%.6g, %.6g, %.6g],\n", offset[0], offset[1], offset[2]);
        fprintf(f, "  \"rmsError\": %.6g\n", rmsError);
        fprintf(f, "}\n");
        fclose(f);
        return true;
    }

    void startCollect() {
        collectMode = true;
        collectCount = 0;
    }

    void cancelCollect() {
        collectMode = false;
        collectCount = 0;
    }
}
