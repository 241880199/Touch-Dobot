#include "InertiaIdentification.h"
#include "../calibration/TcpCalibration.h"
#include <cmath>
#include <cstdio>
#include <vector>

// 模型与坐标系的完整推导在头文件顶部。这里只重申三条实现纪律:
//   1) 重力项与惯性项【走同一个约定】—— 重力项永远是静力学模型的 b_F + A·g_s 与
//      b_M + c_s × (A·g_s), 惯性项永远是 m·a_com^ch 与 I·α^ch + ω^ch × (I·ω^ch),
//      通道坐标 = (A/m) 作用在位姿系坐标上。【不拆 A】, 不引入第二个约定。
//   2) 所有微分都是【解析】的: 先按已知频率拟合成谐波模型, 再对模型求导。
//   3) 门限全部是【比值 1】或统计量自身分布给出的, 没有一个拍定的绝对 N·m 数。

namespace InertiaIdentification {

    static const double PI_ = 3.14159265358979323846;

    static const int HARM_MAX_ORDER = 6;    // 谐波次数上限 (次数由 BIC 定, 这是搜索范围)
    // BIC 里 6n 个观测、p 个参数: 次数 k 要能被解出来, 还得给残差留自由度。
    static const int LA_MAX = 16;           // 法方程最大维数 (1 + 2·6 = 13)
    // 采样点上限: 125 Hz 下 4096 点 ≈ 32.8 s。超过就【明确拒绝】而不是悄悄截断 ——
    // 截断会让"用了多少数据"与调用方以为的不一致, 那种不一致不会报错。
    static const int MAX_SAMPLES = 4096;

    // 噪底那把尺子的两个无量纲常数 (都不是物理量的绝对门限):
    //   PLATEAU_KEEP —— 每升一阶, 残差要降到上一阶的这个比例以下, 才认为那一阶还在解释
    //     真实信号; 降不下去就是拐点, 拐点处的残差就是【带外】那一份。纯比值。
    //   LACK_OF_FIT_K —— 失配度门 1 + K·sqrt(2/dof) 里的单侧假拒率控制 (chi2/dof 的
    //     标准差约为 sqrt(2/dof))。与 PayloadCalibration::fitRaw 的 modelFormLimit 同一族:
    //     那里取 χ² 的 0.997 分位数 (≈ 均值 + 3σ), 这里用它的正态近似, 所以 K 也取 3。
    static const double PLATEAU_KEEP   = 0.9;
    static const double LACK_OF_FIT_K  = 3.0;

    // -----------------------------------------------------------------------------------
    // 小线性代数 (本模块自用; 不进任何别的模块)
    // -----------------------------------------------------------------------------------

    // 对称正定 p×p 的法方程求解 + 求逆 (Gauss-Jordan, 部分主元)。
    // A 被就地破坏; Cinv 可为 nullptr。返回 false = 主元塌了 (秩亏 -> 参数定不下来)。
    // 主元阈值取【相对的】(对 A 的最大对角元): JᵀJ 的量级随采样数、激励幅度、惯量量级一起
    // 变 (可能差好几个数量级), 绝对阈值不可能在两端同时对。
    static bool solveSym(double* A, const double* b, int p, double* x, double* Cinv) {
        if (p < 1 || p > LA_MAX) return false;
        double scale = 0.0;
        for (int i = 0; i < p; i++) if (A[i * p + i] > scale) scale = A[i * p + i];
        if (!(scale > 0.0)) return false;

        double M[LA_MAX][2 * LA_MAX];
        for (int r = 0; r < p; r++) {
            for (int c = 0; c < p; c++) M[r][c] = A[r * p + c];
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
            if (Cinv) for (int c = 0; c < p; c++) Cinv[r * p + c] = M[r][p + c];
            if (x) {
                double acc = 0.0;
                for (int c = 0; c < p; c++) acc += M[r][p + c] * b[c];
                x[r] = acc;
            }
        }
        return true;
    }

    // 对称矩阵 Jacobi 特征值 (M 被就地破坏, 只留特征值)。
    static void jacobiEigenSym(double* M, int n, double* ev) {
        for (int sweep = 0; sweep < 60; sweep++) {
            double off = 0.0, diag2 = 0.0;
            for (int i = 0; i < n; i++) {
                diag2 += M[i * n + i] * M[i * n + i];
                for (int j = i + 1; j < n; j++) off += M[i * n + j] * M[i * n + j];
            }
            if (off <= 1e-28 * (diag2 + 1e-300)) break;
            for (int p = 0; p < n; p++)
                for (int q = p + 1; q < n; q++) {
                    const double apq = M[p * n + q];
                    if (fabs(apq) <= 1e-300) continue;
                    const double theta = (M[q * n + q] - M[p * n + p]) / (2.0 * apq);
                    const double sgn = (theta >= 0.0) ? 1.0 : -1.0;
                    const double t = sgn / (fabs(theta) + sqrt(theta * theta + 1.0));
                    const double c = 1.0 / sqrt(t * t + 1.0), s = t * c;
                    for (int k = 0; k < n; k++) {
                        const double mkp = M[k * n + p], mkq = M[k * n + q];
                        M[k * n + p] = c * mkp - s * mkq;
                        M[k * n + q] = s * mkp + c * mkq;
                    }
                    for (int k = 0; k < n; k++) {
                        const double mpk = M[p * n + k], mqk = M[q * n + k];
                        M[p * n + k] = c * mpk - s * mqk;
                        M[q * n + k] = s * mpk + c * mqk;
                    }
                    M[p * n + q] = 0.0;
                    M[q * n + p] = 0.0;
                }
        }
        for (int i = 0; i < n; i++) ev[i] = M[i * n + i];
    }

    // 对称 3×3 的特征值 (升序)。
    static void symEig3(const double S[9], double ev[3]) {
        double M[9];
        for (int i = 0; i < 9; i++) M[i] = S[i];
        jacobiEigenSym(M, 3, ev);
        for (int a = 0; a < 2; a++)
            for (int b = a + 1; b < 3; b++)
                if (ev[b] < ev[a]) { const double t = ev[a]; ev[a] = ev[b]; ev[b] = t; }
    }

    // 3x3 行主序矩阵乘矢量: y = M·v
    static void matVec3(const double M[9], const double v[3], double y[3]) {
        for (int a = 0; a < 3; a++)
            y[a] = M[a * 3 + 0] * v[0] + M[a * 3 + 1] * v[1] + M[a * 3 + 2] * v[2];
    }

    // 转置矩阵乘矢量: y = Mᵀ·v  (即把基座系矢量送进位姿系)
    static void matTVec3(const double M[9], const double v[3], double y[3]) {
        for (int i = 0; i < 3; i++)
            y[i] = M[0 * 3 + i] * v[0] + M[1 * 3 + i] * v[1] + M[2 * 3 + i] * v[2];
    }

    // 坐标映射: y = W·v, W = A/m (通道坐标 = (A/m) 作用在位姿系坐标上)。
    static void channelVec(const double A[9], double m, const double v[3], double y[3]) {
        for (int a = 0; a < 3; a++)
            y[a] = (A[a * 3 + 0] * v[0] + A[a * 3 + 1] * v[1] + A[a * 3 + 2] * v[2]) / m;
    }

    static void cross3(const double a[3], const double b[3], double c[3]) {
        c[0] = a[1] * b[2] - a[2] * b[1];
        c[1] = a[2] * b[0] - a[0] * b[2];
        c[2] = a[0] * b[1] - a[1] * b[0];
    }

    static double norm3v(const double a[3]) { return sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]); }

    // -----------------------------------------------------------------------------------
    // 1) 谐波拟合 (已知频率) —— 微分的唯一来源
    // -----------------------------------------------------------------------------------
    // x(t) = c0 + Σ_{k=1..K} [ a_k·cos(kΩt) + b_k·sin(kΩt) ],  Ω = 2π·freqHz。
    // 参数排布: theta[0] = c0; theta[1+2(k-1)] = a_k; theta[2+2(k-1)] = b_k。
    static int harmDim(int K) { return 1 + 2 * K; }

    static void harmBasis(double t, double Omega, int K, double* row) {
        row[0] = 1.0;
        for (int k = 1; k <= K; k++) {
            const double ph = k * Omega * t;
            row[1 + 2 * (k - 1)] = cos(ph);
            row[2 + 2 * (k - 1)] = sin(ph);
        }
    }

    // 返回 false = 法方程奇异 (这段采样上基函数定不下来)。
    // sst = Σ(x − mean)², 即"信号"那一份; ssr = Σ 残差²。
    static bool fitHarm(const double* t, const double* x, int n, double Omega, int K,
                        double* theta, double& ssr, double& sst)
    {
        const int p = harmDim(K);
        if (n < p || p > LA_MAX) return false;

        double N[LA_MAX * LA_MAX], rhs[LA_MAX], row[LA_MAX];
        for (int i = 0; i < p * p; i++) N[i] = 0.0;
        for (int i = 0; i < p; i++) rhs[i] = 0.0;

        double mean = 0.0;
        for (int i = 0; i < n; i++) mean += x[i];
        mean /= (double)n;

        for (int i = 0; i < n; i++) {
            harmBasis(t[i], Omega, K, row);
            for (int a = 0; a < p; a++) {
                for (int b = 0; b < p; b++) N[a * p + b] += row[a] * row[b];
                rhs[a] += row[a] * x[i];
            }
        }
        if (!solveSym(N, rhs, p, theta, nullptr)) return false;

        ssr = 0.0; sst = 0.0;
        for (int i = 0; i < n; i++) {
            harmBasis(t[i], Omega, K, row);
            double pred = 0.0;
            for (int a = 0; a < p; a++) pred += row[a] * theta[a];
            const double e = pred - x[i];
            ssr += e * e;
            const double d = x[i] - mean;
            sst += d * d;
        }
        return true;
    }

    // 对【拟合出来的模型】解析求导:
    //   x(t)  = c0 + Σ [a_k cos(kΩt) + b_k sin(kΩt)]
    //   ẋ(t)  = Σ kΩ [ −a_k sin(kΩt) + b_k cos(kΩt) ]
    //   ẍ(t)  = Σ (kΩ)² [ −a_k cos(kΩt) − b_k sin(kΩt) ]
    static void harmEval(const double* theta, double t, double Omega, int K,
                         double& x, double& dx, double& ddx)
    {
        x = theta[0]; dx = 0.0; ddx = 0.0;
        for (int k = 1; k <= K; k++) {
            const double w = k * Omega, ph = w * t;
            const double a = theta[1 + 2 * (k - 1)], b = theta[2 + 2 * (k - 1)];
            const double c = cos(ph), s = sin(ph);
            x   += a * c + b * s;
            dx  += w * (-a * s + b * c);
            ddx += w * w * (-a * c - b * s);
        }
    }

    // -----------------------------------------------------------------------------------
    // 2) 姿态历史 -> ω / α (位姿系)
    // -----------------------------------------------------------------------------------
    // R = Rz(rz)·Ry(ry)·Rx(rx) (TcpCalibration::rpyToMatrix, 单位: 度)。
    // 角速度在基座系: ω_B = ẑ·ṙz + Rz·(ŷ·ṙy) + Rz·Ry·(x̂·ṙx)     (ṙ 为 rad/s)
    // 位姿系:         ω_S = Rᵀ·ω_B
    // 角加速度:       α_S = Rᵀ·ω̇_B
    //   (因为 Ṙ = [ω_B]ₓ·R ⟹ Ṙᵀ = −Rᵀ[ω_B]ₓ ⟹ Ṙᵀ·ω_B = −Rᵀ([ω_B]ₓω_B) = 0, 所以只剩 Rᵀ·ω̇_B)
    static void angularState(const double R[9], const double Rz[9], const double RzRy[9],
                             double rdot[3], double rddot[3], double Omega_S[3],
                             double Alpha_S[3], double omegaDot_B[3])
    {
        // 列矢量 e_z, Rz·e_y, Rz·Ry·e_x
        const double c1[3] = {0.0, 0.0, 1.0};
        const double c2[3] = {Rz[1], Rz[4], Rz[7]};              // Rz·ŷ
        const double c3[3] = {RzRy[0], RzRy[3], RzRy[6]};        // Rz·Ry·x̂

        double wB[3];
        for (int i = 0; i < 3; i++)
            wB[i] = c1[i] * rdot[2] + c2[i] * rdot[1] + c3[i] * rdot[0];

        // 基矢量自己的时间导数:
        //   ċ1 = 0
        //   ċ2 = ṙz·(ẑ × c2)            (Ṙz = ṙz·[ẑ]ₓ·Rz)
        //   ċ3 = ṙz·(ẑ × c3) + Rz·(ṙy·(ŷ × (Ry·x̂)))
        double zc2[3], zc3[3], RyX[3];
        // RzRy 是 Rz·Ry, 所以 Ry·x̂ = Rzᵀ·(Rz·Ry·x̂) —— 这里要的是 Ry·x̂ 本身。
        matTVec3(Rz, c3, RyX);
        const double zhat[3] = {0.0, 0.0, 1.0}, yhat[3] = {0.0, 1.0, 0.0};
        cross3(zhat, c2, zc2);
        cross3(zhat, c3, zc3);
        double yy[3];
        cross3(yhat, RyX, yy);
        double RzYy[3];
        matVec3(Rz, yy, RzYy);

        for (int i = 0; i < 3; i++)
            omegaDot_B[i] = c1[i] * rddot[2]
                          + zc2[i] * rdot[2] * rdot[1] + c2[i] * rddot[1]
                          + (zc3[i] + RzYy[i]) * rdot[2] * rdot[0] + c3[i] * rddot[0];

        matTVec3(R, wB, Omega_S);
        matTVec3(R, omegaDot_B, Alpha_S);
    }

    // -----------------------------------------------------------------------------------
    // 求解
    // -----------------------------------------------------------------------------------
    static void zeroFit(InertiaFit& out) {
        for (int i = 0; i < 9; i++) out.I[i] = 0.0;
        out.rmsForceN = 0.0; out.rmsMomentNm = 0.0; out.cond = 0.0;
        for (int i = 0; i < 6; i++) out.sigma[i] = 0.0;
        out.speedCheckRms = 0.0; out.angSpeedCheckRms = 0.0; out.ok = false;
        out.harmonicOrder = 0; out.harmonicFitRms = 0.0; out.harmonicSignalRms = 0.0;
        out.samples = 0;
        out.momentNoiseFloorNm = 0.0; out.inertiaSignalNm = 0.0;
        out.noiseFloorOrder = 0; out.noiseFloorUpperBound = false;
        out.noiseFloorStatus = NOISE_FLOOR_NO_ORDER;
        out.momentLackOfFitRatio = 0.0; out.momentLackOfFitOk = false;
        out.inertiaScale = 0.0; out.sigmaMax = 0.0;
        out.eig[0] = out.eig[1] = out.eig[2] = 0.0;
        out.triangleMargin = 0.0;
        out.kinematicsOk = false; out.determinacyOk = false;
        out.physicalOk = false; out.forceCheckOk = false;
        out.forceSignalN = 0.0;
    }

    bool identifyInertia(const InertiaSample* s, int n, double freqHz,
                         double m, const double comSensor[3],
                         const double A[9], const double bF[3], const double bM[3],
                         InertiaFit& out)
    {
        zeroFit(out);
        out.samples = n;
        if (!s || n < 4) {
            fprintf(stderr, "[Inertia] 拒绝: 采样点 %d 个, 少于最少 4 个。\n", n);
            return false;
        }
        if (!(freqHz > 0.0) || !(m > 0.0)) {
            fprintf(stderr, "[Inertia] 拒绝: freqHz=%.6g / m=%.6g 非正 —— 这两个是已知的"
                            "输入, 不是待估量。\n", freqHz, m);
            return false;
        }
        const double Omega = 2.0 * PI_ * freqHz;

        // ===== 1) 谐波拟合 + 阶数选择 (BIC) =====
        // 六个位姿通道单位不同 (mm 与 度), 直接相加的 SSR 没有意义 —— 先各自按【自己的
        // 标准差】归一化再合, 这样 BIC 里的残差是【无量纲】的, 门限 (见下) 也就无量纲。
        // ⚠ 归一化要挡住【浮点意义上的常数通道】: 一个通道恒定时 sst 可能不是精确 0 而
        //   是 ~1e-25 (求和的舍入), 于是 scale = sqrt(sst/(n−1)) ~ 1e-14, 归一化后数据
        //   变成 ~1e16 —— 法方程的量级被抬到 1e35, 消元里的舍入就足以毁掉拟合。
        //   判据取【相对该通道自身的量级】的 1e-9 (浮点鲁棒性, 不是物理门限): 低于它就是
        //   "这个通道没有变化", 尺度取 1, 它本来也不携带任何信息。
        double scaleCh[6];
        bool   liveCh[6];
        int    nLive = 0;
        for (int c = 0; c < 6; c++) {
            double mean = 0.0;
            for (int i = 0; i < n; i++) mean += s[i].pose[c];
            mean /= (double)n;
            double v = 0.0;
            for (int i = 0; i < n; i++) { const double d = s[i].pose[c] - mean; v += d * d; }
            v /= (double)(n - 1);
            const double sd = (v > 0.0) ? sqrt(v) : 0.0;
            liveCh[c] = (sd > 1e-9 * (fabs(mean) + 1.0));
            scaleCh[c] = liveCh[c] ? sd : 1.0;
            if (liveCh[c]) nLive++;
        }

        if (n > MAX_SAMPLES) {
            fprintf(stderr, "[Inertia] 拒绝: 采样点 %d 个超过上限 %d (≈%.1f s @125 Hz)。\n",
                    n, MAX_SAMPLES, (double)MAX_SAMPLES / 125.0);
            return false;
        }
        const int nUse = n;
        double tArr[MAX_SAMPLES];
        for (int i = 0; i < nUse; i++) tArr[i] = s[i].t;

        double thetaAll[6][LA_MAX];
        int    bestK = 0;
        double bestBIC = 0.0;
        bool   haveK = false;
        // BIC 只在【活的】通道上算: 常数通道既没有信息, 也不该占参数名额。
        const int M = ((nLive > 0) ? nLive : 1) * nUse;
        for (int K = 0; K <= HARM_MAX_ORDER; K++) {
            const int p = harmDim(K);
            if (nUse < p + 1) break;                  // 基函数都装不满, 再高没意义
            if (nLive * p >= M) break;                // 参数不比观测少 -> BIC 无意义
            double ssrTot = 0.0;
            bool okAll = true;
            for (int c = 0; c < 6; c++) {
                if (!liveCh[c]) continue;
                double xArr[MAX_SAMPLES];
                for (int i = 0; i < nUse; i++) xArr[i] = s[i].pose[c] / scaleCh[c];
                double ssr = 0.0, sst = 0.0;
                if (!fitHarm(tArr, xArr, nUse, Omega, K, thetaAll[c], ssr, sst)) { okAll = false; break; }
                ssrTot += ssr;
            }
            if (!okAll) continue;
            // BIC = M·ln(SSR/M) + p_total·ln(M)。SSR 归一化后无量纲, 所以这两项可比。
            double bic;
            if (ssrTot > 0.0) bic = (double)M * log(ssrTot / (double)M) + (double)(nLive * p) * log((double)M);
            else              bic = -(double)M * 1e30;      // 精确为 0: 这一阶完美, 取它
            if (!haveK || bic < bestBIC) { haveK = true; bestBIC = bic; bestK = K; }
        }
        if (!haveK) {
            fprintf(stderr, "[Inertia] 拒绝: 谐波拟合一阶都没解出来 (采样太少 / 基函数退化)。\n");
            return false;
        }
        out.harmonicOrder = bestK;

        // 用选定的阶数重跑一遍, 拿【未归一化】的系数 (尺度换算回原单位)。
        double ssrNorm = 0.0, sigNorm = 0.0;
        int liveUsed = 0;
        for (int c = 0; c < 6; c++) {
            double xArr[MAX_SAMPLES];
            for (int i = 0; i < nUse; i++) xArr[i] = s[i].pose[c];
            double ssr = 0.0, sst = 0.0;
            if (!fitHarm(tArr, xArr, nUse, Omega, bestK, thetaAll[c], ssr, sst)) {
                fprintf(stderr, "[Inertia] 拒绝: 选定阶数 %d 的谐波拟合失败。\n", bestK);
                return false;
            }
            ssrNorm += ssr / (scaleCh[c] * scaleCh[c]);
            if (liveCh[c]) {
                sigNorm += sst / (scaleCh[c] * scaleCh[c]);
                liveUsed++;
            }
        }
        // 归一化后每个活通道贡献 (n−1), 所以这两个数都在"通道自己的尺度"上, 无量纲,
        // 且【同一条口径】—— 直接比大小就是"解释掉的 vs 没解释掉的"。
        const double Mn = (double)((liveUsed > 0) ? liveUsed : 1) * (double)nUse;
        out.harmonicSignalRms = sqrt(sigNorm / Mn);
        out.harmonicFitRms = sqrt(ssrNorm / Mn);
        const double K_ORDER = bestK;

        // ===== 2) 逐采样点: 解析微分 -> 运动学量 =====
        // 全部在【通道坐标】里 (见头文件): 通道坐标 = (A/m) 作用在位姿系坐标上。
        // 这样重力项 (A·g_s) 与惯性项走的是同一个约定, 零运动时精确退化成静力学模型。
        // 逐采样点的量都放在堆上 (4096 点 × 六组 × 3 分量 ≈ 590 KB, 栈上放不下);
        // 用 vector 是为了让每个 return 路径都自动释放 —— 本模块是纯函数, 不留全局状态。
        std::vector<double> Mr((size_t)nUse * 3), Fr((size_t)nUse * 3);
        std::vector<double> aOch((size_t)nUse * 3), alphaC((size_t)nUse * 3);
        std::vector<double> omegaC((size_t)nUse * 3), acomCh((size_t)nUse * 3);
        std::vector<double> vFit((size_t)nUse * 3);
        // 角速度那一半的独立检查累加器 (拟合的 vs 自报的, 都在机体坐标系, rad/s)
        double dAng2 = 0.0, wFit2 = 0.0, wRep2 = 0.0;

        for (int i = 0; i < nUse; i++) {
            const InertiaSample& smp = s[i];
            double pos_mm[3], acc_m[3], rpy_deg[3], rdot[3], rddot[3];
            for (int c = 0; c < 3; c++) {
                double x = 0.0, dx = 0.0, ddx = 0.0;
                harmEval(thetaAll[c], smp.t, Omega, bestK, x, dx, ddx);
                pos_mm[c] = x;                     // 拟合位姿 (mm) —— 见下面重力项的说明
                acc_m[c] = ddx / 1000.0;           // mm/s^2 -> m/s^2
                vFit[(size_t)i * 3 + c] = dx;      // mm/s —— 与 TCPSpeedActual 同单位
            }
            for (int c = 0; c < 3; c++) {
                double x = 0.0, dx = 0.0, ddx = 0.0;
                harmEval(thetaAll[3 + c], smp.t, Omega, bestK, x, dx, ddx);
                rpy_deg[c] = x;
                rdot[c]    = dx * PI_ / 180.0;     // rad/s
                rddot[c]   = ddx * PI_ / 180.0;    // rad/s^2
            }

            double R[9], Rz[9], Ry[9], RzRy[9];
            TcpCalibration::rpyToMatrix(rpy_deg[0], rpy_deg[1], rpy_deg[2], R);
            TcpCalibration::rpyToMatrix(0.0, 0.0, rpy_deg[2], Rz);
            TcpCalibration::rpyToMatrix(0.0, rpy_deg[1], 0.0, Ry);
            for (int r = 0; r < 3; r++)            // RzRy = Rz·Ry
                for (int c = 0; c < 3; c++) {
                    double acc = 0.0;
                    for (int k = 0; k < 3; k++) acc += Rz[r * 3 + k] * Ry[k * 3 + c];
                    RzRy[r * 3 + c] = acc;
                }

            double OmS[3], AlS[3], wDotB[3];
            angularState(R, Rz, RzRy, rdot, rddot, OmS, AlS, wDotB);

            // 角速度那一半的独立检查: 把机器人自报的 RPY 变化率 (度/s) 用【同一个映射】送进
            // 机体坐标系 —— 两边走同一条换算, 差的就只剩"两个来源报的运动是否一致"。
            double rdotRep[3], rddotZero[3] = {0.0, 0.0, 0.0};
            for (int c = 0; c < 3; c++) rdotRep[c] = smp.speed[3 + c] * PI_ / 180.0;
            double wRepS[3], scratchA[3], scratchB[3];
            angularState(R, Rz, RzRy, rdotRep, rddotZero, wRepS, scratchA, scratchB);
            for (int c = 0; c < 3; c++) {
                const double e = OmS[c] - wRepS[c];
                dAng2 += e * e;
                wFit2 += OmS[c] * OmS[c];
                wRep2 += wRepS[c] * wRepS[c];
            }

            double aOS[3];
            matTVec3(R, acc_m, aOS);               // a_O 在位姿系
            channelVec(A, m, aOS, &aOch[(size_t)i * 3]);     // -> 通道坐标
            channelVec(A, m, OmS, &omegaC[(size_t)i * 3]);
            channelVec(A, m, AlS, &alphaC[(size_t)i * 3]);

            // a_com^ch = a_O^ch + α^ch × c_s + ω^ch × (ω^ch × c_s)   (c_s 本来就在通道坐标里)
            double t1[3], t2[3], t3[3];
            cross3(&alphaC[(size_t)i * 3], comSensor, t1);
            cross3(&omegaC[(size_t)i * 3], comSensor, t2);
            cross3(&omegaC[(size_t)i * 3], t2, t3);
            for (int a = 0; a < 3; a++)
                acomCh[(size_t)i * 3 + a] = aOch[(size_t)i * 3 + a] + t1[a] + t3[a];

            // 重力项: 静态模型的同一个 g —— gravitySensorFrameAtYaw(pose, 0.0, ·), psi 恒为 0。
            // 【用【拟合】位姿算 g, 不用原始位姿】: 本模块的模型只有【一个】—— 那条谐波轨迹。
            // 惯性项全部由它导出, 重力项若改用带噪的原始位姿, 就等于往"已知量"里掺进一份
            // 别处都没有的噪声, 残差里会多出一块与模型无关的量。
            // 零运动时拟合位姿与原始位姿【逐位相同】(常数模型), 所以静力学一致性检验不受
            // 这一选择影响 —— 那一条仍然钉死的是坐标约定, 不是平滑程度。
            double pose6[6] = {pos_mm[0], pos_mm[1], pos_mm[2],
                               rpy_deg[0], rpy_deg[1], rpy_deg[2]};
            double gs[3];
            TcpCalibration::gravitySensorFrameAtYaw(pose6, 0.0, gs);
            double Ag[3];
            matVec3(A, gs, Ag);                    // A·g_s —— 通道坐标里的重力项

            // 力残差: F_meas − b_F − A·g_s
            for (int a = 0; a < 3; a++)
                Fr[(size_t)i * 3 + a] = smp.wrench[a] - bF[a] - Ag[a];
            // 力矩残差: M_meas − b_M − c_s × (A·g_s) − c_s × (A·a_O^S)
            double Ao[3];
            matVec3(A, aOS, Ao);                   // = m·a_O^ch (恒等, 与 W 是否正交无关)
            double cg[3], ca[3];
            cross3(comSensor, Ag, cg);
            cross3(comSensor, Ao, ca);
            for (int a = 0; a < 3; a++)
                Mr[(size_t)i * 3 + a] = smp.wrench[3 + a] - bM[a] - cg[a] - ca[a];
        }

        // ===== 3) 力矩方程对 I_O 的线性最小二乘 =====
        //   M_res = I·α + ω × (I·ω),  未知 x = [Ixx, Iyy, Izz, Ixy, Ixz, Iyz]
        // 展开 (按分量写, 不靠任何矩阵库):
        //   (Iα)_x = Ixx·αx + Ixy·αy + Ixz·αz
        //   (ω×(Iω))_x = ωy·(Iω)_z − ωz·(Iω)_y,  (Iω)_k = I_kx·ωx + I_ky·ωy + I_kz·ωz
        const int PI6 = 6;
        double JtJ[36], Jtb[6];
        for (int i = 0; i < 36; i++) JtJ[i] = 0.0;
        for (int i = 0; i < 6; i++) Jtb[i] = 0.0;

        for (int i = 0; i < nUse; i++) {
            const double* w = &omegaC[(size_t)i * 3];
            const double* al = &alphaC[(size_t)i * 3];
            const double wx = w[0], wy = w[1], wz = w[2];
            const double ax = al[0], ay = al[1], az = al[2];
            const double* mr = &Mr[(size_t)i * 3];
            double row[3][6];
            // 分量 0 (x): (Iα)_x + ωy(Iω)_z − ωz(Iω)_y
            row[0][0] = ax;             row[0][1] = -wz * wy;            row[0][2] = wy * wz;
            row[0][3] = ay - wz * wx;   row[0][4] = az + wy * wx;        row[0][5] = wy * wy - wz * wz;
            // 分量 1 (y): (Iα)_y + ωz(Iω)_x − ωx(Iω)_z
            row[1][0] = wz * wx;        row[1][1] = ay;                  row[1][2] = -wx * wz;
            row[1][3] = ax + wz * wy;   row[1][4] = wz * wz - wx * wx;   row[1][5] = az - wx * wy;
            // 分量 2 (z): (Iα)_z + ωx(Iω)_y − ωy(Iω)_x
            row[2][0] = -wy * wx;       row[2][1] = wx * wy;             row[2][2] = az;
            row[2][3] = wx * wx - wy * wy;                               row[2][4] = ax - wy * wz;
            row[2][5] = ay + wx * wz;

            for (int a = 0; a < 3; a++) {
                for (int c = 0; c < PI6; c++) {
                    for (int e = 0; e < PI6; e++) JtJ[c * PI6 + e] += row[a][c] * row[a][e];
                    Jtb[c] += row[a][c] * mr[a];
                }
            }
        }

        // cond(J) = sqrt(λmax/λmin) of JᵀJ —— 秩亏/病态在这里就暴露
        double JtJcopy[36];
        for (int i = 0; i < 36; i++) JtJcopy[i] = JtJ[i];
        double lam[6];
        jacobiEigenSym(JtJcopy, 6, lam);
        double lamMin = lam[0], lamMax = lam[0];
        for (int i = 1; i < 6; i++) {
            if (lam[i] < lamMin) lamMin = lam[i];
            if (lam[i] > lamMax) lamMax = lam[i];
        }
        // 【线性层失败不提前 return】—— 头文件明确让调用方在 false 之后读诊断字段,
        // 所以下面一路走到底, 只是把 I 置零、把判据置假。提前 return 会把残差/cond 留在
        // 调用方栈上的旧内容里, 读到的就是垃圾 (PayloadCalibration 栽过同一个坑)。
        double sol[6] = {0, 0, 0, 0, 0, 0}, Cinv[36];
        bool rankOk = (lamMin > 0.0);
        bool solved = false;
        if (rankOk) {
            out.cond = sqrt(lamMax / lamMin);       // 数值上奇异也报出来 (不是 0)
            if ((lamMax / lamMin) < 1.0e12) solved = solveSym(JtJ, Jtb, PI6, sol, Cinv);
        } else {
            // σmin 非正 -> cond 数学上是无穷。从前这里留 0, 打印出来是 cond=0, 读起来正好
            // 是"完美条件数", 与"秩亏"说反了 (评审指出的那一处)。
            out.cond = HUGE_VAL;
        }
        if (!rankOk || !solved) {
            for (int k = 0; k < 6; k++) sol[k] = 0.0;   // 诊断用: 残差就退化成 |M_res| 本身
            for (int k = 0; k < 36; k++) Cinv[k] = 0.0;
            if (!rankOk)
                fprintf(stderr, "[Inertia] 线性层失败: 力矩设计矩阵秩亏 (JᵀJ 最小特征值 "
                                "%.6g ≤ 0) —— 这段激励动不到全部六个分量 (单轴往复不够)。\n", lamMin);
            else
                fprintf(stderr, "[Inertia] 线性层失败: 力矩法方程的主元塌了 (cond=%.4g) ——"
                                " 六个分量定不下来。\n", out.cond);
        }

        // 对称张量装回 9 元 (行主序): Ixx Ixy Ixz / Ixy Iyy Iyz / Ixz Iyz Izz
        const int IX = 0, IY = 1, IZ = 2, IXY = 3, IXZ = 4, IYZ = 5;
        double I9[9] = { sol[IX],  sol[IXY], sol[IXZ],
                         sol[IXY], sol[IY],  sol[IYZ],
                         sol[IXZ], sol[IYZ], sol[IZ] };

        // ===== 3b) 力矩通道的【噪底】—— σ 的尺子必须与 I_O 无关 =====
        // 为什么不能用拟合残差当尺子: 残差 = 噪声 + 模型形式错。拿它当尺子, 模型越错尺子越松,
        // 门对着自己放水 (PayloadCalibration::fitRaw 为这个自指病返工过三轮)。这里换一把
        // 【不经过模型】的尺子: 把【原始】力矩通道拟合到更高阶的谐波上 —— 那个拟合里根本
        // 没有 I_O, 所以模型形式错【不可能】把它抬高。
        // ⚠ 有限阶的诚实说明 (别把估计当成本身): 力矩模型是 ω 的二次型, 而 ω 又由 RPY 的
        //   非线性换算而来, 它的谱【不是】有限的 —— 无噪声时逐阶残差实测是
        //   3.8e-4 -> 6.4e-5 -> 4.4e-6 -> 5.8e-7, 一直降。所以阶数取到【残差不再明显下降】
        //   为止 (拐点由数据自己给出, 没有任何绝对 N·m 门限); 若一直降到底, 就把最高阶的
        //   残差记下来并把 noiseFloorUpperBound 置真 —— 那里面还混着带内信号, 尺子偏松。
        //   这一条写进头文件的"必要但不充分", 不在这里假装它是纯噪底。
        double noiseFloor = 0.0;
        int    noiseOrder = 0;
        bool   noiseUpperBound = false;
        int    noiseFail = NOISE_FLOOR_NO_ORDER;
        {
            const int q0 = bestK + 1;          // 与模型同阶的残差里一定还留着二次项的带内信号
            double rPrev = -1.0;
            if (q0 <= HARM_MAX_ORDER) {
                for (int q = q0; q <= HARM_MAX_ORDER; q++) {
                    double ss = 0.0;
                    bool okQ = true;
                    double thetaTmp[LA_MAX];
                    for (int c = 0; c < 3; c++) {
                        double xArr[MAX_SAMPLES];
                        for (int i = 0; i < nUse; i++) xArr[i] = s[i].wrench[3 + c];
                        double ssr = 0.0, sst = 0.0;
                        if (!fitHarm(tArr, xArr, nUse, Omega, q, thetaTmp, ssr, sst)) { okQ = false; break; }
                        ss += ssr;
                    }
                    if (!okQ) { noiseFail = NOISE_FLOOR_NO_SAMPLES; break; }
                    const double r = sqrt(ss / (3.0 * nUse));
                    if (rPrev >= 0.0 && r > PLATEAU_KEEP * rPrev) {
                        // 再加一阶也降不下去了 -> 上一阶的残差就是带外的那一份
                        noiseOrder = q - 1;
                        noiseFloor = rPrev;
                        noiseUpperBound = false;
                        noiseFail = NOISE_FLOOR_OK;
                        break;
                    }
                    rPrev = r;
                    noiseOrder = q;
                    noiseFloor = r;
                    noiseUpperBound = true;    // 暂时; 找到拐点会被改回来
                    noiseFail = NOISE_FLOOR_OK;
                }
            }
        }
        if (noiseFail == NOISE_FLOOR_OK && !(noiseFloor > 0.0)) {
            noiseFail = NOISE_FLOOR_DEGENERATE;   // 通道冻住 / 完美拟合: 尺子不是正数, 做分母没有意义
            noiseFloor = 0.0;
        }
        out.noiseFloorStatus = noiseFail;
        if (noiseFail == NOISE_FLOOR_OK) {
            out.momentNoiseFloorNm = noiseFloor;
            out.noiseFloorOrder = noiseOrder;
            out.noiseFloorUpperBound = noiseUpperBound;
        } else {
            fprintf(stderr, "[Inertia] 尺子不齐(%s): 力矩通道的带外噪底量不出来 —— %s。"
                            " 没有这把与模型无关的尺子, 确定性与失配度都无从判, 因此【拒绝给参数】。\n",
                    (noiseFail == NOISE_FLOOR_NO_ORDER) ? "阶数不够" :
                    (noiseFail == NOISE_FLOOR_NO_SAMPLES) ? "采样不够" : "估计退化",
                    (noiseFail == NOISE_FLOOR_NO_ORDER)
                        ? "模型阶数已经是搜索上限, 再往上没有阶可用来量带外"
                        : (noiseFail == NOISE_FLOOR_NO_SAMPLES)
                          ? "采样点数撑不起更高阶的谐波拟合"
                          : "估计值非正 (通道冻住, 或数据被拟合到完美)");
        }

        // ===== 4) 残差与不确定度 =====
        double ssM = 0.0, sigI2 = 0.0, sigF2 = 0.0, ssF = 0.0;
        for (int i = 0; i < nUse; i++) {
            const double* w = &omegaC[(size_t)i * 3];
            const double* al = &alphaC[(size_t)i * 3];
            const double wx = w[0], wy = w[1], wz = w[2];
            const double ax = al[0], ay = al[1], az = al[2];
            const double Iwx = I9[0] * wx + I9[1] * wy + I9[2] * wz;
            const double Iwy = I9[3] * wx + I9[4] * wy + I9[5] * wz;
            const double Iwz = I9[6] * wx + I9[7] * wy + I9[8] * wz;
            const double pred[3] = { I9[0] * ax + I9[1] * ay + I9[2] * az + (wy * Iwz - wz * Iwy),
                                     I9[3] * ax + I9[4] * ay + I9[5] * az + (wz * Iwx - wx * Iwz),
                                     I9[6] * ax + I9[7] * ay + I9[8] * az + (wx * Iwy - wy * Iwx) };
            double m2 = 0.0;
            for (int a = 0; a < 3; a++) {
                const double e = pred[a] - Mr[(size_t)i * 3 + a];
                ssM += e * e;
                m2 += pred[a] * pred[a];
            }
            sigI2 += m2;
            double f2 = 0.0;
            for (int a = 0; a < 3; a++) {
                const double fpred = m * acomCh[(size_t)i * 3 + a];
                const double e = fpred - Fr[(size_t)i * 3 + a];
                ssF += e * e;
                f2 += fpred * fpred;
            }
            sigF2 += f2;
        }
        // σ 的尺度【不取拟合残差】: 残差 = 噪声 + 模型形式错, 拿它当 s² 就是自指 ——
        // 模型越错, s² 越大, σ 越大, "不确定度 < 张量量级"那一关反而越松。以前这里正是
        // 这么写的 (评审实测: σ 从 2.6e-18 涨到 2.0e-6, 而解已经错了 4~35σ)。
        // 尺度只从下面那把【不经过模型】的尺子来; 尺子不齐时 σ = 0 且整体拒绝。
        const double s2 = out.momentNoiseFloorNm * out.momentNoiseFloorNm;
        out.rmsMomentNm = sqrt(ssM / (3.0 * nUse));
        out.rmsForceN   = sqrt(ssF / (3.0 * nUse));
        out.inertiaSignalNm = sqrt(sigI2 / (3.0 * nUse));
        out.forceSignalN    = sqrt(sigF2 / (3.0 * nUse));
        for (int k = 0; k < 6; k++) out.sigma[k] = sqrt(Cinv[k * PI6 + k] * s2);
        for (int k = 0; k < 6; k++) if (out.sigma[k] > out.sigmaMax) out.sigmaMax = out.sigma[k];
        out.inertiaScale = sqrt(sol[0] * sol[0] + sol[1] * sol[1] + sol[2] * sol[2]
                              + 2.0 * (sol[3] * sol[3] + sol[4] * sol[4] + sol[5] * sol[5]));

        // ===== 5) 运动学检查 (独立来源) =====
        // 拟合出来的速度 vs 机器人自己报的 TCPSpeedActual。两半都比:
        //   · 线性三分量: 直接比 (vFit 与 speed[0..2] 同单位, mm/s)。
        //   · 角速度三分量: 【两边都走同一个 RPY 变化率 -> 机体角速度的换算】再比。
        //     "RPY 变化率 != 机体角速度" 是【换算】, 不是"跳过这一半"的理由 —— 力矩方程
        //     吃掉的正是这一半, 它没有独立的第二个来源, 所以这个换算过的交叉检查才有价值。
        //     换算在下面的主循环里就地做 (用同一个 R/Rz/RzRy), 比的是同一个物理量。
        double d2 = 0.0, vf2 = 0.0, vr2 = 0.0;
        for (int i = 0; i < nUse; i++) {
            double dd = 0.0;
            for (int c = 0; c < 3; c++) {
                const double vf = vFit[(size_t)i * 3 + c];
                const double e = vf - s[i].speed[c];
                dd += e * e;
                vf2 += vf * vf;
                vr2 += s[i].speed[c] * s[i].speed[c];
            }
            d2 += dd;
        }
        out.speedCheckRms = sqrt(d2 / (3.0 * nUse));
        const double vFitRms = sqrt(vf2 / (3.0 * nUse));
        const double vRepRms = sqrt(vr2 / (3.0 * nUse));
        // 角速度那一半 (rad/s 里做比较, 报出来换成度/s, 与 TCPSpeedActual 的角分量同单位)
        const double R2D = 180.0 / PI_;
        out.angSpeedCheckRms = sqrt(dAng2 / (3.0 * nUse)) * R2D;
        const double wFitRms = sqrt(wFit2 / (3.0 * nUse)) * R2D;
        const double wRepRms = sqrt(wRep2 / (3.0 * nUse)) * R2D;

        // ===== 6) 门限 —— 全部是【比值 1】或统计量自身分布给出的 =====
        //
        // ⚠⚠ 【这些门是【必要而不充分】的】。评审用一个独立探针证明了这一点: 让通道响应
        //    A/m 带上真实的 6.5% 各向异性 (此时【不存在】任何对称 I_O 能让模型精确成立),
        //    解已经错了, 而【每一道门都是绿的, ok=1】。原因见头文件 "通道约定"那一段,
        //    以及下面 (f) 的尺子说明。判据通过 != 惯量可信。
        //
        // (a) 运动学: 两个【独立】来源必须对得上 (线性三分量 + 角三分量), 且对得比它们自己
        //     都小 —— "差得比信号还大"就意味着至少有一个根本没在量这个运动。尺度取两者的
        //     【较小者】(保守方向): 任一路变小 (拟合没拟合上, 或机器人报的速度不对), 门立刻收紧。
        // (b) 谐波拟合: 已知频率的谐波模型解释掉的, 必须比它没解释掉的多。
        // (c) 确定性: 【分层说清楚, 不假装是比值 1】。
        //       · 真正在干活的是 solved (秩 + cond): 激励动不到六个分量时由它拒绝。
        //       · σmax < 张量量级 这一条【几乎不可能触发】—— 评审实测 σmax/||I|| 是 1.2e-4,
        //         要触发得让力矩信号比惯量量级小 17 倍。留着它是因为它【在一阶上】正好是
        //         "解出来的数是不是噪声"这句话, 但别把它当主力。
        //       · 加一条【真的会咬】的: 力矩通道的信噪比 —— 运动产生的惯性力矩信号必须
        //         高过实测噪底, 否则整段数据就没有惯量信息可言。
        //     ⚠ 这里 σ 的尺度【不再是拟合残差】(那是自指: 模型越错 σ 越大, 门越松),
        //       而是 3b) 那把与模型无关的尺子 (高阶带外残差)。
        // (d) 物理: 绕任一点的惯量张量【必然正定】。
        //     这一条同时是【符号约定的护栏】: 力矩通道的惯性项若差一个负号, 解出来的是 −I_O,
        //     立刻非正定 -> 拒绝, 而不是报一个"看着合理"的负数张量。
        //     ⚠ 主惯量的【三角不等式】(I1+I2 >= I3) 对真张量同样成立, 但它【不作门限】:
        //       它是二阶性质, 噪声可以让一个对的解略微越界。照实报 margin (见头文件里
        //       "怎么用它"那一段: 与 CAD 对照要先补齐平行轴平移, 或只比绕质心的主惯量)。
        // (e) 力通道 (运动学检查, 不含 I_O): 残差必须小于预测的惯性力信号。
        // (f) 失配度: 模型残差必须与【那把与模型无关的尺子】同量级 —— 这才是有可能真的
        //     抓到"模型形式错"的那一道。判据形如 χ²/dof <= 1 + K·sqrt(2/dof), K 是单侧
        //     假拒率控制, 与 fitRaw 同一族。
        //     实测 (单测 test_anisotropic_response_is_flagged): 细长笔 + 真实 6.5% 各向异性,
        //     张量差 1.4%, 力矩残差是噪底的 13 倍 -> 这一道亮红。同样的数据一旦带上实测
        //     噪声, 那点残差签名就沉到噪底下面 (比值回到 1.00), 抓不到了 —— 这就是"必要
        //     而不充分"的实测边界, 也是评审那个全绿探针的成因。
        //     尺子不齐 -> 不设这道门而是【整体拒绝】(见 3b), 不给"没验过"留后门。
        const double vRef = (vFitRms < vRepRms) ? vFitRms : vRepRms;
        const double wRef = (wFitRms < wRepRms) ? wFitRms : wRepRms;
        out.kinematicsOk = (out.speedCheckRms < vRef) && (out.angSpeedCheckRms < wRef)
                        && (out.harmonicFitRms < out.harmonicSignalRms);
        const int dofFloor = 3 * nUse - 3 * harmDim(out.noiseFloorOrder);
        const double lackBound = 1.0 + LACK_OF_FIT_K * sqrt(2.0 / (double)((dofFloor > 1) ? dofFloor : 1));
        out.momentLackOfFitRatio = (out.momentNoiseFloorNm > 0.0)
                                 ? (out.rmsMomentNm / out.momentNoiseFloorNm) : 0.0;
        out.momentLackOfFitOk = (out.noiseFloorStatus == NOISE_FLOOR_OK)
                             && (out.momentLackOfFitRatio < lackBound);
        out.determinacyOk = solved && (out.inertiaScale > 0.0) && (out.sigmaMax > 0.0)
                         && (out.sigmaMax < out.inertiaScale)
                         && (out.inertiaSignalNm > out.momentNoiseFloorNm);

        double ev[3];
        symEig3(I9, ev);
        out.eig[0] = ev[0]; out.eig[1] = ev[1]; out.eig[2] = ev[2];
        out.triangleMargin = ev[0] + ev[1] - ev[2];
        out.physicalOk = solved && (ev[0] > 0.0) && (ev[1] > 0.0) && (ev[2] > 0.0);
        out.forceCheckOk = (out.forceSignalN > 0.0) && (out.rmsForceN < out.forceSignalN);

        // 照实填输出 (即使拒绝, 调用方也要能读到诊断) —— 但 ok 只在全过时置 true。
        for (int i = 0; i < 9; i++) out.I[i] = I9[i];
        out.ok = out.kinematicsOk && out.determinacyOk && out.physicalOk
              && out.forceCheckOk && out.momentLackOfFitOk;

        // ===== 打印 (拒绝的时候比通过的时候更需要看得见) =====
        fprintf(stderr, "[Inertia] 谐波阶数 K=%d (BIC 从 0..%d 里选, 通道按各自标准差归一化):"
                        " 解释 %.4g / 剩 %.4g (无量纲 rms), 采样 %d 点 @ %.3g Hz\n",
                (int)K_ORDER, HARM_MAX_ORDER, out.harmonicSignalRms, out.harmonicFitRms,
                nUse, freqHz);
        fprintf(stderr, "[Inertia] 运动学检查(独立来源): 拟合速度 rms %.4g mm/s vs "
                        "TCPSpeedActual rms %.4g mm/s -> 差 %.4g mm/s (门限 = 两者较小的 %.4g)\n",
                vFitRms, vRepRms, out.speedCheckRms, vRef);
        fprintf(stderr, "[Inertia] 运动学检查(独立来源, 角速度那一半, 两边都过 rpyToBody): "
                        "拟合 rms %.4g 度/s vs TCPSpeedActual rms %.4g 度/s -> 差 %.4g 度/s"
                        " (门限 = 两者较小的 %.4g)\n",
                wFitRms, wRepRms, out.angSpeedCheckRms, wRef);
        fprintf(stderr, "[Inertia] I_O = [% .6g % .6g % .6g; % .6g % .6g % .6g; % .6g % .6g % .6g]"
                        " kg·m^2   (cond=%.4g)\n",
                I9[0], I9[1], I9[2], I9[3], I9[4], I9[5], I9[6], I9[7], I9[8], out.cond);
        fprintf(stderr, "[Inertia] σ = [Ixx %.3g, Iyy %.3g, Izz %.3g, Ixy %.3g, Ixz %.3g, Iyz %.3g],"
                        " σmax %.3g vs 张量量级 %.3g\n",
                out.sigma[0], out.sigma[1], out.sigma[2], out.sigma[3], out.sigma[4], out.sigma[5],
                out.sigmaMax, out.inertiaScale);
        fprintf(stderr, "[Inertia] 主惯量 (升序) %.6g / %.6g / %.6g kg·m^2, 三角不等式余量 %.3g\n",
                ev[0], ev[1], ev[2], out.triangleMargin);
        fprintf(stderr, "[Inertia] 残差: 力矩 %.4g N·m (惯性力矩信号 %.4g N·m, 带外噪底 %.4g N·m"
                        " = 力矩通道到 %d 阶谐波拟合的带外残差%s), 力 %.4g N (惯性力信号 %.4g N)\n",
                out.rmsMomentNm, out.inertiaSignalNm, out.momentNoiseFloorNm, out.noiseFloorOrder,
                out.noiseFloorUpperBound ? " (⚠ 阶数搜到底仍在降: 里面还混着带内信号, 这把尺子偏松)" : "",
                out.rmsForceN, out.forceSignalN);
        fprintf(stderr, "[Inertia] 失配度: 力矩残差 / 噪底 = %.4g (门 %.4g = 1 + %.1f·sqrt(2/%d))\n",
                out.momentLackOfFitRatio, lackBound, LACK_OF_FIT_K, dofFloor);

        if (!out.ok) {
            if (!out.kinematicsOk)
                fprintf(stderr, "[Inertia] 自检拒绝(运动学): 拟合速度与 TCPSpeedActual 的差 "
                                "%.4g mm/s 不小于两者的较小者 %.4g mm/s, 或角速度那一半的差 "
                                "%.4g 度/s 不小于 %.4g 度/s, 或谐波模型剩得比解释得多"
                                " (%.4g >= %.4g) —— 这段数据不是【已知频率的那个运动】, 微分不可信。\n",
                        out.speedCheckRms, vRef, out.angSpeedCheckRms, wRef,
                        out.harmonicFitRms, out.harmonicSignalRms);
            if (!out.determinacyOk)
                fprintf(stderr, "[Inertia] 自检拒绝(激发不足): σmax %.4g vs 张量量级 %.4g kg·m^2,"
                                " 惯性力矩信号 %.4g N·m vs 噪底 %.4g N·m (尺子状态 %d)"
                                " —— 参数定不下来, 或运动产生的惯性力矩压不过实测噪底。\n",
                        out.sigmaMax, out.inertiaScale, out.inertiaSignalNm,
                        out.momentNoiseFloorNm, (int)out.noiseFloorStatus);
            if (!out.momentLackOfFitOk)
                fprintf(stderr, "[Inertia] 自检拒绝(失配度): 力矩残差 %.4g N·m / 噪底 %.4g N·m = %.4g"
                                " ≥ 门 %.4g —— 残差里除了噪声还有别的东西, 也就是【模型形式对不上】"
                                " (通道约定各向异性、运动学量算错、或力矩通道的别的干扰)。\n",
                        out.rmsMomentNm, out.momentNoiseFloorNm, out.momentLackOfFitRatio, lackBound);
            if (!out.physicalOk)
                fprintf(stderr, "[Inertia] 自检拒绝(非物理): 主惯量 %.4g / %.4g / %.4g kg·m^2"
                                " —— 绕一点的惯量张量必【正定】; 主惯量出现非正值, 多半是"
                                " 力矩通道的惯性项符号约定反了 (解出来的是 −I_O)。\n",
                        ev[0], ev[1], ev[2]);
            if (!out.forceCheckOk)
                fprintf(stderr, "[Inertia] 自检拒绝(运动学, 力通道): 力残差 %.4g N ≥ 惯性力信号"
                                " %.4g N —— 传感器看到的惯性力解释不了实测残差。\n",
                        out.rmsForceN, out.forceSignalN);
            fprintf(stderr, "[Inertia] 【拒绝给出惯量】: 上面的 I_O 只供诊断, 不得下发给机械臂。\n");
            return false;
        }
        fprintf(stderr, "[Inertia] 全部自检通过。⚠ 这是绕【传感器测量原点】的 I_O;"
                        " 发机械臂要的是绕法兰面的, 中间两次平行轴平移必须显式做 (后续任务)。\n");
        return true;
    }

} // namespace InertiaIdentification
