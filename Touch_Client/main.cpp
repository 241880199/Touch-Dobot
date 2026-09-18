#include "config/glut_fix.h"
#include <winsock2.h>
#include <ws2tcpip.h>
#include <HD/hd.h>
#include <HDU/hduVector.h>
#include <iostream>
#include <cmath>
#include <conio.h>

#include "config/Config.h"
#include "core/AppState.h"
#include "core/CalibStore.h"
#include "haptic/HapticDevice.h"
#include "relay/RelayCore.h"
#include "render/SceneRenderer.h"
#include "safety/RobotDiagnostics.h"
#include "calibration/CalibrationSolver.h"
#include "calibration/TcpCalibration.h"
#include "force/ForceCalibration.h"
#include "force/ForceCompensation.h"
#include "force/PayloadCalibration.h"
#include "robot/Kinematics.h"
#include <cstdio>

// ===== 运行模式 =====
static bool g_noRobot = false;
static bool g_noTouch = false;

// ===== FK 实机验证状态 =====
namespace FkValidate {
    static const int MAX_POINTS = 20;
    static bool mode = false;
    static int count = 0;
    static double joints[20][6];
    static double actualPos[20][3];
    static char labels[20][64];
}

// ===== 多姿态零偏检查 (验证 EnableRobot 末端负载参数) =====
// 负载设对 → 空载时 raw 力与姿态无关, 各姿态零偏一致 (极差≈噪声);
// 负载设错 → 残余重力随姿态变化, 零偏随姿态漂移。
// 用法: 'm' 进入 → 笔尖悬空、只改姿态(位置尽量不变), 每到一个姿态按 SPACE
//       (采样 1s) → 'm' 退出并输出报告。
namespace BiasCheck {
    static const int   MAX_POSES  = 12;
    static const DWORD AVG_MS     = 1000;   // 每个姿态的采样时长
    static const int   MIN_SAMPLES = 10;    // 少于这个数说明数据流有问题, 本次作废
    static const double MIN_TILT_SIN = 0.5; // 姿态倾角覆盖下限 (sinθ)
    static const double MIN_SPAN_DEG = 30.0;// 姿态之间至少差这么多, 否则极差只是噪声

    static bool  mode = false;
    static int   count = 0;
    static bool  sampling = false;
    static DWORD sampleStartMs = 0;
    static DWORD lastSampleMs = 0;
    static int   avgCount = 0;
    static double accum[6];
    static double pose[MAX_POSES][6];    // Rx,Ry,Rz (deg) + X,Y,Z (mm)
    static double bias[MAX_POSES][6];    // 该姿态平均 raw 力/力矩

    // 数据只有在「机械臂配置 == 采集时的配置」时才可用于复验。
    // 求解下发了新负载 -> 已采数据作废 (拒绝 report 判定)。
    static bool dataUnderCurrentPayload = true;

    // 连续多少次求解被判"不合理"。达到 Config::CALIB_MAX_CONSECUTIVE_FAILS 后锁住 's'。
    static int  consecutiveFails = 0;
    static bool solveLocked = false;

    static void reset() {
        dataUnderCurrentPayload = true;
        consecutiveFails = 0;
        solveLocked = false;
        count = 0;
        sampling = false;
        avgCount = 0;
        for (int i = 0; i < 6; i++) accum[i] = 0.0;
    }

    // 静默退出 (被其他采集模式抢占时调用, 丢弃已采姿态)
    static void cancel() {
        if (!mode) return;
        mode = false;
        sampling = false;
        // 退出时一定要关拖拽: 柔顺状态漏出去, 机械臂会一直软着
        RelayCore::instance().setDragMode(false);
        std::cout << "[BIAS] Mode OFF (" << count << " poses discarded)" << std::endl;
    }

    // SPACE: 开始对该姿态采样
    static void record() {
        if (sampling) {
            std::cout << "[BIAS] 正在采样中, 保持静止" << std::endl;
            return;
        }
        // 上一批数据是在旧负载下采的 -> 从这里开始算新一批, 旧的全部丢弃
        // 复用 reset(), 免得将来 reset() 加了字段而这里漏跟。
        if (!dataUnderCurrentPayload) {
            std::cout << "[BIAS] 上一批数据是在旧负载下采的, 已丢弃 — 开始新一批采集"
                      << std::endl;
            reset();
        }
        if (count >= MAX_POSES) {
            std::cout << "[BIAS] 已达 " << MAX_POSES << " 个姿态, 按 'm' 输出报告" << std::endl;
            return;
        }
        for (int i = 0; i < 6; i++) accum[i] = 0.0;
        avgCount = 0;
        lastSampleMs = 0;
        sampleStartMs = GetTickCount();
        sampling = true;
        std::cout << "[BIAS] 采样 " << AVG_MS << "ms — 保持静止..." << std::endl;
    }

    // 每帧调用 (idle)。注意 idle 跑得比力数据快: pollForce 自我节流到 33ms,
    // 所以必须靠 lastUpdateMs 去重, 否则同一份数据会被反复累加 (均值不变但
    // 降噪完全失效), 采样窗口也会比标称短。
    static void sample(const AppState::ForceData& fd) {
        if (!mode || !sampling) return;
        DWORD now = GetTickCount();

        if (fd.isStale) {
            if (now - sampleStartMs > AVG_MS * 3) {   // 数据流断了, 别卡死
                sampling = false;
                std::cout << "[BIAS] 采样超时 (力数据中断), 本次作废" << std::endl;
            }
            return;
        }
        if (fd.lastUpdateMs == lastSampleMs) return;  // 没有新数据
        lastSampleMs = fd.lastUpdateMs;

        for (int i = 0; i < 6; i++) accum[i] += fd.raw[i];
        avgCount++;

        if (now - sampleStartMs < AVG_MS) return;

        // 采样窗口结束
        sampling = false;
        if (avgCount < MIN_SAMPLES) {
            std::cout << "[BIAS] 有效样本太少 (" << avgCount << "), 本次作废" << std::endl;
            return;
        }
        if (count >= MAX_POSES) return;
        for (int i = 0; i < 6; i++) bias[count][i] = accum[i] / avgCount;

        EnterCriticalSection(&appState.robotPoseMutex);
        pose[count][0] = appState.robotActualPose.rx;
        pose[count][1] = appState.robotActualPose.ry;
        pose[count][2] = appState.robotActualPose.rz;
        pose[count][3] = appState.robotActualPose.x;
        pose[count][4] = appState.robotActualPose.y;
        pose[count][5] = appState.robotActualPose.z;
        LeaveCriticalSection(&appState.robotPoseMutex);

        printf("[BIAS] Pose %d: R=(%+.1f,%+.1f,%+.1f)deg F=(%+.3f,%+.3f,%+.3f) N\n",
               count + 1, pose[count][0], pose[count][1], pose[count][2],
               bias[count][0], bias[count][1], bias[count][2]);
        count++;
    }

    // 姿态 → 3×3 旋转矩阵 (Rz·Ry·Rx, 与 GetPose/FK 同约定; 输入为度)
    static void rotFromPose(const double p[6], double R[9]) {
        TcpCalibration::rpyToMatrix(p[0], p[1], p[2], R);
    }

    static void report() {
        if (count < 3) {
            std::cout << "[BIAS] 至少需要 3 个姿态才能判断, 当前 " << count << std::endl;
            return;
        }
        if (!dataUnderCurrentPayload) {
            std::cout << "\n[BIAS] 这批数据是在【旧负载】下采的, 不能用来复验当前参数。\n"
                      << "       请直接摆姿态按 SPACE 重新采集 (会开始新一批)。" << std::endl;
            return;
        }
        // ===== 姿态覆盖度: 覆盖不足时极差只是噪声, 不能拿来判定 =====
        //   笔始终朝下 → 工具轴与重力夹角 θ≈0 → 轴向质心误差的力矩响应 ∝ sinθ ≈ 0,
        //   力矩通道"看不见"它; 除以 sinθ 归一化后这种姿态的 drEq 会失去意义。
        double R[MAX_POSES][9];
        double maxTiltSin = 0.0, maxPairDeg = 0.0;
        const double R2D = 180.0 / 3.14159265358979323846;
        for (int p = 0; p < count; p++) {
            rotFromPose(pose[p], R[p]);
            double s = sqrt(R[p][2] * R[p][2] + R[p][5] * R[p][5]);  // 工具轴 z 与重力夹角的正弦
            if (s > maxTiltSin) maxTiltSin = s;
            for (int q = 0; q < p; q++) {
                double tr = 0.0;
                for (int m = 0; m < 9; m++) tr += R[p][m] * R[q][m];  // trace(Rpᵀ·Rq)
                double c = (tr - 1.0) / 2.0;
                if (c > 1.0) c = 1.0;
                if (c < -1.0) c = -1.0;
                double deg = acos(c) * R2D;
                if (deg > maxPairDeg) maxPairDeg = deg;
            }
        }
        bool coverageOk = (maxPairDeg >= MIN_SPAN_DEG) && (maxTiltSin >= MIN_TILT_SIN);

        // ===== 力 (Fx,Fy,Fz) 与 力矩 (Mx,My,Mz) 的跨姿态极差分开看 =====
        //   重力补偿残差 ΔF = Δm·g_tool        → 质量误差污染力通道
        //                ΔM = (p − p_cfg) × g_tool,  p = m·r
        //   注意: 质量误差 Δm 同样会经由 p_cfg 漏进力矩通道, 所以力矩极差变大
        //   既可能是质心不准、也可能是质量不准; 反之质量误差一定同时抬高两者。
        //   力矩通道的独有价值是: 它是唯一能反映质心误差的通道。
        double loF[3], hiF[3], loM[3], hiM[3];
        for (int a = 0; a < 3; a++) {
            loF[a] = hiF[a] = bias[0][a];
            loM[a] = hiM[a] = bias[0][a + 3];
        }
        for (int p = 1; p < count; p++) {
            for (int a = 0; a < 3; a++) {
                if (bias[p][a]     < loF[a]) loF[a] = bias[p][a];
                if (bias[p][a]     > hiF[a]) hiF[a] = bias[p][a];
                if (bias[p][a + 3] < loM[a]) loM[a] = bias[p][a + 3];
                if (bias[p][a + 3] > hiM[a]) hiM[a] = bias[p][a + 3];
            }
        }
        double sF[3], sM[3];
        for (int a = 0; a < 3; a++) { sF[a] = hiF[a] - loF[a]; sM[a] = hiM[a] - loM[a]; }
        double spanF = sqrt(sF[0] * sF[0] + sF[1] * sF[1] + sF[2] * sF[2]);
        double spanM = sqrt(sM[0] * sM[0] + sM[1] * sM[1] + sM[2] * sM[2]);

        double mCfg, cCfgNow[3];
        PayloadCalibration::effective(mCfg, cCfgNow);
        if (!(mCfg > 1e-6)) mCfg = 0.66;   // 防除零
        // |ΔF| 是多轴分量极差的模, 对同一个转动矢量可达真实 Δm 的 ~2 倍 → 只当上界看
        double dmUpper = spanF / 9.81 * 1000.0;                        // 等效质量误差上界 (g)
        double drEq = spanM / (mCfg * 9.81 * maxTiltSin) * 1000.0;     // 等效质心误差 (mm)

        std::cout << "\n======================================================" << std::endl;
        std::cout << "  多姿态零偏检查 — 负载参数验证" << std::endl;
        std::cout << "  姿态数: " << count << "   最大姿态跨度: " << (int)(maxPairDeg + 0.5)
                  << "°   最大 sinθ: " << maxTiltSin << std::endl;
        std::cout << "  当前负载: load=" << mCfg << " kg  center=("
                  << cCfgNow[0] << ", " << cCfgNow[1] << ", " << cCfgNow[2] << ") mm"
                  << (PayloadCalibration::enabled ? "  [实机标定值]" : "  [种子值, 未标定]")
                  << "  CZ符号=" << (PayloadCalibration::comSignZ > 0 ? "+1" : "-1")
                  << std::endl;
        std::cout << "======================================================" << std::endl;
        printf("  跨姿态极差(力):   Fx=%.3f  Fy=%.3f  Fz=%.3f   |ΔF|=%.3f N\n",
               sF[0], sF[1], sF[2], spanF);
        printf("  跨姿态极差(力矩): Mx=%.4f My=%.4f Mz=%.4f |ΔM|=%.4f Nm\n",
               sM[0], sM[1], sM[2], spanM);
        printf("  → 等效质量误差 ≤ %.0f g   /   ", dmUpper);
        if (maxTiltSin >= MIN_TILT_SIN) printf("等效质心误差 %.0f mm\n", drEq);
        else                            printf("等效质心误差 不可观测 (倾角不足)\n");
        std::cout << "------------------------------------------------------" << std::endl;

        // 先判覆盖度: 覆盖不足时任何极差结论都不成立
        if (maxPairDeg < MIN_SPAN_DEG) {
            std::cout << "  ⚠ 姿态跨度不足 " << (int)MIN_SPAN_DEG << "° — 各姿态几乎一样,"
                      << " 极差只是噪声" << std::endl;
        }
        if (maxTiltSin < MIN_TILT_SIN) {
            std::cout << "  ⚠ 倾角覆盖不足 (sinθ<" << MIN_TILT_SIN
                      << ") — 笔始终朝下时轴向质心误差在力矩上无响应,"
                      << " 请把笔摆到水平/朝上再补几个姿态" << std::endl;
        }

        if (!coverageOk) {
            std::cout << "  ? 覆盖不足, 不作判定 — 补姿态后重测" << std::endl;
        } else if (spanF < 0.3 && drEq < 10.0) {
            std::cout << "  ✓ PASS — 零偏与姿态无关, 负载参数正确" << std::endl;
        } else if (spanF < 1.0 && drEq < 30.0) {
            std::cout << "  ⚠ WARN — 有残余姿态依赖, 按下面的量级微调后重测" << std::endl;
        } else {
            std::cout << "  ✗ FAIL — 负载参数明显不准" << std::endl;
        }

        if (spanF >= 0.3) {
            std::cout << "    · 力项偏大 → 质量不准。**上秤称工具链实际总重**复核"
                      << " (别用这里的上界去加减)" << std::endl;
        }
        if (coverageOk && drEq >= 10.0) {
            std::cout << "    · 力矩项偏大 → 质心不准 (但也可能只是质量误差漏进来的)" << std::endl;
        }
        if (!coverageOk || spanF >= 0.3 || drEq >= 10.0) {
            std::cout << "   → 按 's' 用这批数据【求解】并下发修正后的负载参数"
                      << std::endl;
        }
        std::cout << std::endl;
    }

    // 's': 用已采数据最小二乘求解负载参数 → 落盘 → 重新下发 EnableRobot
    static void solveAndApply() {
        if (solveLocked) {
            std::cout << "\n[BIAS] !! 已连续 " << consecutiveFails << " 次判定结果不合理, 已停止求解。\n"
                      << "       [BIAS] !! 原因见最后那次被否掉的判据 (拟合残差 / 符号无法判定)。\n"
                      << "       [BIAS] !! 处理后按 'm' 重新采集 (计数会清零)。" << std::endl;
            return;
        }
        if (count < 4) {
            std::cout << "[BIAS] 求解至少需要 4 个姿态 (当前 " << count
                      << "), 建议 6~8 个" << std::endl;
            return;
        }
        // 采样中途不允许求解
        if (sampling) {
            std::cout << "[BIAS] 正在采样, 稍后再求解" << std::endl;
            return;
        }

        // 机械臂当前实际使用的负载 = 上次下发的值 (标定值优先, 否则种子)
        double mCfg, cCfg[3];
        PayloadCalibration::effective(mCfg, cCfg);

        // 姿态转成求解器要的 [x,y,z,rx,ry,rz]: 本模块存的是 [rx,ry,rz,x,y,z]
        static double sp[12][6];
        static double sf[12][3];
        static double sm[12][3];
        for (int i = 0; i < count; i++) {
            sp[i][0] = pose[i][3]; sp[i][1] = pose[i][4]; sp[i][2] = pose[i][5];
            sp[i][3] = pose[i][0]; sp[i][4] = pose[i][1]; sp[i][5] = pose[i][2];
            for (int a = 0; a < 3; a++) {
                sf[i][a] = bias[i][a];        // 力
                sm[i][a] = bias[i][a + 3];    // 力矩
            }
        }

        PayloadCalibration::Result r;
        if (!PayloadCalibration::solve(sp, sf, sm, count, mCfg, cCfg,
                                       PayloadCalibration::comSignZ, r)) {
            std::cout << "[BIAS] 求解失败 — 姿态数不足/退化(姿态太接近)/解非物理。\n"
                      << "       请确认各姿态差异足够大 (跨度≥30°, 且笔有水平/朝上的姿态)"
                      << std::endl;
            return;
        }

        std::cout << "\n======================================================" << std::endl;
        std::cout << "  负载参数求解结果 (" << r.poses << " 个姿态) —— CZ 符号待实测裁决"
                  << std::endl;
        std::cout << "======================================================" << std::endl;
        printf("  质量:    当前 %.3f kg   →  修正 %+.3f kg   →   %.3f kg\n",
               mCfg, r.dm, r.massKg);
        printf("  质心 X/Y/Z: 当前 (%.1f, %.1f, %.1f)\n", cCfg[0], cCfg[1], cCfg[2]);
        // 裁决之前【不写"下发"】: r.comMm / r.dc[2] 此刻只是按"当前符号约定"折算出来的占位值,
        // 真下发的值要等探针选完才是 r.comCand[chosen]。两者 Z 相差 2·m_cfg·cz/m_true
        // (种子值下约 395 mm) —— 标成"下发"就是在骗操作者。X/Y 与符号无关, 可以直接报。
        printf("             X/Y 修正 (%+.1f, %+.1f) mm  (Z 修正取决于符号, 见下)\n",
               r.dc[0], r.dc[1]);
        printf("             两个候选各自的下发值 (mm, 尚未下发, 待实测裁决):\n"
               "               +1 → (%.1f, %.1f, %+.1f)   -1 → (%.1f, %.1f, %+.1f)\n",
               r.comCand[0][0], r.comCand[0][1], r.comCand[0][2],
               r.comCand[1][0], r.comCand[1][1], r.comCand[1][2]);
        printf("  拟合残差: 力 %.4f N   力矩 %.4f N·m\n", r.rmsForceN, r.rmsMomentNm);

        // ===== CZ 符号候选 (这里不下结论) =====
        // signZ 不进线性系统, 两种符号的拟合残差完全相同 —— 数据区分不了, 所以解算器
        // 不再自行选边: 哪个候选成立由下面的实机探针裁决。(r.signZ / r.signAmbiguous 都只
        // 是"解算器留下的占位值/恒为 true 的标志", 谁都不会覆写它们, 不能当结论打出来。)
        // 判据只看物理质心必须在法兰下方, 所以把两个候选都打出来, 便于人工复核。
        printf("    · 候选 +1: 物理质心 Z = %+.1f mm  %s\n",
               r.cTrueZ[0], r.cTrueZ[0] > 0 ? "✓ 法兰下方" : "✗ 法兰上方 (非物理)");
        printf("    · 候选 -1: 物理质心 Z = %+.1f mm  %s\n",
               r.cTrueZ[1], r.cTrueZ[1] > 0 ? "✓ 法兰下方" : "✗ 法兰上方 (非物理)");

        // ===== 符号实测裁决 =====
        // solve() 只给出两种解释; 这里在【同一个静止姿态】下各下发一次, 谁留下的
        // 力矩残余小谁对。机械臂全程不动 —— 差异纯粹来自符号, 不是姿态。
        // 前提: 当前姿态要有足够倾角, 否则两者都≈0、分不开。
        double rProbe[2] = {0.0, 0.0};
        int    chosen = -1;
        bool   probeOk = true;
        bool   converged = false;   // 已收敛: 配置本身就解释得了数据, 探针无可裁决 (见下)
        // 符号定不了案时, 下面的判据屏必须说出【真实原因】。一共三条路: 倾角不足 /
        // 探针没取到读数 / 取到了但两个候选分不开。别再把它们都说成"两个候选在同一侧"
        // —— 两个候选永远是一对相反的解释, 不会同侧。
        enum SignFail { SIGN_FAIL_NONE, SIGN_FAIL_TILT, SIGN_FAIL_PROBE, SIGN_FAIL_MARGIN };
        double   sinTheta = 0.0;
        SignFail signFail = SIGN_FAIL_NONE;
        {
            double p[6];
            EnterCriticalSection(&appState.robotPoseMutex);
            p[0] = appState.robotActualPose.x;  p[1] = appState.robotActualPose.y;
            p[2] = appState.robotActualPose.z;  p[3] = appState.robotActualPose.rx;
            p[4] = appState.robotActualPose.ry; p[5] = appState.robotActualPose.rz;
            LeaveCriticalSection(&appState.robotPoseMutex);
            double R[9];
            TcpCalibration::rpyToMatrix(p[3], p[4], p[5], R);
            sinTheta = sqrt(R[2] * R[2] + R[5] * R[5]);
            if (sinTheta < Config::SIGN_PROBE_MIN_SIN_THETA) {
                std::cout << "  ✗ 当前姿态倾角不足 (sinθ=" << sinTheta
                          << " < " << Config::SIGN_PROBE_MIN_SIN_THETA
                          << ") — 两个符号分不开。" << std::endl;
                std::cout << "    请把笔摆到明显倾斜/水平再按 's'。" << std::endl;
                signFail = SIGN_FAIL_TILT;
                probeOk = false;
            }
        }
        // ===== 收敛短路: 当前配置本身就解释得了这批数据 =====
        // 两个候选的下发值只差 2·dp/m_true (dp = p_true − p_cfg, 见 solve() 的换算):
        // 配置越接近真值, 两个候选越重合, 探针量到的差 → 0 → margin 与 ratio 双双不过,
        // 于是【"配置已经对了"】被报成"实测分不开" —— 连报几次就锁死 's'。刚标定完再按
        // 一次 's' 正好踩在这个坑里: 机器说"测量失败", 其实已经没什么可改的了。
        // 输的那个候选留下的力矩误差 ≈ 2·|dp|·g·sinθ (见 Config.h 的门限推导); 它小于
        // 差值门限, 就说明【在本姿态下】两个候选的差压不过门限, 探针跑不跑都是同一个结局。
        //   sepNm = r.massKg·|Δc_z|/1000·g·sinθ = m_true·(2|dp|/m_true)·g·sinθ = 2|dp|g·sinθ
        // 所以短路掉探针 (省掉两次使能口往返 + 两个 400ms 采样窗), 并且【不计失败】。
        // 【为什么必须放在倾角门限之后】sepNm 里带 sinθ: 倾角越小, 同一个 dp 算出来的 sepNm
        // 也越小。若放在门限之前, 一个"符号错了 / 配置差得远"的机台只要姿态没摆好, 就会被
        // 误判成"已收敛" —— 那才是真的把失败藏起来。放在门限之后, sinθ >= 0.7 有下界,
        // 短路只在 |dp| <= 0.2/(2×9.81×0.7) = 0.0146 kg·m 时才触发, 这个量级探针本来就
        // 判不出来 (判据同源: 同一个门限, 同一个姿态)。
        if (probeOk) {
            const double sepNm = r.massKg * fabs(r.comCand[1][2] - r.comCand[0][2])
                                 / 1000.0 * 9.81 * sinTheta;
            if (sepNm < Config::SIGN_PROBE_MIN_MARGIN_NM) {
                converged = true;
                // 两个候选分不开, 也就【没有可选项】: 沿用当前符号约定 (chosen 只决定
                // 存哪个符号, 不决定改多少 —— 两个候选的下发值本来就几乎一样)。
                chosen = (PayloadCalibration::comSignZ < 0.0) ? 1 : 0;
                printf("  ✓ 已收敛: 当前配置与数据一致 — 两个候选相差 %.4f N·m (< %.2f N·m "
                       "门限), 无需改配置, 不因\"分不开\"计失败\n",
                       sepNm, Config::SIGN_PROBE_MIN_MARGIN_NM);
                std::cout << "    没有可裁决的东西: 两个候选本来就在仪器的分辨极限之内。"
                          << std::endl;
                std::cout << "    本次照样保存/下发解出的参数 (质量修正仍会生效), "
                          << "只是符号沿用当前约定。" << std::endl;
            }
        }
        if (probeOk && !converged) {
            for (int k = 0; k < 2; k++) {
                if (!RelayCore::instance().probePayloadResidual(r.massKg, r.comCand[k], rProbe[k])) {
                    // probePayloadResidual 的 false 有四种原因: 机械臂未连接、候选下发失败、
                    // 机械臂没采纳这次候选 (回读不符)、采样窗口内样本不够。别让操作员以为
                    // 上面一定有 [Probe] 提示 —— 只有"机械臂未连接"是静默返回 false。
                    std::cout << "  ✗ 符号探针失败 — 这个候选没测到有效读数。" << std::endl;
                    std::cout << "    四种原因: 机械臂未连接 / 候选下发失败 / 机械臂没采纳这次候选 / "
                              << "力数据不足。" << std::endl;
                    std::cout << "    除【机械臂未连接】外, 上面都会打一行 [Probe] 提示 "
                              << "(可能更靠上, 翻一下); 未连接这种看顶部的连接状态。" << std::endl;
                    signFail = SIGN_FAIL_PROBE;
                    probeOk = false;
                    break;
                }
            }
        }
        if (probeOk && !converged) {
            const int    win  = (rProbe[0] <= rProbe[1]) ? 0 : 1;
            const int    lose = 1 - win;
            const double margin  = rProbe[lose] - rProbe[win];
            // 【为什么判差值, 不判胜者的绝对值】raw[] 里含一个姿态常数的传感器零偏 b,
            // 探针只把每个候选约简成一个模长 |b + d_k| —— 常数在 (b+d) - b 里才消, 在
            // |b+d| 与 |b| 的比较里消不掉。判"胜者模长够小"等于在考 |b|: |b| 一到 0.05
            // 的量级两个候选就全过不了, 求解会一路失败到锁死。差值最多被 |b| 吃掉 2|b|,
            // 与 |b| 大小脱钩, 所以判差值。(门限的物理依据见 Config.h 的 SIGN_PROBE_*。)
            const bool separated = (rProbe[lose] >= Config::SIGN_PROBE_MIN_RATIO * rProbe[win]);
            const bool farApart  = (margin >= Config::SIGN_PROBE_MIN_MARGIN_NM);
            printf("  CZ 符号实测: 候选 +1 残余 %.4f N·m / 候选 -1 残余 %.4f N·m "
                   "(差值 %.4f N·m)\n",
                   rProbe[0], rProbe[1], margin);
            if (separated && farApart) {
                chosen = win;
                std::cout << "  CZ 符号约定: 裁决 → " << (win == 0 ? "+1" : "-1")
                          << " (残余小 " << (rProbe[lose] / rProbe[win]) << " 倍)" << std::endl;
            } else {
                std::cout << "  CZ 符号约定: ✗ 实测分不开 — 结果按不合理处理" << std::endl;
                signFail = SIGN_FAIL_MARGIN;
            }
        }
        // 探针已经把两个候选都下发给了机械臂 (最后留在上面的可能是输的那个) —— 但这里
        // 【先不定案】: 定案 (applyResult) 要等合理性判据通过, 否则判据一旦否掉这次求解,
        // 内存里的生效负载就已经被改过了, 下面那句"机械臂负载参数保持原值"就是假的。
        if (chosen < 0) {
            // 没候选胜出: 探针已经把某个候选配到机械臂上了, 恢复成当前生效值
            RelayCore::instance().applyPayloadToRobot();
        }

        // ===== 合理性判据 =====
        // 任一命中即"不合理" -> 拒绝保存和下发, 机械臂保持原参数。
        // 绝不拿一个程序自己都判定为不可信的结果去配置机械臂。
        const double RMS_F_GOOD = 0.10;   // N — 到这个量级才说明模型与数据一致
        const double RMS_F_MAX  = 0.30;   // N — 超过即不合理 (实机: 约定对 ~0.06, 错 ~0.9)
        const bool   fitGood = (r.rmsForceN < RMS_F_GOOD);
        const bool   fitOk   = (r.rmsForceN < RMS_F_MAX);
        const bool   signOk = (chosen >= 0);        // 实测必须能裁决出符号
        // 物理质心 (不是下发值): 要评的是【实测定案的那个候选】。cTrueZ[] 是按候选索引的,
        // 所以下标就是 chosen —— r.signZ 是解算器留下的占位值 (恒等于传入的旧约定),
        // 拿它选下标会去评一个根本没被下发的候选。
        const double cTrueZChosen = signOk ? r.cTrueZ[chosen] : 0.0;
        const bool   comZOk = signOk && (cTrueZChosen > 0.0); // 工具挂在法兰下方 -> 质心 Z 必须为正
        const double comXY  = signOk
            ? sqrt(r.comCand[chosen][0] * r.comCand[chosen][0]
                 + r.comCand[chosen][1] * r.comCand[chosen][1])
            : 0.0;
        const bool   reasonable = fitOk && signOk && comZOk;

        std::cout << "------------------------------------------------------" << std::endl;
        std::cout << "  合理性评估 (三条判据, 任一不满足即拒绝下发):" << std::endl;
        if (fitGood) {
            printf("    ✓ 拟合残差 %.4f N ≈ 噪声本底 (< %.2f N)\n", r.rmsForceN, RMS_F_GOOD);
        } else if (fitOk) {
            printf("    ✓ 拟合残差 %.4f N 在容许范围内 (< %.2f N)\n", r.rmsForceN, RMS_F_MAX);
        } else {
            printf("    ✗ 拟合残差 %.4f N 超过阈值 %.2f N — 模型解释不了这批数据\n",
                   r.rmsForceN, RMS_F_MAX);
        }
        if (signOk && converged) {
            // 这条不等于"实测裁决过" —— 探针压根没跑。收敛态下两个候选的下发值几乎相同,
            // 符号取值在实测层面不可观测, 沿用当前约定只是为了有一个确定的存盘值。
            printf("    ✓ 符号: 沿用当前约定 %s — 本次【无实测裁决】(两个候选分不开, "
                   "选谁都一样; 物理质心 Z +1 → %+.1f mm, -1 → %+.1f mm)\n",
                   chosen == 0 ? "+1" : "-1", r.cTrueZ[0], r.cTrueZ[1]);
        } else if (signOk) {
            printf("    ✓ 符号可判定 — 实测裁决 → %s (两候选的物理质心 Z: +1 → %+.1f mm, -1 → %+.1f mm)\n",
                   chosen == 0 ? "+1" : "-1", r.cTrueZ[0], r.cTrueZ[1]);
        } else {
            // 两条候选是一对【相反】的解释, 不存在"都在同一侧"这种失败。真正的原因只有三种。
            printf("    ✗ 符号无法判定:\n");
            switch (signFail) {
            case SIGN_FAIL_TILT:
                printf("      · 姿态倾角不足 (sinθ=%.2f < %.1f) — 两个符号的力矩差压不过噪声, 分不开\n",
                       sinTheta, Config::SIGN_PROBE_MIN_SIN_THETA);
                break;
            case SIGN_FAIL_PROBE:
                printf("      · 探针没取到有效读数 (机械臂未连接 / 候选下发失败 / "
                       "机械臂没采纳这次候选 / 力数据不足)\n");
                break;
            case SIGN_FAIL_MARGIN:
                printf("      · 探针测到了两个候选但区分不开 (残余 +1: %.4f, -1: %.4f N·m; 判据: "
                       "败者 ≥ 胜者 %.1f 倍 且 两者相差 ≥ %.2f N·m)\n",
                       rProbe[0], rProbe[1], Config::SIGN_PROBE_MIN_RATIO,
                       Config::SIGN_PROBE_MIN_MARGIN_NM);
                break;
            default:
                break;
            }
            printf("      · 两个候选 (不是同侧): 物理质心 Z +1 → %+.1f mm, -1 → %+.1f mm\n",
                   r.cTrueZ[0], r.cTrueZ[1]);
        }
        if (!signOk) {
            printf("    · 物理质心 Z 未评估 — 符号没定案, 无从判断哪个候选才是要下发的那个\n");
        } else if (comZOk) {
            printf("    ✓ 物理质心 Z = %+.1f mm 在法兰下方 (该候选下发 %+.1f mm), 偏心 |XY| = %.1f mm\n",
                   cTrueZChosen, r.comCand[chosen][2], comXY);
        } else {
            printf("    ✗ 物理质心 Z = %+.1f mm 不在法兰下方 — 工具装夹或符号有问题\n",
                   cTrueZChosen);
        }

        std::cout << "------------------------------------------------------" << std::endl;
        if (!reasonable) {
            consecutiveFails++;
            std::cout << "  判定: ✗ 不合理 — 已【拒绝保存和下发】, 机械臂负载参数保持原值 (第 "
                      << consecutiveFails << " 次)" << std::endl;
            std::cout << "  → 请按 'm' 重新采集 (姿态跨度≥30°, 笔要有水平/朝上的姿态), 再按 's'"
                      << std::endl;
            if (consecutiveFails >= Config::CALIB_MAX_CONSECUTIVE_FAILS) {
                solveLocked = true;
                std::cout << std::endl;
                std::cout << "  [BIAS] !! 已连续 " << consecutiveFails << " 次不合理, 停止求解。"
                          << std::endl;
                std::cout << "  [BIAS] !! 问题多不在求解器 —— 按上面那屏判据的实际失败项排查:"
                          << std::endl;
                std::cout << "  [BIAS] !!  · 拟合残差超阈 → 装夹是否松动 / 力传感器是否受挤压 / "
                          << "姿态覆盖是否足够" << std::endl;
                std::cout << "  [BIAS] !!  · 符号定不了案 → 姿态倾角不足 (摆到明显倾斜再按 's'), "
                          << "或探针根本没测到 (机械臂是否连接、" << std::endl;
                std::cout << "  [BIAS] !!    候选负载能否下发、30004 力数据流是否在更新)"
                          << std::endl;
                std::cout << "  [BIAS] !! 处理后按 'm' 重新采集 (计数会清零)。" << std::endl;
            }
            std::cout << std::endl;
            // 判据是在探针【之后】跑的 —— 机械臂此刻的配置还是探针留下的那个候选
            // (可能是输的那个)。所以拒绝不能只是"不保存", 必须把机器人恢复成原参数,
            // 否则上面那句"机械臂负载参数保持原值"就是假的。
            // 内存里的生效值从头到尾没动过, 这一发重发的就是原来的参数。
            RelayCore::instance().applyPayloadToRobot();
            return;
        }
        consecutiveFails = 0;   // 成功一次即清零

        // 定案只发生在这里 (判据全过): 探针选中的候选既是内存生效值, 也是下发值。
        PayloadCalibration::applyResult(r, chosen);
        if (!PayloadCalibration::save(CalibStore::fileFor("payload_calib.json"))) {
            std::cerr << "[BIAS] !! payload_calib.json 写入失败" << std::endl;
        } else {
            std::cout << "  已保存 payload_calib.json (下次启动自动加载)" << std::endl;
        }
        // "下发"只在这里报: 上面所有行都是裁决【之前】的占位值, 把它们标成"下发"会骗操作者。
        const bool   sentOk = RelayCore::instance().applyPayloadToRobot();
        const double comSent[3] = {r.comCand[chosen][0], r.comCand[chosen][1], r.comCand[chosen][2]};
        if (sentOk) {
            printf("  已下发: 质量 %.3f kg, 质心 (%.1f, %.1f, %+.1f) mm  (符号约定 %s)\n",
                   r.massKg, comSent[0], comSent[1], comSent[2], chosen == 0 ? "+1" : "-1");
            printf("  相对原值的修正: 质量 %+.3f kg, 质心 (%+.1f, %+.1f, %+.1f) mm\n",
                   r.dm, comSent[0] - cCfg[0], comSent[1] - cCfg[1], comSent[2] - cCfg[2]);
        } else {
            std::cout << "  ✗ 新负载下发失败 — 机械臂仍在用旧参数 (内存已定案, 重启客户端会重试)"
                      << std::endl;
        }

        // 这批姿态是在【旧负载】下采的。下发新负载之后复验 (report) 必须拒绝它们,
        // 否则会拿旧数据骂新参数 (曾经报出假 FAIL: 求解残差 0.06 N,
        // 紧接着的报告却报 |ΔF| = 4.0 N)。
        dataUnderCurrentPayload = false;
        std::cout << "  → 复验: 摆姿态按 SPACE 采集 (第一次 SPACE 会自动开新一批,"
                  << " 旧数据作废)" << std::endl;
        std::cout << std::endl;
    }
}

// ===== 采集类模式互斥 =====
// 坐标标定 / TCP 标定 / 多姿态检查 / FK 验证都靠 SPACE 采点, 同时开着会互相吞按键
// (最坏情况: MOTION 相在等 SPACE 收尾却被别的模式吃掉, 拖拽模式一直开着)。
// 进入任一个之前先关掉其它的。keep = 本次要进入的模式代号。
static void cancelOtherCaptureModes(char keep) {
    if (keep != 'c' && Calibration::collectMode)   Calibration::cancelCollect();
    if (keep != 't' && TcpCalibration::collectMode) TcpCalibration::cancelCollect();
    if (keep != 'm')                                BiasCheck::cancel();
    if (keep != 'v' && FkValidate::mode) {
        FkValidate::mode = false;
        std::cout << "[FK-VAL] Mode OFF" << std::endl;
    }
}

// ===== GLUT 回调 =====

void keyboard(unsigned char key, int, int);  // forward decl for console polling in idle()

void display() {
    if (appState.isClosing) return;
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glutSwapBuffers();
    if (!g_noRobot) {
        RelayCore::instance().pollFeedback();
    }
}
void idle() {
    if (!appState.isClosing) {
        glutPostRedisplay();

        // MATLAB → C++ 反向命令轮询 (力反馈开关等)
        RelayCore::instance().pollRelayCommands();

        // 控制台键盘轮询 (标定模式等操作不依赖 GLUT 窗口焦点)
        if (_kbhit()) {
            int ch = _getch();
            keyboard((unsigned char)ch, 0, 0);
        }

        // Poll force data at ~30Hz alongside feedback (robot mode only)
        if (!g_noRobot) {
            RelayCore::instance().pollForce();

            // 多姿态零偏检查采样 (负载参数验证)
            if (BiasCheck::mode) {
                AppState::ForceData fdSnap;
                EnterCriticalSection(&appState.forceDataMutex);
                fdSnap = appState.forceData;
                LeaveCriticalSection(&appState.forceDataMutex);
                BiasCheck::sample(fdSnap);
            }

            // Track force calibration state changes
            static ForceCalibration::State lastCalibState = ForceCalibration::State::IDLE;
            ForceCalibration::State curState = ForceCalibration::currentState();
            if (curState != lastCalibState) {
                lastCalibState = curState;
                std::cout << "[Force] Calibration: " << ForceCalibration::statusText() << std::endl;
                if (curState == ForceCalibration::State::DONE) {
                    // Results already applied by ForceCalibration::update() SOLVE phase.
                    // Just print confirmation.
                    std::cout << "[Force] Calibration results saved to force_calib.json" << std::endl;
                }
            }
        }
        // Check haptic watchdog (only when not in --no-robot mode)
        if (!g_noRobot) {
            RelayCore::instance().checkHapticWatchdog();
            // 安全网: 每帧刷新心跳，防止 GLUT 定时器延迟导致误判超时
            RelayCore::instance().resetHeartbeat();
        }
        Sleep(1);
    }
}

void reshape(int w, int h) {
    // 窗口大小变化时由 display() 按比例重新计算所有视口
    glutPostRedisplay();
}

// ===== 定时器 (无机械臂模式下跳过) =====
void poseQueryTimer(int) {
    if (!g_noRobot) {
        RelayCore::instance().queryPose();
    }
    if (!appState.isClosing) {
        glutTimerFunc(Config::POSE_QUERY_INTERVAL, poseQueryTimer, 0);
    }
}

void alarmCheckTimer(int) {
    if (!g_noRobot) {
        RelayCore::instance().checkAlarm();
    }
    if (!appState.isClosing) {
        glutTimerFunc(Config::ALARM_CHECK_INTERVAL, alarmCheckTimer, 0);
    }
}

void jointAngleTimer(int) {
    if (!g_noRobot) {
        RelayCore::instance().queryJointAngles();
    }
    if (!appState.isClosing) {
        glutTimerFunc(200, jointAngleTimer, 0);
    }
}

void safetyStatusTimer(int) {
    if (!g_noRobot) {
        RelayCore::instance().sendSafetyStatus();
        RelayCore::instance().sendSingularity();
    }
    if (!appState.isClosing) {
        glutTimerFunc(200, safetyStatusTimer, 0);
    }
}

void jointMarginTimer(int) {
    if (!g_noRobot) {
        RelayCore::instance().sendJointMargins();
    }
    if (!appState.isClosing) {
        glutTimerFunc(500, jointMarginTimer, 0);
    }
}

void connectionHealthTimer(int) {
    if (!g_noRobot) {
        RelayCore::instance().sendConnectionHealth();
    }
    if (!appState.isClosing) {
        glutTimerFunc(1000, connectionHealthTimer, 0);
    }
}

void keyboard(unsigned char key, int, int) {
    if (key == 'q' || key == 'Q' || key == 27) { // q 或 ESC
        std::cout << "\nShutting down..." << std::endl;
        RelayCore::instance().shutdownRelayReporting();
        RobotDiagnostics::instance().shutdown();
        if (!g_noRobot) {
            std::cout << "Disabling robot..." << std::endl;
            RelayCore::instance().shutdown();  // 发送 DisableRobot() + 断开连接
        }
        if (!g_noTouch) cleanupHapticDevice();
        exit(0);
    }
    if (key == 'e' || key == 'E') {
        if (!g_noRobot) {
            std::cout << "\n[Main] 手动触发脱困..." << std::endl;
            if (RelayCore::instance().triggerEscape()) {
                std::cout << "[Main] 脱困成功，恢复操作" << std::endl;
                RelayCore::instance().stateMachine().onRecovery();
            } else {
                std::cout << "[Main] 脱困失败" << std::endl;
            }
        }
    }

    // ===== 力传感器调零 ('z' key) =====
    // 'z': 静置采集零偏 → 直接应用+存盘。不进 MOTION 相、不开拖拽模式。再按一次中止。
    //      换装工具 (笔夹/笔) 后重新调零用这个。
    if (key == 'z' || key == 'Z') {
        auto& relay = RelayCore::instance();
        if (relay.isForceZeroing()) {
            relay.abortForceCalibration();
            return;
        }
        if (relay.isForceCalibrating()) {
            std::cout << "[Force] 力标定进行中 — 等它结束, 或按 'k' 中止" << std::endl;
            return;
        }
        if (g_noRobot) {
            std::cout << "[Force] --no-robot 模式下无法调零" << std::endl;
            return;
        }
        cancelOtherCaptureModes('z');
        if (relay.startForceZeroing()) {
            std::cout << "[Force] 调零中: 保持机械臂静止, 采集完成后自动应用并存盘"
                      << std::endl;
        }
        return;
    }

    // ===== 力标定全流程 ('k' key) =====
    // 'k': TARE + MOTION(拖动拟合质量) — 需要拖动机械臂。再按一次中止。
    if (key == 'k' || key == 'K') {
        auto& relay = RelayCore::instance();
        if (relay.isForceZeroing()) {
            std::cout << "[Force] 调零进行中 — 等待完成, 或按 'z' 中止" << std::endl;
            return;
        }
        if (relay.isForceCalibrating()) {
            relay.abortForceCalibration();
            return;
        }
        cancelOtherCaptureModes('k');
        if (!g_noRobot && relay.startForceCalibration()) {
            std::cout << "[Force] TARE: keep robot still (2s), then drag to move for mass cal." << std::endl;
        }
        return;
    }

    // ===== Touch→Robot 坐标标定 ('c' key) =====
    if (key == 'c' || key == 'C') {
        if (RelayCore::instance().isForceCalibrating()) {
            std::cout << "[CALIB] 力标定/调零进行中 — 先按 'z' 或 'k' 结束" << std::endl;
            return;
        }
        if (Calibration::collectMode) {
            Calibration::cancelCollect();
            std::cout << "\n[CALIB] Mode OFF" << std::endl;
        } else {
            cancelOtherCaptureModes('c');
            Calibration::startCollect();
            std::cout << "\n[CALIB] Mode ON — "
                      << "Align Touch pen + robot to marker, press SPACE to record,"
                      << " 's' to solve, 'c' to exit" << std::endl;
        }
        return;
    }

    // ===== TCP 偏移标定 ('t' key) =====
    // 't': 切换 TCP 标定采集模式 (笔尖对准固定点, 多姿态记录法兰位姿)
    if (key == 't' || key == 'T') {
        if (TcpCalibration::collectMode) {
            TcpCalibration::cancelCollect();
            std::cout << "\n[TCP-CALIB] Mode OFF" << std::endl;
        } else {
            cancelOtherCaptureModes('t');
            TcpCalibration::startCollect();
            std::cout << "\n[TCP-CALIB] Mode ON — "
                      << "Keep pen TIP at a fixed point, reorient arm, "
                      << "press SPACE to record a pose, 's' to solve, 't' to exit" << std::endl;
        }
        return;
    }

    // ===== 多姿态零偏检查 ('m' key) =====
    // 'm': 切换模式; SPACE 记录一个姿态 (自动平均 1s); 关闭模式时输出报告
    if (key == 'm' || key == 'M') {
        if (!BiasCheck::mode) {
            if (g_noRobot) {
                std::cout << "[BIAS] --no-robot 模式下不可用" << std::endl;
                return;
            }
            if (RelayCore::instance().isForceCalibrating()) {
                std::cout << "[BIAS] 力标定/调零进行中 — 结束后再试" << std::endl;
                return;
            }
            cancelOtherCaptureModes('m');
            BiasCheck::reset();
            BiasCheck::mode = true;
            std::cout << "\n[BIAS] Mode ON — 笔尖悬空, 只改姿态(位置尽量不变),"
                      << " 每到一个姿态按 SPACE (采样 1s)\n"
                      << "       'd' 拖拽模式开关 (摆姿态用; 摆好一定要关掉再采样) /"
                      << " 'm' 退出并输出报告\n"
                      << "       's' 求解负载参数并下发 (符号与合理性由程序自动判定)"
                      << std::endl;
        } else {
            BiasCheck::mode = false;
            // 别把柔顺状态带出模式
            RelayCore::instance().setDragMode(false);
            BiasCheck::report();   // 数据属旧负载时会自行拒绝判定
        }
        return;
    }

    // ===== 拖拽模式开关 ('d' key, 仅 'm' 模式下) =====
    // 标定要摆 6~8 个姿态, 但拖拽开着时机械臂柔顺、姿态会漂, 采到的力数据是脏的。
    // 所以做成手动开关: 拖到位 → 按 'd' 锁住 → 再按 SPACE 采样。
    // 不用示教器也能摆姿态; 退出 'm' 模式会自动关掉, 不会把柔顺状态漏出去。
    if ((key == 'd' || key == 'D') && BiasCheck::mode) {
        if (g_noRobot) {
            std::cout << "[BIAS] --no-robot 模式下无法切换拖拽" << std::endl;
            return;
        }
        auto& relay = RelayCore::instance();
        const bool want = !relay.isDragMode();
        if (!relay.setDragMode(want)) {
            std::cout << "[BIAS] 拖拽模式切换失败 (机械臂未连接?)" << std::endl;
            return;
        }
        std::cout << (want ? "       现在可以手动拖动机械臂摆姿态; 摆好后按 'd' 锁定位姿再 SPACE 采样"
                           : "       位姿已锁定, 可以按 SPACE 采样了")
                  << std::endl;
        return;
    }

    // SPACE during force calibration: start/stop sampling
    // 必须排在 BiasCheck 之前: 否则标定/MOTION 相在用 SPACE 收尾时会被 BiasCheck 吞掉,
    // 导致 MOTION 迟迟不结束 (拖拽模式一直开着)。
    if (key == ' ' && RelayCore::instance().isForceCalibrating()) {
        ForceCalibration::confirmPose();
        return;
    }

    if (key == ' ' && BiasCheck::mode) {
        BiasCheck::record();
        return;
    }

    // 's' in BiasCheck mode: 求解负载参数 → 落盘 → 重新下发
    if ((key == 's' || key == 'S') && BiasCheck::mode) {
        BiasCheck::solveAndApply();
        return;
    }

    // Space: 记录标定点对 (Touch原始坐标 + Robot GetPose)
    if (key == ' ' && Calibration::collectMode) {
        int idx = Calibration::collectCount;
        if (idx >= Calibration::MAX_COLLECT_POINTS) {
            std::cout << "[CALIB] Max " << Calibration::MAX_COLLECT_POINTS << " points reached" << std::endl;
            return;
        }

        // 读取 Touch 原始设备坐标 (未变换)
        hduVector3Dd rawTouch;
        EnterCriticalSection(&appState.devicePosMutex);
        rawTouch = appState.devicePos;
        LeaveCriticalSection(&appState.devicePosMutex);

        // 读取机械臂实际位姿
        EnterCriticalSection(&appState.robotPoseMutex);
        double rx = appState.robotActualPose.x;
        double ry = appState.robotActualPose.y;
        double rz = appState.robotActualPose.z;
        LeaveCriticalSection(&appState.robotPoseMutex);

        // 存储 (原始Touch, Robot实际)
        Calibration::collectTouch[idx][0] = rawTouch[0];
        Calibration::collectTouch[idx][1] = rawTouch[1];
        Calibration::collectTouch[idx][2] = rawTouch[2];
        Calibration::collectRobot[idx][0] = rx;
        Calibration::collectRobot[idx][1] = ry;
        Calibration::collectRobot[idx][2] = rz;
        Calibration::collectCount++;

        std::cout << "[CALIB] Point " << Calibration::collectCount << " recorded:"
                  << " Touch(" << rawTouch[0] << "," << rawTouch[1] << "," << rawTouch[2] << ")"
                  << " -> Robot(" << rx << "," << ry << "," << rz << ")"
                  << std::endl;
        return;
    }

    // Space during TCP calibration: 记录法兰位姿 (笔尖对准固定点)
    if (key == ' ' && TcpCalibration::collectMode) {
        int idx = TcpCalibration::collectCount;
        if (idx >= TcpCalibration::MAX_COLLECT_POSES) {
            std::cout << "[TCP-CALIB] Max " << TcpCalibration::MAX_COLLECT_POSES << " poses reached" << std::endl;
            return;
        }
        EnterCriticalSection(&appState.robotPoseMutex);
        double p[6] = {
            appState.robotActualPose.x, appState.robotActualPose.y, appState.robotActualPose.z,
            appState.robotActualPose.rx, appState.robotActualPose.ry, appState.robotActualPose.rz
        };
        LeaveCriticalSection(&appState.robotPoseMutex);

        for (int i = 0; i < 6; i++) TcpCalibration::collectPose[idx][i] = p[i];
        TcpCalibration::collectCount++;
        std::cout << "[TCP-CALIB] Pose " << TcpCalibration::collectCount << " recorded: ("
                  << p[0] << "," << p[1] << "," << p[2] << "," << p[3] << "," << p[4] << "," << p[5] << ")"
                  << std::endl;
        return;
    }

    // 's': 求解标定并保存
    if ((key == 's' || key == 'S') && Calibration::collectMode) {
        if (Calibration::collectCount < 3) {
            std::cout << "[CALIB] Need at least 3 points, have "
                      << Calibration::collectCount << std::endl;
            return;
        }

        // 构建点对列表
        std::vector<std::pair<Vec3, Vec3>> pairs;
        for (int i = 0; i < Calibration::collectCount; i++) {
            pairs.push_back({
                Vec3(Calibration::collectTouch[i][0],
                     Calibration::collectTouch[i][1],
                     Calibration::collectTouch[i][2]),
                Vec3(Calibration::collectRobot[i][0],
                     Calibration::collectRobot[i][1],
                     Calibration::collectRobot[i][2])
            });
        }

        KabschResult result = solveKabsch(pairs);
        if (!result.valid) {
            std::cout << "[CALIB] Solver failed — points may be degenerate" << std::endl;
            return;
        }

        // 写入标定状态
        for (int i = 0; i < 9; i++) Calibration::R[i] = result.R[i];
        for (int i = 0; i < 3; i++) Calibration::t[i] = result.t[i];
        Calibration::rmsError = result.rmsError;
        Calibration::enabled = true;

        // 保存到文件
        Calibration::save("calibration.json");

        std::cout << "\n[CALIB] Solved! RMS error = " << result.rmsError << " mm" << std::endl;
        std::cout << "[CALIB] R = [" << result.R[0] << ", " << result.R[1] << ", " << result.R[2]
                  << "; " << result.R[3] << ", " << result.R[4] << ", " << result.R[5]
                  << "; " << result.R[6] << ", " << result.R[7] << ", " << result.R[8] << "]" << std::endl;
        std::cout << "[CALIB] t = [" << result.t[0] << ", " << result.t[1] << ", " << result.t[2] << "]" << std::endl;
        std::cout << "[CALIB] Saved to calibration.json" << std::endl;

        RelayCore::instance().sendCalibStatus();

        Calibration::cancelCollect();
        return;
    }

    // 's' during TCP calibration: 求解 TCP 偏移并保存
    if ((key == 's' || key == 'S') && TcpCalibration::collectMode) {
        if (TcpCalibration::collectCount < 3) {
            std::cout << "[TCP-CALIB] Need at least 3 poses, have "
                      << TcpCalibration::collectCount << std::endl;
            return;
        }
        double off[3]; double rms;
        if (!TcpCalibration::solve(TcpCalibration::collectPose, TcpCalibration::collectCount, off, rms)) {
            std::cout << "[TCP-CALIB] Solver failed — keep tip fixed, vary orientation" << std::endl;
            return;
        }
        for (int i = 0; i < 3; i++) TcpCalibration::offset[i] = off[i];
        TcpCalibration::rmsError = rms;
        TcpCalibration::enabled = true;
        TcpCalibration::save(CalibStore::fileFor("tcp_calib.json"));
        std::cout << "\n[TCP-CALIB] Solved! TCP offset = [" << off[0] << ", " << off[1] << ", " << off[2]
                  << "] mm, RMS = " << rms << " mm" << std::endl;
        std::cout << "[TCP-CALIB] Saved to tcp_calib.json" << std::endl;
        TcpCalibration::cancelCollect();
        return;
    }

    // ===== FK 实机验证模式 =====
    // 'v': 切换 FK 验证采集模式
    // Space: 记录 (关节角度 j1..j6, GetPose 实际位姿)
    // 'f': 运行 FK 对比分析并输出报告

    if (key == 'v' || key == 'V') {
        if (!FkValidate::mode) {
            cancelOtherCaptureModes('v');
            FkValidate::mode = true;
            FkValidate::count = 0;
            std::cout << "\n[FK-VAL] Mode ON — move robot to different poses,"
                      << " press SPACE to record, 'f' to analyze, 'v' to exit"
                      << std::endl;
        } else {
            FkValidate::mode = false;
            std::cout << "[FK-VAL] Mode OFF (" << FkValidate::count
                      << " points discarded)" << std::endl;
        }
        return;
    }

    if (key == ' ' && FkValidate::mode) {
        int idx = FkValidate::count;
        if (idx >= FkValidate::MAX_POINTS) {
            std::cout << "[FK-VAL] Max " << FkValidate::MAX_POINTS
                      << " points reached, press 'f' to analyze" << std::endl;
            return;
        }

        EnterCriticalSection(&appState.robotPoseMutex);
        FkValidate::joints[idx][0] = appState.robotActualPose.j1;
        FkValidate::joints[idx][1] = appState.robotActualPose.j2;
        FkValidate::joints[idx][2] = appState.robotActualPose.j3;
        FkValidate::joints[idx][3] = appState.robotActualPose.j4;
        FkValidate::joints[idx][4] = appState.robotActualPose.j5;
        FkValidate::joints[idx][5] = appState.robotActualPose.j6;
        FkValidate::actualPos[idx][0] = appState.robotActualPose.x;
        FkValidate::actualPos[idx][1] = appState.robotActualPose.y;
        FkValidate::actualPos[idx][2] = appState.robotActualPose.z;
        LeaveCriticalSection(&appState.robotPoseMutex);

        snprintf(FkValidate::labels[idx], 64, "pose_%d", FkValidate::count);
        FkValidate::count++;

        std::cout << "[FK-VAL] Point " << FkValidate::count << " recorded: J=("
                  << FkValidate::joints[idx][0] << "," << FkValidate::joints[idx][1] << ","
                  << FkValidate::joints[idx][2] << "," << FkValidate::joints[idx][3] << ","
                  << FkValidate::joints[idx][4] << "," << FkValidate::joints[idx][5] << ") "
                  << "Pose=(" << FkValidate::actualPos[idx][0] << ","
                  << FkValidate::actualPos[idx][1] << ","
                  << FkValidate::actualPos[idx][2] << ")"
                  << std::endl;
        return;
    }

    if ((key == 'f' || key == 'F') && FkValidate::mode) {
        if (FkValidate::count < 1) {
            std::cout << "[FK-VAL] No points recorded" << std::endl;
            return;
        }

        std::cout << "\n======================================================" << std::endl;
        std::cout << "  FK Validation: C++ URDF FK vs Actual GetPose" << std::endl;
        std::cout << "  Points: " << FkValidate::count << std::endl;
        std::cout << "======================================================" << std::endl;
        printf("\n%-7s | %9s %9s %9s | %9s %9s %9s | %8s\n",
               "Point", "FK.x", "FK.y", "FK.z",
               "Actual.x", "Actual.y", "Actual.z", "Err(mm)");
        printf("--------|-----------|-----------|-----------|-----------|-----------|-----------|----------\n");

        double maxErr = 0, sumErr = 0, sumSqErr = 0;
        int maxIdx = 0;

        for (int i = 0; i < FkValidate::count; i++) {
            Vec3 fkPos = Kinematics::forwardPosition(FkValidate::joints[i]);
            double dx = fkPos.x - FkValidate::actualPos[i][0];
            double dy = fkPos.y - FkValidate::actualPos[i][1];
            double dz = fkPos.z - FkValidate::actualPos[i][2];
            double err = sqrt(dx * dx + dy * dy + dz * dz);

            printf("%-7s | %9.2f %9.2f %9.2f | %9.2f %9.2f %9.2f | %8.2f\n",
                   FkValidate::labels[i],
                   fkPos.x, fkPos.y, fkPos.z,
                   FkValidate::actualPos[i][0],
                   FkValidate::actualPos[i][1],
                   FkValidate::actualPos[i][2],
                   err);

            sumErr += err;
            sumSqErr += err * err;
            if (err > maxErr) { maxErr = err; maxIdx = i; }
        }

        double meanErr = sumErr / FkValidate::count;
        double rmsErr = sqrt(sumSqErr / FkValidate::count);

        std::cout << std::endl;
        printf("  Max  error: %.2f mm  (point %d)\n", maxErr, maxIdx);
        printf("  Mean error: %.2f mm\n", meanErr);
        printf("  RMS  error: %.2f mm\n", rmsErr);
        std::cout << std::endl;

        if (maxErr < 5.0) {
            std::cout << "  ✓ PASS — URDF parameters match real robot (error < 5mm)" << std::endl;
        } else if (maxErr < 10.0) {
            std::cout << "  ⚠ WARN — URDF parameters acceptable (error < 10mm)" << std::endl;
        } else {
            std::cout << "  ✗ FAIL — URDF parameters deviate significantly (> 10mm)" << std::endl;
            std::cout << "    → Consider URDF parameter calibration" << std::endl;
        }

        // Save to JSON
        FILE* fOut = fopen("fk_validation_result.json", "w");
        if (fOut) {
            fprintf(fOut, "{\n  \"points\": [\n");
            for (int i = 0; i < FkValidate::count; i++) {
                Vec3 fkPos = Kinematics::forwardPosition(FkValidate::joints[i]);
                double dx = fkPos.x - FkValidate::actualPos[i][0];
                double dy = fkPos.y - FkValidate::actualPos[i][1];
                double dz = fkPos.z - FkValidate::actualPos[i][2];
                double err = sqrt(dx * dx + dy * dy + dz * dz);

                fprintf(fOut,
                    "    {\"label\":\"%s\","
                    "\"joints\":[%.10g,%.10g,%.10g,%.10g,%.10g,%.10g],"
                    "\"fk_ee\":[%.4f,%.4f,%.4f],"
                    "\"actual_ee\":[%.4f,%.4f,%.4f],"
                    "\"error_mm\":%.4f}%s\n",
                    FkValidate::labels[i],
                    FkValidate::joints[i][0], FkValidate::joints[i][1],
                    FkValidate::joints[i][2], FkValidate::joints[i][3],
                    FkValidate::joints[i][4], FkValidate::joints[i][5],
                    fkPos.x, fkPos.y, fkPos.z,
                    FkValidate::actualPos[i][0],
                    FkValidate::actualPos[i][1],
                    FkValidate::actualPos[i][2],
                    err,
                    i < FkValidate::count - 1 ? "," : "");
            }
            fprintf(fOut, "  ],\n");
            fprintf(fOut, "  \"summary\": {\"max_error_mm\":%.4f, \"mean_error_mm\":%.4f, \"rms_error_mm\":%.4f}\n",
                    maxErr, meanErr, rmsErr);
            fprintf(fOut, "}\n");
            fclose(fOut);
            std::cout << "  Results saved to fk_validation_result.json" << std::endl;
        }

        std::cout << "======================================================\n" << std::endl;
        return;
    }
}

// ===== 主函数 =====
int main(int argc, char* argv[]) {
    // 解析命令行参数
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--no-robot") == 0) {
            g_noRobot = true;
        } else if (strcmp(argv[i], "--no-touch") == 0) {
            g_noTouch = true;
        }
    }

    std::cout << "=== Touch-Dobot Digital Twin System v3.0 ===" << std::endl;
    std::cout << "Robot: " << Config::ROBOT_IP << (g_noRobot ? " (DISABLED)" : "") << std::endl;
    std::cout << "Touch: " << (g_noTouch ? "DISABLED" : "enabled") << std::endl;
    std::cout << "Safety: X[" << Config::SAFE_X_MIN << "," << Config::SAFE_X_MAX
              << "] Y[" << Config::SAFE_Y_MIN << "," << Config::SAFE_Y_MAX
              << "] Z[" << Config::SAFE_Z_MIN << "," << Config::SAFE_Z_MAX << "]" << std::endl;

    // 1. GLUT 初始化 (始终执行)
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(Config::WINDOW_W, Config::WINDOW_H);
    glutInitWindowPosition(100, 100);
    glutCreateWindow("Touch-Dobot Digital Twin");
    glutHideWindow();

    glutDisplayFunc(display);
    glutIdleFunc(idle);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // 2. 初始化 Touch 设备 (--no-touch 时跳过)
    if (g_noTouch) {
        std::cout << "Touch device: SKIPPED (--no-touch)" << std::endl;
    } else {
        std::cout << "Initializing Touch device..." << std::endl;
        if (!initHapticDevice()) {
            std::cerr << "ERROR: Touch device init failed" << std::endl;
            std::cerr << "  Use --no-touch to start without Touch device." << std::endl;
            return -1;
        }
    }

    // 3. 连接机械臂 (--no-robot 时跳过)
    if (g_noRobot) {
        std::cout << "Robot: SKIPPED (--no-robot)" << std::endl;
    } else {
        // 末端负载参数必须在使能之前加载 —— EnableRobot 要用它 (见 PayloadCalibration)
        const char* payloadPath = CalibStore::resolve("payload_calib.json");
        if (payloadPath && PayloadCalibration::load(payloadPath)) {
            std::cout << "[Payload] Loaded payload_calib.json (mass=" << PayloadCalibration::massKg
                      << "kg, com=(" << PayloadCalibration::comMm[0] << ","
                      << PayloadCalibration::comMm[1] << "," << PayloadCalibration::comMm[2]
                      << ")mm, " << PayloadCalibration::poses << " poses, sign_z="
                      << (PayloadCalibration::comSignZ > 0 ? "+1" : "-1") << ")" << std::endl;
        } else {
            double m, c[3];
            PayloadCalibration::effective(m, c);
            std::cout << "[Payload] 无可用 payload_calib.json — 用种子值 mass=" << m
                      << "kg com=(" << c[0] << "," << c[1] << "," << c[2] << ")mm\n"
                      << "          实机标定: 启动后按 'm' 采多姿态 → 's' 求解" << std::endl;
        }

        std::cout << "Initializing robot via Relay..." << std::endl;
        if (!RelayCore::instance().init()) {
            std::cerr << "ERROR: Robot init failed" << std::endl;
            std::cerr << "  Use --no-robot to start without robot connection." << std::endl;
            if (!g_noTouch) cleanupHapticDevice();
            return -1;
        }
    }

    // 4. 连接 MATLAB GUI (localhost:8888)
    RelayCore::instance().initRelayReporting();

    // 4.5 启动力传感器实时读取 (30004, 125Hz)
    if (!g_noRobot) {
        RelayCore::instance().initForceReader();
    }

    // 4.6 加载力传感器标定文件
    {
        double massKg, biasF[3], biasM[3];
        const char* forcePath = CalibStore::resolve("force_calib.json");
        if (forcePath && ForceCalibration::loadFromFile(forcePath, massKg, biasF, biasM)) {
            double comZero[3] = {0};
            ForceCompensation::setCalibration(massKg, comZero, biasF, biasM);
            std::cout << "[Force] Loaded force_calib.json (mass=" << massKg
                      << "kg, bias=" << biasF[0] << "," << biasF[1] << "," << biasF[2] << "N)" << std::endl;
        } else {
            std::cout << "[Force] 无可用 force_calib.json — 按 'z' 调零。" << std::endl;
        }
    }

    // 5. 初始化诊断日志
    RobotDiagnostics::instance().init(Config::DIAGNOSTIC_LOG_PATH);

    // 6. 初始化 3D 场景 (始终执行)
    SceneRenderer::init();

    // 6.5 加载标定文件 (如存在)
    if (Calibration::load("calibration.json")) {
        std::cout << "[Calib] Loaded calibration.json (RMS="
                  << Calibration::rmsError << "mm)" << std::endl;
        RelayCore::instance().sendCalibStatus();
    } else {
        std::cout << "[Calib] No calibration file, using default axis mapping" << std::endl;
    }

    // 6.6 加载 TCP 偏移标定 (如存在)
    const char* tcpPath = CalibStore::resolve("tcp_calib.json");
    if (tcpPath && TcpCalibration::load(tcpPath)) {
        std::cout << "[TCP] Loaded tcp_calib.json (offset="
                  << TcpCalibration::offset[0] << "," << TcpCalibration::offset[1] << "," << TcpCalibration::offset[2]
                  << "mm, RMS=" << TcpCalibration::rmsError << "mm)" << std::endl;
    } else {
        std::cout << "[TCP] No tcp_calib.json — press 't' to calibrate TCP offset" << std::endl;
    }

    // 7. 启动定时器
    glutTimerFunc(Config::POSE_QUERY_INTERVAL, poseQueryTimer, 0);
    glutTimerFunc(Config::ALARM_CHECK_INTERVAL, alarmCheckTimer, 0);
    glutTimerFunc(500, jointAngleTimer, 0);
    glutTimerFunc(1000, safetyStatusTimer, 0);
    glutTimerFunc(1500, jointMarginTimer, 0);
    glutTimerFunc(2000, connectionHealthTimer, 0);

    // 8. 进入主循环
    // 刷新心跳时间戳：init()、STL 加载等启动步骤可能耗时超过 HEARTBEAT_TIMEOUT_MS
    RelayCore::instance().resetHeartbeat();
    std::cout << "\nSystem ready." << std::endl;
    std::cout << "  q/ESC: quit" << std::endl;
    if (!g_noTouch && !g_noRobot) {
        std::cout << "  Touch button 1: control robot" << std::endl;
    }
    std::cout << std::endl;

    glutMainLoop();

    return 0; // unreachable
}
