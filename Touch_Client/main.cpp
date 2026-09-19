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
#include <ctime>
#include <cstring>

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
    static double accumTcp[6];           // 同上, 但累加 @720 TCPForce
    static double accumSix[6];           // 同上, 但累加 @1304 SixForceValue (原始值)
    // 与上面三组均值【配对】的平方累加 —— 单靠均值分不出"读数很稳"与"读数在跳"。
    // 每姿态采 ~30 个样本再平均, 所以样本方差量的是【测量噪声本身】: 它与任何模型、
    // 任何重力约定都无关, 这正是模型形式检验需要的、独立于拟合残差的噪声尺子
    // (从前那条判据用的是 paramSigma ← 拟合残差, 于是模型形式错时门限跟着残差一起放松)。
    static double accumSq[6];
    static double accumTcpSq[6];
    static double accumSixSq[6];
    static double pose[MAX_POSES][6];    // Rx,Ry,Rz (deg) + X,Y,Z (mm)
    static double bias[MAX_POSES][6];    // 该姿态平均 raw 力/力矩 (@576 ActualTCPForce)
    // 该姿态平均 @720 TCPForce。两路一起采、一起报, 好对比哪一路才反映负载参数。
    // 2026-09-18 实测: 改 EnableRobot 的负载时 @576 完全不跟着变, @720 按比例变。
    static double biasTcp[MAX_POSES][6];
    // 该姿态平均 @1304 SixForceValue (原始值)。三路一起采、一起报 —— 每次求解落盘的
    // calib_poses.txt 就是给这三路做裁决用的: 传感器离线 / 读错字段 / z 通道真的死了。
    static double biasSix[MAX_POSES][6];
    // 三路各自的【样本方差】(该姿态内, 逐通道)。@1304 那一份是新的线性拟合消费的通道
    // (见 solveAndApply 的 @1304 对齐说明), 所以它随姿态一起存下来, 供 fitRaw 的
    // 模型形式检验换算成"该姿态均值的不确定度" σ = sqrt(var/N)。
    static double var[MAX_POSES][6];
    static double varTcp[MAX_POSES][6];
    static double varSix[MAX_POSES][6];
    // 每个姿态参与平均的样本数 (方差的分母就是它) —— 缺了它, sqrt(var/N) 只是半截信息。
    static int    samples[MAX_POSES];

    // 样本方差 (无偏, 除以 N−1)。用 Σx² − N·mean² 的差式: 这里的量级 (|F| ~ 10 N,
    // N ~ 30, 噪声方差 ~1e-3) 下消去误差 ~1e-13, 比噪声本身小十几个数量级; 为负
    // (纯舍入) 时夹到 0, 免得 sqrt 出 NaN。N < 2 时无方差可言, 返回 0 (= 没有噪声估计)。
    static double sampleVar(double sum, double sumsq, int n) {
        if (n < 2) return 0.0;
        const double mean = sum / (double)n;
        const double v = (sumsq - (double)n * mean * mean) / (double)(n - 1);
        return (v > 0.0) ? v : 0.0;
    }

    // 数据只有在「机械臂配置 == 采集时的配置」时才可用于复验。
    // 求解改了负载/本地补偿 -> 已采数据作废 (拒绝 report 判定)。
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
        for (int i = 0; i < 6; i++) {
            accum[i] = 0.0; accumTcp[i] = 0.0; accumSix[i] = 0.0;
            accumSq[i] = 0.0; accumTcpSq[i] = 0.0; accumSixSq[i] = 0.0;
        }
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
        for (int i = 0; i < 6; i++) {
            accum[i] = 0.0; accumTcp[i] = 0.0; accumSix[i] = 0.0;
            accumSq[i] = 0.0; accumTcpSq[i] = 0.0; accumSixSq[i] = 0.0;
        }
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

        for (int i = 0; i < 6; i++) {
            accum[i] += fd.raw[i];
            accumTcp[i] += fd.tcpForce[i];   // 同时采 @720, 见 biasTcp 的说明
            accumSix[i] += fd.sixForceRaw[i]; // 同时采 @1304, 见 biasSix 的说明
            // 平方累加: 均值不变, 但多出"这批样本跳得有多厉害"这一维 (见 accumSq 的说明)。
            accumSq[i]    += fd.raw[i] * fd.raw[i];
            accumTcpSq[i] += fd.tcpForce[i] * fd.tcpForce[i];
            accumSixSq[i] += fd.sixForceRaw[i] * fd.sixForceRaw[i];
        }
        avgCount++;

        if (now - sampleStartMs < AVG_MS) return;

        // 采样窗口结束
        sampling = false;
        if (avgCount < MIN_SAMPLES) {
            std::cout << "[BIAS] 有效样本太少 (" << avgCount << "), 本次作废" << std::endl;
            return;
        }
        if (count >= MAX_POSES) return;
        for (int i = 0; i < 6; i++) {
            bias[count][i]    = accum[i] / avgCount;
            biasTcp[count][i] = accumTcp[i] / avgCount;
            biasSix[count][i] = accumSix[i] / avgCount;
            // 方差与均值【同一批样本】算出来, 一一对应 —— 后面 fitRaw 要的正是
            // "这个均值有多准", 即 σ = sqrt(var/N)。
            var[count][i]    = sampleVar(accum[i],    accumSq[i],    avgCount);
            varTcp[count][i] = sampleVar(accumTcp[i], accumTcpSq[i], avgCount);
            varSix[count][i] = sampleVar(accumSix[i], accumSixSq[i], avgCount);
        }
        samples[count] = avgCount;

        EnterCriticalSection(&appState.robotPoseMutex);
        pose[count][0] = appState.robotActualPose.rx;
        pose[count][1] = appState.robotActualPose.ry;
        pose[count][2] = appState.robotActualPose.rz;
        pose[count][3] = appState.robotActualPose.x;
        pose[count][4] = appState.robotActualPose.y;
        pose[count][5] = appState.robotActualPose.z;
        LeaveCriticalSection(&appState.robotPoseMutex);

        // 三路并排: @576 派生量 / @720 关节电流反推 / @1304 原始读数 —— 看的是"哪一路
        // 才反映负载", 以及传感器是否真的在线。姿态行保持原有前缀不变 (R=...deg),
        // 力向量各自带偏移标签另起一行, 免得三路被看混。
        // 在线状态取自本帧快照 fd (已在锁内拷贝), 不再去读 appState。
        printf("[BIAS] Pose %d: R=(%+.1f,%+.1f,%+.1f)deg\n",
               count + 1, pose[count][0], pose[count][1], pose[count][2]);
        printf("       @576  F=(%+.3f,%+.3f,%+.3f)  M=(%+.3f,%+.3f,%+.3f)\n",
               bias[count][0], bias[count][1], bias[count][2],
               bias[count][3], bias[count][4], bias[count][5]);
        printf("       @720  F=(%+.3f,%+.3f,%+.3f)  M=(%+.3f,%+.3f,%+.3f)\n",
               biasTcp[count][0], biasTcp[count][1], biasTcp[count][2],
               biasTcp[count][3], biasTcp[count][4], biasTcp[count][5]);
        printf("       @1304 F=(%+.3f,%+.3f,%+.3f)  M=(%+.3f,%+.3f,%+.3f)"
               "   SixForceOnline=%d\n",
               biasSix[count][0], biasSix[count][1], biasSix[count][2],
               biasSix[count][3], biasSix[count][4], biasSix[count][5],
               fd.sixForceOnline);
        // 均值旁边报出【实测噪声】(样本标准差, N 个样本), 以及均值本身的不确定度 sd/sqrt(N)。
        // 求解的模型形式检验只用后者 (它才是"输入值准不准"), 报前者是因为它才是
        // "传感器有多吵"的直观量。数据流有问题时 (抖动远大于典型值) 这两行会先露馅。
        printf("       noise(@1304): sd=(%.4f,%.4f,%.4f)  sd=(%.4f,%.4f,%.4f)"
               "   N=%d  -> sd/sqrt(N)=(%.4f,%.4f,%.4f)\n",
               sqrt(varSix[count][0]), sqrt(varSix[count][1]), sqrt(varSix[count][2]),
               sqrt(varSix[count][3]), sqrt(varSix[count][4]), sqrt(varSix[count][5]),
               avgCount,
               sqrt(varSix[count][0] / avgCount), sqrt(varSix[count][1] / avgCount),
               sqrt(varSix[count][2] / avgCount));
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

        // ===== 同一批数据的第二路来源: @720 TCPForce =====
        // 两路一起报, 才看得出哪一路才随【配置的负载】变化 —— 负载标定该读那一路。
        // 2026-09-18 实测: 改 EnableRobot 的负载时 @576 纹丝不动, @720 按 1.93 倍变。
        double tLoF[3], tHiF[3], tLoM[3], tHiM[3];
        for (int a = 0; a < 3; a++) {
            tLoF[a] = tHiF[a] = biasTcp[0][a];
            tLoM[a] = tHiM[a] = biasTcp[0][a + 3];
        }
        for (int p = 1; p < count; p++) {
            for (int a = 0; a < 3; a++) {
                if (biasTcp[p][a]     < tLoF[a]) tLoF[a] = biasTcp[p][a];
                if (biasTcp[p][a]     > tHiF[a]) tHiF[a] = biasTcp[p][a];
                if (biasTcp[p][a + 3] < tLoM[a]) tLoM[a] = biasTcp[p][a + 3];
                if (biasTcp[p][a + 3] > tHiM[a]) tHiM[a] = biasTcp[p][a + 3];
            }
        }
        double tsF[3], tsM[3];
        for (int a = 0; a < 3; a++) { tsF[a] = tHiF[a] - tLoF[a]; tsM[a] = tHiM[a] - tLoM[a]; }
        const double tSpanF = sqrt(tsF[0]*tsF[0] + tsF[1]*tsF[1] + tsF[2]*tsF[2]);
        const double tSpanM = sqrt(tsM[0]*tsM[0] + tsM[1]*tsM[1] + tsM[2]*tsM[2]);

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
                  << " (信息性: 数据定不了符号, 不影响补偿)" << std::endl;
        std::cout << "======================================================" << std::endl;
        printf("  跨姿态极差(力):   Fx=%.3f  Fy=%.3f  Fz=%.3f   |ΔF|=%.3f N\n",
               sF[0], sF[1], sF[2], spanF);
        printf("  跨姿态极差(力矩): Mx=%.4f My=%.4f Mz=%.4f |ΔM|=%.4f Nm\n",
               sM[0], sM[1], sM[2], spanM);
        printf("  --- 同一批数据的第二路 @720 TCPForce ---\n");
        printf("  @720 跨姿态极差(力): |ΔF|=%.3f N   跨姿态极差(力矩): |ΔM|=%.4f Nm\n",
               tSpanF, tSpanM);
        printf("       F: %.3f/%.3f/%.3f   M: %.4f/%.4f/%.4f\n",
               tsF[0], tsF[1], tsF[2], tsM[0], tsM[1], tsM[2]);
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
            std::cout << "   → 按 's' 用这批数据【求解】残余量, 写进本地补偿 (force_calib.json)"
                      << std::endl;
        }
        std::cout << std::endl;
    }

    // 每次负载求解尝试都落一行 —— 控制台会滚掉, 而"这次标定到底做了什么"必须能追溯。
    // 一行一次尝试, 便于 grep 与表格工具直接读。
    //
    // 表头只在【文件不存在或为空】时写。不要用进程内 static 标志位: 那个标志每次启动都是
    // false, 会截断历次记录 —— 与"可追溯"的立意在字面上相反。打开方式也必须是 "a+",
    // 见下面的说明。
    //
    // r 可为 nullptr: 求解【之前】就返回的出口 (已上锁 / 姿态数不足 / 正在采样) 没有 Result,
    // 此时各数值列记 "-"。
    static void logCalibAttempt(const char* outcome, const PayloadCalibration::Result* r)
    {
        // fileFor 返回 static 缓冲, 调用方必须立即拷贝 (头文件已注明)。
        char path[512];
        snprintf(path, sizeof(path), "%s", CalibStore::fileFor("calib_log.txt"));

        // 【只用 "a+" 打开, 永不截断。】不要先试 "r" 再决定 "w"/"a": "读不了"(被占用/权限)
        // 与"不存在"会被混为一谈, 而前者会走 "w" 把历次记录整个删掉 —— 毁的正是这个文件
        // 存在的理由。(Task 11 复审 Minor #50)
        FILE* f = fopen(path, "a+");
        if (!f) return;
        bool needHeader = true;
        if (fseek(f, 0, SEEK_END) == 0) needHeader = (ftell(f) == 0);
        if (needHeader) {
            fprintf(f, "# 负载标定尝试记录 (每次按 's' 一行)\n");
            fprintf(f, "# time | poses | rmsF_N | rmsM_Nm | psi_deg | dm_kg | mass_kg"
                       " | comZ_mm | outcome\n");
        }
        char sPoses[16], sRmsF[16], sRmsM[16], sPsi[16], sDm[16], sMass[16], sComZ[16];
        if (r) {
            snprintf(sPoses, sizeof(sPoses), "%d",    r->poses);
            snprintf(sRmsF,  sizeof(sRmsF),  "%.4f",  r->rmsForceN);
            snprintf(sRmsM,  sizeof(sRmsM),  "%.4f",  r->rmsMomentNm);
            snprintf(sPsi,   sizeof(sPsi),   "%+.1f", r->sensorYawDeg);
            snprintf(sDm,    sizeof(sDm),    "%+.4f", r->dm);
            snprintf(sMass,  sizeof(sMass),  "%.4f",  r->massKg);
            snprintf(sComZ,  sizeof(sComZ),  "%+.1f", r->comMm[2]);
        } else {
            char* cols[7] = {sPoses, sRmsF, sRmsM, sPsi, sDm, sMass, sComZ};
            for (int i = 0; i < 7; i++) strcpy(cols[i], "-");
        }
        char ts[24];
        const std::time_t now = std::time(nullptr);
        std::tm tmInfo;
        localtime_s(&tmInfo, &now);
        strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", &tmInfo);

        fprintf(f, "%s | %s | %s | %s | %s | %s | %s | %s | %s\n",
                ts, sPoses, sRmsF, sRmsM, sPsi, sDm, sMass, sComZ, outcome);
        fclose(f);
    }

    // 每次求解尝试都把这批姿态连同三路力数据落成 CSV —— 控制台会滚掉, 而"三个力通道
    // 到底哪个是真的"要靠并排的原始数据裁决 (@576 派生量 / @1304 原始读数)。
    // 与 logCalibAttempt 同址 (calib\calib_poses.txt), 同样【只追加, 永不截断】:
    // 打开方式必须是 "a+", 绝不能先试 "r" 再决定 "w"/"a" (理由同 logCalibAttempt)。
    // 表头每次尝试都写, 一个块自描述, 便于单独截一段去分析。
    // 姿态列的顺序是 BiasCheck 的存储顺序 [rx,ry,rz,x,y,z], 【不是】求解器要的
    // [x,y,z,rx,ry,rz] —— 表头已注明, 读的人别搞反。
    static void logPoseData()
    {
        // fileFor 返回 static 缓冲, 调用方必须立即拷贝 (头文件已注明)。
        char path[512];
        snprintf(path, sizeof(path), "%s", CalibStore::fileFor("calib_poses.txt"));

        FILE* f = fopen(path, "a+");
        if (!f) return;

        char ts[24];
        const std::time_t now = std::time(nullptr);
        std::tm tmInfo;
        localtime_s(&tmInfo, &now);
        strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", &tmInfo);

        int online = -1;
        EnterCriticalSection(&appState.forceDataMutex);
        online = appState.forceData.sixForceOnline;
        LeaveCriticalSection(&appState.forceDataMutex);

        fprintf(f, "# attempt %s  poses=%d  sixForceOnline=%d\n", ts, count, online);
        fprintf(f, "# rx,ry,rz,x,y,z,F576x,F576y,F576z,M576x,M576y,M576z,"
                   "F1304x,F1304y,F1304z,M1304x,M1304y,M1304z,"
                   "N1304,sdF1304x,sdF1304y,sdF1304z,sdM1304x,sdM1304y,sdM1304z\n");
        // 末尾 7 列是 2026-09-19 加的 (模型形式检验要实测噪声, 而噪声是【逐姿态】采出来的):
        // N1304 = 该姿态的样本数, sd* = 该姿态内 @1304 各通道的样本标准差。
        // 均值那 18 列的形状【没有】动 —— 离线分析/历史脚本读它们仍然照旧。
        // 有了 (N, sd) 就能还原"均值的不确定度" sd/sqrt(N), 也就是 fitRaw 要的那把尺子。
        for (int i = 0; i < count; i++) {
            fprintf(f, "%.1f,%.1f,%.1f,%.3f,%.3f,%.3f,"
                       "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
                       "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
                       "%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                    pose[i][0], pose[i][1], pose[i][2],
                    pose[i][3], pose[i][4], pose[i][5],
                    bias[i][0], bias[i][1], bias[i][2],
                    bias[i][3], bias[i][4], bias[i][5],
                    biasSix[i][0], biasSix[i][1], biasSix[i][2],
                    biasSix[i][3], biasSix[i][4], biasSix[i][5],
                    samples[i],
                    sqrt(varSix[i][0]), sqrt(varSix[i][1]), sqrt(varSix[i][2]),
                    sqrt(varSix[i][3]), sqrt(varSix[i][4]), sqrt(varSix[i][5]));
        }
        fclose(f);
    }

    // 's': 用已采数据最小二乘求解负载参数 → 把【残余量】装进本地补偿的本会话生效值;
    //      force_calib.json 里落盘的 mass_kg 却是 0。
    //      ⚠ 内存与落盘故意不同 (差的就是"本会话"), 缘由见下面 [本地补偿] 一节的说明 —— 别去"对齐"。
    //      机械臂【是】接受负载参数的: 新值随连接时序的 EnableRobot(1.5,...) 下发。所以重启之后
    //      由机械臂负责整份负载, 本地补偿不再减残余 —— 那就是落盘值写 0 的原因。
    static void solveAndApply() {
        if (solveLocked) {
            std::cout << "\n[BIAS] !! 已连续 " << consecutiveFails << " 次判定结果不合理, 已停止求解。\n"
                      << "       [BIAS] !! 判据只剩一条: 拟合残差 (力) 超阈。检查装夹是否松动"
                      << " / 力传感器是否受挤压 / 姿态覆盖是否足够。\n"
                      << "       [BIAS] !! 处理后按 'm' 重新采集 (计数会清零)。" << std::endl;
            logCalibAttempt("REJECTED locked_out", nullptr);
            return;
        }
        if (count < 4) {
            std::cout << "[BIAS] 求解至少需要 4 个姿态 (当前 " << count
                      << "), 建议 6~8 个" << std::endl;
            char outcome[128];
            snprintf(outcome, sizeof(outcome), "REJECTED too_few_poses (count=%d)", count);
            logCalibAttempt(outcome, nullptr);
            return;
        }
        // 姿态数够了就落盘 —— 【在任何拒绝判据之前】: 被拒绝的那几次同样要留下数据,
        // 否则"为什么被拒"这件事就只剩控制台上滚掉的那几行。只追加, 见 logPoseData。
        logPoseData();
        // 采样中途不允许求解
        if (sampling) {
            std::cout << "[BIAS] 正在采样, 稍后再求解" << std::endl;
            logCalibAttempt("REJECTED sampling_in_progress", nullptr);
            return;
        }

        // ===== 求解基线 (m_cfg / c_cfg): 机械臂【自报】它当前在用的负载 =====
        // 求解器测的是"真值 − 基线"的差 (dp = m_true·c_true − m_cfg·c_cfg), 所以要把 dp 折回
        // 【绝对】质心, 就必须知道基线是谁。机械臂自报的值 (30004 帧 @1168) 才是权威:
        //   · 它说的是"机械臂实际在用什么", 不是"我们以为它该用什么" —— 实测过两者不一致
        //     (自报 0.4061 kg vs 下发 0.404 kg);
        //   · 有了忠实基线, 绝对质心的换算就是直接的: 没有 centerZ 变号要折, 歧义消失。
        // dm/dp 本身与基线无关 (差商把 m_cfg/c_cfg 整项消掉), 变的是折算出来的绝对值。
        double mCfg, cCfg[3];
        bool baselineFromRobot = false;
        {
            EnterCriticalSection(&appState.forceDataMutex);
            baselineFromRobot = appState.forceData.payloadEchoValid;
            if (baselineFromRobot) {
                mCfg = appState.forceData.payloadEchoLoadKg;
                for (int i = 0; i < 3; i++) {
                    cCfg[i] = appState.forceData.payloadEchoCenterMm[i];
                }
            }
            LeaveCriticalSection(&appState.forceDataMutex);
        }
        if (!baselineFromRobot) {
            // 退回我们"以为"的负载 (标定值优先, 否则种子)。【不静默】: 这条路径下折出来的
            // 绝对质心又背上 centerZ 折叠歧义 (两种解释相差 ~2·m_cfg·cz/m_true ≈ 125 mm),
            // 打印出来的 comZ 不能直接采信。
            PayloadCalibration::effective(mCfg, cCfg);
            std::cout << "\n[BIAS] !! 【警告】机械臂未回读负载 (30004 @1168 没收到 / 不合理),"
                      << " 基线退回本客户端的信念值\n"
                      << "[BIAS] !! 此路径下绝对质心 Z 再次带有 comSignZ 折叠歧义"
                      << " (两种解释约差 125 mm),\n"
                      << "[BIAS] !! 下面打印的 comZ 【不可】直接采信 —— 等机械臂回读可用后再求解。"
                      << std::endl;
        }

        // 姿态转成求解器要的 [x,y,z,rx,ry,rz]: 本模块存的是 [rx,ry,rz,x,y,z]
        static double sp[12][6];
        static double sf[12][3];
        static double sm[12][3];
        for (int i = 0; i < count; i++) {
            sp[i][0] = pose[i][3]; sp[i][1] = pose[i][4]; sp[i][2] = pose[i][5];
            sp[i][3] = pose[i][0]; sp[i][4] = pose[i][1]; sp[i][5] = pose[i][2];
            // ===== @1304 原始通道 → 本项目约定的对齐 (z 镜像) =====
            // biasSix[] 是 @1304 SixForceValue 的【原始】读数。它的 z 轴与项目其余部分用的
            // 约定 (@576 ActualTCPForce 满足的 raw = b + m·g) 【反号】—— 这是一次 z 镜像,
            // 不是旋转。2026-09-19 对 calib_poses.txt 那批 7 姿态做自由拟合 (F = b + M·g,
            // M 不限形式) 得到的证据:
            //   · M 的 2×2 (x,y) 块 det = +0.182 > 0 → 是正常旋转, 尺度 +0.4266;
            //   · M[2][2] = −0.4139 → 【负的】, 且量级与 xy 块几乎相等 → 只有 z 被翻。
            // 于是 M = m · diag(1,1,−1) · Rz(−ψ): 一个 z 镜像叠一个绕 z 的偏转。
            // 模型 (raw = b + Δm·g) 里【没有】z 反号这个自由度, 求解器就会拿【质量】去顶:
            // 直接喂原始值解出 Δm = −0.417 (非物理, solve() 直接拒), 镜像后是
            // Δm = +0.417 kg / ψ ≈ +29.5°, 与上面那个 2×2 块的角度对得上。
            //
            // 镜像按张量性质【分别】作用 —— 力与力矩的变换不同, 不能一起处理:
            //   · 力 F 是【真矢量】(polar vector): F → S·F = diag(1,1,−1)·F, 只把 Fz 取反;
            //   · 力矩 M 是【赝矢量】(axial vector): M → det(S)·S·M, 这里 S = diag(1,1,−1)、
            //     det(S) = −1 → diag(−1,−1,+1)·M, 即 Mx、My 取反而 Mz 【不动】。
            // 力/力矩两个残差就是这条对齐对不对的判据 (见下面打印的 rms)。
            // ⚠ 只对齐【喂给求解器的这一份拷贝】: biasSix[] 本身、上面的打印、以及落盘的
            //    calib_poses.txt 全部保持原始值 —— 离线分析要的就是没被动过的那一份。
            sf[i][0] = biasSix[i][0];
            sf[i][1] = biasSix[i][1];
            sf[i][2] = -biasSix[i][2];        // 真矢量: 只有 Fz 反号
            sm[i][0] = -biasSix[i][3];        // 赝矢量: Mx ...
            sm[i][1] = -biasSix[i][4];        //         ... My 反号
            sm[i][2] = biasSix[i][5];         //         Mz 不动
        }

        PayloadCalibration::Result r;
        // comSignZ 一律传 +1.0: 基线忠实 (机械臂自报) 时没有 centerZ 变号要折, 折算就是直接的。
        // 走的是上面那条退回路径也一样传 +1.0 —— 那里已经就"comZ 不可采信"报过警告了。
        if (!PayloadCalibration::solve(sp, sf, sm, count, mCfg, cCfg, +1.0, r)) {
            std::cout << "[BIAS] 求解失败 — 姿态数不足/退化(姿态太接近)/解非物理。\n"
                      << "       请确认各姿态差异足够大 (跨度≥30°, 且笔有水平/朝上的姿态)"
                      << std::endl;
            char outcome[128];
            snprintf(outcome, sizeof(outcome),
                     "REJECTED degenerate_or_nonphysical (count=%d)", count);
            logCalibAttempt(outcome, nullptr);
            return;
        }

        std::cout << "\n======================================================" << std::endl;
        std::cout << "  负载参数求解结果 (" << r.poses << " 个姿态)" << std::endl;
        std::cout << "======================================================" << std::endl;
        // 基线必须打印: 绝对质心是由"基线 + 解出的差"折出来的, 看着基线才能判读下面那行 comZ。
        printf("  基线:    %s  load=%.3f kg  center=(%.1f, %.1f, %.1f) mm\n",
               baselineFromRobot ? "机械臂自报 [30004 @1168]" : "★ 本客户端信念值 [机械臂未回读] ★",
               mCfg, cCfg[0], cCfg[1], cCfg[2]);
        printf("  质量:    当前 %.3f kg   →  修正 %+.3f kg   →   %.3f kg\n",
               mCfg, r.dm, r.massKg);
        printf("  质心 X/Y/Z: 当前 (%.1f, %.1f, %.1f) mm   →   解出 (%.1f, %.1f, %+.1f) mm\n",
               cCfg[0], cCfg[1], cCfg[2], r.comMm[0], r.comMm[1], r.comMm[2]);
        printf("  拟合残差: 力 %.4f N   力矩 %.4f N·m\n", r.rmsForceN, r.rmsMomentNm);
        // 传感器安装偏转角: 【由数据解出, 不是常量】。它在 [-180,180] 上以 0.5° 步长扫出,
        // 取力残差最小者 —— 90° 那套种子值与 ≈80° 正好差在 0.30 N 的门限两侧 (见 Config.h)。
        // 操作者要看得到它: 换工具/重装传感器后这个数会变, 变了才说明标定真的重新定了模型。
        printf("  传感器安装偏转角 psi: %+.1f deg (扫描 [-180,180]/0.5°, 取自最小力残差)\n",
               r.sensorYawDeg);
        // 【上面这两个"绝对"值不再只是记录/显示】: comMm 随 payload_calib.json 持久化, 并在下次
        // 启动的连接时序里下发 (RelayCore 的 EnableRobot)。符号一旦选错, 下发给机械臂的就是错
        // 的那一份质心, 机械臂会照它补偿 —— 所以这里的不确定性必须由人核一次。
        // CZ 符号: 数据定不了它 (两种解释的拟合残差完全相同), 从前靠实机探针裁决, 探针已废除
        // —— 唯一的判据是外部锚点: 取靠近 Config::ROBOT_PAYLOAD_SEED_CZ_MM 的那个候选。
        printf("  CZ 符号提示: 数据区分不了两种解释 — 物理质心 Z = %+.1f mm (+1) / %+.1f mm (-1)\n",
               r.cTrueZ[0], r.cTrueZ[1]);
        printf("               ⚠ 此符号【现在会下发】, 必须核: 正确候选应靠近种子 %.1f mm;"
               " 落在 ~195 mm 就是符号反了\n", Config::ROBOT_PAYLOAD_SEED_CZ_MM);

        // ===== 合理性判据 =====
        // 只剩一条: 拟合残差。符号探针废除之后, 没有任何"待实测裁决"的东西需要挡 ——
        // 本标定的输出是残余量 (dm/dp), 它只取决于拟合质量, 不取决于机械臂接不接受负载参数。
        const double RMS_F_GOOD = 0.10;   // N — 到这个量级才说明模型与数据一致
        const double RMS_F_MAX  = 0.30;   // N — 超过即不合理
        const bool   fitGood = (r.rmsForceN < RMS_F_GOOD);
        const bool   fitOk   = (r.rmsForceN < RMS_F_MAX);
        const bool   reasonable = fitOk;

        std::cout << "------------------------------------------------------" << std::endl;
        std::cout << "  合理性评估 (拟合残差, 不满足即拒绝保存):" << std::endl;
        if (fitGood) {
            printf("    ✓ 拟合残差 %.4f N ≈ 噪声本底 (< %.2f N)\n", r.rmsForceN, RMS_F_GOOD);
        } else if (fitOk) {
            printf("    ✓ 拟合残差 %.4f N 在容许范围内 (< %.2f N)\n", r.rmsForceN, RMS_F_MAX);
        } else {
            printf("    ✗ 拟合残差 %.4f N 超过阈值 %.2f N — 模型解释不了这批数据\n",
                   r.rmsForceN, RMS_F_MAX);
        }

        std::cout << "------------------------------------------------------" << std::endl;
        if (!reasonable) {
            consecutiveFails++;
            std::cout << "  判定: ✗ 不合理 — 已【拒绝保存】: payload_calib.json 与本地补偿均未改动 (第 "
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
                std::cout << "  [BIAS] !! 处理后按 'm' 重新采集 (计数会清零)。" << std::endl;
            }
            std::cout << std::endl;
            // 不需要"恢复机械臂原参数": 判据跑在任何下发【之前】(探针已废除, 这条路里没有
            // 任何东西动过机械臂), 内存生效值也从头到尾没改过。
            char outcome[128];
            snprintf(outcome, sizeof(outcome), "REJECTED rmsF=%.4f", r.rmsForceN);
            logCalibAttempt(outcome, &r);
            return;
        }
        consecutiveFails = 0;   // 成功一次即清零

        // ψ 必须【先于】写本地补偿生效: 上面解出的 dm/dp 是在这个 ψ 下算出来的, 而
        // ForceCompensation 后续每一帧都用 TcpCalibration::gravitySensorFrame 的重力模型减
        // 这一份残余。两者用不同的 ψ, 等于减错方向的力 —— 这正是本模块最怕的"安静地错"。
        TcpCalibration::setSensorYawDeg(r.sensorYawDeg);

        // 记进内存生效值 (只影响机械臂侧显示与下次求解的基准)。
        // (它同时把 psi 记进 PayloadCalibration 的生效值, 供 save() 落盘。)
        PayloadCalibration::applyResult(r);

        // ===== 本次标定的输出: 本地补偿的【本会话】值 ≠ force_calib.json 的【重启后】值 =====
        // 两处数值故意不同, 别把它们"对齐" —— 对齐就是把同一个误差减两次。
        //
        //   · 本会话 (内存): 机械臂要到【下次重启】才拿到新负载, 这一整轮它用的还是旧值。所以
        //     本地补偿必须继续把解出的残余量减掉 —— 不减, 这轮的读数就是错的。
        //   · 落盘 (force_calib.json): 那份文件描述的是【重启之后】的稳态。那时机械臂已经背上
        //     新负载, 残余量归零, 本地补偿不该再减任何东西, 所以 mass_kg 写 0。
        //
        // 从前两处都写残余量, 重启后机械臂的新负载与本地残余叠加, 同一份误差被减两次
        // (~0.25 kg 量级)。这就是本次拆分修掉的 bug。
        //
        // 机械臂那边【确实会采用】这份负载: 连接时序里会下发 EnableRobot(1.5,...)。2026-09-19
        // 实机证实它生效 —— 改 payload_calib.json 后重启, 机械臂快速撞向关节限位。所以重标定
        // 要客户端重启之后才算真正闭环。
        //
        // 拟合出的 dm/dp 就是那一份【残余本身】(相对机械臂当前实际配置, 差商消掉了零偏),
        // 所以这里不需要知道机械臂内部配的是什么值。
        //
        // 符号: ForceCompensation 做 compensated = raw - mass*gTool, gTool = Rᵀ(0,0,+9.81),
        // 与求解器 gravityTool 同一约定, 所以 mass 直接取 dm 即可(可为负)。
        std::cout << "------------------------------------------------------" << std::endl;
        std::cout << "  ✓ 标定结果 → 本地补偿 (本会话生效; 机械臂没补干净的那一份由我们减掉):"
                  << std::endl;
        // 两处落盘的成败都要进日志: 落盘失败却只记 DISPATCHED, "下次启动标定没了"在日志里
        // 看起来就像没发生过 —— 那样追溯就是假的。(机械臂侧同步失败【不算】: 它不影响标定。)
        bool calibWritesOk = true;
        {
            const double resMass = r.dm;              // 残余质量 (kg, 带符号)
            double resCom[3] = {0.0, 0.0, 0.0};       // 残余质心 (m)
            if (fabs(resMass) > 1e-6) {
                for (int i = 0; i < 3; i++) resCom[i] = r.dp[i] / resMass;   // kg·m / kg = m
            }
            double bF[3], bM[3];
            ForceCompensation::currentBias(bF, bM);
            ForceCompensation::setMassCom(resMass, resCom);   // 本会话生效 (机械臂还在用旧负载)
            if (ForceCalibration::saveToFile(CalibStore::fileFor("force_calib.json"),
                                             0.0, bF, bM)) {   // 落盘 0 = 重启后由机械臂负责
                printf("  [本地补偿] 本会话生效: 残余质量 %+.4f kg  残余质心 (%+.1f, %+.1f, %+.1f) mm\n",
                       resMass, resCom[0] * 1000.0, resCom[1] * 1000.0, resCom[2] * 1000.0);
                std::cout << "  [本地补偿] 落盘 mass_kg = 0 — 重启后机械臂背上新负载, 残余归零,"
                          << " 本地不再减" << std::endl;
                std::cout << "  [本地补偿] ⚠ 内存 " << resMass << " kg ≠ 落盘 0: 差的就是【本次会话】。"
                          << "这是设计, 不是笔误" << std::endl;
                std::cout << "             重启之前机械臂用的仍是【旧负载】, 所以这一轮必须继续减这份残余"
                          << std::endl;
            } else {
                std::cerr << "  [本地补偿] !! force_calib.json 写入失败 (本会话内存中的残余补偿已生效,"
                          << " 但它没能落盘)" << std::endl;
                calibWritesOk = false;
            }
        }

        if (!PayloadCalibration::save(CalibStore::fileFor("payload_calib.json"))) {
            std::cerr << "[BIAS] !! payload_calib.json 写入失败" << std::endl;
            calibWritesOk = false;
        } else {
            std::cout << "  已保存 payload_calib.json (下次启动自动加载)" << std::endl;
        }
        // 机械臂侧那份负载【不在运行时下发】—— 2026-09-19 实机证实: 运行中改负载会让机械臂
        // 猛地动起来 (改 payload_calib.json 后重启, 连接时序下发 EnableRobot(1.5,...), 机械臂
        // 快速撞向关节限位并报错)。所以新负载只在【下次启动的连接时序】里下发
        // (RelayCore.cpp 的 enableRobotWithPayload), 本次【一发都不发】。
        // 本次真正生效的只有上面的 [本地补偿]; 在下次重启之前, 机械臂那边用的仍是旧值。
        std::cout << "  [机械臂侧] 本次【未】向机械臂下发负载 (运行时改负载会让机械臂动) —" << std::endl;
        std::cout << "             机械臂仍在用旧值 (由上面的本地补偿修正读数); "
                  << "新值要【下次重启】才随连接下发" << std::endl;
        // 重启就是把 com_mm 交给机械臂的那一刻 —— 在那之前必须把上面那个不确定的符号核掉。
        // 用命令句写, 因为这条只有在重启【之前】看才有意义。
        std::cout << "  [重启前必查] 先确认上面那两个 CZ 候选里, 选中的是靠近 "
                  << Config::ROBOT_PAYLOAD_SEED_CZ_MM << " mm 的那一个 —" << std::endl;
        std::cout << "             重启会把 com_mm 下发出去; 符号反了就是 ~195 mm, 机械臂会"
                  << "据此补偿并移动。" << std::endl;
        std::cout << "             控制台滚掉也不要紧: calib\\calib_log.txt 每次尝试都记了"
                  << " comZ_mm (就是会下发的那个值), 去那里核对。" << std::endl;

        // 这批姿态是在【旧负载】下采的。复验 (report) 必须拒绝它们,
        // 否则会拿旧数据骂新参数 (曾经报出假 FAIL: 求解残差 0.06 N,
        // 紧接着的报告却报 |ΔF| = 4.0 N)。
        // (本地补偿已经改了: 同一份数据在新补偿下不再可比, 所以照样作废。)
        dataUnderCurrentPayload = false;
        std::cout << "  → 复验: 摆姿态按 SPACE 采集 (第一次 SPACE 会自动开新一批,"
                  << " 旧数据作废)" << std::endl;
        std::cout << std::endl;

        // 成功出口: 在【两处写入尝试之后】才记 —— 这样 write_failed 才能落在同一行里。
        char outcome[128];
        snprintf(outcome, sizeof(outcome), "DISPATCHED%s",
                 calibWritesOk ? "" : " write_failed");
        logCalibAttempt(outcome, &r);
    }
}

// ===== 启动零偏漂移检查 =====
// 负载参数是工具的物理属性、不随时间漂移, 所以启动【不】验证负载 —— 它按需重标。
// 会漂的是力传感器零偏, 这里便宜地查它:
//   负载正确时残余力与姿态无关, 因此任意静止姿态读一次 "补偿后读数" 就是漂移量。
// 零操作负担: 不摆姿态、不阻断、不写盘。
static bool g_zeroCheckDone = false;
static DWORD g_zeroCheckStartMs = 0;
static double g_zeroCheckAccum[3] = {0, 0, 0};
static int g_zeroCheckCount = 0;

// 启动加载 force_calib.json 是否成功 (成功才有"存储零偏"可比, 否则无可查)
static bool g_hasStoredZeroCalib = false;

static void runZeroDriftCheck(bool hasStoredZero) {
    if (g_zeroCheckDone || !hasStoredZero) return;
    // --no-robot 下没有力数据可读, 本就没有可查的东西: 直接定稿, 免得每帧空转。
    if (g_noRobot) { g_zeroCheckDone = true; return; }

    DWORD now = GetTickCount();
    if (g_zeroCheckStartMs == 0) { g_zeroCheckStartMs = now; return; }

    AppState::ForceData fd;
    EnterCriticalSection(&appState.forceDataMutex);
    fd = appState.forceData;
    LeaveCriticalSection(&appState.forceDataMutex);

    // 启动后前 2s 让读数稳定, 之后取 1s 均值
    if (fd.isStale || now - g_zeroCheckStartMs < 2000) return;

    for (int i = 0; i < 3; i++) g_zeroCheckAccum[i] += fd.filtered[i];
    g_zeroCheckCount++;
    if (now - g_zeroCheckStartMs < 3000) return;

    // 定稿
    g_zeroCheckDone = true;
    if (g_zeroCheckCount < 10) return;   // 数据太少, 本次不作结论

    const double drift = sqrt(
        (g_zeroCheckAccum[0] / g_zeroCheckCount) * (g_zeroCheckAccum[0] / g_zeroCheckCount) +
        (g_zeroCheckAccum[1] / g_zeroCheckCount) * (g_zeroCheckAccum[1] / g_zeroCheckCount) +
        (g_zeroCheckAccum[2] / g_zeroCheckCount) * (g_zeroCheckAccum[2] / g_zeroCheckCount));

    if (drift > Config::FORCE_ZERO_DRIFT_WARN_N) {
        std::cout << "\n[Force] ⚠ 零偏漂移检查: 补偿后读数 " << drift
                  << " N, 超过阈值 " << Config::FORCE_ZERO_DRIFT_WARN_N << " N" << std::endl;
        std::cout << "[Force]   两种可能:" << std::endl;
        std::cout << "[Force]     · 零偏漂了 (温度/时间)  -> 按 'z' 重新调零" << std::endl;
        std::cout << "[Force]     · 硬件有变化 (加装/拆装) -> 按 'm' 采多姿态后 's' 重标负载"
                  << std::endl;
        std::cout << "[Force]   两种都不影响继续操作, 但建议尽快处理。" << std::endl;
    } else {
        std::cout << "[Force] 零偏漂移检查: 补偿后读数 " << drift
                  << " N, 正常 (< " << Config::FORCE_ZERO_DRIFT_WARN_N << " N)" << std::endl;
    }
}

// ===== 启动运动检测器诊断 =====
// 诊断【只测不改】: 怀疑 isStill() (ForceCompensation.cpp:150) 在生产中永远为假 —— 那样的话
// step() 第 8 步的在线 EMA 零偏更新从来不跑, 而第 6 步的惯性项则一直在跑。
// 读代码判不了这件事, 得看实机静止时的噪声量级。两个可疑点, 都不是"单位写错了":
//   · vel/acc 实际就是 m/s 与 m/s² —— MotionEstimator::update 里已 mm→m 换算
//     (position * 0.001), 所以 0.002 / 0.005 的量纲命名是对的。
//     下面的单位标签是 2026-09-19 才从 mm/s 改正过来的: 旧标签把物理速度说小 1000 倍,
//     操作者照着读会算错量级。阈值常量本来就是这个量纲 (Config.h:123-124)。
//   · 真正可疑的是 dt 与实际采样间隔不符: update() 固定用 dt=1/125s, 而 step() 的唯一
//     调用方 pollForce() 自我节流到 33ms、且它读的 robotActualPose 只由 queryPose()
//     每 100ms 刷新一次 (RelayCore.cpp:1599, main.cpp poseQueryTimer)。即每 3 帧里约 2 帧
//     位姿没变 (vel 恰为 0), 第 3 帧却把 100ms 的位移除以 8ms ⇒ 速度高估 ~12 倍,
//     加速度经 1/dt² 放大更多。这会把 isStill() 往"永远为假"推。
// 所以这里启动后打 30 行实测量, 【一个力帧一行】(见下), 好让上面这个 10Hz 位姿 / 125Hz dt
// 的错配直接出现在输出里 —— 连续几帧 pos 一模一样、vel 恰为 0.000000, 然后一帧大跳 ——
// 而不是靠读代码去推断。原先每 1s 打一行, 30 帧里只采到 1 帧, 那个节奏根本看不出阶梯。
// 2026-09-19 追加: 同时并排打印 30004 帧里的 ToolVectorActual @624 (tcp=) 与 GetPose() 的
// pos=, 用来判定前者的坐标系 —— 名字含 "Tool", 但重力模型假定的是基座系 (见 AppState.h
// tcpPoseActual 的说明)。两边数值一对上, 就说明帧里的位姿可以直接当基座系位姿用 (那就能
// 甩掉 100ms 一次的仪表盘查询)。列 TCPSpeedActual @672 (tcpV=) 是顺手带上, 供同一帧核对。
// 【这是临时诊断: 等 dt/采样节奏与阈值这两件事有了结论并修好, 本段连同
//   ForceCompensation::motionState 一起删除。】
static bool g_motionProbeDone = false;
static DWORD g_motionProbeStartMs = 0;
static DWORD g_motionProbeLastUpdateMs = 0;   // 去重用, 与 BiasCheck::sample 同一套办法
static int g_motionProbeCount = 0;

static void runMotionProbe() {
    if (g_motionProbeDone) return;
    // --no-robot 下没有力数据/位姿流, 没有可测的东西: 直接定稿, 免得每帧空转。
    if (g_noRobot) { g_motionProbeDone = true; return; }

    DWORD now = GetTickCount();
    if (g_motionProbeStartMs == 0) { g_motionProbeStartMs = now; return; }

    AppState::ForceData fd;
    EnterCriticalSection(&appState.forceDataMutex);
    fd = appState.forceData;
    LeaveCriticalSection(&appState.forceDataMutex);

    // 与零偏漂移检查同理: 启动后前 2s 等读数稳住再开始, 免得前几行量的是启动瞬态。
    if (fd.isStale || now - g_motionProbeStartMs < 2000) return;

    // 【一个力帧一行】: 这里必须靠 lastUpdateMs 去重, 不能每个 idle 帧都打。idle() 跑得比
    // 力数据快得多, 而 pollForce() 自我节流到 33ms —— 去重之后本探针的节奏才等于 step()
    // 真正看到的那个节奏 (~30Hz), 也才有可能看出位姿的 10Hz 阶梯; 同一份数据反复打印只会
    // 把阶梯淹掉。(办法与 BiasCheck::sample 一致, 见那里的同名注释。)
    if (fd.lastUpdateMs == g_motionProbeLastUpdateMs) return;
    g_motionProbeLastUpdateMs = fd.lastUpdateMs;

    // 原始位姿 (mm), 与 pollForce 同法在锁内读取 (RelayCore.cpp:1605-1612)。
    double px, py, pz;
    EnterCriticalSection(&appState.robotPoseMutex);
    px = appState.robotActualPose.x;
    py = appState.robotActualPose.y;
    pz = appState.robotActualPose.z;
    LeaveCriticalSection(&appState.robotPoseMutex);

    double vel[3], acc[3];
    int still = ForceCompensation::motionState(vel, acc) ? 1 : 0;
    // tcp= / tcpV= 取自上面那份 ForceData 快照 (同一个 forceDataMutex 临界区), 不再二次加锁。
    printf("[Force] 运动检测 #%02d: pos=(%.3f,%.3f,%.3f)mm  tcp=(%.3f,%.3f,%.3f)mm  vel=%.6f m/s (阈值 %.4f)  acc=%.6f m/s² (阈值 %.4f)  tcpV=(%.4f,%.4f,%.4f)  isStill=%d\n",
           g_motionProbeCount + 1,
           px, py, pz,
           fd.tcpPoseActual[0], fd.tcpPoseActual[1], fd.tcpPoseActual[2],
           sqrt(vel[0]*vel[0] + vel[1]*vel[1] + vel[2]*vel[2]),
           Config::FORCE_MOTION_VEL_THRESH_MS,
           sqrt(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]),
           Config::FORCE_MOTION_ACC_THRESH_MSS,
           fd.tcpSpeedActual[0], fd.tcpSpeedActual[1], fd.tcpSpeedActual[2],
           still);

    // 30 行后永久停: 够看出阶梯节奏, 又不至于一直刷屏。
    if (++g_motionProbeCount >= 30) g_motionProbeDone = true;
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

            // 启动零偏漂移检查 (一次性, 只查零偏, 不阻断)
            runZeroDriftCheck(g_hasStoredZeroCalib);

            // 启动运动检测器诊断 (一次性 30 行, 一个力帧一行 — 见定义处注释)
            runMotionProbe();

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
                      << "       's' 求解负载参数 → 写入本地补偿 (合理性由拟合残差判定)"
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

    // 's' in BiasCheck mode: 求解残余量 → 写本地补偿 (force_calib.json)
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
        if (PayloadCalibration::load(CalibStore::fileFor("payload_calib.json"))) {
            // ψ 必须在这里装上 —— 本地补偿 (ForceCompensation::step) 与负载求解共用
            // TcpCalibration::gravitySensorFrame 这一个重力模型, 装错角度等于把重力矢量
            // 整个转歪。放在负载加载之后、任何一帧力处理之前。
            TcpCalibration::setSensorYawDeg(PayloadCalibration::sensorYawDeg);
            std::cout << "[Payload] Loaded payload_calib.json (mass=" << PayloadCalibration::massKg
                      << "kg, com=(" << PayloadCalibration::comMm[0] << ","
                      << PayloadCalibration::comMm[1] << "," << PayloadCalibration::comMm[2]
                      << ")mm, " << PayloadCalibration::poses << " poses, psi="
                      << TcpCalibration::sensorYawDeg() << "deg, sign_z="
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
        if (ForceCalibration::loadFromFile(CalibStore::fileFor("force_calib.json"),
                                           massKg, biasF, biasM)) {
            g_hasStoredZeroCalib = true;   // 有存储零偏, 启动漂移检查才有得比
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
    if (TcpCalibration::load(CalibStore::fileFor("tcp_calib.json"))) {
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
