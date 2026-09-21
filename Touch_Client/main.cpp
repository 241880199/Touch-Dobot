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
#include "core/SessionReport.h"
#include "haptic/HapticDevice.h"
#include "relay/RelayCore.h"
#include "render/SceneRenderer.h"
#include "safety/RobotDiagnostics.h"
#include "calibration/CalibrationSolver.h"
#include "calibration/TcpCalibration.h"
#include "force/ForceCalibration.h"
#include "force/ForceCompensation.h"
#include "force/ZeroDriftCheck.h"
#include "force/PayloadCalibration.h"
#include "force/RepeatPairRegistry.h"
#include "robot/Kinematics.h"
#include <cstdio>
#include <cstdarg>
#include <ctime>
#include <cstring>
#include <string>
#include <vector>

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

    // ===== 重复姿态 (模型形式检验的尺子) =====
    // 【协议: 原地复采】(2026-09-19 修订)
    //   摆姿态 → SPACE 采样 → ★【保持不动】→ 按 'r' → 再按 SPACE 采一次 (同一姿态)。
    //   'r' 配的是【上一次采样】, 一对 = (上一笔的行号, 紧接着的这一笔); 两次采样之间机械臂
    //   【不许移动】。同一姿态的这两次访问之差, 就是"同一个位姿上再来一次, 读数能差多少" ——
    //   模型形式检验拿它当尺子 (PayloadCalibration::RepeatPair)。
    // 【为什么不是"回到第 1 个姿态再采一次"】(旧协议, 已废)
    //   姿态是【手拖】出来的, 拖不出两次一样的位姿 —— 旧协议在实机上根本执行不了。而更要命
    //   的是它【坏在危险的那一侧】: 一对的两次访问落在不同位姿上时, 差值里混进两个位姿之间的
    //   重力差 (零点几 N, 对着 0.0224 N 的残差), σ_rep 被抬到 ~0.5 N, χ²/dof 落到 ~0.002,
    //   门限随之放宽到【无条件放行】—— 一把永远通过的尺子, 正是本模块要消灭的"绿色但不携带
    //   信息"。原地复采与"拖不回去"这件事无关 —— 但【保持不动】不等于位姿【完全】不变:
    //   实机日志里保持段仍有 ≤0.2° 的残余抖动 (重力注入 ≤0.65× 残差), 只是小到不影响判决。
    //   ⊙ "原地复采为什么成立"的论证与实测数据【只写一份】, 在 spec §8b
    //     (Docs/superpowers/specs/2026-09-19-raw-channel-calibration-design.md); 这里不再复述。
    // 【按 'r' 是【追加】, 不是覆盖】: 尺子自己也有自由度 —— 一对只给 1 个, 而一双观测的
    // 离散很大 (两次凑巧对得很齐 vs 凑巧差很多都是常事)。多按几次 'r' 多采几对, 尺子才稳,
    // 判决的门限才收得紧 (见 PayloadCalibration::fitRaw 里的 modelFormLimit)。
    // 从前这里只存【一个】下标, 再按 'r' 会把它覆盖掉 —— 于是"判决贴着线时补一对"这个补救
    // 承诺是空的 (补进来的还是同一个 1 自由度估量)。
    // 【为什么是显式的 'r' 标记, 而不是"把最后一个姿态当重复姿态"】
    //   · 与顺序无关: 标记之后再补采几个姿态、或重采某个坏姿态, 尺子都不受影响; 而 first/last
    //     方案下, 收尾之后任何一次 SPACE 都会把尺子【悄悄换成一个不相干的姿态】, 没有任何提示;
    //   · 忘按 'r' 是【可检出】的 (repeatCount == 0) —— 求解方会明确拒绝, 不会拿一把假尺子量;
    //   · 不靠距离容差判"是不是同一个姿态": 那既是预设, 又正好是这里要量的事情。
    static const int MAX_REPEATS = 8;  // 尺子的对数上限 (再多也停在 8: 够用, 免得占满姿态位)
    // 一对 = (repeatFirst[i], repeatIdx[i]) —— 两个下标都是【采集侧登记的】, 不再有"恒为 pose 1"
    // 这种常量: first 就是按 'r' 时手上那一笔, second 是紧接着采的这一笔 (见 recordRepeat)。
    // 登记规则本身抽在 RepeatPairRegistry 里 (纯函数, 有单测), 这里只存结果与计数。
    static int  repeatFirst[MAX_REPEATS];  // 每对里【第 1 次访问】落在哪一行 ('r' 之前那一笔)
    static int  repeatIdx[MAX_REPEATS];    // 每对里【第 2 次访问】落在哪一行 ('r' 之后那一笔)
    static int  repeatCount = 0;       // 已登记几对
    static bool pendingRepeat = false; // 本次采样结束后登记为重复访问

    // 【假拒率 (模型正确时被误拒) 的唯一一份抄本】—— ['m' 进模式那条操作文案] 与
    // [只有 1 对时的提示] 都引它, 免得两处各写一份、各自漂移。
    //
    // ⚠ 2026-09-19 拆口径: 这里从前只有【力通道】一张表, 而重复对这把尺子是【两条门共用】的 ——
    //   操作员按 'r' 时看到的是力通道的 0.05%, 而同一把尺子喂给的【力矩分支】实测差着两个数量级
    //   (R=3 时 7~23%)。把一条通道的实测数字贴到另一条通道上, 正是本项目栽得最多的那一类:
    //   数是真的, 归属是错的。现在两条分开写, 各自注明口径。
    //
    // 力通道 —— 唯一来源: PayloadCalibration.cpp 里 RAW_MODEL_FORM_ALPHA 说明中的蒙特卡洛表
    //   (α=0.9999、r=3 那一档; 出厂外模拟, 每点 4 万次)。
    //   (旧文案里的 12%/4.9%/2.5% 是放宽前 α=0.997 那一列 —— 当时只把 R=1 的括注改成了新值,
    //    R=2/R=3 一直虚高约 18x/50x, 把"再多采几对"的好处说得比实测大。)
    // 力矩分支 —— 来源: 生产 fitRaw 直接驱动的零假设蒙特卡洛, 每格 2e5 次、两套噪声模型
    //   (Docs/superpowers/evidence/moment-gate-dA-correction-report.md 的计数,
    //    moment-gate-calibration-report.md §2/§3 的口径)。
    //   ⚠ 夹具把力矩量化到 0.001 N·m, 所以下面是【下界】, 真值只会更差。
    //   为什么差这么多 (2026-09-19 查明): δA —— 力矩模型复用的是【力通道估出来的 A】, 而它自己的
    //   叉乘结构吃不掉 A 的估计误差 (占膨胀的 88%, 超额 ∝|δA|²)。修它要重做统计量, 尚未做。
    //   ⇒ 实操结论: 【R≥5】把力矩分支压到 1% 量级; R=3 会到 7~23%, 那是最不该停下来的地方。
    //     (R<3 未实测, 只会更差。)
    static const char* const FALSE_REJECT_RATES =
        "模型正确时的假拒率 (α=0.9999) —— 力通道: R=1 约 2.1%、R=2 0.27%、R=3 0.05%;"
        "  力矩分支: R=3 约 7~23%、R=5 约 1~5% (随采集而变, 且是下界)";

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

    // 连续多少次求解被拒。【只计数并照实报出来, 不再当闸门】: 从前到了
    // Config::CALIB_MAX_CONSECUTIVE_FAILS 就把 's' 锁死 (整屏诊断被一行顶掉),
    // 而这条路现在只打印、不应用 —— 藏掉输出没有任何东西被保护到。见 solveAndApply 顶上。
    static int  consecutiveFails = 0;
    static bool solveLocked = false;   // 已连续被拒到这个次数 (状态标记; 不决定跑不跑)

    // ===== Task 8a: 发送候选 (最近一次【求解成功】留下的那一份) =====
    // 为什么要有它 (而不是按 'p' 时重算): 打印出来的候选与真正发出去的那一份【必须是同一个
    // 东西】。重算就会隔着两次按键、两次实时读数, 中间还可能又按过 's' —— 屏幕上说 "候选
    // 0.42 kg / (0.3, -0.1, 68.7)" 而发出去的是别的, 正是本项目最忌讳的那种"安静地不一致"。
    // 求解失败时它会被作废 (见 solveAndApply 里落候选那一段)。
    static bool s_sendCandidateValid = false;
    static PayloadCalibration::SendGate s_sendCandidate;

    // ===== Task 8a: 发送前的二次确认 (全文复审 I3, 用户 2026-09-20 批准) =====
    // 'p' 从前是"打印安全规程 + 同一次按键里就发出去", 而这是一个会让实体机械臂动的动作 ——
    // 一次误触没有任何东西挡得住。改成两段: 'p' 只【摆出来】(安全规程 + 这一次到底发什么),
    // 再由一个明确的确认键真正发。拒发 (没有候选 / 过不了闸) 仍旧发生在 'p' 那一刻, 在那之前
    // 不会出现确认提示 —— 否则"确认"会落到一件本来就不会发生的事上。
    //
    // ⚠ 确认发的是 s_confirmCandidate (按下 'p' 那一刻【照抄】下来的那一份), 【不是】重新读
    //   s_sendCandidate: "屏幕上摆出来的"与"发出去的"必须是同一个东西 (与候选本身不许重算是
    //   同一条规矩)。照抄之后即使别处的候选被改写作废, 发出去的仍是屏幕上那一份。
    static bool s_awaitingSendConfirm = false;
    static PayloadCalibration::SendGate s_confirmCandidate;

    static void reset() {
        dataUnderCurrentPayload = true;
        consecutiveFails = 0;
        solveLocked = false;
        count = 0;
        sampling = false;
        avgCount = 0;
        repeatCount = 0;
        for (int i = 0; i < MAX_REPEATS; i++) { repeatFirst[i] = -1; repeatIdx[i] = -1; }
        pendingRepeat = false;
        // 采集重开 = 这批数据丢弃, 所以上一次求解留下的发送候选一并作废 —— 它正是从这批
        // 【已经被丢弃的】数据解出来的。留着它, 'p' 会在"我刚重开采集"之后发出一份旧值。
        s_sendCandidateValid = false;
        // 挂着的发送确认一并撤销 (与候选作废同源: 它确认的是上一批数据解出来的那一份)。
        // ⚠ 【必须出声】, 与 cancelSendConfirm 是同一条规矩: 这是一份"正要向机械臂下发"的
        //   状态, 静默抹掉它, 操作员按确认键会【什么都不发生】—— 那正是本项目最忌讳的
        //   "安静地不一致"。今天走不到这里 (确认提示挂着时, 键盘处理在所有其他键之前就把它
        //   取消掉了, 根本进不到会调 reset() 的那些键), 但可达性是会变的。
        if (s_awaitingSendConfirm) {
            s_awaitingSendConfirm = false;
            std::cout << "[下发] ✗ 发送确认已随采集重开一起取消 —— 本次【什么都没有发出去】。"
                      << std::endl;
        }
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
        // 被其他采集模式抢占 (本函数只从这里被调) -> 挂着的发送确认也撤销: 它是这一次采集的
        // 产物, 而这次采集整个被丢弃了; 留着一个跨模式的待确认状态是操作员想不到的。
        if (s_awaitingSendConfirm) {
            s_awaitingSendConfirm = false;
            std::cout << "[下发] ✗ 发送确认已随 'm' 模式被抢占一起取消 —— 本次【什么都没有发出去】。"
                      << std::endl;
        }
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

    // 'r': 把【紧接着的下一次采样】登记为【上一次采样那个姿态】的重复访问 (协议见上面的说明)。
    // 与 SPACE 走【同一条采样路径】(record), 只是给这一次的结果挂上"与上一笔配成一对"的旗标。
    // ★ 按 'r' 时机械臂必须还在上一次那个位姿上, 并且到采完为止【不许移动】—— 中间一动, 这一
    // 对量到的就是两个姿态之间的重力差, 不是复现性。程序侧不替操作员判"这是不是同一个姿态"
    // (那既是预设, 又正好是这里要量的事情), 但结算时会把 Δ 与 σ_rep 当场打出来 (见 sample)。
    static void recordRepeat() {
        if (!mode) return;
        if (sampling) {
            std::cout << "[BIAS] 正在采样中, 保持静止" << std::endl;
            return;
        }
        if (count < 1) {
            std::cout << "[BIAS] 还没有可复采的上一笔 —— 'r' 配的是【上一次采样】。"
                      << "先摆好姿态按 SPACE 采一次, 【保持不动】再按 'r' + SPACE。" << std::endl;
            return;
        }
        if (repeatCount >= MAX_REPEATS) {
            std::cout << "[BIAS] 重复对已够 " << MAX_REPEATS << " 对, 再多也不会更准 ——"
                      << " 直接按 's' 求解" << std::endl;
            return;
        }
        // 先在 record() 之前把"手里这一笔"的编号记下来: record() 里的旧负载复位 (reset()) 会把
        // count 清零, 之后 count 已经不是"上一次采样"了 —— 那时直接读它会打出 "Pose 0"。
        // 这条复位路径今天走不到 (它唯一的写者 dataUnderCurrentPayload = false 在 #if 0 里),
        // 但报错了还不如不报: 清零之后根本没有"上一笔"可配, 这时该说的是这件事。
        const int prevPose = count;   // 上一次采样的显示编号 (与 "Pose N" / "^ 重复访问" 同一编号)
        record();
        if (sampling) {
            pendingRepeat = true;
            // count == 0 只可能是 record() 里的旧负载复位 (进来时已经保证 count >= 1)。
            if (count == 0) {
                std::cout << "[BIAS] 上一批数据是在旧负载下采的, 已作废 —— 这一次是【新一批】"
                          << "的第 1 笔, 没有可复采的上一笔 (这一对不会被登记)" << std::endl;
            } else {
                std::cout << "[BIAS] 这一次将与上一次 (Pose " << prevPose
                          << ") 配成一对 —— ★ 中间【不要移动机械臂】" << std::endl;
            }
        }
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
                pendingRepeat = false;                // 作废的这一笔不能留下"已登记"的假象
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
            pendingRepeat = false;                    // 同上: 作废即不登记
            std::cout << "[BIAS] 有效样本太少 (" << avgCount << "), 本次作废" << std::endl;
            return;
        }
        if (count >= MAX_POSES) { pendingRepeat = false; return; }
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
        // 这一次是不是"上一次采样的重复访问": 由操作员按 'r' 时挂上的旗标决定,
        // 结算在【采样真的成功之后】—— 作废的那几笔在上面已经清掉了旗标。
        const bool isRepeat = pendingRepeat;
        pendingRepeat = false;
        // 一对 = (上一笔的行号, 这一笔的行号)。协议是【原地复采】(见 repeatFirst 的说明),
        // 所以"上一笔"就是 count-1 —— 不再有"恒为 pose 1"的常量。判断本身抽在
        // RepeatPairRegistry 里 (纯函数, 有单测); 这里只负责把结果存下 + 照实说没登记的原因。
        int pairFirst = -1;
        if (isRepeat) {
            int repFirst = -1, repSecond = -1;
            const int repSt = RepeatPairRegistry::registerPair(count - 1, count, repeatCount,
                                                              MAX_REPEATS,
                                                              &repFirst, &repSecond);
            if (repSt == RepeatPairRegistry::OK) {
                repeatFirst[repeatCount] = repFirst;
                repeatIdx[repeatCount]   = repSecond;
                repeatCount++;
                pairFirst = repFirst;
            } else {
                // 【不登记】比登一对假的强得多。走到这里只有一种实况: 手上没有"上一笔"可配
                // (例如按 'r' 之后 record() 发现整批数据属于旧负载、把 count 归了零) ——
                // 上限那条在 recordRepeat 里已经先挡过。
                std::cout << "[BIAS] 这一对【没有登记】("
                          << (repSt == RepeatPairRegistry::NO_PREVIOUS
                                  ? "找不到可复采的上一笔 —— 'r' 配的是【上一次采样】,"
                                    "刚开的一批里它前面没有样本"
                                  : "已达对数上限")
                          << ")" << std::endl;
            }
        }

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
        // 登记为重复访问时把【与配对那一笔的位姿差】照实打出来 (报告, 不是判据 —— 判"是不是
        // 同一个姿态"要靠操作员的规矩, 不靠一个距离容差): 差得多说明中间动了, 这一对量出来的
        // 就不是复现性而是两个姿态之间的重力差。
        if (pairFirst >= 0) {
            printf("       ^ 重复访问 (与 Pose %d 配成一对): ΔR=(%+.1f,%+.1f,%+.1f)deg"
                   "  Δxyz=(%+.1f,%+.1f,%+.1f)mm\n",
                   pairFirst + 1,
                   pose[count][0] - pose[pairFirst][0], pose[count][1] - pose[pairFirst][1],
                   pose[count][2] - pose[pairFirst][2],
                   pose[count][3] - pose[pairFirst][3], pose[count][4] - pose[pairFirst][4],
                   pose[count][5] - pose[pairFirst][5]);
            // ★ 这一对的【尺子读数】当场打出来 (报告, 不判)。
            // 为什么现在就要打: 操作员最容易犯的错是【按 'r' 之前先动了机械臂】—— 那时 d 混的
            // 是两个不同姿态的重力差 (零点几 N 的量级), σ_rep 被抬到 0.5 N 上下, 门限随之
            // 放宽到几乎不判, 而这一对【什么都不像】却在同一时刻被登记成了尺子。程序侧不去
            // 替操作员判"这是不是同一个姿态" (那既是预设, 又正好是这里要量的事情), 但把这两个
            // 数摆在他眼前是免费的 —— 一个 0.5 N 的尺子在正常读数 (0.01~0.05 N) 旁边一眼就认得出来。
            // 用 @1304 原始读数算, 与求解侧同一口径 (求解侧喂的就是这份【未镜像】的原始值 ——
            // 镜像那一步已经不在新模型里了, 见 solveAndApply 上面的说明)。
            double v0 = 0.0;
            for (int a = 0; a < 3; a++) {
                const double d = biasSix[count][a] - biasSix[pairFirst][a];
                const double s0 = (samples[pairFirst] > 0)
                                      ? varSix[pairFirst][a] / samples[pairFirst] : 0.0;
                const double s1 = (samples[count] > 0) ? varSix[count][a] / samples[count] : 0.0;
                double ex = 0.5 * (d * d - s0 - s1);
                if (!(ex > 0.0)) ex = 0.0;
                v0 += ex + 0.5 * (s0 + s1);
                printf("         尺子读数 力%c: d=%+.4f N  σ_rep=%.4f N\n",
                       "xyz"[a], d, sqrt(ex + 0.5 * (s0 + s1)));
            }
            // 判"这一对干不干净"靠的是【比两个已经打出来的数】, 不是一个绝对门限:
            // σ_rep 与 's' 屏幕上的 rmsF (fit.rmsForceN) 同量级才对。要一个"零点几 N 才说明动过"
            // 的隐含门限是不行的 —— 0.2° 的姿态抖动只花掉 0.65× 残差, 那种门限会放它过去。
            printf("         (三个通道的 σ_rep 平方均值再开方 = %.4f N —— 求解侧就是拿它"
                   " 当尺子的。判它干净与否: 拿这个数去比 's' 屏幕上打印的 rmsF, 同量级才对;"
                   " 明显大于 rmsF 才说明这一对两次采样之间机械臂动过)\n",
                   sqrt(v0 / 3.0));
        }
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
        // 尺子的来处照实报 —— 它是模型形式检验能不能做的【前提】, 不是可选步骤。
        if (repeatCount > 0) {
            std::cout << "  重复姿态对 (" << repeatCount << " 对 = 尺子的自由度):";
            for (int i = 0; i < repeatCount; i++)
                std::cout << " (pose " << repeatFirst[i] + 1 << ", pose " << repeatIdx[i] + 1 << ")";
            std::cout << " —— 原地复采, 同一姿态的两次访问: 姿态间复现性的尺子" << std::endl;
        } else {
            std::cout << "  重复姿态对: 【没有】—— 缺了它, 模型形式无从判定 (求解会拒给参数)。"
                      << "摆好姿态按 SPACE 采一次, 【保持不动】按 'r' 再按 SPACE 采一次"
                      << std::endl;
        }
        std::cout << "  当前负载: load=" << mCfg << " kg  center=("
                  << cCfgNow[0] << ", " << cCfgNow[1] << ", " << cCfgNow[2] << ") mm"
                  << (PayloadCalibration::enabled ? "  [实机标定值]" : "  [种子值, 未标定]")
                  << "  CZ符号=" << (PayloadCalibration::comSignZ > 0 ? "+1" : "-1")
                  << " (信息性: 数据定不了符号, 不影响补偿)" << std::endl;
        // 说清这一行是哪来的: 它是【旧模型 + 当前生效配置】的量, 与 's' 那一屏 (原始通道的
        // 线性解、传感器测量原点以下) 不是一回事 —— 别拿这里的 load/center 去读那一屏的结果。
        std::cout << "  ↑ 这是【当前生效配置/旧模型】的量, 不是 's' 解出的东西 (两回事)。"
                  << std::endl;
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
            std::cout << "   → 按 's' 用这批数据【求解原始通道】, 只打印、不改动任何东西"
                      << " (不写补偿 / 不写 json / 不下发机械臂)" << std::endl;
        }
        std::cout << std::endl;
    }

    // ModelFormStatus 的名字 —— 打印与落盘都要"哪一把尺子缺了就报哪一个"。
    // 【不合并成"没验过"一句话】: 缺噪声 / 缺重复姿态对 / 通道冻住 / 逐姿态有洞 / 自由度不足
    // 是【五件不同的事】, 处置也各不相同 (补采 vs 原地复采按 'r' vs 修通道 vs 多摆姿态),
    // 糊成一句就等于把可行动的信息扔掉。
    static const char* modelFormStatusName(int s) {
        switch (s) {
            case PayloadCalibration::MODEL_FORM_OK:           return "MODEL_FORM_OK";
            case PayloadCalibration::MODEL_FORM_NO_NOISE:     return "MODEL_FORM_NO_NOISE";
            case PayloadCalibration::MODEL_FORM_NO_REPEAT:    return "MODEL_FORM_NO_REPEAT";
            case PayloadCalibration::MODEL_FORM_DEAD_CHANNEL: return "MODEL_FORM_DEAD_CHANNEL";
            case PayloadCalibration::MODEL_FORM_NOISE_HOLES:  return "MODEL_FORM_NOISE_HOLES";
            case PayloadCalibration::MODEL_FORM_NO_DOF:       return "MODEL_FORM_NO_DOF";
            default:                                          return "MODEL_FORM_UNKNOWN";
        }
    }

    // 每次负载求解尝试都落一行 —— 控制台会滚掉, 而"这次标定到底做了什么"必须能追溯。
    // 一行一次尝试, 便于 grep 与表格工具直接读。
    //
    // 表头只在【文件不存在或为空】时写。不要用进程内 static 标志位: 那个标志每次启动都是
    // false, 会截断历次记录 —— 与"可追溯"的立意在字面上相反。打开方式也必须是 "a+",
    // 见下面的说明。
    //
    // ===== 行格式 (2026-09-19 随原始通道求解器一起扩宽) =====
    // 【前 9 列的列号与含义一字未动】—— 历史脚本按列号读, 挪一列就是静默读错:
    //   time | poses | rmsF_N | rmsM_Nm | psi_deg | dm_kg | mass_kg | comZ_mm | outcome
    //   · psi_deg / dm_kg / mass_kg / comZ_mm 是【旧模型】(psi 扫描 + 残余量 dm/dp) 才有的量。
    //     新模型 (原始通道的线性解) 里【根本不存在】这四个量 -> 一律记 "-"。
    //     "-" 不是"没算出来", 是"这个模型里没有这个量"; 别把它读成 0, 更别拿它去填别的数。
    //   · rmsF_N / rmsM_Nm 是【本次所用模型】的拟合残差 (两个模型都有这个量)。
    //   · outcome 的【前缀】如实说这一行代表什么被应用了 (列本身一个没动):
    //       PRINTED_ONLY            求解未通过 ⇒ 确实什么都没装
    //       INSTALLED               已装进本地补偿, 且 force_calib.json 已落盘
    //       INSTALLED save_failed   装上了, 但 force_calib.json 没写成 (重启即丢)
    //       INSTALL_REJECTED        判决通过但 setCalibration 拒收 (理论上不该出现)
    //     (从前恒是 PRINTED_ONLY —— 那时确实一个字节都没写, 所以那是真话;
    //      2026-09-20 起 's' 通过判决后会装本地补偿, 再无条件写 PRINTED_ONLY 就是假话了。)
    // 【新列一律【追加在末尾】】—— 上面那 9 列一个都没挪, 历史行与历史读者照旧。
    // 追加列 (13): solver | m_kg | sv1 | sv2 | sv3 | iso | parity
    //              | cs_x_mm | cs_y_mm | cs_z_mm | cond | maxSigA | modelform
    //   solver    = raw (原始通道线性解) / legacy (旧模型) / - (没走到求解)
    //   modelform = ModelFormStatus 的名字 (哪一把尺子缺了就报哪一个), "-" = 没走到
    //   maxSigA   = A 的 9 个分量里最大的 1σ (kg)。18 个 sigma 全写会把行撑得没法读, 而 A
    //               是物理上最要紧的那一组; 要全量请用 calib_poses.txt 离线重放。
    //
    // 追加列说明行的固定开头 —— 写出与扫描【只此一处定义】, 见 logCalibAttempt 里的用法。
    static const char* const COL_EXT_MARK = "# columns extended ";

    // fit 可为 nullptr: 求解【之前】就返回的出口 (姿态数不足 / 正在采样) 没有结果,
    // 此时【数值列】记 "-"。poses 由调用方单独传: 它是"这次手上有几个姿态", 与求解成没成
    // 无关, 所以【照记】, 不因为 fit 是 nullptr 就退化成 "-"。
    static void logCalibAttempt(const char* outcome, const PayloadCalibration::RawFit* fit,
                                int poses)
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

        // 【老文件没有列名的补救】: 表头只在文件为空时才写, 而机器上现存的 calib_log.txt
        // 是 2026-09-19 之前的 9 列表头 —— 那道口子会让【13 个新列永远没有列名】, 恰好在
        // 需要解释它们的那一刻没有说明。所以: 扫描整个文件找说明行的标记, 没有就补一句,
        // 【只补一次, 且一个已有行都不动】(追加在第一次写新格式行之前)。
        // 扫描而非进程内 static 标志: 标志每次启动都是 false, 会在每次启动各补一句;
        // 而"这个文件到底被解释过没有"是文件的属性, 不是这次进程的属性。
        bool needColNote = !needHeader;
        if (needColNote && fseek(f, 0, SEEK_SET) == 0) {
            char line[512];
            while (fgets(line, sizeof(line), f)) {
                if (strncmp(line, COL_EXT_MARK, strlen(COL_EXT_MARK)) == 0) {
                    needColNote = false;
                    break;
                }
            }
        }
        // 【读完必须重新定位再写】: C99 7.19.5.3 不允许 update stream 上"读紧接写"而中间
        // 不隔着 fflush 或定位调用 —— 上面这段扫描 (提前 break 时停在文件中间) 正是本轮
        // 改动新引入的读, 而紧接着就是 fprintf。实测无碍 ("a+" 在两种 CRT 上都把写落在
        // EOF), 但那个前提没必要留着; fseek 归位是免费的。
        fseek(f, 0, SEEK_END);
        if (needHeader) {
            fprintf(f, "# 负载标定尝试记录 (每次按 's' 一行)\n");
            // 前 9 列 = 历史格式 (列号未动); 第 10 列起是追加的原始通道列 (下一行的说明)。
            fprintf(f, "# time | poses | rmsF_N | rmsM_Nm | psi_deg | dm_kg | mass_kg"
                       " | comZ_mm | outcome\n");
        }
        if (needHeader || needColNote) {
            // 同一句话, 两种场合: 新文件的表头里就带; 老文件在第一次写新格式行之前补上。
            // 两种文件里都是【同一行标记】—— 所以上面扫得到, 也就只会补一次。
            // 每列的含义见本函数上面的长注释 —— 尤其: "-" 是"这个模型里没有这个量", 不是 0。
            fprintf(f, "%s2026-09-19: 第 10 列起为原始通道解新增 ——"
                       " solver | m_kg | sv1 | sv2 | sv3 | iso | parity"
                       " | cs_x_mm | cs_y_mm | cs_z_mm | cond | maxSigA | modelform"
                       " (\"-\" = 该模型里没有这个量, 不是 0)\n", COL_EXT_MARK);
        }
        char ts[24];
        const std::time_t now = std::time(nullptr);
        std::tm tmInfo;
        localtime_s(&tmInfo, &now);
        strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", &tmInfo);

        // 逐列直接写, 【不先攒进一个定长缓冲】: 攒缓冲就得给每个数定一个宽度, 而任何一个数
        // 变长 (比如 c_s 解飞了) 都会把【后面】的列静默挤掉 —— 而落盘的意义正是"控制台滚掉
        // 之后还查得到", 悄悄少一列比不写更坏。fprintf 自己按内容增长。
        // 第 5..8 列 (psi_deg / dm_kg / mass_kg / comZ_mm) 【恒为 "-"】: 新模型里没有这四个量。
        if (!fit) {
            // poses 照记 —— 它说的是"这次手上有几个姿态", 与求解走没走到无关。
            fprintf(f, "%s | %d | - | - | - | - | - | - | %s", ts, poses, outcome);
            fprintf(f, " | - | - | - | - | - | - | - | - | - | - | - | - | -\n");
            fclose(f);
            return;
        }
        // 奇异值 / m / parity / 各向同性比从 A 自己来 (与 decompose 同一份口径, 不另算一份)。
        // A 奇异时它们【无从给出】—— 记 "-", 而不是记 0 (0 会被读成"各向同性比 = 0",
        // 那是另一个意思, 更糟)。
        PayloadCalibration::Decomp d;
        const bool haveD = PayloadCalibration::decompose(fit->A, d);
        double maxSigA = 0.0;
        for (int k = 0; k < 9; k++)
            if (fit->paramSigma[k] > maxSigA) maxSigA = fit->paramSigma[k];
        fprintf(f, "%s | %d | %.4f | %.4f | - | - | - | - | %s",
                ts, poses, fit->rmsForceN, fit->rmsMomentNm, outcome);
        fprintf(f, " | raw");
        if (haveD) {
            fprintf(f, " | %.6f | %.5f | %.5f | %.5f | %.5f | %+.0f",
                    d.m, d.sv[0], d.sv[1], d.sv[2], d.isotropyRatio, d.parity);
        } else {
            fprintf(f, " | - | - | - | - | - | -");
        }
        // c_s 与 m / 奇异值 / parity / Q 同属【线性层解出来了才有】的量: 线性层没解出来时
        // c_s 也是全 0, 而 "0.000" 在这一行里会被读成"质心就在测量原点" —— 一个看着像
        // 测量结果、其实表示"没有这个量"的 0。与上面那 6 列同一条件 (haveD), 记 "-"。
        if (haveD) {
            fprintf(f, " | %.3f | %.3f | %.3f",
                    fit->cS[0] * 1000.0, fit->cS[1] * 1000.0, fit->cS[2] * 1000.0);
        } else {
            fprintf(f, " | - | - | -");
        }
        fprintf(f, " | %.4g | %.6g | %s\n",
                fit->cond, maxSigA, modelFormStatusName(fit->modelFormStatus));
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
        // 重复姿态对的登记也一起落盘: 它是模型形式检验的尺子的【出处】, 离线重放
        // (Task 2 那一类) 没有它就无从还原那次判决是怎么来的。写成注释行而不是新列 ——
        // 列的形状是历史脚本在读的, 别再动 (见上面那 7 列的说明)。
        // 【全部登记的对都写出来】(从前只写一对, 而按 'r' 会覆盖) —— 离线重放要还原的是
        // 【池化后的】那把尺子, 少写一对就还原不出来。
        if (repeatCount > 0) {
            // first/second 都是【逐对登记的】(协议: 原地复采, first = 'r' 之前那一笔) —— 不再
            // 是一个恒定的 first=1。两列都是 1 基的行号, 第 i 个 first 与第 i 个 second 是一对。
            fprintf(f, "# repeat: first=");
            for (int i = 0; i < repeatCount; i++)
                fprintf(f, "%s%d", (i ? "," : ""), repeatFirst[i] + 1);
            fprintf(f, " seconds=");
            for (int i = 0; i < repeatCount; i++)
                fprintf(f, "%s%d", (i ? "," : ""), repeatIdx[i] + 1);
            fprintf(f, "  (共 %d 对: 原地复采, 每一对两次采样之间机械臂不移动)\n", repeatCount);
        } else {
            fprintf(f, "# repeat: none  (没有重复姿态对 -> 模型形式检验没有尺子)\n");
        }
        fprintf(f, "# rx,ry,rz,x,y,z,F576x,F576y,F576z,M576x,M576y,M576z,"
                   "F1304x,F1304y,F1304z,M1304x,M1304y,M1304z,"
                   "N1304,sdF1304x,sdF1304y,sdF1304z,sdM1304x,sdM1304y,sdM1304z,"
                   "F720x,F720y,F720z,M720x,M720y,M720z\n");
        // 末尾 6 列是 2026-09-20 加的: @720 TCPForce。
        //
        // 【为什么加】厂商接口文档把两个量分得很清楚:
        //   ActualTCPForce @576 = "TCP【传感器】力值"
        //   TCPForce       @720 = "TCP力值 (【通过关节电流计算】)"
        //   要"通过关节电流"算出力, 就必须知道负载 (重力矩 + 惯量矩) ⇒ **@720 才反映控制器
        //   正在用的负载参数**。而【当时】一致性闸门比的却是 @576。
        // 而项目自己早有一条实测记录 (relay/RelayCore.cpp): "改 EnableRobot 的负载, @576
        //   纹丝不动, 为什么还不清楚" ⇒ **闸门的设计前提("发送正确负载之后 @576 会与我们一致")
        //   被项目自己的这条测量否证掉了**, 而那件事一直挂在"还不清楚"里。
        // ⇒ 把 @720 落盘, 是为了【核对闸门的参考量该是谁】。核对已有结论 (run-004 §4.5):
        //   参考量换到了这一路 —— 判据的唯一一份定义是 ForceCompensation.cpp 的
        //   guardReferenceValue, 以那里为准。
        //
        // ⚠ 它【一直在采】(accumTcp / biasTcp, 每姿态还在控制台打一行 "@720 F=(...)"),
        //   只是从来没写进文件 —— 所以这不是新增采集, 是补一个漏。
        // 精度取 %.6f: @576 那 6 列是 %.3f (它们只用于 0.1 N 量级的三路裁决); 而 @720 是
        //   闸门判据的参考量 (见 ForceCompensation.cpp 的 guardReferenceValue), 它的力矩列
        //   要与 tol_M = 0.03 N·m 比 —— %.3f 的台阶(1e-3)是那个容差的 3%, 按本文件顶上
        //   "每列的分辨率不得成为消费它的统计量的瓶颈"那条规矩, 取 %.6f。
        // 列一律【追加在末尾】: 上面 24 列一个没挪, 历史行与历史读者照旧 (与 2026-09-19 加那
        //   7 列同一条规矩)。
        // 末尾 7 列是 2026-09-19 加的 (模型形式检验要实测噪声, 而噪声是【逐姿态】采出来的):
        // N1304 = 该姿态的样本数, sd* = 该姿态内 @1304 各通道的样本标准差。
        // 均值那 18 列的形状【没有】动 —— 离线分析/历史脚本读它们仍然照旧。
        // 有了 (N, sd) 就能还原"均值的不确定度" sd/sqrt(N), 也就是 fitRaw 要的那把尺子。
        //
        // ===== 落盘精度 (2026-09-19 修订): 【每列的分辨率不得成为消费它的那个统计量的瓶颈】 =====
        // 立这条规矩的是一次已经发生的损失: 从本文件重放得到的失拟统计量与控制台当时打印的对不上
        // (夹具 6.97/18.36/33.74 vs 控制台 5.359/20.51/61.22)。差多少【不统一】—— 三次的比值分别是
        // ×1.30 / ×0.895 / ×0.551 —— 因为分子与分母都在动。不存在"统一差了两倍"这回事。
        // 下面每一条都是【量出来的】: 用生产 fitRaw 跑合成采集 (10 姿态 + 5 对, 与 15:33 同尺度),
        // 把同一批全精度数据分别按旧/新精度写成文本再读回来比对, 12 组独立噪声。
        // 出处: Docs/superpowers/evidence/precision-sufficiency-report.md
        //
        //   F1304*/M1304*: %.3f -> %.9f      【主因: 力矩失拟偏 −52.4%】
        //     力矩重复对的真实差值 d ≈ 2e-4 N·m, 而旧台阶 1e-3 比 d 本身还大五倍 —— d 于是被换成
        //     一格一格的粗值, 尺子的 σ_sys,M 变成量化驱动的: 实测 σ_rep,M² 被抬高 ×1.27~1.82,
        //     而 σ_rep,M 进的是【分母】, 统计量被压低。只把这一组按旧精度写就复现出 −52.4%。
        //
        //   sdF1304*/sdM1304*: %.4f -> %.9f  【附带, 不是主因 —— 别把它写成关键】
        //     只把这一组按旧精度写, 力矩失拟偏 0.49%。sdM ≈ 1.3e-3 而台阶 1e-4 是它的 8%, 确实
        //     进尺子, 但与上面那 52% 差两个数量级。改它是顺手。
        //
        //   rx,ry,rz: %.1f -> %.3f
        //     实测台阶: %.1f -> 力通道统计量偏 +8.05%;  %.2f -> 0.417%;  %.3f -> 0.032%
        //     (12 组噪声下最坏 0.184%, 相对 1% 的判据留 5.4 倍余量)。取 0.001° 够用, 再细没必要。
        //     ⚠ 【别把理由写成"0.05° 的台阶带来 ~2e-3 N 的系统性力误差"】—— 那是先前写错的说法,
        //     两条恒等式就否掉它:
        //       · rz 【根本不进】重力模型: g = Rᵀe₃ 是 R = Rz·Ry·Rx 的第三【行】, 而 Rz 的第三行
        //         恒为 [0,0,1] —— 绕 z 转多少都不改变 g。rz 的精度因此无关紧要 (写细也无害)。
        //       · rx 的【共同】偏移也是精确的: g(rx+δ) = Rx(δ)·g(rx), 而 A 是完全自由的 3×3,
        //         把这份共同偏移整个吸收掉 —— 实测把所有姿态的 rx 统一加 0.5°, 结果【分毫不动】。
        //     有害的只是【逐姿态各不相同】的那一份舍入误差 (它不在 A 能吸收的那个子空间里)。
        //     实测 %.1f 对 rmsForceN 的影响只有 −1.39%, 不是 14%。
        //     (顺带一条"该不该写细"的依据: 同一个 appState.robotActualPose.rx 被 ForceLogger
        //      以 %.3f 写进 force_demo_log.csv, 那里的值是 179.416 / 61.714 / 164.482 —— 说明内存里
        //      的 double 本来就有三位以上, 写 %.1f 是在丢信息。但这不是上面那个力度估计。)
        //
        //   x,y,z / @576 F,M / N1304: 不动
        //     x,y,z 不进重力模型 (实测: 给它加 0.5 mm 的系统性偏移、或整个砍到 0 位小数, 统计量都
        //     分毫不动 —— 本文件是【静态姿态逐个采样】, 位置列只用于人工核对"这几笔是不是在同一个
        //     位置附近转姿态"); @576 那 6 列【根本不是 fitRaw 的入参】, 只用于 0.1 N 量级的三路裁决;
        //     N1304 是整数, 没有舍入可言。别因为"看起来不一致"就顺手把它们也改了 —— 那只会让每次
        //     采集多写一堆用不上的字符。
        //
        // 用 %.9f 而非 %.17g: 定点记法永不出现指数形式 (朴素分列解析都照旧), 而 1e-9 已经远低于
        //   任何一条判据会去看的量级。
        //
        // 【这条规矩怎么验收】: 下一次采集后, 控制台打印的失拟/门限与【从落盘文件重放】得到的应当
        //   一致。这是"改日志精度"唯一能判定做完没做成的比对 —— 不做它, 就没法说它是做完了。
        for (int i = 0; i < count; i++) {
            fprintf(f, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
                       "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
                       "%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,"
                       "%d,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,"
                       "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                    pose[i][0], pose[i][1], pose[i][2],
                    pose[i][3], pose[i][4], pose[i][5],
                    bias[i][0], bias[i][1], bias[i][2],
                    bias[i][3], bias[i][4], bias[i][5],
                    biasSix[i][0], biasSix[i][1], biasSix[i][2],
                    biasSix[i][3], biasSix[i][4], biasSix[i][5],
                    samples[i],
                    sqrt(varSix[i][0]), sqrt(varSix[i][1]), sqrt(varSix[i][2]),
                    sqrt(varSix[i][3]), sqrt(varSix[i][4]), sqrt(varSix[i][5]),
                    biasTcp[i][0], biasTcp[i][1], biasTcp[i][2],
                    biasTcp[i][3], biasTcp[i][4], biasTcp[i][5]);
        }
        fclose(f);
    }

    // ========================================================================
    // ===== 整屏诊断的【一个 sink, 两处输出】: 屏幕 + calib\calib_report.md =====
    // ========================================================================
    // 从前这一屏只留在滚动的控制台上, 转录成文档全靠手抄 (而"操作员当时看到的整屏"正是
    // 唯一没有别处留下的东西 —— calib_log.txt 只有 22 个窄列, calib_poses.txt 只有原始数据)。
    // 现在求解路径上的每一行都【只经过这里】: 写 stdout 的那一份字节, 同时就是落进文档块的
    // 正文。所以"控制台与落盘逐字节同源"是【结构上】成立的 —— 不是两处各打一遍再指望它们
    // 长得一样 (那正是会漂移的做法, 而本项目的验收判据之一恰恰是两者必须一致)。
    //
    // 【为什么不直接重定向 fd 1 一把抓】: stdout 是【进程共用】的, RelayCore 的 125 Hz 读线程
    // 与触觉回调线程也在往它写 —— 重定向会把别的线程的输出一起卷进本次的文档块。
    // 所以这里只接管【求解路径自己】的打印。
    static std::string s_diagBody;       // 本次求解的正文 (= 与写往 stdout 的同一份字节)
    static std::string s_diagTs;         // 本次的采集时刻 (与 calib_poses.txt 的 # attempt 同来源同格式)
    static int  s_diagPoses = 0;         // 本次手上有几个姿态
    static int  s_diagPairs = 0;         // 本次登记了几对重复姿态 (尺子的自由度)
    static std::string s_diagWarn;       // 本次"有什么没能并进来" (空 = 没有); 只会进文档, 不改判决

    // 字节出口: 屏幕与正文【同一个调用】。别在这里加第二个出口 —— 那就变成"各打一遍"了。
    static void diagEmit(const char* s, size_t n) {
        fwrite(s, 1, n, stdout);
        s_diagBody.append(s, n);
    }

    // printf 风格的出口。求解路径上原本的 printf 逐个换成它 —— 【格式串与实参一个字都没动】,
    // 变的只有"往哪儿写"。用 vsnprintf 先量长度再写, 【不设固定上限】: 定长缓冲遇上变长的数
    // (比如 c_s 解飞了) 会静默截断, 而截断掉的正是落盘要保住的东西。
    static void diagEmitf(const char* fmt, ...) {
        char stackBuf[1024];
        va_list ap;
        va_start(ap, fmt);
        const int need = vsnprintf(stackBuf, sizeof(stackBuf), fmt, ap);
        va_end(ap);
        if (need < 0) return;
        if ((size_t)need < sizeof(stackBuf)) {
            diagEmit(stackBuf, (size_t)need);
            return;
        }
        std::vector<char> big((size_t)need + 1);
        va_start(ap, fmt);
        vsnprintf(big.data(), (size_t)need + 1, fmt, ap);
        va_end(ap);
        diagEmit(big.data(), (size_t)need);
    }

    // 与 std::cout 同形的出口 (供原来的 std::cout << ... << std::endl 链使用)。
    // 【只换写入端, 链本身不动】, 所以格式化规则与 std::cout 完全一样: 本文件里没有任何一处
    // 改过 std::cout 的格式状态 (已核对: 没有 setprecision / fixed / hex / setw)。
    // 用真的 streambuf 而不是"攒到 endl 再猜": endl 之外万一出现别的操纵器也不会走样。
    class DiagBuf : public std::streambuf {
    protected:
        int overflow(int c) override {
            if (c == EOF) return std::char_traits<char>::not_eof(c);
            const char ch = (char)c;
            diagEmit(&ch, 1);
            return c;
        }
        std::streamsize xsputn(const char* s, std::streamsize n) override {
            diagEmit(s, (size_t)n);
            return n;
        }
        // 【std::endl 的冲刷要在, 别以为它自己就有】: std::ostream::flush() 走到这里 ——
        // 不覆盖 sync() 的话它是个空操作 (这个 streambuf 没有别的缓冲, 编译器不会报错),
        // 而 C 的 stdout 在全缓冲时 (被重定向/接进管道时就是这样) 会一直憋到进程退出:
        // 一次跑到一半被结束的求解, 尾巴就没了。从前这里是 std::cout << ... << std::endl,
        // 那时 sync_with_stdio 默认 true, endl 确实会冲刷 —— 换成 diagOut() 之后这行为得自己接回来。
        // 只是把字节推出去, 【不改任何字节】。
        //
        // 【返回值必须是 0, 不是一个"失败码"】: 它唯一的作用是决定 std::ostream 要不要置 badbit
        // (std::ostream::flush() 在 pubsync() 返回非 0 时 setstate(badbit)), 而默认异常掩码下
        // 【不抛异常】—— 于是从那一刻起, 每一个 diagOut() << ... 都变成【静默的空操作】, 而
        // diagEmitf(...) 那一路照旧在打。屏幕和文档块会【同时】缺行, 而且没有任何提示。
        // 最容易撞上的场合: Touch_Client.exe > log.txt 跑到一半卷满 —— 第一次失败的
        // fflush(stdout) 就把这一轮剩下的 ostream 诊断永久静音了, 恰好是本文件开头点名的
        // "诊断被安静地藏起来"。字节已经由 diagEmit 的 fwrite 交给 C 流了, 这里没有"失败"可报
        // (C 流的写失败另有 ferror(stdout) 这条出路), 所以无条件 0。
        int sync() override {
            (void)fflush(stdout);
            return 0;
        }
    };
    static std::ostream& diagOut() {
        static DiagBuf buf;
        static std::ostream os(&buf);
        return os;
    }

    // 一次求解的开头: 定稿时刻与块头要的两个计数, 并把正文清空。
    static void diagBegin() {
        s_diagBody.clear();
        s_diagWarn.clear();
        s_diagPoses = count;
        s_diagPairs = repeatCount;
        // 时间戳的【来源与格式】与 logPoseData 的 "# attempt" 行逐字相同 —— 两份文件靠它对上。
        std::tm tmInfo;
        const std::time_t now = std::time(nullptr);
        localtime_s(&tmInfo, &now);
        char ts[24];
        strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", &tmInfo);
        s_diagTs = ts;
    }

    // 一次求解的收尾: 把正文 + 尾节包成块【追加】到 calib_report.md。
    // 【只追加, 永不截断】—— 打开方式见 SessionReport::appendToFile (只用 "a+")。
    // trailer = 块尾那一节 (机械臂自报负载), 由 diagPayloadSection 造。
    static void diagFinish(const std::string& trailer) {
        const std::string blk = SessionReport::block(s_diagTs.c_str(), s_diagPoses, s_diagPairs,
                                                     s_diagBody, trailer);
        char path[512];
        snprintf(path, sizeof(path), "%s", CalibStore::fileFor("calib_report.md"));
        if (!SessionReport::appendToFile(path, blk))
            diagOut() << "[BIAS] ⚠ 本次报告没落盘 (打不开 " << path << ") —— 整屏诊断只留在"
                      << "控制台上, 没有别的副本。" << std::endl;
        s_diagBody.clear();
    }

    // 块尾那第一节: 机械臂【自报】的负载 (@1168 Load / @1176 CenterX/Y/Z) + d = cz_robot − c_s
    // + (0, 31.5) mm 判据的结论。这是当天 (2026-09-19) 才定下来的新内容, 也正是这个文档
    // 必须带上的 (设计 §6b: 测量原点必须落在传感器体内)。
    //
    // ⚠ 这一节【只是信息】: 不参与任何接受/拒绝, 也不改变 fitRaw 的判决 (它是在判决之后、
    //   在块尾写的, 连读都读不到判决路径里)。
    // ⚠ 数值一律【全精度】(%.17g): 文档是给人读的, 但读的人要靠它复算 —— 沿用 %.3f 会把
    //   横向分量与判据的余量一起抹掉 (与 logPoseData 顶上"落盘精度"是同一条规矩)。
    // ⚠ 缺数据时【照实说"不可用"】, 不许静默省略、不许给 0 —— c_s 全 0 尤其会被读成
    //   "质心就在测量原点", 那是另一个意思 (与 logCalibAttempt 记 "-" 同一考虑)。
    //
    // haveCs = 本次有没有解出 c_s (线性层就没解出来时它是全 0, 不携带信息);
    // parity = sign(det A) (0 = 没解出来), 只用于把"手系"这件事一起记下来。
    // fitOk  = 本次 fitRaw 的判决。它【只用来限定 d 那一节的措辞】: 被拒的那一次块里几行之前
    //          才印着"【拒绝】", 尾节再说一句无条件的"测量原点落在传感器体内"就是同一个块里
    //          两个互相打架的结论。传进来的是判决的【结果】, 这里改不了它, 也不参与任何判决
    //          (brief 硬要求 6) —— 它只决定那一行后面【要不要跟一句"该结论不成立"】。
    static std::string diagPayloadSection(bool haveCs, const double cS[3], double parity,
                                          bool fitOk) {
        std::string s = "\n### 机械臂自报负载 (30004 帧)\n";

        bool echoOk = false;
        double loadKg = 0.0;
        double ctr[3] = {0.0, 0.0, 0.0};
        EnterCriticalSection(&appState.forceDataMutex);
        echoOk = appState.forceData.payloadEchoValid;
        loadKg = appState.forceData.payloadEchoLoadKg;
        for (int i = 0; i < 3; i++) ctr[i] = appState.forceData.payloadEchoCenterMm[i];
        LeaveCriticalSection(&appState.forceDataMutex);

        char buf[512];
        if (echoOk) {
            snprintf(buf, sizeof(buf), "- @1168 Load        = %.17g kg\n", loadKg);
            s += buf;
            snprintf(buf, sizeof(buf), "- @1176 CenterX/Y/Z = (%.17g, %.17g, %.17g) mm\n",
                     ctr[0], ctr[1], ctr[2]);
            s += buf;
        } else {
            s += "- @1168 Load / @1176 CenterX/Y/Z = 【不可用】: 30004 帧里还没回读到负载\n"
                 "  (没连机械臂 / 那一帧还没读进来)。这是【没有数据】, 不是 0。\n";
        }

        if (!haveCs) {
            s += "- c_s (本次解出) = 【不可用】: 本次没解出 c_s (线性层就没解出来 / 姿态数不足)\n"
                 "  ⇒ 沿工具轴的分量无从给出, 下面的 d 判据【无法判定】。\n";
            if (!s_diagWarn.empty()) s += s_diagWarn;
            s += "- ⚠ 本节只是信息: 不参与任何接受/拒绝, 也不改变 fitRaw 的判决。\n";
            return s;
        }

        const double cx = cS[0] * 1000.0, cy = cS[1] * 1000.0, cz = cS[2] * 1000.0;
        const double cMag = sqrt(cx * cx + cy * cy + cz * cz);
        snprintf(buf, sizeof(buf),
                 "- c_s (本次解出, 传感器测量系; 原点 = 传感器的测量原点) = "
                 "(%.17g, %.17g, %.17g) mm, |c_s| = %.17g mm\n", cx, cy, cz, cMag);
        s += buf;
        snprintf(buf, sizeof(buf),
                 "  沿【工具轴】的分量 c_s_z = %.17g mm (该系的 z 轴就是工具轴: 重力模型用\n"
                 "  g = Rᵀ·(0,0,g) 且 psi 传 0, 而 psi 是绕 z 的旋转 —— z 分量与 psi 无关);\n"
                 "  横向分量 c_s_x / c_s_y = %.17g / %.17g mm。parity = sign(det A) = %.17g\n",
                 cz, cx, cy, parity);
        s += buf;
        // ⚠ 这一句【不许替 c_s_z 假定符号】: 本次解出的是 c_s_z 本身, 它的符号不由数据
        //   定 (上面那一段), 所以"反向约定给的是另一个正数 / 不是变负"这种话只在
        //   c_s_z > 0 时成立 —— 而 126cadc 起负值那一支是【明写在测的】
        //   (test_session_report.cpp 的 ..._negative_cs_prints_no_fabricated_signs)。
        //   下面只说【两支的关系】: 约定二 = 约定一 + 2·c_s_z, 谁大谁落进判据全随符号反转。
        s += "  ⚠ 该系 z 轴与工具轴的【指向】是否同向, 数据定不了: 模型在 g → −g, A → −A,\n"
             "    c_s → −c_s 下逐字不变 (A 是自由 3×3, 反射由 parity 报出; 实机上解出\n"
             "    parity = −1), 所以【只有 |c_s_z| 是数据定的, 符号不是】。下面两支的关系是\n"
             "    【约定二 = 约定一 + 2·c_s_z】(即 cz_robot − c_s_z 与 cz_robot + c_s_z):\n"
             "    c_s_z 为正则约定二偏大, 为负则约定一偏大 —— 谁更大、谁落到零以下, 都随本次\n"
             "    解出的符号反过来。所以下面 d 的两支都算、都打。\n";

        if (echoOk) {
            // 【两个约定都算】—— 见 SessionReport::payloadDSection 顶上那段说明:
            // c_s_z 的符号不由数据决定, 而它决定 d 落在判据哪一边。这里【不给单一的 ✓/✗】。
            // fitOk 只决定要不要在结论后面跟一句"该结论不成立" (被拒的那一次), 不参与判决。
            s += SessionReport::payloadDSection(ctr[2], cz, fitOk);
        } else {
            // 判据的两个端点【不许】在这里写第二遍 (从前这行写死的是 "0 < d < 31.5") —— 阈值
            // 只有一份实现: SessionReport::PAYLOAD_D_MIN_MM / _MAX_MM (闸1 用的也是它们)。
            snprintf(buf, sizeof(buf),
                     "- d = cz_robot − c_s_z = 【不可用】: 缺 cz_robot (@1176 CenterZ)\n"
                     "  ⇒ 判据 %.1f < d < %.1f mm 【无法判定】\n",
                     SessionReport::PAYLOAD_D_MIN_MM, SessionReport::PAYLOAD_D_MAX_MM);
            s += buf;
        }
        if (!s_diagWarn.empty()) s += s_diagWarn;
        s += "- ⚠ 本节只是信息: 不参与任何接受/拒绝, 也不改变 fitRaw 的判决。\n";
        return s;
    }

    // 's': 用已采数据【拟合原始力通道】, 全部打印出来, 并在判决通过时【装进本地补偿 + 落盘】。
    //
    // ===== 2026-09-20 起这条路重新"应用"东西了 —— 但只应用【本地】那一半 =====
    // 2026-09-19 (Task 4) 曾把它改成"只打印、什么都不应用", 而当时那条链是断的:
    //   求解出来的 (A, b_F, b_M, c_s) 没有任何一条路能进 ForceCompensation —— 唯一的安装
    //   入口 setMassCom 在 Task 6 随残余模型一并删除, 而计划里唯一以"本地全量补偿"为标题的
    //   Task 只定义了文件格式与补偿公式, 【没有安装点】。
    //   ⇒ 本地补偿永远停在"没有模型", 一致性闸门在【做比较之前】就早退, 报的是
    //     UNCALIBRATED ("没有可比的模型") 而不是 INCONSISTENT ("两边对不上")。
    //   现场证据: 一整场会话里 force_calib.json 从未产生过, 而控制台每 5 s 报一次"未标定"。
    // 现在按判决 (fitOk, 与发送候选同源) 走两步: setCalibration 装进本会话内存 +
    // saveToFile 按 version 3 写 force_calib.json (下次启动自动装载)。
    //
    // 【应用的是本地补偿, 不是机械臂 —— 这两件事必须分开读】
    //   · 本地补偿 compensated = sixForceRaw(@1304) − b_F − A·g 全程在【传感器系内闭环】,
    //     A / b_F / c_s 都是这次从 @1304 直接解出来的 ⇒ 不需要任何法兰换算, 现在就能装;
    //   · 机械臂的负载模型 (EnableRobot 的 load/center) 要的是【挂在它法兰上的整条链】,
    //     从"测量原点以下"换过去要走过 c_s 的原点在哪 + 法兰→测量系那一步, 而这一步
    //     【未定】(spec §6b 末: 标定 c_s = 54.55 mm 与解析几何反推的 75.8 mm 对不上,
    //     而传感器总高只有 31.5 mm)。
    //   ⇒ 下发路径的开通条件仍在 plan Task 9; 本条路径【一个字节都不发给机械臂】,
    //     也【仍然不写 payload_calib.json】。别把这两件事一起接回来。
    //
    // 于是本次的产出有三样: 控制台上那一屏 + calib_log.txt 里的一行 (够在控制台滚掉之后
    // 回看) + 本会话生效 (并已落盘) 的本地补偿模型。
    // ===== Task 8a: 两道闸的结论行 —— 只此一份 =====
    // 两个地方要说同一句话: 按 's' 时打的那一行结论, 与按 'p' 被拒时说"是哪一闸、为什么"。
    // 各写一遍就会出现"屏幕上的结论"与"拒发时的解释"互相打架 —— 本项目栽过这种同型。
    // 判决本身在纯函数 PayloadCalibration::evaluateSendGate 里 (单测覆盖), 这里只把它翻成人话。
    static void formatSendGateConclusion(const PayloadCalibration::SendGate& g, char* out, int len) {
        switch (g.verdict) {
        case PayloadCalibration::SEND_OK:
            snprintf(out, len,
                     "候选可发送（按 'p'）—— 发送的是 m = %.4f kg, "
                     "(cx, cy, cz) = (%.1f, %.1f, %.1f) mm",
                     g.massKg, g.comMm[0], g.comMm[1], g.comMm[2]);
            break;
        case PayloadCalibration::SEND_NOT_MEASURED:
            // 与量级闸【分开】的理由见 PayloadCalibration.h 的 SendMassSource 说明:
            // "数看着合理但来路不对"要去查"标定为什么没跑", 不是去查装夹。
            snprintf(out, len,
                     "候选不可发送：这个质量尺度【不是本次实测的】(种子值 / 上次落盘值) —— "
                     "只有 Decomp::m (本次解出的) 才能当候选; 发旧值等于把机械臂现在就有"
                     "的那一份重发一遍");
            break;
        case PayloadCalibration::SEND_NO_CS:
            snprintf(out, len,
                     "候选不可发送：闸1 无法判定 —— 【没有 c_s】(本次没解出, 不是符号不对)");
            break;
        case PayloadCalibration::SEND_NO_CZ_ROBOT:
            snprintf(out, len,
                     "候选不可发送：闸1 无法判定 —— 【没有 cz_robot】(@1176 还没回读到负载;"
                     " 不退回本客户端下发的值当参照)");
            break;
        case PayloadCalibration::SEND_SIGN_AMBIGUOUS:
            snprintf(out, len,
                     "候选不可发送：闸1 不放行 —— 【两种符号约定都落在 (%.1f, %.1f) mm 内】"
                     "(数据定不了符号, 需要别的论据, 不许二选一猜)",
                     SessionReport::PAYLOAD_D_MIN_MM, SessionReport::PAYLOAD_D_MAX_MM);
            break;
        case PayloadCalibration::SEND_SIGN_NONE_IN_RANGE:
            snprintf(out, len,
                     "候选不可发送：闸1 不放行 —— 【两种符号约定都不在 (%.1f, %.1f) mm 内】"
                     "(哪里错了: 测量原点本该落在传感器体内)",
                     SessionReport::PAYLOAD_D_MIN_MM, SessionReport::PAYLOAD_D_MAX_MM);
            break;
        case PayloadCalibration::SEND_MASS_OUT_OF_RANGE:
            snprintf(out, len,
                     "候选不可发送：闸2 不放行 —— m = %.4f kg 不在 [%.1f, %.1f] kg 内",
                     g.massKg, PayloadCalibration::SEND_GATE_MASS_MIN_KG,
                     PayloadCalibration::SEND_GATE_MASS_MAX_KG);
            break;
        case PayloadCalibration::SEND_COM_OUT_OF_RANGE:
            // 判据是【严格小于】500 (操作单 §6 闸2 原文, 用户 2026-09-20 裁定) ⇒ 被拒的那一
            // 侧是 >= 500, 所以措辞是"不小于"而不是"超过" (恰好 500 时"超过"会说谎)。
            snprintf(out, len,
                     "候选不可发送：闸2 不放行 —— |c| = %.1f mm 不小于 %.0f mm",
                     g.comMagMm, PayloadCalibration::SEND_GATE_COM_MAX_MM);
            break;
        default:
            snprintf(out, len, "候选不可发送：未知判决值 (不该发生)");
            break;
        }
    }

    static void solveAndApply() {
        // ===== 从这一行起, 本次按 's' 的所有输出都进文档 =====
        // 【放在最前面, 而不是"判决之前"】: 被拒的那几次同样要留下 (brief 硬要求 8), 而这里是
        // 唯一一个【每一次按 's' 都必然经过】的位置 —— 连下面那两个提前 return (姿态数不足 /
        // 正在采样) 也一并被记下来, 不会出现"控制台上打了、文档里没有"的口子。文档块里的正文
        // 因此【就是这一次按 's' 在屏幕上出现的那一份字节】, 逐字一致。
        diagBegin();
        // ===== 连续失败计数: 【警告, 不是闸门】 =====
        // 从前这里是一个 return: 第 3 次拒绝起, 整屏诊断被一行 "REJECTED locked_out" 顶掉,
        // 而且【没有任何别的办法把它再弄出来】(logCalibAttempt 只落 22 个窄列, 没有 A / Q /
        // c_s / σ)。在"只打印、不应用"的今天, 那个 return 的【唯一效果】就是藏掉输出 ——
        // 而首次实机跑本来就以被拒为常态 (还没有 'r' 重复对 / 覆盖不足), 操作员反复按 's'
        // 看输出是最自然的用法, 一按就退化成一行。所以: 计数照记、照报 (它本身有信息量),
        // 但【不再中止这条路】—— 下面的诊断体一律照跑照打。
        // solveLocked 因此不再决定"跑不跑"; 它只被 reset()/这里写, 读它的地方是这一行,
        // 意思是"已经连续被拒这么多次了"。真正的上锁语义属于【被停用的应用路径】(Task 11 处理)。
        if (solveLocked) {
            diagOut() << "\n[BIAS] 提示: 已连续 " << consecutiveFails << " 次被拒 ——"
                      << " 仍照常求解并打印全部诊断 (不写补偿 / 不写 json / 不下发机械臂,"
                      << " 不存在 \"写坏\" 的风险)。\n"
                      << "       反复按 's' 不会变好; 按 'm' 重新采集会把计数清零。" << std::endl;
        }
        if (count < 4) {
            diagOut() << "[BIAS] 求解至少需要 4 个姿态 (当前 " << count
                      << "), 建议 6~8 个" << std::endl;
            char outcome[128];
            snprintf(outcome, sizeof(outcome), "REJECTED too_few_poses (count=%d)", count);
            logCalibAttempt(outcome, nullptr, count);
            // 被拒的这一次也要留下块 (正文就是上面那一行) —— 没有 c_s, 尾节照实报"不可用"。
            // 末一个实参是 fitRaw 的判决: 这里【fitRaw 根本没跑】, 传 false 是照实说
            // (而且 haveCs = false 时 d 那一节整段不打印, 它在这里不产生任何字)。
            diagFinish(diagPayloadSection(false, nullptr, 0.0, false));
            return;
        }
        // 姿态数够了就落盘 —— 【在任何拒绝判据之前】: 被拒绝的那几次同样要留下数据,
        // 否则"为什么被拒"这件事就只剩控制台上滚掉的那几行。只追加, 见 logPoseData。
        logPoseData();
        // 尺子的状态也照实说一句: 新求解路径 (fitRaw) 的模型形式检验【要它才成立】,
        // 而"没按 'r'"是操作员最容易漏的一步 —— 让它在控制台上可见, 别等到被拒才发现。
        // 现在还要报出【尺子本身的值】(见 record 里那段说明): 尺子被误登记 (按 'r' 之前先动了
        // 机械臂) 时它会大出一个数量级, 而门的宽度正比于它 —— 一个 0.5 N 的尺子必须当场看得见。
        if (repeatCount > 0) {
            diagOut() << "[BIAS] 重复姿态对 (" << repeatCount << " 对, 每对取不同姿态时 = 尺子的自由度):";
            for (int i = 0; i < repeatCount; i++)
                diagOut() << " (pose " << repeatFirst[i] + 1 << ", pose " << repeatIdx[i] + 1 << ")";
            diagOut() << " —— 原地复采的复现性尺子就位" << std::endl;
            // 逐对 + 池化后的尺子读数 (@1304 原始通道; 求解侧喂的是同一份【未镜像】的原始值,
            // 所以这里的数与 fitRaw 算出的是同一个)。差值取【每一对自己的两笔】—— 与传给
            // fitRaw 的 RepeatPair 逐字同源 (first = repeatFirst[i], 不再有"恒为 pose 1")。
            double pooled[3] = {0.0, 0.0, 0.0};
            for (int a = 0; a < 3; a++) {
                diagEmitf("         力%c: ", "xyz"[a]);
                for (int i = 0; i < repeatCount; i++) {
                    const int r0 = repeatFirst[i];
                    const int r  = repeatIdx[i];
                    const double d = biasSix[r][a] - biasSix[r0][a];
                    const double s0 = (samples[r0] > 0) ? varSix[r0][a] / samples[r0] : 0.0;
                    const double s1 = (samples[r] > 0) ? varSix[r][a] / samples[r] : 0.0;
                    double ex = 0.5 * (d * d - s0 - s1);
                    if (!(ex > 0.0)) ex = 0.0;
                    const double sig2 = ex + 0.5 * (s0 + s1);
                    pooled[a] += sig2;
                    diagEmitf("pair%d d=%+.4f σ_rep=%.4f | ", i + 1, d, sqrt(sig2));
                }
                pooled[a] /= repeatCount;
                diagEmitf("池化 σ_rep=%.4f N\n", sqrt(pooled[a]));
            }
            diagEmitf("         姿态级尺子 (三通道 σ_rep² 均值再开方) = %.4f N —— 判决门限正比于它\n",
                      sqrt((pooled[0] + pooled[1] + pooled[2]) / 3.0));
            if (repeatCount < 2)
                diagOut() << "[BIAS] 尺子只有 " << repeatCount << " 对 —— 尺子自己不够稳: "
                          << FALSE_REJECT_RATES
                          << "。★ 请【换姿态】补到 ≥5 对 (保持不动按 'r' + SPACE);"
                          << " 力矩分支在 R=3 时仍有 7~23% 的冤枉率, 补到 5 对才压到 1% 量级。"
                          << std::endl;
        } else {
            diagOut() << "[BIAS] !! 没有重复姿态对 —— 模型形式检验 (fitRaw) 将【无从判定】并"
                      << "拒给参数。摆好一个姿态按 SPACE 采一次, 【保持不动】按 'r' 再按 SPACE"
                      << " 采一次配成一对; 【每一对换一个姿态】, 建议 ≥5 对 (力矩分支的冤枉率"
                      << " 在 R=5 才降到 1% 量级)。" << std::endl;
        }
        // 采样中途不允许求解
        if (sampling) {
            diagOut() << "[BIAS] 正在采样, 稍后再求解" << std::endl;
            logCalibAttempt("REJECTED sampling_in_progress", nullptr, count);
            // 同上: 被拒的这一次也留下块 (正文 = 尺子那几行 + 这一行), 尾节照实报"不可用"。
            // 同上, fitRaw 没跑 -> 末一个实参 false (haveCs = false, d 那一节不打印)。
            diagFinish(diagPayloadSection(false, nullptr, 0.0, false));
            return;
        }

        // ============================================================
        // ===== 新模型: 原始 @1304 通道的【全线性】解 (无 psi / 无镜像 / 无基线) =====
        // ============================================================
        // spec §2 的力学关系 (不是猜测, 是力学):
        //     F_i = b_F + A · g_i            A: 3×3, 【9 个元素全部由数据定】
        //     M_i = b_M + c_s × (A · g_i)     c_s: 3, 质心 (传感器测量系)
        //     g_i = TcpCalibration::gravitySensorFrameAtYaw(pose_i, 0.0, g)   <- psi 传 0
        // 未知量 9+3+3+3 = 18, 姿态数 n 给 6n 个方程, 【全部线性】-> 一次求解, 不扫描。
        // 分解 (m / Q / parity) 由 decompose(A) 给出, 也不是扫出来的 (spec §3)。
        //
        // 【喂 biasSix[] 原始值, 不做 z 镜像】
        //   镜像那一步的由来是【旧模型】只有 psi 一个自由度, 装不下反射, 只好在边界上手工
        //   把 z 翻过去 (旧路径那段长注释就在下面的 #if 0 里)。新模型的 A 是自由 3×3,
        //   反射与非正交一起吸收, 手系由 det(A) 的符号【报出来】。再镜像一次等于替数据
        //   预定了手系 —— 那正是本轮要拆掉的预设 (spec §1 表格 "z 符号 / 边界手工对齐")。
        //
        // 【也不做基线差商】
        //   旧模型解的是"机械臂没补干净的那一份"(真值 − 机械臂配置值), 所以要拿 @1168 自报
        //   负载当基线才能折回绝对值。新模型解的是【绝对量】: b_F 自己吸收传感器零偏, A 就是
        //   响应本身, 式子里根本没有"基线"这一项 —— 于是也不再依赖机械臂自报的是什么。
        static double sp[12][6];
        static double sf[12][3];
        static double sm[12][3];
        for (int i = 0; i < count; i++) {
            // 姿态转成求解器要的 [x,y,z,rx,ry,rz]: 本模块存的是 [rx,ry,rz,x,y,z]。
            // ⚠ 这个置换排错的话, 求解器会把【位置】当成角度去算重力, 而解出来的 A 依旧长得
            //   像一个合法的响应矩阵 (只是错的) —— 又一次"安静地解错"。离线金标那条用例
            //   (test_payload_calibration.cpp) 用的是逐字相同的重排, 别在这里改顺序。
            sp[i][0] = pose[i][3]; sp[i][1] = pose[i][4]; sp[i][2] = pose[i][5];
            sp[i][3] = pose[i][0]; sp[i][4] = pose[i][1]; sp[i][5] = pose[i][2];
            for (int a = 0; a < 3; a++) {
                sf[i][a] = biasSix[i][a];         // 力: 原始未镜像
                sm[i][a] = biasSix[i][3 + a];     // 力矩: 原始未镜像
            }
        }

        // 逐姿态【实测】噪声 (与上面的均值同一批样本、同一时间窗采下来的; 不是从残差反推的,
        // 所以没有"自己量自己"的毛病)。它有两个用途: 报"这个姿态采得稳不稳"; 以及当尺子的
        // 扣噪项 —— 重复对的差值里要扣掉姿态内噪声那一份, 剩下的才是姿态【间】的复现性。
        static PayloadCalibration::PoseNoise nz[12];
        for (int i = 0; i < count; i++) {
            nz[i].n = samples[i];
            for (int a = 0; a < 3; a++) {
                nz[i].varF[a] = varSix[i][a];
                nz[i].varM[a] = varSix[i][3 + a];
            }
        }
        // 重复姿态对 = 模型形式检验的【尺子】。first / second 【两个都是采集侧登记的】(协议:
        // 原地复采, first 是 'r' 时手上那一笔, 即 second 的前一笔); 按 'r' 是【追加】, 所以
        // 这里可能有多对。一对都没有 -> 尺子没有 -> fitRaw 拒给参数 (不是"没验过也放行")。
        static PayloadCalibration::RepeatPair reps[8];
        for (int i = 0; i < repeatCount; i++) {
            reps[i].first  = repeatFirst[i];
            reps[i].second = repeatIdx[i];
        }

        PayloadCalibration::RawFit fit;
        // ===== 把库打到 stderr 的那一段也并进本次的文档块 =====
        // fitRaw 的【逐姿态残差表】与所有 [Payload] 自检拒绝行 (含"最差是哪个姿态") 是"为什么
        // 被拒"的唯一出处 —— 它们在操作员的屏幕上, 却不经过 diagEmit (库自己往 stderr 打,
        // 用的是 C 的 fprintf —— 所以这里的检查里连那个字样都不留, 免得回归闸门误报)。
        // 所以在这里开一个捕获窗口: 窗口内 fd 2 指向临时文件, 收完【先无条件还原】, 再把收到的
        // 字节【从同一个出口】发出去 (diagEmit) —— 屏幕上的字节序列与块里因此仍然逐字相同。
        // 失败时 stderr 一个字节都不动, 只在块尾照实说一句"这一段没并进来" (不许静默省略)。
        // ⚠ 这个字符串是临时文件的【基名】, 不是最终路径: Begin 会用它派生一个【本次专属】的名字
        //   (基名 + .pid_序号; 见 SessionReport::captureTmpPathFor), 而且【绝不截断已存在的文件】
        //   (_O_EXCL) —— 否则操作员"再按一次 's'"这个最自然的动作, 会把上一次那条"字节还在
        //   临时文件里"的警告所指的【唯一副本】当场截成 0 字节 (两处都说了它没进控制台)。
        //   报出去给操作员看的路径取自 stderrCaptureLeftoverPath(), 所以与盘上那个文件必然一致。
        char errTmp[512];
        snprintf(errTmp, sizeof(errTmp), "%s", CalibStore::fileFor("calib_stderr.tmp"));
        std::string errText;
        const bool errCapOk = SessionReport::stderrCaptureBegin(errTmp);
        // MODEL_FORM_FORCE_ONLY (2026-09-20 起用这条): 尺子不齐【仍然拒给参数】; 力通道不通过
        // 【仍然拒】; 变的只是【力矩通道的失拟不再拦整体判决】。
        //
        // 为什么改: 实机上力矩分支经常拒绝, 而 2026-09-19 已查明成因是 A_F ≠ A_m (力通道估出的
        //   A 不是力矩通道想要的那个), 【不是这批数据脏】—— 2026-09-20 20:52 那次就是:
        //   cond 17.49 (历次最好)、力通道 2.422 < 2.927、尺子 0.0251 N, 只因力矩 35.97 > 10.86
        //   而整体被拒 —— 那份数据什么也装不上, 而下发候选也跟着作废。
        //   判据本身一个字没动 (统计量/门限/打印全在库里), 动的只是"力矩没过时整体还拒不拒"。
        //   ⚠ 力矩没过时 momentFormChecked 保持 false —— 所以下面那行判决【必须】读它, 不能
        //   只看 fitOk, 否则屏幕上会印一个"通过"而力矩那一半其实没过。
        //
        // (不得传 I_ACCEPT_UNVERIFIED_MODEL_FORM: 那个令牌的意思是"模型形式【从未被检验过】
        //  也给我参数"。这里手上就有采集现场 (逐姿态方差与重复对都是刚采的), 没有理由接受一个
        //  没验过的模型。令牌只属于离线重放。)
        const bool fitOk = PayloadCalibration::fitRaw(sp, sf, sm, count, fit, nz, reps,
                                                      repeatCount,
                                                      PayloadCalibration::MODEL_FORM_FORCE_ONLY);
        if (errCapOk) {
            if (!SessionReport::stderrCaptureEnd(&errText)) {
                // 【照实说, 而且要说出字节在哪儿】: 收不回来时临时文件【故意不删】(见
                // SessionReport.h 的 stderrCaptureEnd) —— 上面那几行字节此刻【只有这一个副本】,
                // 它们既没进控制台 (fd 2 在窗口里就指着这个文件) 也没进本块。所以路径必须报出来,
                // 否则这一段就是被安静地销毁了 —— 本项目最不能接受的一类失败。
                s_diagWarn = "      ⚠ 库打到 stderr 的那一段 (逐姿态残差表 / [Payload] 自检行)"
                             "收回来了但读不出来 —— 这一段没能并入本块。\n"
                             "        原始字节没有丢, 它们还在临时文件里: ";
                s_diagWarn += SessionReport::stderrCaptureLeftoverPath();
                s_diagWarn += "\n        (读不出来时【不删】临时文件 —— 删了就连副本都没了;"
                              " 这一段没进控制台, 也没进本块。)\n";
            }
            diagEmit(errText.data(), errText.size());
        } else {
            s_diagWarn = "      ⚠ 库打到 stderr 的那一段 (逐姿态残差表 / [Payload] 自检行)"
                         "没能并入本块 (stderr 捕获窗口没搭起来) —— 它只在控制台上。\n";
        }

        // ===== 打印: 所有量都带【互相分得开】的标签 =====
        // 这一屏是本次唯一的产出 —— 人靠它判"这次标定到底成不成", 所以宁可长, 不可糊:
        // 质量尺度 / A 的 9 个元素 / 奇异值 / 各向同性比 / parity / 安装旋转 Q / c_s /
        // 两个 rms / 条件数 / 逐参数不确定度 / 模型形式判决 (拒绝时是哪一种尺子缺了)。
        // 逐姿态残差的表由 fitRaw 自己打到 stderr (含"最差是哪个姿态"), 不在这里重复。
        PayloadCalibration::Decomp d;
        d.m = 0.0; d.parity = 0.0; d.isotropyRatio = 0.0;
        for (int k = 0; k < 3; k++) d.sv[k] = 0.0;
        for (int k = 0; k < 9; k++) d.Q[k] = 0.0;
        // 【无论 fitRaw 通不通过都做分解】: 被拒时 fit.A 里是【未经自检的线性解】(头文件的
        // 调用契约明文如此), 而 m / 奇异值 / parity 恰恰是判断"这次到底错在哪"最要紧的几个数。
        // 从前这里写成 `if (fitOk)`, 于是"模型形式被拒"时它们一个都不打印 —— 把最有信息量的
        // 那几个数藏在了最需要它们的时刻。A 真的是全 0 (线性层就没解出来) 时 decompose 自己
        // 会失败, 不需要在这里替它挡。
        const bool decompOk = PayloadCalibration::decompose(fit.A, d);
        // det(A) —— parity 就是它的符号; 单独算一份是为了把它【照实打出来】给人看。
        // 只在 fitOk 时有意义 (线性层就失败时 A 是全 0)。
        const double detA = fit.A[0] * (fit.A[4] * fit.A[8] - fit.A[5] * fit.A[7])
                          - fit.A[1] * (fit.A[3] * fit.A[8] - fit.A[5] * fit.A[6])
                          + fit.A[2] * (fit.A[3] * fit.A[7] - fit.A[4] * fit.A[6]);

        diagOut() << "\n======================================================" << std::endl;
        diagOut() << "  原始通道 (@1304 SixForceValue) 线性解 — " << count << " 个姿态" << std::endl;
        diagOut() << "======================================================" << std::endl;
        diagOut() << "  模型:  F = b_F + A·g        M = b_M + c_s × (A·g)" << std::endl;
        diagOut() << "         g = 重力在【传感器测量系】的表示 = R_iᵀ(0,0,9.81); 这个模型里【没有 psi】"
                  << std::endl;
        diagOut() << "  A 的 9 个元素全部自由: 不预设旋转 / 不预设手系 / 不预设偏航;" << std::endl;
        diagOut() << "  无 z 镜像, 无基线差商 (绝对量直接解出) —— 见 solveAndApply 顶上的说明。" << std::endl;
        diagOut() << "------------------------------------------------------" << std::endl;
        diagOut() << "  A (3×3, row-major; 行 = 力分量 x/y/z, 列 = g 的 x/y/z; 量纲 kg):" << std::endl;
        for (int r = 0; r < 3; r++) {
            diagEmitf("      [ %+.7f   %+.7f   %+.7f ]\n",
                      fit.A[3 * r], fit.A[3 * r + 1], fit.A[3 * r + 2]);
        }
        diagEmitf("  b_F (力零偏, N):     (%+.5f, %+.5f, %+.5f)\n", fit.bF[0], fit.bF[1], fit.bF[2]);
        diagEmitf("  b_M (力矩零偏, N·m): (%+.5f, %+.5f, %+.5f)\n", fit.bM[0], fit.bM[1], fit.bM[2]);
        diagOut() << "------------------------------------------------------" << std::endl;
        if (decompOk) {
            diagEmitf("  质量尺度 m = (σ1σ2σ3)^(1/3)               = %.6f kg\n", d.m);
            diagEmitf("  A 的奇异值 (降序)  σ1/σ2/σ3               = %.6f / %.6f / %.6f  (kg)\n",
                      d.sv[0], d.sv[1], d.sv[2]);
            diagEmitf("  各向同性比 σ1/σ3                          = %.5f\n", d.isotropyRatio);
            diagOut() << "      ↑ 【报告量, 不作门限】: A 没有任何正交约束, 非正交是"
                      << "\"这只传感器的响应长这样\"" << std::endl;
            diagOut() << "        的测量结果 (物理属性), 不是模型形式错的证据。" << std::endl;
            diagEmitf("  parity = sign(det A)                      = %+.0f   (det A = %+.7f)\n",
                      d.parity, detA);
            diagOut() << "      ↑ 【手系由数据给出】: +1 = 无反射; -1 = 含一次反射 (实机这批就是 -1)。"
                      << std::endl;
            diagEmitf("  安装旋转 Q = S·A/m  (S = diag(1,1,parity); det Q = +1, 行/列同 A):\n");
            diagOut() << "      ↑ 它【恰好】是旋转矩阵只在 A 正交 (各向同性比 = 1) 时成立; 一般地它是"
                      << "含手系的安装姿态," << std::endl;
            diagOut() << "        非正交的那一部分照实留在 Q 里 (与 A 的各向同性比是同一件事的两面)。"
                      << std::endl;
            for (int r = 0; r < 3; r++) {
                diagEmitf("      [ %+.7f   %+.7f   %+.7f ]\n",
                          d.Q[3 * r], d.Q[3 * r + 1], d.Q[3 * r + 2]);
            }
        } else {
            diagOut() << "  【decompose 没做成 (A 奇异 / 线性层就没解出来)】—— m、奇异值、各向同性比、"
                      << std::endl;
            diagOut() << "  parity、安装旋转 Q 【一律无从给出】。这不是 0, 是【没有】。" << std::endl;
        }
        diagEmitf("  c_s (质心, 【传感器测量系】)              = (%+.3f, %+.3f, %+.3f) mm\n",
                  fit.cS[0] * 1000.0, fit.cS[1] * 1000.0, fit.cS[2] * 1000.0);
        diagEmitf("      |c_s|                                = %.3f mm\n",
                  sqrt(fit.cS[0] * fit.cS[0] + fit.cS[1] * fit.cS[1] + fit.cS[2] * fit.cS[2]) * 1000.0);
        diagOut() << "      ↑ 【原点 = 传感器的测量原点】, 不是法兰面、不是整条工具链 —— 见下面 ★。"
                  << std::endl;
        diagEmitf("  拟合残差  rmsForceN                       = %.6f N\n", fit.rmsForceN);
        diagEmitf("  拟合残差  rmsMomentNm                     = %.6f N·m\n", fit.rmsMomentNm);
        diagEmitf("  条件数    cond (力通道设计矩阵 σmax/σmin) = %.4f\n", fit.cond);
        diagOut() << "      ↑ 姿态激发够不够: 数值大 = 某几个 A 的分量没被姿态覆盖好,"
                  << " 参数定不下来。" << std::endl;
        {
            double sigA = 0.0, sigB = 0.0, sigC = 0.0, sigM = 0.0;
            for (int k = 0; k < 9; k++) if (fit.paramSigma[k] > sigA) sigA = fit.paramSigma[k];
            for (int k = 0; k < 3; k++) {
                if (fit.paramSigma[9 + k]  > sigB) sigB = fit.paramSigma[9 + k];
                if (fit.paramSigma[12 + k] > sigC) sigC = fit.paramSigma[12 + k];
                if (fit.paramSigma[15 + k] > sigM) sigM = fit.paramSigma[15 + k];
            }
            diagEmitf("  参数 1σ 不确定度 (18 个; 量纲随参数):\n");
            diagEmitf("      A   最大 %.3g  逐元素 %.3g %.3g %.3g / %.3g %.3g %.3g / %.3g %.3g %.3g\n",
                      sigA,
                      fit.paramSigma[0], fit.paramSigma[1], fit.paramSigma[2],
                      fit.paramSigma[3], fit.paramSigma[4], fit.paramSigma[5],
                      fit.paramSigma[6], fit.paramSigma[7], fit.paramSigma[8]);
            diagEmitf("      b_F 最大 %.3g N    c_s 最大 %.3g (= %.3g mm)    b_M 最大 %.3g N·m\n",
                      sigB, sigC, sigC * 1000.0, sigM);
            diagOut() << "      ↑ 【不参与任何接受/拒绝判据】: 它来自拟合残差, 拿它当门限就是自指"
                      << " (模型形式错 -> 残差涨 -> 门限跟着松)。" << std::endl;
        }
        // 姿态级尺子 (与 fitRaw 内部同一个口径: 三通道 σ_rep² 的均值再开方) —— 判决的宽窄
        // 正比于它, 所以它必须与判决一起打印, 否则"离门限多远"无从判读。
        // ⚠ 【先平方再平均, 最后开方】—— repeatSigmaF[] 里装的是 σ (N), 不是 σ² (同一个
        // RawFit 里装方差的字段叫 repeatSysF[], 看 fitRaw 打印那两行就分得清)。这里从前漏了
        // 平方, 于是屏幕上这个"尺子"与同一屏库打的那一行对不上: 实机 2026-09-19 18:49:55 那次
        // 库说 0.0208 N, 这里说 0.1349 N (sqrt(Σσ/3) 而不是 sqrt(Σσ²/3), 差 6.5 倍) —— 同一个
        // 名字、同一屏、两个数。库的 poseLevelYardstick 就是带平方的那个口径, 照抄它。
        // 【这只是显示量的算法, 不参与任何判决】: 它是【报告量】, 只喂给下面【四处】diagEmitf ——
        // 判决【通过】那一支两处 (力通道 / 力矩通道各一行) 与判决【拒绝】那一支两处 (照抄判据里
        // 那两个数的那两行)。四处都只把它当"数"印出来, 没有一处拿它去比门限。
        const double yardF = (fit.repeatPairCount > 0)
            ? sqrt((fit.repeatSigmaF[0] * fit.repeatSigmaF[0]
                  + fit.repeatSigmaF[1] * fit.repeatSigmaF[1]
                  + fit.repeatSigmaF[2] * fit.repeatSigmaF[2]) / 3.0) : 0.0;
        const double yardM = (fit.repeatPairCount > 0)
            ? sqrt((fit.repeatSigmaM[0] * fit.repeatSigmaM[0]
                  + fit.repeatSigmaM[1] * fit.repeatSigmaM[1]
                  + fit.repeatSigmaM[2] * fit.repeatSigmaM[2]) / 3.0) : 0.0;

        diagOut() << "------------------------------------------------------" << std::endl;
        // ⚠ 【这一行不许只看 fitOk】: 用了 MODEL_FORM_FORCE_ONLY 之后, fitOk 为真【不再意味着】
        //   力矩那一半也过了 —— 力矩没过时库把 momentFormChecked 压在 false。所以"通过"必须分两种,
        //   否则屏幕上会印一个不加限定的"通过", 而力矩那一半其实没过 (正是本项目最忌的那种
        //   "安静地不一致")。判据仍只在库里有唯一一份实现 —— 这里只是【引用】它的标志, 与下面
        //   那一支同一条规矩: 不复述判据, 只报状态。
        const bool momentHalfNotPassed = fitOk && (fit.lackOfFitMomentDof > 0)
                                              && !fit.momentFormChecked;
        diagEmitf("  模型形式检验 (fitRaw 的判决): %s\n",
                  !fitOk ? "【拒绝】"
                         : (momentHalfNotPassed
                                ? "通过 —— 但【仅力通道】: 力矩通道那一半没过"
                                  " (见 stderr 的 [Payload] 行)"
                                : "通过"));
        diagEmitf("      尺子状态 modelFormStatus = %s\n",
                  modelFormStatusName(fit.modelFormStatus));
        diagEmitf("      重复姿态对 (尺子的自由度, 每对须取不同姿态) = %d 对;  逐姿态噪声来自采集时的样本方差\n",
                  fit.repeatPairCount);
        if (fit.modelFormChecked) {
            // ⚠ 【本分支不复述判据, 只报数 —— 与下面那个 else 分支同一条规矩】。
            // 从前这两行印的是 "X  <  门限 Y"。那是本地替库下结论, 而且【力矩那一半根本没验时
            // 也照印】: 自由 12 参数模型秩亏时 (3n > 12, 见 force/PayloadCalibration.cpp 那一支)
            // 库把 lackOfFitMomentDof 置 0 并明说"力矩通道的失拟检验【没做成】", 而这一行照印
            // "0  <  0" —— 一个【假通过】, 还随正文 append 进 calib_report.md 这份永久记录。
            // 假通过比假失败更坏: 假失败把人支去查一个【没坏】的通道, 假通过让人【什么都不查】。
            // 判据只在库里有一份才不会各说各话, 所以这里只把数摆出来, 过没过以 stderr 的
            // [Payload] 行为准 (这条规矩的出处见下面 MODEL_FORM_NO_DOF 那个分支的注释)。
            diagEmitf("      力通道:   残差÷尺子 χ²/dof = %.4g  门限 %.4g   (dof=%d, 尺子 %.4g N)\n",
                      fit.chi2RepForceRatio, fit.chi2RepForceLimit, fit.chi2DofForce, yardF);
            // 【力矩那一半要问两个【不同】的问题, 所以这里是三分支 —— 别用一个标志把它们混了】
            //   · "验过【且通过】了吗"     -> 读库的标志 fit.momentFormChecked
            //   · "这个统计量【算出来了】吗" -> 读 fit.lackOfFitMomentDof (> 0)
            //   · 只读 momentFormChecked 会把"【验了、没过】"误读成"【没做】" —— 那两种情形
            //     在它上面都是 false。
            // 这三分支是 2026-09-20 引入 MODEL_FORM_FORCE_ONLY 时才需要的: 在那之前"力矩没过"
            //   会在库里直接 return false, 根本走不到本分支 —— 那时"两个标志在那个意义上同进同退"
            //   是对的; 有了 FORCE_ONLY, "【算出来且没过】、但整体放行"成了一个**可达的组合**。
            // ⚠ 库那边的 momentFormChecked 同一时刻也收紧了 (扣掉了 momentFailed), 所以它自己的
            //   语义仍是"做了【且通过】" —— 与本行对它的用法一致, 没有各说各话。
            // ⚠ 下游那个【被拒】分支 (本函数更下面那一处) 早就是按 lackOfFitMomentDof > 0 分的,
            //   理由与本行同源; 两处现在同一条口径。
            if (fit.momentFormChecked) {
                diagEmitf("      力矩通道: 失拟统计量     = %.4g  门限 %.4g   (dof=%d, 尺子 %.4g N·m)\n",
                          fit.lackOfFitMomentRatio, fit.lackOfFitMomentLimit,
                          fit.lackOfFitMomentDof, yardM);
            } else if (fit.lackOfFitMomentDof > 0) {
                // 【2026-09-20 新增的第三种情形 —— 在此之前它不存在】
                // 力矩的失拟【算出来了, 而且没过】, 而整体判决仍然通过。只有 MODEL_FORM_FORCE_ONLY
                // 才可能出现这个组合: 旧策略下"力矩没过"会在库里直接 return false, 根本走不到本分支。
                // ⚠ 没有这一支, 下面那个 else 会把它印成【没有检验】(dof=0) —— 把"验了、没过"
                //   说成"没做", 而这份报告是**永久记录**。实测 2026-09-20 21:02 就印错了一次:
                //   stderr 上写着 dof=6、失拟 153.1, 而报告里写着 dof=0、【没有检验】。
                //   这就是模型形式策略那条改动【自己带进来】的假话, 这一支是它的补丁。
                // ⚠ 《不分两种印法》的理由: 判据在库里只有一份 —— 这一行只把【数】摆出来,
                //   说说它是"做了没过"不是"没做"; 过没过以 stderr 的 [Payload] 行为准。
                diagEmitf("      力矩通道: 失拟统计量     = %.4g  门限 %.4g   (dof=%d, 尺子 %.4g N·m)\n",
                          fit.lackOfFitMomentRatio, fit.lackOfFitMomentLimit,
                          fit.lackOfFitMomentDof, yardM);
                diagEmitf("                判读: 这一半【做了, 但没过】—— 整体仍放行, 因为本次用的是"
                          " MODEL_FORM_FORCE_ONLY (力通道过即可)。\n");
                diagEmitf("                      别把它读成「没有检验」—— 那是另一件事; 库已在"
                          " stderr 的 [Payload] 行上说明。\n");
            } else {
                // dof = 0 是【这一次没验】, 不是"验了得 0"。这里【不许】印比较 —— 0 < 0 是假的,
                // 而它会被读成"力矩通道也过了"。
                // 【两个标志正是在本分支处分道扬镳】, 走得到这里就是因为它们不同进同退: 库在通过
                // 那一支无条件把 modelFormChecked 置 true, 而 momentFormChecked 按 dof > 0 置
                // (PayloadCalibration.cpp:1291-1292) —— 所以读到本行时前者是 true、后者是 false。
                // 这个分岔正是这个 else 存在的理由 (从前这里把因果写反了, 说"两个标志同进同退")。
                diagEmitf("      力矩通道: 【没有检验】(dof=0) —— 自由 12 参数模型在这批姿态上"
                          "秩亏, 失拟统计量【无从给出】;\n");
                diagEmitf("                这一半不是【通过】, 是【没做】—— 库已在 stderr 的"
                          " [Payload] 行上说明。\n");
            }
            diagEmitf("      对照 (姿态内噪声, 【只报告不判】): 力 %.4g N / 力矩 %.4g N·m;"
                      " χ²/dof = %.4g / %.4g\n",
                      fit.noiseForceN, fit.noiseMomentNm, fit.chi2ForceRatio, fit.chi2MomentRatio);
            // 【按"带没带数"锚定, 不按"上面第几行"锚定】: 本行与那两行之间还夹着判决【对照】
            // 那一行 (它也是"数"); 而力矩通道 dof=0 时, 那两行里的第二行印的是【没有检验】——
            // 一句【一个数都没有】的话, 把它叫"只是【数】"是说不通的 (2026-09-19 复审)。
            // 所以这里说的是"凡带着判据里的数的那几行": dof>0 与 dof=0 两支下都成立, 也不会
            // 因为将来在中间再插一行而指错。
            diagOut() << "      ↑ 上面【凡带着判据里的数的那几行】只是【数】, 不是【结论】 ——"
                      << " 过没过【以 stderr 的 [Payload] 行为准】: 判据在库里只有一份, 本行不替"
                      << "它说。" << std::endl;
            diagOut() << "        (力矩通道 dof=0 时上面【没有】带数的那一行, 只有【没有检验】那句"
                      << " —— 它同样不是结论。)" << std::endl;
            diagOut() << "      ↑ 残差若明显大于姿态内噪声、却与【姿态间复现性】相符, 那是"
                      << "采集现场的复现性差, 不是模型错。" << std::endl;
        } else {
            // 拒绝时必须说清【是哪一种】—— "没验过"与"验了没过"是两回事, 处置也完全不同。
            if (fit.modelFormStatus != PayloadCalibration::MODEL_FORM_OK) {
                // ⚠ 这一行解释的是 modelFormStatus, 而它【不一定是最先卡住的那一步】:
                // 这个状态由 fitRawLinear 判定并赋值, 而 fitRawLinear 是 fitRaw 的【第一步】
                // (@1066) —— 所以 fitRaw 后面那几道【非模型形式的】自检 (A 奇异 @1069 /
                // cond @1079 / 质量尺度 @1095) 拒绝时, 这里读到的仍是 fitRawLinear 当时判的
                // 那个值, 通常就是 MODEL_FORM_OK —— 那种情形【进不了本分支】, 走的是下面的
                // else。读到的状态与"真正卡住的那一步"不符的其余情形, 是尺子一侧的判决先
                // 落了地 (DEAD_CHANNEL / NOISE_HOLES / NO_REPEAT), 而不是这里的默认值。
                // 无论哪一种, 真因都已由库打到 stderr 的 [Payload] 那一行上。这里【不复述库
                // 里的常量】(复制一份判据就会与库各说各话), 只把人指向那一行。
                diagOut() << "      (若上面有 [Payload] 自检拒绝行, 【以那一行为准】—— 本行解释的"
                          << "只是 modelFormStatus。)" << std::endl;
                diagEmitf("      → 拒因: 【尺子不齐】(%s) —— 模型形式【没有被检验】, 所以不给参数。",
                          modelFormStatusName(fit.modelFormStatus));
                diagOut() << std::endl;
                diagOut() << "        逐种处置: 缺重复对 -> 摆好姿态按 SPACE, 保持不动按 'r' 再"
                          << "按 SPACE; 缺噪声 -> 采样笔数"
                          << "太少; 通道冻住/有洞 -> 查传感器读数;" << std::endl;
                diagOut() << "        自由度不足 -> 多摆几个姿态 (力通道 12 个未知, 3n−12 要 > 0)。"
                          << std::endl;
            } else {
                // ⚠ 【本分支不复述判据】—— 这条规矩是同一函数里上面那个分支 (MODEL_FORM_NO_DOF)
                // 自己写下的: 判据只有在库里有一份才不会各说各话, 这里只把人指向那一行。
                // 从前这一行违反了它, 而且【无条件地】断言 "力通道 χ²/dof = X 超过门限 Y":
                // 力通道真超线时它是对的, 而拒的是力矩那一半时它就是【假的】—— 实机
                // 2026-09-19 18:49:55 那次力通道 0.6508 < 门限 2.869 (它【过了】), 拒因是力矩
                // (27.23 > 4.909); 库在同一屏的 stderr 上说对了, 而这一行把操作员支去查一个
                // 【没坏】的通道。同一块里两句话互相打架, 比少一句更坏。
                diagEmitf("      → 拒因: 尺子齐备 (modelFormStatus = %s) —— 拒绝【不是尺子不齐"
                          "造成的】; 但是哪一道自检卡住的,【以上面 stderr 的 [Payload] 自检"
                          "拒绝行为准】。\n",
                          modelFormStatusName(fit.modelFormStatus));
                // 【与通过那一支同一处境, 就说同一句话】(2026-09-19 复审: 同一件事在两支下
                // 曾读出两个样子)。dof = 0 是【没验】, 不是"验了得 0" —— 印 "失拟 = 0 门限 0"
                // 与通过那一支从前的 "0 < 0" 是同一个读法陷阱 ("0, 那力矩通道没事"), 只是那一支
                // 更危险 (假通过)。锚点同样改成"带没带数", 不数"下面第几行"。
                diagOut() << "        下面【凡带着判据里的数的那几行】只是把数照抄在旁边 (写出来的"
                          << "是【门限】而不是结论):" << std::endl;
                diagEmitf("          力通道:   残差÷尺子 χ²/dof = %.4g  门限 %.4g   (dof=%d, 残差 %.4g N"
                          " / 尺子 %.4g N)\n",
                          fit.chi2RepForceRatio, fit.chi2RepForceLimit, fit.chi2DofForce,
                          fit.rmsForceN, yardF);
                // 力矩这一行【不能】像通过那一支那样读 momentFormChecked: 判决被拒时它按构造
                // 必为 false (库只在通过那一支才置 true, PayloadCalibration.cpp:1291-1292),
                // 拿它分会把"失拟真的算出来且正是拒因"也读成"没做"。这里问的是【有没有这个数】,
                // 所以按 dof > 0 分。
                if (fit.lackOfFitMomentDof > 0) {
                    diagEmitf("          力矩通道: 失拟             = %.4g  门限 %.4g   (dof=%d,"
                              " 尺子 %.4g N·m)\n",
                              fit.lackOfFitMomentRatio, fit.lackOfFitMomentLimit,
                              fit.lackOfFitMomentDof, yardM);
                } else {
                    diagEmitf("          力矩通道: 【没有检验】(dof=0) —— 自由 12 参数模型在这批姿态"
                              "上秩亏, 失拟统计量【无从给出】;\n");
                    diagEmitf("                    这一半不是【通过】, 是【没做】—— 库已在 stderr 的"
                              " [Payload] 行上说明。\n");
                }
                diagOut() << "        本行不给【超过/通过】这个结论 —— 是哪一种, 按上面 [Payload] 行分:"
                          << std::endl;
                diagOut() << "          · 力通道被拒 -> 先看上面 stderr 的逐姿态残差表: 【只有一两个"
                          << "姿态高】-> 重采那几个; 个个都高 -> 模型形式错。" << std::endl;
                diagOut() << "          · 力矩通道失拟被拒 -> c_s × (A·g) 这个叉乘结构不成立 (自由模型"
                          << "显著解释得更好), 重采个别姿态救不回来。" << std::endl;
                // ⚠ 这一条从前写的是 "(你自己把这两个数与门限比一比, 都没越线)" —— 又一个
                // 【本地替库下结论】: 走到这一支时模型形式那一块【可能压根没跑过】(cond / 质量
                // 尺度 / A 奇异那几道自检排在它前面, 见 fitRaw 的次序), 那两个数就是【没比过】
                // 的; "都没越线"于是可以是一句假通过。而且它与本分支上面自己那句 "本行不给
                // 【超过/通过】这个结论" 互相打架。这里只留【结构】: 上面没有模型形式的自检拒绝
                // 行 -> 拒绝来自后面那几道自检, 是哪一道见 [Payload] 行。
                diagOut() << "          · 上面【没有】模型形式的自检拒绝行 -> 拒绝来自 fitRaw 的其它自检:"
                          << " 质量尺度越界 / cond 过大 / A 奇异(秩亏) —— 同样见 [Payload] 行。"
                          << std::endl;
            }
        }
        if (!fitOk) {
            diagOut() << "      注: 上面的 A / b_F / b_M / c_s 与由 A 分解出的 m / 奇异值 / parity / Q"
                      << " 仍是【线性解】——" << std::endl;
            diagOut() << "          被拒的是它【通不通得过自检】, 不是它没解出来。这些数照看,"
                      << " 但【不得】据此下任何结论;" << std::endl;
            diagOut() << "          线性层本身失败时 (姿态数不足 / 秩亏) 这些字段是全 0, 不携带信息。"
                      << std::endl;
        }

        // ===== 把这一份模型装进本地补偿, 并按 v3 落盘 =====
        //
        // 判决 (fitOk) 早在上面就定了; 到这里才动手, 是因为上面那一屏回答的是"解出来什么",
        // 这一段才回答"拿它做什么"。
        //
        // 【门槛就是 fitOk】: 与发送候选同源。判据只许有一份实现 —— 另立第二套"安装门槛",
        //   就得在别处解释它为什么与候选那道不一样, 而那正是本项目已经栽过的那种不一致。
        //
        // 【为什么先装后落】: setCalibration 会用 modelUsable 拒收不可用的 A (全零 / 退化 /
        //   非有限)。若它拒了却仍落盘, 那份文件就是"自称是标定结果、装载时又会被拒一次"的
        //   坏文件 —— 而 loadFromFile 的返回值【分不开】"文件不存在"与"模型不可用", 下次
        //   启动只会看到一句含糊的失败。所以落盘门控在"确实装上了"之后, 文件永远可装载。
        //
        // 【这一步动的是本地补偿, 不是机械臂】: 它改的是"我们这边怎么理解传感器的读数",
        //   不发 EnableRobot / PayLoad / LoadSwitch 里的任何一条, 机械臂不会因此动一下。
        //   (与"下发给机械臂"是两件事: 后者要的是【法兰系】上整条链, 换算未定, 仍等 Task 9。)
        //
        // 【setCalibration 返回 void】: 它拒收时既不返回值也不抛异常, 只在 stderr 上说一句。
        //   所以"装没装上"只能事后拿 isCalibrated() 问一次 —— 这是那个 API 的实际形态。
        bool installed = false;
        bool saved = false;
        if (fitOk) {
            ForceCompensation::setCalibration(fit.A, fit.bF, fit.bM, fit.cS);
            installed = ForceCompensation::isCalibrated();
            if (installed) {
                saved = ForceCalibration::saveToFile(CalibStore::fileFor("force_calib.json"),
                                                     fit.A, fit.bF, fit.bM, fit.cS);
            }
        }

        // ===== 本次【对本地做了什么 / 没做什么】 =====
        diagOut() << "------------------------------------------------------" << std::endl;
        diagOut() << "  ★ 以上全部是【传感器测量原点以下】的量 —— 不是整条工具链。" << std::endl;
        diagOut() << "    m / A / c_s 描述的是传感器【测量原点向下】那一段负载 (传感器内部质量分布"
                  << " + 笔夹 + 笔);" << std::endl;
        diagOut() << "    机械臂的负载模型 (EnableRobot 的 load/center) 描述的是【挂在它法兰上的"
                  << "整条链】。" << std::endl;
        diagOut() << "    两者原点不同, 换算要走过 c_s 的原点在哪 + 法兰→测量系那一步 ——"
                  << " 这一步【未定】" << std::endl;
        diagOut() << "    (spec §6b 末: 标定 c_s = 54.55 mm 与解析几何反推的 75.8 mm 对不上,"
                  << " 而传感器总高只有 31.5 mm)。" << std::endl;
        if (!fitOk) {
            diagOut() << "  ★ 本次【什么也没应用】: 求解【未通过】(见上面【模型形式检验】那一行,"
                      << " 与 stderr 上那条自检拒绝), 所以【没有】把它装进本地补偿, 也没有写"
                      << " force_calib.json。" << std::endl;
        } else if (!installed) {
            diagOut() << "  ★ 本次【没能装进本地补偿】: setCalibration 拒收了这份模型 (原因见"
                      << " stderr 上那条【拒绝安装】) —— 本地补偿【仍未启用】, 一致性闸门会"
                      << " 继续报\"没有可用模型\"。" << std::endl;
        } else if (!saved) {
            diagOut() << "  ★ 本次【已装进本地补偿, 但没能落盘】: 本会话生效。" << std::endl;
            diagOut() << "    ⚠ 落盘是【以写模式打开文件】的 ⇒ 打开那一刻, 盘上【旧的那一份就"
                      << "已经被截断了】。所以重启之后不是\"还是上一次标定的那一份\", 而是"
                      << "\"没有可用文件\"或\"文件被截断/改坏\"。要恢复只能重按 's'。" << std::endl;
        } else {
            diagOut() << "  ★ 本次【已装进本地补偿, 并已落盘】: 本会话生效; force_calib.json"
                      << " 已按 version 3 写入 (下次启动自动装载)。" << std::endl;
        }
        diagOut() << "    以上动的是【本地补偿】, 不是机械臂 —— 一个字节都没发给它。"
                  << "它现在用的仍是连接时序里 EnableRobot 带下去的那一份。" << std::endl;
        diagOut() << "    ⚠ 从这一刻起闸门【有模型可比了】—— 它不再是在比较之前就早退, 而是"
                  << "真的在比。" << std::endl;
        diagOut() << "      至于比出来是【放行】还是【一致性拒绝】, 取决于【机械臂那边】此刻的"
                  << "负载参数对不对 —— 而这一条本次【没有测过】, 别当成已知。" << std::endl;
        diagOut() << "    payload_calib.json 【仍然不写】; 下发路径的开通条件仍在 plan Task 9"
                  << " (那一步要的是法兰系的整条链, 换算未定)。" << std::endl;
        diagOut() << "    本地补偿不需要那个换算 —— 它全程在传感器系内闭环, 所以这一步等得起。"
                  << std::endl;

        // ===== Task 8a: 发送候选 —— 先摆出来、过两道闸、【此刻一个字节都不发】 =====
        //
        // 这里的职责只有三件: 取数 → 喂给纯函数 PayloadCalibration::evaluateSendGate
        // (两道闸的判决, 不依赖 socket, 单测覆盖) → 把两道闸的数与结论照实打出来。
        // 【真正发送的是另一个键 'p'】, 而且 'p' 【不重算】, 用的就是这里留下的那一份 ——
        // 否则"打印的"与"发出的"会不是同一个东西。
        if (!fitOk) {
            // 本次没解出东西 -> 候选作废。也【不许】把上一次求解留下的候选留在手里当存货:
            // 那是一份与这次按键无关的旧值, 而 'p' 打的是"现在这份候选"的旗号。
            s_sendCandidateValid = false;
            diagOut() << "\n[下发候选] 本次求解【未通过】—— 发送候选作废 (上一次求解留下的候选"
                      << "也一并清掉, 不保留存货)。按 'p' 会被拒: 没有候选。" << std::endl;
            diagOut() << std::endl;
        } else {
            // ===== Task 8a-2: 候选 = 【活路径本次实测出来的那一份】 =====
            //
            // ⚠ 前提复核 (2026-09-20): 上面新加的"装本地补偿"【没有】改变这一段的前提 ——
            //   装的是 ForceCompensation 的全量模型 (A / b_F / b_M / c_s), 而 effective()
            //   报的是 PayloadCalibration 的 (massKg, comMm), 即【机械臂侧】要用的那对参数。
            //   本任务一个字节都没碰后者, 所以 effective() 仍然是旧的 —— 候选仍必须从本次
            //   实测里拼 (下面就是那个拼装)。别因为"s 现在会装东西了"就以为 effective() 新了。
            //
            // 8a 这里取的是 PayloadCalibration::effective() —— 而那是【断的】: 让 effective()
            // 跟着本次求解结果变的 applyResult / save / setMassCom 三个调用【只存在于下面那个
            // #if 0 块里】(已停用的旧 ψ 模型), 所以活路径按 's' 求解成功后 effective() 仍是
            // payload_calib.json 的旧值 (= 机械臂【现在就有】的那一份, 发出等于没改) 或
            // Config 的种子值 (CAD 猜的, 本项目已判定不可用)。两条都不是"把标定结果发出去"。
            // ⇒ 8a-2 把它换成活路径手里【真正有的】那两个量, 且【不留回退】(留回退就是留一条
            //   发种子值的路, 而堵它正是本任务的目的)。
            //
            //   m = PayloadCalibration::Decomp::m   本次实测的质量尺度
            //   c = 机械臂【自报】的 @1176 CenterX/Y/Z (不是算出来的)
            //
            // ⚠ 换帧的事实 (brief §2, 措辞不许美化): 上面那个 m 是【传感器测量原点以下】那一截
            //   的质量, 而 EnableRobot 的 load 槽位要的是【挂在法兰上的整条链】。两者差了传感器
            //   机器人侧那一段, 而【量未定】(Task 9 只为 Z 分量解决了换帧), 所以本次是把一个
            //   "测量原点以下"的量代入"整条链"的槽位。措辞由 formatSendCandidateMassText 统一
            //   给出 (只此一份), 这里不自己写一遍。
            //
            // ⚠ 为什么 cz 取自机械臂自报而不是算出来的: 用户 2026-09-20 选的是【最小改动的
            //   第一步】—— 本次【只改 m, c 不动】。代价用户已知并接受: cz 那一支本次【没有被
            //   纠正】; 若残余主要来自方向, 8b 可能看不出变化。不许把这一步说成"cz 已标定"。
            bool echoOk = false;
            double echoLoadKg = 0.0;
            double echoCenter[3] = {0.0, 0.0, 0.0};
            EnterCriticalSection(&appState.forceDataMutex);
            echoOk = appState.forceData.payloadEchoValid;
            echoLoadKg = appState.forceData.payloadEchoLoadKg;
            for (int i = 0; i < 3; i++) echoCenter[i] = appState.forceData.payloadEchoCenterMm[i];
            LeaveCriticalSection(&appState.forceDataMutex);

            // c_s 可不可用的口径与块尾那一节【同一个】: decompOk —— 线性层就没解出来时
            // fit.cS 是全 0, 不携带信息 (全 0 会被读成"质心就在测量原点", 那是另一个意思)。
            const double csZmm = fit.cS[2] * 1000.0;

            // 候选的构造与判决都在纯函数里 (单测覆盖): "有没有候选" (来源/回读) 与
            // "候选能不能发" (来源判据 + 两道闸) 是两层, 各有各的判决值。
            //   · decompOk == false -> 没有实测质量尺度 -> 【无候选】(不是"候选 = 0")
            //   · echoOk   == false -> 没回读到 @1168/@1176 -> 【无候选】
            //     (【不许】退回本客户端自己下发的值: 那条路带着 centerZ 折叠歧义, 约差 125 mm)
            // ⚠ 照实说: 走到这个 else 时 fitOk 已为真, 而 fitRaw 自己就调 decompose ⇒ decompOk
            //   【必然】为真, 所以"没有实测质量尺度"这一支在生产路径上【当前不可达】。它留着是
            //   防御性的 (把"退回种子值"这条路在【调用点 + 判决层】两层堵死), 不是"现在会发生
            //   的事"。
            // ⚠ 【不许说成"类型层"】(二次复审 Minor 2): measuredMassKg 是 const double*, 它
            //   【不携带来源】—— 任何调用方传个 double* 进来, buildSendCandidate 都会把来源
            //   硬写成 MASS_SOURCE_MEASURED。所以守这条不变式的是两处: 下面这个调用点
            //   (decompOk ? &d.m : nullptr) 与 evaluateSendGate 里那道判决。
            //   真正可达的"无候选"是下面 echoOk == false 那一支。
            const PayloadCalibration::SendCandidate cand = PayloadCalibration::buildSendCandidate(
                decompOk ? &d.m : nullptr, decompOk ? &csZmm : nullptr,
                echoOk ? echoCenter : nullptr);

            if (!cand.present) {
                s_sendCandidateValid = false;
                // 归因走 switch, 【不是】二选一的 ?: —— 从前那是 "== CAND_NO_MEASURED_MASS
                // ? 这句 : 那句", 今天只有两种归因所以是对的, 但加第三种时它会被【静默】
                // 标成"没有回读到负载", 而这两件事的处置完全不同 (去查标定为什么没跑 vs
                // 去查 30004 回读)。default 明写"未知", 不落进任何一支的措辞。
                const char* absentWhy = nullptr;
                switch (cand.absent) {
                case PayloadCalibration::CAND_NO_MEASURED_MASS:
                    absentWhy = "没有【本次实测】的质量尺度 —— 只有实测出来的 m 才能当候选,"
                                " 种子值 / 上次落盘值一律不发 (那正是连接时序已经在发的那个)";
                    break;
                case PayloadCalibration::CAND_NO_PAYLOAD_ECHO:
                    absentWhy = "没有回读到机械臂自报的负载 (@1168 Load / @1176 CenterX/Y/Z)"
                                " —— 不退回本客户端自己下发的值当参照";
                    break;
                case PayloadCalibration::CAND_PRESENT:
                default:
                    absentWhy = "归因未知 (不该发生: present == false 时 absent 必是上面两种之一)";
                    break;
                }
                diagOut() << "\n[下发候选] 本次【没有候选】: " << absentWhy
                          << "。按 'p' 会被拒。" << std::endl;
                diagOut() << std::endl;
            } else {
                s_sendCandidate = cand.gate;
                s_sendCandidateValid = true;

                const double candMag = cand.gate.comMagMm;
                diagOut() << "\n======================================================" << std::endl;
                diagOut() << "  下发候选 (【尚未发送】—— 按 'p' 才发, 且必须先过下面两道闸)"
                          << std::endl;
                diagOut() << "======================================================" << std::endl;
                // m 那一行【连同换帧说明】由库里那一个函数给出 —— 措辞只此一份, 这里不重写
                // (它的三条硬要求见 PayloadCalibration.h 的 formatSendCandidateMassText)。
                {
                    // 1024 而非 512: 这段文本实测约 465 字节 (格式串本身量出来的), 而
                    // snprintf 【静默截断】—— 512 只剩 ~47 字节的余量, 改一次措辞就会砍在
                    // 句子中间 (砍掉的正是"量未定"那句解释, 而不是判决)。
                    char massLine[1024];
                    PayloadCalibration::formatSendCandidateMassText(cand.massKg, massLine,
                                                                    sizeof(massLine));
                    diagOut() << massLine;
                }
                // 候选的 c 打的是【闸1 定过号之后】的那一份 —— 也就是 'p' 真会发出去的那一份。
                // (8a 复审 Minor 7: 从前这里打的是闸前的 com, 屏幕上可能与发出去的不是一个东西。)
                // ⚠ 但"定过号"只在闸1 【真的定下了号】(convention != 0) 时才成立: SEND_SIGN_AMBIGUOUS /
                //   SEND_SIGN_NONE_IN_RANGE 在 convention / czSign / comMm[2] 赋值【之前】就返回了,
                //   那时 comMm[2] 还是机械臂自报的原样 (这一支在有候选 present == true 时【可达】)。
                //   标签若写死"已按闸1 的号定", 屏幕上就同时出现"闸1 无法判定"与"号已定"两句
                //   互相打架的话 (二次复审 Minor 1)。所以标签跟着判决走, 两种情形各说各的。
                //   那句话本身【不在这里写第二遍】—— 由库里唯一的一份给出 (单测钉住措辞)。
                {
                    char centerLabel[64];
                    PayloadCalibration::formatSendCandidateCenterLabel(cand.gate, centerLabel,
                                                                       sizeof(centerLabel));
                    diagEmitf("  候选 (cx, cy, cz)       = (%.1f, %.1f, %.1f) mm"
                              "   [机械臂自报 @1176 的 CenterX/Y/Z, %s]\n",
                              cand.comMm[0], cand.comMm[1], cand.comMm[2], centerLabel);
                }
                diagEmitf("  候选 |c|                = %.1f mm\n", candMag);
                // 闸 1: 两支 d 与判读。⛔ 这里【只报数, 不给勾/叉】(与块尾那一节同一条规矩) ——
                // "恰好一支在内"才是放行, 给单一勾会让人以为"这一支通过了"。
                //
                // ⚠【本次配置下闸1 的局限, 不许读成"cz 是对的"】(brief §3.3): 本次的 cz 就是
                //   取自 cz_robot 本身, 所以 d = cz_robot − c_s_z 这条闸此时【只能】做两件事:
                //   (a) 给 cz 定符号约定; (b) 挡住明显不自洽的 c_s。它【抓不出 68.7 本身是错的】——
                //   那正是 Task 9 记下的"那是一次一致性检验, 不是证明"。
                diagEmitf("  闸1 符号约定 (d = cz_robot − c_s_z, 判据 %.1f < d < %.1f mm):\n",
                          SessionReport::PAYLOAD_D_MIN_MM, SessionReport::PAYLOAD_D_MAX_MM);
                // 条件 = "上面那两支 d 真的算过" (见 evaluateSendGate 头文件里那张次序表):
                // 数据齐 (decompOk && echoOk) 【且】判决没有停在闸1 之前。SEND_NOT_MEASURED 那
                // 一支在生产路径上不可达 (见上), 但这里照写 —— 结构体里的 0 是【没有算过】,
                // 拿它当 d 打出去正是本项目反复记过的"假数字"。
                if (decompOk && echoOk
                    && cand.gate.verdict != PayloadCalibration::SEND_NOT_MEASURED) {
                    diagEmitf("      约定一 c_s_z = %+.3f mm: d = %.3f mm  %s\n",
                              csZmm, cand.gate.dSameDir,
                              cand.gate.dSameIn ? "【在范围内】" : "【在范围外】");
                    diagEmitf("      约定二 c_s_z = %+.3f mm: d = %.3f mm  %s\n",
                              -csZmm, cand.gate.dFlipDir,
                              cand.gate.dFlipIn ? "【在范围内】" : "【在范围外】");
                    if (cand.gate.convention != 0) {
                        diagEmitf("      → 选中的符号约定 = 约定%d, cz 定为 %+.1f mm"
                                  " (候选的 x/y 不动)\n",
                                  cand.gate.convention, cand.comMm[2]);
                    }
                    diagOut() << "      ⚠ 本次的 cz 【就是】机械臂自报的那个 cz_robot (@1176"
                                 " CenterZ, 见下面【与机械臂当前值相比】那一段), 所以这条闸\n"
                              << "        给不出关于它本身对错的任何信息 —— 它能定的只是【符号约定】,"
                                 " 挡的是【明显不自洽的 c_s】。\n"
                              << "        别把【闸1 过了】读成【cz 是对的】。"
                              << std::endl;
                } else if (!decompOk || !echoOk) {
                    // 【不许拿 0 顶上】: c_s 没解出来时 fit.cS 是全 0, 照着打会印出 "d = 68.700" ——
                    // 一个从"没有数据"算出来的、看着像真数的东西。这里的口径与块尾那一节一致:
                    // 缺数据就说"不可用", 这是【没有】, 不是 0。
                    diagEmitf("      【无法判定】: %s%s —— 两支 d 都无从给出 (不是 0, 是【没有】)\n",
                              decompOk ? "" : "没有 c_s (本次没解出); ",
                              echoOk ? "" : "没有 cz_robot (@1176 还没回读到负载)");
                } else {
                    // 数据齐但判决停在闸1 之前 (只剩"来源不是实测"那一支; 生产路径上不可达,
                    // 见上面的说明) —— 同样【不是 0, 是【没有】】。
                    diagEmitf("      【无法判定】: 候选无效 (质量尺度不是本次实测的) ——"
                              " 两支 d 都无从给出 (不是 0, 是【没有】)\n");
                }
                // 闸 2: 两个量与判读。阈值一律取库里那两个常量, 不在这里写第二遍。
                // |c| 是【严格小于】500 (操作单 §6 闸2 的原文, 用户 2026-09-20 裁定)。
                diagEmitf("  闸2 量级 (m ∈ [%.1f, %.1f] kg, |c| < %.0f mm):\n",
                          PayloadCalibration::SEND_GATE_MASS_MIN_KG,
                          PayloadCalibration::SEND_GATE_MASS_MAX_KG,
                          PayloadCalibration::SEND_GATE_COM_MAX_MM);
                diagEmitf("      m = %.4f kg  %s\n", cand.gate.massKg,
                          cand.gate.massOk ? "【在范围内】" : "【超出范围】");
                diagEmitf("      |c| = %.1f mm  %s\n", candMag,
                          cand.gate.comOk ? "【在范围内】" : "【超出范围】");

                // ===== §3.5: 与机械臂【当前值】相比, 这一次到底改了什么 =====
                // 这一段【必须在按 'p' 之前】看得到 —— 它就是给操作员看的那一眼。
                const PayloadCalibration::SendCandidateDiff df =
                    PayloadCalibration::diffSendCandidate(cand, echoLoadKg, echoCenter);
                // 数值走 diagEmitf 的显式格式 (不用 ostream 的默认 6 位有效数字): 这一段是
                // 操作员据以决定"按不按 'p'"的那一眼, 精度不能靠默认值。
                diagEmitf("  与机械臂【当前值】相比 (@1168 Load = %.4f kg, @1176 CenterX/Y/Z = "
                          "(%.1f, %.1f, %.1f) mm):\n",
                          df.echoLoadKg, df.echoCenterMm[0], df.echoCenterMm[1], df.echoCenterMm[2]);
                diagEmitf("      m : %.4f → %.4f kg   (差 %+.4f)\n",
                          df.echoLoadKg, df.candMassKg, df.dm);
                for (int i = 0; i < 3; i++) {
                    const char* axis = (i == 0) ? "cx" : (i == 1) ? "cy" : "cz";
                    diagEmitf("      %s: %+.1f → %+.1f mm   (差 %+.1f)\n",
                              axis, df.echoCenterMm[i], df.candCenterMm[i], df.dc[i]);
                }
                {
                    char concl[512];
                    PayloadCalibration::formatSendCandidateDiffConclusion(df, concl,
                                                                         sizeof(concl));
                    diagEmitf("      结论: %s\n", concl);
                }

                // 结论行: 放行 / 【是哪一闸、为什么】。每一支各说各的 —— 合并成一句"不合格"就
                // 等于把"没数据"、"来路不对"、"数据在但定不了号"混成一个, 而这几件事的处置
                // 完全不同。语句本身与 'p' 被拒时说的那一句【同源】(formatSendGateConclusion)。
                // 512 而非 256: 这段文本最长的一支 (SEND_NOT_MEASURED) 实测约 219 字节,
                // 而 snprintf 【静默截断】—— 256 的余量只有 ~37 字节, 改一次措辞就会砍在
                // 句子中间 (砍掉的是"为什么"那句解释, 而判决本身在别处)。
                char reason[512];
                formatSendGateConclusion(cand.gate, reason, sizeof(reason));
                diagEmitf("  结论: %s\n", reason);
                diagOut() << "  ⚠ 此刻【什么都没有发出去】—— 上面只是候选与闸的判读;"
                          << " 真发送是另一个动作 (按 'p'), 且 'p' 用的是这一份候选, 不重算。"
                          << std::endl;
                diagOut() << std::endl;
            }
        }

        // 这批数据【仍然有效】, 所以【不】动 dataUnderCurrentPayload。
        // 【为什么这次改了本地补偿、数据却仍然可比】: 那面旗子看的是【机械臂侧】的负载 ——
        //   report()/复验 读的是 bias[][] = 各姿态平均的 raw 力/力矩, 来源是 @576
        //   (ActualTCPForce, 机械臂【自己】补偿过的回显), 不是本地补偿的输出。本次装的是
        //   本地模型 (compensated 那一路), 机械臂的负载一个字节没改 ⇒ @576 照旧 ⇒ 数据照旧可比。
        // ⚠ 已知缺口 (本次【不】修, 记在这里): 活路径里现在【没有任何地方】把旗子置 false ——
        //   它唯一的写者躺在 #if 0 里。于是真改了机械臂负载之后 ('p'+'y' 生效之后) 复验仍会
        //   放行, 而那时数据确实不再可比。这条要单独处理, 别顺手在这里糊上。
        // 复验按哪个 'm' 要说清: 【在 'm' 模式里】再按一次 = 退出模式并重出报告 (数据不丢);
        // 从模式外按 'm' = 重新开始采集 (走 reset(), 这批数据丢弃)。两者是同一个键、相反的结果。
        diagOut() << "  → 复验: 【就在 'm' 模式里】再按一次 'm' 即退出并重出报告 (已采数据不作废"
                  << " —— 本次改的是【本地补偿】, 而报告读的 @576 是机械臂侧, 不受它影响);\n"
                  << "     从模式外按 'm' 是【重开采集】—— 那会丢弃这批数据。\n"
                  << "     注意: 那份报告 (report) 报的是【旧模型/当前生效配置】的量"
                  << " (当前负载 / CZ符号), 不是上面这一屏解出的东西。" << std::endl;
        diagOut() << std::endl;

        // 连续失败计数照记、照报 (它本身有信息量: "这台设备此刻解不出可用的东西"), 但
        // 【不】再据此中止任何东西 —— 见 solveAndApply 顶上。solveLocked 只作为这个状态的
        // 标记被写下来 (真正的上锁语义属于已停用的应用路径), 读它的地方只有顶上那一句提示。
        if (!fitOk) {
            consecutiveFails++;
            if (consecutiveFails >= Config::CALIB_MAX_CONSECUTIVE_FAILS) {
                solveLocked = true;
                diagOut() << "  [BIAS] 已连续 " << consecutiveFails << " 次被拒 (>= "
                          << Config::CALIB_MAX_CONSECUTIVE_FAILS << ") —— 按 'm' 重新采集"
                          << " (计数清零)。本次诊断不因它少打一行。" << std::endl << std::endl;
            }
        } else {
            consecutiveFails = 0;
        }

        // outcome 的前缀要如实说【这一行代表什么被应用了】:
        //   PRINTED_ONLY            求解未通过 ⇒ 确实什么都没装
        //   INSTALLED               已装进本地补偿 (并已落盘)
        //   INSTALL_REJECTED        判决通过, 但 setCalibration 拒收了 —— 理论上不该发生;
        //                           真发生就必须有人看见, 所以给它一个自己的前缀
        //   INSTALLED save_failed   装上了, 但 force_calib.json 没写成 ⇒ 重启即丢
        // ⚠ 从前这里恒是 PRINTED_ONLY —— 那时确实一个字节都没写, 所以那是真话。
        //   装进本地补偿之后, 无条件写 PRINTED_ONLY 就变成假话。
        //   (列格式一个字没动, 变的只是 outcome 这一列的内容。)
        const char* what = !fitOk ? "PRINTED_ONLY"
                         : (!installed ? "INSTALL_REJECTED"
                         : (!saved ? "INSTALLED save_failed" : "INSTALLED"));
        char outcome[160];
        snprintf(outcome, sizeof(outcome), "%s %s modelform=%s rmsF=%.4f",
                 what, fitOk ? "fit_ok" : "fit_rejected",
                 modelFormStatusName(fit.modelFormStatus), fit.rmsForceN);
        logCalibAttempt(outcome, &fit, count);

        // ===== 落文档块 (本次按 's' 的出口) =====
        // 【在这里, 不在判决之前】: 正文攒的是上面整屏的字节, 判决 (fitOk) 与它的逐条说明都在
        // 正文里; 块尾那一节 (机械臂自报负载 / d = cz_robot − c_s / (0, 31.5) mm 判据) 只是
        // 【信息】, 它改不了任何判决 (brief 硬要求 6)。
        // 被拒的那几次同样走到这里 —— 那正是"为什么被拒"最需要留在文档里的场合。
        // fitOk 一并传下去【只用来限定尾节 d 那一行的措辞】: 被拒的那一次块里几行之前才印着
        // "模型形式检验: 【拒绝】", 尾节不能再无条件地说"测量原点落在传感器体内" —— 那是一个
        // 自描述块里两个互相打架的结论。判决、阈值、模型一个字节没动。
        diagFinish(diagPayloadSection(decompOk, fit.cS, d.parity, fitOk));

#if 0  // ================= 旧模型 (psi 扫描 + 残余量 dm/dp) —— 已停用 =================
       // 【保留不删, 待 Task 11 统一移除】。停用理由 (两行版的 spec §1): 这个模型预设了
       // "传感器安装 = 绕 z 的纯偏航 psi" + 边界上手工对齐的 z 符号, 而 2026-09-19 的实机
       // 数据把这两个预设都证伪了 —— 它只好把质量符号翻过去 (解出 dm = -0.417 kg, 被非物理
       // 门限拒掉); 更要命的是【它安静地错】: 形式错 0.018 N, 标量残差报不出来。
       // 新模型 (上面一段) 把 A 放开成自由 3×3, 这两个预设都不需要了。
       //
       // 注意: 这一段里的 logCalibAttempt 调用已按新签名改成 nullptr (不再传 Result) ——
       // 重新启用时要把旧模型那几列接回去, 否则日志里只会留下一排 "-"。
       //
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
            logCalibAttempt(outcome, nullptr, count);
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
            logCalibAttempt(outcome, nullptr, count);
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
        logCalibAttempt(outcome, nullptr, count);
#endif  // ============== 旧模型 (psi 扫描 + 残余量 dm/dp) —— 已停用 ==============
    }

    // ===== Task 8a: 发送键 'p' =====
    //
    // 全程序【唯一】会把负载参数发给机械臂的入口 —— 交付物 1 的 RelayCore::sendPayloadToRobot
    // 只被这里调用 (约束见那个函数顶上的注释)。
    //
    // 【不重算】: 用的是 's' 求解成功时留下的那一份候选 (s_sendCandidate)。重算就会隔着两次
    // 按键与两次实时读数 —— 屏幕上说 A 而发出去 B, 正是本项目最忌讳的"安静地不一致"。
    //
    // 【'p' 自己不发送】(I3): 'p' 只摆出安全规程 + 本次到底发什么, 然后等确认键。
    // 见 s_awaitingSendConfirm 那一段的说明。
    static const char SEND_CONFIRM_KEY = 'y';   // 确认键 —— 提示文字里也用它 (只此一处定)

    static void sendCandidate() {
        // ① 没有任何候选 (还没按过 's', 或求解未通过 / 采集已重开把候选作废了)。
        if (!s_sendCandidateValid) {
            std::cout << "[下发] 没有候选 —— 先按 'm' 采姿态, 再按 's' 求解"
                      << " (求解成功才会留下候选)。" << std::endl;
            std::cout << "[下发] 本次【什么都没有发出去】。" << std::endl;
            return;
        }
        // ② 候选存在但过不了闸 —— 说清是哪一闸、为什么 (与 's' 那一屏上那一行同源),
        //    并【什么都不发】。⚠ 拒发必须在【确认提示之前】: 提示一旦出现, 确认键就会落到
        //    一件本来就不会发生的事上。
        if (s_sendCandidate.verdict != PayloadCalibration::SEND_OK) {
            char reason[512];
            formatSendGateConclusion(s_sendCandidate, reason, sizeof(reason));
            std::cout << "[下发] " << reason << std::endl;
            std::cout << "[下发] 本次【什么都没有发出去】。" << std::endl;
            return;
        }
        // ③ 可发送 —— 【先打安全规程与这一次要发的数, 再等确认】。顺序不能反: 这几行是给站在
        //    机械臂旁边的人看的, 而发送一旦开始就不再受这里控制。
        std::cout << "[下发] ⚠ 即将向机械臂下发负载参数：" << std::endl;
        std::cout << "[下发]    · 机械臂在安全姿态" << std::endl;
        std::cout << "[下发]    · 手离开工作空间" << std::endl;
        std::cout << "[下发]    · 急停在手边" << std::endl;
        std::cout << "[下发]    · 一次改到目标值（不要逐步逼近）" << std::endl;
        std::cout << "[下发] ⚠ 运行中改负载会让机械臂动 —— 2026-09-19 实机证实"
                  << "（1.5 kg 那次撞向关节限位）" << std::endl;
        std::cout << "[下发] 本次下发: EnableRobot(m, cx, cy, cz) + LoadSwitch(1) —— "
                  << "顺序取自设计 §6b 的清单, 文档未说明其必要性" << std::endl;
        // 把这一次要发的东西【逐字】摆出来 —— 两条文本都取自 RelayCore 的那两个 formatter
        // (formatPayloadEnableCommand / payloadLoadSwitchCommand), 也就是 sendPayloadCommands
        // 要写进 socket 的那两条。从前这里是第二处拼法 (格式串 + LoadSwitch(1) 各重写一遍),
        // 发送侧一改, 屏幕就会让人确认【另一条】命令 —— 而确认的正是要发给实机的东西。
        {
            char willSend[192];
            RelayCore::formatPayloadEnableCommand(s_sendCandidate.massKg, s_sendCandidate.comMm,
                                                  willSend, sizeof(willSend));
            std::cout << "[下发] 这一次真正要发的命令: " << willSend
                      << "  +  " << RelayCore::payloadLoadSwitchCommand() << std::endl;
        }
        // 照抄下来, 确认时发的就是这一份 (见 s_confirmCandidate 的说明)。
        s_confirmCandidate = s_sendCandidate;
        s_awaitingSendConfirm = true;
        std::cout << "[下发] ⏸ 【尚未发送】—— 确认请按 '" << SEND_CONFIRM_KEY
                  << "' , 按【任何其他键】取消 (取消不会发出任何字节)。" << std::endl;
        std::cout << "[下发]    确认与取消都要在【机械臂旁边的人】就位之后再按。" << std::endl;
    }

    // 确认键按下 -> 真正发送 (发的是按下 'p' 时照抄的那一份候选)。
    static void confirmSendCandidate() {
        if (!s_awaitingSendConfirm) return;
        s_awaitingSendConfirm = false;   // 先清状态: 发送过程里它不该再是真
        // 两条命令与逐条回执在 RelayCore::sendPayloadToRobot 里打 (它能分辨哪一条失败)。
        const bool ok = RelayCore::instance().sendPayloadToRobot(
            s_confirmCandidate.massKg, s_confirmCandidate.comMm);
        if (ok) {
            std::cout << "[下发] ✓ 完成。" << std::endl;
        } else {
            // 【不静默】: 两条只成了一条时机械臂的负载参数是半新半旧的, 而上一条成功那条
            // 可能已经让它动过了 —— 这句话就是"接下来该怎么办"的入口。
            std::cout << "[下发] ✗ 未完成 (见上面的逐条回执) —— 机械臂的负载参数【不要】当作"
                      << "已更新; 按 §1 的安全规程处置, 再决定是否重发。" << std::endl;
        }
    }

    // 非确认键 (或任何其他键) -> 取消。必须出声: 静默地什么都不做, 操作员会以为发出去了。
    static void cancelSendConfirm() {
        if (!s_awaitingSendConfirm) return;
        s_awaitingSendConfirm = false;
        std::cout << "[下发] ✗ 已取消 —— 本次【什么都没有发出去】(要发就重新按 'p', 再按 '"
                  << SEND_CONFIRM_KEY << "')。" << std::endl;
    }

    static bool awaitingSendConfirm() { return s_awaitingSendConfirm; }
    // 确认键给按键处理用 (它在命名空间外) —— 键值只此一处定义, 提示文字与判键都取它。
    static char sendConfirmKey() { return SEND_CONFIRM_KEY; }
    static bool isSendConfirmKey(unsigned char key) {
        return key == (unsigned char)SEND_CONFIRM_KEY
            || key == (unsigned char)(SEND_CONFIRM_KEY - 'a' + 'A');
    }

}

// ===== 启动零偏漂移检查 =====
// 负载参数是工具的物理属性、不随时间漂移, 所以启动【不】验证负载 —— 它按需重标。
// 会漂的是力传感器零偏, 这里便宜地查它:
//   负载正确时残余力与姿态无关, 因此任意静止姿态读一次 "补偿后读数" 就是漂移量。
// 零操作负担: 不摆姿态、不阻断、不写盘。
static bool g_zeroCheckDone = false;
static DWORD g_zeroCheckStartMs = 0;          // 累计窗口的起点 —— 闸门每次拒绝都会【重开】它
static bool  g_zeroCheckRefusalSeen = false;  // 是否已经历过第一次被拒
static DWORD g_zeroCheckFirstRefusalMs = 0;   // 第一次被拒的时刻 —— 【只设一次】, 重开窗口不动它
static double g_zeroCheckAccum[3] = {0, 0, 0};
static int g_zeroCheckCount = 0;

// 一致性闸门在拒绝时【本检查没法做】: fd.filtered 是从被闸门置零的 compensated 推出来的,
// 那是一串 0, 不是零偏 —— 拿它算漂移必然得出 0, 于是这里会在闸门一直拒绝的时候
// 【无条件打印"正常"】。那正是本项目最怕的"安静地错", 而且它是运行时闸门这一次改动
// 【新造出来】的 (复审 Minor 5)。所以: 闸门在拒绝时不作结论、把累计窗口清零重开、等它放行,
// 最多等这么久; 到点仍不放行就定稿并明说"本次没查", 不打印任何结论。
//
// ⚠ 等待与累计【用的是两个时钟】(2026-09-19 复审 Important 1):
//   累计窗口的起点 g_zeroCheckStartMs 在每次拒绝时都被重开, 所以它量不到"等了多久" ——
//   当初拿它当等待的截止时刻, 这个截止就永远到不了 (每 2 s 被推后一次), 结果是闸门一直
//   拒绝时【一行都不打印】: 既不报结论, 也不报"没查"。等待改用 g_zeroCheckFirstRefusalMs:
//   只在第一次被拒那一刻设一次, 之后重开窗口不动它 ⇒ 从第一次被拒起算满 60 s 仍不放行
//   就定稿并打印【未做】。
//   (另一条路是删掉截止、无限重试直到闸门放行 —— 那样更"贴心"(修好负载参数就不必重启),
//    但漂移检查在闸门一直拒绝时会永远安静; 这里选截止 + 明说, 因为"没查"必须有句话。)
//
// ★ 2026-09-21 (Task 5): 上面这条原则原先【自己就没做到】—— 放行后样本不够那一支是
//   静默的 (设完 g_zeroCheckDone 直接 return, 结论和"没查"都不打)。已改成明说
//   (判定在 force/ZeroDriftCheck.h, 那一支现在吐【样本不足, 本次不作结论】)。
// ★ 2026-09-21 (Task 7): 下面 `gs != OK` 这一支【现在覆盖三种拒绝原因】, 第三种是
//   【参考量不可用】(判据那一侧没有数据)。本函数【不按原因分别处置】—— 三种情形下
//   fd.filtered 都是闸门置的 0, 所以结论一样是"本次没查"; 但【该说的话不一样】,
//   那一段文字由 ZeroDriftCheck::decide 按状态给出 (它分得开"去标定/去查下发/
//   去查这一路的数据"三件事)。别在这里再补一句笼统的"闸门在拒绝"。
static const DWORD ZERO_CHECK_GUARD_WAIT_MS = 60000;

// 出结论所需的最少样本数。原先写死在下面的判定里 (字面量 10), 抽判定时提成常量:
// 阈值/等待期/最少样本数这三个旋钮都由本侧传给纯函数, 判定侧不写死任何数。
static const int ZERO_CHECK_MIN_SAMPLES = 10;

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

    // ⚠ 闸门在拒绝 -> 读数被置零, 此刻量不到零偏。【不装作查过】。
    // 判定本身是纯函数 (force/ZeroDriftCheck.h): 本侧只负责【采样、时钟、定稿标志】,
    // 判定侧不看时钟、不读全局、不打印 —— 于是它能被单测直接调用。
    const ForceCompensation::GuardState gs = ForceCompensation::guardState();

    ZeroDriftCheck::Input in;
    in.guard = gs;
    in.thresholdN = Config::FORCE_ZERO_DRIFT_WARN_N;
    in.waitMs = ZERO_CHECK_GUARD_WAIT_MS;
    in.minSamples = ZERO_CHECK_MIN_SAMPLES;

    if (gs != ForceCompensation::GuardState::OK) {
        // 等待的起点是【第一次】被拒那一刻 —— 下面每次都会重开累计窗口, 拿窗口起点当
        // 等待起点的话这个截止永远到不了 (复审 Important 1)。
        if (!g_zeroCheckRefusalSeen) {
            g_zeroCheckRefusalSeen = true;
            g_zeroCheckFirstRefusalMs = now;
        }
        in.refuseElapsedMs = now - g_zeroCheckFirstRefusalMs;
        in.sampleCount = g_zeroCheckCount;   // 拒绝分支不消费它, 但传真值, 免得读的人以为有值
        const ZeroDriftCheck::Decision d = ZeroDriftCheck::decide(in);
        if (d.outcome == ZeroDriftCheck::Outcome::Waiting) {
            // 还没等满: 不是结论, 这一次不出声并重开窗口 —— 已经积进去的那一段是闸门置的 0,
            // 留着会把后面的真读数稀释掉。(只动累计窗口, 不动上面那个等待时钟。)
            g_zeroCheckStartMs = now;
            g_zeroCheckCount = 0;
            for (int i = 0; i < 3; i++) g_zeroCheckAccum[i] = 0.0;
            return;
        }
        // 满等待期仍不放行 -> 【未做】: 明说本次没查, 不给漂移数。
        g_zeroCheckDone = true;
        std::cout << d.text << std::endl;
        return;
    }

    for (int i = 0; i < 3; i++) g_zeroCheckAccum[i] += fd.filtered[i];
    g_zeroCheckCount++;
    if (now - g_zeroCheckStartMs < 3000) return;

    // 定稿。样本够不够、能不能出结论, 都由判定侧说 —— 包括【样本不足】也必须说出口
    // (旧代码在这一支设完 done 就 return, 一句都不打)。
    for (int i = 0; i < 3; i++) in.mean[i] = g_zeroCheckAccum[i] / g_zeroCheckCount;
    in.sampleCount = g_zeroCheckCount;
    const ZeroDriftCheck::Decision d = ZeroDriftCheck::decide(in);
    g_zeroCheckDone = true;
    std::cout << d.text << std::endl;
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
    // ===== Task 8a: 发送确认的拦截 (I3) =====
    // 【必须在所有其他按键之前】(退出键 'q'/ESC 除外, 它在上面已经处理并 exit 了):
    // 确认提示挂着的时候, 除确认键以外的【任何】键都是取消 —— 包括 'm' / 's' / SPACE。
    // 拦截整段按键 (而不是只认一下 'y') 是【故意的】: 一个挂着的"要不要向机械臂下发"提示
    // 不该在操作员按下别的键时悄悄留在那儿, 更不该让那个键顺带做别的事。取消一定出声。
    if (BiasCheck::awaitingSendConfirm()) {
        if (BiasCheck::isSendConfirmKey(key)) {
            BiasCheck::confirmSendCandidate();
        } else {
            std::cout << "[下发] 收到非确认键 ('" << key << "') -> 取消本次下发。" << std::endl;
            BiasCheck::cancelSendConfirm();
        }
        return;
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
                      << "       's' 用这批数据【求解原始通道】, 只打印、不改动任何东西"
                      << " (不写补偿 / 不写 json / 不下发机械臂)\n"
                      << "       'p' 【唯一】会把负载参数发给机械臂的键 —— 它只摆出安全规程与"
                      << " 这一次要发的命令,【要再按 '" << BiasCheck::sendConfirmKey()
                      << "' 才真正发出】; 按任何其他键取消。\n"
                      << "       ★ 重复对 (模型形式检验的尺子): 摆姿态 → SPACE 采样;\n"
                      << "         ★ 保持不动 → 按 'r' → 再按 SPACE 采一次（同一姿态）。\n"
                      << "         两次采样之间机械臂【不许移动】—— 'r' 配的是【上一次采样】,"
                      << " 一移动, 这一对量到的就是两个姿态之间的重力差, 不是复现性。\n"
                      << "         ★ 每一对都要【换一个姿态】: 挑 5 个以上姿态, 每个姿态采两次配成"
                      << " 一对; 不要在同一个姿态上连着配对 —— 那样相邻两对共用一次采样,"
                      << " 尺子的真实自由度只有约 2R/3, 而这里按对数 R 报。\n"
                      << "         ★ 为什么是 5 而不是 2~3: 力矩分支的冤枉率在 R=3 时实测 7~23%,"
                      << " 到 R=5 才降到 1% 量级 (力通道在 R=3 就已经是 0.05%, 两条门差得远)。\n"
                      << "         每一对再按 'r' 是【追加】一对 (尺子的自由度跟着涨): "
                      << BiasCheck::FALSE_REJECT_RATES << "。\n"
                      << "         同一姿态的两次采样之差是【姿态间复现性】: 模型形式检验 (残差是否"
                      << "超出这台设备复现一个姿态的能力) 就拿它当尺子。\n"
                      << "         每采完一对, 结算行会打出这一对的【尺子读数 σ_rep】: 它该与 's'"
                      << " 屏幕上的 rmsF 同量级 —— 明显偏大才是中间机械臂动过。\n"
                      << "         一对都没有的话, 模型形式无从判定, 求解会拒绝给参数 (不是少一个"
                      << "可有可无的步骤)。"
                      << std::endl;
        } else {
            BiasCheck::mode = false;
            // 挂着的发送确认一并撤销。⚠ 顶上的拦截已经保证"确认提示挂着时按 'm' 走的是取消
            // 那一支", 所以这一行今天【到不了】; 留着是因为"待确认状态活过模式切换"这件事
            // 一旦发生, 后果是操作员在一个他以为已经退出的模式里按了确认键。
            if (BiasCheck::awaitingSendConfirm()) {
                BiasCheck::cancelSendConfirm();
            }
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
        if (want) {
            std::cout << "       现在可以手动拖动机械臂摆姿态; 摆好后按 'd' 锁定位姿再 SPACE 采样"
                      << std::endl;
        } else {
            // 【锁定那一刻把当前位姿打出来】(2026-09-20)。
            // 为什么: 在这之前操作员【只有按了 SPACE 才知道当时的角度】—— 而按 SPACE 就已经采样了。
            //   于是"把机械臂摆回上一次那个姿态"这件事【根本没法做】: 没有实时角度就没法"先看再调",
            //   而配对实验 (发送前/后同一姿态) 完全靠它。这个死结在 2026-09-20 的现场讨论里卡住了
            //   整个诊断 —— 补这一行把它解开。
            // 为什么打在这一刻: 拖拽【已经停了】—— 拖拽中读会读到还在动的位姿; 而"锁定"正是操作员
            //   判断"到位了没有"的自然时机。不对就再按 'd' 松开重拖 ⇒ 闭环收敛。
            // 读法: 与 record() 同一个来源、同一把锁 (robotPoseMutex), 所以打出来的数与采样会记的
            //   那个位姿【同源】—— 不是另问一次 GetPose。
            double px, py, pz, prx, pry, prz, jj[6];
            EnterCriticalSection(&appState.robotPoseMutex);
            px  = appState.robotActualPose.x;   py  = appState.robotActualPose.y;
            pz  = appState.robotActualPose.z;
            prx = appState.robotActualPose.rx;  pry = appState.robotActualPose.ry;
            prz = appState.robotActualPose.rz;
            jj[0] = appState.robotActualPose.j1; jj[1] = appState.robotActualPose.j2;
            jj[2] = appState.robotActualPose.j3; jj[3] = appState.robotActualPose.j4;
            jj[4] = appState.robotActualPose.j5; jj[5] = appState.robotActualPose.j6;
            LeaveCriticalSection(&appState.robotPoseMutex);
            std::cout << "       位姿已锁定, 可以按 SPACE 采样了" << std::endl;
            std::cout << "       当前姿态: Rx=" << prx << " Ry=" << pry << " Rz=" << prz
                      << "   (X=" << px << " Y=" << py << " Z=" << pz << ")" << std::endl;
            // 【关节角也打】(2026-09-20)。为什么需要:
            //   · Rx/Ry/Rz 【完全决定】法兰(以及刚性装在上面的传感器)在基座系里的朝向 ——
            //     其余关节的贡献已经含在这三个角里了。所以对 g_法兰(进而对我们的 compensated
            //     与 @576)来说, 配 Rx/Ry/Rz 就够。
            //   · ⚠ 但 **@720 是"通过关节电流计算"的** —— 它跟的是【关节构型】, 而同一个笛卡尔
            //     姿态可以由【不同的关节解】达到(肘上/肘下、腕翻转)。若两轮构型不同, @720 的
            //     对比就不干净。手拖是连续移动、通常落在同一个解上, 但那是【假定】。
            //   ⇒ 打出来就能【核】: 两轮的 j1..j6 一致 ⇒ 构型相同; 不一致 ⇒ 那一笔要单独判。
            //   (顺带: 以后若要"轨迹复现/开回同一姿态", 关节角正是 JointMovJ 要的那六个量。)
            std::cout << "       关节角: J1=" << jj[0] << " J2=" << jj[1] << " J3=" << jj[2]
                      << " J4=" << jj[3] << " J5=" << jj[4] << " J6=" << jj[5] << std::endl;
            std::cout << "       ↑ 对着 Rx/Ry/Rz 凑(差几度以内即可); 不对就【再按 'd' 松开重拖】"
                      << " —— 别按 SPACE, 按了就采了。" << std::endl;
        }
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

    // 'r' in BiasCheck mode: 把【紧接着的下一次采样】登记为【上一次采样】的重复访问 —— 模型
    // 形式检验的尺子 (协议: 原地复采, 从按 'r' 到采完为止机械臂不移动)。
    // 独立于 SPACE 的键, 理由见 BiasCheck::repeatIdx 的说明 (显式标记, 不靠位置也不靠距离)。
    // 每按一次【追加】一对 (不覆盖), 见那里的说明。
    if ((key == 'r' || key == 'R') && BiasCheck::mode) {
        BiasCheck::recordRepeat();
        return;
    }

    // 's' in BiasCheck mode: 拟合原始 @1304 通道并【全部打印】, 判决通过时装进本地补偿 + 落盘。
    //
    // 四件事的【实际样子】—— 逐条对着 solveAndApply 的活路径核过 (写这条注释前的规矩: 每个
    // 前提回到源头核一遍; 本行从前写的是"不写补偿 / 不写 json / 不下发", 把几件事混成了一句):
    //   · 【装】本地补偿 —— 判决 (fitOk) 通过时调 ForceCompensation::setCalibration。
    //     ⚠ 这一条是 2026-09-20 才加上的, 之前【没有】: 那时求解结果没有任何路径能进
    //       ForceCompensation (唯一的安装入口 setMassCom 已随残余模型在 Task 6 删除),
    //       于是本地补偿永远未启用、一致性闸门在【做比较之前】就早退。见 solveAndApply 顶上。
    //   · 【写】force_calib.json —— 同上, 按 version 3 (ForceCalibration::saveToFile), 且
    //     【门控在"确实装上了"之后】: setCalibration 拒收的模型不落盘 —— 否则那份文件就是
    //     "自称是标定结果、装载时又被拒一次"的坏文件。
    //   · 【不】写 payload_calib.json —— 本条路径里没有任何 PayloadCalibration::applyResult /
    //     PayloadCalibration::save 调用 (调它们的是下面的 #if 0 块, 旧模型那一层)。
    //     ⚠ 这一条从前把 TcpCalibration::setSensorYawDeg 也算进来了, 说"三个调用只出现在
    //       #if 0 块内" —— 【那是错的】: 它【在活路径上】, 只是不在这条路径上。真正的调用点是
    //       main() 的启动序列里、紧接 PayloadCalibration::load 之后那一处 (在 `else` of
    //       `if (g_noRobot)` 内, 连机械臂之前; 与 's' 无关, 也不由任何按键触发)。写这条注释
    //       时逐处核过全文件: 那两处之外没有第三个调用, solveAndApply 的【活路径】里一处都没有。
    //       ⚠ 措辞是"活路径"不是"里" —— 严格读"solveAndApply 里"是【假的】: 那个 #if 0 块
    //       (本函数末尾那一大段"旧模型") 就嵌在 solveAndApply 的花括号【内】, 而它里面确实有一处
    //       setSensorYawDeg。(这里【故意不写行号】: 本注释自己一动行号就漂。)
    //   · 【不】下发机械臂 —— 本条路径里没有任何 robotSendEnable / sendPayloadToRobot 调用;
    //     下发是另一个键 'p' 的事 (Task 8a), 而且要先过两道闸。
    //     ⚠ 【装本地补偿不是下发】: 前者改的是"我们这边怎么理解传感器读数", 机械臂一个字节
    //       都收不到; 后者要的是法兰系上整条链, 换算未定, 仍等 plan Task 9 (见 solveAndApply
    //       顶上). 别把这两件事一起接回来 —— 它们的开通条件不是同一个。
    //   ⚠ 它落的文件【最多四份】: calib_log.txt 一行 (logCalibAttempt)、calib_poses.txt 一批
    //     姿态原始数据 (logPoseData)、calib_report.md 一整块 (diagFinish), 以及判决通过时的
    //     force_calib.json。前三份是【记录】, 只有最后一份是【生效值】—— 它是"重启之后"的
    //     那一份, 记的是按下 's' 那一刻装进去的模型。
    //     ⚠ 别把"文件里的零偏"读成"此刻生效的零偏": 模块在【静止】时有一个把 b_F/b_M 拉向
    //       "让 compensated→0"的在线 EMA (ForceCompensation 的 step 末段)。它只在闸门放行
    //       时才跑, 所以今天 (未标定/不一致) 两者相等 —— 但闸门一旦开始放行, 它们就会慢慢分家。
    //       要持久化【此刻生效的】零偏, 仍得走 'z' 那条路。
    if ((key == 's' || key == 'S') && BiasCheck::mode) {
        BiasCheck::solveAndApply();
        return;
    }

    // 'p': 【唯一】会把负载参数发给机械臂的键 (Task 8a)。在 'm' 模式里才有意义 ——
    // 候选正是 'm' 采集 + 's' 求解的产出, 而且这条路只此一处, 别的地方没有第二个入口。
    //
    // ⚠ 【这个闸会挡住一个真的有效的候选, 所以它【必须】出声】(8a 复审 Minor 3):
    //   在 'm' 模式里再按一次 'm' = 退出模式并重出报告 (见上面 'm' 那个分支: mode = false +
    //   report()), 它【不调 reset()】⇒ 候选还活着, 而 mode 已经是 false。那时按 'p' 从前
    //   一行都不打 —— 一个静默无反应的发送键比一个大声拒绝的坏得多 (操作员会以为发出去了)。
    //   ⚠ 提示里【不许】写"再按 'm' 回去按 'p'": 重进 'm' 模式走的是 reset(), 而 reset()
    //     会作废候选 (见 s_sendCandidateValid 的说明)。能下发的窗口就是【这一次采集里】:
    //     按 's' 求解之后、【退出模式之前】。
    if ((key == 'p' || key == 'P') && !BiasCheck::mode) {
        std::cout << "[下发] 'p' 只在 'm' 模式里有效 —— 此刻不在模式里, 所以【什么都没有发出去】。"
                  << std::endl;
        if (BiasCheck::s_sendCandidateValid) {
            std::cout << "[下发] ⚠ 手上【还有】一份过闸的候选 (最近一次 's' 解出来的), 但它的"
                      << "下发窗口随退出 'm' 模式一起关了 ——" << std::endl;
            std::cout << "[下发]    重进 'm' 模式是【重开采集】(会作废这份候选), 所以可下发的"
                      << "时机是: 在这次采集里按 's' 求解之后、【退出模式之前】。"
                      << std::endl;
        }
        return;
    }
    if ((key == 'p' || key == 'P') && BiasCheck::mode) {
        // 只摆出来 + 挂上待确认 (I3)。真正发送在 confirmSendCandidate, 由上面那段
        // 【先于所有按键】的拦截在确认键按下时调用 —— 所以本条分支【不发送任何字节】。
        BiasCheck::sendCandidate();
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
            // ⚠ 2026-09-19 (Task 6) 起, 本地补偿【不再读 ψ】: 全量模型走
            // TcpCalibration::gravitySensorFrameAtYaw(pose, 0.0, ·), 安装旋转由自由 3×3 的
            // A 吸收。这里装上的 ψ 只剩一个消费者 —— 已停用的 psi 扫描求解路径
            // (PayloadCalibration::solve, 见 main.cpp 的 #if 0 块) 与夹具回归用例。
            // 仍然装上: 它是 payload_calib.json 的一部分, 撤掉会改变那条路径读到的状态。
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

    // 4.6 加载力传感器标定文件 (全量模型: A / b_F / b_M / c_s —— Task 6 起的格式)
    {
        double A[9], biasF[3], biasM[3], cS[3];
        if (ForceCalibration::loadFromFile(CalibStore::fileFor("force_calib.json"),
                                           A, biasF, biasM, cS)) {
            g_hasStoredZeroCalib = true;   // 有存储零偏, 启动漂移检查才有得比
            ForceCompensation::setCalibration(A, biasF, biasM, cS);
            std::cout << "[Force] Loaded force_calib.json (mass scale="
                      << ForceCompensation::currentMassKg()
                      << "kg, bias=" << biasF[0] << "," << biasF[1] << "," << biasF[2] << "N"
                      << ", c_s=(" << cS[0] * 1000.0 << "," << cS[1] * 1000.0 << ","
                      << cS[2] * 1000.0 << ")mm)" << std::endl;
        } else {
            // 旧格式被拒时 loadFromFile 已经在 stderr 上响亮地说过是哪一种不兼容,
            // 这里只补一句"现在能做什么", 不重复那一段。
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
