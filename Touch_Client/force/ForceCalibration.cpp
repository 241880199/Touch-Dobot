#define _USE_MATH_DEFINES
#include "ForceCalibration.h"
#include "ForceCompensation.h"
#include "../config/Config.h"
#include "../core/CalibStore.h"
#include "../core/JsonLite.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <windows.h>   // GetTickCount / DWORD —— 实测间隔的时基 (与 ForceCompensation.cpp 同一做法)

// ===== Internal state =====

static ForceCalibration::State g_state = ForceCalibration::State::IDLE;
static bool g_tareOnly = false;   // true = 仅调零流程: TARE 定稿后直接应用+存盘, 不进 MOTION

// TARE
static double g_phaseTimer = 0.0;
static double g_tareAccum[6] = {0};
static int    g_tareCount = 0;
// ★ 2026-09-22 (I1): "样本数不足"那条告警【每次 TARE 只喊一次】(否则每帧刷屏)。
// ⚠ 它【不是】函数内的 static: 那样一次 TARE 喊过之后, 后面每一次按 'z' 都再也不吭声
//   (轮询再停一次也无声) —— 那正好把本条要治的"静默地定稿"换个形式搬回来。
//   ⇒ 复位点放在 start() / startZero() 里 (每启动一次调零就重新允许喊一次)。
static bool   g_tareShortWarned = false;

// ★ 2026-09-22 (A): "两次轮询隔得太久"那条诊断【同样每次 TARE 只喊一次】——
//   理由与上面那条逐字相同: 不节流就是每帧刷屏, 而它治的正是【同一类事件】
//   (轮询停过一段 ⇒ 本帧的经过时间不算数, 见 updateIntervalSec)。
static bool   g_tareGapWarned = false;

// MOTION — record F vs a during user movement
static const int MAX_MOTION_SAMPLES = 300;  // ~10s at 30Hz
static int    g_motionCount = 0;
static double g_motionRaw[300][6];   // raw force at each frame
static double g_motionPos[300][3];   // position (mm) at each frame
static double g_motionRxyz[300][3];  // orientation Rx,Ry,Rz (deg) at each frame

// Results
static double g_massKg = 0.0;
static double g_biasForce[3] = {0};
static double g_biasTorque[3] = {0};

// Drag callback
static void (*g_dragCb)(bool) = nullptr;

// ===== MotionEstimator for MOTION phase =====
// Lightweight: 5-point pos buffer → velocity + acceleration

namespace ForceCalibration {

void setDragModeCallback(void (*cb)(bool)) { g_dragCb = cb; }

State currentState() { return g_state; }

bool isRunning() {
    return g_state != State::IDLE && g_state != State::DONE && g_state != State::ABORTED;
}

bool isZeroing() {
    return g_tareOnly && isRunning();
}

bool isDone() {
    return g_state == State::DONE || g_state == State::ABORTED;
}

const char* statusText() {
    switch (g_state) {
        case State::IDLE:    return "Idle";
        case State::TARE:    return g_tareOnly ? "Zeroing... (keep still)"
                                               : "Taring... (keep still)";
        case State::MOTION:  return "Motion cal... (move robot, SPACE to stop)";
        case State::SOLVE:   return "Solving...";
        case State::DONE:    return "Calibration complete";
        case State::ABORTED: return "Calibration aborted";
    }
    return "Unknown";
}

// 由采集缓冲定稿零偏 (力 + 力矩)
// 上一次定稿时【从零偏里减掉的重力项】—— 只为了让 TARE done 那行报出来。
// 用途: 调零之后若有人怀疑"零偏怎么和原始读数差了 4N", 那一行就是答案 (不是零偏怪,
//   是重力项被归到模型那边去了)。
static double g_lastFinalizeFg[3] = {0, 0, 0};
static double g_lastFinalizeMg[3] = {0, 0, 0};

// 用例驱动的时间步长 (秒)。负数 = 不干预, 走实测。见头文件 setUpdateDtForTest。
static double g_updateDtForTest = -1.0;

// 两次 update() 之间的【真实】耗时 (s)。第一次调用返回 0。
// ★ 2026-09-22: 这就是从前写死在调用点上的那个 0.033 的替代。理由见头文件。
// ⚠ 它靠"每次轮询都被调用"保持新鲜 —— 若调用点又加回 `if (isRunning())` 的门,
//   计时器会停在【上一次运行】那一刻 ⇒ 下次启动的第一个 dt 是那之间的全部时间
//   (可能是几分钟) ⇒ 一步跨过静默期与累计期, 从前【而且不报任何错】。
//   ★★ 2026-09-22: 这个 hazard 现在【被本函数末尾那道 gap 守卫拦住】—— 那正是守卫存在的
//   理由: 超限的间隔返回 0 ⇒ 计时器不跳; 并且守卫自己会喊一声 (见下, 那次改的)。
//   判据见 Task 1 Step 8。
static double updateIntervalSec() {
    double dt;
    if (g_updateDtForTest >= 0.0) {
        // 用例给定的值【也走下面那道守卫】—— 否则"不合理的间隔"这条路径无法被验。
        dt = g_updateDtForTest;
    } else {
        static DWORD lastMs = 0;
        const DWORD now = GetTickCount();
        if (lastMs == 0) { lastMs = now; dt = 0.0; }
        else { dt = (now - lastMs) / 1000.0; lastMs = now; }
    }
    // ★ 2026-09-22 (I1): 不合理的间隔当【没测到】—— 返回 0, 不让计时器跟着跳。
    //   见 Config::FORCE_CALIB_MAX_INTERVAL_S 处的说明。
    // ★ 2026-09-22 (A): 但【守卫自己必须出声】。它从前是静默的: 触发它的那个事件
    //   (轮询停过一段 ⇒ 操作员眼里"TARE 怎么这么久") 在控制台上【不留任何痕迹】,
    //   而下一条告警 (样本数不足) 嵌在 timer >= 2.5 里面, 这里刚把 dt 置 0 ⇒
    //   计时器永远到不了那个分支 ⇒ 它【不可能】替这条路径喊。⇒ 加一行, 节流方式
    //   与那条告警一致 (g_tareGapWarned, 每次 TARE 只喊一次; 复位点在 start/startZero)。
    if (dt > Config::FORCE_CALIB_MAX_INTERVAL_S) {
        if (!g_tareGapWarned) {
            g_tareGapWarned = true;
            printf("[Force] WARNING: 两次轮询间隔 %.2fs (> %.2fs) —— 疑似轮询停过一段。\n"
                   "        本帧的经过时间当【没测到】(dt = 0), 相位计时器不跟着跳。\n",
                   dt, Config::FORCE_CALIB_MAX_INTERVAL_S);
        }
        return 0.0;
    }
    return dt;
}

void setUpdateDtForTest(double sec) {
    g_updateDtForTest = sec;
}

static void finalizeBias() {
    int n = (g_tareCount > 0) ? g_tareCount : 1;
    for (int i = 0; i < 6; i++) g_tareAccum[i] /= n;

    // ★ 2026-09-21 修: 全量模型下, 零偏必须是 "@1304 − A·g", 不是 "@1304 本身"。
    //   模型是 comp = @1304 − bF − A·g (力矩: @1304_M − bM − c_s×(A·g)), 而 @1304 里【含着
    //   重力项】。残余模型时代那条式子里的重力项由标量 mass 提供, 那时 mean(@1304) 正好就是
    //   传感器零点, 所以这里从前直接存平均。**换成全量模型之后 A·g 由模型提供, 同一个重力
    //   就被减了两遍。**
    //   实测后果 (2026-09-21 现场, 按 'z' 之后): comp = −A·g ≈ 4N ⇒ 闸门判 INCONSISTENT ⇒
    //   compensated 全置零 ⇒ **一点力反馈都没有**; 而且 force_calib.json 被写成错零偏 ⇒
    //   此后每次启动都拒。
    //   【独立确认】两个零偏的偏移量 —— 力 (-2.29,-1.80,+3.33) 模长 4.5N、力矩模长 0.13 N·m ——
    //   与 |A·g| = m·g = 0.4196×9.81 = 4.12N、|c_s × Fg| = 0.0554×4.12 = 0.23 N·m 同量级。
    //   ⇒ 读到 step() 算的那一份重力项并减掉, 让"调零之后该姿态上 comp ≈ 0"这条定义成立。
    //   ⚠ A 全 0 (还没有全量模型) 时 Fg/Mg 也是 0 ⇒ 本修法与从前等价 ⇒ 下面那条
    //     "A 全 0" 的警告路径不受影响。
    //   ⚠ 读的是【step() 那一份】(唯一一份定义), 不是在这里另算一遍 —— 同一个量两份实现
    //     正是本 bug 的成因。
    double Fg[3], Mg[3];
    ForceCompensation::currentGravityTerm(Fg, Mg);

    for (int i = 0; i < 3; i++) {
        g_biasForce[i]  = g_tareAccum[i]     - Fg[i];
        g_biasTorque[i] = g_tareAccum[i + 3] - Mg[i];
    }
    // 把减掉的量也报出来: 调零之后若有人怀疑"零偏怎么和原始读数差 4N", 这一行就是答案。
    g_lastFinalizeFg[0] = Fg[0]; g_lastFinalizeFg[1] = Fg[1]; g_lastFinalizeFg[2] = Fg[2];
    g_lastFinalizeMg[0] = Mg[0]; g_lastFinalizeMg[1] = Mg[1]; g_lastFinalizeMg[2] = Mg[2];
}

// 可启动: 空闲, 或上一次已结束 (DONE/ABORTED) — 允许同一进程内重新调零/重标
static bool canStart() {
    return g_state == ForceCalibration::State::IDLE
        || g_state == ForceCalibration::State::DONE
        || g_state == ForceCalibration::State::ABORTED;
}

bool start() {
    if (!canStart()) return false;

    g_state = State::TARE;
    g_tareOnly = false;
    g_phaseTimer = 0.0;
    g_tareCount = 0;
    for (int i = 0; i < 6; i++) g_tareAccum[i] = 0.0;
    g_tareShortWarned = false;   // ★ 2026-09-22 (I1): 每一次新的 TARE 都重新允许喊一次
    g_tareGapWarned = false;     // ★ 2026-09-22 (A): 同上 —— 那条 gap 诊断也重新允许喊一次

    // ★ 2026-09-21: 这里从前硬写 "2s" —— 而 TARE 现在多了一段 0.5s 静默期 (见 update() 的 TARE 分支),
    //   总时长是 2.5s。跟上面那条一样: 时长写错会让操作员在静默期里就松手 ⇒ 又采到瞬态。
    // ★★ 2026-09-22: 那句打印【到今天才第一次是诚实的】。从前 update() 收到的是调用方传的
    //   常数 0.033, 而它实际被调用的节拍是 46~203ms(均值 92ms) ⇒ 计时器走完 2.5s 要 76 次
    //   调用 ≈ 实际 ~7s ⇒ **从前打印 2.5s 而实际等 ~6.75s**, 静默期本身也被拉长 2.7 倍。
    //   现在 dt 是实测的 ⇒ 下面这个数就是操作员真的要等的秒数。
    printf("[Force] Calibration started — TARE phase (keep robot still for %.0fs = "
           "%.1fs 静默期 + %.0fs 累计)...\n",
           Config::FORCE_CALIB_SETTLE_TIME_S + Config::FORCE_CALIB_STILL_COLLECT_S,
           Config::FORCE_CALIB_SETTLE_TIME_S, Config::FORCE_CALIB_STILL_COLLECT_S);
    return true;
}

bool startZero() {
    if (!canStart()) return false;

    g_state = State::TARE;
    g_tareOnly = true;
    g_phaseTimer = 0.0;
    g_tareCount = 0;
    for (int i = 0; i < 6; i++) g_tareAccum[i] = 0.0;
    g_tareShortWarned = false;   // ★ 2026-09-22 (I1): 每一次新的 TARE 都重新允许喊一次
    g_tareGapWarned = false;     // ★ 2026-09-22 (A): 同上 —— 那条 gap 诊断也重新允许喊一次

    // ★ 2026-09-21: 时长与分工要交代清楚 —— 前 0.5s 是【静默期】(不采样), 之后才累计。
    //   不写出来, 操作员会按旧的 2 秒去等, 然后在静默期里就松手/动臂 ⇒ 又采到瞬态。
    // ★★ 2026-09-22: 与 start() 里同一条 —— 这个数【从前是假的】: update() 那时收到常数
    //   0.033 而真实节拍 92ms ⇒ **从前打印 2.5s 而实际等 ~6.75s** (静默期 ~1.35s)。
    //   2026-09-21 加静默期本来就是为了治"操作员提前松手", 而时长被算错 2.7 倍
    //   恰好又制造了同一件事。现在 dt 实测 ⇒ 打印的秒数 = 真实要等的秒数。
    printf("[Force] ZERO started — keep robot still for %.0fs "
           "(前 %.1fs 静默期不采样, 之后 %.0fs 才累计)...\n"
           "        (TARE only: no motion phase, drag mode NOT enabled)...\n",
           Config::FORCE_CALIB_SETTLE_TIME_S + Config::FORCE_CALIB_STILL_COLLECT_S,
           Config::FORCE_CALIB_SETTLE_TIME_S,
           Config::FORCE_CALIB_STILL_COLLECT_S);
    return true;
}

void abort() {
    printf("[Force] Calibration ABORTED (was: %s)\n", statusText());
    if (g_dragCb) g_dragCb(false);
    g_state = State::ABORTED;
    g_tareOnly = false;   // 中止即放弃, 不做任何应用/存盘
}

void confirmPose() {
    if (g_state == State::TARE) {
        // TARE 由 update() 采集满 FORCE_CALIB_STILL_COLLECT_S 后自动定稿 (定时器递增,
        // 跨过阈值的同一次 update() 就切换状态), 因此这里只报告进度、不需要按键。
        printf("[Force] TARE in progress (%.1f/%.0fs) — auto-completes, no key needed\n",
               g_phaseTimer, Config::FORCE_CALIB_STILL_COLLECT_S);
        return;
    }

    if (g_state == State::MOTION) {
        // SPACE to stop motion recording (min 2s)
        if (g_phaseTimer < 2.0) {
            printf("[Force] Min 2s motion needed (%.1fs elapsed), keep moving...\n", g_phaseTimer);
            return;
        }
        printf("[Force] Motion recording stopped (%d samples). Fitting mass...\n", g_motionCount);
        if (g_dragCb) g_dragCb(false);
        g_state = State::SOLVE;
        return;
    }
}

bool update(const double raw[6], const double pose[6]) {
    // ★ 先量节拍, 【再】早退 —— 顺序不能反。见 updateIntervalSec 的说明:
    //   计时器必须每次轮询都刷新, 否则下次启动的第一个 dt 会是"距上次运行的全部时间"。
    const double dt = updateIntervalSec();

    if (g_state == State::IDLE || g_state == State::DONE || g_state == State::ABORTED) {
        return (g_state == State::DONE || g_state == State::ABORTED);
    }

    switch (g_state) {

    // ===== TARE: accumulate static bias =====
    case State::TARE: {
        // ★★ 2026-09-21: 【先丢弃一段静默期, 再开始累计】(用户 2026-09-21 现场要求:
        //   "静止与运动时保持 0, 只有接触到物体时才变化")。
        // 【为什么需要它】现场: 按 'z' 之后静止读数仍有 ~0.14 N 残余 (主要 z 轴)。
        //   而零偏漂移实测只有 0.16 N/小时 ⇒ **几分钟内本应是 ~0.02 N** ⇒ 那个 0.14 N
        //   【不是漂移, 是调零采到了瞬态】: 操作员刚按完 z (可能刚松手、刚离开机械臂、
        //   或臂还在回弹), 而 TARE 从前【立刻】开始平均 ⇒ 那两个秒里混进了过渡过程。
        // 【处置】加一段 FORCE_CALIB_SETTLE_TIME_S 的静默期 —— 那个常数本来就有
        //   (另一条标定流程在用), 只有 TARE 这条路没用它。静默期内【不累计】, 期满才从头收集。
        // 【预期效果】把那个瞬态分量从零偏里去掉 ⇒ 静态残差降到噪声量级 (~0.02~0.05N)
        //   ⇒ 0.20 的死区就能把剩下的噪声整个盖住 ⇒ 静止/运动时读数显 0 ✓
        //   ⇒ 这是"显示 0"与"感觉到 0.3~0.6N 笔压"两条要求能同时成立的前提。
        // ⚠ 总时长因此从 2.0s 变成 2.0 + 0.5 = 2.5s —— 启动提示文字也一起改了 (见 startZero),
        //   否则操作员会按旧的 2 秒去等。
        g_phaseTimer += dt;
        if (g_phaseTimer < Config::FORCE_CALIB_SETTLE_TIME_S) {
            return false;              // 静默期: 不累计
        }
        for (int i = 0; i < 6; i++) g_tareAccum[i] += raw[i];
        g_tareCount++;
        if (g_phaseTimer >= Config::FORCE_CALIB_SETTLE_TIME_S + Config::FORCE_CALIB_STILL_COLLECT_S) {
            // ★ 2026-09-22 (I1): 【样本数不足就不定稿】—— 继续收, 下一帧再判。
            //   没有这一道: 一个 ≥2.5s 的 stall 会让上面的 timer 一步跨过阈值,
            //   于是这里用【单样本】定稿并把零偏落盘 (finalizeBias 的 n = max(count,1))。
            //   它比 gap 守卫更靠内: 即使计时器真的跳了, 垃圾零偏也写不进盘。
            if (g_tareCount < Config::FORCE_CALIB_MIN_TARE_SAMPLES) {
                if (!g_tareShortWarned) {   // 只喊一次, 别每帧刷屏 (复位点在 start/startZero)
                    g_tareShortWarned = true;
                    printf("[Force] WARNING: TARE 只收到 %d 帧 (< %d) —— 疑似轮询停过一段。\n"
                           "        继续采集, 收够再定稿 (不拿单帧零偏落盘)。\n",
                           g_tareCount, Config::FORCE_CALIB_MIN_TARE_SAMPLES);
                }
                break;   // 停在 TARE, 下一帧继续
            }
            // Auto-complete tare after collection time
            finalizeBias();
            printf("[Force] TARE done: biasF=(%+.3f, %+.3f, %+.3f) N  biasM=(%+.4f, %+.4f, %+.4f) Nm\n",
                   g_biasForce[0], g_biasForce[1], g_biasForce[2],
                   g_biasTorque[0], g_biasTorque[1], g_biasTorque[2]);
            // ★ 2026-09-21: 报出【从零偏里扣掉的重力项】。全量模型下 零偏 = mean(@1304) − A·g,
            //   所以它比控制台上看到的 @1304 原始读数【小约 |A·g| = m·g ≈ 4N】——
            //   那不是异常, 是重力项归到模型那边去了。没有这一行, 下一个人会以为零偏算错了
            //   (本 bug 当初就是没意识到这一点才写成的)。
            printf("[Force]   (全量模型: 已从零偏扣掉 A·g=(%+.3f, %+.3f, %+.3f) N 与 "
                   "c_s×(A·g)=(%+.4f, %+.4f, %+.4f) Nm ⇒ 零偏比 @1304 原始读数小约 4N, 不是异常)\n",
                   g_lastFinalizeFg[0], g_lastFinalizeFg[1], g_lastFinalizeFg[2],
                   g_lastFinalizeMg[0], g_lastFinalizeMg[1], g_lastFinalizeMg[2]);

            // ===== 仅调零: 直接应用+存盘, 保留现有全量模型 (A / c_s), 不进 MOTION、不开拖拽 =====
            if (g_tareOnly) {
                double A[9], cS[3];
                ForceCompensation::currentModel(A, cS);
                // 调零只定【零偏】。若手上还没有全量模型 (A 全 0 —— 从没跑过 'm'+'s' 的
                // @1304 标定), 这次调零会写出一份【没有重力项】的模型: 补偿后的读数会随姿态
                // 漂 (整个 A·g 那一项都没减)。这是"换装工具后应该先重标模型"的信号。
                bool aIsZero = true;
                for (int i = 0; i < 9; i++) if (A[i] != 0.0) aIsZero = false;
                if (aIsZero) {
                    printf("[Force] WARNING: 当前没有全量模型 (A 全 0) —— 本文件里的重力项为空, "
                           "补偿后的读数会随姿态漂移。\n"
                           "        换装工具 (笔夹/笔) 后应先按 'm' 采多姿态 + 's' 解出 A, 再按 'z' 调零。\n");
                }
                ForceCompensation::setCalibration(A, g_biasForce, g_biasTorque, cS);
                ForceCalibration::saveToFile(CalibStore::fileFor("force_calib.json"),
                                             A, g_biasForce, g_biasTorque, cS);
                printf("[Force] ZERO complete (A / c_s kept, mass scale %.4f kg): "
                       "force bias=(%+.3f,%+.3f,%+.3f) N, torque bias=(%+.4f,%+.4f,%+.4f) Nm\n",
                       ForceCompensation::currentMassKg(),
                       g_biasForce[0], g_biasForce[1], g_biasForce[2],
                       g_biasTorque[0], g_biasTorque[1], g_biasTorque[2]);
                printf("[Force] Saved force_calib.json\n");
                g_state = State::DONE;
                g_tareOnly = false;
                break;
            }

            g_state = State::MOTION;
            g_motionCount = 0;
            g_phaseTimer = 0.0;
            if (g_dragCb) g_dragCb(true);
            printf("[Force] MOTION phase: move robot with varying speed + direction, then press SPACE\n");
        }
        break;
    }

    // ===== MOTION: record raw force + position during movement =====
    case State::MOTION: {
        g_phaseTimer += dt;
        // Record sample
        if (g_motionCount < MAX_MOTION_SAMPLES) {
            for (int i = 0; i < 6; i++) g_motionRaw[g_motionCount][i] = raw[i];
            g_motionPos[g_motionCount][0] = pose[0];
            g_motionPos[g_motionCount][1] = pose[1];
            g_motionPos[g_motionCount][2] = pose[2];
            g_motionRxyz[g_motionCount][0] = pose[3];
            g_motionRxyz[g_motionCount][1] = pose[4];
            g_motionRxyz[g_motionCount][2] = pose[5];
            g_motionCount++;
        }
        break;
    }

    // ===== SOLVE: fit m from F = m*a =====
    case State::SOLVE: {
        if (g_motionCount < 5) {
            printf("[Force] Not enough motion data (%d samples), aborting\n", g_motionCount);
            g_state = State::ABORTED;
            break;
        }

        // Compute acceleration from position (2nd-order central diff, mm→m)
        // CRITICAL: acceleration is in base frame, force is in sensor frame.
        // Rotate acceleration into sensor frame before fitting F = m*a.
        double sumFa = 0.0, sumA2 = 0.0;
        int used = 0;

        // 5-point position + orientation history
        double px[5] = {0}, py[5] = {0}, pz[5] = {0};
        double rx[5] = {0}, ry[5] = {0}, rz[5] = {0};
        int pi = 0, pn = 0;

        for (int k = 0; k < g_motionCount; k++) {
            px[pi] = g_motionPos[k][0];
            py[pi] = g_motionPos[k][1];
            pz[pi] = g_motionPos[k][2];
            rx[pi] = g_motionRxyz[k][0];
            ry[pi] = g_motionRxyz[k][1];
            rz[pi] = g_motionRxyz[k][2];
            pi = (pi + 1) % 5;
            if (pn < 5) pn++;

            if (pn < 5) continue;

            int i0 = (pi - 1 + 5) % 5;  // newest
            int i1 = (pi - 2 + 5) % 5;  // middle
            int i2 = (pi - 3 + 5) % 5;

            // 3-point stencil acceleration in BASE frame (m/s²)
            double ax_b = (px[i0] - 2.0 * px[i1] + px[i2]) / (dt * dt) * 0.001;
            double ay_b = (py[i0] - 2.0 * py[i1] + py[i2]) / (dt * dt) * 0.001;
            double az_b = (pz[i0] - 2.0 * pz[i1] + pz[i2]) / (dt * dt) * 0.001;

            // Rotate acceleration: a_sensor = R_base→sensor * a_base
            // R_base→sensor = (Rz*Ry*Rx)^T  for the MIDDLE sample's orientation
            double rxx = rx[i1] * M_PI / 180.0;
            double ryy = ry[i1] * M_PI / 180.0;
            double rzz = rz[i1] * M_PI / 180.0;
            double cx = cos(rxx), sx = sin(rxx);
            double cy = cos(ryy), sy = sin(ryy);
            double cz = cos(rzz), sz = sin(rzz);
            // R = Rz*Ry*Rx
            double R00 = cz*cy, R01 = cz*sy*sx - sz*cx, R02 = cz*sy*cx + sz*sx;
            double R10 = sz*cy, R11 = sz*sy*sx + cz*cx, R12 = sz*sy*cx - cz*sx;
            double R20 = -sy,   R21 = cy*sx,              R22 = cy*cx;
            // a_sensor = R^T * a_base
            double ax = R00*ax_b + R10*ay_b + R20*az_b;
            double ay = R01*ax_b + R11*ay_b + R21*az_b;
            double az = R02*ax_b + R12*ay_b + R22*az_b;

            double aMag = sqrt(ax*ax + ay*ay + az*az);
            if (aMag < 0.05) continue;  // skip near-static

            // Force in sensor frame (raw - bias)
            double fx = g_motionRaw[i1][0] - g_biasForce[0];
            double fy = g_motionRaw[i1][1] - g_biasForce[1];
            double fz = g_motionRaw[i1][2] - g_biasForce[2];
            double fMag = sqrt(fx*fx + fy*fy + fz*fz);

            // F = m*a → m ≈ Σ(|F|·|a|) / Σ(|a|²)
            sumFa += fMag * aMag;
            sumA2 += aMag * aMag;
            used++;
        }

        printf("[Force] Motion fit: %d valid samples (a > 0.05 m/s²)\n", used);

        if (used < 10 || sumA2 < 0.01) {
            printf("[Force] WARNING: insufficient motion data, using m=0\n");
            g_massKg = 0.0;
        } else {
            g_massKg = sumFa / sumA2;
            // Sanity check
            if (g_massKg < 0.0) {
                printf("[Force] WARNING: negative mass (%.3f kg), using 0\n", g_massKg);
                g_massKg = 0.0;
            } else if (g_massKg > 50.0) {
                printf("[Force] WARNING: mass too large (%.1f kg), capping at 5.0\n", g_massKg);
                g_massKg = 5.0;
            }
        }

        printf("[Force] SOLVE: motion-fit mass=%.4f kg\n", g_massKg);

        // ⚠ 这个标量质量【不进补偿】。全量模型 (Task 6) 的参数表里没有标量质量 ——
        //   它由 A 分解出来 (m = |det A|^(1/3), 见 ForceCompensation::currentMassKg),
        //   而 A 是从 @1304 多姿态数据解出的 (按 'm' 采 + 's' 解)。上面这个 F=m·a 的拟合
        //   属于【残余模型】那条路 (那时本地减的是 raw@576 − mass·g_ψ), 它现在只剩诊断价值:
        //   它拿 @1304 的力模长与运动加速度比一个标量, 若与 3×3 模型的 |det A|^(1/3) 相差
        //   很远, 说明其中有一步不对。照实打出来, 但【不写进任何东西】。
        //   (本任务不动 MOTION/SOLVE 两相的结构; 把这两相连同残余时代一起删掉是后续的事。)
        {
            double A[9], cS[3];
            ForceCompensation::currentModel(A, cS);
            const double modelMass = ForceCompensation::currentMassKg();
            if (modelMass > 0.0) {
                printf("[Force] 对照: 全量模型的质量尺度 |det A|^(1/3) = %.4f kg "
                       "(差值 %+.4f kg; 上面那个【不参与补偿】)\n",
                       modelMass, g_massKg - modelMass);
            } else {
                printf("[Force] WARNING: 全量模型为空 (A 全 0) —— 本次只更新零偏, "
                       "重力项仍为空。\n"
                       "        按 'm' 采多姿态 + 's' 解出 A 之后, 补偿才有重力那一项。\n");
            }
        }

        // Apply results: 只更新零偏, 全量模型 (A / c_s) 原样保留。
        double A_keep[9], cS_keep[3];
        ForceCompensation::currentModel(A_keep, cS_keep);
        ForceCompensation::setCalibration(A_keep, g_biasForce, g_biasTorque, cS_keep);
        ForceCalibration::saveToFile(CalibStore::fileFor("force_calib.json"),
                                     A_keep, g_biasForce, g_biasTorque, cS_keep);

        printf("[Force] Calibration complete! bias=(%+.3f,%+.3f,%+.3f)N  "
               "(A / c_s kept unchanged)\n",
               g_biasForce[0], g_biasForce[1], g_biasForce[2]);
        g_state = State::DONE;
        break;
    }

    default:
        break;
    }

    return (g_state == State::DONE || g_state == State::ABORTED);
}

// ===== Persistence =====

// ===== 落盘格式 (version 3, 2026-09-19 Task 6) =====
//
// 存全量模型的四个参数块。为什么【不】留一个 "mass_kg" 字段: 全量模型的参数表里没有
// 标量质量 —— 它由 A 分解出来 (m = |det A|^(1/3))。多写一个字段就等于多一份可以与 A
// 漂开的副本, 而"两份一旦漂移就是安静地解错"是本项目栽过多次的一类。
//
// ⚠ 格式变了, 所以 version 从 2 递增到 3, 而【旧文件被拒】—— 这是有意的, 不是不兼容的
//   副作用: 旧文件里是 {mass_kg, b_F, b_M}, 拿它当新格式读会得到 A = 全 0 而零偏照读,
//   于是补偿里【没有重力项】却看起来一切正常 (读数依旧是 N, 不会爆掉) —— 正是本项目最
//   怕的那种"安静地错"。拒掉它, 并且【响亮地说出来】(见 loadFromFile)。
bool saveToFile(const char* path, const double A[9], const double biasForce[3],
                const double biasTorque[3], const double comSensor[3])
{
    FILE* f = fopen(path, "w");
    if (!f) return false;
    fprintf(f, "{\n");
    fprintf(f, "  \"version\": 3,\n");
    fprintf(f, "  \"a_matrix\": [%.9g, %.9g, %.9g, %.9g, %.9g, %.9g, %.9g, %.9g, %.9g],\n",
            A[0], A[1], A[2], A[3], A[4], A[5], A[6], A[7], A[8]);
    fprintf(f, "  \"bias_force_n\": [%.6g, %.6g, %.6g],\n",
            biasForce[0], biasForce[1], biasForce[2]);
    fprintf(f, "  \"bias_torque_nm\": [%.6g, %.6g, %.6g],\n",
            biasTorque[0], biasTorque[1], biasTorque[2]);
    fprintf(f, "  \"com_sensor_m\": [%.9g, %.9g, %.9g]\n",
            comSensor[0], comSensor[1], comSensor[2]);
    fprintf(f, "}\n");
    fclose(f);
    return true;
}

// 读一个长度为 n 的浮点数组 (jsonFind 已定位到 '[' 之后的第一个字符)。
static bool jsonReadArray(const char* p, double* out, int n) {
    if (!p) return false;
    if (*p == '[') p++;
    for (int i = 0; i < n; i++) {
        char* end = nullptr;
        out[i] = strtod(p, &end);
        if (end == p) return false;
        p = end;
        while (*p == ',' || *p == ' ' || *p == '\n' || *p == '\r' || *p == '\t' || *p == ']') p++;
    }
    return true;
}

// 响亮地说出"这份文件不是本格式" —— 不许安静地退化成"没有标定"。
// 为什么必须响: 静默拒绝与静默接受【在控制台上长得一样】(两种情况下都只剩一条
// "无可用 force_calib.json"), 而它们要做的事完全不同 —— 前者是"去按 'z'", 后者是
// "去按 'm'+'s' 重标模型"。这条消息就是这两者的分界, 所以它必须指名道姓地说出
// 看到了什么、该做什么。
static void rejectOldFormat(const char* path, const char* why) {
    fprintf(stderr,
            "[Force] !! force_calib.json 【格式不兼容, 已拒绝】: %s\n"
            "[Force] !!   文件: %s\n"
            "[Force] !!   本版 (Task 6 起) 期望 version=3 的全量模型: "
            "a_matrix(9) + bias_force_n(3) + bias_torque_nm(3) + com_sensor_m(3)。\n"
            "[Force] !!   旧格式 (version 2: mass_kg + 零偏) 【不能】拿新版读 —— "
            "它没有 A, 读进来会得到一份【没有重力项】的模型,\n"
            "[Force] !!   而补偿后的读数依旧是 N, 不会报错 —— 那正是本项目最怕的\"安静地错\"。\n"
            "[Force] !!   处理: 本地补偿【未启用】。先按 'm' 采多姿态 → 's' 解出 A, "
            "再按 'z' 调零存盘。\n",
            why, (path && *path) ? path : "(null)");
    fflush(stderr);
}

bool loadFromFile(const char* path, double A[9], double biasForce[3],
                  double biasTorque[3], double comSensor[3])
{
    FILE* f = fopen(path, "r");
    if (!f) return false;   // 文件不存在 = 还没有标定过, 这是正常路径, 不吵

    char buf[8192];
    size_t n = fread(buf, 1, sizeof(buf) - 1, f);
    fclose(f);
    if (n == 0) return false;
    buf[n] = '\0';

    // ===== 先把版本号判掉, 再读任何一个参数 =====
    // 顺序是要紧的: 先读参数再判版本, 就会在返回 false 之前把半份数据写进调用方的数组里。
    const char* pv = JsonLite::find(buf, "\"version\"");
    if (!pv) {
        rejectOldFormat(path, "文件里没有 version 字段 (是 version 1 的旧文件, 或根本不是本文件)");
        return false;
    }
    const long ver = strtol(pv, nullptr, 10);
    if (ver != 3) {
        char why[128];
        snprintf(why, sizeof(why),
                 "version=%ld (本版要 3; version 2 是旧的 mass_kg + 零偏格式)", ver);
        rejectOldFormat(path, why);
        return false;
    }

    const char* p = JsonLite::find(buf, "\"a_matrix\"");
    if (!jsonReadArray(p, A, 9)) {
        fprintf(stderr, "[Force] !! force_calib.json 写着 version=3, 但 a_matrix 读不出来 —— "
                        "文件被截断或改坏了。本地补偿【未启用】。\n");
        return false;
    }
    p = JsonLite::find(buf, "\"bias_force_n\"");
    if (!jsonReadArray(p, biasForce, 3)) {
        fprintf(stderr, "[Force] !! force_calib.json 的 bias_force_n 读不出来 —— "
                        "文件被截断或改坏了。本地补偿【未启用】。\n");
        return false;
    }
    p = JsonLite::find(buf, "\"bias_torque_nm\"");
    if (!jsonReadArray(p, biasTorque, 3)) {
        fprintf(stderr, "[Force] !! force_calib.json 的 bias_torque_nm 读不出来 —— "
                        "文件被截断或改坏了。本地补偿【未启用】。\n");
        return false;
    }
    p = JsonLite::find(buf, "\"com_sensor_m\"");
    if (!jsonReadArray(p, comSensor, 3)) {
        fprintf(stderr, "[Force] !! force_calib.json 的 com_sensor_m 读不出来 —— "
                        "文件被截断或改坏了。本地补偿【未启用】。\n");
        return false;
    }

    // ===== 数值可用性 (2026-09-19): 四个字段都校验, A 另判退化 =====
    // 为什么【必须】在这里拒: 上面那条 version 判据只看版本号, 于是 "version 3 但 A 全 0"
    // 的文件会被【安静地接受】—— 补偿后的读数依旧是个 N, 不报错, 只是整个重力项没有,
    // 读数随姿态漂。这正是本项目栽过多次的"安静地错"。
    // 判据与 setCalibration 共用 ForceCompensation::modelUsable (同一条判据只许有一份)。
    char why[192];
    if (!ForceCompensation::modelUsable(A, why, sizeof(why))) {
        fprintf(stderr,
                "[Force] !! force_calib.json 【模型不可用, 已拒绝装载】: %s\n"
                "[Force] !!   文件: %s\n"
                "[Force] !!   本地补偿【未启用】(不是\"静默地当作没标定\": 运行时闸门会以\n"
                "[Force] !!   ERR_FORCE_UNCALIBRATED 每帧拒绝传数据, 并在 stderr 上说出这一段原因)。\n"
                "[Force] !!   处理: 按 'm' 采多姿态 (至少 4 个朝向不同的姿态) -> 's' 解出 A, 再 'z' 调零存盘。\n",
                why, (path && *path) ? path : "(null)");
        return false;
    }
    for (int i = 0; i < 3; i++) {
        if (!std::isfinite(biasForce[i])) {
            fprintf(stderr, "[Force] !! force_calib.json 的 bias_force_n[%d] 不是有限数 —— "
                            "本地补偿【未启用】。\n", i);
            return false;
        }
        if (!std::isfinite(biasTorque[i])) {
            fprintf(stderr, "[Force] !! force_calib.json 的 bias_torque_nm[%d] 不是有限数 —— "
                            "本地补偿【未启用】。\n", i);
            return false;
        }
        if (!std::isfinite(comSensor[i])) {
            fprintf(stderr, "[Force] !! force_calib.json 的 com_sensor_m[%d] 不是有限数 —— "
                            "本地补偿【未启用】。\n", i);
            return false;
        }
    }
    return true;
}

} // namespace ForceCalibration
