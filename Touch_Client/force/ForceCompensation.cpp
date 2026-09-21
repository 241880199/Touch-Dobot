#define _USE_MATH_DEFINES
#include "ForceCompensation.h"
#include "../calibration/TcpCalibration.h"
#include "../config/Config.h"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <windows.h>

// ===== Internal state =====
static CRITICAL_SECTION g_calibMutex;
static bool g_mutexInit = false;
static bool g_isCalibrated = false;
static double g_A[9] = {0};            // 全量模型的 3×3 力响应 (kg, row-major)
static double g_comSensor[3] = {0};    // c_s (m, 传感器测量系)
static double g_biasForce[3] = {0};
static double g_biasTorque[3] = {0};
static MotionEstimator g_motion;

// 【本帧的姿态】—— 只为闸门那段打印服务 (2026-09-20 加)。
// 闸门报的那六个数是"残差对姿态的依赖"的读数, 而【没有姿态就没法解释它们】: 现场抄数的人
// 不把姿态一起抄下来, 事后就分不开"随姿态变"与"固定偏置", 也拟合不了 M = 残差/g 的各向同性
// (判 H1 还是 H2)。出处: Docs/superpowers/specs/2026-09-20-raw-channel-calibration-run-004.md §4.3。
// ⚠ 用【帧里那一份】姿态 (与算这六个数用的是同一次 30004 帧), 不在这里再问一次 GetPose ——
//   后者会在机械臂动过之后与那六个数对不上, 而"对不上"正是本项目最忌的那种安静地错。
// ⚠ 只在 step() 里写、只在 setGuardState() 里读 —— 两者同线程 (setGuardState 由 step 调),
//   所以不需要锁。
static double g_lastPose[6] = {0};   // {X_mm, Y_mm, Z_mm, Rx_deg, Ry_deg, Rz_deg}
// ⚠ 【有没有"本帧"】。setGuardState 不只在 step() 里被调 —— setCalibration 的拒收路径也调它,
//   而那时根本没有"本帧姿态"。没有这个标志就会打出一行全零, 而它会被读成"机械臂在原点姿态"
//   —— 那是凭空造了一个数。没有就照实说没有。
static bool g_lastPoseValid = false;

// 【参考量可用性的本帧证据】—— 同上, 只为闸门那段打印服务 (2026-09-21, Task 7)。
// 要说清楚"为什么不可用" (六维力在线状态是多少、帧是不是陈旧), 而 setGuardState 看不到
// ForceData (它不是按 fd 传参的)。所以 step() 在判可用性之前把这两个数记下来 —— 状态【跃迁】
// 那次打印与 5 s 复报那一行都要用它们。
// ⚠ 与 g_lastPose 同一套规矩: 没有"本帧"就【不许】把残留值/默认值当成本帧的事实打出来。
//   ★ 这两个数【不需要】再配一个"是不是本帧"的标志 (2026-09-21 复审去掉了一个, 它到不了):
//     进入 REFERENCE_UNAVAILABLE 的 setGuardState 调用【全程序只有一处】, 紧跟在写这两个数
//     的几行之后 (step() 第 7b 步); 而 resetGuard() 把这两个数复位时, 状态同时被置回
//     UNCALIBRATED ⇒ "本状态成立"与"没有本帧"不会同时发生, 所以打出来的【永远】是触发本状态
//     的那一帧的读数。默认值本身的含义也正是"一帧都没收到" (-1 / 陈旧), 不是"未知"。
static int  g_refOnlineLast = -1;       // @1037 六维力在线状态的最近值 (-1 = 一帧都没收到)
static bool g_refStaleLast  = true;     // 最近一帧的 fd.isStale

// ===== 运行时一致性闸门的状态 (2026-09-19) =====
// 全部由 ForceReader/pollForce 线程访问 (step() 是唯一入口), 与 g_A 那些用 g_calibMutex
// 保护的量不同 —— 这里不加锁, 与 g_motion 同理: 只有一个写者。
static double g_guardEma[6]  = {0};      // compensated − 【参考量】的逐通道 EMA (判据)

// ===== 闸门判据的参考量: 全程序【唯一一份】定义 =====
//
// 厂商 30004 布局:
//   @576 ActualTCPForce = "TCP【传感器】力值"      -> fd.raw[]
//   @720 TCPForce       = "TCP力值(【通过关节电流计算】)" -> fd.tcpForce[]
// 只有 @720 反映【控制器正在用的负载参数】(算它必须知道负载: 重力矩 + 惯量矩)。
//
// ⚠ 参考量本身也要被验证 (本项目记过: 一个通道"叫什么名字"不等于"它是什么")。
//   实测依据: run-004 §4.5 —— @576 的重力系数跨轮次纹丝不动 (改负载 0.404->0.422 后
//   仍是 0.206/0.208/0.211), 即它对"负载有没有被采纳"完全不响应。
// ⚠ 打印端【不许】再抄一遍 @576 / @720 的字面量: 那份文字会在改参考量时撒谎。
//   这条注释就是判据唯一的出处。
static inline double guardReferenceValue(const AppState::ForceData& fd, int ch) {
    return fd.tcpForce[ch];
}

// ===== 参考量【可用不可用】: 全程序【唯一一份】定义 (2026-09-21, Task 7) =====
//
// 为什么要加它 (用户 2026-09-21 指令): 判据是 `compensated − 参考量`。参考量读到 ~0 时,
//   判据退化成"本地输出是否在自己的容差内" —— 而按构造它总是在 ⇒ 闸门在【一个不携带信息
//   的通道上放行】。"零"既可能是"真的没有外力", 也可能是"这一路没有数据 / 已失效",
//   两者从前【不可区分】⇒ 对"没有信息"放行 = fail-open。所以: 不可用【不放行】。
//
// ⚠ 【防紧】性质, 不是修一个正在发生的 bug: 生产链路上 RelayCore 在【同一次 30004 收帧、
//   同一把 forceDataMutex】里一起填 raw[] / tcpForce[] / sixForceRaw[] / sixForceOnline
//   ⇒ "通道其实有数但读数为零"在【实机目前不可达】。它只在【回放 / 夹具】路径出现
//   (tests/fixtures 的四份采集没有参考量那一路的列 ⇒ 判据那一侧恒为 0)。
//
// 判据【只用既有信号, 不新造门限】(依据在这里, 别处不许再定义一份):
//   (1) fd.sixForceOnline —— 30004 帧 @1037「六维力在线状态」, 机械臂【自己】说的。
//       实机实测值 = 1 (采集记录: 四份夹具文件头 `sixForceOnline=1`, 2026-09-19/20 多次);
//       ForceData 的初值 −1 = 【一帧都还没收到】(RelayCore 每收到一帧就写 buf[1037])。
//       ⇒ 只有 == 1 才算"这一路在线"。0 (机械臂自报不在线) 与 −1 (根本没有帧) 都不算。
//       取"正向确认"而不是"没说不在线就算在线": 无法确认时【不放行】才是 fail-closed,
//       而实机实测值就是 1 ⇒ 这不会把正常工况判成不可用。
//   (2) fd.isStale —— 既有超时常量 Config::FORCE_STALE_MS 的落点 (RelayCore::pollForce
//       用 lastUpdateMs 与它算出这个标志, 同一把锁内、就在 step() 之前)。
//       陈旧帧里的参考量是【上一次读数】, 不是"这一路的当前状态" ⇒ 同样不可用。
//       ⚠ 这里【读这个标志】而不是自己再算一遍帧龄: 帧龄的算法只有一份 (RelayCore 那处),
//         库里再算一份就会出现"闸门说新鲜、F| 组帧说陈旧"这种两个答案的场面。
//   ⇒ 两个信号都是布尔/枚举级的事实, 【不需要任何新的数值门限】—— 这也是本判据不许
//     凭感觉取一个"零附近多大算零"的原因: 那种门限会把"真的没有外力"判成不可用。
static inline bool guardReferenceAvailable(const AppState::ForceData& fd) {
    return !fd.isStale && (fd.sixForceOnline == 1);
}

// ===== 第二组 EMA: compensated − @576 (2026-09-20 并排报出 / 2026-09-21 角色互换) =====
// 它原来是"compensated − @720", 与判据那一组【并排报出】, 用来裁决参考量该是谁
//   (若某一路的差显著更小, 那一路才配当参考量)。裁决已有结论 —— 见上面
//   guardReferenceValue 处的出处。⇒ 这一组就换到【判据原来用的那一路】(@576, 传感器侧)
//   上, 于是"另一路差多少"照样随时看得见, 而判据不再看它。
// ⚠ 【只报不判】的定位一个字没变: 它不参与任何容差比较, 只在拒绝时跟着打出来。
// ⚠ 名字里【不许】再带 720 三个字: 它现在的含义与 720 正好相反, 留着就是埋雷。
static double g_guardEmaDiag[6] = {0};   // compensated − @576 的逐通道 EMA (只报不判)
static bool   g_guardSeededDiag = false; // 上面那一组的播种标志
// 逐通道容差。⚠ 【在静态初始化时就填好】, 不留"init() 没跑就是 0"的空档 ——
// 容差为 0 时 |EMA| > 0 都成立, 判决会退化, 而"退化"的方向必须是【拒绝】而不是放行。
static double g_guardTol[6]  = { Config::FORCE_GUARD_TOL_FORCE_N,  Config::FORCE_GUARD_TOL_FORCE_N,
                                 Config::FORCE_GUARD_TOL_FORCE_N,  Config::FORCE_GUARD_TOL_MOMENT_NM,
                                 Config::FORCE_GUARD_TOL_MOMENT_NM, Config::FORCE_GUARD_TOL_MOMENT_NM };
static bool   g_guardSeeded  = false;    // EMA 是否已用第一帧播种
static long   g_guardFrames  = 0;
static ForceCompensation::GuardState g_guardState = ForceCompensation::GuardState::UNCALIBRATED;
// 上一次【出声】(整块或复报那一行) 的时刻。⚠ 它只服务【复报】的节流, 不节流整块 ——
// 状态一变就打整块, 所以不需要记"上一次打整块是什么时候"。本波曾加过那样一个变量
// (拿 0 当"还没打过"的哨兵, 而 0 也是 GetTickCount 的合法值), 配上一条"被节流掉的整块
// 会在间隔到点后补出"的承诺 —— 补出的那条路没有实现过, 已随整块的节流一起删掉,
// 理由见 setGuardState 头上那一段。
static DWORD  g_guardReportMs = 0;

// ===== 【哪些通道参与判决】—— 本数组是掩码的【唯一一份实现】 =====
//   index:      0    1    2     3     4     5
//             Fx   Fy   Fz    Mx    My    Mz
// ⚠ 头文件里 GuardReport::voted 的类内初值【不是】掩码的第二份实现: 它一律 false, 语义是
//   "尚未填充" (给一份默认构造的报告一个不会撒谎的初值 —— 它引不到这里的 static, 任何
//   字面量都会漂)。真值由 guardReport() 从本数组填入。
// ⚠ 测试里【也不许再抄一份字面量】: 要判掩码就读 guardReport().voted (生产 API 里的那一份)。
//
// ----- Fz 【不投票】----- 依据与它的【现状标记】:
//   ⚠ 这条取舍是【参考量还是 @576 的时候】定下的, 依据是那一侧的 z 通道响应实测秩 2
//   (奇异值 [0.212 0.201 0.008], 第三行比另两行小 8~15 倍,
//   Docs/superpowers/plans/2026-09-19-raw-channel-calibration.md:268-274), 它在【我们唯一
//   有的激励 (重力方向)】上不动。一个动不了的对照量既证不了"一致", 也证不了"不一致"。
//   ⚠⚠ 【已复测 (2026-09-21) —— 结论: 那条秩 2 形状【按原样不复现】, 不许读成"已验"】:
//     参考量换到【通过关节电流计算】的那一路之后, 上面那条依据【跟着复测了】, 结果是
//     "第三行比另两行小 8~15 倍"这个形状在新参考量上【不存在】。掩码本身的取舍不在换
//     参考量那次改动内, 但这个前提的现状必须留在这里 (把这段说成"已验"就是让后人照着
//     一个已不复现的前提去改掩码)。
//     ★ 【机制要说准 —— 分子与分母不许混】: 这【不是】"z 行被修好了"。
//       · z 行的【绝对】模【减得少得多, 但不是"没动"】: 旧参考量 `0.0148~0.0201 kg`
//         → 新参考量 `0.0051~0.0164 kg` (其中第 1 轮缩了约 3.7 倍 —— 而那一轮正是
//         下面写着"不单独作数"的那一轮, 所以"z 行没怎么动"不能当三轮平权读);
//       · 【塌下去的是 x/y】: `0.20~0.21 kg` → `0.0075~0.0286 kg`。
//       ⇒ "z 不再被结构性压小"说的是【比值】(第三行 / 另两行), 而比值变小是
//         【由分母 (x/y) 缩小驱动】的, 不是 z 那一路被修好了。
//     ⚠ 定案范围: 第 2、3 轮与三轮合计。第 1 轮那个比值 (0.179) 只比旧参考量的带
//       (0.070~0.094) 高出约 2 倍, 其分子与 se 只差约 2~3σ ⇒ 【它单独不定案】。
//     ⚠ 【依据不复现 ≠ z 该投票】: 要不要把它提为投票通道是另一个决策 —— 那条参考量在
//       【接触状态下】未经验证, 本段只写"旧的依据不再是依据", 掩码一个字没动。
//     证据与复现方法: Docs/superpowers/evidence/2026-09-21-z-gap-report.md (秩复测一节
//     与判决一节) —— 摘要见 Docs/superpowers/specs/2026-09-19-remaining-workflow.md §3.1。
//     残余 (z 方向仍然没有守门) 见 Config.h 里 FORCE_GUARD_TOL_FORCE_N 上方那段。
//   · 让它投票不会让闸门永远通过 (Fx/Fy 在, 现在是 1.7~2.2 N 量级的拒绝);
//   · 却会让闸门【永远拒绝】: 若它对外力也不响应, 则一旦有真实 z 接触, compensated_z
//     有值而对照量_z 恒 ~0 (旧参考量上的实测如此), 差值直接超限 —— 那就是用户明确
//     禁止的"永远不通过"。
//   所以处置是【每次都报出它的比较结果, 但不计票】, 不是"静默跳过"。
//   ⚠ 这一列的证据到此为止: "是 @576 报得坏, 还是机械臂 z 补偿太强" 目前【没有分开】
//     (计划书 :273-274 明说"不许猜")。分开之后【还得在新参考量上复测一遍】, 才谈得上
//     把它提升为投票通道。
//
// ⚠⚠ 【这一列不投票代价有多大 —— 给出数, 不要只说"少一道闸门"】(2026-09-19 复审要求)。
//   力矩门【理论上】能给 z 力当后盾: z 上的模型误差 ΔFz 会经 c_s 叉乘出一个力矩误差
//   Δc × ΔF, 其横向分量量级 = |c_s_横向| · ΔFz。但 c_s 的横向分量实测只有
//   【0.47 ~ 0.78 mm】(四份拟合的 sqrt(cs_x²+cs_y²), 由 test_runtime_consistency_guard_replay
//   现算打印), 而力矩容差是 tol_M = 0.03 N·m ⇒ 若 c_s 向量的横向分量恰好是对的那个方向,
//   ΔFz 要到 tol_M / |c_s_横向| ≈ 0.03 / 0.00078 ~ 0.03 / 0.00047 = 【38 ~ 64 N】量级
//   才能把力矩顶超限。也就是说: z 力方向的模型误差要靠力矩通道兜住, 得大到几十牛 ——
//   本闸门实际上【看不见 z 方向的力模型错误】。这是本次改动里【最大的一处已知漏洞】,
//   明确交给用户定夺 (是补一个 z 的独立判据, 还是接受这个洞), 不是可以靠调容差解决的。
//   (数字来源: 同一份测试打印的两列 —— 逐份 |c_s_横向| 与 tol_M/|c_s_横向|。)
//   ⚠ 2026-09-21 补充: 上面那个"力矩兜得住 z"的假设【现在连假设都不是了】—— 力矩三个
//     分量已经【不投票】(理由见下面那一段), 所以它连"顶超限"这一步都不再参与判决。
//     数字一个都没变 (它是"力矩门若有, 要多大的 z 差才看得见"), 变的只是: 现在【没有】
//     这道门。⇒ 几十牛那个洞【比原来更大了一点】, 开放项 C 的处置因此更紧, 不是更松。
static const bool g_guardVote[6] = { true, true, false, false, false, false };

// ----- 力矩三个分量 【不投票】(2026-09-21 起) —— 依据与【代价】 -----
//   实测 (run-004 §4.5): 参考量换成"通过关节电流计算"的那一路 (@720) 之后, 力矩仍然差
//   Mx +0.061 / My +1.147 / Mz +1.316, 而声明的力矩容差是 0.03。
//   该差距【不是负载效应】: 约 90% 是姿态无关的偏置, 而负载误差产生的残差【必然随姿态变】;
//   且该偏置【在漂】(Mx 20 分钟漂 0.55 N·m) ⇒ 既改不动、也不能标定掉。
//   同姿态秒级复采的重复性本身就有 0.06~0.13 N·m, 比 0.03 的容差还大 2~4 倍 ——
//   连"重复性"这一关都过不去, 更谈不上"一致"。
//   => 力矩与参考量之间【不存在"一致"态】, 强行投票会让整个闸门永远拒绝,
//      而判决是【全或无】-> compensated[] 全置零 -> 【力通道也一起断】。
//      触觉那一条路只消费【三个力分量】(Touch 反射力由 compensated 的前三个推),
//      所以力矩投票一票【换不到任何东西】, 只换来力通道的死。
//      ⚠ 注意【只有触觉那一路】是这样: F| 帧【六个分量全带】(RelayCore 取 filtered[0..5]),
//        力矩照样由 compensated 出门。把上面那句读成"MATLAB 收不到力矩"就会得出
//        "力矩那一段可以删"的错误结论 —— 而它是质心与惯量的唯一测量窗口 (见下)。
//   ⚠ 力矩【不是被删掉】: 照算、照报、照给 MATLAB 的 F| 帧 —— guardReport() 的 ema[]、
//     拒绝时那张逐通道表、复报行里都还在。它仍是【质心与惯量的唯一测量窗口】
//     (负载的 I 只能从力矩通道辨识), 只是不再投票。
//   ⚠ 代价【必须记账 —— 写在这里是为了后人不要改回去】: Fz 早已不投票, 力矩原本【在名义上】
//     兜着 A 的第三行 -> 现在无人兜。上面那段"几十牛"的洞因此【没有任何替补】: 若日后要
//     把 z 方向的力模型错误管起来, 只能补一个【独立的 z 判据】(开放项 C), 不能靠"把力矩
//     的票加回来" —— 那一步已经被实测否掉了 (它连重复性都不够)。

// A 的可用性判据 —— 见头文件声明。|det| / ||A||_F³ 对"标量质量 × 正交"这一族恒为
// 1/(3√3) = 0.19245, 而秩亏时趋于 0。
// ⚠ 取 1e-3 是【很松】的一道: 它挡的是"几乎完全秩亏", 不是"一般病态"。
//   把门限翻译成条件数 (cond = σ1/σ3, 同一个 cond 下比值最大的形状是 σ1=σ2=σ3·cond):
//   比值 = 1/(2.828·cond) ⇒ 1e-3 对应的 cond ≈ 354。也就是说: 只有当 cond ≳ 354
//   才【必然】被拦下; cond 在 350 以内的矩阵里总有一些形状能过, 无论它多病态。
//   用户指令 3 真正点名的"全零 A"由上面那条 allZero 分支挡住 (与这个比值无关),
//   所以这里不必收紧; 但【消息里不许再说"至少一个力方向没有模型"】: 那句话描述的是秩亏,
//   而这条判据拦的是"离秩亏还差三个数量级"的东西。
static const double GUARD_MIN_DET_RATIO = 1e-3;

// 质量尺度 m = |det A|^(1/3)。A = m·S·Q (见 PayloadCalibration::decompose), 所以它的
// 三个奇异值的几何平均恰好是 m —— 而几何平均 = |det|^(1/3), 不需要 SVD。
// 就是 decompose() 报的 massScale, 这里现算一份的理由见头文件 currentMassKg。
static double massScaleOf(const double A[9]) {
    const double det = A[0] * (A[4] * A[8] - A[5] * A[7])
                     - A[1] * (A[3] * A[8] - A[5] * A[6])
                     + A[2] * (A[3] * A[7] - A[4] * A[6]);
    return cbrt(fabs(det));
}

// ===== Euler angles (deg) to rotation matrix =====
// ⚠ 本文件里【当前没有调用方】了: 重力的唯一去处已改成 TcpCalibration::gravitySensorFrameAtYaw
//   (约定只能有一份实现)。此函数与下面的 matTransposeMulVec 保留未删 —— 删不删由所有者定;
//   但【不要】再用它们在这里重新展开一遍重力, 那正是本文件以前和求解器各写一份的老毛病。
// R = Rz(rz_deg) * Ry(ry_deg) * Rx(rx_deg)
// Output: 3x3 row-major R[9]
static void eulerToRotation(double rx_deg, double ry_deg, double rz_deg, double R[9]) {
    double rx = rx_deg * M_PI / 180.0;
    double ry = ry_deg * M_PI / 180.0;
    double rz = rz_deg * M_PI / 180.0;

    double cx = cos(rx), sx = sin(rx);
    double cy = cos(ry), sy = sin(ry);
    double cz = cos(rz), sz = sin(rz);

    // Rz * Ry * Rx  (row-major)
    R[0] = cz * cy;
    R[1] = cz * sy * sx - sz * cx;
    R[2] = cz * sy * cx + sz * sx;
    R[3] = sz * cy;
    R[4] = sz * sy * sx + cz * cx;
    R[5] = sz * sy * cx - cz * sx;
    R[6] = -sy;
    R[7] = cy * sx;
    R[8] = cy * cx;
}

// Matrix-vector multiply: out = M^T * v  (3x3 row-major M, 3-vector v)
// ⚠ 同 eulerToRotation: 当前无调用方 (重力的唯一去处已改成共享函数), 保留未删。
static void matTransposeMulVec(const double M[9], const double v[3], double out[3]) {
    out[0] = M[0] * v[0] + M[3] * v[1] + M[6] * v[2];
    out[1] = M[1] * v[0] + M[4] * v[1] + M[7] * v[2];
    out[2] = M[2] * v[0] + M[5] * v[1] + M[8] * v[2];
}

// Cross product: out = a x b
static void cross(const double a[3], const double b[3], double out[3]) {
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

// ===== MotionEstimator implementation =====

// Butterworth2 LPF coefficient helper (same as ForcePipeline pattern)
static void calcLpfCoeffs(double fc, double fs,
    double& b0, double& b1, double& b2, double& a1, double& a2)
{
    double w0 = 2.0 * M_PI * fc / fs;
    double cos_w0 = cos(w0);
    double sin_w0 = sin(w0);
    double alpha = sin_w0 / sqrt(2.0);
    double a0 = 1.0 + alpha;
    b0 = ((1.0 - cos_w0) / 2.0) / a0;
    b1 = (1.0 - cos_w0) / a0;
    b2 = ((1.0 - cos_w0) / 2.0) / a0;
    a1 = (-2.0 * cos_w0) / a0;
    a2 = (1.0 - alpha) / a0;
}

MotionEstimator::MotionEstimator() : m_idx(0), m_count(0) {
    m_vel[0] = m_vel[1] = m_vel[2] = 0.0;
    m_accRaw[0] = m_accRaw[1] = m_accRaw[2] = 0.0;
    m_accFiltered[0] = m_accFiltered[1] = m_accFiltered[2] = 0.0;
    for (int i = 0; i < BUF_SIZE; i++) {
        m_posBuf[i][0] = m_posBuf[i][1] = m_posBuf[i][2] = 0.0;
    }
    for (int i = 0; i < 3; i++) {
        m_lpfX1[i] = m_lpfX2[i] = m_lpfY1[i] = m_lpfY2[i] = 0.0;
    }
    // 10Hz LPF at effective sample rate
    double fs = static_cast<double>(Config::FORCE_EFFECTIVE_SAMPLE_RATE);
    calcLpfCoeffs(Config::FORCE_ACC_FILTER_CUTOFF_HZ, fs,
        m_lpfB0, m_lpfB1, m_lpfB2, m_lpfA1, m_lpfA2);
}

void MotionEstimator::reset() {
    m_idx = 0; m_count = 0;
    m_vel[0] = m_vel[1] = m_vel[2] = 0.0;
    m_accRaw[0] = m_accRaw[1] = m_accRaw[2] = 0.0;
    m_accFiltered[0] = m_accFiltered[1] = m_accFiltered[2] = 0.0;
    for (int i = 0; i < BUF_SIZE; i++)
        m_posBuf[i][0] = m_posBuf[i][1] = m_posBuf[i][2] = 0.0;
    for (int i = 0; i < 3; i++)
        m_lpfX1[i] = m_lpfX2[i] = m_lpfY1[i] = m_lpfY2[i] = 0.0;
}

void MotionEstimator::update(double x, double y, double z, double dt) {
    // Store in ring buffer (unit: m)
    m_posBuf[m_idx][0] = x * 0.001;  // mm -> m
    m_posBuf[m_idx][1] = y * 0.001;
    m_posBuf[m_idx][2] = z * 0.001;
    m_idx = (m_idx + 1) % BUF_SIZE;
    if (m_count < BUF_SIZE) m_count++;

    if (m_count >= 3) {
        // Central difference velocity (using indices i and i-1)
        int i0 = (m_idx - 1 + BUF_SIZE) % BUF_SIZE;
        int i1 = (m_idx - 2 + BUF_SIZE) % BUF_SIZE;
        for (int k = 0; k < 3; k++) {
            m_vel[k] = (m_posBuf[i0][k] - m_posBuf[i1][k]) / dt;
        }
    }
    if (m_count >= 5) {
        // Central difference acceleration (3-point stencil)
        int i0 = (m_idx - 1 + BUF_SIZE) % BUF_SIZE;
        int i1 = (m_idx - 2 + BUF_SIZE) % BUF_SIZE;
        int i2 = (m_idx - 3 + BUF_SIZE) % BUF_SIZE;
        for (int k = 0; k < 3; k++) {
            m_accRaw[k] = (m_posBuf[i0][k] - 2.0 * m_posBuf[i1][k] + m_posBuf[i2][k]) / (dt * dt);
            // NaN guard
            if (std::isnan(m_accRaw[k]) || std::isinf(m_accRaw[k])) m_accRaw[k] = 0.0;
            // LPF: biquad step per channel
            double out = m_lpfB0 * m_accRaw[k] + m_lpfB1 * m_lpfX1[k] + m_lpfB2 * m_lpfX2[k]
                       - m_lpfA1 * m_lpfY1[k] - m_lpfA2 * m_lpfY2[k];
            m_lpfX2[k] = m_lpfX1[k]; m_lpfX1[k] = m_accRaw[k];
            m_lpfY2[k] = m_lpfY1[k]; m_lpfY1[k] = out;
            m_accFiltered[k] = out;
        }
    }
}

void MotionEstimator::getState(double vel[3], double acc[3]) const {
    for (int k = 0; k < 3; k++) {
        vel[k] = m_vel[k];
        acc[k] = m_accFiltered[k];
    }
}

bool MotionEstimator::isStill() const {
    double vsq = m_vel[0]*m_vel[0] + m_vel[1]*m_vel[1] + m_vel[2]*m_vel[2];
    double asq = m_accFiltered[0]*m_accFiltered[0] + m_accFiltered[1]*m_accFiltered[1] + m_accFiltered[2]*m_accFiltered[2];
    return (sqrt(vsq) < Config::FORCE_MOTION_VEL_THRESH_MS &&
            sqrt(asq) < Config::FORCE_MOTION_ACC_THRESH_MSS);
}

// ===== 闸门的报告 =====

// 【一行式】的拒绝读数 —— 状态【没有变】、且距上次出声已过一个 FORCE_GUARD_REPORT_MS 时走这一条。
// ⚠【跃迁不走这里】(2026-09-21 收口 Fix 1): 状态一变就打整块 (原因 + 处置 + 本帧姿态 +
//   逐通道表, 见 setGuardState)。本波一度让跃迁被节流时改用这一行, 并承诺"整块到点补出" ——
//   那个承诺没有实现: 补出需要有"还欠着一块"的状态, 而代码里没有它。后果是【在节流窗口里
//   进入的状态】只剩这一行: 逐通道的"超限 <== 触发"标记、本帧姿态、成对的处置指引全都看不到。
//   ⇒ 节流只用于复报, 本函数也只有这一种用法。
// ⚠ 它是【一行】: 只带状态 + 原因一句话 + (INCONSISTENT 时) 六个通道的两路读数。
// ⚠ 三种原因的措辞【不许合并】: 处置各不相同 (去标定 / 去查下发 / 去查这一路的数据),
//   而这一行往往是操作员【第一眼】看到的东西。
static void printCompactRefusal(ForceCompensation::GuardState st) {
    static const char* NM[6] = { "Fx(N)", "Fy(N)", "Fz(N)", "Mx(Nm)", "My(Nm)", "Mz(Nm)" };

    if (st == ForceCompensation::GuardState::UNCALIBRATED) {
        // 配置态: 原因一句话说得完 (逐通道表与它无关 —— 那时 compensated 恒为 0)。
        // ⚠ 本支【到不了】(按状态穷举的写法): setGuardState 在"状态没变 + UNCALIBRATED"
        //   时已经先返回, 而跃迁那一支不调本函数。真走到这里时, 该说的话就是下面这句 ——
        //   所以留着它, 但【不假装它是一条活路】。
        fprintf(stderr, "[Force] !! 仍在拒绝: 【没有可用模型】—— 本地补偿未启用,"
                        " 不是\"标定与机械臂不符\"。\n"
                        "[Force] !!   处理: 按 'm' 采多姿态 -> 's' 解出 A, 再按 'z' 调零。\n");
        fflush(stderr);
        return;
    }
    if (st == ForceCompensation::GuardState::REFERENCE_UNAVAILABLE) {
        // ★ 这一路【不能】用下面那行逐通道读数: 那些差是拿"参考量"当被减数算出来的, 而这一路
        //   【没有数据】⇒ 那六个数不是任何一次比较的结果。所以这行只说状态 + 为什么。
        // ⚠ 这两个数【一定是本帧的】: 能进入本状态的 setGuardState 调用全程序只有一处, 就在
        //   写下这两个数的那几行下面 (step() 第 7b 步); 而 resetGuard() 把它们复位时状态同时
        //   被置回 UNCALIBRATED ⇒ "本状态成立"与"没有本帧"不会同时发生。
        fprintf(stderr, "[Force] !! 仍在拒绝: 参考量不可用 —— 判据那一侧没有数据"
                        " (六维力在线状态 @1037 = %d, 只有 1 算在线; 帧陈旧 = %d)。"
                        "逐通道对比表不适用: 没有第二个读数, 没有比过。\n"
                        "[Force] !!   处理: 把参考量这一路的数据找回来 (30004 帧 / 六维力在线状态),"
                        " 然后等它恢复 —— 本闸门会自动放行。\n",
                g_refOnlineLast, g_refStaleLast ? 1 : 0);
        fflush(stderr);
        return;
    }

    // INCONSISTENT: 那一行"判据差 / 诊断差"并排读数就是这条复报要带的全部增量。
    // ⚠ 前一半是【判据】的差 (对参考量), 后一半是【诊断】的差 (对 @576) —— 谁是谁由
    //   guardReferenceValue 那一处定义决定, 这里不许再抄通道号。
    char line[384];
    int off = snprintf(line, sizeof(line),
                       "[Force] !! 仍在拒绝  [判据(与参考量) / 诊断(与 @576)]:");
    if (off < 0) off = 0;   // snprintf 可返回负值; 不管的话下面 (size_t)off 会回绕
    for (int i = 0; i < 6; i++) {
        if ((size_t)off + 40 >= sizeof(line)) break;   // 余量不足就停, 不越界
        const int w = snprintf(line + off, sizeof(line) - (size_t)off, " %s%+.3f/%+.4f",
                               NM[i], g_guardEma[i], g_guardEmaDiag[i]);
        if (w > 0) off += w;
    }
    if ((size_t)off < sizeof(line))
        snprintf(line + off, sizeof(line) - (size_t)off, "\n");
    fprintf(stderr, "%s", line);
    fflush(stderr);
}

// 闸门状态迁移 + 响亮地报出【逐通道】的比较结果。
// 只在【状态变化】时立刻打印; 状态不变时按 FORCE_GUARD_REPORT_MS 复报一次 ——
// 闸门每帧都判 (30Hz), 每帧都印会把控制台冲掉, 而"看不过来"与"没报"在操作上是一回事。
// ⚠ 复报【只对"拒绝"那一侧】: 放行是常态, 每 5 s 印一行"放行"同样是噪音
//   (而且会把真正要紧的那段挤出可视区)。放行只在它【刚刚恢复】时印一次。
// ★ 2026-09-21 收口 (最终复审 Fix 1): 【跃迁一律打整块; 节流只管复报】。
//   本波一度把跃迁也按同一个间隔节流掉, 并在这里写着"整块到点补出" —— 那句话【没有兑现】:
//   补出需要有"还欠着一块"的状态, 代码里没有它, 于是【在节流窗口里进入的状态】永远只剩
//   一行紧凑读数: 原因/处置、本帧姿态、逐通道的"超限 <== 触发"标记全都看不到 —— 而现场
//   抄数要抄的恰恰是这几样。恢复本波之前的行为, 也是本函数原本的意图: 一变就打整块。
//   ⚠ 代价【如实说】: 状态【来回跳】时 (边缘链路上参考量一会儿有一会儿没), 每跳一次就是
//     一整块 —— 那正是本波当初想压掉的那件事, 现在【不压了】。取舍的理由: 一行紧凑读数
//     放不下上面那几样诊断, 而"理由看得见"比"行数少"要紧。稳态不会被冲屏 —— 状态不变时
//     的复报仍按既有间隔节流, 而拒绝长时间不变才是常态。
static void setGuardState(ForceCompensation::GuardState st) {
    const DWORD now = GetTickCount();
    const bool changed = (st != g_guardState);
    g_guardState = st;

    if (st == ForceCompensation::GuardState::OK) {
        if (changed) {
            // ★ 2026-09-21 (Task 7): "放行"现在包含三件事 —— 有模型、参考量【可用】、
            //   且在投票通道上一致。"参考量可用"要写出来: 它是这一行与从前的区别所在
            //   (从前的放行可能发生在【根本没有第二个读数】的时候)。
            fprintf(stderr, "[Force] 一致性闸门: 放行 (本地全量模型与【参考量】在【投票通道】上一致;"
                            " 参考量本帧【可用】)\n");
            fflush(stderr);
        }
        g_guardReportMs = now;
        return;
    }
    const bool uncal = (st == ForceCompensation::GuardState::UNCALIBRATED);
    // ★ 2026-09-21 (Task 7): 有没有【真的比过】。只有 INCONSISTENT 比过 —— UNCALIBRATED
    //   没有模型, REFERENCE_UNAVAILABLE 没有第二个读数。逐通道表只在"比过"时才有意义:
    //   印一张"六个通道全在限内"的表, 就是把"没比过"说成"比过了且没问题"。
    const bool compared = (st == ForceCompensation::GuardState::INCONSISTENT);
    // 【UNCALIBRATED 不在复报之列】
    // 它是【配置态】, 不是【数据态】: "没有模型"这件事不会自己好, 也不随机械臂的动作变,
    // 所以复报出来的那 14 行与上一次【逐字相同】—— 唯一的效果是把别的输出挤出可视区,
    // 而这一屏本来是要在现场读的。实测 2026-09-20: 一次约 95 s 的会话里它出现过 19 次。
    // 启动那一路已经报过 ("无可用 force_calib.json — 按 'z' 调零"), 状态跃迁时这里再报
    // 一次, 就够了。
    // ⚠ INCONSISTENT 【仍然】复报: 它下面那张逐通道表的数据【会变】, 复报带的是新信息 ——
    //   那正是"复报"这个机制原本要服务的情形。
    // ⚠ REFERENCE_UNAVAILABLE 【仍然】复报 (2026-09-21): 它是【数据态】, 会自己好
    //   (帧恢复 / 六维力重新在线), 所以"还在不在这个状态"是要盯的一件事 —— 与
    //   UNCALIBRATED 那种"配置态、复报出来逐字相同"不是一回事。
    if (!changed) {
        if (uncal) return;
        if ((now - g_guardReportMs) < static_cast<DWORD>(Config::FORCE_GUARD_REPORT_MS)) return;
        g_guardReportMs = now;
        // 【复报只打一行】(2026-09-20)。全表只在【状态跃迁】时打。
        // 为什么: 拒绝是常态, 每 5 s 一次那 9 行解释 + 6 行表 + 姿态行会把控制台全冲掉 ——
        //   而现场要读的恰恰是【别的】输出: 's' 的那一屏、'p' 的确认提示、'y' 的逐条回执。
        //   实测代价 (2026-09-20 现场): 因为这条复报, 操作员【看不到 'p' 打了什么】, 于是
        //   无法判定"发送被拒"与"按键根本没收到" —— 一套诊断被彻底淹没。
        //   这与刚被取消的 UNCALIBRATED 复报是【同一个病】: 复报的内容与上次逐字相同。
        // 复报仍然出声 (拒绝没变这件事还得让人看见), 只是不再重抄整块; 哪几个通道超限直接
        //   写在那一行里 —— 那正是复报该带的唯一增量。
        // (走到这里且 !changed 只可能是 INCONSISTENT 或 REFERENCE_UNAVAILABLE:
        //  !changed && uncal 在上面已经 return 了。)
        printCompactRefusal(st);
        return;
    }

    // 状态【变了】: 一律打整块, 【不节流】—— 理由见函数头上那一段。
    g_guardReportMs = now;

    // 逐通道表的标签 (与 printCompactRefusal 里那一份同名同序 —— 两处的下标含义由
    // guardReferenceValue / g_guardVote 那两个唯一定义决定, 各自都不许再抄通道号)。
    static const char* NM[6] = { "Fx(N)", "Fy(N)", "Fz(N)", "Mx(Nm)", "My(Nm)", "Mz(Nm)" };

    // 【三种原因, 三句不同的话】(2026-09-21 Task 7 起; 从前这里是两元的三目运算符)。
    // ⚠ 原因文字与处置指引【必须成对】(见 .h 的三种拒绝原因): 报错了原因而没错处置,
    //   操作员会照着一件不相干的事去忙 —— 那比不报还坏。
    const char* reasonText = nullptr;
    const char* actionText = nullptr;
    // REFERENCE_UNAVAILABLE 的处置文字要带上【本帧刚记下的两个可用性读数】, 所以它得现拼
    // (其余状态的文字都是字面量)。下面那段文字实际约 570 字节, 缓冲区分了三成余量 ——
    // static 缓冲区不够时 snprintf 【静默截断】, 而截掉的正好是末尾那句处置。
    char refAction[768];
    switch (st) {
        case ForceCompensation::GuardState::UNCALIBRATED:
            reasonText = "【没有可用模型】本地补偿未启用 —— 不是\"标定与机械臂不符\"";
            actionText = "[Force] !!   未标定 -> 去标定 ('m' 采多姿态 + 's' 解 A, 再 'z' 调零)。\n";
            break;
        case ForceCompensation::GuardState::REFERENCE_UNAVAILABLE:
            reasonText = "【参考量不可用】判据那一侧【没有数据】—— 不是\"标定与机械臂不符\","
                         " 也不是\"两边对不上\"";
            // 处置【必须与另外两个分开】: 没有第二个读数时, 重标模型与查负载参数这两件事
            // 都没有依据 —— 要做的是把这一路的数据找回来。
            // ★ 2026-09-21 复审: 把下面那两条要查的东西【各自读到几】当场打出来。这两个数
            //   从前【只】出现在 5 s 复报那一行里 (那一行没有用例覆盖), 于是最常见的第一次
            //   拒绝里, "根本没有帧"与"帧到了、但机械臂自报不在线"在操作员眼里【分不开】——
            //   而这两件事要做的处置并不相同。这一行就是"第一眼"能拿到的诊断。
            //   ⚠ 打的是【本帧】的值: 见 g_refOnlineLast 处 —— 进入本状态之前, 这两个数
            //     刚由触发这一状态的那一帧写下, 所以这里不是残留值。
            snprintf(refAction, sizeof(refAction),
                "[Force] !!   参考量这一路没有数据 -> 去查【为什么没有】。本帧这两个数现在是:"
                " 六维力在线状态 @1037 = %d (只有 1 算在线), 帧陈旧 = %d。\n"
                "[Force] !!     · 30004 帧还在不在来 (判据是 fd.isStale / Config::FORCE_STALE_MS);\n"
                "[Force] !!     · 机械臂自报的六维力在线状态 (@1037) 是不是 1 (只有 1 算在线)。\n"
                "[Force] !!   ⚠ 【不要】去重标模型、也【不要】去查负载参数有没有发进去:\n"
                "[Force] !!     那两个动作都以\"存在一个可比的参考读数\"为前提, 而这里没有。\n",
                g_refOnlineLast, g_refStaleLast ? 1 : 0);
            actionText = refAction;
            break;
        case ForceCompensation::GuardState::INCONSISTENT:
            reasonText = "【有模型, 但与机械臂对不上】两边估计的不是同一个外力";
            actionText = "[Force] !!   标定了但对不上 -> 去查负载参数有没有真的发进机械臂 (Task 8)。\n";
            break;
        default:
            // 不该发生 (GuardState 只有上面四个值; OK 在上面已经 return 了)。
            // 但【不许】拿别的状态的话来兜底 —— 那等于替一个不认识的状态撒谎, 而这个项目
            // 记过一笔账: 加枚举值时编译器不会替我们发现漏配 (见 guardErrorCode 段)。
            reasonText = "【未知的闸门状态】—— GuardState 加了新值而这里没配";
            actionText = "[Force] !!   (这个状态没有配处置指引: 它的原因与要做的事都未定义。)\n";
            break;
    }
    // 输出一律走 stderr —— 与 ForceCalibration 的"响亮地说出来"同一条路; stdout 有缓冲,
    // 混着打会让这段在最需要它的时候缺半截。
    fprintf(stderr,
            "[Force] !! ============ 一致性闸门: 拒绝传递数据 ============\n"
            "[Force] !! 原因: %s\n"
            "%s"
            "[Force] !!   【三种原因的处置一样 (都拒绝), 但要做的事不同, 所以原因必须分开报】。\n"
            "[Force] !! compensated[] 已【全 6 个分量置零】 —— 下游 ForcePipeline 由它推\n"
            "[Force] !!   filtered / hapticOut / F| 帧, 所以【传感器力那一条路】断了。\n"
            "[Force] !!   (虚拟约束力【不受影响】: 它由【位置】现算"
            " (SafetyPredictor::computeConstraintForce,\n"
            "[Force] !!    在触觉回调里对当前位置求一次), 与 compensated 无关 ——"
            " 安全边界的推手还在,\n"
            "[Force] !!    只是不再有传感器力。)\n",
            reasonText, actionText);
    // 【把本帧姿态一起打出来】(2026-09-20)。理由见 g_lastPose 的说明: 下面那六个数只能
    //   【连着姿态】才有意义 —— 现场抄数必须一起抄, 否则事后分不开"随姿态变"与"固定偏置",
    //   也拟合不了 M = 残差/g 的各向同性 (判 H1/H2)。出处 run-004 §4.3 的判别判据。
    // ⚠ 只在【拒绝】这一支打 (本函数走到这里的都非 OK): 放行是常态, 每次都给行姿态同样是噪音。
    if (g_lastPoseValid) {
        fprintf(stderr,
                "[Force] !! 本帧姿态: X=%+.3f Y=%+.3f Z=%+.3f  Rx=%+.3f Ry=%+.3f Rz=%+.3f"
                "  (mm / deg) —— 读下面那六个数要连着它一起抄\n",
                g_lastPose[0], g_lastPose[1], g_lastPose[2],
                g_lastPose[3], g_lastPose[4], g_lastPose[5]);
    } else {
        // 【不许打一行 0】: 没有"本帧"时打 0 会被读成"机械臂在原点姿态" —— 凭空造一个数。
        fprintf(stderr, "[Force] !! 本帧姿态: 【没有】—— 这一段不是由某一帧触发的"
                        " (例如装载/拒收路径), 所以没有姿态可报。\n");
    }
    if (!compared) {
        // 【没比过就不打逐通道表】—— 两种状态各有各的"没比过":
        //   · UNCALIBRATED: 那时 compensated 恒为 0, 印出来会是"六个通道全在限内";
        //   · REFERENCE_UNAVAILABLE (Task 7): 被减数那一侧没有数据, 那些差不是比较结果。
        // 两种情形下那张表都只有一个作用: 把"没比过"说成"比过了且没问题"。
        fprintf(stderr, "[Force] !! (%s: 逐通道对比表不适用。上面那一段才是原因。)\n",
                uncal ? "没有模型可比较" : "参考量这一路没有数据, 没有比过");
        if (uncal) {
            fprintf(stderr, "[Force] !! 处理: 先按 'm' 采多姿态 -> 's' 解出 A, 再按 'z' 调零存盘。\n");
        } else {
            // ⚠ 处置【不是】"去重标/去查负载参数" —— 见上面那段 actionText 的理由。
            fprintf(stderr, "[Force] !! 处理: 把参考量这一路的数据找回来 (30004 帧 / 六维力在线状态),"
                            " 然后等它恢复 —— 本闸门会自动放行。\n");
        }
        fflush(stderr);
        return;
    }
    fprintf(stderr, "[Force] !! 逐通道结果 (EMA 差 = compensated − 【参考量】; 单位见各行标签)\n"
                    "[Force] !!   ⚠ 只有【投票通道】参与判决; 标着【不投票】的那四行【照报但不算】\n"
                    "[Force] !!     (依据见 g_guardVote 段: Fz 的秩 2 依据未复测; 力矩无一致态)。\n");
    // 【每行末尾那一列是什么】(2026-09-20 加, 2026-09-21 角色互换): 它原来是"与 @720",
    // 而 @720 现在是判据看的参考量 ⇒ 这一列换成 @576 —— 【只报不判】的诊断侧。
    //   作用: 判据只看得出"对不上", 看不出"是哪一路偏了"; 把另一路并排报出来, 现场才能
    //   一次看全。⚠ 它不参与任何容差比较。
    fprintf(stderr, "[Force] !!   行末的「与 @576」= compensated − @576 —— 只报不判"
                    " (诊断侧, 判据不看它)\n");
    for (int i = 0; i < 6; i++) {
        const bool ex = (g_guardTol[i] > 0.0) && (fabs(g_guardEma[i]) > g_guardTol[i]);
        if (!g_guardVote[i]) {
            // ⚠ 本行【不投票】, 但结果照样印出来 —— "不投票"不等于"不检查、不显示"。
            //   谁不投票【不在这里复述】: 掩码的唯一出处是 g_guardVote 段 (抄一份列在这里
            //   就是第二份实现, 掩码一变它就变成假话)。
            fprintf(stderr, "[Force] !!   %-6s %+10.4f  容差 %.4f  【不投票】"
                            " (照报不判; 依据与代价见 g_guardVote 段)\n",
                    NM[i], g_guardEma[i], g_guardTol[i]);
        } else {
            fprintf(stderr, "[Force] !!   %-6s %+10.4f  容差 %.4f  %-16s └ 与 @576: %+9.4f"
                            " (只报不判)\n",
                    NM[i], g_guardEma[i], g_guardTol[i], ex ? "超限  <== 触发" : "在限内",
                    g_guardEmaDiag[i]);
        }
    }
    // (走到这里只可能是 INCONSISTENT —— 另外两个状态上面已经 return 了)
    fprintf(stderr, "[Force] !! 处理: 查负载参数有没有真的发进机械臂 (Task 8), 或重跑离线一致性检查。\n"
                    "[Force] !!   【不要】靠改容差把它压过去 —— 容差是由实测导出的。\n");
    fflush(stderr);
}

// 把 EMA 复位 —— 换模型/重启之后必须重新采证据, 不能拿旧模型的 EMA 去判新模型。
static void resetGuard() {
    for (int i = 0; i < 6; i++) g_guardEma[i] = 0.0;
    g_guardSeeded = false;
    for (int i = 0; i < 6; i++) g_guardEmaDiag[i] = 0.0;
    g_guardSeededDiag = false;
    g_guardFrames = 0;
    g_guardState  = ForceCompensation::GuardState::UNCALIBRATED;
    // 复报那一行的节流也一起复位 (0 = "还没有过出声时刻", 比较式是 `now - 它 < 间隔`)。
    // ⚠ 整块【不节流】, 所以这里没有"上一次打整块"要复位: 复位之后的第一声拒绝【一定】
    //   打整块 —— 它不依赖任何时刻量。
    g_guardReportMs = 0;
    // 参考量可用性的"本帧证据"一起复位: 复位之后就没有"本帧"了。状态在这里同时被置回
    // UNCALIBRATED ⇒ "参考量不可用"那两处打印不会拿复位后的值当事实 (见 g_refOnlineLast 处)。
    g_refOnlineLast = -1;
    g_refStaleLast  = true;
}

// ===== ForceCompensation namespace =====

namespace ForceCompensation {

void init() {
    if (!g_mutexInit) {
        InitializeCriticalSection(&g_calibMutex);
        g_mutexInit = true;
    }
    g_isCalibrated = false;
    for (int i = 0; i < 9; i++) g_A[i] = 0.0;
    for (int i = 0; i < 3; i++) {
        g_comSensor[i] = 0.0;
        g_biasForce[i] = 0.0;
        g_biasTorque[i] = 0.0;
    }
    g_motion.reset();

    // 闸门: 逐通道容差与状态。容差的【出处】写在 Config.h 的注释里 (实测导出, 不是猜) ——
    // 力通道那一项有两个来源: .superpowers/sdd/runtime-guard-report.md (本地失拟 + 模型类差)
    // 与 Docs/superpowers/specs/2026-09-21-gate-reference-prereq.md (参考量自带的力偏置及其漂移)。
    for (int i = 0; i < 3; i++) {
        g_guardTol[i]     = Config::FORCE_GUARD_TOL_FORCE_N;
        g_guardTol[3 + i] = Config::FORCE_GUARD_TOL_MOMENT_NM;
    }
    resetGuard();
}

void setCalibration(const double A[9], const double biasForce[3],
                    const double biasTorque[3], const double comSensor[3])
{
    // ★ 用户指令 3「全零 A 拒绝传递数据并报错」的落点。
    //   在这里拒掉, 而不是等 step() 每帧去判 —— 那样"已标定"这个状态本身就带着一个
    //   没有重力项的模型, 而补偿后的读数依旧是 N, 不会有任何异常 (本项目最怕的那种
    //   "安静地错")。拒掉之后 g_isCalibrated 保持 false, step() 于是走"没有可用模型"
    //   那条路: 输出置零 + 报错 (ERR_FORCE_UNCALIBRATED)。
    //   ⚠ 现场确实会走到这里: 从未解过 A 时按 'z' 调零, ForceCalibration::update 会拿
    //     currentModel() 的空 A 回灌进来 (那时它自己也已经在报 WARNING)。
    // ⚠ 与装载路径 (ForceCalibration::loadFromFile) 【校验同一组东西】: 那边四个数组都判
    //   有限性, 这里从前只判 A —— 于是"从内存直接装一份带 inf 的零偏"这条不经过文件的路
    //   会静默收下一个 inf, 而 inf 会让 compensated 变 inf 再把闸门的 EMA 污染成 NaN。
    //   (NaN 在闸门里算不一致 -> 拒绝, 但那已经是"用一个坏模型报警", 不如根本不许装进来。)
    for (int i = 0; i < 3; i++) {
        if (!std::isfinite(biasForce[i]) || !std::isfinite(biasTorque[i]) ||
            !std::isfinite(comSensor[i])) {
            fprintf(stderr,
                    "[Force] !! setCalibration 【拒绝安装】: 第 %d 个分量不是有限数 "
                    "(b_F %.6g/%.6g/%.6g, b_M %.6g/%.6g/%.6g, c_s %.6g/%.6g/%.6g)。\n"
                    "[Force] !!   本地补偿保持【未启用】—— 输出置零并报 ERR_FORCE_UNCALIBRATED。\n"
                    "[Force] !!   判据与装载路径 (ForceCalibration::loadFromFile) 完全一致:\n"
                    "[Force] !!   同一个模型不该因为【来自文件】还是【来自内存】而一个收一个不收。\n",
                    i,
                    biasForce[0], biasForce[1], biasForce[2],
                    biasTorque[0], biasTorque[1], biasTorque[2],
                    comSensor[0], comSensor[1], comSensor[2]);
            fflush(stderr);
            EnterCriticalSection(&g_calibMutex);
            g_isCalibrated = false;      // 连旧的也一并作废: 拒绝安装的语义是"现在没有可用模型"
            LeaveCriticalSection(&g_calibMutex);
            resetGuard();
            setGuardState(GuardState::UNCALIBRATED);
            return;
        }
    }

    char why[192];
    if (!modelUsable(A, why, sizeof(why))) {
        fprintf(stderr,
                "[Force] !! setCalibration 【拒绝安装】: %s\n"
                "[Force] !!   本地补偿保持【未启用】—— 输出置零并报 ERR_FORCE_UNCALIBRATED。\n"
                "[Force] !!   一份没有重力项 (或秩亏) 的模型不会报错, 只会安静地算错, 所以不收。\n"
                "[Force] !!   处理: 按 'm' 采多姿态 -> 's' 解出 A (至少 4 个朝向不同的姿态), 再 'z' 调零。\n",
                why);
        fflush(stderr);
        EnterCriticalSection(&g_calibMutex);
        g_isCalibrated = false;      // 连旧的也一并作废: 拒绝安装的语义是"现在没有可用模型"
        LeaveCriticalSection(&g_calibMutex);
        resetGuard();
        setGuardState(GuardState::UNCALIBRATED);
        return;
    }

    EnterCriticalSection(&g_calibMutex);
    for (int i = 0; i < 9; i++) g_A[i] = A[i];
    for (int i = 0; i < 3; i++) {
        g_comSensor[i] = comSensor[i];
        g_biasForce[i] = biasForce[i];
        g_biasTorque[i] = biasTorque[i];
    }
    g_isCalibrated = true;
    LeaveCriticalSection(&g_calibMutex);

    // 换了模型就重新采证据: 拿旧模型的 EMA 去判新模型是错的。
    resetGuard();
}

void currentModel(double A[9], double comSensor[3]) {
    EnterCriticalSection(&g_calibMutex);
    for (int i = 0; i < 9; i++) A[i] = g_A[i];
    for (int i = 0; i < 3; i++) comSensor[i] = g_comSensor[i];
    LeaveCriticalSection(&g_calibMutex);
}

void currentBias(double biasForce[3], double biasTorque[3]) {
    EnterCriticalSection(&g_calibMutex);
    for (int i = 0; i < 3; i++) {
        biasForce[i]  = g_biasForce[i];
        biasTorque[i] = g_biasTorque[i];
    }
    LeaveCriticalSection(&g_calibMutex);
}

// 诊断用: 见头文件里为什么需要它。
// 有意【不】取 g_calibMutex: MotionEstimator 的状态不由它保护, step() 里读它
// (下面的惯性与 EMA 分支) 同样是无锁的, 这里跟着一致即可。
bool motionState(double vel[3], double acc[3]) {
    g_motion.getState(vel, acc);
    return g_motion.isStill();
}

bool isCalibrated() {
    return g_isCalibrated;
}

// A 能不能当【重力模型】用。三个毛病各报各的 —— 它们要做的事不一样:
//   · 非有限  -> 文件/内存被改坏了
//   · 全零    -> 从没解过 A (或调零时 A 是空的): 这是用户指令 3 点名的那一种
//   · 数值退化 -> "解出来了但秩亏": 姿态铺得太窄, 补偿里有两个方向根本没有模型
bool modelUsable(const double A[9], char* why, int whyLen) {
    if (why && whyLen > 0) why[0] = '\0';
    for (int i = 0; i < 9; i++) {
        if (!std::isfinite(A[i])) {
            if (why) snprintf(why, whyLen, "A 的第 %d 个元素不是有限数 (NaN/Inf)", i);
            return false;
        }
    }
    bool allZero = true;
    for (int i = 0; i < 9; i++) if (A[i] != 0.0) allZero = false;
    if (allZero) {
        if (why) snprintf(why, whyLen, "A 全为 0 (没有重力模型)");
        return false;
    }
    const double det = A[0] * (A[4] * A[8] - A[5] * A[7])
                     - A[1] * (A[3] * A[8] - A[5] * A[6])
                     + A[2] * (A[3] * A[7] - A[4] * A[6]);
    double fro2 = 0.0;
    for (int i = 0; i < 9; i++) fro2 += A[i] * A[i];
    const double fro = sqrt(fro2);
    if (!(fro > 0.0)) {
        if (why) snprintf(why, whyLen, "A 的 Frobenius 范数为 0");
        return false;
    }
    const double ratio = fabs(det) / (fro * fro * fro);
    if (!(ratio > GUARD_MIN_DET_RATIO)) {
        // ⚠ 措辞对得上判据 (2026-09-19 复审): 这条拦的是"接近完全秩亏", 不是"任何一个
        //   方向病态" —— 1e-3 换算成条件数约 354 (推导见 GUARD_MIN_DET_RATIO 处的注释),
        //   所以别再说"至少一个力方向没有模型"。
        if (why) snprintf(why, whyLen,
                          "A 数值退化: |det A| / ||A||^3 = %.3g <= %.3g "
                          "(接近完全秩亏: 该比值对'标量质量 x 正交'恒为 0.19245, "
                          "本门限约等于条件数 354)", ratio, GUARD_MIN_DET_RATIO);
        return false;
    }
    return true;
}

GuardState guardState() { return g_guardState; }

void guardReport(GuardReport& out) {
    out.state = g_guardState;
    out.frames = g_guardFrames;
    for (int i = 0; i < 6; i++) {
        out.ema[i]       = g_guardEma[i];
        out.tol[i]       = g_guardTol[i];
        out.voted[i]     = g_guardVote[i];
        out.exceeded[i]  = g_guardVote[i] && (g_guardTol[i] > 0.0)
                        && (fabs(g_guardEma[i]) > g_guardTol[i]);
    }
}

const char* guardStateName(GuardState s) {
    switch (s) {
        case GuardState::OK:            return "OK";
        case GuardState::UNCALIBRATED:  return "UNCALIBRATED";
        case GuardState::INCONSISTENT:  return "INCONSISTENT";
        case GuardState::REFERENCE_UNAVAILABLE: return "REFERENCE_UNAVAILABLE";
    }
    return "UNKNOWN";
}

// 闸门状态 -> 错误码。
// RelayCore 从前自己拿 static_cast<int>(guardState()) 去比字面量 1 和 2 —— 那是把
// "哪个状态配哪个码"存在【两个地方的巧合】里: 改一次枚举的数值, "去标定"与"去查负载
// 参数"这两条完全不同的处置指引就被对调, 而且没有任何测试会发现。现在这里是唯一的实现。
//
// 【"穷举"到什么程度, 说实话】: 这个 switch 没有 default, 但加了新的 GuardState 而忘了配
// 错误码时【编译器不会拦你】: 末尾那句 return 让缺失返回路径不存在 (没有 C4715), 而
// C4062 (unhandled enumerator) 默认关闭 —— 2026-09-19 用探针实测: /W1 /W3 /W4 都不报,
// 只有 /Wall 报, 本项目按 /W1 编译。所以真正把这张表钉住的是 test_force_compensation 的
// guard_error_code_mapping (三条映射逐条断言 + 与 errorCodeName 对上), 不是编译器。
// 末尾那句是"宁可返回 OK 也不掉出函数尾"的兜底。
RobotErrorCode guardErrorCode(GuardState s) {
    switch (s) {
        case GuardState::OK:            return RobotErrorCode::OK;
        case GuardState::UNCALIBRATED:  return RobotErrorCode::ERR_FORCE_UNCALIBRATED;
        case GuardState::INCONSISTENT:  return RobotErrorCode::ERR_FORCE_INCONSISTENT;
        // ⚠ 2026-09-21 (Task 7): 这里【换了一个码, 不是复用】ERR_FORCE_INCONSISTENT。
        //   那个码的字面意思是"有模型, 但与机械臂自报的参考量在某个投票通道上对不上" ——
        //   参考量不可用时【没有比过】, 报成"对不上"就是让日志里出现一个没发生过的事实,
        //   而操作员照它去"查负载参数有没有发进机械臂"会白忙一场 (要做的是查这一路的数据)。
        case GuardState::REFERENCE_UNAVAILABLE: return RobotErrorCode::ERR_FORCE_REFERENCE_UNAVAILABLE;
    }
    return RobotErrorCode::OK;
}

double currentMassKg() {
    EnterCriticalSection(&g_calibMutex);
    double A[9];
    for (int i = 0; i < 9; i++) A[i] = g_A[i];
    LeaveCriticalSection(&g_calibMutex);
    return massScaleOf(A);   // A 全 0 -> det 0 -> 报 0
}

void step(AppState::ForceData& fd, const double poseRxyz[6]) {
    // poseRxyz = {X_mm, Y_mm, Z_mm, Rx_deg, Ry_deg, Rz_deg}

    // 【记下本帧姿态】—— 闸门拒绝时那段打印要用它 (见 g_lastPose 的说明)。与算那六个数用的是
    // 同一次 30004 帧, 所以数与姿态天然对齐。
    for (int i = 0; i < 6; i++) g_lastPose[i] = poseRxyz[i];
    g_lastPoseValid = true;

    // 1. Update motion estimator
    double dt = 1.0 / static_cast<double>(Config::FORCE_EFFECTIVE_SAMPLE_RATE);
    g_motion.update(poseRxyz[0], poseRxyz[1], poseRxyz[2], dt);

    // 2. 默认输出 = 零。【fail closed 的落点】: 任何没走到"闸门放行"的路径都在这里留下 0。
    //    从前这里是"把 @1304 原样抄进 compensated" (透传) —— 那就是评审判定的 Critical:
    //    未标定时 @1304 的 x 通道在零外力下也报 −19.0 ~ −22.7 N
    //    (四份夹具 tests/fixtures/calib_poses_2026-09-19*.txt 的 F1304x 列, 36 个姿态实测),
    //    经下游 ForcePipeline 的映射 (Config::FORCE_MAX_TOUCH_N / FORCE_MAX_SENSOR_N = 3.3/200)
    //    与反射增益 5 之后以 21.9 N 计: 手上得到 ~1.8 N 的恒定推力 (21.9 × 3.3/200 × 5 = 1.81)。
    //    【数据侧一律置零与严重度无关】—— REJECT 只是"拒绝这一帧的运动", 而"不许传递
    //    数据"这件事由这一行无条件保证。
    for (int i = 0; i < 6; i++) fd.compensated[i] = 0.0;
    fd.isCalibrated = false;
    fd.calibMassKg = 0.0;
    for (int i = 0; i < 3; i++) {
        fd.calibComSensor[i] = 0.0;
        fd.calibBiasForce[i] = 0.0;
        fd.calibBiasTorque[i] = 0.0;
    }

    if (!g_isCalibrated) {
        // 指令 1/3: 没有可用模型 ⇒ 直接拒绝 + 报错, 不是透传。
        // (A 全零/退化 的情形在 setCalibration 就被挡下了, 所以这里报的是同一类原因。)
        setGuardState(GuardState::UNCALIBRATED);
        return;
    }

    // 3. Snapshot calibration globals under mutex
    EnterCriticalSection(&g_calibMutex);
    double A[9];
    for (int i = 0; i < 9; i++) A[i] = g_A[i];
    double com[3] = {g_comSensor[0], g_comSensor[1], g_comSensor[2]};
    double bF[3] = {g_biasForce[0], g_biasForce[1], g_biasForce[2]};
    double bM[3] = {g_biasTorque[0], g_biasTorque[1], g_biasTorque[2]};
    bool calib = g_isCalibrated;
    LeaveCriticalSection(&g_calibMutex);

    if (!calib) {   // setCalibration cleared calibration mid-flight
        setGuardState(GuardState::UNCALIBRATED);
        return;
    }

    // 4-5. Gravity in the SENSOR frame, then through the fitted response A.
    //    g = gravitySensorFrameAtYaw(pose, 0.0, g) —— 【psi 传 0】: 全量模型的 A 是自由 3×3,
    //    安装旋转/反射/非正交全被它吸收, 所以补偿式子里【没有 ψ】, 模块态不再参与。
    //    (残余模型走的是 gravitySensorFrame: 读模块态里的 ψ, 再乘一个【标量】质量。)
    //    ⚠ 走共享实现, 别在这里自己展开 Rᵀ(0,0,9.81): 模块自己实现过两次重力约定, 一份写
    //      成 R、一份写成 Rᵀ, 求解器因此安静地解错 —— 约定只能有一份实现。
    double gTool[3];
    TcpCalibration::gravitySensorFrameAtYaw(poseRxyz, 0.0, gTool);

    // Gravity force through the fitted response: Fg = A · g
    // (残余模型是标量质量乘 g —— A 的每一行都是一个"分量怎么随重力方向变"的响应。)
    double Fg[3];
    for (int a = 0; a < 3; a++) {
        Fg[a] = A[3 * a] * gTool[0] + A[3 * a + 1] * gTool[1] + A[3 * a + 2] * gTool[2];
    }

    // Gravity torque: c_s × (A·g) —— 与 Fg 同一个 w = A·g (叉乘结构, 不是独立的 3×3)。
    double Mg[3];
    cross(com, Fg, Mg);

    // 6. Inertia force (only if moving) —— 语义、时机、系数一字未改 (仍是 mass·a);
    //    mass 现在取自全量模型的质量尺度 |det A|^(1/3) (见 currentMassKg)。
    const double mass = massScaleOf(A);
    double Fi[3] = {0, 0, 0};
    if (!g_motion.isStill()) {
        double vel[3], acc[3];
        g_motion.getState(vel, acc);
        Fi[0] = mass * acc[0];
        Fi[1] = mass * acc[1];
        Fi[2] = mass * acc[2];
    }

    // 7. Compensate: compensated = sixForceRaw − bias − gravity − inertia
    //    ⚠ 先算进【局部变量】, 不直接写 fd —— 闸门要在数据出门之前判。
    double comp[6];
    comp[0] = fd.sixForceRaw[0] - bF[0] - Fg[0] - Fi[0];
    comp[1] = fd.sixForceRaw[1] - bF[1] - Fg[1] - Fi[1];
    comp[2] = fd.sixForceRaw[2] - bF[2] - Fg[2] - Fi[2];
    comp[3] = fd.sixForceRaw[3] - bM[0] - Mg[0];
    comp[4] = fd.sixForceRaw[4] - bM[1] - Mg[1];
    comp[5] = fd.sixForceRaw[5] - bM[2] - Mg[2];

    // ===== 7b. 参考量可用性 (Task 7): 先判"有没有得比", 再谈"比得对不对" =====
    // 不可用 ⇒ 拒绝 (compensated 保持第 2 步写的全零), 并且【报的是一个独立的状态】——
    // 它【不是】"不一致": 不一致说的是"两边都读到了数、但对不上"。这里根本没有第二个读数。
    // ⚠ 位置【在更新 EMA 之前】: 参考量不可用时那个差是拿"假设的 0"算出来的 —— 把它喂进
    //   EMA 就是【凭空造一个零读进判据的状态里】, 而本项目最忌凭空造数。跳过更新还让
    //   这层门一恢复就能接着用上一次的【真实】证据判 (若一帧都没比过, 播种标志仍是假,
    //   恢复后由第一帧真实读数播种)。
    if (!guardReferenceAvailable(fd)) {
        // 先记下本帧的可用性证据, 再报状态 —— 状态【跃迁】那次打印与复报那一行都要用它
        // 说清楚"为什么没有数据" (见 g_refOnlineLast 的说明)。这两行是那两个读数【唯一】
        // 的写入点, 而下一行是进入本状态【唯一】的入口 ⇒ 打印端拿到的永远是本帧的值。
        g_refOnlineLast = fd.sixForceOnline;
        g_refStaleLast  = fd.isStale;
        setGuardState(GuardState::REFERENCE_UNAVAILABLE);
        return;
    }

    // ===== 7c. 运行时一致性闸门 (用户指令 1/2) =====
    // 判据与三个原因的分辨写在 .h 里; 这里只做: 更新逐通道 EMA -> 投票 -> 放行或拒绝。
    // 参考量【不在这里写死】: 它是 guardReferenceValue, 判据的唯一一份定义。
    // 【EMA 无条件更新】(包括正在拒绝的时候): 否则闸门一旦拒绝就再也回不来, 而 Task 8
    //  "把负载发进去 -> 看它放行" 正是靠它回来的。
    for (int i = 0; i < 6; i++) {
        // 判据的对照量【只在这里取】, 且取自 guardReferenceValue —— 判据看的是哪一路,
        // 全程序只有那一处定义。别再把这个下标换成字面通道。
        const double d = comp[i] - guardReferenceValue(fd, i);
        // ★ 非有限值的【有界恢复】(2026-09-21 收口)。这条递推式自己【留不住】非有限值:
        //   `NaN + α·(有限值 − NaN)` 恒为 NaN ⇒ 一旦某帧喂进 NaN/Inf, 这个槽位会【永久】
        //   非有限, 于是闸门从此每帧都拒, 直到有人重新标定 —— 一条坏帧掐死一条路, 那是
        //   【对功能的拒绝】, 不是"保守"。处置: 递推的载体一旦不是有限值就【重新播种】
        //   (直接取本帧的 d, 不做平滑):
        //     · 喂进非有限值的那一帧: 播种成非有限 ⇒ 本帧照旧被 isfinite 那一关拒 (fail-closed 不变);
        //     · 之后【第一帧】好读数: 载体已经是有限的 d ⇒ 立刻回到正常比较 (不多拒一帧);
        //     · 若 d 本身一直非有限 (参考量那一侧坏了): 每帧播种成非有限 ⇒ 每帧都拒 ——
        //       拒绝的依据仍是【本帧的 d】, 与状态里的残留值无关, 参考量一恢复就自己回来。
        const bool reseed = !g_guardSeeded || !std::isfinite(g_guardEma[i]);
        if (reseed) g_guardEma[i] = d;
        else g_guardEma[i] += Config::FORCE_GUARD_EMA_ALPHA * (d - g_guardEma[i]);
        // 诊断侧: 与 @576 (fd.raw) 的差。同一个 α、同一帧、同一次 comp —— 只换对照量。
        // 【只报不判】, 见 g_guardEmaDiag 的说明: 它不参与任何容差比较。
        // ⚠ 同上做有界恢复: 这一路不进判决, 但它【是印出来的】—— 永久 NaN 会让现场
        //   每一屏都读到一个不是读数的数。
        const double dDiag = comp[i] - fd.raw[i];
        const bool reseedDiag = !g_guardSeededDiag || !std::isfinite(g_guardEmaDiag[i]);
        if (reseedDiag) g_guardEmaDiag[i] = dDiag;
        else g_guardEmaDiag[i] += Config::FORCE_GUARD_EMA_ALPHA * (dDiag - g_guardEmaDiag[i]);
    }
    g_guardSeeded = true;
    g_guardSeededDiag = true;
    g_guardFrames++;

    bool inconsistent = false;
    for (int i = 0; i < 6; i++) {
        // ⚠ 【非有限值先判, 再问投不投票】—— 顺序不能换: 不投票说的是"这一路的差【不参与
        //   容差比较】", 不是"这一路可以是 NaN"。非有限值【不是一个读数】: 它没有大小、
        //   也就没有任何容差能容纳它 ⇒ 它既不许进这个比较, 也不许被当成这个比较的结果报出去。
        //   (Fz 在本次改动之前就是"先问投不投票"的次序, 所以这一改动把那个既有的次序缺口
        //     一并补上 —— 与实际数值有没有坏无关, 讲的是判据的次序。)
        if (!std::isfinite(g_guardEma[i])) { inconsistent = true; break; }  // NaN/Inf 也算不一致
        if (!g_guardVote[i]) continue;   // 不投票的通道到此为止, 但照报 (见 g_guardVote 段)
        if (fabs(g_guardEma[i]) > g_guardTol[i]) { inconsistent = true; break; }
    }
    if (inconsistent) {
        // 拒绝: fd.compensated 保持第 2 步写下的全零 -> 下游由它推的 filtered / hapticOut /
        // F| 帧断开 (即【传感器力那一条路】)。⚠ 虚拟约束力【不断】—— 它由【位置】现算
        // (SafetyPredictor::computeConstraintForce, 触觉回调里对当前位置求一次),
        // 与 compensated 无关 (同 ForceCompensation.h 顶部与 RelayCore.cpp 那段)。
        setGuardState(GuardState::INCONSISTENT);
        return;
    }
    setGuardState(GuardState::OK);

    for (int i = 0; i < 6; i++) fd.compensated[i] = comp[i];

    // 8. Online EMA bias update (only when still) — 作用/时机/系数一字未改, 只有输入量
    //    跟着换成了 @1304。零偏是【这个通道】的零偏: 拿 @576 去更新它, 就是给另一路量的
    //    零偏做 EMA —— 两路的零偏不是一回事。实测出处: tests/fixtures/calib_poses_2026-09-19.txt
    //    7 个姿态的逐通道均值差 (@1304 − @576) = 19.8 / 1.6 / 1.7 N (x/y/z)。
    //    ⚠ 【新增】只在与【参考量】一致时才更新。不一致时继续在线学零偏, 等于闸门一边拒它、
    //      一边把同样的数据学进 b_F (而 b_F 的 EMA 目标正是把 compensated 拉向 0) ——
    //      那会让不一致自我掩盖, 而"安静地学错"正是这条闸门要防的东西。
    //      代价: 拒绝期间零偏不再自跟踪; 闸门放行后自动恢复。
    if (g_motion.isStill()) {
        double alpha = Config::FORCE_BIAS_EMA_ALPHA;
        // Update local copy, then write back under mutex
        bF[0] += alpha * (fd.sixForceRaw[0] - Fg[0] - bF[0]);
        bF[1] += alpha * (fd.sixForceRaw[1] - Fg[1] - bF[1]);
        bF[2] += alpha * (fd.sixForceRaw[2] - Fg[2] - bF[2]);
        bM[0] += alpha * (fd.sixForceRaw[3] - Mg[0] - bM[0]);
        bM[1] += alpha * (fd.sixForceRaw[4] - Mg[1] - bM[1]);
        bM[2] += alpha * (fd.sixForceRaw[5] - Mg[2] - bM[2]);

        EnterCriticalSection(&g_calibMutex);
        g_biasForce[0] = bF[0];
        g_biasForce[1] = bF[1];
        g_biasForce[2] = bF[2];
        g_biasTorque[0] = bM[0];
        g_biasTorque[1] = bM[1];
        g_biasTorque[2] = bM[2];
        LeaveCriticalSection(&g_calibMutex);
    }

    // 9. Update calib params in ForceData for HUD display / MATLAB relay
    fd.isCalibrated = true;
    fd.calibMassKg = mass;              // 质量尺度 |det A|^(1/3), 不是调用方传进来的
    for (int i = 0; i < 3; i++) {
        fd.calibComSensor[i] = com[i];  // c_s (m, 传感器测量系)
        fd.calibBiasForce[i] = bF[i];
        fd.calibBiasTorque[i] = bM[i];
    }
}

void shutdown() {
    g_isCalibrated = false;
    g_motion.reset();
    resetGuard();
    if (g_mutexInit) {
        DeleteCriticalSection(&g_calibMutex);
        g_mutexInit = false;
    }
}

} // namespace ForceCompensation
