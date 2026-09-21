#pragma once
#include "../core/AppState.h"
#include "../safety/RobotError.h"   // RobotErrorCode —— 闸门状态 -> 错误码的映射在这里 (见 guardErrorCode)

// Motion estimator: tracks tool velocity & acceleration from position history
// Uses 5-point ring buffer for central-difference acceleration estimation
class MotionEstimator {
public:
    MotionEstimator();
    void update(double x, double y, double z, double dt);
    void getState(double vel[3], double acc[3]) const;
    bool isStill() const;
    void reset();
private:
    static const int BUF_SIZE = 5;
    double m_posBuf[5][3];
    int m_idx;
    int m_count;
    double m_vel[3];
    double m_accRaw[3];
    double m_accFiltered[3];
    // Butterworth2-style 10Hz LPF for acceleration
    double m_lpfB0, m_lpfB1, m_lpfB2, m_lpfA1, m_lpfA2;
    double m_lpfX1[3], m_lpfX2[3], m_lpfY1[3], m_lpfY2[3];
};

namespace ForceCompensation {
    // Call once at startup — loads calibration file, initializes filters
    void init();

    // ★ 2026-09-21: 调用率【只有一处】—— 帧率, 实测 122.9 Hz (8.139 ms), 在
    //   RelayCore::forceReaderThread 里紧挨着 30004 帧解析之后 (§一个调用点, 见那里的警告)。
    //   从前它跑在 pollForce 的 ~11 Hz 上, 那是噪声压不下去的直接原因 (推导见 Config::
    //   FORCE_FILTER_CUTOFF / FORCE_FILTER_FS_HZ)。
    // ⚠ 【α 与 dt 都是按实测换算的, 所以本函数对调用率不敏感】—— 换率不会悄悄改时间常数
    //   (stepIntervalSec / feedMotionEstimator)。但【滤波器系数是】: 见上面那两条常数。
    // fd.sixForceRaw[] (@1304, 原始读数) must be fresh; poseRxyz = {X,Y,Z,Rx,Ry,Rz} in mm & deg
    // from GetPose(). Writes fd.compensated[] (6-axis compensated force).
    // ⚠ 输入通道是 @1304, 【不是】@576 (fd.raw) —— 见下面全量模型的说明。
    // ⚠ 【两个对照量都必须是同一帧的】: 一致性闸门拿【参考量】当判据 (哪一路是参考量见
    //   .cpp 里 guardReferenceValue —— 那是判据的唯一一份定义), 而 fd.raw[] (@576) 是
    //   与它并排报出的诊断侧。
    //   (RelayCore 的 ForceReader 在【同一次 30004 收帧】里同时填 raw / tcpForce /
    //    sixForceRaw, 所以三者天然时间对齐, 不需要再对时)。填不上 (全 0) 会被判成不一致。
    // ⚠ 闸门拒绝时 fd.compensated[] 全 6 个分量为 0 (不再透传 @1304)。
    void step(AppState::ForceData& fd, const double poseRxyz[6]);

    // ===== 全量模型 (2026-09-19, Task 6) =====
    //   compensated_F = sixForceRaw − b_F − A·g − Fi
    //   compensated_M = sixForceRaw − b_M − c_s × (A·g)
    //   g = TcpCalibration::gravitySensorFrameAtYaw(pose, 0.0, g)
    // 与【残余模型】的区别 (换掉的就是它): 残余模型吃 @576 并减 mass·g —
    // 那个 mass 是"机械臂没补干净的那一份"(可带符号), 而 g 用的是含安装偏转角 ψ 的模块态。
    // 全量模型的 A 是自由 3×3, 安装旋转/反射/非正交一起吸收, 式子里【没有 ψ】——
    // 这是 ψ 从补偿路径退场的那一步 (参数由 PayloadCalibration::fitRaw 在 @1304 上解出)。
    // A: 3×3 row-major (kg); biasForce (N) / biasTorque (N·m); comSensor 单位【米】。
    // ⚠ A 不可用 (全零 / 非有限 / 数值退化) 时【拒绝安装】: 不更新任何参数、把状态打回
    //   "未标定", 并在 stderr 上说出是 A 的哪一个毛病。这就是用户指令 3
    //   (「全零 A 拒绝传递数据并报错」) 的落点 —— 让"全零 A"根本进不了"已标定"这个状态。
    void setCalibration(const double A[9], const double biasForce[3],
                        const double biasTorque[3], const double comSensor[3]);

    // ===== 运行时一致性闸门 (2026-09-19) =====
    // 判据: 【本地全量模型的输出】与【机械臂自报的参考量】在【投票通道】上逐通道比较。
    //   ⚠ 【投票通道】是哪几个由 .cpp 的 g_guardVote 一处定义 (掩码的唯一一份实现);
    //     当前是 Fx/Fy —— Fz 与力矩三个分量【算出来、报出来, 但不投票】(理由与代价见那一段)。
    //   · @1304 (fd.sixForceRaw) 是原始读数; 本地模型给出 compensated = @1304 − b_F − A·g,
    //     是【外力】的一个估计。
    //   · 参考量是机械臂用【它自己的】负载模型减掉重力之后的估计 —— 同一个外力。
    //   ⇒ 两个模型都对时两者应当一致; 不一致 ⇒ 至少一个错 ⇒ 拒绝把数据往下传。
    // ⚠ 【是哪一路当参考量, 全程序只有一处定义】: .cpp 里紧挨 g_guardVote 的
    //   guardReferenceValue —— 那里写了它为什么是那一路 (实测出处) 以及"打印端不许再抄
    //   通道号"这条规矩。要换参考量就改那一处, 别在这里再记一份。
    //   · 另一路 (fd.raw[], 传感器侧) 仍在同一屏上报出, 但【只报不判】—— 判据不看它。
    // 这就是用户指令 1/2 的机制: 未标定或与标定不符 ⇒ 拒绝 + 报错; 而"标定后 @1304 与
    // 参考量应当一致"正是判据 (Task 8 下发正确负载之后它们才应当收敛)。
    //
    // 不一致 ⇒ 【compensated 全 6 个分量置零】(不是只置 haptic): 下游
    //   ForcePipeline::step 从 compensated 推 filtered/hapticOut/F| 帧 —— 所以断掉的是
    //   【传感器力那一条路】(Touch 上的反射力 + 发给 MATLAB 的 F| 帧)。
    // ⚠ 它【不断】虚拟约束力: 那一条由【位置】现算 (SafetyPredictor::computeConstraintForce,
    //   触觉回调里对当前位置求一次), 与 compensated 无关, 也对
    //   orientExtraForce / orientRepulsionForce 一样 —— 那两条同样是位置驱动的。
    //   不要把这里写成"触觉与约束力两条路一起断": 那是错的, 会让人以为拒绝之后
    //   操作员连安全边界的推手都没有了。
    // ⚠ 未标定 -> step() 不再透传 @1304 (旧行为会把 @1304 的 ~21.9 N 偏置经 ForcePipeline
    //   的 0.0165×5 变成手上 ~1.8 N 的恒定推力; 置零之后没有这个偏置)。
    //
    // 三种拒绝原因【必须分得开】(处置一样 = 都拒绝, 但操作员要做的事不同):
    //   UNCALIBRATED          -> 去标定 (按 'm' 采多姿态 + 's' 解 A, 再 'z' 调零);
    //   INCONSISTENT          -> 去查负载参数有没有真的发进机械臂 (Task 8), 或重跑离线一致性检查;
    //   REFERENCE_UNAVAILABLE -> ★ 处置与上面两个【不同】(那两个动作在这里都没有依据):
    //     等参考量这一路恢复 (查 30004 帧 / 六维力在线状态) —— 不是去标定, 也不是去查负载参数。
    //     ⚠ 它【不是】"不一致": 判据是 compensated − 参考量, 参考量为 0 时判据退化成
    //       "本地输出是否在自己的容差内", 而按构造它总是在 ⇒ 从前会在【一个不携带信息的
    //       通道上放行】。这个状态就是把"没有信息"从"一致"里择出来, 并且不放行。
    // ⚠ 【三种拒绝原因】, 不是两种 (2026-09-21 Task 7 加了第三种)。处置一样 (都拒绝、
    //   compensated 全置零), 但操作员要做的事不同 —— 所以原因必须能分开报。
    enum class GuardState {
        OK = 0,             // 模型在, 参考量可用, 且【投票通道】与参考量一致 -> 数据放行
        UNCALIBRATED = 1,   // 没有可用模型 (未标定 / A 全零 / A 退化) -> 拒绝
        INCONSISTENT = 2,   // 模型在, 参考量【可用】, 但两者在某个【投票通道】上对不上 -> 拒绝
                            //   (参考量 = .cpp 的 guardReferenceValue, 判据的唯一一份定义;
                            //    投票通道 = .cpp 的 g_guardVote, 掩码的唯一一份定义 ——
                            //    当前是 Fx/Fy 两个; Fz 与力矩三个分量【照报不判】,
                            //    理由是它们与参考量之间没有"一致"态, 见 g_guardVote 段)
        REFERENCE_UNAVAILABLE = 3
                            // 模型在, 但【参考量这一路没有数据】-> 拒绝 (2026-09-21, Task 7;
                            //   依据与"为什么不用新门限"见 .cpp 里 guardReferenceAvailable ——
                            //   那是这条判据的唯一一份定义)
    };
    struct GuardReport {
        GuardState state = GuardState::UNCALIBRATED;
        long   frames = 0;
        // ⚠ 这里是【尚未填充】的默认值, 不是掩码。掩码的唯一一份实现是 .cpp 里的 g_guardVote。
        //   从前这里抄了一份字面量, 于是"改一处忘一处"会让一份默认构造的报告对外撒谎
        //   (它声称的掩码与实际生效的不是同一份) —— 而类内初值【引不到】.cpp 里的 static,
        //   任何抄在这里的字面量都注定会漂。全 false 不可能被误读成一份真实的掩码,
        //   所以它就是这里的正确初值。真实掩码由 guardReport() 填 (它已经在填)。
        bool   voted[6]    = {false, false, false, false, false, false};
        // ⚠ 【state == REFERENCE_UNAVAILABLE 时, 本数组与 ema[] 里留的不是为这个状态做的比较】
        //   (2026-09-21 复审定案, 选的是"写在这里"而不是"清零"): 参考量不可用时 step() 在判据
        //   之前就返回, 【不更新】EMA (那个差是拿"假设的 0"算的, 喂进去等于凭空造一个零读数),
        //   而 guardReport() 仍按【冻结的】EMA 现算本数组。⇒ 那两个数组里的值来自【上一次真实
        //   比较】, 不是伪造的数, 但也【不是】这次比较的结果 —— 留着是为了不丢信息
        //   (下一帧恢复后接着用的就是它们)。(tol[] 不在此列: 它每次现填, 与比较做没做过无关。)
        //   要判"这次到底比过没有"【只看 state】: state 是权威, exceeded[] 与 ema[] 不是。
        bool   exceeded[6] = {false, false, false, false, false, false};
        // 同上: 不可用时它是【上一次真实比较】的差, 不是为这个状态算出来的。
        double ema[6] = {0, 0, 0, 0, 0, 0};   // compensated − 【参考量】的 EMA (N / N·m)
        double tol[6] = {0, 0, 0, 0, 0, 0};
    };
    GuardState  guardState();
    void        guardReport(GuardReport& out);
    const char* guardStateName(GuardState s);

    // ⚠ 2026-09-21 记: 本段【原本】被我加过两个访问器 (lastMotionAcc / lastMotionStill) 与
    //   一个 currentBias 重载 —— **三个都是重复**。既有的 motionState() (下面) 与
    //   currentBias() (:180) 本来就有, 其中一个还写着 dt 那处可疑。已全部撤回。
    //   教训: 动一个头文件之前, 【先 grep 这个头文件里有没有现成的访问器】——
    //   "我以为没有" 与 "确实没有" 是两回事。

    // ===== GuardState -> RobotErrorCode (闸门唯一一处"状态转错误码") =====
    // 用 switch 穷举 (没有 default), 但【别指望编译器替你发现漏配】: 末尾那句
    // return RobotErrorCode::OK 让"漏了一个 GuardState"照样编得过 —— 没有缺失返回路径,
    // 就没有 C4715; 而 C4062 (unhandled enumerator) 【默认关闭】: 2026-09-19 用一个
    // "漏一个 case + 末尾兜底 return" 的探针实测过, /W1 /W3 /W4 都不报, 只有 /Wall 报。
    // 本项目按 /W1 编译。真正把这张表钉住的是 test_force_compensation 的
    // guard_error_code_mapping (每条映射逐条断言 + 与 errorCodeName 对上), 不是编译器。
    // ⚠ 2026-09-21 (Task 7) 加了第四个 GuardState —— 那次加值【没有】被编译器发现,
    //   是靠上面这条用例与 guardStateName / 这张表三处一起改才补齐的 (漏一处就是
    //   "状态 3 落到兜底 return OK" ⇒ 拒绝却报无错误, 与本项目最忌的"安静地错"同类)。
    // 同一段说明也写在 ForceCompensation.cpp 的实现处。
    // 为什么非要有这张表 —— RelayCore 从前拿 static_cast<int>(guardState()) 去比字面量 1 和 2,
    // 于是"给 GuardState 换顺序"这种无害重构会把"去标定"与"去查负载参数"两条完全不同的
    // 处置指引对调, 而用户指令 1 的全部意义就在于告诉操作员【该做哪件事】。
    // OK -> RobotErrorCode::OK (调用方据此跳过上报)。
    RobotErrorCode guardErrorCode(GuardState s);

    // A 能不能当【重力模型】用: 非有限 / 全零 / |det A| 相对 ||A||_F³ 近零 (数值退化) 都不行。
    // 返回 false 并把具体原因写进 why。装载 (ForceCalibration::loadFromFile) 与安装
    // (setCalibration) 共用它 —— 同一条判据只许有一份实现。
    bool modelUsable(const double A[9], char* why, int whyLen);

    // 读出当前生效的全量模型 (供「仅调零」把 A / c_s 原样保留着写回文件)。
    void currentModel(double A[9], double comSensor[3]);

    // 取出当前零偏 (供「仅调零」把新零偏写回文件时复用)
    void currentBias(double biasForce[3], double biasTorque[3]);

    // 取出【最近一帧】的重力项 (工具系): Fg = A·g, Mg = c_s×(A·g)。
    // ★ 2026-09-21 加 —— 为调零用。全量模型下零偏必须是 "@1304 − A·g"(力矩减 c_s×(A·g)),
    //   因为 @1304 里含着重力项、而 comp 又会再减一次 A·g。
    //   ⚠ 【A·g 的唯一一份定义在 step() 里】—— 本访问器读的就是那一份。别拿 TcpCalibration
    //     再算一遍: 同一个量两份实现会漂开 (2026-09-21 的调零就是踩了这个:
    //     按残余模型时代的写法直接存原始读数当零偏 ⇒ 重力被减两遍 ⇒ comp = −A·g ≈ 4N)。
    void currentGravityTerm(double Fg[3], double Mg[3]);

    // 取出【最近一帧】算出的惯量项 Fi (传感器系, N)。
    // ★ 2026-09-21 加 —— 给多姿态检查的 SPACE 打印用, 回答【comp 里的摆动有多少是这个量注进去的】。
    //   ⚠ 【Fi 的唯一一份定义在 step() 里】—— 含那个"基座系 acc 映到传感器系"的 R^T 变换。
    //     本访问器读的就是那一份。**别拿 motionState() 的 acc 在调用方自己乘一遍**:
    //     那正是"同一个量两份实现", 而它就漂开过一次 (acc 没映到传感器系 ⇒ 减掉一个方向
    //     不相干的 m·a ⇒ 现场表现为"阻力方向飘")。这是与 currentGravityTerm 同一条规矩。
    //   ☞ 读法: 恰好 (0,0,0) ⇒ 本帧【没在动】(步 6 只在 !isStill() 时算它 ⇒ 惯量项没被注入);
    //     非零 ⇒ 它就是 comp 里那一份注入量, 逐轴对着 comp 的摆动看即可。
    //   ⚠ 闸门拒绝时它【不归零】(与 compensated 不同): 它是"本帧算出来的 Fi", 与闸门判决无关。
    void currentInertiaTerm(double Fi[3]);

    // ===== 测试钩子: 把 step() 认作的时间步长【钉成给定值】(秒) =====
    // 【为什么需要它】step() 的 dt 是【实测】的 (stepIntervalSec ← GetTickCount), 这是刻意的
    //   —— 按假定采样率换算正是本项目反复栽的错 (闸门 EMA 那个 0.02 就是这么坏的)。
    //   代价是【用例没法驱动时间】: 紧循环里 dt≈0 ⇒ 任何"按时间常数换算 α"的 EMA 在用例里
    //   都冻住 ⇒ 只能靠 Sleep 去凑真实时间, 而那会引入墙钟依赖 (本项目已有一条用例因此
    //   间歇性假红, 记为 A19)。⇒ 用它把时间变成【确定的输入】。
    // 传负数 = 恢复实测 (默认)。**只在用例里调**, 生产路径不许调。
    void setStepDtForTest(double sec);

    // 诊断用: 把运动检测器的当前状态与判定读出来。返回 isStill() 的当前值。
    // 存在的理由: isStill() 疑似在生产中永远为假 (那样在线 EMA 零偏更新就不跑),
    // 需要用实机噪声量级来判定, 而不是靠读代码猜。
    // 注意 vel/acc 实际单位是 m/s 与 m/s² (MotionEstimator::update 里已 mm→m 换算),
    // 与阈值常量同量纲 —— 可疑的从来不是单位。
    // ★ 2026-09-21 实机结论 (这条怀疑已经结了): 惯量项【确实一直在被算】——
    //   四个姿态的 SPACE 窗口里 Fi 非零帧 = 22/22, 即 isStill() 几乎总是假。
    //   但 Fi 的量级只有 max 0.0094 N (z), 面对 comp 0.18 N 的摆动是 19 分之一
    //   ⇒ 它【不是】现场抖动的来源 (见 ForceCompensation::currentInertiaTerm)。
    //   所以"EMA 零偏更新跑不跑"仍值得关心, 但那与力反馈的抖动是两件事。
    //   ⚠ 探针读数 (main.cpp runMotionProbe / 本函数) 留在原处, 描述的是【改之前】的
    //     dt/采样节奏错配; 那两件事 2026-09-21 已修 (喂样只在姿态变化时 + dt 用实测耗时)。
    bool motionState(double vel[3], double acc[3]);

    // Check if calibration is active
    bool isCalibrated();

    // 当前生效的惯性补偿质量 (kg) —— 全量模型的【质量尺度】m = |det A|^(1/3), 由 A 现算。
    // 残余模型时代它由调用方 setCalibration(massKg, ...) 传进来 (那时是可带符号的残余质量);
    // 全量模型的参数表里没有标量质量 —— 它本来就从 A 里分解出来 (PayloadCalibration::decompose
    // 的 massScale 是同一个量, 几何平均奇异值 = |det|^(1/3)), 所以这里不再单独存一份,
    // 免得两份漂开。A 的 9 个元素全为 0 (未标定 / 空模型) 时它报 0。
    double currentMassKg();

    // Call on shutdown
    void shutdown();
}
