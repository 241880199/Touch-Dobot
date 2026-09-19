#pragma once

// 末端负载标定 (质量 + 质心) —— 由实机多姿态空载数据解出【机械臂没补干净的那一份】。
//
// 背景: 机械臂内部按 EnableRobot(load,cx,cy,cz) 做重力/惯性补偿, 30004 的
//       ActualTCPForce 是补偿【之后】的净力 (应当如此)。负载参数不准 → 读数随姿态漂移。
//       这些参数只能实测, 所以本模块把它们从编译期常量变成可标定量 (payload_calib.json)。
//
// ⚠ 实机实测 (2026-09-18): 机械臂内部那份负载参数从 TCP 口【改不动】——
//   EnableRobot(load,..) / Payload() / LoadSwitch(1) 三条通道全无响应 (0.25 kg 的变化只
//   引起 0.0002 N·m 的读数变化; 等 3 s、Disable→Enable 也一样)。所以本模块的产出不再是
//   "一份要下发给机械臂的绝对负载", 而是【残余量】: Result::dm / Result::dp, 由调用方交给
//   ForceCompensation 在本地减掉 (见 Result 的说明)。切不可再把它当成"下发绝对参数"的标定。
//
// 模型 (传感器系; g_i = TcpCalibration::gravitySensorFrame(pose_i) 是重力在传感器系的表示 ——
// 含传感器相对法兰的安装偏转角, 与 ForceCompensation::step 共用同一份实现, 别在此另写一份):
//   raw_F_i = b_F + Δm · g_i
//   raw_M_i = b_M + Δp × g_i
// 其中 Δm = m_true − m_cfg, Δp = m_true·c_true − m_cfg·c_cfg。
// 对姿态 0 做差可同时消掉传感器零偏 b_F / b_M:
//   ΔF_k − ΔF_0 = Δm · (g_k − g_0)
//   ΔM_k − ΔM_0 = Δp × (g_k − g_0)
// 这是关于 (Δm, Δp) 的线性最小二乘 (4 个未知量, 每个姿态 6 个方程)。
//
// 解是【带符号】的 —— Δm / Δp 直接就是"机械臂还差多少", 本地补偿减掉它即可, 不需要
// 反推机械臂内部究竟配了什么 (那正是"推出绝对值再下发"那条死路要干的事)。
namespace PayloadCalibration {

    // 求解结果
    //
    // 【本标定的输出是 dm / dp 这两个残余量】—— 不是绝对负载。实机实测: 机械臂内部
    // 的负载参数从 TCP 口【根本改不动】(EnableRobot(load,..) / Payload() / LoadSwitch(1)
    // 三条通道全无响应, 0.25 kg 的变化只引起 0.0002 N·m 的读数变化), 所以"推出绝对值再
    // 下发"这条路是死的, 由我们自己在本地把差值减掉:
    //     compensated = raw − bias − mass·gTool − inertia      (ForceCompensation)
    // dm / dp 正是机械臂【没补干净的那一份】(差商消掉了传感器零偏, 是纯残余), 所以本地
    // 补偿不需要知道机械臂内部究竟配的是什么值。
    struct Result {
        double dm;           // 残余质量 (kg, 带符号)
        double dc[3];        // 绝对质心相对当前配置的增量 (mm, 带符号)
        double dp[3];        // 残余一阶矩 (kg·m, 带符号) —— 力/力矩方程的右端项
        double massKg;       // 换算出的绝对质量 (kg) = mCfg + dm
        // 按传入的 signZ 约定折算出的绝对质心 (mm, 法兰系)。⚠ 只用于记录与显示
        // (payload_calib.json; 机械臂侧那份负载要【下次重启】才随连接时序更新, 运行时下发会让
        // 机械臂动), 本标定不依赖它是否被采纳, 也不依赖这个符号约定选得对不对 ——
        // 本地补偿只用 dm / dp。
        double comMm[3];
        double rmsForceN;    // 力通道拟合残差 (N)
        double rmsMomentNm;  // 力矩通道拟合残差 (N·m)
        int    poses;        // 参与求解的姿态数
        // 本次解出的传感器安装偏转角 (度) —— solve() 在 [-180, 180] 上以 0.5° 步长扫出来的
        // 最优值。它是【模型参数】, 不是残余量: 调用方必须先 TcpCalibration::setSensorYawDeg()
        // 再写本地补偿, 否则重力模型与刚解出的 dm/dp 对不上 (详见 .cpp 里的扫描说明)。
        double sensorYawDeg;

        // CZ 符号的两种解释下的物理质心 Z (mm, {+1, -1})。signZ 不进 buildRows, 所以两种
        // 解释的拟合残差【完全相同】—— 数据本身区分不了符号。这两个值只是把这个不确定性
        // 摆明, 现在没有任何判据或下发依赖它们 (曾经靠实机探针裁决, 已废除: 见 main.cpp)。
        double cTrueZ[2]     = {0.0, 0.0};
    };

    // 纯函数: 最小二乘求解。无全局状态, 便于单测。
    //   poses:    n 个法兰位姿 [x,y,z,rx,ry,rz], 角度单位【度】(= GetPose 的返回)
    //   forces:   n × 3 各姿态平均原始力 (N)
    //   moments:  n × 3 各姿态平均原始力矩 (N·m)
    //   mCfg/comCfg: 当前机械臂里配置的负载 (即上面公式的 m_cfg / c_cfg, com 单位 mm)
    //   comSignZ: 折算【绝对】质心 Z 时用的符号约定 (+1 或 -1)。现在它是一个纯粹的显示/记录
    //             约定 (随 payload_calib.json 持久化), 不再是待裁决的未知量: 数据定不了它,
    //             而本地补偿只用 dm/dp, 与它无关。选错只会让 comMm 的 Z 读数不同。
    // 返回 false: 姿态数不足、姿态退化 (共线/同姿态)、解非物理 (质量 ≤ 0 或不合理)
    bool solve(const double poses[][6], const double forces[][3], const double moments[][3],
               int n, double mCfg, const double comCfg[3], double comSignZ, Result& out);

    // ===== 原始力通道的线性模型 (2026-09-19 重做, 取代上面的 solve() ) =====
    //
    // 上面 solve() 的模型预设了"传感器安装 = 绕 z 的纯偏航 psi", 再在 [-180,180] 上把 psi
    // 扫出来。实机数据 (2026-09-19) 证伪了这个预设: 真实响应还需要一个【反射】, 而扫描补不上
    // 这个自由度 —— 它只好把质量符号翻过去 (解出 dm = -0.417 kg, 被非物理门限拒掉)。
    // 用户的判词: "任何猜测以及偏差(如 psi)都不可取"。
    //
    // 所以这里是【全线性、无任何预设角】的模型 (传感器测量系):
    //   F_i = b_F + A · g_i            A: 3×3, 9 个元素全部由数据定 (反射/非正交都吸收得下)
    //   M_i = b_M + c_s × (A · g_i)    c_s: 3, 质心 —— 力矩是叉乘结构, 不是独立的 3×3
    //   g_i = TcpCalibration::gravitySensorFrameAtYaw(pose_i, 0.0, g)   <- 【传 psi = 0】
    // 力通道 12 个未知 (b_F 3 + A 9), 力矩通道 6 个 (b_M 3 + c_s 3, 给定 A 后仍是线性的)。
    // 全部线性 -> 一次求解, 不扫描。c_s × b_F 是常量, 被 b_M 吸收, 所以上式是完整的。
    // (spec: Docs/superpowers/specs/2026-09-19-raw-channel-calibration-design.md §2 §3)

    // 逐姿态的【实测噪声】—— 模型形式检验的尺子, 与任何模型、任何重力约定无关。
    //
    // 每个姿态是 N 个样本的平均, 所以【均值】的 1σ 不确定度 = sqrt(var/N) (标准误)。方差取
    // 样本方差 (无偏, 除以 N−1)。这批数是采集时顺手采下来的 (main.cpp 的 BiasCheck::sample:
    // 与均值同一批样本、同一时间窗), 不是从拟合残差里反推的 —— 这正是它有价值的原因:
    // 残差里混着"模型形式错"那一份, 拿它当噪声尺子会把判据放松成自指 (见 fitRaw 的说明)。
    //
    // 传 nullptr 给下面的函数 = 【没有噪声估计】-> 模型形式检验【不做】(不是"通过")。
    // 任一方差 ≤ 0 (通道没采到 / 读数冻住 / 没填) 也算"没有噪声估计" —— 宁可说没做检验,
    // 也不拿半批尺子判生死。
    struct PoseNoise {
        int    n;         // 该姿态参与平均的样本数 (1 = 没平均, 但方差已知)
        double varF[3];   // 力三个分量的样本方差 (N²)
        double varM[3];   // 力矩三个分量的样本方差 (N·m²)
    };

    // 原始通道的线性拟合结果。
    struct RawFit {
        // 3×3 row-major: 分量 a 的力 = bF[a] + Σ_c A[a*3+c]·g[c]。9 个元素全自由:
        // 【不】假设它是旋转、【不】假设手系、【不】假设偏航。
        double A[9];
        double bF[3];           // 力零偏 (N)
        double cS[3];           // 质心 (【m】, 传感器测量系) —— 力矩方程里的那个 c_s
        double bM[3];           // 力矩零偏 (N·m)
        double rmsForceN;       // 力通道残差: sqrt(Σr² / (3n))
        double rmsMomentNm;     // 力矩通道残差: sqrt(Σr² / (3n))
        double cond;            // 力通道设计矩阵的 cond = σmax/σmin (姿态激发够不够)
        // 18 个参数的 1σ 不确定度 = sqrt(diag((JᵀJ)⁻¹)·σ²), σ² = SSR/(方程数 − 参数数)。
        // 布局与上面的字段同序: [0..8] = A (row-major, 与 A[9] 同序), [9..11] = bF,
        // [12..14] = cS, [15..17] = bM。力矩段是【给定 A 之后】的 6 参数系统。
        // 【为什么必须报它】残差是小量, 对"模型形式错"不敏感 —— 本批实机数据上形式错只值
        // 0.018 N, 而质量符号已经翻过去了。参数不确定度才让人分得清"±0.05"和"±0.5"。
        // ⚠ 它【不再】参与任何接受/拒绝判据: 它来自拟合残差, 拿它当门限就是自指 (见 fitRaw)。
        // 零自由度时 (方程数 = 参数数) 无法估, 报 0。
        double paramSigma[18];

        // ===== 分解出来的物理量 —— 【报告量, 不参与接受/拒绝】 =====
        double massScale;       // m = (σ1σ2σ3)^(1/3) (kg), 与 decompose() 的 m 同一个值
        double parity;          // sign(det A): +1 / −1
        // σ1/σ3 ≥ 1。力通道的 A 是【9 个元素全自由】的, 数据对它没有任何约束, 所以
        // "非正交"是【传感器实际响应长这样】的测量结果, 不是"模型形式错"的证据
        // (实机那批: 1.06546, 即 6.5% 非正交, 是物理属性)。因此这里只报数, 不判生死。
        double isotropyRatio;

        // ===== 模型形式检验 (χ² 式): 残差 vs 【实测】噪声 =====
        // 力通道: chi2ForceRatio = Σ(e/σ)² / dofF, σ = sqrt(var/N) 是各姿态各通道的均值 1σ。
        //         正确模型下它 ~ 1 (期望 1, 标准差 sqrt(2/dof))。
        // 力矩通道: 它【是】受约束的 (c_s × (A·g), 3 参数 vs 自由的 9), 所以那里问的是
        //         失拟问题 —— lackOfFitMomentRatio = (ssM_free − ssM)/σ̄_M² / 6。
        // 没有噪声估计时 (noise == nullptr) 以上字段全 0 且 modelFormChecked = false。
        double noiseForceN;          // 力通道实测噪声的合并尺度 (N) = sqrt(mean(σ²))
        double noiseMomentNm;        // 力矩通道同上 (N·m)
        double chi2ForceRatio;       // χ²_F / dofF
        int    chi2DofForce;
        double chi2MomentRatio;      // χ²_M / dofM —— 【只报告, 不做判据】(理由见 .cpp)
        int    chi2DofMoment;
        double lackOfFitMomentRatio; // 力矩失拟统计量 / 6 (自由 12 参数 vs 受约束 6 参数)
        int    lackOfFitMomentDof;   // 6, 或 0 = 做不了 (方程数不足以养自由模型)
        bool   modelFormChecked;     // true = 上面这些数【真的算过】并可作判据
    };

    // 纯函数: 线性拟合并做【物理自检】。返回 false = 拒绝给出参数 (原因打到 stderr),
    // 此时 out 里是【未经自检】的线性解, 只供诊断打印, 调用方不得采用。
    // 拒绝的情形 (判据全部由数据/量程给出, 没有一个固定比例是猜的):
    //   1) 模型形式: 拟合残差与【实测】噪声不一致 (χ² 式, 只在不传 noise 时无从判 -> 不做)
    //   2) 质量尺度: m ≤ 0 或超出 CR3 的负载量程 (EnableRobot 的量程)
    //   3) 条件数: cond 过大 -> 姿态激发不足, 12 个参数定不下来
    // 返回 false 也可能是线性层就失败: 姿态数 < 4 (12 个未知) 或姿态退化 (J 秩亏)。
    //
    // noise 给出逐姿态的实测噪声 -> 模型形式检验得以成立 (它需要一把与模型无关的尺子)。
    // 【默认 nullptr 的那一版仍然可用】(测试与离线重放用): 它不做模型形式检验,
    // out.modelFormChecked 会是 false —— 调用方读到 false 就该知道"形式没被验过",
    // 而不是"形式验过了、通过了"。生产路径应当传 noise。
    // 前置条件: out 的内存由调用方持有; noise 非空时其 n 个元素必须与 poses/forces/moments 同序。
    bool fitRaw(const double poses[][6], const double forces[][3], const double moments[][3],
                int n, RawFit& out, const PoseNoise* noise = nullptr);

    // 纯函数: 只做线性拟合, 【不做任何物理自检】。
    // 单独暴露的理由: 自检是【物理结论】的门, 而"任意 3×3 能否被复原"是【线性代数】的性质 ——
    // 两者必须能分开测。一个明显非正交 (甚至带反射) 的 A 可以被精确复原, 而它该不该被采信
    // 是另一个问题; 只有把两层分开, 这两件事才各有各的断言。(生产路径请用 fitRaw。)
    // noise 只影响那些【检验用】的报告字段 (χ² 等), 不参与求解 —— 拟合本身永远是无权的普通
    // 最小二乘 (加权的更换属于改线性代数, 不在本任务内)。
    bool fitRawLinear(const double poses[][6], const double forces[][3], const double moments[][3],
                      int n, RawFit& out, const PoseNoise* noise = nullptr);

    // 从 A 读回物理量 —— 这一步才是"标定", 不是"猜"。
    //
    // 约定: A = m · S · Q, 其中
    //   m   = (σ1σ2σ3)^(1/3) > 0                     质量尺度
    //   S   = diag(1, 1, parity)                     手系镜像 (parity = sign(det A), 固定约定)
    //   Q   = S·A/m                                  含手系的完整三维安装姿态, det Q = +1
    // 于是 round-trip 逐位精确: m·S·Q = m·S·(S·A/m) = A, 对【任意】可逆 A 都成立, 不要求 A 正交。
    // 三个输出因此是【唯一】的 —— 不需要在"取哪个正交因子"之间做选择。
    // (spec §3 写的是 Q = U·diag(1,1,parity)·Vᵀ; 那只在 σ1=σ2=σ3 时才良定义, 而 σ 全相等恰恰
    //  是合格安装的情形 —— 此时 U/V 不唯一, 该式也就跟着不唯一。见 .cpp 的说明。)
    // 返回 false = A 奇异 (|det A| 相对自身量级小到定不出手系与尺度), 此时 out 不保证有效。
    struct Decomp {
        double m;              // 质量尺度 (kg)
        double Q[9];           // 3×3 row-major, det Q = +1 (A 为正交尺度阵时它就是旋转矩阵)
        double parity;         // +1 / -1 = sign(det A)
        double sv[3];          // A 的奇异值, 【降序】 σ1 ≥ σ2 ≥ σ3 ≥ 0
        double isotropyRatio;  // σ1/σ3 ≥ 1 (各向同性 = 1)
    };
    bool decompose(const double A[9], Decomp& out);

    // ===== 生效状态 (存 payload_calib.json) =====
    extern bool   enabled;       // true = 已由实机标定; false = 用 Config.h 种子
    extern double massKg;
    extern double comMm[3];
    extern double rmsForceN;
    extern double rmsMomentNm;
    extern int    poses;
    // 生效的传感器安装偏转角 (度), 随 payload_calib.json 的 "sensor_yaw_deg" 持久化。
    // load() 读不到该字段时 (旧文件) 回退 Config::SENSOR_MOUNT_YAW_DEG。
    // ⚠ 这里存的是【文件里的值】; 真正作用于重力模型的是 TcpCalibration 的模块状态 ——
    //   启动时由 main.cpp 把本值 setSensorYawDeg 过去 (本模块不替调用方动全局状态)。
    extern double sensorYawDeg;
    // CZ 的符号约定 (+1 / -1), 随 payload_calib.json 持久化。
    // ⚠ 现在它【只是信息性】的: 数据定不了这个符号 (两种解释拟合残差完全相同), 而写进
    //    机械臂的那份负载本来也改不动 —— 没有任何东西依赖它的取值, 它只决定 comMm 的 Z
    //    按哪种解释折算, 以及存盘时记哪个值。没有任何代码会从求解结果里"定"下它。
    extern double comSignZ;

    // 当前应当下发给机械臂的负载参数: 已标定则用标定值, 否则回退 Config 种子
    void effective(double& massKgOut, double comMmOut[3]);

    // 用求解结果覆写生效值 (仅内存): 质量/质心/rms/姿态数/psi, 以及 enabled = true。
    // 【不碰 comSignZ】—— 它是持久化的显示约定, 不是求解器的输出 (见上)。
    // 注意: psi 只是被记进本模块 (供 save 落盘与启动时取用), 【不会】替调用方调
    // TcpCalibration::setSensorYawDeg —— 见 main.cpp 的显式调用。
    void applyResult(const Result& r);

    bool load(const char* filepath);
    bool save(const char* filepath);
}
