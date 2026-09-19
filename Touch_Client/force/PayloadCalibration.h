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

    // 逐姿态的【实测噪声】—— 报告量, 外加尺子的"扣噪项", 与任何模型、任何重力约定无关。
    //
    // 每个姿态是 N 个样本的平均, 所以【均值】的 1σ 不确定度 = sqrt(var/N) (标准误)。方差取
    // 样本方差 (无偏, 除以 N−1)。这批数是采集时顺手采下来的 (main.cpp 的 BiasCheck::sample:
    // 与均值同一批样本、同一时间窗), 不是从拟合残差里反推的 —— 所以它没有"自己量自己"的毛病。
    //
    // ⚠ 【但它不是模型形式检验的尺子 —— 这一条本模块栽过两次, 两次的病都在这里】
    //   它量的是【姿态内】的散布; 而模型形式错是【姿态之间】的系统差。同一个姿态多采几秒
    //   读数只会更稳, 不会因为模型错而散开 —— 姿态内噪声对"形式错"是【盲的】。拿它当尺子
    //   两个方向都错 (数都在 task-1-report 的 Fix wave 1 / 2 里):
    //     · 正解被拒: 7 姿态实机数据 rmsF = 0.0224 N, 而姿态内 σ̄ 只有 ~0.011 N
    //       (force_demo_log.csv 同一姿态 76 个样本, 逐轴 sd 0.0985/0.0191/0.0098 N) ——
    //       残差里那 2 倍于姿态内噪声的部分不是"噪声", 是姿态间复现性, 却被当成罪证;
    //     · 门限自指: 尺子从残差来 -> 形式错 -> 残差涨 -> 尺子涨 -> 门对着自己放水。
    //   所以它现在只做两件事: 报"这个姿态采得稳不稳"; 从重复对的差值里扣掉噪声那一份
    //   (见 RepeatPair 与 .cpp 的 yardstickPooled)。
    struct PoseNoise {
        int    n;         // 该姿态参与平均的样本数 (1 = 没平均, 但方差已知)
        double varF[3];   // 力三个分量的样本方差 (N²)
        double varM[3];   // 力矩三个分量的样本方差 (N·m²)
    };

    // 【姿态间复现性】的观测: 采集中【同一个姿态访问了两次】的那两行的下标。
    //
    // 为什么必须有它: 模型形式检验问的是"姿态与姿态之间, 数据的走向是不是模型说的那样"。
    // 唯一能给出这个尺度的实测, 是【回到同一个姿态再采一次】(中间要有真实运动, 否则第二次
    // 只是第一次的复制)。两次访问的差别里, 姿态相关的那一份 (位姿复现、迟滞、漂移、以及
    // 真的模型形式错影响不到的位姿误差) 与"模型形式错"以同一条路径进到数据里 —— 姿态内
    // 噪声则进不到。所以它是对的量尺。
    //
    // 【必须是标记, 不是猜】: 这两行下标由采集端【显式登记】(main.cpp 的 BiasCheck: 操作员
    // 摆完所有姿态后回到第 1 个姿态, 按 'r' 再采一次, 登记的就是 (0, 那一次))。【不能】用
    // "姿态距离小于某容差就算同一个姿态"去认 —— 那就是一个预设, 而且"差多少算同一个姿态"
    // 正是这里要量的事情。
    //
    // ===== 它是【一串】对, 不是一对 (2026-09-19 第三次修复) =====
    // 按 'r' 是【追加】—— 每按一次多一对。为什么必须能有多对:
    //   · 尺子自己也有【自由度】。一对只给 1 个自由度 (见 .cpp 的 yardstickPooled 与
    //     fitRaw 的判决), 而一对的估量本身离散极大: 两次访问凑巧对得很齐 (d≈0) 与凑巧差
    //     很多 (d 大) 是常事, 于是"这次到底该不该判模型错"在一对之下几乎由运气决定。
    //     多池化几对, 尺子的离散按 1/sqrt(N_pairs) 收敛, 判决才稳。
    //   · 因此"判决贴着线过"时【补一对】是真的有用 —— 而不是把同一个 1 自由度的估量
    //     再算一遍 (从前的实现按 'r' 会【覆盖】前一对, 那个补救承诺是空的)。
    // 数组由调用方持有, count 为元素个数; nullptr / count ≤ 0 / 下标越界 / 两次是同一行 =
    // 没有可用的对 -> 模型形式【没有尺子】, fitRaw 因此返回 false (见 ModelFormStatus /
    // ModelFormPolicy)。
    struct RepeatPair {
        int first;    // 第 1 次访问的下标 (协议里是 0; 机制本身不依赖这一点)
        int second;   // 第 2 次访问的下标 (与 first 之间要有真实运动)
    };

    // 模型形式检验的【尺子状态】—— "没验过"与"验过、通过了"必须一眼分得开, 而且
    // "尺子为什么没有"要能分开报 (通道坏了 vs 协议没走完, 是完全不同的两件事)。
    enum ModelFormStatus {
        MODEL_FORM_OK           = 0,   // 尺子齐备 -> 检验真的做了 (modelFormChecked = true)
        MODEL_FORM_NO_NOISE     = 1,   // 没有逐姿态噪声估计 (没传 / N<1)
        MODEL_FORM_NO_REPEAT    = 2,   // 没有重复姿态对 -> 量不出姿态间复现性
        MODEL_FORM_DEAD_CHANNEL = 3,   // 某通道【整批】方差恒为 0 而别的通道是活的: 通道冻住/没接上
        MODEL_FORM_NOISE_HOLES  = 4,   // 【个别】姿态/通道方差为 0 (通道没死, 是那一笔没采到)
        MODEL_FORM_NO_DOF       = 5,   // 残差自由度 0 (姿态数不足) -> 检验做不了
    };

    // 【显式放弃模型形式检验】的令牌 —— 名字故意写得刺眼, 必须由调用方【逐字】写出来。
    // 只有离线重放/回归这类"手上本来就没有采集现场、拿不到重复姿态对"的场合才该用它。
    // 生产路径 (实机采集 -> 求解) 一个字都不该出现它: 那时 O_ 后面写的每一句都是
    // "我明知道这批数据的模型形式没被检验过, 仍要用它定参数"。
    enum ModelFormPolicy {
        MODEL_FORM_REQUIRED            = 0,   // 默认: 尺子不齐 -> 返回 false, 不给参数
        I_ACCEPT_UNVERIFIED_MODEL_FORM = 1,   // 我认了: 没验过也给我参数 (modelFormChecked 仍为 false)
    };

    // 模型形式检验的【显著性水平】—— 【不是物理阈值】。它是统计惯例: "我愿意认几成的
    // 冤枉率"。检验统计量是残差与尺子的比值 (χ²_rep/dof), 它自己的分布由数据定; 这个数只
    // 决定在它的分布上切在哪一刀。推导与实测的冤枉率见 .cpp 的 modelFormLimit。
    // 现在【两侧都用它】: 分子的 χ² 分位数、分母 (尺子) 的单侧置信下界。
    // 逐姿态残差的报告上限 (超过就截断: 只影响打印, 不影响判决)。
    static const int RAW_POSE_REPORT_MAX = 16;

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

        // ===== 检验用的报告字段 =====
        // 【尺子 1: 姿态内噪声 —— 只报告, 不再是判据】
        // 力通道: chi2ForceRatio = Σ(e/σ_in)² / dofF, σ_in = sqrt(var/N) 是各姿态各通道的
        //         均值 1σ; 力矩通道同式 (chi2MomentRatio), 但它的回归量 w = A·g 里带着 A 的
        //         估计误差 (errors-in-variables), 那个残差不是纯噪声统计量, 更不能判。
        // 【为什么不再判】: 姿态内噪声量不出姿态【间】的系统差 —— 那正是模型形式错的样子。
        // 这两个数现在是诊断: 残差远大于姿态内噪声、同时与姿态间复现性相符, 就是"采集现场
        // 的复现性就这么差"; 两者都不符, 才是"模型形式错"。见 fitRaw 的自检 3。
        double noiseForceN;          // 力通道姿态内噪声的合并尺度 (N) = sqrt(mean(σ_in²))
        double noiseMomentNm;        // 力矩通道同上 (N·m)
        double chi2ForceRatio;       // χ²_F(姿态内) / dofF —— 【只报告】
        int    chi2DofForce;
        double chi2MomentRatio;      // χ²_M(姿态内) / dofM —— 【只报告】(理由见上)
        int    chi2DofMoment;

        // 【尺子 2: 姿态间复现性 —— 判据用这一把】
        // 由【重复姿态对】测出 (见 RepeatPair): 同一姿态两次访问之差, 扣掉姿态内噪声那一份,
        // 剩下的才是"换一次姿态再回来, 读数能差多少"。残差与它比, 问的才是正确的问题:
        // "模型的失配, 有没有超出这台设备复现同一个姿态的能力?"
        int    repeatFirst;          // 池化的第 1 对的两次访问下标; -1 = 没有可用重复对
        int    repeatSecond;
        int    repeatPairCount;      // 池化了几对 = 尺子的【自由度】; 0 = 没有尺子
        double repeatDiffF[3];       // 池化后各通道的 |两次访问差| 尺度 (N) —— 照实报
        double repeatDiffM[3];       // 同上 (N·m)
        double repeatSigmaF[3];      // 逐通道姿态间复现性 σ_rep (N) —— 尺子本身 (池化)
        double repeatSigmaM[3];      // 同上 (N·m)
        // 尺子的【两个分量】: σ_rep² = σ_sys² + floor² (池化后)。
        // σ_sys² = "回到同位姿再来一次的离散"; floor² = 单次测量自身有多准 (姿态内噪声)。
        // 判决门限要按 σ_sys²【自己的自由度】放宽 (那是估量, floor² 是申报的已知量), 所以
        // 这两个分量必须都留着。
        double repeatSysF[3];        // σ_sys² (N²)
        double repeatSysM[3];        // σ_sys² (N·m)²
        double repeatFloorF[3];      // floor² (N²)
        double repeatFloorM[3];      // floor² (N·m)²
        double chi2RepForceRatio;    // Σ(e/σ_rep)² / dofF —— 【判据】; 正确模型下期望 ≈ 1
        double chi2RepForceLimit;    // 上面那个判据的门限 (推导见 .cpp 的 modelFormLimit)
        double lackOfFitMomentRatio; // Σ_a(ssM_a − ssFree_a)/σ_rep,M,a² / 6 —— 【判据】
        // 上面的统计量【在零假设下不是以 1 为中心】: 力矩模型复用力通道估出来的 A, Â ≠ A
        // 时叉乘结构吃不掉那一份, 统计量系统性抬高 (实测中心 1.5~5.3)。所以门限是
        //     lackOfFitMomentLimit = c0(α,R)·modelFormLimit(6,R,σ_sysM²,floor_M²) + κ(α,R)·e
        // 而不是 modelFormLimit 自己 (力那一条仍然只用 modelFormLimit, 未改)。
        // 推导、c0/κ 的出处、五条限制、以及离线验证都在 .cpp 的 momentFormLimit 上面。
        double lackOfFitMomentLimit; // 同上, 门限 (重标定过的, 见上)
        // e = δA 引起的【期望】多余量, 以统计量自己的单位计 (无量纲) —— 就是上式里那一项。
        // 由 fit 自己的 A / cS / repeatSigmaM 与三明治 Σ_A (从 PoseNoise 建, 【不是】paramSigma)
        // 经 G/H 二次型算出; 算不出来 (矩阵不可逆等) 时报 0, 门限随之退回 c0·modelFormLimit。
        // 【口径自校】三份冻结夹具上它给出 0.55771 / 0.48289 / 0.48102, 与离线报告
        // (Docs/superpowers/evidence/limit-recalibration-report.md §1.3) 五位小数逐位相同。
        double lackOfFitMomentExcess;
        int    lackOfFitMomentDof;   // 6, 或 0 = 做不了 (方程数不足以养自由模型)

        // ===== 逐姿态残差 (spec §3 表格第 4 行) —— 分得清"一个坏姿态"与"整体形式错" =====
        // 只有标量 rms 时, 这两种情形长得一模一样, 而处置完全不同 (重采那一个姿态 vs 换模型)。
        double poseResidualF[RAW_POSE_REPORT_MAX];   // sqrt(Σ_a e²/3)  (N)
        double poseResidualM[RAW_POSE_REPORT_MAX];   // 同上 (N·m)
        double poseResidualRatioF[RAW_POSE_REPORT_MAX];  // 残差 ÷ 姿态级尺子; 没尺子时 0
        int    poseResidualCount;    // 实际填了几行 (= n, 超过上限时截断)
        int    worstPoseF;           // 力残差最大的姿态下标; -1 = 没算

        int    modelFormStatus;      // ModelFormStatus —— 尺子的状态 (为什么有/没有)
        bool   modelFormChecked;     // true = 力通道【有尺子且判决通过】
        // true = 力矩通道的失拟检验【做了且通过了】—— 与 modelFormChecked 同义 (都是
        // "这一半验过了")。从前它只表示"检验做了", 于是在【力通道被判错、整体拒绝】时
        // 它照样是 true, 读起来像"至少那半边是好的"。现在两个标志只在同一个地方置 true。
        bool   momentFormChecked;
    };

    // 模型形式检验的【判决门限】: 拒绝 ⇔ 统计量 > modelFormLimit(...)。
    //   dofFit   = 分子 (残差) 的自由度: 力通道 3n−12, 力矩失拟 6
    //   repPairs = 分母 (尺子) 的自由度 = 池化进来的重复对数
    //   sys2     = 尺子的【估量】那一半 (σ_sys², 逐通道); floor2 = 申报的已知那一半 (floor²)
    // 尺度全部来自这两个实测/申报量; α 只是【置信水平】(见 .cpp 的 RAW_MODEL_FORM_ALPHA)。
    // ⚠ α 不是显著性水平, 也不单调同向: 两个 χ² 分位数都随 α 增, 所以【调大 α 才放宽门限】。
    // 推导、它为什么不是 "1 + K·sqrt(2/dof)"、以及实测的两个错误率 (冤枉率表 + 三个错误模型的
    // 余量表) 都写在 .cpp 的 modelFormLimit 上面。公开它只为让测试与诊断能【独立复算】门限 ——
    // 生产路径不需要直接调它 (fitRaw 已经把结果放进 RawFit::chi2RepForceLimit 等字段)。
    //
    // ===== 两处【已知的近似】—— 是设计取舍, 不是待查的疑点; 别在原地重构 =====
    //
    // (A) 池化 σ_sys² 逐对截断在 0, 而折扣又是【同一个估量】的函数。
    //     repeatSysF[a] = mean_j max((d_j² − 2·var_a)/2, 0) —— 那个 max(·,0) 让估计量在 0 上
    //     有一个原子。后果是: 恰好量到 0 的时候 (s == 0), modelFormLimit 走"没有测到复现性差
    //     -> 不打折"那一支, 直接返回【最严】的 base —— 也就是在尺子【最不可靠】的时候给它
    //     【最紧】的门限, 方向恰好是反的。这就是 R=1 那一截残余冤枉率的来源: 零假设下统计量
    //     的实际均值不是 1 而是 1.6 上下 (dof=12; dof=9 时 1.5; 评审在它自己的生成器上量到
    //     ~1.9, 口径不同, 方向一致)。
    //     【为什么不做】: 一个更保守的构造 (比如别在 0 处截断、或给尺子取单侧置信【上】界) 能
    //     把这个偏置去掉, 且实测代价为零 —— 错误模型那边落在 1107, 离门限差着两个数量级,
    //     放宽一点也不影响拒绝。本轮按 brief 明令【不重构统计量与池化】, 所以只在此备案:
    //     它是一个【已知属性】, 不是无法解释的残差。
    //
    //     ⚠⚠ 2026-09-19 实测更正 —— 【上面这条备案只适用于力通道(dof=12/9);
    //        套到力矩失拟那条门上, 方向是反的】。三份实机采集 × 两套噪声模型、生产 fitRaw:
    //          · 去掉 max(·,0) 截断  ->  6 格中 5 格门限【收得更紧】(两格直接塌回 base 4.643),
    //             冤枉率最高【涨 2.6 倍】(4.985% -> 12.780%);
    //          · 把 σ_sys² 从分母里拿掉  ->  统计量反而【涨 30~108%】。
    //        也就是说: 在力矩这条门上, 那个截断是【压】统计量的, 不是抬的。备案里那句
    //        "去掉它能把偏置去掉、且实测代价为零"对力通道成立, 对力矩分支【不成立】。
    //
    //     ⚠ 而且力矩分支的主导成因【根本不是】这条 (A): 是 **δA** —— 力矩模型复用的是
    //        【力通道估出来的 A】, 而它自己的叉乘结构吃不掉 A 的估计误差。实测占零假设膨胀的
    //        **88%**, 超额 **∝|δA|²** (剂量响应 0.249~0.254 @s=0.5, 预测 0.25)。
    //        实测冤枉率: 力矩分支 R=3 约 7~23%、R=5 约 1~5%; 同一次蒙特卡洛里力通道 0/12000。
    //        出处: Docs/superpowers/evidence/moment-gate-diagnosis-report.md、
    //              moment-gate-dA-correction-report.md。
    //
    //     ⇒ 教训 (与本模块栽过多次的那一类同型): 【一份写对的备案, 被移植到一个结构不同的
    //        分支上, 就成了错的】。读到"已知属性, 不是待查疑点"时, 先确认它是对哪条通道写的。
    //
    // (B) 三通道先合成一个标量 r 再打折, 而统计量是【逐通道求和】。
    //     r 取 Σ_a σ_sys,a² / Σ_a floor_a² (与 poseLevelYardstick 同一口径), 于是三个通道
    //     被强行等权地打同一个折 —— 这与"逐通道 σ_rep,a² 各自进分母"的统计量并不严格同构。
    //     偏差在尺子【不平衡】(三通道的 σ_sys²/floor² 差得多) 时最大: 评审实测的构造偏斜时
    //     R=1/2 的冤枉率 12.3%/3.5%, 而平衡时 8.1%/2.0%。实机上三通道的底噪本来就不等
    //     (force_demo_log.csv 逐轴 sd 0.0985/0.0191/0.0098 N), 所以这个偏差是【存在但不大】。
    //     【为什么不做】: 改成逐通道打折属于重构统计量, 本任务明令不动。
    double modelFormLimit(double dofFit, int repPairs,
                          const double sys2[3], const double floor2[3]);

    // 【力矩分支的门限】(2026-09-19 重标定) —— 拒绝 ⇔ lackOfFitMomentRatio > momentFormLimit(...)。
    //   excess   = e = δA 引起的期望多余量, 以统计量自己的单位计 (见 RawFit::lackOfFitMomentExcess;
    //              <= 0 当作 0 处理 = 没有修正项, 门限退回 c0·modelFormLimit)
    //   repPairs = 重复对数 R (尺子的自由度)。标定表只有 R = 1/3/5 三个点, 其余
    //              【区间内线性插值, 区间外平夹】—— 理由与它没被验证的那两段都写在
    //              .cpp 的 momentFormCoeffs 上面。
    // 门限 = c0(α,R)·modelFormLimit(6,R,σ_sysM²,floor_M²) + κ(α,R)·e, α = RAW_MODEL_FORM_ALPHA。
    // 【只有力矩这一条支路用它】: 力分支仍然只用 modelFormLimit, 一个字没改。
    // 公开它, 与 modelFormLimit 同一个理由: 让测试与诊断能【独立复算】门限
    // (测试里 moment_gate_limit_is_c0_times_prod_plus_kappa_times_e 就是这么钉的)。
    double momentFormLimit(double excess, int repPairs,
                           const double sys2[3], const double floor2[3]);

    // 纯函数: 线性拟合并做【物理自检】。返回 false = 拒绝给出参数 (原因打到 stderr),
    // 此时 out 里是【未经自检】的线性解, 只供诊断打印, 调用方不得采用。
    // 拒绝的情形 (判据全部由数据/量程给出, 没有一个固定比例是猜的):
    //   1) 模型形式: 拟合残差超出【姿态间复现性】(重复姿态对测出来的尺子)
    //   2) 质量尺度: m ≤ 0 或超出 CR3 的负载量程 (EnableRobot 的量程)
    //   3) 条件数: cond 过大 -> 姿态激发不足, 12 个参数定不下来
    //   4) ★ 尺子不齐 (没有重复姿态对 / 没有逐姿态噪声 / 通道冻住 / 自由度 0):
    //      【模型形式没被检验过 —— 那就【不给参数】】, 见下面的 policy。
    // 返回 false 也可能是线性层就失败: 姿态数 < 4 (12 个未知) 或姿态退化 (J 秩亏)。
    //
    // ===== 调用契约 (2026-09-19 二次修复, 评审 #2 的要点) =====
    // 【"没验过"不再是"通过"】。上一版把"没有尺子"当成"不做检验但照样给参数", 于是
    // 参数能在一个【从未被检验过形式】的模型上落地 —— 而模型形式错正是本项目栽得最惨的那
    // 一件事 (安静地解错, 残差报不出来)。所以现在的契约是:
    //   · 尺子齐备 (modelFormStatus == MODEL_FORM_OK) -> 检验通过才返回 true;
    //   · 尺子不齐 -> 打一条【说清楚是哪一种不齐】的消息, 并返回 false。
    // 只有逐字写出 I_ACCEPT_UNVERIFIED_MODEL_FORM 的调用方 (离线重放, 现场数据本来就不全)
    // 才能拿到参数; 那时 out.modelFormChecked 仍然是 false —— 它【不会】被伪装成通过。
    //
    // noise   = 逐姿态实测噪声 (从重复对的差值里扣掉"噪声那一份"要用它; 也是报告量)。
    // repeats = 重复姿态对的【数组】(尺子的来源); repeatCount = 元素个数。
    //           [1, count) 里的每一对都并进同一把尺子, 尺子的自由度 = 池化进来的对数
    //           (不合法的对会被跳过, 不进自由度)。count ≤ 0 / nullptr = 没有尺子。
    // 前置条件: out 的内存由调用方持有; noise 非空时其 n 个元素必须与 poses/forces/moments
    //           同序; repeats 里每一对的两个下标都必须落在 [0, n) 且不相等。
    bool fitRaw(const double poses[][6], const double forces[][3], const double moments[][3],
                int n, RawFit& out, const PoseNoise* noise = nullptr,
                const RepeatPair* repeats = nullptr, int repeatCount = 1,
                ModelFormPolicy policy = MODEL_FORM_REQUIRED);

    // 纯函数: 只做线性拟合, 【不做任何物理自检】。
    // 单独暴露的理由: 自检是【物理结论】的门, 而"任意 3×3 能否被复原"是【线性代数】的性质 ——
    // 两者必须能分开测。一个明显非正交 (甚至带反射) 的 A 可以被精确复原, 而它该不该被采信
    // 是另一个问题; 只有把两层分开, 这两件事才各有各的断言。(生产路径请用 fitRaw。)
    // noise / repeats 只影响那些【检验用】的报告字段 (尺子、χ²、逐姿态残差), 不参与求解 ——
    // 拟合本身永远是无权的普通最小二乘 (加权的更换属于改线性代数, 不在本任务内)。
    bool fitRawLinear(const double poses[][6], const double forces[][3], const double moments[][3],
                      int n, RawFit& out, const PoseNoise* noise = nullptr,
                      const RepeatPair* repeats = nullptr, int repeatCount = 1);

    // 从 A 读回物理量 —— 这一步才是"标定", 不是"猜"。
    //
    // 约定: A = m · S · Q, 其中
    //   m   = (σ1σ2σ3)^(1/3) > 0                     质量尺度
    //   S   = diag(1, 1, parity)                     手系镜像 (parity = sign(det A), 固定约定)
    //   Q   = S·A/m                                  含手系的完整三维安装姿态, det Q = +1
    // 于是 round-trip 逐位精确: m·S·Q = m·S·(S·A/m) = A, 对【任意】可逆 A 都成立, 不要求 A 正交。
    // 三个输出因此是【唯一】的 —— 不需要在"取哪个正交因子"之间做选择。
    //
    // (历史: spec §3 原文写的是 Q = U·diag(1,1,parity)·Vᵀ, 与这里的实现【不一致】。2026-09-19
    //  评审独立验证后, §3 已更正为 Q = S·A/m —— 就是下面这个式子。别再把代码"改回去":
    //  那个正交因子式在 σ1=σ2=σ3 (即安装合格) 时 U/V 不唯一, 式子跟着不唯一, 且在 σ 不全
    //  相等时不满足 A = m·S·Q。见 .cpp 的说明与 spec §3 的更正块。)
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
