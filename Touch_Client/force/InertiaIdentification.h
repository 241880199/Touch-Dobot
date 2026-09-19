#pragma once

// 工具链惯量张量的【动态辨识】—— 让机械臂按【已知频率】的腕部往复激励运动, 从 @1304 原始
// 力旋量里解出绕【传感器测量原点】的惯量张量 I_O。这是纯离线数学: 不碰机械臂, 不发指令。
// (激励采集本身是后续任务; 本模块只吃一段采好的 InertiaSample 序列。)
//
// 为什么要有它: 机械臂的 PayLoad(weight, inertia) 要惯量, 而从 CAD 算惯量【是对形状做假设】——
// compute_payload.py 里有三处各自独立的形状假设, 每处都能让 Izz 动 10~35%。用户的判词是
// "任何猜测以及偏差(如 psi)都不可取"。所以惯量必须【测】, 不能算。
// (spec: Docs/superpowers/specs/2026-09-19-raw-channel-calibration-design.md §6c)
//
// ---------------------------------------------------------------------------------------
// 模型与坐标系 (这是本文件最要紧的一段; 搞错了会【安静地解错】—— 数看着合理, 其实翻着符号)
// ---------------------------------------------------------------------------------------
//
// 静力学那一半 (PayloadCalibration::fitRaw, spec §2) 已经把原始通道标定成:
//
//     F_i = b_F + A · g_i              A: 3x3, 9 个元素全自由
//     M_i = b_M + c_s × (A · g_i)      c_s: 质心 (m, 传感器测量系)
//     g_i = TcpCalibration::gravitySensorFrameAtYaw(pose_i, 0.0, g)      <- 【psi = 0】
//
// A 是【测量约定本身】: A = m · S · Q, S = diag(1,1,parity), Q = S·A/m。
// 注意 g_i 不是"重力加速度矢量"—— 它是基座系矢量 (0,0,+9.81) 在传感器系里的坐标
// (Rᵀ·(0,0,+G)), 也就是【比力】方向的反向; 重力加速度在基座系是 (0,0,−9.81)。
// 符号约定整个被 A 吸收 (A 的 9 个元素本来就没有任何约束), 所以本模块【不拆 A】。
//
// 设 W := A / m (通道坐标系相对位姿系的映射; A 恰为 m×正交阵时 W 就是正交阵, 此时
// 各向同性比 σ1/σ3 = 1)。把一个位姿系矢量 v 写成通道坐标 v^ch := W·v。静力学给出
// 静态那一项 F_ch = A·g_s, 而 A·g_s = −m·g^ch_s (g^ch_s = W·g_s)。也就是说通道读的是
//
//     F_ch = b_F + m · ( a_com^ch − g^ch_s )
//
// 即【工具受到的接触力旋量】(牛顿, 作用在质心上), 只是表达在通道坐标里。把 a_com 拆成
// a_O + α×c_s + ω×(ω×c_s), 并把静态那一项代回去 (A·g_s = −m·g^ch_s), 得到本模块用的模型:
//
//   F_meas = b_F + A·g_s + m·a_com^ch                                     (f)
//   M_meas = b_M + c_s × (A·g_s) + m·c_s × a_O^ch
//                         + I_O·α^ch + ω^ch × (I_O·ω^ch)                  (m)
//
// 其中 (m) 的第二项来自 m·c_s × (a_O − g) 里的 a_O 那一半; 静态那一半正好就是
// c_s × (A·g_s) —— 与静力学模型【逐位相同】, 这就是"两半用同一个约定"的判据:
// 零运动 (ω = α = a_O = 0) 时 (f)(m) 精确退化成静力学模型, 残差一字不差。
//
// 由 (f)(m) 扣掉重力项之后的【惯性部分】就是辨识方程:
//
//   F_res := F_meas − b_F − A·g_s                = m · a_com^ch
//   M_res := M_meas − b_M − c_s × (A·g_s) − m·c_s × a_O^ch
//                                                = I_O·α^ch + ω^ch × (I_O·ω^ch)
//
// 【力矩那一条对 I_O 是【严格线性】的】—— α、ω 已知, 六分量线性进入。
// 【力那一条完全不含 I_O】, 它是【运动学的检查】, 不是辨识通道 (spec §6c 明说)。
//
// 输出 I[9] 就是 (m) 里那个 I_O, 即"让 (m) 成立的张量", 以 kg·m^2 计, 行主序, 对称。
//
// ⚠ 它与"绕法兰面的惯量"差两次平行轴平移 (O → 质心 → 法兰面), 本模块【不做】这个换算:
//   平移要显式写出并实测, 属于后续任务 (spec §6c 末段)。
//
// ---------------------------------------------------------------------------------------
// 微分 (spec §6c 点名的"本设计最容易出错的地方")
// ---------------------------------------------------------------------------------------
// 位姿序列按【已知的激励频率】拟合成谐波模型 (基波 + 若干次谐波, 次数由数据用 BIC 定),
// 速度/加速度【解析求导】得到 —— 绝不对原始样本做数值二阶微分。本项目的位姿通道里混着
// 一条 10 Hz 的仪表盘路径, 数值微分会在那里烂掉。
// 独立检查: 拟合出来的速度与机器人自己报的 TCPSpeedActual 比 —— 两个不同来源对上, 才说明
// 微分是对的。这一条是【免费的、来源不同的】自检。

namespace InertiaIdentification {

    // 一个采样点。三段都来自 30004 反馈帧的 125 Hz 通道 (spec §6c 表格)。
    struct InertiaSample {
        double t;           // s
        double pose[6];     // 基座系 x,y,z (mm), rx,ry,rz (度) —— ToolVectorActual @624
        double speed[6];    // TCPSpeedActual @672 —— 【只作独立检查, 不进求解】
        double wrench[6];   // @1304 原始力/力矩 —— 测量值
    };

    // 辨识结果。
    struct InertiaFit {
        // ===== 主要输出 =====
        double I[9];              // 绕传感器测量原点的惯量张量 (kg·m^2), 行主序, 对称
        double rmsForceN;         // 力通道残差 (N) —— 运动学检查的残差, 【不含 I_O】
        double rmsMomentNm;       // 力矩通道残差 (N·m)
        double cond;              // 力矩设计矩阵的 cond = σmax/σmin (激励够不够)
        double sigma[6];          // 六个独立分量的 1σ = sqrt(diag((JᵀJ)⁻¹)·σ²)
                                  // 序: [Ixx, Iyy, Izz, Ixy, Ixz, Iyz]
        double speedCheckRms;     // 拟合速度 vs TCPSpeedActual 的 rms 差 (线性三分量, mm/s)
        bool   ok;                // true = 全部自检通过。false = 【拒绝给参数】, 此时 I[9]
                                  // 是【未经自检】的线性解, 只供诊断, 调用方不得采用。

        // ===== 报告字段 (不进求解; 判据的尺度从它们来, 所以必须看得见) =====
        int    harmonicOrder;         // 谐波拟合选定的次数 K (基波 + K 次谐波)
        double harmonicFitRms;        // 位姿谐波拟合的【归一化】残差 rms (无量纲, 1 = 没拟合上)
        double harmonicSignalRms;     // 位姿谐波拟合解释掉的【归一化】信号 rms
        int    samples;               // 参与求解的采样点数
        double momentNoiseNm;         // 力矩通道的【实测噪底】: 实测力矩对同一次谐波拟合的
                                      // 带外残差 rms (N·m)。与 I_O 无关, 是量出来的。
        double inertiaSignalNm;       // 预测的惯性力矩信号 rms = rms|I·α + ω×(I·ω)| (N·m)
        double inertiaScale;          // 张量自身的量级 = sqrt(Σ I_ij²) (kg·m^2) —— 确定性门限
                                      // 拿它当尺子, 门限就是【比值 1】, 不是拍定的绝对值
        double sigmaMax;              // max(sigma[0..5])
        double eig[3];                // I 的三个特征值 (升序) —— 物理门限 (正定) 用它
        double triangleMargin;        // 主惯量的三角不等式余量 I1+I2−I3。【报告量, 不作门限】:
                                      // 见 .cpp 里 physicalOk 的说明 (它是二阶性质, 噪声可越界;
                                      // 用户手上那份 CAD 参考值本身就差 12 倍)。
        // 逐条自检的结果 —— "没验过"与"验过、通过了"必须一眼分得开
        bool   kinematicsOk;          // 速度两来源一致 + 谐波拟合解释了运动
        bool   determinacyOk;         // 参数被激励定得下来 (不确定度 < 张量自身量级)
        bool   physicalOk;            // I_O 正定 (绕任一点的惯量张量必正定) + 三角不等式
        bool   forceCheckOk;          // 力通道 (运动学检查) 的残差小于惯性力信号
        double forceSignalN;          // 预测的惯性力信号 rms = rms|m·a_com^ch| (N)
    };

    // freqHz = 调用方【命令机械臂跑的】激励频率 (已知量, 不是从数据里猜的)。
    // m, comSensor, A, bF, bM 全部来自静力学标定 (PayloadCalibration::fitRaw 的 A / cS /
    // bF / bM, 以及 decompose(A).m) —— 都是【实测】的, 本模块不再估它们。
    // ⚠ 单位随静力学标定那一侧: m [kg]; comSensor [m] (= RawFit::cS, 力矩方程 c_s × w 里
    //   那个, w 是 N —— 所以它是米不是毫米, 别按 spec §6c 里"54.55 mm"那个写法直接传 54.55);
    //   A [kg]; bF [N]; bM [N·m]。InertiaSample 里的 pose/speed 才是 mm / mm·s⁻¹。
    // 返回 false = 拒绝给出惯量 (原因打到 stderr), 此时 out.ok 也是 false。
    bool identifyInertia(const InertiaSample* s, int n, double freqHz,
                         double m, const double comSensor[3],
                         const double A[9], const double bF[3], const double bM[3],
                         InertiaFit& out);

} // namespace InertiaIdentification

// 规格书里这几个名字是【全局】写的。保留同样的拼写, 免得调用端两套名字。
using InertiaIdentification::InertiaSample;
using InertiaIdentification::InertiaFit;
using InertiaIdentification::identifyInertia;
