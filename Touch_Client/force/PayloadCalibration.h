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
