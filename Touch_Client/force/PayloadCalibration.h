#pragma once

// 末端负载标定 (质量 + 质心) —— 由实机多姿态空载数据解出 EnableRobot 的负载参数。
//
// 背景: 机械臂内部按 EnableRobot(load,cx,cy,cz) 做重力/惯性补偿, 30004 的
//       ActualTCPForce 是补偿【之后】的净力。负载参数不准 → 读数随姿态漂移。
//       这些参数只能实测, 所以本模块把它们从编译期常量变成可标定量 (payload_calib.json)。
//
// 模型 (工具系; g_i = R_iᵀ·(0,0,9.81) 是重力在工具系的表示):
//   raw_F_i = b_F + Δm · g_i
//   raw_M_i = b_M + Δp × g_i
// 其中 Δm = m_true − m_cfg, Δp = m_true·c_true − m_cfg·c_cfg。
// 对姿态 0 做差可同时消掉传感器零偏 b_F / b_M:
//   ΔF_k − ΔF_0 = Δm · (g_k − g_0)
//   ΔM_k − ΔM_0 = Δp × (g_k − g_0)
// 这是关于 (Δm, Δp) 的线性最小二乘 (4 个未知量, 每个姿态 6 个方程)。
//
// 解是【带符号】的, 所以既给出"往哪边调、调多少", 也顺带把 EnableRobot 的
// Z 偏心符号约定定死 (符号错了, 解出来的修正量会把误差修回去, 复验残差仍大)。
namespace PayloadCalibration {

    // 求解结果
    struct Result {
        double dm;           // 质量修正量 (kg, 带符号)
        double dc[3];        // 质心修正量 (mm, 带符号)
        double massKg;       // 换算出的绝对质量 (kg)
        double comMm[3];     // 换算出的绝对质心 (mm, 法兰系)
        double rmsForceN;    // 力通道拟合残差 (N)
        double rmsMomentNm;  // 力矩通道拟合残差 (N·m)
        int    poses;        // 参与求解的姿态数

        // ===== CZ 符号自动判定 =====
        // signZ 不进 buildRows, 所以两种符号的拟合残差完全相同 —— 数据本身
        // 区分不了符号, 必须用外部判据: 取离 Config::ROBOT_PAYLOAD_SEED_CZ_MM
        // 更近的候选 (距种子比值差 ≥ SIGN_SEED_MARGIN_RATIO 才采纳),
        // 再用"物理质心 Z 必须为正"交叉复核。
        double signZ         = 1.0;    // 实际选用的符号约定 (+1 / -1)
        double cTrueZ[2]     = {0.0, 0.0};  // {候选+1, 候选-1} 下的物理质心 Z (mm)
        bool   signAmbiguous = true;   // 默认 true = 不可信, 不让漏填的 Result 看起来可用
    };

    // 纯函数: 余量是否足够采纳锚点选出的候选。
    // near = 选中候选到种子的距离, far = 另一个候选的距离。
    // 距离比必须 ≥ Config::SIGN_SEED_MARGIN_RATIO, 否则判不可判定。
    bool seedMarginSufficient(double near, double far);

    // 纯函数: 最小二乘求解。无全局状态, 便于单测。
    //   poses:    n 个法兰位姿 [x,y,z,rx,ry,rz], 角度单位【度】(= GetPose 的返回)
    //   forces:   n × 3 各姿态平均原始力 (N)
    //   moments:  n × 3 各姿态平均原始力矩 (N·m)
    //   mCfg/comCfg: 当前机械臂里配置的负载 (即上面公式的 m_cfg / c_cfg, com 单位 mm)
    //   comSignZ: 机械臂解释 CZ 偏心的符号约定 (+1 或 -1)。见下。
    // 返回 false: 姿态数不足、姿态退化 (共线/同姿态)、解非物理 (质量 ≤ 0 或不合理)
    //
    // 【为什么需要 comSignZ】我们下发的 c_z 与机械臂内部实际使用的 c_eff_z 可能差一个
    // 符号 (c_eff = sign·c_z)。若模型里假设的 sign 与真实相反, 解出的修正量不是把误差
    // 抵消掉而是翻倍, 迭代会发散 (c_next ≈ c_true + 2c_cfg, 且不收敛)。所以把符号
    // 做成可运行时翻转的参数: sign 对了, 一次求解即收敛。
    bool solve(const double poses[][6], const double forces[][3], const double moments[][3],
               int n, double mCfg, const double comCfg[3], double comSignZ, Result& out);

    // ===== 生效状态 (存 payload_calib.json) =====
    extern bool   enabled;       // true = 已由实机标定; false = 用 Config.h 种子
    extern double massKg;
    extern double comMm[3];
    extern double rmsForceN;
    extern double rmsMomentNm;
    extern int    poses;
    extern double comSignZ;      // 机械臂解释 CZ 的符号约定 (+1 / -1), 随文件持久化

    // 当前应当下发给机械臂的负载参数: 已标定则用标定值, 否则回退 Config 种子
    void effective(double& massKgOut, double comMmOut[3]);

    // 用求解结果覆写生效值 (仅内存)
    void applyResult(const Result& r);

    bool load(const char* filepath);
    bool save(const char* filepath);
}
