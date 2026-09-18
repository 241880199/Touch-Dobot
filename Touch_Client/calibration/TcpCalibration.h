#pragma once

// 笔尖 TCP 偏移标定: 求解法兰中心 → 笔尖的固定偏移 (spec §4.5)。
// 方法: 笔尖对准固定参考点, 机械臂多姿态记录法兰位姿 (GetPose),
//      最小二乘求解偏移 t 使 p_i + R_i·t = 常数 (笔尖固定点)。
// 约定: RPY → 旋转矩阵采用 R = Rz(rz) * Ry(ry) * Rx(rx) (与 FK/姿态控制一致)。
namespace TcpCalibration {
    // RPY(度) → 3×3 旋转矩阵 (row-major)。输入单位是【度】——与 GetPose 返回的
    // Rx/Ry/Rz 一致, 也和 ForceCompensation::eulerToRotation 的约定一致。
    void rpyToMatrix(double rx_deg, double ry_deg, double rz_deg, double R[9]);

    // 重力在【传感器系】下的表示, 供力补偿与负载求解【共同】使用:
    //     g = Rz(-psi) · Rᵀ · (0,0,9.81)      psi = sensorYawDeg()
    // pose 为 [x,y,z,rx,ry,rz] (mm, 度)。
    //
    // 【为什么必须共用这一个函数】两个模块各自实现过一次重力约定, 结果一个写了 Rᵀ、
    // 另一个写了 R (转置), 求解器因此安静地解错 (残差 0.68 N 而非 0.045 N, 不报错)。
    // 约定只能有一份实现。
    //
    // 符号约定 (推导, 别改反了): 记传感器系 = 法兰系绕 z 转 +psi 得到 → 同一矢量在
    // 传感器系里的坐标 = Rz(-psi)·(它在法兰系里的坐标)。参考系 = 法兰系 (GetPose 的
    // RPY 描述的那个)。
    //   Rz(-psi) = [[ cos, sin, 0], [-sin, cos, 0], [0, 0, 1]]
    // 即 gx' = cos(psi)·g0x + sin(psi)·g0y, gy' = -sin(psi)·g0x + cos(psi)·g0y, gz' = g0z。
    // (2026-09-18 实机数据: dF = M·dg 解出的 M ≈ Δm·Rz(-78.9°) ⇒ psi ≈ +78.9°, 即上式。
    //  写成 Rz(+psi) 会让残差【变大】而不是变小 —— 方向反了控制台立刻看得出来。)
    void gravitySensorFrame(const double pose[6], double g[3]);

    // 同上, 但 psi 显式给出(度), 且【不读写模块状态】。
    // 存在的唯一理由: 负载标定要扫 psi —— 扫描时若反复 setSensorYawDeg, 一旦中途失败
    // 就会给运行时留下一个被污染的安装角。数学仍然只有一份: 二者走同一个
    // rotateGravityByYaw (见 .cpp), 不是各写一份公式。运行时路径请用 gravitySensorFrame。
    void gravitySensorFrameAtYaw(const double pose[6], double psiDeg, double g[3]);

    // 传感器相对法兰绕工具 z 的安装偏转角 (度)。由负载标定解出并持久化;
    // 未标定时用 Config::SENSOR_MOUNT_YAW_DEG 作种子。
    void   setSensorYawDeg(double deg);
    double sensorYawDeg();

    // 纯函数: 求解 TCP 偏移。
    // poses: n 个法兰位姿 [x,y,z,rx,ry,rz], n >= 3
    // offsetOut: 输出偏移 (法兰系, mm); rmsOut: 拟合残差 (mm)
    // 返回 false 表示退化 (姿态不足/共线/秩亏)
    bool solve(const double poses[][6], int n, double offsetOut[3], double& rmsOut);

    // 纯函数: 应用 TCP 偏移, 求笔尖世界坐标。
    // pose: 法兰位姿 [x,y,z,rx,ry,rz]; toolOffset: 法兰系偏移; tipOut: 输出笔尖坐标
    void apply(const double pose[6], const double toolOffset[3], double tipOut[3]);

    // ===== 标定状态 (存 tcp_calib.json) =====
    extern bool enabled;
    extern double offset[3];
    extern double rmsError;

    // ===== 采集状态 (笔尖对准固定点, 多姿态记录) =====
    extern bool collectMode;
    extern int  collectCount;
    static const int MAX_COLLECT_POSES = 50;
    extern double collectPose[MAX_COLLECT_POSES][6];

    bool load(const char* filepath);
    bool save(const char* filepath);
    void startCollect();
    void cancelCollect();
}
