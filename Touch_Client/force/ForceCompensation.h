#pragma once
#include "../core/AppState.h"

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

    // Call at ~125Hz (from ForceReader thread) or ~30Hz (from pollForce)
    // fd.sixForceRaw[] (@1304, 原始读数) must be fresh; poseRxyz = {X,Y,Z,Rx,Ry,Rz} in mm & deg
    // from GetPose(). Writes fd.compensated[] (6-axis compensated force).
    // ⚠ 输入通道是 @1304, 【不是】@576 (fd.raw) —— 见下面全量模型的说明。
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
    void setCalibration(const double A[9], const double biasForce[3],
                        const double biasTorque[3], const double comSensor[3]);

    // 读出当前生效的全量模型 (供「仅调零」把 A / c_s 原样保留着写回文件)。
    void currentModel(double A[9], double comSensor[3]);

    // 取出当前零偏 (供「仅调零」把新零偏写回文件时复用)
    void currentBias(double biasForce[3], double biasTorque[3]);

    // 诊断用: 把运动检测器的当前状态与判定读出来。返回 isStill() 的当前值。
    // 存在的理由: isStill() 疑似在生产中永远为假 (那样在线 EMA 零偏更新就不跑),
    // 需要用实机噪声量级来判定, 而不是靠读代码猜。
    // 注意 vel/acc 实际单位是 m/s 与 m/s² (MotionEstimator::update 里已 mm→m 换算),
    // 与阈值常量同量纲 —— 可疑的不是单位, 是 update() 的 dt 与 pollForce() 实际
    // 33ms/100ms 的采样节奏不符 (见 main.cpp 里 runMotionProbe 的注释)。
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
