#pragma once

// Force sensor calibration — static bias (TARE) + 全量模型参数表 (force_calib.json)
//
// ⚠ 2026-09-19 (Task 6): 落盘的【不再是】旧的"标量残余质量 + 零偏"格式。
//   本地补偿已换成全量模型 compensated = sixForceRaw(@1304) − b_F − A·g − Fi
//   (力矩 − b_M − c_s×(A·g)), 所以文件里存的是 A(9)/b_F(3)/b_M(3)/c_s(3)。
//   旧文件 (version 2, mass_kg) 【被拒】且【响亮地说出来】—— 见 .cpp 里的说明。
namespace ForceCalibration {

    enum class State {
        IDLE,
        TARE,      // 2s static collection for force/torque bias
        MOTION,    // User moves robot; record F vs a to fit mass
        SOLVE,     // Fit mass + apply results
        DONE,      // Success
        ABORTED    // User interrupt or safety trip
    };

    // Start calibration
    bool start();

    // 仅调零 (TARE only): 静置采集零偏 → 直接应用+存盘, 不进 MOTION 相、不开拖拽模式
    // 用于换装工具 (笔夹/笔) 后重新调零, 无需拖动机械臂
    bool startZero();

    // 当前是否处于「仅调零」流程
    bool isZeroing();

    // Abort immediately
    void abort();

    // SPACE: start/stop sampling in TARE/MOTION phases
    void confirmPose();

    // Drag mode callback (called on state transitions)
    void setDragModeCallback(void (*cb)(bool enable));

    bool isRunning();
    bool isDone();
    State currentState();
    const char* statusText();

    // Called each frame from pollForce (~30Hz)
    bool update(double dt, const double raw[6], const double pose[6]);

    // Persistence —— 全量模型的参数表 (version 3)。
    //   A:          3×3 row-major (kg)          c_s: 质心 (【米】, 传感器测量系)
    //   biasForce:  N                           biasTorque: N·m
    // A / c_s 由 PayloadCalibration::fitRaw 在 @1304 上解出; 本模块的 TARE 只定零偏。
    bool saveToFile(const char* path, const double A[9], const double biasForce[3],
                    const double biasTorque[3], const double comSensor[3]);
    // 返回 false = 文件不存在 / 读不动 / 【不是本格式】(旧版文件) —— 后者会在 stderr
    // 响亮地说出来, 不会安静地退化成"没有标定"。四个输出数组在返回 false 时【未定义】。
    bool loadFromFile(const char* path, double A[9], double biasForce[3],
                      double biasTorque[3], double comSensor[3]);

} // namespace ForceCalibration
