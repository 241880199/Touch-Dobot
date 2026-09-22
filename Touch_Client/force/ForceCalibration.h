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

    // Called each poll tick from pollForce. 【dt 不再由调用方给】——
    // ★ 2026-09-22: 这里从前是 `update(double dt, ...)`，调用方传常数 0.033
    //   (= 名义节拍 FORCE_POLL_INTERVAL_MS，而那个常数是"节流下限"，不是实际节拍)。
    //   实测节拍 46~203ms (均值 92ms) ⇒ TARE 的"静默 0.5s + 累计 2s"在实际时间里
    //   是 ~1.35s + 5.4s，而提示打印的是 2.5s ⇒ 操作员在静默期里就松手 ⇒ 又采到瞬态。
    //   ⇒ 现在按【实测耗时】自算，做法与 ForceCompensation::stepIntervalSec 逐字同构
    //     (那边是 2026-09-21 用同一条理由改的)。
    // ⚠ 本函数的调用点【必须每次轮询都调】—— 实测间隔的计时器靠它保持新鲜，见 .cpp 里的说明。
    bool update(const double raw[6], const double pose[6]);

    // 用例专用: 把 dt 变成【确定的输入】(默认 -1 = 不干预, 走实测)。
    //   与 ForceCompensation::setStepDtForTest 同一个约定、同一条理由。
    void setUpdateDtForTest(double sec);

    // Persistence —— 全量模型的参数表 (version 3)。
    //   A:          3×3 row-major (kg)          c_s: 质心 (【米】, 传感器测量系)
    //   biasForce:  N                           biasTorque: N·m
    // A / c_s 由 PayloadCalibration::fitRaw 在 @1304 上解出; 本模块的 TARE 只定零偏。
    bool saveToFile(const char* path, const double A[9], const double biasForce[3],
                    const double biasTorque[3], const double comSensor[3]);
    // 返回 false = 文件不存在 / 读不动 / 【不是本格式】(旧版文件) / 【模型不可用】——
    // 后两者会在 stderr 响亮地说出来 (指名道姓是哪个字段坏了), 不会安静地退化成"没有标定"。
    // 【模型不可用】= A 非有限 / 全零 / 数值退化 (|det A| 相对 ||A||^3 近零), 或
    // bias_force_n / bias_torque_nm / com_sensor_m 里有非有限数 —— 判据与
    // ForceCompensation::setCalibration 共用 (ForceCompensation::modelUsable)。
    // ⚠ 返回值分不开这两类: "文件不存在"与"模型不可用"都返回 false, 区别只在 stderr。
    //   运行时闸门 (ForceCompensation::step) 会把它们都报成 ERR_FORCE_UNCALIBRATED,
    //   并在 stderr 上重复那段原因 —— 所以"为什么没有模型"事后仍查得到。
    // 四个输出数组在返回 false 时【未定义】。
    bool loadFromFile(const char* path, double A[9], double biasForce[3],
                      double biasTorque[3], double comSensor[3]);

} // namespace ForceCalibration
