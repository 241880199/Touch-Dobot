#pragma once

// ===================================================================
//  配置共享说明
//  - 单一数据源: system_config.json (同目录)
//  - C++ 侧编译时常量: 需与 JSON robot/relay/safety_bounds/connection 段保持同步
//  - MATLAB 侧: relay_config.m 运行时从 JSON 读取 (无需手动同步)
//  - 修改共享参数 (IP/端口/安全边界) 时请先编辑 JSON, 再更新此文件
// ===================================================================

namespace Config {
    // ========== 窗口参数 ==========
    const int WINDOW_W = 1024;
    const int WINDOW_H = 768;

    // ========== Touch 设备坐标范围 ==========
    const double MAX_ABS = 150;
    const double DEV_X_MIN = -MAX_ABS, DEV_X_MAX = MAX_ABS;
    const double DEV_Y_MIN = -MAX_ABS, DEV_Y_MAX = MAX_ABS;
    const double DEV_Z_MIN = -MAX_ABS, DEV_Z_MAX = MAX_ABS;
    const double CENTER_X = (DEV_X_MIN + DEV_X_MAX) / 2.0;
    const double CENTER_Y = (DEV_Y_MIN + DEV_Y_MAX) / 2.0;
    const double CENTER_Z = (DEV_Z_MIN + DEV_Z_MAX) / 2.0;

    // ========== 安全边界（机械臂用户坐标系，单位mm） ==========
    // 初始值设为保守范围，根据实际环境调整
    // CR3 工作半径 620mm，覆盖全工作空间并保留边界余量
    // 用户坐标系安全边界 (机器人静止位姿 ~(-103,-153,381))
    const double SAFE_X_MIN = -300.0, SAFE_X_MAX = 250.0;
    const double SAFE_Y_MIN = -350.0, SAFE_Y_MAX = 250.0;
    const double SAFE_Z_MIN = 140.0,  SAFE_Z_MAX = 500.0;
    const double SAFE_BOUNDARY_BUFFER_RATIO = 0.2; // 20%边界缓冲区，线速度衰减

    // ========== SafetyPredictor 安全预判参数 ==========
    const double WORKSPACE_RADIUS         = 620.0;   // CR3 最大工作半径 (mm)
    const double ROBOT_MAX_Z              = 795.0;   // CR3 总高度 (mm)
    const double SINGULARITY_COND_WARN    = 100.0;   // 雅可比条件数: 警告阈值
    const double SINGULARITY_COND_REJECT  = 500.0;   // 雅可比条件数: 拒绝阈值
    const double ALARM_DANGER_RADIUS      = 30.0;    // 历史报警点: 危险半径 (mm)
    const double ALARM_WARN_RADIUS        = 80.0;    // 历史报警点: 警告半径 (mm)

    // ========== 网络参数 ==========
    constexpr const char* ROBOT_IP = "192.168.101.11";
    const int ENABLE_PORT = 29999;
    const int MOTION_PORT = 30003;
    const int RECV_BUFFER_SIZE = 1024 * 64; // 64KB

    // MATLAB 中继站 GUI 连接
    constexpr const char* RELAY_IP = "127.0.0.1";
    const int RELAY_PORT = 8888;
    const int RELAY_UPDATE_INTERVAL = 33; // 更新间隔 (ms), ~30Hz

    // ========== 机械臂末端负载【种子值】(工具链: 传感器 + 笔夹 + 笔) ==========
    // 注意: 这三个量【只能实机测出来】, 正式数据源是 force/PayloadCalibration
    //       (payload_calib.json, 由 'm' 采集 + 's' 求解生成)。
    //       这里的值只是"还没标定过"时的兜底 —— 第一次启用机械臂前还没有任何
    //       实测数据, 而 EnableRobot 必须先给一个负载才能使能, 所以留一组估计值。
    // 估计来源: Hardware/tools/compute_payload.py (硬件改动后可重跑复核)
    // 实测方法: 启动客户端 → 'm' 采 6~8 个姿态 (笔尖悬空, 只改姿态) → 's' 求解
    //          → 自动重新下发 EnableRobot 并存 payload_calib.json → 再 'm' 复验
    // 负载设置不准 → 碰撞检测误触发 / 拖拽失控 / 30004 力值随姿态漂移 (见
    // Docs/机械臂资料/Dobot CR3机械臂参数文档.md §负载设置)。
    const double ROBOT_PAYLOAD_SEED_KG    = 0.66;   // 工具链总质量估计 (kg)
    const double ROBOT_PAYLOAD_SEED_CX_MM = 0.0;    // 质心偏心 X 估计 (mm)
    const double ROBOT_PAYLOAD_SEED_CY_MM = 0.0;    // 质心偏心 Y 估计 (mm)
    const double ROBOT_PAYLOAD_SEED_CZ_MM = 80.4;   // 质心偏心 Z 估计 (mm, 法兰下方)

    // 【已删: SIGN_PROBE_* 一组常量 (原 68~118 行)】
    // 它们服务于"下发两个候选负载、实测裁决 CZ 符号"的探针。实机实测(2026-09-18)
    // 证明那条路走不通: 机械臂内部负载从 TCP 口【改不动】—— EnableRobot / Payload /
    // LoadSwitch 三条通道全无响应 (0.25 kg 的变化只引起 0.0002 N·m 的读数变化)。
    // 而符号本来只在下发机械臂时才需要; 改成【本地补偿】后整条探针链路已移除,
    // 这几个常量随之作废。(保留记号, 免得后人又照着它们重建一套探针。)

    // ========== 机械臂运动参数 ==========
    const float SpeedL = 100;                // 运动速度比例 (1~100)
    const float MIN_DELTA_THRESHOLD = 1.0f;  // 最小位移阈值 (mm)
    const unsigned int CP_SMOOTH_RATIO = 100; // 平滑过渡比例 (0~100)

    // ========== 力传感器参数 ==========
    const int FORCE_REALTIME_PORT = 30004;       // 实时反馈端口 (125Hz)
    const int FORCE_EFFECTIVE_SAMPLE_RATE = 125; // 传感器数据采样率 (Hz)
    const int FORCE_FILTER_CUTOFF = 30;          // Butterworth 截止频率 (Hz)
    const int FORCE_STALE_MS = 200;              // 数据超时阈值 (ms)
    const double FORCE_RESIDUAL_DEADZONE_N = 0.20; // 补偿后死区 (N) — 略高于运动噪声 0.17N

    // 启动零偏漂移检查的告警阈值 (N)。
    // 负载正确时残余力与姿态无关, 所以任意静止姿态下 "补偿后读数" 就是零偏漂移量。
    // 取 0.5 N: 明显高于死区 0.20 N 与噪声本底 (~0.05 N), 免得天天误报。
    const double FORCE_ZERO_DRIFT_WARN_N = 0.5;

    const double FORCE_MAX_SENSOR_N = 200.0;     // 传感器量程 (N)
    const double FORCE_MAX_TOUCH_N = 3.3;        // Touch 最大安全力 (N)
    const double FORCE_REFLECTION_GAIN = 5.0;    // 力反射增益 — 放大传感器力到可感知范围
    const double FORCE_GRADIENT_LIMIT = 50.0;    // 梯度限幅 (N/frame)
    const int FORCE_RECONNECT_INTERVAL = 2000;   // 断线重试间隔 (ms)

    // ========== 力传感器标定参数 ==========
    const double FORCE_CALIB_SPEED_FACTOR = 0.30;       // 标定期速度因子
    const double FORCE_CALIB_STILL_COLLECT_S = 2.0;      // 初始静止采集时间 (s)
    const double FORCE_CALIB_MOVE_TIMEOUT_S = 5.0;       // 单姿态移动超时 (s)
    const double FORCE_CALIB_SETTLE_TIME_S = 0.5;        // 姿态稳定等待 (s)
    const double FORCE_CALIB_SAMPLE_TIME_S = 0.5;        // 数据采集时间 (s)
    const double FORCE_CALIB_MAX_RESIDUAL_N = 0.3;       // 标定残差阈值 (N)
    const double FORCE_CALIB_POSE_ANGLE_DEG = 15.0;      // 标定姿态偏角 (度)
    const int    FORCE_CALIB_NUM_POSES = 6;               // 标定姿态数

    // ========== 力传感器安装偏转角 (重力模型) ==========
    // 力传感器相对法兰的安装偏转角 (度, 绕【工具 z】)。传感器实际测量的坐标系与 GetPose
    // 的 RPY 所描述的那个之间差这么一个角。
    // ⚠ 现在【只是种子/回退值】, 不再是真值: 真值由负载标定扫出来 (PayloadCalibration::solve),
    //   随 payload_calib.json 的 "sensor_yaw_deg" 持久化, 启动时经 TcpCalibration::setSensorYawDeg
    //   装上。三组实机数据都指向 ≈79–81.5°, 而 90° 与 80° 正好跨过 0.30 N 的验收门限 ——
    //   所以这个数不能再靠手改。以下的 90° 只在这些情况下生效: 没有标定文件 / 旧文件里没有该字段。
    // 消费者仍是唯一一份实现: TcpCalibration::gravitySensorFrame (力补偿与负载求解【共用】)。
    const double SENSOR_MOUNT_YAW_DEG = 90.0;

    // 连续多少次"结果不合理"就打红字错误并停止接受求解。
    // 防的是操作者反复按 's' 却每次被拒的空转 —— 那种情况下问题在硬件/采集, 不在求解器。
    const int CALIB_MAX_CONSECUTIVE_FAILS = 3;

    // ========== 力补偿运行时参数 ==========
    const double FORCE_MOTION_VEL_THRESH_MS = 0.002;      // 静止判定: 速度阈值 (m/s)
    const double FORCE_MOTION_ACC_THRESH_MSS = 0.005;     // 静止判定: 加速度阈值 (m/s²)
    const double FORCE_BIAS_EMA_ALPHA = 0.01;             // 零偏 EMA 更新率 (仅静止态)
    const double FORCE_ACC_FILTER_CUTOFF_HZ = 10.0;       // 加速度估计低通截止 (Hz)

    // ========== 运行时一致性闸门 (2026-09-19) ==========
    // 逐通道比较【本地全量模型的输出 compensated】与【机械臂自报的 @576 (fd.raw)】:
    // 两个模型都对时它们估计的是同一个量 (外力), 所以应当一致; 不一致 ⇒ 至少一个错 ⇒
    // 拒绝把数据往下传 (compensated 置零)。
    //
    // 容差【由实测导出】, 不是猜的。量级依据 (推导见 .superpowers/sdd/runtime-guard-report.md;
    // 全部数字由 tests/test_payload_calibration.cpp 的 test_runtime_consistency_guard_replay
    // 在四份夹具上现算并打印, 那条用例同时断言"容差 > 这个量级"):
    //   eps_F = eps_本地 + eps_模型类 = rmsForceN + max|σ(A) − σ̄|·9.81
    //         四份夹具上 = 0.1726 / 0.2706 / 0.3127 / 0.1761 N, 最坏 0.3127 N (15:30);
    //         · rmsForceN 是本地模型自己的失拟 (夹具实测量: 0.0224/0.0186/0.0191/0.0151 N);
    //         · 第二项是【机械臂那一侧】的下界: 它的力模型是"标量质量 × 旋转",
    //           而本地 A 是自由 3×3, 四份实测 A 的奇异值偏离其均值最多 0.0299 kg
    //           (= 0.2937 N 的重力响应), 这一份它【结构上表达不出来】。
    //   tol_F = 0.50 N = 1.60 × 0.3127 (逐份的比值 1.60~2.90)。
    //   eps_M = rmsMomentNm + |c_s|·max|σ(A) − σ̄|·9.81 = 0.0012 + 0.0165 = 0.0177 N·m
    //   tol_M = 0.03 N·m = 1.70 × 0.0177 (逐份的比值 1.70~3.13)。
    // ⚠ 首版容差, 【必须在实机上复验】: 机械臂那一侧的真值要等正确的负载下发
    //   (Task 8) 之后才量得到。上面的推导只覆盖了"模型类不同"这一项 ——
    //   若 @576 的力矩参考点与传感器原点不同, 还会多出一项 |Δc × F|
    //   (Δc ≤ 14.15 mm, 见 Docs/superpowers/plans/2026-09-19-raw-channel-calibration.md:226;
    //    |F| ≤ 3.0932 N, 四份夹具 36 个姿态的 @576 力模最大值, 出现在 12:38 pose 4)
    //   —— 那一项最大 14.15 mm × 3.0932 N = 0.0438 N·m, 比 tol_M 还大。
    //   若它是真的, 力矩通道会在 Task 8 之后【永远拒绝】。测到之后再改, 不要凭猜放宽。
    const double FORCE_GUARD_TOL_FORCE_N   = 0.50;   // 力通道 (x/y) 一致性容差 (N)
    const double FORCE_GUARD_TOL_MOMENT_NM = 0.03;   // 力矩通道一致性容差 (N·m)
    // 逐通道差的 EMA。alpha=0.02 @30Hz ⇒ 等效平均 ~100 帧 ≈ 3.3 s, 把逐帧噪声
    // (夹具实测力 x/y 0.04~0.06 N, 力矩 0.002~0.006 N·m) 压到容差之下一个量级以上。
    const double FORCE_GUARD_EMA_ALPHA = 0.02;
    // 同一条错误最快多久重报一次 (ms)。状态一变立刻报, 不变的按这个间隔复报。
    const int    FORCE_GUARD_REPORT_MS = 5000;

    // ========== 姿态控制参数 ==========
    const double ORIENT_MAX_STEP_DEG = 3.0;          // 单步最大角度增量 (degrees)
    const double ORIENT_DEADZONE_DEG = 0.05;         // 姿态死区 (degrees)
    const double ORIENT_GAIN = 1.0;                  // 姿态增益 (可调灵敏度)
    const double SAFE_RX_MIN = -180.0, SAFE_RX_MAX = 180.0;  // Roll 安全限位
    const double SAFE_RY_MIN = -90.0,  SAFE_RY_MAX = 90.0;   // Pitch 安全限位
    const double SAFE_RZ_MIN = -180.0, SAFE_RZ_MAX = 180.0;  // Yaw 安全限位

    // ========== 虚拟约束力参数 ==========
    const double CONSTRAINT_BOUNDARY_RANGE      = 50.0;   // 安全边界感应距离 (mm)
    const double CONSTRAINT_BOUNDARY_MAX_FORCE  = 2.0;    // 安全边界最大约束力 (N)
    const double CONSTRAINT_SINGULAR_RANGE      = 80.0;   // 圆柱奇异感应距离 (mm) — 提前预警
    const double CONSTRAINT_SINGULAR_MAX_FORCE  = 2.5;    // 圆柱奇异最大约束力 (N) — Touch 最大输出
    const double CONSTRAINT_ALARM_HISTORY_RANGE    = 80.0; // 报警历史感应距离 (mm)
    const double CONSTRAINT_ALARM_HISTORY_MAX_FORCE = 1.5; // 报警历史最大约束力 (N)
    const double CONSTRAINT_WORKSPACE_EDGE_START     = 550.0; // 工作空间边缘感应起点 (mm) — 提前预警
    const double CONSTRAINT_WORKSPACE_EDGE_MAX_FORCE = 1.0;   // 工作空间边缘最大约束力 (N)

    // ===== Singularity Avoidance (零空间优化) =====
    // Shoulder safety
    const double SINGAVOID_SHOULDER_SAFE_R     = 120.0;  // 肘部安全 r_xy (mm) — 低于此触发零空间优化 + 姿态模式 TCP 微调
    const double SINGAVOID_SHOULDER_CRITICAL_R  = 50.0;   // 肘部危险 r_xy (mm) — 姿态模式下触发临界警告
    // Elbow safety
    const double SINGAVOID_ELBOW_MID_ANGLE      = 0.0;    // J3 最佳位置 (deg) — 零空间吸引子中心
    // Joint limit repulsion
    const double SINGAVOID_JOINT_WARN_MARGIN    = 10.0;   // 关节限位警告裕度 (deg) — 零空间斥力触发距离
    // TCP micro-adjust
    const double SINGAVOID_MAX_POS_ADJUST       = 5.0;    // 姿态模式 TCP 微调上限 (mm)
    // Wrist damping (orientation mode) — Phase 2: continuous curve
    const double SINGAVOID_COND_WRIST_EARLY     = 20.0;   // 腕部早期预警阈值 — 开始触觉斥力
    const double SINGAVOID_COND_WRIST_BLOCK     = 150.0;  // 腕部阻断阈值 — β→0
    // Combined mode damping
    const double SINGAVOID_COND_FULL_WARN       = 15.0;   // 全控模式条件数警告阈值 (Phase 2: 30→15)
    const double SINGAVOID_SINGULAR_RATIO       = 0.05;   // σᵢ/σ₁ 阻尼触发比
    // Null-space optimization
    const int    SINGAVOID_NULLSPACE_ITER       = 20;     // 最大零空间迭代轮数 (Phase 2: 15→20)
    const double SINGAVOID_GRAD_STEP            = 0.3;    // 梯度步长 (deg) — Phase 2: 0.1→0.3
    // Gradient weights (should sum to ~12)
    const double SINGAVOID_W_SHOULDER           = 4.0;    // 肩关节安全权重 (Phase 2: 3.0→4.0)
    const double SINGAVOID_W_ELBOW              = 2.0;    // 肘关节弯曲权重
    const double SINGAVOID_W_JOINT              = 5.0;    // 关节限位权重
    const double SINGAVOID_W_WRIST              = 1.0;    // 腕部对称权重
    // Orient mode constraint force amplification
    const double SINGAVOID_ORIENT_FORCE_AMP     = 2.5;    // 姿态模式下奇异斥力放大倍数 (Phase 2: 1.5→2.5)
    // Directional repulsion (Phase 2 new)
    const double SINGAVOID_SINGULAR_FORCE_MAX_N = 3.0;    // 方向性斥力上限 (N)
    // Dual-singularity detection (Phase 2 new)
    const double SINGAVOID_DUAL_SING_COND_THR   = 60.0;   // 腕部条件数阈值 — 双重奇异检测
    const double SINGAVOID_DUAL_SING_ELBOW_THR  = 80.0;   // 肘部 r_xy 阈值 — 双重奇异检测

    // ========== 连接健康监控参数 ==========
    const int HEARTBEAT_TIMEOUT_MS  = 500;    // 心跳超时 (ms)
    const int RECONNECT_MAX_RETRIES = 5;      // 最大重连次数
    const int RECONNECT_BASE_DELAY_MS = 1000; // 重连基础延迟 (ms), 指数退避
    const int PING_INTERVAL_MS      = 500;    // PING 间隔 (ms)
    const int PING_TIMEOUT_MS       = 500;    // PING 超时 (ms)

    // ========== 错误升级参数 ==========
    const int ESCALATE_WARN_TO_DEGRADE   = 3;   // WARN 连续帧数 → DEGRADE
    const int ESCALATE_DEGRADE_TO_REJECT = 10;  // DEGRADE 连续帧数 → REJECT
    const int DEESCALATE_CLEAR_FRAMES    = 30;  // 清除后多少帧降级

    // ========== 看门狗参数 ==========
    const int WATCHDOG_TIMEOUT_MS = 200;     // 触觉线程看门狗超时 (ms)

    // ========== 升级时间阈值 ==========
    const int MIN_WARN_MS    = 50;           // WARN 至少持续 50ms 才能升级到 DEGRADE
    const int MIN_DEGRADE_MS = 200;          // DEGRADE 至少持续 200ms 才能升级到 REJECT

    // ========== 诊断日志参数 ==========
    constexpr const char* DIAGNOSTIC_LOG_PATH = "robot_diagnostics.log";
    constexpr const char* FORCE_LOG_PATH = "force_demo_log.csv"; // 力反馈演示落盘路径

    // ========== 发送队列参数 ==========
    const int MAX_QUEUE_SIZE = 5;            // 队列容量上限（满时丢弃旧数据）
    const int TCP_SEND_INTERVAL = 10;        // 发送间隔 (ms)
    const int FEEDBACK_TIMEOUT = 2000;       // 反馈读取超时 (ms)
    const int ALARM_CHECK_INTERVAL = 300;    // 报警巡检间隔 (ms)
    const int POSE_QUERY_INTERVAL = 100;    // 位姿查询间隔 (ms)，驱动 3D 模型更新
    const int IDLE_SLEEP_MS = 1;             // 发送线程空闲休眠 (ms)

    // ========== 3D 投影参数 ==========
    const float AXIS_LINE_WIDTH = 3.0f;
    const double BASE_CAM_X = 0.0, BASE_CAM_Y = 70.0, BASE_CAM_Z = 240.0;
    const double NEAR_CLIP = 1.0, FAR_CLIP = 800.0, FOV = 45.0;
    const float MIN_ZOOM = 0.3f, MAX_ZOOM = 5.0f, ZOOM_STEP = 1.5f;
    const float ROTATION_SPEED = 0.5f;

    // ========== 交互参数 ==========
    const int MAX_TRAIL = 300;

    // ========== Logo 参数 ==========
    constexpr const char* LOGO_PATH = "pics/NINELAB.png";
    const int LOGO_WIDTH = 150, LOGO_HEIGHT = 75;

    // ========== 颜色定义 ==========
    const float COLOR_FLOOR[4]       = { 0.22f, 0.25f, 0.30f, 0.55f };
    const float COLOR_BORDER[4]      = { 0.62f, 0.68f, 0.78f, 0.50f };
    const float COLOR_AXIS_X[4]      = { 1.00f, 0.35f, 0.35f, 0.95f };
    const float COLOR_AXIS_Y[4]      = { 0.35f, 0.95f, 0.45f, 0.95f };
    const float COLOR_AXIS_Z[4]      = { 0.35f, 0.55f, 1.00f, 0.95f };
    const float COLOR_CURSOR_DOT[4]  = { 1.00f, 1.00f, 1.00f, 0.95f };
    const float COLOR_TEXT[4]        = { 0.92f, 0.96f, 1.00f, 1.00f };
    const float COLOR_SUCCESS[4]     = { 0.35f, 0.90f, 0.50f, 1.00f };
    const float COLOR_ERROR[4]       = { 1.00f, 0.35f, 0.35f, 1.00f };
    const float COLOR_WARNING[4]     = { 1.00f, 0.78f, 0.28f, 1.00f };
    const float COLOR_TRAIL[4]       = { 0.25f, 0.85f, 1.00f, 0.90f };

    // ========== 坐标表格参数 ==========
    const int TABLE_LEFT = 15;
    const int TABLE_TOP = WINDOW_H - 100;
    const int TABLE_WIDTH = 230;
    const int TABLE_COL1_W = 70;
    const int TABLE_PADDING = 10;
    const int TABLE_TITLE_ROW_H = 34;
    const int TABLE_ROW_H = 26;
    const float TABLE_BG_COLOR[4]       = { 0.10f, 0.13f, 0.17f, 0.82f };
    const float TABLE_BORDER_COLOR[4]   = { 0.42f, 0.56f, 0.78f, 0.85f };
    const float TABLE_ALT_ROW_COLOR[4]  = { 0.12f, 0.16f, 0.21f, 0.82f };
    const float TABLE_CELL_TEXT_COLOR[4]= { 0.92f, 0.96f, 1.00f, 1.00f };
    const float TABLE_TITLE_BG[4]       = { 0.16f, 0.28f, 0.55f, 0.95f };
    const float TABLE_TITLE_TEXT_COLOR[4]={ 0.95f, 0.98f, 1.00f, 1.0f };
    const float TABLE_BORDER_WIDTH = 1.0f;

    // ========== 状态栏参数 ==========
    const int TCP_STATUS_BAR_HEIGHT = 30;
}
