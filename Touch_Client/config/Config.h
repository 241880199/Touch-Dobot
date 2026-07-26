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

    // ========== 力补偿运行时参数 ==========
    const double FORCE_MOTION_VEL_THRESH_MS = 0.002;      // 静止判定: 速度阈值 (m/s)
    const double FORCE_MOTION_ACC_THRESH_MSS = 0.005;     // 静止判定: 加速度阈值 (m/s²)
    const double FORCE_BIAS_EMA_ALPHA = 0.01;             // 零偏 EMA 更新率 (仅静止态)
    const double FORCE_ACC_FILTER_CUTOFF_HZ = 10.0;       // 加速度估计低通截止 (Hz)

    // ========== 虚拟约束力参数 ==========
    const double CONSTRAINT_BOUNDARY_RANGE      = 50.0;   // 安全边界感应距离 (mm)
    const double CONSTRAINT_BOUNDARY_MAX_FORCE  = 2.0;    // 安全边界最大约束力 (N)
    const double CONSTRAINT_SINGULAR_RANGE      = 80.0;   // 圆柱奇异感应距离 (mm)
    const double CONSTRAINT_SINGULAR_MAX_FORCE  = 2.5;    // 圆柱奇异最大约束力 (N)
    const double CONSTRAINT_ALARM_HISTORY_RANGE    = 80.0; // 报警历史感应距离 (mm)
    const double CONSTRAINT_ALARM_HISTORY_MAX_FORCE = 1.5; // 报警历史最大约束力 (N)
    const double CONSTRAINT_WORKSPACE_EDGE_START     = 550.0; // 工作空间边缘感应起点 (mm)
    const double CONSTRAINT_WORKSPACE_EDGE_MAX_FORCE = 1.0;   // 工作空间边缘最大约束力 (N)

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
