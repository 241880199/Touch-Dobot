#pragma once

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
