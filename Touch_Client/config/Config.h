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
    // 【力处理链的真实节拍】RelayCore::pollForce 的入口节流 —— 它决定
    //   ForceCompensation::step 与 MotionEstimator::update 的【实际】调用间隔。
    //   这个名字要显式存在, 是因为它和上面那个 125 不是同一个数(125 是【帧到达】率)。
    //   ⚠ 【已知不一致, 尚未修】(run-005 §14): step() 里给运动估计器传的 dt 仍是
    //     1/FORCE_EFFECTIVE_SAMPLE_RATE (=8ms), 而真实间隔是 33ms ⇒ 速度被高估 4.1×、
    //     加速度被高估 17×, 而 Fi = mass·acc ⇒ 运动时惯量项被放大 17 倍。
    //     改它 = 改行为(17 倍), 要单独验证, 所以本趟【只把这个不一致写在两处】, 不动它。
    const int FORCE_POLL_INTERVAL_MS = 33;       // 力处理链实际节拍 (~30 Hz)

    // 30004 帧自检 (TestValue @48 == 0x0123456789ABCDEF) 的档位。语义与理由见 robot/FrameLayout.h。
    //   0 = 关:  完全不做 (退回 2026-09-21 之前的行为)
    //   1 = 只报不判【默认】: 校验 + 首次报字节序 + 不匹配时响亮报警, 但【照旧收下这一帧】
    //   2 = 判:  不匹配就拒帧 + 断线重连 (把"安静地错位"变成"响亮地重连")
    // ⚠ 为什么默认 1 而不是 2: 我们【还没在真机上见过这个字段】—— 它可能没被填, 也可能
    //   不是我们以为的字节序。判错一档而猜反, 客户端会变成【永久重连循环】(那会挡掉现场工作)。
    //   ⇒ 先在真机上确认一次"自检通过 + 字节序是哪个", 再改 2。
    //   这与"闸门表先并排报 comp−@720、看清参考量之前不改判据"是同一套做法: 先观察, 后判决。
    const int FORCE_FRAME_MAGIC_MODE = 1;
    const int FORCE_FILTER_CUTOFF = 30;          // Butterworth 截止频率 (Hz)
    const int FORCE_STALE_MS = 200;              // 数据超时阈值 (ms)
    const double FORCE_RESIDUAL_DEADZONE_N = 0.20; // 补偿后死区 (N) — 略高于运动噪声 0.17N

    // 启动零偏漂移检查的告警阈值 (N)。
    // 负载正确时残余力与姿态无关, 所以任意静止姿态下 "补偿后读数" 就是零偏漂移量。
    // 取 0.5 N: 明显高于死区 0.20 N 与噪声本底 (~0.05 N), 免得天天误报。
    const double FORCE_ZERO_DRIFT_WARN_N = 0.5;
    // 上面那个检查出结论所需的【最少样本数】(闸门放行后累计到的帧数)。
    // ⚠ 它【必须】只有这一处定义: 从前 main.cpp 里有一份、用例里又硬编码了同一个数,
    //   两边各改各的 —— 常量变了用例不会红, 而是【静默地换了含义】(那几条边界断言
    //   写的是"采到 9 个 / 至少 10 个", 数字一改它们就不再测原来那条边界)。
    //   现在调用侧与用例都读这一个常量, 并且用例显式钉住它的值 (改了就响亮地红)。
    const int FORCE_ZERO_DRIFT_MIN_SAMPLES = 10;

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
    // 在线零偏 EMA 的【时间常数】(s), 仅静止态更新。⚠ 语义是时间常数, 不是"每帧更新率":
    //   每帧的 α 由它与采样率算出 (见 ForceCompensation::step 第 8 步) —— 这样改了采样率,
    //   时间常数不会被悄悄改掉。
    //
    // 【为什么从 3.3 s 改成 600 s (2026-09-21, run-005 §7.4)】
    //   这个 EMA 把 bF 朝"comp → 0"拉, 而它【分不清】"传感器零点的慢漂"与"一个持续的外力"
    //   —— 对 EMA 来说两者是同一个信号。实测: 挂 1.9 N 重物 11 分钟, bF 吸收了 2.09 N
    //   (与载荷 2.076 N 差 0.5%), comp 归零、闸门一路放行 ⇒ 力被【静默擦除】。
    //   三个时间尺度摆在一起就看出原来的数错了:
    //     传感器零点漂移    0.4025 N / 19.5 h = 0.02 N/小时
    //     毛笔触纸的力变化   0.1 ~ 1 s
    //     旧的 EMA (α=0.01)  3.3 s   ← 与毛笔【同一个数量级】⇒ 把秒级的力也当漂移吃掉了
    //   600 s 下: 漂移跟踪滞后 = 0.02 N/h × 600 s = 0.003 N (比死区 0.20 N 小 60 倍),
    //   而 1 s 的力基本不动 ⇒ 两者差 ~600 倍, 分得开。
    //   ⚠ 【不要】改成"关掉": 传感器自身漂移 0.4 N/session 是死区 0.20 N 的 2 倍 ——
    //     关掉换来的是常驻的幽灵力 + 在死区边界上抖动。放慢 ≠ 关掉。
    //   ⚠ 0.02 N/小时 是【一个数据点】推出来的。要定准 τ 应先实测它 —— 'm' 模式的 SPACE
    //     会打印 bF, 开机记一次、一小时后记一次即可。若实测大得多, 这个数要跟着改。
    const double FORCE_BIAS_EMA_TAU_S = 600.0;
    const double FORCE_ACC_FILTER_CUTOFF_HZ = 10.0;       // 加速度估计低通截止 (Hz)

    // ========== 运行时一致性闸门 (2026-09-19) ==========
    // 逐通道比较【本地全量模型的输出 compensated】与【机械臂自报的参考量】。
    // ⚠ 判决【不是逐通道的全集】: 只有【投票通道】的差进判决, 其余通道照报不判 ——
    //   掩码的唯一一份实现是 ForceCompensation.cpp 的 g_guardVote (本文件下方与
    //   ForceCompensation.h 都指向那一处, 不在这里复述是哪几个)。
    // 在投票通道上: 两个模型都对时它们估计的是同一个量 (外力), 所以应当一致;
    // 不一致 ⇒ 至少一个错 ⇒ 拒绝把数据往下传 (compensated 置零)。
    // ⚠ 参考量【是哪一路】不在这里写死: 它由 ForceCompensation.cpp 的 guardReferenceValue
    //   一处定义 (那里也有"为什么是那一路"的实测出处)。换参考量就改那一处。
    //   @576 (fd.raw) 是同屏并排报出的诊断侧, 【只报不判】—— 下面的容差不比它。
    //
    // 容差【由实测导出】, 不是猜的。它约束的量是【compensated − 判据参考量】—— "判据参考量"
    // 是哪一路由 ForceCompensation.cpp 的 guardReferenceValue 一处定义, 所以这里【不写通道号】:
    // 一写死, 下一次换参考量就会留下一句假话 (这段文字已经吃过一次这个亏)。
    //
    // 力通道的容差 = 两个【来源不同】的合法误差项之和。两项都是"即使全都做对, 这个比较也会
    // 显示出来的差异", 所以容差必须把它们的合并盖住; 合并规则与它的代价写在 tol_F 那几行。
    //
    // (甲) 本地失拟 + 机械臂那一侧的模型类差:
    //      推导见 .superpowers/sdd/runtime-guard-report.md; 全部数字由
    //      tests/test_payload_calibration.cpp 的 test_runtime_consistency_guard_replay
    //      在四份夹具上现算并打印, 那条用例同时断言"容差 > 这个量级"。
    //      ⚠ 2026-09-21 起这件事由该用例的【前置遍历】承担 (逐姿态重放那一半在第一份夹具的
    //        第一个姿态就按设计红着 return, 所以留在那里的话只有第一份会跑到这条断言 ——
    //        而第一份恰好是四份里要求最低的)。前置遍历四份全跑, 断言按【最坏的那一份】判。
    //   eps_甲 = rmsForceN + (σ1−σ3)/2 · 9.81
    //         四份夹具上 = 0.15314 / 0.25195 / 0.28710 / 0.17591 N, 最坏 0.28710 N (15:30);
    //         · rmsForceN 是本地模型自己的失拟 (夹具实测量: 0.0224/0.0186/0.0191/0.0151 N);
    //         · 第二项是【机械臂那一侧】的模型类差: 它的力模型是"标量质量 × 正交"(m·Q),
    //           而本地 A 是自由 3×3。A 到【最近的】m·Q 的算子范数距离【恰好】是 (σ1−σ3)/2
    //           (在 m = (σ1+σ3)/2 处取到), 四份实测 = 0.013328/0.023784/0.027323/0.016391 kg
    //           (= 0.1308/0.2333/0.2680/0.1608 N 的重力响应), 这一份它【结构上表达不出来】。
    //           ⚠ 2026-09-19 复审改口径: 从前这里写的是 max|σ(A) − σ̄|。那个量【不是下界,
    //             是上界】—— 它恒 ≥ (σ1−σ3)/2 (四份实测大 0.12% ~ 14.9%)。用上界当依据会把
    //             "容差是合法差的几倍"说小, 而旧注释还称它"fail-closed 的方向": 方向正好
    //             说反 (依据取大了, 容差只会更松)。现在用的是精确距离, 所以下面那个倍数
    //             就是【真正的余量】, 不再有第二层水分。那次改口径时容差的【数值】一个都没动。
    //
    // (乙) 参考量【自带】的力偏置, 以及它跨轮次的漂移  (2026-09-21 加入):
    //      出处: Docs/superpowers/specs/2026-09-21-gate-reference-prereq.md
    //      §3.3 / **§3.3.1 (行集 + 算式 —— 第三方照它可逐位复现)** / §3.4;
    //      原始数据: 三段带力参考量那一列的采集 —— 引它的【冻结快照】
    //      Touch_Client/tests/fixtures/calib_poses_2026-09-20_frozen.txt
    //      ⚠ 不引 CalibStore 运行时可写目录下的那份活文件: 它在那里【只追加、永不截断】,
    //        每次标定采集都会改它 ⇒ 被本段引用的数字会随手漂 (2026-09-21 Task 8 冻的快照)。
    //      (三轮, 每轮 10 行 = 5 对原地复采)。逐轮把该通道的力对重力方向做
    //      "常数 + gx,gy,gz" 的回归, 取截距 c (三轮的三维条件数 22.3/16.7/18.8, 不是病态):
    //        forceOffsetMax_N   = 0.1548 N = max over 轮次 of max_axis |c|
    //                                       (21:58:46 轮的 c_y = −0.154790)
    //        forceOffsetDrift_N = 0.1813 N = max over 轴 of (max c_a − min c_a)
    //                                       【该轴跨轮的带符号极差】
    //        eps_乙 = 0.1548 + 0.1813 = 0.3361 N
    //      ⚠ 归约维度必须写明 —— 计划给的算式没定义它 (该文 §3.1): Drift 取【按轴、带符号的
    //        跨轮极差】这一读法 (x/y/z = 0.1434/0.1813/0.0915 ⇒ 0.1813); "先逐轴取 |c| 再跨轮
    //        相减"那条【次自然】读法给 0.1389 ⇒ 0.2937 (更小), 不采用 (该文 §3.4 两行都在)。
    //      ⚠ eps_乙 把同一个量算了两次 —— 这是【有意保守】, 不是算错: Max 与 Drift 都用到
    //        21:58:46 轮的 c_y: 带符号读法下 Max = |min c_y|、Drift 的下端就是 min c_y,
    //        所以 eps_乙 = max c_y + 2·|min c_y| = 0.0265 + 2×0.1548 = 0.3361。
    //      ⚠ 它【为什么单独一项】: (甲) 是我们自己模型的误差, (乙) 是【尺子自己】的偏置与它的
    //        不稳定性。参考量的力偏置会漂 (这就是 eps_乙 存在的理由), 所以不能并进 (甲)。
    //
    // 合并: eps_F = eps_甲 + eps_乙 = 0.2871 + 0.3361 = 0.6232 N ⇒ tol_F = 2 × 0.6232 = 1.2464 N。
    //   ⚠ 舍入约定 (只有一条, 免得读者看到两个规矩): 每一步都用【本文件印出的那一位小数】
    //     参与下一步运算 —— 即先按 4 位小数取 eps_甲 / eps_乙, 相加得 0.6232, 再乘 2。
    //     若改用未舍入的 0.287098 + 0.336054 = 0.623152 直接乘 2 会得 1.2463, 本文件【不用】
    //     那一条 (两条路只差 1e-4, 但混着写会显得自相矛盾)。落盘的数是 1.2464。
    //   ⚠ 取【正好 2 倍】, 不是"取个整数": 余量正好落在判据 (≥ 2) 上, 于是这是【满足判据的最小
    //     容差】—— 再取大就是白白少抓一截 (容差越大越抓不住东西, 见下面那一段)。所以这个数
    //     是算出来的, 不是凑的; 它不回退成"调到能过为止"的另一个理由是: 0.6232 这一侧是
    //     实测的两项, 与"闸门放不放行"无关 —— 它不来自它自己要检验的量。
    // ⚠ 合并规则取【最坏情形相加】, 这是【判断】, 不是从测量里导出来的 (要说清楚):
    //   两组测量没有给出联合分布, 所以"两者独立"这个前提【没有依据】—— 而它们【可能部分重叠】
    //   (机械臂那一侧的模型类残差, 正好也是参考量偏置的一个可能来源; 该文没有判它, 手上只有
    //   三轮、两个日期)。测量的现状也【不足以】证明相加不会过头: 三轮数据上【直接】量到的
    //   |compensated − 参考量| 逐姿态最大值 (只算投票的 Fx/Fy) 只有 0.1775 N —— 比 0.6232 这个
    //   界【小 3.5 倍】。⇒ 取【保守 (较大) 的那一边】: 相加。代价写下来: 平方和给
    //   √(0.2871² + 0.3361²) = 0.4420 ⇒ tol_F = 0.8840 N; 相加比它宽 0.3624 N (1.41 倍),
    //   折算成负载质量误差是 0.127 kg 而不是 0.090 kg。
    //
    // ★ 上调到 1.2464 N 之后它【还能抓住什么】—— 用户约束是"门限可以适当降低标准, 但安全起见
    //   不符合时还是要拒绝", 所以这一段是硬要求, 不是注解。闸门在
    //   |EMA(compensated − 判据参考量)| > 该通道的容差时拒绝那一路。当前投票掩码【只有
    //   Fx/Fy 两个通道】(掩码的唯一定义见 ForceCompensation.cpp 的 g_guardVote):
    //   Fz 与 Mx/My/Mz 都【照报不判】—— 力矩三个分量自 2026-09-21 起不再投票 (实测它们与
    //   参考量之间不存在"一致"态, 理由与代价见 g_guardVote 段), 而 tol_M 因此在【判决里】
    //   不再被消费: 它仍然印在逐通道表上作为该通道的读数尺度, 但没有任何通道按它投票。
    //   ⚠ 所以由容差 1.2464 N 导出的那条灵敏度【只覆盖 Fx/Fy 两个力方向】; z 方向没有闸门
    //     (见下面那段), 力矩方向现在也不再有任何判决意义。
    //   · 相对上面那个【界】的余量 = 2.00x; 相对【三轮实测到的最大合法差 0.1775 N】= 7.0x。
    //   · 折算成【负载质量误差】: 臂侧负载与本地模型差 Δm 会在重力方向上产生 Δm·9.81 的力,
    //     取最不利取向 ⇒ 门限 Δm = 1.2464 / 9.81 = 0.127 kg
    //     (= 工具链实测质量 0.4159 kg 的 31%)。
    //   · 【已知的那个失效模式仍被抓住】: 机械臂还拿着种子负载 0.66 kg 而实测是 0.4159 kg
    //     ⇒ Δm = 0.244 kg ⇒ 最不利取向下 2.39 N > tol_F, 余量 1.92x。⇒ 这种情况仍然【拒绝】。
    //   · 【代价】: 旧 tol_F = 0.50 N 对应的门限是 Δm = 0.051 kg ⇒ 同一条灵敏度掉了 2.49 倍
    //     (要 2.49 倍大的质量误差才触发)。0.90 N 量级的持续不一致过去会拒, 现在不会了。
    //   · ⚠ 上面全是【静止】口径的灵敏度 (量的是"模型错多少会被拒"), 【不是】"外力多少会被拒":
    //     换参考量后闸门在真实外力下的行为【没有任何仓库文件验证过】(该文 §4.2①, 遗留风险)。
    //
    //   eps_M = rmsMomentNm + |c_s|·(σ1−σ3)/2·9.81; 四份 = 0.00853 / 0.01400 / 0.01626 / 0.01068,
    //         最坏 0.01626 N·m (15:30 = rms 0.0012 + |c_s| 0.0560 × 类差 0.2680)。
    //   tol_M = 0.03 N·m = 1.85 × 0.01626 (逐份的比值 1.85~3.52)。
    // ⚠ 首版容差, 【必须在实机上复验】: 机械臂那一侧的真值要等正确的负载下发
    //   (Task 8) 之后才量得到。力通道的推导现在覆盖两项 [(甲)+(乙), 见上];
    //   力矩通道的推导【仍然只覆盖了"模型类不同"这一项】—— 若【判据参考量】的力矩参考点
    //   与传感器原点不同, 还会多出一项 |Δc × F|
    //   (Δc ≤ 14.15 mm, 见 Docs/superpowers/plans/2026-09-19-raw-channel-calibration.md:226;
    //    |F| ≤ 3.0932 N, 四份夹具 36 个姿态里力模的最大值, 出现在 12:38 pose 4 ——
    //    夹具的力列有【两路】(F576 与 F1304), 缺的是【判据参考量】那一路, 所以这个界
    //    是在【旧参考量】上量的)
    //   —— 那一项最大 14.15 mm × 3.0932 N = 0.0438 N·m, 比 tol_M 还大。
    //   若它是真的, 力矩通道会在 Task 8 之后【永远拒绝】。测到之后再复查, 不要凭猜放宽。
    //   ⚠ 这一条是【在旧参考量上写的假设】, 换参考量之后【没有复测】: 它现在的状态是"未决",
    //     不是"已验"。本段不动力矩容差的数值。
    //   ⚠ 2026-09-21 起力矩三个分量【不再投票】(见 ForceCompensation.cpp 的 g_guardVote 段),
    //     所以"若它是真的, 力矩通道会在 Task 8 之后永远拒绝"这条风险【不再由闸门承担】——
    //     它已经不可能让闸门拒绝了。tol_M 这个数【保留原值】: 它是该通道读数的尺度, 仍然
    //     印在逐通道表上, 只是不再有通道按它投票。
    //
    // ⚠⚠ 【z 力方向实际上没有闸门 —— 给出尺寸, 不要只说"少一道闸门"】(复审 Important 2)。
    //   Fz 不投票 (理由见 ForceCompensation.cpp 的 g_guardVote), 而力矩通道【兜不住它】——
    //   2026-09-21 起它【连票都不投了】(实测力矩与参考量之间没有"一致"态), 所以下面这个
    //   "力矩门若要看见 z 差得多大"的算法【现在是一道不存在的门】:
    //   z 上的模型误差 ΔFz 要经 c_s 叉乘成力矩误差才可能被看见, 量级 = |c_s_横向| · ΔFz。
    //   四份实测 |c_s_横向| 只有【0.47 ~ 0.78 mm】(横向 = sqrt(cs_x²+cs_y²), 由上面那条用例
    //   现算打印), 而 tol_M = 0.03 N·m ⇒ ΔFz 要到 tol_M / |c_s_横向| ≈ 【38 ~ 64 N】
    //   才有机会把力矩顶超限。也就是说: 几十牛量级的 z 力模型错误, 这一版闸门看不见。
    //   这是本次改动【已知的最大一处漏洞】, 明确作为【未决项】交给用户
    //   (补一个 z 的独立判据, 还是接受这个洞) —— 不是靠调容差能解决的。
    //   ⇒ 要补这个洞只能【新增一道独立的 z 判据】, 不能靠"把力矩的票加回来":
    //     力矩不投票是实测结论 (它连同姿态的重复性都不够), 复审判定"Fz 不投票"这个取舍
    //     本身可接受, 错的是没把代价说出来 —— 两件事都不因本次改动而变化。
    // 力通道 (x/y) 一致性容差 (N)。它约束的是【compensated − 判据参考量】这一条差, 参考量由
    // guardReferenceValue 一处定义 ⇒ 这里【不写通道号】。依据 (甲)(乙) 两项之和 = 0.6232 N,
    // 取 2 倍 ⇒ 1.2464 N (余量 2.00x), 以及它【还能抓住什么】, 都写在上面那一段里。
    const double FORCE_GUARD_TOL_FORCE_N   = 1.2464;  // N
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
