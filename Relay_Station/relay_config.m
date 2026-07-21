% Touch-Dobot 中继站配置
% 修改此文件即可适配不同环境

function cfg = relay_config()
    % ===== 机械臂连接 =====
    cfg.robot_ip = '192.168.101.11';
    cfg.port_enable = 29999;
    cfg.port_motion = 30003;

    % ===== 中继站服务器 =====
    cfg.relay_port = 8888;
    cfg.listen_ip = '0.0.0.0';       % 监听所有网卡

    % ===== 超时与重连 =====
    cfg.timeout = 5;                  % 读写超时（秒）
    cfg.reconnect_interval = 3;       % 断连后重试间隔（秒）
    cfg.max_reconnect_attempts = 10;  % 最大重连次数（0=无限）

    % ===== 日志 =====
    cfg.log_dir = 'logs';             % 日志目录（相对于脚本目录）
    cfg.max_log_lines = 200;          % 控制台最大显示行数

    % ===== 安全边界 (mm, 机械臂用户坐标系) =====
    cfg.safe_x_min = 180; cfg.safe_x_max = 420;
    cfg.safe_y_min = -200; cfg.safe_y_max = 200;
    cfg.safe_z_min = 30;  cfg.safe_z_max = 300;
end
