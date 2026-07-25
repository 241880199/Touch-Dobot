% Touch-Dobot 中继站配置
% 数据源: ../Touch_Client/config/system_config.json（单一数据源）
% 修改配置请编辑 JSON 文件，不要直接修改此文件中的共享值。

function cfg = relay_config()
    % ===== 从 JSON 加载共享配置 =====
    % JSON 文件路径: 相对于此脚本所在目录
    script_dir = fileparts(mfilename('fullpath'));
    json_path = fullfile(script_dir, '..', 'Touch_Client', 'config', 'system_config.json');

    if isfile(json_path)
        try
            js = jsondecode(fileread(json_path));
        catch
            warning('relay_config: 无法解析 JSON 配置文件，使用内置默认值');
            js = [];
        end
    else
        warning('relay_config: JSON 配置文件未找到 (%s)，使用内置默认值', json_path);
        js = [];
    end

    % ===== 机械臂连接（来自 JSON robot 段）=====
    if ~isempty(js) && isfield(js, 'robot')
        cfg.robot_ip = js.robot.ip;
        cfg.port_enable = js.robot.port_enable;
        cfg.port_motion = js.robot.port_motion;
    else
        cfg.robot_ip = '192.168.101.11';
        cfg.port_enable = 29999;
        cfg.port_motion = 30003;
    end

    % ===== 中继站服务器（来自 JSON relay 段）=====
    if ~isempty(js) && isfield(js, 'relay')
        cfg.relay_port = js.relay.port;
        cfg.listen_ip = js.relay.listen_ip;
    else
        cfg.relay_port = 8888;
        cfg.listen_ip = '0.0.0.0';
    end

    % ===== 安全边界（来自 JSON safety_bounds 段）=====
    if ~isempty(js) && isfield(js, 'safety_bounds')
        sb = js.safety_bounds;
        cfg.safe_x_min = sb.x_min;  cfg.safe_x_max = sb.x_max;
        cfg.safe_y_min = sb.y_min;  cfg.safe_y_max = sb.y_max;
        cfg.safe_z_min = sb.z_min;  cfg.safe_z_max = sb.z_max;
    else
        % 回退默认值 — 与 system_config.json 保持一致
        cfg.safe_x_min = -300;  cfg.safe_x_max = 250;
        cfg.safe_y_min = -350;  cfg.safe_y_max = 250;
        cfg.safe_z_min = 140;   cfg.safe_z_max = 500;
    end

    % ===== MATLAB 专用参数（来自 JSON matlab 段）=====
    if ~isempty(js) && isfield(js, 'matlab')
        cfg.timeout = js.matlab.timeout_sec;
        cfg.reconnect_interval = js.matlab.reconnect_interval_sec;
        cfg.max_reconnect_attempts = js.matlab.max_reconnect_attempts;
        cfg.log_dir = js.matlab.log_dir;
        cfg.max_log_lines = js.matlab.max_log_lines;
    else
        cfg.timeout = 5;
        cfg.reconnect_interval = 3;
        cfg.max_reconnect_attempts = 10;
        cfg.log_dir = 'logs';
        cfg.max_log_lines = 200;
    end
end
