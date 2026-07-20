% Touch-Dobot 中继站 v2.0
% 功能：TCP 代理 — Touch ↔ 中继站 ↔ Dobot 机械臂
% 运行：cd 到本目录，输入 relay_main

function relay_main()
    % ===== 加载配置 =====
    cfg = relay_config();

    % ===== 初始化日志 =====
    if ~exist(cfg.log_dir, 'dir')
        mkdir(cfg.log_dir);
    end
    log_file = fullfile(cfg.log_dir, ...
        ['relay_', datestr(now, 'yyyy-mm-dd_HH-MM-SS'), '.log']);
    diary(log_file);
    diary on;

    log('=== Touch-Dobot Relay Station v2.0 ===');
    log(sprintf('Robot: %s:%d / %s:%d', ...
        cfg.robot_ip, cfg.port_enable, cfg.robot_ip, cfg.port_motion));
    log(sprintf('Listening: %s:%d', cfg.listen_ip, cfg.relay_port));
    log(sprintf('Log: %s', log_file));

    % ===== 全局状态（用 struct 替代 class） =====
    S = struct();
    S.robot_enable = [];     % tcpclient: 29999 使能端口
    S.robot_motion = [];     % tcpclient: 30003 运动端口
    S.server = [];           % tcpserver: Touch 连接
    S.running = true;        % 主循环开关
    S.relay_count = 0;       % 转发计数
    S.ping_count = 0;        % PING 计数
    S.last_ping_time = tic;  % PING 间隔计时
    S.ping_interval = 1.5;   % PING 间隔（秒）
    S.latency_network = [];  % 网络时延记录

    % ===== 注册 Ctrl+C 清理 =====
    cleanup_obj = onCleanup(@() do_cleanup(S));

    % ===== 1. 连接机械臂 =====
    log('Connecting to robot...');
    S = connect_robot(S, cfg);
    if isempty(S.robot_enable) || isempty(S.robot_motion)
        log('ERROR: Failed to connect to robot. Exiting.');
        return;
    end

    % ===== 2. 启动 TCP 服务器 =====
    log(sprintf('Starting relay server on port %d...', cfg.relay_port));
    try
        S.server = tcpserver(cfg.listen_ip, cfg.relay_port);
        S.server.Timeout = cfg.timeout * 6;
    catch e
        log(sprintf('ERROR: Failed to start server: %s', e.message));
        return;
    end
    log('Waiting for Touch client connection...');

    % ===== 3. 主循环 =====
    log('Relay running. Press Ctrl+C to stop.\n');

    while S.running
        try
            S = handle_touch(S, cfg);       % Touch → 机械臂
            S = handle_robot_feedback(S);    % 机械臂 → Touch
            S = send_ping(S);               % 定时 PING
            S = check_robot_health(S, cfg); % 机械臂断连检测
            pause(0.001);                   % 1ms 轮询
        catch e
            log(sprintf('ERROR in main loop: %s', e.message));
        end
    end
end

% ======================================================================
%  Touch 数据处理：解析 "端口|指令" → 转发到机械臂
% ======================================================================
function S = handle_touch(S, cfg)
    if ~isvalid(S.server) || ~S.server.Connected
        return;
    end
    if S.server.NumBytesAvailable == 0
        return;
    end

    data = read(S.server, S.server.NumBytesAvailable, 'uint8');
    msg = char(data);

    % PING → 回复 PONG
    if startsWith(msg, 'PING|')
        pong = strrep(msg, 'PING', 'PONG');
        write(S.server, pong);
        S.ping_count = S.ping_count + 1;
        return;
    end

    % 解析 "端口|指令"
    parts = strsplit(msg, '|');
    if length(parts) ~= 2
        log(sprintf('WARN: Invalid message format: %s', msg));
        return;
    end

    port = str2double(parts{1});
    cmd = parts{2};
    port_str = num2str(port);

    % 校验端口
    if port ~= cfg.port_motion && port ~= cfg.port_enable
        log(sprintf('WARN: Unknown target port %d', port));
        return;
    end

    % 转发
    client = S.robot_motion;
    if port == cfg.port_enable
        client = S.robot_enable;
    end

    if ~isempty(client) && isvalid(client)
        try
            write(client, cmd);
            S.relay_count = S.relay_count + 1;
            % 只对运动指令打日志（使能指令已在上层可见）
            if port == cfg.port_motion
                log(sprintf('[#%d] → %s', S.relay_count, cmd), 1);
            else
                log(sprintf('[#%d] → %s:%s', S.relay_count, port_str, cmd));
            end
        catch e
            log(sprintf('ERROR: Write to robot %s failed: %s', port_str, e.message));
        end
    end
end

% ======================================================================
%  机械臂反馈处理：读取 → 透传回 Touch
% ======================================================================
function S = handle_robot_feedback(S)
    if ~isvalid(S.server) || ~S.server.Connected
        return;
    end

    ports = {'robot_motion', 'robot_enable'};
    port_nums = [30003, 29999];

    for i = 1:2
        client = S.(ports{i});
        if isempty(client) || ~isvalid(client)
            continue;
        end
        if client.NumBytesAvailable == 0
            continue;
        end

        try
            fb = read(client, client.NumBytesAvailable, 'uint8');
            if ~isempty(fb) && isvalid(S.server) && S.server.Connected
                write(S.server, fb);
                fb_str = char(fb);
                fb_str = strtrim(fb_str);
                if ~isempty(fb_str)
                    log(sprintf('[FB:%d] %s', port_nums(i), fb_str), 2);
                end
            end
        catch e
            log(sprintf('ERROR: Feedback read/write failed: %s', e.message));
        end
    end
end

% ======================================================================
%  定时 PING：测量网络往返时延
% ======================================================================
function S = send_ping(S)
    if toc(S.last_ping_time) < S.ping_interval
        return;
    end
    S.last_ping_time = tic;

    if ~isvalid(S.server) || ~S.server.Connected
        return;
    end

    % 使用应用层 PING（非 ICMP），从 Touch 端拿到真实 RTT
    try
        write(S.server, sprintf('PING|%d', S.ping_count + 1));
    catch
        % Touch 可能断开了，忽略
    end
end

% ======================================================================
%  机械臂健康检查：断连自动重连
% ======================================================================
function S = check_robot_health(S, cfg)
    persistent last_check;
    if isempty(last_check)
        last_check = tic;
    end
    if toc(last_check) < 3  % 每 3 秒检查一次
        return;
    end
    last_check = tic;

    % 检查运动端口
    if isempty(S.robot_motion) || ~isvalid(S.robot_motion)
        log('WARN: Robot motion port (30003) disconnected. Reconnecting...');
        S.robot_motion = reconnect(cfg.robot_ip, cfg.port_motion, cfg);
    end

    % 检查使能端口
    if isempty(S.robot_enable) || ~isvalid(S.robot_enable)
        log('WARN: Robot enable port (29999) disconnected. Reconnecting...');
        S.robot_enable = reconnect(cfg.robot_ip, cfg.port_enable, cfg);
    end
end

% ======================================================================
%  连接机械臂
% ======================================================================
function S = connect_robot(S, cfg)
    S.robot_enable = do_connect(cfg.robot_ip, cfg.port_enable, cfg);
    if ~isempty(S.robot_enable)
        log(sprintf('  Port %d (enable): OK', cfg.port_enable));
    else
        log(sprintf('  Port %d (enable): FAILED', cfg.port_enable));
    end

    S.robot_motion = do_connect(cfg.robot_ip, cfg.port_motion, cfg);
    if ~isempty(S.robot_motion)
        log(sprintf('  Port %d (motion): OK', cfg.port_motion));
    else
        log(sprintf('  Port %d (motion): FAILED', cfg.port_motion));
    end
end

function client = do_connect(ip, port, cfg)
    attempts = 0;
    max_attempts = cfg.max_reconnect_attempts;
    if max_attempts == 0, max_attempts = 1; end

    while attempts < max_attempts
        try
            client = tcpclient(ip, port);
            client.Timeout = cfg.timeout;
            return;  % 成功
        catch
            attempts = attempts + 1;
            if attempts < max_attempts
                pause(cfg.reconnect_interval);
            end
        end
    end
    client = [];
end

function client = reconnect(ip, port, cfg)
    log(sprintf('  Reconnecting to %s:%d...', ip, port));
    client = do_connect(ip, port, cfg);
    if ~isempty(client)
        log(sprintf('  Reconnected to %s:%d', ip, port));
    else
        log(sprintf('  Reconnect to %s:%d FAILED', ip, port));
    end
end

% ======================================================================
%  日志输出（控制台 + 文件）
% ======================================================================
function log(msg, level)
    persistent count;
    if isempty(count), count = 0; end

    if nargin < 2, level = 0; end

    % 时间戳
    t = datestr(now, 'HH:MM:SS.FFF');

    % 级别前缀
    switch level
        case 1, prefix = '';        % 普通转发（不显示时间，紧凑）
        case 2, prefix = '  ';      % 反馈（缩进）
        otherwise, prefix = sprintf('[%s] ', t);
    end

    % 输出
    fprintf('%s%s\n', prefix, msg);

    count = count + 1;
end

% ======================================================================
%  清理：关闭所有连接
% ======================================================================
function do_cleanup(S)
    diary off;
    fprintf('\n[%s] Shutting down...\n', datestr(now, 'HH:MM:SS'));

    % 关闭 Touch 服务器
    if ~isempty(S.server) && isvalid(S.server)
        try
            % 通知 Touch 关闭
            if S.server.Connected
                write(S.server, 'ROBOT_ARM_CLOSED');
            end
            delete(S.server);
            clear S.server;
        catch
        end
    end

    % 关闭机械臂连接
    ports = {'robot_motion', 'robot_enable'};
    for i = 1:2
        client = S.(ports{i});
        if ~isempty(client) && isvalid(client)
            try
                delete(client);
                clear client;
            catch
            end
        end
    end

    fprintf('[%s] Cleanup complete.\n', datestr(now, 'HH:MM:SS'));
end
