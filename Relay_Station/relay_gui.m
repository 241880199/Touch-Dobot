function relay_gui()
% Touch-Dobot Relay Station GUI
% 三栏布局: 指令/反馈 | 力数据 | 3D数字孪生+坐标

    % ===== 全局状态 =====
    S = struct();
    S.running = true;
    S.cmd_log = cell(50, 1);       % 指令日志
    S.cmd_idx = 0;
    S.fb_log = cell(50, 1);        % 反馈日志
    S.fb_idx = 0;
    S.touch_pos = [0 0 0 0 0 0];   % Touch 位姿
    S.robot_pos = [300 0 200 0 0 0]; % 机械臂位姿 (默认值)
    S.robot_target = [300 0 200 0 0 0];
    S.force_raw = [0 0 0];
    S.force_filt = [0 0 0];
    S.touch_relay_delay = 0;
    S.relay_robot_delay = 0;
    S.packet_count = 0;
    S.last_time = tic;

    % TCP 连接
    S.server = [];                 % Touch 客户端连接
    S.robot_enable = [];           % 机械臂使能端口
    S.robot_motion = [];           % 机械臂运动端口

    % ===== 加载配置 =====
    cfg = relay_config();

    % ===== 创建 GUI 窗口 =====
    fig = uifigure('Name', 'Touch-Dobot Relay Station', ...
                   'Position', [100 50 1400 850], ...
                   'Resize', 'off', ...
                   'CloseRequestFcn', @(~,~) onClose());

    % 颜色主题
    bg_dark  = [0.10 0.12 0.16];
    bg_panel = [0.08 0.10 0.14];
    border   = [0.20 0.30 0.50];
    text_on  = [0.90 0.94 1.00];
    text_dim = [0.55 0.58 0.62];
    green    = [0.20 0.80 0.40];
    red      = [0.95 0.25 0.25];
    blue     = [0.30 0.65 1.00];

    fig.Color = bg_dark;

    % ===== 顶部状态栏 =====
    pnl_top = uipanel(fig, 'Position', [5 820 1390 25], ...
                      'BackgroundColor', [0.06 0.08 0.12], ...
                      'BorderType', 'none');

    lbl_title = uilabel(pnl_top, 'Position', [10 3 220 18], ...
                        'Text', 'Touch-Dobot Relay Station', ...
                        'FontColor', blue, 'FontWeight', 'bold', 'FontSize', 11);

    lbl_delay = uilabel(pnl_top, 'Position', [380 3 300 18], ...
                        'Text', 'Touch->Relay: -- ms  |  Relay->Robot: -- ms', ...
                        'FontColor', green, 'FontSize', 10);

    lbl_ip = uilabel(pnl_top, 'Position', [750 3 200 18], ...
                     'Text', ['Robot IP: ' cfg.robot_ip], ...
                     'FontColor', text_dim, 'FontSize', 10);

    lbl_status = uilabel(pnl_top, 'Position', [1250 3 130 18], ...
                         'Text', 'Robot: OFFLINE', ...
                         'FontColor', red, 'FontSize', 10, ...
                         'HorizontalAlignment', 'right');

    % ===== 布局参数 =====
    panel_y = 5;
    panel_h = 790;
    gap = 5;
    left_w = 380;
    center_w = 380;
    right_w = 620;

    % ===== 左栏: 指令 + 反馈 =====
    sub_h = (panel_h - gap) / 2;

    % 左上: 指令日志
    pnl_cmd = uipanel(fig, 'Position', [5 panel_y left_w panel_h], ...
                      'BackgroundColor', bg_panel, 'BorderType', 'none');
    ax_cmd_title = uiaxes(pnl_cmd, 'Position', [0 sub_h left_w 22], 'Visible', 'off');
    title(ax_cmd_title, 'Touch -> Robot (Commands)', 'Color', [0.80 0.90 1.00], 'FontSize', 11);
    lbl_cmd = uilabel(pnl_cmd, 'Position', [6 8 left_w-12 sub_h-26], ...
                      'Text', '', 'FontColor', [0.30 0.85 0.50], ...
                      'FontSize', 9, 'VerticalAlignment', 'top', ...
                      'FontName', 'Consolas');

    % 左下: 反馈日志
    pnl_fb = uipanel(fig, 'Position', [5 panel_y+sub_h+gap left_w sub_h], ...
                     'BackgroundColor', bg_panel, 'BorderType', 'none');
    ax_fb_title = uiaxes(pnl_fb, 'Position', [0 sub_h left_w 22], 'Visible', 'off');
    title(ax_fb_title, 'Robot -> Relay (Feedback)', 'Color', [0.80 0.90 1.00], 'FontSize', 11);
    lbl_fb = uilabel(pnl_fb, 'Position', [6 8 left_w-12 sub_h-26], ...
                     'Text', '', 'FontColor', [0.60 0.65 0.70], ...
                     'FontSize', 9, 'VerticalAlignment', 'top', ...
                     'FontName', 'Consolas');

    % ===== 中栏: 力数据 =====
    cx = left_w + gap * 2;
    pnl_force_raw = uipanel(fig, 'Position', [cx panel_y+sub_h+gap center_w sub_h], ...
                             'BackgroundColor', bg_panel, 'BorderType', 'none');
    ax_fr_title = uiaxes(pnl_force_raw, 'Position', [0 sub_h center_w 22], 'Visible', 'off');
    title(ax_fr_title, 'Force Sensor (Raw)', 'Color', [0.80 0.90 1.00], 'FontSize', 11);
    lbl_force_raw = uilabel(pnl_force_raw, 'Position', [10 40 center_w-20 sub_h-70], ...
                            'Text', {'Awaiting force sensor data...', '', ...
                                     sprintf('Fx: %6.2f N   Fy: %6.2f N   Fz: %6.2f N', 0,0,0)}, ...
                            'FontColor', text_dim, 'FontSize', 10, ...
                            'VerticalAlignment', 'top', 'FontName', 'Consolas');

    pnl_force_filt = uipanel(fig, 'Position', [cx panel_y center_w sub_h], ...
                              'BackgroundColor', bg_panel, 'BorderType', 'none');
    ax_ff_title = uiaxes(pnl_force_filt, 'Position', [0 sub_h center_w 22], 'Visible', 'off');
    title(ax_ff_title, 'Force Output (Filtered -> Touch)', 'Color', [0.80 0.90 1.00], 'FontSize', 11);
    lbl_force_filt = uilabel(pnl_force_filt, 'Position', [10 40 center_w-20 sub_h-70], ...
                             'Text', {'Filtered force for haptic feedback...', '', ...
                                      sprintf('Fx: %6.2f N   Fy: %6.2f N   Fz: %6.2f N', 0,0,0)}, ...
                             'FontColor', text_dim, 'FontSize', 10, ...
                             'VerticalAlignment', 'top', 'FontName', 'Consolas');

    % ===== 右栏: 3D 模型 + 坐标 =====
    rx = cx + center_w + gap;
    rw = right_w;
    r3d_h = 495;
    rcoord_h = panel_h - r3d_h - gap;

    % 右上: 3D 模型
    pnl_3d = uipanel(fig, 'Position', [rx panel_y+rcoord_h+gap rw r3d_h], ...
                     'BackgroundColor', bg_panel, 'BorderType', 'none');
    ax_3d = uiaxes(pnl_3d, 'Position', [5 5 rw-10 r3d_h-10], ...
                   'Color', [0.12 0.14 0.18], ...
                   'XColor', text_dim, 'YColor', text_dim, 'ZColor', text_dim, ...
                   'Box', 'on', 'GridLineStyle', ':');
    title(ax_3d, 'Digital Twin', 'Color', text_on, 'FontSize', 11);
    xlabel(ax_3d, 'X (mm)'); ylabel(ax_3d, 'Y (mm)'); zlabel(ax_3d, 'Z (mm)');
    hold(ax_3d, 'on');
    axis(ax_3d, 'equal');
    ax_3d.View = [45 30];
    xlim(ax_3d, [0 500]); ylim(ax_3d, [-250 250]); zlim(ax_3d, [0 400]);

    % 右下: 坐标 + 力数值
    pnl_coord = uipanel(fig, 'Position', [rx panel_y rw rcoord_h], ...
                        'BackgroundColor', bg_panel, 'BorderType', 'none');
    ax_coord = uiaxes(pnl_coord, 'Position', [0 rcoord_h rw 22], 'Visible', 'off');
    title(ax_coord, 'Robot State', 'Color', [0.80 0.90 1.00], 'FontSize', 11);
    lbl_coord = uilabel(pnl_coord, 'Position', [10 10 rw-20 rcoord_h-35], ...
                        'Text', '', 'FontColor', [0.70 0.85 0.50], ...
                        'FontSize', 10, 'VerticalAlignment', 'top', ...
                        'FontName', 'Consolas');

    % ===== 更新定时器 =====
    tmr = timer('Period', 0.05, 'ExecutionMode', 'fixedRate', ...
                'TimerFcn', @(~,~) updateDisplay(), ...
                'ErrorFcn', @(~,~) disp('Timer error'));

    % ===== 网络初始化 =====
    initNetwork();

    % ===== 绘制初始 3D 场景 =====
    init3DScene();

    % ===== 启动定时器 =====
    start(tmr);

    % ===== 嵌套函数 =====

    function initNetwork()
        % 启动 TCP 服务器
        try
            S.server = tcpserver(cfg.listen_ip, cfg.relay_port);
            S.server.Timeout = cfg.timeout;
            S.server.ConnectionChangedFcn = @onServerConnection;
            fprintf('[Relay] TCP server listening on %s:%d\n', cfg.listen_ip, cfg.relay_port);
        catch e
            fprintf('[Relay] ERROR starting server: %s\n', e.message);
        end

        % 连接机械臂
        try
            S.robot_enable = tcpclient(cfg.robot_ip, cfg.port_enable);
            S.robot_enable.Timeout = cfg.timeout;
            fprintf('[Relay] Connected to robot enable port %d\n', cfg.port_enable);
        catch
            fprintf('[Relay] WARNING: Cannot connect to robot enable port\n');
        end

        try
            S.robot_motion = tcpclient(cfg.robot_ip, cfg.port_motion);
            S.robot_motion.Timeout = cfg.timeout;
            fprintf('[Relay] Connected to robot motion port %d\n', cfg.port_motion);
        catch
            fprintf('[Relay] WARNING: Cannot connect to robot motion port\n');
        end
    end

    function onServerConnection(src, ~)
        if src.Connected
            fprintf('[Relay] Touch client connected\n');
        else
            fprintf('[Relay] Touch client disconnected\n');
        end
    end

    function init3DScene()
        % 绘制地面网格
        [X, Y] = meshgrid(0:50:500, -250:50:250);
        Z = zeros(size(X));
        mesh(ax_3d, X, Y, Z, 'FaceAlpha', 0.1, 'EdgeColor', [0.2 0.25 0.3], 'LineWidth', 0.5);

        % 坐标系
        quiver3(ax_3d, 0, 0, 0, 80, 0, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 5);
        quiver3(ax_3d, 0, 0, 0, 0, 80, 0, 'g', 'LineWidth', 2, 'MaxHeadSize', 5);
        quiver3(ax_3d, 0, 0, 0, 0, 0, 80, 'b', 'LineWidth', 2, 'MaxHeadSize', 5);

        % 安全边界线框
        xLim = [cfg.safe_x_min cfg.safe_x_max];
        yLim = [cfg.safe_y_min cfg.safe_y_max];
        zLim = [cfg.safe_z_min cfg.safe_z_max];
        plot3(ax_3d, xLim([1 1 2 2 1]), yLim([1 2 2 1 1]), zLim([1 1 1 1 1]), 'y--', 'LineWidth', 1);
        plot3(ax_3d, xLim([1 1 2 2 1]), yLim([1 2 2 1 1]), zLim([2 2 2 2 2]), 'y--', 'LineWidth', 1);
        for i = 1:2
            for j = 1:2
                plot3(ax_3d, [xLim(i) xLim(i)], [yLim(j) yLim(j)], zLim, 'y--', 'LineWidth', 1);
            end
        end
    end

    function updateDisplay()
        if ~S.running, return; end

        % 处理 TCP 数据
        processNetworkData();

        % 更新 3D 模型
        update3DModel();

        % 更新文本面板
        updateTextPanels();

        drawnow limitrate;
    end

    function processNetworkData()
        % 从 Touch 客户端读取数据
        if isempty(S.server) || ~isvalid(S.server) || ~S.server.Connected
            return;
        end
        if S.server.NumBytesAvailable == 0
            return;
        end

        try
            data = read(S.server, S.server.NumBytesAvailable, 'uint8');
            msgs = strsplit(char(data), '\n');
            for i = 1:length(msgs)
                msg = strtrim(msgs{i});
                if isempty(msg), continue; end

                S.packet_count = S.packet_count + 1;

                % 解析消息格式
                if startsWith(msg, 'P|')  % Touch 位姿
                    vals = sscanf(msg(3:end), '%f,%f,%f,%f,%f,%f');
                    if length(vals) == 6
                        S.touch_pos = vals';
                    end
                elseif startsWith(msg, 'C|')  % 指令
                    addCmdLog(msg(3:end));
                    % 转发到机械臂
                    forwardToRobot(msg(3:end));
                elseif startsWith(msg, 'R|')  % 力数据
                    vals = sscanf(msg(3:end), '%f,%f,%f');
                    if length(vals) == 3
                        S.force_raw = vals';
                    end
                end
            end
        catch
        end

        % 读取机械臂反馈
        readRobotFeedback();

        % 更新延迟
        t = toc(S.last_time);
        if t > 0.5
            S.touch_relay_delay = t * 1000 / max(S.packet_count, 1);
            S.packet_count = 0;
            S.last_time = tic;
        end
    end

    function forwardToRobot(cmd)
        % 解析端口和指令
        parts = strsplit(cmd, '|');
        if length(parts) ~= 2, return; end
        targetPort = str2double(parts{1});
        cmdStr = parts{2};

        client = [];
        if targetPort == cfg.port_motion && ~isempty(S.robot_motion) && isvalid(S.robot_motion)
            client = S.robot_motion;
        elseif targetPort == cfg.port_enable && ~isempty(S.robot_enable) && isvalid(S.robot_enable)
            client = S.robot_enable;
        end

        if ~isempty(client)
            try
                write(client, cmdStr);
            catch
            end
        end
    end

    function readRobotFeedback()
        % 读取运动端口
        if ~isempty(S.robot_motion) && isvalid(S.robot_motion)
            while S.robot_motion.NumBytesAvailable > 0
                try
                    fb = read(S.robot_motion, S.robot_motion.NumBytesAvailable, 'uint8');
                    fbStr = strtrim(char(fb));
                    if ~isempty(fbStr)
                        addFbLog(['[30003] ' fbStr]);
                        % 尝试解析位姿
                        if contains(fbStr, 'GetPose')
                            vals = sscanf(fbStr, '0,{%f,%f,%f,%f,%f,%f');
                            if length(vals) == 6
                                S.robot_pos = vals';
                            end
                        end
                    end
                catch
                    break;
                end
            end
        end

        % 读取使能端口
        if ~isempty(S.robot_enable) && isvalid(S.robot_enable)
            while S.robot_enable.NumBytesAvailable > 0
                try
                    fb = read(S.robot_enable, S.robot_enable.NumBytesAvailable, 'uint8');
                    fbStr = strtrim(char(fb));
                    if ~isempty(fbStr)
                        addFbLog(['[29999] ' fbStr]);
                    end
                catch
                    break;
                end
            end
        end

        % 更新机器人状态指示器
        robotOnline = (~isempty(S.robot_enable) && isvalid(S.robot_enable)) && ...
                      (~isempty(S.robot_motion) && isvalid(S.robot_motion));
        if robotOnline
            lbl_status.Text = 'Robot: ONLINE';
            lbl_status.FontColor = green;
        end
    end

    function addCmdLog(msg)
        S.cmd_idx = mod(S.cmd_idx, 50) + 1;
        S.cmd_log{S.cmd_idx} = msg;
    end

    function addFbLog(msg)
        S.fb_idx = mod(S.fb_idx, 50) + 1;
        S.fb_log{S.fb_idx} = msg;
    end

    function update3DModel()
        cla(ax_3d);
        init3DScene();

        % 绘制简化的机械臂模型
        drawRobotModel(S.robot_pos);
        drawRobotModel(S.robot_target); % 半透明目标位置

        % 绘制 Touch 笔
        tp = S.touch_pos;
        if any(tp(1:3) ~= 0)
            % 笔身
            [cx, cy, cz] = cylinder([2 1.5], 8);
            cz = cz * 40;
            cx = cx + tp(1);
            cy = cy + tp(2);
            cz = cz + tp(3);
            surf(ax_3d, cx, cy, cz, 'FaceColor', [0.35 0.38 0.42], 'EdgeColor', 'none');
            % 笔尖红球
            [sx, sy, sz] = sphere(12);
            sx = sx * 4 + tp(1);
            sy = sy * 4 + tp(2);
            sz = sz * 4 + tp(3);
            surf(ax_3d, sx, sy, sz, 'FaceColor', [1 0.15 0.1], 'EdgeColor', 'none', 'FaceAlpha', 0.9);
        end
    end

    function drawRobotModel(pose)
        x = pose(1); y = pose(2); z = pose(3);
        % 简化: 绘制末端执行器位置标记
        [sx, sy, sz] = sphere(8);
        sx = sx * 8 + x; sy = sy * 8 + y; sz = sz * 8 + z;
        if isequal(pose, S.robot_target)
            % 目标: 半透明红色线框
            plot3(ax_3d, x, y, z, 'ro', 'MarkerSize', 10, 'LineWidth', 2);
        else
            % 实际: 绿色实体
            surf(ax_3d, sx, sy, sz, 'FaceColor', [0.2 0.85 0.35], 'EdgeColor', 'none', 'FaceAlpha', 0.8);
        end

        % 简易连杆 (底座到末端)
        bx = [300 300 x];
        by = [0 0 y];
        bz = [0 z z];
        plot3(ax_3d, bx, by, bz, 'Color', [0.25 0.28 0.32], 'LineWidth', 5);
    end

    function updateTextPanels()
        % 指令日志
        lines = {};
        for i = 1:50
            idx = mod(S.cmd_idx - i + 50, 50) + 1;
            if ~isempty(S.cmd_log{idx})
                lines{end+1} = S.cmd_log{idx};
            end
        end
        if isempty(lines)
            lbl_cmd.Text = '(waiting for commands...)';
        else
            lbl_cmd.Text = lines;
        end

        % 反馈日志
        lines = {};
        for i = 1:50
            idx = mod(S.fb_idx - i + 50, 50) + 1;
            if ~isempty(S.fb_log{idx})
                lines{end+1} = S.fb_log{idx};
            end
        end
        if isempty(lines)
            lbl_fb.Text = '(waiting for feedback...)';
        else
            lbl_fb.Text = lines;
        end

        % 力数据
        fr = S.force_raw;
        ff = S.force_filt;
        lbl_force_raw.Text = { ...
            'Force sensor data from robot end-effector', '', ...
            sprintf('Fx: %7.2f N   Fy: %7.2f N   Fz: %7.2f N', fr(1), fr(2), fr(3))};
        lbl_force_filt.Text = { ...
            'Filtered force sent to Touch device', '', ...
            sprintf('Fx: %7.2f N   Fy: %7.2f N   Fz: %7.2f N', ff(1), ff(2), ff(3))};

        % 坐标
        rp = S.robot_pos;
        rt = S.robot_target;
        lbl_coord.Text = { ...
            sprintf('Position (mm):    X: %8.2f  (target: %8.2f)', rp(1), rt(1)), ...
            sprintf('                   Y: %8.2f  (target: %8.2f)', rp(2), rt(2)), ...
            sprintf('                   Z: %8.2f  (target: %8.2f)', rp(3), rt(3)), ...
            '', ...
            sprintf('Orientation (deg):  Rx: %7.2f  Ry: %7.2f  Rz: %7.2f', rp(4), rp(5), rp(6)), ...
            '', ...
            sprintf('Force (N):   Fx: %7.2f   Fy: %7.2f   Fz: %7.2f', ff(1), ff(2), ff(3))};

        % 延迟
        lbl_delay.Text = sprintf('Touch->Relay: %.1f ms  |  Relay->Robot: -- ms', S.touch_relay_delay);
    end

    function onClose()
        S.running = false;
        stop(tmr);
        delete(tmr);

        % 关闭连接
        if ~isempty(S.server) && isvalid(S.server)
            delete(S.server);
        end
        if ~isempty(S.robot_enable) && isvalid(S.robot_enable)
            delete(S.robot_enable);
        end
        if ~isempty(S.robot_motion) && isvalid(S.robot_motion)
            delete(S.robot_motion);
        end

        delete(fig);
        disp('[Relay] GUI closed.');
    end
end
