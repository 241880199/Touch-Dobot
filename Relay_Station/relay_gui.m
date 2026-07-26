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
    S.joint_angles = [0 0 0 0 0 0]; % 关节角度 J1~J6 (度, 由 C++ 端 J| 协议上报)
    S.force_raw = [0 0 0];
    S.force_filt = [0 0 0];
    S.force_moment = [0 0 0];
    S.force_stale = 0;
    S.touch_relay_delay = 0;
    S.relay_robot_delay = 0;
    S.packet_count = 0;
    S.last_time = tic;

    % TCP 连接
    S.server = [];                 % C++ Touch_Client 连接 (v3.0: 仅数据上报，不直连机械臂)

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

    lbl_status = uilabel(pnl_top, 'Position', [1230 3 150 18], ...
                         'Text', 'C++ Client: OFFLINE', ...
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
        % 启动 TCP 服务器 (v3.0: 仅监听 C++ 端数据上报，不直连机械臂)
        try
            S.server = tcpserver(cfg.listen_ip, cfg.relay_port);
            S.server.Timeout = cfg.timeout;
            S.server.ConnectionChangedFcn = @onServerConnection;
            fprintf('[Relay] TCP server listening on %s:%d\n', cfg.listen_ip, cfg.relay_port);
        catch e
            fprintf('[Relay] ERROR starting server: %s\n', e.message);
        end
    end

    function onServerConnection(src, ~)
        if src.Connected
            fprintf('[Relay] Touch client connected\n');
            lbl_status.Text = 'C++ Client: CONNECTED';
            lbl_status.FontColor = green;
        else
            fprintf('[Relay] Touch client disconnected\n');
            lbl_status.Text = 'C++ Client: OFFLINE';
            lbl_status.FontColor = red;
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

        try
            processNetworkData();
            update3DModel();
            updateTextPanels();
            drawnow limitrate;
        catch
            % Prevent timer from crashing on transient UI errors
        end
    end

    function processNetworkData()
        if isempty(S.server) || ~isvalid(S.server)
            return;
        end

        try
            nAvail = S.server.NumBytesAvailable;
            while nAvail > 0
                raw = readline(S.server);
                if isempty(raw) || ismissing(raw), nAvail = S.server.NumBytesAvailable; continue; end
                % Force to char vector - R2025b tcpserver returns string type
                if isstring(raw), raw = char(raw); end
                if ~ischar(raw), nAvail = S.server.NumBytesAvailable; continue; end
                msg = strtrim(raw);
                if isempty(msg), nAvail = S.server.NumBytesAvailable; continue; end

                S.packet_count = S.packet_count + 1;

                % 解析消息格式
                if startsWith(msg, 'P|')
                    vals = sscanf(msg(3:end), '%f,%f,%f,%f,%f,%f');
                    if length(vals) == 6
                        S.touch_pos = vals';
                        S.robot_target = vals';
                    end
                elseif startsWith(msg, 'C|')
                    addCmdLog(msg(3:end));
                elseif startsWith(msg, 'R|')
                    vals = sscanf(msg(3:end), '%f,%f,%f');
                    if length(vals) == 3
                        S.force_raw = vals';
                    end
                elseif startsWith(msg, 'F|')
                    vals = str2double(split(msg(3:end), ','));
                    if numel(vals) >= 7
                        S.force_raw = vals(1:3)';
                        S.force_filt = vals(1:3)';
                        S.force_moment = vals(4:6)';
                        S.force_stale = vals(7);
                    end
                elseif startsWith(msg, 'J|')
                    vals = sscanf(msg(3:end), '%f,%f,%f,%f,%f,%f');
                    if length(vals) == 6
                        S.joint_angles = vals';
                    end
                elseif startsWith(msg, 'RP|')
                    vals = sscanf(msg(3:end), '%f,%f,%f,%f,%f,%f');
                    if length(vals) == 6
                        S.robot_pos = vals';
                    end
                end

                nAvail = S.server.NumBytesAvailable;
            end
        catch
        end

        % 延迟统计
        t = toc(S.last_time);
        if t > 0.5
            S.touch_relay_delay = t * 1000 / max(S.packet_count, 1);
            S.packet_count = 0;
            S.last_time = tic;
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
        % CR3 多关节 FK 渲染（URDF 运动学参数）
        % 使用 S.joint_angles 驱动 6 个独立关节
        ja = S.joint_angles;

        % FK: 计算各关节位置 (世界坐标, mm)
        joints = computeFK(ja(1), ja(2), ja(3), ja(4), ja(5), ja(6));
        % joints: 7x3 (base=原点 + J1~J6 位置)

        % 绘制连杆 (粗线)
        for i = 1:6
            plot3(ax_3d, ...
                [joints(i,1) joints(i+1,1)], ...
                [joints(i,2) joints(i+1,2)], ...
                [joints(i,3) joints(i+1,3)], ...
                'Color', [0.25 0.28 0.32], 'LineWidth', 6);
        end

        % 绘制关节球 (蓝色)
        for i = 2:7
            [sx, sy, sz] = sphere(10);
            r = 6;  % 关节球半径
            sx = sx * r + joints(i,1);
            sy = sy * r + joints(i,2);
            sz = sz * r + joints(i,3);
            surf(ax_3d, sx, sy, sz, ...
                'FaceColor', [0.30 0.65 1.00], 'EdgeColor', 'none', 'FaceAlpha', 0.7);
        end

        % 底座 (圆柱)
        [cx, cy, cz] = cylinder([10 12], 12);
        cz = cz * 20;  % 底座高度
        surf(ax_3d, cx + joints(1,1), cy + joints(1,2), cz, ...
            'FaceColor', [0.2 0.22 0.25], 'EdgeColor', 'none');

        % 末端执行器标记
        if isequal(pose, S.robot_target)
            plot3(ax_3d, pose(1), pose(2), pose(3), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
        else
            [sx, sy, sz] = sphere(8);
            sx = sx * 8 + pose(1); sy = sy * 8 + pose(2); sz = sz * 8 + pose(3);
            surf(ax_3d, sx, sy, sz, ...
                'FaceColor', [0.2 0.85 0.35], 'EdgeColor', 'none', 'FaceAlpha', 0.8);
        end
    end

    function joints = computeFK(j1, j2, j3, j4, j5, j6)
        % CR3 正向运动学 (URDF 参数, 角度单位为度, 位置单位为 mm)
        d2r = pi / 180;

        % URDF 关节参数 (mm, 弧度)
        j1_z  = 136.0;  % calibrated against real GetPose (+7.7mm)
        j3_x  = -274.0;
        j4_x  = -230.0;  j4_z = 128.3;
        j5_y  = -116.0;
        j6_y  = 105.0;

        % rpy 固定旋转 (URDF fixed-axis: R = Rz(yaw) * Ry(pitch) * Rx(roll))
        j2_ry = pi/2;  j2_rx = pi/2;   % rpy="1.5708 1.5708 0" → Ry(π/2)*Rx(π/2)
        j4_rz = -pi/2;                  % rpy="0 0 -1.5708" → Rz(-π/2)
        j5_rx = pi/2;                   % rpy="1.5708 0 0" → Rx(π/2)
        j6_rx = -pi/2;                  % rpy="-1.5708 0 0" → Rx(-π/2)

        % 4x4 齐次变换矩阵
        T = eye(4);
        joints = zeros(7, 3);
        joints(1,:) = [0 0 0];  % base 原点

        % J1: base → Link1
        T = T * tr(0, 0, j1_z) * rotz(j1 * d2r);
        joints(2,:) = T(1:3,4)';

        % J2: Link1 → Link2
        T = T * roty(j2_ry) * rotx(j2_rx) * rotz(j2 * d2r);
        joints(3,:) = T(1:3,4)';

        % J3: Link2 → Link3
        T = T * tr(j3_x, 0, 0) * rotz(j3 * d2r);
        joints(4,:) = T(1:3,4)';

        % J4: Link3 → Link4
        T = T * tr(j4_x, 0, j4_z) * rotz(j4_rz) * rotz(j4 * d2r);
        joints(5,:) = T(1:3,4)';

        % J5: Link4 → Link5
        T = T * tr(0, j5_y, 0) * rotx(j5_rx) * rotz(j5 * d2r);
        joints(6,:) = T(1:3,4)';

        % J6: Link5 → Link6
        T = T * tr(0, j6_y, 0) * rotx(j6_rx) * rotz(j6 * d2r);
        joints(7,:) = T(1:3,4)';
    end

    % 辅助: 齐次变换
    function T = tr(x, y, z)
        T = eye(4); T(1:3,4) = [x; y; z];
    end
    function R = rotx(angle)
        c = cos(angle); s = sin(angle);
        R = [1 0 0 0; 0 c -s 0; 0 s c 0; 0 0 0 1];
    end
    function R = roty(angle)
        c = cos(angle); s = sin(angle);
        R = [c 0 s 0; 0 1 0 0; -s 0 c 0; 0 0 0 1];
    end
    function R = rotz(angle)
        c = cos(angle); s = sin(angle);
        R = [c -s 0 0; s c 0 0; 0 0 1 0; 0 0 0 1];
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
        mm = S.force_moment;
        lbl_force_raw.Text = {
            sprintf('Raw:  Fx: %7.2f N  Fy: %7.2f N  Fz: %7.2f N', fr(1), fr(2), fr(3)),
            sprintf('      Mx: %7.2f Nm My: %7.2f Nm Mz: %7.2f Nm', mm(1), mm(2), mm(3)),
            ''};
        lbl_force_filt.Text = {
            sprintf('Filt: Fx: %7.2f N  Fy: %7.2f N  Fz: %7.2f N', ff(1), ff(2), ff(3)),
            ''};
        if S.force_stale
            lbl_force_raw.Text{3} = '*** FORCE SENSOR OFFLINE ***';
            lbl_force_raw.FontColor = [1.0 0.3 0.3];
            lbl_force_filt.Text{2} = '*** FORCE SENSOR OFFLINE ***';
            lbl_force_filt.FontColor = [1.0 0.3 0.3];
        else
            lbl_force_raw.FontColor = [0.6 0.65 0.7];
            lbl_force_filt.FontColor = [0.6 0.65 0.7];
        end

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
            sprintf('Joints (deg):  J1:%7.1f  J2:%7.1f  J3:%7.1f', S.joint_angles(1:3)), ...
            sprintf('               J4:%7.1f  J5:%7.1f  J6:%7.1f', S.joint_angles(4:6)), ...
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

        delete(fig);
        disp('[Relay] GUI closed.');
    end
end
