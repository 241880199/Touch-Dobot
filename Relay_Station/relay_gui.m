function relay_gui()
% Touch-Dobot Relay Station v4.0
% 全可视化 MATLAB GUI: STL 3D + uigridlayout + 完整数据面板
% C++ 端仅保留控制层, 所有显示由此 GUI 负责

    % ===== 全局状态 =====
    S = struct();
    S.running = true;
    S.cmd_log = cell(50,1);  S.cmd_idx = 0;
    S.fb_log  = cell(50,1);  S.fb_idx  = 0;
    S.touch_pos    = [0 0 0 0 0 0];
    S.robot_pos    = [300 0 200 0 0 0];
    S.robot_target = [300 0 200 0 0 0];
    S.joint_angles = [0 0 0 0 0 0];
    S.force_raw    = [0 0 0];
    S.force_filt   = [0 0 0];
    S.force_moment = [0 0 0];
    S.force_stale  = 0;
    S.touch_relay_delay = 0;
    S.packet_count = 0;
    S.last_time = tic;
    % 新增状态字段 (Task 6/7 将使用)
    S.safety_state = 0;      S.safety_speed = 1.0;  S.safety_alarms = 0;
    S.joint_margins = [999 999 999 999 999 999];
    S.z_dist = 999;          S.singular = 0;
    S.calib_enabled = false; S.calib_rms = -1;
    S.diag_code = 0;         S.diag_spd = 1.0;  S.diag_reason = '';
    S.server = [];
    % 3D 场景对象 (Task 7)
    S.linkMesh     = {};    S.linkPatch = gobjects(1,0);  S.linkHg = gobjects(1,0);
    S.stlLoaded    = false;
    S.touchPenBody = [];    S.touchPenTip = [];
    S.eeMarkerActual = [];  S.eeMarkerTarget = [];

    % ===== 加载配置 =====
    cfg = relay_config();

    % ===== 确保 +stl +fk 在路径中 =====
    scriptDir = fileparts(mfilename('fullpath'));
    if ~contains(path, fullfile(scriptDir, '+stl'))
        addpath(scriptDir);
    end

    % ===== 颜色主题 =====
    clr = struct(...
        'bg_dark',  [0.10 0.12 0.16], 'bg_panel',  [0.08 0.10 0.14], ...
        'bg_topbar',[0.06 0.08 0.12], 'bg_axes3d', [0.12 0.14 0.18], ...
        'border',   [0.20 0.30 0.50], 'text_on',   [0.90 0.94 1.00], ...
        'text_dim', [0.55 0.58 0.62], 'green',     [0.20 0.80 0.40], ...
        'red',      [0.95 0.25 0.25], 'blue',      [0.30 0.65 1.00], ...
        'orange',   [1.00 0.60 0.15], 'yellow',    [1.00 0.85 0.20]);

    % ===== 主窗口 =====
    fig = uifigure('Name', 'Touch-Dobot Relay Station', ...
                   'Position', [100 50 1400 850], ...
                   'Color', clr.bg_dark, ...
                   'CloseRequestFcn', @(~,~) onClose());
    fig.SizeChangedFcn = @(~,~) onResize();

    % ===== 外层网格: 2行 × 3列 =====
    g = uigridlayout(fig, [2 3]);
    g.RowHeight = {25, '1x'};            % 顶部 25px + 内容区 fill
    g.ColumnWidth = {'1x', '1x', '1.6x'}; % 左:中:右 = 1:1:1.6
    g.Padding = [2 2 2 2];
    g.RowSpacing = 2;
    g.ColumnSpacing = 2;
    g.BackgroundColor = clr.bg_dark;

    % ===== 顶部状态栏 =====
    pnlTop = uipanel(g, 'BackgroundColor', clr.bg_topbar, 'BorderType', 'none');
    pnlTop.Layout.Row = 1;
    pnlTop.Layout.Column = [1 3];

    glTop = uigridlayout(pnlTop, [1 6]);
    glTop.ColumnWidth = {220, 280, 180, '1x', 150, 120};
    glTop.Padding = [4 1 4 1];
    glTop.BackgroundColor = clr.bg_topbar;

    lblTitle = uilabel(glTop, 'Text', 'Touch-Dobot Relay Station', ...
        'FontColor', clr.blue, 'FontWeight', 'bold', 'FontSize', 11);
    lblDelay = uilabel(glTop, 'Text', 'Touch->Relay: -- ms', ...
        'FontColor', clr.green, 'FontSize', 10, 'FontName', 'Consolas');
    lblIp = uilabel(glTop, 'Text', ['Robot IP: ' cfg.robot_ip], ...
        'FontColor', clr.text_dim, 'FontSize', 10, 'FontName', 'Consolas');
    lblSpacer = uilabel(glTop, 'Text', '');
    lblState = uilabel(glTop, 'Text', '[--]', 'FontColor', clr.text_dim, ...
        'FontSize', 10, 'HorizontalAlignment', 'right', 'FontName', 'Consolas');
    lblConn = uilabel(glTop, 'Text', 'C++ Client: OFFLINE', ...
        'FontColor', clr.red, 'FontSize', 10, 'HorizontalAlignment', 'right', 'FontName', 'Consolas');

    % ===== 左栏 (Col 1): 指令 + 反馈 =====
    pnlLeft = uipanel(g, 'BackgroundColor', clr.bg_panel, 'BorderType', 'none');
    pnlLeft.Layout.Row = 2;  pnlLeft.Layout.Column = 1;
    glLeft = uigridlayout(pnlLeft, [2 1]);
    glLeft.RowHeight = {'1x', '1x'};
    glLeft.Padding = [1 1 1 1];  glLeft.RowSpacing = 2;
    glLeft.BackgroundColor = clr.bg_panel;

    % 指令面板
    pnlCmd = uigridlayout(glLeft, [2 1]);
    pnlCmd.RowHeight = {22, '1x'};
    pnlCmd.Padding = [4 0 4 2];  pnlCmd.RowSpacing = 0;
    pnlCmd.BackgroundColor = clr.bg_panel;
    pnlCmd.Layout.Row = 1;  pnlCmd.Layout.Column = 1;

    lblCmdTitle = uilabel(pnlCmd, 'Text', 'Touch -> Robot (Commands)', ...
        'FontColor', clr.text_on, 'FontSize', 11, 'FontWeight', 'bold');
    lblCmdTitle.Layout.Row = 1;  lblCmdTitle.Layout.Column = 1;

    lblCmd = uilabel(pnlCmd, 'Text', '(waiting for commands...)', ...
        'FontColor', [0.30 0.85 0.50], 'FontSize', 9, ...
        'VerticalAlignment', 'top', 'FontName', 'Consolas');
    lblCmd.Layout.Row = 2;  lblCmd.Layout.Column = 1;

    % 反馈面板
    pnlFb = uigridlayout(glLeft, [2 1]);
    pnlFb.RowHeight = {22, '1x'};
    pnlFb.Padding = [4 0 4 2];  pnlFb.RowSpacing = 0;
    pnlFb.BackgroundColor = clr.bg_panel;
    pnlFb.Layout.Row = 2;  pnlFb.Layout.Column = 1;

    lblFbTitle = uilabel(pnlFb, 'Text', 'Robot -> Relay (Feedback)', ...
        'FontColor', clr.text_on, 'FontSize', 11, 'FontWeight', 'bold');
    lblFbTitle.Layout.Row = 1;  lblFbTitle.Layout.Column = 1;

    lblFb = uilabel(pnlFb, 'Text', '(waiting for feedback...)', ...
        'FontColor', clr.text_dim, 'FontSize', 9, ...
        'VerticalAlignment', 'top', 'FontName', 'Consolas');
    lblFb.Layout.Row = 2;  lblFb.Layout.Column = 1;

    % ===== 中栏 (Col 2): 力数据 =====
    pnlMid = uipanel(g, 'BackgroundColor', clr.bg_panel, 'BorderType', 'none');
    pnlMid.Layout.Row = 2;  pnlMid.Layout.Column = 2;
    glMid = uigridlayout(pnlMid, [2 1]);
    glMid.RowHeight = {'1x', '1x'};
    glMid.Padding = [1 1 1 1];  glMid.RowSpacing = 2;
    glMid.BackgroundColor = clr.bg_panel;

    % 原始力
    pnlFR = uigridlayout(glMid, [2 1]);
    pnlFR.RowHeight = {22, '1x'};
    pnlFR.Padding = [4 0 4 2];  pnlFR.RowSpacing = 0;
    pnlFR.BackgroundColor = clr.bg_panel;
    pnlFR.Layout.Row = 1;  pnlFR.Layout.Column = 1;

    lblFRTitle = uilabel(pnlFR, 'Text', 'Force Sensor (Raw · 30004)', ...
        'FontColor', clr.text_on, 'FontSize', 11, 'FontWeight', 'bold');
    lblFRTitle.Layout.Row = 1;  lblFRTitle.Layout.Column = 1;

    lblForceRaw = uilabel(pnlFR, 'Text', {'Awaiting force sensor data...', '', ...
        'Fx:   0.00 N   Fy:   0.00 N   Fz:   0.00 N'}, ...
        'FontColor', clr.text_dim, 'FontSize', 10, ...
        'VerticalAlignment', 'top', 'FontName', 'Consolas');
    lblForceRaw.Layout.Row = 2;  lblForceRaw.Layout.Column = 1;

    % 滤波力
    pnlFF = uigridlayout(glMid, [2 1]);
    pnlFF.RowHeight = {22, '1x'};
    pnlFF.Padding = [4 0 4 2];  pnlFF.RowSpacing = 0;
    pnlFF.BackgroundColor = clr.bg_panel;
    pnlFF.Layout.Row = 2;  pnlFF.Layout.Column = 1;

    lblFFTitle = uilabel(pnlFF, 'Text', 'Force Output (Filtered -> Touch)', ...
        'FontColor', clr.text_on, 'FontSize', 11, 'FontWeight', 'bold');
    lblFFTitle.Layout.Row = 1;  lblFFTitle.Layout.Column = 1;

    lblForceFilt = uilabel(pnlFF, 'Text', {'Filtered force for haptic feedback...', '', ...
        'Fx:   0.00 N   Fy:   0.00 N   Fz:   0.00 N'}, ...
        'FontColor', clr.text_dim, 'FontSize', 10, ...
        'VerticalAlignment', 'top', 'FontName', 'Consolas');
    lblForceFilt.Layout.Row = 2;  lblForceFilt.Layout.Column = 1;

    % ===== 右栏 (Col 3): 3D + 状态 + 安全 =====
    pnlRight = uipanel(g, 'BackgroundColor', clr.bg_panel, 'BorderType', 'none');
    pnlRight.Layout.Row = 2;  pnlRight.Layout.Column = 3;
    glRight = uigridlayout(pnlRight, [3 1]);
    glRight.RowHeight = {'3x', '1x', '1x'};
    glRight.Padding = [1 1 1 1];  glRight.RowSpacing = 2;
    glRight.BackgroundColor = clr.bg_panel;

    % 3D 视图
    pnl3D = uigridlayout(glRight, [1 1]);
    pnl3D.Padding = [0 0 0 0];
    pnl3D.BackgroundColor = clr.bg_panel;
    pnl3D.Layout.Row = 1;  pnl3D.Layout.Column = 1;
    ax3d = uiaxes(pnl3D, 'BackgroundColor', clr.bg_axes3d, ...
        'XColor', clr.text_dim, 'YColor', clr.text_dim, 'ZColor', clr.text_dim, ...
        'Box', 'on', 'GridLineStyle', ':');
    title(ax3d, 'Digital Twin', 'Color', clr.text_on, 'FontSize', 11);
    xlabel(ax3d, 'X (mm)'); ylabel(ax3d, 'Y (mm)'); zlabel(ax3d, 'Z (mm)');
    hold(ax3d, 'on'); axis(ax3d, 'equal');
    ax3d.Layout.Row = 1;  ax3d.Layout.Column = 1;
    ax3d.View = [60 25];
    xlim(ax3d, [-350 400]); ylim(ax3d, [-400 400]); zlim(ax3d, [-50 800]);

    % Robot State 面板
    pnlState = uigridlayout(glRight, [2 1]);
    pnlState.RowHeight = {22, '1x'};
    pnlState.Padding = [2 2 2 2];  pnlState.RowSpacing = 0;
    pnlState.BackgroundColor = clr.bg_panel;
    pnlState.Layout.Row = 2;  pnlState.Layout.Column = 1;

    lblStateTitle = uilabel(pnlState, 'Text', 'Robot State', ...
        'FontColor', clr.text_on, 'FontSize', 11, 'FontWeight', 'bold');
    lblStateTitle.Layout.Row = 1;  lblStateTitle.Layout.Column = 1;

    lblCoord = uilabel(pnlState, 'Text', 'Initializing...', ...
        'FontColor', [0.70 0.85 0.50], 'FontSize', 10, ...
        'VerticalAlignment', 'top', 'FontName', 'Consolas');
    lblCoord.Layout.Row = 2;  lblCoord.Layout.Column = 1;

    % Safety 面板
    pnlSafety = uigridlayout(glRight, [2 1]);
    pnlSafety.RowHeight = {22, '1x'};
    pnlSafety.Padding = [2 2 2 2];  pnlSafety.RowSpacing = 0;
    pnlSafety.BackgroundColor = clr.bg_panel;
    pnlSafety.Layout.Row = 3;  pnlSafety.Layout.Column = 1;

    lblSafeTitle = uilabel(pnlSafety, 'Text', 'Safety & Diagnostics', ...
        'FontColor', clr.text_on, 'FontSize', 11, 'FontWeight', 'bold');
    lblSafeTitle.Layout.Row = 1;  lblSafeTitle.Layout.Column = 1;

    lblSafety = uilabel(pnlSafety, 'Text', 'Safety: --', ...
        'FontColor', clr.green, 'FontSize', 10, ...
        'VerticalAlignment', 'top', 'FontName', 'Consolas');
    lblSafety.Layout.Row = 2;  lblSafety.Layout.Column = 1;

    % ===== STL 模型加载 =====
    stlDir = fullfile(scriptDir, '..', 'Touch_Client', 'models', 'cr3');
    linkNames = {'base_link', 'Link1', 'Link2', 'Link3', 'Link4', 'Link5', 'Link6'};
    linkMesh = cell(1, 7);
    linkPatch = gobjects(1, 7);
    linkHg = gobjects(1, 7);
    stlLoaded = false;

    for i = 1:7
        stlPath = fullfile(stlDir, [linkNames{i} '.STL']);
        linkMesh{i} = stl.loadBinaryStl(stlPath);
        if linkMesh{i}.triangleCount > 0
            stlLoaded = true;
        end
    end

    % ===== 3D 场景初始化 =====
    % 地面网格
    [Xg, Yg] = meshgrid(-300:50:400, -400:50:400);
    Zg = zeros(size(Xg));
    mesh(ax3d, Xg, Yg, Zg, 'FaceAlpha', 0.1, 'EdgeColor', [0.2 0.25 0.3], 'LineWidth', 0.5);

    % 坐标系
    quiver3(ax3d, 0,0,0, 150,0,0, 'r', 'LineWidth', 2, 'MaxHeadSize', 8);
    quiver3(ax3d, 0,0,0, 0,150,0, 'g', 'LineWidth', 2, 'MaxHeadSize', 8);
    quiver3(ax3d, 0,0,0, 0,0,150, 'b', 'LineWidth', 2, 'MaxHeadSize', 8);

    % 安全边界线框
    xL = [cfg.safe_x_min cfg.safe_x_max];
    yL = [cfg.safe_y_min cfg.safe_y_max];
    zL = [cfg.safe_z_min cfg.safe_z_max];
    plot3(ax3d, xL([1 1 2 2 1]), yL([1 2 2 1 1]), zL([1 1 1 1 1]), 'y--', 'LineWidth', 1);
    plot3(ax3d, xL([1 1 2 2 1]), yL([1 2 2 1 1]), zL([2 2 2 2 2]), 'y--', 'LineWidth', 1);
    for ii = 1:2
        for jj = 1:2
            plot3(ax3d, [xL(ii) xL(ii)], [yL(jj) yL(jj)], zL, 'y--', 'LineWidth', 1);
        end
    end

    % 创建 STL patch 对象 (如果加载成功) 否则 fallback 骨架模型
    if stlLoaded
        for i = 1:7
            linkHg(i) = hgtransform(ax3d);
            linkPatch(i) = patch(linkHg(i), 'Faces', linkMesh{i}.faces, ...
                'Vertices', linkMesh{i}.vertices, ...
                'FaceColor', [0.25 0.28 0.32], 'EdgeColor', 'none', ...
                'FaceLighting', 'gouraud', 'AmbientStrength', 0.5);
        end
    end

    % 添加光源 (独立于 STL 加载状态, 确保场景始终有光照)
    light(ax3d, 'Position', [300 -300 400], 'Style', 'local');

    % Touch 笔可视化对象
    touchPenBody = surface(ax3d, [], [], [], 'FaceColor', [0.35 0.38 0.42], ...
        'EdgeColor', 'none', 'Visible', 'off');
    touchPenTip = surface(ax3d, [], [], [], 'FaceColor', [1 0.15 0.1], ...
        'EdgeColor', 'none', 'FaceAlpha', 0.9, 'Visible', 'off');
    % 末端标记
    eeMarkerActual = surface(ax3d, [], [], [], 'FaceColor', [0.2 0.85 0.35], ...
        'EdgeColor', 'none', 'FaceAlpha', 0.8, 'Visible', 'off');
    eeMarkerTarget = line(ax3d, 0, 0, 0, 'Color', 'r', 'Marker', 'o', ...
        'MarkerSize', 10, 'LineWidth', 2, 'Visible', 'off');

    % 持久化到 S 结构体供后续渲染使用
    S.linkMesh = linkMesh;
    S.linkPatch = linkPatch;
    S.linkHg = linkHg;
    S.stlLoaded = stlLoaded;
    S.touchPenBody = touchPenBody;
    S.touchPenTip = touchPenTip;
    S.eeMarkerActual = eeMarkerActual;
    S.eeMarkerTarget = eeMarkerTarget;

    % Precompute static geometries
    [S.cylX, S.cylY, S.cylZ] = cylinder([2 1.5], 8);
    S.cylZ = S.cylZ * 40;
    [S.sphereX, S.sphereY, S.sphereZ] = sphere(12);
    [S.sphere8X, S.sphere8Y, S.sphere8Z] = sphere(8);

    % ===== 更新定时器 (20Hz) =====
    tmr = timer('Period', 0.05, 'ExecutionMode', 'fixedRate', ...
                'TimerFcn', @(~,~) updateDisplay(), ...
                'ErrorFcn', @(~,~) disp('Timer error'));

    % ===== 启动 =====
    initNetwork();
    start(tmr);
    fprintf('[Relay] GUI ready.\n');

    % ===== 嵌套函数 =====

    function onResize()
        % 窗口大小变化回调 (Task 6 将实现响应式布局调整)
    end

    function onClose()
        S.running = false;
        stop(tmr); delete(tmr);
        if ~isempty(S.server) && isvalid(S.server), delete(S.server); end
        delete(fig);
        disp('[Relay] GUI closed.');
    end

    % ===== 网络初始化 =====
    function initNetwork()
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
            lblConn.Text = 'C++ Client: CONNECTED';
            lblConn.FontColor = clr.green;
        else
            fprintf('[Relay] Touch client disconnected\n');
            lblConn.Text = 'C++ Client: OFFLINE';
            lblConn.FontColor = clr.red;
        end
    end

    % ===== 主更新循环 =====
    function updateDisplay()
        if ~S.running, return; end
        try
            processNetworkData();
            update3DModel();
            updateTextPanels();
            drawnow limitrate;
        catch ME
            fprintf('[Relay] ERROR in updateDisplay: %s\n', ME.message);
        end
    end

    function processNetworkData()
        if isempty(S.server) || ~isvalid(S.server) || S.server.NumBytesAvailable == 0
            return;
        end
        try
            while S.server.NumBytesAvailable > 0
                raw = readline(S.server);
                if isempty(raw) || ismissing(raw), continue; end
                if isstring(raw), raw = char(raw); end
                if ~ischar(raw), continue; end
                msg = strtrim(raw);
                if isempty(msg), continue; end

                S.packet_count = S.packet_count + 1;

                % -- 现有协议 --
                if startsWith(msg, 'P|')
                    vals = sscanf(msg(3:end), '%f,%f,%f,%f,%f,%f');
                    if length(vals) == 6, S.touch_pos = vals'; S.robot_target = vals'; end
                elseif startsWith(msg, 'C|')
                    S.cmd_idx = mod(S.cmd_idx, 50) + 1;
                    S.cmd_log{S.cmd_idx} = msg(3:end);
                elseif startsWith(msg, 'F|')
                    vals = str2double(split(msg(3:end), ','));
                    if numel(vals) >= 7
                        S.force_raw = vals(1:3)'; S.force_filt = vals(1:3)';
                        S.force_moment = vals(4:6)'; S.force_stale = vals(7);
                    end
                elseif startsWith(msg, 'J|')
                    vals = sscanf(msg(3:end), '%f,%f,%f,%f,%f,%f');
                    if length(vals) == 6, S.joint_angles = vals'; end
                elseif startsWith(msg, 'RP|')
                    vals = sscanf(msg(3:end), '%f,%f,%f,%f,%f,%f');
                    if length(vals) == 6, S.robot_pos = vals'; end
                % -- 新协议 --
                elseif startsWith(msg, 'S|')
                    vals = sscanf(msg(3:end), '%d,%f,%d');
                    if length(vals) == 3
                        S.safety_state = vals(1); S.safety_speed = vals(2);
                        S.safety_alarms = vals(3);
                    end
                elseif startsWith(msg, 'L|')
                    vals = sscanf(msg(3:end), '%f,%f,%f,%f,%f,%f');
                    if length(vals) == 6, S.joint_margins = vals'; end
                elseif startsWith(msg, 'G|')
                    vals = sscanf(msg(3:end), '%f,%d');
                    if length(vals) == 2, S.z_dist = vals(1); S.singular = vals(2); end
                elseif startsWith(msg, 'B|')
                    vals = sscanf(msg(3:end), '%d,%f');
                    if length(vals) == 2
                        S.calib_enabled = (vals(1) == 1); S.calib_rms = vals(2);
                    end
                elseif startsWith(msg, 'FB|')
                    S.fb_idx = mod(S.fb_idx, 50) + 1;
                    S.fb_log{S.fb_idx} = msg(4:end);
                elseif startsWith(msg, 'D|')
                    parts = split(msg(3:end), ',');
                    if numel(parts) >= 2
                        S.diag_code = str2double(parts{1});
                        S.diag_spd  = str2double(parts{2});
                        if numel(parts) >= 3, S.diag_reason = strjoin(parts(3:end), ','); end
                    end
                end
            end
        catch ME
            fprintf('[Relay] ERROR in processNetworkData: %s\n', ME.message);
        end
        % 延迟统计
        t = toc(S.last_time);
        if t > 0.5
            S.touch_relay_delay = t * 1000 / max(S.packet_count, 1);
            S.packet_count = 0; S.last_time = tic;
        end
    end

    function update3DModel()
        ja = S.joint_angles;
        % 更新 STL 模型 (hgtransform)
        if stlLoaded
            for i = 0:6
                T = fk.linkTransform(ja(1),ja(2),ja(3),ja(4),ja(5),ja(6), i);
                % hgtransform Matrix is column-major 4x4
                linkHg(i+1).Matrix = T;
            end
        else
            % Fallback: 骨架模型 (复用原 computeFK 逻辑)
            joints = fk.robotFk(ja(1),ja(2),ja(3),ja(4),ja(5),ja(6));
            % NOTE: Fallback path uses delete+redraw per frame.
            % Acceptable for infrequent use; optimize with persistent objects if needed.
            % 清除旧的 fallback 对象 (简化处理: 每帧重绘)
            delete(findobj(ax3d, 'Tag', 'fallback'));
            for i = 1:6
                plot3(ax3d, [joints(i,1) joints(i+1,1)], ...
                           [joints(i,2) joints(i+1,2)], ...
                           [joints(i,3) joints(i+1,3)], ...
                    'Color', [0.25 0.28 0.32], 'LineWidth', 6, 'Tag', 'fallback');
            end
            for i = 2:7
                [sx, sy, sz] = sphere(10);
                r = 6;
                surf(ax3d, sx*r+joints(i,1), sy*r+joints(i,2), sz*r+joints(i,3), ...
                    'FaceColor', [0.30 0.65 1.00], 'EdgeColor', 'none', ...
                    'FaceAlpha', 0.7, 'Tag', 'fallback');
            end
        end

        % Touch 笔可视化
        tp = S.touch_pos;
        if any(tp(1:3) ~= 0)
            set(touchPenBody, 'XData', S.cylX+tp(1), 'YData', S.cylY+tp(2), ...
                'ZData', S.cylZ+tp(3), 'Visible', 'on');
            set(touchPenTip, 'XData', S.sphereX*4+tp(1), 'YData', S.sphereY*4+tp(2), ...
                'ZData', S.sphereZ*4+tp(3), 'Visible', 'on');
        else
            set(touchPenBody, 'Visible', 'off');
            set(touchPenTip, 'Visible', 'off');
        end

        % 末端标记
        rp = S.robot_pos;
        if any(rp(1:3) ~= 0)
            set(eeMarkerActual, 'XData', S.sphere8X*8+rp(1), 'YData', S.sphere8Y*8+rp(2), ...
                'ZData', S.sphere8Z*8+rp(3), 'Visible', 'on');
        else
            set(eeMarkerActual, 'Visible', 'off');
        end
        rt = S.robot_target;
        if any(rt(1:3) ~= 0)
            set(eeMarkerTarget, 'XData', rt(1), 'YData', rt(2), 'ZData', rt(3), 'Visible', 'on');
        else
            set(eeMarkerTarget, 'Visible', 'off');
        end
    end

    function updateTextPanels()
        % -- 指令日志 --
        lines = {};
        for i = 1:50
            idx = mod(S.cmd_idx - i + 50, 50) + 1;
            if ~isempty(S.cmd_log{idx}), lines{end+1} = S.cmd_log{idx}; end
        end
        if isempty(lines), lblCmd.Text = '(waiting for commands...)';
        else, lblCmd.Text = lines; end

        % -- 反馈日志 --
        lines = {};
        for i = 1:50
            idx = mod(S.fb_idx - i + 50, 50) + 1;
            if ~isempty(S.fb_log{idx}), lines{end+1} = S.fb_log{idx}; end
        end
        if isempty(lines), lblFb.Text = '(waiting for feedback...)';
        else, lblFb.Text = lines; end

        % -- 力数据 --
        fr = S.force_raw; ff = S.force_filt; mm = S.force_moment;
        lblForceRaw.Text = {
            sprintf('Raw:  Fx: %7.2f N  Fy: %7.2f N  Fz: %7.2f N', fr(1), fr(2), fr(3));
            sprintf('      Mx: %7.2f Nm My: %7.2f Nm Mz: %7.2f Nm', mm(1), mm(2), mm(3));
            ''};
        lblForceFilt.Text = {
            sprintf('Filt: Fx: %7.2f N  Fy: %7.2f N  Fz: %7.2f N', ff(1), ff(2), ff(3));
            ''};
        if S.force_stale
            lblForceRaw.Text{3} = '*** FORCE SENSOR OFFLINE ***';
            lblForceRaw.FontColor = [1.0 0.3 0.3];
            lblForceFilt.Text{2} = '*** FORCE SENSOR OFFLINE ***';
            lblForceFilt.FontColor = [1.0 0.3 0.3];
        else
            lblForceRaw.FontColor = clr.text_dim;
            lblForceFilt.FontColor = clr.text_dim;
        end

        % -- Robot State --
        rp = S.robot_pos; rt = S.robot_target; ja = S.joint_angles;
        txActive = any(rt(1:3) ~= 0);
        lblCoord.Text = {
            sprintf('Position (mm):    X: %8.2f  (target: %8.2f)', rp(1), rt(1));
            sprintf('                   Y: %8.2f  (target: %8.2f)', rp(2), rt(2));
            sprintf('                   Z: %8.2f  (target: %8.2f)', rp(3), rt(3));
            sprintf('Orientation (deg): Rx: %7.2f  Ry: %7.2f  Rz: %7.2f', rp(4), rp(5), rp(6));
            '';
            sprintf('Joints (deg):  J1:%7.1f  J2:%7.1f  J3:%7.1f', ja(1:3));
            sprintf('               J4:%7.1f  J5:%7.1f  J6:%7.1f', ja(4:6));
            '';
            sprintf('Force (N):   Fx: %7.2f   Fy: %7.2f   Fz: %7.2f', ff(1), ff(2), ff(3));
            '';
            sprintf('TX: %s', ternary(txActive, 'ACTIVE', 'IDLE'))};

        % -- Safety & Diagnostics --
        stateNames = {'RUNNING', 'WARN', 'DEGRADE', 'FATAL'};
        stateColors = {clr.green, clr.yellow, clr.orange, clr.red};
        st = S.safety_state + 1;
        if st < 1, st = 1; elseif st > 4, st = 4; end

        safetyLines = {};
        safetyLines{1} = sprintf('Safety: %s  |  Speed: %.1fx  |  Alarms: %d', ...
            stateNames{st}, S.safety_speed, S.safety_alarms);
        lblSafety.FontColor = stateColors{st};

        % 关节限位
        [minM, worstJ] = min(S.joint_margins);
        if minM < 15
            safetyLines{2} = sprintf('J%d near limit: %.1f deg margin', worstJ, minM);
            if st < 3  % don't downgrade FATAL/DEGRADE red/orange to joint orange
                lblSafety.FontColor = clr.orange;
            end
        else
            safetyLines{2} = sprintf('Joints: OK (min margin %.0f deg)', minM);
        end

        % 奇异位形
        if S.singular
            safetyLines{3} = sprintf('Z-axis dist: %.0f mm  !!SINGULAR!!', S.z_dist);
        else
            safetyLines{3} = sprintf('Z-axis dist: %.0f mm', S.z_dist);
        end

        % 标定
        if S.calib_enabled
            safetyLines{4} = sprintf('Calib: RMS=%.2f mm', S.calib_rms);
        else
            safetyLines{4} = 'Calib: not calibrated';
        end

        % 诊断
        if S.diag_code ~= 0
            safetyLines{5} = sprintf('Last Diag: code=%d speed=%.1f %s', ...
                S.diag_code, S.diag_spd, S.diag_reason);
        else
            safetyLines{5} = 'Diagnostics: (no errors)';
        end

        lblSafety.Text = safetyLines;

        % -- 延迟 + 状态 --
        lblDelay.Text = sprintf('Touch->Relay: %.1f ms', S.touch_relay_delay);
        lblState.Text = sprintf('[%s]  Spd: %.1fx', stateNames{st}, S.safety_speed);
        lblState.FontColor = stateColors{st};
    end

    function r = ternary(cond, tVal, fVal)
        if cond, r = tVal; else, r = fVal; end
    end

end
