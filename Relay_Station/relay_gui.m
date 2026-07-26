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

    % ===== 加载配置 =====
    cfg = relay_config();

    % ===== 确保 +stl +fk 在路径中 =====
    scriptDir = fileparts(mfilename('fullpath'));
    if ~contains(path, fullfile(scriptDir, '+stl'))
        addpath(scriptDir);
    end

    % ===== 颜色主题 =====
    clr = struct(...
        'bg_dark',  [0.10 0.12 0.16], 'bg_panel', [0.08 0.10 0.14], ...
        'border',   [0.20 0.30 0.50], 'text_on',  [0.90 0.94 1.00], ...
        'text_dim', [0.55 0.58 0.62], 'green',    [0.20 0.80 0.40], ...
        'red',      [0.95 0.25 0.25], 'blue',     [0.30 0.65 1.00], ...
        'orange',   [1.00 0.60 0.15], 'yellow',   [1.00 0.85 0.20]);

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
    pnlTop = uipanel(g, 'BackgroundColor', [0.06 0.08 0.12], 'BorderType', 'none');
    pnlTop.Layout.Row = 1;
    pnlTop.Layout.Column = [1 3];

    glTop = uigridlayout(pnlTop, [1 6]);
    glTop.ColumnWidth = {220, 280, 180, '1x', 150, 120};
    glTop.Padding = [4 1 4 1];
    glTop.BackgroundColor = [0.06 0.08 0.12];

    lblTitle = uilabel(glTop, 'Text', 'Touch-Dobot Relay Station', ...
        'FontColor', clr.blue, 'FontWeight', 'bold', 'FontSize', 11);
    lblDelay = uilabel(glTop, 'Text', 'Touch->Relay: -- ms', ...
        'FontColor', clr.green, 'FontSize', 10);
    lblIp = uilabel(glTop, 'Text', ['Robot IP: ' cfg.robot_ip], ...
        'FontColor', clr.text_dim, 'FontSize', 10);
    lblSpacer = uilabel(glTop, 'Text', '');
    lblState = uilabel(glTop, 'Text', '[--]', 'FontColor', clr.text_dim, ...
        'FontSize', 10, 'HorizontalAlignment', 'right');
    lblConn = uilabel(glTop, 'Text', 'C++ Client: OFFLINE', ...
        'FontColor', clr.red, 'FontSize', 10, 'HorizontalAlignment', 'right');

    % ===== 左栏 (Col 1): 指令 + 反馈 =====
    pnlLeft = uipanel(g, 'BackgroundColor', clr.bg_panel, 'BorderType', 'none');
    pnlLeft.Layout.Row = 2;  pnlLeft.Layout.Column = 1;
    glLeft = uigridlayout(pnlLeft, [2 1]);
    glLeft.RowHeight = {'1x', '1x'};
    glLeft.Padding = [1 1 1 1];  glLeft.RowSpacing = 2;
    glLeft.BackgroundColor = clr.bg_panel;

    % 指令面板
    pnlCmd = uipanel(glLeft, 'BackgroundColor', clr.bg_panel, 'BorderType', 'none');
    pnlCmd.Layout.Row = 1;  pnlCmd.Layout.Column = 1;
    lblCmdTitle = uilabel(pnlCmd, 'Text', 'Touch -> Robot (Commands)', ...
        'Position', [6 0 360 22], 'FontColor', clr.text_on, 'FontSize', 11, 'FontWeight', 'bold');
    lblCmd = uilabel(pnlCmd, 'Text', '(waiting for commands...)', ...
        'Position', [6 -350 360 350], 'FontColor', [0.30 0.85 0.50], ...
        'FontSize', 9, 'VerticalAlignment', 'top', 'FontName', 'Consolas');

    % 反馈面板
    pnlFb = uipanel(glLeft, 'BackgroundColor', clr.bg_panel, 'BorderType', 'none');
    pnlFb.Layout.Row = 2;  pnlFb.Layout.Column = 1;
    lblFbTitle = uilabel(pnlFb, 'Text', 'Robot -> Relay (Feedback)', ...
        'Position', [6 0 360 22], 'FontColor', clr.text_on, 'FontSize', 11, 'FontWeight', 'bold');
    lblFb = uilabel(pnlFb, 'Text', '(waiting for feedback...)', ...
        'Position', [6 -350 360 350], 'FontColor', clr.text_dim, ...
        'FontSize', 9, 'VerticalAlignment', 'top', 'FontName', 'Consolas');

    % ===== 中栏 (Col 2): 力数据 =====
    pnlMid = uipanel(g, 'BackgroundColor', clr.bg_panel, 'BorderType', 'none');
    pnlMid.Layout.Row = 2;  pnlMid.Layout.Column = 2;
    glMid = uigridlayout(pnlMid, [2 1]);
    glMid.RowHeight = {'1x', '1x'};
    glMid.Padding = [1 1 1 1];  glMid.RowSpacing = 2;
    glMid.BackgroundColor = clr.bg_panel;

    % 原始力
    pnlFR = uipanel(glMid, 'BackgroundColor', clr.bg_panel, 'BorderType', 'none');
    pnlFR.Layout.Row = 1;  pnlFR.Layout.Column = 1;
    lblFRTitle = uilabel(pnlFR, 'Text', 'Force Sensor (Raw · 30004)', ...
        'Position', [6 0 360 22], 'FontColor', clr.text_on, 'FontSize', 11, 'FontWeight', 'bold');
    lblForceRaw = uilabel(pnlFR, 'Position', [10 -100 350 120], ...
        'Text', {'Awaiting force sensor data...', '', ...
                 'Fx:   0.00 N   Fy:   0.00 N   Fz:   0.00 N'}, ...
        'FontColor', clr.text_dim, 'FontSize', 10, ...
        'VerticalAlignment', 'top', 'FontName', 'Consolas');

    % 滤波力
    pnlFF = uipanel(glMid, 'BackgroundColor', clr.bg_panel, 'BorderType', 'none');
    pnlFF.Layout.Row = 2;  pnlFF.Layout.Column = 1;
    lblFFTitle = uilabel(pnlFF, 'Text', 'Force Output (Filtered -> Touch)', ...
        'Position', [6 0 360 22], 'FontColor', clr.text_on, 'FontSize', 11, 'FontWeight', 'bold');
    lblForceFilt = uilabel(pnlFF, 'Position', [10 -100 350 120], ...
        'Text', {'Filtered force for haptic feedback...', '', ...
                 'Fx:   0.00 N   Fy:   0.00 N   Fz:   0.00 N'}, ...
        'FontColor', clr.text_dim, 'FontSize', 10, ...
        'VerticalAlignment', 'top', 'FontName', 'Consolas');

    % ===== 右栏 (Col 3): 3D + 状态 + 安全 =====
    pnlRight = uipanel(g, 'BackgroundColor', clr.bg_panel, 'BorderType', 'none');
    pnlRight.Layout.Row = 2;  pnlRight.Layout.Column = 3;
    glRight = uigridlayout(pnlRight, [3 1]);
    glRight.RowHeight = {'3x', '1x', '1x'};
    glRight.Padding = [1 1 1 1];  glRight.RowSpacing = 2;
    glRight.BackgroundColor = clr.bg_panel;

    % 3D 视图
    pnl3D = uipanel(glRight, 'BackgroundColor', clr.bg_panel, 'BorderType', 'none');
    pnl3D.Layout.Row = 1;  pnl3D.Layout.Column = 1;
    ax3d = uiaxes(pnl3D, 'BackgroundColor', [0.12 0.14 0.18], ...
        'XColor', clr.text_dim, 'YColor', clr.text_dim, 'ZColor', clr.text_dim, ...
        'Box', 'on', 'GridLineStyle', ':');
    title(ax3d, 'Digital Twin', 'Color', clr.text_on, 'FontSize', 11);
    xlabel(ax3d, 'X (mm)'); ylabel(ax3d, 'Y (mm)'); zlabel(ax3d, 'Z (mm)');
    hold(ax3d, 'on'); axis(ax3d, 'equal');
    ax3d.View = [45 30];
    xlim(ax3d, [0 500]); ylim(ax3d, [-250 250]); zlim(ax3d, [0 400]);

    % Robot State 面板
    pnlState = uipanel(glRight, 'BackgroundColor', clr.bg_panel, 'BorderType', 'none');
    pnlState.Layout.Row = 2;  pnlState.Layout.Column = 1;
    lblStateTitle = uilabel(pnlState, 'Text', 'Robot State', ...
        'Position', [6 2 600 22], 'FontColor', clr.text_on, 'FontSize', 11, 'FontWeight', 'bold');
    lblCoord = uilabel(pnlState, 'Position', [6 -100 600 125], ...
        'Text', 'Initializing...', 'FontColor', [0.70 0.85 0.50], ...
        'FontSize', 10, 'VerticalAlignment', 'top', 'FontName', 'Consolas');

    % Safety 面板
    pnlSafety = uipanel(glRight, 'BackgroundColor', clr.bg_panel, 'BorderType', 'none');
    pnlSafety.Layout.Row = 3;  pnlSafety.Layout.Column = 1;
    lblSafeTitle = uilabel(pnlSafety, 'Text', 'Safety & Diagnostics', ...
        'Position', [6 2 600 22], 'FontColor', clr.text_on, 'FontSize', 11, 'FontWeight', 'bold');
    lblSafety = uilabel(pnlSafety, 'Position', [6 -100 600 125], ...
        'Text', 'Safety: --', 'FontColor', clr.green, ...
        'FontSize', 10, 'VerticalAlignment', 'top', 'FontName', 'Consolas');

    % ===== 嵌套函数 =====

    function onResize()
        % 窗口大小变化回调 (Task 6 将实现响应式布局调整)
    end

    function onClose()
        S.running = false;
        if ~isempty(S.server) && isvalid(S.server)
            delete(S.server);
        end
        delete(fig);
        disp('[Relay] GUI closed.');
    end
end
