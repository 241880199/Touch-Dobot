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

    % ===== 顶部状态栏 (临时占位) =====
    lbl_status = uilabel(fig, 'Position', [10 10 300 20], ...
                         'Text', 'v4.0 skeleton loaded — awaiting Task 6/7 UI panels', ...
                         'FontColor', clr.text_dim, 'FontSize', 10);

    % ===== 等待后续任务添加 UI 面板 =====
    % Task 6: uigridlayout 布局 + 所有面板创建
    % Task 7: 网络/timer/数据更新逻辑

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
