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
    S.robot_pos    = [0 0 0 0 0 0];
    S.robot_target = [0 0 0 0 0 0];
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
    S.warnings = {};          S.warning_count = 0; S.warn_max_level = 0;
    S.conn_enable = 0;       S.conn_motion = 0;   S.conn_force = 0;
    S.conn_ping = 0;         S.conn_uptime = 0;
    % 力历史环形缓冲 (100 点 ≈ 3s @ 30Hz)
    S.force_hist_fx = zeros(1,100); S.force_hist_fy = zeros(1,100);
    S.force_hist_fz = zeros(1,100); S.force_hist_idx = 1;
    % ===== 3D 视野 (2026-09-21) =====
    % 【以【底座】为原点】x/y 中心 = 0 (底座的轴线), z 中心 = 400 (工作高度中段)。
    %   中心定在这里 ⇒ 画面里的坐标原点就是底座, 读数与面板上的 X/Y/Z 是同一个参考系。
    % ⚠ 曾经试过"视野跟着末端走", 现场否掉了: 那样原点会跟着工具跑, 空间参考系就没了。
    % 【三个轴共用一个边长】创建时那句 axis(ax3d,'equal') 要求等比 —— 等边长才是真正的
    %   立方体视野; 不对称的范围会和 axis equal 打架, 把某个轴拉扁。
    % ===== 滚轮放缩开关 (2026-09-21) =====
    % true  = 注册图窗级滚轮回调 (滚轮放大 / 缩小, 见 onScrollZoom)
    % false = 不注册 —— 用工具栏自带的 缩放/平移/旋转 按钮 (它们一直在, 与本开关无关)
    % 【为什么做成开关】现场报告"注册之后拖不动视角"。我在这里查了两件事, 【都是阴性】:
    %   ① ax.Interactions 在注册前后都是 DefaultAxesInteractionSet, 工具栏也还在;
    %   ② 全文件没有"每帧重置视野/视角"的代码 —— ax3d.View 只在创建时设一次,
    %      applyView() 只在【创建时】与【滚轮时】被调。
    %   ⇒ 判不了就别猜, 给现场一个一行的判据: 改成 false 重跑一次 ——
    %     拖动能回来 ⇒ 就是它 (那时我换成别的方式做放缩);
    %     回不来     ⇒ 与它无关, 我从别处查 (那时请把"上一次能拖动是什么时候"告诉我)。
    S.useWheelZoom = false;   % ★ 2026-09-22 现场判定用: 见上面那三行 —— 这一趟跑完按结论定回来
    % ===== 高频遥测【不抽样】落盘开关 (2026-09-24) =====
    % true  = `P| / J| / RP|` 每一条都写进 `_matlab_session_<date>.log`
    % false = 维持"每 10 条记 1 条"（默认；全量约 30+10+10 = 50 条/秒，久了会淹掉文件）
    % 【为什么要这个开关】按钮2 的"笔杆三根轴 ↔ 三个关节"映射必须**按帧率**量:
    %   抽样到 ~2 Hz 之后，段内增量的【轴】会估漂 —— 实测同一动作的两段主方向差 42°
    %   （按 30 Hz 的逐步轴，三条轴两两夹角 74~83°，才是对的）。
    %   这是一次性的标定测量，量完就关回去 ⇒ 做成开关，而不是改掉抽样率。
    % ⚠ 只在【标定那几分钟】打开；打开期间日志不适合人工翻阅。
    % ⚠ 旁路日志本身曾经把界面卡死过（每条一次 fopen/fclose、~39 条/秒）—— 那已修
    %   （持句柄 + 每秒重开 + 同内容去重）。50 条/秒下若界面变卡，先把它关掉再看。
    S.logAllFast = false;
    S.viewCenter  = [0 0 400];   % mm
    % 边长 1400 -> 1000 (2026-09-21 现场: "模型太小了")。等边长 + 中心 z=400 ⇒
    %   x/y ∈ [-500,500], z ∈ [-100,900] —— 纵向仍然盖得住 CR3 总高 795, 而模型在画面里大 1.4 倍。
    % ⚠ 横向 ±500 < 工作半径 620 ⇒ 【伸到最远处时末端会出画面】。这是刻意的取舍:
    %   默认看得清 (现场诉求), 要全局就滚轮缩小或按工具栏的 Restore View。
    S.viewSpan    = 1000;        % mm
    S.viewSpanMin = 200;         % 滚轮放缩的下限/上限 (mm)
    S.viewSpanMax = 4000;
    S.server = [];
    S.ff_enabled = true;      % 力反馈开关状态 (A组=true / B组=false)
    % ===== 力反射增益调参 (2026-09-22) =====
    % 真值【只在 C++ 那一侧】。下面这些字段只是【显示缓存】, 全部由 RG| 回读刷新 ——
    % 绝不当作"我设过什么"的记忆使用 (那正是无声不一致的入口)。
    S.tuning = struct('gain', NaN, 'min', NaN, 'max', NaN, 'ratio', NaN, ...
                      'deadN', NaN, 'satN', NaN, 'defGain', NaN, 'known', false, ...
                      'displayUnknown', false);
    % displayUnknown: 控件上【没有可信的增益数】(回读自相矛盾那一档才为 true)。
    %   只由 showGainUnknown() 置 true、由"赋真值"那两句置 false —— 它描述的是【显示状态】,
    %   不是回读本身, 所以标题行也拿它当判据 (见 updateTextPanels)。
    S.tuningDragging = false;   % 拖动中禁止回读移动滑块 (否则和手指打架)
    S.tuningLastSent = NaN;     % 最近一次发出去的值, 用于判断回读是否与请求不符 (拒收)
    % ===== 断线丢弃的【报数限幅】(2026-09-22) =====
    % 送不出去必须出声 (见 sendToClient), 但 ValueChangingFcn 拖动时每秒几十条 ⇒ 逐条出声
    %   等于刷屏, 而本项目有成文教训: **被刷屏的控制台就是看不见的控制台**
    %   (C++ → MATLAB 那条回读方向当初正是为这个才做了限频)。
    % 这一对字段就是那个限幅的状态: 报过【第一声】之后同类命令只计数, 等连接状态一变再补报条数。
    S.dropNotifiedKey = '';     % 已经报过"丢弃"的那【一类】命令 ('|' 前面那一段: RG / Z / FF)
    S.dropQuiet       = struct();  % 类别 -> 被【压下去、没有逐条报】的条数 (等连接状态一变补报)
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
    g.ColumnWidth = {'1x', '1.2x', '1.6x'}; % 左:中:右 = 1:1.2:1.6
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

    % ===== 左栏 (Col 1): 指令日志 (铺满全高) =====
    pnlLeft = uipanel(g, 'BackgroundColor', clr.bg_panel, 'BorderType', 'none');
    pnlLeft.Layout.Row = 2;  pnlLeft.Layout.Column = 1;
    glLeft = uigridlayout(pnlLeft, [2 1]);
    glLeft.RowHeight = {22, '1x'};
    glLeft.Padding = [4 0 4 2];  glLeft.RowSpacing = 0;
    glLeft.BackgroundColor = clr.bg_panel;

    lblCmdTitle = uilabel(glLeft, 'Text', 'Touch -> Robot (Commands)', ...
        'FontColor', clr.text_on, 'FontSize', 11, 'FontWeight', 'bold');
    lblCmdTitle.Layout.Row = 1;  lblCmdTitle.Layout.Column = 1;

    lblCmd = uilabel(glLeft, 'Text', '(waiting for commands...)', ...
        'FontColor', [0.30 0.85 0.50], 'FontSize', 9, ...
        'VerticalAlignment', 'top', 'FontName', 'Consolas');
    lblCmd.Layout.Row = 2;  lblCmd.Layout.Column = 1;

    % ===== 中栏 (Col 2): 连接状态 + 力数据 + 力历史 =====
    pnlMid = uipanel(g, 'BackgroundColor', clr.bg_panel, 'BorderType', 'none');
    pnlMid.Layout.Row = 2;  pnlMid.Layout.Column = 2;
    glMid = uigridlayout(pnlMid, [4 1]);
    glMid.RowHeight = {55, '1x', '1x', '1.2x'};
    glMid.Padding = [1 1 1 1];  glMid.RowSpacing = 2;
    glMid.BackgroundColor = clr.bg_panel;

    % -- Row 1: Connection Monitor --
    pnlConn = uigridlayout(glMid, [1 1]);
    pnlConn.Padding = [4 2 4 2];
    pnlConn.BackgroundColor = clr.bg_panel;
    pnlConn.Layout.Row = 1;  pnlConn.Layout.Column = 1;
    lblConnMon = uilabel(pnlConn, 'Text', '● Enable  ● Motion  ● Force   PING -- ms', ...
        'FontColor', clr.text_dim, 'FontSize', 10, ...
        'VerticalAlignment', 'center', 'FontName', 'Consolas');
    lblConnMon.Layout.Row = 1;  lblConnMon.Layout.Column = 1;

    % -- Row 2: 原始力 --
    pnlFR = uigridlayout(glMid, [2 1]);
    pnlFR.RowHeight = {22, '1x'};
    pnlFR.Padding = [4 0 4 2];  pnlFR.RowSpacing = 0;
    pnlFR.BackgroundColor = clr.bg_panel;
    pnlFR.Layout.Row = 2;  pnlFR.Layout.Column = 1;

    lblFRTitle = uilabel(pnlFR, 'Text', 'Force Sensor (Raw · 30004)', ...
        'FontColor', clr.text_on, 'FontSize', 11, 'FontWeight', 'bold');
    lblFRTitle.Layout.Row = 1;  lblFRTitle.Layout.Column = 1;

    lblForceRaw = uilabel(pnlFR, 'Text', {'Awaiting force sensor data...', '', ...
        'Fx:   0.00 N   Fy:   0.00 N   Fz:   0.00 N'}, ...
        'FontColor', clr.text_dim, 'FontSize', 10, ...
        'VerticalAlignment', 'top', 'FontName', 'Consolas');
    lblForceRaw.Layout.Row = 2;  lblForceRaw.Layout.Column = 1;

    % -- Row 3: 滤波力 --
    pnlFF = uigridlayout(glMid, [4 1]);
    pnlFF.RowHeight = {22, 26, 40, '1x'};
    pnlFF.Padding = [4 0 4 2];  pnlFF.RowSpacing = 0;
    pnlFF.BackgroundColor = clr.bg_panel;
    pnlFF.Layout.Row = 3;  pnlFF.Layout.Column = 1;

    lblFFTitle = uilabel(pnlFF, 'Text', 'Force Output (Filtered -> Touch)', ...
        'FontColor', clr.text_on, 'FontSize', 11, 'FontWeight', 'bold');
    lblFFTitle.Layout.Row = 1;  lblFFTitle.Layout.Column = 1;

    % 力反馈开关 (A/B 对照实验)
    pnlFFToggle = uigridlayout(pnlFF, [1 2]);
    pnlFFToggle.ColumnWidth = {'1x', 60};
    pnlFFToggle.Padding = [0 0 0 0];  pnlFFToggle.RowSpacing = 0;  pnlFFToggle.ColumnSpacing = 4;
    pnlFFToggle.BackgroundColor = clr.bg_panel;
    pnlFFToggle.Layout.Row = 2;  pnlFFToggle.Layout.Column = 1;

    lblFFToggle = uilabel(pnlFFToggle, 'Text', 'Force Feedback (A/B switch)', ...
        'FontColor', clr.text_dim, 'FontSize', 9);
    lblFFToggle.Layout.Row = 1;  lblFFToggle.Layout.Column = 1;

    % ★ 这里是 @(~,~) onForceFeedbackToggle(swFF.Value) —— 【既有缺陷】, 与本分支的增益调参无关,
    %   是在这个文件里干活时【顺手发现】的 (在分支基线上就能重现)。匿名函数在【赋值完成之前】
    %   就引用了 swFF, 而 MATLAB 的匿名函数按【创建时刻】捕获变量的值 ⇒ 这个句柄从未捕获到
    %   那个开关, 一扳开关就报"函数或变量 'swFF' 无法识别"。
    % 后果不只是"报了个错": uiswitch 的 Value 【自己会翻】(外观变化与回调成败无关), 而
    %   FF|0 / FF|1 【一条都没发出去】⇒ 屏幕上的开关位置与 C++ 那侧的真实状态不一致 ——
    %   正是本分支整趟在消灭的那类不一致。
    % ⇒ 改成从【回调的源参数】取值 (s.Value), 与下面滑条/编辑框那几处同一种写法,
    %   不再捕获外部变量。uiswitch 的 Value 是 char, 内容就是 Items 里的 'ON'/'OFF',
    %   正是 onForceFeedbackToggle 期望的入参。
    swFF = uiswitch(pnlFFToggle, 'Items', {'OFF', 'ON'}, 'Value', 'ON', ...
        'ValueChangedFcn', @(s,~) onForceFeedbackToggle(s.Value));
    swFF.Layout.Row = 1;  swFF.Layout.Column = 2;

    lblForceFilt = uilabel(pnlFF, 'Text', {'Filtered force for haptic feedback...', '', ...
        'Fx:   0.00 N   Fy:   0.00 N   Fz:   0.00 N'}, ...
        'FontColor', clr.text_dim, 'FontSize', 10, ...
        'VerticalAlignment', 'top', 'FontName', 'Consolas');
    lblForceFilt.Layout.Row = 4;  lblForceFilt.Layout.Column = 1;

    % -- 力反射增益调参 (2026-09-22) --
    % 标题行显示【响应窗口】而不是单个饱和点 —— 只给上沿会把死区那个前提藏起来:
    %   gain 300 时窗口是 0.20–0.67N, 死区占了 30%, 可区分的只剩一条缝。
    % 窗口的两个数与比例全部来自 C++ 的 RG| 回读, 本文件【一个魔数都不写】。
    pnlGain = uigridlayout(pnlFF, [2 1]);
    pnlGain.RowHeight = {15, 25};
    pnlGain.Padding = [0 0 0 0];  pnlGain.RowSpacing = 0;
    pnlGain.BackgroundColor = clr.bg_panel;
    pnlGain.Layout.Row = 3;  pnlGain.Layout.Column = 1;

    lblGainTitle = uilabel(pnlGain, 'Text', 'Reflection Gain — 等待 C++…', ...
        'FontColor', clr.text_dim, 'FontSize', 9, 'FontName', 'Consolas');
    lblGainTitle.Layout.Row = 1;  lblGainTitle.Layout.Column = 1;

    pnlGainCtl = uigridlayout(pnlGain, [1 4]);
    pnlGainCtl.ColumnWidth = {'1x', 58, 58, 44};
    pnlGainCtl.Padding = [0 0 0 0];  pnlGainCtl.RowSpacing = 0;  pnlGainCtl.ColumnSpacing = 3;
    pnlGainCtl.BackgroundColor = clr.bg_panel;
    pnlGainCtl.Layout.Row = 2;  pnlGainCtl.Layout.Column = 1;

    % ★ 回读之前【屏幕上不许出现任何看着像增益的数】(2026-09-22 复审 Fix 2)。
    %   原来这里写的是 'Value',120 + Limits [100 300] ⇒ 数值框明明读着 120, 而对面 C++ 可能
    %   正跑在 force_tuning.json 的 200 上 —— 那是唯一一处"屏幕上的数不是正在用的数"。
    %   ★ 实测 (R2025b, 本机): 数值框与滑条【都拒 NaN】—— 构造与赋值都报
    %     ''Value' 必须为位于 'Limits' 的范围之内的双精度标量' ⇒ 提示里那条 NaN 方案不成立。
    %   ⇒ 改用 numeric 编辑框的公开属性 AllowEmpty: 空值 + Placeholder 显示的是一句话, 不是数。
    %     实测空值时框里是 '— 等待 C++ —'; 第一次回读时 AllowEmpty 置 false 并给真值。
    % ⚠ 这里【一个占位魔数都不写】: 编辑框空着 (Limits 取默认 [-Inf Inf]), 滑条用 MATLAB 自己的
    %   默认 Limits [0 100] / Value 0, 且明确把 MajorTickLabels 置空 ⇒ 滑条上也没有数字
    %   (实测 R2025b: 默认 MajorTicks 是 [0 20 40 60 80 100], 而【画出来】的自动标签是
    %    0 4 8 12 … 96 100 一整排 —— 以出图为准, 那又是一排假数)。
    %   占位范围只在"控件禁用、发不出去"这一档存在; 第一次 RG| 回读就用 C++ 的 min/max 顶掉它。
    sldGain = uislider(pnlGainCtl, 'Enable', 'off', 'MajorTickLabels', {}, ...
        'ValueChangingFcn', @(s,e) onGainChanging(e.Value), ...
        'ValueChangedFcn',  @(s,e) onGainChanged(e.Value));
    sldGain.Layout.Row = 1;  sldGain.Layout.Column = 1;

    edGain = uieditfield(pnlGainCtl, 'numeric', 'Value', [], 'AllowEmpty', true, ...
        'Placeholder', '— 等待 C++ —', 'Enable', 'off', ...
        'FontName', 'Consolas', 'FontSize', 10, ...
        'ValueChangedFcn', @(s,e) onGainChanged(e.Value));
    edGain.Layout.Row = 1;  edGain.Layout.Column = 2;

    btnGainDefault = uibutton(pnlGainCtl, 'Text', 'Default', 'FontSize', 9, ...
        'Enable', 'off', 'ButtonPushedFcn', @(~,~) onGainDefault());
    btnGainDefault.Layout.Row = 1;  btnGainDefault.Layout.Column = 3;

    % Zero 与键盘 'z' 【同语义】: 再按一次 = 中止。不在 GUI 里发明第二种语义。
    btnZero = uibutton(pnlGainCtl, 'Text', 'Zero', 'FontSize', 9, ...
        'ButtonPushedFcn', @(~,~) onZeroPressed());
    btnZero.Layout.Row = 1;  btnZero.Layout.Column = 4;

    % -- Row 4: 力历史迷你图 --
    pnlFH = uigridlayout(glMid, [2 1]);
    pnlFH.RowHeight = {22, '1x'};
    pnlFH.Padding = [4 0 4 2];  pnlFH.RowSpacing = 0;
    pnlFH.BackgroundColor = clr.bg_panel;
    pnlFH.Layout.Row = 4;  pnlFH.Layout.Column = 1;

    lblFHTitle = uilabel(pnlFH, 'Text', 'Force History (3s window)', ...
        'FontColor', clr.text_on, 'FontSize', 11, 'FontWeight', 'bold');
    lblFHTitle.Layout.Row = 1;  lblFHTitle.Layout.Column = 1;

    axForceHist = uiaxes(pnlFH, 'BackgroundColor', clr.bg_axes3d, ...
        'XColor', clr.text_dim, 'YColor', clr.text_dim, ...
        'Box', 'on', 'GridLineStyle', ':');
    axForceHist.Layout.Row = 2;  axForceHist.Layout.Column = 1;
    hold(axForceHist, 'on');
    xlim(axForceHist, [0 100]); ylim(axForceHist, [-15 15]);
    axForceHist.XTick = [0 50 100]; axForceHist.XTickLabel = {'3s', '1.5s', '0s'};
    ylabel(axForceHist, 'N');

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
    % 视野与放缩 (2026-09-21): 范围从此由 S.viewCenter / S.viewSpan 一处给出, 见 applyView。
    %   从前那三句固定范围 (x[-350,400] y[-400,400] z[-50,800]) 与 axis equal 是打架的,
    %   而且末端一出盒子就看不见 —— 现场读成"孪生卡住了"。
    applyView();
    if S.useWheelZoom
        fig.WindowScrollWheelFcn = @(~, evt) onScrollZoom(evt);
    end

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

    % ===== 安全边界黄框【已删】(2026-09-21, 用户要求) =====
    % 原来这里有 12 条黄虚线画出 cfg.safe_x/y/z 那个盒子。删它的理由不是"不好看":
    %   **那个盒子【已经不生效了】** —— 客户端那边工作空间钳位整个关掉了
    %   (Config::SAFETY_BOUNDARY_CLAMP_ENABLED = false, 目的是让笔能够落到纸面高度)。
    %   继续画它会让操作员以为机械臂还受它约束 —— 那正是本项目最忌的"画面说的事和代码做的不一样"。
    % ⚠ 恢复: 从 git 历史取回这 12 行即可; 但【先确认钳位真的重新打开了】, 否则画出来的是假约束。
    %   (cfg.safe_* 目前在本文件里没有别的使用者。)

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

    % Force history lines (direct line objects for performance)
    alFx = line(axForceHist, 1:100, zeros(1,100), ...
        'Color', [1.0 0.35 0.35], 'LineWidth', 1.2);
    alFy = line(axForceHist, 1:100, zeros(1,100), ...
        'Color', [0.35 0.95 0.45], 'LineWidth', 1.2);
    alFz = line(axForceHist, 1:100, zeros(1,100), ...
        'Color', [0.35 0.55 1.0], 'LineWidth', 1.2);
    S.alFx = alFx;  S.alFy = alFy;  S.alFz = alFz;
    S.axForceHist = axForceHist;

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
        % 连接状态一变就是"上一段断线到此为止": 把被压下的丢弃条数补报出来并复位
        % (无事时空操作)。⇒ 重连之后的第一条命令【一定】会重新出声, 不会被上一段压掉。
        flushDropNotice();
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

    function onForceFeedbackToggle(newVal)
        S.ff_enabled = strcmp(newVal, 'ON');
        if S.ff_enabled
            sendToClient('FF|1');
        else
            sendToClient('FF|0');
        end
    end

    % ===== 力反射增益 (2026-09-22) =====
    % 真值在 C++。这里只做两件事: 把用户意图发过去、把回读显示出来。

    function onGainChanging(v)
        % 拖动中: 实时下发 —— 拖的过程手上就能感觉到 (C++ 那道 0.25s 斜坡把它摊平)
        S.tuningDragging = true;
        S.tuningLastSent = v;
        sendToClient(sprintf('RG|%.4f', v));
    end

    function onGainChanged(v)
        % 松手 / 编辑框提交: 再发一次。幂等, 保证最后一条一定到
        % (拖动中被 C++ 限频挡下的那条, 由它的补发机制兜住)。
        S.tuningDragging = false;
        S.tuningLastSent = v;
        sendToClient(sprintf('RG|%.4f', v));
    end

    function onGainDefault()
        % ★ 【不硬编码 120】—— 用 C++ 回读里的 defGain。
        %   硬编码的话, Config.h 的默认值一改, 这个按钮就与"默认"无关了 ——
        %   而它做的正是"送回默认值"这件事。
        if ~S.tuning.known, return; end
        S.tuningDragging = false;
        S.tuningLastSent = S.tuning.defGain;
        sendToClient(sprintf('RG|%.4f', S.tuning.defGain));
    end

    function onZeroPressed()
        % 与键盘 'z' 同语义 (C++ 那边负责"再按一次 = 中止")
        % 旁路日志: 与键盘 'z' 区分开 —— 两者发出去的都是 'Z|1', 但一条来自这个按钮、
        %   一条来自键盘; 判"按钮到底有没有接上"时, 日志必须能分辨是哪一个动作。
        tlog('UI', 'Zero button pressed');
        sendToClient('Z|1');
    end

    function showGainUnknown()
        % 回读自相矛盾时【唯一诚实的显示】: 喊一声 + 把两个控件摆成"没有数"这一档。
        %   (2026-09-22 Fix round 2。判据与守门见 processNetworkData 里跳赋值的那一支。)
        % 【为什么不能用 NaN】实测 (R2025b, 本机): 数值框与滑条【都拒】NaN —— 构造与赋值都报
        %   ''Value' 必须为位于 'Limits' 的范围之内的双精度标量'。
        % 数值框: AllowEmpty=true + Value=[] 就是"没有数"这一档, 占位串顶上去 ——
        %   与"第一次回读之前"用的是同一个机制, 不是这里新发明的一种状态。
        % 滑条: 【没有空值这一档】⇒ 用 Enable='off', 刻度标签本来就已置空 (MajorTickLabels={})。
        %   出图核对过: 关掉之后【一个数字都没有】, 只剩变灰的轨道与手柄。
        % ⚠ 先关 Enable 再清编辑框, 顺序不能反: 置 AllowEmpty=true 会把"交出空值"这条路重新
        %   打开, 而 onGainChanged 收到 [] 会发出 sprintf('RG|%.4f', []) = 'RG|' —— 一条畸形
        %   命令 (实测就是 'RG|')。关掉 Enable ⇒ 操作员根本交不出空值。
        % ⚠ 那句 ⚠ 日志【写在本函数里】不是随手放的: 这样任何一处调用都会喊, 不会有"清了屏
        %   幕却没人说话"的调用方式。
        fprintf(['[Relay] ⚠ 回读自相矛盾: 回读的当前增益 %.1f 落在它自己声明的范围 ' ...
                 '[%.0f, %.0f] 之外 —— 增益已显示为【未知】(控件清空+禁用), ' ...
                 '不留上一条回读的旧数\n'], ...
            S.tuning.gain, S.tuning.min, S.tuning.max);
        edGain.Enable      = 'off';
        sldGain.Enable     = 'off';
        edGain.AllowEmpty  = true;
        edGain.Value       = [];
        % 占位串【短】是量出来的, 不是随手写的: 这一格是 58px 宽 (pnlGainCtl 的 ColumnWidth),
        %   实测 '— 回读矛盾 · 增益未知 —' 会被截断成 '— 回读矛盾', 而 '— 未知 —' 完整显示。
        %   细节留给上面那行标题去说 (宽度够, 整句都在), 这一格只回答"有没有数"。
        edGain.Placeholder = '— 未知 —';
        S.tuning.displayUnknown = true;
    end

    % ===== 旁路日志 (2026-09-23): 只写文件, 不参与任何界面逻辑 =====
    % 【为什么加】判读"阻力映射系数可调"那几项时, MATLAB 侧(界面 / 命令窗)与 C++ 侧(控制台)
    %   是【两条互不可见的通道】—— 只能靠人在两边来回看、再口头对齐, 而口头对齐正是本项目
    %   反复出错的环节。这一路把两侧的协议流量落到【同一个文件】, 事后可以逐条对齐。
    % 【只记低频】P| / F| / J| / RP| 是 123 Hz 级遥测 —— 记了会把文件淹掉, 而且它们每一拍
    %   都在变、与本功能无关; 真正要判读的是 RG| / C| / Z| / FF| 这一类: 由操作员动作触发、
    %   频率低。高频那四路在【调用处】就滤掉了(名单只有一份, 就在那一个 if 里)。
    % ⚠ 整个函数套 try/catch: 日志【永远不许】打断收包循环或发送路径。本项目对"静默"极敏感,
    %   但这里是反过来的 —— 日志可以静默失败, 界面不许挂。
    % ⚠ 每次调用各自 fopen/fclose(追加): 不需要持句柄、也就不用判句柄失效; 而且 MATLAB 被
    %   直接关掉时不会丢掉缓冲区里的最后几行。调用频率(几十条/秒上限)下这点开销无感。
    function tlog(tag, text)
        persistent log_path fid pendTag pendText pendN t0
        try
            if isempty(log_path)
                log_path = fullfile(fileparts(mfilename('fullpath')), ...
                    ['_matlab_session_' char(datetime('now', 'Format', 'yyyyMMdd')) '.log']);
                fid = -1; pendTag = ''; pendText = ''; pendN = 0;
                t0 = tic;   % 刷新节拍的计时器：用【带句柄的 tic/toc】而不是 now ——
                            %   后者会被 lint 判为"建议改用 datetime"，而 datetime('now')
                            %   每条消息构造一次太贵，正好与本次"降开销"的目的相反。
                            %   toc(t0) 是显式句柄形式，不受别处的 tic 影响。
            end
            % ===== ★★ 2026-09-23 重写：从"每条消息 fopen/fclose 一次"改成"持句柄 + 每秒重开一次"
            %   【为什么必须改】现场报告"按住按钮时 MATLAB 界面卡死、松手才恢复"。机制：
            %   本函数跑在【收包回调】里，而那条线程正是服务界面的那条 ⇒ 每条一次 open/close
            %   实测达 【~39 条/秒 × 3 次文件操作 ≈ 上百次 I/O/秒】，足以把界面饿死；而按钮一松、
            %   C|ServoP 那一路停了、消息率骤降 ⇒ 界面就"活了"。
            %   ⚠ 这是【测量工具扰动了被测对象】—— 本项目的老教训（09-21 那次"仪器本身出错"）；
            %     所以改法是降低开销，而不是"少测点"。
            %   【为什么不能只持句柄】实测（2026-09-23）：fseek(fid,0,'cof') 【不刷新】——
            %   MATLAB 还在跑时另一个进程读到的是空文件 ⇒ 我的实时读取会失效。
            %   ⇒ 折中：持句柄写（便宜），但【每秒 close+reopen 一次】把数据推出去，
            %     我的读取最多滞后 1 秒。
            % ===== 另一处等量开销：同内容去重 =====
            %   客户端有一路 W| 是【设计上 30 Hz 重发】的（RelayCore.cpp "持续重发 MATLAB 警告"），
            %   实测 21 条/秒，而每一拍的文本【完全相同】—— 信息量 0，却要付全部 I/O 代价。
            %   G|/S| 这类状态量在不变时同理。⇒ 连续同内容【只写第一行】，之后只计数，
            %   等它变了（或该标签换了）再补一行 "(xN 次相同)"。信息不丢，量掉一个数量级。
            if fid < 0
                fid = fopen(log_path, 'a');
                if fid < 0
                    return;
                end
            end
            same = strcmp(tag, pendTag) && strcmp(text, pendText);
            if same
                pendN = pendN + 1;
            else
                if pendN > 1
                    fprintf(fid, '%s %-5s ···· 上一行内容连续相同, 另有 %d 次 (已省略)\n', ...
                        char(datetime('now', 'Format', 'HH:mm:ss.SSS')), 'REP', pendN - 1);
                end
                fprintf(fid, '%s %-5s %s\n', ...
                    char(datetime('now', 'Format', 'HH:mm:ss.SSS')), tag, text);
                pendTag = tag; pendText = text; pendN = 1;
            end
            % 每秒把数据推出去（close 才会真落盘 —— fseek 不刷，已实测）
            if toc(t0) >= 1.0
                if pendN > 1
                    fprintf(fid, '%s %-5s ···· 上一行内容连续相同, 另有 %d 次 (已省略)\n', ...
                        char(datetime('now', 'Format', 'HH:mm:ss.SSS')), 'REP', pendN - 1);
                    pendN = 1;
                end
                fclose(fid); fid = -1;
                t0 = tic;
            end
        catch
            % 刻意静默 —— 理由见本函数开头的注释
        end
    end

    function sendToClient(cmd)
        % ⚠ 送不出去【必须出声】(2026-09-22 终审 Fix 1)。
        %   从前的形状是一句无声的 return。于是 C++ 断线时操作员拖滑条 / 按 [Zero] /
        %   扳 swFF, 屏幕上的意图照旧动, 而【一条都没发出去】—— 这是本功能的中心承诺
        %   ("屏幕上的增益 = 机械臂在用的增益")唯一会失效的状态, 也是计划里那条
        %   "不许静默"的 Global Constraint 唯一被破的地方。
        % 【为什么不改成"断线就把控件禁用掉"】想过, 而且【刻意不做】:
        %   打开这三个控件的只有【两处】—— processNetworkData 里
        %   `~wasKnown || rangeChanged` 那一支, 以及"从【回读自相矛盾】状态恢复"那一支
        %   (它自己带着 `if S.tuning.displayUnknown` 这道门)。
        %   断线时禁用【不会】惊动其中任何一个: C++ 的范围若没在这期间变过, 重连后
        %   到达的那条回读与上次同范围 (第一道不成立); 而 displayUnknown 仍是 false
        %   —— "断线"这件事本身不会把它置 true (第二道也不成立) ⇒ 滑条 / 数值框 /
        %   Default 永远灰着, 操作员再也动不了, 而屏幕上【没有一句话】说的是这件事
        %   (控件为什么是灰的)。那正是本函数要消灭的那种静默。
        %   ⇒ 出声 + 让人能原样重试, 比一个漂亮但会卡死的灰控件诚实。
        if ~isempty(S.server) && isvalid(S.server) && S.server.Connected
            try
                write(S.server, uint8([cmd newline]), 'uint8');
                tlog('OUT', cmd);
                flushDropNotice();   % 真的送出去了 ⇒ 断线那一段到此为止 (无事时是空操作)
            catch e
                fprintf('[Relay] ERROR sending to client: %s\n', e.message);
                tlog('OUTFAIL', sprintf('%s  (%s)', cmd, e.message));
            end
        else
            tlog('DROP', cmd);
            notifyDropped(cmd);
        end
    end

    % ===== 断线丢弃: 出声一次 + 按类别限幅 (2026-09-22) =====
    % 机制: 每一类命令在【一段断线】里报【第一声】之后就不再逐条出声, 只累加条数;
    %   连接状态一变 (真的发出去一条, 或连接断开/重连) 就把每类的条数补报出来并复位。
    %   ⇒ 一次拖动 (不论多长、多少个回调) = 恒定 1 行; 补报最多每类 1 行 (本协议只有 RG/Z/FF)。
    % 为什么不选"每隔 N 秒重复报一次": 那仍随拖动【时长】线性增长 —— 拖 10 秒就是十几行,
    %   而本机制与拖动时长、回调频率【都无关】。
    % 为什么按【类别】分: 拖动 (RG|) / [Zero] (Z|) / swFF (FF|) 是操作员三个独立的意图,
    %   压掉其中任何一类都是新的静默 ⇒ 换类别必须重新出声。
    % ⚠ 计数【不按类别分别清零】: 换类别只是让新类别出声, 旧类别的条数留着等补报 ——
    %   否则"恢复时补报条数"这句承诺在"拖完再按 Zero"这种次序下就成了一句空话。

    function key = cmdKey(cmd)
        % 类别 = '|' 前面那一段 (FF / RG / Z)。本协议的命令都带 '|';
        %   万一没有, 整条当类别 —— 只是分得粗一点, 不影响"每类只出声一次"。
        % makeValidName: 这个 key 要当 S.dropQuiet 的字段名, 非法字符会让【报数】这一步抛错,
        %   而报数正是"不许静默"的落点 ⇒ 宁可把类别名规整一下, 也不让那条路有抛错的形状。
        key = cmd;
        p = find(cmd == '|', 1);
        if ~isempty(p), key = cmd(1:p-1); end
        key = matlab.lang.makeValidName(key);
    end

    function notifyDropped(cmd)
        key = cmdKey(cmd);
        if ~strcmp(key, S.dropNotifiedKey)
            fprintf(['[Relay] ⚠ 未发送 (C++ 客户端未连接), 该命令已【丢弃】: %s ' ...
                     '—— 连接恢复后请重做一次\n' ...
                     '[Relay]    (断线期间同类命令不再逐条刷屏, 恢复时补报条数)\n'], cmd);
            S.dropNotifiedKey = key;
        else
            if ~isfield(S.dropQuiet, key), S.dropQuiet.(key) = 0; end
            S.dropQuiet.(key) = S.dropQuiet.(key) + 1;
        end
    end

    function flushDropNotice()
        % 连接状态变了 (或真发出去了一条): 该把【被压下去的条数】补报出来, 然后复位。
        % ⚠ 【不补发】—— 一条都没缓存: 这句日志说的是"要重做一次", 不是"稍后会自动补上"。
        if isempty(S.dropNotifiedKey), return; end
        keys = fieldnames(S.dropQuiet);
        for k = 1:numel(keys)
            n = S.dropQuiet.(keys{k});
            if n > 0
                fprintf(['[Relay] 断线期间另有 %d 条 %s 类命令被【丢弃】(未逐条刷屏), ' ...
                         '没有缓存、不会补发 —— 需要重做一次\n'], n, keys{k});
            end
        end
        S.dropNotifiedKey = '';
        S.dropQuiet       = struct();
    end

    % ===== 主更新循环 =====
    function updateDisplay()
        if ~S.running, return; end
        try
            processNetworkData();
            update3DModel();
            updateTextPanels();
            updateForceHistory();
            drawnow limitrate;
        catch ME
            fprintf('[Relay] ERROR in updateDisplay: %s\n', ME.message);
        end
    end

    function processNetworkData()
        % 抽样计数（见下面旁路日志那一处）：persistent 必须声明在【函数顶层】——
        %   放进循环体里会被 lint 报 "PERSISTENT 可能会非常低效"（实测 2026-09-23）。
        persistent fastN
        if isempty(fastN), fastN = 0; end

        if isempty(S.server) || ~isvalid(S.server) || S.server.NumBytesAvailable == 0
            return;
        end
        try
            while S.server.NumBytesAvailable > 0
                raw = readline(S.server);
                if isempty(raw) || ismissing(raw), continue; end
                if isstring(raw), raw = char(raw); end
                if ~ischar(raw), continue; end
                % Decode UTF-8 from C++ (/utf-8 flag) to MATLAB Unicode
                raw = native2unicode(uint8(raw), 'UTF-8');
                msg = strtrim(raw);
                if isempty(msg), continue; end

                S.packet_count = S.packet_count + 1;

                % 旁路日志: 低频协议全记；高频遥测【抽样】记。
                %   ★ 2026-09-23 改：从前是"P|/F|/J|/RP| 一律不记"，结果是**日志回答不了
                %   "这几路到底有没有到"** —— 而当天要判的恰恰是这个（RP| 的解析 bug 就是这样
                %   找到的：C++ 侧确证发了，只能从 MATLAB 侧查接收）。
                %   ⇒ 现在除 F|（纯力遥测，与本功能无关）外，其余三路每 10 条记 1 条。
                %     抽样率足够看出"有没有到、值在不在动"，又不会淹掉文件。
                %   （P| 30 Hz、J|/RP| 各 10 Hz ⇒ 抽样后约 3 + 1 + 1 条/秒。）
                %   ★ 2026-09-24: `S.logAllFast = true` 时【不抽样】，三路每条都记 ——
                %     按帧率量按钮2 的笔杆轴映射要用它（见文件顶部那个开关的说明）。
                if ~any(startsWith(msg, {'P|', 'F|', 'J|', 'RP|'}))
                    tlog('IN', msg);
                elseif ~startsWith(msg, 'F|')
                    fastN = fastN + 1;
                    if S.logAllFast || mod(fastN, 10) == 1
                        tlog('IN-S', msg);
                    end
                end

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
                        % Fill force history ring buffer
                        S.force_hist_fx(S.force_hist_idx) = vals(1);
                        S.force_hist_fy(S.force_hist_idx) = vals(2);
                        S.force_hist_fz(S.force_hist_idx) = vals(3);
                        S.force_hist_idx = mod(S.force_hist_idx, 100) + 1;
                    end
                elseif startsWith(msg, 'J|')
                    vals = sscanf(msg(3:end), '%f,%f,%f,%f,%f,%f');
                    if length(vals) == 6, S.joint_angles = vals'; end
                elseif startsWith(msg, 'RP|')
                    % ★★★ 2026-09-23 修 —— 这里从前是 msg(3:end)，**一个字符的错，静默了整条链**。
                    %   'RP|' 与 'RG|' 一样是【三个】字符 ⇒ 载荷从第 4 个字符起。
                    %   msg(3:end) 的开头是 '|'，sscanf('%f,...') 撞上非数字【立刻返回空】
                    %   ⇒ length(vals)==6 永远为假 ⇒ S.robot_pos 一次都没被赋过值。
                    %   【现场症状(2026-09-23)】操作员报告"面板上 orientation 恒为 0" ——
                    %   而 C++ 侧是好的（同一份 robotActualPose 既驱动姿态控制、又发给 RP|，
                    %   且 `pos=`(GetPose) 与 `tcp=`(30004 帧) 实测逐位相同）。
                    %   【代价】两处，都静默：
                    %     · 面板 Position / Orientation 两行恒 0（来自同一个 S.robot_pos）；
                    %     · 3D 里那个 eeMarkerActual（实际位置球标）被 `if any(rp(1:3)~=0)` 挡住
                    %       ⇒ 【从来没显示过】。
                    %   证据：headless 实测 `sscanf('RP|12.34,...'(3:end))` 长度 0、
                    %   `(4:end)` 长度 6。同文件 RG| 那一段早就踩过并写下了警告，本支漏修。
                    vals = sscanf(msg(4:end), '%f,%f,%f,%f,%f,%f');
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
                elseif startsWith(msg, 'RG|')
                    % ★ 七个字段【按位置】解析 —— 线上没有字段名, 错一位就整排错。
                    %   前缀 'RG|' 是【三个】字符 ⇒ 值从第 4 个字符起 (msg(4:end))。
                    %   写成 msg(3:end) 会让第一个字段变成 '|120' ⇒ NaN, 整排跟着错位。
                    vals = str2double(split(msg(4:end), ','));
                    % 可用 = 字段数够 且 全是数 且 上下限真的构成一个区间。
                    %   最后一条不能省: Limits 反了 uislider 会直接抛错, 而那会把整个
                    %   收包循环打断 (外层 try 兜住 ⇒ 那一拍的所有消息都丢)。
                    if numel(vals) >= 7 && all(~isnan(vals)) && vals(2) < vals(3)
                        wasKnown = S.tuning.known;
                        % ★ 覆盖之前先留一份旧范围 —— 下面要靠它判断"声明变了没有"。
                        prevMin  = S.tuning.min;
                        prevMax  = S.tuning.max;
                        S.tuning.gain    = vals(1);
                        S.tuning.min     = vals(2);
                        S.tuning.max     = vals(3);
                        S.tuning.ratio   = vals(4);
                        S.tuning.deadN   = vals(5);
                        S.tuning.satN    = vals(6);
                        S.tuning.defGain = vals(7);
                        S.tuning.known   = true;

                        % 旁路日志: 记下这条回读【解析出来的七个数】—— 判"滑条显示的范围到底
                        %   是不是 C++ 声明的那个"时, 这是唯一可比的证据 (线上没有字段名, 靠位置)。
                        tlog('TUNE', sprintf(['gain=%.4f min=%.4f max=%.4f ratio=%.6f ' ...
                              'deadN=%.4f satN=%.4f defGain=%.4f'], ...
                              S.tuning.gain, S.tuning.min, S.tuning.max, S.tuning.ratio, ...
                              S.tuning.deadN, S.tuning.satN, S.tuning.defGain));

                        % ★ 范围【跟着回读走】, 不是只在第一次认 (2026-09-22 复审 Fix 1)。
                        %   原先只有 ~wasKnown 一条门 ⇒ C++ 重编/重启把 GAIN_MIN/GAIN_MAX 改了,
                        %   而本窗口一直开着: 新的 min/max 存进了 S.tuning 却【永远贴不到控件上】,
                        %   滑条此后一直给出 C++ 会拒的位置 —— "改 C++ 范围零 MATLAB 工作"这句
                        %   就得靠【重启 MATLAB 窗口】兑现。现在声明一变就重贴。
                        rangeChanged = wasKnown && ...
                            (prevMin ~= S.tuning.min || prevMax ~= S.tuning.max);
                        if ~wasKnown || rangeChanged
                            % ★ 【忘掉"我们上次发的是什么"】(2026-09-22 终审 Fix 4)。
                            %   下面那道拒收判据的前提是"我们发的值落在【当前这条回读给出的】范围之外
                            %   ⇒ C++ 拒了它"。而 S.tuningLastSent 可能来自【另一套范围】:
                            %   C++ 被重编成更窄的 GAIN_MAX (比如 300 → 200) 而本窗口一直开着,
                            %   当年被【接受】的 250 相对新范围就成了越界 ⇒ 判据成立, 打出一句
                            %   「增益 250.0 被拒」—— 那是【假话】, 它当时是被接受的。
                            %   ⇒ 范围一被重新声明, 那条"上次发了什么"就不再可比, 置回 NaN
                            %     (= "没发过"), 判据自然不成立。只在【这一支】置: 范围没变时
                            %     的越界回读仍然是真的拒收, 那条日志要留着。
                            S.tuningLastSent = NaN;
                            % 第一次回读: 用 C++ 给的上下限把控件打开 —— 不猜。
                            % 顺序【不能反】: 先 Limits 再(下面那段)Value。实测把 Limits 缩到
                            %   不含当前 Value 时 MATLAB 只夹紧不抛错 (120 → 150);
                            %   而上下限反了会让 uislider 直接抛错, 那条已在上面并进"可用"判据。
                            sldGain.Limits = [S.tuning.min S.tuning.max];
                            % 空值是"没有数"那一档, 只在【回读之前】与【回读自相矛盾】两处合法
                            %   ⇒ 有真值就关掉 AllowEmpty (实测: 置 false 之后赋 [] 会被拒)。
                            % ⚠ 补一条实测 (2026-09-22): 值的的确确还空着时置 false 【不报错】,
                            %   而是把值【悄悄变成 0】; 紧接的下面那句 Limits 写入又把它夹回 min。
                            %   ⇒ 屏幕上看不到这个 0, 而且这几句在同一个同步回调里、中间插不进
                            %   一次重绘 ⇒ 无用户可见效果。记下来是因为它【看起来像没发生】。
                            %   (Fix round 2 起"此后不可能为空"这句不再成立: 回读自相矛盾时
                            %    showGainUnknown() 会把编辑框重新清空。)
                            edGain.AllowEmpty = false;
                            edGain.Limits  = [S.tuning.min S.tuning.max];
                            sldGain.MajorTicks = ...
                                linspace(S.tuning.min, S.tuning.max, 5);
                            sldGain.MajorTickLabels = {};
                            sldGain.Enable = 'on';
                            edGain.Enable  = 'on';
                            btnGainDefault.Enable = 'on';
                            % 旁路日志: 控件【解锁那一刻】的实际 Limits 与是否首次。
                            % ⚠ 这里【刻意不记 Value】: 下面那句 Limits 赋值会把空值悄悄夹成
                            %   min (实测, 见本函数上面那段), 而真值是在更后面 (sldGain.Value =
                            %   S.tuning.gain) 才写进去的 —— 在这一行记 Value 会记到一个中间态
                            %   (实测 2026-09-23: 记成 100, 而实际显示 120), 那正是"拿快照下
                            %   断言"的坑。值改记在它真正落定的地方, 标签是 SET。
                            tlog('CTL', sprintf('enabled limits=[%.1f,%.1f] wasKnown=%d', ...
                                  sldGain.Limits(1), sldGain.Limits(2), wasKnown));
                            if ~wasKnown
                                fprintf(['[Relay] 增益控件已启用: 范围 [%.0f, %.0f], ' ...
                                         '当前 %.1f\n'], ...
                                    S.tuning.min, S.tuning.max, S.tuning.gain);
                            else
                                % 不静默: 范围变了必须看得见, 否则操作员不知道滑条被重贴过
                                fprintf(['[Relay] 增益范围已跟随 C++: [%.0f, %.0f] → ' ...
                                         '[%.0f, %.0f], 当前 %.1f\n'], ...
                                    prevMin, prevMax, S.tuning.min, S.tuning.max, ...
                                    S.tuning.gain);
                            end
                        end

                        % 拒收: 回读与我们刚发的不符 ⇒ 说清原因。
                        % 数字全部来自这条回读, 本文件不写 100/300。
                        % ⚠ 只有"不符"【还不够】—— 回读是 C++ 限频发出来的 (≥100ms 一条),
                        %   所以拖动中/刚松手时队列里躺着的那条报的是【几步之前】的值,
                        %   它与"被拒"长得一模一样。再加两个前置条件:
                        %     · 正在拖动 ⇒ 一律不判 (手指还在动, 回读必然滞后);
                        %     · 我们发的那个值得【落在回读给出的范围之外】才可能被拒
                        %       —— C++ 只按范围拒 (ForceTuning::setGain 校验 [min,max])。
                        if ~S.tuningDragging && ~isnan(S.tuningLastSent) && ...
                           (S.tuningLastSent < S.tuning.min || ...
                            S.tuningLastSent > S.tuning.max) && ...
                           abs(S.tuning.gain - S.tuningLastSent) > 1e-6
                            fprintf(['[Relay] 增益 %.1f 被拒 —— 可取范围 ' ...
                                     '[%.0f, %.0f], 仍是 %.1f\n'], ...
                                S.tuningLastSent, S.tuning.min, ...
                                S.tuning.max, S.tuning.gain);
                            % 旁路日志: 这一句【必须】进文件 —— 它是"拖动没生效"的唯一
                            %   信号, 而它本来只写命令窗(不在这份日志的覆盖范围里)。
                            tlog('REFUSED', sprintf(['sent=%.1f refused, range=[%.0f,%.0f] ' ...
                                  'still=%.1f'], S.tuningLastSent, S.tuning.min, ...
                                  S.tuning.max, S.tuning.gain));
                        end

                        if ~S.tuningDragging
                            % 拖动中不动滑块 —— 否则回读会和手指打架
                            % ⚠ 范围守卫 (Fix 1 之后才需要): 回读自相矛盾时 (gain 在它自己声明的
                            %   范围之外) 这一句会【抛错】而不是把值夹紧 (实测: 只有 Limits 赋值
                            %   才夹紧) ⇒ 那是收包循环里的一声炸, 会把那一拍的所有消息一起带走。
                            if S.tuning.gain >= S.tuning.min && S.tuning.gain <= S.tuning.max
                                sldGain.Value = S.tuning.gain;
                                edGain.Value  = S.tuning.gain;
                                % 旁路日志: 值【落定】的地方 —— 这才是"屏幕上到底是几"。
                                %   非拖动时每条回读都会走到这里 (回读本身被 C++ 限到 ≥100ms,
                                %   且只在值变/重连时才发) ⇒ 量很小, 不会淹掉文件。
                                tlog('SET', sprintf('value=%.3f (from RG readback)', ...
                                      sldGain.Value));
                                % ★ 从"未知"那一档恢复 —— 必须也把控件【恢复成可用】。
                                %   ⚠ 这一步不能指望上面那道范围门: 它只在"第一次回读 / 范围变了"
                                %   时才跑, 而矛盾状态【随时】可能被下一条正常回读解掉 (同一范围内
                                %   报一个合法 gain 就够了) ⇒ 只赋 Value 的话, 屏幕上是有了数,
                                %   两个控件却永远灰着、操作员再也动不了它们。
                                %   顺序: 先赋 Value (上面两句) 再关 AllowEmpty —— 值非空时置 false
                                %   才不会触发"悄悄变 0"那一条 (见上面第一道门的注释)。
                                %   占位串【不必在这里恢复】: 它只在框空着时可见, 而进入"空"只有
                                %   两处 (构造 / showGainUnknown), 两处都各自写了当时该显示的句子。
                                if S.tuning.displayUnknown
                                    sldGain.Enable    = 'on';
                                    edGain.Enable     = 'on';
                                    edGain.AllowEmpty = false;
                                    S.tuning.displayUnknown = false;
                                end
                            else
                                % ★ 跳过赋值的那一支 —— 那句 ⚠ 就挂在这里 (2026-09-22 Fix round 2)。
                                %   从前它写在上面的范围门里 ⇒ 只有"第一次回读 / 范围变了"才打。
                                %   于是【范围没变的矛盾回读】(gain 越界, 而 min/max 与上一条相同)
                                %   一次都不打地走完全程, 控件停在上一条回读的夹紧值上 ——
                                %   一个看着像、却不是机械臂在用的增益, 而且无人吭声。本文件的
                                %   整个立意就是"屏幕上的数 = 机械臂在用的数" ⇒ 不能留这个洞。
                                showGainUnknown();
                            end
                        end
                    else
                        % 不静默: 回读坏了必须看得见 —— 否则界面会一直停在"等待 C++…",
                        %   而没人知道为什么。这正是本项目最忌的"无声失败"。
                        fprintf(['[Relay] RG| 回读不可用 (字段数/数值/上下限 ' ...
                                 '其一不对), 已忽略: %s\n'], msg);
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
                elseif startsWith(msg, 'H|')
                    vals = sscanf(msg(3:end), '%d,%d,%d,%f,%d');
                    if length(vals) == 5
                        S.conn_enable = vals(1); S.conn_motion = vals(2);
                        S.conn_force  = vals(3); S.conn_ping   = vals(4);
                        S.conn_uptime = vals(5);
                    end
                elseif startsWith(msg, 'W|')
                    parts = split(msg(3:end), ',');
                    if numel(parts) >= 4
                        w.level = str2double(parts{1});
                        w.type = char(parts{2});
                        w.message = char(parts{3});
                        w.suggestion = char(parts{4});
                        if numel(parts) >= 5
                            w.param1 = str2double(parts{5});
                        else
                            w.param1 = 0;
                        end
                        if numel(parts) >= 6
                            w.param2 = str2double(parts{6});
                        else
                            w.param2 = 0;
                        end
                        S.warn_max_level = max(S.warn_max_level, w.level);
                        if S.warning_count >= 20
                            S.warning_count = 1;  % wrap ring buffer
                        else
                            S.warning_count = S.warning_count + 1;
                        end
                        S.warnings{S.warning_count} = w;
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
        % ★ 2026-09-21: 前向运动学【先算一次】—— 下面【既用来画骨架, 也用来把视野窗跟着 TCP 走】
        %   (见函数末尾那一段)。从前它只在 fallback 分支里算, 而视野需要 TCP ⇒ 提到最上面。
        %   STL 分支因此多算一次 FK —— 代价可忽略, 换来的是"视野永远知道末端在哪"。
        joints = fk.robotFk(ja(1),ja(2),ja(3),ja(4),ja(5),ja(6));
        % 更新 STL 模型 (hgtransform)
        if stlLoaded
            for i = 0:6
                T = fk.linkTransform(ja(1),ja(2),ja(3),ja(4),ja(5),ja(6), i);
                % hgtransform Matrix is column-major 4x4
                linkHg(i+1).Matrix = T;
            end
        else
            % Fallback: 骨架模型 (复用原 computeFK 逻辑)
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

        % 视野【不再】跟着末端走 —— 现场否掉了这种: 那样坐标原点会跟着工具跑, 空间参考系就没了。
        % 现在视野由 S.viewCenter / S.viewSpan 一处给出 (以【底座】为原点 + 滚轮放缩), 见 applyView。
        % ⚠ 视野【故意不在这里更新】: 它只在【创建时】与【滚轮】两处改。每帧重设 xlim/ylim/zlim
        %   会把 MATLAB 的自动刻度反复触发, 画面会抖 —— 而且视野本来就不需要每帧重算。
    end

    function applyView()
        % 3D 视野的【唯一一份定义】—— 中心与边长都来自 S, 别在别处再写一套 xlim/ylim/zlim。
        % 【三个轴共用同一个边长】创建时那句 axis(ax3d,'equal') 要求等比; 等边长才是真正的
        %   立方体视野。不对称的范围会和 axis equal 打架, 把某个轴拉扁 (从前就是这样)。
        h = S.viewSpan / 2;
        xlim(ax3d, [S.viewCenter(1)-h, S.viewCenter(1)+h]);
        ylim(ax3d, [S.viewCenter(2)-h, S.viewCenter(2)+h]);
        zlim(ax3d, [S.viewCenter(3)-h, S.viewCenter(3)+h]);
    end

    function onScrollZoom(evt)
        % 滚轮放缩。只在【光标位于 3D 面板上】时生效 —— 否则在右边面板上滚一下也会把视野缩掉。
        if ~isfield(evt, 'VerticalScrollCount') || evt.VerticalScrollCount == 0, return; end

        % ⚠ 命中判定整段放在 try 里: CurrentPoint / getpixelposition 在 uifigure 上的可用性
        %   随版本而变, 而"缩不了"比"在不该缩的地方缩了一下"更烦人 ⇒ 判不出来就【放行】。
        try
            cp = fig.CurrentPoint;
            pp = getpixelposition(ax3d);
            if cp(1) < pp(1) || cp(1) > pp(1)+pp(3) || cp(2) < pp(2) || cp(2) > pp(2)+pp(4)
                return;
            end
        catch
            % 判不了 ⇒ 放行 (见上)
        end

        % MATLAB 的 VerticalScrollCount: 【向下滚为正】。⇒ 上滚 (负) 应当【放大】⇒ 边长变小。
        S.viewSpan = min(max(S.viewSpan * (1 + 0.12 * double(evt.VerticalScrollCount)), ...
                             S.viewSpanMin), S.viewSpanMax);
        applyView();
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

        % -- 连接状态 --
        portDot = @(ok) ternary(ok, '●', '○');
        portClr = @(ok) ternary(ok, clr.green, clr.red);
        enOk = S.conn_enable;  moOk = S.conn_motion;  foOk = S.conn_force;
        pingStr = '--';
        if S.conn_ping > 0
            pingStr = sprintf('%.0f', S.conn_ping);
        end
        if S.conn_ping < 20
            pingClr = clr.green;
        elseif S.conn_ping < 100
            pingClr = clr.yellow;
        else
            pingClr = clr.red;
        end
        uptimeStr = sprintf('%02d:%02d:%02d', ...
            floor(S.conn_uptime/3600), floor(mod(S.conn_uptime,3600)/60), mod(S.conn_uptime,60));
        lblConnMon.Text = {...
            sprintf('%s Enable  %s Motion  %s Force   PING %s ms', ...
                portDot(enOk), portDot(moOk), portDot(foOk), pingStr); ...
            sprintf('HB: %s    Uptime: %s', ...
                ternary(enOk, 'OK', 'LOST'), uptimeStr)};
        % Apply colors (only to the first line's port dots — use FontColor for whole thing)
        if enOk && moOk
            lblConnMon.FontColor = clr.text_dim;
        else
            lblConnMon.FontColor = clr.red;
        end

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

        % -- 笔杆 (Touch) 姿态 --
        % P| 那条消息【本来就】带这三个角: C++ 侧发的是 P|x,y,z,sx,sy,sz
        % (RelayCore.cpp 的 sendRelayUpdate), 而这里从前只用 tp(1:3) 去画那支笔, 后三个一直没用。
        % 2026-09-22 加: 现场判"按钮2 大幅晃动"的【Euler 退化】假设要用它 ——
        %   C++ 侧姿态跟随用 Euler 角作差, 提取式 ry = asin(-R[2][0]), 在 ry ≈ ±90° 附近 rx/rz 退化
        %   (奇点分支直接把 rz 定成 0) ⇒ 微小的物理转动会产出巨大的角度差 ⇒ 缓慢转动也大幅晃。
        %   ⇒ 【Ry 接近 ±90° 就是那个假设成立的线索】。同一读数在 C++ 侧也能看: 'm' 模式按 SPACE
        %     会打一行 "笔杆姿态 Rx=.. Ry=.. Rz=.." —— 但那是【快照】, 这里是【连续】的。
        % ⚠ 注意这两组角是【两个不同的设备】: 上面 Orientation 是机械臂的 (来自 RP|),
        %   这一行是手写笔的 (来自 P|) —— 别混着比。
        tp = S.touch_pos;
        stylusLine = sprintf('Stylus (deg):     Rx: %7.2f  Ry: %7.2f  Rz: %7.2f', ...
            tp(4), tp(5), tp(6));
        if abs(abs(tp(5)) - 90) <= 20            % |Ry| ∈ [70,110]: 判据取 ±20° 的环带
            stylusLine = [stylusLine '  <== |Ry| near 90: Euler 退化'];
        end

        lblCoord.Text = {
            sprintf('Position (mm):    X: %8.2f  (target: %8.2f)', rp(1), rt(1));
            sprintf('                   Y: %8.2f  (target: %8.2f)', rp(2), rt(2));
            sprintf('                   Z: %8.2f  (target: %8.2f)', rp(3), rt(3));
            sprintf('Orientation (deg): Rx: %7.2f  Ry: %7.2f  Rz: %7.2f', rp(4), rp(5), rp(6));
            '';
            stylusLine;
            '';
            sprintf('Joints (deg):  J1:%7.1f  J2:%7.1f  J3:%7.1f', ja(1:3));
            sprintf('               J4:%7.1f  J5:%7.1f  J6:%7.1f', ja(4:6));
            '';
            sprintf('Force (N):   Fx: %7.2f   Fy: %7.2f   Fz: %7.2f', ff(1), ff(2), ff(3));
            '';
            sprintf('TX: %s', ternary(txActive, 'ACTIVE', 'IDLE'))};

        % -- Safety & Diagnostics --
        % C++ RobotState enum: 0=DISCONNECTED 1=CONNECTED 2=READY 3=RUNNING
        %   4=DEGRADED 5=ALARM 6=RECOVERING 7=FATAL
        stateNames = {'DISCONNECTED','CONNECTED','READY','RUNNING',...
                      'DEGRADED','ALARM','RECOVERING','FATAL'};
        stateColors = {clr.text_dim, clr.blue, clr.blue, clr.green,...
                       clr.orange, clr.yellow, clr.yellow, clr.red};
        st = S.safety_state + 1;
        if st < 1, st = 1; elseif st > 8, st = 8; end

        safetyLines = {};
        safetyLines{1} = sprintf('Safety: %s  |  Speed: %.1fx  |  Alarms: %d', ...
            stateNames{st}, S.safety_speed, S.safety_alarms);
        lblSafety.FontColor = stateColors{st};

        % 关节限位
        [minM, worstJ] = min(S.joint_margins);
        if minM < 15
            safetyLines{2} = sprintf('J%d near limit: %.1f deg margin', worstJ, minM);
            if st < 5  % don't downgrade DEGRADE/ALARM/RECOVERING/FATAL to joint orange
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

        % -- 延迟 + 状态 (must precede warning override so it can take effect) --
        lblDelay.Text = sprintf('Touch->Relay: %.1f ms', S.touch_relay_delay);
        lblState.Text = sprintf('[%s]  Spd: %.1fx', stateNames{st}, S.safety_speed);
        lblState.FontColor = stateColors{st};

        % -- Warnings (singularity avoidance) --
        if S.warning_count > 0 && S.warn_max_level > 0
            % Show up to 3 most recent warnings
            startIdx = max(1, S.warning_count - 2);
            for wi = startIdx:S.warning_count
                w = S.warnings{wi};
                if w.level == 2
                    prefix = '⬤ CRITICAL';
                    wColor = clr.red;
                elseif w.level == 1
                    prefix = '⬤ WARN';
                    wColor = clr.orange;
                else
                    prefix = '✔ INFO';
                    wColor = clr.blue;
                end
                safetyLines{end+1} = sprintf('%s: %s', prefix, w.message);
                safetyLines{end+1} = sprintf('     → %s', w.suggestion);
            end

            % Top-bar state override for warnings
            if S.warn_max_level == 2
                lblState.Text = '[⚠ SINGULAR RISK]';
                lblState.FontColor = clr.red;
            elseif S.warn_max_level == 1
                lblState.Text = '[⚠ CAUTION]';
                lblState.FontColor = clr.orange;
            end
        end

        % Decay warnings: clear warn_max_level each refresh cycle
        % (C++ re-sends warnings each frame while condition persists)
        S.warn_max_level = 0;

        lblSafety.Text = safetyLines;

        % -- 力反射增益 (2026-09-22) --
        % 把 C++ 回读的三个展示量写成一行 (净比例 + 响应窗口)。
        % 显示【窗口】而不是单个饱和点 —— satN 单给一个数会把死区那个前提藏起来
        % (gain 300 时窗口只有 0.20–0.67N, 死区占了 30%)。
        % ⚠ "该轴分量"这句不能省: HapticCallback 是【三个轴各自夹】, 不是夹合力。
        if S.tuning.known
            if S.tuning.displayUnknown
                % ⚠ 回读自相矛盾时【这行也不能照常写】: ratio/satN 同样是【那一条】回读算出来的
                %   (C++ 侧 netRatioPerGainUnit()*g), 与控件一样不可信。若还照常显示, 面板就
                %   自相矛盾 —— 标题按某条回读报一个窗口, 而控件说"未知"。这里改成说清矛盾。
                lblGainTitle.Text = sprintf( ...
                    'Reflection Gain  ⚠ 回读自相矛盾: 报的 %.1f 不在 [%.0f, %.0f] —— 增益未知', ...
                    S.tuning.gain, S.tuning.min, S.tuning.max);
            else
                lblGainTitle.Text = sprintf( ...
                    'Reflection Gain  ≈%.2f:1   响应窗口 %.2f – %.2f N (该轴分量)', ...
                    S.tuning.ratio, S.tuning.deadN, S.tuning.satN);
            end
        end
    end

    function updateForceHistory()
        % Reconstruct full 100-point timeline from ring buffer
        idx = S.force_hist_idx;
        if idx == 1 && S.force_hist_fx(100) == 0 && S.force_hist_fx(1) == 0
            return;  % no data yet
        end
        order = [idx:100, 1:idx-1];
        fx = S.force_hist_fx(order);
        fy = S.force_hist_fy(order);
        fz = S.force_hist_fz(order);

        % Direct XData/YData assignment (fast)
        set(S.alFx, 'YData', fx);
        set(S.alFy, 'YData', fy);
        set(S.alFz, 'YData', fz);

        % Auto-scale Y axis
        mx = max(max(abs(fx)), max(abs(fy)));
        mx = max(mx, max(abs(fz)));
        if mx < 0.5, mx = 5; end
        ylim(S.axForceHist, [-mx*1.2, mx*1.2]);
    end

    function r = ternary(cond, tVal, fVal)
        if cond, r = tVal; else, r = fVal; end
    end

end
