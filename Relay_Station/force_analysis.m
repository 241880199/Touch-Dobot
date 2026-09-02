function force_analysis(csv_path)
%FORCE_ANALYSIS  力反馈 A/B 对照实验笔压稳定性分析
%  读取 C++ 端落盘的 CSV, 按 ff_enabled 分组, 计算笔压稳定性指标并绘图。
%
%  输入: csv_path — CSV 文件路径 (默认 'force_demo_log.csv')
%
%  指标 (每组): Fz RMS / 方差 / 接触段数(断线) / 书写时长
%  输出: 两组并排 Fz 曲线对比图 + 指标表
%
%  说明: 默认以 Fz 作为法向力(笔压)近似。若实机标定后法向力非传感器 Z 轴,
%        修改下方 AXIS 变量为对应列名 (如 'Fx'/'Fy')。

    if nargin < 1 || isempty(csv_path)
        csv_path = 'force_demo_log.csv';
    end
    if ~isfile(csv_path)
        error('force_analysis: 未找到文件 %s', csv_path);
    end

    % ===== 可调参数 =====
    AXIS      = 'Fz';   % 法向力(笔压)对应的列名
    CONTACT_N = 0.5;    % 接触阈值 (N) — 软垫稳定接触任务落笔判定, 实机可调
    % ====================

    % 读取 CSV (首行表头)
    T = readtable(csv_path);

    % 检查必需列
    required = {'t_ms', AXIS, 'ff_enabled'};
    for i = 1:numel(required)
        if ~any(strcmp(T.Properties.VariableNames, required{i}))
            error('force_analysis: 缺少列 %s', required{i});
        end
    end

    Fz = T.(AXIS);            % 法向力 (笔压)
    t  = T.t_ms / 1000;       % ms -> s
    ff = T.ff_enabled;        % 0=关, 1=开

    % 分组: 力反馈 ON 在前 (A 组), OFF 在后 (B 组)
    groups = sort(unique(ff), 'descend');

    results = table();
    fig = figure('Name', 'Force Feedback A/B 对比', 'Position', [100 100 1200 480]);

    for g = groups'
        idx = (ff == g);
        fg = Fz(idx); tg = t(idx);
        if isempty(fg), continue; end

        inContact = fg > CONTACT_N;
        % 接触段: 差分找上升沿 (抬笔 -> 落笔)
        edges = diff([false; inContact]);
        nContact = numel(find(edges == 1));

        % 接触期间的法向力统计
        fContact = fg(inContact);
        if isempty(fContact)
            rmsV = 0; varV = 0;
        else
            rmsV = sqrt(mean(fContact.^2));
            varV = var(fContact);
        end

        dur = max(tg) - min(tg);

        results = [results; table(g, rmsV, varV, nContact, dur, ...
            'VariableNames', {'ff_enabled','Fz_rms_N','Fz_var_N2','contact_segments','duration_s'})];

        % 绘图
        subplot(1, 2, find(groups == g));
        plot(tg, fg, 'LineWidth', 1.0); hold on;
        yline(CONTACT_N, 'r--', 'LineWidth', 1);
        xlabel('t (s)'); ylabel(sprintf('%s (N)', AXIS));
        if g == 1
            title('A 组: 力反馈 ON');
        else
            title('B 组: 力反馈 OFF');
        end
        grid on; hold off;
    end

    % 输出指标表
    fprintf('\n===== 力反馈 A/B 对照结果 =====\n');
    disp(results);
    fprintf('================================\n\n');
end
