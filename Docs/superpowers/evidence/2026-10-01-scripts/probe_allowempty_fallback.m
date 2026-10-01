% probe_allowempty_fallback.m  ——  Minor 第 13 条的现场判别（2026-10-01）
%
% 【要证明什么】`relay_gui.m` 里编辑框的构造改成了"先按老属性建、再问 isprop 决定要不要开 AllowEmpty"。
%   本机是 R2025b ⇒ 支持 AllowEmpty ⇒ **复现不出老 MATLAB 的构造期失败**。
%   所以退一步：**把同一段代码形状跑两遍** —— 一遍走"支持"分支、一遍**强制走"不支持"分支**
%   —— 确认**两遍都能把控件建出来**（回退路径真的可用, 不是只在注释里写着）。
%
% ⚠ 证据边界（如实）：这验的是【那段代码形状】, 不是 relay_gui() 整个函数
%   （那要真的把窗口起起来）。relay_gui.m 本身的"能解析"由 `checkcode` 单独证明。
%
% ⚠ 2026-10-01 实测：第一版在**一个会话里建了又关多个 uifigure** ⇒ MATLAB fatal error。
%   ⇒ 本版改成**只建一个图窗、里面放两个控件**, 结尾只 close 一次。
%
% 跑法: matlab -batch "probe_allowempty_fallback"

function probe_allowempty_fallback()
    fprintf('=== Minor 13 探针: AllowEmpty 回退路径 ===\n');
    fprintf('MATLAB %s\n\n', version);

    f = uifigure('Visible', 'off');

    % ---------- 1. 走"支持"分支（= 本机 R2025b 的真实路径） ----------
    e1 = buildField(f, true);
    fprintf('[1] 支持分支: isprop(AllowEmpty)=%d  Value 是空? %d\n', ...
        isprop(e1, 'AllowEmpty'), isempty(e1.Value));
    ok1 = isprop(e1, 'AllowEmpty') && isempty(e1.Value);

    % ---------- 2. ★ 强制走"不支持"分支（模拟老 MATLAB） ----------
    e2 = buildField(f, false);
    fprintf('[2] 回退分支: 控件建出来了? %d   Value=%s\n', isvalid(e2), mat2str(e2.Value));
    ok2 = isvalid(e2);

    close(f);
    fprintf('\n结论: 支持分支 %s · 回退分支 %s\n', tf(ok1), tf(ok2));
    if ~(ok1 && ok2)
        error('探针失败');
    end
    fprintf('⇒ 两遍都能把控件建出来 ⇒ 回退路径【真的可用】（老 MATLAB 上窗口不会因它而起不来）。\n');
    fprintf('⚠ 但本机是 R2025b, 【复现不出】老 MATLAB 的构造期失败 —— 那一条只有老版本上才能证。\n');
end

% 与 relay_gui.m 里【同一个形状】的构造
function e = buildField(parent, allowEmptySupported)
    e = uieditfield(parent, 'numeric', 'Enable', 'off', ...
        'FontName', 'Consolas', 'FontSize', 10);
    if allowEmptySupported          % 真实代码里这里是 isprop(e,'AllowEmpty')
        e.AllowEmpty = true;
        e.Value = [];               % 必须在 AllowEmpty 之后
    end
    % 回退路径: 什么都不设 ⇒ 用默认值（老 MATLAB 上就是"框里显示默认数"那一档）
end

function s = tf(c)
    if c, s = 'OK'; else, s = 'FAIL'; end
end
