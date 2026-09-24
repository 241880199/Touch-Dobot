#pragma once

// RG| 回读的【限频判决】—— 纯函数, 不读时钟、不碰 socket、不写 static。
//
// 【出处】: 2026-09-22 的"力反射增益 MATLAB 可调"计划。判决从前内联在
//   RelayCore::sendReflectionGain 里, 而 RelayCore.cpp 不被任何测试编译 ⇒ 零自动化用例。
//   抽出来的理由是一次真实事故: 限频被误传成 force=true 而【静默死掉】, 测试床没响。
//
// 【规格】: 只在目标值真的变了、且距上次回读 ≥ GAIN_REPORT_MIN_INTERVAL_MS 时才发。
//   两条例外由 force=true 表达 —— 【force 必须是第一道】, 理由见下面两段注释。
namespace GainReadbackPolicy {

enum class GainReport {
    Send,           // 现在就发
    SkipUnchanged,  // 目标值相对【上次真的发出去的那条】没变 ⇒ 没有可报的东西
    SkipTooSoon,    // 变了, 但距上次【发送】不足窗口 ⇒ 记下待发, 由 pollRelayCommands 补
};

// 距上次【发送】的最小间隔 (ms)。
// ⚠ 语义是"上次真的发出去的时刻", 不是"上次被挡下的时刻": 若把被挡下的时刻也记进去,
//   拖动期间每一条命令都会把期限往后推 ⇒ 只要命令不停就永远发不出去, 限频变成饥饿。
constexpr unsigned long GAIN_REPORT_MIN_INTERVAL_MS = 100;

// ⚠ force 必须是【第一道短路】, 不许挪到值判定之后:
//   · force=true 有三个用途 —— 连接/重连时的强制回读, 以及【被拒绝的增益改动】。
//   · 被拒 = setGain 在任何 store 之前就返回 ⇒ 生效值【按构造】没变 ⇒ "值没变"那道闸
//     必然命中 ⇒ 一个字节都发不出去。偏偏那一条正是 MATLAB 最需要的: 它的滑条已经动了,
//     正等着被纠正回真值。
//   ⇒ 谁把 force 塞进"值没变"的判断里, test_gain_readback_policy 的第 1 格当场变红。
inline GainReport gainReportDecision(bool force, double g, double lastSentGain,
                                     unsigned long nowMs, unsigned long lastReportMs,
                                     unsigned long windowMs) {
    if (force) return GainReport::Send;
    if (g == lastSentGain) return GainReport::SkipUnchanged;
    // 无符号相减: 32 位时钟环绕时天然正确 (见用例 test_clock_wraparound)。
    if ((nowMs - lastReportMs) < windowMs) return GainReport::SkipTooSoon;
    return GainReport::Send;
}

} // namespace GainReadbackPolicy
