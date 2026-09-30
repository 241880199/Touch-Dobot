#pragma once

// RG| 回读的【状态机】—— 判决 + 三样状态的迁移。头文件内联, 无 .cpp 可链。
//
// 【它解决什么】(2026-09-30, 22 条 Minor 的第 12 条)
//   判决本身 2026-09-24 就抽成了纯函数 (relay/GainReadbackPolicy.h), 有 5 格用例。
//   但【判决够不到的那半】还留在 RelayCore::sendReflectionGain 里 —— 而 RelayCore.cpp
//   【不被任何测试编译】:
//     · 三样状态的迁移 (什么时候推进"上次发送"那两样、pending 什么时候置与清);
//     · 调用点传的【实参】—— 判决用例只看形参, 某个调用点把 force 传错, 5 格一个都不会红。
//   代价已经付过一次 (2026-09-22): 限频被传成 force=true ⇒ 静默死掉, 测试床一声不响。
//   ⇒ 本单元把这两样都变成可测的; 调用点改用【枚举】而不是裸 bool ⇒ 传错的形状被编译器挡住。
//
// 【与 GainReadbackPolicy.h 的分工 —— 别把两者合并】
//   判决在那边 (纯函数、无状态、"给定这些数该不该发", 可以单独证伪);
//   状态在这边 ("发过之后记什么")。本文件 include 它、只用它, **不复制它的规则**。
//
// ⚠ 【线程】三个成员仍是 atomic —— 与抽取前【逐字相同】的理由:
//   GLUT idle 线程 (拖动/拒绝/重连) 与 pollRelayCommands 不同步地碰它们,
//   非原子对象上的不同步读写是 UB。残留的只是【次序】上的竞争, 且无害 ——
//   三个状态【不参与强制发送的决策】(SendMode::Forced 一个判断都不从它们取, 只写)
//   ⇒ 交错最坏 = 多回一条、或晚回一条。

#include <atomic>
#include "GainReadbackPolicy.h"

namespace GainReadback {

// ★ 参数类型化: 调用点不再传裸 bool。
//   抽取前是 `sendReflectionGain(bool force)`, 调用点读作 `sendReflectionGain(true)` ——
//   一个 bool 字面量, **看不出语义、编译器也管不着**。2026-09-22 的事故就长在这个形状上。
enum class SendMode {
    // 无条件发, 不看下面那两条闸。三个用途: 连上时 / 重连成功时 / 【被拒绝的增益改动】。
    //   ⚠ 第三个用途容易被"统一"掉: 被拒 ⇒ 生效值【按构造】没变 ⇒ 走 Throttled 会被
    //     "值没变"那道闸必然命中 ⇒ 一个字节都发不出去, 而 MATLAB 的滑条此刻已经动了。
    Forced,
    // 拖动洪水与 pollRelayCommands 的补发: 走"值变了 + 距上次发送 ≥ 窗口"两道闸。
    Throttled,
};

class State {
public:
    // 判决【并】完成状态迁移, 一步做完。时钟由调用方给 ⇒ 用例不必睡。
    // 返回 true ⇒ 调用方【现在】应当真的把这条回读发出去。
    bool beginSend(SendMode mode, double g, unsigned long nowMs) {
        switch (GainReadbackPolicy::gainReportDecision(
                    mode == SendMode::Forced, g, m_lastSentGain.load(), nowMs,
                    m_lastReportMs.load(),
                    GainReadbackPolicy::GAIN_REPORT_MIN_INTERVAL_MS)) {
        case GainReadbackPolicy::GainReport::SkipUnchanged:
            // ⚠ 必须【顺手清掉待发标志】: 一条被限频挡下的 A→B 之后值又变回 A, 此时"待发"
            //   已无事可做; 留着标志会让 pollRelayCommands 每帧都调进来、每帧都从这里返回
            //   ⇒ 标志卡在 true 再也不动 (无害, 但那个标志从此失去意义)。
            m_pending.store(false);
            return false;
        case GainReadbackPolicy::GainReport::SkipTooSoon:
            // 记下待发, 由 pollRelayCommands 补 —— 最后一条不丢。
            // ⚠ 这里【不】推进 m_lastReportMs: 它记的是"上次【真的发出去】"的时刻。
            m_pending.store(true);
            return false;
        case GainReadbackPolicy::GainReport::Send:
            break;
        }
        // ⚠ 两样状态都落笔在【发送调用之前】(调用方紧接着才发) ⇒ socket 恰在那一瞬间失效时,
        //   这一次算"发过了"、同一个值的重试会被"值没变"挡下。这是【已知且能收敛】的:
        //   重连成功时的 Forced 不看这两条闸, 会把当前值原样再送一条 ⇒ 值最终一定到 MATLAB。
        //   ⇒ 按这个定义读这两个名字, **别按"确认送达"读**。
        m_pending.store(false);
        m_lastReportMs.store(nowMs);
        m_lastSentGain.store(g);
        return true;
    }

    // pollRelayCommands 每帧问它: 有没有被限频挡下、还欠 MATLAB 一条?
    bool pending() const { return m_pending.load(); }

    // 只读回显, 供用例与诊断 —— 【不参与判决】。
    double lastSentGain() const { return m_lastSentGain.load(); }
    unsigned long lastReportMs() const { return m_lastReportMs.load(); }

private:
    // m_lastSentGain 初值刻意选 0 —— 那是 setGain 不会接受的值 ⇒ 在第一次 Forced 之前
    //   若有人用 Throttled 进来, 它一定发得出去 (保守方向)。与抽取前逐字相同。
    std::atomic<unsigned long> m_lastReportMs{0};
    std::atomic<double>        m_lastSentGain{0.0};
    std::atomic<bool>          m_pending{false};
};

} // namespace GainReadback
