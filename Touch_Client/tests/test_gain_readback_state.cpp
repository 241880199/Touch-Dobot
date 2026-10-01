// Standalone test: GainReadback::State —— RG| 回读的【状态机】(判决 + 三样状态的迁移)
// Build: build_gain_readback_state_test.bat
// Run:   test_gain_readback_state.exe
//
// 【为什么有它】判决 (GainReadbackPolicy.h) 2026-09-24 就有用例了, 但【判决够不到的那半】
//   一直在 RelayCore::sendReflectionGain 里, 而 RelayCore.cpp 不被任何测试编译:
//     · 三样状态什么时候推进 / pending 什么时候置与清;
//     · 调用点传的实参 (判决用例只看形参)。
//   代价付过一次: 限频被传成 force=true ⇒ 静默死掉。本文件把第一样钉住;
//   第二样由 SendMode 枚举在【编译期】挡住【形状】(裸 bool / 隐式转换); ⚠ 但挡不住【选错枚举量】
//   (Forced 与 Throttled 传反 = 2026-09-22 那类逻辑错误, 照样编译通过)。
//   (RelayCore.cpp 仍没有覆盖, 别写成"接线上有测试了")。
//
// ⚠ 【不睡】: 时钟由用例作为入参给 (beginSend 的 nowMs), 这是 2026-09-24 起本仓的约定。

#include <iostream>
#include "../relay/GainReadback.h"

using GainReadback::SendMode;
using GainReadback::State;

static int g_passed = 0, g_failed = 0;

#define TEST(name) do { std::cout << "  " << #name << "... "; } while(0)
#define PASS() do { std::cout << "PASS" << std::endl; g_passed++; } while(0)
#define CHECK(cond) do { if (!(cond)) { std::cout << "FAIL: " << #cond << std::endl; g_failed++; return; } } while(0)

// ===== 格 1: 首次 Throttled 就发 (初值 0 是 setGain 不接受的值 ⇒ 保守方向) =====
static void test_first_throttled_sends() {
    TEST(first_throttled_sends);
    State s;
    // 值 120、时刻 1000、从没发过 (lastSentGain = 0) ⇒ 值"变了" ⇒ 发
    CHECK(s.beginSend(SendMode::Throttled, 120.0, 1000));
    CHECK(!s.pending());
    CHECK(s.lastReportMs() == 1000);
    CHECK(s.lastSentGain() == 120.0);
    PASS();
}

// ===== 格 2: Forced 恒发, 不看两个时刻 =====
static void test_forced_always_sends() {
    TEST(forced_always_sends);
    State s;
    CHECK(s.beginSend(SendMode::Forced, 120.0, 1000));
    // 同值、同刻 ⇒ Throttled 会挡, Forced 不挡
    CHECK(s.beginSend(SendMode::Forced, 120.0, 1000));
    CHECK(s.lastReportMs() == 1000);
    PASS();
}

// ===== 格 3: Throttled 值没变 ⇒ 不发, 且【顺手清掉 pending】 =====
// 这是抽取时最容易漏的一条 (抽取前在 RelayCore.cpp 的 SkipUnchanged 分支里 ——
// ⚠ 别记行号: Task 2 会把这些行搬走, 按内容找)。
static void test_unchanged_clears_pending() {
    TEST(unchanged_clears_pending);
    State s;
    CHECK(s.beginSend(SendMode::Throttled, 120.0, 1000));   // 发, 记 120 @1000
    CHECK(!s.beginSend(SendMode::Throttled, 130.0, 1010));  // 值变了但太近 ⇒ 挡, 置 pending
    CHECK(s.pending());
    CHECK(!s.beginSend(SendMode::Throttled, 120.0, 1020));  // 值又变回 120 ⇒ 没可报的
    CHECK(!s.pending());                                    // ★ 顺手清
    CHECK(s.lastReportMs() == 1000);                        // 且【不】推进
    PASS();
}

// ===== 格 4: 值变了但没到窗口 ⇒ 挡 + 置 pending + 【不】推进 lastReportMs =====
static void test_too_soon_sets_pending_without_advancing() {
    TEST(too_soon_sets_pending_without_advancing);
    State s;
    CHECK(s.beginSend(SendMode::Throttled, 120.0, 1000));
    CHECK(!s.beginSend(SendMode::Throttled, 130.0, 1099));  // 窗口 100ms, 差 99 ⇒ 挡
    CHECK(s.pending());
    CHECK(s.lastReportMs() == 1000);                        // ★ 它记的是"上次真的发出去"
    CHECK(s.lastSentGain() == 120.0);                       // ★ 同理, 值也不推进
    PASS();
}

// ===== 格 5: 承接格 4 —— 过了窗口再进来就发, 且 pending 被清 =====
static void test_pending_resend_after_window() {
    TEST(pending_resend_after_window);
    State s;
    CHECK(s.beginSend(SendMode::Throttled, 120.0, 1000));
    CHECK(!s.beginSend(SendMode::Throttled, 130.0, 1099));
    CHECK(s.pending());
    CHECK(s.beginSend(SendMode::Throttled, 130.0, 1100));   // 差 == 窗口 ⇒ 放行
    CHECK(!s.pending());
    CHECK(s.lastReportMs() == 1100);
    CHECK(s.lastSentGain() == 130.0);
    PASS();
}

// ===== 格 6: 被拒的回读必须发得出去 (Forced 不看"值没变") =====
// 规格的中心例子: MATLAB 发 500 被拒 ⇒ 生效值没变 ⇒ 若走 Throttled 就一个字节都发不出去。
static void test_forced_bypasses_unchanged() {
    TEST(forced_bypasses_unchanged);
    State s;
    CHECK(s.beginSend(SendMode::Forced, 120.0, 1000));
    CHECK(s.beginSend(SendMode::Forced, 120.0, 1010));      // 同值、【不同刻】(1010 ≠ 1000), 仍发
    CHECK(s.lastReportMs() == 1010);
    PASS();
}

// ===== 格 7: 32 位时钟环绕 (无符号相减) =====
static void test_clock_wraparound() {
    TEST(clock_wraparound);
    State s;
    CHECK(s.beginSend(SendMode::Throttled, 120.0, 0xFFFFFFF0u));   // 发, last = 0xFFFFFFF0
    // 实际过去 0x210 = 528ms ≥ 100 ⇒ 发
    CHECK(s.beginSend(SendMode::Throttled, 130.0, 0x00000200u));
    CHECK(s.lastReportMs() == 0x00000200u);
    PASS();
}

int main() {
    std::cout << "=== GainReadbackState Tests ===" << std::endl;
    test_first_throttled_sends();
    test_forced_always_sends();
    test_unchanged_clears_pending();
    test_too_soon_sets_pending_without_advancing();
    test_pending_resend_after_window();
    test_forced_bypasses_unchanged();
    test_clock_wraparound();
    std::cout << std::endl;
    std::cout << g_passed << " passed, " << g_failed << " failed" << std::endl;
    return g_failed > 0 ? 1 : 0;
}
