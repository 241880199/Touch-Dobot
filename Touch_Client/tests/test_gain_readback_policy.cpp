// Standalone test: GainReadbackPolicy —— RG| 回读限频的判决 (纯函数)
// Build: build_gain_readback_policy_test.bat
// Run:   test_gain_readback_policy.exe
//
// 【为什么把它抽出来】: 这段判决从前内联在 RelayCore::sendReflectionGain 里, 而
//   RelayCore.cpp 不被任何测试编译 ⇒ 它【一条自动化用例都没有】。代价已经付过一次:
//   一轮修复把 dispatch 传成 force=true, 限频【静默死掉】, 测试床一声不响。
// 【本文件只测判决, 不测接线】: 调用点仍未被任何测试覆盖 (那半只能靠上机)。

#include <iostream>
#include <cmath>
#include "../relay/GainReadbackPolicy.h"

using GainReadbackPolicy::GainReport;
using GainReadbackPolicy::gainReportDecision;
static const unsigned long W = GainReadbackPolicy::GAIN_REPORT_MIN_INTERVAL_MS;

static int g_passed = 0, g_failed = 0;

#define TEST(name) do { std::cout << "  " << #name << "... "; } while(0)
#define PASS() do { std::cout << "PASS" << std::endl; g_passed++; } while(0)
#define CHECK(cond) do { if (!(cond)) { std::cout << "FAIL: " << #cond << std::endl; g_failed++; return; } } while(0)

// ===== 第 1 格: force=true 恒发送 =====
// ★ 这一格【真的出过事故】(2026-09-22: 传成 force=true 让限频静默死掉; 后来反向又差点
//   把 force 参与进"值没变"的判断 ⇒ 被拒的回读一个字节都发不出去)。
//   三个子格把"force 必须【第一道】短路"钉死: 值没变也要发、0ms 也要发、值变了更要发。
static void test_force_true_always_sends() {
    TEST(force_true_always_sends);
    CHECK(gainReportDecision(true, 120.0, 120.0, 1000,  999, W) == GainReport::Send); // 值没变
    CHECK(gainReportDecision(true, 120.0, 120.0, 1000, 1000, W) == GainReport::Send); // 距上次 0ms
    CHECK(gainReportDecision(true, 130.0, 120.0, 5000, 1000, W) == GainReport::Send); // 值变了且过窗口
    PASS();
}

// ===== 第 2 格: 值没变 ⇒ 不发 (拖动洪水里的绝大多数) =====
static void test_unchanged_value_is_skipped() {
    TEST(unchanged_value_is_skipped);
    CHECK(gainReportDecision(false, 120.0, 120.0, 1000, 1000, W) == GainReport::SkipUnchanged);
    CHECK(gainReportDecision(false, 120.0, 120.0, 999999, 1, W) == GainReport::SkipUnchanged);
    PASS();
}

// ===== 第 3 格: 值变了但距上次发送不足窗口 ⇒ 记待发 =====
static void test_too_soon_is_skipped() {
    TEST(too_soon_is_skipped);
    CHECK(gainReportDecision(false, 130.0, 120.0, 1050, 1000, W) == GainReport::SkipTooSoon);
    PASS();
}

// ===== 第 4 格: 窗口的开关边界 =====
// ★ 边界必须【写死在一侧】: 实现里是 `(now - last) < window ⇒ 挡`, 所以
//   差 == window 时【放行】。这一格就是那句话的判据 —— 改 `<` 为 `<=` 它当场红。
static void test_window_boundary() {
    TEST(window_boundary);
    CHECK(gainReportDecision(false, 130.0, 120.0, 1000 + W - 1, 1000, W) == GainReport::SkipTooSoon);
    CHECK(gainReportDecision(false, 130.0, 120.0, 1000 + W,     1000, W) == GainReport::Send);
    PASS();
}

// ===== 第 5 格: 32 位时钟环绕 =====
// s_lastGainReportMs 是 DWORD ⇒ 开机 49.7 天后会绕回来。无符号相减天然正确,
// 这一格把那个语义钉住 (换成有符号比较就是另一回事了)。
static void test_clock_wraparound() {
    TEST(clock_wraparound);
    // last = 0xFFFFFFF0, now = 0x00000200 ⇒ 实际过去了 0x210 = 528ms ≥ 100 ⇒ 发
    CHECK(gainReportDecision(false, 130.0, 120.0, 0x00000200u, 0xFFFFFFF0u, W) == GainReport::Send);
    // last = 0xFFFFFFF0, now = 0x00000050 ⇒ 实际过去了 0x60 = 96ms < 100 ⇒ 挡
    CHECK(gainReportDecision(false, 130.0, 120.0, 0x00000050u, 0xFFFFFFF0u, W) == GainReport::SkipTooSoon);
    PASS();
}

int main() {
    std::cout << "=== GainReadbackPolicy Tests ===" << std::endl;
    test_force_true_always_sends();
    test_unchanged_value_is_skipped();
    test_too_soon_is_skipped();
    test_window_boundary();
    test_clock_wraparound();
    std::cout << std::endl;
    std::cout << g_passed << " passed, " << g_failed << " failed" << std::endl;
    return g_failed > 0 ? 1 : 0;
}
