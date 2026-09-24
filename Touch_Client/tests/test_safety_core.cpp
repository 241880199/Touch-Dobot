// Standalone test: Safety core — state machine, escalation, callbacks
// Build: cl /EHsc /std:c++17 test_safety_core.cpp
//        ../safety/RobotStateMachine.cpp ../safety/RobotDiagnostics.cpp
//        /I"..\..\OpenHaptics\Developer\3.5.0\include"
//        /I"..\..\OpenHaptics\Developer\3.5.0\utilities\include"
//        /Fe:test_safety_core.exe
//        /link /SUBSYSTEM:CONSOLE
// ★ 2026-09-22: 【必须】加 /DTEST_NO_RELAY_CORE —— 缺了它会 LNK2019, 无法解析的外部符号
//   RelayCore::instance / RelayCore::reportDiagnostic (RobotDiagnostics.cpp 里那条 D| 转发)。
//   先例: test_singularity_avoidance.cpp:3 就是在测试源里写明了要 define TEST_SINGAVOID。
//   ⚠ 正式构建脚本是 build_safety_core_test.bat (它带着这个宏); 上面这份手抄配方从前漏了它,
//     照上面构建就会把那个洞重新打开一遍。
// Run: test_safety_core.exe

#include <iostream>
#include <cmath>
#include <windows.h>

// Project headers
#include "../safety/RobotStateMachine.h"
#include "../safety/RobotDiagnostics.h"
#include "../config/Config.h"

static int g_passed = 0, g_failed = 0;

#define TEST(name) do { std::cout << "  " << #name << "... "; } while(0)
#define PASS() do { std::cout << "PASS" << std::endl; g_passed++; } while(0)
#define CHECK(cond) do { if (!(cond)) { std::cout << "FAIL: " << #cond << std::endl; g_failed++; return; } } while(0)

// ★★ 2026-09-24: 时间【注入】, 不睡觉。
//
// 【为什么不再 Sleep】: 判据是 `elapsed >= Config::MIN_WARN_MS`(=50ms)，而 elapsed 由
//   `GetTickCount()` 得到 —— 它的粒度是 15.6ms。从前本文件靠"睡 4 × MIN_WARN_MS = 200ms"
//   把余量拉开，代价是 (a) 每次运行白花 ~1s，(b) 判据对 MIN_WARN_MS ∈ (0, 200] 全都不敏感
//   —— 改到 150 也照样绿。下侧边界（差 1ms 不升级）更是无从谈起。
// 【那件事的如实记录】: 2026-09-22 记的是"实测 60 次里红 5 次（≈8%）"。2026-09-24 复核：
//   加了 200ms 余量之后**跑 180 次零红** ⇒ 那条 flake 已经不存在了。所以本轮不是"修 flake",
//   而是把时间做成【确定的输入】、并把阈值两侧补上。
// 【为什么现在能用注入】: 从前这里写着"`RobotStateMachine` 的 `m_escalation` 是私有的 ⇒
//   用不了那一招" —— **那句是假的**。`RobotStateMachine::escalation()`
//   (safety/RobotStateMachine.h:63) 在 public 段里 (private 从 66 行才开始)，返回可写引用。
// 【代价】: 这要直接写 `m_firstErrorMs` —— public 字段, 但语义上是内部量, 属【测试专用写】。
//   先例: 同目录 `test_escalation.cpp:92` 一直在这么做 (`et.m_firstErrorMs -= MIN_WARN_MS + 1`)。

// ===== Test 1: State machine standard transition chain =====
static void test_state_machine_transition_chain() {
    TEST(state_transition_chain);
    RobotStateMachine sm;

    CHECK(sm.currentState() == RobotState::DISCONNECTED);

    sm.onConnect();
    CHECK(sm.currentState() == RobotState::CONNECTED);

    sm.onEnableSuccess();
    CHECK(sm.currentState() == RobotState::READY);

    sm.onButtonPress();
    CHECK(sm.currentState() == RobotState::RUNNING);

    // 3 consecutive WARN errors should trigger DEGRADED (with time threshold)
    Vec3 delta = {1, 0, 0};
    RobotError warnErr;
    warnErr.code = RobotErrorCode::ERR_CYLINDRICAL_WARN;
    warnErr.severity = Severity::WARN;
    warnErr.timestampMs = GetTickCount64();

    sm.onError(warnErr, delta);
    sm.escalation().m_firstErrorMs -= (Config::MIN_WARN_MS + 1);  // 时间注入, 见文件顶部
    sm.onError(warnErr, delta);
    sm.onError(warnErr, delta);
    CHECK(sm.currentState() == RobotState::DEGRADED);

    sm.onRecovery();
    CHECK(sm.currentState() == RobotState::RUNNING);

    PASS();
}

// ===== Test 2: FATAL callback fires =====
static int g_fatalCallbackCount = 0;
static void fatalCallback() { g_fatalCallbackCount++; }

static void test_fatal_callback() {
    TEST(fatal_callback);
    RobotStateMachine sm;
    g_fatalCallbackCount = 0;

    sm.setFatalCallback(fatalCallback);
    sm.onConnect();
    sm.onEnableSuccess();

    // Trigger FATAL via enable fail
    sm.onEnableFail();
    CHECK(sm.currentState() == RobotState::FATAL);
    CHECK(g_fatalCallbackCount == 1);

    // Re-triggering FATAL should NOT fire callback again (already FATAL)
    sm.onEnableFail();
    CHECK(g_fatalCallbackCount == 1);

    PASS();
}

// ===== Test 3: canMove guard per state =====
static void test_can_move_guard() {
    TEST(can_move_guard);

    RobotStateMachine sm;
    CHECK(!sm.canMove());  // DISCONNECTED

    sm.onConnect();
    CHECK(!sm.canMove());  // CONNECTED

    sm.onEnableSuccess();
    CHECK(!sm.canMove());  // READY

    sm.onButtonPress();
    CHECK(sm.canMove());   // RUNNING

    // DEGRADED still allows motion
    Vec3 delta = {0, 0, 0};
    RobotError warnErr;
    warnErr.code = RobotErrorCode::ERR_CYLINDRICAL_WARN;
    warnErr.severity = Severity::WARN;
    warnErr.timestampMs = GetTickCount64();

    sm.onError(warnErr, delta);
    sm.escalation().m_firstErrorMs -= (Config::MIN_WARN_MS + 1);  // 时间注入, 见文件顶部
    sm.onError(warnErr, delta);
    sm.onError(warnErr, delta);
    CHECK(sm.currentState() == RobotState::DEGRADED);
    CHECK(sm.canMove());   // DEGRADED can still move

    sm.onDisconnect();
    CHECK(!sm.canMove());  // RECOVERING / DISCONNECTED

    PASS();
}

// ===== Test 4: speedFactor per state =====
static void test_speed_factor() {
    TEST(speed_factor);

    RobotStateMachine sm;
    sm.onConnect();
    sm.onEnableSuccess();
    sm.onButtonPress();
    CHECK(fabs(sm.speedFactor() - 1.0) < 0.01);  // RUNNING = 1.0

    // DEGRADED = 0.3
    Vec3 delta = {0, 0, 0};
    RobotError warnErr;
    warnErr.code = RobotErrorCode::ERR_CYLINDRICAL_WARN;
    warnErr.severity = Severity::WARN;
    warnErr.timestampMs = GetTickCount64();

    sm.onError(warnErr, delta);
    sm.escalation().m_firstErrorMs -= (Config::MIN_WARN_MS + 1);  // 时间注入, 见文件顶部
    sm.onError(warnErr, delta);
    sm.onError(warnErr, delta);
    CHECK(fabs(sm.speedFactor() - 0.3) < 0.01);

    // FATAL = 0.0
    RobotError fatalErr;
    fatalErr.code = RobotErrorCode::ERR_EMERGENCY_STOP;
    fatalErr.severity = Severity::FATAL;
    fatalErr.timestampMs = GetTickCount64();
    sm.onError(fatalErr, delta);
    CHECK(fabs(sm.speedFactor() - 0.0) < 0.01);

    PASS();
}

// ===== Test 5: Escalation: 3 frames WARN → DEGRADE =====
static void test_escalation_warn_to_degrade() {
    TEST(escalation_warn_to_degrade);

    EscalationTracker et;
    Vec3 delta = {1, 0, 0};

    CHECK(!et.shouldEscalate());

    et.recordError(RobotErrorCode::ERR_CYLINDRICAL_WARN, delta);
    CHECK(!et.shouldEscalate());  // 1 frame, not enough

    et.m_firstErrorMs -= (Config::MIN_WARN_MS + 1);   // 时间注入, 见文件顶部

    et.recordError(RobotErrorCode::ERR_CYLINDRICAL_WARN, delta);
    et.recordError(RobotErrorCode::ERR_CYLINDRICAL_WARN, delta);
    CHECK(et.shouldEscalate());  // 3 frames + 50ms elapsed

    PASS();
}

// ===== Test 6: Escalation: different error resets counter =====
static void test_escalation_different_error_resets() {
    TEST(escalation_different_error_resets);

    EscalationTracker et;
    Vec3 delta = {0, 0, 0};

    et.recordError(RobotErrorCode::ERR_CYLINDRICAL_WARN, delta);
    CHECK(et.count() == 1);
    CHECK(et.currentCode == RobotErrorCode::ERR_CYLINDRICAL_WARN);

    et.recordError(RobotErrorCode::ERR_JOINTLIMIT_WARN, delta);
    CHECK(et.count() == 1);  // reset to 1 for new error
    CHECK(et.currentCode == RobotErrorCode::ERR_JOINTLIMIT_WARN);

    PASS();
}

// ===== Test 7: De-escalation: reverse motion =====
static void test_deescalation_reverse_motion() {
    TEST(deescalation_reverse_motion);

    EscalationTracker et;
    Vec3 dangerDir = {1, 0, 0};  // danger is in +X direction
    Vec3 towardDanger = {1, 0, 0};
    Vec3 awayFromDanger = {-1, 0, 0};

    // Record an error to get escalated state
    et.recordError(RobotErrorCode::ERR_CYLINDRICAL_WARN, dangerDir);
    et.m_firstErrorMs -= (Config::MIN_WARN_MS + 1);   // 时间注入, 见文件顶部
    et.recordError(RobotErrorCode::ERR_CYLINDRICAL_WARN, dangerDir);
    et.recordError(RobotErrorCode::ERR_CYLINDRICAL_WARN, dangerDir);
    et.escalated = true;

    // Moving toward danger should NOT de-escalate (dot > 0)
    CHECK(!et.shouldDeescalate(towardDanger, dangerDir));

    // Moving away from danger SHOULD de-escalate (dot < 0)
    CHECK(et.shouldDeescalate(awayFromDanger, dangerDir));

    PASS();
}

// ===== Test 8: De-escalation: 30-frame clear =====
static void test_deescalation_clear_frames() {
    TEST(deescalation_clear_frames);

    EscalationTracker et;
    et.escalated = true;
    et.currentCode = RobotErrorCode::ERR_CYLINDRICAL_WARN;
    et.consecutiveFrames = 5;

    for (int i = 0; i < 30; i++) {
        et.onClear();
    }
    // After 30 clear frames, should be fully reset
    CHECK(!et.isEscalated());
    CHECK(et.count() == 0);
    CHECK(et.currentCode == RobotErrorCode::OK);

    PASS();
}

// ===== Test 9: MIN_WARN_MS 的【两侧边界】=====
//
// 【为什么必须有它】: 本文件从前靠"睡 4 × MIN_WARN_MS = 200ms"去满足 50ms 的规则 ——
//   余量 4 倍 ⇒ 判据对 MIN_WARN_MS ∈ (0, 200] 全都不敏感, 改到 150 也照样绿。
//   下面两条把阈值本身钉到 1ms: 差 1ms 不升级, 正好到点升级。
// 【怎么做到不用睡】: 直接注入"距第一次出错过去了多久"——
//   注入的确实是【差】, 但那个差是减在 `recordError()` 内部那次 `GetTickCount()`
//   读数上的 ⇒ 下侧断言看到的 elapsed 实际是 `49 + (T'-T)`, 其中 T'-T 是之后
//   `shouldEscalate()` 里再读一次时钟的增量, 只可能取 0 或一个 15.6ms 量子
//   ⇒ 落到 49 还是 65 取决于这中间有没有跨过刻度 ⇒ 下侧仍留一道残留刀口:
//   **实测 ≈5e-7/次 (2,000,000 次重放里 1 次), 不是 0**。
//   也正是这个量子决定了本时钟下做不出"正好差 1ms"的夹逼 ⇒ 下面这两条已经是
//   在这里能写下的最紧的确定说法。
// 【构造上抓不到"常数取值错"】: 本用例读的就是实现读的那个 `Config::MIN_WARN_MS`
//   ⇒ 把 50 改成别的值, 它会跟着改、照样绿。它钉住的是【阈值处的那次比较】,
//   不是 50 这个数本身 (那个数在别处钉)。
static void test_escalation_boundary_at_min_warn_ms() {
    TEST(escalation_boundary_at_min_warn_ms);

    // 本用例的算术假设: MIN_WARN_MS >= 2 才谈得上"差 1ms"这一侧。
    CHECK(Config::MIN_WARN_MS > 1);

    Vec3 delta = {1, 0, 0};
    EscalationTracker et;
    et.recordError(RobotErrorCode::ERR_CYLINDRICAL_WARN, delta);
    et.recordError(RobotErrorCode::ERR_CYLINDRICAL_WARN, delta);
    et.recordError(RobotErrorCode::ERR_CYLINDRICAL_WARN, delta);
    CHECK(et.count() == 3);                       // 帧数够了, 只差时间

    et.m_firstErrorMs -= (Config::MIN_WARN_MS - 1);   // 差 1ms
    CHECK(!et.shouldEscalate());                      // ★ 不升级 (下侧)

    et.m_firstErrorMs -= 1;                           // 正好到点
    CHECK(et.shouldEscalate());                       // ★ 升级 (阈值本身)

    PASS();
}

int main() {
    std::cout << "=== Safety Core Unit Tests ===" << std::endl;
    test_state_machine_transition_chain();
    test_fatal_callback();
    test_can_move_guard();
    test_speed_factor();
    test_escalation_warn_to_degrade();
    test_escalation_different_error_resets();
    test_deescalation_reverse_motion();
    test_deescalation_clear_frames();
    test_escalation_boundary_at_min_warn_ms();

    std::cout << "\nResults: " << g_passed << " passed, " << g_failed << " failed" << std::endl;
    return g_failed ? 1 : 0;
}
