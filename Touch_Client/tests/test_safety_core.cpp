// Standalone test: Safety core — state machine, escalation, callbacks
// Build: cl /EHsc /std:c++17 test_safety_core.cpp
//        ../safety/RobotStateMachine.cpp ../safety/RobotDiagnostics.cpp
//        /I"..\..\OpenHaptics\Developer\3.5.0\include"
//        /I"..\..\OpenHaptics\Developer\3.5.0\utilities\include"
//        /Fe:test_safety_core.exe
//        /link /SUBSYSTEM:CONSOLE
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
    Sleep(60);  // ensure MIN_WARN_MS (50ms) is satisfied after first error
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
    Sleep(60);  // ensure MIN_WARN_MS (50ms) is satisfied after first error
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
    Sleep(60);  // ensure MIN_WARN_MS (50ms) is satisfied after first error
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

    // Sleep to satisfy MIN_WARN_MS
    Sleep(60);

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
    Sleep(60);
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

    std::cout << "\nResults: " << g_passed << " passed, " << g_failed << " failed" << std::endl;
    return g_failed ? 1 : 0;
}
