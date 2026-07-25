// Standalone test: EscalationTracker — error escalation/de-escalation logic
// Build: cl /EHsc /std:c++17 test_escalation.cpp /Fe:test_escalation.exe
// Run: test_escalation.exe

#include <iostream>
#include <cmath>
#include <windows.h>

#include "../safety/RobotError.h"
#include "../safety/EscalationTracker.h"
#include "../config/Config.h"

static int g_passed = 0, g_failed = 0;

// enum class can't be streamed directly — provide a simple overload
inline std::ostream& operator<<(std::ostream& os, RobotErrorCode c) {
    return os << errorCodeName(c);
}

#define TEST(name) do { std::cout << "  " << #name << "... "; } while(0)
#define PASS() do { std::cout << "PASS" << std::endl; g_passed++; } while(0)
#define CHECK(cond) do { if (!(cond)) { std::cout << "FAIL: " << #cond << std::endl; g_failed++; return; } } while(0)
#define CHECK_EQ(a, b) do { if ((a) != (b)) { std::cout << "FAIL: " << #a << "=" << (a) << " != " << #b << std::endl; g_failed++; return; } } while(0)

// ===== Initial state =====
static void test_initial_state() {
    TEST(initial_state);
    EscalationTracker et;
    CHECK_EQ(et.currentCode, RobotErrorCode::OK);
    CHECK_EQ(et.consecutiveFrames, 0);
    CHECK(!et.escalated);
    CHECK_EQ(et.clearFrames, 0);
    CHECK(!et.isEscalated());
    CHECK_EQ(et.count(), 0);
    PASS();
}

// ===== recordError: first error =====
static void test_record_first_error() {
    TEST(record_first_error);
    EscalationTracker et;
    Vec3 delta = {1.0, 0.0, 0.0};
    et.recordError(RobotErrorCode::ERR_CYLINDRICAL_WARN, delta);
    CHECK_EQ(et.currentCode, RobotErrorCode::ERR_CYLINDRICAL_WARN);
    CHECK_EQ(et.consecutiveFrames, 1);
    CHECK_EQ(et.clearFrames, 0);
    PASS();
}

// ===== recordError: same error accumulates =====
static void test_record_same_error_accumulates() {
    TEST(record_same_error_accumulates);
    EscalationTracker et;
    Vec3 delta = {1.0, 0.0, 0.0};
    et.recordError(RobotErrorCode::ERR_JOINTLIMIT_WARN, delta);
    et.recordError(RobotErrorCode::ERR_JOINTLIMIT_WARN, delta);
    et.recordError(RobotErrorCode::ERR_JOINTLIMIT_WARN, delta);
    CHECK_EQ(et.consecutiveFrames, 3);
    CHECK_EQ(et.currentCode, RobotErrorCode::ERR_JOINTLIMIT_WARN);
    PASS();
}

// ===== recordError: different error resets counter =====
static void test_record_different_error_resets() {
    TEST(record_different_error_resets);
    EscalationTracker et;
    Vec3 delta = {1.0, 0.0, 0.0};
    et.recordError(RobotErrorCode::ERR_CYLINDRICAL_WARN, delta);
    et.recordError(RobotErrorCode::ERR_CYLINDRICAL_WARN, delta);
    CHECK_EQ(et.consecutiveFrames, 2);
    // New error type
    et.recordError(RobotErrorCode::ERR_JOINTLIMIT_WARN, delta);
    CHECK_EQ(et.currentCode, RobotErrorCode::ERR_JOINTLIMIT_WARN);
    CHECK_EQ(et.consecutiveFrames, 1);
    PASS();
}

// ===== shouldEscalate: WARN → DEGRADE after 3 frames =====
static void test_shouldEscalate_warn_to_degrade() {
    TEST(shouldEscalate_warn_to_degrade);
    EscalationTracker et;
    Vec3 delta = {1.0, 0.0, 0.0};

    // WARN severity errors: need 3 consecutive + MIN_WARN_MS elapsed
    et.recordError(RobotErrorCode::ERR_CYLINDRICAL_WARN, delta);
    CHECK(!et.shouldEscalate());  // 1 frame

    et.recordError(RobotErrorCode::ERR_CYLINDRICAL_WARN, delta);
    CHECK(!et.shouldEscalate());  // 2 frames

    // Force time to pass (override m_firstErrorMs)
    et.m_firstErrorMs -= Config::MIN_WARN_MS + 1;

    et.recordError(RobotErrorCode::ERR_CYLINDRICAL_WARN, delta);
    CHECK(et.shouldEscalate());   // 3 frames + time elapsed
    PASS();
}

// ===== shouldEscalate: DEGRADE → REJECT after 10 frames =====
static void test_shouldEscalate_degrade_to_reject() {
    TEST(shouldEscalate_degrade_to_reject);
    EscalationTracker et;
    Vec3 delta = {1.0, 0.0, 0.0};

    // DEGRADE severity: need 10 consecutive + MIN_DEGRADE_MS elapsed
    for (int i = 0; i < 9; i++) {
        et.recordError(RobotErrorCode::ERR_IK_NO_SOLUTION, delta);
        CHECK(!et.shouldEscalate());
    }

    // Force time to pass
    et.m_firstErrorMs -= Config::MIN_DEGRADE_MS + 1;

    et.recordError(RobotErrorCode::ERR_IK_NO_SOLUTION, delta);
    CHECK(et.shouldEscalate());   // 10 frames + time elapsed
    PASS();
}

// ===== shouldEscalate: rejects immediately (no accumulation needed) =====
static void test_shouldEscalate_reject_no_escalation() {
    TEST(shouldEscalate_reject_no_escalation);
    EscalationTracker et;
    Vec3 delta = {1.0, 0.0, 0.0};

    // REJECT severity: shouldEscalate returns false (no escalation beyond REJECT)
    et.recordError(RobotErrorCode::ERR_SAFETY_BOUNDARY, delta);
    CHECK(!et.shouldEscalate());  // REJECT severity — doesn't escalate via counter
    PASS();
}

// ===== shouldEscalate: respects MIN_WARN_MS time threshold =====
static void test_shouldEscalate_respects_min_warn_time() {
    TEST(shouldEscalate_respects_min_warn_time);
    EscalationTracker et;
    Vec3 delta = {1.0, 0.0, 0.0};

    // 5 frames, but time hasn't passed MIN_WARN_MS yet (m_firstErrorMs is "now")
    for (int i = 0; i < 5; i++) {
        et.recordError(RobotErrorCode::ERR_CYLINDRICAL_WARN, delta);
    }
    // Should NOT escalate because MIN_WARN_MS hasn't elapsed
    CHECK(!et.shouldEscalate());
    PASS();
}

// ===== shouldDeescalate: moving away from danger =====
static void test_shouldDeescalate_moving_away() {
    TEST(shouldDeescalate_moving_away);
    EscalationTracker et;
    et.escalated = true;
    et.lastRejectDirection = {0, 1.0, 0};  // was pushing +Y (toward danger)

    // Now moving -Y (away from danger)
    Vec3 delta = {0, -0.5, 0};
    Vec3 dangerDir = {0, 1.0, 0};  // danger is in +Y direction
    CHECK(et.shouldDeescalate(delta, dangerDir));
    PASS();
}

// ===== shouldDeescalate: moving toward danger =====
static void test_shouldDeescalate_moving_toward() {
    TEST(shouldDeescalate_moving_toward);
    EscalationTracker et;
    et.escalated = true;

    // Still moving toward +Y (danger direction)
    Vec3 delta = {0, 0.5, 0};
    Vec3 dangerDir = {0, 1.0, 0};
    CHECK(!et.shouldDeescalate(delta, dangerDir));
    PASS();
}

// ===== shouldDeescalate: not escalated =====
static void test_shouldDeescalate_not_escalated() {
    TEST(shouldDeescalate_not_escalated);
    EscalationTracker et;
    // Not escalated → no de-escalation possible
    Vec3 delta = {0, -1.0, 0};
    Vec3 dangerDir = {0, 1.0, 0};
    CHECK(!et.shouldDeescalate(delta, dangerDir));
    PASS();
}

// ===== onClear: resets after CLEAR_FRAMES =====
static void test_onClear_resets_after_threshold() {
    TEST(onClear_resets_after_threshold);
    EscalationTracker et;
    Vec3 delta = {1.0, 0.0, 0.0};

    // Record an error first
    et.recordError(RobotErrorCode::ERR_CYLINDRICAL_WARN, delta);
    CHECK_EQ(et.currentCode, RobotErrorCode::ERR_CYLINDRICAL_WARN);

    // Clear frames
    for (int i = 0; i < EscalationTracker::CLEAR_FRAMES; i++) {
        et.onClear();
    }
    CHECK_EQ(et.currentCode, RobotErrorCode::OK);
    CHECK_EQ(et.consecutiveFrames, 0);
    CHECK(!et.escalated);
    PASS();
}

// ===== onClear: error re-triggers before clear threshold =====
static void test_onClear_interrupted_by_error() {
    TEST(onClear_interrupted_by_error);
    EscalationTracker et;
    Vec3 delta = {1.0, 0.0, 0.0};

    et.recordError(RobotErrorCode::ERR_CYLINDRICAL_WARN, delta);

    // Clear partially
    for (int i = 0; i < 10; i++) {
        et.onClear();
    }
    CHECK(et.clearFrames == 10);

    // Error re-appears → clear counter reset
    et.recordError(RobotErrorCode::ERR_CYLINDRICAL_WARN, delta);
    CHECK_EQ(et.clearFrames, 0);
    PASS();
}

// ===== reset: full state cleanup =====
static void test_reset_full_cleanup() {
    TEST(reset_full_cleanup);
    EscalationTracker et;
    Vec3 delta = {1.0, 0.0, 0.0};

    et.recordError(RobotErrorCode::ERR_SAFETY_BOUNDARY, delta);
    et.recordError(RobotErrorCode::ERR_SAFETY_BOUNDARY, delta);
    et.escalated = true;
    et.clearFrames = 5;

    et.reset();

    CHECK_EQ(et.currentCode, RobotErrorCode::OK);
    CHECK_EQ(et.consecutiveFrames, 0);
    CHECK(!et.escalated);
    CHECK_EQ(et.clearFrames, 0);
    CHECK_EQ(et.m_firstErrorMs, 0);
    CHECK(!et.isEscalated());
    PASS();
}

// ===== getSeverity: verify all error codes have valid severity =====
static void test_all_error_codes_have_severity() {
    TEST(all_error_codes_have_severity);
    // Spot-check a few representative codes
    CHECK(getSeverity(RobotErrorCode::ERR_CONNECTION_LOST) == Severity::FATAL);
    CHECK(getSeverity(RobotErrorCode::ERR_SAFETY_BOUNDARY) == Severity::REJECT);
    CHECK(getSeverity(RobotErrorCode::ERR_IK_NO_SOLUTION) == Severity::DEGRADE);
    CHECK(getSeverity(RobotErrorCode::ERR_CYLINDRICAL_WARN) == Severity::WARN);
    CHECK(getSeverity(RobotErrorCode::OK) == Severity::INFO);
    PASS();
}

// ===== MAIN =====
int main() {
    std::cout << "=== EscalationTracker Tests ===" << std::endl;

    test_initial_state();
    test_record_first_error();
    test_record_same_error_accumulates();
    test_record_different_error_resets();
    test_shouldEscalate_warn_to_degrade();
    test_shouldEscalate_degrade_to_reject();
    test_shouldEscalate_reject_no_escalation();
    test_shouldEscalate_respects_min_warn_time();
    test_shouldDeescalate_moving_away();
    test_shouldDeescalate_moving_toward();
    test_shouldDeescalate_not_escalated();
    test_onClear_resets_after_threshold();
    test_onClear_interrupted_by_error();
    test_reset_full_cleanup();
    test_all_error_codes_have_severity();

    std::cout << std::endl;
    std::cout << g_passed << " passed, " << g_failed << " failed" << std::endl;
    return g_failed > 0 ? 1 : 0;
}
