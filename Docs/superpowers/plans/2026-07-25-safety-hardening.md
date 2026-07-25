# Safety Hardening Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Close 5 fault-tolerance gaps in the robot safety system: NaN input bypass, missing haptic watchdog, FATAL without hardware stop, frame-count-based escalation, and zero test coverage on core safety logic.

**Architecture:** Each fix is isolated to its module. No cross-module refactoring. The watchdog uses a dual-layer approach (GLUT idle() primary + ForceReader thread fallback). The FATAL callback keeps dependency direction correct (RelayCore → safety, not safety → relay). Time-based escalation adds a `GetTickCount()` guard alongside existing frame counters.

**Tech Stack:** C++17, Windows CRITICAL_SECTION, MSVC Release x64, GLUT, HDAPI

**Spec:** `docs/superpowers/specs/2026-07-25-safety-hardening-design.md`

## Global Constraints

- Build: MSVC Release x64, 0 errors, 0 warnings
- All existing tests (12/12) must continue to pass
- No API changes to existing functions
- No new dependencies or libraries
- `exit(0)` bypass issue (main.cpp:130) is out of scope — do not fix
- `m_alarmList` thread-safety (known pre-existing) is out of scope — do not fix
- SafetyPredictor NaN test omitted (requires Kinematics/SafetyBoundary linkage)

---

### Task 1: Config.h — add watchdog and escalation timing constants

**Files:**
- Modify: `Touch_Client/config/Config.h` (append before closing brace)

**Interfaces:**
- Produces: `Config::WATCHDOG_TIMEOUT_MS` (int, 200), `Config::MIN_WARN_MS` (int, 50), `Config::MIN_DEGRADE_MS` (int, 200)

- [ ] **Step 1: Add constants to Config.h**

In `Touch_Client/config/Config.h`, after line 80 (`const int DEESCALATE_CLEAR_FRAMES = 30;`) and before the diagnostic section, insert:

```cpp
    // ========== 看门狗参数 ==========
    const int WATCHDOG_TIMEOUT_MS = 200;     // 触觉线程看门狗超时 (ms)

    // ========== 升级时间阈值 ==========
    const int MIN_WARN_MS    = 50;           // WARN 至少持续 50ms 才能升级到 DEGRADE
    const int MIN_DEGRADE_MS = 200;          // DEGRADE 至少持续 200ms 才能升级到 REJECT
```

- [ ] **Step 2: Build to verify no breakage**

Run: `cd D:\Projects\Touch && bash build.sh`
Expected: Build succeeds, 0 errors, 0 warnings.

- [ ] **Step 3: Run existing tests**

Run: `cd D:\Projects\Touch && bash build.sh test`
Expected: 12/12 tests pass.

- [ ] **Step 4: Commit**

```bash
git add Touch_Client/config/Config.h
git commit -m "feat(config): add watchdog timeout and escalation timing constants"
```

---

### Task 2: EscalationTracker.h — time-based escalation guard

**Files:**
- Modify: `Touch_Client/safety/EscalationTracker.h`

**Interfaces:**
- Consumes: `Config::MIN_WARN_MS`, `Config::MIN_DEGRADE_MS` (from Task 1)
- Produces: `EscalationTracker::m_firstErrorMs` (DWORD), modified `recordError()`, modified `shouldEscalate()`, modified `reset()`

- [ ] **Step 1: Add m_firstErrorMs member and modify recordError()**

In `Touch_Client/safety/EscalationTracker.h`, the struct currently starts at line 9. Add `#include <windows.h>` at the top of the file if not already present (check first — `RobotStateMachine.h` includes `<windows.h>` but EscalationTracker.h does not).

Add `<windows.h>` include after `#include "../relay/CoordinateTransform.h"`:

```cpp
#pragma once
#include <cmath>
#include <windows.h>
#include "RobotError.h"
#include "../relay/CoordinateTransform.h"
```

Add `DWORD m_firstErrorMs = 0;` after `int clearFrames = 0;` (after line 14):

```cpp
    int clearFrames = 0;  // 错误清除后的帧计数
    DWORD m_firstErrorMs = 0;  // 第一次出错的时间戳 (for time-based escalation)
```

Modify `recordError()` — change the `else` branch to also set `m_firstErrorMs`:

```cpp
    void recordError(RobotErrorCode code, const Vec3& delta) {
        if (code == currentCode) {
            consecutiveFrames++;
        } else {
            currentCode = code;
            consecutiveFrames = 1;
            m_firstErrorMs = GetTickCount();  // 新错误类型，重新计时
        }
        clearFrames = 0;
        lastRejectDirection = delta;
    }
```

- [ ] **Step 2: Modify shouldEscalate() to check time threshold**

Replace the current `shouldEscalate()`:

```cpp
    bool shouldEscalate() const {
        Severity sev = getSeverity(currentCode);
        DWORD elapsed = GetTickCount() - m_firstErrorMs;
        if (sev == Severity::WARN && consecutiveFrames >= WARN_TO_DEGRADE
            && elapsed >= Config::MIN_WARN_MS)
            return true;
        if (sev == Severity::DEGRADE && consecutiveFrames >= DEGRADE_TO_REJECT
            && elapsed >= Config::MIN_DEGRADE_MS)
            return true;
        return false;
    }
```

Note: The constants `WARN_TO_DEGRADE` and `DEGRADE_TO_REJECT` are already defined as `static constexpr int` in the struct (lines 16-17). The Config constants `MIN_WARN_MS` and `MIN_DEGRADE_MS` are in the `Config` namespace — reference them as `Config::MIN_WARN_MS` and `Config::MIN_DEGRADE_MS`. Need to add `#include "../config/Config.h"` at the top of EscalationTracker.h.

Add the include:
```cpp
#pragma once
#include <cmath>
#include <windows.h>
#include "RobotError.h"
#include "../relay/CoordinateTransform.h"
#include "../config/Config.h"
```

- [ ] **Step 3: Modify reset() to clear m_firstErrorMs**

In `reset()`:

```cpp
    void reset() {
        currentCode = RobotErrorCode::OK;
        consecutiveFrames = 0;
        escalated = false;
        clearFrames = 0;
        m_firstErrorMs = 0;
        lastRejectDirection = {0, 0, 0};
    }
```

- [ ] **Step 4: Build and run tests**

Run: `cd D:\Projects\Touch && bash build.sh && bash build.sh test`
Expected: Build 0 errors 0 warnings, 12/12 tests pass.

- [ ] **Step 5: Commit**

```bash
git add Touch_Client/safety/EscalationTracker.h
git commit -m "feat(escalation): add time-based escalation guard — MIN_WARN_MS + MIN_DEGRADE_MS thresholds"
```

---

### Task 3: RobotStateMachine — FATAL callback mechanism

**Files:**
- Modify: `Touch_Client/safety/RobotStateMachine.h`
- Modify: `Touch_Client/safety/RobotStateMachine.cpp`
- Modify: `Touch_Client/relay/RelayCore.cpp`

**Interfaces:**
- Produces: `RobotStateMachine::setFatalCallback(FatalCallback cb)`, `FatalCallback` typedef
- Consumed by: `RelayCore::init()` registers DisableRobot callback

- [ ] **Step 1: Add FatalCallback typedef and setter to header**

In `Touch_Client/safety/RobotStateMachine.h`, in the public section, add after the `transitionTo` declaration (~line 54):

```cpp
    // FATAL 回调 (FATAL 状态进入时自动调用)
    using FatalCallback = void(*)();
    void setFatalCallback(FatalCallback cb) { m_onFatal = cb; }
```

In the private section, add after `mutable CRITICAL_SECTION m_lock;` (~line 63):

```cpp
    FatalCallback m_onFatal = nullptr;
```

- [ ] **Step 2: Modify transitionTo() to invoke callback on FATAL**

In `Touch_Client/safety/RobotStateMachine.cpp`, modify `transitionTo()`. The current code is at lines 199-206:

```cpp
void RobotStateMachine::transitionTo(RobotState newState) {
    if (m_state == newState) return;
    const char* from = stateName(m_state);
    const char* to = stateName(newState);
    std::cout << "[StateMachine] " << from << " → " << to << std::endl;
    RobotDiagnostics::instance().logStateChange(m_state, newState);
    m_state = newState;
}
```

Replace with:

```cpp
void RobotStateMachine::transitionTo(RobotState newState) {
    if (m_state == newState) return;
    const char* from = stateName(m_state);
    const char* to = stateName(newState);
    std::cout << "[StateMachine] " << from << " → " << to << std::endl;
    RobotDiagnostics::instance().logStateChange(m_state, newState);
    m_state = newState;

    // FATAL 回调：在锁外调用以避免回调中的 I/O 阻塞状态机
    if (newState == RobotState::FATAL && m_onFatal) {
        FatalCallback cb = m_onFatal;
        LeaveCriticalSection(&m_lock);
        cb();
        EnterCriticalSection(&m_lock);
    }
}
```

Note: `transitionTo()` may be called from within methods that already hold `m_lock` (like `onError`, `onRecovery`, etc.). Windows CRITICAL_SECTION is reentrant — same thread can enter multiple times. The `LeaveCriticalSection`/`EnterCriticalSection` pair around the callback is safe because: (1) same thread still owns the lock after Leave, (2) callback doesn't call back into state machine, (3) re-enter is guaranteed to succeed on the same thread.

- [ ] **Step 3: Register FATAL callback in RelayCore::init()**

In `Touch_Client/relay/RelayCore.cpp`, in `RelayCore::init()`, add after the `m_stateMachine.onConnect();` line (~line 280) and before the ClearError sequence:

```cpp
    // Register FATAL callback: disable robot hardware on fatal error
    m_stateMachine.setFatalCallback([]() {
        robotSendEnable("DisableRobot()");
        Sleep(100);
        std::cerr << "[Safety] FATAL: robot disabled by state machine" << std::endl;
    });
```

Add `#include <iostream>` at the top of RelayCore.cpp if not already present (check first — it's likely already there).

- [ ] **Step 4: Build and run tests**

Run: `cd D:\Projects\Touch && bash build.sh && bash build.sh test`
Expected: Build 0 errors 0 warnings, 12/12 tests pass.

- [ ] **Step 5: Commit**

```bash
git add Touch_Client/safety/RobotStateMachine.h Touch_Client/safety/RobotStateMachine.cpp Touch_Client/relay/RelayCore.cpp
git commit -m "feat(safety): add FATAL callback — state machine triggers DisableRobot on fatal transition"
```

---

### Task 4: NaN/Inf input guards

**Files:**
- Modify: `Touch_Client/relay/RelayCore.h`
- Modify: `Touch_Client/relay/RelayCore.cpp`
- Modify: `Touch_Client/safety/SafetyPredictor.cpp`

**Interfaces:**
- Produces: `RelayCore::m_nanFrameCount` (int, private)
- Consumes: `RobotErrorCode::ERR_EMERGENCY_STOP` (existing), `SafetyPredictor::evaluate()` goto label (existing from C1 fix)

- [ ] **Step 1: Add m_nanFrameCount to RelayCore.h**

In `Touch_Client/relay/RelayCore.h`, in the private section, add after `bool m_heartbeatLostReported = false;` (~line 83):

```cpp
    int m_nanFrameCount = 0;  // 连续 NaN 帧计数 (>=3 → FATAL)
```

- [ ] **Step 2: Add NaN guard at start of RelayCore::sendPosition()**

In `Touch_Client/relay/RelayCore.cpp`, in `sendPosition()`, add right after the `// ===== 增量式位移` comment block (after `Vec3 current = convertTouchToRobot(devicePos);` around line 406, before `EnterCriticalSection(&m_basePointLock);` around line 407):

No — add it right after computing `dx/dy/dz` (after line 419) and BEFORE the noise gate check (line 425). The NaN check must happen before any use of dx/dy/dz.

In `sendPosition()`, after computing `dx`, `dy`, `dz` (around line 419):
```cpp
    // 更新 Touch 参考点
    m_lastTouchPos = current;
```

Insert NaN guard between the `m_lastTouchPos = current;` line and the noise gate check:

```cpp
    // 更新 Touch 参考点
    m_lastTouchPos = current;

    // NaN/Inf guard: 连续 3 帧异常 → FATAL
    if (std::isnan(dx) || std::isnan(dy) || std::isnan(dz) ||
        std::isinf(dx) || std::isinf(dy) || std::isinf(dz)) {
        m_nanFrameCount++;
        if (m_nanFrameCount >= 3) {
            RobotError error;
            error.code = RobotErrorCode::ERR_EMERGENCY_STOP;
            error.severity = Severity::FATAL;
            error.timestampMs = GetTickCount64();
            Vec3 zeroDelta = {0, 0, 0};
            m_stateMachine.onError(error, zeroDelta);
        }
        LeaveCriticalSection(&m_basePointLock);
        return;
    }
    m_nanFrameCount = 0;  // 正常帧清零

    // 跳过微小增量 (Touch 噪声)
```

Add `#include <cmath>` at the top of RelayCore.cpp if not already present.

- [ ] **Step 3: Add NaN guard at start of SafetyPredictor::evaluate()**

In `Touch_Client/safety/SafetyPredictor.cpp`, in `evaluate()`, add right after the `// ===== Layer 1: hard boundaries (O(1) compute) =====` comment and the `Vec3 clamped;` declaration (~line 31-32):

```cpp
SafetyVerdict SafetyPredictor::evaluate(const Vec3& target) {
    // ===== Layer 1: hard boundaries (O(1) compute) =====
    Vec3 clamped;

    // NaN/Inf 输入防护 (最后防线)
    if (std::isnan(target.x) || std::isnan(target.y) || std::isnan(target.z) ||
        std::isinf(target.x) || std::isinf(target.y) || std::isinf(target.z)) {
        m_lastVerdict.action = SafetyVerdict::REJECT;
        m_lastVerdict.errorCode = RobotErrorCode::ERR_EMERGENCY_STOP;
        m_lastVerdict.reason = "NaN/Inf target position";
        m_lastVerdict.speedFactor = 0.0;
        goto evaluate_done;
    }

    // 1a. workspace radius
```

Add `#include <cmath>` at the top if not already present.

- [ ] **Step 4: Build and run tests**

Run: `cd D:\Projects\Touch && bash build.sh && bash build.sh test`
Expected: Build 0 errors 0 warnings, 12/12 tests pass.

- [ ] **Step 5: Commit**

```bash
git add Touch_Client/relay/RelayCore.h Touch_Client/relay/RelayCore.cpp Touch_Client/safety/SafetyPredictor.cpp
git commit -m "feat(safety): add NaN/Inf input guards — 3-frame threshold, FATAL escalation, evaluate() final defense"
```

---

### Task 5: Haptic watchdog — dual-layer (GLUT + ForceReader)

**Files:**
- Modify: `Touch_Client/relay/RelayCore.h`
- Modify: `Touch_Client/relay/RelayCore.cpp`
- Modify: `Touch_Client/main.cpp`

**Interfaces:**
- Consumes: `Config::WATCHDOG_TIMEOUT_MS` (from Task 1), FATAL callback (from Task 3)
- Produces: `RelayCore::checkHapticWatchdog()`, `RelayCore::lastHapticFrameMs()` (public, const)

- [ ] **Step 1: Add watchdog members to RelayCore.h**

In `Touch_Client/relay/RelayCore.h`, after `bool m_nanFrameCount = 0;` (added in Task 4), add:

```cpp
    // 看门狗
    std::atomic<DWORD> m_lastHapticFrameMs{0};
    bool m_watchdogTripped = false;
```

Add public accessor near the other accessors (after `isTransmitting()`, around line 50):

```cpp
    // 看门狗状态查询
    DWORD lastHapticFrameMs() const { return m_lastHapticFrameMs.load(); }
    void checkHapticWatchdog();
```

- [ ] **Step 2: Update m_lastHapticFrameMs in sendPosition()**

In `Touch_Client/relay/RelayCore.cpp`, in `sendPosition()`, add as the very first line after the early-return checks (after `if (!m_transmitting || !m_basePointSet || !isRobotConnected()) return;` and `if (!appState.isRobotBaseSet) return;`, around line 397):

```cpp
    // 更新触觉线程心跳时间戳
    m_lastHapticFrameMs = GetTickCount();
```

Place this right after the two early-return checks and before the ServoP rate limiter.

- [ ] **Step 3: Implement checkHapticWatchdog()**

In `Touch_Client/relay/RelayCore.cpp`, add new method after `queryJointAngles()` (before `pingRobot()`, around line 699):

```cpp
void RelayCore::checkHapticWatchdog() {
    if (!m_transmitting.load()) return;  // 未运动时不检查
    DWORD now = GetTickCount();
    DWORD lastFrame = m_lastHapticFrameMs.load();
    if (lastFrame > 0 && (now - lastFrame) > (DWORD)Config::WATCHDOG_TIMEOUT_MS) {
        if (!m_watchdogTripped) {
            m_watchdogTripped = true;
            std::cerr << "[Safety] WATCHDOG: haptic thread silent for "
                      << (now - lastFrame) << "ms — triggering FATAL" << std::endl;
            RobotError error;
            error.code = RobotErrorCode::ERR_EMERGENCY_STOP;
            error.severity = Severity::FATAL;
            error.timestampMs = GetTickCount64();
            Vec3 zeroDelta = {0, 0, 0};
            m_stateMachine.onError(error, zeroDelta);
            // FATAL callback (registered in init) will DisableRobot()
        }
    }
}
```

- [ ] **Step 4: Add ForceReader fallback watchdog**

In `Touch_Client/relay/RelayCore.cpp`, in the `forceReaderThread` static function (~line 14), inside the `while (!app.isClosing)` loop, add after the force data parsing block. The current code parses force data at lines 36-44. Add a periodic check:

After the `LeaveCriticalSection(&app.forceDataMutex);` line (~line 44), inside the inner while loop, add:

```cpp
            LeaveCriticalSection(&app.forceDataMutex);

            // 看门狗兜底: 每 300ms 检查一次 (GLUT 可能已死)
            static DWORD lastWatchdogCheck = 0;
            DWORD now = GetTickCount();
            if (now - lastWatchdogCheck > 300) {
                lastWatchdogCheck = now;
                auto& relay = RelayCore::instance();
                DWORD lastHaptic = relay.lastHapticFrameMs();
                if (relay.isTransmitting() && lastHaptic > 0 &&
                    (now - lastHaptic) > (DWORD)(Config::WATCHDOG_TIMEOUT_MS * 2)) {
                    std::cerr << "[Safety] ForceReader WATCHDOG: GLUT appears dead ("
                              << (now - lastHaptic) << "ms since last haptic frame) — "
                              << "sending EmergencyStop" << std::endl;
                    robotSendEnable("DisableRobot()");
                    Sleep(100);
                }
            }
```

- [ ] **Step 5: Call checkHapticWatchdog() from main.cpp idle()**

In `Touch_Client/main.cpp`, in the `idle()` function (~line 78):

```cpp
void idle() {
    if (!appState.isClosing) {
        glutPostRedisplay();
        // Poll force data at ~30Hz alongside feedback
        RelayCore::instance().pollForce();
        // Check haptic watchdog (only when not in --no-robot mode)
        if (!g_noRobot) {
            RelayCore::instance().checkHapticWatchdog();
        }
        Sleep(1);
    }
}
```

- [ ] **Step 6: Build and run tests**

Run: `cd D:\Projects\Touch && bash build.sh && bash build.sh test`
Expected: Build 0 errors 0 warnings, 12/12 tests pass.

- [ ] **Step 7: Commit**

```bash
git add Touch_Client/relay/RelayCore.h Touch_Client/relay/RelayCore.cpp Touch_Client/main.cpp
git commit -m "feat(safety): add dual-layer haptic watchdog — GLUT idle() primary + ForceReader thread fallback"
```

---

### Task 6: Unit tests — test_safety_core.cpp

**Files:**
- Create: `Touch_Client/tests/test_safety_core.cpp`

**Interfaces:**
- Tests consume: `RobotStateMachine`, `EscalationTracker`, `RobotError`, `Config` (all existing)
- No new interfaces produced

- [ ] **Step 1: Write the test file**

Create `Touch_Client/tests/test_safety_core.cpp`:

```cpp
// Standalone test: Safety core — state machine, escalation, callbacks
// Build: cl /EHsc /std:c++17 test_safety_core.cpp
//        ../safety/RobotStateMachine.cpp ../safety/RobotDiagnostics.cpp
//        /I"..\..\OpenHaptics\Developer\3.5.0\include"
//        /I"..\..\OpenHaptics\Developer\3.5.0\utilities\include"
//        /Fe:test_safety_core.exe
//        /link /SUBSYSTEM:CONSOLE
// Run: test_safety_core.exe

#include <iostream>
#include <cassert>
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

    Sleep(60);  // ensure MIN_WARN_MS (50ms) is satisfied

    sm.onError(warnErr, delta);
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

    Sleep(60);
    for (int i = 0; i < 3; i++) sm.onError(warnErr, delta);
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

    Sleep(60);
    for (int i = 0; i < 3; i++) sm.onError(warnErr, delta);
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
```

- [ ] **Step 2: Build the test executable**

Run:
```bash
cd D:\Projects\Touch\Touch_Client\tests
```
Then build with MSVC (the exact command depends on the build system — check how test_constraint_force.exe is built). Follow the existing pattern from `test_constraint_force.cpp:2-6`. Use MSBuild or the project's build script.

Expected: test_safety_core.exe compiles successfully with 0 errors.

- [ ] **Step 3: Run the tests**

Run: `test_safety_core.exe`
Expected output:
```
=== Safety Core Unit Tests ===
  state_transition_chain... PASS
  fatal_callback... PASS
  can_move_guard... PASS
  speed_factor... PASS
  escalation_warn_to_degrade... PASS
  escalation_different_error_resets... PASS
  deescalation_reverse_motion... PASS
  deescalation_clear_frames... PASS

Results: 8 passed, 0 failed
```

- [ ] **Step 4: Commit**

```bash
git add Touch_Client/tests/test_safety_core.cpp
git commit -m "test(safety): add safety core unit tests — 8 tests covering state machine, escalation, callback"
```

---

### Final Verification

After all 6 tasks complete:

- [ ] **Build:** `cd D:\Projects\Touch && bash build.sh` — 0 errors, 0 warnings
- [ ] **Unit tests:** All 20 tests pass (12 existing + 8 new)
- [ ] **Smoke test:** `--no-robot --no-touch` mode starts without crash
- [ ] **Git log:** 6 commits on top of e87da6f
