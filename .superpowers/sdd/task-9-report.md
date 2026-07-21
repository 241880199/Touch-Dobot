# Task 9 Report: RelayCore -- Centralized Data Flow Orchestration

## Summary
Created `Codes/Touch_Client/relay/RelayCore.h` and `Codes/Touch_Client/relay/RelayCore.cpp`, implementing the RelayCore singleton that orchestrates all data flow between Touch, Robot, and Render layers.

## Files Created
- `D:\Projects\Touch\Codes\Touch_Client\relay\RelayCore.h`
- `D:\Projects\Touch\Codes\Touch_Client\relay\RelayCore.cpp`

## Implementation
- **Singleton pattern:** `static RelayCore& instance()` with Meyer's singleton, private constructor, deleted copy/assign.
- **`init()`** -- Sequential robot connection and initialization: connect -> ClearError -> EnableRobot -> CP -> GetPose, with Sleep delays between each step. Reads initial robot pose and stores it as the base pose in AppState.
- **`sendPosition()`** -- Hot-path function (called from 1kHz haptic callback). Non-blocking: early-exits if not transmitting, base point not set, or not connected. Converts Touch coordinates to robot space, computes delta, applies safety boundary, builds ServoP command, runs extension hooks, sends to motion port.
- **`pollFeedback()`** -- Called every display frame. Reads both motion port and enable port feedback, wraps in RobotFeedback structs, and dispatches to registered extensions.
- **`queryPose()`** -- Timer-driven: sends GetPose, receives response, updates AppState::robotActualPose.
- **`checkAlarm()`** -- Timer-driven: sends RobotMode, checks for alarm mode (mode==9), updates AppState::isRobotInAlarm atomically.
- **`registerExtension()`** -- Registers IExtension plugins for send/feedback hooks.
- **`onButtonPress/Release()`** -- Sets/clears the base point and transmission flag.

## Dependencies
Consumes all relay/ modules (ProtocolAdapter, FeedbackParser, SafetyBoundary, CoordinateTransform, IExtension), plus RobotConnection, AppState, and Config.

## Commit
- **Hash:** `126b979`
- **Message:** `feat: add RelayCore singleton for centralized data flow orchestration`

---

## Quality Fix (2026-07-21)

### Issue 1: Heap allocation in 1kHz hot-path (Critical)
- **File:** `RelayCore.cpp` `sendPosition()`
- **Problem:** `ProtocolAdapter::buildServoP()` returned `std::string`, performing heap allocation on every call in the 1kHz haptic callback.
- **Fix:** Replaced with stack-allocated `char cmd[256]` and inline `snprintf`. This eliminates heap allocation in the hot path.
- **Additional:** Inlined all remaining `ProtocolAdapter` calls (buildClearError, buildEnableRobot, buildCP, buildGetPose, buildDisableRobot, buildRobotMode) as they are simple string constants or trivial snprintf calls. Removed `ProtocolAdapter.h` and `ProtocolAdapter.cpp` entirely -- they are no longer referenced anywhere in the codebase.

### Issue 2: Data race on control flags (Important)
- **Files:** `RelayCore.h`, `RelayCore.cpp`
- **Problem:** `m_transmitting`, `m_basePointSet`, and `m_basePoint` were plain `bool`/`Vec3` members read in `sendPosition()` (haptic thread at 1kHz) and written in `onButtonPress/Release()` (UI thread), with no synchronization.
- **Fix:**
  - Changed `m_transmitting` and `m_basePointSet` to `std::atomic<bool>` with `{false}` initialization.
  - Added `CRITICAL_SECTION m_basePointLock` member for protecting `m_basePoint` (a struct of 3 doubles, non-trivially-copyable due to user-defined constructors, so `std::atomic<Vec3>` is not viable).
  - Added constructor (`InitializeCriticalSection`) and destructor (`DeleteCriticalSection`).
  - Wrapped `m_basePoint` reads in `sendPosition()` with `EnterCriticalSection`/`LeaveCriticalSection` (copy under lock, release lock before computation).
  - Wrapped `m_basePoint` write in `onButtonPress()` with `EnterCriticalSection`/`LeaveCriticalSection`.

### Files Modified
- `Codes/Touch_Client/relay/RelayCore.h` -- atomic members, CRITICAL_SECTION, ctor/dtor
- `Codes/Touch_Client/relay/RelayCore.cpp` -- inline commands, lock guards, ctor/dtor
- `Codes/Touch_Client/relay/ProtocolAdapter.h` -- **deleted** (no longer needed)
- `Codes/Touch_Client/relay/ProtocolAdapter.cpp` -- **deleted** (no longer needed)

### Tests
No test suite found in this project (Codes/Touch_Client/). Manual verification of the modified files confirms:
- All `ProtocolAdapter::` references removed from codebase (verified via grep)
- All `m_basePoint` accesses are now lock-protected
- Hot-path no longer allocates heap memory

### Commit
- **Message:** `fix: eliminate heap allocation in haptic hot-path, add synchronization for thread safety`
