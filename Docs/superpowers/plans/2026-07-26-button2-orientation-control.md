# Button 2 Orientation Control Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Wire Touch button 2 to control end-effector orientation (Rx/Ry/Rz) via stylus physical rotation, with button 1+2 combined enabling full 6-DOF. Incremental mode — press captures reference, rotation delta drives target.

**Architecture:** Read `HD_CURRENT_TRANSFORM` (4×4 matrix) in the 1kHz haptic callback, extract ZYX Euler angles, feed into a button 2 state machine symmetric to button 1's, compute incremental orientation deltas in `RelayCore::sendPosition()` alongside existing position deltas, and send dynamic Rx/Ry/Rz in `ServoP()` instead of frozen base values.

**Tech Stack:** C++17, MSVC, OpenHaptics HDAPI, Win32 threads + CRITICAL_SECTION, existing `Vec3` type reused for orientation.

## Global Constraints

- Button 1 position-only behavior must be preserved (no regression)
- `--no-robot` mode must still start and run (orientation data computed but not sent)
- Force feedback activates only when button 1 is held (regardless of button 2)
- Orientation is incremental: press→record reference, release→invalidate, re-press→re-reference (no jumps)
- Euler angle convention: ZYX intrinsic (`R = Rz·Ry·Rx`), matching Dobot RPY convention from `GetPose()`
- Thread safety: `stylusOrient` must use a dedicated CRITICAL_SECTION since haptic thread writes and main thread reads

---

### Task 1: Config.h — Add orientation control constants

**Files:**
- Modify: `Touch_Client/config/Config.h`

**Produces:**
- `Config::ORIENT_MAX_STEP_DEG` (double, 3.0)
- `Config::ORIENT_DEADZONE_DEG` (double, 0.05)
- `Config::ORIENT_GAIN` (double, 1.0)
- `Config::SAFE_RX_MIN` (double, -180.0)
- `Config::SAFE_RX_MAX` (double, 180.0)
- `Config::SAFE_RY_MIN` (double, -90.0)
- `Config::SAFE_RY_MAX` (double, 90.0)
- `Config::SAFE_RZ_MIN` (double, -180.0)
- `Config::SAFE_RZ_MAX` (double, 180.0)

- [ ] **Step 1: Add orientation constants block**

Insert after the force compensation runtime parameters block (after line 84, before the virtual constraint force section):

```cpp
    // ========== 姿态控制参数 ==========
    const double ORIENT_MAX_STEP_DEG = 3.0;          // 单步最大角度增量 (degrees)
    const double ORIENT_DEADZONE_DEG = 0.05;         // 姿态死区 (degrees)
    const double ORIENT_GAIN = 1.0;                  // 姿态增益 (可调灵敏度)
    const double SAFE_RX_MIN = -180.0, SAFE_RX_MAX = 180.0;  // Roll 安全限位
    const double SAFE_RY_MIN = -90.0,  SAFE_RY_MAX = 90.0;   // Pitch 安全限位
    const double SAFE_RZ_MIN = -180.0, SAFE_RZ_MAX = 180.0;  // Yaw 安全限位
```

- [ ] **Step 2: Verify compilation**

```bash
cd Touch_Client && msbuild Touch_Client.vcxproj /p:Configuration=Release /t:Build /v:minimal 2>&1 | tail -5
```

Expected: Build succeeded.

- [ ] **Step 3: Commit**

```bash
git add Touch_Client/config/Config.h
git commit -m "feat(config): add orientation control constants — step cap, deadzone, gain, angle bounds"
```

---

### Task 2: AppState — Add stylusOrient shared state

**Files:**
- Modify: `Touch_Client/core/AppState.h`
- Modify: `Touch_Client/core/AppState.cpp`

**Produces:**
- `AppState::stylusOrient[3]` — current stylus Euler angles (Rx, Ry, Rz in degrees), written by haptic thread
- `AppState::stylusOrientMutex` — CRITICAL_SECTION protecting the above
- `AppState::transformMatrix[16]` now populated from `HD_CURRENT_TRANSFORM`

- [ ] **Step 1: Add stylusOrient fields to AppState.h**

After line 37 (`double transformMatrix[16] = { 0 }; // HD_CURRENT_TRANSFORM 预留`), insert:

```cpp
    double stylusOrient[3] = { 0.0, 0.0, 0.0 };  // 笔杆姿态 ZYX Euler (Rx,Ry,Rz in degrees)
    CRITICAL_SECTION stylusOrientMutex;
```

- [ ] **Step 2: Initialize stylusOrientMutex in AppState.cpp constructor**

After line 19 (`InitializeCriticalSection(&forceDataMutex);`), add:

```cpp
    InitializeCriticalSection(&stylusOrientMutex);
```

- [ ] **Step 3: Destroy stylusOrientMutex in AppState.cpp destructor**

After line 39 (`DeleteCriticalSection(&forceDataMutex);`), add:

```cpp
    DeleteCriticalSection(&stylusOrientMutex);
```

- [ ] **Step 4: Commit**

```bash
git add Touch_Client/core/AppState.h Touch_Client/core/AppState.cpp
git commit -m "feat(appstate): add stylusOrient shared state + mutex for HD_CURRENT_TRANSFORM"
```

---

### Task 3: RelayCore.h — Add orientation members and method declarations

**Files:**
- Modify: `Touch_Client/relay/RelayCore.h`

**Produces:**
- `RelayCore::m_targetOrient` (Vec3) — accumulated orientation target
- `RelayCore::m_lastStylusOrient` (Vec3) — previous frame stylus orientation for delta
- `RelayCore::m_orientRefStylus` (Vec3) — stylus orientation at button2 press (reference)
- `RelayCore::m_orientRefRobot` (Vec3) — robot orientation at button2 press (reference)
- `RelayCore::m_orientValid` (bool) — whether orientation reference is set
- `RelayCore::m_transmittingOrient` (bool) — whether button 2 is held
- `RelayCore::onButton2Press()` — handle button 2 press edge
- `RelayCore::onButton2Release()` — handle button 2 release edge

- [ ] **Step 1: Add orientation member variables**

After line 91 (`bool   m_lastTouchValid = false;`), add:

```cpp
    // ===== 姿态控制 (Button 2) =====
    Vec3 m_targetOrient;          // 累加式姿态目标 (Rx, Ry, Rz in degrees)
    Vec3 m_lastStylusOrient;      // 上一帧笔杆姿态, 用于增量计算
    Vec3 m_orientRefStylus;       // 按下瞬间的笔杆参考姿态
    Vec3 m_orientRefRobot;        // 按下瞬间的末端参考姿态
    bool  m_orientValid = false;
    bool  m_transmittingOrient = false;
```

- [ ] **Step 2: Add method declarations**

After line 23 (`void onButtonRelease();`), add:

```cpp
    void onButton2Press(const Vec3& stylusOrient);
    void onButton2Release();
```

- [ ] **Step 3: Commit**

```bash
git add Touch_Client/relay/RelayCore.h
git commit -m "feat(relay): add orientation state members + button2 method declarations"
```

---

### Task 4: RelayCore.cpp — Implement orientation control logic

**Files:**
- Modify: `Touch_Client/relay/RelayCore.cpp`

**Consumes:**
- `Config::ORIENT_MAX_STEP_DEG`, `Config::ORIENT_DEADZONE_DEG`, `Config::ORIENT_GAIN`
- `Config::SAFE_RX/RY/RZ_MIN/MAX`
- `AppState::stylusOrient[3]`, `AppState::stylusOrientMutex`
- `AppState::robotActualPose.rx/ry/rz`, `AppState::robotPoseMutex`

**Produces:**
- `RelayCore::clampOrientToBounds()` — helper to clamp Vec3 orientation to safe bounds
- `RelayCore::onButton2Press()` — captures reference orientations, sets m_transmittingOrient
- `RelayCore::onButton2Release()` — clears transmitting flag, invalidates reference
- Modified `RelayCore::sendPosition()` — computes orientation delta + sends dynamic Rx/Ry/Rz

- [ ] **Step 1: Add orientation boundary clamp helper**

After the `#include` block (before the `forceReaderThread` function, around line 16), add a `clampOrientToBounds` function:

```cpp
// ===== 姿态安全边界钳位 =====
static Vec3 clampOrientToBounds(const Vec3& target) {
    Vec3 clamped = target;
    bool warned = false;

    if (target.x < Config::SAFE_RX_MIN) { clamped.x = Config::SAFE_RX_MIN; warned = true; }
    if (target.x > Config::SAFE_RX_MAX) { clamped.x = Config::SAFE_RX_MAX; warned = true; }
    if (target.y < Config::SAFE_RY_MIN) { clamped.y = Config::SAFE_RY_MIN; warned = true; }
    if (target.y > Config::SAFE_RY_MAX) { clamped.y = Config::SAFE_RY_MAX; warned = true; }
    if (target.z < Config::SAFE_RZ_MIN) { clamped.z = Config::SAFE_RZ_MIN; warned = true; }
    if (target.z > Config::SAFE_RZ_MAX) { clamped.z = Config::SAFE_RZ_MAX; warned = true; }

    if (warned) {
        std::cerr << "[Safety] Orientation target out of bounds, clamped. Original: ("
                  << target.x << "," << target.y << "," << target.z << ")" << std::endl;
    }
    return clamped;
}
```

- [ ] **Step 2: Implement onButton2Press**

After the existing `onButtonRelease()` function (after line 615), add:

```cpp
void RelayCore::onButton2Press(const Vec3& stylusOrient) {
    EnterCriticalSection(&m_basePointLock);
    // Capture stylus reference orientation at press moment
    m_orientRefStylus = stylusOrient;

    // Capture robot current orientation
    {
        auto& app = appState;
        EnterCriticalSection(&app.robotPoseMutex);
        m_orientRefRobot = Vec3(app.robotActualPose.rx, app.robotActualPose.ry, app.robotActualPose.rz);
        LeaveCriticalSection(&app.robotPoseMutex);
    }

    // Initialize accumulated target and last-frame stylus orientation
    m_targetOrient = m_orientRefRobot;
    m_lastStylusOrient = stylusOrient;
    m_orientValid = true;
    m_transmittingOrient = true;

    // If position mode is not already active, start transmission
    if (!m_transmitting) {
        m_stateMachine.onButtonPress();
        // Seed position target from actual pose (same as button1 press)
        {
            auto& app = appState;
            EnterCriticalSection(&app.robotPoseMutex);
            Vec3 rawPos(app.robotActualPose.x, app.robotActualPose.y, app.robotActualPose.z);
            LeaveCriticalSection(&app.robotPoseMutex);
            m_targetPos = SafetyBoundary::clampToBoundary(rawPos);
        }
        m_transmitting = true;
    }
    LeaveCriticalSection(&m_basePointLock);

    std::cout << "[Relay] Button2 PRESS — orient ref=("
              << m_orientRefStylus.x << "," << m_orientRefStylus.y << "," << m_orientRefStylus.z << ")"
              << " robot ref=(" << m_orientRefRobot.x << "," << m_orientRefRobot.y << "," << m_orientRefRobot.z << ")"
              << std::endl;
}
```

- [ ] **Step 3: Implement onButton2Release**

Add after `onButton2Press`:

```cpp
void RelayCore::onButton2Release() {
    m_transmittingOrient = false;
    m_orientValid = false;

    // If position mode is also not active, stop all transmission
    // (m_transmitting will be false if only button2 was held)
    // Note: m_transmitting is checked independently — keep it set
    // if button1 is still held. We use a separate check:
    // if neither mode is active, stop transmission fully.
    // The haptic callback will call onButtonRelease() separately
    // when button1 is released. Here we only clear orient state.

    std::cout << "[Relay] Button2 RELEASE — orientation control stopped" << std::endl;
}
```

- [ ] **Step 4: Modify sendPosition() — add orientation delta computation**

In `sendPosition()`, replace the ServoP command construction (lines 545-550) that currently uses fixed `robotBaseRx/y/z` with dynamic orientation.

**Old code (lines 545-550):**
```cpp
    // ===== 构造并发送 ServoP =====
    auto& app = appState;
    char cmd[256];
    snprintf(cmd, sizeof(cmd), "ServoP(%.2f,%.2f,%.2f,%.2f,%.2f,%.2f)",
        clamped.x, clamped.y, clamped.z,
        app.robotBaseRx, app.robotBaseRy, app.robotBaseRz);
```

**New code:**
```cpp
    // ===== 姿态增量计算 (Button 2 按下时) =====
    double targetRx = app.robotBaseRx;
    double targetRy = app.robotBaseRy;
    double targetRz = app.robotBaseRz;

    if (m_transmittingOrient && m_orientValid) {
        // Read current stylus orientation (thread-safe)
        double stylusRx, stylusRy, stylusRz;
        EnterCriticalSection(&app.stylusOrientMutex);
        stylusRx = app.stylusOrient[0];
        stylusRy = app.stylusOrient[1];
        stylusRz = app.stylusOrient[2];
        LeaveCriticalSection(&app.stylusOrientMutex);

        Vec3 current(stylusRx, stylusRy, stylusRz);

        // Compute incremental delta from stylus rotation change
        double drx = current.x - m_lastStylusOrient.x;
        double dry = current.y - m_lastStylusOrient.y;
        double drz = current.z - m_lastStylusOrient.z;

        // Update reference for next frame
        m_lastStylusOrient = current;

        // Deadzone filter
        if (fabs(drx) >= Config::ORIENT_DEADZONE_DEG ||
            fabs(dry) >= Config::ORIENT_DEADZONE_DEG ||
            fabs(drz) >= Config::ORIENT_DEADZONE_DEG) {

            // Apply gain
            drx *= Config::ORIENT_GAIN;
            dry *= Config::ORIENT_GAIN;
            drz *= Config::ORIENT_GAIN;

            // Step cap
            if (drx > Config::ORIENT_MAX_STEP_DEG) drx = Config::ORIENT_MAX_STEP_DEG;
            if (drx < -Config::ORIENT_MAX_STEP_DEG) drx = -Config::ORIENT_MAX_STEP_DEG;
            if (dry > Config::ORIENT_MAX_STEP_DEG) dry = Config::ORIENT_MAX_STEP_DEG;
            if (dry < -Config::ORIENT_MAX_STEP_DEG) dry = -Config::ORIENT_MAX_STEP_DEG;
            if (drz > Config::ORIENT_MAX_STEP_DEG) drz = Config::ORIENT_MAX_STEP_DEG;
            if (drz < -Config::ORIENT_MAX_STEP_DEG) drz = -Config::ORIENT_MAX_STEP_DEG;

            // Accumulate and clamp
            m_targetOrient.x += drx;
            m_targetOrient.y += dry;
            m_targetOrient.z += drz;
            m_targetOrient = clampOrientToBounds(m_targetOrient);
        }

        targetRx = m_targetOrient.x;
        targetRy = m_targetOrient.y;
        targetRz = m_targetOrient.z;
    }

    // ===== 构造并发送 ServoP =====
    auto& app = appState;
    char cmd[256];
    snprintf(cmd, sizeof(cmd), "ServoP(%.2f,%.2f,%.2f,%.2f,%.2f,%.2f)",
        clamped.x, clamped.y, clamped.z,
        targetRx, targetRy, targetRz);
```

- [ ] **Step 5: Commit**

```bash
git add Touch_Client/relay/RelayCore.cpp
git commit -m "feat(relay): implement button2 orientation control — incremental delta + dynamic ServoP RxRyRz"
```

---

### Task 5: HapticCallback.cpp — Read stylus rotation + button 2 state machine

**Files:**
- Modify: `Touch_Client/haptic/HapticCallback.cpp`

**Consumes:**
- `AppState::stylusOrient[3]`, `AppState::stylusOrientMutex`, `AppState::transformMatrix[16]`
- `RelayCore::onButton2Press()`, `RelayCore::onButton2Release()`, `RelayCore::isTransmitting()`
- `Config::ORIENT_DEADZONE_DEG`

**Produces:**
- Populated `stylusOrient` and `transformMatrix` from `HD_CURRENT_TRANSFORM`
- Button 2 state machine (edge detect, persistent transmission)
- Combined button 1+2 dispatch logic
- Updated `P|` protocol message with stylus orientation

- [ ] **Step 1: Read HD_CURRENT_TRANSFORM and extract Euler angles**

After the position read block (after line 31, before the coordinate transform section), add:

```cpp
    // ===== 1b. 读取笔杆姿态 (HD_CURRENT_TRANSFORM → Euler ZYX) =====
    {
        hdGetDoublev(HD_CURRENT_TRANSFORM, app.transformMatrix);

        // Column-major 4x4 matrix:
        // | m0  m4  m8  m12 |   rotation 3x3:  [m0 m4 m8 ]
        // | m1  m5  m9  m13 |                   [m1 m5 m9 ]
        // | m2  m6  m10 m14 |                   [m2 m6 m10]
        // | m3  m7  m11 m15 |
        double* m = app.transformMatrix;

        // Extract ZYX intrinsic Euler angles from rotation submatrix
        // R = Rz(rz) * Ry(ry) * Rx(rx)
        double sy = -m[2];  // -R[2][0]
        if (sy > 1.0) sy = 1.0;
        if (sy < -1.0) sy = -1.0;

        double ry_rad = asin(sy);
        double rx_rad, rz_rad;

        if (fabs(cos(ry_rad)) > 1e-6) {
            // Non-gimbal-lock case
            rx_rad = atan2(m[6], m[10]);   // atan2(R[2][1], R[2][2])
            rz_rad = atan2(m[1], m[0]);    // atan2(R[1][0], R[0][0])
        } else {
            // Gimbal lock: ry ≈ ±90°, rx and rz are coupled
            rx_rad = atan2(-m[9], m[5]);   // atan2(-R[1][2], R[1][1])
            rz_rad = 0.0;
        }

        double rx_deg = rx_rad * 180.0 / 3.14159265358979323846;
        double ry_deg = ry_rad * 180.0 / 3.14159265358979323846;
        double rz_deg = rz_rad * 180.0 / 3.14159265358979323846;

        EnterCriticalSection(&app.stylusOrientMutex);
        app.stylusOrient[0] = rx_deg;
        app.stylusOrient[1] = ry_deg;
        app.stylusOrient[2] = rz_deg;
        LeaveCriticalSection(&app.stylusOrientMutex);
    }
```

- [ ] **Step 2: Add button 2 state machine**

Replace the existing button state section (lines 43-69) with expanded logic. The old code:

```cpp
    // ===== 3. 按钮状态 =====
    int buttonState = 0;
    hdGetIntegerv(HD_CURRENT_BUTTONS, &buttonState);
    bool button1 = (buttonState & HD_DEVICE_BUTTON_1) != 0;
    bool button2 = (buttonState & HD_DEVICE_BUTTON_2) != 0;
    app.button2Pressed = button2;

    // ===== 4. 轨迹 (仅按钮按下时记录) =====
    if (button1) { ... }

    // ===== 5. 按钮 1 状态机 -> RelayCore =====
    bool stateChanged = (button1 != app.lastButtonState);
    if (stateChanged) {
        app.lastButtonState = button1;
        if (button1) {
            relay.onButtonPress(robotPos);
        } else {
            relay.onButtonRelease();
        }
    }
```

Replace with:

```cpp
    // ===== 3. 按钮状态 =====
    int buttonState = 0;
    hdGetIntegerv(HD_CURRENT_BUTTONS, &buttonState);
    bool button1 = (buttonState & HD_DEVICE_BUTTON_1) != 0;
    bool button2 = (buttonState & HD_DEVICE_BUTTON_2) != 0;
    app.button2Pressed = button2;

    // ===== 4. 轨迹 (仅按钮按下时记录) =====
    if (button1) {
        EnterCriticalSection(&app.trailMutex);
        app.trailPoints.push_back(localDevicePos);
        while ((int)app.trailPoints.size() > Config::MAX_TRAIL) {
            app.trailPoints.pop_front();
        }
        LeaveCriticalSection(&app.trailMutex);
    }

    // ===== 5. 按钮 1 状态机 -> RelayCore (位置) =====
    bool b1Changed = (button1 != app.lastButtonState);
    if (b1Changed) {
        app.lastButtonState = button1;
        if (button1) {
            relay.onButtonPress(robotPos);
        } else {
            relay.onButtonRelease();
        }
    }

    // ===== 5b. 按钮 2 状态机 -> RelayCore (姿态) =====
    static bool s_lastButton2 = false;
    bool b2Changed = (button2 != s_lastButton2);
    if (b2Changed) {
        s_lastButton2 = button2;
        if (button2) {
            // Build Vec3 from current stylus Euler angles
            double sx, sy, sz;
            EnterCriticalSection(&app.stylusOrientMutex);
            sx = app.stylusOrient[0];
            sy = app.stylusOrient[1];
            sz = app.stylusOrient[2];
            LeaveCriticalSection(&app.stylusOrientMutex);
            relay.onButton2Press(Vec3(sx, sy, sz));
        } else {
            relay.onButton2Release();
        }
    }
```

- [ ] **Step 3: Update continuous transmission check**

The existing continuous transmission (lines 72-74) reads:

```cpp
    // ===== 6. 持续发送（按钮保持按下） =====
    if (relay.isTransmitting()) {
        relay.sendPosition(localDevicePos);
    }
```

Keep this unchanged — `isTransmitting()` already returns true when either button mode is active (since `onButton2Press` sets `m_transmitting = true` if not already set).

- [ ] **Step 4: Update P| message to include stylus orientation**

In `RelayCore::reportPosition()` (in RelayCore.cpp), update the `P|` protocol message. The old code (line 960-961):

```cpp
    snprintf(buf, sizeof(buf), "P|%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
        pos[0], pos[1], pos[2], 0.0, 0.0, 0.0);
```

Change to include stylus Euler angles:

```cpp
    double sx, sy, sz;
    {
        auto& appRef = appState;
        EnterCriticalSection(&appRef.stylusOrientMutex);
        sx = appRef.stylusOrient[0];
        sy = appRef.stylusOrient[1];
        sz = appRef.stylusOrient[2];
        LeaveCriticalSection(&appRef.stylusOrientMutex);
    }
    snprintf(buf, sizeof(buf), "P|%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
        pos[0], pos[1], pos[2], sx, sy, sz);
```

- [ ] **Step 5: Commit**

```bash
git add Touch_Client/haptic/HapticCallback.cpp Touch_Client/relay/RelayCore.cpp
git commit -m "feat(haptic): read HD_CURRENT_TRANSFORM, button2 state machine, P| orientation update"
```

---

### Task 6: Build verification + integration smoke test

**Files:**
- (none — test-only task)

- [ ] **Step 1: Full build**

```bash
cd Touch_Client && msbuild Touch_Client.vcxproj /p:Configuration=Release /t:Rebuild /v:minimal 2>&1 | tail -10
```

Expected: Build succeeded. 0 errors, 0 warnings.

- [ ] **Step 2: Verify --no-robot startup does not crash**

```bash
cd Touch_Client\x64\Release && timeout 5 .\Touch_Client.exe --no-robot --no-touch 2>&1 || true
```

Expected: Program starts, prints system ready message, exits cleanly after 5s. No crash, no access violation.

- [ ] **Step 3: Verify --no-robot startup with touch device**

If Touch device is connected:

```bash
cd Touch_Client\x64\Release && timeout 8 .\Touch_Client.exe --no-robot 2>&1 || true
```

Expected: Touch device initializes. No crash. Stylus movement + button presses do not crash. Program exits cleanly.

- [ ] **Step 4: Commit (if any fixes)**

```bash
git add -A && git commit -m "fix: build verification fixes for button2 orientation control"
```

---

### Task 7: Hardware integration test (manual, on-robot)

**⚠️ This task requires the physical CR3 robot + Touch device.** Do NOT run unattended.

- [ ] **Step 1: Button 1 regression test**

Start `Touch_Client.exe`, press and hold button 1, move stylus → robot follows position only, orientation stays fixed. Release → robot stops.

- [ ] **Step 2: Button 2 orientation test**

Press and hold button 2 (no button 1), rotate stylus → robot follows orientation, position stays fixed. Release → orientation freezes.

- [ ] **Step 3: Button 1+2 combined test**

Press and hold both buttons, move + rotate stylus → robot follows 6-DOF. Release button 2 → orientation freezes, position still follows. Release both → all motion stops.

- [ ] **Step 4: Re-grip test (no jump)**

Press button 2, rotate stylus 30°, release. Rotate stylus back to original angle. Press button 2 again → robot orientation does NOT jump. New reference is captured at press.

- [ ] **Step 5: Force feedback test**

Press button 1+2, push against virtual boundary → haptic force felt. Release button 1 (keep button 2) → force disappears. Press button 1 again → force returns.

- [ ] **Step 6: Orientation safety bounds test**

Rotate stylus aggressively to exceed step cap → orientation delta is clamped. Rotate to exceed `SAFE_RY_MAX` (90°) → target clamps at 90°.

- [ ] **Step 7: Emergency stop**

During any button combination, press `q` or `ESC` → robot disables and program exits cleanly.

---
