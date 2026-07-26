# Button 2 Orientation Control Design

**Date:** 2026-07-26
**Status:** Design Approved
**Branch:** master

## 1. Problem Statement

Currently only Touch button 1 controls the robot: press-and-hold enables 3-DOF incremental position following (X, Y, Z), while end-effector orientation (Rx, Ry, Rz) is frozen to init-time GetPose() values. Button 2 is read from hardware but unused. The user needs:

- **Button 2 only** → orientation control (Rx, Ry, Rz), position frozen
- **Button 1 + Button 2** → full 6-DOF control (XYZ + RxRyRz)
- **Button 1 only** → position-only control (existing behavior, unchanged)

## 2. Goals

1. Read Touch stylus physical rotation via `HD_CURRENT_TRANSFORM` (4×4 matrix → Euler angles)
2. Incremental orientation mapping: stylus rotation delta → robot Rx/Ry/Rz delta
3. Button 2 state machine (press/release edge detection, persistent transmission while held)
4. Combined mode: both buttons held → 6-DOF incremental control from a single Touch device
5. Orientation safety: step limits, angle bounds, deadzone filtering
6. Force feedback unchanged: active whenever button 1 is held (regardless of button 2)
7. No regression to existing button 1 position-only behavior

## 3. Architecture

```
Touch Stylus
  ├── HD_CURRENT_POSITION  ──→ convertTouchToRobot() ──→ position delta  ──→ ServoP(X,Y,Z)
  ├── HD_CURRENT_TRANSFORM ──→ matrix4→euler()       ──→ orientation delta ──→ ServoP(Rx,Ry,Rz)
  └── HD_CURRENT_BUTTONS
        ├── Button1 only → position-only transmission  (unchanged)
        ├── Button2 only → orientation-only transmission (NEW)
        └── Button1+2    → 6-DOF transmission           (NEW)
```

### 3.1 Component Changes

**HapticCallback.cpp** — 1kHz haptic thread

| Change | Detail |
|--------|--------|
| Read `HD_CURRENT_TRANSFORM` | `hdGetDoublev(HD_CURRENT_TRANSFORM, matrix)` → extract Euler angles (ZYX) |
| Button 2 state machine | Edge-detect button2 press/release, symmetric to existing button1 logic |
| Combined mode dispatch | `button1 && button2` → call both press/release + send full 6-DOF |
| Force feedback | Unchanged: activate when button1 is held |

**RelayCore** — data-flow orchestration

| Change | Detail |
|--------|--------|
| New members | `m_targetOrient` (Vec3), `m_lastStylusOrient`, `m_orientRefStylus`, `m_orientRefRobot`, `m_orientValid`, `m_transmittingOrient` |
| `onButton2Press()` | Record stylus reference orientation + robot current orientation; set `m_transmittingOrient = true` |
| `onButton2Release()` | Set `m_transmittingOrient = false`; invalidate orientation reference |
| `sendPosition()` extended | Also compute orientation delta when `m_transmittingOrient`; send dynamic Rx/Ry/Rz in ServoP |
| Orientation safety | Deadzone (0.05°), step cap (`ORIENT_MAX_STEP_DEG`), angle bounds clamp |

**Config.h** — new constants

```cpp
const double ORIENT_MAX_STEP_DEG = 3.0;     // max rotation per frame (degrees)
const double ORIENT_DEADZONE_DEG = 0.05;    // minimum rotation to register
const double ORIENT_GAIN = 1.0;             // tunable sensitivity
const double SAFE_RX_MIN = -180.0, SAFE_RX_MAX = 180.0;
const double SAFE_RY_MIN = -90.0,  SAFE_RY_MAX = 90.0;
const double SAFE_RZ_MIN = -180.0, SAFE_RZ_MAX = 180.0;
```

**AppState.h** — shared state

| Change | Detail |
|--------|--------|
| `stylusOrient[3]` | Current stylus Euler angles (Rx, Ry, Rz in degrees), thread-safe |
| `stylusOrientMutex` | CRITICAL_SECTION for the above |
| `transformMatrix[16]` | Populated from `HD_CURRENT_TRANSFORM` (was reserved, now used) |

### 3.2 Button State Machine

```
Button1 edge detect:
  press  → onButtonPress()    → m_transmittingPos = true
  release → onButtonRelease()  → m_transmittingPos = false

Button2 edge detect (NEW):
  press  → onButton2Press()   → m_transmittingOrient = true
  release → onButton2Release() → m_transmittingOrient = false

Combined: both independently tracked.
  isTransmitting() = m_transmittingPos || m_transmittingOrient
```

### 3.3 Orientation Incremental Algorithm

```
┌─ onButton2Press ──────────────────────────────────────┐
│  m_orientRefStylus = current stylus Euler angles      │
│  m_orientRefRobot  = robotActualPose.rx, ry, rz       │
│  m_targetOrient    = m_orientRefRobot                 │
│  m_lastStylusOrient = m_orientRefStylus               │
│  m_orientValid = true                                 │
└───────────────────────────────────────────────────────┘

┌─ sendPosition (each frame, ~30Hz) ────────────────────┐
│  if (m_transmittingOrient && m_orientValid) {         │
│    // incremental delta from stylus rotation change   │
│    drx = stylusRx - m_lastStylusOrient.rx             │
│    dry = stylusRy - m_lastStylusOrient.ry             │
│    drz = stylusRz - m_lastStylusOrient.rz             │
│    m_lastStylusOrient = current                       │
│                                                       │
│    // deadzone                                        │
│    if (|drx|,|dry|,|drz| all < ORIENT_DEADZONE) skip │
│                                                       │
│    // step cap                                        │
│    clamp each delta to ±ORIENT_MAX_STEP_DEG          │
│                                                       │
│    // accumulate                                      │
│    m_targetOrient += delta * ORIENT_GAIN              │
│    clamp m_targetOrient to SAFE_RX/RY/RZ bounds       │
│  }                                                    │
│                                                       │
│  ServoP(x_target, y_target, z_target,                 │
│         rx_target, ry_target, rz_target)              │
└───────────────────────────────────────────────────────┘
```

## 4. Data Flow

```
HapticCallback (1kHz)
  │
  ├── hdGetDoublev(HD_CURRENT_POSITION, pos)
  ├── hdGetDoublev(HD_CURRENT_TRANSFORM, matrix)   ← NEW
  ├── hdGetIntegerv(HD_CURRENT_BUTTONS, &buttons)
  │
  ├── extractEulerZYX(matrix, rx, ry, rz)          ← NEW
  ├── store to appState.stylusOrient               ← NEW
  │
  ├── Button1 edge → onButtonPress / onButtonRelease
  ├── Button2 edge → onButton2Press / onButton2Release  ← NEW
  │
  └── if (isTransmitting()) sendPosition(pos)
        │
        ├── [pos mode]   delta XYZ from HD_CURRENT_POSITION
        ├── [orient mode] delta RxRyRz from stylusOrient  ← NEW
        │
        └── ServoP(x,y,z, rx,ry,rz) → TCP 30003
              │
              rx,ry,rz are now DYNAMIC when orient mode active
```

## 5. Euler Angle Extraction

From `HD_CURRENT_TRANSFORM` 4×4 matrix (column-major from OpenHaptics):

```
R = top-left 3×3 rotation submatrix
Extract Euler angles in ZYX intrinsic order (matching Dobot RPY convention):
  ry = asin(-R[2][0])          // pitch (Y)
  rx = atan2(R[2][1], R[2][2]) // roll  (X)
  rz = atan2(R[1][0], R[0][0]) // yaw   (Z)

All angles in degrees.
```

Note: `HD_CURRENT_TRANSFORM` gives the stylus tip transform in device coordinates. The rotation encodes the physical gimbal orientation of the stylus. Since we operate in incremental mode, the absolute reference frame is irrelevant — only angle deltas matter.

## 6. Safety

| Layer | Position (existing) | Orientation (new) |
|-------|---------------------|-------------------|
| Deadzone | < 0.05 mm skip | < 0.05° skip |
| Step cap | 4.5 × speedFactor mm | `ORIENT_MAX_STEP_DEG` ° |
| Hard bounds | `SAFE_X/Y/Z_MIN/MAX` | `SAFE_RX/RY/RZ_MIN/MAX` |
| SafetyPredictor | workspace radius, Z range, singularity, alarm history | same (runs on combined 6-DOF target) |
| State machine | WARN→DEGRADE→REJECT escalation | same pipeline |

Orientation safety is enforced BEFORE ServoP construction — bad orientation deltas are clamped, not sent to the robot.

## 7. Force Feedback

Rule unchanged: **force feedback activates when button 1 is held**, regardless of button 2 state.

- Button 1 only → force ON (existing)
- Button 1+2 → force ON
- Button 2 only → force OFF (no position motion, constraint forces meaningless)

## 8. MATLAB GUI Update

The `P|` protocol message (Touch position report) currently sends `rx,ry,rz` as 0. Update to include actual stylus orientation for monitoring:

```
P|x,y,z,rx,ry,rz   ← rx,ry,rz now carry stylus Euler angles
```

## 9. Files Changed

| File | Lines | Content |
|------|-------|---------|
| `Touch_Client/haptic/HapticCallback.cpp` | ~50 | Read `HD_CURRENT_TRANSFORM`, extract Euler, button2 state machine, combined dispatch |
| `Touch_Client/relay/RelayCore.h` | ~15 | `m_targetOrient`, `m_transmittingOrient`, ref members, `onButton2Press/Release` |
| `Touch_Client/relay/RelayCore.cpp` | ~90 | Button2 press/release, orientation delta, dynamic ServoP RxRyRz, angle bounds clamp |
| `Touch_Client/config/Config.h` | ~6 | Orientation limits, step cap, deadzone, gain |
| `Touch_Client/core/AppState.h` | ~8 | `stylusOrient[3]`, `stylusOrientMutex`, populate `transformMatrix[16]` |

No new files. All changes within the existing 5 source files. Existing button 1 behavior is fully preserved.

## 10. Testing

### Manual verification (on hardware)

1. **Button 1 only** — press button 1, move stylus → robot follows position, orientation stays fixed. Release → stops. (Regression check)
2. **Button 2 only** — press button 2, rotate stylus → robot follows orientation, position stays fixed. Release → stops.
3. **Button 1+2** — press both, move + rotate → robot follows 6-DOF. Release one → that dimension freezes, other continues.
4. **Re-grip** — release button 2, rotate stylus to new angle, press button 2 again → robot orientation does NOT jump (incremental re-reference).
5. **Safety bounds** — rotate stylus beyond ±180° range → target clamps at limit.
6. **Force feedback** — button 1+2 held, push against obstacle → haptic force felt. Button 2 only → no force.

### Unit tests (optional, `--no-robot`)

- Euler extraction from known rotation matrices
- Orientation delta computation with deadzone
- Combined button state transitions

## 11. Rollback

If orientation control proves unstable on hardware, revert by:
1. Comment out `hdGetDoublev(HD_CURRENT_TRANSFORM, ...)` in HapticCallback
2. Revert ServoP orientation params to `robotBaseRx/y/z`
3. Button 2 state machine is harmless — just no-ops
