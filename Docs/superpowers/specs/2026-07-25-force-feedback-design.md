# Force Feedback Module — Design Specification

**Date:** 2026-07-25
**Status:** Design Approved
**Branch:** master
**Context:** KWR75B six-axis force sensor + Touch haptic device + CR3 robot arm

---

## 1. Overview

### 1.1 Goal

Add bidirectional force feedback to the Touch-Dobot v3.0 teleoperation system:

- **Sensor → Touch:** Operator feels environment forces on the haptic stylus (force reflection)
- **Sensor → Motion Control:** Future autonomous force-controlled operations (constant force, force-stop)

### 1.2 Primary Use Case (Option C)

Both force-reflected teleoperation **and** force-controlled motion. Force data flows simultaneously to:
1. Touch haptic rendering (1kHz servo loop)
2. MATLAB relay_gui visualization (30Hz, `F|` protocol)
3. Future: ServoP force-control closed loop

### 1.3 Data Sources

| Source | Port | Rate | Field | Priority |
|--------|------|------|-------|----------|
| `GetSixForceData()` | 29999 | ~10Hz | `{Fx,Fy,Fz,Mx,My,Mz}` | Fallback only |
| 30004 realtime feedback | 30004 | 125Hz (8ms) | `ActualTCPForce` (bytes 576-623) | **Primary** |
| 30004 realtime feedback | 30004 | 125Hz | `SixForceValue` (bytes 1304-1351) | Cross-check |

Design uses **30004 port `ActualTCPForce`** as the primary data source for lowest latency.

---

## 2. Architecture

### 2.1 Module Diagram

```
                          ForcePipeline (independent module)
┌─────────────────────────────────────────────────────────┐
│                                                         │
│  CR3:30004 ──→ ForceReader ──→ ForceFilter ──→ Safety  │
│  (125Hz)      (binary       (Butterworth   (watchdog,  │
│                parse, 6dof)  30Hz cutoff)   staleness)  │
│                     │              │            │       │
│                 forceRaw     forceFilt    isStale flag   │
│                   [6]          [6]                      │
└─────────────────────┬───────────┬───────────────────────┘
                      ↓           ↓
          ┌───────────────┐  ┌──────────────────┐
          │ hapticCallback │  │    RelayCore      │
          │    (1kHz)      │  │    (30Hz)         │
          │                │  │                   │
          │ coord transform│  │ F|fx,fy,fz,       │
          │ → hdSetDoublev │  │    mx,my,mz →     │
          │   (HD_CURRENT  │  │    MATLAB GUI     │
          │    _FORCE)      │  │                   │
          └───────────────┘  └──────────────────┘
                   ↓                    ↓
            ┌──────────┐       ┌──────────────┐
            │ Operator  │       │ HUD: force   │
            │ feels     │       │ panels       │
            │ force     │       │ (already      │
            └──────────┘       │ ready)        │
                               └──────────────┘
```

### 2.2 Design Principles

- **Isolation:** ForcePipeline is a pure data module — no dependency on Touch, GLUT, or RelayCore
- **Consumers pull:** haptic thread and RelayCore independently read processed data at their own rates
- **Zero-computation haptic callback:** Coordinate transform pre-computed at 30Hz; haptic callback (1kHz) only reads 3 doubles + calls `hdSetDoublev`
- **Future-proof:** Force-control closed loop (Phase 4) adds a new consumer with zero changes to ForcePipeline

---

## 3. Components

### 3.1 ForceData (shared state)

```cpp
struct ForceData {
    double raw[6] = {0};        // Fx,Fy,Fz,Mx,My,Mz (N, Nm) from sensor
    double filtered[6] = {0};   // Butterworth lowpass output
    double hapticOut[3] = {0};  // Pre-transformed to Touch coordinates, ready for hdSetDoublev
    bool isStale = true;        // >200ms no data → haptic force zeros
    DWORD lastUpdateMs = 0;     // ForceReader heartbeat timestamp
};
CRITICAL_SECTION forceDataMutex;  // ForceReader writes, consumers read
```

### 3.2 ForceReader Thread

- Connects to CR3 port 30004
- Blocks on `recv()` for 1440-byte binary packets
- Parses `ActualTCPForce` (offset 576, 6 doubles = 48 bytes) and `SixForceValue` (offset 1304)
- Writes into `appState.forceData.raw[6]` under `forceDataMutex`
- Updates `lastUpdateMs` heartbeat
- Reconnects every 2000ms on failure
- Exits on `appState.isClosing`

### 3.3 ForceFilter (Butterworth 2nd-order lowpass)

- Cutoff frequency: `fc = 30Hz` (matches ServoP rate, configurable)
- 2nd-order biquad, one instance per channel (6 filters)
- NaN guard: reset state to zero on invalid input
- Force mapping chain:
  ```
  raw → Deadzone (±0.5N) → Butterworth 30Hz → Scale (200N→3.3N) → Clamp → CoordXform → hapticOut
  ```

### 3.4 Force Mapping

```
Touch output (N)
  3.3 ────────────────────────  (HD_MAX_FORCE, Touch safety limit)
      │              ┌─────
      │             /
      │            /
  0.1 ┤──────┐   /              ← deadzone transition
      │      │  /
  0.0 ───────┴──────────────→ Sensor force (N)
     0    0.5           200

Scale: 1N sensor → 0.016N Touch (1:61 ratio)
Deadzone: ±0.5N
Saturation: >200N clamped to 3.3N
```

### 3.5 Safety Watchdog

| Check | Threshold | Action |
|-------|-----------|--------|
| Staleness | `now - lastUpdateMs > 200ms` | Set `isStale`, haptic force → zero, HUD shows red "NO DATA" |
| Gradient limit | `|ΔF|/frame > 50N` | Clamp to 50N/frame (sensor anomaly guard) |
| Touch output clamp | `|hapticOut[i]| > 3.3N` | Hard clamp (protect Touch hardware) |

### 3.6 Coordinate Transform

Force vector transform (Robot tool frame → Touch device frame):

```
TouchFx =  RobotFx       (Fx → Touch X)
TouchFy =  RobotFz       (Fz → Touch Y)
TouchFz = -RobotFy       (-Fy → Touch Z)
```

This uses the same orthogonal matrix as `CoordinateTransform::convertTouchToRobot()`.

**Torque (Mx, My, Mz):** Touch is 3DOF force output only. Torque data is used for HUD display, `F|` protocol to MATLAB, and future force-control closed loop.

---

## 4. Thread Model

```
Main Thread (GLUT)           ForceReader Thread       Haptic Thread (1kHz)
    │                             │                        │
    │                             │ recv() 1440B block     │
    │                             │ parse offset 576       │
    │                             │ write forceData        │
    │                             │ ↓ (125Hz)              │
    │                             │                        │
    │ RelayCore::pollForce()      │                        │
    │  read forceData             │                        │
    │  filter step                │                        │
    │  coord xform → hapticOut    │                        │
    │  F| → MATLAB                │                        │
    │  ↓ (30Hz)                   │                        │
    │                             │              hapticCallback:
    │                             │               read hapticOut
    │                             │               hdSetDoublev()
```

### Timing Properties

| Pipe Segment | Rate | Latency |
|-------------|------|---------|
| Sensor → 30004 packet | 125Hz | 0-8ms |
| ForceReader parse + write | 125Hz | <0.1ms |
| pollForce() filter + xform | 30Hz | 0-33ms |
| hapticCallback read + render | 1kHz | <1ms |
| **End-to-end (sensor → operator)** | | **~10-40ms** |

---

## 5. Protocol: F| Message

### Format

```
F|<fx>,<fy>,<fz>,<mx>,<my>,<mz>,<staleness_flag>\n

Example:
  F|12.34,-5.67,0.12,1.23,-2.34,0.05,0    ← normal data
  F|0,0,0,0,0,0,1                          ← isStale / offline
```

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| fx, fy, fz | double | N | Filtered force (2 decimal places) |
| mx, my, mz | double | Nm | Filtered torque |
| staleness | int | — | 0 = normal, 1 = timed out offline |

### MATLAB relay_gui Changes

- Parse `F|` messages in dispatch switch
- Store `S.force_raw` (3 forces), `S.force_filt` (3 forces), `S.force_moment` (3 torques), `S.force_stale` (flag)
- Update display panels: red warning when `force_stale == 1`
- Existing force panel UI already allocated, only needs data binding

---

## 6. File Impact

### New Files

```
Touch_Client/force/ForcePipeline.h      ForceData struct + filter class declarations
Touch_Client/force/ForcePipeline.cpp    Butterworth impl + force mapping + coord transform
```

### Modified Files

```
Touch_Client/core/AppState.h            forceRaw[3] → ForceData forceData
Touch_Client/core/AppState.cpp          Init forceDataMutex
Touch_Client/relay/RelayCore.h          +pollForce(), +initForceReader(), +shutdownForceReader()
Touch_Client/relay/RelayCore.cpp        pollForce() implementation
Touch_Client/haptic/HapticCallback.cpp  +hdSetDoublev(HD_CURRENT_FORCE, hapticOut)
Touch_Client/render/HudOverlay.cpp      Read forceData.filtered[6] instead of forceRaw[3]
Touch_Client/main.cpp                   Start/stop ForceReader thread
Touch_Client/robot/RobotConnection.h    +connectRealtime(port)
Touch_Client/robot/RobotConnection.cpp  30004 connect + recv (1440B binary)
Touch_Client/config/Config.h            Force config constants
Relay_Station/relay_gui.m              F| parsing + 6DOF display + stale indicator
```

### Unchanged Files

```
CoordinateTransform.h, IExtension.h, FeedbackParser, SceneRenderer,
RobotModel, SafetyPredictor, Kinematics, HapticDevice
```

### Approximate Code Volume

- ~300 lines new code (ForcePipeline + RobotConnection 30004)
- ~150 lines modified (glue code across 10 files)
- Core complexity concentrated in ForcePipeline (independent, testable)

---

## 7. Config Constants (Config.h additions)

```cpp
namespace Config {
    const int FORCE_REALTIME_PORT = 30004;
    const int FORCE_FILTER_CUTOFF = 30;       // Butterworth cutoff (Hz)
    const int FORCE_STALE_MS = 200;           // Data timeout
    const double FORCE_DEADZONE_N = 0.5;      // Deadzone threshold (N)
    const double FORCE_MAX_SENSOR_N = 200.0;  // Sensor full scale
    const double FORCE_MAX_TOUCH_N = 3.3;     // Touch max safe force
    const double FORCE_GRADIENT_LIMIT = 50.0; // Gradient clamp (N/frame)
    const int FORCE_RECONNECT_INTERVAL = 2000;// Reconnect retry (ms)
}
```

---

## 8. Error Handling Matrix

| Scenario | Behavior |
|----------|----------|
| 30004 connect fails | Log warning. Force feedback degraded. Teleop proceeds normally. |
| 30004 disconnects mid-run | `isStale = true`. Haptic force → zero. Reconnect every 2s. |
| `recv()` returns <1440 bytes | Discard frame. Wait for next. |
| Butterworth NaN | Reset filter state to 0 for that channel. |
| Touch device unplugged (`HD_INVALID`) | `hdSetDoublev` becomes no-op silently. No crash. |
| `hapticOut[i] > ±3.3N` | Hard clamp. Protect Touch hardware. |
| `--no-robot` mode | ForceReader connect fails. HUD shows "NO ROBOT". Touch force = 0. All else works. |

---

## 9. Test Strategy

### 9.1 Unit Tests (offline — no robot, no Touch)

| Test Object | Input | Verify |
|-------------|-------|--------|
| Butterworth filter | Step input | Correct rise time and settling |
| Butterworth filter | 100Hz sine | >90% attenuation (noise rejection) |
| Butterworth filter | NaN input | State resets to 0 |
| Force mapping | 0N | Output = 0 |
| Force mapping | 200N | Output = 3.3N |
| Force mapping | 0.3N | Output = 0 (deadzone) |
| Force mapping | 500N | Clamp to 3.3N |
| Coordinate transform | (1,2,3) | → (1,3,-2) |
| Coordinate transform | Vector magnitude | Preserved after transform |
| Stale detection | lastUpdate = 0 | isStale = true |
| Stale detection | lastUpdate = now | isStale = false |

Standalone: `tests/test_force_pipeline.cpp` links only `ForcePipeline.cpp`.

### 9.2 Integration Tests (online)

| Phase | Action | Expected |
|-------|--------|----------|
| Connect | Start program | HUD force panel shows real data instead of "awaiting" |
| Zero | Robot still, no tool | Fx/Fy/Fz ≈ 0±1N (noise in deadzone) |
| Push/pull | Push end flange | Fz reflects direction, torque axes respond |
| Touch render | Press button 1 while touching stylus | Feel reaction force from sensor |
| Stale recovery | Unplug robot ethernet 2s → replug | HUD red NO DATA → auto-recover |
| Gradient guard | Inject 500N spike in code | Output clamped to 50N/frame |

---

## 10. Future Extensions (Out of Scope)

| Extension | Dependencies | Changes Needed |
|-----------|-------------|----------------|
| Force-control closed loop | Force data available | RelayCore: `ServoPForce()` mode (impedance control) |
| 30005 low-rate feedback | 30004 verified | Optional alternate port for lower CPU |
| Torque haptic rendering | 4DOF+ haptic device | `hapticOut` extended beyond 3 axes |
| Force recording/playback | `F|` protocol stable | relay_gui side file logging + replay |
| Impedance control (Ch.5 manual) | Filtered force available | New `ForceControl` consumer of ForcePipeline |

---

## 11. Acceptance Criteria

1. `F|` protocol messages arrive at MATLAB relay_gui at ~30Hz with 6-axis data
2. HUD "Force Sensor" panels display real-time Fx/Fy/Fz/Mx/My/Mz values (not "awaiting integration")
3. When pushing the robot end-effector, Touch stylus renders a proportional force in the correct direction
4. When 30004 connection drops, HUD shows red "OFFLINE" and Touch force returns to zero within 200ms
5. `--no-robot` mode starts and runs normally without force data
6. Unit tests pass for filter, mapping, transformation, and stale detection
7. No regression: existing teleoperation (button press → robot motion) works identically

---

## 12. Implementation Phases

| Phase | Description | Priority |
|-------|-------------|----------|
| P1 — Core Pipeline | ForceReader + ForceFilter + ForceData + unit tests | P0 |
| P2 — Integration | pollForce() + haptic rendering + F| protocol + HUD | P0 |
| P3 — MATLAB | relay_gui F| parsing + 6DOF display + stale indicator | P1 |
| P4 — Polish | Gradient guard + reconnect + --no-robot mode | P1 |
