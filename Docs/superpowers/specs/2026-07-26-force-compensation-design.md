# Force Sensor Gravity & Inertia Compensation Design

**Date:** 2026-07-26
**Status:** Design Approved
**Branch:** master

## 1. Problem Statement

The KWR75B force sensor exhibits significant zero-offset bias even when no external force is applied:

| Axis | Raw Reading (no load) | Noise Amplitude |
|------|----------------------|-----------------|
| FX   | -0.64 ~ -0.67 N      | ~0.03 N         |
| FY   | -1.06 ~ -1.07 N      | ~0.01 N         |
| FZ   | +0.05 ~ +0.06 N      | ~0.01 N         |
| MX   | -0.02 Nm             | negligible      |
| MY   | +0.02 Nm             | negligible      |
| MZ   | 0.00 ~ 0.01 Nm       | ~0.01 Nm         |

Current pipeline applies a 0.2N deadzone via `FORCE_DEADZONE_N` and ~5x reflection gain, which means:
- FX bias (-0.65N) → passes deadzone → ~3.25N phantom force at Touch
- FY bias (-1.07N) → passes deadzone → ~5.35N phantom force at Touch

Additionally, during robot motion, acceleration and gravity components vary with tool orientation, contaminating the force signal and making contact-force extraction unreliable. A simple deadzone cannot address these structured components.

## 2. Goals

1. **Zero-offset calibration (tare):** automatically estimate sensor bias F₀, M₀ via multi-pose least-squares
2. **Gravity compensation:** subtract tool-gravity components that rotate with end-effector orientation
3. **Inertia compensation:** subtract F=ma inertial force during acceleration (point-mass model)
4. **Online bias tracking:** EMA-based slow drift correction while stationary
5. **Safe automatic calibration:** velocity-limited, user-interruptible, boundary-checked multi-pose sweep
6. Persistent calibration file so the system survives restarts

## 3. Architecture

```
30004 raw data (125Hz)
       │
       ▼
┌──────────────────────────────────┐
│   ForceCompensation (NEW)         │
│                                   │
│  Input: raw[6], pose{Rx,Ry,Rz,X,Y,Z} │
│                                   │
│  Step 1: Estimate v_tool, a_tool  │  ← position 2nd-order central diff + 10Hz LPF
│  Step 2: Classify motion state    │  ← still | slow | moving
│  Step 3: Compute rotation R       │  ← Euler(Rx,Ry,Rz) → rotation matrix
│  Step 4: Gravity comp             │  ← F_g = m * R^T * g_base
│         Torque from gravity       │  ← M_g = r_com × F_g
│  Step 5: Inertia comp (if moving) │  ← F_i = m * a_tool (point mass)
│  Step 6: Apply bias subtraction   │  ← compensated = raw - bias - gravity - inertia
│  Step 7: EMA bias update (still)  │  ← bias += α * (raw - gravity)
│                                   │
│  Output: compensated[6]           │
└──────────────┬───────────────────┘
               │
               ▼
┌──────────────────────────────────┐
│   ForcePipeline (MODIFIED)        │
│   Butterworth2 → deadzone →       │
│   gradient-limit → force-map →    │
│   coord-transform → hapticOut     │
│                                   │
│  Deadzone reduced to 0.05N        │  (residual noise only; bias already removed)
└──────────────────────────────────┘
```

Compensation MUST happen before filtering because gravity/inertia are structured deterministic signals, not noise.

## 4. Calibration Model

### 4.1 Unknown Parameters (10 scalars)

| Symbol | Description | Units |
|--------|-------------|-------|
| m      | Load mass (tool + sensor end-effector) | kg |
| c_x, c_y, c_z | Center of mass in sensor frame | m |
| Fx₀, Fy₀, Fz₀ | Force sensor zero-bias | N |
| Mx₀, My₀, Mz₀ | Torque sensor zero-bias | Nm |

### 4.2 Measurement Model (Static, No External Force)

Force equation:
```
F_meas = m · R^T · g_base + F₀
```
where `g_base = (0, 0, -9.81)` m/s², and R is the rotation matrix from base to tool frame obtained from robot pose (Rx, Ry, Rz).

Torque equation:
```
M_meas = r_com × (m · R^T · g_base) + M₀
```

### 4.3 Linearization

Define auxiliary variable `p = m · r_com = (p_x, p_y, p_z)`.

The torque equation becomes linear in unknowns:
```
M_meas = (R^T · g_base) × p + M₀
```

Substituting the gravity vector `g_tool = R^T · g_base = (gx, gy, gz)`:
```
Mx_meas = gy·p_z - gz·p_y + Mx₀
My_meas = gz·p_x - gx·p_z + My₀
Mz_meas = gx·p_y - gy·p_x + Mz₀
```

### 4.4 Linear System Ax = b

Each static pose i contributes 6 equations. Unknown vector:
```
x = [m, p_x, p_y, p_z, Fx₀, Fy₀, Fz₀, Mx₀, My₀, Mz₀]^T  (10×1)
```

For pose i, with R_i and measured forces F_i, M_i:
- Force eqns (3 rows):
  ```
  Fx_i = m · gx_i             + Fx₀
  Fy_i = m · gy_i             + Fy₀
  Fz_i = m · gz_i             + Fz₀
  ```
- Torque eqns (3 rows) using p:
  ```
  Mx_i =          gy_i·p_z - gz_i·p_y + Mx₀
  My_i = gz_i·p_x - gx_i·p_z          + My₀
  Mz_i = gx_i·p_y - gy_i·p_x          + Mz₀
  ```

With N ≥ 2 poses (18+ equations for 10 unknowns), solve via normal equations:
- Build A (6N × 10), b (6N × 1)
- Form normal equations: AᵀA x = Aᵀb  (10×10 system)
- Solve via Gaussian elimination with partial pivoting (no external library needed)
- Extract m, F₀, M₀ from x
- Recover r_com = p / m

N = 6 poses recommended (36 equations) for noise robustness. The 10×10 system is well-conditioned for N ≥ 3 (30 equations).

## 5. Calibration Procedure

### 5.1 State Machine

```
IDLE → TARE → MOVE → SETTLE → SAMPLE  (loop 6x)  → SOLVE → VERIFY → DONE
  │                                      │            │        │
  └─ (any state) safety interrupt ──────┴────────────┴────────┴──→ ABORT
```

### 5.2 Phase Details

**TARE (2s):**
- Robot stationary; collect 50 raw samples
- Compute initial F₀, M₀ as simple mean (crude, refined later by SVD)

**MOVE (per pose, max 5s timeout):**
- Compute target pose offset from current TCP position (position unchanged, rotation only)
- 6 target orientations cover orthogonal gravity-vector projections:
  ```
  {0,0,0}, {+15,0,0}, {-15,0,0}, {0,+15,0}, {0,-15,0}, {0,0,+15}  deg
  ```
- Send ServoP at 30% speed factor
- Safety: each target clamped through `SafetyBoundary::clampToBoundary()`

**SETTLE (0.5s):**
- Wait for mechanical vibration to decay after move completes

**SAMPLE (0.5s, ~60 samples at 125Hz):**
- Record `{raw[6], pose{Rx,Ry,Rz}}` tuples
- Compute per-pose median to reject outliers

**SOLVE:**
- Build A, b from N-pose medians (6N rows × 10 cols)
- Form normal equations AᵀA x = Aᵀb (10×10 system)
- Gaussian elimination with partial pivoting
- Extract parameters; compute residual RMS

**VERIFY:**
- Residual RMS < 0.3N → PASS
- Residual RMS ≥ 0.3N → warn user, option to RETRY or accept

**DONE:**
- Persist to `Touch_Client/calib/force_calib.json`

### 5.3 Safety Constraints During Calibration

| Constraint | Value | Mechanism |
|-----------|-------|-----------|
| Speed factor | 0.30 | `m_stateMachine.speedFactor()` override during calib |
| Max step | 1.5 mm/frame | Half of normal 3mm limit |
| Pose timeout | 5 s | Skip to next pose on timeout; ≥3 skips → ABORT |
| Boundary guard | Full | `SafetyBoundary::clampToBoundary()` on every target |
| Alarm guard | Immediate | `RobotMode() == 9` → ABORT, disable robot |
| User interrupt | Space key / button release | Checked each frame in MOVE/SETTLE; → ABORT |
| Movement range | ±15° rotation only | Center position unchanged from current TCP |
| Pre-check: connected | Required | Robot must be connected and enabled |
| Pre-check: not in alarm | Required | Mode != 9 |
| Pre-check: not transmitting | Required | User must not be actively driving via Touch |
| Pre-check: in workspace | Required | Position within SafetyBoundary |

## 6. Runtime Compensation

### 6.1 Motion Estimator

Position history ring buffer (size 5) for central-difference acceleration:
```
v[k] = (x[k] - x[k-1]) / Δt
a[k] = (x[k] - 2·x[k-1] + x[k-2]) / Δt²
```

Acceleration passed through 10 Hz 2nd-order Butterworth LPF to suppress differentiation noise amplification.

### 6.2 Motion State Classification

| State | Criteria | Bias Update |
|-------|----------|-------------|
| STILL | |v| < 2 mm/s AND |a| < 5 mm/s² | EMA α=0.01 |
| MOVING | otherwise | Frozen |

### 6.3 Compensation Equations

Gravity force in tool frame:
```
g_tool = R_base_to_tool^T · (0, 0, -9.81)
F_gravity = m · g_tool
```

Gravity torque:
```
M_gravity = r_com × F_gravity
```

Inertial force (point mass, MOVING only):
```
F_inertia = m · a_tool
```

Total compensated output:
```
F_comp = F_raw - F_bias - F_gravity - F_inertia
M_comp = M_raw - M_bias - M_gravity
```
(Inertial torque ignored — point-mass model has zero rotational inertia)

### 6.4 Online Bias Drift Tracking (STILL only)

```
F_bias[k] = F_bias[k-1] + α · (F_raw - F_gravity - F_bias[k-1])
M_bias[k] = M_bias[k-1] + α · (M_raw - M_gravity - M_bias[k-1])
```
where α = 0.01 (~5s time constant at 125Hz).

Purpose: track slow sensor drift (temperature, electronics warm-up).

## 7. Parameter Persistence

### 7.1 File Format

`Touch_Client/calib/force_calib.json`:
```json
{
  "version": 1,
  "mass_kg": 2.34,
  "com_sensor_m": [0.012, -0.005, 0.080],
  "bias_force_n": [-0.65, -1.07, 0.055],
  "bias_torque_nm": [-0.02, 0.02, 0.005],
  "residual_rms_n": 0.15,
  "num_poses": 6,
  "timestamp": "2026-07-26T10:30:00"
}
```

### 7.2 Loading

- On startup, `ForceCompensation::init()` loads the file
- If file missing or version mismatch → all params = 0 (no compensation), HUD shows "Force: uncalibrated" warning
- Calibration flag stored in `AppState::ForceData::isCalibrated`

## 8. File Changes

| File | Action | Purpose |
|------|--------|---------|
| `force/ForceCalibration.h` | **NEW** | Calibration state machine, data collection, SVD solver |
| `force/ForceCalibration.cpp` | **NEW** | Implementation |
| `force/ForceCompensation.h` | **NEW** | Motion estimator, gravity/inertia comp, EMA bias tracker |
| `force/ForceCompensation.cpp` | **NEW** | Implementation |
| `force/ForcePipeline.h` | MODIFY | `step()` accepts pre-compensated data; remove internal deadzone |
| `force/ForcePipeline.cpp` | MODIFY | Skip bias-related processing; reduced residual deadzone (0.05N) |
| `core/AppState.h` | MODIFY | `ForceData`: add `compensated[6]`, `isCalibrated`, calib params struct |
| `config/Config.h` | MODIFY | Remove `FORCE_DEADZONE_N`; add calib/runtime constants |
| `RelayCore.h` | MODIFY | Add `startCalibration()` / `isCalibrating()` |
| `RelayCore.cpp` | MODIFY | ForceReader thread passes pose to compensation; calib key binding |
| `main.cpp` | MODIFY | Load calib file on startup; wire key for calib trigger |
| `tests/test_force_compensation.cpp` | **NEW** | Unit tests for gravity comp, motion estimator, SVD solver |

## 9. Configuration Constants

### Removed
- `FORCE_DEADZONE_N` — replaced by bias calibration + residual 0.05N deadzone

### Added
```cpp
// Calibration
const double FORCE_CALIB_SPEED_FACTOR = 0.30;
const double FORCE_CALIB_STILL_COLLECT_S = 2.0;
const double FORCE_CALIB_MOVE_TIMEOUT_S = 5.0;
const double FORCE_CALIB_SETTLE_TIME_S = 0.5;
const double FORCE_CALIB_SAMPLE_TIME_S = 0.5;
const double FORCE_CALIB_MAX_RESIDUAL_N = 0.3;
const double FORCE_CALIB_POSE_ANGLE_DEG = 15.0;
const int    FORCE_CALIB_NUM_POSES = 6;

// Runtime
const double FORCE_MOTION_VEL_THRESH_MS = 0.002;
const double FORCE_MOTION_ACC_THRESH_MSS = 0.005;
const double FORCE_BIAS_EMA_ALPHA = 0.01;
const double FORCE_ACC_FILTER_CUTOFF_HZ = 10.0;
const double FORCE_RESIDUAL_DEADZONE_N = 0.05;
```

## 10. Key Bindings

| Key | Context | Action |
|-----|---------|--------|
| `c` | Idle (not transmitting) | Start force calibration |
| Space | During calibration | Abort calibration |

## 11. Acceptance Criteria

1. After calibration, no-load readings: |F_comp| < 0.1N on all axes, |M_comp| < 0.02 Nm
2. Calibration residual RMS < 0.3N (otherwise warning)
3. `--no-robot` mode starts without crash (uncalibrated, no compensation)
4. Stale detection still works (200ms timeout → force zero)
5. Safety interrupts work (space key, alarm detection)
6. Calibration file persists across restarts
7. HapticCallback receives zero force when robot is stationary and unloaded
8. Existing Butterworth filter pipeline still functions correctly after modifications
