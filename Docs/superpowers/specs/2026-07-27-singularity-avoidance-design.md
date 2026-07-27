# Singularity Avoidance via Null-Space Optimization

**Date:** 2026-07-27
**Status:** Phase 2 optimization (pending review)
**Branch:** master
**Scope:** Touch_Client (C++) + Relay_Station (MATLAB)

## Problem

The CR3 6-DOF arm frequently enters singular configurations that trigger
controller alarms (mode=9), requiring manual escape via `escapeSingularity()`.
The current defense — repulsive force fields + speed attenuation — is purely
reactive and does not prevent the arm from reaching dangerous states.

**Priority by frequency:**
1. **Shoulder singularity** (most common): end-effector near Z-axis, J1 loses lateral authority
2. **Elbow singularity**: arm fully extended, J3 near ±155°
3. **Wrist singularity**: J4 and J6 axes align, J5 → 0° or ±180°

## Core Insight

The system operates in three modes with varying redundancy:

| Mode | User DOFs | Redundant DOFs | Strategy |
|------|-----------|----------------|----------|
| Button 1 (position) | 3 (x,y,z) | 3 (orientation) | Null-space optimization of orientation |
| Button 2 (orientation) | 3 (Rx,Ry,Rz) | 3 (position) | Damp dangerous rotations + TCP micro-adjust |
| Button 1+2 (full) | 6 | 0 | Selective damping along singular directions |

**Key principle:** Actively reconfigure the arm to stay in well-conditioned
configurations — not merely warn about approaching singularities.

## Architecture

```
New module: safety/SingularityAvoidance

┌─────────────────────────────────────────────────────────┐
│ RelayCore::sendPosition()  (30Hz)                       │
│                                                         │
│  ┌──────────────────────────────────────────────────┐   │
│  │ SingularityAvoidance                             │   │
│  │                                                  │   │
│  │  optimizeOrientation()       ← position mode     │   │
│  │  dampOrientationMotion()     ← orientation mode  │   │
│  │  dampFullCommand()           ← combined mode     │   │
│  │                                                  │   │
│  │  computeNullSpaceGradient()  ← internal          │   │
│  │  computeJacobianSVD()        ← internal          │   │
│  │  sendWarning()               ← outputs W| proto  │   │
│  └──────────────────────────────────────────────────┘   │
│         │          │          │                          │
│         ▼          ▼          ▼                          │
│  ServoP()    constraintForce   W| → MATLAB               │
└─────────────────────────────────────────────────────────┘
```

## Files

| File | Action | Content |
|------|--------|---------|
| `safety/SingularityAvoidance.h` | **New** | Three public interfaces + internal helpers |
| `safety/SingularityAvoidance.cpp` | **New** | Null-space projection, SVD damping, gradient optimization |
| `relay/RelayCore.cpp` | **Modify** | Call avoidance APIs in `sendPosition()` |
| `safety/ConstraintForce.cpp` | **Modify** | Amplify singular repulsion in orientation mode |
| `Relay_Station/relay_gui.m` | **Modify** | Parse `W|` protocol, display warning panel |

## Mode 1: Position Control (Button 1)

### Algorithm: Null-Space Orientation Optimization

```
Input:  targetPos (x,y,z), currentJoints[6]
Output: optimalOrient (Rx,Ry,Rz) — orientation that maximizes safety

1. Position IK with DLS:
   Kinematics::inverse(targetPos, currentJoints, q_feasible)
   If IK fails → return current orientation (no optimization)

2. Build position Jacobian J_p (3×6):
   Take first 3 rows of the full 6×6 geometric Jacobian

3. Null-space projector (3×3 inversion, O(54) ops):
   M = J_p * J_p^T          (3×3)
   M_inv = M^{-1}           (Gaussian elimination, 3×3)
   N = I_6 - J_p^T * M_inv * J_p

4. Multi-objective gradient g[6]:

   a) Shoulder safety — push elbow away from Z-axis:
      f = -1 / (r_elbow_xy + ε)
      g_shoulder = ∂f/∂q via elbow Jacobian

   b) Elbow bend — keep J3 centered:
      f = -(q[2])^2          // J3_mid = 0° is optimal
      g_elbow = [0, 0, -2*q[2], 0, 0, 0]

   c) Joint limits — repulsion within 10°:
      For each joint i:
        margin = min(|q[i] - low|, |high - q[i]|)
        if margin < 10°: f += -1/(margin + 0.1)
        g[i] = 1/(margin + 0.1)^2 * sign(direction)

   d) Wrist symmetry — keep J4 and J6 axes misaligned:
      f = -|z4 · z6|
      g_wrist = ∂f/∂q[3:5]

   Total: g = w1*g_shoulder + w2*g_elbow + w3*g_joint + w4*g_wrist

5. Null-space iteration (10-20 rounds):
   for iter in 0..15:
       g_null = N * g(q)
       q += α * g_null
       clamp q to joint limits
       if ||g_null|| < 1e-3: break
       recompute J_p, N  (q has changed)

6. FK for orientation:
   Kinematics::composeTransform(q, T)
   Rx, Ry, Rz extracted from T[4][4] rotation submatrix

7. Return (Rx, Ry, Rz)
```

### Integration in sendPosition()

```cpp
// Old: static orientation
double targetRx = app.robotBaseRx, targetRy = app.robotBaseRy, targetRz = app.robotBaseRz;

// New: null-space optimized orientation
if (appState.lastButtonState && !m_transmittingOrient) {
    double curJoints[6] = {app.robotActualPose.j1, ...};
    Vec3 optOrient = SingularityAvoidance::optimizeOrientation(clamped, curJoints);
    targetRx = optOrient.x; targetRy = optOrient.y; targetRz = optOrient.z;
}
```

### Expected Behavior

Operator pulls stylus toward base → arm wrist/elbow autonomously rotate to
keep elbow r_xy large and J3 centered. End-effector position tracks the
stylus faithfully. Operator feels no difference.

## Mode 2: Orientation Control (Button 2)

### Algorithm: Dual-Layer Protection

```
Input:  targetOrient, userDeltaOrient, frozenTCP, currentJoints[6]
Output: dampedOrient, tcpAdjust, constraintForce, warnings

══════════════════════════════════════
Layer 1: Wrist Singularity Damping
══════════════════════════════════════

1. Extract wrist Jacobian J_w (3×3):
   Columns 3-5 of the orientation Jacobian (J4, J5, J6 rotation axes)

2. SVD of J_w (3×3 Jacobi eigendecomposition):
   J_w^T * J_w → eigenvalues → σ₁ ≥ σ₂ ≥ σ₃
   cond_wrist = σ₁ / σ₃

3. Continuous damping (replaced 3-tier, Phase 2):

   t = clamp((cond − 20) / (150 − 20), 0.0, 1.0)
   β = 1.0 − t²                               // quadratic: gentle onset, rapid near danger

   v_min = eigenvector of σ₃ (most singular direction in J4/J5/J6 space)
   u_min = normalize(J_w * v_min)              // map to task-space rotation direction

   Δθ_parallel = (Δθ · v_min) * v_min          ← damped direction
   Δθ_perp     = Δθ − Δθ_parallel              ← passes through
   Δθ_damped   = Δθ_perp + β * Δθ_parallel

   Directional repulsion force (new, Phase 2):
   proj = dot(deltaOrient, u_min)              // user rotation into danger
   if proj > 0 and cond > 20:
       repMag = (1.0 − β) * SINGULAR_FORCE_MAX_N * min(proj / 5.0, 1.0)
       repulsionForce = repMag * (−u_min)      // oppose dangerous rotation

4. Send warning if β < 1.0:
   W|1,wrist,J5接近对正点(X.X°),请减慢绕此方向的姿态旋转,cond,β

══════════════════════════════════════
Layer 2: TCP Position Micro-Adjust
══════════════════════════════════════

5. Check shoulder safety at frozen TCP:
   Compute elbow position via FK: r_elbow_xy = sqrt(J2_pos.x² + J2_pos.y²)

   if r_elbow_xy < SHOULDER_SAFE_R (e.g., 100mm):

      Orientation Jacobian J_o (3×6) — last 3 rows of full Jacobian
      Null-space projector: N_o = I_6 - J_o^T(J_o J_o^T + εI)^{-1}J_o

      Shoulder repulsion gradient g_shoulder[6]:
      Push J2, J3 to increase elbow r_xy (same multi-objective as Mode 1,
      but with amplified weights)

      g_null = N_o * g_shoulder

      Cap position change:
      posAdjust_mag = ||J_p * g_null||
      if posAdjust_mag > MAX_POS_ADJUST (5mm):
          g_null *= MAX_POS_ADJUST / posAdjust_mag

      q_adjusted = currentJoints + g_null
      posAdjust = FK(q_adjusted) - frozenTCP

      if ||posAdjust|| > 3mm:
          W|0,tcp_adjust,TCP微调+X.Xmm保持安全,位置安全无需手动操作,X.X

   else:
      posAdjust = (0, 0, 0)

══════════════════════════════════════
Layer 3: Enhanced Force + Warning
══════════════════════════════════════

6. Constraint force amplification:
   - Wrist danger: repulsion proportional to (1-β) * 2.0N
     Direction: Touch space force opposing the dangerous rotation
   - Shoulder danger: reuse ConstraintForce::computeSingularForce()
     but amplify by 1.5× in orientation mode
   - Combined force clamped to 3.3N max

7. Send warning if frozen TCP position is inherently risky:
   if r_elbow_xy < 50mm:
       W|2,shoulder,TCP距Z轴仅XXmm,建议松开按钮2先移动TCP远离底座再旋转,dist,safety
```

### Integration in sendPosition()

```cpp
if (m_transmittingOrient && m_orientValid) {
    Vec3 deltaOrient(drx, dry, drz);
    Vec3 posAdjust;
    double curJoints[6] = {app.robotActualPose.j1, ...};
    Vec3 currentTcp(clamped.x, clamped.y, clamped.z);

    Vec3 damped = SingularityAvoidance::dampOrientationMotion(
        m_targetOrient, deltaOrient, currentTcp, curJoints, posAdjust);

    m_targetOrient = damped;
    clamped.x += posAdjust.x; clamped.y += posAdjust.y; clamped.z += posAdjust.z;
}
```

## Mode 3: Combined Position+Orientation (Buttons 1+2)

### Algorithm: Selective SVD Damping

6 DOFs all commanded — no null space. Trade off tracking in the most
dangerous direction.

```
Input:  userDelta6D (Δx,Δy,Δz, ΔRx,ΔRy,ΔRz), currentJoints[6]
Output: dampedDelta6D

1. Compute full Jacobian J(6×6) at currentJoints

2. SVD: J = U Σ V^T
   Σ = diag(σ₁, σ₂, σ₃, σ₄, σ₅, σ₆),  σ₁ ≥ σ₂ ≥ ... ≥ σ₆
   U = [u₁ u₂ ... u₆]  (task-space left singular vectors)
   V = [v₁ v₂ ... v₆]  (joint-space right singular vectors)

3. Compute condition number:
   cond = σ₁ / σ₆
   if cond < COND_FULL_WARN (30): return userDelta6D unchanged

4. For each singular value σᵢ:
   ratioᵢ = σᵢ / σ₁

   if ratioᵢ < SINGULAR_RATIO_THRESHOLD (0.05):
      β = smoothstep(ratioᵢ / SINGULAR_RATIO_THRESHOLD)
      // linear ramp: ratio=0.05→β=1, ratio=0→β=0

      Project user command onto uᵢ (task-space direction):
         component = (userDelta6D · uᵢ) * uᵢ

      Damp:
         dampedDelta6D += β * component
         (perpendicular components pass through undamped)

      if β < 0.5:
         Generate warning describing which physical direction is damped

5. Direction descriptions (human-readable mapping):

   | uᵢ dominant axis | Meaning                   | Suggestion                    |
   |------------------|---------------------------|-------------------------------|
   | X/Y translation  | Lateral motion restricted | "Pull away from Z-axis first" |
   | Rx/Ry rotation   | Wrist approaching sing.   | "Avoid rotating this way"     |
   | Rz rotation      | J6 near limit             | "Rotate Rz opposite direction"|
```

### Direction Mapping

For each damped uᵢ, identify the Cartesian DOF with the largest absolute
component and generate a warning:

```
W|1,full_damp,全控模式{uᵢ描述}阻尼{1-β:.0%},{建议},{ratio},{β}
```

## Warning Protocol: `W|`

### Format

```
W|<level>,<type>,<message>,<suggestion>,<param1>,<param2>
```

| Field | Type | Description |
|-------|------|-------------|
| level | int | 0=INFO (blue), 1=WARN (orange), 2=CRITICAL (red) |
| type | string | `shoulder`, `wrist`, `elbow`, `joint`, `tcp_adjust`, `full_damp` |
| message | string | Human-readable problem description |
| suggestion | string | Actionable advice for the operator |
| param1 | float | Key metric (e.g., distance, angle, damping ratio) |
| param2 | float | Secondary metric (e.g., speed factor, condition number) |

### Examples

```
W|2,shoulder,肘部距Z轴仅45mm迫近奇异,请松开按钮反向拉动TCP远离底座,45.2,0.30
W|1,wrist,腕部J5接近对正点(3.5°),请减慢绕此方向的姿态旋转,3.5,0.50
W|0,tcp_adjust,TCP微调+2.8mm保持安全构型,当前位置安全无需手动操作,2.8,0.00
W|1,full_damp,全控模式横向运动阻尼40%,底座上方请避免大幅横向移动,40.0,0.60
W|1,elbow,J3接近完全伸展140°,请避免继续向外推笔,140.5,0.50
W|2,joint,J3距限位仅2.1°即将报警,请立即反向转动,2.1,0.00
```

### C++ Sender

```cpp
// SingularityAvoidance.cpp — helper
static void sendWarning(int level, const char* type, const char* msg,
                        const char* suggestion, double param1, double param2) {
    char buf[256];
    snprintf(buf, sizeof(buf), "W|%d,%s,%s,%s,%.1f,%.1f",
             level, type, msg, suggestion, param1, param2);
    RelayCore::instance().sendRelayUpdate(buf);
}
```

### MATLAB Parser (relay_gui.m)

```matlab
% State field
S.warnings = {};     % cell array of warning structs
S.warning_count = 0;

% In protocol parser loop:
elseif startsWith(msg, 'W|')
    parts = split(msg(3:end), ',');
    if numel(parts) >= 4
        w.level = str2double(parts{1});
        w.type = parts{2};
        w.message = parts{3};
        w.suggestion = parts{4};
        if numel(parts) >= 5, w.param1 = str2double(parts{5}); else w.param1 = 0; end
        if numel(parts) >= 6, w.param2 = str2double(parts{6}); else w.param2 = 0; end
        S.warning_count = S.warning_count + 1;
        S.warnings{S.warning_count} = w;
    end
```

### MATLAB Display (in refreshTimer callback)

Warnings displayed in the Safety panel, replacing or extending the
existing 5-line `lblSafety.Text`. When warnings are active, the panel
shows them prominently:

```matlab
% Build safety lines — warnings take priority
if S.warning_count > 0
    % Display up to 3 most recent warnings
    for i = max(1, S.warning_count-2) : S.warning_count
        w = S.warnings{i};
        colorChar = getWarningColorChar(w.level);  % ●/○/⚠
        % Warning level sets the overall safety label color
        if w.level == 2
            lblSafety.FontColor = clr.red;
        elseif w.level == 1 && st < 5
            lblSafety.FontColor = clr.orange;
        end
        safetyLines{end+1} = sprintf('%s %s', colorChar, w.message);
        safetyLines{end+1} = sprintf('  → %s', w.suggestion);
    end
end
```

Top-bar status label also reflects highest active warning level.

## Key Parameters (Phase 1 — superseded, see Phase 2 below)

| Parameter | Value | Description |
|-----------|-------|-------------|
| SHOULDER_SAFE_R | 120 mm | Min safe elbow r_xy before null-space pushes |
| SHOULDER_CRITICAL_R | 50 mm | TCP micro-adjust triggers in orient mode |
| ELBOW_MID_ANGLE | 0° | Optimal J3 position |
| JOINT_WARN_MARGIN | 10° | Start null-space repulsion from limit |
| MAX_POS_ADJUST | 5 mm | Max TCP micro-adjust in orient mode |
| COND_WRIST_WARN | 50 | Start damping wrist rotation |
| COND_WRIST_DAMP | 100 | Heavy damping |
| COND_WRIST_REJECT | 200 | Block dangerous axis |
| COND_FULL_WARN | 30 | Start SVD selective damping (combined mode) |
| SINGULAR_RATIO | 0.05 | σᵢ/σ₁ threshold for damping |
| NULLSPACE_ITERATIONS | 15 | Max gradient descent rounds |
| NULLSPACE_ALPHA | 0.1 | Gradient step size (rad) |
| w1 (shoulder) | 3.0 | Shoulder gradient weight |
| w2 (elbow) | 2.0 | Elbow gradient weight |
| w3 (joint_limits) | 5.0 | Joint limit gradient weight |
| w4 (wrist) | 1.0 | Wrist symmetry gradient weight |

## Safety Guarantees

1. **Position accuracy preserved**: Null-space projection mathematically
   guarantees position is unchanged (Mode 1). TCP micro-adjust is capped
   at 5mm (Mode 2).

2. **Existing SafetyPredictor remains active**: The 4-layer safety check
   (boundaries, IK, condition number, alarm blacklist) still runs every
   frame as a safety net.

3. **No new alarm triggers**: The avoidance system intervenes BEFORE the
   arm enters configurations that would cause Dobot controller alarms.
   `escapeSingularity()` is preserved as a fallback.

4. **Orientation clamping**: All optimized/damped orientations are
   clamped to `SAFE_RX/RY/RZ_MIN/MAX` bounds before ServoP.

5. **Gradient bounded**: Null-space gradient step is capped to prevent
   sudden orientation jumps.

## Build & Test

### Compilation
```
cd Touch_Client && build.bat
```

### Unit Tests (new)
```
test_singularity_avoidance.exe
  - test_null_space_projector     # verify N preserves position
  - test_shoulder_gradient        # verify gradient points away from Z
  - test_elbow_gradient           # verify gradient centers J3
  - test_wrist_damping            # verify SVD direction damping
  - test_full_svd_damping         # verify combined mode damping
  - test_orient_mode_position     # verify TCP micro-adjust ≤ 5mm
```

### Integration Test
1. Start MATLAB: `cd Relay_Station && matlab -r "relay_gui"`
2. Start C++: `cd Touch_Client\x64\Release && .\Touch_Client.exe`
3. Verify:
   - Button 1: pull stylus toward base → observe orientation auto-adjusting in 3D view
   - Button 2: rotate stylus near singular orientation → observe damping + TCP shift + warnings
   - Button 1+2: move all 6 DOFs near Z-axis → observe selective damping + warnings
   - No false alarm triggers (mode=9) during normal use

## Exclusions

- No changes to the robot-side Dobot controller firmware
- No changes to the C++ FK/IK URDF model (uses existing Kinematics APIs)
- `escapeSingularity()` retained as-unauthorized fallback only

---

## Phase 2: Optimization (2026-07-27)

**Problem from hardware testing:** Button 2 (orientation-only mode) still
frequently enters singular configurations. Root causes identified:

1. Damping thresholds too high — cond=50 before any intervention
2. Coarse 3-tier damping (0.5/0.2/0.0) creates jarring transitions
3. Shoulder safety only triggers at r_elbow < 50mm (critical) in orientation mode, not at 120mm (safe)
4. No haptic feedback for wrist alignment danger (Touch only has XYZ force, no torque)
5. Gradient step too small (0.1°/iter) for effective null-space reconfiguration
6. Safety gradient weights need amplification after null-space projection attenuation

### Changes

#### 1. Continuous Damping (replaces 3-tier)

```
Old:  cond<50→β=1.0, 50≤cond<100→β=0.5, 100≤cond<200→β=0.2, cond≥200→β=0.0
New:  cond<20→β=1.0, 20≤cond<150→β=1.0−(t²) where t=(cond−20)/(150−20)
                 cond≥150→β≈0.0 (smooth approach to zero)
```

Rationale: Quadratic decay (β = 1.0 − t²) provides gentle onset and rapid
drop near the danger zone. No discontinuous jumps.

#### 2. Directional Haptic Repulsion (new)

Since Touch only outputs 3D translation force, we map the wrist singular
direction into task space and apply a force opposing the user's rotation:

```
u_min = normalize(J_w * v_min)     // singular direction in task space (3D rotation)
proj  = dot(userDeltaOrient, u_min) // how much user rotates into danger
if proj > 0 and cond > 20:
    repulsionForce = (1.0 − β) * SINGULAR_FORCE_MAX_N * smoothstep(proj/5.0) * (−u_min)
```

This creates the sensation of "pushing against a spring" when rotating
toward a singular wrist configuration. The force direction is opposite to
the dangerous rotation, scaled by both proximity to singularity (β) and
rotation speed (proj).

#### 3. Enhanced Null-Space Gradient

| Change | Old | New | Why |
|--------|-----|-----|-----|
| Gradient step | 0.1°/iter | 0.3°/iter | Insufficient reconfiguration at old value |
| Max iterations | 15 | 20 | More convergence headroom |
| Shoulder trigger (orient mode) | r_elbow < 50mm | r_elbow < 120mm | Match safe threshold, not just critical |
| Shoulder weight | 3.0 | 4.0 | Compensate null-space projection attenuation |
| Wrist-shoulder coupling | None | Double gradient when cond>60 AND r_elbow<80mm | Address dual-singularity scenarios |
| Orient force amp | 1.5× | 2.5× | Stronger haptic signal in orientation mode |

#### 4. SVD Full-Command Damping (combined mode)

| Change | Old | New |
|--------|-----|-----|
| Trigger threshold | cond > 30 | cond > 15 |
| Damping function | linear: ratio/threshold | quadratic: ratio²/threshold² |

#### 5. New Files & Interfaces

| File | Change | Detail |
|------|--------|--------|
| `safety/SingularityAvoidance.h` | Modify | `dampOrientationMotion` returns `repulsionOut` Vec3 |
| `safety/SingularityAvoidance.cpp` | Refactor | Continuous damping, directional repulsion, enhanced gradient |
| `safety/ConstraintForce.h` | Add | `computeOrientationSingularForce()` declaration |
| `safety/ConstraintForce.cpp` | Add | Directional repulsion force computation |
| `core/AppState.h` | Add | `orientRepulsionForce[3]`, `hasOrientRepulsion`, mutex |
| `relay/RelayCore.cpp` | Modify | Pass repulsion force to AppState after `dampOrientationMotion` |
| `haptic/HapticCallback.cpp` | Modify | Read and superimpose `orientRepulsionForce` onto Touch output |
| `config/Config.h` | Modify | Updated thresholds + 2 new params |
| `tests/test_singularity_avoidance.cpp` | Add | 6 new test cases |

### Data Flow

```
HapticCallback (1kHz, stylus rotation delta)
    │
    ▼
RelayCore::sendPosition (30Hz)
    │ dampOrientationMotion(deltaOrient, joints)
    │   ├── returns dampedDelta ──→ ServoP command
    │   └── returns repulsionForce ──→ AppState.orientRepulsionForce
    │
    ▼
HapticCallback (1kHz, next frame)
    │ reads AppState.orientRepulsionForce
    │ superimposes onto totalForce[3]
    │ hdSetDoublev(HD_CURRENT_FORCE, totalForce)
    │
    ▼
Touch device — user feels directional resistance
```

### Updated Key Parameters

| Parameter | Old Value | New Value | Description |
|-----------|-----------|-----------|-------------|
| SHOULDER_SAFE_R | 120 mm | 120 mm | (unchanged) |
| SHOULDER_CRITICAL_R | 50 mm | 50 mm | (unchanged) |
| ELBOW_MID_ANGLE | 0° | 0° | (unchanged) |
| JOINT_WARN_MARGIN | 10° | 10° | (unchanged) |
| MAX_POS_ADJUST | 5 mm | 5 mm | (unchanged) |
| COND_WRIST_WARN | 50 | **20** | Early warning onset |
| COND_WRIST_DAMP | 100 | **deleted** | Replaced by continuous function |
| COND_WRIST_REJECT | 200 | **150** | Near-block zone |
| COND_FULL_WARN | 30 | **15** | Earlier combined-mode damping |
| SINGULAR_RATIO | 0.05 | 0.05 | (unchanged) |
| NULLSPACE_ITERATIONS | 15 | **20** | More convergence |
| NULLSPACE_ALPHA (GRAD_STEP) | 0.1 | **0.3** | Stronger gradient steps |
| w1 (shoulder) | 3.0 | **4.0** | Compensate projection attenuation |
| w2 (elbow) | 2.0 | 2.0 | (unchanged) |
| w3 (joint_limits) | 5.0 | 5.0 | (unchanged) |
| w4 (wrist) | 1.0 | 1.0 | (unchanged) |
| ORIENT_FORCE_AMP | 1.5 | **2.5** | Stronger singular force in orient mode |
| **NEW** SINGULAR_FORCE_MAX_N | — | **3.0 N** | Max directional repulsion force |
| **NEW** WRIST_ALIGN_REPEL_RANGE | — | **60** | cond threshold for max repulsion |

### New Test Cases

| Test | Validation |
|------|-----------|
| `test_continuous_damping_monotonic` | β decreases smoothly from 1.0 to 0.0 as cond increases 20→150 |
| `test_repulsion_direction_correct` | When J5≈0°, u_min aligns with wrist rotation axis; repulsion opposes user rotation |
| `test_repulsion_zero_in_safe_zone` | cond<20 → repulsionForce = (0,0,0) |
| `test_shoulder_early_trigger` | r_elbow<120mm activates TCP micro-adjust in orient mode |
| `test_dual_singular_warning` | cond>60 AND r_elbow<80mm sends dual-singular warning |
| `test_gradient_step_effective` | 0.3°/iter produces measurable (>1mm) null-space TCP adjustment |

### Safety Impact

All changes are **within the existing safety envelope**:
- Damping is conservative (never amplifies motion)
- Repulsion force capped at 3N → total Touch force ≤ 8N clamp
- Gradient step 0.3°/frame = 9°/s, well within robot velocity limits
- Existing 4-layer SafetyPredictor remains active as safety net

### Risks

| Risk | Mitigation |
|------|-----------|
| cond=20 threshold too sensitive (false damping) | β=1.0 at cond=20 (no actual damping, only repulsion force ramps from 0) |
| Directional repulsion pushes wrong way | Unit test validates u_min direction against known singular configs |
| Gradient instability at 0.3°/step | Per-frame single step (not iterative), 9°/s max joint velocity |
| Touch force clipping masks repulsion | computeTotalForce clamp is global; repulsion+radial ≤ 8N |

## Open Questions

1. Weight tuning (w1–w4): Updated values based on arm geometry analysis + null-space
   attenuation. May need further adjustment during integration testing on real hardware.

2. Continuous damping curve: Quadratic (β = 1−t²) is the initial choice. If it feels
   too aggressive near cond=20 or too weak near cond=120, can switch to cubic or
   other smoothstep variants based on operator feedback.

3. Directional repulsion gain: `SINGULAR_FORCE_MAX_N = 3.0N` is a starting point.
   May need tuning: too weak → operator ignores it; too strong → interferes with
   normal operation.

4. Orientation mode TCP adjustment cap: 5mm is retained. If the arm frequently
   needs >5mm to escape shoulder singularity, raise to 8mm with user notification.
