# Payload-calib baseline from the robot's own reported payload

Branch `feat/pen-clamp-redesign`, base `156660c`, commit **`042d774`**
`fix(payload-calib): take the solve baseline from the robot's own reported payload`

## What changed

The solver measures `dp = m_true·c_true − m_cfg·c_cfg` (a **difference**). Folding it back to an
absolute centroid needs `m_cfg·c_cfg` — the baseline. Previously `solveAndApply()` used
`PayloadCalibration::effective()`, i.e. our client-side belief, which carries the `centerZ`
sign-fold ambiguity (~125 mm apart). The robot reports its actual baseline in the 30004 frame
(`Load` @1168, `CenterX/Y/Z` @1176) — `RelayCore.cpp` was already reading those four doubles and
throwing them away. They are now stored and used as the baseline.

### Files and lines

**`Touch_Client/core/AppState.h`** (struct `ForceData`, ~L121-135)
Replaced the "已删: payloadEcho[4]" tombstone comment with three new fields +
documentation, keeping the note that the old sign-probe consumer was deleted:
```cpp
        bool   payloadEchoValid = false;
        double payloadEchoLoadKg = 0.0;
        double payloadEchoCenterMm[3] = {0, 0, 0};
```

**`Touch_Client/relay/RelayCore.cpp`** (`forceReaderThread`, one-shot echo block, ~L99-107)
Kept the existing `sane` check and the existing print verbatim; inside the same `if (sane)` block,
after the print, added the store under `app.forceDataMutex`:
```cpp
                    EnterCriticalSection(&app.forceDataMutex);
                    app.forceData.payloadEchoLoadKg = echo[0];
                    for (int i = 0; i < 3; i++) {
                        app.forceData.payloadEchoCenterMm[i] = echo[1 + i];
                    }
                    app.forceData.payloadEchoValid = true;
                    LeaveCriticalSection(&app.forceDataMutex);
```

**`Touch_Client/main.cpp`** (`BiasCheck::solveAndApply()`)
- ~L400-431: baseline selection. Snapshot `payloadEchoValid` / mass / center under
  `appState.forceDataMutex`. If valid → use the robot's report. If not → fall back to
  `PayloadCalibration::effective()` **and print a prominent warning** (no silent fallback).
- ~L446-449: `PayloadCalibration::solve(...)` now receives a literal **`+1.0`** in the
  `comSignZ` slot, with a comment saying a faithful baseline leaves nothing to fold, and that
  the fallback path also passes `+1.0` (the warning already fired there).
- ~L463-465: new baseline line in the solve output, naming the source.

## Exact new console lines

Solve output, robot-report path:
```
  基线:    机械臂自报 [30004 @1168]  load=0.406 kg  center=(0.3, -0.1, 68.7) mm
  质量:    当前 0.406 kg   →   修正 +0.010 kg   →   0.416 kg
```

Solve output, fallback path (printed *before* the result block, when `'s'` is pressed):
```
[BIAS] !! 【警告】机械臂未回读负载 (30004 @1168 没收到 / 不合理), 基线退回本客户端的信念值
[BIAS] !! 此路径下绝对质心 Z 再次带有 comSignZ 折叠歧义 (两种解释约差 125 mm),
[BIAS] !! 下面打印的 comZ 【不可】直接采信 —— 等机械臂回读可用后再求解。
```
and the corresponding source field renders as `★ 本客户端信念值 [机械臂未回读] ★`.

The pre-existing `[Relay] 机械臂实际负载: load=… center=…` one-shot line from `RelayCore.cpp` is
unchanged.

## Build / test evidence

- `tasklist //FI "IMAGENAME eq Touch_Client.exe"` → `No tasks are running` (no `LNK1168`).
- `build.bat` → `Build OK.` / `Touch_Client.vcxproj -> ...\x64\Release\Touch_Client.exe`
  (only the pre-existing `CALLBACK` macro-redefinition warnings).
- `tests\run_tests.bat` → every suite green, 0 failed:
  force_pipeline 5, constraint_force 7, safety_core 8, feedback_parser 28, escalation 15,
  kinematics 18, coord_safety 27, force_compensation 8, relay_command_parser 11,
  force_logger 4, tcp_calibration 7.
- `tests\build_payload_calibration_test.bat` then the exe → **`20 passed, 0 failed`**
  (that suite is not in `run_tests.bat`).

## `comSignZ` is now vestigial

`solve()`'s `signZ` parameter is still used internally in two places — the `cTrueZ[2]` pair
(both candidates, unchanged) and `cSend = cTrue / signZ` for `comMm[2]` — but the only caller now
passes `+1.0` unconditionally, so `cSend == cTrue` and the fold branch is dead in practice.
Everything else is storage/display only:

- `force/PayloadCalibration.cpp` — definition L18, persisted on save L281, restored on load L332.
- `main.cpp` L268 (multi-pose bias check header: `CZ符号=+1/-1`) and L1428 (startup
  `sign_z=+1/-1` in the applied-payload line).
- `main.cpp` ~L473 still prints `CZ 符号提示: 数据区分不了两种解释 …`, which was always a
  statement about the solver's two `cTrueZ` candidates; with a faithful baseline it is moot.
- `tests/test_payload_calibration.cpp` L299-318 (`fit_is_sign_independent` / `sensor_yaw_state`)
  exercises the load/save round-trip of the field.

Nothing was deleted, per scope; removing this (and its tests) is the deliberate follow-up.

## Concerns

1. **Behavioral edge case on `payload_calib.json`.** With `+1.0` hard-wired, a persisted
   `com_sign_z: -1.0` no longer flips `comMm[2]` at solve time, so the `com_mm` written back by
   `applyResult()`/`save()` would differ from the old build in that case. The working tree's
   `Touch_Client/calib/payload_calib.json` currently has `"com_sign_z": 1.0`, so no change is
   observable with today's file — but a checkout with `-1.0` would see the stored `com_mm_z`
   flip sign relative to the previous behavior. That is the intended consequence of "nothing to
   fold", not an accident, but it is the one place the change is not purely additive.
2. **The fallback is reachable only before the first 30004 frame, or never with the real robot.**
   `payloadEchoValid` is set once, on the first sane frame of the force-reader thread. If the user
   presses `'s'` before that frame arrives, the warning fires and `comZ` is untrustworthy as
   stated. There is no re-read or retry, so a transient bad first frame (e.g. `load > 5.0 kg`)
   permanently disables the robot baseline for that session — the print makes it visible, but a
   reconnect would be needed to recover.
3. **The stale `CZ 符号提示` text now overstates the uncertainty.** It says the symbol "现在会下发,
   必须核" and points at `ROBOT_PAYLOAD_SEED_CZ_MM`; with the robot's own baseline the printed
   `comZ` is the physical value directly. Leaving it is correct per the scope limit (it belongs to
   the vestigial `comSignZ` machinery), but it is now the most likely thing to mislead an operator.

## Not touched (per scope)

`payload_calib.json` / `force_calib.json` contents, the solver math in
`force/PayloadCalibration.cpp`, `ForceCompensation`, the A2 lifetime split, the `comSignZ`
machinery, and the two-candidate measurement. Only the three files
`Touch_Client/core/AppState.h`, `Touch_Client/relay/RelayCore.cpp`, `Touch_Client/main.cpp` were
staged and committed; nothing was pushed.
