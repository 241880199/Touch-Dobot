# Task 9 Report — 启动零偏漂移检查

**Status:** DONE (with one noted design concern)
**Commit:** `e016244` — `feat(force): check the stored zero for drift at startup, without blocking`
**Files changed:** `Touch_Client/config/Config.h`, `Touch_Client/main.cpp` (2 files, +66 lines, nothing else staged)

---

## What was added

### 1. `config/Config.h` — threshold constant

`FORCE_ZERO_DRIFT_WARN_N = 0.5` inserted immediately after `FORCE_RESIDUAL_DEADZONE_N`
(line 85), with the brief's comment verbatim. No existing threshold touched.

### 2. `main.cpp` — startup state + one-shot check (new block at line ~578)

Placed between the close of `namespace BiasCheck` and `// ===== 采集类模式互斥 =====`,
i.e. deliberately **outside** `BiasCheck` (this check is a startup check, not a capture
mode). Added verbatim from the brief:

- `g_zeroCheckDone`, `g_zeroCheckStartMs`, `g_zeroCheckAccum[3]`, `g_zeroCheckCount`
- **new** file-level `g_hasStoredZeroCalib` (`= false`), declared with the state above
- `runZeroDriftCheck(bool hasStoredZero)`

### 3. `main.cpp` — `g_hasStoredZeroCalib` set site

In the `force_calib.json` load block in `main()` (now line 1322, the `CalibStore::fileFor`
form Task 8 left behind):

```cpp
if (ForceCalibration::loadFromFile(CalibStore::fileFor("force_calib.json"),
                                   massKg, biasF, biasM)) {
    g_hasStoredZeroCalib = true;   // 有存储零偏, 启动漂移检查才有得比
```

Set only in the **success** branch — with no stored zero there is nothing to compare
against, and the check correctly stays silent.

### 4. `main.cpp` — call site in `idle()`

Inside `if (!appState.isClosing)` → `if (!g_noRobot)` → **immediately after**
`RelayCore::instance().pollForce()` (line 677). Ordering matters and is preserved: the
check reads `forceData` that `pollForce()` just refreshed; the copy is taken under
`appState.forceDataMutex` like the adjacent `BiasCheck::sample` snapshot.

**Step 3 was not executed** (brief marks it 作废, controller note confirms `signOk` and the
sign probe are gone). No sign-related text was searched for or rewritten.
`logCalibAttempt()` / `solveAndApply()` untouched. No restructuring of `main.cpp`,
no `BiasCheck` refactor, no pipeline change.

---

## The `g_noRobot` judgment call — decision and reasoning

**Choice: set the flag and return early.**

```cpp
if (g_zeroCheckDone || !hasStoredZero) return;
// --no-robot 下没有力数据可读, 本就没有可查的东西: 直接定稿, 免得每帧空转。
if (g_noRobot) { g_zeroCheckDone = true; return; }
```

Reasoning, in two parts:

1. **The re-entry the brief worried about is real in principle but currently unreachable.**
   `pollForce()` sits inside `idle()`'s `if (!g_noRobot)` block, and the brief pins the call
   site "immediately after `pollForce()`" — so the call inherits that guard. At today's only
   call site, `g_noRobot` is always false and the branch never fires.
2. **It still deserves the fix, because the current placement is the only thing making it
   unreachable.** A startup diagnostic placed next to the other startup force code is a
   plausible thing to move later (e.g. out of the robot-only block to keep all startup
   diagnostics together), and the moment it moves the flag-less version spins every idle
   frame forever. Setting the flag makes the function correct for any call site instead of
   correct-by-accident at one.

I chose early-return-with-flag over the brief's `if (!g_noRobot) { … }` wrapper because it
matches the function's existing shape (the first line is already an early return), keeps the
body at one indentation level, and makes the intent — "no robot means there is nothing to
measure, so this check is finished, not pending" — explicit on one line instead of implied
by a wrapper.

Verified after the change that `g_noRobot` is a `static bool` at `main.cpp:28`, i.e. in
scope and initialized before `idle()` can run.

---

## Verification

### Build (hard gate) — PASS

```
cmd.exe //c "D:\Projects\Touch\Touch_Client\build.bat"
```

Actual output (tail):

```
  RobotStateMachine.cpp
  SafetyPredictor.cpp
  SingularityAvoidance.cpp
  正在生成代码...
  Touch_Client.vcxproj -> D:\Projects\Touch\Touch_Client\x64\Release\Touch_Client.exe
  Build OK.

[2/2] Copying DLLs...
  DLLs copied.
[3/3] Copying models...
  Models copied.

Build complete. Run: D:\Projects\Touch\Touch_Client\x64\Release\Touch_Client.exe
```

`Touch_Client.exe` confirmed **not** running before the build
(`tasklist //FI "IMAGENAME eq Touch_Client.exe"` → "No tasks are running which match the
specified criteria"), so no `LNK1168`.

Confirmed `main.cpp` was genuinely recompiled in this build, not skipped as up-to-date:
`x64/Release/main.obj` is newer than `main.cpp`, and the .exe was relinked at 11:45:25
after the edit at 11:45:16. (This is exactly the trap the task warned about — a green run
that never compiled the changed file — so I checked it explicitly rather than trusting
`Build OK.` alone.)

Only pre-existing warnings appeared (the known `CALLBACK` macro redefinition between
`minwindef.h` and OpenHaptics' `glut.h`, in `SceneRenderer.cpp`), no new ones from my files.

### Tests — PASS

```
cmd.exe //c "D:\Projects\Touch\Touch_Client\tests\run_tests.bat"   →  EXIT=0
```

All 11 suites green, 138 tests, 0 failed:

| suite | result |
|---|---|
| test_force_pipeline | 5 passed, 0 failed |
| test_constraint_force | 7 passed, 0 failed |
| test_safety_core | 8 passed, 0 failed |
| test_feedback_parser | 28 passed, 0 failed |
| test_escalation | 15 passed, 0 failed |
| test_kinematics | 18 passed, 0 failed |
| test_coord_safety | 27 passed, 0 failed |
| test_force_compensation | 8 passed, 0 failed |
| test_relay_command_parser | 11 passed, 0 failed |
| test_force_logger | 4 passed, 0 failed |
| test_tcp_calibration | 7 passed, 0 failed |

No test target covers `main.cpp` in this repo; none was invented. The build gate above is
the real evidence for this change.

### Commit hygiene

`git add Touch_Client/config/Config.h Touch_Client/main.cpp` — explicit paths, not
`-A`. `git show --stat e016244` confirms exactly those 2 files. The previously-modified
`Docs/superpowers/plans/2026-09-18-calib-lifecycle.md` was left unstaged, and the untracked
runtime artifacts (`alarms.log`, `force_demo_log.csv`, `robot_diagnostics.log`,
`Touch_Client/calib/`, `tests/_build_sa.*`, `_hello.cpp`, `_rebuild_and_test.bat`) remain
untracked. Not pushed.

---

## Self-review findings

1. **The brief's premise checks out, and is actually stronger than stated.**
   `force/ForcePipeline.cpp:86` confirms `fd.filtered[i] = g_filters[i].step(fd.compensated[i])`
   (post-compensation → low-pass). I additionally verified that
   `FORCE_RESIDUAL_DEADZONE_N` is applied **only** in `mapForceToTouch()` (line 73, the
   Touch-output mapping path), *not* to `compensated[]` or `filtered[]`. So there is no
   deadzone clamping the value this check reads — a sub-0.20 N drift is reported as its true
   magnitude rather than 0. (It would be below the 0.5 N threshold either way, so this does
   not change behaviour; it just means the printed number is honest.)
2. **`AppState.h:117`'s comment** ("Butterworth 低通滤波输出") indeed omits "post-compensation".
   Left unchanged as instructed; noting it here only so the next reader has the pointer.
3. **The `< 10` samples guard is safe in the normal case.** The sampling window is the 1s
   between t=2000ms and t=3000ms, and `pollForce()` self-throttles to ~30Hz, so ~30 samples
   accumulate — comfortably above 10. If the stream were stale for that entire second,
   `g_zeroCheckDone` is set with `count == 0` and nothing is printed: silently no conclusion
   rather than a false pass, which matches the brief's stated intent.
4. **`sqrt` is unqualified** as in the brief; `<cmath>` was already included and the project
   build confirms it resolves in this TU (it is also used unqualified elsewhere in the file).
5. Confirmed I did not disturb the neighbouring `BiasCheck::sample(fdSnap)` snapshot block or
   the calibration-state-change tracking that follows it in `idle()`.

## Concerns

1. **(Minor, design — not fixed, out of scope) No stillness gate.** The check averages
   whatever `filtered[]` holds during its 1s window. If the operator starts jogging the arm
   within the first ~3s of startup, the window can contain motion rather than a static pose —
   the very assumption ("任意静止姿态") the drift argument rests on. Inertial compensation
   should keep the residual small, so a false ⚠ is unlikely, but the check has no equivalent
   of `BiasCheck`'s stillness/variance test to refuse a bad window. A cheap guard would be to
   require the per-axis spread within the window to be below some epsilon. Deliberately not
   added: the brief specifies the body verbatim and the scope limit says not to add
   thresholds of my own.
2. **(Minor, informational) The check is startup-only and one-shot.** If the zero drifts
   later during a long session (the exact failure mode the design revision cites — temperature
   over time), nothing re-checks. `'z'` remains the manual remedy. This matches the brief
   (启动期一次性检查) and the "不阻断" design goal, so it is a statement of scope, not a defect.
3. **(Informational) The `g_noRobot` branch is currently unreachable** from the sole call site
   (see the judgment-call section). Kept as defence in depth; if a future reader prefers "no
   dead code" over "safe to move", deleting the line is safe *today* and only today.
