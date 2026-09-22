@echo off
setlocal enabledelayedexpansion
set "TESTDIR=D:\Projects\Touch\Touch_Client\tests"
set "PASSED=0"
set "FAILED=0"

echo ================================================
echo   Touch-Dobot Unit Tests
echo ================================================
echo.

rem ============================================================
rem HOW TEST RESULTS ARE JUDGED -- read this before editing any section.
rem   Every inner (test-result) check in this file is written as
rem
rem       if !ERRORLEVEL! EQU 0 (
rem
rem   with delayed expansion enabled by the "setlocal" on line 2 above.
rem   Do NOT rewrite it in either of these two tempting ways:
rem
rem   (a) if %ERRORLEVEL% EQU 0  -- WRONG. A %VAR% inside a parenthesised
rem       block is expanded when the block is PARSED, i.e. BEFORE the test
rem       exe has run, so it only ever sees the BUILD's exit code and even
rem       a failing suite prints [OK]. (Measured 2026-09-22 with a control
rem       that ran a stand-in returning 7: it still printed [OK].)
rem   (b) if errorlevel 1  -- WRONG. That means "errorlevel >= 1", and a
rem       CRASHING process reports a NEGATIVE code (0xC0000005 reads as
rem       -1073741819), so ">= 1" is false and a crashed suite prints [OK].
rem       (Measured 2026-09-22: a crash stand-in gave [OK] via this form.)
rem
rem   !ERRORLEVEL! is expanded at RUN time and compared numerically with
rem   EQU, so it is correct for passing, failing AND crashing suites.
rem   The OUTER (build-exit) checks are top-level statements, are NOT
rem   affected by delayed expansion, and are correct as they are.
rem
rem WHY "@echo off" FOLLOWS EVERY "call" BELOW (do not delete those lines):
rem   Each build script starts with "@echo on", and that switch is GLOBAL to
rem   the cmd session, so it survives the call. Without restoring it, this
rem   file's own lines get echoed into the output -- including "echo   [FAIL]"
rem   from the branch that was NOT taken. A fully GREEN log would then contain
rem   the literal text "echo   [FAIL]", a false alarm for anything scraping the
rem   log for that marker. Restoring echo off never suppresses the markers
rem   themselves: "echo x" still PRINTS x -- only the echoing of the command
rem   line is suppressed.
rem ============================================================

echo ================================================
echo   Force Compensation Tests
echo ================================================
echo.
echo --- Building test_force_compensation ---
call "%TESTDIR%\build_force_comp_test.bat"
@echo off
if %ERRORLEVEL% EQU 0 (
    echo   Build OK
    echo.
    echo === test_force_compensation.exe ===
    "%TESTDIR%\test_force_compensation.exe"
    if !ERRORLEVEL! EQU 0 (
        set /a PASSED+=1
        echo   [OK]
    ) else (
        set /a FAILED+=1
        echo   [FAIL]
    )
) else (
    echo   [FAIL: build error]
    set /a FAILED+=1
)
echo.

echo ================================================
echo   Relay / Logger / Calibration Standalone Tests
echo ================================================
echo.
echo --- Building test_relay_command_parser ---
call "%TESTDIR%\build_relay_command_test.bat"
@echo off
if %ERRORLEVEL% EQU 0 (
    echo   Build OK
    echo.
    echo === test_relay_command_parser.exe ===
    "%TESTDIR%\test_relay_command_parser.exe"
    if !ERRORLEVEL! EQU 0 (
        set /a PASSED+=1
        echo   [OK]
    ) else (
        set /a FAILED+=1
        echo   [FAIL]
    )
) else (
    echo   [FAIL: build error]
    set /a FAILED+=1
)
echo.
echo --- Building test_force_logger ---
call "%TESTDIR%\build_force_logger_test.bat"
@echo off
if %ERRORLEVEL% EQU 0 (
    echo   Build OK
    echo.
    echo === test_force_logger.exe ===
    "%TESTDIR%\test_force_logger.exe"
    if !ERRORLEVEL! EQU 0 (
        set /a PASSED+=1
        echo   [OK]
    ) else (
        set /a FAILED+=1
        echo   [FAIL]
    )
) else (
    echo   [FAIL: build error]
    set /a FAILED+=1
)
echo.
echo --- Building test_tcp_calibration ---
call "%TESTDIR%\build_tcp_calibration_test.bat"
@echo off
if %ERRORLEVEL% EQU 0 (
    echo   Build OK
    echo.
    echo === test_tcp_calibration.exe ===
    "%TESTDIR%\test_tcp_calibration.exe"
    if !ERRORLEVEL! EQU 0 (
        set /a PASSED+=1
        echo   [OK]
    ) else (
        set /a FAILED+=1
        echo   [FAIL]
    )
) else (
    echo   [FAIL: build error]
    set /a FAILED+=1
)
echo.

rem ============================================================
rem Suites wired into "build then run" on 2026-09-22:
rem   force_pipeline / feedback_parser / escalation / kinematics / coord_safety
rem   They used to be run by a first section that ran them "if the exe exists"
rem   and NEVER rebuilt them, so their binaries were months old. That first
rem   section has been deleted; these five are now built, then run.
rem The FORM of the result check is NOT special to these five -- every
rem section uses it. See "HOW TEST RESULTS ARE JUDGED" at the top of this
rem file for the rule and for the two tempting-but-wrong alternatives.
rem ============================================================

echo --- Building test_force_pipeline ---
call "%TESTDIR%\build_force_pipeline_test.bat"
@echo off
if %ERRORLEVEL% EQU 0 (
    echo   Build OK
    echo.
    echo === test_force_pipeline.exe ===
    "%TESTDIR%\test_force_pipeline.exe"
    if !ERRORLEVEL! EQU 0 (
        set /a PASSED+=1
        echo   [OK]
    ) else (
        set /a FAILED+=1
        echo   [FAIL]
    )
) else (
    echo   [FAIL: build error]
    set /a FAILED+=1
)
echo.

echo --- Building test_feedback_parser ---
call "%TESTDIR%\build_feedback_parser_test.bat"
@echo off
if %ERRORLEVEL% EQU 0 (
    echo   Build OK
    echo.
    echo === test_feedback_parser.exe ===
    "%TESTDIR%\test_feedback_parser.exe"
    if !ERRORLEVEL! EQU 0 (
        set /a PASSED+=1
        echo   [OK]
    ) else (
        set /a FAILED+=1
        echo   [FAIL]
    )
) else (
    echo   [FAIL: build error]
    set /a FAILED+=1
)
echo.

echo --- Building test_escalation ---
call "%TESTDIR%\build_escalation_test.bat"
@echo off
if %ERRORLEVEL% EQU 0 (
    echo   Build OK
    echo.
    echo === test_escalation.exe ===
    "%TESTDIR%\test_escalation.exe"
    if !ERRORLEVEL! EQU 0 (
        set /a PASSED+=1
        echo   [OK]
    ) else (
        set /a FAILED+=1
        echo   [FAIL]
    )
) else (
    echo   [FAIL: build error]
    set /a FAILED+=1
)
echo.

echo --- Building test_kinematics ---
call "%TESTDIR%\build_kinematics_test.bat"
@echo off
if %ERRORLEVEL% EQU 0 (
    echo   Build OK
    echo.
    echo === test_kinematics.exe ===
    "%TESTDIR%\test_kinematics.exe"
    if !ERRORLEVEL! EQU 0 (
        set /a PASSED+=1
        echo   [OK]
    ) else (
        set /a FAILED+=1
        echo   [FAIL]
    )
) else (
    echo   [FAIL: build error]
    set /a FAILED+=1
)
echo.

echo --- Building test_coord_safety ---
call "%TESTDIR%\build_coord_safety_test.bat"
@echo off
if %ERRORLEVEL% EQU 0 (
    echo   Build OK
    echo.
    echo === test_coord_safety.exe ===
    "%TESTDIR%\test_coord_safety.exe"
    if !ERRORLEVEL! EQU 0 (
        set /a PASSED+=1
        echo   [OK]
    ) else (
        set /a FAILED+=1
        echo   [FAIL]
    )
) else (
    echo   [FAIL: build error]
    set /a FAILED+=1
)
echo.

rem ============================================================
rem test_constraint_force: NOT BUILT AND NOT RUN BY THIS HARNESS.
rem   Its build script (build_constraint_test.bat) is excluded by
rem   Touch_Client/tests/.gitignore line 1, so it is NOT in HEAD and does
rem   not exist on a fresh clone. Nothing COMMITTED can build this suite's
rem   exe. (The ignored script DOES exist locally and DOES work -- which is
rem   exactly why this is a repository-hygiene gap, not a broken suite.)
rem   Running the exe here would present a stale binary as evidence -- the
rem   exact failure mode this harness was fixed to remove. Pending a
rem   decision on that .gitignore line.
rem   (Previously this suite WAS run here, from a prebuilt binary that
rem    nothing rebuilt. Removing it makes the gap visible instead of
rem    silently producing false evidence.)
rem   See also the [NOT RUN] block printed at the end of each run.
rem ============================================================

echo --- Building test_safety_core ---
call "%TESTDIR%\build_safety_core_test.bat"
@echo off
if %ERRORLEVEL% EQU 0 (
    echo   Build OK
    echo.
    echo === test_safety_core.exe ===
    "%TESTDIR%\test_safety_core.exe"
    if !ERRORLEVEL! EQU 0 (
        set /a PASSED+=1
        echo   [OK]
    ) else (
        set /a FAILED+=1
        echo   [FAIL]
    )
) else (
    echo   [FAIL: build error]
    set /a FAILED+=1
)
echo.

echo --- Building test_session_report ---
call "%TESTDIR%\build_session_report_test.bat"
@echo off
if %ERRORLEVEL% EQU 0 (
    echo   Build OK
    echo.
    echo === test_session_report.exe ===
    "%TESTDIR%\test_session_report.exe"
    if !ERRORLEVEL! EQU 0 (
        set /a PASSED+=1
        echo   [OK]
    ) else (
        set /a FAILED+=1
        echo   [FAIL]
    )
) else (
    echo   [FAIL: build error]
    set /a FAILED+=1
)
echo.

echo --- Building test_noise_probe ---
call "%TESTDIR%\build_noise_probe_test.bat"
@echo off
if %ERRORLEVEL% EQU 0 (
    echo   Build OK
    echo.
    echo === test_noise_probe.exe ===
    "%TESTDIR%\test_noise_probe.exe"
    if !ERRORLEVEL! EQU 0 (
        set /a PASSED+=1
        echo   [OK]
    ) else (
        set /a FAILED+=1
        echo   [FAIL]
    )
) else (
    echo   [FAIL: build error]
    set /a FAILED+=1
)
echo.

echo ================================================
echo   Tests complete
echo ================================================
echo ================================================
set /a TOTAL=PASSED+FAILED
echo ================================================
echo   Summary: %TOTAL% of 20 suites run - %PASSED% OK, %FAILED% FAILED
echo ================================================
echo.
rem ------------------------------------------------------------
rem [NOT RUN] -- the suites this harness does NOT build or run. Kept in the RUN
rem   LOG (not only in the source) so that a green run cannot be misread as
rem   "every suite in this repository is green".
rem
rem   The "20" and the "8" below are HAND-MAINTAINED -- nothing computes them.
rem   To recompute: test_*.cpp in the repo = 20; sections wired in below = 12;
rem   20 - 12 = 8 not run. If you wire one in, update BOTH numbers here AND the
rem   "12" that the MISMATCH check further down asserts.
rem ------------------------------------------------------------
echo   [NOT RUN] 8 of the 20 test_*.cpp in this repo are not built or run here:
echo.
echo     test_constraint_force
echo       Its build script is ignored by Touch_Client/tests/.gitignore line 1, so
echo       it is NOT in HEAD -- nothing COMMITTED can build this suite's exe.
echo       (Undecided; see the gap note further up this file.)
echo.
echo     test_payload_calibration
echo       Has a DELIBERATELY RED assertion: its fixture carries no reference-
echo       channel columns, so the case cannot be decided either way. Wiring it
echo       in would make this harness exit non-zero forever. Known, not a bug.
echo.
echo     test_calib_store / test_calibration / test_frame_layout /
echo     test_inertia_identification / test_self_collision /
echo     test_singularity_avoidance
echo       These six have WORKING build scripts and currently pass; nothing ever
echo       wired them in. Same orphan defect class this harness just fixed for
echo       five other suites -- they belong in the next plan, not silently absent.
echo.
echo   A green line above therefore means "the 12 suites that ran all passed".
echo   It does NOT mean every suite in this repository is green.
echo ================================================
echo.

rem ------------------------------------------------------------
rem Report the result to the CALLER, not only to stdout.
rem  * Every suite must be counted exactly once, so PASSED+FAILED must equal
rem    the number of build sections (12). If it does not, a section was
rem    silently skipped and the Summary above cannot be trusted.
rem  * The script must exit NON-ZERO when anything failed. Without this, cmd
rem    returns 0 after a failing command and the harness always looks green
rem    to whatever ran it (CI, another script, a human checking the code).
rem ------------------------------------------------------------
set "HARNESS_RC=%FAILED%"
if %TOTAL% NEQ 12 (
    echo ================================================
    echo   MISMATCH: expected 12 counted suites, but counted %TOTAL%
    echo   A section was skipped -- the Summary above is NOT trustworthy.
    echo ================================================
    set "HARNESS_RC=1"
)
echo.
echo   Suites counted: %TOTAL%    Exit code: %HARNESS_RC%
endlocal & exit /b %HARNESS_RC%
