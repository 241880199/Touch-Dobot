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
rem ============================================================

echo ================================================
echo   Force Compensation Tests
echo ================================================
echo.
echo --- Building test_force_compensation ---
call "%TESTDIR%\build_force_comp_test.bat"
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
rem   not exist on a fresh clone. Nothing in the tree can build
rem   this suite's exe, so running the exe here would present a stale
rem   binary as evidence -- the exact failure mode this harness was
rem   fixed to remove. Pending a decision on that .gitignore line.
rem   (Previously this suite WAS run here, from a prebuilt binary that
rem    nothing rebuilt. Removing it makes the gap visible instead of
rem    silently producing false evidence.)
rem ============================================================

echo --- Building test_safety_core ---
call "%TESTDIR%\build_safety_core_test.bat"
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
echo   Summary: %PASSED% suite(s) OK, %FAILED% suite(s) FAILED
echo ================================================

rem ------------------------------------------------------------
rem Report the result to the CALLER, not only to stdout.
rem  * Every suite must be counted exactly once, so PASSED+FAILED must equal
rem    the number of build sections (12). If it does not, a section was
rem    silently skipped and the Summary above cannot be trusted.
rem  * The script must exit NON-ZERO when anything failed. Without this, cmd
rem    returns 0 after a failing command and the harness always looks green
rem    to whatever ran it (CI, another script, a human checking the code).
rem ------------------------------------------------------------
set /a TOTAL=PASSED+FAILED
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
