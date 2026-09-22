@echo off
setlocal
set "TESTDIR=D:\Projects\Touch\Touch_Client\tests"
set "PASSED=0"
set "FAILED=0"

echo ================================================
echo   Touch-Dobot Unit Tests
echo ================================================
echo.

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
    if %ERRORLEVEL% EQU 0 (
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
    if %ERRORLEVEL% EQU 0 (
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
    if %ERRORLEVEL% EQU 0 (
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
    if %ERRORLEVEL% EQU 0 (
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
rem
rem * WHY THE RESULT CHECK BELOW IS "if errorlevel 1", NOT "if %ERRORLEVEL% EQU 0"
rem   A %ERRORLEVEL% inside a parenthesised block is expanded when the block is
rem   PARSED -- i.e. BEFORE the test exe has run -- so it can only ever see the
rem   BUILD's exit code and will print [OK] for any test result whatsoever.
rem   Measured 2026-09-22: an exe returning 7 still printed [OK] here.
rem   "if errorlevel 1" is evaluated at RUN time, so it is correct inside a block.
rem   DO NOT "tidy" this back to %ERRORLEVEL% without re-measuring first.
rem ============================================================

echo --- Building test_force_pipeline ---
call "%TESTDIR%\build_force_pipeline_test.bat"
if %ERRORLEVEL% EQU 0 (
    echo   Build OK
    echo.
    echo === test_force_pipeline.exe ===
    "%TESTDIR%\test_force_pipeline.exe"
    if errorlevel 1 (
        set /a FAILED+=1
        echo   [FAIL]
    ) else (
        set /a PASSED+=1
        echo   [OK]
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
    if errorlevel 1 (
        set /a FAILED+=1
        echo   [FAIL]
    ) else (
        set /a PASSED+=1
        echo   [OK]
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
    if errorlevel 1 (
        set /a FAILED+=1
        echo   [FAIL]
    ) else (
        set /a PASSED+=1
        echo   [OK]
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
    if errorlevel 1 (
        set /a FAILED+=1
        echo   [FAIL]
    ) else (
        set /a PASSED+=1
        echo   [OK]
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
    if errorlevel 1 (
        set /a FAILED+=1
        echo   [FAIL]
    ) else (
        set /a PASSED+=1
        echo   [OK]
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
    if %ERRORLEVEL% EQU 0 (
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
    if %ERRORLEVEL% EQU 0 (
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
    if %ERRORLEVEL% EQU 0 (
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
endlocal
