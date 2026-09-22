@echo off
setlocal
set "TESTDIR=D:\Projects\Touch\Touch_Client\tests"
set "PASSED=0"
set "FAILED=0"

echo ================================================
echo   Touch-Dobot Unit Tests
echo ================================================
echo.

for %%e in (
    test_force_pipeline.exe
    test_constraint_force.exe
    test_feedback_parser.exe
    test_escalation.exe
    test_kinematics.exe
    test_coord_safety.exe
) do (
    if exist "%TESTDIR%\%%e" (
        echo === %%e ===
        "%TESTDIR%\%%e"
        if %ERRORLEVEL% EQU 0 (
            set /a PASSED+=1
            echo   [OK]
        ) else (
            set /a FAILED+=1
            echo   [FAIL]
        )
    ) else (
        echo === %%e === [SKIP: not built]
        set /a FAILED+=1
    )
    echo.
)

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
endlocal
