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
    test_safety_core.exe
    test_feedback_parser.exe
    test_escalation.exe
    test_kinematics.exe
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
echo   Tests complete
echo ================================================
endlocal
