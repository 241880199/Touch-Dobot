@echo off
setlocal

echo ============================================================
echo   Touch-Dobot Integration Test - No-Robot Mode
echo ============================================================
echo.
echo   Purpose: Verify Touch device - C++ - MATLAB data pipeline
echo            without connecting to a real robot arm.
echo.
echo   Prerequisites:
echo     1. Touch haptic device connected via USB
echo     2. OpenHaptics SDK 3.5.0 installed
echo     3. MATLAB with Instrument Control Toolbox
echo ============================================================
echo.

set "PROJECT=%~dp0..\Touch_Client"
set "RELAY_DIR=%~dp0..\Relay_Station"
set "MATLAB=matlab"

echo [Step 1/3] Starting MATLAB relay_gui...
echo ============================================================
echo.
echo   Starting MATLAB in a new window (this may take ~30s)...
echo   Close the MATLAB window to stop the relay.
echo.

start "Touch-Dobot Relay GUI" cmd /c "cd /d "%RELAY_DIR%" && "%MATLAB%" -nosplash -nodesktop -r "relay_gui""

echo   Waiting for MATLAB TCP server (port 8888) to be ready...
echo   (This script waits 10s - adjust if MATLAB starts slower)
timeout /t 10 /nobreak >nul

echo.
echo [Step 2/3] Checking C++ build...
echo ============================================================
echo.

set "EXE=%PROJECT%\x64\Release\Touch_Client.exe"
if not exist "%EXE%" (
    echo   [ERROR] Touch_Client.exe not found at:
    echo     %EXE%
    echo.
    echo   Please build first: cd Touch_Client ^&^& build.bat
    echo   Or run: scripts\build_client.bat
    pause
    exit /b 1
)
echo   Touch_Client.exe found. OK.

echo.
echo [Step 3/3] Starting Touch_Client (--no-robot mode)...
echo ============================================================
echo.
echo   Mode: --no-robot (no CR3 connection, Touch + MATLAB only)
echo   Controls:
echo     - Move Touch stylus to see position in both GUIs
echo     - Press Touch button 1 (no robot, so no motion commands)
echo     - Press 'q' or ESC to quit
echo.
echo   What to verify:
echo     [ ] MATLAB status bar shows "C++ Client: CONNECTED" (green)
echo     [ ] Moving Touch updates position in C++ HUD
echo     [ ] Moving Touch shows pen in C++ 3D viewport
echo     [ ] P| messages appear in MATLAB command log
echo     [ ] Joint angles show zeros (no robot connected)
echo ============================================================
echo.

"%EXE%" --no-robot

echo.
echo ============================================================
echo   Test complete.
echo ============================================================
echo.
echo   If all checkboxes above passed: data pipeline is working.
echo   Next step: run scripts\test_full.bat with real robot.
echo.
pause
