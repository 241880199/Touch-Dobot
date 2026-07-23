@echo off
setlocal

echo ============================================================
echo   Touch-Dobot Integration Test - Full System
echo ============================================================
echo.
echo   Purpose: Full end-to-end test with Touch + CR3 robot arm.
echo.
echo   *** WARNING ***
echo   This will send motion commands to the real robot arm!
echo   Ensure:
echo     1. Robot is powered on and emergency stop is accessible
echo     2. Robot workspace is clear of obstacles
echo     3. Safety boundaries are configured (Config.h + relay_config.m)
echo     4. Touch device is calibrated and ready
echo ============================================================
echo.
echo   Press Ctrl+C NOW to abort, or
pause

set "PROJECT=%~dp0..\Touch_Client"
set "RELAY_DIR=%~dp0..\Relay_Station"
set "MATLAB=matlab"

echo.
echo [Step 1/3] Starting MATLAB relay_gui...
echo ============================================================
echo.

start "Touch-Dobot Relay GUI" cmd /c "cd /d "%RELAY_DIR%" && "%MATLAB%" -nosplash -nodesktop -r "relay_gui""

echo   Waiting for MATLAB TCP server (port 8888) to initialize...
timeout /t 10 /nobreak >nul

echo.
echo [Step 2/3] Checking C++ build...
echo ============================================================
echo.

set "EXE=%PROJECT%\x64\Release\Touch_Client.exe"
if not exist "%EXE%" (
    echo   [ERROR] Touch_Client.exe not found.
    echo   Build first: cd Touch_Client ^&^& build.bat
    pause
    exit /b 1
)
echo   Touch_Client.exe found. OK.

echo.
echo [Step 3/3] Starting Touch_Client (FULL mode - robot enabled)...
echo ============================================================
echo.
echo   Connection sequence:
echo     1. C++ connects to CR3:29999 (enable) + 30003 (motion)
echo     2. Init: ClearError - EnableRobot - CP - GetPose (base)
echo     3. C++ connects to MATLAB:8888 for data reporting
echo     4. 100ms pose query, 200ms joint angle query, 300ms alarm check
echo.
echo   Controls:
echo     - Move Touch stylus to define motion delta
echo     - Press and HOLD Touch button 1 to stream ServoP to robot
echo     - Release button 1 to stop motion
echo     - Press 'q' or ESC to quit (calls DisableRobot)
echo.
echo   Monitor:
echo     - C++ GLUT window: 3D robot model + HUD
echo     - MATLAB relay_gui: P|/RP|/J| data + FK stick model
echo.
echo   Safety checks:
echo     - Safety boundary hard clamp
echo     - Alarm polling every 300ms (RobotMode==9 triggers alarm)
echo     - Button release = immediate motion stop
echo ============================================================
echo.

"%EXE%"

echo.
echo ============================================================
echo   System stopped.
echo ============================================================
echo.
echo   Check MATLAB relay_gui window - close it manually if needed.
echo.
pause
