@echo off
chcp 65001 >nul
setlocal

set "PROJECT=%~dp0..\Touch_Client"
set "RELAY_DIR=%~dp0..\Relay_Station"
set "MATLAB=matlab"

echo ============================================================
echo   Touch-Dobot v3.0 — 联调启动
echo ============================================================
echo.

:: ===== Step 1: Build =====
echo [1/4] Building Touch_Client...
call "%PROJECT%\build.bat"
if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] Build failed!
    pause
    exit /b 1
)
echo.

:: ===== Step 2: MATLAB relay_gui =====
echo [2/4] Starting MATLAB relay_gui (port 8888)...
start "Touch-Dobot Relay GUI" cmd /c "cd /d "%RELAY_DIR%" && "%MATLAB%" -nosplash -nodesktop -r "relay_gui""

echo   Waiting for MATLAB to initialize (15s)...
timeout /t 15 /nobreak >nul

:: ===== Step 3: Verify connection =====
echo [3/4] Checking relay port 8888...
powershell -Command "if ((Test-NetConnection -ComputerName 127.0.0.1 -Port 8888 -WarningAction SilentlyContinue).TcpTestSucceeded) { Write-Host '  MATLAB relay: READY' } else { Write-Host '  MATLAB relay: NOT READY (continuing anyway)' }"
echo.

:: ===== Step 4: Launch =====
echo [4/4] Launching Touch_Client...
echo ============================================================
echo   Controls:
echo     Touch button 1 (hold): control robot
echo     q / ESC: quit
echo     e: escape / recovery
echo.
echo   Data flow:
echo     Touch --USB--^> C++ --30003--^> CR3 (ServoP)
echo     CR3 --30004--^> C++ (force sensor, 125Hz)
echo     C++ --8888--^> MATLAB relay_gui (P^|/RP^|/J^|/F^|)
echo ============================================================
echo.

"%PROJECT%\x64\Release\Touch_Client.exe"

echo.
echo ============================================================
echo   Touch_Client exited. Close MATLAB window manually.
echo ============================================================
pause
endlocal
