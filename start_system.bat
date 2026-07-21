@echo off
chcp 65001 >nul
setlocal

echo ================================================
echo   Touch-Dobot Remote Control - System Launcher
echo ================================================
echo.
echo   Architecture:
echo     Touch_Client --TCP:8888--^> MATLAB Relay --TCP--^> Robot(192.168.101.11)
echo.

set "MSBUILD=D:\Program Files\Microsoft Visual Studio\2022\BuildTools\MSBuild\Current\Bin\MSBuild.exe"
set "PROJECT=D:\Projects\Touch\Codes\Touch_Client\Touch_Client.vcxproj"
set "OH_SDK=D:\Projects\Touch\OpenHaptics\Developer\3.5.0"
set "OUTDIR=D:\Projects\Touch\Codes\Touch_Client\x64\Release"
set "RELAY_DIR=D:\Projects\Touch\Relay_Station"
set "MATLAB=matlab"

echo ================================================
echo   Step 1/3: Start MATLAB Relay Station
echo ================================================
echo.
echo   Starting MATLAB relay in a new window...
echo   (Close the MATLAB window to stop the relay)
echo.

start "Touch-Dobot Relay" cmd /c "cd /d %RELAY_DIR% && %MATLAB% -nosplash -nodesktop -r "relay_main""

echo   Waiting for relay to initialize...
timeout /t 5 /nobreak >nul

echo ================================================
echo   Step 2/3: Build Touch_Client
echo ================================================
echo.

"%MSBUILD%" "%PROJECT%" /p:Configuration=Release /p:Platform=x64 /v:minimal
if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] Build failed!
    pause
    exit /b 1
)

echo ================================================
echo   Step 3/3: Copy DLLs and Run
echo ================================================
echo.

copy /Y "%OH_SDK%\lib\x64\Release\hd.dll"  "%OUTDIR%\" >nul
copy /Y "%OH_SDK%\utilities\lib\x64\Release\hdu.dll" "%OUTDIR%\" >nul
copy /Y "%OH_SDK%\utilities\lib\x64\Release\glut32.dll" "%OUTDIR%\" >nul

echo   Starting Touch_Client...
echo   (Press Touch button 1 to control robot, type 'q' to quit)
echo ================================================
echo.

"%OUTDIR%\Touch_Client.exe"

echo.
echo ================================================
echo   System stopped.
echo ================================================
pause
