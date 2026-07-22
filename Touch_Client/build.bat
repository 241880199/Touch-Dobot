@echo off
chcp 65001 >nul
setlocal

set "MSBUILD=D:\Program Files\Microsoft Visual Studio\2022\BuildTools\MSBuild\Current\Bin\MSBuild.exe"
set "PROJECT=%~dp0Touch_Client.vcxproj"
set "OUTDIR=%~dp0x64\Release"
set "OH_SDK=D:\Projects\Touch\OpenHaptics\Developer\3.5.0"

echo ================================================
echo   Touch_Client v3.0 Build
echo ================================================
echo.

echo [1/2] Building...
"%MSBUILD%" "%PROJECT%" /p:Configuration=Release /p:Platform=x64 /v:minimal
if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] Build failed!
    pause
    exit /b 1
)
echo   Build OK.

echo.
echo [2/2] Copying DLLs...
copy /Y "%OH_SDK%\lib\x64\Release\hd.dll" "%OUTDIR%\" >nul
copy /Y "%OH_SDK%\utilities\lib\x64\Release\hdu.dll" "%OUTDIR%\" >nul
copy /Y "%OH_SDK%\utilities\lib\x64\Release\glut32.dll" "%OUTDIR%\" >nul
echo   DLLs copied.

echo.
echo [3/3] Copying models...
if exist "%~dp0models" (
    xcopy /E /I /Y "%~dp0models" "%OUTDIR%\models" >nul
    echo   Models copied.
) else (
    echo   models/ not found, skipped
)

echo.
echo Build complete. Run: %OUTDIR%\Touch_Client.exe
endlocal
