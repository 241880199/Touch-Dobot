@echo off
setlocal

echo ============================================================
echo   Touch_Client Build
echo ============================================================
echo.

set "MSBUILD=D:\Program Files\Microsoft Visual Studio\2022\BuildTools\MSBuild\Current\Bin\MSBuild.exe"
set "PROJECT=%~dp0..\Touch_Client\Touch_Client.vcxproj"
set "OUTDIR=%~dp0..\Touch_Client\x64\Release"
set "OH_SDK=%~dp0..\OpenHaptics\Developer\3.5.0"

if not exist "%MSBUILD%" (
    echo [ERROR] MSBuild not found at:
    echo   %MSBUILD%
    echo.
    echo   Install VS2022 Build Tools or update MSBUILD path in this script.
    pause
    exit /b 1
)

echo [1/2] Building Touch_Client...
"%MSBUILD%" "%PROJECT%" /p:Configuration=Release /p:Platform=x64 /v:minimal
if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] Build failed!
    pause
    exit /b 1
)
echo   Build OK.

echo.
echo [2/2] Copying DLLs + models...
copy /Y "%OH_SDK%\lib\x64\Release\hd.dll"    "%OUTDIR%\" >nul
copy /Y "%OH_SDK%\utilities\lib\x64\Release\hdu.dll"   "%OUTDIR%\" >nul
copy /Y "%OH_SDK%\utilities\lib\x64\Release\glut32.dll" "%OUTDIR%\" >nul
if exist "%~dp0..\Touch_Client\models" (
    xcopy /E /I /Y "%~dp0..\Touch_Client\models" "%OUTDIR%\models" >nul
    echo   DLLs + models copied.
) else (
    echo   DLLs copied (models/ not found, skipping).
)

echo.
echo   Build complete: %OUTDIR%\Touch_Client.exe
endlocal
