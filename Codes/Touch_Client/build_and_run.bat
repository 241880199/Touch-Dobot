@echo off
chcp 65001 >nul
setlocal

echo ============================================
echo   Touch-Dobot Remote Control - Build Script
echo ============================================
echo.

set "MSBUILD=D:\Program Files\Microsoft Visual Studio\2022\BuildTools\MSBuild\Current\Bin\MSBuild.exe"
set "PROJECT=D:\Projects\Touch\Codes\Touch_Client\Touch_Client.vcxproj"
set "OH_SDK=D:\Projects\Touch\OpenHaptics\Developer\3.5.0"
set "OUTDIR=D:\Projects\Touch\Codes\Touch_Client\x64\Release"

:: Step 1: Compile
echo [1/3] Compiling...
"%MSBUILD%" "%PROJECT%" /p:Configuration=Release /p:Platform=x64 /v:minimal
if %ERRORLEVEL% NEQ 0 (
    echo.
    echo [ERROR] Compilation failed!
    pause
    exit /b 1
)
echo       Done.

:: Step 2: Copy DLLs
echo [2/3] Copying runtime DLLs...
copy /Y "%OH_SDK%\lib\x64\Release\hd.dll"  "%OUTDIR%\" >nul
copy /Y "%OH_SDK%\lib\x64\Release\hl.dll"  "%OUTDIR%\" >nul 2>nul
copy /Y "%OH_SDK%\utilities\lib\x64\Release\hdu.dll" "%OUTDIR%\" >nul
echo       Done.

:: Step 3: Run
echo [3/3] Starting...
echo.
echo ============================================
"%OUTDIR%\Touch_Client.exe"
echo.
echo ============================================
echo Program exited.
pause
