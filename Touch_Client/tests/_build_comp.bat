@echo off
call "D:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvarsall.bat" x64 >nul 2>&1
if %ERRORLEVEL% neq 0 (
    echo vcvarsall.bat FAILED
    exit /b 1
)
cd /d "D:\Projects\Touch\Touch_Client\tests"
cl /EHsc /std:c++17 ^
  /I"D:\Projects\Touch\OpenHaptics\Developer\3.5.0\include" ^
  /I"D:\Projects\Touch\OpenHaptics\Developer\3.5.0\utilities\include" ^
  /DWIN32 /DWIN32_LEAN_AND_MEAN /DNOMINMAX ^
  /D_CRT_SECURE_NO_WARNINGS /D_WINSOCK_DEPRECATED_NO_WARNINGS ^
  test_force_compensation.cpp ^
  ..\force\ForceCompensation.cpp ^
  ..\force\ForceCalibration.cpp ^
  /Fe:test_force_compensation.exe
echo BUILD_EXIT=%ERRORLEVEL%
