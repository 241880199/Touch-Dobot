@echo off
call "D:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvarsall.bat" x64 >nul 2>&1
cd /d "D:\Projects\Touch\Touch_Client\tests"
cl /EHsc /std:c++17 /I"D:\Projects\Touch\OpenHaptics\Developer\3.5.0\include" /I"D:\Projects\Touch\OpenHaptics\Developer\3.5.0\utilities\include" /DTEST_SINGAVOID /DWIN32 /DWIN32_LEAN_AND_MEAN /DNOMINMAX /D_CRT_SECURE_NO_WARNINGS /D_WINSOCK_DEPRECATED_NO_WARNINGS test_singularity_avoidance.cpp ..\safety\SingularityAvoidance.cpp ..\robot\Kinematics.cpp /Fe:test_singularity_avoidance.exe
if %ERRORLEVEL% EQU 0 test_singularity_avoidance.exe
