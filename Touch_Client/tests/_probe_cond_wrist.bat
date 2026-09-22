@echo off
rem One-shot probe (2026-09-22): reverse-engineer the wrist singularity damping factor
rem beta from the production dampOrientationMotion, using field-supplied joint angles.
rem ASCII only: a non-ASCII rem comment gets mis-decoded and silently eats the next line.
if not defined VCINSTALLDIR call "D:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvarsall.bat" x64 >nul 2>&1
cd /d "D:\Projects\Touch\Touch_Client\tests"
cl /EHsc /std:c++17 /I"D:\Projects\Touch\OpenHaptics\Developer\3.5.0\include" /I"D:\Projects\Touch\OpenHaptics\Developer\3.5.0\utilities\include" /DTEST_SINGAVOID /DWIN32 /DWIN32_LEAN_AND_MEAN /DNOMINMAX /D_CRT_SECURE_NO_WARNINGS /D_WINSOCK_DEPRECATED_NO_WARNINGS _probe_cond_wrist.cpp ..\safety\SingularityAvoidance.cpp ..\robot\Kinematics.cpp /Fe:_probe_cond_wrist.exe
echo BUILD_EXIT=%ERRORLEVEL%
