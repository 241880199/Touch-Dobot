@echo off
call "D:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvarsall.bat" x64 >nul 2>&1
cd /d "D:\Projects\Touch\Touch_Client\tests"
cl /EHsc /std:c++17 /I"D:\Projects\Touch\OpenHaptics\Developer\3.5.0\include" /I"D:\Projects\Touch\OpenHaptics\Developer\3.5.0\utilities\include" /DWIN32 /DNOMINMAX /D_CRT_SECURE_NO_WARNINGS /D_WINSOCK_DEPRECATED_NO_WARNINGS /Fe:debug_sc.exe debug\debug_sc.cpp ..\robot\Kinematics.cpp ..\safety\SelfCollision.cpp >nul 2>&1
debug_sc.exe
