@echo on
call "D:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvarsall.bat" x64
cd /d "D:\Projects\Touch\Touch_Client\tests"
rem NoiseProbe.h is a pure header (no .cpp to link) -- that is deliberate: the statistics that
rem decide "does averaging buy anything" must be testable without a socket, a robot, or OpenHaptics.
rem NOTE: keep this file ASCII-only. Non-ASCII comments in a .bat get mis-decoded by cmd.exe
rem under a non-UTF-8 codepage and can silently swallow the following line.
cl /EHsc /std:c++17 /I"D:\Projects\Touch\OpenHaptics\Developer\3.5.0\include" /I"D:\Projects\Touch\OpenHaptics\Developer\3.5.0\utilities\include" /DWIN32 /DWIN32_LEAN_AND_MEAN /DNOMINMAX /D_CRT_SECURE_NO_WARNINGS /D_WINSOCK_DEPRECATED_NO_WARNINGS test_noise_probe.cpp /Fe:test_noise_probe.exe
echo BUILD_EXIT=%ERRORLEVEL%
