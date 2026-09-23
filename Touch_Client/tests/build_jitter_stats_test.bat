@echo on
rem 2026-09-22: guard the vcvarsall call -- PATH grows ~1350 chars per call and cmd dies at ~8191,
rem   so a second unguarded call in the same session can abort the rest of run_tests.bat.
if not defined VCINSTALLDIR call "D:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvarsall.bat" x64
cd /d "D:\Projects\Touch\Touch_Client\tests"
rem JitterStats.h is a PURE header (inline implementation, no .cpp to link) -- that is the whole
rem   point of Task 1 of the jitter plan: the statistics must be unit-testable without the
rem   pipeline, the socket or OpenHaptics. Nothing else is linked here on purpose; if this suite
rem   ever needs a second translation unit, that means the accumulator stopped being pure.
rem NOTE: keep this file ASCII-only. Non-ASCII comments in a .bat get mis-decoded by cmd.exe
rem under a non-UTF-8 codepage and can silently swallow the following line.
cl /EHsc /std:c++17 /DWIN32 /DWIN32_LEAN_AND_MEAN /DNOMINMAX /D_CRT_SECURE_NO_WARNINGS test_jitter_stats.cpp /Fe:test_jitter_stats.exe
echo BUILD_EXIT=%ERRORLEVEL%
