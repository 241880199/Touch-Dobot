@echo on
rem 2026-09-22: guard the vcvarsall call -- PATH grows ~1350 chars per call and cmd dies at ~8191,
rem   so a second unguarded call in the same session can abort the rest of run_tests.bat.
if not defined VCINSTALLDIR call "D:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvarsall.bat" x64
cd /d "D:\Projects\Touch\Touch_Client\tests"
rem Linked in on purpose: ForceTuning.cpp is the unit under test; CalibStore.cpp satisfies
rem   the linker for the CalibStore::fileFor references inside loadOnStartup/tick.
rem   ForcePipeline.cpp is NOT linked here -- Task 2 has its own test for the wiring.
rem NOTE: keep this file ASCII-only. Non-ASCII comments in a .bat get mis-decoded by
rem cmd.exe under a non-UTF-8 codepage and can silently swallow the following line.
cl /EHsc /std:c++17 /I"D:\Projects\Touch\OpenHaptics\Developer\3.5.0\include" /I"D:\Projects\Touch\OpenHaptics\Developer\3.5.0\utilities\include" /DWIN32 /DWIN32_LEAN_AND_MEAN /DNOMINMAX /D_CRT_SECURE_NO_WARNINGS /D_WINSOCK_DEPRECATED_NO_WARNINGS test_force_tuning.cpp ..\force\ForceTuning.cpp ..\core\CalibStore.cpp /Fe:test_force_tuning.exe
echo BUILD_EXIT=%ERRORLEVEL%
