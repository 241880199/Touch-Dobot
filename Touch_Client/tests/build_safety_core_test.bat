@echo on
rem NOTE: keep this file ASCII-only. Non-ASCII comments in a .bat get mis-decoded by
rem cmd.exe under a non-UTF-8 codepage and can silently swallow the following line.
rem Reuse the toolchain if a parent script already ran vcvarsall. Every vcvarsall
rem call appends to PATH again, and after ~4 nested calls (measured 2026-09-22:
rem run_tests.bat -> build_*.bat -> vcvarsall) PATH overflows cmd's 8191-char
rem command-line limit and vcvarsall dies with "The input line is too long.",
rem which aborts the rest of run_tests.bat. This suite is the 5th build step in
rem run_tests.bat, so without this guard it is the one that tips over.
if not defined VCINSTALLDIR call "D:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvarsall.bat" x64
cd /d "D:\Projects\Touch\Touch_Client\tests"
rem NOTE 2026-09-22: this script did NOT exist, and RobotDiagnostics.cpp calls
rem   RelayCore::instance().reportDiagnostic(...). The link therefore failed with
rem   LNK2019 (RelayCore::instance / RelayCore::reportDiagnostic), so the .exe on
rem   disk was from 2026-07-25 -- OLDER than test_safety_core.cpp itself -- while
rem   RobotStateMachine.cpp (07-26) and RobotError.h (09-21) had changed since.
rem   run_tests.bat only ran it, never rebuilt it, so its "7 passed, 1 failed" was
rem   zero information about the current code. Same class as build_coord_safety_test.bat
rem   missing a .cpp: running the suite verified nothing.
rem TEST_NO_RELAY_CORE compiles that one transmit call out (see RobotDiagnostics.cpp);
rem   file logging and the in-memory counters are unaffected. Same shape as the
rem   TEST_SINGAVOID guard in SingularityAvoidance.cpp.
cl /EHsc /std:c++17 /I"D:\Projects\Touch\OpenHaptics\Developer\3.5.0\include" /I"D:\Projects\Touch\OpenHaptics\Developer\3.5.0\utilities\include" /DWIN32 /DWIN32_LEAN_AND_MEAN /DNOMINMAX /D_CRT_SECURE_NO_WARNINGS /D_WINSOCK_DEPRECATED_NO_WARNINGS /DTEST_NO_RELAY_CORE test_safety_core.cpp ..\safety\RobotStateMachine.cpp ..\safety\RobotDiagnostics.cpp /Fe:test_safety_core.exe
echo BUILD_EXIT=%ERRORLEVEL%
