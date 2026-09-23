@echo on
rem 2026-09-23: guard the vcvarsall call -- PATH grows ~1350 chars per call and cmd dies at ~8191,
rem   so a second unguarded call in the same session can abort the rest of run_tests.bat.
rem   With this guard, vcvarsall runs at most once per cmd session.
if not defined VCINSTALLDIR call "D:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvarsall.bat" x64
cd /d "D:\Projects\Touch\Touch_Client\tests"
rem Button2Joint is a PURE function (relay/Button2Joint.{h,cpp}) -- no socket, no appState, no
rem   OpenHaptics. Its only header dependency is config/Config.h, which includes nothing, so
rem   unlike build_button2_mapping_test.bat this suite needs NO /I paths at all. If a future edit
rem   ever needs one, that means the mapping stopped being pure -- think twice before adding it.
rem NOTE: keep this file ASCII-only. Non-ASCII comments in a .bat get mis-decoded by cmd.exe
rem under a non-UTF-8 codepage and can silently swallow the following line.
cl /EHsc /std:c++17 /DWIN32 /DWIN32_LEAN_AND_MEAN /DNOMINMAX /D_CRT_SECURE_NO_WARNINGS test_button2_joint.cpp ../relay/Button2Joint.cpp /Fe:test_button2_joint.exe
echo BUILD_EXIT=%ERRORLEVEL%
