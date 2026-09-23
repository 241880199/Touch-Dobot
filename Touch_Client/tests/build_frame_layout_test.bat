@echo on
rem 2026-09-22: guard the vcvarsall call -- PATH grows ~1350 chars per call and cmd dies at ~8191,
rem   so a second unguarded call in the same session can abort the rest of run_tests.bat.
rem   With this guard, vcvarsall runs at most once per cmd session. (ASCII only -- see
rem   build_force_pipeline_test.bat's note about non-ASCII comments in .bat files.)
if not defined VCINSTALLDIR call "D:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvarsall.bat" x64
cd /d "D:\Projects\Touch\Touch_Client\tests"
rem FrameLayout.h is a pure header (no .cpp to link) -- that is exactly why the predicate was
rem extracted as a pure function: this test needs no socket and no OpenHaptics. The two
rem OpenHaptics /I paths are kept so that a future edit pulling in AppState.h needs no change here.
rem NOTE: keep this file ASCII-only. Non-ASCII comments in a .bat get mis-decoded by cmd.exe
rem under a non-UTF-8 codepage and can silently swallow the following line.
rem NOTE 2026-09-23: ../calibration/CalibrationIO.cpp is REQUIRED as of this date. Two cases at the
rem   end of test_frame_layout.cpp now call touchToRobotMatrix / convertTouchToRobot, and
rem   CoordinateTransform.h declares Calibration::enabled / R / t as extern -- their DEFINITIONS
rem   live in calibration/CalibrationIO.cpp. Without it the link dies with exactly 3 unresolved
rem   externals (LNK2019 x3 + LNK1120; measured 2026-09-23 before this line was added). Same root
rem   cause as the note in build_coord_safety_test.bat -- see also test_coord_safety.cpp lines 6-10.
cl /EHsc /std:c++17 /I"D:\Projects\Touch\OpenHaptics\Developer\3.5.0\include" /I"D:\Projects\Touch\OpenHaptics\Developer\3.5.0\utilities\include" /DWIN32 /DWIN32_LEAN_AND_MEAN /DNOMINMAX /D_CRT_SECURE_NO_WARNINGS /D_WINSOCK_DEPRECATED_NO_WARNINGS test_frame_layout.cpp ../calibration/CalibrationIO.cpp /Fe:test_frame_layout.exe
echo BUILD_EXIT=%ERRORLEVEL%
