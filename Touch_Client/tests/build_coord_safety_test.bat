@echo on
rem 2026-09-22: guard the vcvarsall call -- PATH grows ~1350 chars per call and cmd dies at ~8191,
rem   so a second unguarded call in the same session can abort the rest of run_tests.bat.
rem   With this guard, vcvarsall runs at most once per cmd session. (ASCII only -- see
rem   build_force_pipeline_test.bat's note about non-ASCII comments in .bat files.)
if not defined VCINSTALLDIR call "D:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvarsall.bat" x64
cd /d "D:\Projects\Touch\Touch_Client\tests"
rem NOTE 2026-09-21: CalibrationIO.cpp was MISSING from this link. CoordinateTransform.h declares
rem   Calibration::enabled / R / t as extern, and SafetyBoundary.h includes it, so the link failed
rem   with 3 unresolved externals -- meaning this suite had NEVER been buildable as scripted (its
rem   .exe was stale-or-absent, i.e. running it verified nothing). The definitions live in
rem   calibration/CalibrationIO.cpp, which needs nothing beyond CoordinateTransform.h + libc.
rem NOTE: keep this file ASCII-only. Non-ASCII comments in a .bat get mis-decoded by cmd.exe
rem under a non-UTF-8 codepage and can silently swallow the following line.
cl /EHsc /std:c++17 /I"D:\Projects\Touch\OpenHaptics\Developer\3.5.0\include" /I"D:\Projects\Touch\OpenHaptics\Developer\3.5.0\utilities\include" /DWIN32 /DWIN32_LEAN_AND_MEAN /DNOMINMAX /D_CRT_SECURE_NO_WARNINGS /D_WINSOCK_DEPRECATED_NO_WARNINGS test_coord_safety.cpp ../calibration/CalibrationIO.cpp /Fe:test_coord_safety.exe
echo BUILD_EXIT=%ERRORLEVEL%
