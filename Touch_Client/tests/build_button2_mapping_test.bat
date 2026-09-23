@echo on
rem 2026-09-22: guard the vcvarsall call -- PATH grows ~1350 chars per call and cmd dies at ~8191,
rem   so a second unguarded call in the same session can abort the rest of run_tests.bat.
rem   With this guard, vcvarsall runs at most once per cmd session. (ASCII only -- see
rem   build_force_pipeline_test.bat's note about non-ASCII comments in .bat files.)
if not defined VCINSTALLDIR call "D:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvarsall.bat" x64
cd /d "D:\Projects\Touch\Touch_Client\tests"
rem The two OpenHaptics /I paths are needed because relay/CoordinateTransform.h includes HDU.
rem ../calibration/TcpCalibration.cpp is REQUIRED: test_button2_mapping.cpp calls
rem   TcpCalibration::rpyToMatrix, whose DEFINITION lives there. Same root cause as the note in
rem   build_coord_safety_test.bat (missing .cpp -> LNK2019) and the note in
rem   build_frame_layout_test.bat about calibration/CalibrationIO.cpp.
rem NOTE: keep this file ASCII-only. Non-ASCII comments in a .bat get mis-decoded by cmd.exe
rem under a non-UTF-8 codepage and can silently swallow the following line.
cl /EHsc /std:c++17 /I"D:\Projects\Touch\OpenHaptics\Developer\3.5.0\include" /I"D:\Projects\Touch\OpenHaptics\Developer\3.5.0\utilities\include" /DWIN32 /DWIN32_LEAN_AND_MEAN /DNOMINMAX /D_CRT_SECURE_NO_WARNINGS /D_WINSOCK_DEPRECATED_NO_WARNINGS test_button2_mapping.cpp ../relay/Button2Mapping.cpp ../calibration/TcpCalibration.cpp /Fe:test_button2_mapping.exe
echo BUILD_EXIT=%ERRORLEVEL%
