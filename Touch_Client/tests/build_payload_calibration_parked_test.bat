@echo on
rem 2026-09-22: guard the vcvarsall call -- PATH grows ~1350 chars per call and cmd dies at ~8191,
rem   so a second unguarded call in the same session can abort the rest of run_tests.bat.
rem   With this guard, vcvarsall runs at most once per cmd session. (ASCII only -- see
rem   build_force_pipeline_test.bat's note about non-ASCII comments in .bat files.)
if not defined VCINSTALLDIR call "D:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvarsall.bat" x64
cd /d "D:\Projects\Touch\Touch_Client\tests"
rem Builds the PARKED variant of test_payload_calibration.cpp: the one case that is red by
rem   design gets its own exe so the main suite can go green and be wired into run_tests.bat.
rem   This suite is named in NOTRUN_LIST and is therefore NOT built or run by the bench --
rem   build it by hand when you want to look at that case. Same link line as the main one.
cl /EHsc /std:c++17 /I"D:\Projects\Touch\OpenHaptics\Developer\3.5.0\include" /I"D:\Projects\Touch\OpenHaptics\Developer\3.5.0\utilities\include" /DWIN32 /DWIN32_LEAN_AND_MEAN /DNOMINMAX /D_CRT_SECURE_NO_WARNINGS /D_WINSOCK_DEPRECATED_NO_WARNINGS test_payload_calibration_parked.cpp ..\force\PayloadCalibration.cpp ..\force\ForceCompensation.cpp ..\calibration\TcpCalibration.cpp /Fe:test_payload_calibration_parked.exe
echo BUILD_EXIT=%ERRORLEVEL%
