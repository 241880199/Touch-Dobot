@echo on
rem 2026-09-23: guard the vcvarsall call -- PATH grows ~1350 chars per call and cmd dies at ~8191,
rem   so a second unguarded call in the same session can abort the rest of run_tests.bat.
rem   With this guard, vcvarsall runs at most once per cmd session.
if not defined VCINSTALLDIR call "D:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvarsall.bat" x64
cd /d "D:\Projects\Touch\Touch_Client\tests"
rem 2026-09-23 fix2: this suite NOW needs the OpenHaptics /I paths and one extra .cpp. The old
rem   comment here said the opposite ("needs NO /I paths at all -- if a future edit ever needs
rem   one, that means the mapping stopped being pure"). The mapping did NOT stop being pure.
rem   What changed: the hookup-layer predicates (isTrustworthyJointRef / clampJointStep) were
rem   extracted out of RelayCore.cpp into relay/Button2Joint.{h,cpp} -- RelayCore.cpp is compiled
rem   by no test suite, so those two had no test coverage at all, and one of them is plain
rem   arithmetic. isTrustworthyJointRef calls Kinematics::isWithinJointLimits, and that header
rem   chain reaches <HDU/hduVector.h>:
rem       relay/Button2Joint.cpp -> robot/Kinematics.h -> relay/CoordinateTransform.h -> HDU
rem   hence the two /I paths, and hence Kinematics.cpp must be linked for that symbol.
rem   Still pure in the sense that mattered (no socket, no appState, no OpenHaptics CALLS) -- but
rem   no longer header-free, so the old "think twice" rule is now: think twice before making it
rem   depend on anything that is NOT itself a pure translation unit.
rem   Checked, so the next reader does not have to: Kinematics.cpp needs nothing but <cmath> and
rem   <cstring> (no Calibration:: symbols), which is why CoordinateTransform.cpp is NOT linked
rem   here the way build_button2_mapping_test.bat links TcpCalibration.cpp.
rem NOTE: keep this file ASCII-only. Non-ASCII comments in a .bat get mis-decoded by cmd.exe
rem under a non-UTF-8 codepage and can silently swallow the following line.
cl /EHsc /std:c++17 /I"D:\Projects\Touch\OpenHaptics\Developer\3.5.0\include" /I"D:\Projects\Touch\OpenHaptics\Developer\3.5.0\utilities\include" /DWIN32 /DWIN32_LEAN_AND_MEAN /DNOMINMAX /D_CRT_SECURE_NO_WARNINGS test_button2_joint.cpp ../relay/Button2Joint.cpp ../robot/Kinematics.cpp /Fe:test_button2_joint.exe
echo BUILD_EXIT=%ERRORLEVEL%
