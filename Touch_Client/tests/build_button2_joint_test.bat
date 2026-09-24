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
rem   no longer header-free, so the old "think twice" rule is restated here (2026-09-23 fix2,
rem   review finding 3). It used to read: think twice before making it depend on anything that is
rem   NOT itself a pure translation unit. That is the WRONG criterion. The right question is
rem   TRANSITIVE: what does the HEADER drag in, and is its link closure already satisfied?
rem   Reason "is this unit pure?" fails: a pure unit's HEADERS can carry INLINE functions that
rem   reference externs, and CALLING such an inline is what creates the link-time dependency.
rem   Worked example, this very chain: relay/CoordinateTransform.h declares "extern bool enabled;"
rem   (line 24) and "extern double R[9];" (line 25) inside namespace Calibration, and the INLINE
rem   convertTouchToRobot (line 57) reads both. So a file that includes that header and calls that
rem   inline needs those symbols at link time, even though every translation unit involved is pure.
rem   Not hypothetical: it is exactly why build_coord_safety_test.bat must also compile the .cpp
rem   that DEFINES them (calibration/CalibrationIO.cpp, "namespace Calibration" at line 7) or take
rem   LNK2019 x3 -- see test_coord_safety.cpp lines 6-10 for the recorded, reproduced failure.
rem   Checked here, so the next reader does not have to: Button2Joint.cpp, Kinematics.cpp and
rem   test_button2_joint.cpp all reach CoordinateTransform.h (via robot/Kinematics.h), yet NONE of
rem   the three calls convertTouchToRobot (grep: no hit in any of them) -- the inline is never
rem   instantiated, those Calibration:: symbols are never referenced, so CalibrationIO.cpp is NOT
rem   needed here. Kinematics.cpp itself needs nothing but <cmath> and <cstring> either (no
rem   Calibration:: reference at all -- checked).
rem   CORRECTION (2026-09-23): the line here used to say "CoordinateTransform.cpp is NOT linked" --
rem   no such file exists in this repo (verified). The DEFINITIONS of those externs live in
rem   calibration/CalibrationIO.cpp. That .cpp is the one that is not linked here.
rem   Cf. build_button2_mapping_test.bat, which DOES link a calibration .cpp (TcpCalibration.cpp)
rem   for a symbol its test actually calls -- the same transitive rule, applied that way.
rem NOTE: keep this file ASCII-only. Non-ASCII comments in a .bat get mis-decoded by cmd.exe
rem under a non-UTF-8 codepage and can silently swallow the following line.
rem
rem 2026-09-24: button2JointTarget no longer differences Euler angles; it builds the real
rem   delta-R and takes its rotation vector, so Button2Joint.cpp now CALLS
rem   TcpCalibration::rpyToMatrix -- the single source of truth for the Rz*Ry*Rx convention.
rem   Hence ../calibration/TcpCalibration.cpp is now REQUIRED here (LNK2019 without it).
rem   TcpCalibration.cpp needs only cmath/cstdio/cstring/cstdlib, so nothing else comes along.
cl /EHsc /std:c++17 /I"D:\Projects\Touch\OpenHaptics\Developer\3.5.0\include" /I"D:\Projects\Touch\OpenHaptics\Developer\3.5.0\utilities\include" /DWIN32 /DWIN32_LEAN_AND_MEAN /DNOMINMAX /D_CRT_SECURE_NO_WARNINGS test_button2_joint.cpp ../relay/Button2Joint.cpp ../robot/Kinematics.cpp ../calibration/TcpCalibration.cpp /Fe:test_button2_joint.exe
echo BUILD_EXIT=%ERRORLEVEL%
