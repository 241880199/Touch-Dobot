@echo on
rem 2026-10-01: guard the vcvarsall call -- PATH grows ~1350 chars per call and cmd dies at ~8191,
rem   so a second unguarded call in the same session can abort the rest of run_tests.bat.
rem   With this guard, vcvarsall runs at most once per cmd session. (ASCII only -- see
rem   build_force_pipeline_test.bat's note about non-ASCII comments in .bat files.)
if not defined VCINSTALLDIR call "D:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvarsall.bat" x64
cd /d "D:\Projects\Touch\Touch_Client\tests"
rem SweepPlan.h is a PURE header (inline implementation, no .cpp to link), and it includes
rem   only <cmath>. Nothing else is linked on purpose; if this suite ever needs a second
rem   translation unit, that means the segment scheduler stopped being a pure function.
cl /EHsc /std:c++17 /DWIN32 /DWIN32_LEAN_AND_MEAN /DNOMINMAX /D_CRT_SECURE_NO_WARNINGS test_sweep_plan.cpp /Fe:test_sweep_plan.exe
echo BUILD_EXIT=%ERRORLEVEL%
