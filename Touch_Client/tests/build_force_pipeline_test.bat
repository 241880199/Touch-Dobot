@echo on
call "D:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvarsall.bat" x64
cd /d "D:\Projects\Touch\Touch_Client\tests"
rem 2026-09-21: this script did NOT exist. test_force_pipeline.cpp's header said
rem   "Build: see task-10-brief for exact command" -- and that brief lives under
rem   .superpowers/sdd/ which is gitignored, so the build command was never on disk.
rem   That is exactly how a test rots: an .exe from July was sitting here with the OLD
rem   FORCE_REFLECTION_GAIN baked in, so running it would have verified nothing.
rem   (Same class as the project's "what is not on disk does not exist".)
rem Linked in on purpose: ForcePipeline.cpp is the unit under test.
rem NOTE: keep this file ASCII-only. Non-ASCII comments in a .bat get mis-decoded by
rem cmd.exe under a non-UTF-8 codepage and can silently swallow the following line.
cl /EHsc /std:c++17 /I"D:\Projects\Touch\OpenHaptics\Developer\3.5.0\include" /I"D:\Projects\Touch\OpenHaptics\Developer\3.5.0\utilities\include" /DWIN32 /DWIN32_LEAN_AND_MEAN /DNOMINMAX /D_CRT_SECURE_NO_WARNINGS /D_WINSOCK_DEPRECATED_NO_WARNINGS test_force_pipeline.cpp ..\force\ForcePipeline.cpp /Fe:test_force_pipeline.exe
echo BUILD_EXIT=%ERRORLEVEL%
