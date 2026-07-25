@echo off
call "D:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvarsall.bat" x64 >nul 2>&1
echo === test_force_pipeline ===
"D:\Projects\Touch\Touch_Client\tests\test_force_pipeline.exe"
echo.
echo === test_constraint_force ===
"D:\Projects\Touch\Touch_Client\tests\test_constraint_force.exe"
echo.
echo DONE
