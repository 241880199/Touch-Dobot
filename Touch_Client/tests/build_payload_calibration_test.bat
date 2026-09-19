@echo on
call "D:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvarsall.bat" x64
cd /d "D:\Projects\Touch\Touch_Client\tests"
rem ForceCompensation.cpp is linked in on purpose (Task 6 acceptance case runs the production
rem compensation code itself -- setCalibration + step -- instead of re-writing the formula here).
cl /EHsc /std:c++17 /I"D:\Projects\Touch\OpenHaptics\Developer\3.5.0\include" /I"D:\Projects\Touch\OpenHaptics\Developer\3.5.0\utilities\include" /DWIN32 /DWIN32_LEAN_AND_MEAN /DNOMINMAX /D_CRT_SECURE_NO_WARNINGS /D_WINSOCK_DEPRECATED_NO_WARNINGS test_payload_calibration.cpp ..\force\PayloadCalibration.cpp ..\force\ForceCompensation.cpp ..\calibration\TcpCalibration.cpp /Fe:test_payload_calibration.exe
echo BUILD_EXIT=%ERRORLEVEL%
