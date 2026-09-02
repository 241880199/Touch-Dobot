@echo on
call "D:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvarsall.bat" x64
cd /d "D:\Projects\Touch\Touch_Client\tests"
cl /EHsc /std:c++17 /D_CRT_SECURE_NO_WARNINGS /D_WINSOCK_DEPRECATED_NO_WARNINGS test_tcp_calibration.cpp ..\calibration\TcpCalibration.cpp /Fe:test_tcp_calibration.exe
echo BUILD_EXIT=%ERRORLEVEL%
