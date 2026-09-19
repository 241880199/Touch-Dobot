@echo on
call "D:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvarsall.bat" x64
cd /d "D:\Projects\Touch\Touch_Client\tests"
cl /EHsc /std:c++17 /utf-8 /D_CRT_SECURE_NO_WARNINGS test_session_report.cpp /Fe:test_session_report.exe
echo BUILD_EXIT=%ERRORLEVEL%
