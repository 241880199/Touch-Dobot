@echo on
call "D:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvarsall.bat" x64
cd /d "D:\Projects\Touch\Touch_Client\tests"
cl /EHsc /std:c++17 /D_CRT_SECURE_NO_WARNINGS /D_WINSOCK_DEPRECATED_NO_WARNINGS test_relay_command_parser.cpp ..\relay\RelayCommandParser.cpp /Fe:test_relay_command_parser.exe
echo BUILD_EXIT=%ERRORLEVEL%
