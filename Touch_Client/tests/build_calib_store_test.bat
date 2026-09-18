@echo on
call "D:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvarsall.bat" x64
cd /d "D:\Projects\Touch\Touch_Client\tests"
cl /EHsc /std:c++17 /D_CRT_SECURE_NO_WARNINGS test_calib_store.cpp ..\core\CalibStore.cpp /Fe:test_calib_store.exe
echo BUILD_EXIT=%ERRORLEVEL%
