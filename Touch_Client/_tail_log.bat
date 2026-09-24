@echo off
REM Must match LOG in _run_captured.bat, or you will be watching the previous file.
REM Keep this file PURE ASCII outside the quoted title below.
set "LOG=D:\Projects\Touch\Touch_Client\session_2026-09-24.log"
start "Touch-Client Output (read only)" powershell -NoProfile -Command "Get-Content -Path '%LOG%' -Wait -Tail 40"
