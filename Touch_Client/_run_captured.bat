@echo off
REM ============================================================
REM  Bump LOG to today's date before every on-machine session.
REM  Hardcoded on purpose (not auto-dated): cmd's %DATE% format
REM  follows the locale, so a mis-parse writes a silently wrong
REM  filename. One visible string is easier to eyeball.
REM  WARNING: forgetting to bump it OVERWRITES the previous
REM  session's evidence log -- those raw outputs are cited by the
REM  docs and cannot be recovered once gone.
REM  NOTE: keep this file PURE ASCII. cmd reads it as GBK; UTF-8
REM  Chinese in an unquoted line gets split into bogus commands.
REM ============================================================
set "LOG=D:\Projects\Touch\Touch_Client\session_2026-09-24.log"
cd /d "D:\Projects\Touch\Touch_Client\x64\Release"
start "Touch-Client" cmd /c ".\Touch_Client.exe > %LOG% 2>&1"
