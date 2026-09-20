@echo off
chcp 65001 >nul
setlocal

echo ============================================
echo   Touch-Dobot Remote Control - Build Script
echo ============================================
echo.

rem ---------------------------------------------------------------
rem KEEP THIS FILE ASCII-ONLY.
rem A .bat that runs "chcp 65001" and then contains non-ASCII bytes
rem desyncs cmd.exe's parser (it re-reads the file under the new
rem codepage), producing garbage like "'ause' is not recognized".
rem Use rem (not ::) for comments containing quotes or punctuation.
rem ---------------------------------------------------------------
rem Paths are derived from this script's own location, so the script
rem survives the repo being moved. It previously hardcoded the
rem pre-v3.0 "Codes\Touch_Client" directory, which no longer exists
rem (MSBuild then fails with MSB1009, project file not found).
rem ---------------------------------------------------------------

set "MSBUILD=D:\Program Files\Microsoft Visual Studio\2022\BuildTools\MSBuild\Current\Bin\MSBuild.exe"
set "PROJECT=%~dp0Touch_Client.vcxproj"
set "OH_SDK=D:\Projects\Touch\OpenHaptics\Developer\3.5.0"
set "OUTDIR=%~dp0x64\Release"

rem Step 0: Check the inputs up front. Without this, a missing tool or a
rem stale hardcoded path surfaces much later as a cryptic MSB1009 from
rem MSBuild, or as "The system cannot find the path specified." from
rem each copy below.
if not exist "%MSBUILD%" (
    echo [ERROR] MSBuild not found:
    echo           %MSBUILD%
    echo         Install VS2022 Build Tools, or fix MSBUILD in this script.
    pause
    exit /b 1
)
if not exist "%PROJECT%" (
    echo [ERROR] Project file not found:
    echo           %PROJECT%
    pause
    exit /b 1
)
if not exist "%OH_SDK%\lib\x64\Release\hd.dll" (
    echo [ERROR] OpenHaptics SDK not found:
    echo           %OH_SDK%
    echo         Put the 3.5.0 SDK there, or fix OH_SDK in this script.
    pause
    exit /b 1
)

rem Step 1: Compile
echo [1/3] Compiling...
"%MSBUILD%" "%PROJECT%" /p:Configuration=Release /p:Platform=x64 /v:minimal
if %ERRORLEVEL% NEQ 0 (
    echo.
    echo [ERROR] Compilation failed!
    pause
    exit /b 1
)
echo       Done.

rem Step 2: Copy DLLs. Only the three that actually ship in the
rem OpenHaptics 3.5.0 SDK. The old script also copied hdu.dll, which
rem is not present in lib/ or utilities/lib/ (silent no-op).
rem copy reports its failures on stderr, which ">nul" does NOT suppress,
rem so each one is checked instead of being left to scroll past.
echo [2/3] Copying runtime DLLs...
if not exist "%OUTDIR%" mkdir "%OUTDIR%"
copy /Y "%OH_SDK%\lib\x64\Release\hd.dll"  "%OUTDIR%\" >nul || goto :copy_failed
copy /Y "%OH_SDK%\lib\x64\Release\hl.dll"  "%OUTDIR%\" >nul || goto :copy_failed
copy /Y "%OH_SDK%\utilities\lib\x64\Release\glut32.dll" "%OUTDIR%\" >nul || goto :copy_failed
echo       Done.
goto :run

:copy_failed
echo.
echo [ERROR] Could not copy the OpenHaptics runtime DLLs into:
echo           %OUTDIR%
echo         Check that %OH_SDK% contains hd.dll, hl.dll and glut32.dll.
pause
exit /b 1

:run
rem Step 3: Run
rem ---------------------------------------------------------------
rem NO redirection here, on purpose. Read this before adding any.
rem
rem 2026-09-20: this step briefly ran
rem     Touch_Client.exe > send_transcript.txt 2>&1
rem with a second window tailing the file. That was wrong twice over:
rem
rem   1. cmd has no tee. Sending stdout to a file takes it OFF the
rem      screen, so the operator ends up typing in one window and
rem      reading another. The client reads keys through the console
rem      (_kbhit/_getch), so the window that ACCEPTS input is the one
rem      showing NOTHING - the opposite of what anyone reaches for.
rem
rem   2. stdout attached to a file or a pipe is FULL-buffered by the
rem      CRT. The per-pair settle line is printed with printf and is
rem      never flushed, so it would sit in the buffer instead of
rem      appearing where the operator is looking for it. On the
rem      console stdout is line-buffered and everything is live.
rem
rem Capturing the send text this way is not merely awkward, it is
rem impossible: sendPayloadCommands writes with std::cout, so it is
rem on stdout, and stdout is exactly the stream that cannot be both
rem shown on screen and filed to disk. The fix belongs inside the
rem program (log the send step to a file of its own), not here -
rem see the on-machine checklist, section 9.
rem ---------------------------------------------------------------
echo [3/3] Starting...
echo.
echo ============================================
if not exist "%OUTDIR%\Touch_Client.exe" (
    echo [ERROR] Touch_Client.exe was not produced at:
    echo           %OUTDIR%\Touch_Client.exe
    pause
    exit /b 1
)

echo Keys go into THIS window, and everything to read is here too.
echo.

"%OUTDIR%\Touch_Client.exe"
echo.
echo ============================================
echo Program exited.
pause
exit /b 0
