@echo off
setlocal enabledelayedexpansion
set "TESTDIR=D:\Projects\Touch\Touch_Client\tests"
set "PASSED=0"
set "FAILED=0"

echo ================================================
echo   Touch-Dobot Unit Tests
echo ================================================
echo.

rem ============================================================
rem HOW TEST RESULTS ARE JUDGED -- read this before editing any section.
rem   Every inner (test-result) check in this file is written as
rem
rem       if !ERRORLEVEL! EQU 0 (
rem
rem   with delayed expansion enabled by the "setlocal" on line 2 above.
rem   Do NOT rewrite it in either of these two tempting ways:
rem
rem   (a) if %ERRORLEVEL% EQU 0  -- WRONG. A %VAR% inside a parenthesised
rem       block is expanded when the block is PARSED, i.e. BEFORE the test
rem       exe has run, so it only ever sees the BUILD's exit code and even
rem       a failing suite prints [OK]. (Measured 2026-09-22 with a control
rem       that ran a stand-in returning 7: it still printed [OK].)
rem   (b) if errorlevel 1  -- WRONG. That means "errorlevel >= 1", and a
rem       CRASHING process reports a NEGATIVE code (0xC0000005 reads as
rem       -1073741819), so ">= 1" is false and a crashed suite prints [OK].
rem       (Measured 2026-09-22: a crash stand-in gave [OK] via this form.)
rem
rem   !ERRORLEVEL! is expanded at RUN time and compared numerically with
rem   EQU, so it is correct for passing, failing AND crashing suites.
rem   The OUTER (build-exit) checks are top-level statements, are NOT
rem   affected by delayed expansion, and are correct as they are.
rem
rem BUT THE OUTER CHECK IS ONLY AS GOOD AS THE BUILD SCRIPTS' LAST LINE
rem   (measured 2026-09-23). The ERRORLEVEL that "call build_x.bat" leaves
rem   behind is whatever the SCRIPT's last command left. echo, set, if, for
rem   and set /a all PRESERVE it -- which is why every build script ending in
rem   its "echo BUILD_EXIT=..." line works: the compiler's exit code survives
rem   the echo. cd and ver, by contrast, RESET it to 0 (both measured).
rem   So if anyone ever appends a cd -- or anything else that resets it --
rem   AFTER the compiler call inside a build script, a FAILED BUILD goes
rem   silent: this file prints "Build OK" and then runs the stale exe, which
rem   is the precise failure mode this harness was rebuilt to remove.
rem   Keep the compiler call last in those scripts, or re-check the code.
rem
rem WHY "@echo off" FOLLOWS EVERY "call" BELOW (do not delete those lines):
rem   Each build script starts with "@echo on", and that switch is GLOBAL to
rem   the cmd session, so it survives the call. Without restoring it, this
rem   file's own lines get echoed into the output -- including "echo   [FAIL]"
rem   from the branch that was NOT taken. A fully GREEN log would then contain
rem   the literal text "echo   [FAIL]", a false alarm for anything scraping the
rem   log for that marker. Restoring echo off never suppresses the markers
rem   themselves: "echo x" still PRINTS x -- only the echoing of the command
rem   line is suppressed.
rem ============================================================

echo ================================================
echo   Force Compensation Tests
echo ================================================
echo.
echo --- Building test_force_compensation ---
call "%TESTDIR%\build_force_comp_test.bat"
@echo off
if %ERRORLEVEL% EQU 0 (
    echo   Build OK
    echo.
    echo === test_force_compensation.exe ===
    "%TESTDIR%\test_force_compensation.exe"
    if !ERRORLEVEL! EQU 0 (
        set /a PASSED+=1
        echo   [OK]
    ) else (
        set /a FAILED+=1
        echo   [FAIL]
    )
) else (
    echo   [FAIL: build error]
    set /a FAILED+=1
)
echo.

echo ================================================
echo   Relay / Logger / Calibration Standalone Tests
echo ================================================
echo.
echo --- Building test_relay_command_parser ---
call "%TESTDIR%\build_relay_command_test.bat"
@echo off
if %ERRORLEVEL% EQU 0 (
    echo   Build OK
    echo.
    echo === test_relay_command_parser.exe ===
    "%TESTDIR%\test_relay_command_parser.exe"
    if !ERRORLEVEL! EQU 0 (
        set /a PASSED+=1
        echo   [OK]
    ) else (
        set /a FAILED+=1
        echo   [FAIL]
    )
) else (
    echo   [FAIL: build error]
    set /a FAILED+=1
)
echo.
echo --- Building test_force_logger ---
call "%TESTDIR%\build_force_logger_test.bat"
@echo off
if %ERRORLEVEL% EQU 0 (
    echo   Build OK
    echo.
    echo === test_force_logger.exe ===
    "%TESTDIR%\test_force_logger.exe"
    if !ERRORLEVEL! EQU 0 (
        set /a PASSED+=1
        echo   [OK]
    ) else (
        set /a FAILED+=1
        echo   [FAIL]
    )
) else (
    echo   [FAIL: build error]
    set /a FAILED+=1
)
echo.
echo --- Building test_tcp_calibration ---
call "%TESTDIR%\build_tcp_calibration_test.bat"
@echo off
if %ERRORLEVEL% EQU 0 (
    echo   Build OK
    echo.
    echo === test_tcp_calibration.exe ===
    "%TESTDIR%\test_tcp_calibration.exe"
    if !ERRORLEVEL! EQU 0 (
        set /a PASSED+=1
        echo   [OK]
    ) else (
        set /a FAILED+=1
        echo   [FAIL]
    )
) else (
    echo   [FAIL: build error]
    set /a FAILED+=1
)
echo.

rem ============================================================
rem Suites wired into "build then run" on 2026-09-22:
rem   force_pipeline / feedback_parser / escalation / kinematics / coord_safety
rem   They used to be run by a first section that ran them "if the exe exists"
rem   and NEVER rebuilt them, so their binaries were months old. That first
rem   section has been deleted; these five are now built, then run.
rem The FORM of the result check is NOT special to these five -- every
rem section uses it. See "HOW TEST RESULTS ARE JUDGED" at the top of this
rem file for the rule and for the two tempting-but-wrong alternatives.
rem ============================================================

echo --- Building test_force_pipeline ---
call "%TESTDIR%\build_force_pipeline_test.bat"
@echo off
if %ERRORLEVEL% EQU 0 (
    echo   Build OK
    echo.
    echo === test_force_pipeline.exe ===
    "%TESTDIR%\test_force_pipeline.exe"
    if !ERRORLEVEL! EQU 0 (
        set /a PASSED+=1
        echo   [OK]
    ) else (
        set /a FAILED+=1
        echo   [FAIL]
    )
) else (
    echo   [FAIL: build error]
    set /a FAILED+=1
)
echo.

echo --- Building test_feedback_parser ---
call "%TESTDIR%\build_feedback_parser_test.bat"
@echo off
if %ERRORLEVEL% EQU 0 (
    echo   Build OK
    echo.
    echo === test_feedback_parser.exe ===
    "%TESTDIR%\test_feedback_parser.exe"
    if !ERRORLEVEL! EQU 0 (
        set /a PASSED+=1
        echo   [OK]
    ) else (
        set /a FAILED+=1
        echo   [FAIL]
    )
) else (
    echo   [FAIL: build error]
    set /a FAILED+=1
)
echo.

echo --- Building test_escalation ---
call "%TESTDIR%\build_escalation_test.bat"
@echo off
if %ERRORLEVEL% EQU 0 (
    echo   Build OK
    echo.
    echo === test_escalation.exe ===
    "%TESTDIR%\test_escalation.exe"
    if !ERRORLEVEL! EQU 0 (
        set /a PASSED+=1
        echo   [OK]
    ) else (
        set /a FAILED+=1
        echo   [FAIL]
    )
) else (
    echo   [FAIL: build error]
    set /a FAILED+=1
)
echo.

echo --- Building test_kinematics ---
call "%TESTDIR%\build_kinematics_test.bat"
@echo off
if %ERRORLEVEL% EQU 0 (
    echo   Build OK
    echo.
    echo === test_kinematics.exe ===
    "%TESTDIR%\test_kinematics.exe"
    if !ERRORLEVEL! EQU 0 (
        set /a PASSED+=1
        echo   [OK]
    ) else (
        set /a FAILED+=1
        echo   [FAIL]
    )
) else (
    echo   [FAIL: build error]
    set /a FAILED+=1
)
echo.

echo --- Building test_coord_safety ---
call "%TESTDIR%\build_coord_safety_test.bat"
@echo off
if %ERRORLEVEL% EQU 0 (
    echo   Build OK
    echo.
    echo === test_coord_safety.exe ===
    "%TESTDIR%\test_coord_safety.exe"
    if !ERRORLEVEL! EQU 0 (
        set /a PASSED+=1
        echo   [OK]
    ) else (
        set /a FAILED+=1
        echo   [FAIL]
    )
) else (
    echo   [FAIL: build error]
    set /a FAILED+=1
)
echo.

rem ============================================================
rem test_constraint_force: NOT BUILT AND NOT RUN BY THIS HARNESS.
rem   Its build script (build_constraint_test.bat) is excluded by
rem   Touch_Client/tests/.gitignore line 1, so it is NOT in HEAD and does
rem   not exist on a fresh clone. Nothing COMMITTED can build this suite's
rem   exe. (The ignored script DOES exist locally and DOES work -- which is
rem   exactly why this is a repository-hygiene gap, not a broken suite.)
rem   Running the exe here would present a stale binary as evidence -- the
rem   exact failure mode this harness was fixed to remove. Pending a
rem   decision on that .gitignore line.
rem   (Previously this suite WAS run here, from a prebuilt binary that
rem    nothing rebuilt. Removing it makes the gap visible instead of
rem    silently producing false evidence.)
rem   See also the [NOT RUN] block printed at the end of each run.
rem ============================================================

echo --- Building test_safety_core ---
call "%TESTDIR%\build_safety_core_test.bat"
@echo off
if %ERRORLEVEL% EQU 0 (
    echo   Build OK
    echo.
    echo === test_safety_core.exe ===
    "%TESTDIR%\test_safety_core.exe"
    if !ERRORLEVEL! EQU 0 (
        set /a PASSED+=1
        echo   [OK]
    ) else (
        set /a FAILED+=1
        echo   [FAIL]
    )
) else (
    echo   [FAIL: build error]
    set /a FAILED+=1
)
echo.

echo --- Building test_session_report ---
call "%TESTDIR%\build_session_report_test.bat"
@echo off
if %ERRORLEVEL% EQU 0 (
    echo   Build OK
    echo.
    echo === test_session_report.exe ===
    "%TESTDIR%\test_session_report.exe"
    if !ERRORLEVEL! EQU 0 (
        set /a PASSED+=1
        echo   [OK]
    ) else (
        set /a FAILED+=1
        echo   [FAIL]
    )
) else (
    echo   [FAIL: build error]
    set /a FAILED+=1
)
echo.

echo --- Building test_noise_probe ---
call "%TESTDIR%\build_noise_probe_test.bat"
@echo off
if %ERRORLEVEL% EQU 0 (
    echo   Build OK
    echo.
    echo === test_noise_probe.exe ===
    "%TESTDIR%\test_noise_probe.exe"
    if !ERRORLEVEL! EQU 0 (
        set /a PASSED+=1
        echo   [OK]
    ) else (
        set /a FAILED+=1
        echo   [FAIL]
    )
) else (
    echo   [FAIL: build error]
    set /a FAILED+=1
)
echo.

echo ================================================
echo   Force Tuning Tests
echo ================================================
echo.
echo --- Building test_force_tuning ---
call "%TESTDIR%\build_force_tuning_test.bat"
@echo off
if %ERRORLEVEL% EQU 0 (
    echo   Build OK
    echo.
    echo === test_force_tuning.exe ===
    "%TESTDIR%\test_force_tuning.exe"
    if !ERRORLEVEL! EQU 0 (
        set /a PASSED+=1
        echo   [OK]
    ) else (
        set /a FAILED+=1
        echo   [FAIL]
    )
) else (
    echo   [FAIL: build error]
    set /a FAILED+=1
)
echo.

rem ============================================================
rem Suites wired into "build then run" on 2026-09-23:
rem   calib_store / calibration / frame_layout / inertia_identification /
rem   self_collision / singularity_avoidance
rem   These six had WORKING build scripts that nothing ever called -- the
rem   same orphan defect class fixed above on 2026-09-22 for five others.
rem   Measured green on 2026-09-23, right after a fresh build. Name and
rem   numbers are written as PAIRS on purpose: a bare row of counts would be
rem   read against whatever order the reader assumes, and this block is a
rem   measurement, not a number to be re-derived.
rem     calibration 8/0      calib_store 3/0        frame_layout 12/0
rem     inertia_identification 12/0   self_collision 6/0   singularity_avoidance 12/0
rem   (assertions passed / failed; all six exes exit 0.)
rem   The FORM of the result check is NOT special to these six -- every
rem   section in this file uses it. See "HOW TEST RESULTS ARE JUDGED" at the
rem   top of this file for the rule and the two tempting-but-wrong forms.
rem ============================================================

echo ================================================
echo   Calibration / Kinematics / Safety Standalone Tests
echo ================================================
echo.
echo --- Building test_calibration ---
call "%TESTDIR%\build_calibration_test.bat"
@echo off
if %ERRORLEVEL% EQU 0 (
    echo   Build OK
    echo.
    echo === test_calibration.exe ===
    "%TESTDIR%\test_calibration.exe"
    if !ERRORLEVEL! EQU 0 (
        set /a PASSED+=1
        echo   [OK]
    ) else (
        set /a FAILED+=1
        echo   [FAIL]
    )
) else (
    echo   [FAIL: build error]
    set /a FAILED+=1
)
echo.

echo --- Building test_calib_store ---
call "%TESTDIR%\build_calib_store_test.bat"
@echo off
if %ERRORLEVEL% EQU 0 (
    echo   Build OK
    echo.
    echo === test_calib_store.exe ===
    "%TESTDIR%\test_calib_store.exe"
    if !ERRORLEVEL! EQU 0 (
        set /a PASSED+=1
        echo   [OK]
    ) else (
        set /a FAILED+=1
        echo   [FAIL]
    )
) else (
    echo   [FAIL: build error]
    set /a FAILED+=1
)
echo.

echo --- Building test_frame_layout ---
call "%TESTDIR%\build_frame_layout_test.bat"
@echo off
if %ERRORLEVEL% EQU 0 (
    echo   Build OK
    echo.
    echo === test_frame_layout.exe ===
    "%TESTDIR%\test_frame_layout.exe"
    if !ERRORLEVEL! EQU 0 (
        set /a PASSED+=1
        echo   [OK]
    ) else (
        set /a FAILED+=1
        echo   [FAIL]
    )
) else (
    echo   [FAIL: build error]
    set /a FAILED+=1
)
echo.

echo --- Building test_inertia_identification ---
call "%TESTDIR%\build_inertia_identification_test.bat"
@echo off
if %ERRORLEVEL% EQU 0 (
    echo   Build OK
    echo.
    echo === test_inertia_identification.exe ===
    "%TESTDIR%\test_inertia_identification.exe"
    if !ERRORLEVEL! EQU 0 (
        set /a PASSED+=1
        echo   [OK]
    ) else (
        set /a FAILED+=1
        echo   [FAIL]
    )
) else (
    echo   [FAIL: build error]
    set /a FAILED+=1
)
echo.

echo --- Building test_self_collision ---
call "%TESTDIR%\build_self_collision_test.bat"
@echo off
if %ERRORLEVEL% EQU 0 (
    echo   Build OK
    echo.
    echo === test_self_collision.exe ===
    "%TESTDIR%\test_self_collision.exe"
    if !ERRORLEVEL! EQU 0 (
        set /a PASSED+=1
        echo   [OK]
    ) else (
        set /a FAILED+=1
        echo   [FAIL]
    )
) else (
    echo   [FAIL: build error]
    set /a FAILED+=1
)
echo.

echo --- Building test_singularity_avoidance ---
call "%TESTDIR%\build_singavoid_test.bat"
@echo off
if %ERRORLEVEL% EQU 0 (
    echo   Build OK
    echo.
    echo === test_singularity_avoidance.exe ===
    "%TESTDIR%\test_singularity_avoidance.exe"
    if !ERRORLEVEL! EQU 0 (
        set /a PASSED+=1
        echo   [OK]
    ) else (
        set /a FAILED+=1
        echo   [FAIL]
    )
) else (
    echo   [FAIL: build error]
    set /a FAILED+=1
)
echo.

echo ================================================
echo   Tests complete
echo ================================================
set /a TOTAL=PASSED+FAILED

rem ------------------------------------------------------------
rem How many suites EXIST in this repository: counted at RUN time from the
rem   sources on disk, never typed by hand. "%TESTDIR%\test_*.cpp" is the
rem   definition of "a suite lives here", so dropping in a new test_*.cpp
rem   moves this number by itself -- that is the whole point. A number that
rem   is typed by hand instead goes stale in the SILENT direction: it keeps
rem   printing a total that looks complete while the new suite is missing
rem   from the disclosure below.
rem ------------------------------------------------------------
set /a NTESTS=0
for %%F in ("%TESTDIR%\test_*.cpp") do set /a NTESTS+=1

rem ------------------------------------------------------------
rem The suites this harness deliberately does NOT build or run. One entry
rem   per suite, space separated, name WITHOUT the .cpp suffix, NO wildcards.
rem   A suite belongs here ONLY if it is named in this list; every other
rem   suite must be wired into "build then run" above and therefore must be
rem   counted by PASSED+FAILED. That invariant is asserted at the bottom of
rem   this file.
rem
rem THIS LIST IS VALIDATED, NOT TRUSTED (added 2026-09-23). Counting the
rem   entries in the string instead would put back the exact defect this file
rem   was fixed for: a name that does not exist on disk -- a typo, say --
rem   still counts as one excluded suite, the arithmetic still balances, the
rem   harness still exits 0, and the suite it was supposed to stand for is
rem   neither run nor disclosed. The three checks below close that.
rem ------------------------------------------------------------
set "NOTRUN_LIST=test_constraint_force test_payload_calibration"

rem A dictionary of the suites that EXIST, keyed by exact file stem. Entries
rem   are looked up in it rather than trusted, so a misspelled name matches
rem   nothing. (Keys are exact stems, so a wildcard never matches one either,
rem   but see the explicit wildcard check below -- that one exists because the
rem   for below would EXPAND a pattern before we ever get to look it up.)
for %%F in ("%TESTDIR%\test_*.cpp") do set "NR_SUITE_%%~nF=1"

set /a NNOTRUN=0
set /a NN_BAD=0
rem "if defined" is NOT decoration on the two loops below. Measured
rem   2026-09-23: an EMPTY set does not skip -- it runs the body ONCE with an
rem   empty for-variable. Without the guard an emptied list would therefore
rem   report a phantom entry (a [LIST ERROR] naming "") and count it. (The
rem   test_*.cpp count above needs no guard because a wildcard with no match
rem   really does loop zero times -- measured 0, same day.)
if defined NOTRUN_LIST for %%G in (%NOTRUN_LIST%) do if not defined NR_SUITE_%%G (
    echo   [LIST ERROR] NOTRUN_LIST entry "%%G" does not name exactly one
    echo               test_*.cpp in this repo. See the notes on NOTRUN_LIST.
    set /a NN_BAD+=1
)

rem Count DISTINCT names, so repeating an entry cannot inflate the count.
if defined NOTRUN_LIST for %%G in (%NOTRUN_LIST%) do if not defined NR_SEEN_%%G (
    set "NR_SEEN_%%G=1"
    set /a NNOTRUN+=1
)

rem A pattern in the list is worse than a typo: the for above would expand it
rem   into REAL suites, and the disclosure would name suites that actually ran
rem   while hiding the ones actually meant to be excluded.
rem   findstr is used deliberately instead of cmd's %VAR:*/?=% substitution,
rem   which was measured 2026-09-23 to leave a literal, unexpandable line
rem   behind (a syntax error) when the character is absent -- so the
rem   substitution idiom cannot be used to ask "does this contain an asterisk".
echo %NOTRUN_LIST%| findstr /c:"*" >nul && set /a NN_BAD+=1
echo %NOTRUN_LIST%| findstr /c:"?" >nul && set /a NN_BAD+=1

set /a ACCOUNTED=TOTAL+NNOTRUN

echo ================================================
echo   Summary: %TOTAL% of %NTESTS% suites accounted for - %PASSED% OK, %FAILED% FAILED
echo ================================================
echo.
rem ------------------------------------------------------------
rem [NOT RUN] -- the suites this harness does NOT build or run. Kept in the RUN
rem   LOG (not only in the source) so that a green run cannot be misread as
rem   "every suite in this repository is green".
rem
rem   NOTHING in this block is typed by hand any more (changed 2026-09-23):
rem   the total comes from counting test_*.cpp on disk, the not-run count and
rem   the NAME LIST below come from NOTRUN_LIST above, and the "suites that
rem   ran" figure comes from PASSED+FAILED. This block used to spell the
rem   numbers out, which meant every change needed a matching edit here --
rem   and a missed edit printed a state of the world that looked complete.
rem   Do not reintroduce a literal count in this block.
rem
rem   The REASON paragraph further down is the one surviving hand-maintained
rem   list: it repeats the two names. NOTHING enforces that the two stay in
rem   step, so if you add a name to NOTRUN_LIST, add its reason here too --
rem   otherwise the run discloses a suite it never explains.
rem ------------------------------------------------------------
echo   [NOT RUN] %NNOTRUN% of the %NTESTS% test_*.cpp in this repo are not built or run here:
echo.
if defined NOTRUN_LIST for %%F in (%NOTRUN_LIST%) do echo     %%F
echo.
echo   Why each of those is excluded (every name printed above needs a line
echo   here -- NOTRUN_LIST is the source of truth for WHICH suites are
echo   excluded, this paragraph is only the REASON):
echo.
echo     test_constraint_force
echo       Its build script is ignored by Touch_Client/tests/.gitignore line 1, so
echo       it is NOT in HEAD -- nothing COMMITTED can build this suite's exe.
echo       (Undecided; see the gap note further up this file.)
echo.
echo     test_payload_calibration
echo       Has a DELIBERATELY RED assertion: its fixture carries no reference-
echo       channel columns, so the case cannot be decided either way. Wiring it
echo       in would make this harness exit non-zero forever. Known, not a bug.
echo.
echo   The line above therefore means: %PASSED% passed, %FAILED% failed, out of the
echo   %NTESTS% test_*.cpp in this repo (%NNOTRUN% of which this harness does not run).
echo   It does NOT mean every suite in this repository is green.
echo   (The six suites with working-but-never-called build scripts -- calib_store,
echo    calibration, frame_layout, inertia_identification, self_collision,
echo    singularity_avoidance -- were wired into "build then run" on 2026-09-23
echo    and are counted in the %TOTAL% above, not in this block.)
echo ================================================
echo.

rem ------------------------------------------------------------
rem Report the result to the CALLER, not only to stdout.
rem  * Every test_*.cpp in this repo must be accounted for exactly once: it
rem    either RAN (and was counted by PASSED+FAILED) or it is named in
rem    NOTRUN_LIST above. So TOTAL + NNOTRUN must equal the number of
rem    test_*.cpp counted off the disk. When it does not, a section was
rem    silently skipped, a section was wired to something that is not a
rem    test_*.cpp, or a NEW suite was added to the repo without being wired
rem    in -- and the Summary above cannot be trusted.
rem    (Until 2026-09-23 this compared the count against a HARDCODED number
rem    of build sections, 13. That form could only catch a SKIPPED section:
rem    it did not know how many suites exist, so a newly added test_*.cpp
rem    sailed past it and simply dropped out of the [NOT RUN] disclosure.
rem    Do not go back to a hardcoded section count here.)
rem    WHAT THIS DOES NOT CATCH, stated plainly so the next editor does not
rem    trust it further than it goes: it is a check on the NUMBERS, not on the
rem    NAMES. A compensating pair -- one section deleted AND a section wired
rem    to a non-suite (e.g. the orphan build_fk_validate.bat) -- keeps the
rem    total right and passes. Each of those two edits ALONE is caught, so
rem    this needs two deliberate mistakes, but nobody should read a green run
rem    as "every section is wired to a real suite".
rem  * NN_BAD is set by the NOTRUN_LIST validation above (a name with no
rem    matching test_*.cpp, or a wildcard in the list). Those mistakes would
rem    otherwise leave the arithmetic perfectly balanced and exit 0.
rem  * The script must exit NON-ZERO when anything failed. Without this, cmd
rem    returns 0 after a failing command and the harness always looks green
rem    to whatever ran it (CI, another script, a human checking the code).
rem ------------------------------------------------------------
set "HARNESS_RC=%FAILED%"
if %ACCOUNTED% NEQ %NTESTS% (
    echo ================================================
    echo   MISMATCH: %NTESTS% test_*.cpp on disk, but %TOTAL% suites counted
    echo             + %NNOTRUN% declared not-run = %ACCOUNTED%.
    echo   A suite is unaccounted for -- the Summary above is NOT trustworthy.
    echo ================================================
    set "HARNESS_RC=1"
)
if %NN_BAD% GTR 0 (
    echo ================================================
    echo   MISMATCH: NOTRUN_LIST is invalid. Problems found: %NN_BAD%
    echo   See the LIST ERROR lines above. The list is:
    echo     %NOTRUN_LIST%
    echo ================================================
    set "HARNESS_RC=1"
)
echo.
echo   Suites accounted: %ACCOUNTED% of %NTESTS% (ran %TOTAL% + not-run %NNOTRUN%)    Exit code: %HARNESS_RC%
endlocal & exit /b %HARNESS_RC%
