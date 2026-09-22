# 测试基建修复 Implementation Plan（2026-09-22 之二）

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让 `run_tests.bat` 真的**构建并报告**每一个套件 —— 而不是"存在就跑、且判定标记是死的"。之后"套件全绿"才第一次成为可用来判断任何事的证据。

**Architecture:** 纯 `.bat` 与注释层面的改动，不碰任何生产代码、不碰任何测试断言。三件事：① 把 `vcvarsall` 的调用做成幂等（消掉 PATH 累积，那是 `test_noise_probe` 从不运行的根因）；② 把六个孤儿 build 脚本接进"先建再跑"，并**删掉第一段"只跑不建"的循环**（那个循环的判定标记是死的，删掉它同时消掉了那个缺陷）；③ 修两处会误导人的头部构建配方。

**Tech Stack:** cmd `.bat`（**必须纯 ASCII**）、MSVC 2022 BuildTools、OpenHaptics SDK（仅头文件路径）。

## Global Constraints

- **分支**：先从 `master` 建一条新分支再动代码 —— 当前 `master` 是默认分支，不在它上面提交。
  （`master` 现在 = `245b46c`，含已合并的全部工作，且**落后 `origin/master` 300 个 commit、未推送**。）
- **⚠ `.bat` 文件必须纯 ASCII（含注释）。** 非 UTF-8 代码页下 cmd.exe 会误解码、**可能静默吞掉后面那一行**。
  要写中文说明就写进计划/规格文档或 `.bat` 之外的地方。
- **不许**改任何断言、任何容差/门限、任何生产 `.cpp`。本计划只动 `.bat` 与两个测试文件的**头部注释**。
- **不许**为了让某个套件"看起来绿"而动它。若某个套件在真构建之后**真的红**，那是**真发现** —— 照实带回，
  不要修断言、不要把它从跑列表里摘掉。
- 跑测试不是只读操作（会按工作目录写标定状态）⇒ 跑之前确认没有正在运行的 `Touch_Client.exe`。

## 开工前的事实地基（2026-09-22 实测侦察，本计划据此）

**★ 一句话结论：七个套件**（六个 + `test_noise_probe`）**全都能构建、全都通过，且新构建的判定与现有 exe 逐字相同
—— 但只有 `force_pipeline` 一个的二进制是当前的。⇒ 那些绿是【对的但不可证伪】，不是"验过了"。**

| 套件 | build 脚本 | run_tests 调用它吗 | 文档配方与脚本一致吗 | 能构建? | 新构建结果 | 陈旧吗 |
|---|---|---|---|---|---|---|
| force_pipeline | ✅ | **❌ 孤儿** | **❌ 死指针** | ✅ exit 0 | 7/0 | 否（唯一当前的） |
| constraint_force | ✅ `build_constraint_test.bat` | **❌ 孤儿** | ✅（仅格式差异） | ✅ exit 0 | 7/0 | 是（Config.h） |
| feedback_parser | ✅ | **❌ 孤儿** | ✅ | ✅ exit 0 | 28/0 | 是（RobotError.h，仅注释） |
| escalation | ✅ | **❌ 孤儿** | ✅ | ✅ exit 0 | 15/0 | **是，约两个月** |
| kinematics | ✅ | **❌ 孤儿** | ✅ | ✅ exit 0 | 18/0 | **是，且具体** |
| coord_safety | ✅（含 `CalibrationIO.cpp`） | **❌ 孤儿** | **❌ 头部漏 `CalibrationIO.cpp`，已证 LNK2019** | ✅ exit 0 | 27/0 | 是（SafetyBoundary.h、Config.h） |
| noise_probe | ✅ | ✅（第 7 个构建步） | ✅ | 独立 cmd 里 ✅ | 5/0 | 是（自己的 .cpp + NoiseProbe.h） |

### 三个新缺陷类别（`test_safety_core` 那个先例没覆盖）

**(A) 第一段的退出码判定是结构性死的。**
`run_tests.bat:12-35` 把 `if %ERRORLEVEL% EQU 0` 写在带括号的 `for` 体里，而脚本**没开**
`enabledelayedexpansion` ⇒ `%ERRORLEVEL%` **在解析时展开一次** ⇒ 每次循环都在测**循环前**那个值。
**已用替身证明**：一个真的返回 1 的 exe，那个循环照样打 `[OK]`（对照版用 `!ERRORLEVEL!` 打 `[FAIL]`）。
⇒ 那六个套件的 `[OK]`/`[FAIL]` **不携带退出码信息**。
（另：`PASSED`/`FAILED` 两个计数器是死代码 —— 脚本从头到尾没打印过它们。）

**(B) 六个 build 脚本全是孤儿。** `run_tests.bat` 只调用七个脚本
（`build_force_comp_test` / `build_relay_command_test` / `build_force_logger_test` /
`build_tcp_calibration_test` / `build_safety_core_test` / `build_session_report_test` / `build_noise_probe_test`），
**六个套件的那六个一次都没被调用**。⇒ 让它们"能编译"是**必要但不充分**：没有任何东西会刷新它们的二进制。
⚠ 这条也覆盖 `build_coord_safety_test.bat`：上次补 `CalibrationIO.cpp` 让它**可构建**了，但测试床**从不构建**它
⇒ 那个修复**至今没有承重**。

**(C) 在这棵树里 mtime 不能当陈旧判据。**
58 个 `.cpp/.h/.bat` 共享同一个**内容没变**的批量 touch 时间戳（`09-22 13:29:50`，**晚于**最后一次提交
`245b46c` 的 13:25:12），而提交滞后又让某些文件的 mtime **早于它自己的提交**。
⇒ 任何"源比 exe 新就重建"的规则都会在 58 个文件上误报，同时**漏掉真的漂移**：
**`test_kinematics.exe` 比它的测试 `.cpp` 还新 6 秒**，却**早于 `robot/Kinematics.h` 的改动一整天** ——
那次改动是 `J1_Z: 128.3 → 136.0`（注释："实机标定: +7.7mm"）。**⇒ 那个 exe 里编进去的是 128.3。**

### `test_noise_probe` 的根因（已精确复现，且**是我们自己挪过去的**）

- 实测同一 cmd 会话里 **PATH 每调一次 `vcvarsall` 涨 ~1350 字符**：`3264 → 4613 → 5962 → 7311`；
  **第 5 次调用时 cmd 整体中止**（退出 255，批处理当场死在半途，连下一个 `echo` 都不执行）。
- 按 `run_tests.bat` 的真实顺序复现：第 1~4 步（`force_comp`/`relay_command`/`force_logger`/`tcp_calibration`）
  **无守卫**地调 `vcvarsall`；第 5~6 步（`safety_core`/`session_report`）**有守卫**；
  **第 7 步 `noise_probe` 无守卫 ⇒ 正好是第 5 次真实调用 ⇒ 死。**
- ⇒ **`test_noise_probe` 之所以成为受害者，恰恰因为上次给第 5、6 步加了守卫，把溢出从第 5 步挪到了第 7 步。**
  那个修法是**挪动了故障，不是修好了它**。（`build_safety_core_test.bat` 自己的注释预言的正是这件事。）
- ⚠ **"只有 noise_probe 失败"是本环境的性质，不是保证**：断点取决于**起始 PATH**。起始 PATH 更短的机器上，
  第 5、6 步也会一起死。

## 明确**不在**本计划内的

- **`test_kinematics` 那 18/0 是否真的验过 `J1_Z`** —— 侦察没审"哪些断言消费 `J1_Z`"。
  本计划**只让它的二进制变成当前的**；若重建后它**变红**，那是真发现，照实带回并**停下来**（见 Global Constraints）。
- 六条**本来就错**的 citation（`main.cpp:2714/:2720/:2766`、`RelayCore.cpp:2133` 等）—— 属另一个计划。
- `test_payload_calibration` 那条刻意留红（等重采带 `@720` 列的夹具）。
- 上机那批（笔压/横向方向/姿态依赖/触觉摇晃）。

---

> ## ⚠★ 2026-09-22 执行中【scope 被推翻】—— 本 Task 最初只写了"四个文件"，那是错的
>
> 第一版把范围定成"`run_tests.bat` **当前**调用的那四个无守卫脚本"。实现者审计了**全部 22 个**
> `build_*.bat` 之后指出：**今天无害，但做完 Task 2 就会再炸一次** ——
> Task 2 要接进来六个**孤儿**脚本（`build_force_pipeline_test` / `build_constraint_test` /
> `build_feedback_parser_test` / `build_escalation_test` / `build_kinematics_test` /
> `build_coord_safety_test`），**它们全都是无守卫的**，再加上受害者本身
> `build_noise_probe_test.bat` 也没守卫。⇒ 做完 Task 2 就有 **7 次真实调用**
> ⇒ `1917 + 7×1349 = 11360 > 8191` ⇒ **cmd 又在第 5 次中止，形状正是本计划批评的"挪动故障"。**
>
> **控制方已独立核实**（一条 grep）：`build_*.bat` 共 **22** 个，**无守卫 16 个、有守卫 6 个**，
> 而那六个孤儿 + `noise_probe` **全部**在无守卫名单里。⇒ 实现者的意见成立，**本 Task 的范围定错了**：
> 我把"当前调用图"当成了"将来的调用图"，而 Task 2 恰好要改调用图。
>
> **⇒ 修正后的范围：把 `tests/` 下【所有】无守卫的 `call ...vcvarsall` 都加上守卫**，
> 使"一条 cmd 会话里最多真正调用一次"成为**全目录不变量**。这不比原计划多多少工作
> （每个脚本一行），但它把判据从"我们想到了 4 个"变成**一条可 grep 验证的不变量**。
>
> **新的验收判据（取代"改 4 个文件"）：`tests/` 下无守卫的 `call .*vcvarsall` 计数 == 0。**
>
> ⚠ 顺带更正 **Step 1 的方法错**：它让分五次 `cmd //c` 来复现溢出 —— 而**每次 `cmd //c` 都是
> 全新会话，PATH 根本不可能累积** ⇒ 那个步骤**结构上就复现不了**它声称要复现的东西。
> （实现者照做后没复现，并自己在一个**真正的单一会话**里复现了真症状 —— 这是对的。）
> 基线必须在**一个** cmd 会话里连续调用来做。

### Task 1: `vcvarsall` 幂等化 —— 一条会话里最多真正调用一次（全目录不变量）

**为什么**：这是 `test_noise_probe` 从不运行的**根因**，也是"全量跑一趟"不可信的原因。
现在的做法是**给个别脚本打补丁**，那只把溢出点往后挪。正确做法是**每个脚本都自带守卫**
⇒ 一条 cmd 会话里只有**第一次**真正调用 `vcvarsall`，PATH 只涨一次 ~1350 字符（上限 8191）
⇒ **任何顺序、任何调用次数都不会溢出**，而且每个脚本**仍然能单独用**。

**Files:**（**修正后**：`tests/` 下所有无守卫的 `build_*.bat`。执行时以 grep 结果为准，别照抄这份名单）
- 已改（commit `7c63249`）：`build_force_comp_test.bat` / `build_relay_command_test.bat` /
  `build_force_logger_test.bat` / `build_tcp_calibration_test.bat`
- **待改（12 个）**：`build_calib_store_test.bat` / `build_calibration_test.bat` /
  `build_constraint_test.bat` / `build_coord_safety_test.bat` / `build_escalation_test.bat` /
  `build_feedback_parser_test.bat` / `build_fk_validate.bat` / `build_force_pipeline_test.bat` /
  `build_frame_layout_test.bat` / `build_inertia_identification_test.bat` /
  `build_kinematics_test.bat` / `build_noise_probe_test.bat` /
  `build_payload_calibration_test.bat` / `build_self_collision_test.bat` /
  `build_singavoid_test.bat` / `build_test.bat` ← **以执行时的 grep 为准**（本名单可能已过时；
  控制方核实过的是"无守卫 16 个、有守卫 6 个"）
- 不动：已有守卫的 6 个（含 `build_safety_core_test.bat` / `build_session_report_test.bat` —— 它们是先例）

**Interfaces:**
- Produces: 本仓 `.bat` 的既有约定 —— 形如
  `if not defined VCINSTALLDIR call "D:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvarsall.bat" x64`
  （**逐字**照 `build_session_report_test.bat:2` 与 `build_safety_core_test.bat` 的写法）

- [ ] **Step 1: 先在一个【单一 cmd 会话】里复现溢出（这是判据基线）**

⚠★ **必须在【同一个】cmd 会话里连续调用**。分多次 `cmd //c` 是**几个全新会话**，
PATH 根本不可能累积 ⇒ **那样测结构上就不可能复现**（本计划第一版把这个方法写反了，实现者照做后没复现、
并自己用一个真正的单一会话复现了真症状 —— 那是对的）。

做法：写一个**临时**的 ASCII 助手 `.bat`（放 `/tmp`，**不要放进仓库**），在里面**按顺序 call**
四个无守卫脚本 + `build_noise_probe_test.bat`，每步之后记一次 `%PATH%` 长度与 `%ERRORLEVEL%`，
一次 `cmd //c` 跑完它。**期望**：PATH 每次涨 ~1350 字符（`3264 → 4613 → 5962 → 7311`），
**第 5 次真实调用时 cmd 中止、exit 255、后面的步骤全不执行**。
把每步的 PATH 长度与 exit 抄进报告。
⚠ 若本 shell 的起始 PATH 不同导致断点不同，如实说明；**别为了好看去调环境**。

- [ ] **Step 2: 给四个脚本各加一行守卫**

对 `build_force_comp_test.bat`、`build_relay_command_test.bat`、`build_force_logger_test.bat`、
`build_tcp_calibration_test.bat`：把原先那行
```
call "D:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvarsall.bat" x64
```
换成
```
rem 2026-09-22: guard the vcvarsall call -- PATH grows ~1350 chars per call and cmd dies at ~8191,
rem   so a second unguarded call in the same session can abort the rest of run_tests.bat.
rem   With this guard, vcvarsall runs at most once per cmd session. (ASCII only -- see
rem   build_force_pipeline_test.bat's note about non-ASCII comments in .bat files.)
if not defined VCINSTALLDIR call "D:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvarsall.bat" x64
```
⚠ **纯 ASCII**。⚠ 保持这两个 `rem` 行与 `if` 行之间的**行数与内容**一致即可，别动脚本其余部分。

- [ ] **Step 3: 判据 —— 幂等性可测，不依赖"能不能复现溢出"**

**判据 (a) —— 全目录不变量（这是主判据，取代"改了 4 个文件"）**：
```bash
cd /d/Projects/Touch/Touch_Client/tests
grep -lE '^[[:space:]]*call .*vcvarsall' build_*.bat    # 期望：无输出（计数 0）
grep -cE 'if not defined VCINSTALLDIR call .*vcvarsall' build_*.bat | grep -v ':0' | wc -l
```
判据：**无守卫的 `call .*vcvarsall` 计数为 0**。这条比"我们想到了哪几个"强得多 ——
它把要求变成一条**可 grep 验证的性质**，而不是一份可能过时的名单。
（控制方在 2026-09-22 核实过基线是"无守卫 16、有守卫 6、共 22"。你的执行结果应使前者归零。）

**判据 (b) —— 幂等性本身（性质层面，不依赖某个起始 PATH 会不会溢出）**：

新开一个 `cmd`，**在同一个会话里连续调用同一个脚本三次**，观察 PATH 长度是否只涨一次：
```bash
cmd //c "cd /d D:\Projects\Touch\Touch_Client\tests && echo PATHLEN=%PATH:~0,1% >nul & \
         call build_force_comp_test.bat >nul 2>&1 & \
         for /f \"tokens=*\" %i in ('cmd /c echo %PATH%') do @echo len-after-1st=%i" 2>&1 | tail -1
```
（若上面这条 `cmd` 拼接过绕，就用更简单可靠的办法：写一个**临时**的 ASCII `.bat`（放 `/tmp`，
不要放进仓库）连续 `call` 同一个已守卫的脚本三次，并在每次之后把 `%PATH%` 的长度写进日志。
判据是：**第 2、3 次调用后 PATH 长度不变**。）
**这是本 Task 真正的验收判据**，因为它测的是"幂等"这个性质本身，而不是某个特定起始 PATH 下会不会溢出。

- [ ] **Step 4: 确认四个脚本仍能【单独】用**

每个脚本单独跑一次（各自独立的 `cmd`），都必须 `BUILD_EXIT=0`
⇒ 证明守卫没有把它们变成"依赖别人先设好环境"。

- [ ] **Step 5: 提交**

```bash
git add Touch_Client/tests/build_force_comp_test.bat Touch_Client/tests/build_relay_command_test.bat \
        Touch_Client/tests/build_force_logger_test.bat Touch_Client/tests/build_tcp_calibration_test.bat
git commit -m "build(tests): 四个 build 脚本的 vcvarsall 调用加幂等守卫 —— 修 noise_probe 从不运行的根因

实测: 同一 cmd 会话里 PATH 每调一次 vcvarsall 涨 ~1350 字符(3264→4613→5962→7311),
第 5 次调用时 cmd 整体中止(退出 255) ⇒ run_tests.bat 从那一步起全不执行。
test_noise_probe 正是那个受害者 —— 而它成为受害者恰恰因为上次给第 5、6 步加了守卫,
把溢出从第 5 步挪到了第 7 步(挪动故障, 不是修好)。

改成每个脚本都带守卫 ⇒ 一条会话里 vcvarsall 最多真正调用一次 ⇒ 任何顺序都不溢出,
且每个脚本仍可单独使用。写法照 build_session_report_test.bat:2 的既有先例。"
```

---

### Task 2: 把六个孤儿接进"先建再跑"，并删掉第一段"只跑不建"

> **★ 前置条件（Task 1 的修正带来的）**：本 Task **必须**在 Task 1 的"全目录不变量"成立之后做 ——
> 即 `tests/` 下无守卫的 `call .*vcvarsall` 计数为 **0**。
> **否则本 Task 会把溢出重新引爆**：它要接进来六个**无守卫**的孤儿脚本
> （外加受害者 `build_noise_probe_test.bat` 本来也没守卫）⇒ 一共 7 次真实调用
> ⇒ `1917 + 7×1349 = 11360 > 8191` ⇒ cmd 在第 5 次中止，**看到的现象与本 Task 要修的一模一样**，
> 而原因换成了我们自己接进来的东西。**开工前先跑一遍那条 grep 确认它是 0。**

**为什么**：这才是让那六个套件**变成可信证据**的那一步。
今天它们"能构建"（Task 1 之外的事实）却**没有任何东西会重建它们**，而且第一段的 `[OK]`/`[FAIL]`
**结构性地不携带退出码**（缺陷 A）。两件事一起解决：把六个套件挪进"先建再跑"，
**第一段那个 `for` 循环就可以整个删掉** —— 缺陷 A 随之消失（"先建再跑"那些段落里的
`if %ERRORLEVEL% EQU 0` 是**顶层**语句，不在括号里，没有延迟展开问题）。

**Files:**
- Modify: `Touch_Client/tests/run_tests.bat`（删第一段循环；加六段"先建再跑"）

**Interfaces:**
- Consumes: 六个既有脚本名 —— `build_force_pipeline_test.bat` / `build_constraint_test.bat` /
  `build_feedback_parser_test.bat` / `build_escalation_test.bat` / `build_kinematics_test.bat` /
  `build_coord_safety_test.bat`，以及它们产出的 exe 名（`test_force_pipeline.exe` /
  `test_constraint_force.exe` / `test_feedback_parser.exe` / `test_escalation.exe` /
  `test_kinematics.exe` / `test_coord_safety.exe`）
- Produces: 一个**每个套件都会被重建**的 `run_tests.bat`

- [ ] **Step 1: 先记录"改之前"的行为，作为对照**

```bash
cmd //c "D:\Projects\Touch\Touch_Client\tests\run_tests.bat" > /tmp/before.log 2>&1; echo "exit=$?"
grep -cE "^--- Building" /tmp/before.log
grep -nE "^=== test_.*\.exe ===|^--- Building|^BUILD_EXIT" /tmp/before.log
```
把**构建次数**（`--- Building` 的行数）抄下来 —— 改之前应该是 **7**。

- [ ] **Step 2: 删掉第一段 `for` 循环**

把 `run_tests.bat` 里那个 `for %%e in ( test_force_pipeline.exe ... test_coord_safety.exe ) do ( ... )`
**整段删除**（连同它上面那句 `echo ===` 与下面的空行），把它的职责交给 Step 3 的六段。

- [ ] **Step 3: 加六段"先建再跑"**

在已有的 `test_tcp_calibration` 那一段之后、`test_session_report` 之前（或任何位置 —— 但**保持与既有段落同形**），
各加一段，形状**逐字**照现有段落：
```bat
echo --- Building test_force_pipeline ---
call "%TESTDIR%\build_force_pipeline_test.bat"
if %ERRORLEVEL% EQU 0 (
    echo   Build OK
    echo.
    echo === test_force_pipeline.exe ===
    "%TESTDIR%\test_force_pipeline.exe"
    if %ERRORLEVEL% EQU 0 (
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
```
六个套件各一段（换成对应的脚本名/exe 名）。⚠ **纯 ASCII**。
⚠ 注意 `set /a PASSED+=1` 在括号块里**不受**延迟展开影响（`+=` 不是 `%VAR%` 读取）—— 照抄即可。

**顺带修掉那两个死计数器**：`PASSED` / `FAILED` 是**死代码** —— 脚本从头到尾**从没打印过它们**，
最后一句是 `endlocal` 把它们的值直接扔掉。既然本 Task 的题目就是"让测试床真的**报告**"，
就把最后那句 `endlocal` 换成**先打印再 `endlocal`**：
```bat
echo ================================================
echo   Summary: %PASSED% suite(s) OK, %FAILED% suite(s) FAILED
echo ================================================
endlocal
```
⚠ 必须放在**顶层**（不在任何括号块里）—— `%PASSED%` 在顶层是按执行时的值展开的。
⚠ 把这一段的**行数与位置**保持在原来的 `endlocal` 处即可。
⚠ 判据：Step 4 跑完后，日志末尾应当出现 `Summary: N suite(s) OK, M suite(s) FAILED`，
且 **N + M = 13**（本计划之后的构建步数）。若 N+M ≠ 13 ⇒ 【有一个套件既没记 OK 也没记 FAILED】
⇒ 查是哪一段没走到。

- [ ] **Step 4: 跑全量，确认构建次数从 7 变成 13**

```bash
cmd //c "D:\Projects\Touch\Touch_Client\tests\run_tests.bat" > /tmp/after.log 2>&1; echo "exit=$?"
echo "build steps: $(grep -cE '^--- Building' /tmp/after.log)"      # 期望 13 = 7 + 6
grep -nE "^=== test_.*\.exe ===|^BUILD_EXIT|passed" /tmp/after.log
```
判据：**六个套件各自出现 `--- Building ... ---` + `BUILD_EXIT=0` + 自己的 `=== test_X.exe ===`**，
且它们的结果与侦察记录一致（7/0、7/0、28/0、15/0、18/0、27/0）。
⚠ 若某个套件在**真构建**之后**红了** ⇒ **真发现**，停下来照实上报（见 Global Constraints）。

- [ ] **Step 5: 确认每个 exe 的 mtime 都被刷新了**

```bash
cd /d/Projects/Touch/Touch_Client/tests
for t in test_force_pipeline test_constraint_force test_feedback_parser test_escalation test_kinematics test_coord_safety; do
  printf "%-26s %s\n" "$t" "$(date -r $t.exe '+%m-%d %H:%M:%S')"
done
```
判据：**六个 mtime 都是"刚刚"** ⇒ 证明真的重建了，而不是复用了旧二进制。
（这一步是必须的：本项目吃过"跑它等于什么都没验"的亏，而那时**唯一**能分辨的办法就是看 mtime。）

- [ ] **Step 6: 加一个【负对照】—— 证明新的报告机制真的会报失败**

**这是本计划最要紧的一步之一**：把第一段删掉的理由是"它的判定标记是死的"，
那么必须证明**新的**机制**不是**死的。做法（**临时**，必须还原）：

1. 用 Task 1 的方式新建一个临时脚本（放 `/tmp`，**不要放进仓库**），产出一个
   **必定返回非零**的假 exe：例如编译一个 `int main(){return 7;}` 到 `/tmp/_failtest.exe`；
2. 在 `run_tests.bat` 里**临时**把某一段的 exe 路径指向它（或临时把某个 `test_*.exe` 换成它）；
3. 跑全量 ⇒ **必须**看到那个套件打 `[FAIL]`；
4. **立刻还原**（`git checkout -- Touch_Client/tests/run_tests.bat` 最稳），
5. 重跑确认恢复全绿，且 `git status --porcelain` **空**。
⚠ 第 4、5 步不能省 —— "临时改完忘了还原"是同一类自欺。

- [ ] **Step 7: 提交**

```bash
git add Touch_Client/tests/run_tests.bat
git commit -m "build(tests): 六个孤儿套件接进'先建再跑', 并删掉第一段'只跑不建'的循环

两个缺陷一起解决:
① 六个 build 脚本(force_pipeline/constraint/feedback_parser/escalation/kinematics/
   coord_safety)run_tests.bat 一次都没调用过 ⇒ 它们的 exe 几个月没被重建。
   实测: 只 force_pipeline 是当前的; escalation 陈旧约两个月; kinematics 的 exe 里
   编进去的还是 J1_Z = 128.3(实际已改成 136.0 实机标定)。
② 第一段那个 for 循环的判定标记是【结构性死的】: if %ERRORLEVEL% EQU 0 写在带括号的
   for 体里而脚本没开 enabledelayedexpansion ⇒ %ERRORLEVEL% 在解析时展开一次,
   每次循环都在测循环前那个值 ⇒ 一个真的返回 1 的 exe 照样打 [OK](已用替身证明)。
   删掉第一段 ⇒ 该缺陷随之消失(新段落里的 ERRORLEVEL 判断是顶层语句, 没有这个问题)。

六个套件的断言、容差、生产代码一个字都没动。"
```

---

### Task 3: 修两处会误导人的构建配方，并把侦察事实落进记录

**为什么**：两处头部注释**照着做就会失败或缺东西**，而这份文档正是下一个人会照抄的东西。
另外把 Task 1/2 查出来的三个缺陷类别与 `J1_Z` 那个具体发现写进既有的规格文档
（`Docs/superpowers/specs/2026-09-22-test-harness-state.md`）—— 否则它们只活在本次会话里。

**Files:**
- Modify: `Touch_Client/tests/test_force_pipeline.cpp`（头部注释：死指针）
- Modify: `Touch_Client/tests/test_coord_safety.cpp`（头部注释：漏 `CalibrationIO.cpp`）
- Modify: `Docs/superpowers/specs/2026-09-22-test-harness-state.md`（补三个缺陷类别 + `J1_Z` + 更新表格）

**Interfaces:** 无新 API。

- [ ] **Step 1: 修 `test_force_pipeline.cpp` 的头部**

现在写的是 `// Build: see task-10-brief for exact command (add -I paths for OpenHaptics SDK)`。
**那个 brief 里没有任何 `cl`、`/Fe:`，也从不提 `test_force_pipeline`**（已核）⇒ 死指针。
改成指名真正能用的脚本：
```cpp
// Build: call build_force_pipeline_test.bat (in this directory) -- it carries the exact cl line.
//   ⚠ 从前这里指向 .superpowers/sdd/task-10-brief.md 里的"exact command"，而那个文件
//     (a) 被 gitignore(盘上存在但不在仓里)、(b) 里面根本没有 cl /Fe: 也不提本用例
//     ⇒ 那是一根死指针 (2026-09-22 核过)。
```

- [ ] **Step 2: 修 `test_coord_safety.cpp` 的头部**

现在的配方**没有** `../calibration/CalibrationIO.cpp` ⇒ **照着做会 `LNK2019`**
（`Calibration::enabled` / `Calibration::R` / `Calibration::t` 三个符号，已实测复现）。
把它补进配方，并说明为什么非它不可：
```cpp
// Build: cl /EHsc /std:c++17 test_coord_safety.cpp ../calibration/CalibrationIO.cpp
//        /I"..\..\OpenHaptics\Developer\3.5.0\include"
//        /I"..\..\OpenHaptics\Developer\3.5.0\utilities\include" /Fe:test_coord_safety.exe
//        /link /SUBSYSTEM:CONSOLE
//   ★ 2026-09-22: ../calibration/CalibrationIO.cpp 【不能省】—— CoordinateTransform.h 把
//     Calibration::enabled / R / t 声明为 extern, 而它们的【定义】在那个 .cpp 里
//     (namespace Calibration, 第 8 行)。少它 ⇒ LNK2019 三个符号(实测复现)。
//     本文件调用 convertTouchToRobot ⇒ 实例化那个 inline 函数 ⇒ 拉进这三个 extern。
//     或直接 call build_coord_safety_test.bat (它的配方是对的)。
```

- [ ] **Step 3: 把侦察事实写进规格文档**

在 `Docs/superpowers/specs/2026-09-22-test-harness-state.md` 里补：
- **三个新缺陷类别 (A)(B)(C)**（退出码判定解析期冻结并已用替身证明、六个 build 脚本是孤儿、
  mtime 在这棵树里不可用）；
- **`test_kinematics` 的 `J1_Z = 128.3` vs 136.0**（一个陈旧二进制里编着一个已被取代的物理常数，
  而它的 18/0 只因没有断言钉那个常数）；
- **`test_noise_probe` 的根因链条**（含"上次的守卫把溢出从第 5 步挪到第 7 步"这条自我批评），
  并更正文档里现在写的那句"根因不是环境问题"的说法（它说得对，但**归因给了 PATH 累积**是对的，
  缺的是"为什么偏偏是它"）；
- **把那两个表格里的结果列改成 Task 2 之后的真实状态**（全部重建过）。

- [ ] **Step 4: 只改注释与文档 ⇒ 构建 + 跑，确认零行为变化**

```bash
cd /d/Projects/Touch/Touch_Client/tests
for t in test_force_pipeline test_coord_safety; do ./$t.exe 2>&1 | grep -E "passed|Results:" | tail -1; done
```
Expected: `7 passed, 0 failed` 与 `27 passed, 0 failed`（本 Task 一行可执行代码都没动）。
⚠ 改了 `.cpp` 的注释会移动行号 —— 若那两个文件里有 `文件:行号` 形式的自引，**逐个核对**。

- [ ] **Step 5: 提交**

```bash
git add Touch_Client/tests/test_force_pipeline.cpp Touch_Client/tests/test_coord_safety.cpp \
        Docs/superpowers/specs/2026-09-22-test-harness-state.md
git commit -m "docs(tests): 修两处会误导人的构建配方 + 把测试基建的侦察事实落进记录

① test_force_pipeline.cpp 头部指向 .superpowers/sdd/task-10-brief.md 里的'exact command'
   —— 那是死指针: 那个文件被 gitignore、且里面根本没有 cl /Fe: 也不提本用例。
   改成指名 build_force_pipeline_test.bat。
② test_coord_safety.cpp 头部配方漏 ../calibration/CalibrationIO.cpp ⇒ 照着做 LNK2019
   三个符号(Calibration::enabled/R/t, 定义在那边而 CoordinateTransform.h 只声明 extern)。
   已实测复现 ⇒ 补进配方并写明为什么不能省。
③ 规格文档补三个新缺陷类别(判定标记解析期冻结/六个 build 脚本是孤儿/mtime 不可用)、
   test_kinematics 里 J1_Z 陈旧(128.3 vs 136.0)、以及 test_noise_probe 的完整根因链
   (含'上次的守卫把溢出从第 5 步挪到第 7 步'这条自我批评)。"
```

---

## 收尾

- [ ] 全量跑一遍并把**逐套件**结果记进 `Docs/superpowers/specs/2026-09-22-test-harness-state.md`
      （这次每个套件都真的被重建了 —— 这是本计划存在的理由）。
- [ ] **更新记忆**：`test-harness-stale-binaries.md` 补三个缺陷类别与 `J1_Z` 那条具体发现；
      把"七个套件只跑不建"这条**事实本身**更新为"已修成先建再跑（13 个构建步）"。
- [ ] 因为 `master` 落后 `origin/master` 300 个 commit 且未推送 ⇒ 与用户确认是否推送。
- [ ] **不要**合并到 `master`、不要开 PR，除非用户明说。

## 明确【不要】做的

- 不要改任何断言、任何容差/门限、任何生产 `.cpp`。若某个套件真构建之后变红 ⇒ **真发现**，停下来上报。
- 不要为了"让它绿"而把某个套件从跑列表里摘掉。
- 不要动 `test_payload_calibration`（那条刻意留红）与上机那批。
- **不要把 Task 2 Step 6 的负对照留在树里**（它必须临时、且必须证明还原干净）。
- `.bat` 里**不要写中文**。
