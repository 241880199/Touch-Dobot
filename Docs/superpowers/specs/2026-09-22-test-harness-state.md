# 测试套件现状与两个陷阱（2026-09-22）

> **这份是为了不用再查一遍而写的。** 2026-09-22 那次"离线修复"（`Docs/superpowers/plans/2026-09-22-offline-fixes.md`）
> 花了最多时间的地方不是改代码，而是**发现"测试绿了"这句话本身不可信**。两条根因都是
> `Touch_Client/tests/run_tests.bat` 的结构造成的。
>
> ### ★★★ 2026-09-22 最终状态：**两个陷阱都已修**
> 由 `Docs/superpowers/plans/2026-09-22-test-harness-builds.md`（4 个 Task + 3 轮修）完成。
> **本文件正文下面那些"【还在】/要等 Task 2"的话是【写作当时的快照】，已过期** ——
> 保留它们是为了留下"当时以为是什么样"的记录，**以本节为准**：
> - **陷阱一已修**：删掉了"只跑不建"的第一段；**五个**孤儿套件接进了"先建再跑"；
>   **12 个构建步**，每个套件都先建再跑；结果判定用 `if !ERRORLEVEL! EQU 0` + 延迟展开
>   （**崩溃的套件也会报失败**）；末尾有 `Summary` 与 `exit /b`（有失败**或**计数不符都返回非零）。
> - **陷阱二已修**：22 / 22 个 `build_*.bat` 都带 `if not defined VCINSTALLDIR` 守卫。
>   `test_noise_probe` **已在全量跑里真的跑成**（`BUILD_EXIT=0`、`5 passed, 0 failed`）——
>   这是**端到端**结果，不再是推理。
> - **有意例外**：`test_constraint_force` **不构建也不跑**（它的构建脚本被 `tests/.gitignore:1` 忽略、
>   不在 HEAD 里）⇒ 源码里留了一段明标缺口的注释。⚠ 那条忽略**尚未被决定**。
> - ### ★★ 2026-09-23 再进一步：**六个孤儿套件已接进来，三个手维护的数字改成运行时推导**
>   * **六个**"有构建脚本、从来没人接"的套件（`calib_store` / `calibration` / `frame_layout` /
>     `inertia_identification` / `self_collision` / `singularity_avoidance`）接进了"先建再跑"，
>     各自实测绿（`8/0` · `3/0` · `12/0` · `12/0` · `6/0` · `12/0`，均 exit 0）。
>   * `Summary` 的 `20`、`[NOT RUN]` 的 `8`、末尾 `if %TOTAL% NEQ 13` **全部删掉**，改成
>     **运行时数出 `test_*.cpp` 个数并断言**：磁盘上每个 `test_*.cpp` 必须**恰好被交代一次**
>     ——要么跑了，要么在 `NOTRUN_LIST` 里具名。⇒ 旧断言只能抓"段被跳过"，
>     抓不到"**新套件来了没人接线**"（那正是上面那句"会静默过期"的情形）。
>   * 现在：**`19 of 21 suites accounted for`，`Suites accounted: 21 of 21 (ran 19 + not-run 2)`**。
>   * `NOTRUN_LIST` 本身**也被校验**（名单必须对应磁盘上真实存在的 `test_*.cpp`、不许重复、
>     不许写通配符）——否则一个拼错的名字仍然会计入、算术照样平、照样 exit 0。
>   * ⚠ **本文件正文里那些 `run_tests.bat:<行号>` 引用在 09-23 之后全部平移**（文件长了约 160 行）
>     ⇒ 按**符号/描述**找，别按行号。
> - 侦察还查出**三个新缺陷类别**（见本文件【三个新缺陷类别 (A)(B)(C)】）——
>   其中一条是**一个陈旧二进制里编着已被取代的物理常数**（`J1_Z = 128.3` vs 源里的 `136.0`）。
>   ⚠ 另一条（判定标记解析期冻结）在修的过程里**衍生出两个新形态**，见下节末。

## ★ 陷阱一：6 个套件"只跑不建" ⇒ 它们的 exe 可能是几个月前的

`run_tests.bat` 分两段：第一段是一个 `for %%e in (...)`，**只 `if exist` 就跑**；
第二段才是 `call build_xxx.bat` 再跑。**只有第二段那 7 个构建步会被重建。**

**★ 2026-09-22 逐行核对 `run_tests.bat` 的结果**（下表第二列的**唯一**依据 —— 不再靠印象）：

- 第一段那个循环里**恰好 6 个** exe（`run_tests.bat:13-18`）：`test_force_pipeline` /
  `test_constraint_force` / `test_feedback_parser` / `test_escalation` / `test_kinematics` /
  `test_coord_safety` ⇒ **这 6 个才是"只跑不建"**。
- 第二段**无条件 `call` 七个** build 脚本（`run_tests.bat:42` / `:66` / `:85` / `:104` / **`:124`** /
  `:144` / `:164`），各自的 exe 紧随其后被跑 ⇒ **这 7 个是"先建再跑"**。
- ⚠ **`test_safety_core` 属于后者**：它**不在**那个 `for` 循环里，而 `run_tests.bat:124` 一直在
  `call build_safety_core_test.bat`、`:129` 跑它的 exe。
  **本表原先把它的第二列写成 `❌ 只跑`，那是错的**（并且与本文件下面那条 `test_safety_core` 的正文
  自相矛盾：那边明写"已把它移进'先建再跑'组"）—— 已改正。
  ⇒ 所以"只跑不建"的是 **6 个**（不是 7 个），而**孤儿的 build 脚本也正好是 6 个**，两者恰好重合。
  ⚠ 这个错**是"只更新结果列、没有重新审计第二列"造成的** —— 教训：**改了表里一列，要顺带把
  别的列也真核一遍**，否则会留下一个和自己正文打架的单元格。

实测时间戳（2026-09-22，HEAD `ea747ab` 时）：

**★ 2026-09-22 侦察更新（同日）**：下表那**六个 ❌ 的套件**（= 六个孤儿）**全部重建过**（exe mtime 09-22 13:54–13:55），
并逐个复跑；`test_force_pipeline` / `test_coord_safety` 在 14:05 因改注释又重编一次。最右列是重建后的复测结果
（**已逐条独立复跑核对**，不是转述）。

**★ 侦察期最要紧的那条结论**：**七个套件全都能构建、全都通过，而且新构建打出的判定与那个陈旧 exe
【逐字相同】** —— 但其中**只有 `test_force_pipeline` 一个的二进制是当前的**。
⇒ **那些绿是【对的但不可证伪】，不是"验过了"。** 这次"逐字相同"意味着：
**拿"结果对不对"当判据，永远发现不了陈旧** —— 陈旧 exe 和当前 exe 会告诉你同一句话。
（⚠ "逐字相同"这一条是**侦察记录**里的；2026-09-22 本次复跑只核了结果**数字**，
**没有**重做逐字 diff —— 而且**也做不了**了：旧 exe 已被重建覆盖。）
⚠ **但"只跑不建"这个结构当时仍在**：`run_tests.bat` 的接线（`2026-09-22-test-harness-builds` 计划的 Task 2）
**在写这份更新时尚未落地**。**"仍然成立"这句话只对上面 row-by-row 核过的那 6 个套件成立** ——
**不是一句概括**（概括恰好会漏掉 `test_safety_core` 那种情况：它看起来像"第 7 个只跑不建"，
其实一直在第二段）。区别在于：这次的绿**不再是"对的但不可证伪"**了 ——
二进制确实是当前的（见缺陷类别 (C)：本树里没法用 mtime 证明这一点，是**重建**证明的）。

> ⚠★ **下表是【修之前】的快照**（所以那六个 `❌ 只跑` 在当时是真的）。
> **现在的状态**：其中五个（force_pipeline / feedback_parser / escalation / kinematics /
> coord_safety）**已接进"先建再跑"**；`test_constraint_force` **有意排除**（不构建、不跑，
> 因为它的构建脚本不在 HEAD 里）。`test_safety_core` 本来就在"先建再跑"组里。
> ⇒ 读下表的 `❌` 时请记住它是**历史**，不是现状。

| 套件 | **当时** run_tests 会重建吗（改前快照） | 原 exe 日期 | 当时为什么可疑 | 重建后复测 |
|---|---|---|---|---|
| test_force_pipeline | ❌ 只跑 | 09-21 22:32 | 有 `build_force_pipeline_test.bat`（2026-09-21 补的），但没被 call | **7 / 0** |
| test_constraint_force | ❌ 只跑 | **09-21 20:02** | cpp 是 07-25 | **7 / 0** |
| **test_safety_core** | ✅ **先建再跑** | **07-25 15:53** | **见下**（陈旧 + ≈8% flake，是**两件事**）。⚠ **原表此处误写 `❌ 只跑`**：`run_tests.bat:124` 一直无条件 `call build_safety_core_test.bat`、`:129` 跑它 ⇒ 已改正（依据见上节逐行核对） | **8 / 0**（另有 ≈8% 时序 flake） |
| test_feedback_parser | ❌ 只跑 | 09-21 14:34 | 有 build 脚本但没被 call | **28 / 0** |
| test_escalation | ❌ 只跑 | **07-25 20:23** | ⚠ 可能是假绿 —— 陈旧约两个月 | **15 / 0** |
| test_kinematics | ❌ 只跑 | 07-25 21:02 | 与 cpp 同分钟，但见缺陷类别 (C) 里的 `J1_Z` | **18 / 0** |
| test_coord_safety | ❌ 只跑 | 09-21 22:36 | 有 build 脚本但没被 call；**头部配方还漏一个 `.cpp`（Task 3 已修）** | **27 / 0** |
| test_force_compensation / relay_command / force_logger / tcp_calibration / session_report | ✅ 先建再跑 | — | 可信 | 见下表 |
| test_noise_probe | ✅ 先建再跑 | — | ❌ **在 Task 1 补守卫之前，它从没跑成过** —— 见陷阱二 | **5 / 0**（单独跑） |

### 这条咬出过什么

1. **`test_safety_core` 报了 `7 passed, 1 failed` —— 那个红是【假的】。**
   exe 是 07-25 15:53 的，比它自己的测试源（07-25 15:59）还早 6 分钟，
   而 `RobotStateMachine.cpp`（07-26）、`RobotError.h`（09-21）之后都改过。
   **而且它当时根本构建不出来**：`RobotDiagnostics.cpp` 调
   `RelayCore::instance().reportDiagnostic(...)`，测试配方不带 RelayCore ⇒
   `LNK2019`（`RelayCore::instance` / `reportDiagnostic`）。
   **修好之后是 `8 passed, 0 failed`。**（2026-09-22 已补 `build_safety_core_test.bat` +
   `#ifndef TEST_NO_RELAY_CORE` 守卫，并把它移进"先建再跑"组。）
   ⇒ **一条"红"在动手修之前，先确认它跑的是不是当前代码**，否则会去修一个不存在的缺陷。
   （当时还编过一条"陈旧二进的指纹"——"旧 exe 的日志里 `RUNNING → DEGRADED` 出现两次"——
   **被实测反证**：那两行是两个不同发射点的正常输出，新旧 exe 都成对打。站得住的证据只有
   mtime、**那时还没有** build 脚本、链接本就不通过这三条 —— 都是**当时**的事实，不是现状。）

   ### ⚠★ 但这条"假红"的结论【只讲对了一半】—— 同一个套件还**本来就有时序 flake**

   2026-09-22 收尾时又跑了一次 `test_safety_core`，得到 **`7 passed, 1 failed`** —— 而**同一个
   二进制**在那之前明明报过 `8/0`。实测：**60 次里失败 5 次（≈8%）**，失败的是两条断言之一：
   - `can_move_guard`：`sm.currentState() == RobotState::DEGRADED`
   - `speed_factor`：`fabs(sm.speedFactor() - 0.3) < 0.01`

   **同一个根因**：这两条用例都是"发 3 个 WARN、中间 `Sleep(60)`"然后期望升级到 `DEGRADED`。
   而判据是 `elapsed >= Config::MIN_WARN_MS`（= **50**），`elapsed` 来自 `GetTickCount()`——
   **它的粒度是 15.6ms** ⇒ 真实睡了 ~60~78ms，**量出来的差值可能只有 ~46ms** ⇒
   `shouldEscalate()` 返回 false ⇒ 状态停在 `RUNNING`（`speedFactor = 1 − 3×0.15 = 0.55`）⇒ 断言红。
   ⇒ **`Sleep(60)` 对 `MIN_WARN_MS = 50` 只留了 ~10ms 余量，而时钟粒度是 15.6ms。**

   **★ 这条把上面那个结论的强度降下来了**：那条 `7/1` 有**两个**可能来源 ——
   ① 陈旧二进制（**当时**的三条证据：mtime 比自己的测试源还早 / **那时确实还没有** build 脚本 /
   链接本就不通过 —— ⚠ 这三条说的是**那次 `7/1` 当时**，**不是现在**：现在
   `build_safety_core_test.bat` 已存在，且 `run_tests.bat:124/129` 一直在构建并跑它）；
   ② **这一处本来就有的 flake**（用**新构建**也复现，8%）。
   ⇒ **"重建后跑一次是绿的"并不能证明那条红是假的** —— 对一条 8% 概率红的用例，
   单次绿几乎没有信息量。**要证伪一个红，得跑够次数**（这里 10 次都抓不到，60 次才抓到 5 次）。

   ⚠ 顺带更正一处**记录内部的分歧**：Task 4 的实现者说旧二进制失败在 `can_move_guard`，
   而控制方当时的日志显示失败在 `speed_factor` —— **两边都对**，因为 flake 会随机命中两条断言之一。
   （当时按"转述待核"记了一笔，没有抹掉；现在证明那一笔记得对。）

   **修法（尚未做）**：把那个 `Sleep(60)` 的余量拉开（例如 `Sleep(MIN_WARN_MS * 4)` 或直接
   `Sleep(200)`），或让判据用一个确定性的时间来源。**属测试自身的缺陷，与本计划的改动无关。**

2. **还有一个"绿"也可能是假的**：上表里 `test_escalation.exe`（07-25 20:23）与
   `test_constraint_force.exe`（09-21 20:02，但 cpp 是 07-25）报的 `15/0` 与 `7/0`
   **没有被核对过是不是当前代码跑出来的**。
   ⇒ **2026-09-22 侦察期已处理**：两者都重建过、复跑仍是 `15/0` 与 `7/0`（**已逐条独立复跑核对**）。
   ⚠ 但注意这**不等于**"那个绿当时是真的" —— 侦察期一共查出**三类**上面这条先例没覆盖的缺陷
   （见下面的【三个新缺陷类别】），其中 (B) 就是"它们从来没人构建过"。
   真正的收尾（把它们接进"先建再跑"）是 `2026-09-22-test-harness-builds` 计划的 **Task 2**。

## ★ 陷阱二：全量 `run_tests.bat` 里，排在后面的套件会【静默地不运行】

**同一 cmd 会话里连续 `call vcvarsall.bat`，PATH 会累积，超过 cmd 的 8191 字符上限后就报
`The input line is too long.` 并中止那一行。**

实测：连调 5 次，**前 4 次成功、第 5 次失败**。旁证：`build_session_report_test.bat:2` **本来就**
写着 `if not defined VCINSTALLDIR call "..." x64` ⇒ 去掉它之后，`run_tests.bat` 里**无保护**的
调用顺序是 force_comp → relay_command → force_logger → tcp_calibration（=4，成功）→
**第 5 个起全挂**。

⇒ **`test_noise_probe` 在全量跑里【从来没跑成过】**（构建失败 ⇒ 从不运行），
而它**单独跑是好的** ⇒ 这个洞长期没被发现。
⇒ 表现是：全量脚本退出码 **255**，控制台里留一行看不懂的错误。

### ⚠ 归因写对了，但缺的是"**为什么偏偏是它**" —— 而答案是我们自己造成的

上面把根因归给 PATH 累积是**对的**（实测同一 cmd 会话里每调一次 `vcvarsall`，
PATH 涨 **~1350** 字符：`3264 → 4613 → 5962 → 7311`；**第 5 次调用时 cmd 整体中止**：退出 255、
批处理当场死在半途，连下一个 `echo` 都不执行）。但**光有这条解释不了"为什么只有 noise_probe 受害"**。

按 `run_tests.bat` 的**真实顺序**排一遍就清楚了：

| 步 | 套件 | 当时有没有守卫 | 累计的真实 `vcvarsall` 调用 |
|---|---|---|---|
| 1–4 | force_comp / relay_command / force_logger / tcp_calibration | ❌ 无 | 1、2、3、4 |
| 5 | safety_core | ✅ 有 | 仍 4 |
| 6 | session_report | ✅ 有 | 仍 4 |
| **7** | **noise_probe** | **❌ 无** | **5 ⇒ 正好撞上断点 ⇒ 死** |

⇒ **`test_noise_probe` 之所以成为唯一的受害者，恰恰因为上一次给第 5、6 步加了守卫。**
**那次修法是把溢出从第 5 步【挪到了】第 7 步 —— 挪动了故障，不是修好了它。**
（`build_safety_core_test.bat` 自己的注释预言的正是这件事。）
**这是本项目"改行为后没回读未修改但描述它的文件"那条教训的又一个实例**：
当时只想着"补上这个套件"，没有把"下一个无守卫的调用点会被推到断点上"推一遍，
而且**没有回读**`本文件`里那句已经写好的归因。

⚠ **"只有 noise_probe 失败"是本环境的性质，不是保证**：断点取决于**起始 PATH 有多长**。
起始 PATH 更短的机器上，第 5、6 步也会一起死 —— 那时会被误读成"这两个套件坏了"。

**修法（2026-09-22 已完成）**：每个 build 脚本里写成
`if not defined VCINSTALLDIR call "...\vcvarsall.bat" x64`。
**已核实 22 个 `build_*.bat` 全部带这层守卫**（`build_*.bat` 共 22 个，逐个 grep `if not defined VCINSTALLDIR`，无遗漏）
⇒ 同一 cmd 会话里 `vcvarsall` **至多真的跑一次** ⇒ 溢出条件消失，
路径长度不再随调用次数增长，**与起始 PATH 多长无关**。
（本文件上一版写的是"只在新加的 `build_safety_core_test.bat` 上加了这层；其它脚本还没有" —— 那已经过期。）

✅ **已补上端到端验证（2026-09-22 晚）**：`test_noise_probe` 在全量 `run_tests.bat` 里
真的跑成了 —— 日志里出现了 `--- Building test_noise_probe ---` + `BUILD_EXIT=0` +
`=== test_noise_probe.exe ===` + `5 passed, 0 failed`。
（⚠ 实现者指出一个细节：它在**他自己的基线里**本来就跑得成 ⇒ 这条端到端结果算
 **Task 1 的功劳，不是 Task 2 的**。区分记得对。）

> 下面是**当时**（还没跑全量那会儿）写下的一段，保留它是因为它把"推理"与"测过"分得很清 ——
> 那正是本文件的主题。**现在已经不是当时的状况了。**
>
> ⚠★ **但"`test_noise_probe` 现在能在全量跑里跑成了"这句话，当时【没有端到端验证过】。**
> 当时给出的只是：**已核实的守卫覆盖面（22/22 个 `build_*.bat`）+ 一条推理**（守卫让
> `vcvarsall` 至多真跑一次 ⇒ 溢出条件不再出现）。**那不是跑出来的结果。**
⚠ 之所以要把这条分得这么清：**本计划存在的理由就是"不可证伪的绿"** ——
"守卫齐全"和"它真的跑成了"是同一类差别，混起来就又制造了一条不可证伪的断言。

## 怎么判断一条用例真的被跑了

- 看 run_tests 输出里有没有 `--- Building test_xxx ---` 与 `BUILD_EXIT=0`；
- ⚠★ **别再用"比 mtime"这招** —— 本文件上一版这里写的是"或直接比 `xxx.exe` 与 `xxx.cpp` 的 mtime"。
  **在这棵树里它不成立**（见下面的缺陷类别 **(C)**）：会**同时**误报和漏报。
  可靠的判据只有两条：**输出里真的出现了 `--- Building` + `BUILD_EXIT=0`**，或**看着它重建**。
- ⚠ **"能构建"不等于"链接成功"**：`build_coord_safety_test.bat` 曾漏一个 `.cpp`
  （`calibration/CalibrationIO.cpp`，提供 `Calibration::enabled/R/t`）⇒ **那条用例的链接从来没成功过**，
  过期 exe 一直躺在那里 ⇒ 跑它等于什么都没验。补上后才是第一次真验（27/0）。
  ⚠ 但这**只在脚本里**补对了：见下面缺陷类别 **(B)** —— 测试床**从不调用**那个脚本。

## ★ 2026-09-22 侦察查出的三个新缺陷类别（`test_safety_core` 那个先例没覆盖）

上面第一条先例说的是"一条红要先确认跑的是不是当前代码"。**下面这三条是同一类问题的另外三个形态**，
都在 2026-09-22 那次离线侦察里实测到，而且**一个比一个不容易看见**。

### (A) 第一段的退出码判定是【结构性死的】

`run_tests.bat:12-35`（第一段那个 `for %%e in ( test_force_pipeline.exe ... )`）把
`if %ERRORLEVEL% EQU 0` 写在**带括号的 `for` 体**里，而脚本**没开** `enabledelayedexpansion`
⇒ `%ERRORLEVEL%` **在解析时展开一次** ⇒ 每次循环都在测**循环开始前**那个值。
**⇒ 那六个套件的 `[OK]`/`[FAIL]` 不携带任何退出码信息。**
**已用替身证明**：拿一个**真的返回 1** 的 exe 喂给那个循环，它照样打 `[OK]`；
对照版把 `%ERRORLEVEL%` 换成 `!ERRORLEVEL!` ⇒ 立刻打 `[FAIL]`。
（另：`PASSED` / `FAILED` 两个计数器是**死代码** —— 脚本从头到尾没打印过它们，`endlocal` 把值直接扔掉。）

⚠ 这一条比"mtime 不可靠"更严重：它让**任何**依赖 `run_tests.bat` 输出来判成败的做法失效。

### (B) 六个 build 脚本是【孤儿】

`run_tests.bat` 只调用**七个**脚本（`build_force_comp_test` / `build_relay_command_test` /
`build_force_logger_test` / `build_tcp_calibration_test` / `build_safety_core_test` /
`build_session_report_test` / `build_noise_probe_test`），
**六个套件的那六个一次都没被调用**（force_pipeline / constraint / feedback_parser / escalation /
kinematics / coord_safety）。

⇒ **让它们"能编译"是必要但不充分的**：没有任何东西会刷新它们的二进制。
⚠ 这条**也覆盖 `build_coord_safety_test.bat`**：上次给它补 `CalibrationIO.cpp` 让它**可构建**了，
但测试床**从不构建它** ⇒ **那个修复至今没有承重** ——"补好了"和"被用上了"是两件事。

### (C) 在这棵树里，mtime **不能**当陈旧判据

- **50 个** `.cpp/.h/.bat`（全类型共 135 个文件）共享**同一个**、**内容却没变**的批量 touch 时间戳
  `2026-09-22 13:29:50`（**晚于**最后一次提交 `245b46c` 的 13:25:12）；
- 而**提交滞后**又让某些文件的 mtime **早于它自己的提交**。

⇒ 任何"源比 exe 新就重建"的规则会在这 50 个文件上**误报**，**同时漏掉真的漂移**。
（侦察记录里这个数写的是 58；2026-09-22 按 `*.cpp/*.h/*.bat` 复算得 **50**、按全类型得 135
—— **以复算为准**，量级结论不变。这也正是"别照抄数字"的一个样本。）

#### ★ 具体后果：`test_kinematics.exe` 里编着一个**已被取代的物理常数**

| 事件 | 时间 |
|---|---|
| `test_kinematics.cpp` 定稿 | 07-25 21:02:49 |
| 当时那个 `test_kinematics.exe` | 07-25 **21:02:5x** —— 比自己的 `.cpp` **还新几秒** |
| `robot/Kinematics.h` 的改动落地（commit `645ea7b`：`J1_Z: 128.3 → 136.0`，注释"实机标定: +7.7mm"） | 07-26 17:51 |

⇒ **那个 exe 比 `Kinematics.h` 的这次改动早约 20 小时** ⇒ **它里面编进去的是 `J1_Z = 128.3`**，
而源里早就是 `136.0`（`render/RobotModel.h:31` 的 `0.136f` 也印证：`+7.7mm vs URDF nominal 0.1283`）。
**`128.3 vs 136.0` 是 7.7mm 的实机标定差，不是舍入。**

**为什么它的 `18/0` 没抓到这个？** 因为 `test_kinematics.cpp` **根本没有引用 `J1_Z`**（grep：零命中）
⇒ **没有任何断言钉住那个常数** ⇒ 那个 `18/0` 对这次改动是**瞎的**。
⚠ 这是"绿"最危险的形态：**不是假绿，是"没验"** —— 它看起来和真验过一模一样。

⚠ **现状（很重要）**：2026-09-22 侦察期**已重建** `test_kinematics.exe`（09-22 13:55）⇒
**现在那个 exe 里是 136.0** ⇒ **这是一条【已记录、未修复】的发现**。
它作为缺陷类别 (C) 的证据继续成立：**那个 `18/0` 曾经可信，靠的只是运气 ——
没有任何人知道它验的是哪个常数。**
（exe 被重建后，那"还新 6 秒"的精确秒数已无法回读 ⇒ 本表那一格用 `5x` 标出，
它来自侦察记录与本文件上一版的"与 cpp 同分钟"。）

## ⚠ `文件:行号` 形式的引用：改一行注释就会移动它下面所有的行号

2026-09-22 修那两个头部构建配方时（`2026-09-22-test-harness-builds` 的 Task 3）：

- `Touch_Client/tests/test_coord_safety.cpp` 头部 **+5 行** ⇒ 下文所有行号 **+5**；
- `Touch_Client/tests/test_force_pipeline.cpp` 头部 **+4 行** ⇒ 下文所有行号 **+4**。

**⇒ 编辑点【下方】的 `文件:行号` 引用全部要重算。** 已逐条核对：

| 引用位置 | 里面写的 | 现在应是 | 状态 |
|---|---|---|---|
| `Touch_Client/relay/SafetyBoundary.h:53` | `tests/test_coord_safety.cpp:265/280/293/302/316` | **270/285/298/307/321** | ⚠ **未改** —— 那是生产头文件；Task 3 的边界只有三个文件 ⇒ 留给下一步 |
| `Docs/superpowers/plans/2026-09-22-offline-fixes.md`（`:443` / `:453` / `:462` / `:844` 等） | 按旧号引 `test_force_pipeline.cpp` | 旧 `:68`(saturation)→**72**、`:169`(stale_detection)→**173** | 计划是**历史记录**，未回改 |
| `test_force_pipeline.cpp` 自己引的 `ForcePipeline.cpp:79-89 / :126 / :135` | **别的文件**的行号 | **不变** | ✅ 已核**仍然准确**（79-89 = `mapForceToTouch` 全体；126 = 符号乘；135 = 增益乘） |

（那 5 处 `270/285/298/307/321` 正是 `SafetyBoundary::computeSpeedFactor(target)` 的 5 个调用点 —— 已逐个 grep 确认。）

⚠ **教训**：引用**符号名**（函数名/常量名）永远比引用**行号**耐用。
`SafetyBoundary.h:53` 那种"指到另一个文件的绝对行号"的写法，
**任何**一次注释改动都会让它变错，而且**错得完全无声**。

## 2026-09-22 收口时的逐套件结果

**★ 2026-09-22 本表已更新为"侦察期全部重建过"之后的真实状态。**
左列七套件的二进制**都已刷新**（09-22 13:54–13:55；`force_pipeline` / `coord_safety` 因改注释 14:05 又重编一次），
且结果**由本次逐条独立复跑核对**（不是转述侦察记录）；`test_safety_core` 的行**没有重跑**，沿用控制方那 60 次的结论。

| 套件 | 结果 | 来源 |
|---|---|---|
| test_force_pipeline | 7 / 0 | 重建后**本次独立复跑** |
| test_constraint_force | 7 / 0 | 重建后**本次独立复跑**（上一版这里写的是"⚠ 只跑不建，未核实"） |
| test_safety_core | **8 / 0，但 ≈8% 概率报 7/1** | 控制方重建后跑 10 次全绿、**跑 60 次红 5 次** ⇒ **时序 flake**（见上）；原报的 7/1 = 陈旧二进制 **+ 这个 flake** 两件事 |
| test_feedback_parser | 28 / 0 | 重建后**本次独立复跑** |
| test_escalation | 15 / 0 | 重建后**本次独立复跑**（上一版"未核实"，现已核实） |
| test_kinematics | 18 / 0 | 重建后**本次独立复跑** —— ⚠ 但这个 `18/0` **对 `J1_Z` 是瞎的**，见缺陷类别 (C) |
| test_coord_safety | 27 / 0 | 重建后**本次独立复跑** |
| test_force_compensation | **42 / 0** | 收口波（基线 38/1 → 40/0 → 42/0；本波加了 2 条守卫用例） |
| test_relay_command_parser | 11 / 0 | 基线 |
| test_force_logger | 7 / 0 | 基线 |
| test_tcp_calibration | 7 / 0 | 基线 |
| test_session_report | 20 / 0 | 基线 |
| test_noise_probe | **5 / 0** | 重建后**本次独立复跑**（**单独跑**）。上一版这里是"**没跑**" —— 那说的是**全量跑**里跑不成（陷阱二），不是它坏了 |
| test_payload_calibration | 74 / 1 | **那 1 条是刻意留红**（等重采带 `@720` 列的夹具）；见计划文件"不在本计划内" |

⚠ **计数会随用例增减漂** —— 引用前重跑，别照抄本表。

## 还有一件与"跑测试"有关的既有事实

**跑测试不是只读操作**：`test_force_compensation` / `test_payload_calibration` 会按**工作目录**
写标定状态（实测：仓库根 `calib/force_calib.json` 被测试运行改写）。
⇒ **跑之前确认没有正在运行的 `Touch_Client.exe`**（否则构建还会 `LNK1168`）。

## `.bat` 本身的坑

**不要写非 ASCII 注释。** 非 UTF-8 代码页下 cmd.exe 会误解码，**可能静默吞掉后面那一行**
—— 即"注释写错一个字符 ⇒ 编译命令消失 ⇒ 构建失败得莫名其妙"。
`build_force_pipeline_test.bat` 头部写着这条。

## 2026-09-24 —— 【运行失败】分支的实测负对照（此前从未被当成目标反证过）

`run_tests.bat` 里同一块判定手抄了 **24** 遍 —— 这是**此刻**的数：每接进一个套件就多一处，
所以它只会朝"少"的方向**静默**过期（本节初稿写的 19 是 2026-09-23 `0a72524` 处的数；
那天收工已是 22，09-24 又涨到 24 —— 而文档只在初稿里记过那一次）。
量法（四种独立计数一致）：`set /a PASSED+=1` / `echo   [FAIL: build error]` /
`call "%TESTDIR%\build_*.bat"` 各 24 处，运行日志里 `=== test_*.exe ===` 也 24 次，
harness 收尾自报 `ran 24`。⚠ 裸 `grep 'echo   [FAIL]'` 会数出 **26** —— 多出的两行是
`run_tests.bat:51`/`:53` 那段 "HOW TEST RESULTS ARE JUDGED" 教学示例里的字面文本，
**它们不是判定块**（同一个"裸 grep 把散文一起数进去"的坑，这个文档反复记）。

其内层 `else`（**构建成功、测试 exe 返回非零**）**此前从未被【当成目标】刻意反证过** ——
上一轮（2026-09-23）打的六个负对照全是**构建失败**那一路。同批的 Task 2 与 Task 4 各**顺手**
走过一次这一支（见 `_harness_task2_redwiring.log` / `_harness_task4_neg.log`，后者与本轮基数相同），
但那两次**套件自己也报了红**，且**都没有量调用方的退出码** ⇒ 分不开"判定靠退出码"
与"判定靠扫摘要"。本轮补上的正是这一半。
而"把失败传给调用方"只有一条链：`set "HARNESS_RC=%FAILED%"` → `endlocal & exit /b %HARNESS_RC%`
（本轮树上是 `:879` 与 `:898`；行号会随注释漂移，按 2026-09-24 的树看）。

做法：把 `test_calib_store.cpp` 的 `main()` 末尾 `return g_failed ? 1 : 0;` 临时改成 `return 1;`
（**构建照旧成功**）。该行在**第 47 行**（本轮执行单写的 `:163` 落在文件之外 —— 那文件总共只有 48 行；
引用 `文件:行号` 前先核文件长度 —— 本文件**前面**那节（`:277`）也说了这件事）。

**这一趟的判别力在这里**：该套件自己打的是 `3 passed, 0 failed` —— 即"文本说绿、退出码说红"。
所以它验的是判定**用了退出码**，而不是**扫套件打印的摘要**；后者是很容易被写成的假绿。

实测三条同时成立（抄自 `_harness_runfail.log` 那一趟：245499 B / 2951 行，整趟跑完）：

1. 该段打出 `[FAIL]`，不是 `[FAIL: build error]`：

```
=== test_calib_store.exe ===
=== CalibStore Tests ===
  derive_dir_normal... PASS
  derive_dir_shallow... PASS
  derive_dir_bad_input... PASS

3 passed, 0 failed
  [FAIL]
```

该段上方是 `BUILD_EXIT=0` 与 `  Build OK` ⇒ 走的是**内层** `else`。
全日志里 `[FAIL` 出现 **1** 次、`FAIL: build error` 出现 **0** 次。

2. `FAILED` = **1**。⚠ 本轮执行单把这条写成"`Suites accounted` 那行里的 `FAILED` 计数" ——
   实测 **`FAILED` 在 `Summary` 行、不在 `Suites accounted` 行**，两行都抄：

```
  Summary: 24 of 26 suites accounted for - 23 OK, 1 FAILED
```

```
  Suites accounted: 26 of 26 (ran 24 + not-run 2)    Exit code: 1
```

（这行尾巴上的 `Exit code:` 是 harness **自述**，见本节最后那条 ⚠ —— 它不是调用方量到的退出码。）

⇒ `Suites accounted` 那行**基数完全没变**（26 / ran 24 / not-run 2，与 baseline 逐字相同），
只有 `Exit code:` 从 0 变成 1；失败的套件**仍被算进 `ran`**，所以那条
`ACCOUNTED == NTESTS` 的恒等式在失败态下依然成立。

3. **进程退出码 = 1**。这一条是**从调用方量的** —— bash 里
   `MSYS_NO_PATHCONV=1 cmd.exe /c ".\run_tests.bat"` 之后的 `$?`，不是 harness 自己 echo 的那行：

```
exit=1
```

⚠ 这一条**只存在于本轮的终端记录里，没有被落进任何日志文件** —— `_harness_runfail.log` 里有的是
harness 自述的 `Exit code: 1`（`:2951`），**没有**调用方量的那个数。将来别去那个文件里找它。

与 baseline 的 `diff` 只差 5 行：`[OK]`→`[FAIL]`、`24 OK, 0 FAILED`→`23 OK, 1 FAILED`、
`24 passed, 0 failed`→`23 passed, 1 failed`、`Exit code: 0`→`Exit code: 1`，
外加一行蒙特卡洛用时（1.8 s→1.9 s，噪声）。其余 2946 行逐字节相同。

⚠ **2026-09-24 复审订正**：这里原文写的是 `1.7 s→1.9 s`。按日志实测是 **`1.8 s→1.9 s`**
（`_harness_task4.log:2603` = `用时 1.8 s`，`_harness_runfail.log:2603` = `用时 1.9 s`；
`diff` 逐行核过，两文件之间恰好只有下面列的这 5 行不同）。基线那份是 **`_harness_task4.log`**
（245497 B），不是 `_harness_baseline.log`（162283 B，更早的一趟，连蒙特卡洛那行都没有）。

还原后 sha256 与动手前**逐字相同**
（`68d32651b33a89a4dc1073f8c8c46c0875362cd30feb54aedaba7830600943f4`，1766 B），
`git status` 无该文件；随后整床 `exit=0`：

```
  Suites accounted: 26 of 26 (ran 24 + not-run 2)    Exit code: 0
```

（⚠ **2026-09-24 复审订正**：原文说 `_harness_after_neg.log` "与 baseline 只差那一行计时" ——
**不成立**。它与 baseline（`_harness_task4.log`，245497 B）**逐字节相同**：`diff` 空，
两者 sha256 同为 `3e3b8417e9a0752b9c0cecffa664e0d673524f9a42a1fd4db0e9f53831c4cea5`。还原不但把
`[FAIL]` 收回去了，连那行计时也回到了 1.8 s。）

⚠ **还原这一步有个坑：`git show HEAD:<path> > <path>` 还原不出动手前的字节。**
本文件的**工作区副本是 CRLF**（1766 B），而 **HEAD blob 是 LF**（1718 B，`afc55722…`）。
所以 brief Step 4 那条处方写出的是 LF 那一份（`9e257b04…`），而且 `git status` 会把这文件
报成 ` M`（`git diff` 却是空的 —— `core.autocrlf=true` 下按内容比是相同的）。
实测**能**还原的是 CRLF 那一份：`git show HEAD:<path> | perl -pe 's/\n/\r\n/'` ⇒
1766 B / `68d32651…`，与动手前逐字相同、`git status` 干净。

⇒ 教训：**"哪一份才是规范字节"是【每个文件各自】的** —— 取决于它的工作区副本上一次是被 git
（⇒ CRLF）还是被某个文本工具（⇒ LF）写的。Task 3 那个文件是 LF（2 734 是它动手前的样子，
`git checkout --` 出来的 2 811 才对不上）；本文件偏偏是 CRLF。两边都别想当然，
**唯一可靠的做法是拿动手前记下的 `sha256sum` 去对**；本次先按处方走、对不上，再实测 CRLF 那份才对上。

⚠ **`Exit code:` 那个数是 harness 自己 echo 的自述**（`:897`，紧接着 `:898` 才是真的 `exit /b`）。
**自述 ≠ 调用方量到的退出码。** 2026-09-24 那份 `_harness_task2_redwiring.log` 正落在这个缺口里：
它有 `[FAIL]`、有 `Summary: 23 of 25 suites accounted for - 22 OK, 1 FAILED`、也有
`Exit code: 1`，但**没有任何外部量的退出码** —— 那毕竟是一次别的改动
（`GainReadbackPolicy` 的头文件）顺手跑出来的，而且那条用例自己就打了 `4 passed, 1 failed`，
所以它分不开"判定靠退出码"与"判定靠扫摘要"。本轮的 `3 passed, 0 failed` + `[FAIL]` 才是那个判别。

⚠ **这是一次性实测，不是常驻保护**：没有任何东西会在将来再验一遍。要常驻，唯一的路是把那 24 处
调用点重构成 `call :judge` 子程序 —— 那会动一个 `cmd` 语义陷阱密集的文件
（见 `cmd-batch-semantics` 记的十一类实测），本轮**刻意不做**。
