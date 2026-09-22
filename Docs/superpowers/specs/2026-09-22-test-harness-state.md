# 测试套件现状与两个陷阱（2026-09-22）

> **这份是为了不用再查一遍而写的。** 2026-09-22 那次"离线修复"（`Docs/superpowers/plans/2026-09-22-offline-fixes.md`）
> 花了最多时间的地方不是改代码，而是**发现"测试绿了"这句话本身不可信**。两条根因都是
> `Touch_Client/tests/run_tests.bat` 的结构造成的，**都还在**（只补掉了其中一处）。

## ★ 陷阱一：7 个套件"只跑不建" ⇒ 它们的 exe 可能是几个月前的

`run_tests.bat` 分两段：第一段是一个 `for %%e in (...)`，**只 `if exist` 就跑**；
第二段才是 `call build_xxx.bat` 再跑。**只有第二段那 6 个会被重建。**

实测时间戳（2026-09-22，HEAD `ea747ab` 时）：

| 套件 | run_tests 会重建吗 | exe 日期 | 备注 |
|---|---|---|---|
| test_force_pipeline | ❌ 只跑 | 09-21 22:32 | 有 `build_force_pipeline_test.bat`（2026-09-21 补的），但没被 call |
| test_constraint_force | ❌ 只跑 | **09-21 20:02** | cpp 是 07-25 |
| **test_safety_core** | ❌ 只跑 | **07-25 15:53** | **见下** |
| test_feedback_parser | ❌ 只跑 | 09-21 14:34 | 有 build 脚本但没被 call |
| test_escalation | ❌ 只跑 | **07-25 20:23** | ⚠ 仍可能是假绿 |
| test_kinematics | ❌ 只跑 | 07-25 21:02 | 与 cpp 同分钟 |
| test_coord_safety | ❌ 只跑 | 09-21 22:36 | 有 build 脚本但没被 call |
| test_force_compensation / relay_command / force_logger / tcp_calibration / session_report / noise_probe | ✅ 先建再跑 | — | 可信 |

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
   mtime、无 build 脚本、链接本就不通过这三条。）

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
   ① 陈旧二进制（mtime/无脚本/链接不过，**这三条证据仍然成立**）；
   ② **这一处本来就有的 flake**（用**新构建**也复现，8%）。
   ⇒ **"重建后跑一次是绿的"并不能证明那条红是假的** —— 对一条 8% 概率红的用例，
   单次绿几乎没有信息量。**要证伪一个红，得跑够次数**（这里 10 次都抓不到，60 次才抓到 5 次）。

   ⚠ 顺带更正一处**记录内部的分歧**：Task 4 的实现者说旧二进制失败在 `can_move_guard`，
   而控制方当时的日志显示失败在 `speed_factor` —— **两边都对**，因为 flake 会随机命中两条断言之一。
   （当时按"转述待核"记了一笔，没有抹掉；现在证明那一笔记得对。）

   **修法（尚未做）**：把那个 `Sleep(60)` 的余量拉开（例如 `Sleep(MIN_WARN_MS * 4)` 或直接
   `Sleep(200)`），或让判据用一个确定性的时间来源。**属测试自身的缺陷，与本计划的改动无关。**

2. **还有一个"绿"也可能是假的，尚未处理**：上表里 `test_escalation.exe`（07-25 20:23）与
   `test_constraint_force.exe`（09-21 20:02，但 cpp 是 07-25）报的 `15/0` 与 `7/0`
   **没有被核对过是不是当前代码跑出来的**。⇒ **下一个计划的第一件事**：给它们补 build 脚本、
   移进"先建再跑"组，再拿一次真结果。

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

**修法**：脚本里写成 `if not defined VCINSTALLDIR call "...\vcvarsall.bat" x64`。
2026-09-22 只在新加的 `build_safety_core_test.bat` 上加了这层；**其它脚本还没有**。

## 怎么判断一条用例真的被跑了

- 看 run_tests 输出里有没有 `--- Building test_xxx ---` 与 `BUILD_EXIT=0`；
- 或直接比 `xxx.exe` 与 `xxx.cpp`（以及它依赖的 `.cpp`/`.h`）的 mtime；
- ⚠ **"能构建"不等于"链接成功"**：`build_coord_safety_test.bat` 曾漏一个 `.cpp`
  （`calibration/CalibrationIO.cpp`，提供 `Calibration::enabled/R/t`）⇒ **那条用例的链接从来没成功过**，
  过期 exe 一直躺在那里 ⇒ 跑它等于什么都没验。补上后才是第一次真验（27/0）。

## 2026-09-22 收口时的逐套件结果

| 套件 | 结果 | 来源 |
|---|---|---|
| test_force_pipeline | 7 / 0 | 收口波实现者跑 |
| test_constraint_force | 7 / 0 | ⚠ 只跑不建，**未核实** |
| test_safety_core | **8 / 0，但 ≈8% 概率报 7/1** | 控制方重建后跑 10 次全绿、**跑 60 次红 5 次** ⇒ **时序 flake**（见上）；原报的 7/1 = 陈旧二进制 **+ 这个 flake** 两件事 |
| test_feedback_parser | 28 / 0 | 基线（控制方跑） |
| test_escalation | 15 / 0 | ⚠ 只跑不建，**未核实** |
| test_kinematics | 18 / 0 | ⚠ 只跑不建，**未核实** |
| test_coord_safety | 27 / 0 | 基线（控制方跑） |
| test_force_compensation | **42 / 0** | 收口波（基线 38/1 → 40/0 → 42/0；本波加了 2 条守卫用例） |
| test_relay_command_parser | 11 / 0 | 基线 |
| test_force_logger | 7 / 0 | 基线 |
| test_tcp_calibration | 7 / 0 | 基线 |
| test_session_report | 20 / 0 | 基线 |
| test_noise_probe | **没跑** | 陷阱二（PATH 累积）；单独跑是好的 |
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
