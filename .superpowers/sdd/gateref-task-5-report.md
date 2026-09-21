# Task 5 Report — 验证"零偏漂移检查"首次真的运行

**Branch:** `feat/pen-clamp-redesign`
**Commits:** `65500cb` (实现 + 用例) / `a95f637` (落地记录)
**Date:** 2026-09-21

## ★ 一句话边界（先说，免得被读成别的）

**本任务的交付是"可单测 + 逐分支有覆盖"，【不是】"已在实机验证过"。**
现场验证（启动客户端、等闸门放行、看这条检查真给出什么结论）**仍未做** —— 它要机械臂，
已随硬件工作一起延后（计划里与夹具重采同批）。下面所有 PASS 都是**判定**的 PASS，
不是**这条路径在真机上跑通了**。

## 1. 做了什么

判定从 `main.cpp` 的 file-static `runZeroDriftCheck()` 里抽成纯函数
`ZeroDriftCheck::decide()`，`main.cpp` 只留采样、时钟与定稿标志；四支各有用例；
并修掉"样本不足"那条静默路径。

## 2. 路由选择：**路由 ①（头文件内联纯函数）**

新文件 `Touch_Client/force/ZeroDriftCheck.h`，函数是 `inline`，判定、枚举、文字全在头里。

**为什么选 ①：** 单测那套构建是 `cl` + **显式源文件列表**，项目构建走 **MSBuild**；
单独 `.cpp` 要**同时**改两处（`.vcxproj` 与 `build_force_comp_test.bat`），漏一处就变成
"单测编得过、真机构建链接不上"（或反过来）—— 而这正是本任务里最不该出现的失效模式。
内联头文件没有这个二义性，且 **main.cpp 与用例编译的是同一份定义**（头文件内联的唯一一份），
不存在"测试里另写一份复制品"的可能。

**关于构建文件，如实说明：**

- `Touch_Client/tests/build_force_comp_test.bat`：**未改动**。头文件不是翻译单元，
  用例通过 `#include "../force/ZeroDriftCheck.h"` 取到实现，源文件列表无需新增条目。
- `Touch_Client/Touch_Client.vcxproj`：**加了一行 `<ClInclude Include="force\ZeroDriftCheck.h" />`**。
  这是**枚举性**条目、**无编译/链接动作**（`ClInclude` 不是 build action），加它是为了跟该项目
  "每个头都列进工程"的既有习惯一致，不是为了让它能编过。判据是随后的**全量构建**。

## 3. 纯函数的接口与边界

```
输入  Input{ guard, mean[3], refuseElapsedMs, sampleCount, thresholdN, waitMs, minSamples }
输出  Decision{ outcome, driftN, text }
Outcome = Waiting | Normal | OverThreshold | NotDone | InsufficientSamples
```

- **不读时钟、不读全局、不写盘、不打印。** 全部 module-static 累计量与 `GetTickCount()`
  留在 `main.cpp` 调用侧。
- `mean` 收的是**三轴均值**（不是累计和）—— 与旧代码里 `sqrt((Σ/c)²+…)` 逐位等价。
- **多了一个 `Waiting`**（超出简报说的四个）：调用侧需要一个**非终态**信号才知道"要不要重开
  累计窗口"；把 `elapsedMs >= waitMs` 这个比较同时写在调用侧和纯函数里就是两份判据。
  它是**算出来的**（同一个量的补集），不是判断。
- **三个旋钮（阈值 / 等待期 / 最少样本数）都由调用侧传入**，判定侧不写死任何数 ——
  否则阈值一变，用例里走哪一支会跟着变，断言会**静默换含义**。阈值仍只有 `Config.h` 一处定义。
- "这检查该不该跑 / 这帧该不该采样"（有没有存储零偏、`--no-robot`、读数是否 stale、是否已定稿）
  **留在调用侧** —— 那些是**状态与副作用**，不是判定。

## 4. TDD 证据

### RED-A（实现不存在 ⇒ 编不过）

```
cmd.exe //c "D:\Projects\Touch\Touch_Client\tests\build_force_comp_test.bat"
```
```
test_force_compensation.cpp: fatal error C1083:
  无法打开包括文件: "../force/ZeroDriftCheck.h": No such file or directory
BUILD_EXIT=2
```
**为什么预期：** 五条用例、六处断言全部指向还不存在的实现。这一步证明用例确实**依赖那个
实现**（而不是自说自话）。

### RED-B（静默路径，**断言级**的红）

先落一份**忠实于旧行为**的实现：样本不足那一支照旧 `return` 一个**空的 text**（= 旧代码
"设完 done 就 return，一句都不打"），然后跑：

```
Results: 34 passed, 1 failed
  zero_drift_insufficient_samples_speaks... FAIL: !d.text.empty()
```
**为什么预期：** 这正是要修的那条静默路径 —— 旧行为下这一支**没有任何文字**，
用例断言"必须出声"就必然红。它是本任务唯一一条**断言级**的红，且**只有它一条**。

> 如实记录一次中间态：RED-B 的**第一次**跑是 `33 passed / 2 failed`，多出来的
> `zero_drift_exactly_at_threshold_is_normal... FAIL: d.driftN == in.thresholdN`
> 是**我自己用例的缺陷**（`driftInput(in, in.thresholdN, …)` —— 实参在进入函数**之前**求值，
> 那时 `thresholdN` 还是默认 0，等于把均值传成了 0）。改成 `driftInput()` 之后再取
> `in.thresholdN` 即消。**缺陷在用例、不在实现**，这里点明以免被读成"实现返工"。

### GREEN

```
cmd.exe //c "D:\Projects\Touch\Touch_Client\tests\build_force_comp_test.bat"   -> BUILD_EXIT=0
./test_force_compensation.exe                                                 -> Results: 35 passed, 0 failed
  zero_drift_normal_is_a_conclusion... PASS
  zero_drift_over_threshold_is_a_conclusion... PASS
  zero_drift_exactly_at_threshold_is_normal... PASS
  zero_drift_not_done_after_full_wait... PASS
  zero_drift_waiting_stays_silent... PASS
  zero_drift_insufficient_samples_speaks... PASS
```
（构建一次、**单独执行** `.exe` 一次 —— `run_tests.bat` 不覆盖每一套。）

**修法：** 样本不足那一支现在明说 ——
`【样本不足】—— 闸门放行后只累计到 N 个样本 (至少需要 M 个), 样本不足, 本次不作结论。`
外加一句"这里**不**打印正常"的理由（样本不够时，"查了没发现问题"与"根本没查"必须分得开）。

## 5. 四支（+ 边界）怎么被覆盖，以及"调的是真实现"

| 分支 | 怎么钉住 |
|---|---|
| **放行 → 正常** | 断言 `outcome == Normal` **且 `!= NotDone`**（"真的产出了结论"）；报的数是**它自己算的三轴模**（用 0.2/0.2/0.2：模 `0.3464…` 与任一**分量都不同**，所以"把分量当结论报"会被抓住）；再断言**文字里出现的正是那个数**（`textHasNumber` 用同一个 `%g` 格式化成子串去 find），而不是测试塞进去的值 |
| **放行 → 超阈** | 同上，另断言**阈值也写进话里**。用例取 **0.8 N 静偏** —— 闸门容差 `1.2464 N` 之内 ⇒ **闸门会放行、本检查照报**，正是"本检查比闸门紧"那件事 |
| **拒绝满等待期 → 【未做】** | `UNCALIBRATED` 与 `INCONSISTENT` **两种拒绝原因都跑一遍**（都走这一支）；断言文字含「未做」、含等待秒数；**断言不含「补偿后读数」**（拒绝期间那个数恒为 0，不能当读数报）；`driftN == 0` |
| **样本不足** | 断言 `outcome == InsufficientSamples`、**`!text.empty()`**（不许静默）、含「样本不足」「不作结论」、含样本数与人下限、**不含「补偿后读数」**（不许借机编结论）、`driftN == 0` |
| 边界：**恰好等于阈值** | `0.5 N` 单轴 ⇒ 模精确 `0.5`，断言**判为正常** ⇒ 判据是**严格大于**（与抽出前逐字一致）。取 0.5 是因为它在二进制里精确、`sqrt(0.25)` 精确 —— 这条边界**不是浮点碰运气** |
| 边界：**等待中**（差 1 ms） | 断言 `outcome == Waiting`（**不是**未做）且 **`text.empty()`** —— 还在等就是不出声，旧行为保持不变 |
| 边界：**样本数** | 同一用例内 `minSamples-1` ⇒ 样本不足；`minSamples` ⇒ **必须给出结论** |

**"调的是真实现"：** 用例 `#include "../force/ZeroDriftCheck.h"`，调的是**头文件内联的唯一
一份 `ZeroDriftCheck::decide`** —— `main.cpp` 那侧 `#include` 的是同一个文件、同一份定义。
测试文件里**没有任何一份判定逻辑的副本**（用例里只有组装输入、以及"独立算一遍期望的模"
这一个校验用的算式）。

## 6. 构建与两套计数（实际数字，不是"全绿"）

- **全量构建（本次重构唯一的验证手段）：**
  `cmd.exe //c "D:\Projects\Touch\Touch_Client\build.bat"` ⇒ **`Build OK.`**
  （`main.cpp` 确实被重编；唯一告警是既有的 `C4005 CALLBACK 宏重定义`，来自 OpenHaptics 的
  `glut.h`，与本次改动无关。单测那套**不编译** `main.cpp`，所以这一步是必需的、不是形式。）
- **`test_force_compensation`：35 passed / 0 failed**（基线 29 + 新增 6）。
- **`test_payload_calibration`：69 passed / 1 failed** —— **原样**。那条断言按计划**故意红着**
  （四份夹具缺参考量那一列），归夹具重采，**本任务一个字都没碰**（也未碰它的 ⚠ 注）。

## 7. 行为有没有被改

- **阈值：没有改**（`Config::FORCE_ZERO_DRIFT_WARN_N = 0.5 N`），也没跟闸门的 `1.2464 N`
  "对齐" —— 两个数各有各的理由。
- **三条既有输出路径的文字逐字保留**：未做那四行、超阈那四行、正常那一行。数字的渲染从
  `std::cout << double` 换成 `snprintf("%g")` —— `%g` 与 ostream 默认格式（精度 6、定点/科学
  计数法的切换点）一致，所以同一组数印出来的字不变。等待秒数由 `unsigned long` 经 `%g` 出
  （`60.0 -> "60"`），与旧的字面输出一致。
- **唯一有意的可观变化：** 样本不足那一支从"一声不吭"变成明说（用户指令，见 §4）。
- **未重构 `main.cpp` 里任何别的东西**：采样/时钟/`g_noRobot`/`hasStoredZero`/stale 判断/
  两次累计窗口重开的位置全部原样。

## 8. 文件改动

- **新增** `Touch_Client/force/ZeroDriftCheck.h` —— 判定（纯函数）+ 文字。
- **改** `Touch_Client/main.cpp` —— 加 `#include "force/ZeroDriftCheck.h"`；加
  `ZERO_CHECK_MIN_SAMPLES = 10`（原为判定里的字面量，提成常量，值不变）；`runZeroDriftCheck()`
  的判定段换成组装输入 + 调 `decide()` + 打印 `d.text`；补两处注释（静默路径已修；旋钮由本侧传）。
- **改** `Touch_Client/tests/test_force_compensation.cpp` —— 六个新用例 + 两个小助手 + 在
  `main()` 的**显式调用列表**里注册（排在最后那条 ⚠ 会 `init()` 的用例**之前**）。
- **改** `Touch_Client/Touch_Client.vcxproj` —— 一行 `ClInclude`（见 §2，无编译/链接动作）。
- **改** `Docs/superpowers/specs/2026-09-21-gate-reference-prereq.md` —— 追加 §8（落地记录，
  含 §8.6 的边界声明）。计划原本让把**现场验证**记在这里；现场验证延后，所以这里记的是
  **覆盖情况 + 现场验证仍未做**。

未提交任何构建/测试产物（`calib/`、日志、`*.exe` 一概未 `git add`）。

## 9. 自审发现

1. **残留缺口（重要）：判定有覆盖，接线没有覆盖。** 用例够不到 `main.cpp`（它是
   `main()` 那一侧、不在单测的源文件列表里）。所以"调用侧确实传了真实常量、确实把
   `mean` 传成 `accum/count` 而不是累计和、确实只在 `Waiting` 时重开窗口"这几件事，
   **只有读码 + 全量构建编得过**在保证，**没有用例**。要补只能再抽一层（把调用侧也抽出
   可注入的形式），那超出本任务的范围。**把接线交给现场那一次跑**，这也是 §8.6 说
   "有覆盖 ≠ 实机验证"的第二个理由。
2. **落盘**：判定侧完全不写盘（与旧代码一致；旧代码本来就没有写盘副作用，简报里"读它的
   写盘副作用"那条前提不成立 —— 已在计划修订里更正）。跑用例会往 CWD 写标定文件，本次
   未提交任何这类文件。
3. **保留的既有边界**：均值若含 `NaN`，`NaN > 阈值` 为假 ⇒ 会被判成"正常"。**这是旧代码
   同样的行为，不是本次引入**，且 Task 4 之后非有限值会让闸门 fail-closed（`UNCALIBRATED`
   /`INCONSISTENT`），走不到这一支。**没改** —— 改它就是未被要求的语义变更。
4. **一处可忽略的差异**：旧的未做那四行是逐行 `std::endl`（逐行 flush），新的是整段
   一次性 flush。**文字相同**；差别只在"中途崩溃会丢掉整段"这一点上。
5. **`Waiting` 是第五个 outcome**（简报列了四个）。理由见 §3；它的边界由用例钉住
   （差 1 ms 不许滑到"未做"）。

## 10. 结论

- 判定已抽成**不读时钟、不读全局**的纯函数；**四支 + 两条边界**逐支有用例；用例调的是
  **真实实现**（同一份内联定义）。
- 静默路径**已修**（明说"样本不足, 本次不作结论"）并被用例钉住。
- **全量构建 `Build OK.`**；`test_force_compensation` **35/0**、
  `test_payload_calibration` **69/1（故意红，未碰）**。
- **现场验证仍然未做** ⇒ 这是**"可单测 + 有覆盖"**，**不是"已在实机验证"**。
