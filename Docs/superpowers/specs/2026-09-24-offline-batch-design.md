# 2026-09-24 离线批 —— 设计（测试床三根刺 + 两处可测性接缝）

> 本批**全部离线可证**：不碰机械臂、不碰 MATLAB，每条改动都能在本机跑出判据。
> 现场侧的三条（关节模式下抑制斥力写入 / 组合模式 REJECT 每帧刷 / `sendRelayUpdate` 帧界）
> **不在本批** —— 它们要机械臂才能验，改了只能挂着当债。

## 0. 立项前先订正三条记忆（**实测，不是转述**）

本批的范围是按下面这些实测重新划的。原计划里有两项的形态与记忆里写的不一样。

| 记忆里的说法 | 2026-09-24 实测 |
|---|---|
| `test_safety_core` 有 ≈8% 时序 flake，**测试床会因它返回非零** | **跑 180 次（60 + 120）零红**。`245b46c` 那个 200 ms 余量已经把它压掉了。根因仍在，但它**不是活的** |
| 夹具读取器对未知列数**静默截断**，重采前必须先改成响亮拒绝 | **2026-09-21 已经做完**：`t6Load` / `mgLoad` / `t6LoadAttempt` / `t6ScanLastBlock` 四个都**响亮拒绝**，且有拿**真读取器**钉的用例 |
| （新）| ★ 但**钉它们的用例全部在 `test_payload_calibration` 里，而该套件在 `NOTRUN_LIST` ⇒ 一条都不跑** |
| `test_payload_calibration` "有意留红" | 新鲜构建实测 **`74 passed, 1 failed`**、exit 1。唯一一条 FAIL 是 `12:38 pose 1 不是 INCONSISTENT (state=REFERENCE_UNAVAILABLE)` |
| （新）| ★ **带 `@720` 的实机夹具已经在仓库里**：`tests/fixtures/calib_poses_2026-09-21.txt`（run-005 采，31 列，末 6 列是真数）。该套件里那条**独立验证** `..._fresh_capture` 已经吃上它，实测 **PASS**（块 `2026-09-21 17:53:00`，10 行全喂实测参考量，闸门放行 10/10） |

⇒ 结论：**"参考量那一侧"的独立验证今天就在离线跑，只是没跑在测试床上**。本批的高价值项是把它接上。

---

## 1. 甲1 —— `test_safety_core` 确定化

### 现状（核过源码）
- 5 处 `Sleep(kSettleMs)`，`kSettleMs = 4 × Config::MIN_WARN_MS = 200 ms`，只为了让 `GetTickCount()` 的 15.6 ms 粒度不咬人（`test_safety_core.cpp:38` 那段注释自己写着）。
- 判据是 `elapsed >= Config::MIN_WARN_MS`（50 ms），`elapsed` 由 `GetTickCount()` 减 `m_firstErrorMs` 得到（`EscalationTracker.h:40-45`）。
- `test_safety_core.cpp:36-37` 写着"用不了那一招（时间注入）：`RobotStateMachine` 的 `m_escalation` 是私有的"——**这句是假的**：`RobotStateMachine.h:63` 的 `EscalationTracker& escalation()` 在 `public:` 段里（`private:` 从 66 行才开始）。

### 真正的问题（三条，都不是"flake"）
1. 那句**假注释**——本项目的头号缺陷类别（"注释在说假话"）。
2. 判据对 `MIN_WARN_MS ∈ (0, 200]` **全都不敏感**：余量 200 ms 对 50 ms 的规则，改到 150 也照样绿。
3. 每次运行白花 **~1 s** 真实睡眠。

### 设计
- 5 处 `Sleep(kSettleMs)` 全部删掉，改成把时间做成**确定的输入**：
  `sm.escalation().m_firstErrorMs -= Config::MIN_WARN_MS + 1;`
  （先例：`test_escalation.cpp:92` / `:112` 早就在用同一招。）
- 补**下侧边界**：`elapsed = MIN_WARN_MS - 1` ⇒ `shouldEscalate()` 必须为 **false**（今天这条两侧都没有）。
- 保留一条**真时钟冒烟**（不睡）：`recordError` 之后立刻 `shouldEscalate()` 为 false，证明"刚记下的错误不会被当场升级"。
- 删掉 `kSettleMs` 及其那段解释，换成"为什么时间必须注入"的说明（把假前提那句一并订正）。

### 判据
- 整床绿；`test_safety_core` 的 `N passed, 0 failed` 由运行输出抄录（现有 8 条 + 本轮新增的边界条数）。
- ★ **负对照（必做，且必须打在【实现】上，不是打在常数上）**：
  把 `EscalationTracker.h` 里 `shouldEscalate()` 的比较从 `elapsed >= Config::MIN_WARN_MS`
  改成 `elapsed > Config::MIN_WARN_MS`（或 `+ 10`）⇒ 新加的边界用例**必须红**。
  ⚠ **不要**用"把 `MIN_WARN_MS` 改成 150"当负对照 —— 那条用例是**按常数参数化**写的
  （`m_firstErrorMs -= Config::MIN_WARN_MS - 1`），改常数它**照样绿**，那个对照什么都证明不了。
  这是本轮我自己先写错的一处（记在这里，免得下一轮再写一遍）。
  做完还原并用**文件哈希**核对。
- 墙钟时间降约 1 s（5 × 200 ms 睡眠消失）。

### 风险
`m_firstErrorMs` 是 public 字段但语义上是内部量。直接改它是**测试专用写**，必须在文件里写明（照 `test_escalation.cpp` 的先例）。

---

## 2. 甲2 —— 测试床【运行失败】分支的负对照

### 现状（核过源码）
- `run_tests.bat` 里同一块判定**手抄了 24 遍**（本文件原先写"19"，是 2026-09-23 的快照、**没核过**；
  `git show` 数出的漂移是 `0a72524`=19 → `5de9d3f`=22 → `c01076e`=23 → `77e7724`=24）。
  ⚠ **裸 `grep -c 'echo   \[FAIL\]'` 会数出 26** —— 文件头 "HOW TEST RESULTS ARE JUDGED" 段里
  还有两行**教学示例**（`:51` / `:53`）。可靠计数用 `set /a PASSED+=1`（=24）。
- 块形如：`if %ERRORLEVEL% EQU 0 ( 建 ⇒ 跑 ⇒ if !ERRORLEVEL! EQU 0 (PASSED+=1) else (FAILED+=1) ) else ( 构建失败 )`。
- **内层 `else`（构建成功但测试 exe 返回非零）从未被【当成目标】刻意反证过**
  —— 上一轮（`0a72524`）的六个负对照打的都是**构建失败**那一路。
  ⚠ **本句原先写"从未被看到变红"，那是错的**（Task 5 复审实测纠正）：同批的 Task 2 与 Task 4
  各**顺手**走过这一支（`_harness_task2_redwiring.log` / `_harness_task4_neg.log`，后者基数与本轮相同），
  但那两次**套件自己也报了红**、且**都没量调用方的退出码** ⇒ 分不开"判定靠退出码"与"判定靠扫摘要"。
  **Task 5 补的正是这一半。**
- 缺射程的后果是具体的：`set "HARNESS_RC=%FAILED%"` + `exit /b %HARNESS_RC%` 是唯一把失败传给调用方的东西（`run_tests.bat:786-805`）；这条链没被反证过。

### 设计
一次**实测负对照**（本项目已验证过的做法，见 `2026-09-23` 那六个负对照）：
1. 临时让一个套件的 exe **构建成功但返回非零**（改该 `test_*.cpp` 的 `main()` `return 1`，**先记下改动前的哈希**）。
2. 跑 `run_tests.bat`，断言三件事同时成立：
   (a) 该段打出 `[FAIL]`（不是 `[FAIL: build error]`）；
   (b) Summary 行 `N OK, M FAILED` 里 `M == 1`；
   (c) **进程退出码 = 1**（这一条是重点 —— 它是唯一的对外信号）。
3. **还原**并把三条结果记进测试床文档。

### 判据
- 负对照三条全中；
- 还原后 `git status` 该文件干净、再跑一次整床全绿。
- ★ **反证要求**：还原必须先核对文件哈希，不能凭"我记得改回去了"。

### 明确不做
不给测试床加"预期失败/预期红"机制 —— 那是把未验证的事变绿，本项目一贯拒绝。
**也不**把 24 处调用点重构成 `call :judge` —— 那会动一个 `cmd` 语义陷阱密集的文件（见 `cmd-batch-semantics` 那十一类实测），本轮不做，风险收益不划算。

### 已知边界（要写进文档）
这是一次性实测，**不是常驻保护**：没人会因为它而天天变红。要常驻只能做上面那个重构。

---

## 3. 甲3 —— 把 `test_payload_calibration` 接进测试床（拆分 + 接线）

### 现状（核过源码 + 实测）
- 新鲜构建（`build_payload_calibration_test.bat`，`BUILD_EXIT=0`）后运行：**`74 passed, 1 failed`**，exit 1，**只有一条 FAIL 行**。
- 那条红属于 `test_runtime_consistency_guard_replay()`（`test_payload_calibration.cpp:4459`，约 370 行），它回放**四份 09-19 老夹具**（18 / 25 列，**没有** `@720` 列），期望闸门判 `INCONSISTENT`，而闸门只能判 `REFERENCE_UNAVAILABLE`。它的 FAIL 正文自己写着：**"要它重新有判别力只能靠【重采一份带参考量那一路的列的夹具】… 不是靠改这里"**。
- 该套件在 `NOTRUN_LIST`（`run_tests.bat:667`），排除理由写在 `:743-746`："Wiring it in would make this harness exit non-zero forever."
- 于是**74 条已写好的断言从不被执行**，其中包括那批**守卫列数布局**的用例 —— 也就是"重采夹具"收口时要用来判定的那些。
- `main()` 里还有一条自认的 TODO（`:5423-5428`）："`run_tests.bat` 【不跑本文件】… runner 是受保护的, 不在这里改"。

### 设计
**拆分 + 接线，零重复代码：**

1. `test_payload_calibration.cpp` 的 `main()` 分两套，用 `#ifdef PC_PARKED_REPLAY_ONLY` 择一：
   - **默认**（无宏）：跑 74 条，**不调用**那条红的；
   - **parked**（有宏）：**只**跑那条红的。
2. 新增 `test_payload_calibration_parked.cpp`，内容就是：
   `#define PC_PARKED_REPLAY_ONLY` + `#include "test_payload_calibration.cpp"`。
   ⇒ 第二个 exe、**零重复代码**、无需把 5461 行文件里的 helper 抽成头文件。
3. **默认 exe 每次运行都响亮打印**被隔离那条用例的**名字**与原因（照 `[NOT RUN]` 那段的样式）—— 被隔离的东西必须每次都点名，不能只写在源码注释里。
4. 把 `test_payload_calibration` 接进测试床（新增 build 脚本调用段 + 计数）；`test_payload_calibration_parked` 进 `NOTRUN_LIST`（并在 `[NOT RUN]` 的理由段里补它一条）。
5. ★ **parked 的代码仍由默认 exe 每次编译**（`#ifdef` 只切 `main()`，不切函数定义）⇒ 生产 API 变了它**编译就红**，不会腐烂。只是不调用。
6. 顺带修三条**已核实**的缺陷（都在本文件内，且都属于"静默"类）：
   - `replay_real_capture`（`:1711`）：18 列 `sscanf` 对**更宽的行静默忽略**（文件自己承认）⇒ 改成先数真实列数、多列即响亮拒绝。
   - `mgParseRepeat`（`:1967` / 截断在 `:1992`）：复采对 **>8 静默截断**、且**没有任何用例** ⇒ 响亮拒绝 + 补用例。
   - `t6ScanLastBlock`（`:3086-3097`）：把"**列太宽**"也报成"**缺 @720 那六列**"⇒ 两句文案分开（误诊会把操作者引到错方向）。

### 判据
- 整床：`26 of 26 (ran 24 + not-run 2)`、该套件 `74 passed, 0 failed`、**exit 0**。
  ⚠ **2026-09-24 实测订正**：这一行是**动手前**的预测，落地的数是 **`76 passed, 0 failed`** ——
  实现期间比写这份设计时多加了 2 条用例（`76` 是**用例数**：`PASS()` 每调一次 +1，不是断言数）。
  实测输出见 `test_payload_calibration.exe` 末行；提交信息里用的也是这个量到的值。
  （基线实测：本批动手前是 **`24 of 24 (ran 22 + not-run 2)`，exit 0** —— 记忆里的 `19 of 21` 是中间态；`test_gain_readback_policy` 与 `test_payload_calibration_parked` 各 +1。）
- ★ **负对照一（证明接线真的在携带那 74 条）**：临时去掉 `#ifdef`、让默认 exe 又跑那条红的 ⇒ 主套件回到 `74/1`、整床 **exit 1**。做完还原。
- ★ **负对照二（证明三条新守卫有判别力）**：每条各配一份**改前必红**的合成夹具用例 —— 32 列（已有同类先例 `:3247`）、>8 复采对、超宽行。
- **单独构建一次 parked exe**，确认它跑的就是那一条、且**仍然红**（记录原样输出）—— 证明被隔离的确实是它。

### 风险与已知边界
- `#include "*.cpp"` 是不常见的写法。**必须在那个三行文件里写明**为什么这么写（否则下一个人会以为是笔误）。
- 主套件"**跑了**、但里面一条被静默隔离"是**基数≠身份**的又一例 —— 计数**抓不到**它。唯一的缓解是第 3 步那个"每次运行点名"，**别把绿读成"整个文件都跑了"**。
- parked exe 不在测试床上（同 `test_constraint_force` 的先例），要有人主动构建它才会跑。

---

## 4. 乙4 —— `RG|` 回读限频的决策抽成纯函数

### 现状（核过源码）
- `RelayCore::sendReflectionGain(bool force)`（`RelayCore.cpp:2505`）：内联 `GetTickCount()`（`:2506`）、三个 file-static 原子（`:2501-2503`）、限频窗口是**字面量 `100`**（`:2521`）。
- `RelayCore.cpp` **不被任何测试编译** ⇒ 这段判决**一条自动化用例都没有**。
- ★ 这处**真实发生过事故**：一轮修复把 dispatch 传成 `force=true`，限频**静默死掉**，测试床一声不响。

### 设计
新增 `relay/GainReadbackPolicy.{h,cpp}`（照 `relay/RelayCommandParser` 的形状）：

⚠ **2026-09-24 订正（与落地的形状不符）**：实际是**纯头文件** `relay/GainReadbackPolicy.h`，
**没有也不应该有 `.cpp`** —— 函数是 `inline` 定义在头里（`GainReadbackPolicy.h:30`），
照 `relay/JitterStats.h` 那个先例（计划里明确覆盖了本行写的 `.h/.cpp` 两文件形状）。
⇒ 只读本节会去找一个**不存在**的 `.cpp`。判据是构建脚本自己那句注释：
`build_gain_readback_policy_test.bat` 里写着 "GainReadbackPolicy.h is a PURE header
(inline implementation, no .cpp to link) -- same shape as JitterStats.h. Nothing else is linked
on purpose"。**若哪天这个套件需要第二个编译单元，就说明这个策略不再"纯"了。**

```cpp
enum class GainReport { Send, SkipUnchanged, SkipTooSoon };
GainReport gainReportDecision(bool force, double g, double lastSentGain,
                              unsigned long nowMs, unsigned long lastReportMs,
                              unsigned long windowMs);
```
**纯函数**：不读时钟、不碰 socket、不写 static。`RelayCore::sendReflectionGain` 改为调用它
（把 4 个 static 的值传进去、按返回值决定）。
窗口那个字面量 `100` 提成具名常数 `GAIN_REPORT_MIN_INTERVAL_MS`，**定义在 `GainReadbackPolicy.h`**
（与函数同一份定义处；`RelayCore.cpp` 只引用，不再写数字）。

### 判据
新 `test_gain_readback_policy.cpp`（照 `build_relay_command_test.bat` 直接编译 `..\relay\*.cpp`）钉四格：
- `force=true` ⇒ **恒 Send**（★ **这正是出过事故那一格**：一旦有人把 force 参与"值没变"的判断，它必须红）；
- 值没变 ⇒ `SkipUnchanged`；
- 值变了但 `now - last < window` ⇒ `SkipTooSoon`；
- 值变了且 `now - last >= window` ⇒ `Send`。
- ★ **负对照**：把"值没变"那一格写成不带 force 短路 ⇒ 第 1 格必红。

**接进测试床**（新增 build 脚本 + 段 + 计数）。

### ⚠ 需要你确认的一点
让 `RelayCore` 调用这个纯函数**是改生产代码**（行为应逐字不变，但编进 exe 的内容会变）
⇒ **下次上机前必须重建**（反正 `BTN2_J4_SIGN` 也要重建）。
**回滚 = 改回内联**。

**不采用**"只加纯函数与用例、不动 RelayCore"——那会留下**两份实现**，正是本项目栽过的那类漂移。

---

## 5. 乙5 —— `ForceTuning::tick()` 的时间接缝

### 现状（核过源码）
- `ForceTuning::tick()`（`ForceTuning.cpp:118`）：内联 `GetTickCount()`（`:120`），与 `s_dirtyMs` 比 `TUNING_DEBOUNCE_MS`（= 1000，`ForceTuning.h:18`）。
- **1 秒防抖是"跨重启保留"的唯一实现**（它是唯一的落盘路径）。
- `test_force_tuning.cpp` 有 **11** 条用例（本文件原先写"12"，是错的；Task 1 实现者按实测订正），
  **没有一条碰 `tick()`** ⇒ 生产走的那条路**一条断言都没有**（而规格的验收行「落盘 → 读回」只经由显式路径那对函数验证过）。

### ★ 本节设计里的一处缺陷（2026-09-24 实现时才发现，已修）
上面写着"断言文件不存在或内容未变"用的是 `test_force_tuning.cpp` 的临时文件 `kTmp` ——
但 `tickAt` 落盘写的是 `CalibStore::fileFor("force_tuning.json")`（**由 exe 位置推出的真实路径**），
不是 `kTmp`。**照本节字面实现，那几条"还没写"的断言会【恒真】**（文件永远不存在 ⇒ 永远绿 ⇒
防抖根本没被测到）。实现者的修法：加 `ForceTuning::setStorePathForTest(path)` 把落盘目标钉到
`kTmp`（照既有 `ForceCompensation::setStepDtForTest` 的接缝先例）。**不采用**"去断言那个真实路径"
—— 那会让测试运行时**覆盖并删掉现场的增益文件**（本项目记过这一笔：测试按工作目录写标定状态）。
⇒ 教训与本项目一贯的那条同形：**"测试绿了"要先问它是不是空转**。

### 设计
```cpp
void tickAt(unsigned long nowMs);   // 新：时间作为输入
void tick();                        // = tickAt(GetTickCount())
```
用例（`test_force_tuning.cpp` 内加，不需要新的只读访问器）：
- `t0 = GetTickCount()`（`setGain` 刚返回，`s_dirtyMs` 与它只差几微秒）；
- `tickAt(t0)` / `tickAt(t0 + 999)` ⇒ **不落盘**（断言文件不存在或内容未变）；
- `tickAt(t0 + 1000)` ⇒ **落盘**，且文件里就是那个增益；
- 不 dirty 时 `tickAt(t0 + 10^6)` ⇒ 什么都不做（不重复落盘）。

### 判据
- 四条全绿；**负对照**：去掉 `tickAt` 的窗口判断那一行 ⇒ **第 1 条**（`test_tick_at_debounces_persistence`）必红。
  ⚠ 本文件原先写"第 2 条必红"，是错的（Task 1 复审按实际输出订正：删窗口判断时第 1 条红、
  第 2 条仍绿，因为第一次 `tickAt` 已经清掉了 `s_dirty`）。实现者另补了一条**打在另一处实现**上的
  负对照（删 `s_dirty = false;` ⇒ 第 2 条红），两条合起来才说明**两条用例都不空转**。
- 沿用 `test_force_tuning.cpp` 已有的临时文件名与清理约定（`_tuning_test_tmp.json`）。

---

## 6. 明确不做（本轮）

- 第 1 组现场修法（关节模式下抑制两处斥力写入 / 组合模式 `Orient TCP REJECT` 每帧刷 / `sendRelayUpdate` 帧界）—— **要机械臂才能验**。
- 翻 `BTN2_J4_SIGN` —— 你已定"攒着和 J5/J6 一起翻"。
- **改** `test_payload_calibration` 那条红的**语义**（不"改这里"，照它自己的字面要求）。
- 给测试床加"预期红/预期失败"机制。
- 第 4 组那 22 条 Minor 的清扫（另起一批）。
- 推送 / 合并（两条分支都还没推，仓库是公开的）。
- `.gitignore:1` 那条（等你一句话）。

## 7. 本批完成后的状态

- 测试床从 `24 of 24 (ran 22 + not-run 2)` 变成 **`26 of 26 (ran 24 + not-run 2)`**，多出 **74 条**从此每次执行。
- 三处"只有人工审读守着"的接缝，各多一组用例（回读限频 / 落盘防抖 / 状态机时间判据）。
- 三条记忆订正落地（safety_core 不是活 flake；读取器守卫早已做、但没被强制执行；`test_payload_calibration` 的真实红只有一条）。
