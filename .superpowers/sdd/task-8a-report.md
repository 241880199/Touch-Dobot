# Task 8a 报告 — 离线写"把负载参数发给机械臂"的代码

**日期:** 2026-09-20 · **分支:** `feat/pen-clamp-redesign` · **基线 HEAD:** `9a3bbfe`
**简报:** `.superpowers/sdd/task-8a-brief.md`
**状态:** DONE_WITH_CONCERNS —— 四条交付物全部落地、构建与全套件绿；但**简报里有两处前提与代码不符**
（见 §7），其中一处的后果会直接影响 8b 的有效性。

---

## 1. 落地清单（每个交付物的 `文件:行`）

### 交付物 1 — `RelayCore::sendPayloadToRobot`

| 位置 | 内容 |
|---|---|
| `Touch_Client/relay/RelayCore.h:56-63` | 公开成员函数声明（含"只由用户显式触发"的约束注释） |
| `Touch_Client/relay/RelayCore.cpp:226-289` | file-static `sendPayloadCommands(massKg, comMm[3])` —— 两条命令 + 逐条回执 |
| `Touch_Client/relay/RelayCore.cpp:1698-1706` | `RelayCore::sendPayloadToRobot` —— 转发 + `touchHeartbeat()` |

- 格式串与 `sendEnableRobotWithPayload` **逐字一致**：`snprintf(cmd, sizeof(cmd), "EnableRobot(%.3f,%.1f,%.1f,%.1f)", ...)`。
- 顺序：先 `EnableRobot`，后 `LoadSwitch(1)`。注释只写"**顺序取自设计 §6b 的清单（:144-146），文档未说明其必要性**"，没有编理由。
- 两条都走使能口，`robotDrainEnable()` → `robotSendEnable()` → `Sleep(100)` → `robotRecvEnable()`（照 `escapeSingularity` 的既有写法）。
- 控制台分得清 **命令文本 / 机械臂回执 / 哪一条失败**，返回两条都成功才 `true`。
- **约束写死**：不得被 `init()` / `enableRobotWithPayload` / `reenableRobotWithConnectPayload` 调用。
- **`RelayCore.cpp` 的 `LoadSwitch(0)` 一个字节都没动**（见 §4.5）。
- 附带的两个判断（简报未提，均已在代码里注明理由）：
  - **成功 = 收到回执 且 `FeedbackParser::isSuccess(fb)`**（首字符 `'0'`，即 ErrorID == 0）。只判"收到了一行"会把机械臂的拒绝读成成功。
  - **末尾调 `touchHeartbeat()`**：两条命令最坏阻塞 2×(Sleep 100 + socket 收 200ms) ≈ 600ms > `Config::HEARTBEAT_TIMEOUT_MS`(500)，而心跳检查在同一个 GLUT 线程的 `pollFeedback()` 里 ⇒ 恢复后第一帧会误报 `ERR_HEARTBEAT_LOST`(FATAL, 会下使能)。`touchHeartbeat()` 正是 RelayCore.h:91-97 为这种"故意阻塞主线程"的操作准备的。

### 交付物 2 — 候选值：先算、先打印、过两道闸

**闸的判决（纯函数，无 socket）:**

| 位置 | 内容 |
|---|---|
| `Touch_Client/force/PayloadCalibration.h:449-451` | 闸2 的三个阈值常量（公开，供打印端引用） |
| `Touch_Client/force/PayloadCalibration.h:453-461` | `enum SendGateVerdict`（7 个判决值） |
| `Touch_Client/force/PayloadCalibration.h:463-477` | `struct SendGate` |
| `Touch_Client/force/PayloadCalibration.h:479-481` | `evaluateSendGate(...)` 声明 |
| `Touch_Client/force/PayloadCalibration.cpp:1876-1925` | 实现 |
| `Touch_Client/core/SessionReport.h:99-124` | `PAYLOAD_D_MIN_MM` / `PAYLOAD_D_MAX_MM` + `payloadDValues()` |
| `Touch_Client/core/SessionReport.h:126-133` | `payloadDSection` 改为调用 `payloadDValues()`（输出字节不变） |

**打印候选（在 `BiasCheck::solveAndApply()` 求解成功之后）:**

| 位置 | 内容 |
|---|---|
| `Touch_Client/main.cpp:1080-1125` | `formatSendGateConclusion()` —— 结论行的**唯一**一份实现 |
| `Touch_Client/main.cpp:1592-1692` | 候选块本体（取数 → 过闸 → 打印 → 留给 `'p'`） |
| `Touch_Client/main.cpp:171-175` | `s_sendCandidateValid` / `s_sendCandidate` 两个静态 |
| `Touch_Client/main.cpp:187-190` | `reset()` 里把候选作废 |
| `Touch_Client/main.cpp:1598-1603` | 求解失败时把候选作废（不保留上一次的存货） |

打印内容（每一行的数字都来自变量）：
- 候选 `m` / `(cx,cy,cz)` / `|c|`，来源标签取 `PayloadCalibration::enabled`（`"实机标定值 [payload_calib.json]"` / `"种子值, 未标定 [Config 的 ROBOT_PAYLOAD_SEED]"`）。
- 闸1 的两支 `d` 与判读；闸2 的两个量与判读。
- 结论行：`候选可发送（按 'p'）` 或 `候选不可发送：<哪一闸、为什么>`。
- `⚠ 此刻【什么都没有发出去】`。

`cz_robot` 取 `appState.forceData.payloadEchoCenterMm[2]`（`payloadEchoValid` 为真时）；取不到传 `nullptr` ⇒ 闸1 报 `SEND_NO_CZ_ROBOT` 并不放行，**没有**退回自己下发的值。
`c_s` 可不可用的口径与块尾那一节一致（`decompOk`）；不可用时**连 `d` 的数字都不打**（见 §6 自查 2）。
两支 `d` 的算术**没有第二份实现**：闸1 与文档块那一节共用 `SessionReport::payloadDValues()`。

### 交付物 3 — 显式发送键 `'p'`

| 位置 | 内容 |
|---|---|
| `Touch_Client/main.cpp:2005-2052` | `BiasCheck::sendCandidate()` |
| `Touch_Client/main.cpp:2596-2602` | 按键分派（紧挨 `'s'` 那一段之后） |

逐条验收：
1. 没有候选 → 打 `没有候选 —— 先按 'm' 采姿态, 再按 's' 求解` + `本次【什么都没有发出去】`，**不发送**。
2. 过不了闸 → 打出行 `候选不可发送：<哪一闸、为什么>`（与 `'s'` 那一屏**同源**）+ `本次【什么都没有发出去】`，**不发送**。
3. 可发送 → **先**打安全规程四条 + `⚠ 运行中改负载会让机械臂动 —— 2026-09-19 实机证实（1.5 kg 那次撞向关节限位）`，**再**发送；发送后打结果与逐条回执。
4. `'p'` 是唯一入口：见 §4.4 的 grep。
5. **不重算**：用 `'s'` 留下的 `s_sendCandidate`。

### 交付物 4 — 修掉失真注释

`main.cpp:2573-2593`（原 `main.cpp:2362`）。**原文 → 新文**见 §5。

---

## 2. TDD 证据

### RED

命令：`cmd.exe //c "D:\Projects\Touch\Touch_Client\tests\build_payload_calibration_test.bat"`
（先加测试、断言 `PayloadCalibration::evaluateSendGate`，实现还不存在）

```
test_payload_calibration.cpp(3347): error C2039: "SendGate": 不是 "PayloadCalibration" 的成员
test_payload_calibration.cpp(3348): error C2039: "evaluateSendGate": 不是 "PayloadCalibration" 的成员
test_payload_calibration.cpp(3349): error C2039: "SEND_OK": 不是 "PayloadCalibration" 的成员
... (每个用例各报一遍)
```

**为什么这个失败是预期的**：测试先写，被测的纯函数与它的枚举/结构体都还不存在 ⇒ 编译期就红。
这正是"测试真的在调那个待实现的接口"的证据（而不是"写了个恒真的断言、编译过了"）。

### GREEN

命令：`cmd.exe //c "D:\Projects\Touch\Touch_Client\tests\build_payload_calibration_test.bat"` → `BUILD_EXIT=0`

```
--- Task 8a: pre-send gates (sign convention + magnitude) ---
  send_gate_convention_one_wins_and_signs_cz... PASS
  send_gate_convention_two_wins_and_signs_cz... PASS
  send_gate_two_conventions_in_range_is_ambiguous... PASS
  send_gate_no_convention_in_range_is_refused... PASS
  send_gate_interval_is_open_at_both_ends... PASS
  send_gate_mass_bounds_are_inclusive... PASS
  send_gate_mass_outside_bounds_is_refused... PASS
  send_gate_com_magnitude_bound_is_inclusive_at_500... PASS
  send_gate_refuses_without_cs_and_says_so... PASS
  send_gate_refuses_without_cz_robot_and_says_so... PASS

59 passed, 0 failed
```

基线（改动前同一条命令）：`49 passed, 0 failed` ⇒ 本任务新增 10 条，全部通过。

### 用例 → 断言对照（简报 §3 表）

| # | 用例 | 输入 | 断言 |
|---|---|---|---|
| 1 | 恰好约定一落在 (0,31.5) | `c_s_z=+55.556`, `cz_robot=68.700` | `SEND_OK`，`convention==1`，`czSign==+1`；输入 `com=(0.3,-0.1,-68.7)` 时输出 `cz=+68.7` 且 `x/y` 不动 |
| 2 | 恰好约定二落在 (0,31.5) | `c_s_z=−55.556`, `cz_robot=68.700` | `SEND_OK`，`convention==2`，`czSign==+1`（钉住"约定二 = 约定一 + 2·c_s_z"） |
| 3 | 两支都落在内 | `c_s_z=1.0`, `cz_robot=15.75` | `SEND_SIGN_AMBIGUOUS`，`convention==0`，`czSign==0` |
| 4 | 两支都不在内 | `c_s_z=+55.556`, `cz_robot=200` | `SEND_SIGN_NONE_IN_RANGE` |
| 5 | `d` 恰为 0 / 31.5 | `cz_robot=55.556` / `87.056` | 两支都 `SEND_SIGN_NONE_IN_RANGE`（开区间） |
| 6 | `m` 在 0.2 / 1.5 边界 | `m=0.2` / `1.5` | `SEND_OK`（闭区间） |
| 7 | `m` = 0.19 / 1.51 | 同上 | `SEND_MASS_OUT_OF_RANGE` |
| 8 | `\|c\|` = 500 / 501 | `com=(500,0,0)` / `(501,0,0)` | 500 `SEND_OK`；501 `SEND_COM_OUT_OF_RANGE` |
| 9 | `c_s` 不可用 | `csZmm=nullptr` | `SEND_NO_CS`，且**显式断言它不等于** `SEND_SIGN_NONE_IN_RANGE` / `SEND_SIGN_AMBIGUOUS` |
| 10 | `cz_robot` 不可用 | `czRobotMm=nullptr` | `SEND_NO_CZ_ROBOT`，且**显式断言它不等于** `SEND_SIGN_NONE_IN_RANGE` |

---

## 3. 构建安全

`tasklist //FI "IMAGENAME eq Touch_Client.exe"` → `INFO: No tasks are running which match the specified criteria.`（构建前后各查一次，全程没有 `taskkill`）。

---

## 4. 验收（简报 §4）—— 原始输出

### 4.1 构建

```
$ tasklist //FI "IMAGENAME eq Touch_Client.exe"
INFO: No tasks are running which match the specified criteria.

$ cmd.exe //c "D:\\Projects\\Touch\\Touch_Client\\build.bat"
...
[2/2] Copying DLLs...
  DLLs copied.

[3/3] Copying models...
  Models copied.

Build complete. Run: D:\Projects\Touch\Touch_Client\x64\Release\Touch_Client.exe
```

（带 `error` 过滤再看一遍：只命中 `  Build OK.` 一行，无任何 error。）

### 4.2 全套件 + payload 套件

```
$ cmd.exe //c "D:\\Projects\\Touch\\Touch_Client\\tests\\run_tests.bat"
  Touch-Dobot Unit Tests
=== test_force_pipeline.exe ===
Results: 5 passed, 0 failed
  [OK]
=== test_constraint_force.exe ===
Results: 7 passed, 0 failed
  [OK]
=== test_safety_core.exe ===
Results: 8 passed, 0 failed
  [OK]
=== test_feedback_parser.exe ===
28 passed, 0 failed
  [OK]
=== test_escalation.exe ===
15 passed, 0 failed
  [OK]
=== test_kinematics.exe ===
18 passed, 0 failed
  [OK]
=== test_coord_safety.exe ===
Results: 27 passed, 0 failed
  [OK]
  Build OK
=== test_force_compensation.exe ===
Results: 23 passed, 0 failed
  [OK]
  Build OK
=== test_relay_command_parser.exe ===
11 passed, 0 failed
  [OK]
  Build OK
=== test_force_logger.exe ===
4 passed, 0 failed
  [OK]
  Build OK
=== test_tcp_calibration.exe ===
7 passed, 0 failed
  [OK]
  Build OK
=== test_session_report.exe ===
20 passed, 0 failed
  [OK]
  Tests complete
```

```
$ cmd.exe //c "D:\\Projects\\Touch\\Touch_Client\\tests\\build_payload_calibration_test.bat"
BUILD_EXIT=0

$ cmd.exe //c "D:\\Projects\\Touch\\Touch_Client\\tests\\test_payload_calibration.exe"
  send_gate_refuses_without_cz_robot_and_says_so... PASS

59 passed, 0 failed
```

（`run_tests.bat` 的输出里会夹着几行光秃秃的 `echo   [FAIL]` / `echo   [FAIL: build error]`
—— 那是 `if/else` 两个分支的**字面 echo 语句**被回显，不是测试结果。成因：它 `call` 的
`build_*.bat` 第一行是 `@echo on`（已核：`build_session_report_test.bat:1`），回显状态会从
被调脚本漏回 `run_tests.bat`，于是后面的分支体被逐行回显。**这个形状在我改动之前就是这样**
（同一份文件、同样的调用次序）。每条真正的结果是 `[OK]`，没有一处 `[FAIL]` 是判决。）

### 4.3 候选值量级合理 — **⚠ 这一条在本环境【跑不出来】，见下**

简报要求"**打印出来的** `m` 0.2~1.5 kg、`|c| < 500 mm`"。这条**需要接机械臂**：
候选块在 `BiasCheck::solveAndApply()` 求解成功的出口上，而 `'m'` 模式在 `--no-robot` 下被
直接拒绝（`main.cpp:2466-2469`：「`[BIAS] --no-robot 模式下不可用`」），`'s'` 又要求 ≥4 个
**实机采的**姿态。全程序没有 `--replay` / `--selftest` 之类的离线驱动入口（已 grep 确认）。
所以这一条的"原始输出"我**给不出来**，不拿别的东西冒充。

能给的最接近的证据（**不是**那个打印块，来源是文件与已入库的断言）：
- 候选来源是 `PayloadCalibration::effective()`（未标定 ⇒ `Config` 种子）
  = `m 0.66 kg`、`c = (0.0, 0.0, 80.4) mm`（`Touch_Client/config/Config.h:63-66`）；
- 该回退由已入库的 `test_effective_falls_back_to_seed` 逐位钉住（`tests/test_payload_calibration.cpp:358-366`）；
- 量级：`m = 0.66 ∈ [0.2, 1.5]` ✓，`|c| = √(0² + 0² + 80.4²) = 80.4 ≤ 500` ✓。

**⇒ 这一条必须在 8b（上机）时补跑。** 顺带：8b 上机时若 `payload_calib.json` 存在，候选就是
那份文件里的值 —— 也就是**上一次**标定的值，不是本次 `'s'` 解出来的（原因见 §7.1）。

### 4.4 `'s'` 仍然什么都不发 —— `sendPayloadToRobot` 的调用点只有 `'p'` 一处

```
$ grep -rn "sendPayloadToRobot" --include=*.cpp --include=*.h .
./main.cpp:2007:    // 全程序【唯一】会把负载参数发给机械臂的入口 —— 交付物 1 的 RelayCore::sendPayloadToRobot
./main.cpp:2040:        // 两条命令与逐条回执在 RelayCore::sendPayloadToRobot 里打 (它能分辨哪一条失败)。
./main.cpp:2041:        const bool ok = RelayCore::instance().sendPayloadToRobot(
./main.cpp:2584:    //   · 【不】下发机械臂 —— 活路径里没有任何 robotSendEnable / sendPayloadToRobot 调用;
./relay/RelayCore.cpp:1702:bool RelayCore::sendPayloadToRobot(double massKg, const double comMm[3]) {
./relay/RelayCore.h:62:    bool sendPayloadToRobot(double massKg, const double comMm[3]);
```

六处命中里：1 处定义（`.cpp:1702`）、1 处声明（`.h:62`）、3 处注释、**1 处真调用**
（`main.cpp:2041`，在 `BiasCheck::sendCandidate()` 里，而 `sendCandidate` 只被 `'p'` 那一支调）。
⇒ 唯一的调用路径是 `'p'`。

### 4.5 `LoadSwitch(0)` 还在（未被顺手改掉）

```
$ grep -n "LoadSwitch(0)" relay/RelayCore.cpp
240://  · 【不】碰连接时序里那条 LoadSwitch(0) (init 里那三行"主动关掉灵敏度") —— 那是【独立
534:    robotSendEnable("LoadSwitch(0)");           // 关闭负载自适应

$ git diff --numstat -- Touch_Client/relay/RelayCore.cpp
77      0       Touch_Client/relay/RelayCore.cpp

$ git diff -- Touch_Client/relay/RelayCore.cpp | grep -c "^-[^-]"
0
```

⚠ **行号变了：473 → 534** —— 因为我在它**上面**加了 61 行（`sendPayloadCommands`）。
`git diff` 显示该文件 **77 行新增、0 行删除**，所以那一行是**逐字节未改动**的；
简报里写的 `473` 只是本任务之前的位置。

---

## 5. 交付物 4 —— 原文 → 新文

**文件：** `Touch_Client/main.cpp`（改前 `:2361-2362`，改后 `:2573-2593`）

### 原文

```cpp
    // 's' in BiasCheck mode: 拟合原始 @1304 通道并【全部打印】—— 不写补偿 / 不写 json / 不下发
    // (为什么只打印不应用: 见 BiasCheck::solveAndApply 顶上的说明, 原点未定)
```

### 新文

```cpp
    // 's' in BiasCheck mode: 拟合原始 @1304 通道并【全部打印】。
    //
    // 三件事的【实际样子】—— 逐条对着 solveAndApply 的活路径核过 (写这条注释前的规矩: 每个
    // 前提回到源头核一遍; 本行从前写的是"不写补偿 / 不写 json / 不下发", 而它把三件事混成
    // 了一句, 也把"不写 json"读成了"什么都不落盘"):
    //   · 【不】写本地补偿 —— 活路径里没有任何 setMassCom 调用 (该入口在 Task 6 已随残余模型
    //     一并【删除】, 本项目已无此函数; 对它唯一残留的引用在下面的 #if 0 块里, 拆那层时
    //     才会编不过 —— 见 Docs\superpowers\specs\2026-09-19-remaining-workflow.md §2 的
    //     第 11 步)。
    //   · 【不】写生效配置 —— PayloadCalibration::applyResult / PayloadCalibration::save /
    //     TcpCalibration::setSensorYawDeg 三个调用只出现在 #if 0 块内 (旧模型那一层),
    //     不在活路径上: payload_calib.json 与 force_calib.json 都不会被这条路径改动。
    //   · 【不】下发机械臂 —— 活路径里没有任何 robotSendEnable / sendPayloadToRobot 调用;
    //     下发是另一个键 'p' 的事 (Task 8a), 而且要先过两道闸。
    //   ⚠ 但它【不是】"什么都不写": 本次诊断会落三份文件 —— calib_log.txt 一行 (logCalibAttempt)、
    //     calib_poses.txt 一批姿态原始数据 (logPoseData)、calib_report.md 一整块 (diagFinish)。
    //     这三份是【记录】, 不是生效值 —— 别把"落盘了"读成"应用了"。
    // (为什么活路径只打印不应用: 见 BiasCheck::solveAndApply 顶上的说明, 原点未定)
```

**我是怎么核的（没有照抄简报）：** 从头到尾读了 `solveAndApply()`，并用
`grep -n "applyResult|setMassCom|PayloadCalibration::save|setSensorYawDeg|#if 0|#endif" main.cpp` 定位每处调用的**归属**：

```
1055:    // (那个"写本地补偿本会话值"的入口 setMassCom 已在 Task 6 随残余模型一并删除 ——
1575:#if 0  // ================= 旧模型 (psi 扫描 + 残余量 dm/dp) —— 已停用 =================
1744:        TcpCalibration::setSensorYawDeg(r.sensorYawDeg);
1748:        PayloadCalibration::applyResult(r);
1784:            ForceCompensation::setMassCom(resMass, resCom);
1802:        if (!PayloadCalibration::save(CalibStore::fileFor("payload_calib.json"))) {
1839:#endif  // ============== 旧模型 (psi 扫描 + 残余量 dm/dp) —— 已停用 ==============
```

（上面是**改动前**的行号。）四个调用**全部**落在 `#if 0`(1575) … `#endif`(1839) 之内 ⇒
都不在活路径上。另：`ForceCompensation::setMassCom` 在 Task 6 已被**删除**，全仓库只剩
`#if 0` 块里那一处引用（正是 remaining-workflow §2 第 11 步要处理的悬空引用）。

---

## 6. 自查（fresh eyes）发现并修掉的问题

1. **阈值在提示文字里被抄了第二遍。** 第一版把 `0.2 / 1.5 / 500 / 31.5` 直接写进
   `snprintf` 的格式串里 —— 改判据时那些字就会撒谎（本项目反复栽的"同一个量两处文字"）。
   改为从库里引用：`PayloadCalibration::SEND_GATE_MASS_MIN_KG/MAX_KG/COM_MAX_MM`
   （`force/PayloadCalibration.h:449-451`）与 `SessionReport::PAYLOAD_D_MIN_MM/MAX_MM`
   （`core/SessionReport.h:108-109`，`payloadDValues` 本身也改用它们）。
2. **`c_s` 不可用时闸1 会印出假数字。** 第一版无条件打印两支 `d`，而 `c_s` 没解出来时
   `fit.cS` 是全 0 ⇒ 屏幕上会出现 `d = 68.700` 这种"从没有数据算出来、看着像真数"的东西。
   改为：`decompOk && echoOk` 才打印两支 `d`，否则打
   `【无法判定】: <缺哪个> —— 两支 d 都无从给出 (不是 0, 是【没有】)`。
3. **结论行重复实现。** 第一版在 `solveAndApply` 里写了一遍 switch，`'p'` 的拒绝路径还要再写
   一遍。抽成 `formatSendGateConclusion()`（`main.cpp:1084`），两处同源。
4. **候选作废的两条路径**（简报没写，防的是"发一份来路不明的旧值"）：
   `reset()`（重开采集 ⇒ 那批数据已丢弃）与求解失败出口。`'p'` 因此永远不会发出与"当前这批
   数据"无关的候选。
5. **一个自己引入的编译错误**（`static void reset()` 被复制成两行）在第一次构建时就被
   编译器挡下并当场修掉 —— 记在这里，因为它是"改前先读、改后必构建"这条规矩生效的证据。
6. **交付物 4 里我原本引用了 `main.cpp:1748`**，但那行号在本次改动后已漂到 `1911`；改成不写行号
   （并注明"那份计划里写的 1748 已经对不上了"）。

---

## 7. ★ 我实际核过、与简报说法不一致的地方

### 7.1 ★★ 简报 §2 交付物 2 的前提**是错的**：`applyResult` 不在活路径上

简报写道：

> 取 `PayloadCalibration::effective(m, c)`（求解成功后它已被 `applyResult` 更新为**本次解出的
> 绝对值**，见 `main.cpp:1748`）

**实际：`main.cpp:1748` 在 `#if 0` 块（1575-1839）里面**（见 §5 的定位输出）。
`PayloadCalibration::applyResult` 在整个 `main.cpp` 里**只有这一处调用**，它不被编译。

⇒ **后果（这条会直接影响 8b）：** 按 `'s'` 求解成功后，`effective()` **不会**变成本次解出的值；
它仍然是 `payload_calib.json` 里的值（或 `Config` 种子）。而**活路径从不写 `payload_calib.json`**
（`PayloadCalibration::save` 同样只在 `#if 0` 里）。所以在本机当前状态（`Touch_Client/calib/`
只有 `force_calib.json.expired`，无 `payload_calib.json`）：

> **8b 按 `'p'` 发出去的，会是 `Config` 的种子值 `(0.66 kg, 0, 0, 80.4 mm)` —— 与连接时序
> 已经在发的**是同一份**。也就是说 `'p'` 相对连接时序真正新增的效果只有 `LoadSwitch(1)`
> 这一条，`EnableRobot` 那一份参数并没有换成"本次标定解出来的"。**

我**没有**去"修"这件事，因为：

- 简报明令候选来源就是 `effective()`（这一条本身是自洽的：`'p'` 要发的正是"当前应当下发的
  那一份"，而全项目只有 `effective()` 这一条通道会把负载交给机械臂）；
- 把 `applyResult` 接回活路径 = 重新启用被明文停用的写入路径，而活路径上写着
  「**别好心把它们接回来**」（`main.cpp:1525` 一带）与「下发路径的开通条件在 plan Task 9」；
- 而且活路径**没有**把本次 `fit` 折算成法兰系绝对负载的代码（旧 `solve()`/`Result` 那一层才
  有，`RawFit` 只有传感器测量系的 `cS`）—— 这属于"简报没有预料到的重构"。

**⇒ 需要决策（我没有权限替它决定）：** 8a/8b 的意图是否真的是"把本次标定结果发给机械臂"？
若是，缺的是一段**把 `RawFit` 折算成法兰系 `(m, c)` 并落盘**的代码 —— 那正是简报说"归 Task 9"
的那一步。建议在 8b 之前明确：**要么**接受"`'p'` 目前只是把当前生效值重发一遍 + 打开
`LoadSwitch`"（那 8b 测的是"打开负载设置之后闸门放不放行"），**要么**先补 Task 9 的折算。

### 7.2 简报 §2 交付物 4 的前提**也是错的**

简报写道：`ForceCompensation::setMassCom`(`main.cpp:1798`) 与 `PayloadCalibration::save`
(`main.cpp:1802`) 在 `solveAndApply()` 里"确实"被调了，所以那条注释"已经失真"。

**实际：那两行都在 `#if 0` 里**（§5 有原始定位输出），而且 `setMassCom` 这个函数**已经被删除**
（Task 6），全仓库只剩 `#if 0` 块里的悬空引用。以简报给的 `1798` / `1802` 为准去核，
`1798` 甚至不是 `setMassCom` 那一行（真正的调用在被删除的行号 1784 一带）。

⇒ 那条注释的**三个断言在活路径上恰好都是对的**。我按简报的**指令**（"照着代码把这三件事各
写成它真实的样子，不许照抄简报"）改写成了逐条、可核、且补上"它其实**会**落三份记录文件"
这一句 —— 因为 `不写 json` 很容易被读成"这条路径不落任何盘"。

### 7.3 简报的出处引用只覆盖了一半

简报 §1 写「`Docs/机械臂资料/TCP_IP远程控制接口文档.md:256-261` 与设计 §6b」—— 但
`:256-261` 是 **`LoadSwitch`** 那一节。`EnableRobot` 在 **`:116-125`**（原型 `:117`，
`centerX/Y/Z` 单位 mm、范围 ±500 在 `:121-123`）。设计 §6b 在
`Docs/superpowers/specs/2026-09-19-raw-channel-calibration-design.md:131-147`
（两条命令的表在 `:135-139`，三条的清单在 `:144-146`）。代码注释里已按核过的位置写。

### 7.4 闸2 的 `|c|` 边界：简报的用例表与操作单原文有**一个点**的分歧

- `on-machine-checklist.md:155`（§6 闸2）写的是「`|c| < 500 mm`」（不含）；
- 简报 §3 用例 8 写的是「`|c|` = 500 边界 → **含 500 放行**；501 不放行」（含）。

**我按简报的用例表实现（含）**，并在 `force/PayloadCalibration.h:449-451` 的注释里把这个口径
差异照实记下来，没有静默二选一。⇒ 若以操作单为准，需要把 `<=` 改成 `<`（一行）。

### 7.5 简报 §4.5 的行号已经过期

`LoadSwitch(0)` 现在在 `RelayCore.cpp:534`（不是 473）—— 因为本任务在它上面加了 61 行。
`git diff --numstat` = `77 0`，即**该行逐字节未变**。简报的 473 是改动前的位置。

### 7.6 简报 §4.3 在本环境不可能跑出来（不是我没跑）

见 §4.3 的说明：`--no-robot` 下 `'m'` 被拒、`'s'` 要 ≥4 个实机姿态、全程序没有离线驱动入口。

---

## 8. 文件清单

**改动（`git diff --numstat` 实测；除交付物 4 那两行原注释外，无一行删除）：**

| 文件 | 增 / 删 |
|---|---|
| `Touch_Client/relay/RelayCore.h` | +8 / -0 |
| `Touch_Client/relay/RelayCore.cpp` | +82 / -0 |
| `Touch_Client/force/PayloadCalibration.h` | +62 / -0 |
| `Touch_Client/force/PayloadCalibration.cpp` | +60 / -0 |
| `Touch_Client/core/SessionReport.h` | +37 / -4（把 `payloadDSection` 的两支算术抽成共用的 `payloadDValues`；**打印字符串一字未改**） |
| `Touch_Client/main.cpp` | +237 / -2（候选块 + `'p'` 键 + 交付物 4 的注释改写；那 -2 就是被替换的两行原注释） |
| `Touch_Client/tests/test_payload_calibration.cpp` | +187 / -0（10 条新用例 + 1 个小工具函数） |

**没有动**（简报硬约束）：门限公式 / 求解器 / 模型形式 / `A` 用 `A_F` / 两份日志的列格式 /
`#if 0` 块 / 证据文件 / `RelayCore.cpp:473`(现 534) 的 `LoadSwitch(0)` / `tests/run_tests.bat`。

**`core/SessionReport.h` 的那次抽取值得单独说明**（它是本次唯一一处"改动既有代码"）：
简报 §2 交付物 2 要求"**复用** `main.cpp:1038-1058` 已有的两支计算，**不要另写一份**"。
那两支计算的实体在 `core/SessionReport.h:95-127`（`main.cpp:1038-1058` 只是它的调用点）。
所以我把 `dSameDir/dFlipDir/sameIn/flipIn` 抽成 `payloadDValues()`，**两个调用方共用**；
`payloadDSection` 的打印字节一个都没变（`test_session_report` 20/20 通过，含 5 条逐字断言
`d` 那一节的用例）。

---

## 9. 未决 / 需要人决定的事

1. **§7.1（最重要）**：`'p'` 目前发的是 `effective()`（= 上次落盘值或种子），**不是本次标定
   解出来的** —— 因为把本次结果折算法兰系并落盘的那段（Task 9）还没写。8b 之前必须定：
   接受现状，还是先补 Task 9。
2. **§7.4**：`|c| = 500` 含还是不含。
3. **8b 上机时补跑 §4.3**（打印块本身没在本环境跑过）。
