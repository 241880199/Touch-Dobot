# Task 8a-2 报告 — 接上"标定结果 → 可下发候选"那条断链 + 8a 复审遗留

**日期:** 2026-09-20 · **分支:** `feat/pen-clamp-redesign` · **基线 HEAD:** `8a6aae0`
**提交:** `37327ec` (单个提交; 为什么不是两个见 §8)
**简报:** `.superpowers/sdd/task-8a2-brief.md` · **8a 报告:** `.superpowers/sdd/task-8a-report.md` (未覆盖)
**状态:** DONE_WITH_CONCERNS —— 交付物与 §5 验收全部落地、构建与全套件绿;
但**简报 §3.5 的那个核心前提是错的** (见 §6.1), 而它正是那一条要求的立论基础。

---

## 1. 落地清单 (每处改动的 `文件:行`)

### 1.1 Task 8a-2 —— 候选只来自本次实测 (§3.1 / §3.2)

| 位置 | 内容 |
|---|---|
| `force/PayloadCalibration.h:468` | `SendGateVerdict` 新增 `SEND_NOT_MEASURED` |
| `force/PayloadCalibration.h:481-485` | `enum SendMassSource { MASS_SOURCE_SEED, MASS_SOURCE_MEASURED }` —— 来源参数, **无默认值** |
| `force/PayloadCalibration.h:504-506` | `evaluateSendGate(..., SendMassSource massSource)` |
| `force/PayloadCalibration.h:528-544` | `SendCandidateAbsent` / `SendCandidate` |
| `force/PayloadCalibration.h:546-548` | `buildSendCandidate(measuredMassKg, csZmm, echoCenterMm)` |
| `force/PayloadCalibration.h:557-571` | `SendCandidateDiff` / `diffSendCandidate` |
| `force/PayloadCalibration.h:579/583` | `formatSendCandidateMassText` / `formatSendCandidateDiffConclusion` |
| `force/PayloadCalibration.cpp:1903-1912` | 来源判据实现 (排在量级与符号**之前**) |
| `force/PayloadCalibration.cpp:1956-1990` | `buildSendCandidate` |
| `force/PayloadCalibration.cpp:1992-2012` | `diffSendCandidate` |
| `force/PayloadCalibration.cpp:2014-2038` | 两个措辞函数 |
| `main.cpp:1665-1667` | 候选块改为 `buildSendCandidate(decompOk ? &d.m : nullptr, ..., echoOk ? echoCenter : nullptr)` —— **`effective()` 取数分支整段删掉, 不留回退** |
| `main.cpp:1670-1680` | `!cand.present` ⇒ 打印"没有候选"的**具体归因**并作废候选 |
| `main.cpp:1692-1693` | m 那一行由 `formatSendCandidateMassText` 产出 (措辞不在此处重写) |
| `main.cpp:1698-1700` | 候选的 c 打的是**闸1 定过号**的那一份 (= 真会发出去的那一份) |
| `main.cpp:1097-1104` | `formatSendGateConclusion` 新增 `SEND_NOT_MEASURED` 分支 |

**候选块删掉的 18 行** (`effective()` + 其"来源标签"注释 + `candM`/`candC`): 简报 §3.1 要求
"删掉 `effective()` 那个取数分支, 不要留回退"。删干净了 —— `git grep` 确认候选块里已无
`effective(` / `enabled` 的引用。

### 1.2 §3.5 —— 与机械臂当前值的逐分量比对

| 位置 | 内容 |
|---|---|
| `main.cpp:1753-1790` | 逐分量打 `发出值 − @1176 当前值` (m / cx / cy / cz 四条), **在按 'p' 之前** |
| `force/PayloadCalibration.cpp:1992-2012` | 差值与两个标志的**纯函数**实现 (可测) |
| `force/PayloadCalibration.cpp:2024-2038` | 结论行: `c 未变 —— 本次【只改 m】` / `c 已变` / 翻号 ⇒ `⚠【高危】` 且**不含**"只改 m" |

### 1.3 复审遗留

| # | 位置 | 改动 |
|---|---|---|
| **Important 1** | `force/PayloadCalibration.cpp:1901` | `<=` → **`<`** (严格小于 500) |
| | `force/PayloadCalibration.h:441-447` | 口径说明重写: 引操作单原文 + 用户 2026-09-20 裁定 |
| | `tests/test_payload_calibration.cpp:3456-3479` | 用例改名 `..._is_exclusive_at_500`: 500 **拒**、499.999 **放**、501 拒 |
| | `main.cpp:1136` | 结论行措辞 "超过" → **"不小于"** (恰好 500 时 "超过" 会说谎) |
| | `main.cpp:1744` | 打印的判据 `\|c\| <= 500` → **`\|c\| < 500`** |
| **Important 2** | `main.cpp:2680-2690` | `setSensorYawDeg` 那句断言**收到本条路径**, 并单列它在启动序列里的活调用 |
| **Minor 3** | `main.cpp:2711-2723` | `'p'` 不在 `'m'` 模式时**出声拒绝** (新增整个 `!BiasCheck::mode` 分支) |
| **Minor 4** | `tests/test_payload_calibration.cpp:3542-3565` | 新用例走到 `czSign == −1` (简报给的那组数) |
| **Minor 5** | `force/PayloadCalibration.cpp:1936-1945` | 去掉"必然", 补上前提 `\|选中的 c_s_z\| > 31.5` 与推导 |
| **Minor 9** | `main.cpp:1054-1058` | 写死的 `0 < d < 31.5` 改用 `SessionReport::PAYLOAD_D_MIN_MM/MAX_MM` |

**未修 (按指令只记录):** Minor 6 (`RelayCore.cpp` 里 `robotSendEnable` 的返回值被丢弃)、
Minor 8 (我 8a 报告里的过期行号)、Minor 10 (`main.cpp:1624-1627` 与 `:997-1001` 重复的回读)。

---

## 2. TDD 证据

### RED

先写测试 (断言 `MASS_SOURCE_*` / `SEND_NOT_MEASURED` / `SendCandidate` / `buildSendCandidate` /
`diffSendCandidate` / 两个 `formatSendCandidate*`), 此时实现还不存在:

```
$ cmd.exe //c "D:\Projects\Touch\Touch_Client\tests\build_payload_calibration_test.bat"
test_payload_calibration.cpp(3349): error C2039: "MASS_SOURCE_MEASURED": 不是 "PayloadCalibration" 的成员
test_payload_calibration.cpp(3349): error C2065: “MASS_SOURCE_MEASURED”: 未声明的标识符
test_payload_calibration.cpp(3528): error C2039: "MASS_SOURCE_SEED": 不是 "PayloadCalibration" 的成员
test_payload_calibration.cpp(3529): error C2039: "SEND_NOT_MEASURED": 不是 "PayloadCalibration" 的成员
test_payload_calibration.cpp(3594): error C2039: "SendCandidate": 不是 "PayloadCalibration" 的成员
test_payload_calibration.cpp(3594): error C2146: 语法错误: 缺少“;”(在标识符“c”的前面)
... (每个新符号/每处调用各报一遍, 共 13 处调用点 + 全部新符号)
```

**这是预期的红**: 编译期就红 ⇒ 证明测试真的在调那两个还不存在的接口, 而不是"写了个恒真的
断言、编译过了"。`test_send_gate_com_magnitude_bound_is_exclusive_at_500` 那一条此时也编不过
(它用的是带来源参数的新签名), 所以边界判据的改动**确实**先落在测试上。

### GREEN

```
$ cmd.exe //c "D:\Projects\Touch\Touch_Client\tests\build_payload_calibration_test.bat"
BUILD_EXIT=0
```

---

## 3. §5 验收 —— 原始输出

### 3.1 `build.bat` → `Build OK.`

```
$ tasklist //FI "IMAGENAME eq Touch_Client.exe"
INFO: No tasks are running which match the specified criteria.

$ cmd.exe //c "D:\Projects\Touch\Touch_Client\build.bat"
  Build OK.
Build complete. Run: D:\Projects\Touch\Touch_Client\x64\Release\Touch_Client.exe
```

(构建前后各查一次 `tasklist`, 全程**没有** `taskkill`。带 `warning` 过滤再看一遍: 只有一条
`MSB8004: Output 目录未以斜杠结尾` —— 那是 `.vcxproj` 里原有的工程级警告, 不是本次代码产生的;
编译器的 `C4xxx` 警告 **0 条**。)

### 3.2 `run_tests.bat` 全绿 + payload 套件 `N passed, 0 failed`

```
$ cmd.exe //c "D:\Projects\Touch\Touch_Client\tests\run_tests.bat"
=== test_force_pipeline.exe ===
=== ForcePipeline Unit Tests ===
Results: 5 passed, 0 failed
  [OK]
=== test_constraint_force.exe ===
=== ConstraintForce Unit Tests ===
Results: 7 passed, 0 failed
  [OK]
=== test_safety_core.exe ===
=== Safety Core Unit Tests ===
Results: 8 passed, 0 failed
  [OK]
=== test_feedback_parser.exe ===
=== FeedbackParser Tests ===
28 passed, 0 failed
  [OK]
=== test_escalation.exe ===
=== EscalationTracker Tests ===
15 passed, 0 failed
  [OK]
=== test_kinematics.exe ===
=== Kinematics Tests ===
18 passed, 0 failed
  [OK]
=== test_coord_safety.exe ===
=== CoordinateTransform + SafetyBoundary Tests ===
Results: 27 passed, 0 failed
  [OK]
=== test_force_compensation.exe ===
=== ForceCompensation + Calibration Tests ===
Results: 23 passed, 0 failed
  [OK]
=== test_relay_command_parser.exe ===
=== RelayCommandParser Tests ===
11 passed, 0 failed
  [OK]
=== test_force_logger.exe ===
=== ForceLogger Tests ===
4 passed, 0 failed
  [OK]
=== test_tcp_calibration.exe ===
=== TcpCalibration Tests ===
7 passed, 0 failed
  [OK]
=== test_session_report.exe ===
=== SessionReport Tests ===
20 passed, 0 failed
  [OK]

$ cmd.exe //c "...\run_tests.bat" | grep -c "^  \[OK\]$"
12
$ cmd.exe //c "...\run_tests.bat" | grep -c "^  \[FAIL\]"
0
```

```
$ cmd.exe //c "D:\Projects\Touch\Touch_Client\tests\build_payload_calibration_test.bat"
BUILD_EXIT=0

$ cmd.exe //c "D:\Projects\Touch\Touch_Client\tests\test_payload_calibration.exe"
--- Task 8a: pre-send gates (sign convention + magnitude) ---
  send_gate_convention_one_wins_and_signs_cz... PASS
  send_gate_convention_two_wins_and_signs_cz... PASS
  send_gate_two_conventions_in_range_is_ambiguous... PASS
  send_gate_no_convention_in_range_is_refused... PASS
  send_gate_interval_is_open_at_both_ends... PASS
  send_gate_mass_bounds_are_inclusive... PASS
  send_gate_mass_outside_bounds_is_refused... PASS
  send_gate_com_magnitude_bound_is_exclusive_at_500... PASS
  send_gate_refuses_without_cs_and_says_so... PASS
  send_gate_refuses_without_cz_robot_and_says_so... PASS
--- Task 8a-2: candidate construction + provenance + wording ---
  send_gate_refuses_unmeasured_mass_and_says_so... PASS
  send_gate_convention_two_wins_with_negative_cz_robot... PASS
  send_gate_small_positive_cz_robot_flips_the_sign... PASS
  build_send_candidate_normal_path... PASS
  build_send_candidate_without_measured_mass_has_none... PASS
  build_send_candidate_without_echo_has_none... PASS
  send_candidate_mass_label_matches_brief_wording... PASS
  send_candidate_diff_reports_unchanged_c_on_the_real_machine_case... PASS
  send_candidate_diff_flags_flipped_cz_as_high_risk... PASS

68 passed, 0 failed
```

**基线对照:** 改动前 `59 passed, 0 failed` ⇒ 新增 **9** 条, 改判据 **1** 条
(`..._is_inclusive_at_500` → `..._is_exclusive_at_500`, 按 Important 1 的裁定),
其余 9 条闸用例的**判据与数字一个都没动** (只加了 `MASS_SOURCE_MEASURED` 这个新实参)。

### 3.3 `grep -n "sendPayloadToRobot" main.cpp` ⇒ 只有 `'p'` 那一处调用

```
$ grep -n "sendPayloadToRobot" main.cpp
2096:    // 全程序【唯一】会把负载参数发给机械臂的入口 —— 交付物 1 的 RelayCore::sendPayloadToRobot
2129:        // 两条命令与逐条回执在 RelayCore::sendPayloadToRobot 里打 (它能分辨哪一条失败)。
2130:        const bool ok = RelayCore::instance().sendPayloadToRobot(
2679:    //   · 【不】下发机械臂 —— 活路径里没有任何 robotSendEnable / sendPayloadToRobot 调用;
```

4 处命中: 3 处**注释**、**1 处真调用** (`main.cpp:2130`, 在 `BiasCheck::sendCandidate()` 里,
而 `sendCandidate` 只被 `'p'` 那一支调 —— `main.cpp:2725`)。⇒ `'s'` 仍然什么都不发。

### 3.4 `grep -n "payload_calib.json" main.cpp relay/RelayCore.cpp` ⇒ 新增代码里没有任何写入

```
$ grep -n "payload_calib.json" main.cpp relay/RelayCore.cpp
main.cpp:1070:    // payload_calib.json → (下次启动的连接时序里) 下发机械臂。现在【一个字都不写】。
main.cpp:1599:        diagOut() << "  ★ 本次【什么也没应用】: 不写本地补偿、不写 payload_calib.json /"
main.cpp:1626:            // payload_calib.json 的旧值 (= 机械臂【现在就有】的那一份, 发出等于没改) 或
main.cpp:1935:        // 【上面这两个"绝对"值不再只是记录/显示】: comMm 随 payload_calib.json 持久化, 并在下次
main.cpp:1968:            std::cout << "  判定: ✗ 不合理 — 已【拒绝保存】: payload_calib.json 与本地补偿均未改动 (第 "
main.cpp:2014:        // 实机证实它生效 —— 改 payload_calib.json 后重启, 机械臂快速撞向关节限位。所以重标定
main.cpp:2054:        if (!PayloadCalibration::save(CalibStore::fileFor("payload_calib.json"))) {
main.cpp:2055:            std::cerr << "[BIAS] !! payload_calib.json 写入失败" << std::endl;
main.cpp:2058:            std::cout << "  已保存 payload_calib.json (下次启动自动加载)" << std::endl;
main.cpp:2061:        // 猛地动起来 (改 payload_calib.json 后重启, 连接时序下发 EnableRobot(1.5,...), 机械臂
main.cpp:2672:    //     #if 0 块, 旧模型那一层), 所以 payload_calib.json 与 force_calib.json 都不会被这条
main.cpp:3052:        if (PayloadCalibration::load(CalibStore::fileFor("payload_calib.json"))) {
main.cpp:3057:            // 仍然装上: 它是 payload_calib.json 的一部分, 撤掉会改变那条路径读到的状态。
main.cpp:3059:            std::cout << "[Payload] Loaded payload_calib.json (mass=" << PayloadCalibration::massKg
main.cpp:3068:            std::cout << "[Payload] 无可用 payload_calib.json — 用种子值 mass=" << m
relay/RelayCore.cpp:169:// 负载值优先取实机标定结果 (PayloadCalibration / payload_calib.json),
```

**全文件唯一的写入是 `main.cpp:2054` 的 `PayloadCalibration::save(...)` —— 它在 `#if 0` 块内**
(块边界实测 `main.cpp:1827` `#if 0` … `:2091` `#endif`, 见下), 本次改动**没有新增任何写入**;
`RelayCore.cpp:169` 只是一句注释。**候选只在内存** ⇒ 下次重启的连接时序不会自动下发新负载。

```
$ awk 'NR>=1827 && NR<=2091 && (/^#if 0/ || /^#endif/ || /PayloadCalibration::save/)' main.cpp
#if 0  // ================= 旧模型 (psi 扫描 + 残余量 dm/dp) —— 已停用 =================
        if (!PayloadCalibration::save(CalibStore::fileFor("payload_calib.json"))) {
#endif  // ============== 旧模型 (psi 扫描 + 残余量 dm/dp) —— 已停用 ==============
```

### 3.5 §5.5 核查项 —— `Decomp::m` 是"测量原点以下"还是"整条链"

**结论: 简报 §2 的说法【正确】—— `Decomp::m` 是【传感器测量原点以下】那一段的质量,
不是法兰上的整条链。证据与出处:**

1. **`Decomp::m` 的算法在 `.cpp`**: `force/PayloadCalibration.cpp:1492`
   `const double m = pow(sv[0] * sv[1] * sv[2], 1.0 / 3.0);` —— 它只是 `A` 的奇异值几何平均,
   本身**不带帧信息**。所以帧由**"A 是从哪个通道拟合出来的"**决定 ⇒ 下面这两条才是判据。
2. **活路径拟合的是 `@1304` (原始六维力通道)**: `main.cpp:1341`
   `diagOut() << "  原始通道 (@1304 SixForceValue) 线性解 — " << count << " 个姿态"` ——
   那一屏的 `d` (含 `d.m`, 打印在 `main.cpp:1358`) 就是这次 `@1304` 拟合的分解结果。
3. **`@1304` 的绝对值 ≠ 机械臂负载模型要的那个量** (逐字出处 `progress.md:728-730`):
   ```
   728: **因此 `@1304` 的绝对值 ≠ 机械臂负载模型要的那个量**:
   729: - `payload_calib.json` / `EnableRobot` 要的是**整条链**（含传感器, 0.657 / 80.4）
   730: - `@1304` 测的是**传感器以下**（~0.42 / cz≈130）
   ```
   同一条结论在代码里也早已写着: `main.cpp:1575-1577`
   「★ 以上全部是【传感器测量原点以下】的量 —— 不是整条工具链。m / A / c_s 描述的是
   传感器【测量原点向下】那一段负载」。
4. **量级自洽**: `progress.md:725` 实测 `@1304` 尺度 **418.6 g**, 而四次采集解出的 `m` 是
   `0.4177~0.4224 kg` (本报告 §6.4 的夹具重放也复现 `m = 0.422357 kg`) ⇒ 与"传感器以下
   ~0.42"吻合, 与"整条链 0.657"不符。

⇒ **证据与简报 §2 一致, 没有冲突, 因此没有停下来。** 本报告 §1.1 的打印措辞与
`formatSendCandidateMassText` 的用语逐字照 §2 的要求写
(标签 = "本次实测的质量尺度 · 传感器测量原点以下"; 紧接着写明与 EnableRobot 要的
【法兰上整条链】差了传感器机器人侧那一段、【量未定】; 禁止出现"标定结果 / 绝对负载 /
整条链质量" —— 这三条由 `test_send_candidate_mass_label_matches_brief_wording` 钉住)。

---

## 4. §3.5 符号翻转 —— 处理与**一个被推翻的前提**

### 4.1 做了什么 (§3.5 三条要求)

1. **逐分量打印** `发出值 − @1176 当前值` (`main.cpp:1759-1770`), 外加一句结论行。
2. **翻号 ⇒ 标高危** 且**不许**再打"只改 m": `formatSendCandidateDiffConclusion`
   (`force/PayloadCalibration.cpp:2024-2038`), 由
   `test_send_candidate_diff_flags_flipped_cz_as_high_risk` 断言
   (`find("高危") != npos` **且** `find("只改 m") == npos`)。
3. **位置**: 这一段在候选块里、`'p'` 之前 —— 按 's' 那一屏就能看到, 不用先按 'p'。

### 4.2 ⚠ 简报 §3.5 的前提**是错的**: `68.7 → −68.7` 这个翻转**不可能发生**

简报 §3.5 写:

> 当本次解出的 `c_s_z` 与 `cz_robot` **异号**时, 胜出的会是**约定二**, `cz` 被翻号
> (`68.7 → −68.7`, 量级上是一次**巨大的**负载改动, 不是"c 不动")。

**按代码算一遍**(`force/PayloadCalibration.cpp:1946-1948`):

```
czSign = sign(选中的 c_s_z)           // 选中的 = 约定一取 C, 约定二取 −C
候选 cz = czSign · |cz_robot|
```

而胜出那支的 `d` 满足 (两支都推得出同一个式子): **选中的 `c_s_z` = `cz_robot` − `d`**,
其中 `d ∈ (0, 31.5)`。⇒ `czSign = sign(Z − d)`, 于是

**候选 `cz` = `sign(Z − d) · |Z|`, 其中 `Z = cz_robot`、`d ∈ (0, 31.5)`。**

| `Z` 的范围 | 结论 |
|---|---|
| `Z > 31.5` | `Z − d > 0` 恒成立 ⇒ `czSign = +1` ⇒ 候选 `cz = +Z` **值不变** |
| `Z < 0` | `Z − d < 0` 恒成立 ⇒ `czSign = −1` ⇒ 候选 `cz = −|Z| = Z` **值不变** |
| **`0 < Z < 31.5`** | `d > Z` 时 `czSign = −1` ⇒ 候选 `cz = −Z` **← 只有这一格会翻号** |

**本机 `cz_robot = 68.700 > 31.5` ⇒ 无论本次解出的 `c_s_z` 是什么, 候选的 `cz` 都是 `+68.7`,
一次都翻不了。** ⇒ §3.5 举的那个例子 (`68.7 → −68.7`) 在这一配置下**不可达**;
"这是一次巨大的负载改动"这个警告在本机上**不会触发**。

**这个反证不是我推的, 是仓库里 8a 已经入库的用例**:
`test_send_gate_convention_two_wins_and_signs_cz` (`tests/test_payload_calibration.cpp:3365`)
用的正是简报 §3.5 描述的那种"异号"配置 (`c_s_z = −55.556`, `cz_robot = +68.700`),
而它断言的是:

```cpp
CHECK(g.convention == 2);
CHECK(g.czSign == +1.0);                  // ← 不是 −1
CHECK(nearRefAbs(g.comMm[2], +68.7));     // ← 没有被翻成 −68.7
```

**⇒ 简报 §3.5 的那句话与 8a 自己的用例直接矛盾。** 读法应该是:
"`c_s_z` 与 `cz_robot` 异号"只会让**约定二胜出**, 但胜出后取的是 `−c_s_z`, 其号恰好回到
`cz_robot` 的号上 —— 所以**符号被"定"了, 值没被"改"**。

### 4.3 连带: 复审 Minor 4 的措辞也需要修正 (但要求已满足)

Minor 4 写"`czSign == -1` 是 the only sign-flipping code path"。按 §4.2 的表格, 这两件事
**不是一回事**: `czSign == -1` 在 `|Z| > 31.5` 时**不改任何值**。我按 Minor 4 的原话做了它
要的用例 (简报给的 `c_s_z = +55.556` / `cz_robot = −40` ⇒ 约定二胜出、选中 `−55.556`、
`czSign == −1`、候选 cz 为负), **并且另加了一条真正会翻号的用例** (`cz_robot = +10`,
`c_s_z = −10` ⇒ 候选 cz 由 `+10` 翻成 `−10`) —— 因为 §3.5 的高危分支要的正是后者。

---

## 5. 自查 (fresh eyes) 发现并修掉的问题

1. **`'p'` 出声的提示原本会给出一个错的去路。** 第一版写"按 'm' 回去再按 'p'", 但重进
   `'m'` 走的是 `reset()` (`main.cpp:2495`), 而 `reset()` 会作废候选 (`main.cpp:189`)
   ⇒ 照那句话做的人会发现候选没了。改成照实说:"窗口就是这一次采集里, `'s'` 之后、
   退出模式之前"。
2. **打印的候选与发出的候选原本差一个号** (复审 Minor 7 的实体)。8a 的打印打的是**闸前**的
   `candC`; 现在打的是 `cand.comMm` (= `gate.comMm`, 闸1 定过号的那一份), 与
   `sendCandidate()` 送进 `sendPayloadToRobot` 的是同一个数组。
3. **闸1 那一屏的免责声明原本指错了地方。** 我第一版写"本次的 cz 就是**上面那个** cz_robot",
   而 `cz_robot` 的值其实打在**下面**那一段 (`与机械臂当前值相比`)。已改成显式指路。
4. **`|c| < 500` 改了之后, 结论行的"超过 500 mm"会说谎。** 被拒侧现在是 `>= 500`
   (恰好 500 也拒), 所以改成"**不小于** 500 mm"。
5. **`evaluateSendGate` 的"哪些数字填了"这个隐性契约变了** (来源判据插在最前面 ⇒
   `SEND_NOT_MEASURED` 时两支 `d` 是结构体初值 0)。头文件里补了完整次序表, 并明说
   「0 在这里是**没有算过**, 不是 `d = 0`」; `main.cpp` 的打印条件也随之加了
   `verdict != SEND_NOT_MEASURED` 一档 —— 不依赖"它不可达"这个证明活着。
6. **两个措辞函数里的引号/转义。** `formatSendCandidateMassText` 里用了 `\"` 转义的引号;
   而在 `main.cpp` 里我第一版把 ASCII 双引号直接写进了中文串, 会当场编不过 —— 被编译器挡住,
   已改成 `【】`。
7. **一处过度设计被自己拆掉**: 起初想给 `diffSendCandidate` 只传 diff 结构体, 但结论行要打
   "由 A 翻为 B" 的两个数, 而 `echo + (cand − echo)` 在浮点下**未必**逐位回到 `cand`。
   改成结构体里同时存 `candCenterMm` / `echoCenterMm` —— 打印的两个数就是参与比较的那两个数。

---

## 6. ★ 我实际核过、与简报/复审说法不一致的地方

### 6.1 ★★ 简报 §3.5 的前提是错的 (最重要, 详见 §4.2)

"`c_s_z` 与 `cz_robot` 异号 ⇒ cz 被翻号 (`68.7 → −68.7`)" —— **本机 `cz_robot = 68.700` 时
不可能**。翻号的充要条件是 **`0 < cz_robot < 31.5`** 且胜出的 `d > cz_robot`。
推导 + 8a 自己那条用例的证伪都在 §4.2。

**后果 (好消息, 但必须说清):** 用户选 `cz = cz_robot` 想要的"最小改动 (只改 m, c 不动)"
**在本机是成立的** —— 但**理由不是**简报给的那个 ("cz 取自 cz_robot 所以不动"), 而是
"`|cz_robot| = 68.7 > 31.5` ⇒ `sign(Z − d)` 必然等于 `sign(Z)` ⇒ 号的定法恰好还原成原值"。
两者在 `cz_robot` 落到 `(0, 31.5)` 时会**分道扬镳** —— 那时"最小改动"就不成立了。
所以 §3.5 要求的高危分支我**照做了**(它在别的 `cz_robot` 下是真会触发的), 只是本机不会走到。

### 6.2 复审 Minor 3 的行号引用是错的 (主张是对的)

复审写「`main.cpp:962-967`: re-pressing `'m'` exits without `reset()`」。实测**改动前**
`main.cpp:962-967` 是 `diagFinish()` (把块追加到 `calib_report.md` 的那个函数),
与 `'m'` 模式毫无关系。真正的分支在**改动前** `main.cpp:2521-2526`:

```cpp
        } else {
            BiasCheck::mode = false;
            RelayCore::setDragMode(false);
            BiasCheck::report();       // 数据属旧负载时会自行拒绝判定
        }
```

**主张本身经核成立**: `reset()` 只在**进入**模式那一支被调 (`main.cpp:2495`),
退出这一支不调; 全文件只有 4 处写 `s_sendCandidateValid` (声明 `:174`、`reset()` `:189`、
求解失败 `:1616`、无候选 `:1670`、成功 `:1681`) ⇒ **候选确实能活过模式退出**, Minor 3 描述的
静默 no-op 是真的。我按"出声拒绝"那一侧修 (没有撤掉模式闸 —— 理由见 §7)。

### 6.3 复审 Minor 5 的行号引用是错的 (内容是对的)

复审写 `PayloadCalibration.cpp:1906-1908`。改动前那段注释实际在 **`:1922-1924`**
(改动后 `:1936-1945`, 因为我在这段前面插了来源判据)。内容与复审转述的一致, 已按其要求
补上 `|选中的 c_s_z| > 31.5` 这个前提并去掉"必然"。

### 6.4 简报 §0 的行号引用: `#if 0` 那三条全对, "`1263`/`1272`" 那条不对

| 简报的引用 | 实测 (改动前 `8a6aae0` 的 main.cpp) | 判定 |
|---|---|---|
| `#if 0` 块 `:1739`–`:2003` | `1739:#if 0` … `2003:#endif` | ✅ 逐字吻合 |
| `applyResult` `:1912` | `1912: PayloadCalibration::applyResult(r);` | ✅ |
| `setMassCom` `:1948` | `1948: ForceCompensation::setMassCom(resMass, resCom);` | ✅ |
| `save` `:1966` | `1966: if (!PayloadCalibration::save(CalibStore::fileFor("payload_calib.json")))` | ✅ |
| "`Decomp::m` 与 `RawFit::cS` 在 `main.cpp:1263`/`:1272` 附近" | `1263` = `for (int a = 0; a < 3; a++) {`、`1272` = `for (int i = 0; i < repeatCount; i++) {` —— **都不是**。真实位置: `Decomp d` 在 `:1324-1327`、`decompose(fit.A, d)` 在 `:1333`、`d.m` 用在 `:1358`、`fit.cS[2]` 在 `:1631` | ❌ 行号错 |

**实质是对的**: 那几处**确实**都在 `#if 0`(`1739`) **之前**的活区, 所以简报想要的那个结论
(活路径手里有料) 成立; 只是给的两个行号指到了不相干的循环上。

### 6.5 简报 §5 第 5 条"8a 已有的 10 条闸用例不许改判据、不许改数字"与本任务的
### 复审 Important 1 直接冲突 —— 我按**裁定**办, 并在此记明

简报 §5 要求"8a 已有的 10 条闸用例**不许改判据、不许改数字**"; 而复审 Important 1 明确要求
"改比较 **AND** 那条断言 500 含的用例 **一起**改", 用户 2026-09-20 已就 `|c|` 的口径做过裁定。
两者不可能同时满足 ⇒ 我按裁定办 (改 **1** 条用例的边界判据), 其余 **9** 条一字未动。
这不是"顺手放宽断言", 而是**只在用户裁定过的那个点上**把实现与规格对齐。

### 6.6 简报 §3.2 的"新判据"与 §3.1 的"无候选"有一处**语义重叠**, 我按两层都实现

§3.1 说 `decompOk == false ⇒ **无候选**`; §3.2 说要给"未实测"一个**独立的判决值**。
函数上这是两件事 (前者是"有没有候选", 后者是"候选能不能发"), 我都做了:
`buildSendCandidate` 在拿不到实测质量尺度时返回 `present = false` (+ `CAND_NO_MEASURED_MASS`),
而 `evaluateSendGate` 在来源不是 `MASS_SOURCE_MEASURED` 时给 `SEND_NOT_MEASURED`。
**照实说明:** 走活路径时 `fitOk` 为真 ⇒ `fitRaw` 自己就调过 `decompose` ⇒ `decompOk` 必然为真
⇒ **`CAND_NO_MEASURED_MASS` 这一支在生产路径上当前不可达**。它留着是把"退回种子值"这条路在
**调用点与判决层各堵一次** (防御性), 不是"现在会发生的事" —— 这句话已逐字写在
`main.cpp` 的注释里 (那次编辑前的 `:1662-1665`)。真正可达的"无候选"是 `echoOk == false` 那一支。

> ⚠ **2026-09-20 二次复审 Minor 2 更正 (本报告原文有误)**: 上面这段初版写的是"**类型层**与判决层
> 各堵一次", **说强了**。`buildSendCandidate` 的 `measuredMassKg` 是 `const double*`, **它不携带
> 来源** —— 任何调用方传一个 `double*` 进来, 里面都把来源硬写成 `MASS_SOURCE_MEASURED`, 类型上
> 拦不住任何东西。真正守着这条不变式的只有两处: `main.cpp` 的调用点 (`decompOk ? &d.m : nullptr`)
> 与 `evaluateSendGate` 里那道判决。**唯一"类型"性质的东西只是 `massSource` 参数没有默认值、
> 漏写编不过** —— 那不是"堵死", 因为候选路径上那个值是写死的。

---

## 7. 硬约束复核 (逐条, 全部通过)

```
$ git show --numstat --format="%h %s" HEAD
37327ec feat(force): 候选只来自本次实测 —— 补上"标定结果 → 可下发"那条断链 (Task 8a-2) + 8a 复审遗留

111	4	Touch_Client/force/PayloadCalibration.cpp
116	13	Touch_Client/force/PayloadCalibration.h
206	81	Touch_Client/main.cpp
244	19	Touch_Client/tests/test_payload_calibration.cpp
```

| 约束 | 实测 |
|---|---|
| 两条命令且顺序固定 (EnableRobot → LoadSwitch(1)) | 未动 `sendPayloadToRobot` / `sendPayloadCommands` 一行 |
| `'p'` 是唯一会发的键; `'s'` 什么都不发 | §3.3 的 grep: 唯一的调用在 `'p'` 那一支 |
| `RelayCore.cpp` 的 `LoadSwitch(0)` 逐字节不变 | 该文件 `git diff --numstat` **为空** (本次一行未碰); `grep -n "LoadSwitch(0)"` → `:245` (注释) / `:539` (代码) |
| **不写 `payload_calib.json`** | §3.4: 唯一写入 `main.cpp:2054` 在 `#if 0` 块内, 本次未新增 |
| `#if 0` 块 (改动前 `:1739`-`:2003`) 不许动 | `awk '/^#if 0/,/^#endif/'` 对该块整段比对: 改动前/后均 **265 行**, `diff` **无输出** ⇒ 逐字节相同 |
| 求解器 / 模型形式 / 门限公式 / `A` 用 `A_F` | 未动 |
| 两份日志的列格式 (`calib_log.txt` / `calib_poses.txt`) | 未动 (`logCalibAttempt` / `logPoseData` 无改动) |
| `Docs/superpowers/evidence/` | `git diff --numstat` **为空** |
| `tests/run_tests.bat` | `git diff --numstat` **为空** |
| 构建前查 `Touch_Client.exe` / 不许 `taskkill` | 每次构建前查, 全部 `No tasks are running...`; 全程未用 `taskkill` |
| 命令行里不放反斜杠 | 本次所有 shell 命令均无反斜杠 (路径一律用 `/` 或引号包裹的 Windows 路径) |

**`buildSendCandidate` 只由候选块调用, 且候选块只在 `'s'` 的活路径上** ⇒ 本次新增的代码
**没有任何一条**能在连接时序里被执行到 ⇒ "下次重启自动下发"那条 `2026-09-18` 的路径没有被重开。

---

## 8. 为什么是一个提交 (不是两个)

派单允许"一个提交 (或两个清晰的提交)"。做成两个提交会**逼出一个编不过的中间提交**:
简报要求的边界判据修正 (`|c|` 严格小于) 必须**同时**改实现与那条用例, 而那条用例在新签名
(`evaluateSendGate(..., MASS_SOURCE_MEASURED)`) 下才写得出 —— 也就是说"Important 1 + 它的用例"
依赖 8a-2 的枚举。硬拆的代价是"先落一个测不了的实现改动", 那与 TDD 直接冲突。
⇒ 单个提交, 并在提交信息里把"复审遗留"与"8a-2"两段分开写清。

---

## 9. 未决 / 需要人决定的事

1. **§6.1 (最重要)**: 简报 §3.5 的翻号前提在本机不成立。我照做了要求的高危分支 (它在
   `cz_robot ∈ (0, 31.5)` 时才会触发, 本机 `68.7` 走不到)。**若 8b 想看到"cz 被纠正"这件事,
   本次的最小改动 [只改 m] 与它无关** —— 简报 §1 已经把这个代价写明并由用户接受。
2. **`'p'` 的模式闸我没撤, 只让它出声**(复审给了"二选一")。理由: `'p'` 是改机械臂负载的
   键, 保留"你在标定流程里"这道范围限制是保守的一侧; 而且候选的生命周期本身已经限定了
   它只可能来自一次成功的 `'s'`。代价照实说: 退出模式后即使候选仍然有效也发不出去,
   必须留在模式内。**若希望"退出模式后仍可发", 那是撤掉 `&& BiasCheck::mode` 一行的事** ——
   这是个产品判断, 我没有替它决定。
3. **§3.2 的 `CAND_NO_MEASURED_MASS` 分支在生产路径上不可达** (§6.6), 属防御性代码。
   若认为"不可达的分支就是负担", 可以只留 `evaluateSendGate` 里那一层。
4. **8b 上机时补跑**: 本次的候选打印块 (含 §3.5 的逐分量差) 需要**接机械臂**才跑得出来
   (`'m'` 在 `--no-robot` 下被拒, `'s'` 要 ≥4 个实机姿态, 全程序没有离线驱动入口 ——
   与 8a §4.3 同一情形)。我能给的最强证据是 §3.5 的 9 条纯函数用例, 覆盖了那个块的
   **全部算术与全部措辞**; 没有被跑过的是**排版**。

---

# 附录 A —— 二次复审遗留 Minor 修复波 (2026-09-20, 在 `a4036b3` 之上)

**范围**: 二次复审给 **Approved / 无 Critical 无 Important**, 遗留 6 条 Minor。本波只改**文字与
一处判据的承载**, 未动任何判决、阈值、比较符、算术。⑷ 复审自己标了"仅报告", **按派单不做**。

## A.1 逐条

| # | 位置 (改后行号) | 改了什么 |
|---|---|---|
| 1 | `main.cpp:1711` | 候选 c 那一行的标签由**写死**"cz 已按闸1 的号定"改成**跟判决走**: `cand.gate.convention != 0 ? "cz 已按闸1 定的号" : "闸1 未定号, cz 即自报原样"` |
| 1 | `PayloadCalibration.h:570` (+`:583-590`) / `PayloadCalibration.cpp:2012`, `:2037` | `SendCandidateDiff` 加 `bool sendable` (= `cand.gate.verdict == SEND_OK`, 在 `diffSendCandidate` 里填); `formatSendCandidateDiffConclusion` 多一支: `!sendable` 时**不再**打"c 未变 —— 本次【只改 m】", 改打"【候选不可发送】…" |
| 2 | `main.cpp:1663` | "在**类型/判决**两层都堵死" → "在**【调用点 + 判决层】**两层堵死", 并补一段说明为什么不能叫"类型层" |
| 2 | 本报告 §6.6 | 同上更正: 原句**就地改**为"调用点与判决层各堵一次", 并在下面加一个引用原措辞的更正块 —— 旧说法留在引用里, 不静默消失 |
| 3 | `main.cpp:2701` | "solveAndApply 里【一处都没有】" → "solveAndApply 的**【活路径】**里一处都没有" |
| 5 | `tests/test_payload_calibration.cpp:3472` | 注释 "499.999 **含**" → "499.999 **放行**" (断言本来就对) |
| 6 | `tests/test_payload_calibration.cpp:3537-3539` | `test_send_gate_refuses_unmeasured_mass_and_says_so` 加断言: `dSameDir/dFlipDir == 0.0`、`!dSameIn && !dFlipIn`、`convention == 0 && czSign == 0.0` |

**新增用例**: `test_send_candidate_diff_conclusion_never_says_only_m_when_unsendable`
(`tests/test_payload_calibration.cpp:3725`, 注册在 `:3875`) —— 两个半边: ① 闸1 定不了号 ⇒ 断言
**有**"候选不可发送"、**无**"只改 m"、无"高危"; ② **翻号 + 发不出去** (m = 2.0 超闸2) ⇒ 断言
**有**"高危"、无"只改 m"。

### A.1.1 分支次序: 为什么"翻号/高危"排在"发不出去"**前面**

初版把 `!sendable` 放在最前, 读起来更顺 ("先说不发得出去")。**改掉了**: §3.5 第 2 条要求"翻号
**【必须】**标高危", 而 `czSignFlipped` 与 `!sendable` **可以同时为真** (cz 号已定且被翻, 但 m 或
|c| 超闸) —— 那时若 `!sendable` 先命中, **高危就被顶掉了**, 等于在没被要求的地方削掉一条硬要求。
现在的次序: 翻号 ⇒ 高危 (不受 `sendable` 影响) / `!sendable` ⇒ 候选不可发送 / 其余照旧。
两个半边都在用例里钉住。

## A.2 覆盖用的命令与原始输出

```
$ ./tests/build_payload_calibration_test.bat        # 工作目录 = Touch_Client/tests
BUILD_EXIT=0                                        # 无 error, 无 warning

$ ./test_payload_calibration.exe
  send_gate_refuses_unmeasured_mass_and_says_so... PASS
  ...
  send_candidate_diff_conclusion_never_says_only_m_when_unsendable... PASS
69 passed, 0 failed                                  # 基线 68 -> 69 (新增 1 条)
```

**反向对照 (证明新用例真的钉住了)**: 把 `else if (!d.sendable)` 临时改成 `else if (false && ...)`,
重编重跑 ——

```
  send_candidate_diff_conclusion_never_says_only_m_when_unsendable... FAIL: t.find("候选不可发送") != std::string::npos
68 passed, 1 failed
```

然后逐字改回 (`grep TEMP-NEGCTRL` 无输出), 重编重跑 ⇒ `69 passed, 0 failed`。
**没有这一条对照, "用例钉住了"就只是我说的话。**

**全量**: `./tests/run_tests.bat` ⇒ **12/12 `[OK]`, 0 `[FAIL]`, exit 0** (与 `a4036b3` 基线同)。

**App 侧**: `./build.bat` ⇒ `Build OK`。唯一一条警告是
`minwindef.h CALLBACK 宏重定义 (glut.h)` —— **与本次改动无关** (本次一行都没碰包含关系),
`a4036b3` 上就在。

## A.3 ★ 复审的行号与主张与代码对不上的地方

派单说了"别信行号, 自己回源头核"。核的结果如下 —— **六条主张的实质全部成立**, 行号错 3 处:

| 复审说的 | 实际 | 判定 |
|---|---|---|
| ① 标签在 `main.cpp:1698-1700` | `diagEmitf` 正在 `:1698-1700` | ✅ 准确 |
| ② `main.cpp:1663-1665` | 主张那半句**只在 `:1663`**;`:1664-1665` 是"真正可达的那一支"与 `buildSendCandidate(...)` 调用, **不是**这句话 | ⚠️ 范围虚胖 (报告 §6.6 里写的 `:1662-1665` 同样) |
| ③ `main.cpp:2690` | 那句在 **`:2689`** | ⚠️ **差 1 行** |
| ⑤ `test_payload_calibration.cpp:3472` | 正是 `// 499.999 含` | ✅ 准确 |
| ⑥ `main.cpp:1715-1720` | `:1715-1716` 正是那道 `if (decompOk && echoOk && verdict != SEND_NOT_MEASURED)`, `:1717-1722` 是两支 d 的打印 | ✅ 准确 |
| ① "`SEND_SIGN_AMBIGUOUS` / `SEND_SIGN_NONE_IN_RANGE` 在 `czSign`/`comMm[2]` 赋值前返回" | `:1931` 返回, 赋值在 `:1946-1947` | ✅ 准确 |

**另外三条回源头核出来的、派单里没有的**:

1. **可达的"有候选但发不出去"不止那两支**。复审只举了 `AMBIGUOUS` / `NONE_IN_RANGE`;实际上
   `SEND_MASS_OUT_OF_RANGE` / `SEND_COM_OUT_OF_RANGE` 也**返回在 `czSign` 赋值之后**
   (`:1949-1950`) —— 那两支**号是定了的**, 所以标签那一行本来就没错, **错的只有结论行**。
   这两支正是"`sendable` 与 `convention != 0` 不能混用"的原因: 标签用 `convention != 0`,
   结论行用 `verdict == SEND_OK`, **两个判据各管各的**。
   反过来, `SEND_NO_CS` / `SEND_NO_CZ_ROBOT` / `SEND_NOT_MEASURED` 经 `buildSendCandidate`
   **不可达**: 没有实测质量尺度 ⇒ 先返回"无候选"; `czRobotMm` 传的是 `&echoCenterMm[2]`,
   **永远非空**。所以"present == true 且发不出去"的可达集**恰好是那四支**。
2. **`#if 0` 块确实嵌在 `solveAndApply` 的花括号内**: 块 `1838-2102` (改动前编号),
   `solveAndApply` 起于 `1145`、闭于 `2103` —— 所以 `:2007` 那处 `setSensorYawDeg` 严格讲
   **在 `solveAndApply` 里**。复审 Minor 3 的判断对, 且"活路径"这个限定是**必要**的, 不是修辞。
   该块内三处调用: `setSensorYawDeg:2007` / `applyResult:2023` / `save:2077`。
   全文件另外**只有一处**在活路径上: `main()` 启动序列 `else of if (g_noRobot)` 里、紧接
   `PayloadCalibration::load` 之后 —— 与注释里写的那个位置一致。**没有第三个**。
   ⇒ 新注释**故意不写行号** (本注释一动行号就漂, 本项目已多次栽在照抄行号上)。
3. **`else` 那一支 ("c 已变 —— 逐分量见上") 经真实路径不可达**: `dc[0]=dc[1]` 恒为 0 (cx/cy 照抄自报值),
   `dc[2] = ±echo.cz − echo.cz` ⇒ `cUnchanged ⟺ !czSignFlipped`。所以"走到 `else`"要求
   既要 `!flipped` 又要 `!cUnchanged` —— 不可能。**本次没删它** (超范围), 只记在这里。

## A.4 没做到的 / 需要人接手的

1. **`main.cpp:1711` 那一行标签没有单测**。它在 `diagEmitf` 的打印端 (写 stdout), 而
   `diagEmitf` 不在本用例的可链接范围内 —— 与 8b §4.3"候选块跑不出来"是同一个原因。
   能做到的是: 它的**判据** (`convention == 0` / `!= 0`) 在库层两条用例里各钉了一次
   (`test_send_gate_two_conventions_in_range_is_ambiguous` / `..._convention_one_wins_and_signs_cz`)。
   **这一行的措辞本身要 8b 上机时用眼睛核。**
2. 本波**没有**跑出候选块的真实屏幕 —— 同上, 需要接机械臂 (`--no-robot` 下 `'m'` 被拒)。

---

# 附录 B — 全分支终审修复波 (2026-09-20, base `68828bd`)

范围: 2 条 Important (I1/I2) + 6 条 Minor (M1..M6) + 2 处**用户批准的行为改变** (I1 活刷新、I3 确认键)。
全部落地。构建: `MSBuild -p:Configuration=Release -p:Platform=x64` -> `Touch_Client.exe` (无 error,
只有既有的 C4005 CALLBACK 宏重定义警告)。构建前 `tasklist //FI "IMAGENAME eq Touch_Client.exe"`
-> `No tasks are running`。

## B.0 测试结果 (原始)

```
# 聚焦 (test_payload_calibration, 不在 run_tests.bat 里)
BUILD_EXIT=0
70 passed, 0 failed

# 全量 (tests/run_tests.bat)
grep -E "passed, 0 failed" | wc -l  ->  12        (12 个用例程序全部 0 failed)
grep -E "failed" | grep -v "0 failed"  ->  (空)
```

## B.1 逐条修复 (行号是**这次的**工作树, 不是 brief 给的; 本项目不许在注释里写行号)

### I1 — `@1168`/`@1176` 回读改成**活刷新** (行为改变, 用户批准)

`Touch_Client/relay/RelayCore.cpp` 实时读取线程内那一段 (`loadEchoDiagPrinted` 在 :87/:104;
写字段在 :93-100 前后)。

- 从前: `static bool loadEchoReported` 在 **sanity 之前**置位, **只在首帧读一次**, 之后整个会话不再刷新。
- 现在: 每收到一帧**合规**的帧就刷新那四个字段 (与下面那批力数据同一把 `forceDataMutex`,
  不需要限流); **诊断行只在第一帧合规帧打印一次** (开关改名为 `loadEchoDiagPrinted`,
  **只在 sanity 通过时置位**)。
- **冻结 bug 修掉**: sanity 不过的帧既不写字段、也不置"已打过"的位 —— 所以首帧坏掉不会
  把整个会话变成"没有回读", 下一次好帧会正常落数据。
- 契约不变: 字段仍只在 `forceDataMutex` 内写, 读方 (main.cpp 的求解基线与候选块) 仍持
  同一把锁读。`core/AppState.h` 上那三个字段的注释已同步 (写明"这三个字段是活的")。
- `RelayCore.cpp` 连接时序里的 `LoadSwitch(0)` **一个字节没动** (见 B.4 证据)。

### I2 — 闸 1 那条安全注释里的**假前提** (comment-only)

`Touch_Client/force/PayloadCalibration.cpp:1937`。

- 原文: "这一步只在 `|选中的 c_s_z| > 31.5` 时成立" -> 改为 `|cz_robot| > 31.5`,
  并把 brief 给的反例 (`cz_robot = +40, c_s_z = +30` => `d = 10` 在 `(0,31.5)` 内 => 约定一 =>
  `c_s_z = +30`, 不大于 31.5 且号没翻) 写进去。
- 全文件搜过 `选中的 c_s`: 只有这一处是错的; 头文件 (`PayloadCalibration.h`) 与用例注释
  本来就写的是 `|cz_robot| > 31.5` 的正确形式。**零行为改变**。

### I3 — `'p'` 加确认键 (行为改变, 用户批准)

`Touch_Client/main.cpp`。确认键定为 **`'y'`** (唯一一处定义: `SEND_CONFIRM_KEY` 在 :2172;
判键与提示文字都取它)。

- `'p'` -> `BiasCheck::sendCandidate()` (:2160 起的那个函数) 现在只做三件: 打安全规程 +
  **逐字打出这一次真正要发的命令** (`EnableRobot(%.3f,%.1f,%.1f,%.1f)` 加 `LoadSwitch(1)`) +
  把候选**照抄**进 `s_confirmCandidate` 并置 `s_awaitingSendConfirm`。**本条分支不发任何字节。**
- 按键处理里**先于所有按键**的拦截 (`main.cpp:2598`, 就在退出键 `'q'`/ESC 那一支之后):
  挂着确认时, 确认键 -> `confirmSendCandidate()` (:2223); **其他任何键 -> `cancelSendConfirm()`
  (:2240) 并出声** ("已取消 —— 本次【什么都没有发出去】")。
- **拒发仍在 `'p'` 那一刻**: 没有候选 / 闸不过 (`verdict != SEND_OK`) 都在设置确认状态之前
  `return`, 所以确认提示不会落到一件本来不会发生的事上。
- **确认发的是屏幕上那一份**: `s_confirmCandidate` 是按下 `'p'` 时的**拷贝**, 不是重新读
  `s_sendCandidate`。
- 清理: `reset()` 与被抢占的 `cancel()` 都会撤销挂着的确认; `'m'` 关模式那一支也加了防御性撤销。
- 提示文字: `'m'` 模式的屏幕说明里加了 `'p'` 那一行 (含确认键); 上机操作单 §6 加了一条注
  (`Docs/superpowers/specs/2026-09-19-on-machine-checklist.md`, 并顺手把那句"三条一起发"
  标成"设计 §6b 的原始清单", 免得与现行两条例程打架 —— 这一句是加了注之后必须补的, 否则
  同一节里两句话互相矛盾)。

### M1 — 不可达分支加注释 (不删)

`Touch_Client/force/PayloadCalibration.cpp:2077` 起 (那个 `else` = "c 已变 —— 逐分量见上")。
注释写明: 在今天唯一的调用点上不可达, 理由是 `dc[0] == dc[1] == 0` 恒成立 (cx/cy 是同一个
echo 数组的逐位拷贝), 而 `dc[2] != 0` 只可能来自翻号 (已被 `czSignFlipped` 走掉)。
**核过**: `evaluateSendGate` 开头是 `for i: g.comMm[i] = comMmIn[i]`, `buildSendCandidate` 末尾
是 `c.comMm[i] = c.gate.comMm[i]`, 而 `diffSendCandidate` 拿的是同一个 `echoCenter` 数组。成立。

### M2 — 候选块 c 那一行的标签抽进库里 + 补测

- 新增 `PayloadCalibration::formatSendCandidateCenterLabel(const SendGate&, char*, int)`
  (声明 `PayloadCalibration.h:594`, 实现 `PayloadCalibration.cpp:2030`), 措辞**逐字沿用**
  原有的两句 ("cz 已按闸1 定的号" / "闸1 未定号, cz 即自报原样"), 打印端 (`main.cpp:1748`) 改成调它。
- 新用例 `test_send_candidate_center_label_follows_the_verdict` 两种情形各钉一次 (逐字相等)。

### M3 — 两个操作员可见的 `snprintf` 缓冲加余量

**用格式串本身量过** (不是估的):

- `formatSendGateConclusion` 最长一支 (`SEND_NOT_MEASURED`) = **219 字节** -> `char reason[256]` 的
  两处**都**改成 `[512]` (`main.cpp:1841` 候选块那一屏、`main.cpp:2186` `'p'` 被拒那一支)。
- `formatSendCandidateMassText` = **465 字节** -> `char massLine[512]` 改成 `[1024]` (`main.cpp:1734`)。
- 见 B.3 第 1 条: brief 说 "467 into 512", 实测 465, 而那个 512 缓冲是 `massLine`, **不是**
  `concl[512]` (后者当时最长只有 250 字节)。

### M4 — 发送侧返回值不再丢弃

`Touch_Client/relay/RelayCore.cpp:283` / `:299` (两条命令各一处)。

- `robotSendEnable` 的返回值存进 `sentEnable` / `sentLoadSwitch`; 发送失败时**不再去 recv**
  (不会把一条从没发出去的命令的回执算进来), 回执行打 **`(发送失败 — 命令没能写进 socket)`**,
  与 "`(无回执, 超时)`"、与 "`机械臂拒绝 (ErrorID != 0)`" 三者互斥、各说各的。
- 汇总行也补一句: 失败条目里有发送侧的失败时明写"命令没写进 socket, 不是机械臂拒绝"
  (两者处置不同: 前者查链路, 后者查命令)。
- 这一条比 finding 的字面多了一点: 发送失败时**跳过 Sleep+recv**。理由与判据同源
  (不能把没发出去的命令说成"超时"), 但它确实是行为改变, 记在这里。

### M5 — 归因改 `switch`

`Touch_Client/main.cpp:1701` 起。`absentWhy` 用 `switch (cand.absent)`, 两个已知归因各一支,
`CAND_PRESENT`/`default` 合起来给"归因未知 (不该发生)"。加第三种归因时**不会**被静默标成
"没有回读到负载"。措辞逐字不变。

### M6 — 高危结论行的"要不要发"按**发得出去**分岔

`Touch_Client/force/PayloadCalibration.cpp:2048` 起。翻号那一支拆成两个 `snprintf`:

- `d.sendable` -> 保留原来的 "按 §1 的安全规程处置后**再决定发不发**";
- `!d.sendable` (翻号 + 闸不过, **可达**) -> "【候选不可发送】—— 按 'p' 会被拒 (理由见下面闸的
  结论行), 所以这里没有'发不发'可决定; 按 §1 的安全规程处置"。

两句话都带"高危"与"c 已变" (§3.5 的硬要求不被"发不出去"顶掉)。用例两侧各加 CHECK。

## B.2 两条反向对照 (原始输出)

### 对照 1 — 临时交换 `formatSendCandidateDiffConclusion` 的两支 (`czSignFlipped` 与 `!sendable` 换序)

```
BUILD_EXIT=0
  send_candidate_diff_conclusion_never_says_only_m_when_unsendable... FAIL: tf.find("高危") != std::string::npos

69 passed, 1 failed
```

**判读**: 失败的正是该用例的**后半段** (翻号 + 发不出去那一组, 用 `tf`), 前半段 (发不出去但没翻号,
用 `t`) 仍然过 —— 也就是说"翻号必须优先标高危"这条**次序**性质是被次序专属的那一半抓住的,
**没有**被另一半顺带抓住。另外 69 条 (含翻号且发得出去那条 `flipped_cz_as_high_risk`) 全过。
之后已还原, 重跑 `70 passed, 0 failed`。

### 对照 2 — 临时把中心标签退回"写死号已定"的旧谓词 (忽略 `gate.convention`)

```
BUILD_EXIT=0
  send_candidate_center_label_follows_the_verdict... FAIL: t == "闸1 未定号, cz 即自报原样"

69 passed, 1 failed
```

**直说**: **新的 M2 用例确实抓住了它** —— 失败点正是下半段 (闸 1 没定号那一组) 的逐字比较。
所以这条标签从此不再是"没有测试钉住的操作员可见字符串" (旧报告 §A.4 第 1 条记的正是这个缺口)。
之后已还原, 重跑 `70 passed, 0 failed`。

## B.3 这些发现的**行号/说法与实际代码对不上的地方**

1. **M3 的 "467 into 512" 数对、缓冲说错了**: 实测 465 字节、那个 512 缓冲是
   `formatSendCandidateMassText` 的 `char massLine[512]`, **不是** `concl[512]`
   (`formatSendCandidateDiffConclusion`, 当时最长 250 字节)。两个 "512" 在同一屏里, 光按数字
   找会改错那一个。另外那个 256 缓冲 (`char reason[256]`) 在文件里**出现两次**
   (候选块那一屏 + `'p'` 被拒那一支), brief 说"两个缓冲"实际是"两种、三处"。
2. **M6 之后 `concl[512]` 的余量从 250/512 涨到 352/512**: 翻号 + 发不出去那一支变长了。
   仍在 30% 余量以上, 且它不在 M3 点名的两个缓冲里, 所以**这次没动**; 记在这里供下次改措辞时参考。
3. **M1 的"不可达"与本报告 §A.3 是同一件事** —— A.3 当时已经把它记下来 (结论一致, 理由也一致),
   本次只是把结论落成代码注释 (M1 要的正是这个)。**没有冲突**。
4. **I2 的错误前提确实是"评审里来的"**: brief 说的"源自第一次任务评审 Minor 5、被逐字写进代码"
   与实际相符 —— 注释里那半句与头文件/用例里的正确形式**互相矛盾**, 而代码本身按正确的那个跑
   (`g.czSign = (csZSelected >= 0.0) ? +1.0 : -1.0` 只看选中的 c_s_z 的号, 与 `|cz_robot|` 无关)。
   即: **错误只在注释里, 判据本身一直是对的**。
5. **I1 的两个附带事实都核过**: (a) `static bool loadEchoReported` 是**函数级 static**, 而那个函数
   就是线程体, 所以它跨 `robotCloseRealtime()` / 重连**都不重置** —— brief 说"连重连也不重置", 对;
   (b) 全零 `@1168` 确实过得了那条 sanity (`echo[0] >= 0.0 && <= 5.0`, 其余 `fabs <= 500`), 所以
   "首帧坏掉就冻结整个会话"成立。四个字段的写者也确实全项目只有这一处 (grep 过)。
6. **一处 brief 没提、但同型的陈旧行号**: 上机操作单 §0 写着连接时序的 `LoadSwitch(0)` 在
   `RelayCore.cpp:473` —— 在 base `68828bd` 上它已经是 **539**, 本次改动后是 **577**。
   属"注释里的行号会漂"这个老毛病 (brief 自己也点了这一条)。**本次没改** (超出点名的范围,
   且 §6 只许加一条最小改动), 记在这里当作待办。

## B.4 约束核对 (逐条)

- 两条命令、顺序 `EnableRobot` -> `LoadSwitch(1)`、**不发 `PayLoad`**: 未动 (`RelayCore.cpp`
  的 `sendPayloadCommands` 一个字没改命令文本与顺序)。
- `'p'` (+ 确认) 仍是**唯一**会发东西给机械臂的路径; `'s'` 路径不发任何东西: 未动。
- **连接时序的 `LoadSwitch(0)` 逐字节不变**: `git diff -U0 Touch_Client/relay/RelayCore.cpp | grep
  "LoadSwitch(0)"` -> **无输出** (即该行既不在增行也不在删行里); 该行文本与 `git show HEAD` 里
  的那一行逐字相同 (行号 539 -> 577 只是位移)。
- 没有任何东西写 `payload_calib.json`: 未动。
- 没碰 `#if 0` 块 / 求解器 / 模型形式 / 门限公式 / `A` 仍是 `A_F` / `calib_log.txt` 与
  `calib_poses.txt` 的列格式 / 证据文件 / `tests/run_tests.bat`。
- `|c| < 500` 仍严格小于; 质量带 `[0.2, 1.5]` kg 仍闭区间: 未动。
- 除本波点名的三处缓冲大小、M4 的发送失败分支、I3 的按键流、以及 I1 的活刷新之外,
  **没有改任何判决、比较或算术**。
- 所有新注释里的前提/出处/数字都对着**存在的文件**核过 (30004 偏移 @1168/@1176 见
  `Docs/superpowers/evidence/robot-baseline-report.md` 与 `.../task-session-report.md`;
  缓冲字节数见 B.3)。**注释里一律没有写行号。**

## B.5 需上机用眼睛核的

1. `'p'` -> `'y'` 的真实屏幕 (含"这一次真正要发的命令"那一行) 只有在 `'m'` 模式里、接上机械臂
   之后才看得到 —— 本波离线跑不出来, 与 8b §4.3 同因。
2. I1 的活刷新在**真机**上的样子: 发一条 `'p'` 之后, 下一次 `'s'` 的"与机械臂【当前值】相比"
   应当**跟着变** (地址对齐了才对齐) —— 这正是 Task 10 闭环要用的那条性质。离线无法证。
