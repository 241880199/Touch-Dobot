# 闸门参考量换 @720 + 力矩不投票 + A 第三行守门 —— 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development 执行本计划。
> 步骤用 `- [ ]` 语法跟踪。

**依据（必读）:**
- `Docs/superpowers/specs/2026-09-20-raw-channel-calibration-run-004.md` —— **§4.5 / §7.1 / §7.2 是本计划的全部事实来源**
- `Docs/superpowers/specs/2026-09-19-remaining-workflow.md` —— 其 8b / Task 10 的判据写的是 `@576`，
  **已被 run-004 §4.5 判定为错参考量。本计划取代该判据部分**，但不取代其中的安全规程与步骤编排。

**目标:** 让"传感器力那一条路"能通 —— 把一致性闸门的参考量从 `@576` 换成 `@720`，把力矩通道
从投票里拿掉（`Fz` 早已不投票），并为 `A` 的第三行补一个不依赖机械臂的守门。

**架构:** 三件事**必须一起做**，缺一件链路都不通：

```
只换参考量            -> Fx/Fy 放行, 但 Mx/My/Mz 仍超限 -> 整体仍拒
只让力矩不投票        -> Fx/Fy 仍超限 (对 @576 时 Fx 超限 5 倍) -> 整体仍拒
两件一起              -> 闸门放行 -> compensated[] 有数据 -> 触觉拿到传感器力
第三件 (A 第三行守门) -> 是前两件换来的安全债, 必须同期还
```

**为什么力矩投票是致命的（给零上下文的实现者）:** 触觉设备**只收三个力分量**
（`ForcePipeline` 只映射 `fx / -fz / fy`，`hapticOut` 是 `double[3]`）。
但闸门判决是**全或无**：任一投票通道超限 -> `compensated[]` 全 6 个分量置零 ->
下游 `filtered` / `hapticOut` / `F|` 帧**一起断**。
**⇒ 力矩对不上会把"力"也一起掐死，而演示根本不消费力矩。**

**用户约束（原话，逐条照抄）:**
- "1.5 kg太重了，昨天已经测试过了，会导致机械臂迅速运动报错"
- "直接修改参数让机械臂动太危险了，有没有别的方案"
- "门限可以适当降低标准，但安全起见不符合时还是要拒绝"
- "在运行过程中机械臂需要得到正确的负载参数才能正确执行指令"
- **不要杀运行中的 `Touch_Client.exe`**
- "任何猜测以及偏差都不可取" · "不引入人工判断"（需要判断的地方一律由程序算）

## Global Constraints

- **⚠ 本计划只写符号名，不写仓库内代码的行号与计数。** 出处：本项目记过
  "派单不许写仓库内代码的行号与计数 —— 三个修复波共纠错 12 处，实质主张全成立、错的只是引用"。
  行号只对外部稳定文档（`Docs/superpowers/specs/` 下）写。**发现本计划里任何一句与代码不符时，
  按代码改本计划，并在报告里说明是哪一句。**
- **编译命令固定为：** `cmd.exe //c "D:\Projects\Touch\Touch_Client\build.bat"`，成功标志 `Build OK.`
- **完整构建是硬性要求**：测试脚本不定义 `WIN32_LEAN_AND_MEAN`，真实项目定义 ——
  单测通过**不等于**项目能编。
- **测试必须真的跑起来**：构建脚本只编译。本计划涉及 `test_force_compensation`，用
  `Touch_Client/tests/build_force_comp_test.bat` 构建，再**单独执行** `test_force_compensation.exe`。
  **`run_tests.bat` 不覆盖全部套件**，不要拿它当"全绿"的证据。
- **编译前确认 `Touch_Client.exe` 没在运行**，否则 `LNK1168`。**不要替用户杀进程** —— 报告并停下。
- **不要写以反斜杠结尾的 `//` 注释** —— 会吞掉下一行。
- **不引入人工判断**：需要判断的地方一律由程序算。
- **本计划的全部改动都不需要机械臂动、不需要动负载。** 任何需要机械臂动的步骤都不在本计划内。

## 背景：三件事实（每条都有实测出处）

| # | 事实 | 出处 |
|---|---|---|
| 1 | `@576` = "TCP【传感器】力值"；`@720` = "TCP力值（【通过关节电流计算】）"。**只有后者反映控制器在用的负载参数** | 厂商接口文档 30004 布局表；run-004 §4.5 |
| 2 | 对 `@720`，力通道差 `Fx +0.0435 / Fy +0.156`（容差 0.5）；力矩通道差 `Mx +0.061 / My +1.147 / Mz +1.316`（容差 0.03） | run-004 §4.5 |
| 3 | `@720` 的力矩有**约 90% 是姿态无关的偏置**，且该偏置**在漂**（`Mx` 20 分钟漂 0.55 N·m）。姿态无关的偏置**不可能是负载效应** ⇒ **换负载参数改不动它** | run-004 §4.5① 与 §7.2 |

**⇒ 事实 3 是本计划只做"不投票"、不去"修 `@720`"的原因。**

## 现状的耦合点（实现者必须逐个处理，缺一个都会留下"说的和做的不一致"）

| 耦合点 | 符号 | 处置 |
|---|---|---|
| 判据参考量 | `step()` 里喂给 `g_guardEma` 的那个量（现为 `fd.raw`，即 `@576`） | Task 2 改成一处定义的 `@720` |
| **`@720` 的诊断 EMA** | `g_guardEma720` / `g_guardSeeded720` | Task 2 **角色互换**：变成为 `@576` 的诊断 |
| 投票掩码 | `g_guardVote` | Task 4 改 |
| **掩码的第二份实现** | `GuardReport::voted` 的**类内字面量初值** | Task 4 必须消掉这个歧义 |
| 写死 `@576` 的文字 | `setGuardState` 的放行消息 · `GuardState` 枚举注释 · `GuardReport::ema` 注释 · `RobotErrorCode::ERR_FORCE_INCONSISTENT` 注释 | Task 2 / Task 4 随各自改动更新 |
| 逐通道表与复报行 | `g_guardEma` / `g_guardEma720` 的打印分支 | Task 2 |
| 钉住掩码的测试 | `test_guard_moment_channel_votes` | Task 4 **显式改写**（它现在会红，而且应该红） |

---

### Task 1: 前置复核（只读，不改代码）

**为什么有这一步:** 本计划把 `@720` 当成判据参考量，而支撑它的读数**部分是控制台抄录**
（run-004 §3 已声明 `'p'`/`'y'` 与闸门输出不落盘）。本项目记过"前提凭记忆写"的账，
所以先把这几个数落到纸上，再让后面所有任务引用它们。

**Files:**
- Create: `Docs/superpowers/specs/2026-09-21-gate-reference-prereq.md`

**Interfaces:**
- Produces: 两个数，Task 3 要用 ——
  `forceOffsetMax_N`（`@720` 力通道偏置的最坏观测值）与
  `forceOffsetDrift_N`（该偏置跨轮次的漂移幅度）

- [ ] **Step 1: 从已有记录里摘出 `@720` 的读数**

数据源**只用仓库里已有的文件**，不要去现场、不要动机械臂：
`Docs/superpowers/specs/2026-09-20-raw-channel-calibration-run-004.md`（§4.5 与 §4.5③）、
`.superpowers/sdd/progress.md`、`Touch_Client/calib/calib_poses.txt`（三份带 `F720*`/`M720*` 列的尝试）。

逐条记下并**注明出处文件与段落**：
- `comp − @720` 的逐通道值（六个）与当时的容差
- `@720` 力通道的偏置 `c` 与跨轮次漂移
- `@720` 力矩通道的偏置 `c` 与跨轮次漂移

- [ ] **Step 2: 算出 Task 3 要用的两个数**

```
forceOffsetMax_N   = max(所有轮次里 |@720 力通道偏置|)        <- 取绝对值最大的那个
forceOffsetDrift_N = max(所有轮次) - min(所有轮次) 的幅度
```

- [ ] **Step 3: 写下判决与它的反面**

在文档末尾写两段，**必须都写**：
1. **支持换参考量的证据**（引用 Step 1 的数）
2. **可能推翻它的证据** —— 至少包括：`@576` 与 `@720` 的关系里**没有文档依据**的那一条
   （run-004 §7.2 记的"`@720` 与 `@1304` 反相关，符号约定无文档依据"）

- [ ] **Step 4: 提交**

```bash
git add Docs/superpowers/specs/2026-09-21-gate-reference-prereq.md
git commit -m "docs(gate): @720 作为判据参考量的前置复核 (只读)"
```

**⚠ 这一步不改任何代码，也不需要机械臂。若复核推翻"换 `@720`"，停下并报告 —— 不要继续 Task 2。**

---

### Task 2: 参考量换到 `@720` —— 一处定义 + 角色互换

**Files:**
- Modify: `Touch_Client/force/ForceCompensation.cpp`
- Modify: `Touch_Client/force/ForceCompensation.h`
- Modify: `Touch_Client/tests/test_force_compensation.cpp`

**Interfaces:**
- Consumes: Task 1 的 `forceOffsetMax_N`（只在报告里引用，不在本任务判数）
- Produces: `ForceCompensation::guardReferenceValue(const AppState::ForceData& fd, int channel) -> double`
  —— **闸门判据参考量的唯一一份实现**。Task 4 不改它。

- [ ] **Step 1: 写失败的测试**

加到 `test_force_compensation.cpp`。**先读同文件里 `test_guard_passes_when_consistent` 的写法**，
照它的构造方式（`ForceCompensation::init()` -> `setCalibration(...)` -> 填 `AppState::ForceData`
-> `step(fd, pose)` -> 读 `guardState()`）来写。

```cpp
// 判据参考量必须是机械臂【通过关节电流算】的那一路 (@720)，不是传感器侧 (@576)。
// 构造: 让 @720 与本地一致, 而 @576 严重不符 -> 必须【放行】。
static void test_guard_reference_is_the_current_derived_channel() {
    TEST(guard_reference_is_the_current_derived_channel);
    ForceCompensation::init();
    double A[9]; diagA(1.0, A);
    double com[3] = {0, 0, 0};
    double bF[3] = {0, 0, 0}, bM[3] = {0, 0, 0};
    ForceCompensation::setCalibration(A, bF, bM, com);

    double pose[6] = {0, 0, 0, 0, 0, 0};
    AppState::ForceData fd;
    fd.sixForceRaw[2] = 9.81;   // 本地算出 compensated = (0,0,0)
    fd.tcpForce[0]    = 0.05;   // @720 说 x 上几乎没外力      <- 判据该看这个
    fd.raw[0]         = 0.90;   // @576 说 x 上有 0.9 N         <- 不该再看这个
    ForceCompensation::step(fd, pose);

    CHECK(ForceCompensation::guardState() == ForceCompensation::GuardState::OK);
}
```

- [ ] **Step 2: 跑它，确认失败**

```
cmd.exe //c "D:\Projects\Touch\Touch_Client\tests\build_force_comp_test.bat"
D:\Projects\Touch\Touch_Client\tests\test_force_compensation.exe
```
Expected: `test_guard_reference_is_the_current_derived_channel` FAIL
（现在判据看 `fd.raw`，而 `fd.raw[0] = 0.90` 超容差）

- [ ] **Step 3: 实现 —— 把参考量收成一处**

在 `ForceCompensation.cpp` 里，紧挨 `g_guardVote` 的定义处加：

```cpp
// ===== 闸门判据的参考量: 全程序【唯一一份】定义 =====
//
// 厂商 30004 布局:
//   @576 ActualTCPForce = "TCP【传感器】力值"      -> fd.raw[]
//   @720 TCPForce       = "TCP力值(【通过关节电流计算】)" -> fd.tcpForce[]
// 只有 @720 反映【控制器正在用的负载参数】(算它必须知道负载: 重力矩 + 惯量矩)。
//
// ⚠ 参考量本身也要被验证 (本项目记过: 一个通道"叫什么名字"不等于"它是什么")。
//   实测依据: run-004 §4.5 —— @576 的重力系数跨轮次纹丝不动 (改负载 0.404->0.422 后
//   仍是 0.206/0.208/0.211), 即它对"负载有没有被采纳"完全不响应。
// ⚠ 打印端【不许】再抄一遍 @576 / @720 的字面量: 那份文字会在改参考量时撒谎。
//   这条注释就是判据唯一的出处。
static inline double guardReferenceValue(const AppState::ForceData& fd, int ch) {
    return fd.tcpForce[ch];
}
```

然后把**判据 EMA**（`g_guardEma`）的喂入量换成 `guardReferenceValue(fd, i)`，
并把它原来用的 `fd.raw[i]` 交给**诊断 EMA**。

- [ ] **Step 4: 角色互换 —— `g_guardEma720` 变成为 `@576` 的诊断**

`g_guardEma720` / `g_guardSeeded720` **改名**为 `g_guardEmaDiag` / `g_guardSeededDiag`，
喂入量改为 `fd.raw[i]`（`@576`），并保留"**只报不判**"的定位 —— 它不参与任何容差比较。
**改名而不是留旧名**: 旧名带着 `720` 三个字，而它现在的含义正好相反，留着就是埋雷。

- [ ] **Step 5: 更新同批的文字（不更新就是"说的和做的不一致"）**

| 位置 | 改成 |
|---|---|
| `setGuardState` 的放行消息 | "本地全量模型与【参考量】逐通道一致"，**不要再写字面量 `@576` 或 `@720`** |
| `GuardState` 枚举里 `INCONSISTENT` 的注释 | 同上，说"与参考量对不上"，并指向 `guardReferenceValue` 是判据出处 |
| `GuardReport::ema` 的注释 | 改成 "compensated − 【参考量】的 EMA" |
| `RobotErrorCode::ERR_FORCE_INCONSISTENT` 的注释 | 同样去掉写死的 `@576` |
| 复报行 / 逐通道表 | "与 @720" 那半边的标签改成 "与 @576"（它现在是诊断侧） |

**⚠ 这一条是本任务最容易漏的一步。** 本项目记过："提示文字里再抄一遍阈值/口径，
就是会在改判据时撒谎的第二来源。"

- [ ] **Step 6: 跑全部用例，确认全绿**

```
cmd.exe //c "D:\Projects\Touch\Touch_Client\tests\build_force_comp_test.bat"
D:\Projects\Touch\Touch_Client\tests\test_force_compensation.exe
```
Expected: 全 PASS。**⚠ `test_guard_refuses_when_inconsistent` 可能变红** ——
它的构造用的是 `fd.raw[0] = 0.90`（`@576` 侧）。若红，**按同一个语义把它改到 `@720` 侧**
（即让 `fd.tcpForce[0]` 超限），**不要**改成"反正不看了"。

- [ ] **Step 7: 完整构建**

```
cmd.exe //c "D:\Projects\Touch\Touch_Client\build.bat"
```
Expected: `Build OK.`

- [ ] **Step 8: 提交**

```bash
git add Touch_Client/force/ForceCompensation.cpp Touch_Client/force/ForceCompensation.h Touch_Client/tests/test_force_compensation.cpp
git commit -m "feat(gate): 判据参考量换成 @720 (通过关节电流计算), 收成一处定义; @576 退为只报不判的诊断"
```

---

### Task 3: 力通道容差按 `@720` 重标

**Files:**
- Modify: `Touch_Client/config/Config.h`
- Modify: `Touch_Client/force/ForceCompensation.cpp`（只改注释里的出处）
- Modify: `Docs/superpowers/specs/2026-09-21-gate-reference-prereq.md`（补一节记录决策）

**Interfaces:**
- Consumes: Task 1 的 `forceOffsetMax_N` / `forceOffsetDrift_N`；Task 2 换好的参考量
- Produces: `Config::FORCE_GUARD_TOL_FORCE_N` 的最终取值 + 它的依据文字

- [ ] **Step 1: 用 Task 1 的数算余量，把算式写下来**

```
最坏情形 ≈ forceOffsetMax_N + forceOffsetDrift_N
余量     = FORCE_GUARD_TOL_FORCE_N / 最坏情形
判据: 余量 ≥ 2
```

**把这三个数写进 `Config.h` 的注释里**，连同出处（`2026-09-21-gate-reference-prereq.md`）。

- [ ] **Step 2: 余量不足则调容差；够则保持并写明"为什么够"**

⚠ **不要凭感觉调。** 若需要上调，**必须同时写下上调后它还能抓住什么** ——
一个宽到抓不住任何东西的容差不是容差。用户约束是"门限可以适当降低标准，
但安全起见不符合时还是要拒绝"。

- [ ] **Step 3: 在 `Config.h` 里写清这个容差是给谁的**

现有注释写的是"力通道 (x/y) 一致性容差 (N)"。补上：
- **是相对哪个参考量**（指向 `guardReferenceValue`，不要写 `@720` 字面量）
- **`@720` 的力自带一个会漂的偏置**（引用 `forceOffsetDrift_N`），所以余量里已经算了它

- [ ] **Step 4: 跑测试 + 完整构建**

```
cmd.exe //c "D:\Projects\Touch\Touch_Client\tests\build_force_comp_test.bat"
D:\Projects\Touch\Touch_Client\tests\test_force_compensation.exe
cmd.exe //c "D:\Projects\Touch\Touch_Client\build.bat"
```
Expected: 测试全 PASS；`Build OK.`
⚠ 若有测试断言了旧的容差值，**按新值改测试并写清理由**。

- [ ] **Step 5: 提交**

```bash
git add Touch_Client/config/Config.h Touch_Client/force/ForceCompensation.cpp Docs/superpowers/specs/2026-09-21-gate-reference-prereq.md
git commit -m "fix(gate): 力通道容差按 @720 重标, 并把 @720 自带的漂移偏置算进余量"
```

---

### Task 4: 力矩不投票 —— 掩码一处定义 + 报告不撒谎

**Files:**
- Modify: `Touch_Client/force/ForceCompensation.cpp`
- Modify: `Touch_Client/force/ForceCompensation.h`
- Modify: `Touch_Client/tests/test_force_compensation.cpp`

**Interfaces:**
- Consumes: Task 2 的 `guardReferenceValue`
- Produces: 新的生效掩码 —— `Fx` / `Fy` 投票，`Fz` / `Mx` / `My` / `Mz` **不投票但照报**

- [ ] **Step 1: 写失败的测试**

```cpp
// 力矩通道【不投票】: 力矩严重不符, 也必须放行 —— 但那一半仍要照实报出来。
static void test_guard_moment_channel_reports_but_does_not_vote() {
    TEST(guard_moment_channel_reports_but_does_not_vote);
    ForceCompensation::init();
    double A[9]; diagA(1.0, A);
    double com[3] = {0, 0, 0};
    double bF[3] = {0, 0, 0}, bM[3] = {0, 0, 0};
    ForceCompensation::setCalibration(A, bF, bM, com);

    double pose[6] = {0, 0, 0, 0, 0, 0};
    AppState::ForceData fd;
    fd.sixForceRaw[2] = 9.81;      // 本地算出 compensated = (0,0,0)
    fd.tcpForce[0] = 0.05;         // 力: 在限内
    fd.tcpForce[1] = 0.05;
    fd.tcpForce[3] = 1.20;         // 力矩: 远超声明的力矩容差
    ForceCompensation::step(fd, pose);

    // 不投票 -> 放行
    CHECK(ForceCompensation::guardState() == ForceCompensation::GuardState::OK);
    // 但必须照实报 —— "不投票"不等于"不检查、不显示"
    ForceCompensation::GuardReport rep;
    ForceCompensation::guardReport(rep);
    CHECK(rep.voted[3] == false);
    CHECK(fabs(rep.ema[3] - (-1.20)) < 1e-9);
}

// 力通道必须【仍然】投票 —— 防止掩码被改过头。
static void test_guard_force_channels_still_vote() {
    TEST(guard_force_channels_still_vote);
    ForceCompensation::init();
    double A[9]; diagA(1.0, A);
    double com[3] = {0, 0, 0};
    double bF[3] = {0, 0, 0}, bM[3] = {0, 0, 0};
    ForceCompensation::setCalibration(A, bF, bM, com);

    double pose[6] = {0, 0, 0, 0, 0, 0};
    AppState::ForceData fd;
    fd.sixForceRaw[2] = 9.81;
    fd.tcpForce[0] = 5.00;         // 力严重不符
    ForceCompensation::step(fd, pose);

    CHECK(ForceCompensation::guardState() == ForceCompensation::GuardState::INCONSISTENT);
}

// 默认构造的 GuardReport 【不许】声称任何一份掩码。
static void test_guard_default_report_claims_no_mask() {
    TEST(guard_default_report_claims_no_mask);
    ForceCompensation::GuardReport rep;
    for (int i = 0; i < 6; i++) CHECK(rep.voted[i] == false);
}
```

- [ ] **Step 2: 跑它，确认失败**

```
cmd.exe //c "D:\Projects\Touch\Touch_Client\tests\build_force_comp_test.bat"
D:\Projects\Touch\Touch_Client\tests\test_force_compensation.exe
```
Expected: 三条新用例 FAIL（现掩码是 `{true,true,false,true,true,true}`）

- [ ] **Step 3: 改掩码，并把"为什么"写在它旁边**

```cpp
// 逐通道投票掩码 —— 本数组是【掩码的唯一一份实现】。
//   index:      0    1    2     3     4     5
//             Fx   Fy   Fz    Mx    My    Mz
static const bool g_guardVote[6] = { true, true, false, false, false, false };
```

**必须一并写下的两条理由（缺一条后人就会把它改回去）:**

```
Fz 不投票: 理由与出处见 Config.h 里 FORCE_GUARD_TOL_FORCE_N 上方那段
           (z 轴缺口未查清, 见开放项 C)。

力矩三个分量不投票 (2026-09-21 起):
  实测 (run-004 §4.5): 换成 @720 后力矩仍差 Mx +0.061 / My +1.147 / Mz +1.316,
  而容差 0.03。
  该差距【不是负载效应】: 约 90% 是姿态无关的偏置, 而负载误差产生的残差必然随姿态变;
  且该偏置【在漂】(Mx 20 分钟漂 0.55 N·m) -> 既改不动、也不能标定掉。
  同姿态秒级复采的重复性本身就有 0.06~0.13 N·m, 比 0.03 的容差还大 2~4 倍。
  => 力矩与参考量之间不存在"一致"态, 强行投票会让整个闸门永远拒绝,
     而判决是全或无 -> compensated[] 全置零 -> 【力通道也一起断】。
  ⚠ 力矩【不是被删掉】: 照算、照报、照给 MATLAB 的 F| 帧。它仍是质心与惯量的
     唯一测量窗口 (payLoad 的 I 只能从力矩通道辨识), 只是不再投票。
  ⚠ 代价【必须记账】: Fz 早已不投票, 力矩原本兜着 A 的第三行 -> 现在无人兜。
     见开放项 C 与 Task 6。
```

- [ ] **Step 4: 消掉掩码的第二份实现**

`ForceCompensation.h` 里 `GuardReport::voted` 的**类内字面量初值**是掩码的第二份实现，
两份不一致时一个默认构造的 `GuardReport` 会对外报出与实际生效不同的掩码。

**处置: 把类内初值改成"全 false"，并在注释里写明它表示"尚未填充"。**
理由：类内初值无法引用 `.cpp` 里的 `static`，任何字面量都会漂；
而"全 false"不可能被误读成一份真实的掩码。填充由 `guardReport()` 负责（它已经在填）。

**★ 2026-09-21 增补（Task 2 复审发现，计划此前只知道两份）: 掩码还有【第三份实现】**
`Touch_Client/tests/test_payload_calibration.cpp` 的 `test_runtime_consistency_guard_replay` 里
有一个**函数内局部**的 `const bool vote[6] = { true, true, false, true, true, true };`，
它用来自算该用例的判决。**本任务必须一并处理它** —— 否则掩码改了之后，
这份拷贝会**静默地继续模拟旧闸门**，而该用例的所有读数都会与生产不一致。
处置与另外两份同源：**让它从生产的那一份取值（或按同一处定义派生），不要再抄字面量。**
⚠ 同一个用例里还有一处打印拿 `@576` 的秩 2 当 "`Fz` 不投票" 的依据，
而该依据**从未在新参考量上复测**（Task 2 已给它加了"未复测"标注）——
本任务不得把它当作已验证的前提来用；要么复测，要么明确标注它仍未验证。

```cpp
// ⚠ 这里是【尚未填充】的默认值, 不是掩码。掩码的唯一一份实现是 .cpp 里的 g_guardVote。
//   从前这里抄了一份字面量, 于是"改一处忘一处"会让默认构造的报告对外撒谎。
//   guardReport() 会把真实掩码填进来。
bool   voted[6]    = {false, false, false, false, false, false};
```

- [ ] **Step 5: 显式改写钉住掩码的那条旧用例**

`test_guard_moment_channel_votes` 的**名字和断言都是为旧掩码写的**。
把它改名为 `test_guard_moment_channel_reports_but_does_not_vote` 的语义（或直接删除，
用它换成了 Step 1 的三条新用例 —— **但要在提交信息里说明删了哪条、为什么**）。
**⚠ 不许留着一条名字说"votes"而断言相反的用例。**

- [ ] **Step 6: 跑全部用例 + 完整构建**

```
cmd.exe //c "D:\Projects\Touch\Touch_Client\tests\build_force_comp_test.bat"
D:\Projects\Touch\Touch_Client\tests\test_force_compensation.exe
cmd.exe //c "D:\Projects\Touch\Touch_Client\build.bat"
```
Expected: 全 PASS；`Build OK.`

- [ ] **Step 7: 提交**

```bash
git add Touch_Client/force/ForceCompensation.cpp Touch_Client/force/ForceCompensation.h Touch_Client/tests/test_force_compensation.cpp
git commit -m "feat(gate): 力矩通道不再投票 (实测无一致态), 掩码收成一处定义; 默认报告不再声称掩码"
```

---

### Task 5: 验证"零偏漂移检查"首次真的运行

**为什么:** 那个检查**等闸门放行**才做事，而闸门一直在拒 -> **它至今跑的全是"未做"那一支**。
Task 2 + Task 4 之后闸门会放行，于是**一条从未在现场跑过的路径会被打开**。
方向是好的，但它没有履历 —— 要显式验一次。

**Files:**
- Modify: `Touch_Client/tests/test_force_compensation.cpp`（或该检查所在的测试套件）
- Create: `Docs/superpowers/specs/2026-09-21-gate-reference-prereq.md` 追加一节（验证记录）

- [ ] **Step 1: 找到那个检查，读清它放行后会做什么**

搜 `ZERO_CHECK_GUARD_WAIT_MS` 与"零偏漂移检查"。**读它的写盘副作用**：
它会写哪个文件、写什么、失败时报什么。

- [ ] **Step 2: 为"放行"那一支写一条用例**

构造 `guardState() == OK` 的前置状态，驱动该检查走完，
断言：**它真的产出了结论**（不是"未做"），且产出的是它自己算出来的数。
**⚠ 若它需要真实时间/真实采样，就用它的可注入点（若有）；没有可注入点就记为"无法单测"，
改用 Step 3 的现场验证，并在报告里写明"无单测覆盖"。**

**★ 2026-09-21 修订（用户拍板）—— 上面这条退路【不适用】，Step 2 与 Step 3 都改**

**为什么改（已核查代码，非推测）:** 该检查是 `main.cpp` 里的 file-static（`runZeroDriftCheck`），
由主循环用**真实时间**（`GetTickCount`）驱动，读 `appState.forceData` 与 `ForceCompensation::guardState()`，
**没有可注入点**；它**不写任何文件**，只有 stdout。而 Step 3 的现场验证已随硬件工作一起**延后**
⇒ 按原退路走，这条**从未在实机跑过**的路径会落成【零验证】。**用户不接受。**

**Step 2 改为（抽纯函数 + 逐分支单测）:**
- 把判定逻辑抽成**纯函数**：输入 = `guardState`、三轴均值、已用时长 ms、样本数、阈值；
  输出 = 结论（正常 / 超阈 / 样本不足 / 未做）+ 该说的话。**时间与全局状态留在调用侧。**
- **每一个分支都要有用例**：放行后正常、放行后超阈、闸门拒绝满等待期 ⇒ 未做、
  以及**样本不足**那一支。
- 断言"**它真的产出了结论**（不是『未做』）"，且结论里的数**是它自己算出来的**。

**★ 同期修一条静默路径（用户指令）:** 样本数不足时当前**设完 `g_zeroCheckDone` 就直接返回**
—— 既不报结论、也不报"没查"。**这与该文件自己写的原则冲突**（其上文原话："『没查』必须有句话"）
⇒ **改成明说**（如"样本不足, 本次不作结论"），**并加用例钉住**。

**Step 3:** **延后**（与夹具重采同批）。⇒ **本任务交付的是"可单测 + 有覆盖"，
【不是】"已在实机验证过"。报告里必须明写这一点，且不许把"有单测"说成"已验证"。**

- [ ] **Step 3: 现场验证（需要机械臂在运行 —— 只读，不动负载）**

```
cmd.exe //c "D:\Projects\Touch\Touch_Client\build.bat"
```
启动客户端，等闸门放行后观察：零偏漂移检查是否给出结论、结论是什么。
**记进 `2026-09-21-gate-reference-prereq.md`。**
⚠ **不要杀运行中的 `Touch_Client.exe`**；若要构建，先确认它没在跑。

- [ ] **Step 4: 提交**

```bash
git add Touch_Client/tests/test_force_compensation.cpp Docs/superpowers/specs/2026-09-21-gate-reference-prereq.md
git commit -m "test(force): 零偏漂移检查在闸门放行后的分支首次被覆盖"
```

---

### Task 6: A 第三行守门 —— 先查清 z 缺口

**为什么这是诊断任务而不是编码任务:** 开放项 C 记的是"z 轴的缺口 ~38~64 N"。
**在查清这个缺口的来源之前，"守门判据该长什么样"无从设计** —— 现在写判据就是猜。
用户约束："任何猜测以及偏差都不可取"。

**Files:**
- Create: `Docs/superpowers/evidence/2026-09-21-z-gap-report.md`
- Modify: `Docs/superpowers/specs/2026-09-19-remaining-workflow.md`（把开放项 C 的结论写回去）

- [ ] **Step 1: 把 z 缺口的现有证据收齐，逐条注明出处**

搜"缺口"、`Fz`、`A` 的第三行、开放项 C。**只用仓库里已有的记录**，
每条都写上它出自哪个文件。
⚠ 若某条主张在仓库里**找不到出处**，**不要写它** —— 本项目的账就是这么欠下的。

- [ ] **Step 2: 列出能解释 38~64 N 的候选，并给每条一个可判别的signature**

至少考虑（不限于）：
- `A` 的第三行（`Fz` 对重力方向的响应）本身估错
- `@576` 的 `Fz` 与 `@720` 的 `Fz` 是两个不同的量（事实 1 的直接推论 —— **若成立，缺口可能是"比错了对象"**）
- 换帧（原点/平移）在 z 方向的影响

**每条都要写出"若它是真的，数据上该看到什么"。** 写不出可判别 signature 的候选，
标为"不可判别"，不要留成开放项。

- [ ] **Step 3: 用已有数据逐条判**

**优先用 Task 1 复核过的 `@720` 数据**（`calib_poses.txt` 里带 `F720*` 列的那几轮）。
**不新增机械臂动作。**

- [ ] **Step 4: 写下判决 —— 允许的结论只有三种**

1. **缺口是"比错了对象"** -> 那它不是模型缺陷，Task 2 已经把它治了 -> **关闭开放项 C**
2. **缺口是 `A` 第三行估错** -> 那 **Task 2 之后它仍然存在**，且 `Fz` 不投票会把它放行 ->
   **给出一个具体的守门判据设计**（要能说清：判什么量、门限怎么来、误拒率多少）
3. **判不出来** -> 明写"判不出来"，并给出**下一个能判别的测量**（要它不需要动负载）

- [ ] **Step 5: 提交**

```bash
git add Docs/superpowers/evidence/2026-09-21-z-gap-report.md Docs/superpowers/specs/2026-09-19-remaining-workflow.md
git commit -m "docs(gate): 查清 A 第三行/z 缺口 —— 结论与它取代的旧账"
```

**⚠ 若 Step 4 落在第 2 或第 3 支，不要顺手实现守门判据 —— 另开计划。**
本任务的交付物是**判决与设计**，不是代码。

---

### Task 7: 参考量"不可用"时 fail-closed（**用户 2026-09-21 指令追加**）

**为什么有这个任务:** 闸门现在在参考量读到 ~0 时【放行】—— 因为本地模型自己的残差就在容差内
（判据是 `comp[i] − 参考量`，参考量为 0 时判据退化成"本地输出是否在自己容差内"）。
"零"既可能是"真的没有外力"，也可能是"这一路没数据 / 已失效"，**两者当前不可区分**
⇒ **对"没有信息"放行 = fail-open**。用户指令：加存在性守卫，**不可用时不放行**。

**⚠ 复审已澄清的边界（写下来，免后人夸大）:** 生产链路上 `RelayCore` 在**同一帧、同一把锁**里
一起填 `raw[]` 与 `tcpForce[]` ⇒ "通道其实有数但为零"在**实机目前不可达**；
它只在**回放 / 夹具**路径出现。⇒ 本任务是**【防紧】性质，不是修一个正在发生的 bug**。
**报告里不许写成"修了一个活 bug"。**

**判据不许猜:** 依据只能取自**已有信号** —— `AppState::ForceData::sixForceOnline`（30004 帧 @1037）
与既有超时常量 `Config::FORCE_STALE_MS`。**不许新造一个凭感觉的门限。**
若你认为必须新增门限，**停下报告**，说明它为什么不能由既有信号导出。

**★ 最要紧的一条设计约束（用户已明确否掉"把未验证的东西变绿"）:**
本守卫**不得**让 `test_payload_calibration.cpp` 的 `runtime_consistency_guard_replay`
那条**红线断言因错误的原因变绿**。那四份夹具**缺 `@720` 列**（已核实：四个文件的表头都没有
`F720*`），若守卫把"参考量不可用"判成"拒绝"，那条"必须拒绝"的断言就会**通过**，
而它**仍然什么都没验证**。
⇒ **"参考量不可用"必须与"不一致"可区分**（独立状态或独立报告字段），
该用例必须据此更新为"参考量不可用"，且**不得**被计成原断言得到满足。
**绕过这一条就是绕过用户明确否决的东西。**

**⚠ 加新状态有涟漪:** `GuardState` 若新增枚举值，**不许**让别处拿
`static_cast<int>(guardState())` 去比字面量（本项目记过这条账）——
必须经 `guardStateName()` / 既有的状态表，并同步 `RelayCore` 的映射与报错码
（选择诚实的 `RobotErrorCode`，并把注释一起更新）。

**Files:**
- Modify: `Touch_Client/force/ForceCompensation.cpp`
- Modify: `Touch_Client/force/ForceCompensation.h`
- Modify: `Touch_Client/tests/test_force_compensation.cpp`
- Modify: `Touch_Client/tests/test_payload_calibration.cpp`（把该用例改判为"参考量不可用"）
- Modify: `Docs/superpowers/specs/2026-09-21-gate-reference-prereq.md`（记录决策与依据）

**Interfaces:**
- Consumes: Task 2 的 `guardReferenceValue`；Task 4 的掩码（若先做则本任务依赖它）
- Produces: 一个"参考量可用性"判据 —— **必须能在无机械臂的单测里驱动**
  （直接置 `sixForceOnline` 为非在线、或让参考量停在零并越过 `FORCE_STALE_MS`）

- [ ] **Step 1: 写失败的测试** —— 参考量不可用时**不得**给出"通过"；且该结果**必须与
      `INCONSISTENT` 可区分**（用一条用例同时钉住这两件事）。
- [ ] **Step 2: 跑它，确认失败**（构建脚本只编译，**必须单独执行 exe**）。
- [ ] **Step 3: 实现**，依据只用既有信号，并把"为什么是这些信号"写在该判据旁边。
- [ ] **Step 4: 更新 `test_payload_calibration.cpp` 的那条用例**：它现在是"参考量不可用"，
      不是"不一致"。**明确写下它仍然没有验证过新参考量**（这是那条红线断言的现状）。
- [ ] **Step 5: 跑全部用例 + 完整构建**。Expected: `test_force_compensation` 全 PASS；
      `test_payload_calibration` 变为 **70/0**，**但报告里必须写明那个 0 不代表原断言通过了**
      —— 它被改判成了"参考量不可用"。**在任何地方都不许说"全绿"。**
- [ ] **Step 6: 提交**
      `feat(gate): 参考量不可用时 fail-closed, 并与"不一致"可区分`

---

## 完成本计划后仍然存在的（明写，避免被读成"都解决了"）

| 项 | 状态 |
|---|---|
| **`A` 的第三行** | Task 6 给出判决；若落在第 2/3 支则**仍未受守** |
| **质量换帧差额** | **未动**。Task 2/4 只改"怎么判"，不改"发什么"。下发的 `m` 仍是"测量原点以下"的量代入"整条链"的槽位，差额量未定 |
| **下发不持久** | **未动**。活路径不写 `payload_calib.json`，重启后连接时序仍发旧值 |
| **力矩自带偏置** | **未修，也不打算修**（事实 3：改不动、标不掉）—— 它退回"只报不判"的诊断 |
| **`@720` 力矩自噪声** | 同姿态秒级复采 0.06~0.13 N·m，**未改善** —— 只报不判之后它不再致命 |
| **Task 10 闭环** | 判据仍待重写（本计划只重写了**闸门**的判据） |
| **★ 重采带 `@720` 的夹具** | **本计划的【收口必做项】，不是可选项。** `tests/fixtures/calib_poses_2026-09-19*.txt` 四份**表头都没有 `F720*` 列**（已核实）—— 它们采于 2026-09-19，而记录程序 2026-09-20 才加 `@720`。于是 `test_runtime_consistency_guard_replay` 的"必须拒绝"那半边**问了数据答不了的问题**，被**有意保留为红**（基线 `test_payload_calibration` 69/1、`test_force_compensation` 26/0）。**在拿到带 `@720` 的夹具之前，不许声称闸门在新参考量上被验证过，也不许说"全绿"。** 重采方式与原采法相同：笔尖悬空、只改姿态、**不动负载**。它同时能回答"那条断言该拒绝还是该放行"—— **正确答案很可能是"放行"**（这些采集真值外力 ≈0），但**没有数据就不能断言**。 |
| **★ `@720` 的符号约定** | **未验证，且不在本计划内**。仓库里没有任何文件给出它。静止时 `comp≈0` 且 `@720≈0` ⇒ "两者一致"是零信息量 ⇒ **换参考量只在静止态被验证过**。若符号相反，真实外力下闸门会看到 ~2× 偏差。验证需动机械臂。依据：Task 1 的 `Docs/superpowers/specs/2026-09-21-gate-reference-prereq.md` §4.2。 |
| **★ 零偏漂移检查的现场验证** | **延后**（与夹具重采同批）。Task 5 只做到"抽出纯函数 + 逐分支单测"——**那是"可验"，不是"已验"**。这条检查**从未在实机跑过**，且它比闸门更紧（它 `0.5 N` 就报警，而闸门到 `1.2464 N` 才拒）⇒ 放行后它可能立刻报出漂移，那正是它在起作用，不是故障。 |
| **测试会往 CWD 写文件** | 两个套件按工作目录写标定状态（2026-09-21 实测：仓库根的 `calib/force_calib.json` 被一次测试运行改写）⇒ **"跑测试"不是只读操作**，最终复审与后续自动化都要知道这一点。 |

## Self-Review

- **覆盖:** 用户要求的三件 —— 换参考量（Task 2 + Task 3）、力矩不投票（Task 4）、
  补 A 第三行守门（Task 6）—— 都有任务。另加 Task 1（前置复核）与 Task 5（首次运行的路径）。
- **占位符:** 无 TBD / "适当处理" / "类似 Task N"。Task 6 是**诊断任务**，
  交付物明确是"判决与设计"，不是代码 —— 这是刻意的，不是偷懒。
- **符号一致:** `guardReferenceValue` 在 Task 2 定义、Task 3 引用；
  `g_guardEmaDiag` / `g_guardSeededDiag` 在 Task 2 改名、Task 2 Step 5 的打印分支沿用；
  `GuardReport::voted` 在 Task 4 改初值、同任务改它的测试。
- **本计划不含行号与计数**（Global Constraints 第 1 条）。
