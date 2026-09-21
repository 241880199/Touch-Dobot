# Task 4 报告: 力矩不投票 —— 掩码一处定义 + 报告不撒谎

分支 `feat/pen-clamp-redesign`。计划: `Docs/superpowers/plans/2026-09-21-*.md` 的 Task 4。

## 1. 实现了什么

生效掩码从 `{true,true,false,true,true,true}` 改为 `{true,true,false,false,false,false}` ——
**只有 `Fx` / `Fy` 投票**; `Fz` 与力矩三个分量**照算、照报, 但不参与判决**。

改动的六处:

1. `Touch_Client/force/ForceCompensation.cpp` —— 掩码改值, 并把两条理由与**代价**写在它旁边
   (见 §4); 判决循环与逐通道表的文字改成"投票通道/照报不判"; 放行那一行从
   "逐通道一致"改成"在【投票通道】上一致"。
2. `Touch_Client/force/ForceCompensation.h` —— `GuardReport::voted` 类内初值改全 `false`
   并注明"尚未填充"; `GuardState` 的 `OK` / `INCONSISTENT` 注释改成"投票通道";
   顶部判据段补上"投票通道由 `g_guardVote` 一处定义"。
3. `Touch_Client/safety/RobotError.h` —— `ERR_FORCE_INCONSISTENT` 的注释从"逐通道对不上"
   改成"在某个【投票通道】上对不上", 并指向两处唯一定义。
4. `Touch_Client/config/Config.h` —— 三处已经变成假话的文字改掉 (见 §6)。
5. `Touch_Client/tests/test_force_compensation.cpp` —— 三条新用例 + 一条旧断言改写 (见 §7)。
6. `Touch_Client/tests/test_payload_calibration.cpp` —— 第三份掩码实现消掉 (见 §3)。

## 2. TDD Evidence

### RED

命令:

```
cmd.exe //c "D:\Projects\Touch\Touch_Client\tests\build_force_comp_test.bat"
D:\Projects\Touch\Touch_Client\tests\test_force_compensation.exe
```

输出 (改掩码之前):

```
  guard_fz_reported_but_not_voted... FAIL: rep.voted[3] == false && rep.voted[4] == false && rep.voted[5] == false
  guard_moment_channel_reports_but_does_not_vote... FAIL: ForceCompensation::guardState() == ForceCompensation::GuardState::OK
  guard_force_channels_still_vote... PASS
  guard_default_report_claims_no_mask... FAIL: rep.voted[i] == false

Results: 25 passed, 3 failed
```

三条失败**都是预期的**, 且各自失败在预期的那一句上:

- `guard_fz_reported_but_not_voted` —— 它读 `guardReport().voted`, 而那时掩码第 3/4/5 位
  还是 `true`。失败在掩码那三条 CHECK, 不是别的。
- `guard_moment_channel_reports_but_does_not_vote` —— 力矩那时候还投票, 所以注入的 1.20
  (>> tol_M 0.03) 让闸门拒绝, 第一句 `state == OK` 就断了。**它红的正是"力矩还在投票"这件事**。
- `guard_default_report_claims_no_mask` —— 类内初值那时抄着旧的六位字面量, 所以默认构造的
  报告声称了一份真实掩码。

`guard_force_channels_still_vote` 在 RED 阶段就 PASS —— 这是**对的**: 它是防止掩码被改过头的
那一半 (`Fx/Fy` 必须仍然投票), 改动前后都该绿。它绿不说明任何东西被测到了, 它只在掩码被
改成全 `false` 时会红。

### GREEN

改掩码 + 改类内初值 + 消掉第三份实现之后:

```
Results: 28 passed, 0 failed          (test_force_compensation)
69 passed, 1 failed                   (test_payload_calibration, 与基线逐字相同)
Build OK.                             (完整构建)
```

用例数从 26 变 28: 新增 3 条, 删掉 1 条 (旧 `test_guard_moment_channel_votes`)。

## 3. 三份掩码实现的处置

| 位置 | 处置 | 现在靠什么不漂 |
|---|---|---|
| `ForceCompensation.cpp` 的 `g_guardVote` | **改值**, 并在注释里标成"掩码的唯一一份实现" | 它自己就是那份定义 |
| `ForceCompensation.h` 的 `GuardReport::voted` 类内初值 | **改成全 `false`**, 注释写明这是"尚未填充"、不是掩码 | 全 `false` 不可能被误读成一份真实掩码; 真值由 `guardReport()` 填 |
| `test_payload_calibration.cpp` 里 `test_runtime_consistency_guard_replay` 的函数内局部 `vote[6]` | **删掉字面量**, 改成从生产 API 读 | 见下 |

第三份的做法:

```cpp
ForceCompensation::GuardReport maskRep;
ForceCompensation::guardReport(maskRep);
const bool   (&vote)[6] = maskRep.voted;   // 引用绑定 —— 不复制, 所以不会成为第二份
const double (&tol)[6]  = maskRep.tol;
```

用**引用绑定**而不是复制, 是为了让"这份测试手里的掩码"与"生产判决用的掩码"在**同一个对象**
上 —— 它连"忘了同步"这个动作都做不出来。旁边的 `tol[6]` 映射 (把两个容差常数摊到六个通道)
做同样的处置: 它也是从 `guardReport().tol` 读, 不再写第二份。

实测证据 (跑出来的那一行, 改之前它不可能这样打):

```
pose 1 逐通道 (EMA 差, 超限倍数):  Fx +0.010(0.0x)  Fy +0.009(0.0x)  Fz=不投票  Mx=不投票  My=不投票  Mz=不投票
```

四个 `不投票` 是从生产掩码读出来的, 不是这份测试自己断言的。

## 4. 写在掩码旁边的两条理由 (与代价)

`Fz` 那一段**保留原有的"未复测"标记, 并且加粗**了:

- 依据出处 (z 响应实测秩 2, 奇异值 0.212/0.201/0.008, 计划书) 原样保留;
- 现状标成 `⚠⚠ 【未复测 —— 这一条到现在仍然是"未验", 不许读成"已验"】`, 并写明
  "把这段删掉或说成'已验'就是让后人照着一个未复测的前提去改掩码"。

力矩那一段 (新增), 按简报的强制内容写全:

- 实测数字 (换参考量后仍差 `Mx +0.061 / My +1.147 / Mz +1.316`, 容差 0.03);
- **不是负载效应**的判别 (约 90% 姿态无关, 而负载误差的残差必然随姿态变);
- 偏置**在漂** (`Mx` 20 分钟漂 0.55 N·m) ⇒ 既改不动也不能标定掉;
- 重复性 (0.06~0.13 N·m) 比容差大 2~4 倍;
- ⇒ 不存在"一致"态; 判决全或无 ⇒ 力通道一起断; 触觉只消费三个力分量 ⇒ 投票换不到东西;
- `⚠ 力矩不是被删掉`: 照算照报照给 `F|` 帧, 仍是质心与惯量的唯一测量窗口;
- `⚠ 代价必须记账`: `Fz` 早已不投票、力矩原本在名义上兜着 `A` 的第三行 ⇒ 现在无人兜,
  且**"把力矩的票加回来"这条路已被实测否掉**, 要补只能新增独立的 z 判据 (开放项 C)。

同时把 `Config.h` 里那段"力矩门理论上能给 z 力当后盾"的算法标注为
**"现在算的是一道不存在的门"** (数字一个没改, 只说清它的地位变了)。

## 5. `test_payload_calibration` 的计数

- 基线 (改动前实测): **69 passed, 1 failed**
- 改动后: **69 passed, 1 failed**

**数字一动没动**。那一条故意红的断言仍是同一条、同一个原因:

```
FAIL: 12:38 pose 1 竟然放行了 (state=OK)
```

它的 ⚠⚠ 说明原样保留 (四份夹具没有参考量那一路的列 ⇒ 那半边问的是数据回答不了的问题)。
本次改动**没有**让它变绿, 也没有让任何一条新的用例变红。

两个套件都**不是"全绿"**, 本报告不使用这个说法。

## 6. 其它被改掉的、已经变成假话的文字

- `Config.h` —— 原文写着"`Mx/My/Mz` **此刻同样投票** —— '力矩不再投票'是 Task 4 的事"。
  Task 4 已做, 这句现在是假的。改成"当前投票掩码**只有** `Fx/Fy`", 并说明 `tol_M` 因此在
  **判决里不再被消费** (只作为该通道读数的尺度印出来)。
- `Config.h` —— 力矩容差那段里"若它是真的, 力矩通道会在 Task 8 之后**永远拒绝**"这条风险
  已**不再由闸门承担** (力矩不投票了, 它不可能再让闸门拒绝); 明确写出这一点并说明
  **`tol_M` 的数值保持原值**。
- `Config.h` —— "**不**改投票掩码: 复审判定'Fz 不投票'这个取舍本身可接受" 改写成:
  要补 z 的洞只能新增独立判据, **不能靠把力矩的票加回来** (那是实测否掉的)。
- `RobotError.h`、`ForceCompensation.h` 的枚举与判据说明 —— "逐通道"改成"投票通道"。
- `ForceCompensation.cpp` 的 `setGuardState` —— 逐通道表加了表头说明"标着【不投票】的那四行
  照报但不算", 表格里不投票那一支的标签从 "z 通道的判据缺口未查清" 改成中性且**不复述理由**
  的说法 (原文案只对 `Fz` 成立, 直接复用会让力矩三行也跟着说"z 缺口", 那就是另一句假话)。
- `test_payload_calibration.cpp` 的 `"=不投票(秩2)"` 同样改成 `"=不投票"` (秩 2 只对 `Fz` 成立)。

## 7. 旧的钉掩码用例

**删除**了 `test_guard_moment_channel_votes` (它的名字与断言都是为旧掩码写的:
`fd2.tcpForce[3] = 0.20 - 0.12` ⇒ 差 0.12 > 容差 0.03 ⇒ 断言 `INCONSISTENT`)。

**理由**: 它断言的行为已被实测否掉 (Task 4 的全部内容就是"力矩不再投票"), 而它的名字
说的是"votes" —— 保留它就等于留一条名字与断言相反的用例 (简报明令禁止)。它换来的是简报
Step 1 的三条新用例, 这个替换写进了提交信息。

**另外**改写了 `test_guard_fz_reported_but_not_voted` 里钉掩码的那一条断言 ——
它原来写 `rep.voted[3] == true && rep.voted[4] == true && rep.voted[5] == true`,
那是**第二处钉旧掩码的断言**(简报没有点出这一处, 是我在改动前通读时找到的; 不改它这条
用例会在 GREEN 阶段假红)。现在它断言三个都是 `false`, 于是这一条与相邻两条一起把**生效
掩码的完整形状** (`Fx/Fy` 两个 `true`、其余四个 `false`) 钉住。

三条新用例都已注册进 `main()` 的显式调用表 (未注册 = 从不运行):

```
test_guard_moment_channel_reports_but_does_not_vote();
test_guard_force_channels_still_vote();
test_guard_default_report_claims_no_mask();
```

## 8. `Fz` 的前提标记

**没有恢复、也没有删除那个标记。** 那一段的前提 (`z` 响应实测秩 2) 是在**旧判据通道**上量的,
Task 2 已明说它没有跟着复测 —— 本次**没有在新参考量上复测它** (那需要新的采集, 不在本任务内)。

处置: 保留全部原有出处, 并把状态标记**加重**成 `⚠⚠ 【未复测 —— 这一条到现在仍然是"未验",
不许读成"已验"】`, 另外写明"把这段删掉或说成'已验'就是让后人照着一个未复测的前提去改掩码"。
`test_payload_calibration.cpp` 里那一行打印同样保留
`(⚠ 这条秩 2 是在【旧判据通道】上实测的, 换到参考量之后【未重测】)`。

## 9. 力矩仍然到达输出路径 (确认)

- **算**: `step()` 的 `comp[]` 六个分量公式一字未改。
- **报**: `guardReport()` 的 `ema[]` / `tol[]` / `voted[]` / `exceeded[]` 仍然逐通道填;
  拒绝时那张逐通道表仍然把六行都印出来 (`Mx/My/Mz` 标着 `【不投票】` 但**带着数值**)。
- **传**: 闸门放行时 `for (int i = 0; i < 6; i++) fd.compensated[i] = comp[i];` 一字未改。
- **滤波**: `ForcePipeline.cpp` 的 `fd.filtered[i] = g_filters[i].step(fd.compensated[i]);`
  对六个分量都跑。
- **给 MATLAB**: `RelayCore.cpp` 的 `F|` 帧发的是 `filtered[0..5]` —— 含力矩三个分量。
- **触觉**: `ForcePipeline.cpp` 的 `hapticOut` 只由 `filtered[0..2]` 推 (三个力分量) ——
  这正是"力矩投票换不到任何东西"的依据。

用一个会响的断言把"力矩那一半仍然被算出来、被报出来"钉住了:
`test_guard_moment_channel_reports_but_does_not_vote` 里
`rep.ema[3] == -1.20` (注入的参考量力矩差), 以及 `rep.exceeded[3] == false`。

## 10. 命令与结果

| 命令 | 结果 |
|---|---|
| `cmd.exe //c "D:\Projects\Touch\Touch_Client\tests\build_force_comp_test.bat"` | `BUILD_EXIT=0` |
| `D:\Projects\Touch\Touch_Client\tests\test_force_compensation.exe` | RED `25 passed, 3 failed` → GREEN `28 passed, 0 failed` |
| `cmd.exe //c "D:\Projects\Touch\Touch_Client\tests\build_payload_calibration_test.bat"` | `BUILD_EXIT=0` |
| `D:\Projects\Touch\Touch_Client\tests\test_payload_calibration.exe` | `69 passed, 1 failed` (基线与改动后相同) |
| `cmd.exe //c "D:\Projects\Touch\Touch_Client\build.bat"` | **`Build OK.`** |

`run_tests.bat` **没有**被当作证据引用 (它不覆盖全部套件)。
两个套件都是**分别执行** `.exe` 得到的上述数字, 不是只看构建成功。

## 11. 改动文件

- `Touch_Client/force/ForceCompensation.cpp`
- `Touch_Client/force/ForceCompensation.h`
- `Touch_Client/config/Config.h`
- `Touch_Client/safety/RobotError.h`
- `Touch_Client/tests/test_force_compensation.cpp`
- `Touch_Client/tests/test_payload_calibration.cpp`

## 12. 自查发现 / 顾虑

1. **注入值仍然有判别力** (简报要求确认, 不是假设):
   - `fd.tcpForce[0] = 1.10` 对 `compensated[0] = 1.05` ⇒ 差 0.05 < `tol_F` 1.2464 ⇒ 放行分支成立;
   - `fd.tcpForce[0] = 5.00` ⇒ 差 5.00 > 1.2464 (余量 ~4 倍) ⇒ **仍然判别**;
   - `fd.tcpForce[3] = 1.20` ⇒ 差 1.20 vs `tol_M` 0.03 (40 倍) ⇒ 力矩那一侧差别巨大。
   - 三条用例都加了 `CHECK(x < / > Config::FORCE_GUARD_TOL_*)`, 于是"容差以后被改到让用例
     退化成空壳"这件事会**先红**, 而不是安静地失去判别力。**没有改任何容差**。
   - 过程中我第一版把 `compensated[0]` 断言写成 `-0.05` —— 那是错的 (`-0.05` 是 **EMA 的差**,
     不是补偿输出)。改成真的注入一个外力 `sixForceRaw[0] = 1.05`、并同时断言
     `compensated[0] == 1.05` 与 `ema[0] == -0.05`。这样这一条能分辨"放行"与"拒绝"
     (拒绝时 `compensated` 恒 0, 它必定失败)。
2. **掩码现在只有 `Fx/Fy` 投票 ⇒ 闸门比原来松**。这是本任务的**目的** (让力通道能通),
   但代价是 z 方向的洞"更大了一点" (原来注释里的"力矩兜得住 z"连名义上的兜底都没了)。
   已按简报要求把这个代价**记账**在掩码旁边、`Config.h` 与 `test_payload_calibration` 的打印里。
   **这是我建议复审重点看的一处**: 掩码变松之后, 安全性靠的是 `Fx/Fy` 那两路 + 独立的
   约束力路径 (位置驱动, 不受闸门影响), 这个组合是否够, 属于所有者定夺的范围。
3. **`g_guardTol` 是容差映射的第二份实现** (一份在静态初值, 一份在 `init()` 里按 `Config`
   重填)。它**不在本次任务范围内** (简报只点了掩码), 两份目前逐位相同, 所以没有实际风险 ——
   但它是同一类"改一处忘一处"的结构, 留着给后续任务。
4. `test_payload_calibration.cpp` 的第三份掩码改从生产 API 读之后, 那份回放**在掩码再变时
   会自动跟着变** —— 这是要的效果, 但也意味着它不再能"独立地"发现掩码被改错了。这是简报
   明确选择的取舍 (第三份字面量会安静地建模旧闸门, 那更坏)。
5. `Touch_Client.exe` 在本次全程**没有在运行**, 构建未报"文件被占用", 没有杀任何进程。

---

# 附: 复审修复 (2026-09-21, 同一任务的收尾)

复审结论是"逻辑正确" (一份掩码、一个判决循环、没有影子投票路径、全或无置零完好),
但点出 1 条 Important + 3 条 Minor。**掩码、容差常量、判决语义一个字都没动**。

## A. 改了什么

| # | 文件 | 改动 |
|---|---|---|
| 1 (Important) | `force/ForceCompensation.cpp` | 力矩那段注释里的假话: 原文称触觉与 F 帧 (F 竖线帧) 都只消费前三个分量。F 帧那一半是假的 —— `RelayCore` 把 `filtered[0..5]` 全格式化进去, 而 `ForcePipeline` 每个 `filtered[i]` 都取自 `compensated[i]`; 只有 `hapticOut` 是前三个推的。删掉"F 帧与"这半句, 并把"只有触觉那一路如此"写明白。 |
| 2 (Minor) | `force/ForceCompensation.cpp` | 判决循环里 `std::isfinite` 移到 `if (!g_guardVote[i]) continue;` **之上** —— 非有限值与投不投票无关。 |
| 3 (Minor) | `config/Config.h` | 闸门段开头那句从"逐通道比较 … ⇒ 不一致就拒绝"改成明确"只有投票通道进判决", 指向 `g_guardVote` 那一处实现, 不在那里复述掩码 (下文那段 `⚠ 2026-09-21 起力矩…` 保持不动)。 |
| 4 (Minor) | `force/ForceCompensation.cpp` | 逐通道表的说明行不再用散文复述掩码 (原来写着"这四行【不投票】(Fz + 力矩三个分量)", 是硬编码的份数与名单, 会随掩码漂)。改成指向 `g_guardVote`。 |
| 5 | `tests/test_force_compensation.cpp` | 新用例 `test_guard_nonfinite_refuses_even_on_nonvoting_channel`, **已注册进 `main()` 的显式调用表**。 |

**没有改**: `g_guardVote[6]` 的字面量 (逐字符相同)、`FORCE_GUARD_TOL_FORCE_N` = 1.2464、
`FORCE_GUARD_TOL_MOMENT_NM` = 0.03、`g_guardTol` 的填法、判决的"全或无"语义。
`git diff` 里对这三行的改动数为 **0** (用 `grep -E "^[+-].*(g_guardVote\[6\]|FORCE_GUARD_TOL_)"`
在整份 diff 上筛过, 零命中)。

那处**重复的容差映射** (`g_guardTol` 静态初值 + `init()` 里重填) 复审判为可接受,
**按简报要求没有动**。我也认为该保持现状: 静态初值在 `init()` 之前让闸门 fail closed,
而不是拿 0 当容差去比较 —— 这个作用 `init()` 的重填替代不了。

## B. 新用例的 RED / GREEN (TDD)

用例形状 (非空壳的理由): 它**同时**喂"极大但有限"的同一个通道当对照 —— 若哪天该通道被改成
投票通道, 只喂 NaN 时两条分支都会拒绝, 用例会绿着什么都没测到。三条支路:

- (a) 对照 —— 同一通道喂**极大但有限**的差 (1.20, 是 `tol_M` 的 40 倍) ⇒ **放行**,
  且 `compensated[0] == 0.5` (没被置零)。这一条证明该通道**确实不投票**;
- (b) 同一通道换成 `NaN` ⇒ 必须 `INCONSISTENT` (投票通道 Fx 完全正常, 拒绝只可能来自那个 NaN);
- (c) 同 (b) 换成 `Fz` —— 它在力矩之前就已经不投票, 所以这是那个**既有**缺口。

### RED (把 `isfinite` 临时挪回 `continue` 之下 = 改动前的顺序)

构建: 同下, `BUILD_EXIT=0`。执行 `test_force_compensation.exe`:

```
  guard_nonfinite_refuses_even_on_nonvoting_channel... FAIL: ForceCompensation::guardState() == ForceCompensation::GuardState::INCONSISTENT
Results: 28 passed, 1 failed
```

**恰好一处失败, 且落在 (b) 的 `INCONSISTENT` 断言上** —— 即旧顺序确实把一个不投票通道上的
NaN 放行了 (它没有退化成"两条路径都因无关理由拒绝"的空壳: (a) 的放行断言在 RED 下也是过的)。

### GREEN (恢复 `isfinite` 在 `continue` 之上)

```
  guard_nonfinite_refuses_even_on_nonvoting_channel... PASS
Results: 29 passed, 0 failed
```

## C. 两个套件的计数 (构建后**分别执行** `.exe`)

| 套件 | 改动前 | 改动后 |
|---|---|---|
| `test_force_compensation.exe` | 28 passed / 0 failed | **29 passed / 0 failed** (新用例 +1, 是预期内的) |
| `test_payload_calibration.exe` | 69 passed / 1 failed | **69 passed / 1 failed** (那条**故意红着**的断言与它的 ⚠ 注一字未动) |

`test_payload_calibration` 那一条失败在改动前后**是同一条**:
`FAIL: 12:38 pose 1 竟然放行了 (state=OK)`。**不称这两个套件"全绿"** —— 它不成立。

全量构建: `cmd.exe //c "D:\Projects\Touch\Touch_Client\build.bat"` ⇒ **`Build OK.`**
(`Touch_Client.exe` 全程未在运行, 构建未报占用, 没有杀任何进程)。

## D. Fix 2 带来的一处**故意的行为变更**: `Fz` 那半边也被一并修好

`Fz` 在这次任务之前就**已经不投票**, 而 `isfinite` 一直排在 `continue` 之下 ⇒ **`Fz` 上的
非有限值原本就是放行的**。把那两行对调, 等于把 `Fz` 这个**既有**缺口一并关上了。

- 力矩三个分量: 本次任务把 `isfinite` 的漏法从 1 个通道**扩到了 4 个**, 修复把它们收回去;
- `Fz`: **不是本次任务引入的**, 是顺带修好的旧缺口。
- 方向上两边都是**收紧** (更多数据被拒), 与"安全起见不符合时还是要拒绝"一致;
  它**不影响**任何投票通道的容差比较, 也不改变放行时 `compensated` 的取值。

## E. 遗留 / 顾虑

1. `F|` 帧那条假话的**根因**是"引用另一个模块的行为却凭记忆写" (与记忆中
   `verify-premises-at-the-source` 那条同源)。我这次按 `RelayCore.cpp` / `ForcePipeline.cpp`
   的实际代码核过才落笔, 并在注释里点明了"说成 MATLAB 收不到力矩 ⇒ 下一步就会删掉力矩那一段"
   这条**错误结论是怎么来的**, 免得它再被写回去。
2. `test_payload_calibration` 里那条**故意红着**的断言仍在 (属于该套件的既有状态, 简报明令不动)。
3. 测试运行会写 `calib/force_calib.json` (工作目录相关, `CalibStore::fileFor`)。**该文件未被
   纳入本次提交** (它是未被跟踪的 `calib/`, 提交里只有本任务有意改的 3 个文件)。
4. 未解项与本次修复无关, 不因本次改动而变化: `@720` 与 `@1304` 反相关的符号约定、
   `c_s` 可信度边界、开放项 C (z 轴缺口)。
