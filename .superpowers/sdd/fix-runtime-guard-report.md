# 运行时一致性闸门 —— 复审遗留修复报告 (2026-09-19)

**被修的提交:** `673524e`（HEAD，运行时一致性闸门）
**起点:** `.superpowers/sdd/runtime-guard-report.md` 的复审结论 —— 无 Critical，
Critical 的修复已端到端走过一遍（未标定那条路到不了操作员的手上；EMA 第一帧就用瞬时差播种，
所以既有的不一致在第 1 帧就被判）；复审复现了报告里每一个数字。
**本轮修的:** 3 条 Important + 2 条 Minor + 5 条零碎。**没有动判决逻辑、容差数值、投票掩码。**

> **行号一律是【本轮修复之后】的行号。** 原报告 §1 那张表是提交时的行号，那里已加了一行提示。
> 复审给出的行号（`ForceCompensation.cpp:624` / `ForceCompensation.h:89` / `:620`）与本轮
> 起点文件的**实际**行号对不上（该文件当时 596 行、头文件 121 行），
> 所以下面每条都按**内容**定位并给出实测行号。

---

## 1. Important 3 —— `eps` 被写成"下界"，实际是上界

**选的是哪一条:** **换成 `(σ1−σ3)/2`**，让倍数就是余量本身。
**为什么选它（而不是"留着旧量、只改措辞"）:**

1. `(σ1−σ3)/2` 是**精确**的那个量：`A` 到最近的 `m·Q`（标量质量 × 正交）的算子范数距离
   **恰好**等于它，在 `m = (σ1+σ3)/2` 处取到。换成它之后"容差 = 1.74 × 合法差"这句话
   不再需要任何星号注脚。
2. 换回原意。原文自己写的是"取**最小**能覆盖的量级"——而 `max|σ−σ̄|` 恒 ≥ `(σ1−σ3)/2`，
   **从来不是最小**。用精确距离之后那句话才终于成立。
3. 另一条的代价是**多一层注脚**: 留着上界就得写"实际余量比 1.74 更大，是 1.74~1.94"——
   注脚不会更少，只会更多，而且下一个读者还是要自己算一遍。
4. **容差的数值一个都没改**（`0.50 N` / `0.03 N·m`），指令 2 的硬约束没被触碰。

**数字（全部由用例现算打印，可复跑）:**

| 采集 | `max\|σ−σ̄\|` (kg) | **`(σ1−σ3)/2` (kg)** | 旧 eps_F → **新 eps_F** (N) | 旧比值 → **新比值** |
|---|---|---|---|---|
| 12:38 | 0.015309 | 0.013328 | 0.17257 → **0.15314** | 2.90 → **3.27** |
| 15:25 | 0.025686 | 0.023784 | 0.27061 → **0.25195** | 1.85 → **1.98** |
| 15:30 | 0.029937 | **0.027323** | 0.31274 → **0.28710** | 1.60 → **1.74** |
| 15:33 | 0.016410 | 0.016391 | 0.17610 → **0.17591** | 2.84 → **2.84** |

`max|σ−σ̄|` 比 `(σ1−σ3)/2` 大 **0.12% ~ 14.9%**（与复审给的 0~15% 一致）。

**改了哪里:**
* `Touch_Client/config/Config.h:133-151` —— 依据式、四份实测值、倍数（力 1.74~3.27 /
  力矩 1.85~3.52）、以及一段标明 **[复审改口径]** 的说明（旧量是上界、旧说法"fail-closed
  的方向"说反了）。
* `Touch_Client/config/Config.h:160-168` —— 顺带把 z 力漏洞的尺寸写进去（见 §2）。
* `Touch_Client/tests/test_payload_calibration.cpp:3165-3206` —— 判据换成
  `distScalar = 0.5 * (sg[0] - sg[2])`；**两个量都打印**（旧量标"这是【上界】，只用于对照"），
  方便下次改口径时一眼看出差多少；断言仍是 `tolF > epsF && tolM > epsM`。
* `.superpowers/sdd/runtime-guard-report.md` §0 / §2.2 / §2.3 / §7.4 —— 逐处标 **[更正]**。

---

## 2. Important 2 —— z 实际上没有闸门，缺口两个数量级大于报告的说法

**不改投票掩码**（复审判定 Fz 不投票这个取舍可接受），只把**洞的尺寸**写出来：

* 力矩通道是 Fz 唯一可能的替补：`ΔFz` 要经 `c_s` 叉乘成力矩误差才可能被看见，
  量级 = `|c_s_横向| · ΔFz`。
* 而 `c_s` 的横向分量实测**只有 0.47 ~ 0.78 mm**：

  | 采集 | \|c_s\| (m) | 轴向 (m) | **\|c_s_横向\| (m)** | `tol_M / \|c_s_横向\|` |
  |---|---|---|---|---|
  | 12:38 | 0.054555 | 0.054549 | 0.000781 | **38.4 N** |
  | 15:25 | 0.055548 | 0.055545 | 0.000499 | **60.1 N** |
  | 15:30 | 0.056040 | 0.056037 | 0.000566 | **53.0 N** |
  | 15:33 | 0.054677 | 0.054674 | 0.000471 | **63.7 N** |

* 即 **z 力模型误差要到 38 ~ 64 N 量级**，力矩通道才有机会拦（还得 `c_s_横向` 正好在
  对的方向上——这是最好情况）。
* **明确标为未决项交给用户**：这是本次改动里最大的一处已知漏洞。补一个 z 的独立判据，
  还是接受这个洞？

**改了哪里:**
* `Touch_Client/config/Config.h:160-168`
* `Touch_Client/force/ForceCompensation.cpp:44-56`（`g_guardVote` 上方）
* `Touch_Client/tests/test_payload_calibration.cpp:3177, 3185-3187, 3196-3205, 3308-3312`
  —— 逐份打印 `|c_s|` / 轴向 / `|c_s_横向|` / `tol_M/|c_s_横向|`，末尾再打一行四份范围。
* 报告新增 §3.1.1（含上表与未决项），§7 第 3 条加了数字。

---

## 3. Important 4 —— `GuardState → RobotErrorCode` 无测试且按字面量索引

**修法:** 映射收进 `ForceCompensation::guardErrorCode`（唯一一份实现），`RelayCore` 只调它。

* `Touch_Client/force/ForceCompensation.h:3` 新增 `#include "../safety/RobotError.h"`；
  `:99-107` 声明 + 说明为什么（穷举 + 唯一实现）。
* `Touch_Client/force/ForceCompensation.cpp:486-493` 实现 —— `switch` **无 `default`**
  （加了新枚举成员时 /W4 会给 C4062），语句里也写清了：本项目按 **/W1** 编译，
  那条警告**不会**出现，所以真正把表钉住的是**测试**，末尾那句 `return OK` 只是
  掉不出函数尾的兜底。**这一点如实写进了注释**，没有假装编译器会兜。
* `Touch_Client/relay/RelayCore.cpp:1697` 改成 `const GuardState guardSt = …guardState();`；
  `:1749-1768` 去掉 `guardSt == 1 || guardSt == 2` 与三元表达式，
  改用 `guardSt != GuardState::OK` + `guardErrorCode(guardSt)`。
  `lastGuardSt` 仍是 `int`（复报节流的比较键），但由 `static_cast<int>` 现取。
* **新测试** `test_guard_error_code_mapping`（`tests/test_force_compensation.cpp:310-342`，
  在 `main` 的 `:820` 注册）钉住：三条映射逐条相等、两个码不同、
  **两个码与 `GuardState` 的数值索引不相等**（"按 `static_cast<int>` 对上"这种巧合
  不许再被依赖）、名字对得上、严重度两个都是 `REJECT` 且相等。

### 3.1 严重度：`REJECT` 现在**不被消费**（复审指出，注释已改）

* 事实：报错走 `RobotDiagnostics::logError`（`RobotDiagnostics.cpp:94-110`），
  它只写日志 + `RelayCore::reportDiagnostic` 发 `D|` 帧；**不调用**
  `RobotStateMachine::onError`。所以这里填 `FATAL` 同样**不会** `DisableRobot`，
  填 `REJECT` 也**不会**"拒绝该帧运动"——两条路效果一样。
* 因此原注释里"FATAL 会在启动后 1 秒内把机械臂禁掉"**是错的**。
* **效果是对的**（复审判定）：`compensated` 全 6 个分量无条件置零 + 三条咨询性消息
  （stderr / `robot_diagnostics.log` / `D|` 帧）。
* 改在 `Touch_Client/safety/RobotError.h:105-118`：明说严重度**现在只是给日志读的一个标签**，
  保留 `REJECT` 只剩一条很弱的理由（字面语义最贴）；要让它真生效得先接进 `onError`。
* 报告 §4.1 已加 **[更正]** 块。

---

## 4. Minor 5 —— 启动零偏漂移检查在闸门拒绝时被静默证伪

**选的是哪一条:** **不加注释就跳过** —— 准确说：**闸门非 `OK` 时不积不判，把窗口重开
等它放行；到点仍不放行就明说"未做"，不打印任何结论。**

* 为什么不是"换一个不被闸门置零的源": 这个检查要的是"补偿后的读数"，而被闸门切断之后
  **本来就没有**一个"补偿后读数"可读——换源就不再是同一个检查了（`sixForceRaw` 含重力，
  不是漂移信号）。所以正确的动作是**不装**，而不是找一个看起来像的数。
* 为什么带重试（而不是一遇到拒绝就直接放弃）: 闸门在 Task 8 "发负载 → 看它放行" 之后
  会自动恢复放行，那时检查才**有意义**；直接放弃会把一次本来做得成的检查白白丢掉。
* 上限 `ZERO_CHECK_GUARD_WAIT_MS = 60000` ms：等不到就明说，不空转。
* 重开窗口时**把已积的 0 清掉**——留着会把后面的真读数稀释成 0。

改在 `Touch_Client/main.cpp:1853-1858`（新常量 + 理由）与 `:1889-1909`（闸门拒绝时的分支）。
打印的是：「`零偏漂移检查: 【未做】—— 一致性闸门一直在拒绝……0 不是零偏……`」。

⚠ **没能验证**：`runZeroDriftCheck` 是 `main.cpp` 里的静态函数，本项目的测试栈不编
`main.cpp`（它要 OpenHaptics/GLUT），所以这条**没有单测**，只做过编译与人工推演。

---

## 5. Minor 6 —— 一句写错的注释（"触觉与约束力两条路"）

* 事实：虚拟约束力在 `haptic/HapticCallback.cpp:168` 由 `SafetyPredictor::computeConstraintForce`
  拿**位置**现算，与 `compensated` 无关；`orientExtraForce`（`RelayCore.cpp:892` 由
  `ConstraintForce::computeSingularForce` 按 servoCmd 位置算）与 `orientRepulsionForce`
  （`RelayCore.cpp:842`，手腕对齐斥力）同样与 `compensated` 无关。
* 所以闸门断开的是**传感器力那一条路**（Touch 反射力 + `F|` 帧），**不是**约束力。
* 改在 `Touch_Client/force/ForceCompensation.cpp:247-251`（运行时的三段拒绝消息）与
  `Touch_Client/force/ForceCompensation.h:65-73`（判据说明）。
  原文"两条路一起断"如果留着，会让操作员以为拒绝之后连安全边界的推手都没有了。

---

## 6. 零碎（都便宜）

| # | 项 | 改在哪 |
|---|---|---|
| a | `GuardReport::voted` 默认值 `{false,false,false,true,true,true}` → **`{true,true,false,true,true,true}`**，与 `g_guardVote` 逐位一致；并注明掩码的唯一实现是 `.cpp` 里那一份 | `force/ForceCompensation.h:84-92` |
| b | `GUARD_MIN_DET_RATIO` 的消息改成与判据相符：**不改门限**（改了就是改判决逻辑），改措辞并给出它实际挡的东西 —— 该比值换算成条件数约 **354**（同一 `cond` 下比值最大的形状 `σ1=σ2=σ3·cond` 给 `1/(2.828·cond)`），即"只有当 cond ≳ 354 才必然被拦下"；指令 3 点名的全零 `A` 由 `allZero` 分支挡住，与该比值无关 | `force/ForceCompensation.cpp:440-448`（消息）、`:58-67`（常量处的推导）。注：复审写的是"cond ≈ 350"，精确值是 353.6 |
| c | `RobotError.h` 的错误码数 `(24种)` → **`(25种)`**，并注明必须与 `RobotDiagnostics::ERROR_CODE_SLOTS`（=25）相等 | `safety/RobotError.h:6-8` |
| d | `setCalibration` 只校验 `A` 的有限性 → **四个数组都校验**，与 `loadFromFile` 对齐（否则"从内存装一份带 `inf` 的零偏"这条不经过文件的路会静默收下，`inf` 会把闸门 EMA 污染成 NaN）。拒收时与 `A` 那条一样：作废旧模型 + `resetGuard` + 报 `UNCALIBRATED` | `force/ForceCompensation.cpp:325-350` |
| e | 反面对照的口径：`t6LocalModel` 是 `step()` 公式的**逐字副本** ⇒ `d ≡ 0` 是**代数结论**，**与容差无关**；它证的是**分支存在性**（"放行"这条路没坏），**证不了**容差松紧（0.50 N 与 1.5 N 在这里都是 36/36 放行） | `tests/test_payload_calibration.cpp:2898-2903`（`t6LocalModel` 上方）、`:3268-3280`（反面对照现场）；报告加了 §0.1，§5.1 也补了口径 |

---

## 7. 测试命令与输出（本轮全部复跑过）

```
> cmd /c <repo>\Touch_Client\build.bat
  ...
  Touch_Client.vcxproj -> D:\Projects\Touch\Touch_Client\x64\Release\Touch_Client.exe
  Build OK.
  Build complete.   (只有既有的 glut.h/CALLBACK C4005 警告)

> cmd /c <repo>\Touch_Client\tests\build_force_comp_test.bat           BUILD_EXIT=0
> cmd /c <repo>\Touch_Client\tests\build_payload_calibration_test.bat  BUILD_EXIT=0
> cmd /c <repo>\Touch_Client\tests\build_session_report_test.bat       BUILD_EXIT=0

> tests\test_force_compensation.exe   ->  Results: 23 passed, 0 failed   (原 22/0, +1 = guard_error_code_mapping)
> tests\test_payload_calibration.exe  ->  49 passed, 0 failed           (不变)
> tests\test_session_report.exe       ->  20 passed, 0 failed           (不变)

> cmd /c <repo>\Touch_Client\tests\run_tests.bat
  12 个套件全部 [OK]，0 个 [FAIL]：
  test_force_pipeline 5/0 · test_constraint_force 7/0 · test_safety_core 8/0 ·
  test_feedback_parser 28/0 · test_escalation 15/0 · test_kinematics 18/0 ·
  test_coord_safety 27/0 · test_force_compensation 23/0 · test_relay_command_parser 11/0 ·
  test_force_logger 4/0 · test_tcp_calibration 7/0 · test_session_report 20/0
  （test_safety_core 那条既有的偶发本轮没有出现；没有碰 RobotStateMachine。）
```

**命令注记（给下一个跑的人）:** Git Bash 里 `cmd /c` 会被 MSYS 改写、bat 名也找不到
（`NoDefaultCurrentDirectoryInExePath`）；可用 `cmd //c "$(cygpath -w /d/…/xxx.bat)"`。
另外本环境的 shell 文本里反斜杠会被折叠（`\b` 变退格），路径一律走 `cygpath`。

**没杀任何进程**：构建期没有出现 `LNK1168`。

---

## 8. 没能验证的地方（如实列）

1. **`main.cpp` 的零偏漂移检查没有单测**（§4）——测试栈不编 `main.cpp`，
   只做过编译 + 人工推演；`ZERO_CLOSE_GUARD_WAIT_MS = 60000` 这个上限**没有实测过**
   它在实机上的手感。（常量名见 `main.cpp:1858`，`ZERO_CHECK_GUARD_WAIT_MS`。）
2. **`|c_s_横向|` 的四份范围来自四份夹具、用 `fitRawLinear` 现解出来的 `c_s`** ——
   它是**本轮唯一的 z 漏洞量级来源**；发送正确负载之后 `c_s` 会重解，届时这个数要重算。
3. **"容差会不会太松"仍然没有机器可检查的答案。** 本轮唯一新增的是"`tol > eps`"
   （挡太小）与"映射被钉住"；§2 的洞是被**写明**了，**不是被堵上了**。
4. **复审给的行号与本轮起点文件对不上**（`ForceCompensation.cpp:624`、
   `ForceCompensation.h:89/:620`）。本轮按**内容**逐条核对 —— 五条内容全部找到并处理，
   但**没有办法核**"复审当时看的是哪个快照"，所以没有把它的行号写进任何注释。
5. **`@576` 力矩参考点与传感器原点不同会多出 `|Δc × F|`** 这一项（原报告 §2.4）
   依旧**没量**；它比 `tol_M` 还大，仍是最可能需要调整的地方。
6. **`GuardReport` 的默认值改了，但没有用例去断言"默认构造的那个 `voted` 数组"**。
   改动本身是**逐位对照 `g_guardVote`** 写的（`{true,true,false,true,true,true}`），
   `guard_fz_reported_but_not_voted` 断言的是**填过之后**的 `rep.voted[2] == false`，
   那条走的是 `guardReport()` 的填写路径，覆盖不到默认成员初始化器。
   即：**"默认值"这一处只做了代码一致性，没有测试**。
