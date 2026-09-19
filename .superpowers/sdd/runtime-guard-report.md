# 运行时一致性闸门 —— 实现报告 (2026-09-19)

**规格:** `.superpowers/sdd/runtime-consistency-guard-brief.md`（用户三条指令逐字为准）
**实现:** `D:\Projects\Touch\Touch_Client`，MSVC 2022 BuildTools，分支 `feat/pen-clamp-redesign`
**结论一句话:** 未标定 → 拒绝 + 报错（不再透传）；有模型但与 `@576` 对不上 → 拒绝 + 报错，
**两个原因两个错误码**；`@576` 的 z 通道【不投票、但每次都报】。
当前状态（机械臂里存的负载是旧的）下，四份实机采集的**全部 36 个姿态都被拒绝**。

---

## 0. 一句话回答「它现在判什么、发送负载之后会不会放行」

* **现在判：拒绝。** 四份夹具 36/36 个姿态全部 `INCONSISTENT`，逐通道超限倍数
  Fx 5.1~6.0× / Fy 1.5~3.4× / Mx 3.4~7.0× / My 6.5~7.8× / Mz 1.6~2.5×。
* **发送正确负载之后会不会放行：离线【量不到】，所以没有做任何"会通过"的断言。**
  能说的是：容差是**按"模型都对时两者该差多少"这个量级的 1.6~2.9 倍**定的
  （下一节），并且同一套姿态上把 `@576` 换成与本地模型一致的值，**36/36 全部放行** ——
  即"放行"这条路是通的，不是坏掉的闸门。**真正的判据要在 Task 8 之后在实机上量。**

---

## 1. 改动清单（行号按本次提交后的文件）

| 文件 | 位置 | 改了什么 |
|---|---|---|
| `Touch_Client/config/Config.h` | `:128-158` | 新增闸门四个常量（容差 / EMA 率 / 复报间隔），**容差的推导与全部出处写在注释里** |
| `Touch_Client/safety/RobotError.h` | `:34-40` | 新增两个错误码 `ERR_FORCE_UNCALIBRATED` / `ERR_FORCE_INCONSISTENT` |
| `Touch_Client/safety/RobotError.h` | `:101-113` | 两者都映射到 `Severity::REJECT`，并写明**为什么不选 FATAL** |
| `Touch_Client/safety/RobotError.h` | `:154-155` | 两个名字进 `errorCodeName` |
| `Touch_Client/safety/RobotDiagnostics.h` | `:37-39, :57` | 新增 `ERROR_CODE_SLOTS = 25`；`m_errorCounts[24] → [ERROR_CODE_SLOTS]` |
| `Touch_Client/safety/RobotDiagnostics.cpp` | `:55, :114, :128` | 三处硬编码 `24` 换成 `ERROR_CODE_SLOTS` |
| `Touch_Client/force/ForceCompensation.h` | `:55-93` | 闸门 API（`GuardState` / `GuardReport` / `guardState` / `guardReport` / `guardStateName` / `modelUsable`）+ 判据与"两个原因为什么必须分开"的长注释 |
| `Touch_Client/force/ForceCompensation.cpp` | `:20-45` | 闸门状态与常量；`g_guardVote[6]`（Fz 不投票）与它的完整理由 |
| `Touch_Client/force/ForceCompensation.cpp` | `:199-259` | `setGuardState()` —— 逐通道打印（stderr）+ 状态变化即报 / 拒绝态 5 s 复报；`:261` `resetGuard()` |
| `Touch_Client/force/ForceCompensation.cpp` | `:292` `:333` `:589` | `init()` / `setCalibration()` / `shutdown()` 复位闸门 |
| `Touch_Client/force/ForceCompensation.cpp` | `:295-334` | ★ `setCalibration()` **拒收**全零 / 非有限 / 退化的 `A`（用户指令 3 的落点） |
| `Touch_Client/force/ForceCompensation.cpp` | `:368-400` | `modelUsable()` —— 装载与安装**共用同一条判据** |
| `Touch_Client/force/ForceCompensation.cpp` | `:402-423` | `guardState` / `guardReport` / `guardStateName` |
| `Touch_Client/force/ForceCompensation.cpp` | `:433-459` | ★ `step()` 默认输出改为**全 6 个分量置零**（删掉旧的 `@1304` 透传）+ `isCalibrated=false` |
| `Touch_Client/force/ForceCompensation.cpp` | `:468-472` | 标定中途被清的路径也报 `UNCALIBRATED` |
| `Touch_Client/force/ForceCompensation.cpp` | `:521-552` | ★ 补偿结果先算进局部变量 → **闸门判决** → 放行才写 `fd.compensated[]`；在线零偏 EMA 改为**只在放行时**更新 |
| `Touch_Client/force/ForceCalibration.cpp` | `:482-516` | ★ `loadFromFile` 读完全部字段后**验数值可用性**（`modelUsable` + 三个零偏/质心的有限性），不通过则拒绝装载并指名道姓 |
| `Touch_Client/force/ForceCalibration.h` | `:54-62` | 文档：返回值分不开"文件不存在"与"模型不可用"，区别只在 stderr |
| `Touch_Client/relay/RelayCore.cpp` | `:1694-1697` | 取闸门状态 |
| `Touch_Client/relay/RelayCore.cpp` | `:1742-1779` | ★ 走现成通道报错：状态变化即报 + 拒绝态 5 s 复报；两个原因两个错误码 |
| `Touch_Client/calibration/TcpCalibration.h` | `:32-38` | 过期注释更正（原写"运行时请用 `gravitySensorFrame`"，而生产早已改用 `…AtYaw(·,0.0)`） |
| `Touch_Client/tests/test_force_compensation.cpp` | 见 §6 | 改写 2 条、新增 8 条 |
| `Touch_Client/tests/test_payload_calibration.cpp` | 见 §6 | 新增闸门重放 + 容差推导打印；修 LOO 漏项；三处 `fd.raw` 补齐 |

**没动**（硬约束）：门限 `c0·LIMIT_prod + κ·e`、求解器、模型形式、用的是**力通道**的 `A_F`、
两个日志文件的列、`#if 0` 块、夹具、`calib_poses.txt`、`calib_report.md`、
`logCalibAttempt` / `logPoseData`。

---

## 2. 容差与它的推导

### 2.1 判据

逐通道比较 `d[i] = compensated[i] − fd.raw[i]`（`compensated` = 本地全量模型的外力估计，
`fd.raw` = `@576` = 机械臂用**它自己的**负载模型减完重力后的外力估计）。两个模型都对时
估计的是同一个量 ⇒ 应当一致。判决用 `d` 的 **EMA**（`alpha = 0.02`，`init`/`setCalibration`
后第 1 帧按瞬时差判）。

**先确认一件事（否则整个比较不成立）：`@1304` 与 `@576` 是同一帧的。**
`RelayCore.cpp:126-132` 在**同一次 30004 收帧**里同时填 `raw[]`(`@576`) 与
`sixForceRaw[]`(`@1304`)，两者天然时间对齐，不存在"慢一路"造成的假差。

### 2.2 「模型对时两者该差多少」这个量级 —— 两项，都可核

```
eps_F = eps_本地 + eps_模型类 = rmsForceN + max|σ(A) − σ̄| · 9.81
eps_M = rmsMomentNm + |c_s| · max|σ(A) − σ̄| · 9.81
```

* **`eps_本地`** —— 本地模型自己的失拟，夹具实测量：`rmsForceN` =
  0.0224 / 0.0186 / 0.0191 / 0.0151 N，`rmsMomentNm` = 0.0014 / 0.0010 / 0.0012 / 0.0019 N·m。
  旁证（跨通道）：`@576 = P·@1304 + q` 的拟合残差 `rms = 0.014456 N`（12:38），
  四次范围 0.0091~0.0145 N（`Docs/superpowers/plans/2026-09-19-raw-channel-calibration.md:255-257`）
  —— 与上面同一量级。
* **`eps_模型类`** —— **机械臂那一侧**的下界。它的力模型是"标量质量 × 旋转"（3 个自由度），
  而本地的 `A` 是**自由 3×3**。`A` 偏离"最近的标量旋转"的那一份，它**结构上表达不出来**，
  下界 = `max|σ(A) − σ̄| · g`。
* **9.81** = 标准重力，与 `TcpCalibration` 的重力约定同一个常数。

**四份夹具上现算（由 `test_runtime_consistency_guard_replay` 打印，可复跑核对）:**

| 采集 | σ(A) (kg) | σ̄ | max\|σ−σ̄\| (kg) | eps_F (N) | eps_M (N·m) |
|---|---|---|---|---|---|
| 12:38 | 0.433855 / 0.426469 / 0.407198 | 0.422508 | 0.015309 | 0.17257 | 0.00959 |
| 15:25 | 0.443856 / 0.425777 / 0.396287 | 0.421973 | 0.025686 | 0.27061 | 0.01503 |
| 15:30 | 0.443014 / 0.423533 / 0.388369 | 0.418305 | **0.029937** | **0.31274** | **0.01769** |
| 15:33 | 0.435090 / 0.418759 / 0.402309 | 0.418719 | 0.016410 | 0.17610 | 0.01069 |

（15:33 那行与 `...-runs-001-003-analysis.md:570` 记的 `σ = 0.434953 / 0.418963 / 0.402722`
分别差 3.1e-4 / 4.9e-4 / 1.0e-3 相对 —— 差的是夹具的旧打印精度，那份文档 §7 自己写了
三份 15:xx 夹具仍是粗精度。另外 12:38 那行的 `isotropyRatio = σ1/σ3 = 1.06546`
与 `Touch_Client/force/PayloadCalibration.h:192` 记的"实机那批: 1.06546"逐位吻合，
也与已入库的 `test_replay_real_capture` 打印的 `iso=1.06546` 一致。）

### 2.3 门限取在它的几倍

| | 值 | 依据 |
|---|---|---|
| `FORCE_GUARD_TOL_FORCE_N` | **0.50 N** | = **1.60 ×** 最坏 `eps_F` (0.3127 N, 15:30)；逐份比值 1.60~2.90 |
| `FORCE_GUARD_TOL_MOMENT_NM` | **0.03 N·m** | = **1.70 ×** 最坏 `eps_M` (0.01769 N·m, 15:30)；逐份比值 1.70~3.13 |

* 倍数是**怕 `eps_模型类` 被低估**的余量：上表那一项算的是"机械臂那一侧**最好**的成员"，
  它实际用的 `(m, center)` 不一定是最优的那个。**取最小能覆盖的量级，是 fail-closed 的方向。**
* 这条关系**由测试断言**：`if (!(tolF > epsF && tolM > epsM)) FAIL` ——
  容差将来若被改到实测导出的量级之下，测试会红，而不是安静地变成"永远拒绝"。

### 2.4 ⚠ 首版，必须在实机上复验（简报要求，写进了代码注释）

**正确的量级要在发送正确负载之后（Task 8）才量得到。** 上界的推导只覆盖了"模型类不同"
这一项。已知还会多出一项但**没量到**：

> 若 `@576` 的**力矩参考点**与传感器原点不同，比较里会多一项 `|Δc × F|`。
> `Δc ≤ 14.15 mm`（Task 9 量到的 `d ∈ [11.75, 14.15] mm`，
> `…/2026-09-19-raw-channel-calibration.md:226`），`|F| ≤ 3.0932 N`
> （四份夹具 36 个姿态里 `@576` 力模的最大值，出现在 12:38 pose 4）
> ⇒ **最大 14.15 mm × 3.0932 N = 0.0438 N·m**。
> 那一项**比 `tol_M = 0.03` 还大** —— **如果它是真的，力矩通道会在 Task 8 之后永远拒绝。**
> 这是本轮最可能需要调整的地方。**必须先测，不许凭猜放宽。**

**失败方向是拒绝（fail closed）**：容差偏紧 → 拒绝；EMA 里出现 NaN → 拒绝；
`fd.raw` 没填（全 0）→ 拒绝；`g_guardTol` 未初始化 → **在静态初始化时就填好**（不留
"容差 == 0 ⇒ 永不超限 ⇒ 静默放行"的空档）。

---

## 3. 逐通道的处置（含 z 那一列）

**力 Fx / Fy 投票，力矩 Mx / My / Mz 投票，Fz【报出比较结果但不投票】。**

| 通道 | 投票 | 容差 | 当前逐份 EMA 均值 (四份的范围) | 逐份最大超限倍数 |
|---|---|---|---|---|
| Fx | ✅ | 0.50 N | +1.689 ~ +2.159 N | 5.11 ~ 5.98× |
| Fy | ✅ | 0.50 N | +0.383 ~ +0.962 N | 1.52 ~ 3.39× |
| **Fz** | ❌ **不投票** | 0.50 N | **−0.0573 ~ +0.0552 N**（全在限内） | — |
| Mx | ✅ | 0.03 N·m | −0.0926 ~ −0.0640 | 3.38 ~ 7.03× |
| My | ✅ | 0.03 N·m | +0.1096 ~ +0.1406 | 6.46 ~ 7.85× |
| Mz | ✅ | 0.03 N·m | −0.0222 ~ −0.0029 | 1.60 ~ 2.48× |

（本表的六个"逐份 EMA 均值"由 `test_runtime_consistency_guard_replay` 直接打印，见 §5。）

### 3.1 为什么 Fz 不投票 —— 三条理由，逐条可核

1. **它的参照物动不了。** `@576` 的 z 响应实测**秩 2**：`@576 = P·@1304 + q` 的力块 `P`
   **第三行比另两行小 8~15 倍**，奇异值 `[0.212 0.201 0.008]`
   （`Docs/superpowers/plans/2026-09-19-raw-channel-calibration.md:268-274`）。
   **在【我们唯一有的激励（重力方向）上它不动** —— 夹具四份里 `@1304` 的 z 跨姿态变化
   7.2 N（−3.218 ~ +3.975，`F1304z` 列），而 `@576` 的 z 全程在 **±0.21 N** 以内。
   一个动不了的参考，既证不了"一致"，也证不了"不一致"。
2. **让它投票不会让闸门永远通过** —— Fx（5.1~6.0×）与 My（6.5~7.8×）独立地就把四份
   采集全拒了；就算只剩力矩三个通道，四份的 Mx 均值 −0.0640 ~ −0.0926、My 均值
   +0.1096 ~ +0.1406 也都超过 0.03 N·m 的容差，照样拒。
   所以"不投票"不会把闸门变成永远通过。
3. **让它投票会让闸门【永远拒绝】。** 若它对外力也不响应，则一旦有真实 z 接触，
   `compensated_z` 有值而 `@576_z` 恒 ~0，差值直接超限 ⇒ 用户明令禁止的"永远不通过"。
   （"它为什么不动"目前**没有分开**：是 `@576` 报得坏，还是机械臂 z 补偿太强 ——
   `…-raw-channel-calibration.md:273-274` 明说"不许猜"。**本条是本轮唯一一处
   "宁可少一个通道、也不让它把闸门锁死"的取舍，写在这里备查。**）

### 3.2 它【不是静默跳过】

每一条拒绝消息里都逐通道打印，Fz 那一行固定写着：

```
[Force] !!   Fz(N)    +0.0183  容差 0.5000  【不投票】@576 的 z 响应秩 2 (奇异值 0.212/0.201/0.008), 它动不了就证不了什么
```

`GuardReport` 里字段分开：`voted[2] == false`、`exceeded[2] == false`、`ema[2]` **照填真值**。
测试 `guard_fz_reported_but_not_voted` 同时钉住 (a) 3 N 的 z 差不触发拒绝、(b) 那 3 N 读得出来。

### 3.3 ⚠ 一处与简报的出入（如实记）

简报把奇异值 `[0.212 0.201 0.008]` 的出处在括号里指到
`Docs/superpowers/specs/2026-09-19-raw-channel-calibration-runs-001-003-analysis.md`。
**该文件里没有这三个数**（grep 全库只在 `Docs/superpowers/plans/2026-09-19-raw-channel-calibration.md:269`
与 `.superpowers/sdd/progress.md:1747` 里出现）。代码注释与报告一律按**计划书**引用。

---

## 4. 两个原因怎么分得开

处置一样（**都拒绝**：`compensated[]` 全 6 个分量置零），但它们分在三层上：

| | `UNCALIBRATED` | `INCONSISTENT` |
|---|---|---|
| 触发 | `g_isCalibrated == false`（未标定 / `A` 全零 / `A` 退化；装载被拒亦然） | 有模型，但某个投票通道的 \|EMA\| > 容差 |
| 错误码 | `ERR_FORCE_UNCALIBRATED` | `ERR_FORCE_INCONSISTENT` |
| 进哪 | `RobotDiagnostics` → `robot_diagnostics.log` + `D|` 帧到 MATLAB GUI | 同左 |
| 严重度 | `Severity::REJECT` | 同左 |
| 消息 | "【没有可用模型】本地补偿未启用 —— 不是\"标定与机械臂不符\"" | "【有模型, 但与机械臂对不上】两边估计的不是同一个外力" |
| 处理指引 | 按 `'m'` 采多姿态 → `'s'` 解 `A` → `'z'` 调零 | 查负载参数有没有真的发进机械臂（Task 8） |

三层都分得开是**刻意的**：错误码进日志（持久记录），消息进控制台（当场看得见），
`GuardState` 进 API（上层/测试可判）。测试 `guard_two_causes_are_distinguishable`
断言 `stA != stB` 且两个名字字符串不同。

### 4.1 严重度为什么选 `REJECT` 而不是 `FATAL`（简报要求"保守优先"，这里给了理由）

`FATAL` 会 `DisableRobot`。**两条反对理由**（`RobotError.h:100-113` 里逐条写了）：

1. 本闸门**连续判决**：未标定或负载没发进去时它每一帧都拒 ⇒ `FATAL` 会在启动后 1 秒内
   把机械臂禁掉，**Task 8 那个"发负载 → 看闸门放行"的闭环就再也走不了**（闸门自己把
   要验证的那一步锁死）。
2. **真正"不许往下传"的东西（`compensated`）已经无条件置零了，与严重度无关** ——
   数据侧已经是 fail closed；`FATAL` 只会额外停掉**不依赖这份数据**的运动。

⇒ 数据侧一律拒绝（最保守），严重度取 `REJECT`（本帧运动拒绝）。
要改成 `FATAL` 只需改 `RobotError.h` 那一行，其余不用动。

---

## 5. 闸门在四份离线采集上怎么判（重放证据）

夹具：`tests/fixtures/calib_poses_2026-09-19*.txt`（四份，共 7+9+10+10 = **36 个姿态**）。
每个姿态：`ForceCompensation::init()` → `setCalibration(fitRawLinear 解出的 A_F/b/c_s)` →
连喂 8 帧该姿态的均值读数 → 读 `guardReport()`。

```
=> 真实夹具: 36 / 36 个姿态【拒绝】    反面对照 (@576 与本地一致): 36 / 36 个姿态放行
```

逐份（`pose 1` 的逐通道 EMA 差与超限倍数）：

| 采集 | Fx | Fy | Mx | My | Mz | Fz |
|---|---|---|---|---|---|---|
| 12:38 (n=7) | +1.173 (2.3×) | +0.205 (0.4×) | −0.023 (0.8×) | +0.035 (1.2×) | −0.074 (2.5×) | 不投票 |
| 15:25 (n=9) | +1.337 (2.7×) | +0.109 (0.2×) | −0.027 (0.9×) | +0.069 (2.3×) | −0.063 (2.1×) | 不投票 |
| 15:30 (n=10) | +1.647 (3.3×) | +0.165 (0.3×) | −0.031 (1.0×) | +0.086 (2.9×) | −0.048 (1.6×) | 不投票 |
| 15:33 (n=10) | +1.265 (2.5×) | +0.609 (1.2×) | −0.034 (1.1×) | +0.023 (0.8×) | −0.048 (1.6×) | 不投票 |

**逐份的最大超限倍数（投票通道）**：Fx 5.11~5.98 / Fy 1.52~3.39 / Mx 3.38~7.03 /
My 6.46~7.85 / Mz 1.60~2.48 —— **没有一份、没有一个通道是"贴线"过的**。

> ⚠ **一处自查更正。** 本节最早的版本里，"当前实测 EMA 差"那一列用的是我先写的一个
> 一次性探针的输出。那个探针**自己在力矩那一列越界读了**（`M576[i][a]` 写成了 `[i][a]`
> 而非 `[i][a-3]`），于是力矩的统计量是错的（表里的逐姿态值是对的，只有 `stat` 那几行错）。
> 探针已经删掉，上面的数字**全部来自已入库的 `test_runtime_consistency_guard_replay`**
> （它打印逐份的均值与最大超限倍数）。**探针的那些错数没有被写进任何注释或常量。**
> 力通道的统计量当时是对的（`a < 3` 那一支索引没写错），与现在的输出逐位一致
> （Fx 2.11286 / Fy 0.382571 / Fz −0.0572857 等）。

### 5.1 它凭什么会在发送负载之后放行（以及这一条**没被证明**）

* **通的路是通的**：同一批姿态、同一套模型，把 `@576` 换成与本地模型一致的值 →
  **36/36 全部放行**（`test_runtime_consistency_guard_replay` 的反面对照；
  没有这一条，"拒绝"可能只是因为闸门坏了）。
* **但"发送正确负载之后就会一致"是【预测】，不是本轮的测量。** 现在量不到，
  因此测试里**没有**任何"它会通过"的断言。要等 Task 8 之后在实机上量。

---

## 6. 测试

### 6.1 精确命令与输出

```
> cmd /c D:\Projects\Touch\Touch_Client\build.bat
  ...
  Touch_Client.vcxproj -> D:\Projects\Touch\Touch_Client\x64\Release\Touch_Client.exe
  Build OK.
  Build complete.

> cmd /c D:\Projects\Touch\Touch_Client\tests\build_force_comp_test.bat      (BUILD_EXIT=0)
> cmd /c D:\Projects\Touch\Touch_Client\tests\build_payload_calibration_test.bat (BUILD_EXIT=0)
> cmd /c D:\Projects\Touch\Touch_Client\tests\build_session_report_test.bat  (BUILD_EXIT=0)

> tests\test_force_compensation.exe   ->  Results: 22 passed, 0 failed     (本轮改动前 14/0)
> tests\test_payload_calibration.exe  ->  49 passed, 0 failed             (本轮改动前 48/0)
> tests\test_session_report.exe       ->  20 passed, 0 failed             (未变)

> cmd /c D:\Projects\Touch\Touch_Client\tests\run_tests.bat
  12 个套件全部 [OK]，0 个 [FAIL]：
  test_force_pipeline 5/0 · test_constraint_force 7/0 · test_safety_core 8/0 ·
  test_feedback_parser 28/0 · test_escalation 15/0 · test_kinematics 18/0 ·
  test_coord_safety 27/0 · test_force_compensation 22/0 · test_relay_command_parser 11/0 ·
  test_force_logger 4/0 · test_tcp_calibration 7/0 · test_session_report 20/0
```

（注：`run_tests.bat` 只**跑**不重建前面那批 exe —— 上面 22/49/20 那三行是先用各自的
`build_*.bat` 重建之后再跑的。）

### 6.2 新增/改写的用例

**`test_force_compensation.cpp`（14 → 22）**

| 用例 | 钉住什么 |
|---|---|
| `comp_uncalibrated_refuses` **(改写)** | 未标定 ⇒ **全 6 个分量都是 0**（旧的 `comp_no_calib` 断言的是"照抄 @1304"，那正是 Critical） |
| `setcalib_rejects_zero_and_degenerate_A` | `A` 全零 / 秩 1 / 有 NaN ⇒ **拒收**；好 `A` 照收；**拒收时连旧模型一并作废** |
| `guard_passes_when_consistent` | 一致 ⇒ 放行，且 `compensated` 真是模型值（不是被置零的 0） |
| `guard_refuses_when_inconsistent` | 不一致 ⇒ 全 6 个分量置零 + `INCONSISTENT` + `GuardReport` 逐通道可读 |
| `guard_two_causes_are_distinguishable` | 两个状态的枚举值与名字都不同 |
| `guard_fz_reported_but_not_voted` | 3 N 的 z 差不触发；那 3 N 读得出来；`voted[2]==false` |
| `guard_moment_channel_votes` | 力矩通道同样能拒绝 |
| `guard_ema_needs_sustained_mismatch` | 第 1 帧按瞬时差判；之后是 EMA（第 5 帧不够、第 41 帧够）—— 把时间常数钉住 |
| `calib_file_rejects_unusable_A` | 装载路径：`A` 全零 / 秩 1 / NaN / `bias_force_n` 有 `inf` 一律拒；**同批字段的好文件必须收** |
| `comp_gravity_goes_through_A` **(改写)** | 原来用秩 1 的 `A`（只有 `A[2]`），现在会被指令 3 拒收 ⇒ 换成**置换阵**（`det = −1`，"过 A 不过标量"这件事没变） |
| `comp_moment_is_cross_of_Ag` / `zero_only_no_motion` / `zero_abort_not_applied` / `zero_restartable` | 补 `fd.raw`（闸门要求）；`zero_abort_not_applied` 从"全零 A"改成"正常模型 + 已知旧零偏"，改成断言**旧零偏逐位没动** |

**`test_payload_calibration.cpp`（48 → 49）**

* `test_runtime_consistency_guard_replay` —— §5 那张表，**并把 §2.2 的容差推导现算打印 +
  断言"容差 > 实测导出的量级"**。
* `test_runtime_compensation_pose_independence`：三处 `step()` 调用补上 `fd.raw`
  （一处改为喂 `t6LocalModel` 算出的值，因为那个用例量的是**模型输出**，直接喂夹具的
  `@576` 会被闸门置零）。**口径自校（散布 == `fitRawLinear` 的 rms）仍然通过**，说明
  量到的还是原来的东西。

### 6.3 顺手收的复审遗留

* ✅ `TcpCalibration.h:32` 过期注释 → 改成事实（并核过：`gravitySensorFrame` 现在
  **没有任何生产调用方**，grep 只剩测试）。
* ✅ `test_comp_gravity_goes_through_A` 第三例的空断言 → 现在三个分量都填、都断言
  （原先 `rx=ry=0` 时 `Rz(ψ)` 是恒等，测不出 ψ）。
* ✅ LOO 漏 `Δm·g` 项 → `t6FitOldAtYaw` 增加 `dmOut`，LOO 那一列改为 `@576 − b_F − Δm·g`。
  **实测影响（先量后改，两组数都记在这里）**：

  | 采集 | LOO 力（改前 → 改后） | LOO 力矩（改前 → 改后） |
  |---|---|---|
  | 12:38 | 0.4284 → **0.4474** N | 0.0290 → 0.0290 N·m |
  | 15:25 | 0.4270 → **0.2822** N | 0.0198 → 0.0198 N·m |
  | 15:30 | 0.4532 → **0.4021** N | 0.0260 → 0.0260 N·m |
  | 15:33 | 0.5299 → **0.4836** N | 0.0283 → 0.0283 N·m |

  **12:38 那一份改后【变大】**（旧模型在这份上反而更差 → "新优于旧"的倍数从 5.66× 降到
  5.91×… 逐份是 5.66→5.91 / 12.17→8.04 / 14.21→12.61 / 21.79→19.89 倍）。照实报告：
  修的是"与 `ssF` 的口径不一致"，不是"让新模型赢得更漂亮"。**力矩那一列不变** ——
  旧模型的力矩式子里本来就没有 `dm`。
* ⏭ **`#if 0` 块引用已删的 `setMassCom`**：**没动** —— 本轮硬约束里 `#if 0` 块在
  "不许动"清单上。Task 11 拆那层之前要先处理它，否则编不过。

---

## 7. 没能验证的地方（如实列）

1. **"发送正确负载之后闸门会放行"—— 完全没验证。** 离线量不到；本轮的容差是首版，
   必须在 Task 8 之后在实机上复验。**最可能出问题的是力矩容差**（§2.4 的 `|Δc × F|` 项）。
2. **`@576` 的 z 通道"为什么不动"没有分开**（报得坏 vs 机械臂 z 补偿太强）。因此
   Fz 只能是"报而不投"；分开之后应当把它提升为投票通道。
3. **Fz 不投票 = z 轴上少了一道闸门。** 若只有 z 方向出问题（力矩合力的方向），
   这一版看不见。已知代价，写在这里。
4. **`eps_模型类` 只是下界。** 它算的是"机械臂那一侧**最好**的成员"；它实际用的
   `(m, center)` 不一定最优。倍数 1.60/1.70 就是给这一条留的，**没有实测支撑**。
5. **闸门的响应时间是 ~3 s**（EMA `alpha=0.02` @30Hz 等效平均 ~100 帧）。持续的不一致
   不会在第 1 帧之后立刻触发；这是一个**模型一致性**检查，不是碰撞/急停那种快保护。
6. **`fd.raw` 与 `sixForceRaw` 的"同帧"是对着 `RelayCore.cpp:126-132` 读出来的**，
   没有在实机上验证过两路在机械臂内部的采样时刻是否真的一致（同一次 30004 反馈帧，
   但没有实测延迟差）。
7. **拒绝的瞬态**：`step()` 置零的是 `compensated[]`；下游 `ForcePipeline::step` 用
   Butterworth（30 Hz @ 120 Hz）从 `compensated` 推 `filtered`/`hapticOut`，
   所以手上的力是**衰减**到 0 而不是跳变（时间常数在几十毫秒量级）。**这个衰减我没有实测**，
   只按滤波器参数推断。若要求"立刻断"，需要在 `ForcePipeline` 里加一条复位 ——
   本轮**没有**动它（简报说"下游拓扑若还需要闸别的东西，在报告里说，不要自己假设"）。
8. **`test_safety_core` 在 `run_tests.bat` 里偶尔红一条**（`state_transition_chain`，
   `FAIL: sm.currentState() == RobotState::DEGRADED`）。**与本次改动无关**：
   那个 exe 是 2026-07-25 构建的（`run_tests.bat` 只跑不重建），单独连跑 5 次全 8/0；
   那条用例自己有"时间阈值"的字样（`test_safety_core.cpp:41`），是**既有的偶发**。
   本轮没有碰 `safety/RobotStateMachine.*`。
9. **`RobotDiagnostics` 的报错在 `pollForce` 线程里落盘** —— 我按每 5 s 一次限了频
   （30 Hz 每帧落一行会把诊断日志冲掉），但**没有在实机上量过它对 30 Hz 环路的开销**。
10. **`test_zero_restartable` 会用 `A = diag(0.42)` 写一份 `force_calib.json` 到 `tests\`**
    （既有的行为，文件被 `.gitignore` 覆盖）。现在那份文件是**可用**的（不会被我新加的
    装载判据拒掉），但下一个用例 `test_calib_file_roundtrip` 会删掉它 —— 顺序没变。

---

## 8. 复现 §2 那张表的办法

```
cmd /c D:\Projects\Touch\Touch_Client\tests\build_payload_calibration_test.bat
D:\Projects\Touch\Touch_Client\tests\test_payload_calibration.exe 2>nul | findstr /C:"runtime consistency guard" /C:"eps_F"
```

会打出逐份的 `σ(A)` / `σ̄` / `max|σ−σ̄|` / `eps_F` / `eps_M` 以及**容差 ÷ eps** 的比值。
