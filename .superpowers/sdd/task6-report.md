# Task 6 Report — 运行时通道切换 (本地全量补偿)

**Status:** DONE (一个未接线项 + 一条没能删掉的东西, 都在下面照实报)
**Commit:** 代码 / 测试 / 本报告在**同一次**提交里, 它就是
`git log -1 --format=%h%n%s` 在写这份报告的那一刻给出的那一次。
⚠ **这里故意不写死 hash**: 文件写不下自己所在那次提交的 hash —— 报告文本每改一次,
那次提交的 hash 就跟着变一次, 于是任何写进来的数都会是**上一版**的。以**提交主题**
为准, 或用上面那条命令现取。
> subject: `feat(force): 本地补偿换全量模型 —— 通道 @576→@1304, 重力过自由 3×3 的 A, ψ 退场`
**Branch:** `feat/pen-clamp-redesign`
**Brief:** `.superpowers/sdd/task6-runtime-channel-brief.md`

---

## 0. 一句话

本地补偿从残余模型 (`@576` − b_F − 标量质量·g(ψ)) 换成全量模型
(`@1304` − b_F − A·g, 力矩 − b_M − c_s×(A·g)), ψ 从补偿路径退场;
参数表落盘换成 `force_calib.json` version 3 (旧文件被拒且响亮);
**姿态无关性逐通道实测: 四份采集上力与力矩【都】变好** (力 12.3~27.6×, 力矩 12.6~18.4×;
留一交叉验证下同样都变好)。

---

## 1. 改动与行号

### 1.1 `Touch_Client/force/ForceCompensation.h`

| 行 | 内容 |
|---|---|
| `:31-34` | `step()` 的契约: 输入通道由 `fd.raw`(@576) 改成 **`fd.sixForceRaw`(@1304)** |
| `:36-46` | 新增全量模型的说明块 + `setCalibration(A[9], b_F[3], b_M[3], c_s[3])` |
| `:48-49` | 新增 `currentModel(A, c_s)` —— 供「仅调零」把模型原样保留着写回文件 |
| `:65-71` | `currentMassKg()` 重述: 现在是 **`|det A|^(1/3)`**, 不是调用方传进来的标量 |
| ~~`:39-44`~~ | **`setMassCom` 已删除** (见 §4) |

### 1.2 `Touch_Client/force/ForceCompensation.cpp`

| 行 | 内容 |
|---|---|
| `:15-16` | 状态: `g_massKg` → **`g_A[9]`** |
| `:19-30` | 新增 `massScaleOf(A) = |det A|^(1/3)` —— 奇异值的几何平均就是它, 不需要 SVD |
| `:32-33` | 顶部注释: 重力的唯一去处改成 `gravitySensorFrameAtYaw` |
| `:186-199` | `setCalibration` 新签名 (A / b_F / b_M / c_s); **`setMassCom` 整段删除** |
| `:200-205` | `currentModel` |
| `:228-234` | `currentMassKg` = `massScaleOf(g_A)` |
| `:247-249` | **step 第 2 步: 未标定时的默认值也换成 `sixForceRaw`** —— 切换通道时最容易漏的一行 |
| `:263-272` | `g = TcpCalibration::gravitySensorFrameAtYaw(pose, 0.0, gTool)` —— **ψ 传 0** |
| `:275-279` | `Fg = A · g` |
| `:281-283` | `Mg = c_s × (A·g)` —— 与 `Fg` 同一个 `w = A·g` |
| `:286-296` | `Fi = mass·acc` —— **语义/时机/系数一字未改**; mass 改取 `|det A|^(1/3)` |
| `:298-303` | `compensated = sixForceRaw − b − Fg − Fi` / `− b_M − Mg` |
| `:307-317` | 在线 EMA: 时机/系数一字未改, 输入量换成 `sixForceRaw` |

### 1.3 `Touch_Client/force/ForceCalibration.h` / `.cpp`

| 位置 | 内容 |
|---|---|
| `.h:1-8` | 头注: 落盘格式变了, 旧文件被拒 |
| `.h:49-57` | `saveToFile` / `loadFromFile` 新签名 (A / b_F / b_M / c_s) |
| `.cpp:164-192` | 「仅调零」(`'z'`): 保留 A / c_s, 只换零偏; A 全 0 时**大声警告**(旧代码里那句"残余质量非零"的警告删掉了 —— 那个概念没了) |
| `.cpp:300-341` | 「全流程」(`'k'`) SOLVE 相: 只更新零偏 + 保留 A / c_s; 那个 `F=m·a` 的标量质量**不再进补偿**(全量模型的参数表里没有它), 只打印出来与 `|det A|^(1/3)` 对照 |
| `.cpp:350-380` | `saveToFile` version **3**, 字段 `a_matrix(9) / bias_force_n(3) / bias_torque_nm(3) / com_sensor_m(3)`; 上方写明为什么不留 `mass_kg` 字段 |
| `.cpp:396-411` | `jsonReadArray` 助手 |
| `.cpp:413-427` | **`rejectOldFormat`** —— 响亮地说出"不是本格式" |
| `.cpp:429-475` | `loadFromFile`: **先判 version 再读任何一个参数** (顺序要紧: 反过来会在返回 false 之前把半份数据写进调用方的数组) |

### 1.4 `Touch_Client/main.cpp`

| 行 | 内容 |
|---|---|
| `:1050-1057` | 「's' 什么都不写」那段注释: 不再把已删除的 `setMassCom` 当作现存的入口 |
| `:1519-1521` | 打印给操作员的那一行**不再点名一个已不存在的函数** (原文 `不写本地补偿 (setMassCom)`) |
| `:2650-2656` | ψ 的注释重写: Task 6 起本地补偿不读 ψ, 模块态 ψ 只剩已停用的旧求解路径一个消费者 |
| `:2691-2704` | 启动加载: 新格式 + `setCalibration(A, bF, bM, cS)` |

### 1.5 `Touch_Client/relay/RelayCore.cpp`

| 行 | 内容 |
|---|---|
| `:1680-1686` | **TARE 的输入通道 `fd.raw` → `fd.sixForceRaw`** |

> 为什么这一行是必须的: 调零定的零偏是**全量模型那个通道**的零偏。继续喂 @576 会让 TARE
> 平均出**另一路量**的零偏, 而两路的零偏不是一回事 —— 12:38 那份夹具上逐轴均值差
> **19.8 / 1.6 / 1.7 N** (`tests/fixtures/calib_poses_2026-09-19.txt`, 7 姿态逐通道均值)。
> 扣错之后读数依旧是个 N, 不报错。

### 1.6 测试

- `tests/test_force_compensation.cpp`: **8 → 14** 个用例。改了 4 个既有用例的参数形态
  (`setCalibration(A, …)` + `sixForceRaw`), 新增 6 个:
  `comp_gravity_goes_through_A` (重力确实过 3×3 而不是标量)、
  `comp_moment_is_cross_of_Ag` (叉乘的顺序/符号)、
  `calib_file_roundtrip` / `calib_file_rejects_old_format` / `..._no_version` / `..._truncated`。
- `tests/test_payload_calibration.cpp`: **47 → 48**。新增
  `test_runtime_compensation_pose_independence` —— §5 的验收, 见下。
- `tests/build_payload_calibration_test.bat`: 链进 `..\force\ForceCompensation.cpp` ——
  验收用例跑的是**生产补偿代码本身** (`setCalibration` + `step`), 不是把公式再抄一遍。

---

## 2. ★ 验收 (Step 5) —— 姿态无关性: 新 vs 旧, 逐通道, 四份采集

**判据**: 对每一份采集, 用**该次采集解出的**参数做补偿, 再量补偿后读数在姿态之间的散布
`dep = sqrt( (1/(3n))·Σ_a Σ_i (c[a,i] − mean_i c[a,i])² )`。力三轴一个数、力矩三轴一个数,
**不合账**。两侧同口径。

**新 (全量) 侧跑的是生产代码**: `ForceCompensation::init()` + `setCalibration(fit.A, fit.bF,
fit.bM, fit.cS)` + `step(fd, pose)`, 每次调用前重做一次 `init` —— 于是运动估计器判"静止"
(`Fi=0`), 而 EMA 在 `step` 的第 8 步、输出在第 7 步就算完了 ⇒ 读到的就是纯模型。**口径自校**:
两侧散布逐位等于 `fitRawLinear` 报的 `rmsForceN` / `rmsMomentNm` (断言 `< 1e-9`)。

**旧 (残余) 侧是"它最好的样子", 不是稻草人**: 在同一份采集上把 `(ψ, Δm, Δp, b_F, b_M)`
全部重新最小二乘定一遍 (ψ 扫 [-180,180]/0.5°, 判据同生产旧求解器的 `fitAtYaw`),
b_F/b_M 也按 LS 定 (现场是 TARE 采的, 离线没有)。因为 b 是截距, LS 残差均值恒为 0
⇒ `dep_old` 就是 LS 残差的 RMS。

### 表 (n = 该采集的姿态行数, 含重复访问那几笔)

| 采集 | n | ψ_old | 力 旧→新 (N) | 倍 | 力矩 旧→新 (N·m) | 倍 |
|---|---|---|---|---|---|---|
| 12:38 | 7 | −98.0° | 0.2924 → **0.0224** | **13.06×** | 0.0176 → **0.0014** | **12.57×** |
| 15:25 | 9 | +80.0° | 0.2285 → **0.0186** | **12.27×** | 0.0162 → **0.0010** | **15.62×** |
| 15:30 | 10 | −101.5° | 0.3557 → **0.0191** | **18.66×** | 0.0228 → **0.0012** | **18.44×** |
| 15:33 | 10 | −99.5° | 0.4174 → **0.0151** | **27.61×** | 0.0245 → **0.0019** | **12.97×** |

### 留一交叉验证 (回应"全量模型 12 个自由参数 vs 旧模型 4 个, 样本内当然拟合得更好")

每折留出一个姿态、在其余姿态上重定两侧参数, 再量被留出那个姿态的补偿后读数散布。
(旧模型的 ψ 在 LOO 里固定为整份采集扫出来的那个 —— ψ 是模型形式参数, 每折重扫 721 次
既无必要也慢。)

| 采集 | 力 旧→新 (N) | 倍 | 力矩 旧→新 (N·m) | 倍 |
|---|---|---|---|---|
| 12:38 | 0.4284 → **0.0756** | **5.66×** | 0.0290 → **0.0023** | **12.40×** |
| 15:25 | 0.4270 → **0.0351** | **12.17×** | 0.0198 → **0.0012** | **16.39×** |
| 15:30 | 0.4532 → **0.0319** | **14.21×** | 0.0260 → **0.0016** | **16.71×** |
| 15:33 | 0.5299 → **0.0243** | **21.79×** | 0.0283 → **0.0023** | **12.11×** |

### 逐通道结论

- **力通道: 变好 —— 四份采集、两个口径 (样本内 + LOO) 全部变好**, 5.7× ~ 27.6×。
- **力矩通道: 也变好 —— 同样八格全部变好**, 12.1× ~ 18.4×。
  (力矩模型 `c_s × (A·g)` 本项目自己记录为不完整, 见
  `Docs/superpowers/evidence/moment-gate-diagnosis-report.md`; 这里**没有**把它与力合成一个数。)
- 两个通道都变好 ⇒ 用例里**两条断言都下** (`CHECK(worseF==0)` 与 `CHECK(worseM==0)`),
  没有把任何一侧降级成一句打印。

### 关于 brief 里那个 "rmsF 0.651"

brief §1 写「`A_F` 最小化力残差（实测 rmsF 0.651）」。**核过: 0.651 不是以 N 为单位的 rms**,
它是**力门那个 χ²_rep/dof 统计量** (`0.650928`, 出处 `Docs/superpowers/evidence/joint-A-report.md:173`
与 `joint-A-prod-report.md:141`; `A_joint` 把它抬到 2.069381)。这一点**不改结论**
(A_F 确实赢), 但这个数**没有被我写进任何注释** —— 本项目实际测出的 rmsF 是
`0.0224 N` (12:38 那份夹具, = `test_payload_calibration.cpp:1601` 的冻结金标 `REF_RMSF`)。

---

## 3. `force_calib.json` 的格式与旧文件被拒的行为

### 新格式 (version 3)

```json
{
  "version": 3,
  "a_matrix": [0.36..., 0.21..., ...9 个...],
  "bias_force_n": [-21.9, -1.4, 2.6],
  "bias_torque_nm": [-0.18, 0.38, -0.025],
  "com_sensor_m": [0.000598, -0.000502, 0.054549]
}
```

- `a_matrix`: 3×3 row-major, 单位 kg; `com_sensor_m`: 质心, 单位**米**(传感器测量系)。
- **没有 `mass_kg` 字段, 这是有意的**: 全量模型的参数表里没有标量质量 —— 它由 A 分解出来
  (`m = |det A|^(1/3)`)。多写一个字段 = 多一份可以与 A 漂开的副本。
- `a_matrix` / `com_sensor_m` 用 `%.9g` (A 的 9 个元素跨 4 个数量级; 6 位有效数字不够),
  零偏仍用 `%.6g`。

### 旧文件被拒 —— 行为 (响亮, 不静默)

版本号**先判**, 读到 `version != 3` 或没有 `version` 字段:

1. 一行 `[Force] !! force_calib.json 【格式不兼容, 已拒绝】: version=2 (...)` 到 **stderr**,
   带**文件路径**、**期望的四个字段名**、**为什么不能拿新版读**(旧文件没有 A ⇒ 读进来是一份
   没有重力项的模型, 而补偿后的读数依旧是 N, 不会报错)、以及**该做什么**(先 `'m'`+`'s'`, 再 `'z'`)。
2. 返回 `false`; `main.cpp:2691` 那一支再补一句 "无可用 force_calib.json — 按 'z' 调零"。
3. **四个输出数组一个字都不写** (`loadFromFile` 的契约里写明: 返回 false 时未定义 ——
   实现上是不写, 因为 version 判定排在所有读取之前)。
4. **文件不存在**时返回 false 但**不吵** (那是正常路径: 还没标定过)。这两种 false 在返回值上
   分不开, 区别只在 stderr 那一段 —— 用例把它钉住了 (`calib_file_rejects_old_format` 末尾)。

覆盖: `calib_file_roundtrip` / `calib_file_rejects_old_format` (version 2) /
`calib_file_rejects_no_version` (version 1) / `calib_file_rejects_truncated` (version=3 但数组短)。

---

## 4. 删掉了什么, 依据是什么

| 删掉的东西 | 依据 | 备注 |
|---|---|---|
| `ForceCompensation::setMassCom` (声明 + 定义) | brief §3 第 1 条 | **调用点只剩 `main.cpp:1781`, 它在 `#if 0` 块里 ⇒ 按 §4 "不许动 `#if 0` 块" 没删**。见 §6 顾虑 3 |
| "残余质量可以带符号"那一整套说法 | brief §3 第 2 条 | 头/源文件里的说明、`'z'` 分支里"保留的残余质量非零"的警告、`'k'` 分支 "mass=%.3f kg" 的措辞, 全换成全量模型的说法 |
| 「残余」概念的**调用点** | brief §3 第 2 条 | `main.cpp:1050-1057`、`:1519-1521`、`RelayCore.cpp:1680-1686`、启动加载 |
| `fd.calibMassKg = mass` 的语义 | 跟随模型 | 字段保留 (无读者), 现在装的是质量尺度 |
| `'k'` SOLVE 相那个标量质量**进补偿**这件事 | brief §1 "只换重力/零偏那部分" + §2 参数表 | 拟合仍在跑、仍打印, 但**不写进文件也不进 setCalibration** —— 防止它把 A 覆盖成 0 (那正是"安静地错") |

**没有删** (brief 点名但我按 §4 停下了):

- **A2 的「内存 ≠ 落盘」拆分** —— brief §3 第 3 条要求删, 但要求"先读相关记录; 找不到依据先报告再动"。
  依据**找到了**: `.superpowers/sdd/a2-lifetime-split-report.md` (以及 brief 点名的
  `Docs/superpowers/evidence/task-session-report.md`)。而那段拆分的**全部文字**如今只存在于
  **`main.cpp:1747-1797`, 在 `#if 0` 块内** (`#if 0` 在 `:1572`, `#endif` 在 `:1836`)。
  §4 明写"`#if 0` 块【不许动】(那是 Task 11 的事)" ⇒ **两条规定在这里直接冲突, 我按 §4 停下并报告**。
  实况: 那条拆分在**活代码里已经不存在**了 —— 活着的 `'s'` (`:1509-1526`) 本来就"什么都不应用",
  `payload_calib.json` / `force_calib.json` 在活路径上一个字节都不写, 所以没有"内存值"可以 ≠ 落盘值。
  **留给 Task 11 与 `#if 0` 块一起移除。**

**没动的** (按 §4): 门限 (`c0·LIMIT_prod + κ·e`)、求解器 (`fitRaw` / `fitRawLinear` / `A` 的估法)、
模型形式、`logCalibAttempt` / `logPoseData`、两份日志的列、`#if 0` 块、
`calib_poses.txt` / `calib_report.md` / `tests/fixtures/` 四份夹具。
**`A_joint` / 联合估计一个字都没带回来** (它已被 `c864e9d` 回退)。

---

## 5. 测试命令与输出

全部经 `.bat` 跑 (它们自己 `cd` 进 `tests/`), 从 `D:\Projects\Touch\Touch_Client\tests` 调用:

```
cmd.exe //c "call %CD%\build.bat"                          (在 Touch_Client/ 下)
    -> "  Build OK."      (仅既有 MSB8004 / C4005 CALLBACK 重定义两个警告, 无新增)

cmd.exe //c "call %CD%\build_force_comp_test.bat"   -> BUILD_EXIT=0
./test_force_compensation.exe
    -> "Results: 14 passed, 0 failed"        (基线 8; 本任务改的就是它)

cmd.exe //c "call %CD%\build_payload_calibration_test.bat"  -> BUILD_EXIT=0
./test_payload_calibration.exe
    -> "48 passed, 0 failed"                 (基线 47; 新增的那 1 条 = §2 的验收)

cmd.exe //c "call %CD%\build_session_report_test.bat" -> BUILD_EXIT=0
./test_session_report.exe
    -> "20 passed, 0 failed"

cmd.exe //c "%CD%\run_tests.bat"
    -> "Tests complete", EXIT=0; test_force_compensation [OK] 14/0
       force_pipeline 5 / constraint_force 7 / safety_core 8 / feedback_parser 28 /
       escalation 15 / kinematics 18 / coord_safety 27 / relay_command_parser 11 /
       force_logger 4 / tcp_calibration 7 / session_report 20  —— 0 failed everywhere
```

`test_payload_calibration` **不在 `run_tests.bat` 的清单里** (这是既有状况, 见
`.superpowers/sdd/a2-lifetime-split-report.md` 的 follow-up 8) ⇒ §2 那条验收**只在显式跑该 exe 时执行**。

**没有结束任何进程**: 全程没有出现 `LNK1168`, 没有调用 `taskkill`。

---

## 6. 没能验证的地方 / 顾虑

1. **★ 运行时的全量补偿现在【没有生产路径把参数喂进去】。**
   启动加载只认 version 3; 能写 version 3 的只有 `'z'` / `'k'` 两个 TARE 分支, 而它们**保留**
   当前的 A / c_s —— 而 A 的初始值是 0。活着的 `'s'` (`main.cpp:1509-1526`) 明确"什么都不应用"
   (`PRINTED_ONLY`), 它的理由 (解出的量原点未定、下发路径由 plan Task 9 开通) 与本次补偿切换
   **不是同一件事** (那是"下发给机械臂", 本地补偿不是)。**brief 的 §1-§5 没有要求我接这条线,
   我也没有接** —— 那是改判决输出与 `PRINTED_ONLY` 语义的大改动, 不在本任务内。
   **后果**: 切过去的补偿路径目前只有"手工把一份 version 3 文件放进 `calib/`"才走得到。
   §2 的验收是离线跑参数的, **绕过了这个缺口**。**这一条要有人决定。**

2. **没有实机验证。** 验收按 brief 是离线的。没跑过的东西: 真实 125 Hz 帧流、
   `MotionEstimator` 的 `isStill()` (它的 `dt` 与实际节奏不符是已知的老问题)、在线 EMA 的收敛、
   `RelayCore::pollForce` 的实际时序。全量模型**没有**在实机上做过一次"摆姿态看零漂"。

3. **`#if 0` 块里留着一个对已删除函数的调用。** `main.cpp:1781` 仍是
   `ForceCompensation::setMassCom(resMass, resCom)`, `:1782` 仍是旧签名的
   `ForceCalibration::saveToFile(path, 0.0, bF, bM)`。它们**不被编译** (在 `#if 0` 里),
   所以不影响任何东西; 但**谁把那个块打开, 它直接编不过** —— 而在 §4 之下我不能进去改。
   建议 Task 11 移除整块时一并处理。

4. **`'k'` 全流程的 `F=m·a` 标量质量现在没有消费者。** 它仍被拟合、仍被打印 (与
   `|det A|^(1/3)` 并排做对照), 但**不写进文件也不进补偿**。它属于残余时代; 本任务不动
   MOTION/SOLVE 两相的结构 (brief 没让删)。**这是一段寿命被延长了的死代码** —— 记在这里。

5. **`fd.calibMassKg` / `calibComSensor[]` 的语义变了, 但全仓库没有读者**
   (grep 过 `.cpp/.h/.md`, 除 `AppState.h` 的定义与 `ForceCompensation::step` 的写入之外没有
   读它的地方)。所以这个改动**只可能影响将来的 HUD/MATLAB 消费者** —— 现在看不到。

6. **旧模型基线是我在测试里重写的**, 不是调用生产的 `PayloadCalibration::solve`。
   原因: `solve()` 在非物理 (`mTrue ≤ 0.01` 或 `> 5`) 时返回 false **且不填 `out`**
   (`PayloadCalibration.cpp:194` 在 `:209` 之前 return), 而 12:38 那批在旧模型下解出的
   `Δm < 0` —— 拿不到参数就没法量。所以我在夹具上写了同式的最小二乘 (差值形式换成带截距的
   形式, 自由度更宽 = 对旧模型更有利), **只有 ψ 的扫描判据沿用生产那一套**。
   风险: 这个"更有利的旧模型"是我量的, 不是生产的那个数字; 但**八格全输**, 差着
   5.7× 到 27.6×, 不是边际。

7. **`ψ` 的模块状态 (以及 `TcpCalibration::gravitySensorFrame` / `gravitySensorFrameAtYaw` 的
   ψ-带参入口) 没有删。** brief 只说"补偿路径退场"。现在 `gravitySensorFrame` 的消费者只剩
   `test_payload_calibration` 里那几条约定用例 —— 生产路径上**没有人读模块态的 ψ 了**。
   删不删由所有者定 (删它会连带动 `payload_calib.json` 的 `sensor_yaw_deg` 与那几条用例)。

8. **工作区里有两处与本任务无关的既有改动, 我没有碰、也没有提交**:
   `.superpowers/sdd/task-9-report.md` 与 `Docs/superpowers/plans/2026-09-18-calib-lifecycle.md`
   (会话开始时就是 modified 状态)。

9. **跑 `test_force_compensation` 会往仓库根写一个标定文件。** `CalibStore::dir()` 从
   **exe 路径**上溯两级: `Touch_Client\tests\test_force_compensation.exe` → `Touch_Client\` →
   `Touch\` ⇒ 落点是 `Touch\calib\force_calib.json`, 而**不是**应用的运行期目录
   `Touch_Client\calib\`。这是**既有**行为 (那个文件在本任务之前就存在, 当时是 version 2),
   本次只是把它换成了 version 3。两个 `calib\` 都是未跟踪的运行期目录, 没有覆盖任何证据文件
   (`calib_poses.txt` / `calib_report.md` 的 mtime 都还是 18:49, 未被动过)。
   **"测试会写文件"这件事本身值得有人看一眼** —— 那个文件现在的内容是空模型 + 测试用的假零偏。

---

## 7. 写进注释的每个数字的出处

| 数字 | 出处 |
|---|---|
| 姿态无关性的全部数字 (0.2924 / 0.0224 / 13.06× …) | **本任务自己测出来的**, 由 `test_payload_calibration.exe` 打印 (§5) |
| `19.8 / 1.6 / 1.7 N` (两通道零偏差) | `tests/fixtures/calib_poses_2026-09-19.txt` 7 姿态逐通道均值之差 (@1304 − @576) |
| `rmsF 0.650928` / `2.069381` 的**性质** (是 χ²_rep/dof 不是 N) | `Docs/superpowers/evidence/joint-A-report.md:173`, `joint-A-prod-report.md:141` |
| `0.0224 N` 是 12:38 的力残差 | `tests/test_payload_calibration.cpp:1601` 的冻结金标 `REF_RMSF` |
| 力矩模型不完整 | `Docs/superpowers/evidence/moment-gate-diagnosis-report.md` (存在) |
| `1.06546` / `54.55 mm` 等 | 只出现在**引用**里 (源文件原有注释), 本次没有新写 |

---

## 8. 一条教训 (记下来)

量"姿态无关性"这件事**差点被一个下标写错毁掉**: 第一版把 `cap.F1304[i][a]` 拿去填
`fd.sixForceRaw[a]` (a 取到 5), 越界读进了下一行的力数据 ⇒ **力通道的数字全对、力矩通道
差了 3300 倍**。抓住它的是那条**口径自校** (散布 vs `fitRawLinear` 的 rms, 断言 `< 1e-9`) ——
它是先写的, 不是后补的。**"力那边对上了"不足以证明量对了。**
