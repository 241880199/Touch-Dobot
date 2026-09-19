# 原始力通道本地标定 —— 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development 执行本计划。

**规格:** `Docs/superpowers/specs/2026-09-19-raw-channel-calibration-design.md`（**必读**）

**目标:** 把负载标定从"预设模型形式 + 扫描 ψ"换成"**从原始力通道线性解出完整变换**"，
并把机械臂需要的**全部**参数（质量、质心、惯量、负载开关）传给它。

**用户约束（原话）:**
- "从根本上解决模型的问题，需要能够真正拟合实机参数并正确标定；**任何猜测以及偏差（如 ψ）都不可取**"
- "采用原始值本地计算并标定，将计算后的负载参数传回机械臂，**对比本地处理后的数据与机械臂得到新的负载参数后加工得到的数据是否一致**"
- "**在运行过程中机械臂需要得到正确的负载参数才能正确执行指令**"
- "三条一起用，需要将正确的机械臂所需要的参数都传给它"

## Global Constraints

- **文件保持 ASCII 或既有中文注释风格**；`.bat` 必须 CRLF。
- **编译命令固定为：** `cmd.exe //c "D:\Projects\Touch\Touch_Client\build.bat"`，成功标志 `Build OK.`
- **完整构建是硬性要求**：测试脚本不定义 `WIN32_LEAN_AND_MEAN`，真实项目定义 —— 单测通过**不等于**项目能编。已咬过一次。
- **测试必须真的跑起来**：构建脚本只编译，每步要单独执行 `tests\<name>.exe`。
- **编译前确认 `Touch_Client.exe` 没在运行**，否则 `LNK1168`。**不要替用户杀进程** —— 报告并停下。
- **不要写以反斜杠结尾的 `//` 注释** —— 会吞掉下一行。已咬过一次。
- **`run_tests.bat` 不覆盖全部套件**（漏 `test_payload_calibration` / `test_calib_store` /
  `test_singularity_avoidance` / `test_self_collision` / `test_fk_validate` / `test_calibration`）——
  涉及这些的改动要**单独跑对应 exe**。
- **不引入人工判断**：需要判断的地方一律由程序算（用户明确要求）。
- **发射机械臂是危险动作**：任何会改变机械臂负载配置的步骤，都必须先打印给人看、再单独一步执行。

---

### Task 1: 纯函数核心 —— 线性模型 + 分解 + 自检

**Files:**
- Modify: `Touch_Client/force/PayloadCalibration.h`
- Modify: `Touch_Client/force/PayloadCalibration.cpp`
- Modify: `Touch_Client/tests/test_payload_calibration.cpp`

**Interfaces:**
- Produces:
  - `struct RawFit { double A[9]; double bF[3]; double cS[3]; double bM[3];
     double rmsForceN; double rmsMomentNm; double cond; double paramSigma[18]; }`
  - `bool fitRaw(const double poses[][6], const double forces[][3], const double moments[][3],
                int n, RawFit& out)`
  - `struct Decomp { double m; double Q[9]; double parity; double sv[3]; double isotropyRatio; }`
  - `bool decompose(const double A[9], Decomp& out)`

- [ ] **Step 1: 模型与拟合**

```
F_i = b_F + A · g_i              A: 3×3 自由（9 参数）
M_i = b_M + c_s × (A · g_i)      c_s: 3 参数；力矩是叉乘结构，不是独立 3×3
```
`g_i = TcpCalibration::gravitySensorFrameAtYaw(pose_i, 0.0, g)` —— **不再有 ψ**。
全部线性：力 12 未知（b_F 3 + A 9）、力矩 6 未知（b_M 3 + c_s 3）。
用 `(JᵀJ)⁻¹·σ²` 给出 `paramSigma[]`，并报 `cond = σmax/σmin(J)`。

- [ ] **Step 2: 分解（这一步才是"标定"）**

```
SVD: A = U·Σ·Vᵀ
m    = (σ₁σ₂σ₃)^(1/3)
parity = sign(det A)
Q    = U·diag(1,1,parity)·Vᵀ          （正交，含手系）
```
`isotropyRatio = σmax/σmin`。

- [ ] **Step 3: 自检必须能拒绝**

`fitRaw` 或调用方在以下情况**返回 false / 不给参数**：
- `isotropyRatio` 与 1 的偏离**超出 `paramSigma` 能解释的范围**（不是固定的 1.05；
  实测 @1304 是 1.066、m 的不确定度 3.3% —— **判据要对着不确定度**）
- `m` 非正、或超出 `EnableRobot` 的负载量程
- `cond` 过大（姿态激发不足）

- [ ] **Step 4: TDD（先写用例）**

- `test_rawfit_recovers_arbitrary_A`: 合成数据用**任意 3×3**（含反射、含非正交），
  断言 `A` 逐元素复原（容差按 `paramSigma`）
- `test_decompose_recovers_rotation_and_parity`: 合成 `m·S·Rz(θ)`，断言 `m`、`parity`、`θ`
- `test_isotropy_gate_rejects_nonorthogonal`: 喂一个各向异性明显的 `A`，断言**被拒**
- `test_rawfit_uses_no_psi`: 合成数据里**不含**任何 ψ 概念，断言仍能复原

- [ ] **Step 5: 跑测试 + 完整构建**

`build_payload_calibration_test.bat` → exe → 全绿；`build.bat` → `Build OK.`

- [ ] **Step 6: 提交**

```
feat(payload-calib): fit the raw force channel linearly and decompose it, with no preset angle
```

---

### Task 2: 离线重放验证（用真实 7 姿态数据）

**Files:**
- Modify: `Touch_Client/tests/test_payload_calibration.cpp`（新增一个可选用例）

- [ ] **Step 1**: 若 `Touch_Client/calib/calib_poses.txt` **存在**，读取并跑 `fitRaw`，
  断言: `m ≈ 0.422 ± 0.03 kg`、`isotropyRatio < 1.15`、`parity == -1`、
  `rmsForceN < 0.03`、`c_s[2] ≈ ±54 mm`。**文件不存在则 SKIP（不失败）** —— 它是运行期产物。
- [ ] **Step 2**: 跑测试；提交。

> 控制器已用 Python 独立算出这些数（`m=0.42236`、`σ=[0.43386 0.42647 0.40720]`、
> `det=-0.075`、`rmsF=0.02239`、`c_s=(+0.60,-0.50,+54.55)mm`、力矩 rms=0.001397 N·m）。
> **实现结果必须与这些数一致** —— 对不上就是实现有问题。

---

### Task 3: 惯量计算

**Files:**
- Modify: `Hardware/tools/compute_payload.py`

- [ ] **Step 1**: 用每个零件的质量与位置，按平行轴定理算**相对法兰面的完整惯量张量**，
  打印三个主惯量 + 张量本身。
- [ ] **Step 2**: 交叉验证总质量仍为 656.9 g、质心仍为 cz = 80.4 mm（回归护栏）。
- [ ] **Step 3**: 提交。

> `PayLoad(weight, inertia)` 的 `inertia` 是**单一标量**，文档未定义取哪个分量。
> 三个主惯量都报出来；实机上以 **`P` 塌到 0** 为判据确定厂家取的是哪一个。

---

### Task 4: main.cpp 接线 —— 新模型，**先不发机械臂**

**Files:**
- Modify: `Touch_Client/main.cpp`

- [ ] **Step 1**: `solveAndApply()` 改调 `fitRaw` + `decompose`，**删掉** ψ 扫描相关调用与
  `comSignZ`。
- [ ] **Step 2**: 打印全部量，**标签要让人分不出错**:
  `m(kg) / 各向同性比 / parity / c_s(mm) / 各自的 rms / cond / m 的不确定度`。
- [ ] **Step 3**: **不发机械臂**（保持现状：不调用任何 `EnableRobot`/`PayLoad`/`LoadSwitch`）。
- [ ] **Step 4**: 完整构建 + 单独跑 payload 测试。提交。

---

### Task 5: 实机验证 A（交用户，**只算不发**）

`'m'` → 6~8 个姿态（含笔水平/朝上）→ `'s'` → **不重启、不发机械臂**。

**判读**: `isotropyRatio` 接近 1；`m` 在 0.35~0.50 kg；`c_s` 的 z 量级 ~50 mm；
两个 rms 都小。**把这四组数发回控制器。**

---

### Task 6: 运行时通道切换（本地全量补偿）

**Files:**
- Modify: `Touch_Client/force/ForceCalibration.cpp/.h`（文件格式）
- Modify: `Touch_Client/force/ForceCompensation.cpp/.h`（输入通道）
- Modify: `Touch_Client/main.cpp`

- [ ] **Step 1**: `force_calib.json` 新格式：存 `A`(9) + `b_F`(3) + `b_M`(3)，
  `version` 递增；**旧文件会被拒**（这是有意的，格式变了）。
- [ ] **Step 2**: `ForceCompensation::step` 的输入从 `fd.raw[]`(@576) 换成 `fd.sixForceRaw[]`(@1304)，
  补偿式 `compensated = F_1304 − b_F − A·g`。
- [ ] **Step 3**: 删掉 `setMassCom` / 残余概念 / A2 的"内存≠落盘"拆分。
- [ ] **Step 4**: 测试 + 完整构建。提交。

> ⚠️ 到这一步为止，**机械臂那边仍是旧配置** —— 本地补偿自成一体，不依赖它。

---

### Task 7: 实机验证 B（交用户）

`'m'` → `'s'` → **本地补偿后的读数应当与姿态无关**（这正是本地模型自洽的定义）。

---

### Task 8: 把参数传给机械臂（三条一起）

**Files:**
- Modify: `Touch_Client/relay/RelayCore.cpp/.h`
- Modify: `Touch_Client/main.cpp`

- [ ] **Step 1**: 加 `sendPayloadToRobot(m_full, c_flange[3], inertia)`，发：
  - `EnableRobot(m, cx, cy, cz)` —— 质量 + 质心（**唯一**通道）
  - `PayLoad(m, I)`（别名 `LoadSet`）—— 惯量
  - `LoadSwitch(1)` —— 打开负载设置（**现行代码发的是 `LoadSwitch(0)`，在 `RelayCore.cpp:473`；
    确认新流程不再拍掉它**）
- [ ] **Step 2**: **先打印、后发送**，分成两个动作：`'s'` 只算并打印候选值；
  另设一个显式键（如 `'p'`）才发送。发送前提示手离开工作空间。
- [ ] **Step 3**: `m_full` / `c_flange` 由标定结果换算（见 Task 9 的坐标系问题）。
- [ ] **Step 4**: 测试 + 完整构建。提交。

---

### Task 9: `c_s` 的坐标系对账（**必须先解决**）—— ✅ **离线部分已做完 (2026-09-19)**

**原问题**：标定 `c_s = 54.55 mm` 与解析几何的 130.3 mm 反推出"测量原点在法兰面下 75.8 mm"，
而传感器总高只有 31.5 mm —— 看着"对不上"。

**① 那个 75.8 mm 是【定义错位】，不是物理矛盾。**
`130.3 mm` 已**精确复现**：它是不含传感器那些零件相对法兰面的质心 (`Σmd/Σm = 49092.02/376.9
= 130.25`)。而 `c_s = 54.55` 是**测量原点以下**的质心 —— 那个集合**包含传感器自己朝工具侧的一段**。
**两个不同集合的质心相减，不是测量原点的位置。** 所以 75.8 这个数从来没有物理含义，也就
不该和 31.5 相等。（这条不依赖任何密度假设。）

**② CAD 的质量不可信 ⇒ 它也定不出原点。** 用户 2026-09-19 明示。实测：
`compute_payload.py` 总质量 **657 g** vs 标定 `m` **418.7 g** (差 57%)。脚本每个质量都建在
密度假设上（PETG 用**实心**体积×标称密度，没建打印填充率；笔杆用竹木 0.8；紧固件是估算的质点）。
`Hardware/README.md:66` 本来就留着一行`工具链实测总质量 (秤) | （待填）`。
⇒ **CAD 只能提供【位置】，不能提供【质量】；校正之前它的质心不能用。**

**③ 纯载荷数据【定不出】测量原点 —— 可识别性问题，不是数据量问题。**
传感器的每个读数都是相对它自己的测量原点；**一个薄的和一个厚的传感器，对同一负载给出逐位相同的
六维读数**。所以任何采集都定位不了原点，多采也没用。转接法兰同理。**必须有外部参照。**

**④ 外部参照就在 30004 帧里 —— 机械臂自己报的负载。**
```
| Load        | @1168 | 末端负载重量 (kg) |
| CenterX/Y/Z | @1176 | 负载偏心距离 (mm) |     ← 就是机械臂自己的法兰系
```
（`RelayCore.cpp:99-107` 已在读，只打印。）**实测记录**：
`机械臂自报 load = 0.404 kg  center = (0.3, -0.1, 68.7) mm`，且**回读与下发分毫不差**
⇒ 它确实采纳了负载（`Docs/superpowers/evidence/robot-baseline-report.md:52`、`progress.md:602`）。

**⑤ 于是 `d` 就算出来了，而且自洽：**
```
d = cz_robot − c_s = 68.7 − 54.55 = 14.15 mm
ψ 时代用机械臂自报基线独立解出 comZ = +66.3 mm  ⇒  d = 11.75 mm   (progress.md:603)
⇒ d ∈ [11.75, 14.15] mm —— 【落在传感器内部】(0~31.5) 且【靠近中位】(15.75)
```
> ⚠ **保留**：那个 68.7 是**我们自己**早先（ψ 时代 + CAD 种子 80.4）下发进去的，不是独立第三方。
> 所以 ⑤ 是一次**一致性检验**，不是证明。**它把 `d` 收进一个物理上合理的小区间，仅此而已。**

**⑥ 转接法兰厚度不必单独知道。** `cz = d + c_s`，而 `d` **已经含了**转接法兰厚度 + 传感器内部偏移。
**那个"和"有一个值就够。**

- [x] **Step 1**: ~~用 `compute_payload.py` 的几何独立算原点~~ → **不可行**（②③），已改为
      **用机械臂自报的 `center` 反解 `d`**，见 ①②④⑤。
- [x] **Step 2**: ~~用秤实测笔杆质量并覆盖~~ → **用户 2026-09-19 明示无法测量**。改由 ⑤ 的数据路线。
- [ ] **Step 3**: **判据（写死，不许放宽）**：任何候选 `cz` 必须让
      **`d = cz − c_s` 落在 `(0, 31.5) mm` 内**（传感器内部）。
      **落在外面 ⇒ 哪里错了, 不得下发。** 这条不需要任何额外测量。
- [ ] **Step 4**: 最终 `cz` 由 Task 10 的闭环确定（那才是测量），本任务只提供**区间与判据**。

---

### Task 10: 闭环 —— 从"验证"升级为**测量** `cz` 与 `m`

**为什么升级**：Task 9 ③ 已证：**载荷数据定不出测量原点**，而物理测量拿不到。
**闭环是唯一能把 `cz` 定出来的仪器**，而且它天然把转接法兰一起算进去。

**Files:**
- Modify: `Touch_Client/force/PayloadCalibration.cpp/.h`（或 main.cpp）
- Modify: `Touch_Client/main.cpp`

- [ ] **Step 1**: 实现 `fitAffine576From1304()`：拟合 `@576 = P·@1304 + q`（12 参数），
  报 `rms` 与 `P`。
  ✅ **离线已跑过 (2026-09-19, 用现成四次采集)**：力通道 `rms = 0.0091~0.0145 N`
  （12:38 那次算出 **0.014456**，与先前验证的 0.01446 逐位吻合 ⇒ 口径对得上）；
  **`P` 的迹/3 = 0.2082 / 0.2202 / 0.2107 / 0.2232** —— 四次稳定在 **0.21** 附近。
  ⚠ **力矩通道的 `P` 不可用**：元素高达 ±24（如 15:33 的 +15.08 / −24.61），rms 0.007~0.053 ——
  `M1304` 的动态范围太小（正是 ① 里那个量化问题的同源），回归量近共线，`P` 不可解释。
  **只用力通道那条。**
  ⚠⚠ **撤销一条先前写在这里的判据**：我写过"`P ≈ 1` 就说明 `@1304` 的量纲是 N"。
  **那是错的** —— `@576` 是机械臂用自己的负载模型减过之后的**残余**，所以 `P` 是
  **【未被补偿的那一份】**，不是单位换算。`P ≈ 0.21` 说明机械臂当前大约补偿掉了 79%。
  **量纲这件事不能靠 `P` 来判**（另见规格 §6b 里那条保留：真正的旁证是机械臂自报的
  `Load`（文档明确单位 kg）与标定的 `m` 吻合到 3.6%）。
- [ ] **Step 2**: 在 Task 8 发送**之前**与**之后**各跑一次，打印 `P`。
  **判据: 机械臂采纳正确负载后 `P` 应塌到 ≈0 —— ⚠ 但【只对前两行成立】。**
  实测（Step 1 的离线预跑）`P` 的**第三行比另两行小 8~15 倍**（四次一致），
  正是 ① 早已量到的"`@576` 的 z 响应坏掉（奇异值 [0.212 0.201 0.008]，秩 2）"。
  而**质量误差会三等分地影响三个力分量** —— 这里却是 x/y ~50%、z ~2%，**是结构性的**。
  ⇒ **`P` 的 z 行永远到不了 0，别拿它当判据**（否则会把一条永远失败的判据当成"标定没做好"）。
  ⚠ 这条要在实现时**写进代码注释**，不然下一个人会照 `||P|| → 0` 去判，然后永远失败。
  若日后的闭环要用 z 行，**先单独解释它为什么小**（是 `@576` 报得坏，还是机械臂的 z 补偿太强）
  —— 目前两者**没有分开**，**不许猜**。
- [ ] **Step 3**: 读回 `@1168 / @1176-1199`（机械臂自报），与下发值对账；并算 `d` 过 Task 9 Step 3 的判据。
- [ ] **Step 4**: 若 `P` 不塌，用它**反解**真值与候选值之差，迭代 `(m, cz)`。
  **起点已经很小**：`cz ≈ 68~70 mm`、`m ≈ 0.42~0.56 kg`（`m` 的上界含传感器机器人侧那一段 —— 见
  Task 9 ⑤ 下面的保留：机械臂自报的 `load = 0.404` **小于**标定的 `m = 0.4187`，方向反了，
  所以它大概率偏小 ~0.15 kg）。
- [ ] **Step 5**: 提交。

> 控制器已在真实数据上验证这条关系成立：`rms = 0.01446 N`（12 参数、21 方程）。
> 这是"对比本地处理后的数据与机械臂加工后的数据"的**定量形式**。
>
> **安全**（规格 §7）：机械臂在安全姿态、手离开工作空间、急停在手边、**一次改到目标值**。
> `m = 0.42 kg` 远小于那次惹祸的 1.5 kg。

---

### Task 11: 清理旧机制 —— **⚠ 范围实测后重估; 且【排在 Task 6 之后】**

**实测波及面（2026-09-19，`grep` 计数）** —— 这一节原来只写了 5 行，**低估了**：

| 符号 | 测试引用 | 生产引用 | 备注 |
|---|---|---|---|
| `solve(` | **20** | 13 | 删它要拆掉大半个测试套件（现全绿） |
| `comSignZ` | 4 | 13 | |
| `cTrueZ` | 3 | 7 | |
| `setSensorYawDeg` | 5 | 12 | |
| `sensorYawDeg` | 13 | **18** | **其中含活路径** |
| `SENSOR_MOUNT_YAW_DEG` | 2 | 9 | |

另：`main.cpp:1214~1478` 有 **264 行 `#if 0`** 装着整个旧模型。

**⚠⚠ 两条【不许现在做】的理由**

1. **`sensorYawDeg` 有活路径**：`ForceCompensation.cpp:261` 走 `TcpCalibration::gravitySensorFrame`，
   用的是模块态 ψ。而 **Task 6 本来就要改这里** —— 新模型的 `A` 吸收了安装旋转，
   本地补偿该用 `A`，不该再用 `Rz(-ψ)`。**先删 ψ 再改 Task 6 = 同一处做两遍，
   而且中间那一段时间里补偿与求解的约定不一致。**
2. **拆 `solve()` 要连测试一起拆**（20 处、多个整用例），而那是当前的回归网。

**⇒ 次序：Task 6 → Task 11。** 在 Task 6 落地、补偿改用 `A` 之后，ψ 才真正成为无引用物。

**Task 6 之后要做的事（范围不变，但那时是安全的）**

- [ ] 删 ψ 扫描（`PSI_MIN/MAX/STEP/N_SCAN`、扫描循环、`sensorYawDeg` 的持久化）
- [ ] 删 `PayloadCalibration::solve()` 与其辅助（`gravityTool` 的 ψ 参数、`buildRows`）——
      **连同引用它的那些 ψ 时代用例一起删**（是删，不是改写：那些用例断言的是旧模型的行为）
- [ ] 删 `comSignZ` / `cTrueZ[2]` / `com_sign_z`
- [ ] 删 `main.cpp` 里的 z 符号手工对齐（`f624b7e`）与那段 264 行 `#if 0`
  ⚠ **删之前先改一处测试锚点**（2026-09-19 复审判出，是**这次改动会踩的坑**，不是现有缺陷）:
  `tests/test_session_report.cpp` 里 `solve_path_prints_only_through_the_sink` 的扫描窗口
  **止于字面量 `#if 0  // ===`**（`main.cpp:1537`，那段死代码的起点）。删掉那段块之后
  `src.find("#if 0  // ===")` 会返回 `npos`，`CHECK(stop != npos)` **变红** ——
  而红的原因与那条闸门要守的东西毫无关系。**先换成别的稳定锚点（或改成扫到文件尾），再删块。**
- [ ] 删 `payload_calib.json` 的绝对质心驱动（改由 `EnableRobot` 直接发；文件只留记录）

**⚠ 明确【保留】** —— 与上面几行相反，这几样**不是**旧机制，删了会把今天的结论一起删掉：

- **`TcpCalibration::gravitySensorFrameAtYaw`**：新模型就是在用它（`psi` 传 0）。**留着。**
- **`TcpCalibration::gravitySensorFrame`**：`ForceCompensation` 在用。**留到 Task 6 一并处理。**
- **`@1168` 回声**：原计划写"保留作验证"，**现在要加一条更重的用途** ——
  它是 **Task 9 里定 `d` 的唯一外部参照**（机械臂自报的 `center` 就在它自己的法兰系）。
  **删它等于删掉原点问题的解。** 保留作：原点参照 + 闭环对账。
- **`Config::SENSOR_MOUNT_YAW_DEG`**：在 Task 6 之前仍是 `ForceCompensation` 的输入。
- [ ] 修正所有"改不动"的伪证文案（`PayloadCalibration.h:5,10,32,86-90`、`ForceCompensation.h:40`、
  `main.cpp:400-402`、`PayloadCalibration.cpp:190-192,263-267`）
- [ ] `run_tests.bat` 补 6 个漏掉的套件
- [ ] 提交
