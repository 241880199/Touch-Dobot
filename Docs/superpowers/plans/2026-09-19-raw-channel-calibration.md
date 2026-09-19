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

### Task 9: `c_s` 的坐标系对账（**必须先解决**）

**规格 §6b 已记下这个不一致**：标定 `c_s = 54.55 mm` 与解析几何的 130.3 mm 反推出
传感器测量原点在法兰面下方 75.8 mm，而传感器总高只有 31.5 mm —— **对不上**。

- [ ] **Step 1**: 用 `compute_payload.py` 的几何，独立算出"传感器测量原点相对法兰面"的可能位置，
  与标定的 `c_s` 对照。
- [ ] **Step 2**: 若解析模型的零件参数可疑（脚本自述"笔杆密度建议用秤实测"），
  **用秤实测笔杆质量并覆盖**，重算。
- [ ] **Step 3**: 结论写进规格；**`c_flange` 的换算在解决前不得用于发送**。

---

### Task 10: 闭环验证 —— `P` 判据

**Files:**
- Modify: `Touch_Client/force/PayloadCalibration.cpp/.h`（或 main.cpp）
- Modify: `Touch_Client/main.cpp`

- [ ] **Step 1**: 实现 `fitAffine576From1304()`：拟合 `@576 = P·@1304 + q`（12 参数），
  报 `rms` 与 `P`。
- [ ] **Step 2**: 在 Task 8 发送**之前**与**之后**各跑一次，打印 `P`。
  **判据: 机械臂采纳正确负载后 `P` 应塌到 ≈0。**
- [ ] **Step 3**: 提交。

> 控制器已在真实数据上验证这条关系成立：`rms = 0.01446 N`（12 参数、21 方程）。
> 这是"对比本地处理后的数据与机械臂加工后的数据"的**定量形式**。

---

### Task 11: 清理旧机制

- [ ] 删 ψ 扫描（`PSI_MIN/MAX/STEP/N_SCAN`、扫描循环、`sensorYawDeg` 的持久化）
- [ ] 删 `comSignZ` / `cTrueZ[2]` / `com_sign_z`
- [ ] 删 `main.cpp` 里的 z 符号手工对齐（`f624b7e`）
- [ ] 删 `@1168` 回声作**求解基线**（保留作**验证**）
- [ ] 删 `payload_calib.json` 的绝对质心驱动（改由 `EnableRobot` 直接发；文件只留记录）
- [ ] 修正所有"改不动"的伪证文案（`PayloadCalibration.h:5,10,32,86-90`、`ForceCompensation.h:40`、
  `main.cpp:400-402`、`PayloadCalibration.cpp:190-192,263-267`）
- [ ] `run_tests.bat` 补 6 个漏掉的套件
- [ ] 提交
