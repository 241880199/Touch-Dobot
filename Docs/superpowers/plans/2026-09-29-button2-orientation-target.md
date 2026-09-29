# 按钮2「姿态目标 + 解 J4/J5/J6」实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把按钮2 的"一根器件轴喂一个关节"换成"先算**想要的末端朝向**，再用腕部雅可比解 J4/J5/J6"，
使得**器件前摆 ⇒ 末端往基座 +Y 摆**在任意位姿下都成立。

**Architecture:** 纯函数三段拼接 —— ①器件姿态增量 → ②目标末端朝向（**摆动左乘锁基座 + 自转右乘绕自身轴**）
→ ③3×3 角雅可比 + FK 回代的牛顿迭代解出 J4/J5/J6。安全链路（I1/I2/FK 位置门/逐帧限幅）**一字不动**。
旧实现**保留**在一个编译期开关后面作 A/B 与回滚。

**Tech Stack:** C++17 / MSVC；测试走 `Touch_Client/tests/`（套件数运行时数出并断言，当前 26）。

**Spec:** `Docs/superpowers/specs/2026-09-29-button2-orientation-target-design.md`（**先读它**；
其中的 §1 已被 2026-09-29 晚的「修订」取代，读修订那一节）。

## Global Constraints

- **回滚 = 翻一个 bool**：`Config::BTN2_ORIENT_TARGET_ENABLED`（默认 `true`）。翻 false ⇒ **逐字**走旧的
  `button2JointTarget`。**旧函数一个字都不许改。**
- **`RelayCore.cpp` 不被任何测试编译** ⇒ 接线那一步**没有自动化用例**，只能靠复审 + 上机。**如实写在报告里。**
- **不许改**：`ORIENT_DEADZONE_DEG` / `ORIENT_MAX_OFFSET_DEG` / `ORIENT_MAX_STEP_DEG` / `BTN2_J*_SIGN` 的**现值**；
  I1/I2/FK 位置门/`clampJointStep` 的**控制流**。
- C++ 注释中文；`.bat` 纯 ASCII；每步提交后整床 **exit 0** 且 `Suites accounted: N of N`。
- ⚠ **本仓最怕假绿**：新用例的期望值**不许**写成"动了就算过"（`>1e-6` 那种）。凡涉及数值必须断言**具体数**，
  且**每条新判据都要有一条负对照实测红**（把实现改坏 ⇒ 该用例必须红）。
- 单位：**全程用度**（`rotVecDeg` 返回度；角雅可比的角速度行是单位轴 ⇒ `Jw·dq` 与 `dq` 同单位）。

## File Structure

| 文件 | 职责 | 动作 |
|---|---|---|
| `Touch_Client/relay/Button2Joint.h` | 3×3 纯算术（inline，**可测**）· 两个新纯函数的**契约** | Modify |
| `Touch_Client/relay/Button2Joint.cpp` | 两个新纯函数的实现 | Modify |
| `Touch_Client/config/Config.h` | 4 个新常数 + 1 个开关 | Modify |
| `Touch_Client/tests/test_button2_joint.cpp` | 新增用例 ⑲~㉕ | Modify |
| `Touch_Client/relay/RelayCore.cpp` | 调用点分叉（开关） | Modify |
| `Touch_Client/tests/build_button2_joint_test.bat` | **不动**（依赖不变：`Kinematics.cpp` + `TcpCalibration.cpp` 已在链上） | — |

---

### Task 1: 3×3 纯算术（旋转向量↔矩阵 · 乘 · 转置 · 求逆）

**Files:**
- Modify: `Touch_Client/relay/Button2Joint.h`（在 `button2JointTarget` 声明**之前**插入一段 inline 实现）
- Test: `Touch_Client/tests/test_button2_joint.cpp`（新增用例 ⑲）

**Interfaces:**
- Produces（后续任务全靠这四个）：
  ```cpp
  void button2RotVecToMatDeg(const double rv[3], double R[9]);   // 行主序 R[3*r+c]
  void button2Mat3Mul(const double A[9], const double B[9], double out[9]);
  void button2Mat3T(const double A[9], double out[9]);
  bool button2Mat3Inv(const double A[9], double out[9]);         // 奇异 ⇒ false
  ```
  ⚠ 行主序：`R[r*3+c]` 是第 r 行第 c 列。**与 `Kinematics::composeTransform` 的 `T[i][j]` 同一约定**
  （已核：`Kinematics.cpp:19-45` 是 `T = T*R` 行主序，`T[0..2][0..2]` 即旋转）。

- [ ] **Step 1: 写失败用例 ⑲**

在 `tests/test_button2_joint.cpp` 的 `main()` 里 `ⓡ⑱` 之后追加：

```cpp
    // ⑲ 3×3 纯算术：旋转向量↔矩阵 的往返 + 一个【具体数】的已知值
    //    ⚠ 不用"动了就算过"的断言：这里钉的是具体数值。
    {
        TEST(⑲(a) 往返: rotVecDeg∘rotVecToMatDeg 还原原向量);
        const double cases[4][3] = {{0,0,0}, {10,0,0}, {0,-25,0}, {12,-7,3}};
        for (int c = 0; c < 4; ++c) {
            double R[9], rv[3];
            button2RotVecToMatDeg(cases[c], R);
            button2RotVecDegForTest(R, rv);        // 复用 Button2Joint.cpp 里的实现，经下面 Step 3 暴露
            for (int i = 0; i < 3; ++i)
                CHECK(std::fabs(rv[i] - cases[c][i]) < 1e-9);
        }
    }
    {
        TEST(⑲(b) 绕 X 转 90° 的具体矩阵);
        const double rv[3] = {90, 0, 0};
        double R[9];
        button2RotVecToMatDeg(rv, R);
        // Rx(90) = [[1,0,0],[0,0,-1],[0,1,0]]（行主序）
        const double exp[9] = {1,0,0, 0,0,-1, 0,1,0};
        for (int i = 0; i < 9; ++i) CHECK(std::fabs(R[i] - exp[i]) < 1e-9);
    }
    {
        TEST(⑲(c) 乘与转置：A·Aᵀ == I（A 是旋转）);
        const double rv[3] = {12, -7, 3};
        double A[9], At[9], P[9];
        button2RotVecToMatDeg(rv, A);
        button2Mat3T(A, At);
        button2Mat3Mul(A, At, P);
        for (int i = 0; i < 9; ++i)
            CHECK(std::fabs(P[i] - (i % 4 == 0 ? 1.0 : 0.0)) < 1e-9);
    }
    {
        TEST(⑲(d) 求逆：A·A⁻¹ == I；奇异矩阵返回 false);
        const double rv[3] = {12, -7, 3};
        double A[9], Ai[9], P[9];
        button2RotVecToMatDeg(rv, A);
        CHECK(button2Mat3Inv(A, Ai));
        button2Mat3Mul(A, Ai, P);
        for (int i = 0; i < 9; ++i)
            CHECK(std::fabs(P[i] - (i % 4 == 0 ? 1.0 : 0.0)) < 1e-9);
        const double sing[9] = {1,2,3, 2,4,6, 0,0,1};   // 第 1、2 行线性相关
        CHECK(!button2Mat3Inv(sing, Ai));
    }
```

**⚠ 负数步的用例**：`{0,-25,0}` 与 `{12,-7,3}` 必须收进来 —— 本仓有成文教训：
只测单轴正值时，符号错了也全绿。

- [ ] **Step 2: 跑，确认失败**

Run: `Touch_Client\tests\build_button2_joint_test.bat`
Expected: **编译失败**（`button2RotVecToMatDeg` 未声明）—— 那正是"红"。

- [ ] **Step 3: 实现（`Button2Joint.h`）**

在 `void button2JointTarget(...)` 声明**之前**插入：

```cpp
// ============================================================================
//  3×3 纯算术（inline，放头文件里是为了【可测】—— 同 ForcePipeline.h 的 softDeadzone 先例）
// ============================================================================
// ⚠ 行主序：R[r*3+c] = 第 r 行第 c 列。与 Kinematics 的 T[i][j] 同一约定（已核 Kinematics.cpp:19-45）。
// ⚠ 单位：角度一律【度】。
inline void button2RotVecToMatDeg(const double rv[3], double R[9]) {
    const double D2R = 3.14159265358979323846 / 180.0;
    const double x = rv[0] * D2R, y = rv[1] * D2R, z = rv[2] * D2R;
    const double th2 = std::sqrt(x * x + y * y + z * z);
    if (th2 < 1e-12) {                       // θ≈0 ⇒ 单位阵（含 rv 全 0）
        R[0]=1; R[1]=0; R[2]=0; R[3]=0; R[4]=1; R[5]=0; R[6]=0; R[7]=0; R[8]=1;
        return;
    }
    // Rodrigues：R = I + sinθ·[u]× + (1-cosθ)·[u]×²
    const double k = 1.0 / th2;              // 归一化
    const double ux = x * k, uy = y * k, uz = z * k;
    const double c = std::cos(th2), s = std::sin(th2), t = 1.0 - c;
    R[0] = c + ux*ux*t;        R[1] = ux*uy*t - uz*s;   R[2] = ux*uz*t + uy*s;
    R[3] = uy*ux*t + uz*s;     R[4] = c + uy*uy*t;      R[5] = uy*uz*t - ux*s;
    R[6] = uz*ux*t - uy*s;     R[7] = uz*uy*t + ux*s;   R[8] = c + uz*uz*t;
}

inline void button2Mat3Mul(const double A[9], const double B[9], double out[9]) {
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) {
            double s = 0.0;
            for (int k = 0; k < 3; ++k) s += A[r*3+k] * B[k*3+c];
            out[r*3+c] = s;
        }
}

inline void button2Mat3T(const double A[9], double out[9]) {
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) out[c*3+r] = A[r*3+c];
}

// 伴随矩阵法求逆；|det| 太小 ⇒ false（调用方据此拒发，别给一个离谱的解）
inline bool button2Mat3Inv(const double A[9], double out[9]) {
    const double c00 = A[4]*A[8] - A[5]*A[7];
    const double c01 = A[5]*A[6] - A[3]*A[8];
    const double c02 = A[3]*A[7] - A[4]*A[6];
    const double det = A[0]*c00 + A[1]*c01 + A[2]*c02;
    if (std::fabs(det) < 1e-12) return false;
    const double id = 1.0 / det;
    out[0] = c00*id;                              out[1] = (A[2]*A[7]-A[1]*A[8])*id;  out[2] = (A[1]*A[5]-A[2]*A[4])*id;
    out[3] = c01*id;                              out[4] = (A[0]*A[8]-A[2]*A[6])*id;  out[5] = (A[2]*A[3]-A[0]*A[5])*id;
    out[6] = c02*id;                              out[7] = (A[1]*A[6]-A[0]*A[7])*id;  out[8] = (A[0]*A[4]-A[1]*A[3])*id;
    return true;
}
```

- [ ] **Step 4: 把 `rotVecDeg` 暴露成可测的薄壳**

`Button2Joint.cpp` 里 `rotVecDeg` 目前在匿名命名空间，测试看不见。**不改它**，只**加一个薄壳**：

```cpp
// 仅供测试：把文件内的 rotVecDeg 暴露出来（不改实现，只转发）
void button2RotVecDegForTest(const double R[9], double rv[3]) { rotVecDeg(R, rv); }
```
并在 `Button2Joint.h` 里声明它（注释写清"**只为测试暴露**，生产路径不经它"）。

- [ ] **Step 5: 跑，确认通过**

Run: `build_button2_joint_test.bat` 然后 `test_button2_joint.exe`
Expected: ⑲ 四条**全绿**，其余用例不受影响。

- [ ] **Step 6: 负对照（必须实测红）**

把 `button2RotVecToMatDeg` 的 Rodrigues 里 `(1-c)` 改成 `(1+c)` ⇒ **⑲(b) 必须红**。
改回。把 `button2Mat3Inv` 的 `1.0/det` 改成 `1.0/fabs(det)` ⇒ **⑲(d) 的 `{0,-25,0}` 那条必须红**。改回。

- [ ] **Step 7: 整床 + 提交**

```bash
cd Touch_Client/tests && cmd //c ".\run_tests.bat"   # 期望 exit 0 且 Suites accounted: 26 of 26
git add Touch_Client/relay/Button2Joint.h Touch_Client/relay/Button2Joint.cpp Touch_Client/tests/test_button2_joint.cpp
git commit -m "feat(btn2): 3×3 纯算术（rotVec↔Mat / 乘 / 转置 / 求逆）+ 用例⑲（含负对照）"
```

---

### Task 2: 目标末端朝向（摆动锁基座 + 自转绕自身轴）

**Files:**
- Modify: `Touch_Client/config/Config.h`（**本任务就要加常数** —— 自审时抓到：Task 5 才加的话 Task 2 编不过）
- Modify: `Touch_Client/relay/Button2Joint.h`（声明）
- Modify: `Touch_Client/relay/Button2Joint.cpp`（实现）
- Test: `Touch_Client/tests/test_button2_joint.cpp`（新增用例 ⑳ ⑳b ⑳c ㉑）

**Interfaces:**
- Consumes: Task 1 的四个 helper。
- Produces（Task 4/5 依赖）：`Config::BTN2_TILT_SIGN_X` / `BTN2_TILT_SIGN_Y` / `BTN2_ROLL_SIGN` /
  `BTN2_TILT_PHI_DEG` / `BTN2_ORIENT_TARGET_ENABLED` / `BTN2_WRIST_TOL_DEG` /
  `BTN2_WRIST_MAX_ITER` / `BTN2_WRIST_DAMP`。

- [ ] **Step 0: `Config.h` 加常数**（放在 `BTN2_JOINT_SPACE_ENABLED` 那一段之后，与它同风格）

```cpp
    // ★★★ 2026-09-29 按钮2 新实现（"姿态目标 + 解 J4/J5/J6"）—— 设计见
    //   `Docs/superpowers/specs/2026-09-29-button2-orientation-target-design.md`（读【修订】那一节）
    //   回滚 = 翻 `BTN2_ORIENT_TARGET_ENABLED` ⇒ 逐字走旧的 `button2JointTarget`
    //   （那是"一根器件轴喂一个关节"的老映射，**一个字都不许改**）。
    const bool   BTN2_ORIENT_TARGET_ENABLED = true;
    const double BTN2_TILT_PHI_DEG  = 0.0;    // M = Rx(φ)。φ=0 由用户 2026-09-29 定案（M = I）
    const double BTN2_TILT_SIGN_X   = -1.0;   // ⚠ 器件侧约定，**【上机看方向确认】**，默认沿用 BTN2_J4_SIGN
    const double BTN2_TILT_SIGN_Y   = -1.0;   // ⚠ 同上，默认沿用 BTN2_J5_SIGN
    const double BTN2_ROLL_SIGN     = -1.0;   // 沿用 BTN2_J6_SIGN 的实测结论
    const double BTN2_WRIST_TOL_DEG = 0.05;   // 牛顿迭代收敛门限（度）
    const int    BTN2_WRIST_MAX_ITER = 24;
    const double BTN2_WRIST_DAMP    = 1e-3;   // 阻尼 λ 的下限
```
- Produces:
  ```cpp
  // refR/outR: 行主序 3x3；refStylus/curStylus: {Rx,Ry,Rz} 度（同 appState.stylusOrient 次序）
  // ★ phiDeg 是【入参】而不是直接读 Config —— 理由见下面实现那一段的 ⚠：
  //   编译期常数在运行期改不了 ⇒ 用例写不出有牙齿的对照（那本案就会退化成让实现自己给自己背书）。
  //   **所有调用点（用例 ⑳/⑳b/⑳c/⑳d/㉑ 与 Task 4）一律传 `Config::BTN2_TILT_PHI_DEG`。**
  void button2OrientTarget(const double refR[9], const double refStylus[3],
                           const double curStylus[3], double phiDeg, double outR[9]);
  ```

**公式（设计 §1 修订版）：**

```
① R_s_ref = rpyToMatrix(refStylus)   R_s_cur = rpyToMatrix(curStylus)
② ΔR = R_s_refᵀ · R_s_cur                      （器件系增量）
③ rv = rotVecDeg(ΔR)                           （度）
④ 摆动: ω_base = (SX·rv[0], SY·rv[1], 0)       （φ=0 ⇒ M=I；SX/SY 是两个待上机确认的符号）
   R_tilt = rotVecToMatDeg(ω_base) · refR      ← 左乘 = 在基座系里摆
⑤ 自转: R_new = R_tilt · Rz(S6·rv[2])          ← 右乘 = 绕【新的】末端自身轴
```

- [ ] **Step 1: 写失败用例 ⑳（**验收标准本身**）**

```cpp
    // ⑳ ★ 验收标准：器件前摆 ⇒ 末端往基座 +Y 摆（任意位姿下都成立）
    //    判据：绕基座 +X 正转 θ 时，末端轴（参考位姿下朝下）的 tip 应往 +Y 移动。
    {
        TEST(⑳ 前摆 ⇒ tip 往基座 +Y 摆（三个不同位姿）);
        const double poses[3][6] = {
            {0,0,0,0,-90,0}, {30,-60,45,20,-70,10}, {-120,40,-30,90,-45,180}
        };
        for (int p = 0; p < 3; ++p) {
            double T[4][4]; Kinematics::composeTransform(poses[p], T);
            double refR[9];
            for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) refR[r*3+c] = T[r][c];
            // 参考笔杆姿态 = 单位（试两个不同参考，证明与参考无关）
            const double refStylus[3] = {0, 0, 0};
            // 构造"器件 X 轴 +10°"：按定义 ΔRx=+10 ≡ R·Rx(10) ⇒ 用 rpyToMatrix 造 curStylus
            double Rx[9]; button2RotVecToMatDeg((const double[]){10,0,0}, Rx);
            double Rs[9]; TcpCalibration::rpyToMatrix(0,0,0, Rs);
            double Rs_cur[9]; button2Mat3Mul(Rs, Rx, Rs_cur);
            double cur[3]; button2RotVecDegForTest(Rs_cur, cur);   // ← 见下：需要矩阵→欧拉
            ...
        }
    }
```

⚠ **这一步遇到一个真实的接口问题**：造"绕器件 X 转 10°"的**欧拉输入**需要一个**矩阵→欧拉**的函数，
而本仓**没有**（`rpyToMatrix` 只有单向）。**计划选 (a)**：用例侧自写一个 `matToRpyZYX`（约 12 行，
**只用在测试里**，注释写明）。**不做 (b)**（为测试去扩生产头文件的接口）。

```cpp
// ⚠ 只在本【用例文件】里用：矩阵 → ZYX 欧拉（度）。用来【造输入】。
//   生产路径【不】用它的逆——`rpyToMatrix` 才是欧拉约定的唯一真相源。
static void matToRpyZYX(const double R[9], double rpy[3]) {
    const double D2R = 3.14159265358979323846 / 180.0;
    const double R2D = 1.0 / D2R;
    double sy = -R[6];
    if (sy > 1.0) sy = 1.0;  if (sy < -1.0) sy = -1.0;
    const double ry = std::asin(sy);
    double rx, rz;
    if (std::fabs(sy) < 0.999999) {                 // 常规支路
        rx = std::atan2(R[7], R[8]);
        rz = std::atan2(R[3], R[0]);
    } else {                                        // 万向锁：rz 置 0（与 HapticCallback 同一取舍）
        rx = std::atan2(-R[5], R[4]);
        rz = 0.0;
    }
    rpy[0] = rx * R2D; rpy[1] = ry * R2D; rpy[2] = rz * R2D;
}
```

⇒ 用例 ⑳ 的实际写法（**完整**）：

```cpp
    {
        TEST(⑳ 前摆 ⇒ 末端 tip 往基座 +Y 摆（三个位姿 × 两个参考笔杆姿态）);
        const double poses[3][6] = {
            {0,0,0,0,-90,0}, {30,-60,45,20,-70,10}, {-120,40,-30,90,-45,180}
        };
        const double refs[2][3] = {{0,0,0}, {-58.93, 11.83, -6.41}};
        for (int p = 0; p < 3; ++p) {
            double T[4][4]; Kinematics::composeTransform(poses[p], T);
            double refR[9];
            for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) refR[r*3+c] = T[r][c];
            for (int q = 0; q < 2; ++q) {
                // 造输入：绕【器件 X】转 10° ⇒ R_cur = R_refStylus · Rx(10)
                double Rs[9], Rx[9], Rsc[9];
                TcpCalibration::rpyToMatrix(refs[q][0], refs[q][1], refs[q][2], Rs);
                const double a10[3] = {10, 0, 0};
                button2RotVecToMatDeg(a10, Rx);
                button2Mat3Mul(Rs, Rx, Rsc);
                double curStylus[3]; matToRpyZYX(Rsc, curStylus);

                double outR[9];
                button2OrientTarget(refR, refs[q], curStylus, outR);

                // 期望 = rotVecToMatDeg(SX·10, 0, 0) · refR  ← 经 SX 表达，【不写死符号】
                double Mw[9], expR[9];
                const double w[3] = { Config::BTN2_TILT_SIGN_X * 10.0, 0.0, 0.0 };
                button2RotVecToMatDeg(w, Mw);
                button2Mat3Mul(Mw, refR, expR);
                for (int i = 0; i < 9; ++i) CHECK(std::fabs(outR[i] - expR[i]) < 1e-9);

                // ★ 与实现无关的**【验收判据】**：这次姿态变化的【世界系旋转向量】必须
                //   **沿基座 X 轴**（即"往 +Y 摆"的那根轴），其余两分量必须是 0。
                //   ⚠ 不去猜"FK 末端系的哪一列是笔尖指向"—— 那是未核实的前提（计划自审时抓到）。
                double outT[9], D[9], refT[9];
                button2Mat3T(outR, outT);
                button2Mat3Mul(outR, refT /*见下*/, D);
                double rvw[3]; button2RotVecDegForTest(D, rvw);
                CHECK(std::fabs(rvw[1]) < 1e-9);   // 无 Y 分量
                CHECK(std::fabs(rvw[2]) < 1e-9);   // 无 Z 分量
                CHECK(std::fabs(std::fabs(rvw[0]) - 10.0) < 1e-9);   // 大小 = 10°（符号经 SX 表达）
            }
        }
    }
```

⚠ 上面那段里 `refT` 必须**先**由 `button2Mat3T(refR, refT);` 得到（`D = outR · refRᵀ`
就是"这次变化在世界系里的旋转"）。补上那一行：`button2Mat3T(refR, refT);` 放在 `button2Mat3Mul` 之前。

- [ ] **Step 2: 写失败用例 ㉑（自转 ⇒ 只绕末端自身轴）**

```cpp
    {
        TEST(㉑ 自转 ⇒ 末端轴指向不变、绕自身轴转 ROLL_SIGN·20°);
        const double poses[2][6] = {{0,0,0,0,-90,0}, {30,-60,45,20,-70,10}};
        for (int p = 0; p < 2; ++p) {
            double T[4][4]; Kinematics::composeTransform(poses[p], T);
            double refR[9];
            for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) refR[r*3+c] = T[r][c];
            const double refStylus[3] = {0, 0, 0};
            double Rs[9], Rz20[9], Rsc[9];
            TcpCalibration::rpyToMatrix(0,0,0, Rs);
            const double a20[3] = {0, 0, 20};
            button2RotVecToMatDeg(a20, Rz20);
            button2Mat3Mul(Rs, Rz20, Rsc);
            double curStylus[3]; matToRpyZYX(Rsc, curStylus);

            double outR[9];
            button2OrientTarget(refR, refStylus, curStylus, outR);

            // 判据1：接近轴（第 3 列）逐位不变
            for (int r = 0; r < 3; ++r) CHECK(std::fabs(outR[r*3+2] - refR[r*3+2]) < 1e-9);
            // 判据2：outR·refRᵀ 是绕该轴转 |ROLL_SIGN·20| 的旋转
            double outT[9], D[9];
            button2Mat3T(outR, outT);
            button2Mat3Mul(outR, outT, D);   // 应≈I（两矩阵都正交）—— 顺带证明 outR 是旋转
            for (int i = 0; i < 9; ++i)
                CHECK(std::fabs(D[i] - (i % 4 == 0 ? 1.0 : 0.0)) < 1e-9);
            double Rs2[9], Rr[9], dR[9];
            button2Mat3T(refR, Rs2);
            button2Mat3Mul(outR, Rs2, dR);
            double rvd[3]; button2RotVecDegForTest(dR, rvd);
            double ax[3] = {refR[2], refR[5], refR[8]};      // 接近轴
            double n = std::sqrt(ax[0]*ax[0]+ax[1]*ax[1]+ax[2]*ax[2]);
            for (int i = 0; i < 3; ++i) ax[i] /= n;
            for (int i = 0; i < 3; ++i)
                CHECK(std::fabs(rvd[i] - ax[i] * Config::BTN2_ROLL_SIGN * 20.0) < 1e-6);
        }
    }
```

- [ ] **Step 2b: 死区与偏移限幅 —— 设计 §3「语义要重新定的两个常数」（**别漏，自审时抓到的**）**

在 `button2OrientTarget` 里算完 `rv` 之后、构造 `w`/`roll` **之前**：

```cpp
    // ① 逐分量死区（与旧实现 axisGate **逐字同语义**：>= 门限才放行，`>` 会把"恰好等于"吞掉）
    const double dz = Config::ORIENT_DEADZONE_DEG;
    for (int i = 0; i < 3; ++i)
        if (!(std::fabs(rv[i]) >= dz)) rv[i] = 0.0;

    // ② 整体偏移限幅：ΔR 的【旋转角】θ > 150° ⇒ 把 ΔR【整体缩比】到 150°（轴不变）。
    //    ⚠ 不是逐分量夹 —— 逐分量夹会破坏方向（解出的一根向量被夹歪，末端就朝错的方向走）。
    {
        double rvAll[3]; button2RotVecDegForTest(dR, rvAll);
        const double th = std::sqrt(rvAll[0]*rvAll[0] + rvAll[1]*rvAll[1] + rvAll[2]*rvAll[2]);
        if (th > Config::ORIENT_MAX_OFFSET_DEG && th > 1e-12) {
            const double k = Config::ORIENT_MAX_OFFSET_DEG / th;
            for (int i = 0; i < 3; ++i) rv[i] *= k;      // 缩比在【分量】上做，轴不变 ⇒ 方向不变
        }
    }
```

**用例**（加进 ⑳ 之后）：

```cpp
    {
        TEST(⑳b 死区：低于门限的分量必须归 0；恰好等于门限必须放行);
        const double refR[9] = {1,0,0, 0,1,0, 0,0,1};
        const double refStylus[3] = {0,0,0};
        {   // 低于门限 ⇒ 输出 == refR（一点没动）
            double Rs[9], Rx[9], Rsc[9], cur[3], outR[9];
            TcpCalibration::rpyToMatrix(0,0,0, Rs);
            const double a[3] = { Config::ORIENT_DEADZONE_DEG * 0.5, 0, 0 };
            button2RotVecToMatDeg(a, Rx); button2Mat3Mul(Rs, Rx, Rsc); matToRpyZYX(Rsc, cur);
            button2OrientTarget(refR, refStylus, cur, outR);
            for (int i = 0; i < 9; ++i) CHECK(std::fabs(outR[i] - refR[i]) < 1e-12);
        }
        {   // 恰好等于门限 ⇒ 必须放行（与 `>` 相反）
            double Rs[9], Rx[9], Rsc[9], cur[3], outR[9];
            TcpCalibration::rpyToMatrix(0,0,0, Rs);
            const double a[3] = { Config::ORIENT_DEADZONE_DEG, 0, 0 };
            button2RotVecToMatDeg(a, Rx); button2Mat3Mul(Rs, Rx, Rsc); matToRpyZYX(Rsc, cur);
            button2OrientTarget(refR, refStylus, cur, outR);
            double outT[9], D[9], refT[9];
            button2Mat3T(outR, outT); button2Mat3T(refR, refT); button2Mat3Mul(outR, refT, D);
            double rv2[3]; button2RotVecDegForTest(D, rv2);
            CHECK(std::fabs(std::fabs(rv2[0]) - Config::ORIENT_DEADZONE_DEG) < 1e-9);
        }
    }
    {
        TEST(⑳c 偏移限幅：超 150° ⇒ 旋转角被夹到 150°，且【轴不变】);
        // 造一个"器件 X 轴转 170°"的输入；期望：世界系旋转向量 = 基座 X × 150°
        // ⚠ 轴不变是这条的要点：逐分量夹会把轴也改掉 ⇒ 复算轴与 150° 前的轴比
    }
```
⚠ ⑳c 的完整断言照 ⑳ 的写法（`rotvec(outR·refRᵀ)` 的三个分量）来写；
**负对照**：把整体缩比换成逐分量 `clamp` ⇒ ⑳c 的"轴不变"必须红。

```cpp
    {
        TEST(⑳d ★ φ 不是死常数：φ=90° 时"器件左右摆"必须映到【绕基座 Z 转】);
        //   今天 φ=0 ⇒ 左右摆映到绕基座 Y。把 φ 传成 90° 时，(0,cosφ,sinφ) = (0,0,1)
        //   ⇒ 世界系旋转向量必须【沿基座 Z】且大小仍是 10°。**这一条专治"常数没被用上"**。
        const double refR[9] = {1,0,0, 0,1,0, 0,0,1};
        const double refStylus[3] = {0,0,0};
        double Rs[9], Ry10[9], Rsc[9], cur[3], outR[9];
        TcpCalibration::rpyToMatrix(0,0,0, Rs);
        const double a[3] = {0, 10, 0};                       // 器件 Y 轴 +10°（左右摆）
        button2RotVecToMatDeg(a, Ry10);
        button2Mat3Mul(Rs, Ry10, Rsc);
        matToRpyZYX(Rsc, cur);
        button2OrientTarget(refR, refStylus, cur, 90.0, outR);   // ← φ = 90

        double refT[9], D[9];
        button2Mat3T(refR, refT);
        button2Mat3Mul(outR, refT, D);                        // 世界系增量
        double rvw[3]; button2RotVecDegForTest(D, rvw);
        CHECK(std::fabs(rvw[0]) < 1e-9);                      // 无基座 X 分量
        CHECK(std::fabs(rvw[1]) < 1e-9);                      // 无基座 Y 分量
        CHECK(std::fabs(std::fabs(rvw[2]) - 10.0) < 1e-9);    // 大小 = 10°（方向由 SY 定，故取绝对值）
    }
```
**★ 负对照（**保证红**，不依赖任何假设）**：把实现里的 `phi` 恒置 0（即把入参丢掉）
⇒ `rvw[2]` 会变成 ≈0、而 `rvw[1]` 变成 ±10 ⇒ **⑳d 必须红**。
⚠ 特别提醒：**这条对照是写给"常数没被用上"这个具体缺陷的** —— 本仓栽过"常数写进去但没人读"，
而那种缺陷**在 φ=0 的默认值下完全看不出来**（今天正是 φ=0）。

- [ ] **Step 3: 跑，确认失败**（未声明）
- [ ] **Step 4: 实现 `button2OrientTarget`**

```cpp
void button2OrientTarget(const double refR[9], const double refStylus[3],
                         const double curStylus[3], double outR[9]) {
    double Rref[9], Rcur[9], RrefT[9], dR[9];
    TcpCalibration::rpyToMatrix(refStylus[0], refStylus[1], refStylus[2], Rref);
    TcpCalibration::rpyToMatrix(curStylus[0],   curStylus[1],   curStylus[2],   Rcur);
    button2Mat3T(Rref, RrefT);
    button2Mat3Mul(RrefT, Rcur, dR);                    // ΔR = R_refᵀ · R_cur（器件系）
    double rv[3]; button2RotVecDegForTest(dR, rv);

    // 摆动向量先按逐轴符号折进去，再整体乘 M = Rx(φ)。
    //   ⚠ φ 必须【真的被用上】，而且必须【能测】：
    //     ① 写成"硬编成 0"会把这个常数变成**死常数**（上机调它没效果，调用方以为调了）；
    //     ② 它若是编译期常数，用例就没法在运行期改它 ⇒ 写不出有牙齿的对照。
    //   ⇒ **φ 是入参**，调用点传 `Config::BTN2_TILT_PHI_DEG`（见 Task 5）。默认 0 ⇒ M = I。
    const double D2R0 = 3.14159265358979323846 / 180.0;
    const double phi = phiDeg * D2R0;
    const double t0 = Config::BTN2_TILT_SIGN_X * rv[0];
    const double t1 = Config::BTN2_TILT_SIGN_Y * rv[1];
    const double w[3] = { t0, std::cos(phi) * t1, std::sin(phi) * t1 };   // = Rx(φ)·(t0,t1,0)
    double Rtilt[9], Mw[9];
    button2RotVecToMatDeg(w, Mw);
    button2Mat3Mul(Mw, refR, Rtilt);                     // 左乘 = 基座系里摆

    const double roll = Config::BTN2_ROLL_SIGN * rv[2];
    double Mr[9], Rroll[9];
    button2RotVecToMatDeg((const double[]){0, 0, roll}, Mr);
    button2Mat3Mul(Rtilt, Mr, outR);                     // 右乘 = 绕新的自身轴
}
```
⚠ `(const double[]){...}` 是 C99 复合字面量，**MSVC 的 C++ 不支持**。改用局部数组变量。

- [ ] **Step 5: 跑，确认通过**
- [ ] **Step 6: 负对照**：把第 ④ 步的左乘改成右乘（`button2Mat3Mul(refR, Mw, ...)`）⇒ **⑳ 必须红**；
  把第 ⑤ 步的右乘改成左乘 ⇒ **㉑ 必须红**。各自改回。
- [ ] **Step 7: 整床 + 提交**

---

### Task 3: 腕部求解（3×3 角雅可比 + FK 回代 + 阻尼 + 自验门）

**Files:** 同 Task 2 的几个文件；用例 ㉒㉓㉔

**Interfaces:**
- Produces:
  ```cpp
  // 解 J4/J5/J6 使 FK_R(ref[0..2], q) == targetR。成功 ⇒ true 且写 out[3..5]；
  // 失败（不收敛 / 雅可比退化）⇒ false，且 out 写回 ref（**不动**）。
  bool button2SolveWrist(const double ref[6], const double targetR[9], double maxStepDeg, double out[6]);
  ```

**算法（设计 §2）：**
```
q ← ref[3..5]
for iter in 1..MAXITER:
    J[6][6] ← Kinematics::jacobian({ref0,ref1,ref2,q0,q1,q2})
    Jw[i][j] = J[3+i][3+j]                       ← 角雅可比（行 3..5、列 3..5，已核 Kinematics.cpp:272）
    T ← composeTransform({ref0..ref2,q0..q2}) ; Rq = T[0..2][0..2]
    e ← rotVecDeg(targetR · Rqᵀ)                  ← 世界系误差（度）
    |e| < BTN2_WRIST_TOL_DEG ⇒ 收敛
    dq ← Jwᵀ (Jw·Jwᵀ + λ²I)⁻¹ e                  ← 阻尼最小二乘（λ 自适应，同 Kinematics::inverse 的形状）
    逐轴把 dq 夹到 ±maxStepDeg，q += dq
末尾再自验一次 |e|；不过 ⇒ false
```
**不赌球腕**：用 FK 回代，不用闭式欧拉分解 ⇒ 腕部几何将来改了不用重推。

- [ ] **Step 1: 三个用例（**完整代码**）**

```cpp
// 小工具：从 refJoints 取 FK 旋转（行主序 9 元）
static void fkR(const double j[6], double R[9]) {
    double T[4][4]; Kinematics::composeTransform(j, T);
    for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) R[r*3+c] = T[r][c];
}
// 两个旋转之间的角（度）：|rotvec(A·Bᵀ)|
static double angBetweenDeg(const double A[9], const double B[9]) {
    double Bt[9], D[9]; button2Mat3T(B, Bt); button2Mat3Mul(A, Bt, D);
    double rv[3]; button2RotVecDegForTest(D, rv);
    return std::sqrt(rv[0]*rv[0] + rv[1]*rv[1] + rv[2]*rv[2]);
}
```

```cpp
    {
        TEST(㉒ 目标 = 参考姿态 ⇒ 解 = 参考（恒等）);
        const double poses[3][6] = {{0,0,0,0,-90,0},{30,-60,45,20,-70,10},{-120,40,-30,90,-45,180}};
        for (int p = 0; p < 3; ++p) {
            double R[9]; fkR(poses[p], R);
            double out[6];
            CHECK(button2SolveWrist(poses[p], R, Config::ORIENT_MAX_STEP_DEG, out));
            for (int i = 0; i < 6; ++i) CHECK(std::fabs(out[i] - poses[p][i]) < 1e-6);
        }
    }
    {
        TEST(㉓ ★ 跨位姿 FK 回验：随机 200 组，解出来后 FK(解) 必须等于目标（<0.05°）);
        //   这是"解对了"的**直接证据**，不依赖任何符号常数（sx/sy/s6 都不进这条）。
        unsigned seed = 20260929u;
        auto rnd = [&seed]() { seed = seed * 1103515245u + 12345u;
                               return (double)((seed >> 16) & 0x7FFF) / 32767.0; };
        int ok = 0;
        for (int t = 0; t < 200; ++t) {
            double ref[6] = { rnd()*720-360, rnd()*720-360, rnd()*310-155,
                              rnd()*720-360, rnd()*720-360, rnd()*720-360 };
            double Rr[9]; fkR(ref, Rr);
            // 目标：在 Rr 上左乘一个 ≤15° 的随机小旋转（保证附近可达）
            const double ax[3] = { rnd()*2-1, rnd()*2-1, rnd()*2-1 };
            double n = std::sqrt(ax[0]*ax[0]+ax[1]*ax[1]+ax[2]*ax[2]);
            if (n < 1e-6) continue;
            const double sv[3] = { ax[0]/n*15*rnd(), ax[1]/n*15*rnd(), ax[2]/n*15*rnd() };
            double Md[9], Rt[9]; button2RotVecToMatDeg(sv, Md); button2Mat3Mul(Md, Rr, Rt);

            double out[6];
            if (!button2SolveWrist(ref, Rt, Config::ORIENT_MAX_STEP_DEG, out)) continue;
            double Ro[9]; fkR(out, Ro);
            if (angBetweenDeg(Rt, Ro) < Config::BTN2_WRIST_TOL_DEG) ++ok;
            // 且 J1/J2/J3 必须逐位不变（本任务只解腕）
            for (int i = 0; i < 3; ++i) CHECK(out[i] == ref[i]);
        }
        CHECK(ok >= 190);   // 容许 ~5% 因奇异/不可达被拒
    }
    {
        TEST(㉔ 不变量：若返回 true，则 FK(解) 必须【真的】等于目标（两个腕部奇异位姿）);
        // ★★ 这条【不能】写成"奇异位姿必须拒发"。理由两条，都是实测/算术：
        //   ① 写死"必须拒发"= 让用例替实现背书（在 ref 附近腕部未必解不出）；
        //   ② 更要命的是它**结构上无法变红** —— 见下面 Step 5 的裁决。
        //   ⇒ 断言的只能是【不变量】：**返回 true ⇒ 解真的到得了目标**。
        const double refs[2][6] = {{0,0,0,0,0,0}, {0,0,0,0,180,0}};  // J5=0 与 J5=±180 两处腕部奇异
        const double bigs[2][3] = {{0,60,0}, {0,0,60}};
        for (int c = 0; c < 2; ++c) {
            double Rr[9]; fkR(refs[c], Rr);
            double Md[9], Rt[9];
            button2RotVecToMatDeg(bigs[c], Md);
            button2Mat3Mul(Md, Rr, Rt);
            double out[6] = {9,9,9,9,9,9};
            const bool ok = button2SolveWrist(refs[c], Rt, Config::ORIENT_MAX_STEP_DEG, out);
            if (!ok) {
                for (int i = 0; i < 6; ++i) CHECK(out[i] == refs[c][i]);   // 失败 ⇒ 必须原样退回参照
            } else {
                double Ro[9]; fkR(out, Ro);
                CHECK(angBetweenDeg(Rt, Ro) < Config::BTN2_WRIST_TOL_DEG); // ★ 真的不变量（**能红**）
                for (int i = 0; i < 3; ++i) CHECK(out[i] == refs[c][i]);
            }
        }
    }
```

⚠ **不要**把 `else` 分支写成 `CHECK(std::fabs(out[i]-ref[i]) < 180)` —— 那是**恒真式**：
`maxStepDeg = ORIENT_MAX_STEP_DEG = 3.0`、迭代 ≤ `BTN2_WRIST_MAX_ITER = 24`
⇒ `|out−ref| ≤ 72° < 180°` **永远成立** ⇒ 那条断言抓不住任何东西。
（这是本轮复审实测出来的第 5 条空转负对照，作者是控制方本人。）

- [ ] **Step 2: 跑，确认失败**（未声明）
- [ ] **Step 3: 实现 `button2SolveWrist`**（照上面「算法」那一节逐行写；`λ` 初值取 `BTN2_WRIST_DAMP`，
  每轮 `|e|` 变小则 `λ*=0.5`、变大则 `λ*=2.0`，夹在 `[1e-6, 1.0]`）
- [ ] **Step 4: 跑，确认通过**
- [ ] **Step 5: 负对照（**必须实测红**）**

> ⚠⚠ **这一条已被复审实测推翻过两次，两版都作废 —— 读清楚再动手**：
> · 一版「λ≡0 ⇒ 红」：**空转** —— 不收敛会被自验门转成 `return false` ⇒ ㉔ 照旧绿。
> · 二版「(b) 把容差改成 `1e9`」：**不成立** —— ① ㉔ 里**根本没有** `angBetweenDeg`（它在 ㉓ 里）；
>   ② **把容差放宽只会更绿，永远不会红**（方向反了）。
> · 二版「(a) 拆自验门」：**要先证明它会红**（见下）。
> ⇒ 教训一句话：**负对照必须先在用例的输入端算一遍"被突变的那段会不会真的变"。**

**主对照（期望红，但【必须实测】）**：
把 `button2SolveWrist` 末尾的自验门删掉（无条件 `return true`）⇒ **㉔ 的
`CHECK(angBetweenDeg(Rt, Ro) < BTN2_WRIST_TOL_DEG)` 必须红**。
⚠ **它成立的前提是"那个位姿上确实解不出来"**。若实测**没红**，说明自验门在该位姿上没起作用
⇒ **如实写进报告，并换一个更极端的位姿重试**，**不要假装通过**。

**兜底对照（**保证红**，用来证明 ㉔ 这条断言有牙齿）**：
在 `button2SolveWrist` 返回前插入 `out[4] += 30.0;`（故意把解弄错）**但仍 `return true`**
⇒ ㉔ **必须红**。它**不依赖**任何"解不出来"的假设 ⇒ 一定红。
两条各自改回、重建、确认复绿。

⛔ **不要再写这两种**（它们都在这一轮被实测证伪）：
- 「删掉雅可比退化守卫 `if (std::fabs(det) < 1e-12) return false;` ⇒ ㉔ 红」——**不保证**：
  不收敛会被自验门转成 `false`，而 ㉔ 的失败分支断言的是 `out == ref` ⇒ 仍绿。
- 「把用例的容差放宽 ⇒ 红」——**方向反了**。
- [ ] **Step 6: 整床 + 提交**

---

### Task 4: 拼成对外纯函数 `button2OrientJointTarget`

**Files:** 同 Task 2；用例 ㉕

**Interfaces:**
- Produces:
  ```cpp
  void button2OrientJointTarget(const double refJoints[6], const double refStylus[3],
                                const double curStylus[3], double outJoints[6]);
  ```
- **契约（与旧函数逐条对齐，除了"一根轴喂一个关节"那条被设计取代）**：
  - `out[0..2] = ref[0..2]` **无条件**（第一句就写，任何分支都不许再碰这三个下标）
  - 笔杆任一分量 NaN/Inf ⇒ **六位全回参照**
  - 求解失败 ⇒ **六位全回参照**
  - 签名里**没有位置入参**（编译期性质，用函数指针类型钉住，用例 ③ 的做法）
  - `maxStepDeg` 传进 Task 3 的内部限幅；**逐帧累加器仍由调用方 `clampJointStep` 持有**（不重复）

- [ ] **Step 1: 实现（**完整代码**）**

```cpp
void button2OrientJointTarget(const double refJoints[6], const double refStylus[3],
                              const double curStylus[3], double outJoints[6]) {
    // ---- 第一句就写 J1/J2/J3，且【无条件】—— 与旧函数同一条纪律：
    //      下面任何分支都不许再碰这三个下标。
    outJoints[0] = refJoints[0];
    outJoints[1] = refJoints[1];
    outJoints[2] = refJoints[2];

    // ---- 笔杆 NaN/Inf 守卫（与旧函数的契约逐条对齐）----
    //   ⚠ 只守【笔杆侧】：`refJoints` 若非有限，本函数**原样传出去**（没有安全值可退，
    //     那一刻 0 是一个真实关节角，机械臂会真的转过去）。这一点与旧函数**逐字相同**。
    bool stylusBad = false;
    for (int i = 0; i < 3; ++i)
        if (!std::isfinite(refStylus[i]) || !std::isfinite(curStylus[i])) stylusBad = true;
    if (stylusBad) {
        for (int i = 0; i < 6; ++i) outJoints[i] = refJoints[i];
        return;
    }

    // ---- 参照姿态（FK 的旋转部分，行主序）----
    double refR[9];
    {
        double T[4][4];
        Kinematics::composeTransform(refJoints, T);
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c) refR[r*3 + c] = T[r][c];
    }

    // ---- 想要的末端朝向（Task 2）----
    double targetR[9];
    button2OrientTarget(refR, refStylus, curStylus, Config::BTN2_TILT_PHI_DEG, targetR);

    // ---- 解 J4/J5/J6（Task 3）；**失败 ⇒ 六位全回参照**（与 I1/I2/FK 门的"本帧不下发"同款形状）
    if (!button2SolveWrist(refJoints, targetR, Config::ORIENT_MAX_STEP_DEG, outJoints)) {
        for (int i = 0; i < 6; ++i) outJoints[i] = refJoints[i];
    }
}
```

- [ ] **Step 2: 五个用例（**完整代码**；都在 `static void` 里，`main()` 依次调用）**

```cpp
// 签名里【没有位置入参】是编译期性质，用函数指针类型钉住（与旧函数用例③同一招）
typedef void (*Btn2OrientSig)(const double[6], const double[3], const double[3], double[6]);

static void test_orient_joint_end_to_end_reaches_target() {
    TEST(㉕(a) ★ 端到端：任意参照位姿 + 任意小摆动 ⇒ FK(解) 必须真的到得了目标);
    //   这一条把 Task 2 与 Task 3 串起来验：**不依赖 sx/sy/s6**（它比的是"算出来的目标"与"解出来的朝向"）。
    unsigned seed = 20260929u;
    auto rnd = [&seed]() { seed = seed*1103515245u + 12345u; return (double)((seed>>16)&0x7FFF)/32767.0; };
    int ok = 0, ran = 0;
    for (int t = 0; t < 60; ++t) {
        double ref[6] = { rnd()*720-360, rnd()*720-360, rnd()*310-155,
                          rnd()*720-360, rnd()*720-360, rnd()*720-360 };
        // 笔杆从"单位姿态"转到"绕器件某轴 ≤15°"——用测试侧的 matToZyRpy 造输入（别再写第二份！）
        double Rs[9], Rd[9], Rsc[9], cur[3];
        TcpCalibration::rpyToMatrix(0,0,0, Rs);
        const double rv[3] = { rnd()*30-15, rnd()*30-15, rnd()*30-15 };
        button2RotVecToMatDeg(rv, Rd);
        button2Mat3Mul(Rs, Rd, Rsc);
        matToZyRpy(Rsc, cur);
        const double refStylus[3] = {0,0,0};

        double out[6];
        button2OrientJointTarget(ref, refStylus, cur, out);
        ++ran;
        // J1/J2/J3 必须逐位不变
        CHECK(out[0] == ref[0]);
        CHECK(out[1] == ref[1]);
        CHECK(out[2] == ref[2]);
        // 目标朝向（用同一套纯函数算一遍）与 FK(解) 必须一致
        double refR[9]; fkR(ref, refR);
        double tgtR[9];
        button2OrientTarget(refR, refStylus, cur, Config::BTN2_TILT_PHI_DEG, tgtR);
        double outR[9]; fkR(out, outR);
        if (angBetweenDeg(tgtR, outR) < Config::BTN2_WRIST_TOL_DEG) ++ok;
    }
    CHECK(ran == 60);
    CHECK(ok >= 55);      // 容许少数位姿真的解不出来（那正是自验门该拒的）
    PASS();
}

static void test_orient_joint_identity_and_nan() {
    TEST(㉕(b) 笔杆不动 ⇒ 六关节逐位不动；NaN/Inf ⇒ 六位全回参照);
    const double ref[6] = {10, -20, 30, 40, -50, 60};
    double out[6];
    {   // 不动
        const double s[3] = {0,0,0};
        button2OrientJointTarget(ref, s, s, out);
        for (int i = 0; i < 6; ++i) CHECK(out[i] == ref[i]);
    }
    {   // NaN 在 curStylus
        const double s0[3] = {0,0,0};
        const double sN[3] = {0, std::numeric_limits<double>::quiet_NaN(), 0};
        button2OrientJointTarget(ref, s0, sN, out);
        for (int i = 0; i < 6; ++i) CHECK(out[i] == ref[i]);
    }
    {   // Inf 在 refStylus
        const double sI[3] = {std::numeric_limits<double>::infinity(), 0, 0};
        const double s0[3] = {0,0,0};
        button2OrientJointTarget(ref, sI, s0, out);
        for (int i = 0; i < 6; ++i) CHECK(out[i] == ref[i]);
    }
    PASS();
}

static void test_orient_joint_signature_has_no_position_input() {
    TEST(㉕(c) 签名里没有位置入参（编译期性质）);
    Btn2OrientSig fp = &button2OrientJointTarget;
    CHECK(fp != nullptr);
    PASS();
}
```

⚠ `fkR` / `angBetweenDeg` / `matToZyRpy` 都是**测试文件里已有的**（`matToZyRpy` 在 `:87` 附近，
另两个是 Task 3 加的）—— **一个都不许重写**。

- [ ] **Step 3: 跑，确认通过**（`Results:` 应从 Task 3 结束时的数再涨 3）
- [ ] **Step 4: 负对照（**必须实测红**）**

- **(a) 把 J1/J2/J3 的保持去掉**（例如把 `outJoints[0] = refJoints[0];` 改成
  `outJoints[0] = refJoints[0] + 1.0;`）⇒ **㉕(a) 必须红**（它逐位断言 `out[0] == ref[0]`）。改回。
- **(b) 把 NaN 守卫删掉** ⇒ **㉕(b) 必须红**。改回。
  ⚠ 删守卫后 `rpyToMatrix(NaN,…)` 会产出非有限矩阵 ⇒ 后续要么是 NaN 关节角、要么被 Task 3 的自验门拒掉后
  **回退成 `out == ref`（那 ㉕(b) 就照旧绿）**。**若实测不红，如实报告**，并改用下面这条：
- **(c)【保证红】** 把 NaN 分支里的 `for (int i = 0; i < 6; ++i) outJoints[i] = refJoints[i];`
  改成 `outJoints[5] = refJoints[5] + 1.0;` ⇒ **㉕(b) 的 NaN 那一段必须红**（它逐位比六个）。
  ⚠ 它**不依赖**任何"下游会不会把 NaN 转成回退"的假设 ⇒ 一定红。

- [ ] **Step 5: 整床 + 提交**

---

### Task 5: 接线（`RelayCore.cpp` 的调用点分叉）

**Files:**
- Modify `Touch_Client/relay/RelayCore.cpp` —— **调用点在 `:1594`**（⚠ 写计划时写的"约 :1544"是**旧的**；
  `grep -n "button2JointTarget" Touch_Client/relay/RelayCore.cpp` 现场核过，真实落点是 **`:1594`**。
  例外见下面 Step 0 里那条"逐行核实"的要求 —— 别照抄这个数，**动手前再 grep 一次**。）
- Modify `Touch_Client/config/Config.h` —— **只改一行注释**（见 Step 0b）

⚠ **`RelayCore.cpp` 不被任何测试编译** ⇒ 这一步**只能靠复审 + 上机**。报告里必须写明这句。

> ⚠ **本任务不加 `Config` 常数** —— 它们在 **Task 2 的 Step 0** 就加了（否则 Task 2 编不过）。
> 本任务只做**接线** + 两处注释订正。

- [ ] **Step 0: 动手前先 `grep` 三个名字**（本仓有成文教训：**常数与行号一样脆**）

```
grep -n "button2JointTarget"            Touch_Client/relay/RelayCore.cpp          # 找调用点
grep -n "BTN2_ORIENT_TARGET_ENABLED"    Touch_Client/config/Config.h              # 确认开关在
grep -n "BTN2_WRIST_DAMP"               Touch_Client/config/Config.h              # 见 Step 0b
```
**以 grep 的输出为准**，不要用本文件里的任何行号。

- [ ] **Step 0b: 顺手订正一处**跨文件常数角色不符**（Task 3 复审的 Minor）**

`Config.h` 里 `BTN2_WRIST_DAMP` 的注释写的是"阻尼 λ 的下限"，而 `Button2Joint.cpp` 里
它是 λ 的**初值**（真正夹的下限是代码里的字面量 `1e-6`，上界 `1.0`）。
⇒ 把注释改成如实（例如"阻尼 λ 的**初值**（每轮按误差变大/变小先 ×2 / ×0.5，再夹到 [1e-6, 1.0]）"）。
**只改注释，不改数值。** ⚠ 这条是"注释说假话"类 —— 本仓最忌，所以哪怕只是一行也顺手改掉。

- [ ] **Step 1: 调用点分叉**

在 `RelayCore.cpp` 里那一行 `button2JointTarget(m_jointRef, sRef, m_btn2StylusFilt, j);`（grep 到的落点）
改成：

```cpp
        // ★★★ 2026-09-29：按钮2 的新实现（"姿态目标 + 解 J4/J5/J6"）与旧实现（"一根器件轴喂一个关节"）
        //   在这里分叉。**回滚 = 翻 Config::BTN2_ORIENT_TARGET_ENABLED 一个 bool。**
        //   设计见 `Docs/superpowers/specs/2026-09-29-button2-orientation-target-design.md`（读【修订】那一节）。
        if (Config::BTN2_ORIENT_TARGET_ENABLED) {
            button2OrientJointTarget(m_jointRef, sRef, m_btn2StylusFilt, j);
        } else {
            button2JointTarget(m_jointRef, sRef, m_btn2StylusFilt, j);   // 旧路，**逐字保留**
        }
```

**下游（I1 参照闸 / I2 关节限位闸 / FK 位置门 / `clampJointStep` / `ServoJ`）一行不改。**

⚠ **上面那句"NaN 到不了 ServoJ"的既有注释要核一遍**：它现在的理由是"`button2JointTarget` 自己的
守卫（`allFinite3`）会让返回值退回参照"。**新函数也必须有同款守卫**（Task 4 的契约里写了）
⇒ **现场核 `Button2Joint.cpp` 的新函数确有该守卫**；若没有，那条注释就变成了假话，
要么补守卫、要么改注释。**别放过这一条 —— 它是安全链路上的一句注释。**

- [ ] **Step 2: 编译 + 负对照**

1. 正常编译：**0 error**（既有的 `CALLBACK` 宏重定义警告不算）。
2. **负对照 A**：往 `RelayCore.cpp` 里注入一处语法错误 ⇒ **必须报 `error C…`**
   （证明这次构建**确实在编它**，而不是因为增量构建把它跳过了）。
3. **负对照 B（本任务特有）**：把开关临时改成 `false` ⇒ 编译通过；再核
   **旧的那一行仍然逐字在**（`git diff` 里 `else` 支那一行应当是**纯新增**，不是"移动"）。
   ⚠ 这一步**测不出行为**（本文件无自动化用例）⇒ 它只证明"两行都在"，**不是**功能验证。

- [ ] **Step 3: 逐条确认四个常数**真的被读**（Task 2 复审留下的要求）**

在报告里**逐条**给出 `grep` 结果，证明这四个常数**每个都有一个真实的消费点**：
`BTN2_ORIENT_TARGET_ENABLED`（本步的分叉）· `BTN2_TILT_PHI_DEG`（Task 4 传进 `button2OrientTarget`）·
`BTN2_WRIST_TOL_DEG` / `BTN2_WRIST_MAX_ITER` / `BTN2_WRIST_DAMP`（`button2SolveWrist` 内部）。
**任何一个没有消费点 ⇒ 报出来**（本仓栽过"常数写进去没人读"）。

- [ ] **Step 4: 整床 + 提交**

⚠ 提交信息里必须写明：**`RelayCore.cpp` 不被任何测试编译 ⇒ 接线这一层仍只有复审 + 上机。**
⚠ **仍开着的一个问题（等用户裁决，不是本任务的阻塞项）**：本任务会把
`Config::ORIENT_MAX_STEP_DEG`（=3.0）当作 `button2SolveWrist` 的**内部牛顿步长上限**传进去
⇒ **单次调用可改变的朝向被限在 24×3° = 72°**，而设计里 `ORIENT_MAX_OFFSET_DEG` 写的是 150°
⇒ 实际生效上限是 **72° 不是 150°**。**在提交信息与报告中如实点名**，等用户定"72 够不够"。

---

### Task 6: 文档收口 + 执行单

> ⚠⚠ **一件【明确不在本计划内】的事，别让它悄悄消失**（计划自审时抓到）：
> 设计 §3 有一条「**上游那条必须一起处理的**」—— `RelayCore` 侧算偏移是**欧拉分量相减**，
> 且**低通也做在欧拉域**（`ORIENT_STYLUS_LPF_TAU_S = 0.25`）⇒ 笔杆 Rz 跨 ±180 时会喂进一串
> "看起来平滑的假偏移"，**把本纯函数内刚获得的"跨接缝免疫"架空**。
> **本计划不做它**，理由：它改的是 **`RelayCore.cpp` 的接线（不被任何测试编译）**，
> 与"实现一个可测的纯函数"是两种风险；**混在一起做，出问题时说不清是哪一半。**
> ⇒ **后果如实记账：Task 1–5 做完，跨 ±180 那条隐患【原样还在】。**
> ⇒ 单开一批做，判据：跨接缝时不应出现大幅关节位移。

- [ ] **Step 1:** 更新设计文档的状态行（"已实施 / 待上机确认两个符号"；
  并在 §3 那条「上游必须一起处理」上标注 **"本批未做，见计划 Task 6 的注记"**）
- [ ] **Step 2:** 在执行单 `Docs/superpowers/specs/2026-09-29-on-machine-run-sheet.md` 加一节：

```
## 10. 按钮2 新实现 —— 上机只看两件事
① 前摆 ⇒ 末端往基座 +Y 摆？  不对 ⇒ 翻 BTN2_TILT_SIGN_X（一行）＋重建
② 左摆 ⇒ 末端往基座 −X 摆？  不对 ⇒ 翻 BTN2_TILT_SIGN_Y（一行）＋重建
③ 自转 ⇒ 只动 J6？（沿用旧结论，顺手复核）
④ 非正位（J1 转 45°）再走一遍 ①② —— **这正是本次要修的那条**
```
- [ ] **Step 3:** 提交
