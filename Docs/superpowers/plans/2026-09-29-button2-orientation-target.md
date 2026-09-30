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

  ⚠ **本段签名是【历史记录】**：Task 4 当时就是按这个签名实现的；其后提交 `223dd8b`（整支终审 I-1）把返回类型从 `void` 改成 `Btn2JointResult` ⇒ 上面两行与下面 Step 1 的代码只反映**当时**，现行签名见 `Touch_Client/relay/Button2Joint.h`。

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

- [ ] **Step 0（顺带，成本零）:** 订正 `RelayCore.cpp` 里一句**字面就是假的**注释

Task 5 复审的 Minor：接线处上方那条注释写着「⚠ **两条路入参次序不同**：`m_jointRef` 是 j1..j6，
`m_btn2StylusFilt` 是笔杆 Rx,Ry,Rz；…位置与旧调用点一一对应」。
⚠ 本文件里"**路**"一律指**路径**（新/旧）⇒ 这句话字面在说"**两条路径**的入参次序不同"，
**与事实相反**（两条路径的调用点位置逐位相同），而且**下一句自己就否了它**。
⇒ 改成说的是**两个入参**：`两条路入参次序不同` → **`两个入参的次序约定不同`**。
**只改这一句注释，别的都不动。**

- [ ] **Step 1:** 更新设计文档 `Docs/superpowers/specs/2026-09-29-button2-orientation-target-design.md`：
  1. 状态行改成 **"已实施（Task 1–5）· 待上机确认两个符号"**，并写上分支与最终提交区间；
  2. §3 那条「上游低通在欧拉域必须一起处理」上加标注 **"本批未做 ⇒ 跨 ±180 的隐患原样还在"**
     （理由：它改的是不被任何测试编译的接线，与本批"实现可测纯函数"是两种风险）；
  3. ★ **把 72° 这件事写进设计文档**（不能只活在计划与提交信息里）：
     `button2SolveWrist` 的内部牛顿步长上限被传成 `ORIENT_MAX_STEP_DEG`(3.0)、迭代 ≤ `BTN2_WRIST_MAX_ITER`(24)
     ⇒ **单次调用可改变的朝向 ≤ 72°**，而本文档 §3 写的是 `ORIENT_MAX_OFFSET_DEG = 150°`。
     **失败模式要说准**：超预算 ⇒ 末尾无条件自验不过 ⇒ 返回 `false` ⇒ **六位全回参照**，
     **不是**"夹到 72° 就停"。标注为 **"待用户裁决"**（够用 ⇒ 把 150 改成如实；不够 ⇒ 解耦）。
- [ ] **Step 2:** 在执行单 `Docs/superpowers/specs/2026-09-29-on-machine-run-sheet.md` 加一节：

```
## 10. 按钮2 新实现（姿态目标 + 解 J4/J5/J6）—— 上机判据
（⚠ 小节号是 **10** 不是 11 —— 实测那个执行单文件当时只到 `## 9`，写 11 会跳号。
 此处原写 11 是控制方凭印象填的，落笔前没核文件实际到第几节。）

【前置】客户端必须**重建**（`Config::BTN2_ORIENT_TARGET_ENABLED` 是编译期常量，机器上翻不到）。
        ⚠ 开关**默认 true** ⇒ 重建后走的就是**新路**。

【本趟只判三件事，每件一句话】
① 前摆 ⇒ 末端往基座 **+Y** 摆？      不对 ⇒ 翻 `Config::BTN2_TILT_SIGN_X`（一行）＋重建
② 左摆 ⇒ 末端往基座 **−X** 摆？      不对 ⇒ 翻 `Config::BTN2_TILT_SIGN_Y`（一行）＋重建
③ 自转 ⇒ **只动 J6**？               不对 ⇒ 复核 `BTN2_ROLL_SIGN`（旧结论是 −1）

★ 每条都要在**两个位姿**上各做一遍：**正位**，以及**非正位（J1 转约 45°）**。
  **非正位那一遍才是本次要修的那条** —— 旧实现在那里方向就是错的。

④（顺带，判"每腕关节每次 72°"这条行为）**把笔倾到很大** ⇒
  预期是机械臂**保持/回到按下按钮2 时的位姿**（求解返回 false ⇒ 六位回参照 ⇒ 照常 `ServoJ` 发），
  **不是**"夹到某个角度就停"，更**不是**乱走。
  ⚠ **不是"本帧不下发"** —— 真正"不下发"的只有 I1/I2/FK 那三道门；本仓对同一措辞早有订正
  （`RelayCore.cpp` 里"死区全归 0 ⇒ 原地保持（**不是**这一帧不下发）"）。

⑤（回归）非正位下**别**出现关节超速报警 —— 那是 2026-09-22 那次 `ORIENT_SEAM_FIX_ENABLED`
  的教训同类（见 `Config.h` 那条警示）。
```

⚠ **这四步全都要写"看到什么算过、看到什么算不过"** —— 本仓明文教训：没有判据的待办会退化成"感觉上还没做"。
⚠ 同时把 **①的失败含义**写清：方向不对 ⇒ **只翻那一个常数**，**不要**顺手改别的。
- [ ] **Step 3:** 提交

---

### Task 7: 单次旋转预算 72° → 180°（把生效上限还给输入端那条 150°）

> **2026-09-29 用户裁决**：72° 不够用 ⇒ 走 **(甲) 只调大迭代上限**，**不做解耦**（不解耦成两个步长常数）。
> 为什么是"调迭代次数"而不是"调每轮步长"：
> · 每轮 **3° 这条没动** ⇒ "小步 ⇒ 偏向近解、不跳解支"这条性质**逐字不变**；
> · 解耦要**新开一个步长常数**；而**直接调大 `ORIENT_MAX_STEP_DEG` 是禁止的** ——
>   它同时是姿态路径的每帧步长与 `clampJointStep` 的每帧步长（两条**已被现场验证过**的路），
>   设计 §3 那条 ⚠ 明写了"别直接调大它"；
> · 迭代上限是**求解器内部读 `Config`**、四个测试调用点传的是 `maxStepDeg`
>   ⇒ 改它**测试自动跟上**，没有"手工同步 N 个调用点"的漂移面。

**Files:**
- Modify: `Touch_Client/config/Config.h`（`BTN2_WRIST_MAX_ITER` 的**值** + 注释；顺带订正 `:722` 一处）
- Modify: `Touch_Client/tests/test_button2_joint.cpp`（㉔ 的 c2 换构造 · ㉕(d) 翻判据 · 新增 ㉗）
- Modify: `Docs/superpowers/specs/2026-09-29-button2-orientation-target-design.md`（§3 那条 72° 的注）
- Modify: `Docs/superpowers/specs/2026-09-29-on-machine-run-sheet.md`（§10.3）
- **不动**：`Touch_Client/relay/Button2Joint.{h,cpp}` —— 本任务**只改一个常数的值**，求解器一行不碰。

**Interfaces:**
- Consumes（签名**都不变**）：
  - `bool button2SolveWrist(const double ref[6], const double targetR[9], double maxStepDeg, double out[6])`
  - `Btn2JointResult button2OrientJointTarget(const double ref[6], const double refStylus[3], const double curStylus[3], double out[6])`（I-1 之后的新签名）
  - `void button2OrientTarget(const double refR[9], const double refStylus[3], const double curStylus[3], double phiDeg, double outR[9])`
- Produces：**无新接口**。唯一的行为变化是 `Config::BTN2_WRIST_MAX_ITER` 的值 `24 → 60`。

**★ 先写下断言要建在上面的那条硬上界（`Button2Joint.cpp:397-404`）**：
每轮迭代**先**把 `dq` 逐轴夹到 `±maxStepDeg`、**再** `q[i] += dq[i]`
⇒ `|q[i] − ref[i]| ≤ maxStepDeg × Config::BTN2_WRIST_MAX_ITER` —— **与迭代质量无关**
（就算每轮都走满也是这个数，既不会更多也不会更少）。
现值 `3.0 × 24 = 72`；改后 `3.0 × 60 = 180`。

**为什么取 60（而不是刚好 50）**：入口那条限幅（`ORIENT_MAX_OFFSET_DEG = 150.0`，`Config.h:844`）
只夹**倾斜**分量；**自转**分量（`rv[2]`）**不过那条限幅**，而 `rotvec` 自身的最短弧上限是 **180°**
⇒ 入口能造出的目标离参照最多 **180°** ⇒ 预算必须 ≥ 180，否则"输入端允许、求解器却到不了"这种
自相矛盾的输入仍然存在（那正是今天这条 72° 的病）。50×3 = 150 **不够**：它只盖住倾斜、盖不住自转。

⚠ **如实记账，别把本任务读成"从此不再有求解失败"**：60×3 = 180 **恰等于**那条上限 ⇒ **零余量**。
腕三轴**不正交**（实测 `|z4·z6| = |cos J5|`）⇒ 非正交位姿下同一个末端转角要的**关节量可以大于转角本身**
⇒ 接近腕部奇异（`J5 → 0` 或 `±180`）时**仍会**求解失败。**那是对的**（那时臂本来也不该硬转）。

- [ ] **Step 1: 先把新判据写死（此时常数还是 24 ⇒ 预期红）**

**1a.** `Touch_Client/tests/test_button2_joint.cpp` —— 把 ㉕(d)（I-1 那条 `…_solve_failure_is_visible_and_falls_back`）
**整条换掉**。⚠ 不要盲改字节：I-1 刚落，**先读当前函数体**，只改"判据与输入构造"这两件事，
保留 I-1 那段注释里仍然成立的部分（`refStylus`/`curStylus` 全有限 ⇒ 不走 NaN 守卫）。
目标形态：

```cpp
// ㉕(d) ★ 2026-09-29 Task 7：**输入端上限处的 150° 倾斜必须解得出来**（预算 ≥ 输入上限）。
//    【为什么从"必须失败"翻成"必须成功"】本条原为 I-1 的用例，用 {150,0,0} 造"求解失败"，
//      靠的正是 24×3 = 72° 的步长预算 < 150°。Task 7 把预算抬到 60×3 = 180° 之后，
//      这条输入**必然解得出来** ⇒ 原判据变成**假判据**（若照抄不改，它会是"实现背书的对偶"：
//      一条永远为真、且无法变红的断言）。
//    【它与 ㉗ 的分工】本用例量**端到端行为**（入口 → 流水线 → 求解器）；常数之间那条
//      "预算 ≥ 输入上限"的**关系**由 ㉗ 单独钉（那里才是能对常数变化报警的地方）。
//    【负对照（实测，写进报告）】把 `BTN2_WRIST_MAX_ITER` 改回 24 ⇒ 本条**必须红**
//      （预期红在 `FAIL: res == Btn2JointResult::Ok`）。★ 这正是本任务的主对照。
//    ⚠ 150 用**字面量**、不用 `Config::ORIENT_MAX_OFFSET_DEG`：那个常数若被调小，目标会跟着缩、
//      反而落回预算内 ⇒ 用例**静默**失去意义（与它原来那条 ⚠ 同一条理由）。
//    ⚠ `refStylus`/`curStylus` 全**有限** ⇒ 本用例走的**不是** NaN 守卫那条路（那条由 ㉕(b) 钉）。
static void test_orient_joint_reaches_max_input_offset() {
    TEST(㉕(d) ★ 输入端上限（150° 倾斜）必须解得出来（预算 ≥ 输入上限）);
    const double s0[3]  = {0, 0, 0};
    const double sFar[3] = {150.0, 0, 0};   // 见上：|rv| = 150 = 限幅的上界 ⇒ 原样保留、不缩比
    // 两个参照：非正位（腕部一般位姿）+ 全零（J5 = 0 ⇒ **腕部奇异**位姿，最容易被拒的地方）
    const double refs[2][6] = { {10, -20, 30, 40, -50, 60}, {0,0,0,0,0,0} };
    for (int c = 0; c < 2; ++c) {
        double out[6] = {9,9,9,9,9,9};                 // 哨兵：必须被真正改写
        const Btn2JointResult res = button2OrientJointTarget(refs[c], s0, sFar, out);
        CHECK(res == Btn2JointResult::Ok);
        // ★ 真有牙齿的那一条：解得出来还不够，必须**真的到得了**入口算出来的那个目标。
        //   目标用**入口同一套常数**重算（与 ㉕(a) 同一招）⇒ 不依赖任何符号常数的取值。
        double Rr[9]; fkR(refs[c], Rr);
        double targetR[9];
        button2OrientTarget(Rr, s0, sFar, Config::BTN2_TILT_PHI_DEG, targetR);
        double Ro[9]; fkR(out, Ro);
        CHECK(angBetweenDeg(targetR, Ro) < Config::BTN2_WRIST_TOL_DEG);
    }
    PASS();
}
```

同时在 `main()` 里把调用点从 `test_orient_joint_solve_failure_is_visible_and_falls_back();`
改成 `test_orient_joint_reaches_max_input_offset();`（**旧函数名不许留着**：一条不再被调用的用例
正是"看着还在、其实从不执行"的那个形状 —— 本仓为它栽过不止一次）。

**1b.** 新增 **㉗**（放在 ㉕ 之后、㉖ 之前或之后皆可，只要在 `fkR` 定义之后）：

```cpp
// ㉗ ★ 常数关系：**求解预算必须盖住输入端能造出的最大目标**。
//    【为什么值一条独立断言】这条关系**跨两个文件、两种语义**（入口的目标限幅 vs 求解器的迭代预算）。
//      两边各自的用例都绿、而它们之间的**关系**坏了的时候，症状是"某些输入永远解不出来 ⇒
//      臂停在按下位姿"，**一次上机才会发现**。今天这条 72° 就是关系坏了（72 < 150）而全套绿灯。
//    【它能红（实测）】把 `BTN2_WRIST_MAX_ITER` 改回 24 ⇒ `72 >= 180` 假 ⇒ 红。
//    【右端为什么是 180 而不是 `ORIENT_MAX_OFFSET_DEG`】那条限幅只夹**倾斜**分量，
//      **自转**分量（`rv[2]`）不过它；而 `rotVecDeg` 的最短弧上限恒为 **180°**
//      ⇒ 入口能造出的最大目标量是 180°、**不是** 150°。取 150 会给出一个**偏高**的许可。
//    ⚠ 这条是**必要**条件，**不是**充分条件 —— 非正交位姿下关节需求会大于转角
//      （`|z4·z6| = |cos J5|`）⇒ 接近腕部奇异时仍会求解失败。别把它读成"到此为止就万事大吉"。
static void test_wrist_budget_covers_input_cap() {
    const double budgetDeg = Config::ORIENT_MAX_STEP_DEG * (double)Config::BTN2_WRIST_MAX_ITER;
    TEST(㉗ ★ 常数关系：求解预算 ≥ 入口能造出的最大目标（180°）);
    CHECK(budgetDeg >= 180.0);
    PASS();
}
```

并在 `main()` 里 `test_...();` 一行（照文件里其它用例的写法）。

- [ ] **Step 2: 跑，确认红**

Run: `Touch_Client\tests\build_button2_joint_test.bat` 然后 `test_button2_joint.exe`
（或 `cd Touch_Client/tests && cmd //c ".\run_tests.bat"`）
Expected: **㉕(d) 与 ㉗ 都红**，其余全绿；行数形如 `35 passed / 2 failed`（具体数照实记）。
**把这一段原文抄进报告** —— 没有这一段就没有"判据先写死"的证据。

- [ ] **Step 3: 改常数（唯一一处行为改动）**

`Touch_Client/config/Config.h:955`：

```cpp
    const int    BTN2_WRIST_MAX_ITER = 60;    // 每腕关节单次预算 = 本值 × ORIENT_MAX_STEP_DEG
                                              //   = 60 × 3.0 = **180°**（2026-09-29 用户裁决：由 24 调上来）
                                              // ★ 为什么是 180：入口的 `ORIENT_MAX_OFFSET_DEG`(150°) 只夹
                                              //   **倾斜**分量，**自转**分量不过它，而 `rotvec` 的最短弧上限
                                              //   恒为 180° ⇒ 入口能造出的最大目标量是 180°，**不是** 150°。
                                              //   预算 < 180 ⇒ "输入端允许、求解器却到不了"这种自相矛盾的
                                              //   输入仍然存在（24×3 = 72 就是那个病）。
                                              // ⚠ **零余量**：非正交位姿（`|z4·z6| = |cos J5|`）下关节需求可以
                                              //   大于转角 ⇒ 接近腕部奇异（J5→0 或 ±180）时**仍会**求解失败，
                                              //   那是**对的**（那时臂不该硬转）。别读成"从此不会失败"。
                                              // ⚠ **别改 `ORIENT_MAX_STEP_DEG` 来达到同样效果**：它同时是
                                              //   姿态路径与 `clampJointStep` 的**每帧**步长（两条已被现场
                                              //   验证过的路），而且"每轮步子大 ⇒ 更易跳解支"。
                                              //   改迭代上限 ⇒ 每轮仍是 3°、"小步偏向近解"这条性质逐字不变。
```

**顺带（零成本，同一行区域内）：** `Config.h:722` 那句注释 `… —— 见上：两个消费方`
现在**有三个**消费方（姿态路径的每帧步长 · `clampJointStep` 的每帧步长 · `button2SolveWrist` 的**每迭代**步长）。
⚠ **只改"两个"这个数目字**，上面那段 ⚠★ 说明（它逐条列了消费方）**已经是全的、不要重写** ——
动手前先读那一段确认它列了几条，按它实际列的数目写。

- [ ] **Step 4: 跑，确认 Step 2 那两条转绿，并把 ㉔ 的红如实接下来**

Run: 同 Step 2 的命令
Expected: ㉕(d) 与 ㉗ **转绿**；**㉔ 预期转红**（`CHECK(failedCount >= 1)`）——
它的 `c2` 原来是靠 72° 预算被拒的，预算一抬就解得出来 ⇒ 那个"抗空转守卫"**按它自己写明的职责报警**了
（`test_button2_joint.cpp:1253-1265` 早就预言过这一条）。**这不是回归，是守卫在干活**。
**把原文抄进报告。** 若 ㉔ **没有**红 ⇒ 停下、记现象（说明 c2 的失败另有机制，与注释里写的不是一回事）。

- [ ] **Step 5: 修 ㉔ 的 c2 —— 换成一个**与常数无关**的失败构造**

现状（`test_button2_joint.cpp:1266-1277`）：三组输入共用 `Config::ORIENT_MAX_STEP_DEG` 当 `maxStepDeg`，
靠 `bigs[2] = {0,179,0}` 超出 72° 预算落进 fail 分支。

**为什么必须换构造（不是把 179 再调大）**：`button2RotVecToMatDeg` 吃的是旋转向量，而
`button2SolveWrist` 的误差走 `rotVecDeg`（**最短弧 ≤ 180°**）⇒ **`bigs` 取任何值都造不出 >180° 的需求**
⇒ 在 180° 的预算下，**没有哪个 `bigs` 能靠默认常数落进 fail 分支**。唯一干净的构造是
**由调用方把 `maxStepDeg` 调小**（`maxStepDeg` 本来就是这个函数的入参，"预算"二字指的就是它）。

改成（替换 `refs`/`bigs` 那两行与循环里那次调用）：

```cpp
    const double refs[3][6] = {{0,0,0,0,0,0}, {0,0,0,0,180,0}, {0,0,0,0,0,0}};
    const double bigs[3][3] = {{0,60,0}, {0,0,60}, {0,179,0}};
    // ★ 2026-09-29 Task 7：**逐案**的步长（原来是三案共用 `Config::ORIENT_MAX_STEP_DEG`）。
    //   c2 用 1.0° ⇒ 预算 60 × 1.0 = **60°** < 它的需求 179° ⇒ **按构造**必然落进 fail 分支，
    //   与 `BTN2_WRIST_MAX_ITER` 的取值**无关**（硬上界见 `Button2Joint.cpp:397-404`）。
    //   ⚠ 为什么不能靠"把 bigs 调大到超出预算"：误差走 `rotVecDeg` ⇒ 最短弧恒 ≤ 180°，
    //     而预算已是 180° ⇒ 需求**永远**超不过它。见本任务 Step 5 的说明。
    //   ⚠ c0/c1 仍用**默认**步长 ⇒ 它们量的仍是"真实生产路径"下的可解性。
    const double steps[3] = {Config::ORIENT_MAX_STEP_DEG, Config::ORIENT_MAX_STEP_DEG, 1.0};
```
```cpp
        const bool ok = button2SolveWrist(refs[c], Rt, steps[c], out);
```

并把上面那段注释里已经**不成立**的两处改掉（**别整段重写**，逐句核）：
- `c2 —— 奇异位姿 + 极端目标（179°）⇒ 步长预算内解不出来` ⇒ 补上"**该案的 `maxStepDeg` 取 1.0°**（预算 60°）"；
- `实测：把 maxStepDeg 换成 30° 后 c2 解得出来` ⇒ 这个探针**要重跑一遍**（换成 30.0 后 30×60 = 1800 ⇒ 仍应解得出来），**把你实测到的结果写进去**，不要照抄这句。

- [ ] **Step 6: 跑全绿 + 整床**

Run: `Touch_Client\tests\build_button2_joint_test.bat` 然后 `test_button2_joint.exe`
Expected: 全绿（预期 `37 passed / 0 failed`）。
Run: `cd Touch_Client/tests && cmd //c ".\run_tests.bat"`
Expected: `exit 0` 且 `Suites accounted: 26 of 26`。

- [ ] **Step 7: 探针 —— 换完预算后，入口层还能不能造出 `SolveFailed`？**

**为什么问**：`SolveFailed` 是 I-1 那条"求解失败要出声"的**唯一**数据来源。预算抬到 180° 之后，
入口层"因预算而失败"这条路**按上面的推理已经堵死**（需求 ≤ 180 ≤ 预算）⇒ 只剩**几何退化**一条 —— 具体是 `button2SolveWrist` 里**仅有的两个** `return false`
（`button2Mat3Inv` 求逆失败、末尾自检不过；I-1 的实现者在报告 §7 里核过这两处的存在）。
**这条要实测，不能只写"推理上只剩几何"。**

**判据先写死**（跑之前不许改）：写一个临时探针（**不进正式用例**，跑完删；输出抄进报告），
扫 `ref` 关节角网格（例如 J4/J5/J6 各取 `-180..180` 步长 30°，J1/J2/J3 取 3~5 个值）×
输入取 `{150,0,0}` / `{0,150,0}` / `{0,0,180}` / `{−150,0,0}` 四种，统计
`button2OrientJointTarget(...) == Btn2JointResult::SolveFailed` 的样本数：

- **若找到 ≥ 1 个**（记下最小的那个 `ref` 与输入，**冻结成一条新用例**，编号接着 ㉘）⇒
  `SolveFailed` 在入口层**仍可离线构造**；报告里写出机制（是几何退化还是别的）。
- **若一个都没有** ⇒ 报告里**如实写**："预算 ≥ 输入上限之后，入口层的 `SolveFailed` 只能由
  真实几何退化触发，离线扫 N 个样本一个都没扫到"；**并把这条结论加进设计文档 §3**
  （见 Step 8）。**不要**为了凑一条用例去改生产代码或调常数。

- [ ] **Step 8: 文档**

**8a.** 设计文档 `Docs/superpowers/specs/2026-09-29-button2-orientation-target-design.md`
第 `298-333` 行那一整节（`### ★★ 单次调用的**能力上界 = 72°**，不是上面那个 150°（**待用户裁决**）`）：

- **标题改掉**（"能力上界 = 72°"与"待用户裁决"都已作废）；
- 正文改成**实施后**的事实：预算 = `BTN2_WRIST_MAX_ITER`(60) × `ORIENT_MAX_STEP_DEG`(3.0) = **180°**，
  ≥ 入口能造出的最大目标（180°，理由见 Task 7 头那段）⇒ **实际先撞上的是输入端那条 150°（倾斜）
  / 180°（最短弧）的限幅，不再是求解器的预算**；
- **保留并更新那三段 ⚠**（"不是夹到上限就停" / "不是乱走" / "**不是**本帧不下发"）——
  它们讲的是**失败时**的行为，现在依然成立，只是失败**变少了**；
- **新增一条 ⚠ 零余量**：60×3 = 180 恰等于上限 ⇒ 非正交位姿下关节需求可大于转角 ⇒
  接近腕部奇异时仍会失败；**并把 Step 7 的实测结论原样抄进来**（找到了就写找到了 + 用例号，
  没找到就写没找到 + 样本数）；
- 把"**待用户裁决**"那段（`:328-333`）改成"**已裁决（2026-09-29）：走 (甲)，只调迭代上限；
  不做解耦**"，并保留其中**仍然有效**的那条 ⚠（"别直接把 `ORIENT_MAX_STEP_DEG` 调大"）。

**8b.** 执行单 `Docs/superpowers/specs/2026-09-29-on-machine-run-sheet.md` 的 **§10.3**
（现标题 `### 10.3 顺带：判"能力预算 = 72°"这条行为`，`:413-434`）：**整节重写**成判**新**行为，
判据必须能区分"旧预算还在"与"新预算生效"：

```
### 10.3 顺带：判"单次旋转预算不再是 72°"

**把笔倾到 80~100°**（**超过旧预算 72°**）⇒

- **✅ 过**：机械臂**跟到**一个离"按下按钮2 那一刻的姿态"约那么大的朝向上，方向与手一致。
- **❌ 不过**：机械臂**保持 / 回到按下按钮2 时的位姿** ⇒ 预算没生效
  （先核 `Config.h` 的 `BTN2_WRIST_MAX_ITER` 是不是还写着 24）。
- **❌ 不过**：机械臂**夹在 ~72° 处停住** —— 本实现**没有**"夹到上限就停下"这条出路，
  看到这个现象说明代码与设计文档不是同一版。

**再往大转**：松**开**按钮2、**重按**，可以接着往同方向继续转
（`onButton2Press` 会把关节参照与笔杆参照**一起重取**、并重置低通 ⇒ 参照、预算、滤波三者**同时**重置）。
这一条要**两遍**都做：**正位**与**非正位（J1 转约 45°）**。

**转到超过 150° 的目标** ⇒ **✅ 过**：幅度被**缩到 150°**、**方向不变**（限幅是"缩比"不是"回退"）——
这与"求解失败 ⇒ 回参照"是**两种不同的观感**，别混。

⚠ 说准了：**不是**"这一帧不下发" —— 这条路上 `ServoJ(j)` **照样每帧发**。
但 ⚠ **`j` 不等于参照本身**：纯函数的输出还要过一道 **`clampJointStep`**（`relay/Button2Joint.cpp`），
它逐关节把"本帧要走的量"夹到 `Config::ORIENT_MAX_STEP_DEG`（**3.0°/帧**）再累加到
"上一帧**已下发**"上（`RelayCore.cpp` 的 `clampJointStep(m_btn2JointCmd, j, …)`）。
⇒ **想看那一帧的 `j`，只能看 `cmd`**（MATLAB 的 `C|` 行 / `app.lastCommandSent` 存的都是 `cmd`）。
```

并把 §10.3 末尾那句"详见设计文档 §3 那条 72° 的注（含"72 够不够"这个**待用户裁决**的问题）"
改成指向**新的那节标题**（Step 8a 改完后它叫什么就写什么）。
**顺带**：§10.3 里如果还留着"（顺带，判"每腕关节每次 72°"这条行为）"那种口径，一并订正。

- [ ] **Step 9: 提交**

```bash
git add -A
git commit -m "fix(btn2): 单次旋转预算 72°→180° —— 迭代上限 24→60，生效上限回到输入端那条 150°" \
           -m "用户裁决走 (甲)：只调 BTN2_WRIST_MAX_ITER，每轮仍 3°（不跳解支这条性质不变）。预算 60×3=180 覆盖入口能造出的最大目标（rotvec 最短弧上限 180，不是 150 —— 倾斜那条限幅不夹自转分量）。㉕(d) 翻成『上限处必须解得出来』并新增 ㉗ 钉住常数关系；㉔ 的 c2 改用调用方传入的小 maxStepDeg（与常数无关的失败构造）。文档：设计 §3 那节 + 执行单 §10.3 重写。"
```

**报告要包含**（写进 `.superpowers/sdd/task-7-report.md`，追加式）：
1. Step 2 的**红**原文（哪些用例、什么行）；
2. Step 4 的**㉔ 红**原文 —— 若没红，如实记；
3. **主对照实测**：把 `BTN2_WRIST_MAX_ITER` 临时改回 24 ⇒ 哪几条红（预期 ㉕(d) + ㉗），
   **然后改回 60 并再跑一遍全绿**；
4. Step 5 里那个 `maxStepDeg = 30°` 探针的**实测**结果；
5. Step 7 探针的样本数与结论（找到 / 没找到，两种都照实写）；
6. 两次整床（`26 of 26`）的原文；
7. **没做到 / 做不到的事是什么** —— 照实写，别圆。
