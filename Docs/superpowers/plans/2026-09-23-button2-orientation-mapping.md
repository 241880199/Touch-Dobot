# 按钮2 姿态映射重做 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让按钮2 的姿态控制**严格按用户的规格**工作：笔杆平移时机械臂不动；笔尖左（右）摆 ⇒ 尖端向 **−X（+X）** 倾；笔尖前（后）摆 ⇒ 尖端向 **+Y（−Y）** 倾；笔杆自转 ⇒ **J6** 转。

**Architecture:** 现状把「笔杆 Euler 角之差」**当成旋转向量**用（`robot_dR = M · (drx,dry,drz)`），再把三个角**逐分量加到**按下时的参照姿态上。这只在小角度下近似成立，而实际摆幅 ±50~80°、且本机末端姿态常年贴着 `rx≈±180`（RPY 表示的接缝）⇒ 映射在数学上不成立、并被钳位夹死（2026-09-23 实测：`|rx|>170` 占 85.8%）。改为**真正的旋转合成**：`ΔR = R_cur·R_refᵀ`（笔杆相对按下点的旋转）→ 用**与平移路径同一张**坐标变换做相似变换 `M·ΔR·Mᵀ` → `R_target = ΔR_robot·R_ref_robot` → 最后才提取一次 RPY。规格里的「笔杆自转 ⇒ J6」由该合成的 **tool-roll 分量**自然满足（工具轴为 J6 轴时）。

**Tech Stack:** C++17（MSVC `/std:c++17`）、既有 `TcpCalibration::rpyToMatrix`（`R = Rz·Ry·Rx`，入参为**度**）、自研 `Vec3`；测试走 `Touch_Client/tests/` 的独立 exe + `run_tests.bat`（2026-09-23 起套件数**运行时数出并断言**）。

## Global Constraints

- **不改** `Config::FORCE_CONSTRAINT_FORCES_ENABLED`、不改任何力映射/死区/增益。
- **不动** `SAFE_RX/RY/RZ` 的取值与 `clampOrientToBounds` 的**绝对值 ±180** 语义（它们"没有出处"这件事是**另一个**未结项；本计划只把**相对参照的限幅**从"逐分量 ±150°"改成"**旋转角** ≤ `ORIENT_MAX_OFFSET_DEG`"）。
- **平移路径的行为不得改变**（`convertTouchToRobot` 的映射结果逐位不变；它日常在用）。
- `.bat` 纯 ASCII；C++ 注释用中文（与邻码一致）。
- **回滚必须是一行**：新映射放在开关后（`Config::BTN2_ROTATION_COMPOSE_ENABLED`），`false` 时走旧路径（与 09-22 那次实验同一惯例）。
- 每次提交后 `Touch_Client\tests\run_tests.bat` 必须 **exit 0** 且 `Suites accounted: N of N`。

---

### Task 1: 把坐标变换抽成**唯一一份**定义

**Files:**
- Modify: `Touch_Client/relay/CoordinateTransform.h`（新增导出函数）
- Modify: `Touch_Client/relay/RelayCore.cpp:1121-1133`（改为调用该函数）
- Test: `Touch_Client/tests/test_frame_layout.cpp`（已存在；追加两条用例）
- Test harness: 无需改 `run_tests.bat`（该套件已在跑）

**Interfaces:**
- Produces: `void touchToRobotMatrix(double M[9]);` —— 行主序 3×3，`robot = M · touch`；硬编码兜底（`Calibration::enabled == false`）时返回 `[1,0,0, 0,0,-1, 0,1,0]`，与 `convertTouchToRobot` 的兜底**逐位一致**。

- [ ] **Step 1: 写失败测试**（`test_frame_layout.cpp` 末尾追加）

```cpp
// 2026-09-23: 平移路径与姿态路径必须用【同一张】坐标变换 ——
//   09-21 起姿态那条是自己又写了一遍（且源码里明写"从未被验证过"），漂开的风险是结构性的。
static void test_touch_to_robot_matrix_is_the_position_mapping() {
    double M[9] = {0};
    touchToRobotMatrix(M);
    // 与 convertTouchToRobot 的兜底逐位比：对三个基向量各跑一次
    const double basis[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
    for (int c = 0; c < 3; c++) {
        Vec3 r = convertTouchToRobot(basis[c]);
        TEST(M[0*3+c] == r.x && M[1*3+c] == r.y && M[2*3+c] == r.z);
    }
}
static void test_touch_to_robot_matrix_is_orthonormal() {
    double M[9] = {0};
    touchToRobotMatrix(M);
    for (int i = 0; i < 3; i++) {
        double n = M[i*3]*M[i*3] + M[i*3+1]*M[i*3+1] + M[i*3+2]*M[i*3+2];
        CHECK(fabs(n - 1.0) < 1e-12);                       // 每行单位长
        for (int j = i+1; j < 3; j++) {
            double d = M[i*3]*M[j*3] + M[i*3+1]*M[j*3+1] + M[i*3+2]*M[j*3+2];
            CHECK(fabs(d) < 1e-12);                          // 行间正交
        }
    }
    double det = M[0]*(M[4]*M[8]-M[5]*M[7]) - M[1]*(M[3]*M[8]-M[5]*M[6]) + M[2]*(M[3]*M[7]-M[4]*M[6]);
    CHECK(fabs(det - 1.0) < 1e-12);                          // 右手系（det=+1，不是镜像）
}
```

- [ ] **Step 2: 跑测试确认它失败**

Run: `cmd /c "cd /d D:\Projects\Touch\Touch_Client\tests && .\build_frame_layout_test.bat && .\test_frame_layout.exe"`
Expected: 编译失败（`touchToRobotMatrix` 未声明）—— 这就是"红"。

- [ ] **Step 3: 实现**

`CoordinateTransform.h`（放在 `convertTouchToRobot` 之前）：

```cpp
// ★★ 2026-09-23: 器件系 → 基座系 的【唯一一份】3×3（行主序），robot = M · touch。
// 【为什么要抽出来】平移路径(convertTouchToRobot)与姿态路径(RelayCore 的 robot_dR)各自写了一遍
//   同一张表，而姿态那一份的注释写着"从未被验证过" ⇒ 两边【可能】已经漂开，且漂开时没有任何东西会报。
//   抽成一处之后，"它们是不是同一张表"变成一条【可断言的】性质（见 test_frame_layout 的两条用例）。
// ⚠ Calibration::enabled 为真时，平移路径走的是标定出来的 R/t（含平移项）—— 那不是一张纯 3×3，
//   本函数【只描述兜底那一份】；调用者要按 Calibration::enabled 自己分支（与现在一致）。
inline void touchToRobotMatrix(double M[9]) {
    M[0] = 1.0; M[1] = 0.0; M[2] =  0.0;
    M[3] = 0.0; M[4] = 0.0; M[5] = -1.0;
    M[6] = 0.0; M[7] = 1.0; M[8] =  0.0;
}
```

`RelayCore.cpp` 里把硬编码的那 9 个赋值换成：

```cpp
            } else {
                // 与平移路径同一张表（唯一一份定义）—— 见 CoordinateTransform::touchToRobotMatrix
                touchToRobotMatrix(&R00);   // R00,R01,R02,R10,... 在内存里是连续的 9 个 double
            }
```
⚠ 该 `else` 里现在写的是 9 个独立变量 `R00..R22`。**先确认它们在内存中连续**；若不连续（例如中间夹了别的声明），改成先用局部数组 `double M[9]; touchToRobotMatrix(M);` 再逐个赋过去 —— **不要**为省事去改调用点之外的结构。

- [ ] **Step 4: 跑测试确认通过**

Run: `cmd /c "cd /d D:\Projects\Touch\Touch_Client\tests && .\build_frame_layout_test.bat && .\test_frame_layout.exe"`
Expected: `Results: 14 passed, 0 failed`（原 12 条 + 新 2 条）。

- [ ] **Step 5: 跑整床 + 提交**

Run: `cmd /c "cd /d D:\Projects\Touch\Touch_Client\tests && .\run_tests.bat"` → 期望 exit 0、`Suites accounted: N of N`。

```bash
git add Touch_Client/relay/CoordinateTransform.h Touch_Client/relay/RelayCore.cpp Touch_Client/tests/test_frame_layout.cpp
git commit -m "refactor(relay): 坐标变换抽成唯一一份 + 断言平移/姿态两条路用同一张表"
```

---

### Task 2: 纯函数 `button2OrientationTarget`（真旋转合成）

**Files:**
- Create: `Touch_Client/relay/Button2Mapping.h` / `.cpp`
- Create: `Touch_Client/tests/test_button2_mapping.cpp`
- Create: `Touch_Client/tests/build_button2_mapping_test.bat`
  ⚠ **链接行必须含测试用到的那些 .cpp**：Task 1 的实测先例 —— `convertTouchToRobot` 一被调用就
  LNK2019×3（`Calibration::enabled/R/t` 的定义在 `calibration/CalibrationIO.cpp`）；本任务用
  `TcpCalibration::rpyToMatrix` ⇒ 需要 `..\calibration\TcpCalibration.cpp`（先例见
  `build_coord_safety_test.bat`）。**提交时 .bat 要一起进**（Task 1 的简报漏了它，被实现者补上）。
- Modify: `Touch_Client/tests/run_tests.bat`（接线一个段）

**Interfaces:**
- Consumes: `touchToRobotMatrix`（Task 1）、`TcpCalibration::rpyToMatrix`。
- Produces: `Vec3 button2OrientationTarget(const double refRobotRpy[3], const double refStylusRpy[3], const double curStylusRpy[3]);`（三个入参都是**度**，返回目标 RPY 度）

- [ ] **Step 1: 写失败测试**（`test_button2_mapping.cpp`）

推导表（**本计划的依据，写在 `Button2Mapping.h` 注释里，用例按它写**）：
Touch 标准器件系 = X 右 / Y 上 / Z 朝用户；`M`（Task 1）给出 `robot = M·dev` ⇒
`dev +X → robot +X` · `dev +Y → robot +Z` · `dev +Z → robot −Y`。
笔尖指 **dev −Y**（笔朝下）⇒ 用 M 反推每次"物理摆动"在基座系里是绕哪根轴：

| 物理动作 | 器件系旋转 | 基座系旋转 | 体现在器件 Euler 上 |
|---|---|---|---|
| 笔尖**左**摆（尖端向 −X） | 绕 **−Z** | 绕 **+Y** | `sz` 减小 |
| 笔尖**右**摆（尖端向 +X） | 绕 **+Z** | 绕 **−Y** | `sz` 增大 |
| 笔尖**前**摆（尖端向 +Y） | 绕 **+X** | 绕 **+X** | `sx` 增大 |
| 笔尖**后**摆（尖端向 −Y） | 绕 **−X** | 绕 **−X** | `sx` 减小 |
| 笔杆**自转** | 绕 **±Y** | 绕 **±Z** | `sy` ± |

★ **重要：这张表与现有硬编码矩阵逐行一致**（`robot_dRx=+drx` / `robot_dRy=−drz` / `robot_dRz=+dry`
正好就是 `ω_robot = M·ω_dev` 的三行，我逐行验算过）⇒ **错的不在"表"，在"算术"**：
把 Euler 角之差当旋转向量、再把三个角逐分量加到参照上。**这正是本计划的靶子。**

```cpp
// 判据一律用【矩阵】比，不用 RPY 比 —— RPY 有表示歧义，矩阵没有。
// 期望：R_target = R_axis(±θ) · R_ref（左乘 = 在基座系里倾斜，与规格"尖端朝某方向"同义）
// ⚠ 用本文件【真正求值】的那个宏（Task 1 用的是 CHECK）—— 见本节末尾的假绿警告。
static void axisR(double ax, double ay, double az, double thDeg, double R[9]) {  // 轴角 → 矩阵
    double th = thDeg * 3.14159265358979323846 / 180.0, c = cos(th), s = sin(th), t = 1 - c;
    R[0]=t*ax*ax+c;    R[1]=t*ax*ay-s*az; R[2]=t*ax*az+s*ay;
    R[3]=t*ax*ay+s*az; R[4]=t*ay*ay+c;    R[5]=t*ay*az-s*ax;
    R[6]=t*ax*az-s*ay; R[7]=t*ay*az+s*ax; R[8]=t*az*az+c;
}
static void mul3(const double A[9], const double B[9], double C[9]) {
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) {
        double v=0; for (int k=0;k<3;k++) v += A[i*3+k]*B[k*3+j]; C[i*3+j]=v; }
}
static double angDeg(const double A[9], const double B[9]) {   // 两旋转矩阵夹角（度）
    double T[9]; for (int i=0;i<3;i++) for (int j=0;j<3;j++) T[i*3+j]=A[0*3+i]*B[0*3+j]+A[1*3+i]*B[1*3+j]+A[2*3+i]*B[2*3+j];
    double tr = T[0]+T[4]+T[8]; double c = (tr-1)/2; if (c>1) c=1; if (c<-1) c=-1;
    return acos(c) * 180.0 / 3.14159265358979323846;
}
static void targetM(const double ref[3], const double refS[3], const double cur[3], double R[9]) {
    Vec3 t = button2OrientationTarget(ref, refS, cur); rpyToMatrix(t.x, t.y, t.z, R);
}

// ① 笔杆不动 ⇒ 逐位返回参照（本函数没有位置入参 ⇒ 平移天然不参与）
static void test_no_motion_returns_reference() {
    double ref[3] = {-173.0, -22.0, -118.0}, refS[3] = {-20.0, 12.0, 30.0};
    Vec3 t = button2OrientationTarget(ref, refS, refS);
    double Rt[9], Rr[9]; rpyToMatrix(t.x, t.y, t.z, Rt); rpyToMatrix(ref[0], ref[1], ref[2], Rr);
    CHECK(angDeg(Rt, Rr) < 1e-6);
}

// ②③ 左右摆：sz 减/增 θ ⇒ 基座系绕 +Y/−Y
static void test_left_right_is_yaw_about_base_Y() {
    double ref[3] = {0,0,0}, refS[3] = {0,0,0};
    double L[3] = {0, 0, -30.0}, R[3] = {0, 0, +30.0};
    double RL[9], RR[9], Ry_p[9], Ry_m[9], I[9];
    targetM(ref, refS, L, RL);
    rotY( +30.0, Ry_p); rotY(-30.0, Ry_m); identity(I);
    CHECK(angDeg(RL, Ry_p) < 1e-6);      // 左摆 = 绕 +Y 转 30°
    CHECK(angDeg(RR, Ry_m) < 1e-6);      // 右摆 = 绕 −Y 转 30°
}

// ④⑤ 前后摆：sx 增/减 θ ⇒ 基座系绕 +X/−X
static void test_fore_aft_is_pitch_about_base_X() {
    double ref[3] = {0,0,0}, refS[3] = {0,0,0};
    double F[3] = {+30.0, 0, 0}, B[3] = {-30.0, 0, 0};
    double RF[9], RB[9], Rx_p[9], Rx_m[9];
    targetM(ref, refS, F, RF); targetM(ref, refS, B, RB);
    axisR(1,0,0, +30.0, Rx_p); axisR(1,0,0, -30.0, Rx_m);
    CHECK(angDeg(RF, Rx_p) < 1e-6);     // 前摆 = 绕 +X 转 30°
    CHECK(angDeg(RB, Rx_m) < 1e-6);     // 后摆 = 绕 −X 转 30°
    PASS();
}

// ⑥ 自转：sy 增 θ ⇒ 基座系绕 Z 转 θ（= 工具 roll ⇒ 关节上就是 J6，见实现的注释）
static void test_twist_is_roll_about_base_Z() {
    double ref[3] = {0,0,0}, refS[3] = {0,0,0};
    double T1[3] = {0, +30.0, 0}, T2[3] = {0, -30.0, 0};
    double R1[9], R2[9], Rz_p[9], Rz_m[9];
    targetM(ref, refS, T1, R1); targetM(ref, refS, T2, R2);
    axisR(0,0,1, +30.0, Rz_p); axisR(0,0,1, -30.0, Rz_m);
    CHECK(angDeg(R1, Rz_p) < 1e-6);
    CHECK(angDeg(R2, Rz_m) < 1e-6);
    PASS();
}

// ⑦ ★ 大角度（旧实现真正翻车的地方）：90° 的左右摆仍必须【精确】是绕 +Y 转 90°。
//    旧写法把 Euler 差当旋转向量、还在参照上逐分量加 ⇒ 这里会差几十度。
static void test_large_tilt_is_exact() {
    double ref[3] = {-173.0, -22.0, -118.0};      // 用本机真实的贴接缝姿态
    double refS[3] = {-20.0, 12.0, 30.0};
    double L90[3] = {refS[0], refS[1], refS[2] - 90.0};   // 左摆 90°（sz −90）
    double Rr[9], Rt[9], Ry90[9], want[9];
    rpyToMatrix(ref[0], ref[1], ref[2], Rr);
    targetM(ref, refS, L90, Rt);
    axisR(0,1,0, +90.0, Ry90);
    mul3(Ry90, Rr, want);                          // 期望 = R_y(90°) · R_ref
    CHECK(angDeg(Rt, want) < 1e-6);
    PASS();
}

// ⑧ 限幅按【旋转角】：超过 ORIENT_MAX_OFFSET_DEG 的输入 ⇒ 目标与参照的夹角恰为该上限
static void test_offset_is_clamped_by_rotation_angle() {
    double ref[3] = {0,0,0}, refS[3] = {0,0,0};
    double L170[3] = {0, 0, -170.0};               // 左摆 170°，超过上限 150°
    double Rr[9], Rt[9];
    rpyToMatrix(ref[0], ref[1], ref[2], Rr);
    targetM(ref, refS, L170, Rt);
    CHECK(fabs(angDeg(Rt, Rr) - 150.0) < 1e-3);    // 夹到 ORIENT_MAX_OFFSET_DEG
    PASS();
}
```

⚠⚠ **不要照抄 `TEST(...)`** —— 2026-09-23 Task 1 的实现者**实测复现了假绿**：`test_frame_layout.cpp` 里的 `TEST` 是**标签打印器**（不求值、不计失败），照它写出来的断言即使把映射表改错也照样印 `Results: 12 passed, 0 failed`、退出码 0。
⇒ 用该文件里**真正求值的那个宏**（Task 1 用的是 `CHECK`），并**为每条补 `PASS()`**；**先在你要照抄的那个文件里确认宏的语义**（宏的含义是逐文件不同的，不是仓库级的）。

- [ ] **Step 2: 跑测试确认失败**
Run: `cmd /c "cd /d D:\Projects\Touch\Touch_Client\tests && .\build_button2_mapping_test.bat && .\test_button2_mapping.exe" 2>&1 | tail -3`
Expected: 编译失败（`button2OrientationTarget` 未声明）。

- [ ] **Step 3: 实现**（`Button2Mapping.cpp`）

```cpp
// 真旋转合成（替换"Euler 差当旋转向量 + 逐分量相加"）：
//   ΔR_dev  = R(cur_stylus) · R(ref_stylus)ᵀ        // 笔杆相对按下点的旋转（器件系）
//   ΔR_rob  = M · ΔR_dev · Mᵀ                        // 同一张 M 做相似变换（旋转的坐标变换）
//   R_tgt   = ΔR_rob · R(ref_robot)                  // 左乘 = 在【基座系】里倾斜（规格说的就是基座方向）
//   ⇒ 最后才提取一次 RPY
// 角度限幅：把 ΔR_rob 的【旋转角】夹到 ORIENT_MAX_OFFSET_DEG，再重新合成
//   （旧写法夹的是 RPY 的三个分量 —— 那是表示量，不是旋转量；本改动顺带修掉这一点）
// 「笔杆自转 ⇒ J6」：自转是绕【笔杆自身轴】的旋转 ⇒ ΔR_rob 里含一个 tool-roll 分量；
//   工具轴 = J6 轴时，IK 会用 J6 实现它 ⇒ 规格第 4 条由此自然满足（上机要核 J6 读数，见 Task 4）。
// ★ 2026-09-23 现场+推导结论：现有那张硬编码矩阵【不是】病根 —— `ω_robot = M·ω_dev` 逐行验算成立
//   （见表）。病根是【算术】：drx/dry/drz 是 Euler **角之差**，不是旋转向量的分量；
//   三者又各自被**逐分量加到**参照姿态上。两者都只在小角度下近似成立。
Vec3 button2OrientationTarget(const double refRobotRpy[3], const double refStylusRpy[3], const double curStylusRpy[3]) { ... }
```
（实现要用到 3×3 乘法/转置/轴角↔矩阵；若仓里已有就复用，没有就在本文件内写 `static` 小函数 —— **不要**扩散到公共头。）

- [ ] **Step 4: 跑测试确认通过 + 整床**

- [ ] **Step 5: 提交**

```bash
git add Touch_Client/relay/Button2Mapping.* Touch_Client/tests/test_button2_mapping.cpp Touch_Client/tests/build_button2_mapping_test.bat Touch_Client/tests/run_tests.bat
git commit -m "feat(relay): 按钮2 姿态映射改成真旋转合成（纯函数 + 单测）"
```

---

### Task 3: 接进 RelayCore（开关后，回滚一行）

**Files:**
- Modify: `Touch_Client/relay/RelayCore.cpp:1121-1191`（姿态块）
- Modify: `Touch_Client/config/Config.h`（新增 `BTN2_ROTATION_COMPOSE_ENABLED`）

- [ ] **Step 1: 加开关**（`Config.h`，仿 `ORIENT_SEAM_FIX_ENABLED` 的写法与注释风格，含"回滚 = 一行"）
      **默认值 `true`** —— 新行为就是规格要求的行为；与 09-22 那次实验同一惯例：出问题翻回 `false` 即可，
      **代码与理由都留着**（那次的教训是"回滚 = 一行"救回了成本）。
- [ ] **Step 2: 在姿态块里分支**：`true` 走 `button2OrientationTarget(...)`；`false` 走现有路径（**逐字不动**）
- [ ] **Step 3: `clampOrientOffset` 按新语义**：新增一个按**旋转角**限幅的版本，旧函数保留（`false` 分支仍用它）
- [ ] **Step 4: 保留**死区（`axisGate`）、低通（`s_stylusOffFilt`）、NaN 守卫、`SafetyPredictor::evaluatePositionOnly` 那两道门 —— **不许动它们的语义**
- [ ] **Step 5: 整床 + 提交**（`git commit -m "feat(relay): 按钮2 接新映射（开关后，回滚一行）"`）

---

### Task 4: 上机验证单（写进执行单，不是本计划的代码部分）

**Files:**
- Modify: `Docs/superpowers/specs/2026-09-22-on-machine-run-sheet.md`（新增一节）

- [ ] 四条规格逐条判据（每条含"该看哪个读数"）+ 一条**平移不变性**复查（09-23 的 ① 那次要打问号，需在**不饱和**的环里重测：用一个 `rx` 远离 ±180 的位姿）
- [ ] **J6 复查**：笔杆自转时读 `J6`（从 `RP|`/`J|` 两条都能看），确认"自转 ⇒ J6 转、其余关节基本不动"
- [ ] **接缝稳定性**：在 `rx` 贴 ±180 的常态位姿下，确认没有"抖动/超速"（09-22 那条被否掉的修法就是死在这里）
- [ ] ⚠ **若接缝仍咬死**：下一步不是继续调映射，而是评估 **ServoJ（关节空间伺服）** —— 用 RPY 表示一个常年落在 `rx≈±180` 的姿态，本身就在**表示奇点**附近（本机 `ry` 还常在 −66~−90，RPY 的奇点在 `ry=±90`）。

---

## Self-Review

**1. 规格覆盖**：平移不动 → Task 2（函数没有位置入参）+ Task 4（上机复查）；左/右摆 → Task 2 用例②③；前/后摆 → 用例④⑤；自转 → J6 → Task 2 的 tool-roll + Task 4 的 J6 复查 ✓。
**2. 占位符**：Task 2 的 Step 1 明写"先做 Step 3 的推导再回来写完整用例"—— 这是**刻意的顺序约束**（推导表是这套映射的唯一依据，用例必须按它写），不是占位符；推导表本身要在 Step 3 的注释里写全 ✓。
**3. 命名一致**：`touchToRobotMatrix` / `button2OrientationTarget` / `BTN2_ROTATION_COMPOSE_ENABLED` 三个名字在任务间一致 ✓。
**4. 已知风险**（写进计划而非藏起来）：本机末端姿态常年贴 `rx≈±180`、`ry` 靠近 −90 ⇒ **RPY 表示本身就在奇点附近**；本计划修正的是**映射算法**，若上机发现接缝/奇点仍咬死，那是**表示层**的问题，出口是 ServoJ（Task 4 已记账）。
