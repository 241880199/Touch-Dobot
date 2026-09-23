# 按钮2 改关节空间（J4/J5/J6）Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 按钮2 改为**关节空间**控制：**笔杆前后摆 ⇒ J4** · **左右摆 ⇒ J5** · **自转 ⇒ J6**；J1/J2/J3 保持按下时的值；**平移不参与**。

**Architecture:** 一个**纯函数** `button2JointTarget(refJoints[6], refStylus[3], curStylus[3], out[6])`：
笔杆三个 Euler 角的增量 → 逐轴死区 → 增益 → **偏移角限幅** → **一一对应**加到**按下时的关节参照**上；
`RelayCore` 用**厂商的 `ServoJ`**（关节空间动态跟随）下发全六个关节。**无 IK、无 RPY、无 `clampOrientToBounds`。**

**Tech Stack:** C++17 / MSVC；厂商接口 `ServoJ(J1..J6, t, lookahead_time, gain)`（文档：建议 **33 Hz / 30 ms** 循环调用；
例 `ServoJ(0,0,-90,0,90,0,t=0.1,lookahead_time=50,gain=500)`）；测试走 `Touch_Client/tests/`（套件数**运行时数出并断言**，当前 23）。

## Global Constraints

- **为什么改**（写进代码注释）：原方案（末端 RPY）在本机**常年贴着 RPY 的接缝与奇点**（`|rx|>170` 占 85.8%、
  `ry` 常在 −66~−90）⇒ 才需要 `clampOrientToBounds` / 逐分量限幅 / 跨接缝回避那一整套，而 09-22 的关节超速
  就出在那里。**关节空间绕开整层** ✓（与项目旧结论"要真正解决多半得走关节空间"一致）。
- **器件侧的源轴已实测**（2026-09-23，三姿势 SPACE，不按按钮、臂不动）：**前后摆 = 笔杆 `Rx`（主导 +20.3°）** ·
  **左右摆 = 笔杆 `Rz`（主导 −34.7°）** · **自转 = `Ry`** ✓ ⇒ **源侧不再是假设**。
- **回滚必须是一行**：开关 `Config::BTN2_JOINT_SPACE_ENABLED`（默认 `true`），`false` 时**逐字**走现有
  `button2OrientationTarget`（RPY）那条路 ✓。
- **平移仍然不参与** ✓（新函数没有位置入参）。
- **安全**：位置类检查（工作空间/Z 行程/奇异/报警点）**不许删** ⇒ 关节目标先**正解**（`robot/Kinematics`）
  得到末端位姿，再走**现有**的 `SafetyPredictor` 入口 ✓（与 RPY 那条路同一个门 ✓）。
- `.bat` 纯 ASCII；C++ 注释中文；每次提交后整床 **exit 0** 且 `Suites accounted: N of N`。

---

### Task 1: 纯函数 `button2JointTarget` + 单测

**Files:**
- Create: `Touch_Client/relay/Button2Joint.h` / `.cpp`（纯函数；**不碰 socket、不碰 appState**）
- Create: `Touch_Client/tests/test_button2_joint.cpp` + `build_button2_joint_test.bat`
- Modify: `Touch_Client/tests/run_tests.bat`（接线一段；套件数 23 → 24）

**Interfaces:**
- Produces: `void button2JointTarget(const double refJoints[6], const double refStylus[3], const double curStylus[3], double outJoints[6]);`（角度全为**度**）

- [ ] **Step 1: 写失败测试**（真值用构造输入，不读实现）

```cpp
// 结构性判据（这是本方案的核心，且**与符号无关**）—— 一条用例就能钉住"一一对应"：
//   只动笔杆的一个角 ⇒ **恰好一个关节**动，且 J1/J2/J3 **逐位不变**。
static void test_each_stylus_axis_moves_exactly_one_joint() {
    double ref[6] = {10, 20, 30, 40, 50, 60}, refS[3] = {0,0,0};
    // 前后摆(笔杆 Rx +10) ⇒ 只有 J4 变
    double cur[3] = {+10.0, 0, 0}, out[6];
    button2JointTarget(ref, refS, cur, out);
    CHECK(fabs(out[3] - ref[3]) > 1e-6);
    CHECK(fabs(out[0]-ref[0])<1e-9 && fabs(out[1]-ref[1])<1e-9 && fabs(out[2]-ref[2])<1e-9);
    CHECK(fabs(out[4]-ref[4])<1e-9 && fabs(out[5]-ref[5])<1e-9);
    // 左右摆(笔杆 Rz −10) ⇒ 只有 J5 变；自转(笔杆 Ry +10) ⇒ 只有 J6 变   ……(同样两条)
}
// 还有三条：① 笔杆不动 ⇒ 逐位等于参照；② 平移无入参（函数签名里就没有 ✓）；
//          ③ 偏移限幅: 笔杆转 170° ⇒ |J4 参照差| == Config::ORIENT_MAX_OFFSET_DEG
//          ④ 死区: 笔杆只转 0.01°(< ORIENT_DEADZONE_DEG) ⇒ 关节逐位不变
```

⚠ **断言宏**：本仓 `TEST` 是**标签打印器（会假绿）**——照抄前先在目标文件里核宏语义，用真求值的那个（多为 `CHECK`）+ `PASS()`。
⚠ 新增套件 ⇒ **必须在同一次改动里**接进 `run_tests.bat`（不接线会 MISMATCH + exit 1 —— 那是设计 ✓）。

- [ ] **Step 2: 跑测试确认失败**（未声明）
- [ ] **Step 3: 实现**（`Button2Joint.cpp`）

```cpp
// 映射（源侧已实测 ⇒ 这三条是【标定结论】, 不是推导）：
//   笔杆 Rx(前后摆) ⇒ J4 · 笔杆 Rz(左右摆) ⇒ J5 · 笔杆 Ry(自转) ⇒ J6
// 逐轴符号: 先取 Config::BTN2_J4_SIGN / _J5_SIGN / _J6_SIGN（默认 +1, 由 Task 3 上机实测钉）
// 限幅: 对【关节增量】用 ORIENT_MAX_OFFSET_DEG（150°），比 ORIENT_DEADZONE_DEG 逐轴门限（0.05°）
//       统一用现成的常数 —— 但在本文件注释里写明"它们现在是【关节增量】的度, 不是末端姿态的度"。
```

- [ ] **Step 4: 跑测试确认通过** + 整床（exit 0、`N of N`）
- [ ] **Step 5: 提交**

---

### Task 2: 接进 `RelayCore`（开关后，回滚一行）

**Files:**
- Modify: `Touch_Client/config/Config.h`（`BTN2_JOINT_SPACE_ENABLED` 默认 true + 逐轴符号常数）
- Modify: `Touch_Client/relay/RelayCore.cpp`（按下按钮2 时抓**关节**参照；命令构造处 `:1384` 分叉）

**要点（照此实现）**
1. **参照**：`onButton2Press` 里除了现有的笔杆/姿态参照，再抓 `app.robotActualPose.j1..j6`（**已有的字段** ✓，
   见 `:991/:1220` 的用法 ✓）⇒ 新函数只需要 `refJoints/refStylus/curStylus` ✓（**无需** `m_orientRefRobot` ✗）。
2. **下发**：`RelayCore.cpp:1384` 处按开关分叉 —— 新路径构造
   `snprintf(cmd, sizeof(cmd), "ServoJ(%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,0.1,50,500)", j1..j6)` ✓
   （参数照厂商示例 ✓；**保持现有的 30 Hz 节流** —— 厂商建议 33 Hz ✓ 同量级 ✓）。
3. **安全**：关节目标先用 `robot/Kinematics` 算**末端位姿**，再走**现有**的 `SafetyPredictor` 位置门 ✓
   （工作空间/Z 行程/奇异/报警点 ⇒ 全保留 ✓）；**不做** `clampOrientToBounds`（新路径不需要 ✓）。
4. **保留**：死区/增益/限幅语义（现在作用在**关节增量**上 ✓，注释写明）；`false` 分支**逐字**走旧路径 ✓。
- [ ] Step 1 加开关与符号常数 · Step 2 接线（上面 4 条）· **Step 3 编译 + 负对照**（注入语法错误 ⇒ 必须报 `error C…`，
      客户端**先退出**再构建，否则 `LNK1168`）· Step 4 整床 · Step 5 提交
      （提交信息写明：开关名/默认值/回滚、以及"`RelayCore.cpp` 不被任何测试编译 ⇒ 接线只能靠编译+负对照+上机守"）

---

### Task 3: 上机验证单（写进执行单，不是代码）

- [ ] 三条逐轴判据（**一次只动一根笔杆轴**，看**恰好一个关节**在动、另五个不动）：
      前后摆 ⇒ **J4** · 左右摆 ⇒ **J5** · 自转 ⇒ **J6**（`J|` 一行就能看 ✓）
- [ ] **符号**：每个轴往正方向摆 ⇒ 记下关节往哪边走 ⇒ 三种情况决定 `BTN2_J4/J5/J6_SIGN` ✓
      ⚠ 这是本方案**唯一没量过的部分**（源侧已实测 ✓，机器人侧的方向/比例未量 ✓）
- [ ] **保持性**：J1/J2/J3 在按住期间**逐位不变** ✓（若有漂 ⇒ 参照没抓对 ✗）
- [ ] **平移不变**：按住按钮2 平移笔杆 ⇒ 关节全不动 ✓
- [ ] **安全门仍在**：把末端推向工作空间/Z 行程边缘 ⇒ 应当被**拒绝**（不是静默钳位 ✓）
- [ ] 若"J4/J5/J6 谁对应谁"实测**不是**前后/左右/自转 ⇒ 报回来（改一个映射表 + 两条用例 ✓ 不动结构）

---

## 明确【不要】做的

- 不删现有 RPY 路径（它留在 `false` 分支 ✓ —— 它也解释了"为什么不能那么做" ✓）。
- 不动力映射/死区/增益（那是**另一条线**：接触期横向噪声 ✓）。
- 不改 `SAFE_*` 常数；不为了让"看得动"而关安全门 ✗。
