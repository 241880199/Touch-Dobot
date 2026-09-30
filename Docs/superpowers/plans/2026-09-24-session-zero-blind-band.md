# 空载残余（盲带）修复 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让"空载自由运动时手上没有力"（= 关掉力反馈时的感觉），做法是把补偿后那道**慢速零偏残余**从源头消掉，而不是继续在环路或死区形状上换方案。

**Architecture:** 启动时在主线程采两段**静止且未按按钮**的窗口（两段之间机械臂的姿态必须真的变过）⇒ 得到一个**慢速残余**。**只在残余 (a) 超过死区 且 (b) 与姿态无关** 时才把它作为**会话零偏**装进流水线（只作用于 `filtered` 的下游，**不进一致性闸门的 EMA**）；否则**拒绝安装并明说"去重标模型"**。判定的全部逻辑放在一个纯函数里以便单测与负对照。**不采用连续在线估计器**（它会把操作员的持续按压学成零偏）。

**Tech Stack:** C++（MSVC，`cl`，逐套件 `build_*.bat`）、本仓自带的 `TEST/PASS/CHECK` 测试床（`Touch_Client/tests/run_tests.bat`）、MATLAB relay 不参与本计划。

## Global Constraints

- **判据（用户给定，唯一验收标准）**：**正确手感 = 关掉力反馈开关时的感觉**（空载手上**没有**力）；**按压/写字仍要有力量感**。
- **不许碰环路**：斥力 / 位置耦合 / 姿态耦合 / 惯量 / 粘性阻尼 / `−F/k` 六个方向已全部实测失败（它们是**放大器**，不是激励源）。本计划**不改**它们。
- **不许改死区形状、不许改增益**：残余 0.32 N 与写字横向信号 0.35 N **同量级** ⇒ 任何固定幅值整形都分不开。
- **会话零偏只存内存，绝不落盘**：`force_calib.json` 描述的是「重启之后的稳态」，写回它会把一次可能受污染的零偏固化成下一场的基准。
- **闸门看到的必须是未修正的模型**：会话零偏**不得**进入 `ForceCompensation` 的闸门 EMA（`guardReport.ema`）；它只作用在 `ForcePipeline::step` 里 `fd.filtered` 的产生点。
- **常量一处定义**：共享量（死区）一律从 `Config.h` 取，**不许在别处再写一份字面量**。
- **测试**：新用例**必须**带至少一个**负对照**（把被测的那条防线拿掉 ⇒ 该用例必须变红）；每次提交前 `Touch_Client/tests/run_tests.bat` 必须 exit 0。
- **`.bat` 必须纯 ASCII**；提交信息用中文；`git commit -F -`（不要 `-m`）。
- **push 一律** `git -c http.sslBackend=schannel push`；仓库**公开**，推前核范围，**永不** `git add -A`。
- **一行可回滚**：本计划每个 Task 结束时都必须处于「可独立回滚」的状态。

---

## File Structure

| 文件 | 责任 | 动作 |
|---|---|---|
| `Touch_Client/config/Config.h` | 三个共享常数的**唯一**定义处。改 `FORCE_ZERO_DRIFT_WARN_N` | Modify |
| `Touch_Client/force/SessionZero.h` | **新**：判定纯函数 + 三个模块内常量（照 `ForceTuning.h` 的先例把模块自己的常量放自己头里） | Create |
| `Touch_Client/force/ForcePipeline.h` / `.cpp` | 会话零偏的**安装/清除/回读** + 在 `step()` 里应用 | Modify |
| `Touch_Client/main.cpp` | 采样状态机（两段窗口 + 姿态变化判定 + 调用安装 + 打印） | Modify |
| `Touch_Client/tests/test_force_pipeline.cpp` | 盲带不变量用例 + 会话零偏**生效**用例（值断言） | Modify |
| `Touch_Client/tests/test_force_compensation.cpp` | `SessionZero::decide` 的用例（含两个负对照**对应的**红） | Modify |
| `Docs/superpowers/specs/2026-09-24-on-machine-run-sheet.md` | 上机判据（Task 2 / Task 5 各一条） | Modify |

**为什么把测试加进两个既有套件而不是新建套件**：本仓测试床是「运行时数出 `test_*.cpp` 并断言 `TOTAL+NNOTRUN==NTESTS`」，新建套件要连带新 `build_*.bat` 与 `NOTRUN_LIST` 记账（见 `test-harness-stale-binaries` 的教训）。两个既有套件已经分别装着「补偿/判定」与「流水线整形」这两个主题 —— 加用例不需要动测试床。

---

### Task 1: 让盲带会响（1 行 + 一条不变量断言）

**问题**：现在「报警容差 0.5 N」**大于**「能拦住的死区 0.20 N」⇒ 中间 [0.20, 0.50] 这条**盲带**里，检查说"正常"而手上的力全额过去。实测残余 0.3187 N 正落在这里。

**Files:**
- Modify: `Touch_Client/config/Config.h`（`FORCE_ZERO_DRIFT_WARN_N`，现 `0.5`，第 185 行附近）
- Test: `Touch_Client/tests/test_force_pipeline.cpp`（追加一个用例 + 在 `main()` 列表里加一行）

**Interfaces:**
- Consumes: `Config::FORCE_ZERO_DRIFT_WARN_N`、`Config::FORCE_RESIDUAL_DEADZONE_N`（都在 `Touch_Client/config/Config.h`）
- Produces: 无（本 Task 只改一个常数与一条断言）

- [ ] **Step 1: 先写会红的用例**

在 `Touch_Client/tests/test_force_pipeline.cpp` 的 `test_saturation_sensor_n()` 之后追加：

```cpp
// ===== 盲带不变量 (2026-09-24) =====
// 判据: 【允许的残余】必须 <= 【能拦住的残余】。
//   报警容差 = 启动检查说"正常"的上界; 死区 = 空载时手上仍有力的上界。
//   容差大于死区 ⇒ 中间那一段(盲带)里, 检查说正常而手上的力【全额】过去
//   (softDeadzone 在门限以上【原样放行】, 不减幅)。
// ⚠ 这条断言是【关系式】而不是数字: 钉住的是"两个常数不许再走岔", 不是某个具体值。
static void test_warn_threshold_within_deadzone() {
    TEST(warn_threshold_within_deadzone);
    const double warnN  = Config::FORCE_ZERO_DRIFT_WARN_N;
    const double deadN  = Config::FORCE_RESIDUAL_DEADZONE_N;
    std::cout << "(warn=" << warnN << " deadzone=" << deadN << ") ";
    CHECK(warnN > 0.0);        // 非正数等于关掉整条检查, 不行
    CHECK(warnN <= deadN);     // ★ 盲带不变量
    PASS();
}
```

在 `main()` 的用例调用列表里加一行（位置照现有列表顺序即可）：

```cpp
    test_warn_threshold_within_deadzone();
```

- [ ] **Step 2: 跑它，确认是【红】的（负对照的第 0 步）**

```bash
cd /d/Projects/Touch/Touch_Client/tests && cmd /c ".\build_force_pipeline_test.bat" && ./test_force_pipeline.exe
```

Expected: `warn_threshold_within_deadzone` 打出 `FAIL: warnN <= deadN`（因为 warn=0.5、deadzone=0.2），进程列表里该用例**不是** PASS。

- [ ] **Step 3: 改那一行常数**

`Touch_Client/config/Config.h`，把

```cpp
    const double FORCE_ZERO_DRIFT_WARN_N = 0.5;
```

改成（连带把原来那句已经过期的理由改掉，因为它正是盲带的来源）：

```cpp
    // ★ 2026-09-24 深夜: 0.5 → 0.15。旧值写的理由是"明显高于死区 0.20 N 与噪声本底
    //   (~0.05 N), 免得天天误报" —— 而【高于死区】正是盲带的来源: 残余落在
    //   [死区 0.20, 本值] 区间时, 死区拦不住(softDeadzone 门限以上原样放行)、
    //   本检查又说"正常" ⇒ 空载时手上凭空一股力。现场实测残余 0.3187 N 就落在那里。
    //   本值量的是【窗口均值】, 不是单帧: 单帧噪声 sd ≈ 0.147 N, 而 1 s 窗口(≈123 帧)
    //   的均值噪声 ≈ 0.147/√123 ≈ 0.013 N ⇒ 0.15 N 离误报还有 ~11σ, 不会天天报。
    //   ⚠ 它必须 <= FORCE_RESIDUAL_DEADZONE_N —— 由 test_force_pipeline.cpp 的
    //     test_warn_threshold_within_deadzone 钉住, 别只改一边。
    const double FORCE_ZERO_DRIFT_WARN_N = 0.15;
```

- [ ] **Step 4: 跑它，确认变【绿】**

```bash
cd /d/Projects/Touch/Touch_Client/tests && cmd /c ".\build_force_pipeline_test.bat" && ./test_force_pipeline.exe
```

Expected: `warn_threshold_within_deadzone` → `PASS`，且整套件无 FAIL。

- [ ] **Step 5: 负对照（把防线拿掉，确认它确实会红）**

把常数临时改回 `0.5` ⇒ 重跑 ⇒ **必须**复现 Step 2 的红；再改回 `0.15`。**这一步不做完不算完成**。

- [ ] **Step 6: 全套件回归**

```bash
cd /d/Projects/Touch/Touch_Client/tests && cmd /c ".\run_tests.bat"; echo "exit=$?"
```

Expected: 全部套件通过、`exit=0`（当前基线 `26 of 26 (ran 24 + not-run 2)`）。

- [ ] **Step 7: 提交**

```bash
cd /d/Projects/Touch && git add Touch_Client/config/Config.h Touch_Client/tests/test_force_pipeline.cpp && git commit -F - <<'MSG'
fix(force): 报警容差 0.5 → 0.15 —— 关掉"允许的残余比能拦住的还大"这条盲带

残余落在 [死区 0.20, 容差 0.5] 时: softDeadzone 在门限以上【原样放行】, 而启动检查
说"正常" ⇒ 空载时手上凭空一股力(实测 0.3187 N ⇒ ×1.98 ≈ 0.63 N), 且它就是
闭环自激的激励源。本提交只让这种情况【会响】, 不改力。

钉住它的是一条【关系式】断言(warn <= deadzone), 而不是数字 —— 防的是两个常数再次走岔。
负对照已实测: 改回 0.5 ⇒ 该用例变红。
MSG
```

---

### Task 2: 量「残余在一场会话里会不会长大」（**阻塞项**）

**为什么阻塞**：修法的形态取决于它。**不长大** ⇒ 启动一次性调零就够（风险最小）；**长大（热漂）** ⇒ 必须允许第二次机会（Task 6），那就得正面处理"估计器把操作员手劲学走"的坑。**在拿到这条数据之前不要落地 Task 5 的接线**（Task 3/4 只写纯函数与接缝，没有行为改动，可以先做）。

**Files:**
- Modify: `Touch_Client/main.cpp`（新增一个**只读**的周期轨迹函数 + 在同一调用点调用）
- Modify: `Docs/superpowers/specs/2026-09-24-on-machine-run-sheet.md`（记判据）

**Interfaces:**
- Consumes: `appState.forceData.filtered[0..2]`（持 `appState.forceDataMutex`）、`appState.robotActualPose.rx/ry/rz`（持 `appState.robotPoseMutex`）、`ForceCompensation::guardState()`
- Produces: 控制台行，前缀固定为 `[ZeroTrace]`（供人眼与脚本同时用）

- [ ] **Step 1: 加轨迹函数**

在 `Touch_Client/main.cpp` 里 `runZeroDriftCheck(...)` 的**下面**追加。⚠ 它必须是**纯只读**：不改任何状态、不影响任何力，只打印。

```cpp
// ===== 会话内残余轨迹 (2026-09-24) =====
// 只读诊断: 每 60 s 打一次窗口均值, 用来回答一个决定了修法形态的问题 ——
//   【补偿后的零偏残余, 在一场会话里会不会随时间长大?】
// 判据(先写死, 免得事后圆说):
//   · 全程 |残余| 都在启动值附近(±0.05 N) ⇒ 残余是【常数】⇒ 启动一次性调零就够。
//   · 单调增大且超过启动值 +0.10 N 以上 ⇒ 有【热漂/机械漂】⇒ 必须允许第二次机会(Task 6)。
// ⚠ 这里打印的是【未过死区、未修正】的 fd.filtered —— 将来装了会话零偏之后,
//   残余 = 本行打印值 + 已装零偏(见 [ZeroTrace] 行里的 offset), 别把它读成"残余消失了"。
static DWORD g_zeroTraceLastMs = 0;
static void runResidualTrace() {
    const DWORD now = GetTickCount();
    if (g_zeroTraceLastMs != 0 && (now - g_zeroTraceLastMs) < 60000) return;
    g_zeroTraceLastMs = now;

    AppState::ForceData fd;
    EnterCriticalSection(&appState.forceDataMutex);
    fd = appState.forceData;
    LeaveCriticalSection(&appState.forceDataMutex);
    if (fd.isStale) { std::cout << "[ZeroTrace] 帧陈旧, 本次不采\n" << std::flush; return; }

    double px, py, pz;
    EnterCriticalSection(&appState.robotPoseMutex);
    px = appState.robotActualPose.rx; py = appState.robotActualPose.ry; pz = appState.robotActualPose.rz;
    LeaveCriticalSection(&appState.robotPoseMutex);

    double off[3];
    ForcePipeline::sessionZeroOffset(off);
    const double r = ForcePipeline::residualNorm(fd.filtered);   // 见 Task 4 的 helper
    std::cout << "[ZeroTrace] 残余(|filtered|)=" << r
              << " N  分量=(" << fd.filtered[0] << "," << fd.filtered[1] << "," << fd.filtered[2] << ")"
              << "  已装零偏=(" << off[0] << "," << off[1] << "," << off[2] << ")"
              << "  姿态=(" << px << "," << py << "," << pz << ")"
              << "  t=" << (now / 1000) << "s\n" << std::flush;
}
```

- [ ] **Step 2: 在同一调用点调用它**

`runZeroDriftCheck` 与 `runConstraintDisableNotice()` 在**同一个调用点**（`grep -n "runConstraintDisableNotice()" Touch_Client/main.cpp` 定位那一对调用）。在它们旁边加：

```cpp
        runResidualTrace();
```

- [ ] **Step 3: 构建（不跑测试床也能编，但跑一遍更省事）**

```bash
cd /d/Projects/Touch/Touch_Client/tests && cmd /c ".\run_tests.bat"; echo "exit=$?"
```

Expected: `exit=0`（本 Task 只加打印，测试床不动）。

- [ ] **Step 4: 重建客户端**（用户已授权关闭客户端；按 PID 优雅关，**别在操作员做动作时关**）

```bash
cd /d/Projects/Touch/Touch_Client && cmd /c ".\build.bat"
```

- [ ] **Step 5: 上机跑一次，记两件事**

启动后**不要动**，让 `[ZeroTrace]` 出第一行（t≈60s）；然后**正常做 10 分钟**（含按压/写字），收尾前再抄一行。**把两行贴回给计划执行者**。

Expected: 两行的 `残余` 与 `姿态` 都可以对上；据此选 Task 6 做/不做。

- [ ] **Step 6: 提交**

```bash
cd /d/Projects/Touch && git add Touch_Client/main.cpp && git commit -F - <<'MSG'
diag(force): 加 [ZeroTrace] —— 残余在一场会话里会不会随时间长大

只读、每 60 s 一行。判据先写死在注释里: 常数则启动一次性调零够; 单调增大 ⇒ 要第二次机会。
这是修法形态的【阻塞项】: 它决定 Task 6 做不做。
MSG
```

---

### Task 3: 判定纯函数 `SessionZero::decide`（**含"能拒绝"**）

**Files:**
- Create: `Touch_Client/force/SessionZero.h`
- Test: `Touch_Client/tests/test_force_compensation.cpp`（该套件已 `#include "../force/ZeroDriftCheck.h"`，同风格追加）

**Interfaces:**
- Produces:
  - `SessionZero::Action { Waiting, Nothing, Install, RefusePosture }`
  - `struct SessionZero::Input`（字段见下面代码，全部为值类型）
  - `struct SessionZero::Decision { Action action; double offset[3]; double driftN; double postureDeltaN; std::string text; }`
  - `SessionZero::Decision SessionZero::decide(const Input& in)`
  - 常量 `SessionZero::POSE_STABLE_DEG` / `MIN_POSE_CHANGE_DEG` / `POSTURE_TOL_FRACTION`（模块自己的常量放自己头里，照 `ForceTuning.h` 的先例）
- Consumes（Task 5 会传）：`Input.deadzoneN` 由调用方从 `Config::FORCE_RESIDUAL_DEADZONE_N` 填入 —— **纯函数内部不许读 Config**

- [ ] **Step 1: 先写会红的用例**

在 `Touch_Client/tests/test_force_compensation.cpp` 顶部加 `#include "../force/SessionZero.h"`，然后追加五个用例：

```cpp
// ===== 会话零偏判定 (2026-09-24) =====
// 全部是【纯函数】用例: 不碰硬件、不碰全局。判据来自现场:
//   空载手上不该有力; 而残余【超过死区】且【与姿态无关】时, 它就是零偏 ⇒ 可以减。
//   ⚠ 两条防线各有一个负对照用例(见文件末的 NC-1 / NC-2)。
static SessionZero::Input baseInput() {
    SessionZero::Input in;
    in.guardOk = true; in.frameFresh = true; in.buttonHeld = false; in.poseStableA = true;
    in.deadzoneN = Config::FORCE_RESIDUAL_DEADZONE_N;   // 0.20
    in.minSamples = 8;
    in.countA = 40; in.countB = 40; in.haveB = true;
    in.poseA[0] = -175.0; in.poseA[1] = 4.0;  in.poseA[2] = -134.0;
    in.poseB[0] = -170.0; in.poseB[1] = 4.0;  in.poseB[2] = -134.0;   // 差了 5.0°
    in.meanA[0] = 0.31; in.meanA[1] = 0.02; in.meanA[2] = -0.03;
    in.meanB[0] = 0.29; in.meanB[1] = 0.01; in.meanB[2] = -0.02;
    return in;
}

static void test_session_zero_installs_when_consistent_and_over_deadzone() {
    TEST(session_zero_installs_when_consistent_and_over_deadzone);
    SessionZero::Input in = baseInput();
    const SessionZero::Decision d = SessionZero::decide(in);
    CHECK(d.action == SessionZero::Action::Install);
    // 值断言(不是"没崩"): 偏移 = 两段窗口的均值
    CHECK(fabs(d.offset[0] - 0.30) < 1e-9);
    CHECK(fabs(d.offset[1] - 0.015) < 1e-9);
    CHECK(fabs(d.offset[2] - (-0.025)) < 1e-9);
    CHECK(fabs(d.driftN - SessionZero::norm3(d.offset)) < 1e-9);
    CHECK(d.text.find("装") != std::string::npos);
    PASS();
}

static void test_session_zero_nothing_inside_deadzone() {
    TEST(session_zero_nothing_inside_deadzone);
    SessionZero::Input in = baseInput();
    in.meanA[0] = 0.05; in.meanA[1] = 0.01; in.meanA[2] = -0.01;
    in.meanB[0] = 0.04; in.meanB[1] = 0.01; in.meanB[2] = -0.01;   // 都在死区内
    const SessionZero::Decision d = SessionZero::decide(in);
    CHECK(d.action == SessionZero::Action::Nothing);
    CHECK(fabs(d.offset[0]) < 1e-12 && fabs(d.offset[1]) < 1e-12 && fabs(d.offset[2]) < 1e-12);
    CHECK(d.text.find("死区") != std::string::npos);
    PASS();
}

// ★★ 这一条就是"能拒绝": 两段姿态下残余不一致 ⇒ 它是【姿态相关】的
//    ⇒ 不是零偏, 而是模型/负载不对(换装工具后的典型症状) ⇒ 不许减, 要求去重标。
static void test_session_zero_refuses_when_posture_dependent() {
    TEST(session_zero_refuses_when_posture_dependent);
    SessionZero::Input in = baseInput();
    in.meanB[0] = 0.05; in.meanB[1] = 0.40; in.meanB[2] = -0.30;   // 与姿态 A 明显不一致
    const SessionZero::Decision d = SessionZero::decide(in);
    CHECK(d.action == SessionZero::Action::RefusePosture);
    CHECK(fabs(d.offset[0]) < 1e-12 && fabs(d.offset[1]) < 1e-12 && fabs(d.offset[2]) < 1e-12);
    CHECK(d.text.find("重标") != std::string::npos);
    PASS();
}

// ★★ 这一条是"别把操作员的手劲学走": 按着按钮 = 他正在用力 ⇒ 一律不结论。
static void test_session_zero_waits_while_button_held() {
    TEST(session_zero_waits_while_button_held);
    SessionZero::Input in = baseInput();
    in.buttonHeld = true;
    const SessionZero::Decision d = SessionZero::decide(in);
    CHECK(d.action == SessionZero::Action::Waiting);
    CHECK(fabs(d.offset[0]) < 1e-12);
    PASS();
}

static void test_session_zero_waits_until_pose_actually_changed() {
    TEST(session_zero_waits_until_pose_actually_changed);
    SessionZero::Input in = baseInput();
    in.poseB[0] = in.poseA[0] + 1.0;   // 只差 1°, 小于 MIN_POSE_CHANGE_DEG(5°)
    const SessionZero::Decision d = SessionZero::decide(in);
    CHECK(d.action == SessionZero::Action::Waiting);
    CHECK(d.text.find("姿态还没变") != std::string::npos);
    PASS();
}
```

在 `main()` 里按现有列表加五行：

```cpp
    test_session_zero_installs_when_consistent_and_over_deadzone();
    test_session_zero_nothing_inside_deadzone();
    test_session_zero_refuses_when_posture_dependent();
    test_session_zero_waits_while_button_held();
    test_session_zero_waits_until_pose_actually_changed();
```

- [ ] **Step 2: 跑它，确认是【红】的**

```bash
cd /d/Projects/Touch/Touch_Client/tests && cmd /c ".\build_force_compensation_test.bat"
```

Expected: **编不过**（`SessionZero.h` 还不存在 / `SessionZero` 未定义）—— 这就是这一步的"红"。

- [ ] **Step 3: 写实现**

新建 `Touch_Client/force/SessionZero.h`：

```cpp
#pragma once
// ===== 会话零偏的判定 (2026-09-24) =====
// 目的: 空载自由运动时手上【不该有力】。补偿后那道【慢速残余】在超过死区时会被
//   softDeadzone【原样放行】(门限以上不减幅) ⇒ 全额到手(×净比 ≈2) ⇒ 它还是闭环自激的
//   激励源。实测残余 0.3187 N ⇒ 手上 ≈0.63 N 恒力。
//
// 为什么是【会话内零偏】而不是调死区/调增益:
//   残余 0.32 N 与写字时的横向信号 0.35 N(同口径 raw sd)【同量级】⇒ 任何【固定】的
//   幅值整形都分不开它们(调低 ⇒ 空载漏力; 调高 ⇒ 掐掉写字反馈)。能分开它们的只有
//   【时间结构】: 残余是慢的(直流), 写字信号是快的 ⇒ 把慢的减掉。
//
// 为什么要有【能拒绝】这一条 —— 这是本模块存在的理由:
//   残余【不是】零偏时(典型: 换装工具后负载模型不对), 它是【姿态相关】的。此时按一个
//   姿态的读数去减, 会把那个姿态修好、把别的姿态弄坏(可能更坏)。
//   ⇒ 减去之前必须先证明"与姿态无关"; 证不出来就【不减】, 并且明说去重标模型。
//
// ⚠ 本模块【不】做连续自适应: 估计器分不清"慢的力"与"零偏", 会把操作员长匀速笔画时的
//   手劲慢慢学走(阻力几秒内消失)。所以只在【未按按钮 + 静止】的窗口里采, 且只用两段
//   窗口给出的常数零偏。
//
// 纯函数: 不读全局、不看时钟、不打印。时钟/采样/打印都在 main.cpp 的调用方。
//   模块自己的常量放在本头里(照 force/ForceTuning.h 的先例)。

#include <string>
#include <cstdio>
#include <cmath>

namespace SessionZero {

    // 一个采样窗口内姿态允许的最大波动(度) —— 超过就认为"机械臂在动", 该窗口作废。
    constexpr double POSE_STABLE_DEG = 1.0;
    // 两段窗口之间姿态必须变过的最小角度(度) —— 否则两段其实是同一个姿态, 证不了姿态无关。
    constexpr double MIN_POSE_CHANGE_DEG = 5.0;
    // 两段窗口一致性容差 = 死区的一个比例(见 decide 的注释: 为什么不用绝对值)。
    constexpr double POSTURE_TOL_FRACTION = 0.5;

    enum class Action {
        Waiting,        // 条件不满足, 不作结论(不是"通过")
        Nothing,        // 残余已在死区以内 ⇒ 空载本来就安静, 不需要做任何事
        Install,        // 残余超死区且与姿态无关 ⇒ 装成会话零偏
        RefusePosture   // 残余与姿态相关 ⇒ 不是零偏 ⇒ 不许减, 去重标模型
    };

    struct Input {
        bool guardOk       = false;   // ForceCompensation::guardState() == OK
        bool frameFresh    = false;   // !fd.isStale
        bool buttonHeld    = false;   // appState.lastButtonState —— 按着按钮 = 正在用力
        bool poseStableA   = true;    // 窗口 A 内姿态波动 < POSE_STABLE_DEG
        bool poseStableB   = true;    // 窗口 B 内同理
        int  minSamples    = 0;       // 每段窗口最少样本数
        int  countA        = 0;
        int  countB        = 0;
        bool haveB         = false;
        double meanA[3]    = {0, 0, 0};   // 窗口 A 的均值 (N)
        double meanB[3]    = {0, 0, 0};
        double poseA[3]    = {0, 0, 0};   // 窗口 A 的姿态 (deg, rx/ry/rz)
        double poseB[3]    = {0, 0, 0};
        double deadzoneN   = 0.0;         // 由调用方填 Config::FORCE_RESIDUAL_DEADZONE_N
    };

    struct Decision {
        Action action = Action::Waiting;
        double offset[3] = {0, 0, 0};     // 仅 Install 时非零
        double driftN = 0.0;              // |offset|
        double postureDeltaN = 0.0;       // |meanB − meanA|
        std::string text;
    };

    inline double norm3(const double v[3]) {
        return std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    }
    inline double norm3diff(const double a[3], const double b[3]) {
        double d[3] = {a[0]-b[0], a[1]-b[1], a[2]-b[2]};
        return norm3(d);
    }
    inline double num(double v) {
        char buf[32]; std::snprintf(buf, sizeof(buf), "%g", v); return std::string(buf);
    }

    inline Decision decide(const Input& in) {
        Decision d;
        d.text =
            std::string("残余(") + num(norm3(in.meanA)) + " N 于姿态A)";

        // 1) 先看"这一刻能不能采" —— 这些不是结论, 是【还不能说】
        if (!in.guardOk)     { d.action = Action::Waiting; d.text += " —— 闸门未放行, 本次不作结论"; return d; }
        if (!in.frameFresh)  { d.action = Action::Waiting; d.text += " —— 帧陈旧, 本次不作结论"; return d; }
        if (in.buttonHeld)   { d.action = Action::Waiting; d.text += " —— 正按着按钮(操作员在用力), 不作结论"; return d; }
        if (!in.poseStableA) { d.action = Action::Waiting; d.text += " —— 窗口A内机械臂在动, 本次不作结论"; return d; }
        if (in.countA < in.minSamples) { d.action = Action::Waiting; d.text += " —— 样本不足, 本次不作结论"; return d; }

        // 2) 残余已在死区以内 ⇒ 空载本来就安静, 什么都不用做
        //    ⚠ 用【每轴绝对值】比死区, 因为 softDeadzone 就是【逐轴】生效的。
        for (int i = 0; i < 3; i++) {
            if (std::fabs(in.meanA[i]) > in.deadzoneN) goto over_deadzone;
        }
        d.action = Action::Nothing;
        d.text += " —— 在死区内(≤" + num(in.deadzoneN) + " N/轴), 空载无感, 不需要调零";
        return d;

    over_deadzone:
        // 3) 需要第二段姿态才能证明"与姿态无关"
        if (!in.haveB)                 { d.action = Action::Waiting; d.text += " —— 残余超死区, 等第二个姿态的窗口"; return d; }
        if (in.countB < in.minSamples) { d.action = Action::Waiting; d.text += " —— 第二个窗口样本不足"; return d; }
        if (!in.poseStableB)           { d.action = Action::Waiting; d.text += " —— 窗口B内机械臂在动"; return d; }
        if (norm3diff(in.poseB, in.poseA) < MIN_POSE_CHANGE_DEG) {
            d.action = Action::Waiting;
            d.text += " —— 姿态还没变(需 ≥" + num(MIN_POSE_CHANGE_DEG) + "°), 证不了姿态无关";
            return d;
        }

        // 4) 两段窗口的残余是否一致?
        //    容差取【死区的一个比例】而不是某个绝对值: 零偏本身就在死区量级上下,
        //    要求两段一致到远小于死区是自欺(传感器分辨率与窗口长度都够不到)。
        d.postureDeltaN = norm3diff(in.meanB, in.meanA);
        const double tol = in.deadzoneN * POSTURE_TOL_FRACTION;
        if (d.postureDeltaN > tol) {
            d.action = Action::RefusePosture;
            d.text += " —— 但它在两个姿态下差 " + num(d.postureDeltaN) +
                      " N (容差 " + num(tol) + " N) ⇒ 【与姿态相关】, 不是零偏。"
                      "这通常是【负载/工具模型不对】(换装工具后)。⇒ 【不调零】; "
                      "按 's' 重新标定模型, 或核对工具质量与质心。";
            return d;
        }

        // 5) 超死区 + 与姿态无关 ⇒ 是零偏, 减掉它
        for (int i = 0; i < 3; i++) d.offset[i] = 0.5 * (in.meanA[i] + in.meanB[i]);
        d.driftN = norm3(d.offset);
        d.action = Action::Install;
        d.text += " —— 且两个姿态下一致(差 " + num(d.postureDeltaN) + " N ≤ " + num(tol) +
                  " N) ⇒ 是零偏。已装【会话零偏】(" + num(d.offset[0]) + "," + num(d.offset[1]) +
                  "," + num(d.offset[2]) + ") N ⇒ 空载手感应与关掉力反馈相同。"
                  "⚠ 只在本会话内存内生效, 【不落盘】。";
        return d;
    }

}  // namespace SessionZero
```

- [ ] **Step 4: 跑，确认五个用例全【绿】**

```bash
cd /d/Projects/Touch/Touch_Client/tests && cmd /c ".\build_force_compensation_test.bat" && ./test_force_compensation.exe
```

Expected: 五个 `session_zero_*` 全部 `PASS`，且无其他 FAIL。

- [ ] **Step 5: 负对照 NC-1 —— 把"按钮守卫"拿掉，`waits_while_button_held` 必须变红**

临时删除 `if (in.buttonHeld) { ... return d; }` 那两行 ⇒ 重建 ⇒ `test_session_zero_waits_while_button_held` **必须** FAIL。确认后**恢复**那两行并重建。

- [ ] **Step 6: 负对照 NC-2 —— 把"姿态相关性检查"拿掉，`refuses_when_posture_dependent` 必须变红**

临时把 `if (d.postureDeltaN > tol) { ... }` 那一段注释掉（或把 `tol` 改成 `1e9`）⇒ 重建 ⇒ `test_session_zero_refuses_when_posture_dependent` **必须** FAIL（它会变成 `Install`，于是 `CHECK(d.action == RefusePosture)` 红）。确认后**恢复**并重建。**这一条是本计划的安全底线，两步都不能省。**

- [ ] **Step 7: 全套件回归**

```bash
cd /d/Projects/Touch/Touch_Client/tests && cmd /c ".\run_tests.bat"; echo "exit=$?"
```

Expected: `exit=0`。

- [ ] **Step 8: 提交**

```bash
cd /d/Projects/Touch && git add Touch_Client/force/SessionZero.h Touch_Client/tests/test_force_compensation.cpp && git commit -F - <<'MSG'
feat(force): SessionZero 判定纯函数 —— 残余超死区且【与姿态无关】才允许当零偏减

现场判据: 空载手上不该有力(残余 0.3187 N 越过死区 0.20 后【原样放行】⇒ 手上 ≈0.63 N,
且它就是闭环自激的激励源)。残余与写字横向信号同量级 ⇒ 固定幅值整形分不开,
只能按时间结构把慢的那一份减掉。

★ 安全底线 = 【能拒绝】: 两段姿态下残余不一致 ⇒ 它是姿态相关的(换装工具后模型不对)
  ⇒ 不许减, 要求去重标。另加"按着按钮不结论"守卫(别把操作员手劲学走)。
纯函数 + 5 个用例; 两个负对照(NC-1 去按钮守卫 / NC-2 去姿态检查)均已实测变红。
MSG
```

---

### Task 4: 会话零偏的**应用**（接缝 + 值断言）

**Files:**
- Modify: `Touch_Client/force/ForcePipeline.h`（声明 + 内联访问器）
- Modify: `Touch_Client/force/ForcePipeline.cpp`（在 `step()` 的**最前面**应用）
- Test: `Touch_Client/tests/test_force_pipeline.cpp`

**Interfaces:**
- Produces:
  - `void ForcePipeline::installSessionZeroOffset(const double off[3]);`（内存内，**不落盘**）
  - `void ForcePipeline::clearSessionZeroOffset();`
  - `void ForcePipeline::sessionZeroOffset(double off[3]);`
  - `double ForcePipeline::residualNorm(const double filtered6[6]);`（Task 2 的打印用；**必须**在 `ForcePipeline.cpp` 里定义，只算前三维的模）
- Consumes: 无

**为什么插在 `step()` 最前面（对 `fd.compensated`）而不是 `filtered` 之后**：
1. `ForceCompensation::step()` 的闸门 EMA **已经**在它内部算完了 ⇒ 现在改 `compensated` **不会**影响闸门看到的模型（闸门必须继续看到"未修正的模型"，否则它会失去发现模型错误的能力）；
2. 放在滤波与梯度限幅**之前** ⇒ 装/清零偏那一瞬间的跳变会被 Butterworth 与 `FORCE_GRADIENT_LIMIT` 抹平，操作员不会感到一次"咯噔"。

- [ ] **Step 1: 先写会红的值断言**

在 `Touch_Client/tests/test_force_pipeline.cpp` 追加（⚠ 会话零偏是**全局**的 ⇒ 照该文件 `GainScope` 的先例写一个 RAII，中途 CHECK 失败也要清掉）：

```cpp
// ===== 会话零偏: 装了之后空载应当【没有输出】=====
// 判据来自现场: "正确的手感 = 关掉力反馈时的感觉"(空载手上没有力)。
namespace { struct SessionZeroScope {
    SessionZeroScope()  { ForcePipeline::clearSessionZeroOffset(); }
    ~SessionZeroScope() { ForcePipeline::clearSessionZeroOffset(); }
}; }

static void test_session_zero_offset_kills_residual() {
    TEST(session_zero_offset_kills_residual);
    SessionZeroScope scope;
    ForcePipeline::init();
    ForceTuning::setGain(120.0);

    // 先量: 残余 0.30 N 在 X 上(死区 0.20 以上) ⇒ 未调零时手上【有】力
    AppState::ForceData a;
    a.isStale = false;
    a.compensated[0] = 0.30; a.compensated[1] = 0.0; a.compensated[2] = 0.0;
    for (int i = 0; i < 400; i++) ForcePipeline::step(a);   // 让滤波器/限幅走到稳态
    const double before = a.hapticOut[0];
    CHECK(fabs(before) > 0.1);      // 前提: 未调零时确实是可感的(否则本用例证明不了什么)

    // 装零偏 = 那个残余 ⇒ 输出应当回到 ~0
    double off[3] = {0.30, 0.0, 0.0};
    ForcePipeline::installSessionZeroOffset(off);
    for (int i = 0; i < 400; i++) ForcePipeline::step(a);
    CHECK(fabs(a.hapticOut[0]) < 0.02);
    CHECK(fabs(a.hapticOut[1]) < 0.02);
    CHECK(fabs(a.hapticOut[2]) < 0.02);

    // 清掉 ⇒ 又回到有力(before) —— 证明上面那句是零偏的功劳, 不是别的原因
    ForcePipeline::clearSessionZeroOffset();
    for (int i = 0; i < 400; i++) ForcePipeline::step(a);
    CHECK(fabs(a.hapticOut[0] - before) < 0.05);
    PASS();
}

static void test_session_zero_offset_does_not_touch_gate_input() {
    TEST(session_zero_offset_does_not_touch_gate_input);
    SessionZeroScope scope;
    ForcePipeline::init();
    AppState::ForceData a;
    a.isStale = false;
    a.compensated[0] = 0.30;
    double off[3] = {0.30, 0.0, 0.0};
    ForcePipeline::installSessionZeroOffset(off);
    ForcePipeline::step(a);
    // ★ 闸门看到的必须是【未修正】的 compensated: 本函数只许动 filtered 的【产生过程】,
    //   不许回写 compensated。(写回 compensated 会让闸门失去发现模型错误的能力。)
    CHECK(fabs(a.compensated[0] - 0.30) < 1e-9);
    PASS();
}
```

在 `main()` 里加两行：

```cpp
    test_session_zero_offset_kills_residual();
    test_session_zero_offset_does_not_touch_gate_input();
```

- [ ] **Step 2: 跑，确认是【红】**

```bash
cd /d/Projects/Touch/Touch_Client/tests && cmd /c ".\build_force_pipeline_test.bat"
```

Expected: **编不过**（`installSessionZeroOffset` 未声明、`SessionZeroScope` 用到未定义符号）。

- [ ] **Step 3: 加声明与访问器**

`Touch_Client/force/ForcePipeline.h`（放在 `ForceTuning` 那一段函数声明附近）：

```cpp
    // ===== 会话零偏 (2026-09-24) =====
    // 把启动时测得的【慢速残余】减掉 —— 判据见 force/SessionZero.h。
    // ⚠ 只在本会话内存内生效, 【绝不落盘】: force_calib.json 描述的是"重启之后的稳态",
    //   把一次可能受污染的零偏写回去, 会固化成下一场的基准。
    // ⚠ 应用点在 ForcePipeline::step() 的【最前面】(改 fd.compensated), 于是:
    //   · 闸门看不到它(ForceCompensation::step 的 EMA 在更早就算完了 ⇒ 闸门仍看未修正的模型);
    //   · 装/清那一瞬间的跳变被滤波与梯度限幅抹平(不会有"咯噔")。
    void installSessionZeroOffset(const double off[3]);
    void clearSessionZeroOffset();
    void sessionZeroOffset(double off[3]);          // 回读(诊断/打印用)
    double residualNorm(const double filtered6[6]); // 前三维的模(诊断打印用)
```

- [ ] **Step 4: 实现（含在 `step()` 最前面应用）**

`Touch_Client/force/ForcePipeline.cpp`：在匿名 namespace 里加状态，在 `step()` 开头加应用，并在文件里定义四个函数：

```cpp
// (文件顶部已有 namespace { ... } 的静态状态区, 在那一处追加)
static double g_sessionZero[3] = {0.0, 0.0, 0.0};   // 会话零偏 (N), 内存内, 不落盘
```

```cpp
void step(AppState::ForceData& fd) {
    // ===== 0. 会话零偏 (2026-09-24) =====
    //   把启动时测得的慢速残余从 compensated 里减掉。
    //   ⚠ 必须在【滤波与梯度限幅之前】: 装/清那一瞬间的跳变会被它们抹平。
    //   ⚠ 闸门不受影响: ForceCompensation::step 的 EMA 在本函数【之前】就算完了,
    //     它看到的是未减零偏的 compensated —— 那是刻意的(闸门要继续能发现模型错误)。
    if (g_sessionZero[0] != 0.0 || g_sessionZero[1] != 0.0 || g_sessionZero[2] != 0.0) {
        for (int i = 0; i < 3; i++) fd.compensated[i] -= g_sessionZero[i];
        // ⚠ 力矩不碰: 本零偏量的是【力】的均值(见 SessionZero::Input.meanA/B), 力矩没有依据。
    }

    // 1. Butterworth filter
    for (int i = 0; i < 6; i++) {
        fd.filtered[i] = g_filters[i].step(fd.compensated[i]);
    }
    // ... 以下原样不动 ...
```

```cpp
// (文件末尾, 与其它命名空间函数并列)
void installSessionZeroOffset(const double off[3]) {
    for (int i = 0; i < 3; i++) g_sessionZero[i] = off[i];
}
void clearSessionZeroOffset() {
    for (int i = 0; i < 3; i++) g_sessionZero[i] = 0.0;
}
void sessionZeroOffset(double off[3]) {
    for (int i = 0; i < 3; i++) off[i] = g_sessionZero[i];
}
double residualNorm(const double filtered6[6]) {
    return std::sqrt(filtered6[0]*filtered6[0] + filtered6[1]*filtered6[1] + filtered6[2]*filtered6[2]);
}
```

⚠ 若 `ForcePipeline.cpp` 没有包含 `<cmath>`，在文件顶部补 `#include <cmath>`。

- [ ] **Step 5: 跑，确认变【绿】**

```bash
cd /d/Projects/Touch/Touch_Client/tests && cmd /c ".\build_force_pipeline_test.bat" && ./test_force_pipeline.exe
```

Expected: 两个新用例 `PASS`，无其他 FAIL。

- [ ] **Step 6: 负对照 NC-3 —— 把应用那一行删掉，`kills_residual` 必须变红**

临时注释掉 `step()` 里那个 `for (...) fd.compensated[i] -= g_sessionZero[i];` ⇒ 重建 ⇒ `test_session_zero_offset_kills_residual` **必须**在 `CHECK(fabs(a.hapticOut[0]) < 0.02)` 处 FAIL。确认后恢复并重建。

- [ ] **Step 7: 提交**

```bash
cd /d/Projects/Touch && git add Touch_Client/force/ForcePipeline.h Touch_Client/force/ForcePipeline.cpp Touch_Client/tests/test_force_pipeline.cpp && git commit -F - <<'MSG'
feat(force): 会话零偏的应用面 —— 只改 step() 的输入, 不动闸门看到的模型

应用点在 step() 最前面(对 compensated), 理由两条:
 · 闸门 EMA 在 ForceCompensation::step 里已算完 ⇒ 闸门继续看到未修正的模型
   (它必须保有发现模型错误的能力); 用例 test_..._does_not_touch_gate_input 钉住这句。
 · 在滤波/梯度限幅之前 ⇒ 装/清那一瞬间的跳变被抹平, 不会有"咯噔"。
内存内, 不落盘。负对照 NC-3(删应用行⇒用例变红)已实测。
MSG
```

---

### Task 5: 接线（采样状态机 + 上机验收）

**Files:**
- Modify: `Touch_Client/main.cpp`
- Modify: `Docs/superpowers/specs/2026-09-24-on-machine-run-sheet.md`

**Interfaces:**
- Consumes: `SessionZero::decide()`（Task 3）、`ForcePipeline::installSessionZeroOffset()`（Task 4）、`ForceCompensation::guardState()`、`appState.lastButtonState`、`appState.robotActualPose`
- Produces: 控制台行，前缀固定为 `[SessionZero]`

**⚠ 前置**：Task 2 的 `[ZeroTrace]` 数据必须已经拿到（残留在 session 内是常数还是会长大）。若"会长大" ⇒ 本 Task 落地后**紧接着做 Task 6**。

- [ ] **Step 1: 写状态机**

在 `Touch_Client/main.cpp` 里 `runResidualTrace()` 的**下面**追加。设计要点：**采样窗口只在「未按按钮 + 闸门放行 + 帧新鲜 + 姿态稳定」时积累**；窗口 A 攒够后**等一次 ≥5° 的姿态变化**再采窗口 B；`decide()` 一旦给出终局结论就 `done`（`Waiting` 不算结论，继续等）。

```cpp
// ===== 会话零偏: 启动采样状态机 (2026-09-24) =====
// 判据与全部逻辑在 force/SessionZero.h 的纯函数里; 这里只负责【采样、时钟、打印、应用】。
// 判据(现场): 空载手感应与关掉力反馈相同(没有力); 按压/写字仍要有力。
// ⚠ 本状态机【不】做连续自适应: 只在未按按钮 + 静止的窗口里采, 且只采两段。
//    连续自适应会把操作员长匀速笔画的手劲当零偏学走(阻力几秒内消失)。
static bool   g_szDone = false;
static int    g_szStage = 0;            // 0=采A 1=等姿态变化 2=采B 3=已定稿
static DWORD  g_szStageStartMs = 0;
static double g_szAccumA[3] = {0,0,0}, g_szAccumB[3] = {0,0,0};
static int    g_szCountA = 0, g_szCountB = 0;
static double g_szPoseA[3] = {0,0,0};   // 窗口A的姿态(取第一帧)
static double g_szPoseB[3] = {0,0,0};

static void runSessionZero() {
    if (g_szDone) return;

    const bool guardOk  = (ForceCompensation::guardState() == ForceCompensation::GuardState::OK);
    const bool button   = appState.lastButtonState;

    AppState::ForceData fd;
    EnterCriticalSection(&appState.forceDataMutex);
    fd = appState.forceData;
    LeaveCriticalSection(&appState.forceDataMutex);

    double pose[3];
    EnterCriticalSection(&appState.robotPoseMutex);
    pose[0] = appState.robotActualPose.rx;
    pose[1] = appState.robotActualPose.ry;
    pose[2] = appState.robotActualPose.rz;
    LeaveCriticalSection(&appState.robotPoseMutex);

    // 「能不能采」= 未按按钮 + 闸门放行 + 帧新鲜。姿态"稳不稳"由下面的窗口起点定死:
    // 一旦窗口内姿态偏离起点超过 POSE_STABLE_DEG, 该窗口作废重开(它已经不是"同一个姿态")。
    const bool canSample = guardOk && !fd.isStale && !button;

    const DWORD now = GetTickCount();
    if (g_szStageStartMs == 0) { g_szStageStartMs = now; }

    // 窗口波动检查
    auto poseSpreadOk = [&](const double ref[3]) {
        for (int i = 0; i < 3; i++) if (fabs(pose[i] - ref[i]) > SessionZero::POSE_STABLE_DEG) return false;
        return true;
    };
    auto resetWindow = [&](int stage) {
        g_szStage = stage; g_szStageStartMs = now;
        g_szCountA = 0; g_szCountB = 0;
        for (int i = 0; i < 3; i++) { g_szAccumA[i] = 0; g_szAccumB[i] = 0; }
        for (int i = 0; i < 3; i++) { g_szPoseA[i] = pose[i]; g_szPoseB[i] = pose[i]; }
    };

    if (g_szStage == 0) {
        if (!canSample || !poseSpreadOk(g_szPoseA)) { resetWindow(0); return; }
        for (int i = 0; i < 3; i++) g_szAccumA[i] += fd.filtered[i];
        g_szCountA++;
        // 窗口长度沿用启动检查的口径: 2s 稳定 + 1s 采样 (见 runZeroDriftCheck 的注释)
        if (g_szCountA >= Config::FORCE_ZERO_DRIFT_MIN_SAMPLES) g_szStage = 1;
        return;
    }

    if (g_szStage == 1) {
        // 等机械臂真的换个姿态 —— 操作员马上就会动, 所以这不需要他额外做事。
        if (SessionZero::norm3diff(pose, g_szPoseA) >= SessionZero::MIN_POSE_CHANGE_DEG) resetWindow(2);
        return;
    }

    if (g_szStage == 2) {
        if (!canSample || !poseSpreadOk(g_szPoseB)) { resetWindow(2); return; }
        for (int i = 0; i < 3; i++) g_szAccumB[i] += fd.filtered[i];
        g_szCountB++;
        if (g_szCountB < Config::FORCE_ZERO_DRIFT_MIN_SAMPLES) return;

        SessionZero::Input in;
        in.guardOk = guardOk; in.frameFresh = !fd.isStale; in.buttonHeld = button;
        in.poseStableA = true; in.poseStableB = true;   // 窗口内波动已在上面把关
        in.minSamples = Config::FORCE_ZERO_DRIFT_MIN_SAMPLES;
        in.countA = g_szCountA; in.countB = g_szCountB; in.haveB = true;
        for (int i = 0; i < 3; i++) {
            in.meanA[i] = g_szAccumA[i] / g_szCountA;
            in.meanB[i] = g_szAccumB[i] / g_szCountB;
            in.poseA[i] = g_szPoseA[i];
            in.poseB[i] = g_szPoseB[i];
        }
        in.deadzoneN = Config::FORCE_RESIDUAL_DEADZONE_N;
        const SessionZero::Decision d = SessionZero::decide(in);

        if (d.action == SessionZero::Action::Waiting) return;   // 不是结论, 继续等
        if (d.action == SessionZero::Action::Install) {
            ForcePipeline::installSessionZeroOffset(d.offset);
        }
        // Nothing / RefusePosture ⇒ 不动任何力, 只说话
        g_szDone = true;
        std::cout << "[SessionZero] " << d.text << std::endl;
        return;
    }
}
```

- [ ] **Step 2: 调用它**

在 Task 2 加的 `runResidualTrace();` 旁边加：

```cpp
        runSessionZero();
```

- [ ] **Step 3: 构建 + 全套件**

```bash
cd /d/Projects/Touch/Touch_Client/tests && cmd /c ".\run_tests.bat"; echo "exit=$?" && cd /d/Projects/Touch/Touch_Client && cmd /c ".\build.bat"
```

Expected: `exit=0`；客户端链接成功（**若客户端在跑会报 LNK1168 ⇒ 按 PID 优雅关掉再建**，用户已授权）。

- [ ] **Step 4: 上机验收（**按用户给的判据**，两条都要）**

在 `Docs/superpowers/specs/2026-09-24-on-machine-run-sheet.md` 追加一段并在现场逐条判：

1. 启动后**不动**，等控制台出 `[SessionZero]`（应当 ~5 s 内，因为状态机在第 1 步等你动一下）。
2. **空载自由运动**（不碰纸、不压笔）：手感**应当与关掉 Force Feedback 开关时相同**（即**没有力**），机械臂**不再抖**。→ 判「对/不对」。
3. **按压/写字**：仍要有力量感。→ 判「对/不对」。
4. 抄下 `[SessionZero]` 那一行原文（它写着装了多大的零偏、或为什么没装）。

判据先写死（防事后圆说）：**2 与 3 都对** ⇒ Task 5 通过；**2 对 3 不对** ⇒ 会话零偏把写字信号也削了 ⇒ 回滚本 Task 并回到计划层面（残余与信号同量级的墙没绕过去）；**2 不对** ⇒ 残余不是零偏（回到 Task 3 的 `RefusePosture` 分支，去查工具/负载模型）。

- [ ] **Step 5: 提交**

```bash
cd /d/Projects/Touch && git add Touch_Client/main.cpp Docs/superpowers/specs/2026-09-24-on-machine-run-sheet.md && git commit -F - <<'MSG'
feat(force): 启动会话零偏接线 + 上机验收单

采样: 只在【未按按钮 + 闸门放行 + 帧新鲜 + 姿态稳定】的窗口里采;
      窗口A攒够后等一次 ≥5° 的姿态变化采窗口B(操作员马上就会动, 不需额外操作);
      判定交纯函数, 只有 Install 才装零偏, RefusePosture/Nothing 只说话。
验收(用户给的判据): ① 空载手感应与关掉力反馈相同 ② 按压/写字仍要有力。
MSG
```

---

### Task 6（**条件项**：仅当 Task 2 的 `[ZeroTrace]` 显示残余在一场会话里会长大时才做）

**Files:**
- Modify: `Touch_Client/main.cpp`（把状态机的 `g_szDone` 终局条件放宽成"可再试一次"）
- Test: `Touch_Client/tests/test_force_compensation.cpp`（加一条守卫用例）

**Interfaces:**
- Consumes: Task 5 的状态机
- Produces: 无新接口（只放宽一次机会）

- [ ] **Step 1: 先写会红的新用例（守卫: 两次之间必须有最小间隔，且只在未按按钮时重开）**

```cpp
static void test_session_zero_second_chance_rate_limited() {
    TEST(session_zero_second_chance_rate_limited);
    // 第二次机会的守卫: 距上次安装不足 RETRY_MIN_INTERVAL_MS ⇒ Waiting(不许采)
    SessionZero::RetryInput r;
    r.sinceLastInstallMs = 1000;
    r.buttonHeld = false; r.poseStable = true;
    CHECK(SessionZero::retryAllowed(r) == false);
    r.sinceLastInstallMs = SessionZero::RETRY_MIN_INTERVAL_MS + 1;
    CHECK(SessionZero::retryAllowed(r) == true);      // 够久 ⇒ 允许
    r.buttonHeld = true;
    CHECK(SessionZero::retryAllowed(r) == false);     // ★ 按着按钮一律不许 —— 手劲不能被学走
    PASS();
}
```

在 `main()` 里加一行 `test_session_zero_second_chance_rate_limited();`。

- [ ] **Step 2: 跑，确认是【红】**（`RetryInput` / `retryAllowed` / `RETRY_MIN_INTERVAL_MS` 未定义 ⇒ 编不过）

```bash
cd /d/Projects/Touch/Touch_Client/tests && cmd /c ".\build_force_compensation_test.bat"
```

- [ ] **Step 3: 在 `SessionZero.h` 里加实现**

```cpp
    // 第二次机会的最小间隔(ms)。取 10 分钟: 长于任何一段连续书写, 短于一场会话。
    // ⚠ 只在【未按按钮 + 姿态稳定】时才允许重开 —— 这是"别把操作员手劲学走"的那条守卫。
    constexpr unsigned long RETRY_MIN_INTERVAL_MS = 600000;

    struct RetryInput {
        unsigned long sinceLastInstallMs = 0;
        bool buttonHeld = false;
        bool poseStable = false;
    };
    inline bool retryAllowed(const RetryInput& r) {
        if (r.buttonHeld)  return false;
        if (!r.poseStable) return false;
        return r.sinceLastInstallMs >= RETRY_MIN_INTERVAL_MS;
    }
```

- [ ] **Step 4: 跑，确认变【绿】**

```bash
cd /d/Projects/Touch/Touch_Client/tests && cmd /c ".\build_force_compensation_test.bat" && ./test_force_compensation.exe
```

- [ ] **Step 5: 在 `main.cpp` 里接上**（`g_szDone` 改成记录安装时刻 `g_szInstalledMs`；`runSessionZero()` 开头改成：若已装过，则仅当 `SessionZero::retryAllowed({now - g_szInstalledMs, button, poseStable})` 为真才重置状态机重采一次，且**每场会话最多再试一次**（加 `g_szRetryUsed`）。

- [ ] **Step 6: 负对照 NC-4** —— 把 `retryAllowed` 里的 `if (r.buttonHeld) return false;` 拿掉 ⇒ `second_chance_rate_limited` 必须变红。恢复。

- [ ] **Step 7: 全套件 + 提交**

```bash
cd /d/Projects/Touch/Touch_Client/tests && cmd /c ".\run_tests.bat"; echo "exit=$?"
cd /d/Projects/Touch && git add Touch_Client/force/SessionZero.h Touch_Client/main.cpp Touch_Client/tests/test_force_compensation.cpp && git commit -F - <<'MSG'
feat(force): 会话零偏的第二次机会(限频 + 无按键守卫) —— 仅当残余在一场会话内会长大时启用

[ZeroTrace] 实测残余随时间增大才做本 Task。守卫: 距上次安装 ≥10 分钟、且【未按按钮】、
姿态稳定 —— 按着按钮时一律不许重采(别把操作员长匀速笔画的手劲学成零偏)。
负对照 NC-4(去按键守卫⇒用例变红)已实测。
MSG
```

---

## Self-Review

**1. 覆盖检查（每条 Global Constraint 指到哪个 Task）**

| 约束 | 落地处 |
|---|---|
| 判据（空载无感 / 按压有感） | Task 5 Step 4 的两条现场判定 + Task 4 的值断言 |
| 不碰环路 / 不改死区形状 / 不改增益 | 全计划无一处触及（只新增 4 个函数与 1 个常数） |
| 会话零偏不落盘 | Task 4 的接口注释 + 实现里没有 `saveToFile` 调用 |
| 闸门看未修正的模型 | Task 4 Step 1 的 `does_not_touch_gate_input` 用例 |
| 常量一处定义 | Task 1 的关系式断言 + Task 3 的常量放 `SessionZero.h`（照 `ForceTuning.h` 先例） |
| 每个新用例有负对照 | Task 1 Step 5、Task 3 Step 5/6、Task 4 Step 6、Task 6 Step 6 |
| 一行可回滚 | 每个 Task 一次提交，且都只新增（无删除性改动） |

**2. 占位符扫描**：无 TBD/TODO；每个 code step 都给了完整代码；命令都带预期输出。**唯一一处需要现场核名的地方**在 Task 3 之前的注释里说明了：`Config::FORCE_ZERO_DRIFT_MIN_SAMPLES` 已在本仓 `main.cpp` 的 `runZeroDriftCheck` 里被引用（可直接用）；若实现者要另找运动阈值常量，**先 grep 再写，不要另造字面量**。

**3. 类型/命名一致性**：`SessionZero::Action` 的四个枚举值在 Task 3（定义）、Task 5（消费）、Task 6（扩展）三处拼写一致；`installSessionZeroOffset` / `clearSessionZeroOffset` / `sessionZeroOffset` / `residualNorm` 在 Task 2（消费 `sessionZeroOffset`、`residualNorm`）与 Task 4（定义）处一致；`Input.meanA/meanB/poseA/poseB/countA/countB/haveB/deadzoneN/minSamples` 在 Task 3 与 Task 5 的填写处逐字段一致。

**4. 已知未覆盖面（刻意不做，写清楚以免被读成遗漏）**
- **不做**连续在线估计器（风险见 Global Constraints 的架构段）。
- **不动** ③（重力项姿态滞后 1.55 N）—— 那是另一条未结线，本计划不声称解决它。
- **不改** 环路增益。若本次修完、写字时那 0.35 N 的真实信号仍让机械臂自激，那是**另一件事**（真实信号的放大），不要指望本计划一并解决。

---

## Execution Handoff

**Plan complete and saved to `Docs/superpowers/plans/2026-09-24-session-zero-blind-band.md`. Two execution options:**

1. **Subagent-Driven (recommended)** —— 每个 Task 派一个干净的子代理，任务之间我来审查，迭代快（见 `prefer-subagent-execution`）。
2. **Inline Execution** —— 在本会话里按 `superpowers:executing-plans` 批量执行，带检查点。

**但请注意执行顺序上的一个硬约束**：**Task 2 是阻塞项**（它决定 Task 6 做不做）。Task 1 / 3 / 4 可以先做（纯函数与接缝，无行为改动）；**Task 5 的接线在 `[ZeroTrace]` 数据到手前不要落地**。
