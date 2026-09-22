# 离线修复 Implementation Plan（2026-09-22）

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 清掉 2026-09-21 收工时留下的、不依赖机械臂的那批缺陷 —— 其中一处是**活的**（调零的时长被算错 2.7 倍），并把"测试到底验了什么"这件事本身修到可信。

**Architecture:** 全部是局部修法，不动任何模块边界。Task 1 把 `ForceCalibration` 的时间步从"调用方传常数"改成"用实测间隔自算"（照抄 `ForceCompensation::stepIntervalSec` 已经验证过的做法）；Task 2/3 修两条不诚实的用例（一条红是测试隔离、一条绿是恒真）；Task 4 让 `test_safety_core` 真的能被构建出来，从而第一次拿到它的真实结果；Task 5 把死代码结论与被推翻的那一条落成注释。

**Tech Stack:** C++17 / MSVC 2022 BuildTools / 手写 .bat 构建脚本 / OpenHaptics SDK（仅头文件路径）。

## Global Constraints

- **分支**：`feat/pen-clamp-redesign`（HEAD `ea747ab`，已推送、**未合并**）。在本分支上继续，**不要**合并到 `master`、不要开 PR —— 用户 2026-09-21 明确"先不提交"之外的决定另说。
- **禁止**在 `pollForce()` 里再调 `ForceCompensation::step` / `ForcePipeline::step` —— 那两个现在跑在 `forceReaderThread` 的帧率（123 Hz）上，每帧只能推一次（`RelayCore.cpp:2101-2105` 有警告）。
- **不许**把"按假定采样率换算"的常数再引入任何一处。本项目已因此踩过三次（`FORCE_GUARD_EMA_ALPHA`、`FORCE_BIAS_EMA_TAU_S`、本计划 Task 1）。按时间换算的东西一律用**实测耗时**，或按**秒**定义时间常数。
- **不许**为了让断言好看去改容差 / 门限。`FORCE_GUARD_TOL_FORCE_N = 1.2464`、`FORCE_RESIDUAL_DEADZONE_N = 0.20`、`FORCE_FILTER_CUTOFF = 5`、`FORCE_FILTER_FS_HZ = 125` 都不动。
- **跑测试不是只读操作**：`test_force_compensation` / `test_payload_calibration` 会按**工作目录**写标定状态（实测：仓库根 `calib/force_calib.json` 被测试运行改写）。跑之前确认没有正在运行的 `Touch_Client.exe`。
- 提交信息用中文、`fix(force):` / `test(...)` 风格，与本仓近期提交一致。**每个 Task 结束提交一次**。

## 开工前的事实地基（2026-09-22 实测，HEAD `ea747ab`，工作树干净）

跑的是 `Touch_Client/tests/run_tests.bat`（全量），加单独跑的 `test_payload_calibration`：

| 套件 | run_tests 会重建吗 | 结果 | 可信吗 |
|---|---|---|---|
| test_force_pipeline | ❌ 只跑不建 | 7 / 0 | exe 与 cpp 同一分钟（09-21 22:32）⇒ 大致新 |
| test_constraint_force | ❌ 只跑不建 | 7 / 0 | exe 09-21 20:02 > cpp 07-25 ⇒ 存疑 |
| **test_safety_core** | ❌ 只跑不建 | **7 / 1** ⇒ 重建后 **8 / 0** | **不可信：exe 是 07-25 的；且当时【构建不出来】。★ 重建后 8/0 ⇒ 那条红是【陈旧二进制造的假红】** |
| test_feedback_parser | ❌ 只跑不建 | 28 / 0 | exe 09-21 14:34 > cpp 07-25 ⇒ 大致新 |
| test_escalation | ❌ 只跑不建 | 15 / 0 | ⚠ **exe 07-25**（Task 4 复核确认） |
| test_kinematics | ❌ 只跑不建 | 18 / 0 | exe 07-25（与 cpp 同分钟） |
| test_coord_safety | ❌ 只跑不建 | 27 / 0 | exe 09-21 22:36 > cpp ⇒ 大致新 |
| **test_force_compensation** | ✅ 重建 | **38 / 1** | **可信：这条红是真的** |
| test_relay_command_parser | ✅ 重建 | 11 / 0 | 可信 |
| test_force_logger | ✅ 重建 | 7 / 0 | 可信 |
| test_tcp_calibration | ✅ 重建 | 7 / 0 | 可信 |
| test_session_report | ✅ 重建 | 20 / 0 | 可信 |
| test_noise_probe | ✅ 重建，但**构建失败** | **没跑** | ★ Task 4 查出根因：同一 cmd 会话里连续 `call vcvarsall` 会让 **PATH 累积超过 cmd 的 8191 上限**（实测连调 5 次：前 4 次 OK、第 5 次失败）⇒ **全量 run_tests.bat 里它从来没跑成过**；单独跑是好的，所以这个洞一直没被发现。修法与"是旧账"的 A/B 见 Task 4 |
| test_payload_calibration | 不在 run_tests.bat 里 | 74 / 1 | 那 1 条是**刻意留红**（等重采带 `@720` 的夹具），符合记忆 |

**⇒ 两条结论，后面每个 Task 都据此**
1. **唯一的"真红"是 `test_force_compensation` 的 `38 / 1`** —— 就是 Task 2 要修的那条。
2. `test_safety_core` 的 `7 / 1` **不是关于当前代码的证据**（跑的是一个七月二进制）。它的构建当时**是断的**：`RobotDiagnostics.cpp:108` 调 `RelayCore::instance().reportDiagnostic(...)`，而测试的构建配方不带 `RelayCore` ⇒ `LNK2019: 无法解析的外部符号 ... RelayCore::instance / reportDiagnostic`。
   **★ Task 4 已给出答案：重建后是 `8 passed, 0 failed` ⇒ 那条红是【陈旧二进制造的假红】，不是真缺陷。**
   ⇒ **开工前那份基线里所谓的"两个失败"，有一个是虚构的。**

---

### Task 1: `ForceCalibration` 的 dt 改用实测间隔

**为什么**：`RelayCore.cpp:2095` 给 `ForceCalibration::update()` 传的是**常数 `0.033`**（= 名义节拍 `FORCE_POLL_INTERVAL_MS = 33`，而那个常数的注释自己写着它是"处理节流**下限**"），可它实际被调用的节拍是 `pollForce` 的真实节拍 —— 实测 **46~203 ms、均值 92 ms（~11 Hz）**。于是 TARE 那段"静默 0.5 s + 累计 2 s"在**实际时间里是 ~1.35 s + 5.4 s**，而控制台提示打印的是 `2.5s`。
**⇒ 操作员按提示等 2.5 秒就松手/动臂 ⇒ 又采到瞬态** —— 正是 2026-09-21 加这段静默期要治的那件事。这是"按假定采样率换算"这个 bug 家族的**第三次**出现。

**设计选择（以及被否掉的那个）**：把测量**放进** `ForceCalibration`（照抄 `ForceCompensation::stepIntervalSec`，`ForceCompensation.cpp:961-974`），**删掉 `dt` 参数**。
- 否掉"让 RelayCore 量好再传进来"：它保留了"调用方可以传错一个 dt"这个洞，而本项目反复吃这个洞的亏。删掉参数 ⇒ **没有任何地方能传错**。与既有做法一致（`ForceCompensation` 就是这么做的）。
- 否掉"给窗口做平滑"：TARE 是**累加**计时器，抖动自会平均掉；而 MOTION/SOLVE 那条路已注明只剩诊断价值（`ForceCalibration.cpp:371-377`）。YAGNI。

**Files:**
- Modify: `Touch_Client/force/ForceCalibration.h:44-45`（签名 + 新增测试钩子）
- Modify: `Touch_Client/force/ForceCalibration.cpp`（新增 `updateIntervalSec()` / `setUpdateDtForTest()`；`update()` 改签名并取 dt）
- Modify: `Touch_Client/relay/RelayCore.cpp:2094-2099`（调用点）
- Modify: `Touch_Client/tests/test_force_compensation.cpp:1160, 1217, 1255, 1283`（4 个调用点）
- Test: `Touch_Client/tests/test_force_compensation.cpp`（新增一条钉住"实测 dt 被用上"的用例）

**Interfaces:**
- Produces: `bool ForceCalibration::update(const double raw[6], const double pose[6]);`（**`dt` 参数删除**）
- Produces: `void ForceCalibration::setUpdateDtForTest(double sec);`（默认 `-1.0` = 走实测；用例钉死时间的唯一入口，与 `ForceCompensation::setStepDtForTest` 同形）
- Consumes: `Config::FORCE_CALIB_SETTLE_TIME_S = 0.5`、`Config::FORCE_CALIB_STILL_COLLECT_S = 2.0`（`Config.h:280,282`，不改）

- [ ] **Step 1: 写失败用例 —— 钉住"dt 来自实测间隔，而不是常数"**

在 `test_force_compensation.cpp` 里，`test_zero_restartable` 之后新增：

```cpp
// ★ 2026-09-22: 钉住"dt 是实测/用例给定的，不是常数"。
// 【为什么需要这条】从前 RelayCore 传常数 0.033（名义节拍），而真实节拍是 46~203ms
//   ⇒ 静默期 0.5s 在实际时间里是 ~1.35s。这条用例把时间变成【确定的输入】，
//   于是"采够 2.5s 才定稿"这件事可以被验 —— 从前不可能验，因为没人能驱动时间。
static void test_calib_tare_timing_follows_measured_dt() {
    TEST(calib_tare_timing_follows_measured_dt);
    ForceCompensation::init();
    double A[9]; diagA(0.42, A);
    double cS[3] = {0, 0, 0.03};
    double bF[3] = {0, 0, 0}, bM[3] = {0, 0, 0};
    ForceCompensation::setCalibration(A, bF, bM, cS);

    double raw[6] = {-0.48, -1.35, -0.02, 0.010, -0.020, 0.005};
    double pose[6] = {0, 0, 0, 0, 0, 0};
    const double dt = 0.033;                 // ★ 故意用那个【错】的常数当步长

    CHECK(ForceCalibration::startZero());
    ForceCalibration::setUpdateDtForTest(dt);

    // ★ 钉住【边界】而不是"大概够了"。
    //   路径 (update() 的 TARE 分支): 每步 g_phaseTimer += dt; 若 < 0.5 直接早退(不累计);
    //   越过 0.5 之后才累计, 并在同一步里判 >= 0.5+2.0 == 2.5 就定稿。
    //   dt = 0.033 ⇒ 计时器 = 0.033n:
    //     n = 75 ⇒ 2.475  < 2.5 ⇒ 【必须还没 DONE】
    //     n = 76 ⇒ 2.508 >= 2.5 ⇒ 【必须 DONE】(累计从 n=16 越过 0.5 那步开始)
    //   ⇒ 这两句一起把"时长真的按 dt 走、且真的是 2.5s 而不是别的数"钉死了。
    for (int i = 0; i < 75; i++) ForceCalibration::update(raw, pose);
    CHECK(!ForceCalibration::isDone());      // 75 × 0.033 = 2.475s < 2.5s
    ForceCalibration::update(raw, pose);
    CHECK(ForceCalibration::isDone());       // 76 × 0.033 = 2.508s >= 2.5s

    // ⚠ 本用例【不断言零偏数值】⇒ 不依赖 currentGravityTerm 的缓存
    //   (那是 Task 2 那条红的成因); 别在这里照抄 Task 2 的热身, 那是另一件事。
    ForceCalibration::setUpdateDtForTest(-1.0);   // 还原, 免得污染后面的用例
    PASS();
}
```

**并且必须把它注册进 `main()`** —— 本文件的用例全靠 `main()` 里逐个显式调用，漏一行就等于这条用例根本不存在（`test_force_compensation.cpp:1743` 起是 `main()`；在 `test_zero_restartable();` 那一行之后加 `test_calib_tare_timing_follows_measured_dt();`）。
```

- [ ] **Step 2: 跑它，确认它【编译不过】—— 这正是"红"的形态**

```bash
cmd //c "D:\Projects\Touch\Touch_Client\tests\build_force_comp_test.bat"
```
Expected: **编译失败**，信息里出现 `ForceCalibration::setUpdateDtForTest` 未声明 + `update` 实参个数不匹配。
（本步的"红"是编译错误而不是断言失败 —— 因为新 API 还不存在。这是 TDD 在 C++ 里的正常形态。）

- [ ] **Step 3: 改头文件 —— 删掉 `dt` 参数，加测试钩子**

`Touch_Client/force/ForceCalibration.h`，把第 44-45 行那段

```cpp
    // Called each frame from pollForce (~30Hz)
    bool update(double dt, const double raw[6], const double pose[6]);
```

换成

```cpp
    // Called each poll tick from pollForce. 【dt 不再由调用方给】——
    // ★ 2026-09-22: 这里从前是 `update(double dt, ...)`，调用方传常数 0.033
    //   (= 名义节拍 FORCE_POLL_INTERVAL_MS，而那个常数是"节流下限"，不是实际节拍)。
    //   实测节拍 46~203ms (均值 92ms) ⇒ TARE 的"静默 0.5s + 累计 2s"在实际时间里
    //   是 ~1.35s + 5.4s，而提示打印的是 2.5s ⇒ 操作员在静默期里就松手 ⇒ 又采到瞬态。
    //   ⇒ 现在按【实测耗时】自算，做法与 ForceCompensation::stepIntervalSec 逐字同构
    //     (那边是 2026-09-21 用同一条理由改的)。
    // ⚠ 本函数的调用点【必须每次轮询都调】—— 实测间隔的计时器靠它保持新鲜，见 .cpp 里的说明。
    bool update(const double raw[6], const double pose[6]);

    // 用例专用: 把 dt 变成【确定的输入】(默认 -1 = 不干预, 走实测)。
    //   与 ForceCompensation::setStepDtForTest 同一个约定、同一条理由。
    void setUpdateDtForTest(double sec);
```

- [ ] **Step 4: 改实现 —— 实测间隔 + 先量时长再早退**

`Touch_Client/force/ForceCalibration.cpp`：文件顶部（`g_lastFinalizeMg` 附近，第 76 行之后）加：

```cpp
// 用例驱动的时间步长 (秒)。负数 = 不干预, 走实测。见头文件 setUpdateDtForTest。
static double g_updateDtForTest = -1.0;

// 两次 update() 之间的【真实】耗时 (s)。第一次调用返回 0。
// ★ 2026-09-22: 这就是从前写死在调用点上的那个 0.033 的替代。理由见头文件。
// ⚠ 它靠"每次轮询都被调用"保持新鲜 —— 若调用点又加回 `if (isRunning())` 的门,
//   计时器会停在【上一次运行】那一刻 ⇒ 下次启动的第一个 dt 是那之间的全部时间
//   (可能是几分钟) ⇒ 一步跨过静默期与累计期, 而且不会报任何错。判据见 Task 1 Step 8。
static double updateIntervalSec() {
    if (g_updateDtForTest >= 0.0) return g_updateDtForTest;
    static DWORD lastMs = 0;
    const DWORD now = GetTickCount();
    if (lastMs == 0) { lastMs = now; return 0.0; }
    const double dt = (now - lastMs) / 1000.0;
    lastMs = now;
    return dt;
}

void setUpdateDtForTest(double sec) {
    g_updateDtForTest = sec;
}
```

再把 `update()` 的开头（第 183-186 行）

```cpp
bool update(double dt, const double raw[6], const double pose[6]) {
    if (g_state == State::IDLE || g_state == State::DONE || g_state == State::ABORTED) {
        return (g_state == State::DONE || g_state == State::ABORTED);
    }
```

换成

```cpp
bool update(const double raw[6], const double pose[6]) {
    // ★ 先量节拍, 【再】早退 —— 顺序不能反。见 updateIntervalSec 的说明:
    //   计时器必须每次轮询都刷新, 否则下次启动的第一个 dt 会是"距上次运行的全部时间"。
    const double dt = updateIntervalSec();

    if (g_state == State::IDLE || g_state == State::DONE || g_state == State::ABORTED) {
        return (g_state == State::DONE || g_state == State::ABORTED);
    }
```

`update()` 函数体其余部分**一个字都不改** —— 它本来就用 `dt` 这个局部名（TARE 的 `g_phaseTimer += dt`、MOTION 的 `g_phaseTimer += dt`、SOLVE 的三点模板 `(dt * dt)`）。

- [ ] **Step 5: 改生产调用点 —— 去掉 `isRunning()` 的门**

`Touch_Client/relay/RelayCore.cpp:2094-2099`，把

```cpp
    if (ForceCalibration::isRunning()) {
        ForceCalibration::update(0.033, app.forceData.sixForceRaw, pose);
        if (ForceCalibration::isDone()) {
            // Apply results handled in idle() / keyboard callback
        }
    }
```

换成

```cpp
    // ★ 2026-09-22: 【去掉 isRunning() 的门, 改成无条件调用】。
    //   两件事一起改的, 不能只改一件:
    //   ① update() 自己测"距上次调用的实测耗时"当 dt (从前是调用方传常数 0.033,
    //      而真实节拍 46~203ms ⇒ 静默期被拉长 ~2.7 倍)。
    //   ② 那个计时器靠【每次轮询都被调用】保持新鲜。若继续用 isRunning() 门着,
    //      计时器会停在"上一次标定运行"那一刻 —— 下次按 'z' 时第一个 dt 就是那之间的
    //      全部时间 (几分钟), 一步跨过静默期与累计期, 而且【不会报任何错】。
    //   非运行态下 update() 自己早退 (只做一次 GetTickCount), 所以无条件调用无副作用。
    ForceCalibration::update(app.forceData.sixForceRaw, pose);
```

- [ ] **Step 6: 改 4 个测试调用点**

`Touch_Client/tests/test_force_compensation.cpp`，第 1160、1217、1255、1283 行的 `ForceCalibration::update(0.5, raw, pose);` 全部改成 `ForceCalibration::update(raw, pose);`，并在**每个用例的 `startZero()`/`start()` 之后**加一行 `ForceCalibration::setUpdateDtForTest(0.5);`、在用例结束前加一行 `ForceCalibration::setUpdateDtForTest(-1.0);`。
四处循环体的注释 `// 只采 0.5s, 未达阈值` / `// 静置 2s: 每次 0.5s...` 保持原意不变。

- [ ] **Step 7: 跑，确认两条都过**

```bash
cmd //c "D:\Projects\Touch\Touch_Client\tests\build_force_comp_test.bat" && \
  ./test_force_compensation.exe
```
Expected: 构建 `BUILD_EXIT=0`；新增用例 `calib_tare_timing_follows_measured_dt... PASS`；
总数从 `38 passed, 1 failed` 变成 **`39 passed, 1 failed`**（只多了这一条）。
⚠ 剩下的那 1 条红是 `test_zero_only_no_motion`，**不在本 Task 的范围内**（Task 2 修它）。不要在这里动它。

- [ ] **Step 8: 回读未修改但描述这件事的文件**

- `Touch_Client/config/Config.h:122` 那行注释（`FORCE_POLL_INTERVAL_MS = 33 // 处理节流【下限】`）—— 确认它没有被本 Task 说成"实际节拍"；若是，就地改掉。
- `Touch_Client/force/ForceCalibration.cpp` 里 `start()`/`startZero()` 的提示文字（第 127-130、145-150 行）—— 它们打印的是 `FORCE_CALIB_SETTLE_TIME_S + FORCE_CALIB_STILL_COLLECT_S` = `2.5s`。**本 Task 之后这个数字才第一次变诚实**，把这句话写进注释（"从前打印 2.5s 而实际等 ~6.75s"）。

- [ ] **Step 9: 提交**

```bash
git add Touch_Client/force/ForceCalibration.h Touch_Client/force/ForceCalibration.cpp \
        Touch_Client/relay/RelayCore.cpp Touch_Client/tests/test_force_compensation.cpp
git commit -m "fix(force): 调零的 dt 改用实测间隔 —— 常数 0.033 让静默期被拉长 2.7 倍

pollForce 实测节拍 46~203ms(均值 92ms), 而 ForceCalibration::update 收到的是
常数 0.033(= 名义节流下限 FORCE_POLL_INTERVAL_MS) ⇒ TARE 的'静默 0.5s + 累计 2s'
在实际时间里是 ~1.35s + 5.4s, 而提示打印 2.5s ⇒ 操作员在静默期里就松手 ⇒ 又采到瞬态
(正是 2026-09-21 加静默期要治的那件事)。

改成按实测耗时自算, 做法与 ForceCompensation::stepIntervalSec 同构; dt 参数删除
⇒ 结构上没有任何地方能传错。调用点去掉 isRunning() 的门(计时器靠每次轮询保持新鲜),
非运行态由 update() 自己早退。新增 setUpdateDtForTest 把时间变成确定的输入。"
```

---

### Task 2: 修 `test_force_compensation` 那条真红（测试间状态污染）

**为什么**：`test_zero_only_no_motion` 里 `CHECK(fabs(fd.compensated[2] - 0.90) < 0.05)` 现在是**真的红**（Task 1 之后仍是 `39/1` 里的那 1）。
⚠ **那个 `0.90` 本身也是错的**（它是个陈旧期望值）—— 见下面那个 ★ 块；本段只讲**两个成因中的第一个：缓存污染**。
**真因**：`finalizeBias()` 读的是 `ForceCompensation::currentGravityTerm()`（`ForceCalibration.cpp:98`，缓存 `g_lastFg`，只由 `step()` 写）。而本用例**在 TARE 循环之前从没调过 `step()`** ⇒ 它读到的 `A·g` 是**同文件里更早某条用 `diagA(1.0)` 的用例**留在缓存里的 `g = (0,0,9.81)` ⇒ 零偏被减掉 9.81 而不是 `0.42×9.81 = 4.1202` ⇒ 打印出的 `bias=(-0.480,-1.350,-9.830)`、`compensated[2] = 5−(−9.83)−4.1202 ≈ 10.7` ≠ 0.90。
**⇒ 生产路径上 `step()` 每 8 ms 就跑一次刷新缓存 ⇒ 这是【测试隔离】问题，不是生产 bug**（已由 `git stash` + 原样重建证明：HEAD 上一模一样 38/1）。
**修法**：让用例自己**先喂一帧让缓存变成它自己的 A·g**，再开始调零。修的是"用例没声明依赖的输入"，不是放宽断言。

> ### ⚠★ 2026-09-22 执行中发现：这条红有**两个独立成因**，本计划原先只写了一个
>
> 上面那个缓存污染修好之后（TARE 的 `biasF` z 实测从 **−9.830 → −4.140**，证明热身生效），
> **断言仍然是红的** —— 因为 z 的**期望值本身是陈旧的**。
>
> **代数（已逐行核过，不是推导）：**
> - `ForceCalibration::finalizeBias()`（`ForceCalibration.cpp:124`）：`bF = mean(raw_tare) − A·g`
> - `ForceCompensation::step()`（`ForceCompensation.cpp:1114`）：`comp[2] = raw[2] − bF[2] − Fg[2] + Fi[2]`
> - ⇒ 两者合起来 **`comp = raw − mean(raw_tare)`**（`A·g` 只减一次，干净）
> - 代进本用例的数：`raw[2]=5.0`、`mean(raw_tare[2])=−0.02`、`Fi=0`
>   ⇒ **`comp[2] = 5.0 − (−0.02) = 5.02`**，**不是** `0.90`
>
> **`0.90` 是哪来的**：`8fb1536`（2026-09-18）按**旧语义**写的 —— 那时 `finalizeBias` 存的是
> `mean(raw)` 本身，所以 `comp = raw − mean − A·g = 5.0 + 0.02 − 4.1202 = 0.8998`。
> 而 `0c45094`（2026-09-21，"重力被减了两遍"那个修复）把 `A·g` **挪进了零偏**，
> 此后 z 再多减一次 `4.1202` 就是**重复计**。
> `git merge-base --is-ancestor 8fb1536 0c45094` ⇒ **YES**（期望值是那个改动的**祖先**，从没跟着更新过）。
>
> **★ 判据是同一条用例自己的 x/y 两行**：它们写的是 `5.0 − (−0.48) = 5.48`、`5.0 − (−1.35) = 6.35`
> —— 正是 `raw − mean(raw_tare)` 这条规则。**只有 z 那一行破了这个模式**（多减了 `4.1202`）。
>
> **★ 结论：代码是对的，期望值是陈旧的。** 证据是 `0c45094` 治的是**现场真病**：
> 按 `'z'` 之后 `comp = −A·g ≈ 4N` ⇒ 闸门判不一致 ⇒ 全置零 ⇒ **一点力反馈都没有**。
> 要保留 `0.90` 就等于把那个故障恢复回来。
>
> **用户 2026-09-22 决定：改成 5.02。**（这是订正一个**陈旧期望值**，不是"为了好看改断言"——
> 顺手它让 z 与 x/y 服从同一条规则。Global Constraints 里禁的是改**容差/门限**，不是订正
> 一个已被证伪的期望值。）
>
> **⚠ 我（写计划的人）在这里犯的错**：Step 3 原先同时预测 `biasF z ≈ −4.14` **和** `comp ≈ 0.90`
> —— 这两个**不可能同时成立**。`0.90` 是我从那条**待修的断言**里照抄来的，没有拿
> `finalizeBias` 的当前语义复算一遍。这正是本项目那条"前提要回到源头核"的教训的又一例。

**Files:**
- Test: `Touch_Client/tests/test_force_compensation.cpp:1142-1198`（`test_zero_only_no_motion`）
- Test: `Touch_Client/tests/test_force_compensation.cpp`（同文件的其它 TARE 用例：加同一条热身，或注明为何不需要）

**Interfaces:**
- Consumes: `gateVisibleFrame()`（`test_force_compensation.cpp:42-47`，返回 `isStale=false` / `sixForceOnline=1` 的帧）、`ForceCompensation::step(AppState::ForceData&, const double pose[6])`、`diagA(double, double[9])`（`:29`）
- Produces: 无新 API。

- [ ] **Step 1: 先确认这条红【现在就存在】，并抄下打印的证据**

```bash
cmd //c "D:\Projects\Touch\Touch_Client\tests\build_force_comp_test.bat" && \
  ./test_force_compensation.exe 2>&1 | grep -E "FAIL|Results:"
```
Expected: `FAIL: fabs(fd.compensated[2] - 0.90) < 0.05` + `Results: 39 passed, 1 failed`
（Task 1 已加了一条用例 ⇒ 基线从 38/1 变成 39/1；**引用计数前重跑**，别照抄本文档）。
把 TARE done 那一行的 `biasF=(...)` 抄下来 —— 期望看到 z 分量 ≈ **−9.83**（不是 −4.14）。**这就是"读到了别人的 A·g"的直接证据。**

- [ ] **Step 2: 加"热身一帧"，让缓存属于本用例**

`test_zero_only_no_motion` 里，在 `ForceCompensation::setCalibration(A, bF, bM, cS);`（第 1148 行）之后、`ForceCalibration::startZero();`（第 1153 行）**之前**插入：

```cpp
    // ★ 2026-09-22: 必须【先让 step() 跑一帧】再开始调零。理由:
    //   finalizeBias() 读的是 ForceCompensation::currentGravityTerm() —— 那是 step() 的缓存
    //   (g_lastFg), 而本用例此前从没调过 step() ⇒ 它读到的是【同文件里更早某条用例
    //   (用 diagA(1.0)) 留在那里的 g = (0,0,9.81)】⇒ 零偏被减掉 9.81 而不是 4.1202
    //   ⇒ compensated[2] 的断言红 (实测 bias z = −9.83 就是证据)。
    //   ⚠ 这条红有【两个独立成因】: 本处修的是缓存污染; 另一条是 z 的期望值本身陈旧
    //     (0.90 是 2026-09-18 按旧语义写的, 而 0c45094 把重力项挪进了零偏 ⇒ 应为 5.02),
    //     那一半见下面 Step 3 —— 两半都做完这条断言才会绿。
    //   生产路径上 step() 每 8ms 跑一次刷新缓存 ⇒ 这是【测试隔离】问题, 不是生产 bug
    //   (已用 git stash + 原样重建证明: 任务开工时的 HEAD 上一模一样 39/1 —— 那条红是旧账)。
    //   ⚠★ 2026-09-22 订正(复审指出, 已核): 这一帧【不必】让闸门放行 —— 缓存 g_lastFg 是在
    //     ForceCompensation.cpp:1057 写的, 【在】参考量可用性判定(:1119)与投票(:1136/:1186)
    //     【之前】⇒ 即使这一帧被拒, 缓存照样刷新成本用例的 A·g。
    //     (从前这里写的是"喂的帧必须让闸门放行, 否则 compensated 被置零" —— 那句话把
    //      "compensated 会被置零" 与 "缓存不会刷新" 混成了一件因果, 是错的。)
    //     喂的帧数值本身不参与后面的断言。
    {
        AppState::ForceData warm = gateVisibleFrame();
        double warmPose[6] = {0, 0, 0, 0, 0, 0};
        ForceCompensation::step(warm, warmPose);
    }
```

- [ ] **Step 3: 跑一次 —— 确认热生效、但断言**仍然红**（因为期望值陈旧）**

```bash
./test_force_compensation.exe 2>&1 | grep -E "FAIL|Results:|biasF"
```
Expected: TARE done 那行的 `biasF` z 分量从 **−9.83 变成 ≈ −4.14**（= −0.02 − 4.1202）
⇒ **证明热身生效**；但 `FAIL: fabs(fd.compensated[2] - 0.90) < 0.05` **仍在**，
总数仍是 `39 passed, 1 failed`。
⚠ 这一步是**预期的中间状态**，不是热身没做对。它把"两个成因"当场分开 —— 别在这一步去改 Step 2。

- [ ] **Step 4: 订正陈旧的 z 期望值（用户 2026-09-22 已批准）**

`test_zero_only_no_motion` 里两处 `0.90` 都要改（**只改这两处数字，不动容差 `< 0.05`**）：

1. 参考量那一侧（约 `:1190`）：`fd.tcpForce[2] = 0.90;` → `fd.tcpForce[2] = 5.02;`
   （闸门会拿 `compensated` 与参考量比，两边必须自洽，否则被判不一致 ⇒ `compensated` 全置零 ⇒ 断言看不到东西。）
   ⚠ **注意这一处今天其实是【惰性】的，但照样要改** —— 已核过，别被"反正没影响"骗过去：
   · 闸门第一帧是**播种**而不是平滑（`ForceCompensation.cpp:1162-1163`：`if (reseed) g_guardEma[i] = d;`），
     所以 `z` 那一路的差就是**本帧的 4.12 N**，远超容差 `1.2464`；
   · 它之所以没把闸门判成不一致，是因为 **`z` (Fz) 那一票【不投票】**
     （`g_guardVote = {T,T,F,F,F,F}`，`ForceCompensation.cpp:1186` 的 `continue`）。
   · ⇒ 这条用例的 z 侧闸门检查**今天是惰性的**。而 **"Fz 该不该投票"是 `2026-09-21-open-items` 里
     一条**未结**的决定**（原依据已被推翻，见 `2026-09-21-gate-plan-executed.md`）——
     若将来 Fz 恢复投票，参考侧还留着 `0.90` 就会让这条用例**因为一个假前提而红**。
     ⇒ 改它是为了不让这个测试编码一个谎话，不是为了让今天绿。
2. 期望值（约 `:1197`）：`CHECK(fabs(fd.compensated[2] - 0.90) < 0.05);` → `... - 5.02 ...`

并把上面那段注释里的算式（现在写的是 `z: 5 − (−0.02) − 4.1202 = 0.8998`）改成：

```cpp
    // 期望值 (A = 0.42·I, g = (0,0,9.81) -> Fg = (0,0,4.1202), c_s 沿 z -> Mg = 0):
    //   x: 5 − (−0.48)          = 5.48
    //   y: 5 − (−1.35)          = 6.35
    //   z: 5 − (−0.02)          = 5.02     ← ★ 三轴同一条规则: comp = raw − mean(raw_tare)
    //   M: 5 − (0.010, −0.020, 0.005)
    // ★ 2026-09-22 订正: z 这里【从前写的是 `5 − (−0.02) − 4.1202 = 0.8998`】—— 那是陈旧的。
    //   0c45094 (2026-09-21, "重力被减了两遍") 之后 finalizeBias 存的是 `mean(raw) − A·g`,
    //   而 step() 里 comp 再减一次 Fg ⇒ A·g【总共只减一次】⇒ comp = raw − mean(raw_tare)。
    //   代进去 = 5.0 + 0.02 = 5.02。(A 是对角阵且 pose 全 0 ⇒ 重力只在 z 轴上,
    //   这正是只有 z 这一行破模式、x/y 本来就对的原因。)
    //   ⚠ 若把 0.90 改回去, 等于要求 comp 把重力减【两遍】—— 那正是 0c45094 治的那个现场故障
    //     (按 'z' 之后 comp = −A·g ≈ 4N ⇒ 闸门拒 ⇒ 一点力反馈都没有)。别改回去。
```

- [ ] **Step 5: 跑，确认这条变绿且总数变 40/0**

```bash
./test_force_compensation.exe 2>&1 | grep -E "FAIL|Results:|zero_only"
```
Expected: `zero_only_no_motion... PASS`，`Results: 40 passed, 0 failed`。
**若仍是红** ⇒ 停下来，把 TARE 的 `biasF` 那一行、闸门状态、以及 `compensated` 的实测值带回报告；
**不要**继续调数字直到它绿 —— 那恰好是本计划 Global Constraints 禁止的事。

- [ ] **Step 6: 扫同文件其它 TARE 用例，修或注明**

对 `test_zero_abort_not_applied`（`:1205`）、`test_sweep_still_enters_motion`（`:1243`）、`test_zero_restartable`（`:1270`）逐个判断：
- 断言了**零偏数值**的 ⇒ 必须加同样的热身（同 Step 2 的代码块）。
- 只断言 `isDone()` / 状态的 ⇒ **不需要**，但就地加一行注释说明"本用例不断言零偏数值，所以不依赖 `currentGravityTerm` 的缓存"。
⚠ **不要**盲目给所有用例加热身：那会把"这条用例其实依赖别处的状态"这件事藏起来。
⚠ **若加了热身反而变红** ⇒ 那条用例本来就在**依赖被污染的状态**（它把自己的期望值建在别人的 `A·g` 上了）。**那是真发现**：停下来，把该用例、它断言的值、以及新旧两次数值写进报告，**不要**改断言去迁就，也不要顺手删热身。本 Task 只把 `test_zero_only_no_motion` 修绿，别的用例一旦变红按"真发现"上报。

- [ ] **Step 7: 提交**

```bash
git add Touch_Client/tests/test_force_compensation.cpp
git commit -m "test(force): 修 test_zero_only_no_motion —— 缓存污染 + 一个陈旧的 z 期望值

这条红有【两个独立成因】, 两半都修完才绿:

① 缓存污染: finalizeBias() 读的是 ForceCompensation::currentGravityTerm()
   (step() 的缓存), 而本用例在 TARE 之前从没调过 step() ⇒ 读到同文件更早用例
   (diagA(1.0))留下的 g ⇒ 零偏减掉 9.81 而不是 4.1202 (实测 bias z = −9.83)。
   生产路径 step() 每 8ms 刷一次 ⇒ 测试隔离问题, 非生产 bug(stash+原样重建证明)。
   修法: 调零前先喂一帧声明自己的输入。

② z 的期望值陈旧: 0.90 是 8fb1536(2026-09-18)按旧语义写的 —— 那时 finalizeBias
   存 mean(raw) 本身, 故 comp = raw − mean − A·g = 0.8998。而 0c45094(2026-09-21)
   把 A·g 挪进了零偏 ⇒ comp = raw − mean(raw_tare) 只减一次 ⇒ 应为 5.02。
   判据是同一条用例自己的 x/y 两行(5.48 / 6.35)已经是这条规则, 只有 z 破了模式。
   git merge-base --is-ancestor 8fb1536 0c45094 ⇒ YES(期望值从没跟着更新)。
   ⇒ 代码是对的, 期望值是陈旧的; 保留 0.90 等于要求重力减两遍, 那正是 0c45094 治的现场故障。"
```

---

### Task 3: `saturation` 用例补成真的验证饱和

**为什么**：`test_force_pipeline.cpp:68` 那条用例叫 `saturation`，但**只调了 1 次 `step()`**。`fc=5 Hz / fs=125 Hz` 的二阶 Butterworth 一步只给 `b0·500 ≈ 6.7 N`，映射后 `fabs(hapticOut[0]) ≈ 6.7 × 0.0165 × 120 ≈ 13.3`，离夹子 `3.3 × 120 = 396` 差得远 ⇒ 那句 `<= clampedMax + 0.01` **恒真**（用例里的注释已经照实承认了这一点，本次把它做完）。
**修法**：跑到收敛，然后断言输出**正好落在夹子上**（不是"不超过"）。

**已核实的算式**（本计划开工前把每一环都读过一遍，不是照抄记忆）：
- `mapForceToTouch`（`ForcePipeline.cpp:79-89`）：`softDeadzone` → `× ratio`（`3.3/200 = 0.0165`）→ **硬夹到 ±`FORCE_MAX_TOUCH_N`=3.3**。
- `ForcePipeline.cpp:126`：`hapticOut[0] = fx × FORCE_FEEDBACK_LATERAL_SIGN(−1)`；`:135` 再 `× FORCE_REFLECTION_GAIN(120)`。
- ⇒ 收敛后 `hapticOut[0] = 3.3 × (−1) × 120 = −396 = −clampedMax`。
- 收敛速度受 `FORCE_GRADIENT_LIMIT = 50 N/frame` 限制（`ForcePipeline.cpp:100`）⇒ 越过映射门限（需 `filtered > 200`）只需 4 步；用 200 步与 `test_soft_deadzone_no_jump` 同一惯例。

**Files:**
- Test: `Touch_Client/tests/test_force_pipeline.cpp:68-92`（`test_saturation`）
- Test: `Touch_Client/tests/build_force_pipeline_test.bat`（构建脚本，只是用来跑，不改）

**Interfaces:**
- Consumes: `ForcePipeline::init()` / `ForcePipeline::step(AppState::ForceData&)`、`Config::FORCE_MAX_TOUCH_N`、`Config::FORCE_REFLECTION_GAIN`、`Config::FORCE_FEEDBACK_LATERAL_SIGN`
- Produces: 无新 API。

- [ ] **Step 1: 改断言 —— 从"不超过"改成"正好在夹子上"**

把 `test_force_pipeline.cpp` 的 **73-90 行**替换为（**第 91 行的 `PASS();` 保留** —— 它不在替换区内）：

```cpp
    // 远高于满量程 ⇒ 映射后必然打到夹子。★ 2026-09-22: 【跑到收敛再断言】。
    // 从前只调 1 次 step(): fc=5Hz/fs=125Hz 的二阶 Butterworth 一步只给
    // b0·500 ≈ 6.7 N, 映射后 ≈13.3, 离夹子 396 差得远 ⇒ 那句 `<= clampedMax` 恒真,
    // 用例名说 saturation 却什么都没验 (旧注释已照实承认, 本次做完)。
    fd.compensated[0] = 500.0; fd.compensated[1] = 0.0; fd.compensated[2] = 0.0;
    fd.compensated[3] = 0.0; fd.compensated[4] = 0.0; fd.compensated[5] = 0.0;
    fd.lastUpdateMs = GetTickCount();

    // 200 步与 test_soft_deadzone_no_jump 同一惯例 (≈10 个时间常数 ⇒ 充分收敛)。
    // ★ 2026-09-22 订正(实测): 越过映射门限(需 filtered > 200N)是【第 6 步】才到的
    //   (实测 filtered: 第 5 步 173N、第 6 步 223N)。约束它的是【滤波器自身的上升时间】,
    //   不是梯度限幅: fc=5Hz/fs=125Hz 的 τ ≈ 4 个采样, 第 5/6 步的理论值 ≈177/223N,
    //   与实测吻合。梯度限幅反而是【不生效】的 —— 因为单步滤波器输出只动 ~6.7N,
    //   远在 50 N/frame 之下。(本处原写"200/50=4 步", 那是把梯度限幅当成了约束, 归因错了。)
    for (int i = 0; i < 200; i++) ForcePipeline::step(fd);

    // ★ 判据从"不超过夹子"改成"【正好落在夹子上】"—— 这才是饱和。
    //   算式 (逐环核过, 见 ForcePipeline.cpp:79-89 / :126 / :135):
    //     softDeadzone(500)=500 → ×(3.3/200)=8.25 → 硬夹到 3.3 → ×SIGN(−1) → ×120
    //     = −396 = −FORCE_MAX_TOUCH_N × FORCE_REFLECTION_GAIN
    //   ⇒ 若谁把 mapForceToTouch 里的硬夹去掉, 输出会是 −8.25×(−1)×120 = −990 ⇒ 本条立刻红。
    double clampedMax = Config::FORCE_MAX_TOUCH_N * Config::FORCE_REFLECTION_GAIN;
    CHECK(fabs(fabs(fd.hapticOut[0]) - clampedMax) < 0.01);   // 正好在夹子上
    CHECK(fd.hapticOut[0] < 0.0);   // 正输入 ⇒ 负输出 (横向映射 FORCE_FEEDBACK_LATERAL_SIGN = −1)
```

（原第 80 行的 `fabs(fd.hapticOut[0]) <= clampedMax + 0.01` 与第 85 行的符号断言被上面两句取代；原第 86-90 行那段"已知的弱点"注释**删掉** —— 它描述的事情本次已经做完了。）

⚠ **不要丢掉原 81-84 行那段"符号断言为何故意写死负号"的理由**（它落在替换区内，上面的代码块没有复现它）。
那段话的要点是：**拿 `Config` 的常数去算断言会让断言对那个常数恒真**，所以这里故意写死负号；
同样的理由在 `test_coord_transform` 里还有完整论述。做法：把那 2-3 行理由**缩成一句注**留在新断言旁
（例：`// 符号故意写死负号(拿 Config 常数算 = 对常数恒真), 详见 test_coord_transform 的同段说明。`），
**不要**整段删掉 —— 否则"为什么不能写成 `> 0`"这个教训就只剩一个地方记得了。

- [ ] **Step 2: 跑，确认它【现在才第一次真的在判】且通过**

```bash
cmd //c "D:\Projects\Touch\Touch_Client\tests\build_force_pipeline_test.bat" && \
  ./test_force_pipeline.exe 2>&1 | grep -E "saturation|FAIL|Results:"
```
Expected: `saturation... PASS`，`Results: 7 passed, 0 failed`。
**若它红了 ⇒ 那是一个真发现**（说明映射里的硬夹不在我以为的位置）—— 带原始输出停下来，不要改断言去迁就。

- [ ] **Step 3: 反证这条断言真的有效（推荐做，2 分钟）**

**这是一次【临时】改动，必须还原，且还原本身要验。** 步骤严格如下：

1. 把 `ForcePipeline.cpp:86-87` 的两行硬夹**临时**注释掉；
2. 重跑 ⇒ Expected: `saturation` **红**，且 `hapticOut[0] ≈ −990`（= `8.25 × (−1) × 120`）；
3. **立刻还原那两行**（`git checkout -- Touch_Client/force/ForcePipeline.cpp` 是最稳的还原方式）；
4. **重跑确认 `Results: 7 passed, 0 failed`**；
5. `git status --porcelain Touch_Client/force/ForcePipeline.cpp` 必须**空** —— 空才证明还原干净。

⚠ 第 4、5 步**不能省**：本项目记过教训（恒真的断言让人以为验过了），而"临时改完忘了还原"是同一类自欺。
⚠ 若第 2 步**没有变红** ⇒ 说明真正的夹子不在那里，或还有第二处夹子 ⇒ **那是真发现**，把它写进报告，然后照第 3-5 步还原并如实说明"反证失败"。

- [ ] **Step 4: 提交**

```bash
git add Touch_Client/tests/test_force_pipeline.cpp
git commit -m "test(force): saturation 用例补成真的验证饱和 —— 跑到收敛后断言正好落在夹子上

从前只调 1 次 step(), 而 fc=5Hz/fs=125Hz 一步只给 b0·500 ≈ 6.7N, 映射后 ≈13.3,
离夹子 396 差得远 ⇒ 那句 '<= clampedMax' 恒真。现跑到 200 步收敛, 判据改成
|hapticOut[0]| == FORCE_MAX_TOUCH_N × FORCE_REFLECTION_GAIN (== 396), 并保留符号断言。
去掉硬夹会立刻变红(已反证)。"
```

---

### Task 4: 让 `test_safety_core` 能被构建 —— 第一次拿到它的真实结果

**为什么**：`run_tests.bat` 对 `test_safety_core` **只跑不建**（第一批 for 循环里那 7 个 exe 是"存在就跑"），而它**根本没有任何 build 脚本**；现在磁盘上的 `test_safety_core.exe` 是 **2026-07-25 15:53** 的，比 `test_safety_core.cpp`（07-25 15:59）还早 6 分钟，而 `RobotStateMachine.cpp` 是 07-26、`RobotError.h` 是 09-21 改的。
**⇒ 它报的那条 `7 / 1` 是关于当前代码的【零信息】。** 实测重建还**构建不过**：`RobotDiagnostics.cpp:108` 调 `RelayCore::instance().reportDiagnostic(...)`，测试配方不带 RelayCore ⇒ `LNK2019`（两个符号：`RelayCore::instance` / `RelayCore::reportDiagnostic`）。
**⇒ 本条与 2026-09-21 记下的 `build_coord_safety_test.bat` 是同一类坑**（那条漏 `.cpp` ⇒ 链接从来没成功过 ⇒ 跑它等于什么都没验），只是更大。

**Files:**
- Modify: `Touch_Client/safety/RobotDiagnostics.cpp:105-110`（把 RelayCore 调用加进一个 `#ifndef` 守卫）
- Create: `Touch_Client/tests/build_safety_core_test.bat`
- Modify: `Touch_Client/tests/run_tests.bat`（把 `test_safety_core` 从"只跑"移进"先建再跑"那一组）
- Delete: `Touch_Client/tests/_fresh_safety_core.bat`（取证用的临时脚本，本计划开工时建的）

**Interfaces:**
- Produces: 宏 `TEST_NO_RELAY_CORE` —— 定义它即把 `RobotDiagnostics::logError` 里那条"转发给 RelayCore 发 D| 帧"的调用编译掉。
- 沿用既有先例：`touch_client/safety/SingularityAvoidance.cpp:17-26` 用 `#ifndef TEST_SINGAVOID` 挡掉 `RelayCore::instance().reportWarning(...)`，`test_singularity_avoidance.cpp:3` 注明理由。本 Task 照这个形状做，但**换一个更通用的名字**（不止一个套件需要挡 RelayCore）。

- [ ] **Step 1: 先确认"构建不出来"这件事本身**

（已经在计划的地基一节实测过；本步是让执行者自己复现一次，别信转述。）

```bash
cmd //c "D:\Projects\Touch\Touch_Client\tests\_fresh_safety_core.bat"
```
Expected: `LNK2019: 无法解析的外部符号 ... RelayCore::instance(void)` 与 `... RelayCore::reportDiagnostic(int,double,char const *)`，`BUILD_EXIT=2`。

- [ ] **Step 2: 给 RelayCore 调用加守卫**

`Touch_Client/safety/RobotDiagnostics.cpp`，把第 105-110 行

```cpp
    e.constraintForceMag = constraintMag;
    log(e);

    RelayCore::instance().reportDiagnostic(
        static_cast<int>(error.code), error.speedFactor, errorCodeName(error.code));
}
```

改成

```cpp
    e.constraintForceMag = constraintMag;
    log(e);

    // 把这条诊断转发到线上 (D| 帧) —— 需要 RelayCore 整个链接进来。
    // ★ 2026-09-22: 用例里挡掉它。理由与 SingularityAvoidance.cpp 的 TEST_SINGAVOID 相同:
    //   test_safety_core 只验状态机与升级逻辑, 它【不该】为了那一帧把 RelayCore
    //   (以及它的 winsock / OpenHaptics / 全部依赖) 拉进来 —— 那条链现在甚至链接不过
    //   (LNK2019), 于是这条用例的 exe 一直是个 07-25 的陈旧二进制, 它报的 FAIL 是零信息。
    //   沿用同一形状, 但用更通用的名字: 不止一个套件需要挡 RelayCore。
    //   落盘 (log(e)) 与内存计数不受影响 —— 被挡掉的只是"再发一帧给 MATLAB"。
#ifndef TEST_NO_RELAY_CORE
    RelayCore::instance().reportDiagnostic(
        static_cast<int>(error.code), error.speedFactor, errorCodeName(error.code));
#endif
}
```

- [ ] **Step 3: 建构建脚本**

创建 `Touch_Client/tests/build_safety_core_test.bat`（照现有脚本的形状，`/D` 上那个新宏）：

```bat
@echo on
call "D:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvarsall.bat" x64
cd /d "D:\Projects\Touch\Touch_Client\tests"
rem NOTE: keep this file ASCII-only. Non-ASCII comments in a .bat get mis-decoded by
rem cmd.exe under a non-UTF-8 codepage and can silently swallow the following line.
cl /EHsc /std:c++17 /I"D:\Projects\Touch\OpenHaptics\Developer\3.5.0\include" /I"D:\Projects\Touch\OpenHaptics\Developer\3.5.0\utilities\include" /DWIN32 /DWIN32_LEAN_AND_MEAN /DNOMINMAX /D_CRT_SECURE_NO_WARNINGS /D_WINSOCK_DEPRECATED_NO_WARNINGS /DTEST_NO_RELAY_CORE test_safety_core.cpp ..\safety\RobotStateMachine.cpp ..\safety\RobotDiagnostics.cpp /Fe:test_safety_core.exe
echo BUILD_EXIT=%ERRORLEVEL%
```

⚠★ **这个 `.bat` 必须是纯 ASCII**（含注释）。项目里 `build_force_pipeline_test.bat` 的头部就写着这条：
非 ASCII 注释在非 UTF-8 代码页下会被 cmd.exe 误解码，**可能静默吞掉后面那一行** ——
即"注释写错一个字符 ⇒ 编译命令消失 ⇒ 构建失败得莫名其妙"。要写中文说明就写到别处（例如
`.superpowers/sdd/` 的报告里），**不要写进 .bat**。

- [ ] **Step 4: 删掉取证用的临时脚本**

```bash
rm -f "D:\Projects\Touch\Touch_Client\tests\_fresh_safety_core.bat"
```
（它是本计划开工时为"不覆盖既有 exe 而另建一个名字"取证用的，Step 3 之后由正式脚本取代。）

- [ ] **Step 5: 跑 —— 这一步才第一次得到关于当前代码的真结果**

```bash
cmd //c "D:\Projects\Touch\Touch_Client\tests\build_safety_core_test.bat" && \
  ./test_safety_core.exe 2>&1 | tail -12
```
Expected: `BUILD_EXIT=0`；输出里**每条用例只出现一次的**状态机迁移行（旧 exe 的日志里 `RUNNING → DEGRADED` 出现了**两次**，那是它比当前源文件旧的直接证据）。
**结果的两种可能，都必须照实带回：**
- `Results: 8 passed, 0 failed` ⇒ 那条红是**陈旧二进制**造出来的，不是真红。就地删掉本条在记忆里的"待修"身份。
- 仍有 `FAIL: fabs(sm.speedFactor() - 0.3) < 0.01` ⇒ **这是真红**，是一条与 `SAFETY_BOUNDARY_CLAMP_ENABLED` 那一批改动可能相关的回归。**停下来，另开 systematic-debugging，不要在本 Task 里顺手修。**

- [ ] **Step 6: 把 `test_safety_core` 从"只跑"移进"先建再跑"**

`Touch_Client/tests/run_tests.bat`：从第一个 `for %%e in (...)` 列表里**删掉** `test_safety_core.exe` 那一行，并在 `test_tcp_calibration` 那一段之后照同样的形状加一段"build → run"（构建脚本名 `build_safety_core_test.bat`、exe 名 `test_safety_core.exe`）。

- [ ] **Step 7: 全量重跑，确认它被真的重建了**

```bash
cmd //c "D:\Projects\Touch\Touch_Client\tests\run_tests.bat" > /tmp/rt2.log 2>&1
grep -nE "^=== test_|^Results: |^[0-9]+ passed|BUILD_EXIT" /tmp/rt2.log
```
Expected: `--- Building test_safety_core ---` 出现、`BUILD_EXIT=0`、随后 `=== test_safety_core.exe ===` 与它的真实结果。

- [ ] **Step 8: 提交**

```bash
git add Touch_Client/safety/RobotDiagnostics.cpp Touch_Client/tests/build_safety_core_test.bat \
        Touch_Client/tests/run_tests.bat
git commit -m "test(safety): 让 test_safety_core 真的能被构建 —— 它一直跑的是 07-25 的陈旧 exe

RobotDiagnostics.cpp 调 RelayCore::instance().reportDiagnostic(), 而这条用例的配方
不带 RelayCore ⇒ LNK2019 ⇒ 没有 build 脚本可用, 磁盘上是 07-25 的 exe(比测试源还早
6 分钟), 而 RobotStateMachine.cpp 07-26 / RobotError.h 09-21 都改过
⇒ 它报的 7/1 是关于当前代码的零信息。与 2026-09-21 那条 build_coord_safety_test.bat
漏 .cpp 是同一类坑。

照 SingularityAvoidance 的 TEST_SINGAVOID 先例加 #ifndef TEST_NO_RELAY_CORE 守卫
(只挡'再发一帧给 MATLAB', 落盘与计数不受影响), 新增 build_safety_core_test.bat,
并把它从 run_tests.bat 的'只跑不建'那一组移进'先建再跑'。"
```

---

### Task 5: 把死代码的三条结论落成注释 —— 含一条**被推翻**的

**为什么**：2026-09-21 记下"三处死代码"，本计划开工时**逐条核了调用图与取值**，结果是 **1 条被推翻、2 条成立**。错的结论必须就地更正，否则下一个人会照它去删一段其实在起作用的保护。

| 结论 | 核验结果 |
|---|---|
| ① 姿态钳位 `clampOrientToBounds` 从不夹东西 | **❌ 被推翻（它确实在夹）** |
| ② `SafetyPredictor` 两处"边界越界就 REJECT" | ✅ 成立（一处可证死，另一处只因为开关关着才是死的） |
| ③ `SafetyBoundary::computeSpeedFactor` 无生产调用点 | ✅ 成立 |

**① 为什么被推翻**（★ 2026-09-22 执行后按代码订正过措辞 —— 本段原先有两处不准确，
实现者按代码改对了；**committed 的注释以代码为准**）：
钳位边界确实等于角度满量程（±180/±90/±180），但**被夹的值不是规范化的角度**。
准确机制（逐行核过）：
- 期望目标 `desired = m_orientRefRobot + robot_dR`（`:1047-1053`）—— 代码在 `:1047` 自己写着
  **"【不是累加】"**，因为 `m_orientRefRobot` 是按下按钮2那一刻抓的、整个按住期间不变；
  （我原先写成"被夹的值是 `ref + offset` 的累加和"，把**期望**与**累加器**混为一谈了。）
- 真正逐帧累加的是 `m_targetOrient` 自己（`:1090-1093`：`+= damped` 后立刻钳位），
  它**朝那个参照收敛** ⇒ 被夹的量的界就是参照，而参照抄自那一刻的实际姿态（`:1303`）。
**要点在【从不 wrap】：全仓没有任何一处把它规范化到 (−180,180] ⇒ ±180/±90 是【真】边界。
⇒ `:1093` 那一处是该量唯一的约束者。**
**⚠ 证据等级（由 Task 5 的实现者纠正，我接受）**：`force_demo_log.csv` 里那列 `pose_rx`
**是【机器人实际姿态】**（该列来自 `app.robotActualPose.rx`），**不是**本函数的输出
⇒ 它证明的是"**参照**坐在边界上 ⇒ 再加正向偏移就越界"（最贴近的一次差 0.012°；179.560 那档差 0.44°），
**不是"本函数确实夹过"的直接证据**。⇒ **结论不变，但依据是"参照贴着边界"，不是"观测到它夹了"。**
**⚠ 顺带揪出一处注释与代码不符**：`Config.h:39` 写着钳位点是"RelayCore 的 4 个下发点（目标位置/姿态路径）"，但 `clampToBoundaryActive` 那 4 处**全是位置**，姿态那条路**走的是另一个入口** ⇒ 关掉 `SAFETY_BOUNDARY_CLAMP_ENABLED` **不会**关掉姿态钳位。

**Files:**
- Modify: `Touch_Client/relay/RelayCore.cpp:27-43`（`clampOrientToBounds` 就地更正注释）
- Modify: `Touch_Client/config/Config.h:39`（更正"4 个下发点"的说法）
- Modify: `Touch_Client/safety/SafetyPredictor.cpp:65-71` 与 `:271-277`（标注为死）
- Modify: `Touch_Client/relay/SafetyBoundary.h:48`（`computeSpeedFactor` 标注无生产调用点）

**Interfaces:** 无新 API；**本 Task 只改注释，不改任何一行可执行代码**（死代码是**标注**而不是删除 —— 与本项目既有做法一致："代码保留、翻回即恢复、取消原因就地写明"。若用户要求删除，那是另一个决定。）

- [ ] **Step 1: 更正 `clampOrientToBounds` 的注释（把错的结论改对）**

`Touch_Client/relay/RelayCore.cpp`，在 `clampOrientToBounds` 函数体上方加：

```cpp
// ⚠★ 2026-09-22 更正: 这里【不是】死代码 —— 2026-09-21 曾记下"范围等于满量程 ⇒ 从来没夹到过
//   任何东西", 那句话是错的。边界确实等于角度满量程, 但【被夹的值不是规范化的角度】:
//   它是 m_orientRefRobot + R·(笔杆偏移) 的【累加和】(见 :1029-1031, 按 ≤3°/frame 步进),
//   初值取自按下按钮2那一刻的实际 RPY (:1291)。角度只做加法、从不 wrap
//   ⇒ 本函数是唯一在约束它的东西。
//   【而且它现在真的活在边界上】: force_demo_log.csv 实测 pose_rx ∈ ±179.95~±179.99
//   (9686 行在 [+179,+180]、4801 行在 [−180,−179], 相邻行跨 ±180 跳变)
//   ⇒ 往外多转 0.44° 就撞 SAFE_RX_MAX, 然后每帧都夹。
// ⚠ 它还【不受】SAFETY_BOUNDARY_CLAMP_ENABLED 管: clampToBoundaryActive 的 4 个调用点
//   (:849/:1240/:1306/:1341) 全是【位置】, 姿态这一路根本没走那个开关
//   ⇒ 关掉那个开关【不会】关掉本函数。Config.h:39 的注释说钳位在"目标位置/姿态路径"
//   的 4 个下发点 —— 与代码不符, 已一并更正。
```

- [ ] **Step 2: 更正 `Config.h:39` 那句**

把"① 钳位: RelayCore 的 4 个下发点 (目标位置/姿态路径)"改成如实的两条：

```cpp
//   ① 钳位【位置】: RelayCore 的 4 个下发点 (clampToBoundaryActive 在 :849/:1240/:1306/:1341),
//      由本开关管。⚠【姿态】的钳位 (clampOrientToBounds, RelayCore.cpp:27) 【不】走这个开关
//      —— 它是独立的一处, 关掉本开关不会关掉它 (2026-09-22 核过)。
```

⚠ **改完必须回读同一段的后半，那里有一句会因此变成假话**（这正是本项目那条
"改行为后没回读未修改但描述它的文件"的教训）。该段末尾现在写着：

```
//   ⇒ 两道【仍然一起关】, 但理由不是"否则拦得住", 而是【保持两者一致】: ...
//   两道都由 SafetyBoundary::clampToBoundaryActive 这一个入口把关。
```

"两道都由 … 这一个入口把关" 在**姿态那一路**上不成立（它根本没走那个入口）⇒
那句要改成如实的三条，例如：
"① / ② 两道由 `SafetyBoundary::clampToBoundaryActive` 这一个入口把关；**姿态钳位是第三处、
独立的一处，不受本开关管**。" 请连同上面那句"仍然一起关"的措辞一起核一遍，**不要留下自相矛盾的两句**。

- [ ] **Step 3: 标注 `SafetyPredictor` 的两处死判**

在 `Touch_Client/safety/SafetyPredictor.cpp:65` 与 `:271` 各自的 `if` 上方加（两处措辞**要不同**，因为死的理由不同）：

`:65` 处：
```cpp
    // ⚠★ 2026-09-22: 本谓词【可证恒为假】—— 喂进来的 target 已经是 :64 的 clamp 输出
    //   (调用点只此一处: RelayCore.cpp:852, 实参就是 :849 的 clamped), 而 clampToBoundary
    //   是幂等的 ⇒ clamp(clamp(x)) == clamp(x) ⇒ 比较永远不成立。
    //   【开关两种状态都死】: 开关关 == identity; 开关开 == 幂等。
    //   保留它是为了"意图可读", 但别把它当保护 —— 真正的执行点是 :64 那次 clamp。
```

`:271` 处：
```cpp
    // ⚠★ 2026-09-22: 本谓词【今天】恒为假, 但理由与上面 :65 那条【不同】, 别混为一谈。
    //   这里拿到的 tcpCheck 来自 servoCmdX/Y/Z (:1120-1121), 而那三个抄自 :1114 的 clamped。
    //   一般情况下那个 clamped 已被 :849 夹过 ⇒ 幂等 ⇒ 死。但有一条例外:
    //   :849 那次钳位在 `if (appState.lastButtonState)` 里 (:806) —— 纯姿态模式(只按按钮2)
    //   会跳过它, 于是 clamped 保持 m_targetPos, 随后 :1078-1080 又被 tcpAdj (≤5mm 的
    //   奇异避让调整) 【推到夹紧之后】。此时若 m_targetPos 贴在盒面上 5mm 以内
    //   (实测 pose_y 到过 −342.2, 而 SAFE_Y_MIN = −350 ⇒ 只差 7.8mm), 本谓词【会】拒。
    //   ⇒ 它今天死, 只是因为 SAFETY_BOUNDARY_CLAMP_ENABLED == false (clampToBoundaryActive
    //     退化成 identity)。开关翻回来后, 这条【不是】死的。
```

- [ ] **Step 4: 标注 `computeSpeedFactor`**

`Touch_Client/relay/SafetyBoundary.h:48` 上方加：

```cpp
// ⚠★ 2026-09-22 核过 (全仓 + .bat + .vcxproj): 本函数【没有任何生产调用点】,
//   唯一的使用者是 tests/test_coord_safety.cpp:265/280/293/302/316。
//   ⚠ 别与两个【活着】的同名概念混淆:
//     · RobotStateMachine::speedFactor()  (safety/RobotStateMachine.cpp:28,
//       被 RelayCore.cpp:835/:1382/:1888 消费) —— 生产在用;
//     · SafetyVerdict::speedFactor 字段 (safety/SafetyPredictor.h:22) —— 生产在用。
//   ⇒ 删或留由用户定; 现状是【留着 + 标出来】, 免得下一个人以为边界降速已经接上了。
```

- [ ] **Step 5: 只改注释 ⇒ 构建 + 全量重跑，确认零行为变化**

```bash
cmd //c "D:\Projects\Touch\Touch_Client\tests\run_tests.bat" > /tmp/rt3.log 2>&1
grep -nE "^=== test_|^Results: |^[0-9]+ passed" /tmp/rt3.log
```
Expected: 与 Task 4 Step 7 的结果**逐条相同**（本 Task 一行可执行代码都没动）。

- [ ] **Step 6: 提交**

```bash
git add Touch_Client/relay/RelayCore.cpp Touch_Client/config/Config.h \
        Touch_Client/safety/SafetyPredictor.cpp Touch_Client/relay/SafetyBoundary.h
git commit -m "docs(safety): 更正/标注三处'死代码' —— 其中姿态钳位那条是错的, 它确实在夹

2026-09-21 记下'三处死代码', 2026-09-22 逐条核了调用图与取值:
· clampOrientToBounds: 【推翻】。边界确实等于满量程, 但被夹的值是 ref+R·offset 的
  【累加和】(从不 wrap) ⇒ 本函数是唯一在约束它的东西; 且实测 pose_rx ∈ ±179.95~±179.99
  ⇒ 往外多转 0.44° 就撞 180。它还【不受】SAFETY_BOUNDARY_CLAMP_ENABLED 管 (那 4 个
  调用点全是位置) ⇒ 一并更正 Config.h:39 与该开关有关的说法。
· SafetyPredictor 两处边界 REJECT: 【成立】, 但两处死的理由不同 —— :65 由幂等可证死;
  :271 只因开关关着才死 (纯姿态模式会被 ≤5mm 的 tcpAdj 推到夹紧之后)。
· computeSpeedFactor: 【成立】, 无生产调用点, 只有用例; 与两个活着的同名概念区分开。"
```

---

## 收尾

## ★ 合并前必做：文字类收口（复审累积，全是【注释/文字】错，无行为影响）

> 这些**不是** Task 5 的内容（都在 `tests/` 下的测试文件里），也**不是**功能缺陷。
> 但本项目已经吃过"注释撒谎"的亏（见 `Config.h` 里"那份文字会在改参考量时撒谎"），
> 而且**其中 3 条是写本计划的人（我）今天写错的** —— 正如实地留在记录里。

1. **`test_force_pipeline.cpp:81-82`** —— "梯度限幅 50 N/frame, 越过映射门限(需 filtered > 200N)只要 4 步"。
   **错两处**: ① 约束是**滤波器上升时间**不是梯度限幅（复审用双线性系数独立复算：
   Ω=tan(2π·5/250)=0.12635 ⇒ b0=0.013364, y5=178.4, y6=225.7，与实测 173/223 吻合；
   而 y5 ≤ 178 < 200 ⇒ 第 5 步**不可能**越过，与限幅无关）；② `filtered > 200` 是**严格**不等式，
   而 200/50=4 恰好**等于** 200。
   改成：越过发生在**第 6 步**，约束是滤波器上升时间（τ≈4 采样），梯度限幅只起边际作用。
2. **`test_force_compensation.cpp` 的热身注释** —— "HEAD 上一模一样 38/1" 应为 **39/1**
   （Task 1 加了一条用例之后）。
3. **同处** —— "喂的帧必须让闸门【放行】, 否则 compensated 被置零" 把两件因果混成一件。
   **已核**: `g_lastFg` 写在 `ForceCompensation.cpp:1057`，**在**参考量可用性判定(`:1119`)与
   投票(`:1136`/`:1186`)**之前** ⇒ 被拒的帧**照样**刷新缓存。改成"不必放行"。
4. **`test_force_pipeline.cpp` 新断言旁** —— 补回原 81-84 行那条教训的**一句话**：
   "符号故意写死负号 —— 拿 `Config` 常数去算断言会对那些常数**恒真**；完整论述见
   `test_coord_transform` 的同段。"
5. **`test_force_pipeline.cpp:90-91`** —— 加一条 `CHECK(clampedMax > 100.0);`。
   理由：现在的期望值是从**流水线自己乘的那两个常数**推出来的 ⇒ 那两个常数被重调/回归时
   断言会一直绿。这与**上面第 4 条要补回的警告**正是同一个道理，而文件里已有同形的先例：
   `test_residual_deadzone:28` 的 `CHECK(dz > 0.1);`（注释："钉住'门限是 0.2 那个量级',
   免得将来它被改小到让上面那句又变成实话"）。⇒ 照抄那个做法。
6. **`test_safety_core.cpp:2-7` 的头部构建配方仍缺 `/DTEST_NO_RELAY_CORE`** ——
   **★ 这正是 Task 4 刚修掉的那个 LNK2019，还留在测试文件的注释里**：谁照着那段头部注释去
   构建，就立刻把那个洞重新打开一遍。（新脚本 `build_safety_core_test.bat` 是对的，只有
   文件头那几行没跟上。）先例是 `test_singularity_avoidance.cpp:3` —— 那边**在测试源里**
   写明了要 define `TEST_SINGAVOID`。⇒ 照它补一行即可。

**记入下一个计划（不在本轮做）**：
- **`run_tests.bat` 那 6 个"只跑不建"的套件** —— 见上面"不在本计划内"那一节：`test_escalation.exe`
  与 `test_constraint_force.exe` 已确认是 07-25 的 ⇒ 它们的绿同样可能是假的。**这是下一轮的第一件事。**
- **`run_tests.bat` 里其它脚本还没有 `if not defined VCINSTALLDIR` 这层保护** ⇒ 全量跑一趟时，
  排在后面的套件（如 `test_noise_probe`）会因为 PATH 累积而构建失败、**静默地不运行**。
- **`test_force_pipeline.cpp:169` 的 `stale_detection` 是恒真的**：
  `CHECK(fd.isStale == false || fd.isStale == true);`（文件自己也写着 "trivially passes"）
  ⇒ 而且整条用例**对 `step()` 的输出什么都没断言**。**这是 Task 3 修掉的那一类缺陷的下一处**，
  但它需要先想清"这条用例到底该验什么"（属设计，不是替换一行），所以不塞进本轮。
- `test_force_compensation.cpp` 里那句"同时证明保留的 A 确实进了 setCalibration"**已不被该断言支持**
  （`A` 恒等抵消 ⇒ `setCalibration` 丢掉 `A` 也照样 5.02）。该性质仍由 `currentModel` 往返检查覆盖。
- `test_sweep_still_enters_motion` / `test_zero_restartable` 打印出**数值正确**的 `biasF`，
  只是因为前一条用例恰好留下同一个 `A=diagA(0.42)` 的缓存 ⇒ **没有任何断言能发现重排序**。
- **★ 我（写计划的人）在 Task 4 里又写错一条判据**：计划说"旧 exe 的日志里 `RUNNING → DEGRADED`
  出现两次 ⇒ 是陈旧二进的证据" —— **被实测反证**。那两行是**两个不同发射点**的正常输出
  （`RobotStateMachine.cpp:87` 带原因行 + `:212` 通用迁移行），**新旧 exe 都成对打**。
  站得住的证据是：mtime（exe 比 cpp 早 6 分钟）+ 无 build 脚本 + 链接本就不通过 +
  `can_move_guard` 的结局不同。另外我猜的失败断言（`speedFactor`）也与实际（`can_move_guard`）不同。
  ⇒ 教训与 [[verify-premises-at-the-source]] 同类：**"看着像指纹"的规律，下结论前要在两个版本上都跑一遍。**

---

- [ ] 全量测试终态记录：把 `/tmp/rt3.log` 的逐套件结果写进 `Docs/superpowers/specs/` 一份短记录（含"哪些套件 run_tests.bat 只跑不建"这份清单 —— 它是 Task 4 顺带查出来的、下次还会咬人的事实）。
- [ ] **更新记忆**：`2026-09-21-known-reds-and-dead-code` ①（那条红已修，真因是测试隔离）+ ⑤（姿态钳位那条**被推翻**）；`2026-09-21-open-items` 的 A 节 2/3/4/5 勾掉、A 节 1 的提交清单更新。
- [ ] 清掉会话产物（`2026-09-21-open-items.md` 第 17 条，低优先，**确认无用再删**）：`Touch_Client/tests/_build_sa.bat` / `_build_sa.log` / `_hello.cpp` / `_rebuild_and_test.bat`。它们都是未跟踪的草稿；`_rebuild_and_test.bat` 的内容是重建 `test_singularity_avoidance`（与既有的 `build_singavoid_test.bat` 功能重叠，删前比一眼）。**`Hardware/stl.zip` 与那几个 `.log` / `.csv` / `send_transcript.txt` 不删**（`2026-09-21-open-items` 第 1 条说是"有意留着"）。
- [ ] **不要**合并到 `master`、不要开 PR（用户 2026-09-21 的决定）。不要 `git push`，除非用户明说。

## 明确【不在】本计划内的（别顺手做）

- **上机项**：笔压是否还活着 / 横向方向 / 姿态依赖 / 触觉摇晃 / 死区重定（`0.20 → 更小`）/ 孪生拖不动视角 —— 全部要机械臂，见 `2026-09-21-open-items.md` 的 B 节。
- **`test_payload_calibration` 那条刻意留红**（`12:38 pose 1 不是 INCONSISTENT`）—— 它等的是**重采带 `@720` 列的夹具**，而重采时**必须同时**把夹具读取器改成"响亮拒绝未知列数"（现在的 `mgSplitRow(q, c, 25)` 会把 31 列的行**静默截断**成 25 列 ⇒ 重采完会看起来做完了却什么都没验）。那是独立的一次改动、要它自己的复审。
- **`run_tests.bat` 里另外 6 个"只跑不建"的套件**（constraint_force / feedback_parser / escalation / kinematics / coord_safety / force_pipeline）—— 本计划只修了 `test_safety_core` 那一个（因为只有它报红）。**★ Task 4 之后这条仍是活隐患，而且已经拿到硬证据**：`test_escalation.exe` 与 `test_constraint_force.exe` 都是 **07-25** 的二进制 ⇒ 它们报的 `15/0` 与 `7/0` 同样可能是假绿。**下一个计划的第一件事就该是逐个补 build 脚本 + 移进"先建再跑"，并给它们各自的真结果。** 在此之前，**不要把它们的绿当证据**。
- **`test_noise_probe` 在全量 `run_tests.bat` 里从来没跑成过** —— 根因不是"环境问题"，而是**同一 cmd 会话里连续 `call vcvarsall` 让 PATH 累积超过 cmd 的 8191 上限**（实测：连调 5 次，前 4 次 OK、第 5 次失败；它正好是第 5 个消费点）。单独跑是好的 ⇒ 这个洞一直没被发现。Task 4 在自己的脚本上用 `if not defined VCINSTALLDIR call ...` 绕开了；**`run_tests.bat` 里其它脚本还没有这层保护**，所以"全量跑一遍"这件事本身仍不完全可信。
- **安全开关要不要恢复**（虚拟约束力 / 工作空间钳位 / 垂直触觉轴）—— 用户现场决定，且恢复前要先定量级。
