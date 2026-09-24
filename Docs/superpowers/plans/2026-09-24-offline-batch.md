# 离线批（测试床三根刺 + 两处可测性接缝）实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把三根测试床的刺与两处"只有人工审读守着"的接缝变成可执行的断言，并把 `test_payload_calibration` 里 74 条从不执行的断言接进测试床。

**Architecture:** 五项互相独立，各改一处、各带判据与负对照。时间相关的两处（`ForceTuning::tick`、`EscalationTracker` 的用例）统一走"把时间做成输入"，不再靠 `Sleep` 拉余量；`RG|` 限频的判决从 `RelayCore.cpp` 里抽成纯函数（`RelayCore.cpp` 不被任何测试编译）；`test_payload_calibration` 用 `#ifdef` 拆成两个 exe（零重复代码），只把其中一条按设计红着的用例隔离出去并**每次运行点名**。

**Tech Stack:** C++17 / MSVC 19.44 (`cl /EHsc /std:c++17`)、`cmd` 批处理测试床、无第三方库。

**Spec:** `Docs/superpowers/specs/2026-09-24-offline-batch-design.md`（本计划的前置，含全部实测事实）

## Global Constraints

- **仓库根** `D:\Projects\Touch`；**测试床工作目录** `Touch_Client/tests`。
- **分支** `fix/offline-testbed-seams`（已建）。**禁止** `git push`、**禁止**切到/改动 `master`。
- **从 bash 调 `.bat` 的唯一正确姿势**（两个坑都要躲）：
  `MSYS_NO_PATHCONV=1 cmd.exe /c ".\<script>.bat"`
  - 少了 `MSYS_NO_PATHCONV=1`：MSYS 会把 `/c` 当成路径转换掉，cmd 起成交互式 ⇒ 脚本**根本没跑**。
  - 少了 `.\`：`NoDefaultCurrentDirectoryInExePath` 让 cmd 不在当前目录找脚本 ⇒ "not recognized"。
  - 这两种失败都**不报错**（看起来像跑过了）⇒ 每次跑完必须**核产物时间戳或日志内容**，别只看退出码。
- **`.bat` 文件必须纯 ASCII**（非 ASCII 注释在非 UTF-8 代码页下会被 cmd 误解码并吞掉下一行）。
- **基线（本批动手前实测）**：整床 `Suites accounted: 24 of 24 (ran 22 + not-run 2)`，**exit 0**；
  `test_payload_calibration` 单独构建后 `74 passed, 1 failed`、exit 1。
- **证据纪律**：
  - 每条判据都要**跑出来**，输出抄进提交信息或文档，不许写"应该会绿"。
  - 负对照必须打在**实现**上，不许打在**常数**上（常数参数化的用例对改常数免疫）。
  - 还原类操作（临时改实现/改源码做负对照）必须先用**文件哈希**核对还原前后一致。
  - 单次绿证明不了一条红的真伪；单次红也可能是 flake（本项目实测过 ≈8% 的时序 flake）。
- 每个 Task 结束**必须提交**；提交信息用中文，与仓库现有风格一致。

---

### Task 1: `ForceTuning::tick()` 的时间接缝

**Files:**
- Modify: `Touch_Client/force/ForceTuning.h:46-48`
- Modify: `Touch_Client/force/ForceTuning.cpp:118-133`
- Test: `Touch_Client/tests/test_force_tuning.cpp`（在第 146 行之后加两条用例 + 在第 160 行附近加调用）
- 构建脚本已存在、已接线：`Touch_Client/tests/build_force_tuning_test.bat`（链接 `..\force\ForceTuning.cpp`）

**Interfaces:**
- Produces: `void ForceTuning::tickAt(unsigned long nowMs)` —— `tick()` 变成它的薄包装。
- 语义：`nowMs` 与 `setGain` 记下的 `s_dirtyMs` 比较，差 `>= TUNING_DEBOUNCE_MS` 才落盘。

**为什么不做"只加测试不动实现"**：`tick()` 内联 `GetTickCount()`，测试无法在不睡 1 秒的情况下驱动防抖；不接受把 1 秒睡眠写进测试（那正是 Task 3 要清掉的东西）。

- [ ] **Step 1: 写失败用例（C++ 里"红"= 编译失败）**

在 `Touch_Client/tests/test_force_tuning.cpp` 的 `test_corrupt_file_rejected()` 之后（第 146 行后）插入：

```cpp
// ===== tick(): 防抖落盘的【生产路径】—— 时间作为输入 (2026-09-24) =====
//
// 【为什么要有它】: `TUNING_DEBOUNCE_MS` 那 1 秒防抖是"跨重启保留"的【唯一】实现 ——
//   `tick()` 是唯一的落盘路径 (RelayCore::pollRelayCommands 每帧调它)。
//   而本文件此前 12 条用例【没有一条碰过 tick()】: 规格的验收行「落盘 → 读回」只经由
//   `saveToFile` / `loadFromFile` 那对显式函数验证过 ⇒ 生产走的那条路一条断言都没有。
// 【时间怎么进来】: `tickAt(nowMs)` 是接缝, `tick()` 在它外面包一层 GetTickCount()。
//   基准 `t0` 取 `GetTickCount()`: `setGain` 刚刚返回, 它的 `s_dirtyMs` 与 t0 只差几微秒
//   ⇒ t0 之后的偏移就是 s_dirtyMs 之后的偏移 (窗口 1000ms 对几微秒的误差免疫)。
// ⚠ 两条用例都【自己建立前置状态】(先 setGain 把静默期起点钉在 t0) —— 不依赖 main() 里的
//   调用顺序, 也不依赖 s_dirty 在进入本用例时是什么值 (它是 file-static, 会跨用例残留)。
static bool fileExistsAt(const char* path) {
    FILE* f = fopen(path, "r");
    if (!f) return false;
    fclose(f);
    return true;
}

static void test_tick_at_debounces_persistence() {
    TEST(tick_at_debounces_persistence);

    remove(kTmp);
    const unsigned long t0 = GetTickCount();
    CHECK(ForceTuning::setGain(150.0));       // 标脏, 并把静默期起点钉在 t0

    ForceTuning::tickAt(t0);                  // 立刻: 还在静默期
    CHECK(!fileExistsAt(kTmp));
    ForceTuning::tickAt(t0 + 999);            // 差 1ms: 仍然不写
    CHECK(!fileExistsAt(kTmp));
    ForceTuning::tickAt(t0 + 1000);           // 到点: 写
    CHECK(fileExistsAt(kTmp));

    double v = 0.0;                           // 写的必须是【目标值】
    CHECK(ForceTuning::loadFromFile(kTmp, &v));
    CHECK(fabs(v - 150.0) < 1e-9);

    remove(kTmp);
    PASS();
}

static void test_tick_at_does_not_rewrite_when_clean() {
    TEST(tick_at_does_not_rewrite_when_clean);

    remove(kTmp);
    const unsigned long t0 = GetTickCount();
    CHECK(ForceTuning::setGain(160.0));
    ForceTuning::tickAt(t0 + 1000);           // 写第一次
    CHECK(fileExistsAt(kTmp));

    remove(kTmp);                             // 删掉: 若下面又写, 文件会重新出现
    ForceTuning::tickAt(t0 + 5000);           // 已经不脏了 ⇒ 什么都不做
    CHECK(!fileExistsAt(kTmp));               // ★ "不是每次都重写"的判据

    PASS();
}
```

在 `main()` 里 `test_corrupt_file_rejected();` 之后加两行：

```cpp
    test_tick_at_debounces_persistence();
    test_tick_at_does_not_rewrite_when_clean();
```

- [ ] **Step 2: 构建，确认它失败（未声明 `tickAt`）**

```bash
cd /d/Projects/Touch/Touch_Client/tests && MSYS_NO_PATHCONV=1 cmd.exe /c ".\build_force_tuning_test.bat" 2>&1 | grep -E "error C2039|BUILD_EXIT"
```

Expected: `error C2039: "tickAt": 不是 "ForceTuning" 的成员`（或等价），`BUILD_EXIT=2`。

- [ ] **Step 3: 实现（最小）**

`Touch_Client/force/ForceTuning.h`，把第 46-48 行那段：

```cpp
    // 防抖落盘: 值变过 且 距上次改动 ≥ TUNING_DEBOUNCE_MS 才写。
    // 由 RelayCore::pollRelayCommands() 每帧调用 (借现成的空闲循环当心跳, 不新起线程)。
    void tick();
```

替换成：

```cpp
    // 防抖落盘: 值变过 且 距上次改动 ≥ TUNING_DEBOUNCE_MS 才写。
    // 由 RelayCore::pollRelayCommands() 每帧调用 (借现成的空闲循环当心跳, 不新起线程)。
    void tick();

    // 同上, 但把"现在"作为【输入】—— 接缝, 只给测试用。
    //   `tick()` 就是 `tickAt(GetTickCount())`; 生产路径一个字没变。
    //   【为什么需要它】: 1 秒防抖是落盘的唯一实现, 而它此前【一条断言都没有】——
    //   要在测试里驱动它, 不注入时间就只能睡 1 秒 (本项目拒绝把睡眠写进测试)。
    //   nowMs 与 setGain 记下的时刻比较; 差 >= TUNING_DEBOUNCE_MS 才写。
    void tickAt(unsigned long nowMs);
```

`Touch_Client/force/ForceTuning.cpp`，把 `void tick() { ... }`（第 118-133 行）整段替换成：

```cpp
void tick() {
    tickAt(GetTickCount());
}

void tickAt(unsigned long nowMs) {
    if (!s_dirty) return;
    if ((nowMs - s_dirtyMs) < TUNING_DEBOUNCE_MS) return;   // 还在动, 再等等

    const double v = s_gain.load();
    const char* path = CalibStore::fileFor("force_tuning.json");
    if (saveToFile(path, v)) {
        printf("[Tuning] 力反射增益已落盘: %.1f\n", v);
    } else {
        // 落盘失败必须出声 —— 否则"我调好了"与"下次开机没了"之间没有任何提示。
        fprintf(stderr, "[Tuning] !! 力反射增益【落盘失败】: %s —— 本次会话有效, 重启会丢\n", path);
        fflush(stderr);
    }
    s_dirty = false;
}
```

⚠ `s_dirtyMs` 是 `DWORD`（32 位无符号），参数是 `unsigned long`。Windows 上两者同为 32 位 ⇒ 相减的环绕语义与原来逐字一致（`s_dirtyMs` 可能比 `nowMs` "大"仅当环绕，原式也是这么算的）。

- [ ] **Step 4: 构建并运行，确认通过**

```bash
cd /d/Projects/Touch/Touch_Client/tests && MSYS_NO_PATHCONV=1 cmd.exe /c ".\build_force_tuning_test.bat" 2>&1 | tail -2 && ./test_force_tuning.exe
```

Expected: `BUILD_EXIT=0`，然后 `13 passed, 0 failed`（原 11 条 + 新 2 条）。

- [ ] **Step 5: 负对照（打在实现上）**

把 `ForceTuning.cpp` 里 `tickAt` 的 `if ((nowMs - s_dirtyMs) < TUNING_DEBOUNCE_MS) return;` 这一行**临时删掉**，重建并运行。
Expected: `test_tick_at_debounces_persistence` **变红**（`tickAt(t0)` 就写了盘）。记录输出，然后**还原**并核对文件哈希：

```bash
cd /d/Projects/Touch && sha256sum Touch_Client/force/ForceTuning.cpp && git diff --stat Touch_Client/force/ForceTuning.cpp
```

还原后 `git diff` 只应剩下 Step 3 那处真实改动。

- [ ] **Step 6: 提交**

```bash
cd /d/Projects/Touch && git add Touch_Client/force/ForceTuning.h Touch_Client/force/ForceTuning.cpp Touch_Client/tests/test_force_tuning.cpp && git commit -m "test(tuning): tick() 的时间接缝 + 落盘路径的第一组用例

1 秒防抖是落盘的【唯一】实现（RelayCore::pollRelayCommands 每帧调 tick），
而 test_force_tuning 此前 12 条用例没有一条碰过它 —— 规格的验收行
「落盘 → 读回」只经由 saveToFile/loadFromFile 那对显式函数验证过。

tickAt(nowMs) 把\"现在\"做成输入，tick() = tickAt(GetTickCount())，生产路径一字未变。
两条用例各自建立前置状态（先 setGain 把静默期起点钉住），不依赖用例顺序。
负对照实测：删掉窗口判断 ⇒ 第 1 条当场变红。"
```

---

### Task 2: `RG|` 回读限频的判决抽成纯函数

**Files:**
- Create: `Touch_Client/relay/GainReadbackPolicy.h`
- Create: `Touch_Client/tests/test_gain_readback_policy.cpp`
- Create: `Touch_Client/tests/build_gain_readback_policy_test.bat`
- Modify: `Touch_Client/relay/RelayCore.cpp:2505-2528`
- Modify: `Touch_Client/tests/run_tests.bat`（在 `:633` 之后、"Tests complete" 之前插入一段）

**Interfaces:**
- Produces: `GainReadbackPolicy::GainReport gainReportDecision(bool force, double g, double lastSentGain, unsigned long nowMs, unsigned long lastReportMs, unsigned long windowMs)` 与 `constexpr unsigned long GainReadbackPolicy::GAIN_REPORT_MIN_INTERVAL_MS = 100;`
- 返回 `GainReport::Send` / `SkipUnchanged` / `SkipTooSoon`；`RelayCore` 按它决定"发 / 清待发标志 / 置待发标志"。

⚠ **头文件只用 `inline` 实现**（无 `.cpp`）—— 与 `JitterStats.h` / `EscalationTracker.h` 同一先例，测试只需编译自己一个文件。

- [ ] **Step 1: 写失败用例**

创建 `Touch_Client/tests/test_gain_readback_policy.cpp`：

```cpp
// Standalone test: GainReadbackPolicy —— RG| 回读限频的判决 (纯函数)
// Build: build_gain_readback_policy_test.bat
// Run:   test_gain_readback_policy.exe
//
// 【为什么把它抽出来】: 这段判决从前内联在 RelayCore::sendReflectionGain 里, 而
//   RelayCore.cpp 不被任何测试编译 ⇒ 它【一条自动化用例都没有】。代价已经付过一次:
//   一轮修复把 dispatch 传成 force=true, 限频【静默死掉】, 测试床一声不响。
// 【本文件只测判决, 不测接线】: 调用点仍未被任何测试覆盖 (那半只能靠上机)。

#include <iostream>
#include <cmath>
#include "../relay/GainReadbackPolicy.h"

using GainReadbackPolicy::GainReport;
using GainReadbackPolicy::gainReportDecision;
static const unsigned long W = GainReadbackPolicy::GAIN_REPORT_MIN_INTERVAL_MS;

static int g_passed = 0, g_failed = 0;

#define TEST(name) do { std::cout << "  " << #name << "... "; } while(0)
#define PASS() do { std::cout << "PASS" << std::endl; g_passed++; } while(0)
#define CHECK(cond) do { if (!(cond)) { std::cout << "FAIL: " << #cond << std::endl; g_failed++; return; } } while(0)

// ===== 第 1 格: force=true 恒发送 =====
// ★ 这一格【真的出过事故】(2026-09-22: 传成 force=true 让限频静默死掉; 后来反向又差点
//   把 force 参与进"值没变"的判断 ⇒ 被拒的回读一个字节都发不出去)。
//   三个子格把"force 必须【第一道】短路"钉死: 值没变也要发、0ms 也要发、值变了更要发。
static void test_force_true_always_sends() {
    TEST(force_true_always_sends);
    CHECK(gainReportDecision(true, 120.0, 120.0, 1000,  999, W) == GainReport::Send); // 值没变
    CHECK(gainReportDecision(true, 120.0, 120.0, 1000, 1000, W) == GainReport::Send); // 距上次 0ms
    CHECK(gainReportDecision(true, 130.0, 120.0, 5000, 1000, W) == GainReport::Send); // 值变了且过窗口
    PASS();
}

// ===== 第 2 格: 值没变 ⇒ 不发 (拖动洪水里的绝大多数) =====
static void test_unchanged_value_is_skipped() {
    TEST(unchanged_value_is_skipped);
    CHECK(gainReportDecision(false, 120.0, 120.0, 1000, 1000, W) == GainReport::SkipUnchanged);
    CHECK(gainReportDecision(false, 120.0, 120.0, 999999, 1, W) == GainReport::SkipUnchanged);
    PASS();
}

// ===== 第 3 格: 值变了但距上次发送不足窗口 ⇒ 记待发 =====
static void test_too_soon_is_skipped() {
    TEST(too_soon_is_skipped);
    CHECK(gainReportDecision(false, 130.0, 120.0, 1050, 1000, W) == GainReport::SkipTooSoon);
    PASS();
}

// ===== 第 4 格: 窗口的开关边界 =====
// ★ 边界必须【写死在一侧】: 实现里是 `(now - last) < window ⇒ 挡`, 所以
//   差 == window 时【放行】。这一格就是那句话的判据 —— 改 `<` 为 `<=` 它当场红。
static void test_window_boundary() {
    TEST(window_boundary);
    CHECK(gainReportDecision(false, 130.0, 120.0, 1000 + W - 1, 1000, W) == GainReport::SkipTooSoon);
    CHECK(gainReportDecision(false, 130.0, 120.0, 1000 + W,     1000, W) == GainReport::Send);
    PASS();
}

// ===== 第 5 格: 32 位时钟环绕 =====
// s_lastGainReportMs 是 DWORD ⇒ 开机 49.7 天后会绕回来。无符号相减天然正确,
// 这一格把那个语义钉住 (换成有符号比较就是另一回事了)。
static void test_clock_wraparound() {
    TEST(clock_wraparound);
    // last = 0xFFFFFFF0, now = 0x00000200 ⇒ 实际过去了 0x210 = 528ms ≥ 100 ⇒ 发
    CHECK(gainReportDecision(false, 130.0, 120.0, 0x00000200u, 0xFFFFFFF0u, W) == GainReport::Send);
    // last = 0xFFFFFFF0, now = 0x00000050 ⇒ 实际过去了 0x60 = 96ms < 100 ⇒ 挡
    CHECK(gainReportDecision(false, 130.0, 120.0, 0x00000050u, 0xFFFFFFF0u, W) == GainReport::SkipTooSoon);
    PASS();
}

int main() {
    std::cout << "=== GainReadbackPolicy Tests ===" << std::endl;
    test_force_true_always_sends();
    test_unchanged_value_is_skipped();
    test_too_soon_is_skipped();
    test_window_boundary();
    test_clock_wraparound();
    std::cout << std::endl;
    std::cout << g_passed << " passed, " << g_failed << " failed" << std::endl;
    return g_failed > 0 ? 1 : 0;
}
```

- [ ] **Step 2: 写构建脚本，跑一次确认失败（头文件还不存在）**

创建 `Touch_Client/tests/build_gain_readback_policy_test.bat`（**纯 ASCII**；照 `build_jitter_stats_test.bat` 的形状，那份就是"纯头文件套件"的先例）：

```bat
@echo on
rem 2026-09-22: guard the vcvarsall call -- PATH grows ~1350 chars per call and cmd dies at ~8191,
rem   so a second unguarded call in the same session can abort the rest of run_tests.bat.
rem   With this guard, vcvarsall runs at most once per cmd session. (ASCII only -- see
rem   build_force_pipeline_test.bat's note about non-ASCII comments in .bat files.)
if not defined VCINSTALLDIR call "D:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvarsall.bat" x64
cd /d "D:\Projects\Touch\Touch_Client\tests"
rem GainReadbackPolicy.h is a PURE header (inline implementation, no .cpp to link) -- same shape
rem   as JitterStats.h. Nothing else is linked on purpose; if this suite ever needs a second
rem   translation unit, that means the policy stopped being pure.
cl /EHsc /std:c++17 /DWIN32 /DWIN32_LEAN_AND_MEAN /DNOMINMAX /D_CRT_SECURE_NO_WARNINGS test_gain_readback_policy.cpp /Fe:test_gain_readback_policy.exe
echo BUILD_EXIT=%ERRORLEVEL%
```

```bash
cd /d/Projects/Touch/Touch_Client/tests && MSYS_NO_PATHCONV=1 cmd.exe /c ".\build_gain_readback_policy_test.bat" 2>&1 | grep -E "error C1083|BUILD_EXIT"
```

Expected: `fatal error C1083: 无法打开包括文件: "GainReadbackPolicy.h"`，`BUILD_EXIT=2`。

- [ ] **Step 3: 实现头文件**

创建 `Touch_Client/relay/GainReadbackPolicy.h`：

```cpp
#pragma once

// RG| 回读的【限频判决】—— 纯函数, 不读时钟、不碰 socket、不写 static。
//
// 【出处】: 2026-09-22 的"力反射增益 MATLAB 可调"计划。判决从前内联在
//   RelayCore::sendReflectionGain 里, 而 RelayCore.cpp 不被任何测试编译 ⇒ 零自动化用例。
//   抽出来的理由是一次真实事故: 限频被误传成 force=true 而【静默死掉】, 测试床没响。
//
// 【规格】: 只在目标值真的变了、且距上次回读 ≥ GAIN_REPORT_MIN_INTERVAL_MS 时才发。
//   两条例外由 force=true 表达 —— 【force 必须是第一道】, 理由见下面两段注释。
namespace GainReadbackPolicy {

enum class GainReport {
    Send,           // 现在就发
    SkipUnchanged,  // 目标值相对【上次真的发出去的那条】没变 ⇒ 没有可报的东西
    SkipTooSoon,    // 变了, 但距上次【发送】不足窗口 ⇒ 记下待发, 由 pollRelayCommands 补
};

// 距上次【发送】的最小间隔 (ms)。
// ⚠ 语义是"上次真的发出去的时刻", 不是"上次被挡下的时刻": 若把被挡下的时刻也记进去,
//   拖动期间每一条命令都会把期限往后推 ⇒ 只要命令不停就永远发不出去, 限频变成饥饿。
constexpr unsigned long GAIN_REPORT_MIN_INTERVAL_MS = 100;

// ⚠ force 必须是【第一道短路】, 不许挪到值判定之后:
//   · force=true 有三个用途 —— 连接/重连时的强制回读, 以及【被拒绝的增益改动】。
//   · 被拒 = setGain 在任何 store 之前就返回 ⇒ 生效值【按构造】没变 ⇒ "值没变"那道闸
//     必然命中 ⇒ 一个字节都发不出去。偏偏那一条正是 MATLAB 最需要的: 它的滑条已经动了,
//     正等着被纠正回真值。
//   ⇒ 谁把 force 塞进"值没变"的判断里, test_gain_readback_policy 的第 1 格当场变红。
inline GainReport gainReportDecision(bool force, double g, double lastSentGain,
                                     unsigned long nowMs, unsigned long lastReportMs,
                                     unsigned long windowMs) {
    if (force) return GainReport::Send;
    if (g == lastSentGain) return GainReport::SkipUnchanged;
    // 无符号相减: 32 位时钟环绕时天然正确 (见用例 test_clock_wraparound)。
    if ((nowMs - lastReportMs) < windowMs) return GainReport::SkipTooSoon;
    return GainReport::Send;
}

} // namespace GainReadbackPolicy
```

- [ ] **Step 4: 构建并运行，确认通过**

```bash
cd /d/Projects/Touch/Touch_Client/tests && MSYS_NO_PATHCONV=1 cmd.exe /c ".\build_gain_readback_policy_test.bat" 2>&1 | tail -2 && ./test_gain_readback_policy.exe
```

Expected: `BUILD_EXIT=0`，然后 `5 passed, 0 failed`，exit 0。

- [ ] **Step 5: 负对照（打在实现上）**

把 `GainReadbackPolicy.h` 里的 `if (force) return GainReport::Send;` **临时删掉**，重建运行。
Expected: `test_force_true_always_sends` **变红**（值没变那一格返回 `SkipUnchanged`）。
再把它改成在值判定**之后**（`if (g == lastSentGain) ...` 之后）跑一次，也应红。
记录输出后还原，并用 `sha256sum` 核对。

- [ ] **Step 6: 把 `RelayCore` 接上去**

`Touch_Client/relay/RelayCore.cpp`：在文件顶部的 `#include` 区（与其它 `relay/` 头文件放一起）加：

```cpp
#include "GainReadbackPolicy.h"
```

把 `RelayCore::sendReflectionGain`（第 2505-2528 行）里 `if (!force) { ... }` 那一整段（含它内部两道闸与 `s_gainReportPending` 的两处赋值）替换成：

```cpp
    // 判决【不在本文件里】—— 抽成了纯函数 (relay/GainReadbackPolicy.h), 理由与
    //   force 必须第一道的说明都写在那里, 并由 test_gain_readback_policy 钉住。
    //   本文件只负责: 取值、传参、按判决改这三样状态 (sent 值 / 上次发送时刻 / 待发标志)。
    switch (GainReadbackPolicy::gainReportDecision(
                force, g, s_lastSentGain.load(), now,
                s_lastGainReportMs.load(), GainReadbackPolicy::GAIN_REPORT_MIN_INTERVAL_MS)) {
    case GainReadbackPolicy::GainReport::SkipUnchanged:
        // ⚠ 必须【顺手清掉待发标志】: 一条被限频挡下的 A→B 之后值又变回 A, 此时"待发"已
        //   无事可做; 留着标志会让 pollRelayCommands 每帧都调进来、每帧都从这里返回 ⇒
        //   标志卡在 true 再也不动 (无害, 但那个标志从此失去意义)。
        s_gainReportPending.store(false);
        return;
    case GainReadbackPolicy::GainReport::SkipTooSoon:
        // 记下待发, 由 pollRelayCommands 补 —— 最后一条不丢。
        // ⚠ 这里【不】更新 s_lastGainReportMs: 它记的是"上次真的发出去"的时刻 (见头文件)。
        s_gainReportPending.store(true);
        return;
    case GainReadbackPolicy::GainReport::Send:
        break;   // 落到下面发送
    }
```

⚠ 原来那两段解释"为什么拒绝必须走 force=true"的长注释（第 2487-2500 行、`:2516-2524`、`:2585-2598`）**保持原样**，并在第 2585 段里补一句指向新位置：

```
//   （判决本身已抽到 relay/GainReadbackPolicy.h, 由 test_gain_readback_policy 钉住。）
```

- [ ] **Step 7: 构建客户端，确认无编译错误**

```bash
cd /d/Projects/Touch/Touch_Client && MSYS_NO_PATHCONV=1 cmd.exe /c ".\build.bat" 2>&1 | tail -20
```

Expected: 无 `error C...`，且末尾打出 `Build complete. Run: ...\Touch_Client.exe`。
⚠ 这一步会重编客户端 exe ⇒ **下次上机前必须重建**是既成事实（`BTN2_J4_SIGN` 本来也要重建）。

- [ ] **Step 8: 接进测试床**

`Touch_Client/tests/run_tests.bat`：在第 633 行（`test_button2_joint` 那段的收尾 `echo.`）之后、"Tests complete" 之前插入：

```bat
rem ============================================================
rem Suite wired into "build then run" on 2026-09-24:
rem   gain_readback_policy -- the pure decision function behind RG| readback rate
rem   limiting (relay/GainReadbackPolicy.h). It was extracted out of RelayCore.cpp,
rem   which no test compiles, after the limiter was once silently disabled by a
rem   mis-passed force=true argument -- a defect the test bench did not notice.
rem   Only the DECISION is covered here; the call site in RelayCore.cpp still has
rem   no automated coverage.
rem   The assertion count is deliberately NOT copied here: the suite prints it on
rem   every run, and a hand-typed copy only goes stale in the silent direction
rem   (see the note on hand-typed numbers in the NOT RUN block at the bottom).
rem   Adding this .cpp is what makes the runtime suite count on disk move
rem   from 24 to 25; if this section is ever deleted while the .cpp stays,
rem   the harness asserts at the bottom and exits 1 -- by design.
rem   The FORM of the result check is NOT special to this suite -- every
rem   section in this file uses it. See "HOW TEST RESULTS ARE JUDGED" at
rem   the top of this file for the rule and the two tempting-but-wrong forms.
rem ============================================================

echo --- Building test_gain_readback_policy ---
call "%TESTDIR%\build_gain_readback_policy_test.bat"
@echo off
if %ERRORLEVEL% EQU 0 (
    echo   Build OK
    echo.
    echo === test_gain_readback_policy.exe ===
    "%TESTDIR%\test_gain_readback_policy.exe"
    if !ERRORLEVEL! EQU 0 (
        set /a PASSED+=1
        echo   [OK]
    ) else (
        set /a FAILED+=1
        echo   [FAIL]
    )
) else (
    echo   [FAIL: build error]
    set /a FAILED+=1
)
echo.
```

- [ ] **Step 9: 跑整床，确认计数与绿**

```bash
cd /d/Projects/Touch/Touch_Client/tests && MSYS_NO_PATHCONV=1 cmd.exe /c ".\run_tests.bat" > _harness_task2.log 2>&1; echo "exit=$?"; grep -E "Suites accounted|test_gain_readback_policy|\[FAIL" _harness_task2.log | tail -8
```

Expected: `Suites accounted: 25 of 25 (ran 23 + not-run 2)`、该套件 `[OK]`、**`exit=0`**。
⚠ 若计数不符，**先看日志里那三行 MISMATCH**，不要改数字迁就。

- [ ] **Step 10: 提交**

```bash
cd /d/Projects/Touch && git add Touch_Client/relay/GainReadbackPolicy.h Touch_Client/relay/RelayCore.cpp Touch_Client/tests/test_gain_readback_policy.cpp Touch_Client/tests/build_gain_readback_policy_test.bat Touch_Client/tests/run_tests.bat && git commit -m "refactor(relay): RG| 回读限频的判决抽成纯函数 + 5 格用例

判决从前内联在 RelayCore::sendReflectionGain，而 RelayCore.cpp 不被任何测试编译
⇒ 零自动化用例。代价已付过一次：限频被误传 force=true 而静默死掉，测试床没响。

GainReadbackPolicy.h 是纯头文件（inline，同 JitterStats.h 的先例），
5 格含 force 短路、值未变、窗口内、窗口开关边界、32 位时钟环绕。
负对照实测：删掉 force 短路 ⇒ 第 1 格当场变红。

RelayCore 只保留取值/传参/改三样状态；调用点仍无自动化覆盖（那半要上机）。
测试床 24 -> 25，实测 Suites accounted: 25 of 25 (ran 23 + not-run 2)，exit 0。"
```

---

### Task 3: `test_safety_core` 确定化 + `MIN_WARN_MS` 两侧边界

**Files:**
- Modify: `Touch_Client/tests/test_safety_core.cpp:30-38`（删 `kSettleMs`）、`:64`、`:124`、`:154`、`:183`、`:221`（5 处 `Sleep`）、`:255-268`（main 加调用）
- 构建脚本已存在：`Touch_Client/tests/build_safety_core_test.bat`

**Interfaces:**
- Consumes: `RobotStateMachine::escalation()`（`safety/RobotStateMachine.h:63`，**public**，返回可写引用）、`EscalationTracker::m_firstErrorMs`（public 字段）。

- [ ] **Step 1: 先记下"改之前"的实测基线（防 flake 误判）**

```bash
cd /d/Projects/Touch/Touch_Client/tests && fails=0; for i in $(seq 1 60); do ./test_safety_core.exe >/dev/null 2>&1 || fails=$((fails+1)); done; echo "before: red=$fails / 60"
```

Expected: `red=0 / 60`（2026-09-24 实测过 0/180 —— 那条 ≈8% 的 flake 在 200ms 余量下**已不存在**）。
若这里出现非零，**停**：说明有另一条真 flake，先查它再往下做。

- [ ] **Step 2: 写失败用例（新增边界用例，先让它红）**

在 `Touch_Client/tests/test_safety_core.cpp` 的 `test_deescalation_clear_frames()` 之后（第 253 行后）插入：

```cpp
// ===== Test 9: MIN_WARN_MS 的【两侧边界】=====
//
// 【为什么必须有它】: 本文件从前靠"睡 4 × MIN_WARN_MS = 200ms"去满足 50ms 的规则 ——
//   余量 4 倍 ⇒ 判据对 MIN_WARN_MS ∈ (0, 200] 全都不敏感, 改到 150 也照样绿。
//   下面两条把阈值本身钉到 1ms: 差 1ms 不升级, 正好到点升级。
// 【怎么做到不用睡】: 直接注入"距第一次出错过去了多久"——
//   注入的是【差】, 与 GetTickCount 的 15.6ms 粒度无关。
static void test_escalation_boundary_at_min_warn_ms() {
    TEST(escalation_boundary_at_min_warn_ms);

    // 本用例的算术假设: MIN_WARN_MS >= 2 才谈得上"差 1ms"这一侧。
    CHECK(Config::MIN_WARN_MS > 1);

    Vec3 delta = {1, 0, 0};
    EscalationTracker et;
    et.recordError(RobotErrorCode::ERR_CYLINDRICAL_WARN, delta);
    et.recordError(RobotErrorCode::ERR_CYLINDRICAL_WARN, delta);
    et.recordError(RobotErrorCode::ERR_CYLINDRICAL_WARN, delta);
    CHECK(et.count() == 3);                       // 帧数够了, 只差时间

    et.m_firstErrorMs -= (Config::MIN_WARN_MS - 1);   // 差 1ms
    CHECK(!et.shouldEscalate());                      // ★ 不升级 (下侧)

    et.m_firstErrorMs -= 1;                           // 正好到点
    CHECK(et.shouldEscalate());                       // ★ 升级 (阈值本身)

    PASS();
}
```

在 `main()` 末尾 `test_deescalation_clear_frames();` 之后加：

```cpp
    test_escalation_boundary_at_min_warn_ms();
```

⚠ 此刻它会**假绿**：`shouldEscalate` 的实现与用例写的是同一个阈值，用例必然通过。**这不是真绿** —— Step 3 用负对照把它变成真判据。

- [ ] **Step 3: 负对照（打在实现上，证明这条用例真的在判那个比较）**

把 `Touch_Client/safety/EscalationTracker.h:42` 的
`&& elapsed >= Config::MIN_WARN_MS)` **临时**改成 `&& elapsed > Config::MIN_WARN_MS)`：

```bash
cd /d/Projects/Touch/Touch_Client/tests && MSYS_NO_PATHCONV=1 cmd.exe /c ".\build_safety_core_test.bat" 2>&1 | tail -2 && ./test_safety_core.exe; echo "exit=$?"
```

Expected: `test_escalation_boundary_at_min_warn_ms` **FAIL**（第二条 `CHECK(et.shouldEscalate())` 挂在 50 == 50 上），整床 `8 passed, 1 failed`、exit 1。
若它**不红**，说明这条用例没有判别力 —— 停下查原因，别往下走。

记录输出后**还原**该文件，`sha256sum` 核对。

- [ ] **Step 4: 删掉 5 处 `Sleep`，换成时间注入**

把第 30-38 行（`Sleep(60)` 那段解释 + `kSettleMs` 定义）整体换成：

```cpp
// ★★ 2026-09-24: 时间【注入】, 不睡觉。
//
// 【为什么不再 Sleep】: 判据是 `elapsed >= Config::MIN_WARN_MS`(=50ms)，而 elapsed 由
//   `GetTickCount()` 得到 —— 它的粒度是 15.6ms。从前本文件靠"睡 4 × MIN_WARN_MS = 200ms"
//   把余量拉开，代价是 (a) 每次运行白花 ~1s，(b) 判据对 MIN_WARN_MS ∈ (0, 200] 全都不敏感
//   —— 改到 150 也照样绿。下侧边界（差 1ms 不升级）更是无从谈起。
// 【那件事的如实记录】: 2026-09-22 记的是"实测 60 次里红 5 次（≈8%）"。2026-09-24 复核：
//   加了 200ms 余量之后**跑 180 次零红** ⇒ 那条 flake 已经不存在了。所以本轮不是"修 flake",
//   而是把时间做成【确定的输入】、并把阈值两侧补上。
// 【为什么现在能用注入】: 从前这里写着"`RobotStateMachine` 的 `m_escalation` 是私有的 ⇒
//   用不了那一招" —— **那句是假的**。`RobotStateMachine::escalation()`
//   (safety/RobotStateMachine.h:63) 在 public 段里 (private 从 66 行才开始)，返回可写引用。
// 【代价】: 这要直接写 `m_firstErrorMs` —— public 字段, 但语义上是内部量, 属【测试专用写】。
//   先例: 同目录 `test_escalation.cpp:92` 一直在这么做 (`et.m_firstErrorMs -= MIN_WARN_MS + 1`)。
```

5 处替换（`test_state_machine_transition_chain` / `test_can_move_guard` / `test_speed_factor` 走状态机；`test_escalation_warn_to_degrade` / `test_deescalation_reverse_motion` 直接构造 tracker）：

| 位置 | 原 | 改成 |
|---|---|---|
| `:64` | `Sleep(kSettleMs);  // ...` | `sm.escalation().m_firstErrorMs -= (Config::MIN_WARN_MS + 1);  // 时间注入, 见文件顶部` |
| `:124` | 同上 | 同上 |
| `:154` | 同上 | 同上 |
| `:183` | `Sleep(kSettleMs);   // ...` | `et.m_firstErrorMs -= (Config::MIN_WARN_MS + 1);   // 时间注入, 见文件顶部` |
| `:221` | `Sleep(kSettleMs);   // 见 kSettleMs 的说明` | `et.m_firstErrorMs -= (Config::MIN_WARN_MS + 1);   // 时间注入, 见文件顶部` |

⚠ 用 `MIN_WARN_MS + 1`（安全越过阈值）而不是精确的 `- 1` 算术 —— 后者只在 Step 2 那条边界用例里用，避免 `MIN_WARN_MS` 极小时出现无符号回绕。

- [ ] **Step 5: 构建并运行**

```bash
cd /d/Projects/Touch/Touch_Client/tests && MSYS_NO_PATHCONV=1 cmd.exe /c ".\build_safety_core_test.bat" 2>&1 | tail -2 && ./test_safety_core.exe; echo "exit=$?"
```

Expected: `BUILD_EXIT=0`、`9 passed, 0 failed`、`exit=0`。

- [ ] **Step 6: 跑 60 次确认零红（新的判据本身也要抗 flake）**

```bash
cd /d/Projects/Touch/Touch_Client/tests && fails=0; for i in $(seq 1 60); do ./test_safety_core.exe >/dev/null 2>&1 || fails=$((fails+1)); done; echo "after: red=$fails / 60"
```

Expected: `red=0 / 60`。

- [ ] **Step 7: 提交**

```bash
cd /d/Projects/Touch && git add Touch_Client/tests/test_safety_core.cpp && git commit -m "test(safety): 时间注入替代 5 处 Sleep + MIN_WARN_MS 两侧边界

本文件从前靠\"睡 4 x MIN_WARN_MS = 200ms\"满足 50ms 的规则，余量 4 倍
⇒ 判据对 MIN_WARN_MS in (0,200] 全都不敏感，且每次运行白花 ~1s。

那句\"m_escalation 是私有的所以用不了注入\"是假的：RobotStateMachine.h:63 的
escalation() 在 public 段（private 从 66 行才开始），返回可写引用。
先例是 test_escalation.cpp:92 一直在用的同一招。

新增边界用例把阈值钉到 1ms（差 1ms 不升级 / 正好到点升级）。
负对照实测：把 EscalationTracker.h 的 >= 改成 > ⇒ 该用例当场变红。
顺手订正一条陈旧记录：2026-09-22 记的 8% flake 在 200ms 余量下实测 180 次零红，
已不存在 —— 本轮不是修 flake（改前改后各跑 60 次，均 0 红）。"
```

---

### Task 4: `test_payload_calibration` 拆分接线 + 两条真静默截断 + 一条错文案

**Files:**
- Modify: `Touch_Client/tests/test_payload_calibration.cpp`（`main()` 分两套；`replay_real_capture` 加列数守卫；`mgParseRepeat` 改响亮拒绝；`t6ScanLastBlock` 文案分开）
- Create: `Touch_Client/tests/test_payload_calibration_parked.cpp`
- Create: `Touch_Client/tests/build_payload_calibration_parked_test.bat`
- Modify: `Touch_Client/tests/run_tests.bat`（新增 `test_payload_calibration` 段；`NOTRUN_LIST` 换一项；`[NOT RUN]` 理由段改）

**Interfaces:**
- Consumes: `t6Load` / `t6LoadAttempt` / `t6LoadFrozenCalib` / `t6LocalModel` / `t6SigmaA` / `t6AnnounceRefSource` / `MG_MAXCOLS` / `MG_COLSCAN` / `mgSplitRow` / `mgRejectColumns`（都在同文件内，`parked` 变体靠 `#include` 复用）。
- Produces: `PC_PARKED_REPLAY_ONLY` 这个编译开关；`mgParseRepeat` 的新契约（**返回 -1 = 响亮拒绝**）。

⚠ 本 Task 是全批最大的一处。**每改一段就重建跑一次**，不要攒着一起改。

- [ ] **Step 1: 记下"改之前"的基线（跑一次，抄下来）**

```bash
cd /d/Projects/Touch/Touch_Client/tests && MSYS_NO_PATHCONV=1 cmd.exe /c ".\build_payload_calibration_test.bat" 2>&1 | tail -1 && ./test_payload_calibration.exe > _pc_before.log 2>&1; echo "exit=$?"; grep -E "passed, .* failed" _pc_before.log; grep -n "FAIL" _pc_before.log
```

Expected: `BUILD_EXIT=0`、`74 passed, 1 failed`、`exit=1`、**只有一行 FAIL**（`12:38 pose 1 不是 INCONSISTENT`）。

- [ ] **Step 2: 把"列布局"那一节上移（为 Step 3 的守卫让路）**

`replay_real_capture`（`:1666`）在 `mgSplitRow`（`:2026`）**之前**，而 Step 3 要它按真实列数判。
把第 **1997-2037 行**整块（`// ===== 列布局: 认识的清单 + "看真列数的扫描上限" (2026-09-21) =====` 注释 + `MG_COLSCAN` + `MG_MAXCOLS` + `mgRejectColumns` + `mgSplitRow`）**整体剪切**，粘到 `replay_real_capture` 的注释块**之前**（即第 1580 行附近、`// =====` 那段 `replay_real_capture` 说明的前面）。

⚠ 只搬位置，**一个字符都不改**（除了必要时补一句"本节从下方上移，因为 replay_real_capture 也要按真列数判"）。

```bash
cd /d/Projects/Touch/Touch_Client/tests && MSYS_NO_PATHCONV=1 cmd.exe /c ".\build_payload_calibration_test.bat" 2>&1 | tail -1 && ./test_payload_calibration.exe 2>&1 | tail -2
```

Expected: 与 Step 1 逐字相同（`74 passed, 1 failed`）——**这一步必须是纯搬运**。

- [ ] **Step 3: `replay_real_capture` 按真列数判（堵住"更宽的行被静默收下"）**

⚠ **用内容定位，不要用行号** —— Step 2 的搬运让这一段的绝对行号变了约 +41。

先加**唯一一份**列数守卫（放在 Step 2 搬过去的 `mgSplitRow` 之后，紧挨着它）：

```cpp
// 金标夹具 (`calib_poses_2026-09-19.txt`) 的数据行必须【正好 18 列】。
//   返回 false = 已响亮拒绝 (原因已打印), 调用方不许再拿 c 里的数往下算。
// 【为什么是函数而不是在调用处内联】: 用例必须走【真的读取器】—— 本项目在这条上栽过
//   (把判据在测试里抄一遍, 于是测试绿着而真路径坏着)。有了它, 用例与 replay_real_capture
//   走的是同一段算术。
static bool goldRowColumns(const char* row, double* c, const char* file) {
    // 【先数真实列数, 再判】: sscanf 数够 18 个转换就返回 18, 第 19 列起【从不被解析】
    //   ⇒ "列数变多了"它看不见 (完整说明见 mgSplitRow 那段)。
    //   金标只对【18 列】的布局成立 ⇒ 多于 18 列 = 另一次采集 / 布局变了 ⇒ 【响亮失败】,
    //   不截断后拿它去对金标 (那会把"读到了别人的数据"变成一条绿)。
    const int nc = mgSplitRow(row, c, MG_COLSCAN);
    if (nc != 18) { mgRejectColumns(file, nc, "18 列 (这份金标夹具的布局)"); return false; }
    return true;
}
```

再把 `replay_real_capture` 里那段 `double c[18]; ... if (sscanf(q, "%lf,...,&c[17]) != 18) { n = -2; break; }`
（**按内容找**：以 `double c[18];` 开头、到那行 `n = -2;` 的 `break;` 结束）整段替换成：

```cpp
        double c[MG_COLSCAN];
        if (!goldRowColumns(q, c, used)) { n = -3; break; }
```

同时把该函数里那句 `FAIL: 采集文件读不动 (行数超上限, 或有一行的【前 18 个字段】读不出 18 个浮点) ...`
的文案补上这一支：

```cpp
        std::cout << "FAIL: 采集文件读不动 (行数超上限, 或有一行的【真实列数不是 18】, 或有一行的"
                     " 前 18 个字段读不出 18 个浮点) —— 布局变了就别拿它去对金标。" << std::endl;
```

构建运行：

```bash
cd /d/Projects/Touch/Touch_Client/tests && MSYS_NO_PATHCONV=1 cmd.exe /c ".\build_payload_calibration_test.bat" 2>&1 | tail -1 && ./test_payload_calibration.exe 2>&1 | tail -2
```

Expected: 仍是 `74 passed, 1 failed`（09-19 夹具每行都是 18 列，见下）。

- [ ] **Step 4: 给列数守卫补一条"改前必红"的用例（走真的守卫）**

在 `test_reader_rejects_fixture_wider_than_supported()` 之后插入：

```cpp
// ===== 金标夹具的读取器也必须按【真列数】判 (2026-09-24) =====
//
// 【为什么】: `replay_real_capture` 从前用 `sscanf(18 个 %lf) != 18` 当判据 —— 它只看得见
//   "列变少", 看不见"列变多"(数够 18 个就返回 18)。金标只对 18 列的布局成立, 而
//   「重采夹具」正是接下来要做的事 ⇒ 一份更宽的夹具会被静默收下, 然后拿去对一份【不属于它】的金标。
// 【本用例走【真的守卫】goldRowColumns()】, 不是把判据在本文件里抄一遍 —— 抄一遍的话
//   测试绿着而真路径坏着也能过 (本项目记过账的那类)。所以它的红/绿与那条判决是同一件事。
// 【非空证明(实测)】: 把 goldRowColumns 退回 `sscanf(...) != 18` 那版, 21 列的行会被收下
//   ⇒ 下面第一条断言当场变红。
static void test_gold_fixture_reader_rejects_wider_rows() {
    TEST(gold_fixture_reader_rejects_wider_rows);

    char row21[2048];   // 18 列布局 + 3 个多余列 (模拟"重采时多带了几列")
    mgSynthRow(row21, sizeof(row21), 21);

    double c[MG_COLSCAN];
    // ★ 走真守卫: 旧行为(sscanf 版)对它返回 true
    CHECK(!goldRowColumns(row21, c, "(synth 21 列)"));

    // 反面: 正好 18 列必须【照收】—— 否则重采的真夹具会被挡在门外
    char row18[2048];
    mgSynthRow(row18, sizeof(row18), 18);
    CHECK(goldRowColumns(row18, c, "(synth 18 列)"));
    CHECK(fabs(c[0] - 10.0) < 1e-9);    // 列值 = 10.0 + 下标 (见 mgSynthRow)
    CHECK(fabs(c[17] - 27.0) < 1e-9);

    PASS();
}
```

（`mgSynthRow(char* dst, size_t cap, int ncols)` 已存在，本 Task 不必新增任何生成器。）

并在 `main()` 的 `test_reader_rejects_fixture_wider_than_supported();` 之后加：

```cpp
    test_gold_fixture_reader_rejects_wider_rows();
```

⚠ **射程如实写在代码注释里**：这条用例覆盖的是**守卫函数** `goldRowColumns`（而
`replay_real_capture` 与它走的是同一份实现）。它**不**覆盖"`replay_real_capture` 在真夹具上仍然绿"
—— 那一条由既有用例 `test_replay_real_capture` 自己对金标的断言保证。

- [ ] **Step 5: `mgParseRepeat` 改响亮拒绝**

把 `mgParseRepeat`（`:1967-1995`）替换成：

```cpp
// "# repeat: first=1,3,5 seconds=2,4,6  (共 3 对...)" -> 0 基下标对。返回解析出的对数。
// "# repeat: none" -> 0。列号是 1 基的行号, 与 main.cpp 落盘时逐字一致。
//
// ★ 2026-09-24 【返回值 -1 = 响亮拒绝】: 从前这一支有【三处静默丢弃】——
//   (a) 单侧列表超过 16 项时循环到上限就停, 多出来的项【无声消失】;
//   (b) 解析出的对数超过调用方的缓冲 (MG_MAXREP) 时, `cnt = maxOut` 把多出来的对【无声砍掉】;
//   (c) 两侧项数不等时 `cnt = min(nf, ns)` 把多出来的【无声丢掉】。
//   三者都是"夹具说的话比我们装下的更多" ⇒ 收下就是拿【不完整】的复采对去算自由度,
//   而结果看起来完全正常。改成一律拒: 调用方 (mgLoad) 大声失败。
//   ⚠ 三处一起关的理由: 它们是【同一类缺陷】(静默丢弃数据), 分开关只会让下一个读的人
//     以为剩下那些是故意的。实测: 仓库里现存的夹具头 (3/5/8 对) 两侧项数全部相等, 无回归。
static int mgParseRepeat(const char* line, PayloadCalibration::RepeatPair* out, int maxOut) {
    int firsts[16], seconds[16], nf = 0, ns = 0;
    const char* p = strstr(line, "first=");
    if (!p) return 0;
    p += 6;
    while (*p && *p != ' ' && nf < 16) {
        char* e = nullptr;
        const long v = strtol(p, &e, 10);
        if (e == p) break;
        firsts[nf++] = (int)v;
        p = e;
        if (*p == ',') p++;
    }
    // 循环因上限而停 = 后面还有项 ⇒ (a)
    if (nf == 16 && *p == ',') return -1;
    p = strstr(line, "seconds=");
    if (!p) return 0;
    p += 8;
    while (*p && *p != ' ' && ns < 16) {
        char* e = nullptr;
        const long v = strtol(p, &e, 10);
        if (e == p) break;
        seconds[ns++] = (int)v;
        p = e;
        if (*p == ',') p++;
    }
    if (ns == 16 && *p == ',') return -1;
    if (nf != ns) return -1;        // (c)
    if (nf > maxOut) return -1;     // (b)
    for (int i = 0; i < nf; i++) { out[i].first = firsts[i] - 1; out[i].second = seconds[i] - 1; }
    return nf;
}

// 【响亮拒绝】的唯一一份打印 —— 与 mgRejectColumns 同一形状 (那边是列, 这边是复采对)。
static void mgRejectRepeat(const char* file, const char* why) {
    printf("    ★★ 读取器【拒收这一行】: 夹具 %s 的 `# repeat:` %s ⇒ 不截断后收下:"
           " 那样算出的自由度不是这份数据的自由度。\n", file, why);
}
```

`mgLoad`（`:2047-2050`）改成：

```cpp
        if (strstr(line, "# repeat:") != nullptr) {
            const int rep = mgParseRepeat(line, d.reps, MG_MAXREP);
            if (rep < 0) {
                mgRejectRepeat(pathUsed, "比本读取器能装下/能使用的更长 (或两侧项数不等)");
                fclose(fp);
                return false;
            }
            d.repCount = rep;
            continue;
        }
```

- [ ] **Step 6: 给 `mgParseRepeat` 补"改前必红"的用例**

在 `test_gold_fixture_reader_rejects_wider_rows()` 之后插入：

```cpp
// ===== `# repeat:` 行也不许静默截断 =====
// 【非空证明】: 把 mgParseRepeat 的返回值判断退回 `cnt = maxOut` 那版, 第二条断言当场变红
//   (9 对会被砍成 8 对并返回 8, 而它必须返回 -1)。
static void test_repeat_header_is_not_silently_truncated() {
    TEST(repeat_header_is_not_silently_truncated);

    PayloadCalibration::RepeatPair reps[MG_MAXREP];

    // 8 对 = 缓冲正好装得下 ⇒ 照收 (这是现存夹具的最大规模)
    const char* ok8 = "# repeat: first=1,2,3,4,6,7,8,9 seconds=2,3,4,5,7,8,9,10\n";
    CHECK(mgParseRepeat(ok8, reps, MG_MAXREP) == 8);

    // 9 对 > 缓冲 ⇒ 拒 (旧行为: 返回 8, 第 9 对无声消失)
    const char* tooMany = "# repeat: first=1,2,3,4,6,7,8,9,10 seconds=2,3,4,5,7,8,9,10,11\n";
    CHECK(mgParseRepeat(tooMany, reps, MG_MAXREP) == -1);

    // 两侧项数不等 ⇒ 拒 (旧行为: 取较小者, 多出来的丢掉)
    const char* mismatch = "# repeat: first=1,3,5 seconds=2,4,6,8\n";
    CHECK(mgParseRepeat(mismatch, reps, MG_MAXREP) == -1);

    // 单侧超过 16 项 (解析缓冲本身的上限) ⇒ 拒 (旧行为: 静默停在 16)
    const char* over16 = "# repeat: first=1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17"
                         " seconds=1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17\n";
    CHECK(mgParseRepeat(over16, reps, MG_MAXREP) == -1);

    // 没有 `# repeat:` 语义的头部照旧返回 0 (不是拒)
    CHECK(mgParseRepeat("# columns extended\n", reps, MG_MAXREP) == 0);

    PASS();
}
```

在 `main()` 里加 `test_repeat_header_is_not_silently_truncated();`。

⚠ **顺序要紧**：`mgSynthRow21` / `mgSynthRow` 与 `MG_MAXREP` 都在本 Task 之前就存在；
`MG_MAXCOLS` / `mgSplitRow` 在 Step 2 已上移到本区域之前 ⇒ 两处新用例都能看到它们。
若编译报"未声明的标识符"，**先回去核 Step 2 的搬运位置**，不要就地再定义一份。

- [ ] **Step 7: 修 `t6ScanLastBlock` 那条会误导人的文案**

把 `:3086-3098` 的 `if (blk.nc != MG_MAXCOLS) { ... }` 那一支替换成：

```cpp
    if (blk.nc > MG_MAXCOLS) {
        // ★ 2026-09-24: 从前"太宽"与"太窄"共用同一句"没有参考量那一路"—— 对一份【多带了几列】
        //   的采集, 那句话会把操作者引到反方向 (去补 @720, 而它就在那儿)。两种情况分开报。
        std::cout << "    ★★ 新采集 " << blk.path << " 的最后一块 attempt " << blk.stamp
                  << " 的数据行是 " << blk.nc << " 列, 比本读取器支持的最宽布局 ("
                  << MG_MAXCOLS << " 列) 还宽 ⇒ 【不是"少了参考量那六列", 而是多了不认识的列】。"
                  << std::endl;
        std::cout << "       ⇒ 判据那一侧会按【这一行不是 31 列】拒掉它。先去核: 记录程序是不是"
                     "又往末尾追加了新列 (那要同步改本读取器), 而不是去补 @720。" << std::endl;
        return T6FreshStatus::Broken;
    }
    if (blk.nc < MG_MAXCOLS) {
        // ⚠ 这一支【不是为了改变结论】(没有它, 下面 t6LoadAttempt 也会拒掉这一块), 而是为了
        //   把【最可能的那一种错】说准: 少了参考量那六列时, t6LoadAttempt 只能报"读不出来"
        //   并猜"截断 / 姿态太多"—— 那两句会把操作者引到错的方向 (实测过: 25 列的那一份走
        //   那一条路时打印的是"头部自报 poses=4 读不出来")。这里先按【列数】判, 点名缺了什么。
        std::cout << "    ★★ 新采集 " << blk.path << " 的最后一块 attempt " << blk.stamp
                  << " 的数据行是 " << blk.nc << " 列, 不是 " << MG_MAXCOLS << " 列 ⇒ 这一块"
                     "【少了参考量那一路 (@720 的 F720*/M720* 六列)】。" << std::endl;
        std::cout << "       ⇒ 判据那一侧没有实测值可喂, 这份数据回答不了本用例的问题, 而采集的"
                     " 目的正是它。回实机上让记录程序写满末尾那六列 (上机清单 §1 的陷阱 1)。"
                  << std::endl;
        return T6FreshStatus::Broken;
    }
```

⚠ 文案里那句 `"少了参考量那六列", 而是多了不认识的列` 含**双引号**，在 C++ 字符串里要转义：
写成 `\"少了参考量那六列\", 而是多了不认识的列`。

构建运行，Expected 仍是 `74 passed, 1 failed`（这条文案改动不触发任何用例）。

- [ ] **Step 8: 把 `main()` 拆成两套（默认 / parked）**

`test_payload_calibration.cpp` 的 `main()`（`:5313`）整体改成：

```cpp
#ifdef PC_PARKED_REPLAY_ONLY
// ===== parked 变体 (见同目录 test_payload_calibration_parked.cpp) =====
//
// 【为什么有这个变体】: 本文件里有一条【按设计红着】的用例 —— test_runtime_consistency_guard_replay
//   回放四份 2026-09-19 的采集, 那四份【没有参考量 (@720) 那一列】⇒ 闸门只能判
//   REFERENCE_UNAVAILABLE ⇒ 那条断言永远红。它红着就接不进测试床 (测试床的退出码 = FAILED 数),
//   而删掉它 = 让一条未验证的断言静默消失 (本项目一贯拒绝)。
//   ⇒ 拆成两个 exe: 默认变体跑其余全部 (接进测试床), 本变体只跑那一条。
// 【为什么它不是死代码】: `#ifdef` 只切 main(), 不切函数定义 ⇒ 默认 exe 每次运行都仍然
//   【编译】那条用例的全部代码。生产 API 改了, 默认 exe 当场编译失败 (只是不调用它)。
int main() {
    std::cout << "=== PayloadCalibration Tests (PARKED replay only) ===" << std::endl;
    std::cout << "  ⚠ 本变体【只】跑那条按设计红着的闸门回放; 它【不在】测试床上运行"
                 " (见 run_tests.bat 的 [NOT RUN] 段)。" << std::endl;
    test_runtime_consistency_guard_replay();
    std::cout << "\n" << g_passed << " passed, " << g_failed << " failed" << std::endl;
    return g_failed ? 1 : 0;
}
#else
int main() {
    // ...(现将有的全部调用原样搬进来, 唯一改动见下面那段 PARKED 披露)...
}
#endif
```

**具体做法**：把现有 `main()` 的**函数体一字不动**地搬进 `#else` 那一支，只把最后三行里的
`std::cout << "--- runtime consistency guard (replay on the four 09-19 captures) ---" << std::endl;`
与 `test_runtime_consistency_guard_replay();`**删掉**，并在紧随其后、`Task 8a` 那段
`std::cout << "--- Task 8a: pre-send gates ..."` **之前**插入下面那段 PARKED 披露：

⚠ **搬完整段后必须核"只改了该改的"**（`main()` 有 148 行，手工搬运最容易多删一行）：

```bash
cd /d/Projects/Touch && git diff --stat Touch_Client/tests/test_payload_calibration.cpp && git diff -U0 Touch_Client/tests/test_payload_calibration.cpp | grep -E "^[-+]" | grep -vE "^[-+]{3}" | head -40
```

Expected: 删掉的行**只应**是 `main()` 原签名那一行 + 那两行（`std::cout` 与调用），
加上脚本 `#ifdef`/`#else`/`#endif` 与披露块；**所有 `test_*();` 调用行都不许出现在删除侧**。
逐行看过再继续 —— 这一步错了会让某些用例**静默不跑**，而计数与全绿都看不出来。

```cpp
    // ★★ 2026-09-24 【一条被刻意隔离, 每次运行都点名】
    //   ⚠ 不许只写在源码注释里: 计数是【基数】检查不是【身份】检查, 它抓不到
    //     "这个 exe 跑了、但里面有 1 条从不执行" —— 所以必须每次打在运行输出里。
    std::cout << std::endl;
    std::cout << "  [PARKED] 1 case in this suite is deliberately NOT run here:" << std::endl;
    std::cout << "    test_runtime_consistency_guard_replay" << std::endl;
    std::cout << "      It replays the four 2026-09-19 captures, which carry no reference-column" << std::endl;
    std::cout << "      (@720) data, so the gate can only answer REFERENCE_UNAVAILABLE and its" << std::endl;
    std::cout << "      assertion is red BY DESIGN. A re-capture that carries the reference" << std::endl;
    std::cout << "      columns is what un-parks it; it cannot be fixed by editing that case." << std::endl;
    std::cout << "      It is named in run_tests.bat's NOTRUN_LIST as" << std::endl;
    std::cout << "      test_payload_calibration_parked, which builds and runs it on demand." << std::endl;
    std::cout << std::endl;
```

创建 `Touch_Client/tests/test_payload_calibration_parked.cpp`：

```cpp
// 本文件【不是一个独立的测试套件】—— 它是一个编译开关。
//
// 存在的理由: test_payload_calibration 里有一条【按设计红着】的用例
//   (test_runtime_consistency_guard_replay: 回放四份没有参考量列的 09-19 夹具),
//   它红着就接不进测试床 (测试床退出码 = FAILED 数 ⇒ 整床永远非零); 而删掉它就是把一条
//   未验证的断言【静默消失】。⇒ 拆成两个 exe: 默认变体跑其余全部 (接进测试床), 本变体只跑它。
//
// 为什么用 #include 而不是把 helper 抽成头文件: 主文件 5461 行, 那条用例依赖 t6Load /
//   t6LoadAttempt / t6LocalModel / t6SigmaA / t6AnnounceRefSource / MG_COLSCAN 等一堆
//   file-static 与常量。抽头文件要动那些 helper 的边界 (风险), 而 #include 一行就把
//   "零重复代码" 做到了。
//   ⇒ 本文件【有意】只有下面三行 —— 别把它当笔误, 也别顺手往里加第二个套件。
#define PC_PARKED_REPLAY_ONLY
#include "test_payload_calibration.cpp"
```

创建 `Touch_Client/tests/build_payload_calibration_parked_test.bat`（照 `build_payload_calibration_test.bat` 抄，只改源文件名与产物名）：

```bat
@echo on
rem 2026-09-22: guard the vcvarsall call -- PATH grows ~1350 chars per call and cmd dies at ~8191,
rem   so a second unguarded call in the same session can abort the rest of run_tests.bat.
rem   With this guard, vcvarsall runs at most once per cmd session. (ASCII only -- see
rem   build_force_pipeline_test.bat's note about non-ASCII comments in .bat files.)
if not defined VCINSTALLDIR call "D:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvarsall.bat" x64
cd /d "D:\Projects\Touch\Touch_Client\tests"
rem Builds the PARKED variant of test_payload_calibration.cpp: the one case that is red by
rem   design gets its own exe so the main suite can go green and be wired into run_tests.bat.
rem   This suite is named in NOTRUN_LIST and is therefore NOT built or run by the bench --
rem   build it by hand when you want to look at that case. Same link line as the main one.
cl /EHsc /std:c++17 /I"D:\Projects\OpenHaptics\Developer\3.5.0\include" /I"D:\Projects\OpenHaptics\Developer\3.5.0\utilities\include" /DWIN32 /DWIN32_LEAN_AND_MEAN /DNOMINMAX /D_CRT_SECURE_NO_WARNINGS /D_WINSOCK_DEPRECATED_NO_WARNINGS test_payload_calibration_parked.cpp ..\force\PayloadCalibration.cpp ..\force\ForceCompensation.cpp ..\calibration\TcpCalibration.cpp /Fe:test_payload_calibration_parked.exe
echo BUILD_EXIT=%ERRORLEVEL%
```

⚠ **包含路径必须逐字照抄 `build_payload_calibration_test.bat` 那一行**（上面写的是
`D:\Projects\OpenHaptics\...`；以现有脚本里的实际值为准 —— **先 `cat` 它，照抄，不要凭记忆写**）。

- [ ] **Step 9: 两个变体各构建运行一次**

```bash
cd /d/Projects/Touch/Touch_Client/tests && MSYS_NO_PATHCONV=1 cmd.exe /c ".\build_payload_calibration_test.bat" 2>&1 | tail -1 && ./test_payload_calibration.exe > _pc_after.log 2>&1; echo "main exit=$?"; grep -E "passed, .* failed|PARKED" _pc_after.log
```

Expected: `BUILD_EXIT=0`、`main exit=0`、`76 passed, 0 failed`（原 74 条通过 + Step 4/6 新增 2 条；
**条数以实际输出为准，抄进提交信息**，不要照抄这里的预测数）、且打出 `[PARKED] 1 case ...`。

```bash
cd /d/Projects/Touch/Touch_Client/tests && MSYS_NO_PATHCONV=1 cmd.exe /c ".\build_payload_calibration_parked_test.bat" 2>&1 | tail -1 && ./test_payload_calibration_parked.exe > _pc_parked.log 2>&1; echo "parked exit=$?"; grep -E "passed, .* failed|FAIL" _pc_parked.log
```

Expected: `BUILD_EXIT=0`、`parked exit=1`、`0 passed, 1 failed`、那行 FAIL 与 Step 1 抄下的那一行**逐字相同** —— 证明被隔离的确实是它、且它确实仍然红。

- [ ] **Step 10: 接进测试床 + 换 `NOTRUN_LIST`**

`run_tests.bat`：
1. 在第 633 行之后插入与 Task 2 Step 8 同形的一段（`build_payload_calibration_test.bat` /
   `test_payload_calibration.exe`；注释里写清"它带一条被刻意隔离的用例，每次运行会自己点名"，
   并把 "from 25 to 26" 写进去）。
2. 第 667 行改成：

```bat
set "NOTRUN_LIST=test_constraint_force test_payload_calibration_parked"
```

3. 第 743-747 行（`[NOT RUN]` 的理由段）里 `test_payload_calibration` 那一条改成
   `test_payload_calibration_parked`，并照实写明来由：

```
echo     test_payload_calibration_parked
echo       Is the PARKED variant of test_payload_calibration.cpp -- it runs ONLY the one
echo       case that is red by design (the gate replay over the four 2026-09-19 captures,
echo       which carry no reference-column data). The main suite excludes that case and is
echo       wired in above; this variant is what keeps the case NAMED instead of deleted.
echo       Build it by hand with build_payload_calibration_parked_test.bat when you want
echo       to see it. Re-capturing a fixture with the @720 columns is what un-parks it.
```

- [ ] **Step 11: 跑整床**

```bash
cd /d/Projects/Touch/Touch_Client/tests && MSYS_NO_PATHCONV=1 cmd.exe /c ".\run_tests.bat" > _harness_task4.log 2>&1; echo "exit=$?"; grep -E "Suites accounted|NOT RUN\]|test_payload_calibration|\[FAIL" _harness_task4.log | tail -10
```

Expected: `Suites accounted: 26 of 26 (ran 24 + not-run 2)`、`[NOT RUN] 2 of the 26`、**`exit=0`**。

- [ ] **Step 12: 负对照（证明接线真的在携带那 74 条）**

临时把 `main()` 的 `#ifdef PC_PARKED_REPLAY_ONLY` 分支**反转**（让默认变体也跑那条红的：
把 `#ifdef` 改成 `#ifndef`），重建跑整床：

```bash
cd /d/Projects/Touch/Touch_Client/tests && MSYS_NO_PATHCONV=1 cmd.exe /c ".\run_tests.bat" > _harness_task4_neg.log 2>&1; echo "exit=$?"; grep -E "Suites accounted|test_payload_calibration|\[FAIL" _harness_task4_neg.log | tail -6
```

Expected: 该段 `[FAIL]`、**`exit=1`**。记录后**还原**（`#ifndef` → `#ifdef`）并用 `sha256sum` 核对，
再跑一次整床确认回到 exit 0。

- [ ] **Step 13: 提交**

```bash
cd /d/Projects/Touch && git add Touch_Client/tests/test_payload_calibration.cpp Touch_Client/tests/test_payload_calibration_parked.cpp Touch_Client/tests/build_payload_calibration_parked_test.bat Touch_Client/tests/run_tests.bat && git commit -m "test(payload): 拆分接线 —— 75 条断言进测试床, 那条按设计红着的单独具名

实测(新鲜构建): 74 passed / 1 failed, 唯一红的是四份 09-19 老夹具的闸门回放 ——
那四份没有 @720 参考量列, 闸门只能判 REFERENCE_UNAVAILABLE, 而它自己的 FAIL 正文
写着\"不是靠改这里\"。它红着就接不进测试床; 删掉它则是让一条未验证的断言静默消失。

⇒ #ifdef PC_PARKED_REPLAY_ONLY 拆两个 exe: 默认变体跑其余全部并接进测试床,
parked 变体(三行 #include, 零重复代码)只跑那一条、进 NOTRUN_LIST 具名披露。
默认 exe 每次运行都会打出 [PARKED] 点名 —— 计数是基数检查不是身份检查, 抓不到
\"跑了但有一条不执行\", 所以必须打在运行输出里。parked 的代码仍由默认 exe 每次编译。

顺带堵两条真静默截断 + 修一条会误导人的文案:
- replay_real_capture: 18 列 sscanf 对更宽的行静默忽略 ⇒ 改成按真列数判(响亮拒绝)
- mgParseRepeat: 三处静默丢弃(>16 项 / >缓冲 / 两侧不等) ⇒ 一律返回 -1 响亮拒绝
- t6ScanLastBlock: 把\"列太宽\"也报成\"缺 @720\" ⇒ 两种情况分开报(误诊会把操作者引到反向)

负对照实测: 反转 #ifdef ⇒ 该段 [FAIL] 且整床 exit 1。
测试床 25 -> 26, 实测 Suites accounted: 26 of 26 (ran 24 + not-run 2), exit 0。"
```

---

### Task 5: 测试床【运行失败】分支的实测负对照

> **★ 订正（2026-09-24，任务完成后按实测补）** —— 本节正文里有三处**我写时没核**的东西。
> 保留原文以便对照，但**以本框为准**：
> 1. **「手抄了 19 遍」→ 实测 24**。`19` 是 **2026-09-23 的快照**（`git show` 数过：
>    `0a72524`=19 → `5de9d3f`=22 → `c01076e`=23 → `77e7724`=24）。这个数**每接一个套件就涨一次**。
>    ⚠ **裸 `grep -c 'echo   \[FAIL\]'` 会数出 26** —— 多出来的两行是文件头
>    "HOW TEST RESULTS ARE JUDGED" 里的**教学示例**（`run_tests.bat:51` / `:53`）。
>    可靠的计数：`set /a PASSED+=1` / `[FAIL: build error]` / `call "%TESTDIR%\build_` 各 24。
> 2. **「内层 `else` 从未被看到变红」→ 不实**。同批 Task 4 的整床负对照
>    （`_harness_task4_neg.log`，基数与本轮相同）与 Task 2 的 `_harness_task2_redwiring.log`
>    都走过这一支。准确说法：**它从未被【当成目标】刻意反证过**，且那两次的套件**自己也报红**、
>    都没量**调用方的退出码** ⇒ 分不开"判定靠退出码"与"判定靠扫摘要"。Task 5 补的正是这一半。
> 3. **`test_calib_store.cpp:163` → 第 47 行**（那文件只有 48 行）。
>
> 另：本节 Step 4 的还原处方（`git show HEAD:<path> > <path>`）在本文件上**也**还原不出动手前的字节
> —— 见 [[git-restore-line-endings]]：`core.autocrlf=true` 下**行尾规范是每个文件各自的**，
> 唯一可靠的判据是拿**动手前记的 sha256** 去对。

**Files:**
- Modify (临时，用完还原): `Touch_Client/tests/test_calib_store.cpp:163`
- Modify: `Docs/superpowers/specs/2026-09-22-test-harness-state.md`（记结果）

**背景**：`run_tests.bat` 里同一块判定手抄了 19 遍，内层
`else ( set /a FAILED+=1  echo [FAIL] )`（**构建成功但测试 exe 返回非零**）**从未被看到变红**。
缺射程的后果具体：`HARNESS_RC=%FAILED%` + `exit /b %HARNESS_RC%` 是唯一把失败传给调用方的
东西（`:786-805`），这条链没被反证过。

- [ ] **Step 1: 记下前置状态**

```bash
cd /d/Projects/Touch && git status --short Touch_Client/tests/test_calib_store.cpp && sha256sum Touch_Client/tests/test_calib_store.cpp
```

Expected: 无未提交改动 + 抄下那个哈希。

- [ ] **Step 2: 制造"构建成功但运行失败"**

把 `test_calib_store.cpp:163` 的 `return g_failed ? 1 : 0;` 临时改成 `return 1;`

- [ ] **Step 3: 跑整床，核三条**

```bash
cd /d/Projects/Touch/Touch_Client/tests && MSYS_NO_PATHCONV=1 cmd.exe /c ".\run_tests.bat" > _harness_runfail.log 2>&1; echo "exit=$?"; grep -n -B2 -A6 "test_calib_store.exe ===" _harness_runfail.log | head -20; grep -E "Summary:|Suites accounted" _harness_runfail.log
```

三条必须**同时**成立（逐条抄进文档）：

1. `test_calib_store` 那一段打出的是 `[FAIL]`，**不是** `[FAIL: build error]`；
2. `Suites accounted` 那行里 `FAILED` 计数 = **1**（其余仍 OK）；
3. **`exit=1`**。

⚠ 若 exit=0，那是一个**真缺陷**（测试失败却对调用方报成功）⇒ 停，转去查 `HARNESS_RC` 那条链，
不要顺手改 `run_tests.bat` 蒙过去。

- [ ] **Step 4: 还原并核对**

⚠ **不要用 `git checkout --` 当还原手段**（Task 3 实现者实测：本仓库 `core.autocrlf=true`
⇒ `git checkout -- <file>` 会把工作区文件的 LF 改写成 CRLF（实测 2734 → 2811 字节），
于是**原始 sha256 对不上，而 `git status` 仍说干净** —— 会把一次正确的还原误判成没还原）。

```bash
cd /d/Projects/Touch && git show HEAD:Touch_Client/tests/test_calib_store.cpp > Touch_Client/tests/test_calib_store.cpp && sha256sum Touch_Client/tests/test_calib_store.cpp && git status --short
```

Expected: 哈希与 Step 1 抄的**逐字相同**、`git status` 无该文件。
（`git show HEAD:<path>` 写出来的是**仓库里的规范字节**，与提交 blob 逐字一致 —— 这正是我们要核的东西。）

- [ ] **Step 5: 再跑一次整床，确认回到 baseline**

```bash
cd /d/Projects/Touch/Touch_Client/tests && MSYS_NO_PATHCONV=1 cmd.exe /c ".\run_tests.bat" > _harness_after_neg.log 2>&1; echo "exit=$?"; grep -E "Suites accounted" _harness_after_neg.log
```

Expected: `exit=0`、`Suites accounted: 26 of 26 (ran 24 + not-run 2)`。

- [ ] **Step 6: 把结果记进测试床状态文档**

`Docs/superpowers/specs/2026-09-22-test-harness-state.md` 末尾追加一节：

```markdown
## 2026-09-24 —— 【运行失败】分支的负对照（此前从未被反证）

`run_tests.bat` 里同一块判定手抄了 19 遍，其中内层 `else`（**构建成功、测试 exe 返回非零**）
从来没被看到变红 —— 上一轮（2026-09-23）打的六个负对照全是**构建失败**那一路。

做法：把 `test_calib_store.cpp` 的 `main()` 临时改成 `return 1;`（**构建照旧成功**），跑整床。
实测三条同时成立（抄自 `_harness_runfail.log` 那一趟）：

1. 该段打出 `[FAIL]`，不是 `[FAIL: build error]` —— <逐字抄那一行>
2. `Suites accounted: 26 of 26 (ran 24 + not-run 2)` 行里 `FAILED` = 1 —— <逐字抄那一行>
3. **进程退出码 = 1** —— <逐字抄>

随后还原（`git checkout --`，`sha256sum` 核对），再跑一次整床 `exit=0`。

⚠ **这是一次性实测，不是常驻保护**：没有任何东西会在将来再验一遍。
要把它变成常驻，唯一的路是把那 19 处调用点重构成一个 `call :judge` 子程序 ——
那会动一个 `cmd` 语义陷阱密集的文件（见 `cmd-batch-semantics` 记的十一类实测），
本轮**刻意不做**。
```

- [ ] **Step 7: 提交**

```bash
cd /d/Projects/Touch && git add Docs/superpowers/specs/2026-09-22-test-harness-state.md && git commit -m "docs(harness): 补【运行失败】分支的实测负对照 —— 此前从未被反证

run_tests.bat 里同一块判定手抄 19 遍，内层 else（构建成功但 exe 返回非零）
从未见过红；而 HARNESS_RC=%FAILED% + exit /b 是唯一把失败传给调用方的链。

做法：把 test_calib_store 的 main 临时改成 return 1（构建照旧成功），跑整床。
三条同时成立（[FAIL] 而非 [FAIL: build error] / FAILED=1 / 退出码 1），
还原后 sha256 核对一致、整床回到 exit 0。逐字输出抄在文档里。

明写边界：这是一次性实测不是常驻保护；要常驻得重构那 19 处调用点为 call :judge，
本轮刻意不做（cmd 语义陷阱密集）。"
```

---

## 收尾（全部 Task 完成后）

- [ ] **整床最终一次**：`Suites accounted: 26 of 26 (ran 24 + not-run 2)`、`exit 0`，抄下 Summary 行。
- [ ] **对账 spec**：把 `Docs/superpowers/specs/2026-09-24-offline-batch-design.md` §7"本批完成后的状态"
      里的预测数改成**实测数**（特别是有没有哪条没做到）。
- [ ] **不动的事**：不推送、不合并、不改 `master`；`BTN2_J4_SIGN` 仍等 J5/J6 一起翻。
- [ ] **提醒交接**：本批改了 `ForceTuning.cpp` / `RelayCore.cpp` / `GainReadbackPolicy.h`
      ⇒ **下次上机前必须重建客户端**（与 `BTN2_J4_SIGN` 一并做）。
