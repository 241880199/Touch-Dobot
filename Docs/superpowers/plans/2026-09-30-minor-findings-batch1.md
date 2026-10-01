# 22 条 Minor 收口（第一批）实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把 2026-09-22 那批 22 条 Minor 里**可离线验证的 8 条**收口 —— 其中唯一有设计内容的是第 12 条（把 `RelayCore` 里那三样状态与迁移抽成可测单元，并把调用点的裸 `bool` 参数换成枚举）；其余 7 条是"如实化"。

**Architecture:** 新增一个 header-only 单元 `relay/GainReadback.h`（判决在 `GainReadbackPolicy.h`，状态在这里），`RelayCore` 退化成"取值 / 组包 / 发送"三件事；然后逐条把过宽或过时的注释改到与代码一致。

**Tech Stack:** C++17 / MSVC；测试走 `Touch_Client/tests/`（套件数**运行时从磁盘数出**并断言，不需要手改任何计数）。

**Spec:** `Docs/superpowers/specs/2026-09-30-minor-findings-batch1-design.md`（**先读它**）

## Global Constraints

- **除 Task 1/2 的 `bool force` → `GainReadback::SendMode` 与状态搬迁外，不改任何可执行语义。**
  RS 载荷的 7 个字段顺序、`snprintf` 格式串、三样状态的**初值与推进时机**逐字不动。
- **不许**改 `GainReadbackPolicy.h` 的判决逻辑、`GAIN_REPORT_MIN_INTERVAL_MS`、`ForceTuning` 的任何 API、
  以及 `Relay_Station/relay_gui.m`（**本批一个字都不动**）。
- 每步之后 **整床 exit 0 且 `Suites accounted: N of N`**；新增套件必须**同时**接进 `run_tests.bat`（build + run 两处）。
- ⚠ **本仓最怕假绿**：新用例必须断言**具体值**；**每条新判据都要有一条负对照实测红**。
- C++ 注释中文；`.bat` 纯 ASCII。
- ⚠ **如发现本计划某条与代码事实不符 ⇒ 停下、如实记，不要照抄**（本仓为此栽过多次）。

---

### Task 1: `GainReadback.h` —— 判决+状态的单元与用例（第 12 条的前半）

**Files:**
- Create: `Touch_Client/relay/GainReadback.h`
- Create: `Touch_Client/tests/test_gain_readback_state.cpp`
- Create: `Touch_Client/tests/build_gain_readback_state_test.bat`
- Modify: `Touch_Client/tests/run_tests.bat`（在 `test_gain_readback_policy` 那一段之后加一段，形制逐字照抄它）

**Interfaces:**
- Consumes：`GainReadbackPolicy::gainReportDecision(bool, double, double, unsigned long, unsigned long, unsigned long)` 与 `GainReadbackPolicy::GainReport{Send, SkipUnchanged, SkipTooSoon}`、`GAIN_REPORT_MIN_INTERVAL_MS`（全在 `relay/GainReadbackPolicy.h`，**不改**）
- Produces（Task 2 要靠它）：`GainReadback::SendMode{Forced, Throttled}` · `GainReadback::State` 的 `bool beginSend(SendMode, double g, unsigned long nowMs)` / `bool pending() const` / `double lastSentGain() const` / `unsigned long lastReportMs() const`

> ⚠ **与设计 §2.2 的一处偏离（YAGNI）**：设计里列了 `void clearPending()`。
> **不要加它** —— 今天那个 pending 标志只在判决/发送分支里被改（`RelayCore.cpp` 抽取前就是这样），
> **没有任何外部调用方需要清它**。加一个没人调的公开方法正是复审会打回的形状。

- [ ] **Step 1: 先写用例（此时 `GainReadback.h` 还不存在 ⇒ 预期编译失败）**

`Touch_Client/tests/test_gain_readback_state.cpp`：

```cpp
// Standalone test: GainReadback::State —— RG| 回读的【状态机】(判决 + 三样状态的迁移)
// Build: build_gain_readback_state_test.bat
// Run:   test_gain_readback_state.exe
//
// 【为什么有它】判决 (GainReadbackPolicy.h) 2026-09-24 就有用例了, 但【判决够不到的那半】
//   一直在 RelayCore::sendReflectionGain 里, 而 RelayCore.cpp 不被任何测试编译:
//     · 三样状态什么时候推进 / pending 什么时候置与清;
//     · 调用点传的实参 (判决用例只看形参)。
//   代价付过一次: 限频被传成 force=true ⇒ 静默死掉。本文件把第一样钉住;
//   第二样由 SendMode 枚举在【编译期】挡住 (RelayCore.cpp 仍没有覆盖, 别写成"接线上有测试了")。
//
// ⚠ 【不睡】: 时钟由用例作为入参给 (beginSend 的 nowMs), 这是 2026-09-24 起本仓的约定。

#include <iostream>
#include "../relay/GainReadback.h"

using GainReadback::SendMode;
using GainReadback::State;

static int g_passed = 0, g_failed = 0;

#define TEST(name) do { std::cout << "  " << #name << "... "; } while(0)
#define PASS() do { std::cout << "PASS" << std::endl; g_passed++; } while(0)
#define CHECK(cond) do { if (!(cond)) { std::cout << "FAIL: " << #cond << std::endl; g_failed++; return; } } while(0)

// ===== 格 1: 首次 Throttled 就发 (初值 0 是 setGain 不接受的值 ⇒ 保守方向) =====
static void test_first_throttled_sends() {
    TEST(first_throttled_sends);
    State s;
    // 值 120、时刻 1000、从没发过 (lastSentGain = 0) ⇒ 值"变了" ⇒ 发
    CHECK(s.beginSend(SendMode::Throttled, 120.0, 1000));
    CHECK(!s.pending());
    CHECK(s.lastReportMs() == 1000);
    CHECK(s.lastSentGain() == 120.0);
    PASS();
}

// ===== 格 2: Forced 恒发, 不看两个时刻 =====
static void test_forced_always_sends() {
    TEST(forced_always_sends);
    State s;
    CHECK(s.beginSend(SendMode::Forced, 120.0, 1000));
    // 同值、同刻 ⇒ Throttled 会挡, Forced 不挡
    CHECK(s.beginSend(SendMode::Forced, 120.0, 1000));
    CHECK(s.lastReportMs() == 1000);
    PASS();
}

// ===== 格 3: Throttled 值没变 ⇒ 不发, 且【顺手清掉 pending】 =====
// 这是抽取时最容易漏的一条 (抽取前在 RelayCore.cpp 的 SkipUnchanged 分支里 ——
// ⚠ 别记行号: Task 2 会把这些行搬走, 按内容找)。
static void test_unchanged_clears_pending() {
    TEST(unchanged_clears_pending);
    State s;
    CHECK(s.beginSend(SendMode::Throttled, 120.0, 1000));   // 发, 记 120 @1000
    CHECK(!s.beginSend(SendMode::Throttled, 130.0, 1010));  // 值变了但太近 ⇒ 挡, 置 pending
    CHECK(s.pending());
    CHECK(!s.beginSend(SendMode::Throttled, 120.0, 1020));  // 值又变回 120 ⇒ 没可报的
    CHECK(!s.pending());                                    // ★ 顺手清
    CHECK(s.lastReportMs() == 1000);                        // 且【不】推进
    PASS();
}

// ===== 格 4: 值变了但没到窗口 ⇒ 挡 + 置 pending + 【不】推进 lastReportMs =====
static void test_too_soon_sets_pending_without_advancing() {
    TEST(too_soon_sets_pending_without_advancing);
    State s;
    CHECK(s.beginSend(SendMode::Throttled, 120.0, 1000));
    CHECK(!s.beginSend(SendMode::Throttled, 130.0, 1099));  // 窗口 100ms, 差 99 ⇒ 挡
    CHECK(s.pending());
    CHECK(s.lastReportMs() == 1000);                        // ★ 它记的是"上次真的发出去"
    CHECK(s.lastSentGain() == 120.0);                       // ★ 同理, 值也不推进
    PASS();
}

// ===== 格 5: 承接格 4 —— 过了窗口再进来就发, 且 pending 被清 =====
static void test_pending_resend_after_window() {
    TEST(pending_resend_after_window);
    State s;
    CHECK(s.beginSend(SendMode::Throttled, 120.0, 1000));
    CHECK(!s.beginSend(SendMode::Throttled, 130.0, 1099));
    CHECK(s.pending());
    CHECK(s.beginSend(SendMode::Throttled, 130.0, 1100));   // 差 == 窗口 ⇒ 放行
    CHECK(!s.pending());
    CHECK(s.lastReportMs() == 1100);
    CHECK(s.lastSentGain() == 130.0);
    PASS();
}

// ===== 格 6: 被拒的回读必须发得出去 (Forced 不看"值没变") =====
// 规格的中心例子: MATLAB 发 500 被拒 ⇒ 生效值没变 ⇒ 若走 Throttled 就一个字节都发不出去。
static void test_forced_bypasses_unchanged() {
    TEST(forced_bypasses_unchanged);
    State s;
    CHECK(s.beginSend(SendMode::Forced, 120.0, 1000));
    CHECK(s.beginSend(SendMode::Forced, 120.0, 1010));      // 同值、【不同刻】(1010 ≠ 1000), 仍发
    CHECK(s.lastReportMs() == 1010);
    PASS();
}

// ===== 格 7: 32 位时钟环绕 (无符号相减) =====
static void test_clock_wraparound() {
    TEST(clock_wraparound);
    State s;
    CHECK(s.beginSend(SendMode::Throttled, 120.0, 0xFFFFFFF0u));   // 发, last = 0xFFFFFFF0
    // 实际过去 0x210 = 528ms ≥ 100 ⇒ 发
    CHECK(s.beginSend(SendMode::Throttled, 130.0, 0x00000200u));
    CHECK(s.lastReportMs() == 0x00000200u);
    PASS();
}

int main() {
    std::cout << "=== GainReadbackState Tests ===" << std::endl;
    test_first_throttled_sends();
    test_forced_always_sends();
    test_unchanged_clears_pending();
    test_too_soon_sets_pending_without_advancing();
    test_pending_resend_after_window();
    test_forced_bypasses_unchanged();
    test_clock_wraparound();
    std::cout << std::endl;
    std::cout << g_passed << " passed, " << g_failed << " failed" << std::endl;
    return g_failed > 0 ? 1 : 0;
}
```

- [ ] **Step 2: 跑，确认红（编译失败）**

Run（**必须先带 vcvarsall 守卫** —— 少了它在新开的 shell 里 `cl` 不存在，得到的是**噪音红**而不是想要的那条；
仓库里每个 `build_*.bat` 顶上那一句就是干这个的）：

```
if not defined VCINSTALLDIR call "D:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvarsall.bat" x64
cd /d "D:\Projects\Touch\Touch_Client\tests"
cl /EHsc /std:c++17 /DWIN32 /DWIN32_LEAN_AND_MEAN /DNOMINMAX /D_CRT_SECURE_NO_WARNINGS test_gain_readback_state.cpp /Fe:test_gain_readback_state.exe
```

Expected: **编译失败**，形如 `cannot open source file "../relay/GainReadback.h"`。
**把原文抄进报告**（没有这一段就没有"判据先写死"的证据）。

- [ ] **Step 3: 写 `Touch_Client/relay/GainReadback.h`**

```cpp
#pragma once

// RG| 回读的【状态机】—— 判决 + 三样状态的迁移。头文件内联, 无 .cpp 可链。
//
// 【它解决什么】(2026-09-30, 22 条 Minor 的第 12 条)
//   判决本身 2026-09-24 就抽成了纯函数 (relay/GainReadbackPolicy.h), 有 5 格用例。
//   但【判决够不到的那半】还留在 RelayCore::sendReflectionGain 里 —— 而 RelayCore.cpp
//   【不被任何测试编译】:
//     · 三样状态的迁移 (什么时候推进"上次发送"那两样、pending 什么时候置与清);
//     · 调用点传的【实参】—— 判决用例只看形参, 某个调用点把 force 传错, 5 格一个都不会红。
//   代价已经付过一次 (2026-09-22): 限频被传成 force=true ⇒ 静默死掉, 测试床一声不响。
//   ⇒ 本单元把这两样都变成可测的; 调用点改用【枚举】而不是裸 bool ⇒ 传错的形状被编译器挡住。
//
// 【与 GainReadbackPolicy.h 的分工 —— 别把两者合并】
//   判决在那边 (纯函数、无状态、"给定这些数该不该发", 可以单独证伪);
//   状态在这边 ("发过之后记什么")。本文件 include 它、只用它, **不复制它的规则**。
//
// ⚠ 【线程】三个成员仍是 atomic —— 与抽取前【逐字相同】的理由:
//   GLUT idle 线程 (拖动/拒绝/重连) 与 pollRelayCommands 不同步地碰它们,
//   非原子对象上的不同步读写是 UB。残留的只是【次序】上的竞争, 且无害 ——
//   三个状态【不参与强制发送的决策】(SendMode::Forced 一个判断都不从它们取, 只写)
//   ⇒ 交错最坏 = 多回一条、或晚回一条。

#include <atomic>
#include "GainReadbackPolicy.h"

namespace GainReadback {

// ★ 参数类型化: 调用点不再传裸 bool。
//   抽取前是 `sendReflectionGain(bool force)`, 调用点读作 `sendReflectionGain(true)` ——
//   一个 bool 字面量, **看不出语义、编译器也管不着**。2026-09-22 的事故就长在这个形状上。
enum class SendMode {
    // 无条件发, 不看下面那两条闸。三个用途: 连上时 / 重连成功时 / 【被拒绝的增益改动】。
    //   ⚠ 第三个用途容易被"统一"掉: 被拒 ⇒ 生效值【按构造】没变 ⇒ 走 Throttled 会被
    //     "值没变"那道闸必然命中 ⇒ 一个字节都发不出去, 而 MATLAB 的滑条此刻已经动了。
    Forced,
    // 拖动洪水与 pollRelayCommands 的补发: 走"值变了 + 距上次发送 ≥ 窗口"两道闸。
    Throttled,
};

class State {
public:
    // 判决【并】完成状态迁移, 一步做完。时钟由调用方给 ⇒ 用例不必睡。
    // 返回 true ⇒ 调用方【现在】应当真的把这条回读发出去。
    bool beginSend(SendMode mode, double g, unsigned long nowMs) {
        switch (GainReadbackPolicy::gainReportDecision(
                    mode == SendMode::Forced, g, m_lastSentGain.load(), nowMs,
                    m_lastReportMs.load(),
                    GainReadbackPolicy::GAIN_REPORT_MIN_INTERVAL_MS)) {
        case GainReadbackPolicy::GainReport::SkipUnchanged:
            // ⚠ 必须【顺手清掉待发标志】: 一条被限频挡下的 A→B 之后值又变回 A, 此时"待发"
            //   已无事可做; 留着标志会让 pollRelayCommands 每帧都调进来、每帧都从这里返回
            //   ⇒ 标志卡在 true 再也不动 (无害, 但那个标志从此失去意义)。
            m_pending.store(false);
            return false;
        case GainReadbackPolicy::GainReport::SkipTooSoon:
            // 记下待发, 由 pollRelayCommands 补 —— 最后一条不丢。
            // ⚠ 这里【不】推进 m_lastReportMs: 它记的是"上次【真的发出去】"的时刻。
            m_pending.store(true);
            return false;
        case GainReadbackPolicy::GainReport::Send:
            break;
        }
        // ⚠ 两样状态都落笔在【发送调用之前】(调用方紧接着才发) ⇒ socket 恰在那一瞬间失效时,
        //   这一次算"发过了"、同一个值的重试会被"值没变"挡下。这是【已知且能收敛】的:
        //   重连成功时的 Forced 不看这两条闸, 会把当前值原样再送一条 ⇒ 值最终一定到 MATLAB。
        //   ⇒ 按这个定义读这两个名字, **别按"确认送达"读**。
        m_pending.store(false);
        m_lastReportMs.store(nowMs);
        m_lastSentGain.store(g);
        return true;
    }

    // pollRelayCommands 每帧问它: 有没有被限频挡下、还欠 MATLAB 一条?
    bool pending() const { return m_pending.load(); }

    // 只读回显, 供用例与诊断 —— 【不参与判决】。
    double lastSentGain() const { return m_lastSentGain.load(); }
    unsigned long lastReportMs() const { return m_lastReportMs.load(); }

private:
    // m_lastSentGain 初值刻意选 0 —— 那是 setGain 不会接受的值 ⇒ 在第一次 Forced 之前
    //   若有人用 Throttled 进来, 它一定发得出去 (保守方向)。与抽取前逐字相同。
    std::atomic<unsigned long> m_lastReportMs{0};
    std::atomic<double>        m_lastSentGain{0.0};
    std::atomic<bool>          m_pending{false};
};

} // namespace GainReadback
```

- [ ] **Step 4: 建 build 脚本 `Touch_Client/tests/build_gain_readback_state_test.bat`**

形制**逐字照抄** `build_gain_readback_policy_test.bat`（同一个 vcvarsall 守卫、同样的 `/D` 宏），只改三处：
`rem` 说明、源文件名、exe 名。**纯 ASCII**。正文：

```bat
@echo on
rem 2026-09-30: guard the vcvarsall call -- PATH grows ~1350 chars per call and cmd dies at ~8191,
rem   so a second unguarded call in the same session can abort the rest of run_tests.bat.
rem   With this guard, vcvarsall runs at most once per cmd session. (ASCII only -- see
rem   build_force_pipeline_test.bat's note about non-ASCII comments in .bat files.)
if not defined VCINSTALLDIR call "D:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvarsall.bat" x64
cd /d "D:\Projects\Touch\Touch_Client\tests"
rem GainReadback.h is a PURE header (inline implementation, no .cpp to link) -- same shape as
rem   GainReadbackPolicy.h and JitterStats.h. Nothing else is linked on purpose; if this suite
rem   ever needs a second translation unit, that means the state machine stopped being pure.
cl /EHsc /std:c++17 /DWIN32 /DWIN32_LEAN_AND_MEAN /DNOMINMAX /D_CRT_SECURE_NO_WARNINGS test_gain_readback_state.cpp /Fe:test_gain_readback_state.exe
echo BUILD_EXIT=%ERRORLEVEL%
```

- [ ] **Step 5: 跑，确认全绿**

Run: `Touch_Client\tests\build_gain_readback_state_test.bat` 然后 `test_gain_readback_state.exe`
Expected: `BUILD_EXIT=0`；**`7 passed, 0 failed`**、exit 0。

- [ ] **Step 6: 负对照（**必须实测**，三条）—— 改坏实现，每条都要看到对应那格红，然后改回**

1. 把 `SkipUnchanged` 分支里的 `m_pending.store(false);` 删掉 ⇒ **格 3 红**（`FAIL: !s.pending()`）。
2. 把 `SkipTooSoon` 分支整个并进 `Send`（即删掉 `SkipTooSoon` 这个 case，让它落到发送）⇒
   **格 3 / 4 / 5 三条都红**（它们都断言"太近的那一次被挡"；**如实记下红的是哪几条**，别写成只红格 4）。
3. 在 `beginSend` 开头加 `mode = SendMode::Throttled;`（把 Forced 降级）⇒ **格 2 与格 6 红**。

把三条红原文抄进报告；**改回后必须再跑一遍全绿**。

- [ ] **Step 7: 接进测试床 `Touch_Client/tests/run_tests.bat`**

在 `test_gain_readback_policy` 那一段（`rem ============ ... gain_readback_policy ... echo.`）**之后**、下一段之前，插入一段，
**形制逐字照抄** `gain_readback_policy` 那段（`echo --- Building ...` → `call` → `@echo off` → `if %ERRORLEVEL% EQU 0 (` → 内层 `if !ERRORLEVEL! EQU 0` 的成功/失败分支 → `) else ( [FAIL: build error] )` → `echo.`）。
`rem` 说明写清：**本套件钉的是状态迁移那半；`RelayCore.cpp` 的调用点仍无覆盖**。

⚠ 计数**不需要手工改**：`NTESTS` 由 `for %%F in ("%TESTDIR%\test_*.cpp")` 数出，`TOTAL`/`NNOTRUN`/`ACCOUNTED` 全是运行期算的。

- [ ] **Step 8: 跑整床**

Run: `cd Touch_Client/tests && cmd //c ".\run_tests.bat"`
Expected: `exit 0`，且 `Suites accounted: 27 of 27 (ran 25 + not-run 2)`（**具体数字照实抄**）。

- [ ] **Step 9: 提交**

```bash
git add Touch_Client/relay/GainReadback.h Touch_Client/tests/test_gain_readback_state.cpp Touch_Client/tests/build_gain_readback_state_test.bat Touch_Client/tests/run_tests.bat
git commit -m "feat(minor-12): 增益回读的状态机抽成可测单元 GainReadback.h" -m "判决 2026-09-24 已抽成纯函数, 但它够不到的那半 (三样状态的迁移) 仍留在 RelayCore.cpp 里, 而那个文件不被任何测试编译。本单元把它搬出来: 7 格用例 + 3 条负对照。SendMode 枚举在下一个提交里替掉调用点的裸 bool。"
```

---

### Task 2: `RelayCore` 接线 —— `bool force` → `GainReadback::SendMode`（第 12 条的后半）

**Files:**
- Modify: `Touch_Client/relay/RelayCore.h:170-194`（声明 + 那段按调用点分 force=true/false 的文档块）
- Modify: `Touch_Client/relay/RelayCore.cpp`（`sendReflectionGain` 的实现 + **5 个调用点**）

**Interfaces:**
- Consumes：Task 1 的 `GainReadback::SendMode` 与 `GainReadback::State`
- Produces：`void RelayCore::sendReflectionGain(GainReadback::SendMode mode)`

**⚠ 这个文件不被任何测试编译** ⇒ 本任务**没有自动化证据**，只能靠：编译通过 + 整床仍绿（证明没连带弄坏别处）+ 人工审读。**报告里不许写成"接线也有测试了"。**

- [ ] **Step 1: 改 `RelayCore.h` 的声明与文档块**

```cpp
    void sendReflectionGain(GainReadback::SendMode mode);
```

同段文档块里凡写 `force=true` / `force=false` 的地方，改成 `SendMode::Forced` / `SendMode::Throttled`
（**逐条含义不变、三个/两个调用点的清单不变**）；并**加一句**指向新单元：

```
    //   ⚠ 参数是【枚举】不是 bool (2026-09-30): 从前这里收 `bool force`, 调用点读作
    //     `sendReflectionGain(true)` —— 一个看不出语义、编译器也管不着的字面量。
    //     2026-09-22 的事故正是某个调用点把它传错。状态与迁移现在在 relay/GainReadback.h,
    //     由 test_gain_readback_state 钉住; 本函数只剩"取值 / 组包 / 发送"。
```

- [ ] **Step 2: 改 `RelayCore.cpp` 的实现**

把 2794-2796 的三个 `static std::atomic<...>` 与 2798-2847 的函数体，换成：

```cpp
// 三样状态与它们的迁移【不在本文件】—— 在 relay/GainReadback.h, 由 test_gain_readback_state 钉住。
//   本文件只负责: 取值、传参、按判决组包发送。线程约定与抽取前逐字相同 (见那个头文件)。
static GainReadback::State s_gainReadback;

void RelayCore::sendReflectionGain(GainReadback::SendMode mode) {
    const DWORD now = GetTickCount();
    const double g = ForceTuning::gain();
    if (!s_gainReadback.beginSend(mode, g, now)) return;   // 被挡下 ⇒ pending 已由 beginSend 管好

    char buf[160];
    snprintf(buf, sizeof(buf), "RG|%.6g,%.6g,%.6g,%.4f,%.6g,%.6g,%.6g",
             g,
             ForceTuning::GAIN_MIN,
             ForceTuning::GAIN_MAX,
             ForcePipeline::netRatioPerGainUnit() * g,   // ratio
             Config::FORCE_RESIDUAL_DEADZONE_N,          // deadN
             ForcePipeline::saturationSensorN(g),        // satN
             ForceTuning::defaultGain());                // defGain
    sendRelayUpdate(buf);
}
```
（★ 上面这 7 个实参与格式串**逐字照抄现状、一个字都不许改** —— 顺序错一位 MATLAB 侧就全读错位，
而本侧没有任何单测能发现。抄之前先 `sed -n '2837,2846p' Touch_Client/relay/RelayCore.cpp` 核一眼。）

⚠ **上面 2794-2796 那一大段注释（为什么 atomic、S/DWORD 语义、"发出去"的精确含义）不要删** ——
把仍然描述**本函数**的部分留下，把描述**状态**的部分搬进 `GainReadback.h`（Task 1 已经搬过一份，
这里避免两份：**留下的那份要跟头文件一致，别自相矛盾**）。

- [ ] **Step 3: 改 5 个调用点**

| 位置 | 现在 | 改成 |
|---|---|---|
| `RelayCore.cpp:2527` | `sendReflectionGain(true);` | `sendReflectionGain(GainReadback::SendMode::Forced);` |
| `RelayCore.cpp:2610` | `sendReflectionGain(true);` | `sendReflectionGain(GainReadback::SendMode::Forced);` |
| `RelayCore.cpp:2874` | `sendReflectionGain(false);` | `sendReflectionGain(GainReadback::SendMode::Throttled);` |
| `RelayCore.cpp:2895` | `sendReflectionGain(true);` | `sendReflectionGain(GainReadback::SendMode::Forced);` |
| `RelayCore.cpp:2928` | `sendReflectionGain(false);` | `sendReflectionGain(GainReadback::SendMode::Throttled);` |

⚠ **改之前先逐处核一眼上下文**（行号会漂）：判据是**语义**不是行号 ——
`initRelayReporting` 连上时 / `ensureRelayConnected` 重连成功时 / `dispatchRelayCommand` 的**拒绝**分支 ⇒ `Forced`；
**接受**分支 / `pollRelayCommands` 的补发 ⇒ `Throttled`。**任何一处与上表不符 ⇒ 停下、记，别硬改。**

- [ ] **Step 4: 编译整个客户端（这是本任务唯一的机械证据）**

> ⛔ **2026-10-01 订正（实现者顶回、控制方回仓库核实）：原计划点的是 `Touch_Client\tests\_build_comp.bat`，
> 而那条只编 `test_force_compensation.cpp` + 四个 force/calibration 源文件（`_build_comp.bat:13-18`），
> 【不编客户端】⇒ 它的"编译通过"与本任务毫无关系，是一条**假绿**。**
> 本任务真正要证明的是 **`RelayCore.cpp` 编得过、链得过**。

Run（**直调 MSBuild** —— 客户端项目文件是这件事的唯一入口）:

```
"D:\Program Files\Microsoft Visual Studio\2022\BuildTools\MSBuild\Current\Bin\MSBuild.exe" D:\Projects\Touch\Touch_Client\Touch_Client.vcxproj /p:Configuration=Release /p:Platform=x64 /v:minimal
```

⚠ **在 Git Bash 里跑要在前面加** `MSYS2_ARG_CONV_EXCL='*' MSYS_NO_PATHCONV=1` —— MSYS 会改写 `/p:...` 这类参数，
不加会报 `MSB1008`（2026-10-01 实现者实测）。

Expected: 编译 + 链接通过（退出码 0），**且日志里出现过 `RelayCore.cpp`** —— 没出现就等于没编到它。
⚠ **不要直接调 `Touch_Client\build.bat`** —— 它失败分支里有 `pause`，会把代理挂住。
⚠ **若报 LNK1168**：`Touch_Client.exe` 正在跑 ⇒ 按 PID **优雅关闭**（`taskkill /PID <pid>`，**不加 `/F`**），
且**别在操作员正做动作时关**（会停掉 `ServoP/ServoJ` 命令流）。

- [ ] **Step 5: 跑整床**

Run: `cd Touch_Client/tests && cmd //c ".\run_tests.bat"` ⇒ Expected: `exit 0`、`Suites accounted: 27 of 27`。

- [ ] **Step 6: 提交**

```bash
git add Touch_Client/relay/RelayCore.h Touch_Client/relay/RelayCore.cpp
git commit -m "refactor(minor-12): 调用点改传 SendMode 枚举, 替掉裸 bool force" -m "五个调用点按语义分别映射 (连上/重连/拒绝 ⇒ Forced, 接受/补发 ⇒ Throttled)。RelayCore.cpp 不被任何测试编译 ⇒ 本次的证据只有'编译通过 + 整床仍绿 + 人工审读', 接线那层仍然没有自动化覆盖。"
```

---

### Task 3: 注释与措辞如实化（第 1 · 2 · 5 · 9 · 17 条）

**Files:**
- Modify: `Touch_Client/main.cpp:4144-4146` · `Touch_Client/main.cpp:2944-2949`
- Modify: `Touch_Client/force/ForceTuning.cpp:87`
- Modify: `Touch_Client/tests/test_force_pipeline.cpp:165-168`
- Modify: `Touch_Client/tests/test_relay_command_parser.cpp:96-98`

**Interfaces:** 无（**纯注释**，不改任何可执行行）。

- [ ] **Step 1（第 1 条）: `main.cpp:4144-4146`**

原文：

```cpp
    // ★ 晚了会怎样: 回读报的是旧值而实际生效的是文件里的值 ⇒ 界面一个数、手上另一个数,
    //   无声地不一致; 这个不一致要等到下一次回读 —— 重连, 或一次被接受的增益改动 —— 才会
    //   被纠正, 没有别的事件会自己纠正它。这正是本设计要消灭的状态。
```

改成（**依据**：`dispatchRelayCommand` 的**拒绝**分支也无条件回读，`RelayCore.cpp` 那一处）：

```cpp
    // ★ 晚了会怎样: 回读报的是旧值而实际生效的是文件里的值 ⇒ 界面一个数、手上另一个数,
    //   无声地不一致; 这个不一致要等到下一次回读 —— 重连, 或一次增益改动 (【接受与拒绝
    //   都会回读】, 见 RelayCore.cpp 的 dispatchRelayCommand) —— 才会被纠正,
    //   没有别的事件会自己纠正它。这正是本设计要消灭的状态。
```

- [ ] **Step 2（第 2 条）: `main.cpp:2947-2949` 补自证**

在 `// ⚠ 【这里故意不重复判那三个条件】...` 那段之后追加：

```cpp
        // ★ 2026-09-30: 上面那句"原因就是那三个条件"现在【有自证】了 ——
        //   startForceZeroing() 只有两条失败路: ① forceCalibPreconditions (就是这三个条件)
        //   ② ForceCalibration::startZero() 的 !canStart()。而 canStart() ≡ !isRunning()
        //   ≡ !relay.isForceCalibrating(), 上面那条守卫【已经把它排除】⇒ 走到本 else
        //   只可能是那三个条件之一。
        //   ⚠ 所以别把本 else 读成"原因未知": 它是一个【防御性】分支。
```

- [ ] **Step 3（第 5 条）: `ForceTuning.cpp:87`**

原文：`    if (!f) return false;   // 文件不存在 = 还没调过, 正常路径, 不吵`

改成：

```cpp
    // ⚠ 这一行把【不存在】与【打不开】当成同一件事 (权限 / 被占用 / 路径是目录…)。
    //   对"没见过这份文件"来说这是正常的; 对"打不开"来说它安静得过分 —— 但操作员并非全盲:
    //   loadOnStartup 那行仍会打 "未采用 <path>", 而它就在启动横幅上。
    if (!f) return false;
```

- [ ] **Step 4（第 9 条）: `test_force_pipeline.cpp:165-168`**

原文：

```cpp
    // ⚠ 100 / 300 是 ForceTuning::GAIN_MIN / GAIN_MAX 的当前值 (范围的唯一定义在 ForceTuning.h)。
    //   前置条件必须【被命名】: 范围若收窄, setGain 返回 false 且【什么都不改】, 那么下面那些
    //   数值断言会红成"斜坡错了/映射错了", 病因指错地方。这两条 CHECK 红了就是"量程对不上"。
```

在**保留原文**的前提下追加：

```cpp
    // ★ 2026-09-30 补【双向】事实 (Minor 第 9 条): 上面这句从前只说了一半。
    //   · 【收窄】由本文件下面那两条 CHECK(setGain(...)) 钉住 —— 它们红了就是"量程对不上";
    //   · 【放宽】不在本文件, 而在兄弟套件 test_force_tuning.cpp 的 `set_gain_bounds`:
    //     它用字面量断言 `fabs(ForceTuning::gain() - 100.0) < 1e-9`(下限) 与 `300.0`(上限),
    //     所以 GAIN_MIN 若放宽到 50, 那条当场红。
    //   ⇒ 清单里"放宽范围两句话就假了却全绿"这半句【已不成立】(2026-09-30 回核)。
```

- [ ] **Step 5（第 17 条）: `test_relay_command_parser.cpp:96-98`**

在原文那句"注意这条用例【不引用 ForceTuning.h】"之后追加一行：

```cpp
    // ⚠ 2026-09-30 补: 上面提到的 `ForceTuning::GAIN_MIN/GAIN_MAX` 是一处
    //   【非编译器强制】的引用 —— 这个文件不 include 它, 所以那两个符号改名时,
    //   编译器【不会】在这里报错, 只能靠人回来改这一行。改 ForceTuning 的公开名字时请搜这里。
```

- [ ] **Step 6: 跑（纯注释 ⇒ 只需证明没弄坏）**

Run: `cd Touch_Client/tests && cmd //c ".\run_tests.bat"` ⇒ Expected: `exit 0`、`Suites accounted: 27 of 27`。

- [ ] **Step 7: 提交**

```bash
git add Touch_Client/main.cpp Touch_Client/force/ForceTuning.cpp Touch_Client/tests/test_force_pipeline.cpp Touch_Client/tests/test_relay_command_parser.cpp
git commit -m "docs(minor): 五处注释如实化 (第 1/2/5/9/17 条)" -m "全部零行为变化: main.cpp 两处 (第三个纠正事件 / 加自证), ForceTuning 的 fopen 措辞, 以及两处测试注释的双向事实与非强制引用告警。"
```

---

### Task 4: 测试卫生（第 16 条）

**Files:**
- Modify: `Touch_Client/tests/test_force_tuning.cpp`（`main()` 与 `test_missing_file_is_quiet_false`）

**Interfaces:** 无。

- [ ] **Step 1: 修临时文件泄漏**

现状：`kTmp = "_tuning_test_tmp.json"`（`:24`），清理是**各用例末尾的 `remove(kTmp)`**（`:125/145/185/205/216/223`）
⇒ 任何一条 `CHECK` 早退（宏里是 `return`）都会让文件**留在 `tests/`**。

在 `main()` 的**第一行**加：

```cpp
    // ★ 2026-09-30 (Minor 第 16 条): 任何一条 CHECK 早退都会跳过它那一格的 remove(kTmp)
    //   ⇒ 临时文件会留在 tests/ 里。main() 出口兜一次, 这样【失败的那一次运行】也不留垃圾。
    std::atexit([] { remove(kTmp); });
```

并在 `main()` 里每个用例之间（或紧接着每个用例之后）**也**不必再加 —— atexit 已覆盖进程正常退出的路径。
⚠ 如 `main()` 现有的结构不适合（例如已有别的 atexit），**如实记下并换成一个 RAII 小结构**，别硬塞。
⚠ `remove` 需要 `<cstdio>`（该文件已 include）。

- [ ] **Step 2: 把"quiet"从用例名与注释里去掉（不假装断言了 stderr）**

原文（`:129-135`）：

```cpp
static void test_missing_file_is_quiet_false() {
    TEST(missing_file_is_quiet_false);
    double v = 0.0;
    // 文件不存在 = 还没调过, 正常路径 ⇒ 返回 false 但不出声 (出声的是"文件在、内容不合规")
    CHECK(!ForceTuning::loadFromFile("_definitely_not_here_12345.json", &v));
    PASS();
}
```

改成：

```cpp
static void test_missing_file_returns_false() {
    TEST(missing_file_returns_false);
    double v = 0.0;
    // 文件不存在 = 还没调过, 正常路径 ⇒ 返回 false。
    // ⚠ 2026-09-30 (Minor 第 16 条): 本用例【只断言返回值】。从前它叫 ..._is_quiet_false,
    //   而"不出声"那半**没有任何断言** —— 断言它要捕获 stderr, 本测试床没有那个机制。
    //   ⇒ 名字改成它真正断言的东西。要真断言"不出声", 得先给测试床加捕获。
    CHECK(!ForceTuning::loadFromFile("_definitely_not_here_12345.json", &v));
    PASS();
}
```

- [ ] **Step 3: 记得改 `main()` 里的调用点**（旧函数名**不许留着** —— "看着还在、其实从不执行"是本仓记过的形状）

- [ ] **Step 4: 跑**

Run: `Touch_Client\tests\build_force_tuning_test.bat` 然后 `test_force_tuning.exe` ⇒ Expected: 全绿、exit 0。
Run: `cd Touch_Client/tests && cmd //c ".\run_tests.bat"` ⇒ `exit 0`、`27 of 27`。
**另外核一眼**：跑完后 `Touch_Client/tests/` 下**没有** `_tuning_test_tmp.json`
（`ls _tuning_test_tmp.json` 应报"不存在"）。把这条命令与输出抄进报告。

- [ ] **Step 5: 负对照（实测红）**

把 `test_missing_file_returns_false` 里的 `CHECK(!...)` 改成 `CHECK(...)` ⇒ 该用例必须红；
**改回后再跑一遍全绿**。把两条原文抄进报告。

- [ ] **Step 6: 提交**

```bash
git add Touch_Client/tests/test_force_tuning.cpp
git commit -m "test(minor-16): 临时文件不再泄漏 + 去掉用例名里没被断言的 quiet" -m "main() 出口 atexit 兜底清理 (CHECK 早退会跳过那格自己的 remove); missing_file 用例改名成它真正断言的东西, 并在注释里写明'不出声'那半没有被断言及其原因。"
```

---

### Task 5: `vcxproj` 分组（第 18 条）

**Files:**
- Modify: `Touch_Client/Touch_Client.vcxproj`

- [ ] **Step 1: 把 `core\JsonLite.h` 从 `<!-- force -->` 块挪到 `<!-- core -->` 块**

删掉 `:138` 那一行（在 `<!-- force -->` 块里、`force\ForcePipeline.h` 与 `force\ForceTuning.h` 之间），
插到 `<!-- core -->` 块里 `core\MathUtils.h` 之后（即 `:84` 之后、`<!-- robot -->` 之前）。

⚠ **只搬这一行**，不动任何其它条目。`JsonLite.h` 是 header-only（`core\` 下唯一的这种），
归到 core 组只是分组正确，**对构建零影响**。

- [ ] **Step 2: 实测能编过（这是本条唯一的证据）**

> ⛔ **2026-10-01 订正：原计划点的 `_build_comp.bat` 既不编客户端、也【不读 `.vcxproj`】**，
> 而本 Step 改的正是 `.vcxproj` 里的一条 ⇒ 照它验必是**假绿**（"编译通过"与本次改动无关）。
> ⚠ **这是同一个错误在本计划里的第二处**（第一处见 Task 2 Step 4）。

Run（直调 MSBuild —— 仓库里只有它读 `.vcxproj`）:

```
"D:\Program Files\Microsoft Visual Studio\2022\BuildTools\MSBuild\Current\Bin\MSBuild.exe" D:\Projects\Touch\Touch_Client\Touch_Client.vcxproj /p:Configuration=Release /p:Platform=x64 /v:minimal
```

⇒ Expected: `Build succeeded`、退出码 0。
⚠ **在 Git Bash 里跑要在前面加** `MSYS2_ARG_CONV_EXCL='*' MSYS_NO_PATHCONV=1`（MSYS 会改写 `/p:...`，不加报 `MSB1008`）。
⚠ **不要调 `Touch_Client\build.bat`** —— 失败分支里有 `pause`，会把代理挂住。
⚠ LNK1168 ⇒ 按 Task 2 Step 4 同款处理（优雅关闭、别在操作员动作时关）。

- [ ] **Step 3: 跑整床**

Run: `cd Touch_Client/tests && cmd //c ".\run_tests.bat"` ⇒ `exit 0`、`27 of 27`。

- [ ] **Step 4: 提交**

```bash
git add Touch_Client/Touch_Client.vcxproj
git commit -m "chore(minor-18): core\\JsonLite.h 归到 vcxproj 的 core 组"
```

---

### Task 6: 收口 —— 22 条结局表（设计 §3.3）

**Files:**
- Modify: `Docs/superpowers/specs/2026-09-30-minor-findings-batch1-design.md`（追加"执行后状态"一节）
- Modify: `.superpowers/sdd/minor-findings-rollup.md`（**同内容追加**；⚠ 该目录**被 `.gitignore` 忽略、不进版控** ⇒ 权威副本是上面那份 spec）

**Interfaces:** 无。

- [ ] **Step 1: 写结局表**

在**设计文档**末尾追加一节 `## 6. 执行后：22 条结局表`。**内容就照下面这 22 行写**
（"已修"那 8 条要把**真实提交号**填进去 —— 从 `git log --oneline` 取，别照抄本表里的占位）：

| # | 归宿 | 依据 |
|---|---|---|
| 1 | 已修（本批） | `main.cpp` 那句改成"接受与拒绝都会回读"（Task 3 的提交） |
| 2 | 已修（本批） | 补自证：`canStart() ≡ !isRunning()`，由 `isForceCalibrating()` 守卫排除（Task 3 的提交） |
| 3 | **接受** | `ForcePipeline::shutdown()` 归零 `g_gainRamp`；与既有 `g_filters` 复位同类，作者原判可接受 |
| 4 | **接受**（**控制方回核后改判**） | `Config.h:407-408` 已给算式 + 自标"范围一变它就变" ⇒ 非**静默** |
| 5 | 已修（本批） | `fopen` 失败的两义性写进注释 + 指向启动横幅的 `未采用 <path>`（Task 3 的提交） |
| 6 | **接受** | 由重连的强制回读自愈；作者原判 |
| 7 | **第二批**（`relay_gui.m`） | 本机无 MATLAB，改了只能人工审读 —— 不当作"已改" |
| 8 | **第二批**（`relay_gui.m`） | 同上（纯措辞） |
| 9 | 已修（本批） | 补双向事实并订正"放宽仍全绿"那半句（Task 3 的提交） |
| 10 | **接受** | 不变式由 `test_force_tuning` 的 `static_initial_value_is_legal` 兜住；作者原判 |
| 11 | **第二批**（`relay_gui.m`） | 同上 |
| 12 | 已修（本批） | `GainReadback.h` + 调用点换 `SendMode`（Task 1/2 的提交）。**接线那层仍无自动化覆盖** |
| 13 | **第二批**（`relay_gui.m`，**行为项**） | `AllowEmpty` 无 `isprop` 回退 ⇒ 老 MATLAB 构造期整窗起不来 |
| 14 | **第二批**（`relay_gui.m`） | 诊断缺口；`relay_gui.m:965-967` |
| 15 | **第二批**（`relay_gui.m`，**行为项**） | 清空后按 `[Default]` 逼不出回读 |
| 16 | 已修（本批） | `atexit` 兜底清理 + 用例改名（Task 4 的提交） |
| 17 | 已修（本批） | 补"非编译器强制引用"告警（Task 3 的提交） |
| 18 | 已修（本批） | `core\JsonLite.h` 归到 `<!-- core -->`（Task 5 的提交） |
| 19 | **第二批**（`relay_gui.m`） | 文件内自相矛盾（`:272` 声称完整显示 vs 清单说被截断）⇒ **必须上机看渲染** |
| 20 | ✅ **已关闭** | `sendToClient` 断线已出声（`relay_gui.m` 的 `notifyDropped`/`tlog('DROP')`） |
| 21 | ⛔ **作废（不可恢复）** | 所指的 harness Task 7 报告不进 git；且现行 `run_tests.bat` 里 `call` 之后的 `@echo off` 已是 **25** 行（全文 27 处：另两处是文件头与 `:48` 的说明行）。⚠ **2026-10-01 实测订正**：本行原写 "26" —— 那是**本批自己加套件前**的值，本批又加了一个套件 |
| 22 | ✅ **已关闭** | "运行失败分支"的负对照 2026-09-24 已刻意做过（`Docs/superpowers/specs/2026-09-22-test-harness-state.md:335-377`） |

（合计：已修 **8** · 接受 **4** · 第二批 **7** · 已关闭 **2** · 作废 **1** = **22**。）

⚠ **不许**出现"应该已经修了"这种话 —— 依据要么是**提交号**、要么是 **file:line**。

- [ ] **Step 2: 同内容追加到 `.superpowers/sdd/minor-findings-rollup.md`**，并在其开头加一行
`> ★ 2026-09-30: 结局表见本文件末尾；权威副本在 Docs/superpowers/specs/2026-09-30-minor-findings-batch1-design.md（本目录不进版控）。`

- [ ] **Step 3: 提交**

```bash
git add Docs/superpowers/specs/2026-09-30-minor-findings-batch1-design.md
git commit -m "docs(minor): 22 条结局表 —— 每条一个成立的归宿" -m "收口的定义是每条都有归宿, 不是全变绿: 本批修 8 条, 已关闭 2 条, 不可恢复 1 条 (第 21 条所指报告不进 git), 接受 4 条 (含控制方回核后改判的第 4 条), 第二批 7 条 (全在 relay_gui.m, 本机无 MATLAB)。"
```

---

## 报告要求（每个 Task 都要）

追加到 `.superpowers/sdd/minor-batch1-report.md`（**Task 号分段**），必须含：
1. **红 → 绿**的原文（哪些用例、什么输出）；
2. **负对照实测**的原文（Task 1 三条、Task 4 一条）；
3. 整床命令与输出原文；
4. **没做到 / 做不到的事** —— 尤其：**Task 2/5 的接线与分组没有自动化覆盖**、Task 1 不构成"`RelayCore` 有测试了"。
