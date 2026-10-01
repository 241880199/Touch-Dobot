# 力-频率扫描的回放叠加 —— 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 给客户端加一个**可重复**的受控动作源 —— 把日志里的一段轨迹缩到 ±8 mm，按倍速叠加到 ServoP 目标上，一键跑完 8 段（4 倍速 × FF 开/关），用来把 `F(f)` 量干净。

**Architecture:** 两个**纯函数单元**（`SweepWaveform.h` 波形查表、`SweepPlan.h` 段调度）+ 一处**接线**（`RelayCore.cpp` 的 ServoP 路径局部叠加、局部过安全门）+ 一个**离线提取器**（从 `force_wave.csv` 生成波形数据头）。

**Tech Stack:** C++17 / MSVC；测试走 `Touch_Client/tests/`（套件数**运行时从磁盘数出**，不需要手改计数）；离线工具用 Python 3。

**Spec:** `Docs/superpowers/specs/2026-10-01-force-frequency-sweep-replay-design.md`（**先读它**）

## Global Constraints

- **不改任何现有可执行语义**（除了"按住按钮1 且扫描开着"时叠加的偏移）。
- **不许**改：`ForceCompensation` / `ForcePipeline` / 死区 / 增益 / 滤波 / `FORCE_*` / `BTN2_*` / `SAFE_*` 的任何常数。
- **不许**改 `Relay_Station/relay_gui.m`。
- **偏移绝不进入 `m_targetPos`** —— 否则逐帧积分，臂漂走。
- **偏移必须过 `SafetyPredictor::evaluatePositionOnly`** —— 不许绕过任何安全门。
- C++ 注释中文；`.bat` 纯 ASCII；新增套件必须**同时**接进 `run_tests.bat`（build + run 两处）。
- ⚠ **本仓最怕假绿**：新用例必须断言**具体值**；**每条新判据都要有一条负对照实测红**。
- **回滚 = 一行**：`Config::SWEEP_REPLAY_ENABLED = false`。
- ⚠ **如发现本计划某条与代码事实不符 ⇒ 停下、如实记，不要照抄**。
- ⚠ **行号会漂**：一切定位按**内容**，不按行号。

---

### Task 1: 波形单元 —— 离线提取器 + 数据头 + 查表

**Files:**
- Create: `Docs/superpowers/evidence/2026-10-01-scripts/_extract_sweep_waveform.py`
- Create: `Touch_Client/relay/SweepWaveformData.h`（**由上面那个脚本生成**）
- Create: `Touch_Client/relay/SweepWaveform.h`
- Create: `Touch_Client/tests/test_sweep_waveform.cpp`
- Create: `Touch_Client/tests/build_sweep_waveform_test.bat`
- Modify: `Touch_Client/tests/run_tests.bat`（在 `test_gain_readback_state` 那段之后加一段，**形制逐字照抄它**）

**Interfaces:**
- Produces（Task 3 要用）：
  - `SweepWaveform::kF0Hz`（`constexpr double`，源轨迹的主频）
  - `SweepWaveform::lookup(double phase01) -> Vec3`（**单位峰值**的位移；`phase01` 任意实数，内部取模）

- [ ] **Step 1: 写离线提取器**

`_extract_sweep_waveform.py`。**输入**：`Touch_Client/x64/Release/force_wave.csv`。
**选出**块头时间戳为 `2026-10-01 13:29:00` 的那一块（按块头的 `# wave <时间戳>` 前缀匹配，**不要按块序号**）。

```python
# -*- coding: utf-8 -*-
"""从 force_wave.csv 抽一段【单频】轨迹，生成 relay/SweepWaveformData.h。

取法(B)（见设计 §3.1）：取一块臂确实跟上的 tgt_x/y/z -> 去线性趋势 -> 稳健主频 f0
-> 取恰好一个周期 -> 只保留 f0 分量 -> 归一化到单位峰值 -> 输出 128x3 表。

源块: 2026-10-01 13:29:00（主频 ~0.36 Hz; tgt sd 28.80 ~ act sd 28.76 => 臂跟上了）
"""
import math, sys

CSV = sys.argv[1] if len(sys.argv) > 1 else \
    r"D:\Projects\Touch\Touch_Client\x64\Release\force_wave.csv"
WANT = "2026-10-01 13:29:00"
N = 128

def read_blocks(path):
    out, cur = [], None
    for line in open(path, encoding="utf-8", errors="replace"):
        line = line.rstrip("\r\n")
        if line.startswith("# wave"):
            cur = {"head": line, "rows": []}
            out.append(cur)
        elif line.startswith("#") or not line:
            continue
        elif cur is not None:
            cur["rows"].append([float(x) for x in line.split(",")])
    return out

def detrend(y):
    n = len(y); xm = (n - 1) / 2.0; ym = sum(y) / n
    sxy = sum((i - xm) * (y[i] - ym) for i in range(n))
    sxx = sum((i - xm) ** 2 for i in range(n))
    b = sxy / sxx if sxx else 0.0
    a = ym - b * xm
    return [y[i] - (a + b * i) for i in range(n)]

def dom_freq(y, fs, lo=0.15, hi=10.0):
    d = detrend(y); n = len(d)
    w = [0.5 - 0.5 * math.cos(2 * math.pi * i / (n - 1)) for i in range(n)]
    d = [d[i] * w[i] for i in range(n)]
    best, f = None, lo
    while f <= hi:
        om = 2 * math.pi * f / fs
        re = sum(d[i] * math.cos(om * i) for i in range(n))
        im = sum(d[i] * math.sin(om * i) for i in range(n))
        p = re * re + im * im
        if best is None or p > best[1]:
            best = (f, p)
        f += 0.005
    return best[0]

def main():
    blk = [b for b in read_blocks(CSV) if WANT in b["head"]]
    if len(blk) != 1:
        sys.exit(f"!! 期望恰好 1 块匹配 {WANT}, 实际 {len(blk)}")
    rows = blk[0]["rows"]
    # 列序（26 列）: 0 t_us, 7..9 = tgt_x/y/z
    t = [(r[0] - rows[0][0]) / 1e6 for r in rows]
    fs = (len(rows) - 1) / (t[-1] - t[0])
    f0 = dom_freq([r[7] for r in rows], fs)
    print(f"块 {blk[0]['head'].strip()}  fs={fs:.2f}  f0={f0:.4f} Hz  周期={1.0/f0:.3f}s")
    # 取恰好一个周期 -> 重采样到 N 点 -> 只保留 f0 分量（单点 DFT 的 2/N 幅度）= 纯正弦
    harm = []
    for k in range(3):
        y = detrend([r[7 + k] for r in rows])   # ★ 先去趋势, 否则线性漂移会漏进 f0 分量
        re = im = 0.0
        for i in range(len(y)):
            om = 2 * math.pi * f0 * t[i]
            re += y[i] * math.cos(om)
            im += y[i] * math.sin(om)
        amp = 2.0 * math.hypot(re, im) / len(y)
        ph = math.atan2(im, re)
        harm.append((amp, ph))
        print(f"  轴{k}: f0 分量 幅度={amp:.4f} mm  相位={math.degrees(ph):.1f} deg")
    tab = []
    for j in range(N):
        th = 2 * math.pi * j / N
        tab.append([harm[k][0] * math.cos(th + harm[k][1]) for k in range(3)])
    peak = max(math.sqrt(sum(v * v for v in row)) for row in tab)
    if peak <= 0:
        sys.exit("!! 峰值 0 —— 源块没有周期成分")
    tab = [[v / peak for v in row] for row in tab]
    # 自检
    pk = max(math.sqrt(sum(v * v for v in row)) for row in tab)
    assert abs(pk - 1.0) < 1e-12, pk
    assert abs(math.dist(tab[0], tab[-1])) > 0, "首末点不该相同(环是按 N 点闭合)"
    print(f"自检: 归一化峰值={pk:.12f}  平面性={max(abs(v[2]-tab[0][2]) for v in tab):.2e}")
    with open("Touch_Client/relay/SweepWaveformData.h", "w", encoding="utf-8") as fh:
        fh.write("// 本文件由 _extract_sweep_waveform.py 生成 —— 不要手改。\n")
        fh.write("// 源: force_wave.csv 的 %s 块（主频 %.4f Hz）。\n" % (WANT, f0))
        fh.write("#pragma once\n\nnamespace SweepWaveform {\n")
        fh.write("constexpr double kF0Hz = %.6f;\n" % f0)
        fh.write("constexpr int kTableN = %d;\n" % N)
        fh.write("constexpr double kTable[kTableN][3] = {\n")
        for row in tab:
            fh.write("    {%.9f, %.9f, %.9f},\n" % tuple(row))
        fh.write("};\n}  // namespace SweepWaveform\n")
    print("已写 Touch_Client/relay/SweepWaveformData.h")

main()
```

- [ ] **Step 2: 跑它，并把自检输出抄进报告**

Run: `cd /d/Projects/Touch && python Docs/superpowers/evidence/2026-10-01-scripts/_extract_sweep_waveform.py`

Expected: 打印 `f0=`（应在 **0.30~0.45 Hz** 之间）、三轴幅度相位、`归一化峰值=1.000000000000`。
⚠ **若 `WANT` 匹配到 0 块或 ≥2 块 ⇒ 停下如实记**（按内容找块可能因为我记的时间戳不准而失配；那就把实际块头抄回来）。

- [ ] **Step 3: 人眼核产物**（写进报告）

打开 `Touch_Client/relay/SweepWaveformData.h`，确认：① `kTableN = 128`；② 每行三个数都被 `1/peak` 除过（峰值行里至少有一个分量的绝对值接近 1）；③ `kF0Hz` 与 Step 2 打印的一致。

- [ ] **Step 4: 写查表单元的用例（此时 `SweepWaveform.h` 还不存在 ⇒ 预期编译失败）**

`Touch_Client/tests/test_sweep_waveform.cpp`：

```cpp
// 独立测试: SweepWaveform::lookup —— 单位峰值轨迹的【环上查表】
// Build: build_sweep_waveform_test.bat   Run: test_sweep_waveform.exe
// 【钉什么】① phase 取模(负值/超一圈); ② 线性插值的端点与中点; ③ 环闭合(相位 0 与 1 同点)。
// ⚠ 【不睡】: 纯函数, 无时钟。
#include <iostream>
#include <cmath>
#include "../relay/SweepWaveform.h"

using SweepWaveform::Vec3;      // 表与 lookup 都在 namespace SweepWaveform 里

static int g_passed = 0, g_failed = 0;
#define TEST(name) do { std::cout << "  " << #name << "... "; } while(0)
#define PASS() do { std::cout << "PASS" << std::endl; g_passed++; } while(0)
#define CHECK(cond) do { if (!(cond)) { std::cout << "FAIL: " << #cond << std::endl; g_failed++; return; } } while(0)

// 格 1: phase=0 就是表的第一行
static void test_phase_zero_is_first_row() {
    TEST(phase_zero_is_first_row);
    Vec3 v = SweepWaveform::lookup(0.0);
    CHECK(std::fabs(v.x - SweepWaveform::kTable[0][0]) < 1e-12);
    CHECK(std::fabs(v.y - SweepWaveform::kTable[0][1]) < 1e-12);
    CHECK(std::fabs(v.z - SweepWaveform::kTable[0][2]) < 1e-12);
    PASS();
}

// 格 2: 环闭合 —— phase=1 与 phase=0 是同一点（不是表的最后一行）
static void test_phase_one_wraps_to_zero() {
    TEST(phase_one_wraps_to_zero);
    Vec3 a = SweepWaveform::lookup(0.0);
    Vec3 b = SweepWaveform::lookup(1.0);
    CHECK(std::fabs(a.x - b.x) < 1e-12 && std::fabs(a.y - b.y) < 1e-12 && std::fabs(a.z - b.z) < 1e-12);
    PASS();
}

// 格 3: 取模 —— -0.25 与 0.75 同点
static void test_negative_phase_wraps() {
    TEST(negative_phase_wraps);
    Vec3 a = SweepWaveform::lookup(-0.25);
    Vec3 b = SweepWaveform::lookup(0.75);
    CHECK(std::fabs(a.x - b.x) < 1e-12 && std::fabs(a.y - b.y) < 1e-12 && std::fabs(a.z - b.z) < 1e-12);
    PASS();
}

// 格 4: 超一圈取模 —— 2.5 与 0.5 同点
static void test_multi_turn_wraps() {
    TEST(multi_turn_wraps);
    Vec3 a = SweepWaveform::lookup(2.5);
    Vec3 b = SweepWaveform::lookup(0.5);
    CHECK(std::fabs(a.x - b.x) < 1e-12 && std::fabs(a.y - b.y) < 1e-12 && std::fabs(a.z - b.z) < 1e-12);
    PASS();
}

// 格 5: 相邻表项的【中点】就是两者平均（线性插值）
static void test_midpoint_is_linear() {
    TEST(midpoint_is_linear);
    const double h = 1.0 / SweepWaveform::kTableN;
    Vec3 a = SweepWaveform::lookup(0.0);
    Vec3 b = SweepWaveform::lookup(h);
    Vec3 m = SweepWaveform::lookup(h * 0.5);
    CHECK(std::fabs(m.x - 0.5 * (a.x + b.x)) < 1e-12);
    CHECK(std::fabs(m.y - 0.5 * (a.y + b.y)) < 1e-12);
    CHECK(std::fabs(m.z - 0.5 * (a.z + b.z)) < 1e-12);
    PASS();
}

// 格 6: 表上每一点的模长都 <= 1（归一化到单位峰值, 不是"每点都等于 1"）
static void test_peak_is_one_and_never_exceeded() {
    TEST(peak_is_one_and_never_exceeded);
    double mx = 0.0;
    for (int j = 0; j < SweepWaveform::kTableN; j++) {
        Vec3 v = SweepWaveform::lookup((double)j / SweepWaveform::kTableN);
        double r = std::sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
        if (r > mx) mx = r;
    }
    CHECK(std::fabs(mx - 1.0) < 1e-9);          // 峰值恰为 1
    PASS();
}

int main() {
    std::cout << "=== SweepWaveform Tests ===" << std::endl;
    test_phase_zero_is_first_row();
    test_phase_one_wraps_to_zero();
    test_negative_phase_wraps();
    test_multi_turn_wraps();
    test_midpoint_is_linear();
    test_peak_is_one_and_never_exceeded();
    std::cout << std::endl << g_passed << " passed, " << g_failed << " failed" << std::endl;
    return g_failed > 0 ? 1 : 0;
}
```

- [ ] **Step 5: 跑，确认红（编译失败）**

Run（**必须先带 vcvarsall 守卫**）:
```
if not defined VCINSTALLDIR call "D:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvarsall.bat" x64
cd /d "D:\Projects\Touch\Touch_Client\tests"
cl /EHsc /std:c++17 /DWIN32 /DWIN32_LEAN_AND_MEAN /DNOMINMAX /D_CRT_SECURE_NO_WARNINGS test_sweep_waveform.cpp /Fe:test_sweep_waveform.exe
```
Expected: **编译失败**，形如 `cannot open source file "../relay/SweepWaveform.h"`。**把原文抄进报告**。

- [ ] **Step 6: 写 `Touch_Client/relay/SweepWaveform.h`**

```cpp
#pragma once
#include <cmath>
#include "SweepWaveformData.h"   // 生成物: kF0Hz / kTableN / kTable[N][3]

// 扫描回放用的【单位峰值】位移轨迹: 一个 128 点的闭环, 环上任意相位线性插值。
//
// 【为什么是纯函数】与 GainReadbackPolicy.h 同款理由: 接线那半 (RelayCore.cpp)
//   不被任何测试编译, 所以能测的这半要尽量厚。这里没有时钟、没有状态。
//
// ⚠ 【环是 128 点闭合的】: 相位 1.0 与 0.0 是【同一点】, 不是表的最后一行。
//   ⇒ 查表索引要取模到 [0, N), 并且插值要**跨过末尾回到开头**(用 %N 拿下一个点)。
namespace SweepWaveform {

struct Vec3 { double x, y, z; };

inline Vec3 lookup(double phase01) {
    // 取模到 [0,1): 支持负数与多圈
    double p = phase01 - std::floor(phase01);
    const double scaled = p * (double)kTableN;
    int i0 = (int)std::floor(scaled);
    if (i0 >= kTableN) i0 = kTableN - 1;      // p 极接近 1 时的兜底
    const int i1 = (i0 + 1) % kTableN;        // ★ 环: 末点之后回到第 0 点
    const double f = scaled - (double)i0;
    const double a = 1.0 - f;
    return Vec3{
        kTable[i0][0] * a + kTable[i1][0] * f,
        kTable[i0][1] * a + kTable[i1][1] * f,
        kTable[i0][2] * a + kTable[i1][2] * f,
    };
}

}  // namespace SweepWaveform
```

- [ ] **Step 7: 建 build 脚本 `Touch_Client/tests/build_sweep_waveform_test.bat`**

形制**逐字照抄** `build_gain_readback_state_test.bat`（同一个 vcvarsall 守卫、同样的 `/D` 宏），只改三处：`rem` 说明、源文件名、exe 名。**纯 ASCII**。
`rem` 里要写明：`SweepWaveform.h` 是纯头文件（连同生成的 `SweepWaveformData.h`），无 `.cpp` 可链。

- [ ] **Step 8: 跑，确认全绿**

Run: `Touch_Client\tests\build_sweep_waveform_test.bat` 然后 `test_sweep_waveform.exe`
Expected: `BUILD_EXIT=0`；**`6 passed, 0 failed`**、exit 0。

- [ ] **Step 9: 负对照（**必须实测**，三条）—— 改坏实现，每条都要看到对应那格红，然后改回**

1. 把 `lookup` 里的取模 `phase01 - std::floor(phase01)` 改成直接用 `phase01` ⇒ **格 3 与格 5? 见下**（如实记红的是哪几条；`-0.25` 会算出负索引 ⇒ 行为未定义，**可能崩溃**，那就如实记"崩溃"）。
2. 把 `const int i1 = (i0 + 1) % kTableN;` 改成 `(i0 + 1)`（去掉环）⇒ 最后一个区间越界 ⇒ **格 6 红或崩溃**（如实记）。
3. 把 `phase01 - std::floor(phase01)` 改成 `phase01 - (int)phase01` ⇒ **格 3 红**（负数取整方向不对）。

把三条红的原文抄进报告；**改回后必须再跑一遍全绿**。

- [ ] **Step 10: 接进测试床 `Touch_Client/tests/run_tests.bat`**

在 `test_gain_readback_state` 那一段**之后**、下一段之前，插入一段，**形制逐字照抄** `gain_readback_state` 那段。
`rem` 说明写清：**本套件钉的是"环上查表"那半；`RelayCore.cpp` 的接线仍无覆盖**。

- [ ] **Step 11: 跑整床**

Run: `cd Touch_Client/tests && cmd //c ".\run_tests.bat"` ⇒ Expected: `exit 0`、
`Suites accounted: 28 of 28 (ran 26 + not-run 2)`（**具体数字照实抄**）。

- [ ] **Step 12: 提交**

```bash
git add Docs/superpowers/evidence/2026-10-01-scripts/_extract_sweep_waveform.py \
        Touch_Client/relay/SweepWaveformData.h Touch_Client/relay/SweepWaveform.h \
        Touch_Client/tests/test_sweep_waveform.cpp Touch_Client/tests/build_sweep_waveform_test.bat \
        Touch_Client/tests/run_tests.bat
git commit -m "feat(sweep): 波形单元 —— 从日志抽单频轨迹 + 环上查表" -m "源块 13:29:00(0.36Hz, 臂确实跟上)。生成 128 点单位峰值表 + 纯函数 lookup。6 格用例 + 3 条负对照。"
```

---

### Task 2: 调度单元 —— `SweepPlan.h`

**Files:**
- Create: `Touch_Client/relay/SweepPlan.h`
- Create: `Touch_Client/tests/test_sweep_plan.cpp`
- Create: `Touch_Client/tests/build_sweep_plan_test.bat`
- Modify: `Touch_Client/tests/run_tests.bat`（同上，再插一段）

**Interfaces:**
- Consumes：无（自足）
- Produces（Task 3/4 要用）：
  - `struct SweepPlan::State { int seg; double speed; bool ffOn; double phase01; }`
  - `SweepPlan::State sweepStateAt(double elapsedSec)`
  - `SweepPlan::kSegSec`（`10.0`）· `SweepPlan::kSegCount`（`8`）· `SweepPlan::kTotalSec`（`80.0`）

- [ ] **Step 1: 先写用例（此时 `SweepPlan.h` 还不存在 ⇒ 预期编译失败）**

`Touch_Client/tests/test_sweep_plan.cpp`：

```cpp
// 独立测试: SweepPlan::sweepStateAt —— 8 段扫描的【段/倍速/FF/相位】调度
// Build: build_sweep_plan_test.bat   Run: test_sweep_plan.exe
// 【钉什么】段边界(10.0s 恰好落在哪一段) · 总时长之后的行为 · elapsed<0 · 倍速与 FF 的序列 · 相位。
// ⚠ 【不睡】: 时钟是入参。
#include <iostream>
#include <cmath>
#include "../relay/SweepPlan.h"

static int g_passed = 0, g_failed = 0;
#define TEST(name) do { std::cout << "  " << #name << "... "; } while(0)
#define PASS() do { std::cout << "PASS" << std::endl; g_passed++; } while(0)
#define CHECK(cond) do { if (!(cond)) { std::cout << "FAIL: " << #cond << std::endl; g_failed++; return; } } while(0)
#define NEAR(a,b) (std::fabs((a)-(b)) < 1e-9)

using SweepPlan::sweepStateAt;

// 格 1: t=0 是第 0 段、倍速 0.5、FF 开、相位 0
static void test_start_segment() {
    TEST(start_segment);
    auto s = sweepStateAt(0.0);
    CHECK(s.seg == 0);
    CHECK(NEAR(s.speed, 0.5));
    CHECK(s.ffOn);
    CHECK(NEAR(s.phase01, 0.0));
    PASS();
}

// 格 2: 段边界 —— t=10.0 恰好进入第 1 段（不是第 0 段）
static void test_boundary_is_half_open() {
    TEST(boundary_is_half_open);
    auto a = sweepStateAt(9.999999);
    auto b = sweepStateAt(10.0);
    CHECK(a.seg == 0);
    CHECK(b.seg == 1);
    PASS();
}

// 格 3: 倍速序列 = 0.5 / 1 / 2 / 4（四段一轮）
static void test_speed_sequence() {
    TEST(speed_sequence);
    CHECK(NEAR(sweepStateAt(0.0).speed, 0.5));
    CHECK(NEAR(sweepStateAt(10.0).speed, 1.0));
    CHECK(NEAR(sweepStateAt(20.0).speed, 2.0));
    CHECK(NEAR(sweepStateAt(30.0).speed, 4.0));
    PASS();
}

// 格 4: FF 在后四段关
static void test_ff_off_in_second_half() {
    TEST(ff_off_in_second_half);
    CHECK(sweepStateAt(39.0).ffOn);
    CHECK(!sweepStateAt(40.0).ffOn);
    CHECK(!sweepStateAt(79.9).ffOn);
    PASS();
}

// 格 5: 相位 = (段内时间 / 10s) × 倍速, 取模到 [0,1) —— 倍速 2 的段里 1 秒走两圈
static void test_phase_accumulates_with_speed() {
    TEST(phase_accumulates_with_speed);
    CHECK(NEAR(sweepStateAt(2.5).phase01, 0.125));    // 段0: 0.5x, 2.5s/10s*0.5 = 0.125
    CHECK(NEAR(sweepStateAt(12.5).phase01, 0.25));    // 段1: 1x
    CHECK(NEAR(sweepStateAt(22.5).phase01, 0.5));     // 段2: 2x
    CHECK(NEAR(sweepStateAt(32.5).phase01, 0.0));     // 段3: 4x, 2.5/10*4 = 1.0 -> 取模 0
    //  ⚠ 2026-10-01 订正: 本行原写 `1.0 % 1.0` —— 【那在 C++ 里编不过】(MSVC C2296/C2297, `%` 只吃整型)。
    //    实现者把它改成语义相同的 `0.0`(原注释自己就写着期望 0)。计划里这处是【真错】, 不是措辞问题。
    PASS();
}

// 格 6: 跑完之后(>= 总时长) 停在【最后一段的末尾】而不是越界
static void test_after_end_clamps_to_last_segment() {
    TEST(after_end_clamps_to_last_segment);
    auto s = sweepStateAt(SweepPlan::kTotalSec + 5.0);
    CHECK(s.seg == SweepPlan::kSegCount - 1);
    CHECK(!s.ffOn);
    PASS();
}

// 格 7: elapsed < 0 当作 0（不崩、不越界）
static void test_negative_elapsed_is_zero() {
    TEST(negative_elapsed_is_zero);
    auto s = sweepStateAt(-3.0);
    CHECK(s.seg == 0);
    CHECK(NEAR(s.speed, 0.5));
    PASS();
}

int main() {
    std::cout << "=== SweepPlan Tests ===" << std::endl;
    test_start_segment();
    test_boundary_is_half_open();
    test_speed_sequence();
    test_ff_off_in_second_half();
    test_phase_accumulates_with_speed();
    test_after_end_clamps_to_last_segment();
    test_negative_elapsed_is_zero();
    std::cout << std::endl << g_passed << " passed, " << g_failed << " failed" << std::endl;
    return g_failed > 0 ? 1 : 0;
}
```

- [ ] **Step 2: 跑，确认红（编译失败）** —— 同 Task 1 Step 5 的形制，把原文抄进报告。

- [ ] **Step 3: 写 `Touch_Client/relay/SweepPlan.h`**

```cpp
#pragma once
#include <cmath>

// 扫描的【段调度】: 给定从按下 'r' 起算的秒数, 返回这一段用哪个倍速、力反馈开不开、走到环上哪个相位。
// 纯函数、无状态、无时钟 —— 理由同 GainReadbackPolicy.h: 接线那半测不了, 能测的这半要厚。
//
// 排布: 前 4 段 FF 开, 后 4 段 FF 关; 每段内部倍速依次 0.5 / 1 / 2 / 4。
//   ⇒ 同一段里 FF 开与关【动作逐帧相同】—— 这正是 B' 那次栽掉的对照条件。
namespace SweepPlan {

constexpr double kSegSec   = 10.0;
constexpr int    kSegCount = 8;                 // 4 倍速 × 2 个 FF 状态
constexpr double kTotalSec = kSegSec * kSegCount;
constexpr double kSpeed[4] = { 0.5, 1.0, 2.0, 4.0 };

struct State { int seg; double speed; bool ffOn; double phase01; };

inline State sweepStateAt(double elapsedSec) {
    double t = elapsedSec;
    if (!(t > 0.0)) t = 0.0;                    // 负 / NaN 都当 0
    int seg = (int)std::floor(t / kSegSec);
    double inSeg = t - seg * kSegSec;
    if (seg >= kSegCount) {                     // 跑完: 停在最后一段的末尾
        seg = kSegCount - 1;
        inSeg = kSegSec;
    }
    const double speed = kSpeed[seg % 4];
    const bool   ffOn  = (seg < 4);
    double phase = (inSeg / kSegSec) * speed;   // 段内进度 × 倍速 = 走了多少圈
    phase -= std::floor(phase);                 // 取模到 [0,1)
    return State{ seg, speed, ffOn, phase };
}

}  // namespace SweepPlan
```

- [ ] **Step 4: 建 build 脚本** —— 形制照抄 Task 1 Step 7。

- [ ] **Step 5: 跑，确认全绿** ⇒ `7 passed, 0 failed`。

- [ ] **Step 6: 负对照（**必须实测**，三条）**

1. 把 `if (seg >= kSegCount)` 的钳位整段删掉 ⇒ **格 6 红**。
   ⚠ **2026-10-01 实测订正**：红的是 `s.seg` 那条断言 —— **不是**原文说的"越界读 `kSpeed`"。
   `kSpeed[seg % 4]` 把下标钉在 [0,3]（探针：`t=85 ⇒ seg=8, speed=kSpeed[0]=0.5`，**没有越界**）。
2. 把段边界改成 `(int)std::ceil(t / kSegSec)` ⇒ **实测红 3 条（格 2/4/5）**。
   ⚠ **2026-10-01 实测订正**：红在 `t=9.999999` 处，**不是**原文说的"10.0 会落到段 1"
   —— `ceil` 与 `floor` 在 `t=10.0` 上**一致**。
3. 把 `const bool ffOn = (seg < 4);` 改成 `(seg < 5)` ⇒ **格 4 红**。

改回后重跑全绿。

- [ ] **Step 7: 接进测试床 + 跑整床** ⇒ `28 of 28`→`29 of 29`（照实抄）。

- [ ] **Step 8: 提交**

```bash
git add Touch_Client/relay/SweepPlan.h Touch_Client/tests/test_sweep_plan.cpp \
        Touch_Client/tests/build_sweep_plan_test.bat Touch_Client/tests/run_tests.bat
git commit -m "feat(sweep): 段调度单元 SweepPlan.h" -m "8 段(4 倍速 x FF 开/关)的纯函数调度, 7 格用例 + 3 条负对照。"
```

---

### Task 3: 接线（一）—— 偏移 + 本地安全门（`RelayCore`）

**Files:**
- Modify: `Touch_Client/relay/RelayCore.h`（新增 `m_sweepStartMs` 与两个方法声明）
- Modify: `Touch_Client/relay/RelayCore.cpp`（`startSweep` / `stopSweep` / `sweepRunning` + ServoP 路径里那段）
- Modify: `Touch_Client/config/Config.h`（三个常数 + 一行开关）

**Interfaces:**
- Consumes：`SweepWaveform::lookup` / `SweepWaveform::kF0Hz`（Task 1）· `SweepPlan::sweepStateAt` 等（Task 2）
- Produces（Task 4 要用）：`void RelayCore::startSweep()` · `void RelayCore::stopSweep()` · `bool RelayCore::sweepRunning() const`

**⚠ 这个文件不被任何测试编译** ⇒ 本任务**没有自动化证据**，只能靠：编译通过 + 整床仍绿 + 人工审读。**报告里不许写成"接线也有测试了"。**

- [ ] **Step 1: 在 `Config.h` 的按钮2 那一族附近加常数**

```cpp
    // ★★★ 2026-10-01 力-频率扫描的回放叠加（设计见 Docs/superpowers/specs/2026-10-01-...-design.md）
    //   回滚 = 翻 【一个 bool】。
    const bool   SWEEP_REPLAY_ENABLED = true;
    const double SWEEP_AMPLITUDE_MM   = 8.0;   // 偏移的【峰值】位移(mm)。硬上限, 运行时再夹一道。
```

- [ ] **Step 2: `RelayCore.h` 加成员与声明**

在 `m_btn2JointCmd` 那一族旁边加：

```cpp
    // 扫描回放: 0 = 没在跑; 否则 = 按下 'r' 那一刻的 GetTickCount()
    unsigned long m_sweepStartMs = 0;
    bool          m_sweepPrevFf   = true;   // 扫描前的 FF 状态, 停止时还原
    int           m_sweepLastSeg  = -1;     // 段标记去重
    bool          m_sweepRejectNoticed = false;
public:
    void startSweep();
    void stopSweep();
    bool sweepRunning() const;
```

- [ ] **Step 3: `RelayCore.cpp` 加三个小函数**（放在 `sendReflectionGain` 那一族附近，形状照抄它）

```cpp
void RelayCore::startSweep() {
    if (!Config::SWEEP_REPLAY_ENABLED) { std::cout << "[Sweep] 已关闭(SWEEP_REPLAY_ENABLED=false)\n"; return; }
    m_sweepPrevFf    = appState.forceFeedbackEnabled;   // ★ 记下扫描前的 FF, 停止时还原
    m_sweepLastSeg   = -1;
    m_sweepRejectNoticed = false;
    m_sweepStartMs   = GetTickCount();
    std::cout << "[Sweep] 开始: 8 段 x " << SweepPlan::kSegSec << "s, 幅度 +/-"
              << Config::SWEEP_AMPLITUDE_MM << "mm, 源主频 " << SweepWaveform::kF0Hz << "Hz\n";
}
void RelayCore::stopSweep() {
    if (m_sweepStartMs) {
        // ★ 后四段把 FF 关了 ⇒ 不还原的话扫完手上是【没有力反馈】的, 而操作员不知道。
        appState.forceFeedbackEnabled = m_sweepPrevFf;
        std::cout << "[Sweep] 停止: FF 已还原为 " << (m_sweepPrevFf ? "ON" : "OFF") << "\n";
    }
    m_sweepStartMs = 0;
}
bool RelayCore::sweepRunning() const { return m_sweepStartMs != 0; }
```

- [ ] **Step 4: 在 ServoP 路径上叠加**（★ **按内容定位，不按行号**）

找到 `// ===== Compute ServoP position =====` 之后那三行 `servoCmdX = clamped.x;` / `servoCmdY = clamped.y;` / `servoCmdZ = clamped.z;`，
**紧跟其后**插入：

```cpp
    // ★ 2026-10-01 扫描回放: 偏移【只加在本地】, 绝不写回 m_targetPos（否则逐帧积分, 臂漂走）。
    //   并且【必过一道安全门】—— 本函数上面的总闸 evaluate(clamped) 在本行【之前】就跑完了,
    //   在这里直接加会绕过它 ⇒ 用 evaluatePositionOnly 就地补一道（本文件 . 已有同样用法）。
    if (Config::SWEEP_REPLAY_ENABLED && m_sweepStartMs && appState.lastButtonState) {
        const double el = (double)(GetTickCount() - m_sweepStartMs) / 1000.0;
        const SweepPlan::State sp = SweepPlan::sweepStateAt(el);
        const SweepWaveform::Vec3 off = SweepWaveform::lookup(sp.phase01);
        double amp = Config::SWEEP_AMPLITUDE_MM;
        if (amp < 0.0) amp = 0.0;                        // 硬夹: 常数被改坏也不放大
        const Vec3 withOff(servoCmdX + off.x * amp, servoCmdY + off.y * amp, servoCmdZ + off.z * amp);
        const SafetyVerdict sv = SafetyPredictor::instance().evaluatePositionOnly(withOff);
        if (sv.action == SafetyVerdict::REJECT) {
            if (!m_sweepRejectNoticed) {                 // 节流: 每次扫描只喊一声（复位在 startSweep）
                m_sweepRejectNoticed = true;
                std::cout << "[Sweep] 目标把偏移拒了, 本帧不加: " << sv.reason << std::endl;
            }
        } else {
            servoCmdX = withOff.x;  servoCmdY = withOff.y;  servoCmdZ = withOff.z;
        }
        // FF 每帧跟着段走（客户端直接设, 不经过 MATLAB）
        appState.forceFeedbackEnabled = sp.ffOn;
    }
```

并在 `RelayCore.h` 加 `bool m_sweepRejectNoticed = false;`，在 `startSweep()` 里置 `false`。

⚠ **`Vec3` 是 `SweepWaveform::Vec3` 还是本文件已有的 `Vec3`？** —— 实现时**先看本文件顶部 `using`/`struct Vec3`**；若已有同名类型，用**本文件的**（把 `SweepWaveform::lookup` 的返回值逐分量取出来构造），不要引入第二个 `Vec3`。

- [ ] **Step 4b: "松手即停"要写在 `onButtonRelease()` 里，不能写在上面的块里**

⚠ **为什么**：上面那段 ServoP 路径**被 `m_transmitting` 挡在门外**
（本函数顶上那句 `if (!m_transmitting || ...) return;`）⇒ **按钮1 一松开，那段代码根本不跑**，
在它里面判"松手"永远判不到。

⇒ 在 `RelayCore::onButtonRelease()` **开头**加：

```cpp
    // ★ 2026-10-01: 松开按钮1 ⇒ 扫描整个停掉（不是"暂停"—— 免得手一松一按就跳到后面的段）。
    if (m_sweepStartMs) stopSweep();
```

- [ ] **Step 5: 编译（本任务唯一的机械证据）**

Run（**直调 MSBuild**；Git Bash 里要加 `MSYS2_ARG_CONV_EXCL='*' MSYS_NO_PATHCONV=1`，否则 MSYS 改写 `/p:...` 报 `MSB1008`）:
```
"D:\Program Files\Microsoft Visual Studio\2022\BuildTools\MSBuild\Current\Bin\MSBuild.exe" D:\Projects\Touch\Touch_Client\Touch_Client.vcxproj /p:Configuration=Release /p:Platform=x64 /v:minimal
```
Expected: 退出码 0，**且日志里出现 `RelayCore.cpp`**。
⚠ **不要调 `Touch_Client\build.bat`**（失败分支有 `pause`，会挂住代理）。
⚠ `LNK1168` ⇒ 客户端在跑 ⇒ 按 PID `taskkill /PID <pid>`（**不加 `/F`**），**别在操作员做动作时关**。

- [ ] **Step 6: 跑整床** ⇒ `exit 0`、`29 of 29`。

- [ ] **Step 7: 提交**

```bash
git add Touch_Client/config/Config.h Touch_Client/relay/RelayCore.h Touch_Client/relay/RelayCore.cpp
git commit -m "feat(sweep): 偏移叠加 + 本地安全门 (接线, 无自动化覆盖)" -m "偏移只加在本地、不写回 m_targetPos; 因总闸 evaluate(clamped) 在本行之前已跑完, 就地补一道 evaluatePositionOnly。RelayCore.cpp 不被任何测试编译 ⇒ 证据只有编译通过 + 整床绿 + 人工审读。"
```

---

### Task 4: 接线（二）—— 按键 `r` + 段标记打印

**Files:**
- Modify: `Touch_Client/main.cpp`（`keyboard()` 里加 `'r'`；照抄 `'w'` 那一段的形状）

**Interfaces:**
- Consumes：`RelayCore::startSweep()` / `stopSweep()` / `sweepRunning()`（Task 3）

- [ ] **Step 1: 在 `keyboard()` 里 `'w'` 那一段之后加**（照抄它的形状）

```cpp
    // ★ 2026-10-01: 'r' = 扫描回放 开 / 关（见 RelayCore::startSweep 与 SweepPlan.h）。
    //   ⚠ 只在【按住按钮1】时才会真的叠加偏移；松开按钮1 即不叠加（偏移那一处自己判 lastButtonState）。
    if (key == 'r' || key == 'R') {
        if (RelayCore::instance().sweepRunning()) RelayCore::instance().stopSweep();
        else                                      RelayCore::instance().startSweep();
        return;
    }
```

- [ ] **Step 2: 段切换时打一行标记**

在 Task 3 Step 4 那段里、算出 `sp` 之后加（**用一个成员记住上一段的段号**，`int m_sweepLastSeg = -1;`，`startSweep()` 里复位）：

```cpp
        if (sp.seg != m_sweepLastSeg) {
            m_sweepLastSeg = sp.seg;
            std::cout << "[Sweep] 段 " << (sp.seg + 1) << "/" << SweepPlan::kSegCount
                      << "  speed=" << sp.speed << "x  ff=" << (sp.ffOn ? "ON" : "OFF")
                      << "  (源主频 " << SweepWaveform::kF0Hz << "Hz -> 约 "
                      << (SweepWaveform::kF0Hz * sp.speed) << "Hz)\n";
        }
```

⚠ **这行必须在 `cout` 上是安全的**：它跑在 ServoP 那条路上（触觉/GLUT 线程）。**只在该段第一次成立**，不是每帧。

- [ ] **Step 3: 编译** ⇒ 同 Task 3 Step 5。Expected: 退出码 0。

- [ ] **Step 4: 跑整床** ⇒ `exit 0`、`29 of 29`。

- [ ] **Step 5: 提交**

```bash
git add Touch_Client/main.cpp
git commit -m "feat(sweep): 'r' 键开关 + 段标记打印" -m "照抄 'w' 那一段的形状。段标记只在该段第一次成立时打, 不是每帧。"
```

---

### Task 5: 上机执行单（本功能的判据）

**Files:**
- Create: `Docs/superpowers/specs/2026-10-01-sweep-on-machine-run-sheet.md`

- [ ] **Step 1: 写执行单**，必须含：

| 项 | 内容 |
|---|---|
| 前置 | exe 必须**当天重建**（`SWEEP_REPLAY_ENABLED` 是编译期常量）· 旧 `force_wave.csv` 改名 · `S.logAllFast=false` |
| 操作 | **按住按钮1** → 按 `r` → 保持 80 秒别动别松 → 按 `n` 落盘（80s > 环的 8.3s ⇒ **中途按 3~4 次 `n`**，每次只留最近 8.3s！）|
| ⚠ 关键 | **环只有 1024 帧 ≈ 8.3 s**，而一轮 80 s ⇒ **必须在段与段之间按 `n`**，否则数据丢 |
| 段标记 | 控制台 `[Sweep] 段 N/8 speed=... ff=... (源主频 ... -> 约 ...Hz)` —— **分析按它切段** |
| 停止 | 再按 `r` / 松开按钮1 ⇒ 立即停 |
| 回落 | 跑完在 MATLAB 里把 `swFF` 勾选框**点回一致**（客户端改过它，MATLAB 不知道）|
| 判据 | **不变**：`F(f)` 归一化后 —— 恒定⇒惯性 / 随频率降⇒结构振动 / 无关⇒噪声源 |
| ⚠ 必须记 | **用 `act` 实测频率, 不用名义倍速** —— 臂跟不上时两者会差开 |

- [ ] **Step 2: 提交**

```bash
git add Docs/superpowers/specs/2026-10-01-sweep-on-machine-run-sheet.md
git commit -m "docs(sweep): 上机执行单 —— 含'环只有 8.3s 而一轮 80s'那条陷阱"
```

---

## 报告要求（每个 Task 都要）

追加到 `.superpowers/sdd/sweep-replay-report.md`（**Task 号分段**），必须含：

1. **红 → 绿**的原文（哪些用例、什么输出）；
2. **负对照实测**的原文（Task 1 三条、Task 2 三条）；
3. 整床命令与输出原文；
4. **没做到 / 做不到的事** —— 尤其：**Task 3/4 的接线没有自动化覆盖**，
   **Task 1 不构成"回放功能有测试了"**（只测了纯的那半）。
