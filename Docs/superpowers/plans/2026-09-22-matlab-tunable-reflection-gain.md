# 力反射增益 —— MATLAB 界面可调 · 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把 `Config::FORCE_REFLECTION_GAIN`（现在 120，编译期常量）变成能在 MATLAB 界面上实时改、跨重启保留、且 GUI 显示值必然等于实际生效值的运行时参数。

**Architecture:** 新增 `force/ForceTuning` 模块持有 `std::atomic<double>` 目标值（静态初值取自 `Config.h`，**不读文件**）。`ForcePipeline::step` 从它读增益，并在本模块内加一道 0.25 s 斜坡。MATLAB↔C++ 复用已有的 `relay socket` 反向命令通道（`RelayCommandParser` + `RelayCore::pollRelayCommands`），新增 `RG|` 双向协议 —— C++ 在连接时、以及每收到一条 `RG|` 后回读**当前实际生效值**，MATLAB 只显示 C++ 说的数。

**Tech Stack:** C++17 / MSVC (BuildTools 2022 x64) · Win32 手写 JSON（无第三方库）· MATLAB uifigure (App Building) · cmd `.bat` 测试床

## Global Constraints

- **规格文件**：`Docs/superpowers/specs/2026-09-22-matlab-tunable-reflection-gain-design.md` —— 本计划的唯一需求出处。与代码冲突时以规格为准，并回头改规格。
- **增益范围**：`GAIN_MIN = 100.0`，`GAIN_MAX = 300.0`，默认 = `Config::FORCE_REFLECTION_GAIN`（现值 `120.0`，`Config.h:223`）。范围**只有一份定义**（在 `ForceTuning.h`）。
- **协议前缀**：已占用 `B C D F FB G H J L P RP S W`。本计划只用 **`RG|`** 与 **`Z|`**。⚠ **`G|` 已被占用**（`relay_gui.m:506`），不要用它。
- **`.bat` 必须纯 ASCII**。非 ASCII 注释在非 UTF-8 代码页下会被 cmd.exe 误解码并**吞掉下一行**。检查它时**不要用 `grep -P`**（在这个仓库里会假绿）。
- **cmd 语义**：判定必须写 `if !ERRORLEVEL! EQU 0`（`setlocal enabledelayedexpansion` 已开）。`if errorlevel 1` 会把崩溃的负退出码判成通过；块内 `%ERRORLEVEL%` 是解析期冻结的。
- **常量引用规矩**：本计划里所有引用 `Config.h` 的常数都**在写计划时回核过**。实施时若发现与代码不符，以代码为准并改本计划。
- **不许静默**：读文件失败 / 值被拒 / 落盘失败，都必须在控制台或日志里出声。

---

### Task 1: `ForceTuning` 模块 + 抽出共享的 `JsonLite::find`

**Files:**
- Create: `Touch_Client/core/JsonLite.h`
- Create: `Touch_Client/force/ForceTuning.h`
- Create: `Touch_Client/force/ForceTuning.cpp`
- Modify: `Touch_Client/force/ForceCalibration.cpp:532-540`（删掉 static `jsonFind`，改用它）
- Modify: `Touch_Client/Touch_Client.vcxproj`（加 `force\ForceTuning.h` / `force\ForceTuning.cpp`）
- Test: `Touch_Client/tests/test_force_tuning.cpp`
- Test: `Touch_Client/tests/build_force_tuning_test.bat`

**Interfaces:**
- Consumes: `Config::FORCE_REFLECTION_GAIN`（`Config.h:223`，值 `120.0`）、`CalibStore::fileFor`（`core/CalibStore.h:25`）
- Produces: `ForceTuning::{GAIN_MIN, GAIN_MAX, defaultGain, gain, setGain, parseGainJson, saveToFile, loadFromFile, loadOnStartup, tick}`、`JsonLite::find`

- [ ] **Step 1: 建 `core/JsonLite.h`（抽出共享的 JSON 取值助手）**

为什么要抽：`ForceCalibration.cpp` 里那个 `jsonFind` 是 `static` 的（文件私有），本任务需要一个同形状的函数。
**不另写一份** —— 本项目对"同一个规则两份实现、改一份忘一份"有成文教训（`force/ForcePipeline.h:21-25` 逐字写了这条）。
这个助手是通用的（不是数值规则），但抽出来只需 6 行改动，比留下一份重复更便宜。

```cpp
#pragma once
#include <cstring>

// 极简 JSON 取值助手 —— 【全程序唯一一份】。
// 从前 ForceCalibration.cpp 里有一份 static jsonFind; ForceTuning 又要一个同形状的,
// 于是抽到这里, 两边共用。别在别处再写第三份。
//
// 语义: 找到 key (含引号, 如 "\"version\"") 之后的第一个非空字符, 跳过 ':'。
//   返回指向 '[' 或第一个数字/'-' 的指针 (供 strtod / 数组解析接着读);
//   找不到 key 时返回 nullptr。
// ⚠ 这不是一个合规的 JSON 解析器: 它不认转义、不认嵌套、不管 key 出现在字符串值里
//   的情况。只用于读【本程序自己写出来的】那种极简文件。
namespace JsonLite {
    inline const char* find(const char* buf, const char* key) {
        if (!buf || !key) return nullptr;
        const char* p = strstr(buf, key);
        if (!p) return nullptr;
        p += strlen(key);
        while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r') p++;
        if (*p == ':') p++;
        while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r') p++;
        return p;
    }
}
```

- [ ] **Step 2: 先让 `ForceCalibration.cpp` 用它 —— 并【先跑测试看它是绿的】**

这一步**先做、先验证**，因为 `ForceCalibration` 是已标定过的关键模块，而 `test_payload_calibration`
**不在测试床里**（它有刻意留红的断言）⇒ 它出错不会被自动拦住。先拿到基线。

Run: `Touch_Client\tests\build_force_comp_test.bat` 然后 `test_force_compensation.exe`
Expected: 记下当前的 `N passed, M failed` 数字。**下一步改完必须一模一样。**

- [ ] **Step 3: 改 `ForceCalibration.cpp` 用 `JsonLite::find`**

删掉 `ForceCalibration.cpp:532-540` 那个 static `jsonFind` 定义（含它上面那行注释），
在文件顶部的 include 区加 `#include "../core/JsonLite.h"`，然后把 3 个调用点改名：

| 行 | 现在 | 改成 |
|---|---|---|
| `:590` | `jsonFind(buf, "\"version\"")` | `JsonLite::find(buf, "\"version\"")` |
| `:604` | `jsonFind(buf, "\"a_matrix\"")` | `JsonLite::find(buf, "\"a_matrix\"")` |
| `:610` | `jsonFind(buf, "\"bias_force_n\"")` | `JsonLite::find(buf, "\"bias_force_n\"")` |

⚠ 用 grep 确认没有第 4 个调用点：`grep -n "jsonFind" Touch_Client/force/ForceCalibration.cpp`
（`jsReadArray` 上面那句注释里提到过它的名字，那处**是注释，不要改**。）

- [ ] **Step 4: 重跑同一个测试 —— 必须与 Step 2 数字完全一致**

Run: `Touch_Client\tests\build_force_comp_test.bat` 然后 `test_force_compensation.exe`
Expected: **与 Step 2 记录的 passed/failed 数字逐字相同**。不一致就回退这一步，别往下走。

- [ ] **Step 5: 写失败的测试 —— `test_force_tuning.cpp`**

⚠ 文件顶部必须写下这条：**本测试【不调】`loadOnStartup()`**。
理由：它读的是 `CalibStore` 的真实路径，会被现场调参污染 ⇒ 测试红绿漂移。
（本项目已有一次同形态：`tests/test_force_pipeline.cpp:98` 那句"那两个常数被重调时它会一直绿"。）

```cpp
// Standalone test: ForceTuning — MATLAB 可调增益的模块
// Build: build_force_tuning_test.bat
// Run:   test_force_tuning.exe
//
// !! 本测试【不调】ForceTuning::loadOnStartup() !!
//    它读的是 CalibStore 的真实路径 (calib/force_tuning.json), 会被【现场调参】污染 ——
//    昨天在 MATLAB 上拖到 300, 今天的测试就跟着变。测试必须只看静态初值 (Config 的 120)。
//    读写用例一律用【显式路径的临时文件】, 不用模块级的那条路径。

#include <iostream>
#include <cstdio>
#include <cmath>
#include <string>
#include "../force/ForceTuning.h"
#include "../config/Config.h"     // 用例要直接核 Config::FORCE_REFLECTION_GAIN

static int g_passed = 0, g_failed = 0;

#define TEST(name) do { std::cout << "  " << #name << "... "; } while(0)
#define PASS() do { std::cout << "PASS" << std::endl; g_passed++; } while(0)
#define CHECK(cond) do { if (!(cond)) { std::cout << "FAIL: " << #cond << std::endl; g_failed++; return; } } while(0)

static const char* kTmp = "_tuning_test_tmp.json";

// ===== 纯解析 =====

static void test_parse_ok() {
    TEST(parse_ok);
    double v = 0.0;
    CHECK(ForceTuning::parseGainJson("{\n \"version\": 1,\n \"reflection_gain\": 250.0\n}\n", &v));
    CHECK(fabs(v - 250.0) < 1e-9);
    PASS();
}

static void test_parse_wrong_version() {
    TEST(parse_wrong_version);
    double v = 0.0;
    CHECK(!ForceTuning::parseGainJson("{\n \"version\": 2,\n \"reflection_gain\": 250.0\n}\n", &v));
    PASS();
}

static void test_parse_missing_version() {
    TEST(parse_missing_version);
    double v = 0.0;
    CHECK(!ForceTuning::parseGainJson("{\n \"reflection_gain\": 250.0\n}\n", &v));
    PASS();
}

static void test_parse_out_of_range() {
    TEST(parse_out_of_range);
    double v = 0.0;
    // 低于下限
    CHECK(!ForceTuning::parseGainJson("{\"version\":1,\"reflection_gain\":99.9}", &v));
    // 高于上限
    CHECK(!ForceTuning::parseGainJson("{\"version\":1,\"reflection_gain\":300.1}", &v));
    PASS();
}

static void test_parse_nan_inf() {
    TEST(parse_nan_inf);
    double v = 0.0;
    // strtod 会把 "nan" / "inf" 解析成非有限数 ⇒ 必须被 isfinite 挡住
    CHECK(!ForceTuning::parseGainJson("{\"version\":1,\"reflection_gain\":nan}", &v));
    CHECK(!ForceTuning::parseGainJson("{\"version\":1,\"reflection_gain\":inf}", &v));
    PASS();
}

static void test_parse_garbage() {
    TEST(parse_garbage);
    double v = 0.0;
    CHECK(!ForceTuning::parseGainJson("not json at all", &v));
    CHECK(!ForceTuning::parseGainJson("{\"version\":1,\"reflection_gain\":}", &v));
    CHECK(!ForceTuning::parseGainJson("", &v));
    CHECK(!ForceTuning::parseGainJson(nullptr, &v));
    PASS();
}

// ===== setGain 与范围 =====

static void test_set_gain_bounds() {
    TEST(set_gain_bounds);
    ForceTuning::setGain(250.0);
    CHECK(fabs(ForceTuning::gain() - 250.0) < 1e-9);

    CHECK(ForceTuning::setGain(ForceTuning::GAIN_MIN));      // 下限本身接受
    CHECK(fabs(ForceTuning::gain() - 100.0) < 1e-9);
    CHECK(ForceTuning::setGain(ForceTuning::GAIN_MAX));      // 上限本身接受
    CHECK(fabs(ForceTuning::gain() - 300.0) < 1e-9);

    ForceTuning::setGain(200.0);
    CHECK(!ForceTuning::setGain(99.9));                       // 拒收
    CHECK(fabs(ForceTuning::gain() - 200.0) < 1e-9);          // 且【值不变】
    CHECK(!ForceTuning::setGain(300.1));
    CHECK(fabs(ForceTuning::gain() - 200.0) < 1e-9);
    CHECK(!ForceTuning::setGain(std::nan("")));
    CHECK(fabs(ForceTuning::gain() - 200.0) < 1e-9);
    CHECK(!ForceTuning::setGain(INFINITY));
    CHECK(fabs(ForceTuning::gain() - 200.0) < 1e-9);
    PASS();
}

static void test_static_initial_value_is_legal() {
    TEST(static_initial_value_is_legal);
    // 防默认值悄悄漂走。
    CHECK(fabs(ForceTuning::defaultGain() - Config::FORCE_REFLECTION_GAIN) < 1e-9);
    // 防"恢复默认"按钮发出一个被 setGain 自己拒收的值 —— 那在界面上表现为"按了没反应"。
    CHECK(ForceTuning::defaultGain() >= ForceTuning::GAIN_MIN);
    CHECK(ForceTuning::defaultGain() <= ForceTuning::GAIN_MAX);
    PASS();
}

// ===== 落盘 / 读回 (显式路径, 不碰现场文件) =====

static void test_save_load_roundtrip() {
    TEST(save_load_roundtrip);
    CHECK(ForceTuning::saveToFile(kTmp, 275.5));
    double v = 0.0;
    CHECK(ForceTuning::loadFromFile(kTmp, &v));
    CHECK(fabs(v - 275.5) < 1e-6);
    remove(kTmp);
    PASS();
}

static void test_missing_file_is_quiet_false() {
    TEST(missing_file_is_quiet_false);
    double v = 0.0;
    // 文件不存在 = 还没调过, 正常路径 ⇒ 返回 false 但不出声 (出声的是"文件在、内容不合规")
    CHECK(!ForceTuning::loadFromFile("_definitely_not_here_12345.json", &v));
    PASS();
}

static void test_corrupt_file_rejected() {
    TEST(corrupt_file_rejected);
    FILE* f = fopen(kTmp, "w");
    CHECK(f != nullptr);
    fprintf(f, "{\n \"version\": 1,\n \"reflection_gain\": 9999.0\n}\n");   // 越界
    fclose(f);
    double v = 0.0;
    CHECK(!ForceTuning::loadFromFile(kTmp, &v));
    remove(kTmp);
    PASS();
}

int main() {
    std::cout << "=== ForceTuning Tests ===" << std::endl;
    test_parse_ok();
    test_parse_wrong_version();
    test_parse_missing_version();
    test_parse_out_of_range();
    test_parse_nan_inf();
    test_parse_garbage();
    test_set_gain_bounds();
    test_static_initial_value_is_legal();
    test_save_load_roundtrip();
    test_missing_file_is_quiet_false();
    test_corrupt_file_rejected();
    std::cout << std::endl;
    std::cout << g_passed << " passed, " << g_failed << " failed" << std::endl;
    return g_failed > 0 ? 1 : 0;
}
```

- [ ] **Step 6: 写 `build_force_tuning_test.bat`（纯 ASCII！）**

照 `build_force_pipeline_test.bat` 的样子写，多链 `ForceTuning.cpp` 与 `CalibStore.cpp`。

```bat
@echo on
rem 2026-09-22: guard the vcvarsall call -- PATH grows ~1350 chars per call and cmd dies at ~8191,
rem   so a second unguarded call in the same session can abort the rest of run_tests.bat.
if not defined VCINSTALLDIR call "D:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvarsall.bat" x64
cd /d "D:\Projects\Touch\Touch_Client\tests"
rem Linked in on purpose: ForceTuning.cpp is the unit under test; CalibStore.cpp satisfies
rem   the linker for the CalibStore::fileFor references inside loadOnStartup/tick.
rem   ForcePipeline.cpp is NOT linked here -- Task 2 has its own test for the wiring.
rem NOTE: keep this file ASCII-only. Non-ASCII comments in a .bat get mis-decoded by
rem cmd.exe under a non-UTF-8 codepage and can silently swallow the following line.
cl /EHsc /std:c++17 /I"D:\Projects\Touch\OpenHaptics\Developer\3.5.0\include" /I"D:\Projects\Touch\OpenHaptics\Developer\3.5.0\utilities\include" /DWIN32 /DWIN32_LEAN_AND_MEAN /DNOMINMAX /D_CRT_SECURE_NO_WARNINGS /D_WINSOCK_DEPRECATED_NO_WARNINGS test_force_tuning.cpp ..\force\ForceTuning.cpp ..\core\CalibStore.cpp /Fe:test_force_tuning.exe
echo BUILD_EXIT=%ERRORLEVEL%
```

- [ ] **Step 7: 跑测试看它【失败】（还没实现）**

Run: `Touch_Client\tests\build_force_tuning_test.bat`
Expected: 编译失败（`ForceTuning.h` 不存在 / `ForceTuning` 未定义）。

- [ ] **Step 8: 写 `force/ForceTuning.h`**

```cpp
#pragma once

// MATLAB 可调的力反射增益 (2026-09-22)。
//
// 【本模块只管目标值】校验 + 持久化 + 防抖落盘。
//   增益【斜坡】(0.25s 平滑到位) 是信号处理, 在 ForcePipeline::step 里, 不在这里 ——
//   所以本模块的 gain() 返回的是【目标值】, MATLAB 回读报的也是它 (否则界面上的数字会自己动)。

namespace ForceTuning {
    // ===== 可取范围 —— 【唯一一份定义】=====
    // MATLAB 滑条的上下限、被拒提示里的数字, 全部由 RG| 回读下发, 不在这里之外再写一遍。
    //   下限 100 ⇒ 界面上试不了 120 以下 (要更低只能改 Config.h)。
    constexpr double GAIN_MIN = 100.0;
    constexpr double GAIN_MAX = 300.0;

    // 落盘前的静默期 (ms)。拖动滑条会产生几十条命令/秒, 不可能每条都写盘。
    // 代价如实说: 拖完立刻杀进程, 最后 1 秒的改动会丢。
    constexpr unsigned long TUNING_DEBOUNCE_MS = 1000;

    // 出厂默认 = Config::FORCE_REFLECTION_GAIN (Config.h:223)。
    // ⚠ 【优先级】calib/force_tuning.json > Config.h 的默认值。
    //   改了 Config.h 却"没反应", 先看这一行和启动横幅打出的来源。
    double defaultGain();

    // 当前生效的【目标值】。线程安全 (atomic)。
    double gain();

    // 校验 [GAIN_MIN, GAIN_MAX] 且有限 → 过则 store + 标脏, 返回 true; 否则不改任何状态。
    bool setGain(double v);

    // 纯函数: 从 json 文本解出 gain。version 必须是 1; 值必须有限且落在范围内。
    // 任何不合规都返回 false (调用方负责出声)。
    bool parseGainJson(const char* text, double* outGain);

    // 显式路径版本 —— 测试用, loadOnStartup / tick 也复用它们。
    bool saveToFile(const char* path, double gain);
    bool loadFromFile(const char* path, double* outGain);

    // 启动时调一次: 用 CalibStore 的路径读, 读到了就装上, 并打一行横幅写明【值 + 来源】。
    // ⚠ 必须在 RelayCore::initRelayReporting() 【之前】调用 —— 理由见 main.cpp 那一处的注释。
    void loadOnStartup();

    // 防抖落盘: 值变过 且 距上次改动 ≥ TUNING_DEBOUNCE_MS 才写。
    // 由 RelayCore::pollRelayCommands() 每帧调用 (借现成的空闲循环当心跳, 不新起线程)。
    void tick();
}
```

- [ ] **Step 9: 写 `force/ForceTuning.cpp`**

```cpp
#include "ForceTuning.h"
#include "../config/Config.h"
#include "../core/CalibStore.h"
#include "../core/JsonLite.h"
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <windows.h>

namespace ForceTuning {

// ★★ 静态初值直接来自 Config.h, 【不读文件】。
//   这是"测试不被现场调参污染"的落点: 只有显式的 loadOnStartup() 才会改它,
//   而测试【不调】loadOnStartup() (见 tests/test_force_tuning.cpp 顶部那段)。
static std::atomic<double> s_gain{ Config::FORCE_REFLECTION_GAIN };

// 防抖状态。只由 GLUT idle 线程碰 (setGain 与 tick 都在那里) ⇒ 不需要原子。
// 用独立的 bool 而不是"时间戳 == 0"当哨兵: 开机后第一秒 GetTickCount() 可能真是 0。
static bool  s_dirty = false;
static DWORD s_dirtyMs = 0;

double defaultGain() { return Config::FORCE_REFLECTION_GAIN; }
double gain() { return s_gain.load(); }

static bool inRange(double v) {
    return std::isfinite(v) && v >= GAIN_MIN && v <= GAIN_MAX;
}

bool setGain(double v) {
    if (!inRange(v)) return false;
    s_gain.store(v);
    s_dirty = true;
    s_dirtyMs = GetTickCount();
    return true;
}

bool parseGainJson(const char* text, double* outGain) {
    if (!text || !outGain) return false;

    // 【版本先判】顺序要紧: 先读值再判版本, 就会在返回 false 之前把半个结果写进调用方。
    // 教训出处: ForceCalibration::loadFromFile 顶上那段 (同一个理由)。
    const char* pv = JsonLite::find(text, "\"version\"");
    if (!pv) return false;
    if (strtol(pv, nullptr, 10) != 1) return false;

    const char* pg = JsonLite::find(text, "\"reflection_gain\"");
    if (!pg) return false;
    char* end = nullptr;
    const double v = strtod(pg, &end);
    if (end == pg) return false;   // 那里根本不是个数
    // ⚠ strtod 会把 "nan" / "inf" 都解析成功 ⇒ inRange 里的 isfinite 是必需的, 不是装饰。
    if (!inRange(v)) return false;

    *outGain = v;
    return true;
}

bool saveToFile(const char* path, double g) {
    FILE* f = fopen(path, "w");
    if (!f) return false;
    fprintf(f, "{\n  \"version\": 1,\n  \"reflection_gain\": %.6g\n}\n", g);
    fclose(f);
    return true;
}

bool loadFromFile(const char* path, double* outGain) {
    FILE* f = fopen(path, "r");
    if (!f) return false;   // 文件不存在 = 还没调过, 正常路径, 不吵
    char buf[1024];
    const size_t n = fread(buf, 1, sizeof(buf) - 1, f);
    fclose(f);
    if (n == 0) {
        fprintf(stderr, "[Tuning] !! force_tuning.json 【是空文件, 已忽略】: %s\n", path);
        fflush(stderr);
        return false;
    }
    buf[n] = '\0';
    if (!parseGainJson(buf, outGain)) {
        // 响亮地说出"这份文件不是本格式 / 值不可用" —— 不许安静地退化成"用默认值"。
        // 为什么必须响: "用默认值"与"读到了默认值"在控制台上长得一样, 而它们要做的事
        //   完全不同 (前者要去查文件, 后者什么都不用做)。
        fprintf(stderr,
                "[Tuning] !! force_tuning.json 【不可用, 已忽略】: %s\n"
                "[Tuning] !!   期望 { \"version\": 1, \"reflection_gain\": <%.0f..%.0f 之间的有限数> }\n"
                "[Tuning] !!   处置: 本次用 Config.h 的默认值。要重建这份文件, 在 MATLAB 上拖一下滑条即可。\n",
                path, GAIN_MIN, GAIN_MAX);
        fflush(stderr);
        return false;
    }
    return true;
}

void loadOnStartup() {
    const char* path = CalibStore::fileFor("force_tuning.json");
    double v = 0.0;
    if (loadFromFile(path, &v)) {
        s_gain.store(v);
        printf("[Tuning] 力反射增益 = %.1f (来源: %s)\n", v, path);
    } else {
        // 走到这里有两种可能: 文件不存在 (不吵过, 正常), 或文件在但不可用 (上面已响亮报过)。
        // 这一行把【来源】写出来, 这样"改了 Config.h 却没反应"在第一行日志里就有答案。
        printf("[Tuning] 力反射增益 = %.1f (来源: Config.h 默认值; 未采用 %s)\n",
               defaultGain(), path);
    }
}

void tick() {
    if (!s_dirty) return;
    const DWORD now = GetTickCount();
    if ((now - s_dirtyMs) < TUNING_DEBOUNCE_MS) return;   // 还在动, 再等等

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

} // namespace ForceTuning
```

- [ ] **Step 10: 跑测试，确认全绿**

Run: `Touch_Client\tests\build_force_tuning_test.bat` 然后 `test_force_tuning.exe`
Expected: `11 passed, 0 failed`

⚠ 若 `test_static_initial_value_is_legal` 失败 ⇒ 有人把 `Config.h:223` 改到了 `[100,300]` 之外，
那是**真发现**，不要改测试去迁就它。

- [ ] **Step 11: 在 `Config.h:223` 补一句优先级（**两处都要写，这是第二处**）**

`Config.h:223` 现在是：

```cpp
    const double FORCE_REFLECTION_GAIN = 120.0;  // 力反射增益 (净比例 ≈2:1)
```

改成：

```cpp
    // ⚠★ 2026-09-22 起本值【不再是真值】, 只是【出厂默认 / 兜底值】。
    //   【优先级】calib/force_tuning.json  >  本值。
    //   运行时真值住在 ForceTuning (force/ForceTuning.h) —— 在 MATLAB 界面上可改、跨重启保留。
    //   ⇒ 改了本行却"没反应"是【正常】的: 去看启动横幅那一行 "[Tuning] 力反射增益 = ..., 来源: ..."。
    //   为什么这句必须写在这里而不只写在 ForceTuning.h 里: 下一个人会先来改这一行,
    //   而"改了没反应"正是本项目反复出现的"旧前提"形态。
    const double FORCE_REFLECTION_GAIN = 120.0;  // 力反射增益: 出厂默认 (净值 ≈2:1; 运行时真值见 ForceTuning)
```

⚠ **上面那段注释里的 `Config.h:223` 行号要在改之前回核**（`grep -n "FORCE_REFLECTION_GAIN = 120" Touch_Client/config/Config.h`）——
本计划和规格里所有行号都是**写计划时**核的，改过几轮之后会漂。

- [ ] **Step 12: 把两个新文件加进 `Touch_Client.vcxproj`**

在 `Touch_Client.vcxproj` 里，紧挨着已有的 force 组条目（搜 `ForcePipeline.h`）加：

```xml
    <ClInclude Include="core\JsonLite.h" />
    <ClInclude Include="force\ForceTuning.h" />
```

并在 `ClCompile` 组里（搜 `ForcePipeline.cpp`）加：

```xml
    <ClCompile Include="force\ForceTuning.cpp" />
```

- [ ] **Step 13: 提交**

```bash
git add Touch_Client/core/JsonLite.h Touch_Client/force/ForceTuning.h Touch_Client/force/ForceTuning.cpp \
        Touch_Client/force/ForceCalibration.cpp Touch_Client/config/Config.h \
        Touch_Client/Touch_Client.vcxproj \
        Touch_Client/tests/test_force_tuning.cpp Touch_Client/tests/build_force_tuning_test.bat
git commit -m "feat(tuning): ForceTuning 模块 + 抽出 JsonLite::find

增益可调的地基: atomic 目标值, 静态初值取自 Config.h 的 120 且【不读文件】,
故现场调参不会污染测试 (测试不调 loadOnStartup)。

顺带把 ForceCalibration.cpp 里 static 的 jsonFind 抽成 core/JsonLite.h ——
本模块需要同形状的助手, 与其写第二份, 不如让两边共用一份。
改前改后都跑了 test_force_compensation, 数字一致。"
```

---

### Task 2: 把增益接进 `ForcePipeline`（含 0.25 s 斜坡）+ 净比例/饱和阈值的唯一一份定义

**Files:**
- Modify: `Touch_Client/force/ForcePipeline.cpp:126-136`（符号那段之后、增益那里）
- Modify: `Touch_Client/force/ForcePipeline.cpp:54-73`（`init()` 里让斜坡就位）
- Modify: `Touch_Client/force/ForcePipeline.cpp:139-143`（`shutdown()` 复位）
- Modify: `Touch_Client/force/ForcePipeline.h`（加 `netRatioPerGainUnit` / `saturationSensorN`）
- Modify: `Touch_Client/config/Config.h`（加 `FORCE_GAIN_SLEW_PER_S`）
- Modify: `Touch_Client/tests/build_force_pipeline_test.bat`（多链 `ForceTuning.cpp`、`CalibStore.cpp`）
- Test: `Touch_Client/tests/test_force_pipeline.cpp`（加两个用例）

**Interfaces:**
- Consumes: `ForceTuning::gain()`（Task 1）
- Produces: `ForcePipeline::netRatioPerGainUnit()`、`ForcePipeline::saturationSensorN(double gain)`、`Config::FORCE_GAIN_SLEW_PER_S`

- [ ] **Step 1: 先跑现有测试拿基线**

Run: `Touch_Client\tests\build_force_pipeline_test.bat` 然后 `test_force_pipeline.exe`
Expected: 记下 `N passed, M failed`。

⚠ **确认它真的被构建了**：看 `BUILD_EXIT=0`。这个套件踩过"陈旧二进制"的坑
（`build_force_pipeline_test.bat` 顶上那段注释就是为它写的）。

- [ ] **Step 2: 在 `Config.h` 加斜坡常数**

加在 `Config.h:285` 的 `FORCE_GRADIENT_LIMIT` **旁边**（它们是同类：都是信号处理保护）。

```cpp
    // 力反射增益的【斜坡速率】(增益单位/秒)。2026-09-22 新增, 与 FORCE_GRADIENT_LIMIT 同类。
    // 为什么要它: gain 从 120 拖到 300 时, 手上力会在一帧内变成 2.5 倍。
    //   总夹 (±3.3N, HapticCallback.cpp:228) 一直在, 所以不会失控, 但那股突变是"没预备"的。
    //   斜坡把它摊到 0.25 秒里 (800/秒 ⇒ 走完 100→300 的全程 200 个单位正好 0.25s)。
    // ⚠ 与 FORCE_GRADIENT_LIMIT 的分工: 那个作用在 filtered[] 上, 【管不到增益之后】;
    //   本常数作用在增益本身上。两者不重叠。
    const double FORCE_GAIN_SLEW_PER_S = 800.0;
```

- [ ] **Step 3: 在 `ForcePipeline.h` 加净比例与饱和阈值的唯一一份定义**

在 `namespace ForcePipeline {` 里、`softDeadzone` 之后加：

```cpp
    // ===== 净比例与打顶阈值 —— 【全程序唯一一份】=====
    // 净比例 = FORCE_MAX_TOUCH_N / FORCE_MAX_SENSOR_N (逐单位增益)。现值 3.3/200 = 0.0165。
    // 【为什么抽出来】MATLAB 的 RG| 回读要报 ratio 与 satN, 而这两个量都从 3.3/200 来。
    //   若在 RelayCore 里再写一遍 3.3 和 200, 就是"同一个规则两份实现" ——
    //   本项目有成文教训 (见上面 softDeadzone 那段)。
    inline double netRatioPerGainUnit() {
        return Config::FORCE_MAX_TOUCH_N / Config::FORCE_MAX_SENSOR_N;
    }

    // 该轴的打顶阈值 (传感器牛顿): 输出撞上 FORCE_MAX_TOUCH_N 那个夹子时的输入值。
    //   = FORCE_MAX_TOUCH_N / (netRatioPerGainUnit() × gain) = 200/gain (gain=120 ⇒ 1.67N)
    // ⚠ 两点前提, 界面上也要写:
    //   (1) 【该轴分量】—— HapticCallback.cpp:228-234 是三个轴各自夹, 不是夹合力。
    //   (2) 它依赖 FORCE_CONSTRAINT_FORCES_ENABLED = false (Config.h:702)。
    //       那个开关翻回 true ⇒ totalForce 变成叠加值 ⇒ 夹点提前 ⇒ 这个数当场作废。
    //       这是一句"当前状态"的结论, 不是恒等式。
    inline double saturationSensorN(double gain) {
        const double ratio = netRatioPerGainUnit() * gain;
        if (ratio <= 0.0) return 0.0;
        return Config::FORCE_MAX_TOUCH_N / ratio;
    }
```

`ForcePipeline.h` 顶上要加 `#include "../config/Config.h"`。

⚠ **顺序要紧**：`ForcePipeline.cpp:83` 现有的 `double ratio = Config::FORCE_MAX_TOUCH_N / Config::FORCE_MAX_SENSOR_N;`
要改成 `const double ratio = netRatioPerGainUnit();` —— 否则"唯一一份"就又是谎话。

- [ ] **Step 4: 在 `ForcePipeline.cpp` 加斜坡状态并在 `step()` 里用它**

`ForcePipeline.cpp:49-50` 那两行 static 旁边加：

```cpp
// 增益斜坡的当前值 (向 ForceTuning::gain() 逼近)。
// 【为什么状态在这里而不在 ForceTuning】斜坡是【信号处理】(与 FORCE_GRADIENT_LIMIT 同类),
//   不是"参数" —— ForceTuning 只持有目标值。分开的直接好处: MATLAB 回读报的是目标值,
//   界面上的数字不会自己动。
static double g_gainRamp = 0.0;
```

`ForcePipeline::init()` 里、循环之后加：

```cpp
    // 让斜坡直接就位 —— 否则启动时增益会从 0 爬到目标值 (那 0.25 秒里手上力是错的)。
    g_gainRamp = ForceTuning::gain();
```

`ForcePipeline::shutdown()` 里加：

```cpp
    g_gainRamp = 0.0;
```

把 `step()` 末尾那段（`ForcePipeline.cpp:130-136`）整体替换成：

```cpp
    // 5. Apply reflection gain (amplify for human perception)
    //    Typical contact forces (5-30N) → clearly perceptible (0.4-2.5N at Touch)
    //    Safety clamp at FORCE_MAX_TOUCH_N still applies in hapticCallback
    //
    //    ★ 2026-09-22: 增益来源从 Config 的编译期常量改成 ForceTuning 的运行时值,
    //      并加一道斜坡 (见 Config::FORCE_GAIN_SLEW_PER_S)。
    //      每帧最多动 (FORCE_GAIN_SLEW_PER_S / FORCE_FILTER_FS_HZ) 个增益单位。
    {
        const double target = ForceTuning::gain();
        const double maxStep = Config::FORCE_GAIN_SLEW_PER_S / (double)Config::FORCE_FILTER_FS_HZ;
        const double d = target - g_gainRamp;
        if (d > maxStep)       g_gainRamp += maxStep;
        else if (d < -maxStep) g_gainRamp -= maxStep;
        else                   g_gainRamp = target;

        for (int i = 0; i < 3; i++) {
            fd.hapticOut[i] *= g_gainRamp;
        }
    }
```

`ForcePipeline.cpp` 顶部加 `#include "ForceTuning.h"`。

- [ ] **Step 5: 改 `build_force_pipeline_test.bat` 多链两个 .cpp（纯 ASCII）**

把 `cl` 那一行的源文件列表从

```
test_force_pipeline.cpp ..\force\ForcePipeline.cpp
```

改成

```
test_force_pipeline.cpp ..\force\ForcePipeline.cpp ..\force\ForceTuning.cpp ..\core\CalibStore.cpp
```

并加一行 `rem`（ASCII）：

```bat
rem 2026-09-22: ForceTuning.cpp added (ForcePipeline now reads its gain from there);
rem   CalibStore.cpp satisfies the linker for ForceTuning's CalibStore::fileFor references.
rem   Without these two the link fails -- and a failed link must NOT be mistaken for a pass.
```

- [ ] **Step 6: 加两个用例到 `test_force_pipeline.cpp`**

顶部加 `#include "../force/ForceTuning.h"`，然后在 `main()` 里、`test_soft_deadzone...` 之前插入两个函数，
并在 `main()` 里按顺序调用。

```cpp
// 2026-09-22: 增益旋钮【真的接上了】的证据。
// 【为什么必须有这条】只断言 ForceTuning::setGain 的返回值不构成证据 —— 那证明的是
//   "模块内部一致", 不是"ForcePipeline 真的读它"。本项目踩过"先确认它真的被执行过"的亏。
static void test_gain_actually_changes_output() {
    TEST(gain_actually_changes_output);

    AppState::ForceData fd;
    for (int i = 0; i < 6; i++) { fd.compensated[i] = 0.0; }
    fd.compensated[0] = 1.0;              // 1.0 N 远在死区 0.20 之上 ⇒ softDeadzone 原样返回
    fd.lastUpdateMs = GetTickCount();

    // gain = 300: 输出 = 1.0 × (3.3/200) × 300 = 4.95, 横向符号 −1 ⇒ −4.95
    ForceTuning::setGain(300.0);
    ForcePipeline::init();                // 让斜坡直接就位 (不然要等 0.25s)
    for (int i = 0; i < 300; i++) ForcePipeline::step(fd);   // 让滤波器收敛
    CHECK(fabs(fd.hapticOut[0] - (-4.95)) < 0.05);

    // gain = 120: 同一个输入 ⇒ −1.98
    ForceTuning::setGain(120.0);
    ForcePipeline::init();
    for (int i = 0; i < 300; i++) ForcePipeline::step(fd);
    CHECK(fabs(fd.hapticOut[0] - (-1.98)) < 0.03);

    PASS();
}

// 斜坡: 增益不是一步到位, 而是 800/秒 (⇒ 100→300 走 0.25s = 31 帧 @125Hz)。
// 判据取一个区间而不是精确帧数 —— 精确值会随常数微调而红, 那不是缺陷。
static void test_gain_ramp_is_gradual() {
    TEST(gain_ramp_is_gradual);

    AppState::ForceData fd;
    for (int i = 0; i < 6; i++) { fd.compensated[i] = 0.0; }
    fd.compensated[0] = 1.0;
    fd.lastUpdateMs = GetTickCount();

    ForceTuning::setGain(100.0);
    ForcePipeline::init();                       // 斜坡就位在 100
    for (int i = 0; i < 300; i++) ForcePipeline::step(fd);   // 先让滤波器收敛
    const double at100 = fd.hapticOut[0];
    CHECK(fabs(at100 - (-1.65)) < 0.03);         // 1.0 × 0.0165 × 100 = 1.65

    // 跳到 300, 数多少帧才到位
    ForceTuning::setGain(300.0);
    int frames = 0;
    while (fabs(fd.hapticOut[0] - (-4.95)) > 0.05 && frames < 200) {
        ForcePipeline::step(fd);
        frames++;
    }
    CHECK(frames < 200);                         // 必须【到达】—— 卡住说明斜坡没在动
    CHECK(frames >= 20);                         // 必须【不是一步到位】—— 否则斜坡是假的
    CHECK(frames <= 45);                         // 也别慢得离谱 (理论 31 帧)

    ForceTuning::setGain(Config::FORCE_REFLECTION_GAIN);   // 复原, 免得污染后面的用例
    ForcePipeline::init();
    PASS();
}
```

- [ ] **Step 7: 跑测试，确认全绿**

Run: `Touch_Client\tests\build_force_pipeline_test.bat` 然后 `test_force_pipeline.exe`
Expected: Step 1 的 passed 数 **+2**，failed 数不变（0）。

⚠ 若 `coord_transform` 那条红了：它钉的是"Fz→Touch Y 那一轴为 0"，与本次改动无关 ——
说明有人动了符号，停下来核，不要改测试。

- [ ] **Step 8: 提交**

```bash
git add Touch_Client/force/ForcePipeline.cpp Touch_Client/force/ForcePipeline.h \
        Touch_Client/config/Config.h Touch_Client/tests/test_force_pipeline.cpp \
        Touch_Client/tests/build_force_pipeline_test.bat
git commit -m "feat(tuning): 增益接进 ForcePipeline + 0.25s 斜坡 + 净比例唯一一份定义

step() 从 ForceTuning::gain() 读增益 (不再读编译期常量), 并加一道斜坡
(FORCE_GAIN_SLEW_PER_S=800/s ⇒ 100→300 走 0.25s)。斜坡是信号处理, 故状态
在 ForcePipeline 而不在 ForceTuning ⇒ 回读报的是目标值, 界面数字不会自己动。

netRatioPerGainUnit()/saturationSensorN() 抽进 ForcePipeline.h —— 原来 3.3/200
只写在 mapForceToTouch 里, 而 RG| 回读也要报这两个量; 不抽就是两份实现。
mapForceToTouch 的 ratio 一并改用它。

两个新用例: 旋钮真的改变 step 输出 / 斜坡既到达又不是一步到位。"
```

---

### Task 3: 协议 —— `RelayCommandParser` 加 `RG|` 与 `Z|`

**Files:**
- Modify: `Touch_Client/relay/RelayCommandParser.h`
- Modify: `Touch_Client/relay/RelayCommandParser.cpp`
- Test: `Touch_Client/tests/test_relay_command_parser.cpp`（加用例）

**Interfaces:**
- Consumes: 无
- Produces: `RelayCommandParser::Command::{SetReflectionGain, ForceZero}`、`RelayCommandParser::parse(const char*, double* valueOut = nullptr)`

- [ ] **Step 1: 加失败用例到 `test_relay_command_parser.cpp`**

在 `main()` 之前插入，并在 `main()` 里按顺序调用（放在现有 11 条之后）。

```cpp
// ===== 2026-09-22 新增: 增益与调零 =====

static void test_rg_valid() {
    TEST(rg_valid);
    double v = 0.0;
    CHECK(RelayCommandParser::parse("RG|200.5", &v) == R::SetReflectionGain);
    CHECK(fabs(v - 200.5) < 1e-9);
    PASS();
}

static void test_rg_range_not_checked_here() {
    TEST(rg_range_not_checked_here);
    // 【解析器不管范围】—— 范围是 ForceTuning 的事 (单一定义)。解析器只负责"这是个合法的数"。
    // 这里钉住这条分工, 免得将来有人把 100/300 塞进解析器、于是范围有了第二份实现。
    double v = 0.0;
    CHECK(RelayCommandParser::parse("RG|5000", &v) == R::SetReflectionGain);
    CHECK(fabs(v - 5000.0) < 1e-9);
    PASS();
}

static void test_rg_missing_value_out_param() {
    TEST(rg_missing_value_out_param);
    // 没给 out 参数也不能崩
    CHECK(RelayCommandParser::parse("RG|200") == R::SetReflectionGain);
    PASS();
}

static void test_rg_invalid() {
    TEST(rg_invalid);
    double v = 0.0;
    CHECK(RelayCommandParser::parse("RG|", &v) == R::None);
    CHECK(RelayCommandParser::parse("RG|abc", &v) == R::None);
    CHECK(RelayCommandParser::parse("RG|200x", &v) == R::None);
    CHECK(RelayCommandParser::parse("RG|2 00", &v) == R::None);
    CHECK(RelayCommandParser::parse("RG|-", &v) == R::None);
    PASS();
}

static void test_zero_command() {
    TEST(zero_command);
    CHECK(RelayCommandParser::parse("Z|1") == R::ForceZero);
    CHECK(RelayCommandParser::parse("Z|1\n") == R::ForceZero);
    CHECK(RelayCommandParser::parse("Z|0") == R::None);   // 只有 1, 没有 0
    PASS();
}

static void test_rg_not_confused_with_ff() {
    TEST(rg_not_confused_with_ff);
    // FF| 与 RG| 是两条不同命令, 前缀不能互相吞
    CHECK(RelayCommandParser::parse("FF|1") == R::ForceFeedbackOn);
    double v = 0.0;
    CHECK(RelayCommandParser::parse("RG|1", &v) == R::SetReflectionGain);
    CHECK(fabs(v - 1.0) < 1e-9);   // 是 1.0, 【不是】被当成 FF|1
    PASS();
}
```

`main()` 里加这 6 个调用。

- [ ] **Step 2: 跑测试看它失败**

Run: `Touch_Client\tests\build_relay_command_test.bat` → `test_relay_command_parser.exe`
Expected: 编译失败（`SetReflectionGain` / `ForceZero` 未定义、`parse` 不接受第二参数）。

- [ ] **Step 3: 改 `RelayCommandParser.h`**

```cpp
#pragma once

// MATLAB → C++ 反向命令解析 (单行, 以 '|' 分隔)
// 协议:
//   "FF|0" 关力反馈, "FF|1" 开力反馈
//   "RG|<数值>" 设力反射增益 (范围由 ForceTuning 管, 【不在这里管】)
//   "Z|1"  力传感器调零 (与键盘 'z' 同语义: 再发一次 = 中止)
namespace RelayCommandParser {
    enum class Command {
        None,
        ForceFeedbackOn,
        ForceFeedbackOff,
        SetReflectionGain,
        ForceZero
    };

    // 解析一行命令。返回 Command::None 表示未知/空/非法输入。
    // valueOut: 仅 SetReflectionGain 时写入解析出的数值。
    //   【带默认值】—— 现有的 12 条测试一个字都不用改, 照旧编译通过并当回归网。
    Command parse(const char* line, double* valueOut = nullptr);
}
```

- [ ] **Step 4: 改 `RelayCommandParser.cpp`**

```cpp
#include "RelayCommandParser.h"
#include <cstring>
#include <cstdlib>

namespace RelayCommandParser {

    // 跳过前导空白, 返回第一个非空白字符。
    static const char* skipWs(const char* p) {
        while (*p == ' ' || *p == '\t') p++;
        return p;
    }

    // 值之后只允许空白或行尾。
    static bool tailIsClean(const char* p) {
        while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;
        return *p == '\0';
    }

    // "RG|<number>" -> 数值。整个 token 必须是一个数 (不允许 "200x" / "2 00")。
    static bool parseNumberToken(const char* p, double* out) {
        if (!p || *p == '\0') return false;
        char* end = nullptr;
        const double v = strtod(p, &end);
        if (end == p) return false;
        if (!tailIsClean(end)) return false;
        if (out) *out = v;
        return true;
    }

    Command parse(const char* line, double* valueOut) {
        if (line == nullptr) return Command::None;

        // 跳过前导空白
        while (*line == ' ' || *line == '\t' || *line == '\r' || *line == '\n') line++;
        if (*line == '\0') return Command::None;

        // ---- FF|<0|1> ----
        if (strncmp(line, "FF|", 3) == 0) {
            const char* p = skipWs(line + 3);
            if (*p != '0' && *p != '1') return Command::None;
            const char val = *p++;
            if (!tailIsClean(p)) return Command::None;
            return (val == '1') ? Command::ForceFeedbackOn : Command::ForceFeedbackOff;
        }

        // ---- RG|<数值> ----
        // ⚠ 【范围不在这里判】—— 100..300 只有一份定义, 在 ForceTuning::GAIN_MIN/GAIN_MAX。
        //   在这里再写一遍就是"同一个规则两份实现, 改一份忘一份"(本项目有成文教训)。
        //   本函数只保证"这是一个合法的数", 越界由 ForceTuning::setGain 拒收。
        if (strncmp(line, "RG|", 3) == 0) {
            const char* p = skipWs(line + 3);
            if (!parseNumberToken(p, valueOut)) return Command::None;
            return Command::SetReflectionGain;
        }

        // ---- Z|1 ----
        // 语义与键盘 'z' 一致 (再发一次 = 中止), 而"多次"这件事由调用方的状态决定,
        // 所以协议本身就是一条 "Z|1" —— 不搞 0/1 的开关形式。
        if (strncmp(line, "Z|", 2) == 0) {
            const char* p = skipWs(line + 2);
            if (*p != '1') return Command::None;
            if (!tailIsClean(p + 1)) return Command::None;
            return Command::ForceZero;
        }

        return Command::None;
    }
}
```

- [ ] **Step 5: 跑测试，确认全绿**

Run: `Touch_Client\tests\build_relay_command_test.bat` → `test_relay_command_parser.exe`
Expected: `17 passed, 0 failed`（原 11 + 新 6）

⚠ 原来的 11 条**必须依然全绿** —— 它们是这次改动的回归网。任何一条红了说明改动破坏了旧行为。

- [ ] **Step 6: 提交**

```bash
git add Touch_Client/relay/RelayCommandParser.h Touch_Client/relay/RelayCommandParser.cpp \
        Touch_Client/tests/test_relay_command_parser.cpp
git commit -m "feat(tuning): RelayCommandParser 加 RG| (设增益) 与 Z|1 (调零)

parse() 加了一个带默认值的 valueOut 参数 ⇒ 现有 12 条测试一个字不改、照旧当回归网。
不重构成通用的 type|value 结构: 那要重写现有测试而收益是零 (一共三条命令)。

【范围判据不在解析器里】—— 100..300 只有一份定义 (ForceTuning)。解析器
只保证"是个合法的数", 越界由 setGain 拒收。有专门用例钉住这条分工。"
```

---

### Task 4: `RelayCore` 接线 —— 分发、限频回读、连接时回读、调零标志

**Files:**
- Modify: `Touch_Client/relay/RelayCore.h:125-136` 附近（加 `sendReflectionGain`、`consumeForceZeroRequest`）
- Modify: `Touch_Client/relay/RelayCore.h:199-201`（加 `m_forceZeroRequested`）
- Modify: `Touch_Client/relay/RelayCore.cpp:1981-1996`（`dispatchRelayCommand`）
- Modify: `Touch_Client/relay/RelayCore.cpp:1998-2026`（`pollRelayCommands`）
- Modify: `Touch_Client/relay/RelayCore.cpp:1803-1822`（`initRelayReporting`）
- Modify: `Touch_Client/relay/RelayCore.cpp:1854-1870`（`ensureRelayConnected`）

**Interfaces:**
- Consumes: `ForceTuning::gain/tick`（Task 1）、`ForcePipeline::netRatioPerGainUnit/saturationSensorN`（Task 2）、`RelayCommandParser::{SetReflectionGain, ForceZero}`（Task 3）
- Produces: `RelayCore::sendReflectionGain(bool force)`、`RelayCore::consumeForceZeroRequest()`

- [ ] **Step 1: 改 `RelayCore.h`**

在 `void pollRelayCommands();`（`:136`）下面加：

```cpp
    // ===== 力反射增益回读 (RG| 协议, 2026-09-22) =====
    // 【唯一真值通道】两个方向都用 RG|: MATLAB 发 RG|<值> 设, C++ 在连接时、以及每收到
    //   一条 RG| 之后【不论接受还是拒绝】都回一条
    //     RG|<gain>,<min>,<max>,<ratio>,<deadN>,<satN>,<defGain>
    //   ⇒ MATLAB 永远不需要"记住"自己设过什么, 它只显示这里说的数
    //   ⇒"GUI 显示的值 ≠ 实际生效的值"这个状态【在结构上无法存在】。
    // 报的是【目标值】, 不是斜坡的瞬时值 (见 ForceTuning.h 顶上那段)。
    // 限频: force=false 时距上次 <100ms 只记下待发, 由 pollRelayCommands 每帧补发 —— 拖动
    //   滑条几十条/秒, 全回会堆在 MATLAB 侧; 而"最后一条一定到"由补发保证。
    void sendReflectionGain(bool force);

    // 调零请求 (Z| 协议)。只置标志 —— 真正的处置在 main.cpp 的 requestForceZero(),
    // 因为 g_noRobot / cancelOtherCaptureModes 都是那个文件的 file-static, 这里拿不到。
    // 【为什么不让 RelayCore 自己判】复制一份"标定中/调零中"的守卫链就是本项目最忌讳的
    //   两份实现; 而两条入口 (键盘 'z' / MATLAB) 共用同一个函数, 守卫链就只有一份。
    // 返回 true 表示本次调用消费掉了一个待处理请求 (读到即清)。
    bool consumeForceZeroRequest();
```

在 `private:` 区、`int m_relayRecvLen = 0;`（`:200`）下面加：

```cpp
    std::atomic<bool> m_forceZeroRequested{false};   // MATLAB 的 Zero 按钮 (由 dispatchRelayCommand 置)
```

- [ ] **Step 2: 改 `dispatchRelayCommand`（`RelayCore.cpp:1981-1996`）**

整体替换成：

```cpp
void RelayCore::dispatchRelayCommand(const char* line) {
    using R = RelayCommandParser::Command;
    double value = 0.0;
    switch (RelayCommandParser::parse(line, &value)) {
    case R::ForceFeedbackOn:
        appState.forceFeedbackEnabled = true;
        std::cout << "[Relay] Force feedback ENABLED (MATLAB command)" << std::endl;
        break;
    case R::ForceFeedbackOff:
        appState.forceFeedbackEnabled = false;
        std::cout << "[Relay] Force feedback DISABLED (MATLAB command)" << std::endl;
        break;
    case R::SetReflectionGain:
        if (ForceTuning::setGain(value)) {
            std::cout << "[Tuning] 力反射增益 → " << value << " (MATLAB command)" << std::endl;
        } else {
            // 拒收必须出声, 而且要说清范围 —— 范围取自 ForceTuning 那一份定义, 不另写数字。
            std::cout << "[Tuning] 增益 " << value << " 【被拒】: 可取范围 ["
                      << ForceTuning::GAIN_MIN << ", " << ForceTuning::GAIN_MAX
                      << "], 仍是 " << ForceTuning::gain() << std::endl;
        }
        // 【不论接受还是拒绝都回读】—— 回的是当前实际生效值。
        // 于是被拒时 MATLAB 会把滑条弹回真值, 而不是让界面继续显示一个假的数。
        sendReflectionGain(true);
        break;
    case R::ForceZero:
        // 只置标志: 真正的处置在 main.cpp 的 requestForceZero() (与键盘 'z' 同一个函数)。
        // 为什么不在这个线程直接做 —— 见 RelayCore.h 里 consumeForceZeroRequest 的说明。
        m_forceZeroRequested.store(true);
        std::cout << "[Tuning] 收到 MATLAB 的调零请求" << std::endl;
        break;
    case R::None:
    default:
        break;
    }
}
```

`RelayCore.cpp` 顶部加 `#include "../force/ForceTuning.h"` 与 `#include "../force/ForcePipeline.h"`。

- [ ] **Step 3: 加 `sendReflectionGain` 与 `consumeForceZeroRequest`**

放在 `dispatchRelayCommand` 上面（紧邻，便于一起读）：

```cpp
// 回读限频 (见 RelayCore.h 里的说明)。只由 GLUT idle 线程调 ⇒ 不需要原子。
static DWORD s_lastGainReportMs = 0;
static bool  s_gainReportPending = false;

void RelayCore::sendReflectionGain(bool force) {
    const DWORD now = GetTickCount();
    if (!force && (now - s_lastGainReportMs) < 100) {
        s_gainReportPending = true;   // 记下待发, 由 pollRelayCommands 补 —— 最后一条不丢
        return;
    }
    s_gainReportPending = false;
    s_lastGainReportMs = now;

    const double g = ForceTuning::gain();
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

bool RelayCore::consumeForceZeroRequest() {
    return m_forceZeroRequested.exchange(false);
}
```

- [ ] **Step 4: 改 `pollRelayCommands`（`RelayCore.cpp:1998-2026`）**

在函数体开头（`EnterCriticalSection` 之前）加：

```cpp
    // 力反射增益的两件家常事 (2026-09-22):
    //   tick()      —— 防抖落盘 (值变过且静默 ≥1s 才写盘)。借本循环当心跳, 不新起线程/定时器。
    //   补发待发的回读 —— 拖动滑条时被限频挡下的那一条, 在这里补上, 保证"最后一条一定到"。
    ForceTuning::tick();
    if (s_gainReportPending) sendReflectionGain(false);
```

- [ ] **Step 5: 连接时回读 —— 改 `initRelayReporting`（`:1803-1808`）**

```cpp
void RelayCore::initRelayReporting() {
    if (connectRelaySocket()) {
        std::cout << "[Relay] GUI reporting connected to " << Config::RELAY_IP
                  << ":" << Config::RELAY_PORT << std::endl;
        // ★ 2026-09-22: 连上就回读一次增益。必须有 —— 否则 MATLAB 在 C++ 启动时看到的
        //   是滑条的初值 (它自己猜的), 而实际生效值可能来自 force_tuning.json 的 300。
        //   那正是本设计要消灭的"GUI 显示的值 ≠ 实际生效的值"。
        sendReflectionGain(true);
        return;
    }
```

- [ ] **Step 6: 重连时也回读 —— 改 `ensureRelayConnected`（`:1854-1870`）**

把

```cpp
    if (!connectRelaySocket()) return false;
    s_relayDownReported = false;
    std::cout << "[Relay] GUI reporting 【已重连】到 " << Config::RELAY_IP << ":"
              << Config::RELAY_PORT << std::endl;
    return true;
```

改成

```cpp
    if (!connectRelaySocket()) return false;
    s_relayDownReported = false;
    std::cout << "[Relay] GUI reporting 【已重连】到 " << Config::RELAY_IP << ":"
              << Config::RELAY_PORT << std::endl;
    // ★ 重连后同样要回读增益: MATLAB 可能是在 C++ 之后才起来的, 那条 RG| 从没送到过。
    //   sendRelayUpdate 不调本函数 (它在 socket 无效时只返回 -1), 所以这里【不会递归】。
    sendReflectionGain(true);
    return true;
```

- [ ] **Step 7: 编译整个工程，确认没坏**

Run:
```bash
cd Touch_Client && "D:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvarsall.bat" x64 && msbuild Touch_Client.vcxproj /p:Configuration=Release /p:Platform=x64 /v:m
```
Expected: 编译通过，0 error。

⚠ 若报 `sendReflectionGain` 在 `ensureRelayConnected` 里未声明 —— 检查 Task 4 Step 1 的
头文件声明是否加在了 `public:` 区、且在 `ensureRelayConnected` 之前可见（头文件里的顺序无所谓，
顺序问题只可能是加到了 `private:` 里）。

- [ ] **Step 8: 跑全套测试，确认没有任何回归**

Run: `Touch_Client\tests\run_tests.bat`
Expected: **`Summary: 12 of 20 suites run - 12 OK, 0 FAILED`**（本任务还没接新套件，所以仍是 12/20）
⚠ 这套里 `test_safety_core` 有约 8% 的时序 flake（`Sleep(60)` 对 `MIN_WARN_MS=50` 只留 ~10ms
余量）。若只有它红，重跑一次确认；**单次绿不能证明一条红是假的，单次红也不一定是真的**。

- [ ] **Step 9: 提交**

```bash
git add Touch_Client/relay/RelayCore.h Touch_Client/relay/RelayCore.cpp
git commit -m "feat(tuning): RelayCore 接线 RG| 回读 + Z| 调零请求

- dispatchRelayCommand: RG| → setGain, 【不论接受还是拒绝都回读实际生效值】
- sendReflectionGain: 回读限频 100ms (拖动滑条几十条/秒), 待发的由 pollRelayCommands 补
- 连接时与每次重连后各回读一次 —— 否则 MATLAB 在 C++ 启动时看到的是它自己猜的初值
- Z| 只置标志: 守卫链在 main.cpp (g_noRobot / cancelOtherCaptureModes 是那里的 file-static),
  不在这里复制一份; 两条入口共用同一个函数 (Task 5)
- pollRelayCommands 开头加 ForceTuning::tick() —— 防抖落盘借现成循环当心跳

回读里的 ratio/deadN/satN 全部取各自【唯一一份定义】(ForcePipeline / Config), 本处不写数字。"
```

---

### Task 5: `main.cpp` 接线 —— 启动顺序（关键陷阱）+ 抽出 `requestForceZero`

**Files:**
- Modify: `Touch_Client/main.cpp`（顶部 include 区）
- Modify: `Touch_Client/main.cpp:2801-2822` 之间（在 `idle()` 之前加 `requestForceZero`）
- Modify: `Touch_Client/main.cpp:2823-2834`（`idle()` 里消费调零请求）
- Modify: `Touch_Client/main.cpp:3744-3756`（**在 `initRelayReporting` 之前**加载增益）
- Modify: `Touch_Client/main.cpp:3105-3128`（`'z'` 分支改调 `requestForceZero`）

**Interfaces:**
- Consumes: `ForceTuning::loadOnStartup`（Task 1）、`RelayCore::consumeForceZeroRequest`（Task 4）
- Produces: `static void requestForceZero(const char* src)`

- [ ] **Step 1: ★ 先理解这个顺序陷阱（写错了界面会显示一个假的数）**

`main.cpp` 的启动顺序是：

```
:3755  4.   initRelayReporting()      ← 连上 MATLAB, 并【在这里回读一次增益】
:3759  4.5  initForceReader()         ← 里面调 ForcePipeline::init(), 斜坡在这里就位
:3762  4.6  ForceCalibration::loadFromFile(...)
```

两处都要求增益**已经加载完毕**：

- 若在 `:3755` **之后**加载 ⇒ 连接时的回读报的是旧值（120），而实际生效的是文件里的 300；
  之后没有任何事件会再回读 ⇒ **界面显示 120、手上是 300，而且不会自己纠正**。
  这正是本设计声称"在结构上无法存在"的那个状态。
- 若在 `:3759` **之后**加载 ⇒ `ForcePipeline::init()` 已把斜坡就位在旧值，启动后头 0.25 秒
  增益是错的（不会失控，总夹还在，但没必要）。

⇒ **`loadOnStartup()` 必须是 `:3755` 之前的一条独立步骤。**

- [ ] **Step 2: 在 `main.cpp:3753-3755` 之间插入加载步骤**

把

```cpp
    }

    // 4. 连接 MATLAB GUI (localhost:8888)
    RelayCore::instance().initRelayReporting();
```

改成

```cpp
    }

    // 3.9 加载力反射增益 (calib/force_tuning.json; 没有就用 Config.h 的默认值)
    // ★★ 【必须在 initRelayReporting 之前】—— 那一步连上 MATLAB 后会立刻回读一次增益。
    //   若在它之后加载, 回读报的是旧值而实际生效的是文件里的值, 且之后【没有任何事件】
    //   会纠正它 ⇒ 界面显示 120、手上是 300, 无声地不一致。这正是本设计要消灭的状态。
    // ★ 也必须在 initForceReader 之前 —— 那里的 ForcePipeline::init() 会把增益斜坡就位;
    //   晚了那 0.25 秒里增益是错的。
    ForceTuning::loadOnStartup();

    // 4. 连接 MATLAB GUI (localhost:8888)
    RelayCore::instance().initRelayReporting();
```

在 `main.cpp` 的 include 区加 `#include "force/ForceTuning.h"`（照现有 `#include "force/..."` 的写法对齐）。

- [ ] **Step 3: 抽出 `requestForceZero`**

在 `main.cpp:2801` 的 `cancelOtherCaptureModes` **之后**、`:2823` 的 `idle()` **之前**插入：

```cpp
// 力传感器调零 —— 【全程序唯一一份守卫链】。键盘 'z' 与 MATLAB 的 Zero 按钮【都走这里】。
//
// 【为什么不把守卫链放到 RelayCore】g_noRobot 与 cancelOtherCaptureModes 都是本文件的
//   file-static, RelayCore 拿不到; 而在那里复制一份"标定中/调零中"的判断, 就是本项目最
//   忌讳的"同一个规则两份实现, 改一份忘一份"(成文教训见 force/ForcePipeline.h:21-25)。
//   ⇒ 让 RelayCore 只置一个标志, 由本函数在 GLUT 线程上统一处置。
//
// src: 标明是谁触发的, 写进日志 (两条入口的日志必须能分辨)。
static void requestForceZero(const char* src) {
    auto& relay = RelayCore::instance();
    char msg[192];

    if (relay.isForceZeroing()) {
        relay.abortForceCalibration();
        snprintf(msg, sizeof(msg), "[Force] 调零【已中止】 (来自 %s)", src);
        std::cout << msg << std::endl;
        relay.reportCommand(msg);
        return;
    }
    if (relay.isForceCalibrating()) {
        snprintf(msg, sizeof(msg),
                 "[Force] 力标定进行中 — 等它结束, 或按 'k' 中止 (来自 %s)", src);
        std::cout << msg << std::endl;
        relay.reportCommand(msg);
        return;
    }
    if (g_noRobot) {
        snprintf(msg, sizeof(msg), "[Force] --no-robot 模式下无法调零 (来自 %s)", src);
        std::cout << msg << std::endl;
        relay.reportCommand(msg);
        return;
    }

    cancelOtherCaptureModes('z');
    if (relay.startForceZeroing()) {
        snprintf(msg, sizeof(msg),
                 "[Force] 调零中: 保持机械臂静止, 采集完成后自动应用并存盘 (来自 %s)", src);
        std::cout << msg << std::endl;
        relay.reportCommand(msg);
    }
}
```

⚠ **`reportCommand` 不是装饰**：操作员在 MATLAB 前面**看不到 stdout**。不回显就等于
按了按钮什么都反馈不了。它走 `C|` → `relay_gui.m` 的命令日志面板。

⚠ 两段文本**都不许含逗号** —— `C|` 本身是 `C|<文本>` 所以逗号在这里没问题，但保持与
`reportWarning` 同样的自律（`:114-124` 那段说明了逗号会怎么错位）。

- [ ] **Step 4: `'z'` 分支改调 `requestForceZero`**

把 `main.cpp:3105-3128` 整段替换成：

```cpp
    // ===== 力传感器调零 ('z' key) =====
    // 'z': 静置采集零偏 → 直接应用+存盘。不进 MOTION 相、不开拖拽模式。再按一次中止。
    //      换装工具 (笔夹/笔) 后重新调零用这个。
    // ★ 2026-09-22: 守卫链抽进 requestForceZero() —— MATLAB 的 Zero 按钮走【同一个函数】。
    if (key == 'z' || key == 'Z') {
        requestForceZero("键盘");
        return;
    }
```

- [ ] **Step 5: `idle()` 里消费 MATLAB 的调零请求**

把 `main.cpp:2827-2828`

```cpp
        // MATLAB → C++ 反向命令轮询 (力反馈开关等)
        RelayCore::instance().pollRelayCommands();
```

改成

```cpp
        // MATLAB → C++ 反向命令轮询 (力反馈开关 / 反射增益 / 调零请求)
        RelayCore::instance().pollRelayCommands();

        // MATLAB 的 Zero 按钮: 在这里 (GLUT 线程) 消费, 走与键盘 'z' 完全同一个函数。
        // 为什么不在 pollRelayCommands 里直接做: g_noRobot / cancelOtherCaptureModes 是
        // 本文件的 file-static (见 requestForceZero 顶上的说明)。
        if (RelayCore::instance().consumeForceZeroRequest()) {
            requestForceZero("MATLAB");
        }
```

- [ ] **Step 6: 编译**

Run:
```bash
cd Touch_Client && "D:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvarsall.bat" x64 && msbuild Touch_Client.vcxproj /p:Configuration=Release /p:Platform=x64 /v:m
```
Expected: 0 error。

⚠ 若报 `requestForceZero` 未声明：它必须定义在 `idle()` **之前**（`:2823`）。若你没找到
`cancelOtherCaptureModes` 的位置，用 `grep -n "static void cancelOtherCaptureModes" main.cpp`。

- [ ] **Step 7: 跑全套测试**

Run: `Touch_Client\tests\run_tests.bat`
Expected: `Summary: 12 of 20 suites run - 12 OK, 0 FAILED`

- [ ] **Step 8: 提交**

```bash
git add Touch_Client/main.cpp
git commit -m "feat(tuning): main 接线 —— 增益加载顺序 + 调零入口合一

★ loadOnStartup() 必须在 initRelayReporting() 【之前】:
  那一步连上 MATLAB 后会立刻回读一次增益; 晚于它 ⇒ 回读报旧的 120 而实际生效是
  文件里的值, 而且之后没有任何事件会纠正 ⇒ 界面与手上无声地不一致。
  也必须在 initForceReader() 之前 (那里的 ForcePipeline::init() 把斜坡就位)。

requestForceZero() 抽出守卫链: 键盘 'z' 与 MATLAB 的 Zero 按钮共用同一个函数。
不把守卫链放进 RelayCore —— g_noRobot / cancelOtherCaptureModes 是 main.cpp 的
file-static, 复制一份就是本项目最忌讳的两份实现。

处置文字走 reportCommand (C|) 回显到 MATLAB 命令日志 —— 操作员看不到 stdout。"
```

---

### Task 6: `relay_gui.m` —— 滑条 + 数值框 + Default + Zero

**Files:**
- Modify: `Relay_Station/relay_gui.m:59-66`（S 初始化）
- Modify: `Relay_Station/relay_gui.m:176-205`（`pnlFF` 内部 grid 从 `[3 1]` 扩成 `[4 1]`）
- Modify: `Relay_Station/relay_gui.m:455-530`（`processNetworkData` 加 `RG|` 分支）
- Modify: `Relay_Station/relay_gui.m:668+`（`updateTextPanels` 更新标题行）
- Modify: `Relay_Station/relay_gui.m:422-439`（回调区加 4 个新回调）

**Interfaces:**
- Consumes: `sendToClient(cmd)`（`:431`）、`RG|` 与 `Z|` 协议（Task 3/4）
- Produces: 无（终端）

- [ ] **Step 1: S 初始化（`:60` 的 `S.ff_enabled` 旁边）加**

```matlab
    % ===== 力反射增益调参 (2026-09-22) =====
    % 真值【只在 C++ 那一侧】。下面这些字段只是【显示缓存】, 全部由 RG| 回读刷新 ——
    % 绝不当作"我设过什么"的记忆使用 (那正是无声不一致的入口)。
    S.tuning = struct('gain', NaN, 'min', NaN, 'max', NaN, 'ratio', NaN, ...
                      'deadN', NaN, 'satN', NaN, 'defGain', NaN, 'known', false);
    S.tuningDragging = false;   % 拖动中禁止回读移动滑块 (否则和手指打架)
    S.tuningLastSent = NaN;     % 最近一次发出去的值, 用于判断回读是否与请求不符 (拒收)
```

- [ ] **Step 2: `pnlFF` 扩一格并插入控件**

`relay_gui.m:176-180` 现在是：

```matlab
    pnlFF = uigridlayout(glMid, [3 1]);
    pnlFF.RowHeight = {22, 26, '1x'};
```

改成：

```matlab
    pnlFF = uigridlayout(glMid, [4 1]);
    pnlFF.RowHeight = {22, 26, 40, '1x'};
```

`:201-205` 的 `lblForceFilt` 的 `Layout.Row` 从 `3` 改成 `4`。

在 `:205` 之后（`lblForceFilt` 那几行之后、`% -- Row 4: 力历史迷你图 --` 之前）插入：

```matlab
    % -- 力反射增益调参 (2026-09-22) --
    % 标题行显示【响应窗口】而不是单个饱和点 —— 只给上沿会把死区那个前提藏起来:
    %   gain 300 时窗口是 0.20–0.67N, 死区占了 30%, 可区分的只剩一条缝。
    % 窗口的两个数与比例全部来自 C++ 的 RG| 回读, 本文件【一个魔数都不写】。
    pnlGain = uigridlayout(pnlFF, [2 1]);
    pnlGain.RowHeight = {15, 25};
    pnlGain.Padding = [0 0 0 0];  pnlGain.RowSpacing = 0;
    pnlGain.BackgroundColor = clr.bg_panel;
    pnlGain.Layout.Row = 3;  pnlGain.Layout.Column = 1;

    lblGainTitle = uilabel(pnlGain, 'Text', 'Reflection Gain — 等待 C++…', ...
        'FontColor', clr.text_dim, 'FontSize', 9, 'FontName', 'Consolas');
    lblGainTitle.Layout.Row = 1;  lblGainTitle.Layout.Column = 1;

    pnlGainCtl = uigridlayout(pnlGain, [1 4]);
    pnlGainCtl.ColumnWidth = {'1x', 58, 58, 44};
    pnlGainCtl.Padding = [0 0 0 0];  pnlGainCtl.RowSpacing = 0;  pnlGainCtl.ColumnSpacing = 3;
    pnlGainCtl.BackgroundColor = clr.bg_panel;
    pnlGainCtl.Layout.Row = 2;  pnlGainCtl.Layout.Column = 1;

    % 上下限先随便给一个占位值, 但控件【禁用】—— 收到第一次 RG| 回读才用 C++ 给的
    % 真实范围打开。不猜上下限。
    sldGain = uislider(pnlGainCtl, 'Limits', [100 300], 'Value', 120, ...
        'MajorTicks', [100 200 300], 'MajorTickLabels', {}, 'Enable', 'off', ...
        'ValueChangingFcn', @(s,e) onGainChanging(e.Value), ...
        'ValueChangedFcn',  @(s,e) onGainChanged(e.Value));
    sldGain.Layout.Row = 1;  sldGain.Layout.Column = 1;

    edGain = uieditfield(pnlGainCtl, 'numeric', 'Value', 120, ...
        'Limits', [100 300], 'Enable', 'off', ...
        'FontName', 'Consolas', 'FontSize', 10, ...
        'ValueChangedFcn', @(s,e) onGainChanged(e.Value));
    edGain.Layout.Row = 1;  edGain.Layout.Column = 2;

    btnGainDefault = uibutton(pnlGainCtl, 'Text', 'Default', 'FontSize', 9, ...
        'Enable', 'off', 'ButtonPushedFcn', @(~,~) onGainDefault());
    btnGainDefault.Layout.Row = 1;  btnGainDefault.Layout.Column = 3;

    % Zero 与键盘 'z' 【同语义】: 再按一次 = 中止。不在 GUI 里发明第二种语义。
    btnZero = uibutton(pnlGainCtl, 'Text', 'Zero', 'FontSize', 9, ...
        'ButtonPushedFcn', @(~,~) onZeroPressed());
    btnZero.Layout.Row = 1;  btnZero.Layout.Column = 4;
```

- [ ] **Step 3: 在回调区（`:429` 的 `onForceFeedbackToggle` 之后）加 4 个回调**

```matlab
    % ===== 力反射增益 (2026-09-22) =====
    % 真值在 C++。这里只做两件事: 把用户意图发过去、把回读显示出来。

    function onGainChanging(v)
        % 拖动中: 实时下发 —— 拖的过程手上就能感觉到 (C++ 那道 0.25s 斜坡把它摊平)
        S.tuningDragging = true;
        S.tuningLastSent = v;
        sendToClient(sprintf('RG|%.4f', v));
    end

    function onGainChanged(v)
        % 松手 / 编辑框提交: 再发一次。幂等, 保证最后一条一定到
        % (拖动中被 C++ 限频挡下的那条, 由它的补发机制兜住)。
        S.tuningDragging = false;
        S.tuningLastSent = v;
        sendToClient(sprintf('RG|%.4f', v));
    end

    function onGainDefault()
        % ★ 【不硬编码 120】—— 用 C++ 回读里的 defGain。
        %   硬编码的话, Config.h 的默认值一改, 这个按钮就与"默认"无关了 ——
        %   而它做的正是"送回默认值"这件事。
        if ~S.tuning.known, return; end
        S.tuningDragging = false;
        S.tuningLastSent = S.tuning.defGain;
        sendToClient(sprintf('RG|%.4f', S.tuning.defGain));
    end

    function onZeroPressed()
        % 与键盘 'z' 同语义 (C++ 那边负责"再按一次 = 中止")
        sendToClient('Z|1');
    end
```

- [ ] **Step 4: `processNetworkData` 加 `RG|` 分支**

在 `:509` 的 `B|` 分支**之后**插入（放在同一串 `elseif` 里）：

```matlab
                elseif startsWith(msg, 'RG|')
                    vals = str2double(split(msg(4:end), ','));
                    if numel(vals) >= 7 && all(~isnan(vals))
                        wasKnown = S.tuning.known;
                        S.tuning.gain    = vals(1);
                        S.tuning.min     = vals(2);
                        S.tuning.max     = vals(3);
                        S.tuning.ratio   = vals(4);
                        S.tuning.deadN   = vals(5);
                        S.tuning.satN    = vals(6);
                        S.tuning.defGain = vals(7);
                        S.tuning.known   = true;

                        if ~wasKnown
                            % 第一次回读: 用 C++ 给的上下限把控件打开 —— 不猜
                            sldGain.Limits = [S.tuning.min S.tuning.max];
                            edGain.Limits  = [S.tuning.min S.tuning.max];
                            sldGain.MajorTicks = ...
                                linspace(S.tuning.min, S.tuning.max, 5);
                            sldGain.MajorTickLabels = {};
                            sldGain.Enable = 'on';
                            edGain.Enable  = 'on';
                            btnGainDefault.Enable = 'on';
                            fprintf('[Relay] 增益控件已启用: 范围 [%.0f, %.0f], 当前 %.1f\n', ...
                                S.tuning.min, S.tuning.max, S.tuning.gain);
                        end

                        % 拒收: 回读与我们刚发的不符 ⇒ 说清原因。
                        % 数字全部来自这条回读, 本文件不写 100/300。
                        if ~isnan(S.tuningLastSent) && ...
                           abs(S.tuning.gain - S.tuningLastSent) > 1e-6
                            fprintf(['[Relay] 增益 %.1f 被拒 —— 可取范围 ' ...
                                     '[%.0f, %.0f], 仍是 %.1f\n'], ...
                                S.tuningLastSent, S.tuning.min, ...
                                S.tuning.max, S.tuning.gain);
                        end

                        if ~S.tuningDragging
                            % 拖动中不动滑块 —— 否则回读会和手指打架
                            sldGain.Value = S.tuning.gain;
                            edGain.Value  = S.tuning.gain;
                        end
                    end
```

- [ ] **Step 5: `updateTextPanels` 里刷新标题行**

在 `updateTextPanels()`（`:668`）内部找一个合适的末尾位置加：

```matlab
        % 力反射增益: 把 C++ 回读的三个展示量写成一行 (比例 + 响应窗口)
        if S.tuning.known
            lblGainTitle.Text = sprintf( ...
                'Reflection Gain  ≈%.2f:1   响应窗口 %.2f – %.2f N (该轴分量)', ...
                S.tuning.ratio, S.tuning.deadN, S.tuning.satN);
        end
```

- [ ] **Step 6: 语法检查（不启动整个系统就能做的验证）**

Run:
```bash
cd Relay_Station && matlab -batch "checkcode('relay_gui.m')"
```
Expected: 没有 error 级别的问题。

⚠ 若 MATLAB 不在 PATH 上，就在 MATLAB 里手工跑 `checkcode relay_gui.m`，或者直接
`relay_gui` 看它能不能起窗口（**不需要**机械臂 / C++ 客户端 —— `S.server` 没连上时
滑条会保持禁用并显示"等待 C++…"，那正是设计的行为）。

- [ ] **Step 7: 提交**

```bash
git add Relay_Station/relay_gui.m
git commit -m "feat(tuning): MATLAB 界面加力反射增益滑条 + Default + Zero

滑条/数值框联动, 拖动实时下发 (ValueChangingFcn), 松手再发一次 (幂等)。
收到第一次 RG| 回读才用 C++ 给的上下限打开控件 —— 不猜范围。
Default 按钮用回读里的 defGain, 【不硬编码 120】。
Zero 按钮发 Z|1, 与键盘 'z' 同语义 (再按一次 = 中止)。

标题行显示【响应窗口】而非单个饱和点: gain 300 时死区占 30%,
只给上沿会把那个前提藏起来。

S.tuningDragging 挡住拖动中的回写 (否则回读和手指打架);
S.tuningLastSent 用于识别拒收并把原因打出来。"
```

---

### Task 7: 把新套件接进测试床（含两个手工维护的数字）

**Files:**
- Modify: `Touch_Client/tests/run_tests.bat`（加一段 + 改两个数字）

**Interfaces:**
- Consumes: `build_force_tuning_test.bat`（Task 1）
- Produces: 无

- [ ] **Step 1: 先记下基线**

Run: `Touch_Client\tests\run_tests.bat`
Expected: `Summary: 12 of 20 suites run - 12 OK, 0 FAILED`

- [ ] **Step 2: 加一个"先建再跑"的段**

在 `run_tests.bat` 里**最后一段**（`:310` 那个 `test_noise_probe` 段）之后、
`:330` 的 `echo ==== Tests complete ====` **之前**插入：

```bat
echo ================================================
echo   Force Tuning Tests
echo ================================================
echo.
echo --- Building test_force_tuning ---
call "%TESTDIR%\build_force_tuning_test.bat"
@echo off
if %ERRORLEVEL% EQU 0 (
    echo   Build OK
    echo.
    echo === test_force_tuning.exe ===
    "%TESTDIR%\test_force_tuning.exe"
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

⚠ 三处**不能改**：`@echo off`（每个 `call` 之后都要有，理由在 `:29-34`）、
`if %ERRORLEVEL% EQU 0`（外层 build 检查，顶层语句，正确）、
`if !ERRORLEVEL! EQU 0`（内层结果检查，必须延迟展开，理由在 `:14-27`）。

- [ ] **Step 3: ★ 改那三个【手工维护】的数字（漏了就是"看着做完了、其实没验证"）**

`run_tests.bat:346-347` 的注释自己写着：
"The "20" and the "8" below are HAND-MAINTAINED -- nothing computes them."
**先数，再填** —— 不要照抄下面的期望值：

```bash
ls Touch_Client/tests/test_*.cpp | wc -l
grep -c 'call "%TESTDIR%.build_' Touch_Client/tests/run_tests.bat
grep -c 'set /a PASSED+=1'      Touch_Client/tests/run_tests.bat
```

⚠ **别用 `grep -c 'build_.*\.bat'`** —— 它多匹配到 `[NOT RUN]` 那一段文本里提到的
"build script"，给出 **13** 这个错数（本计划第一版就写成了这条错命令）。
用上面两条；它们写计划时实测都是 **12**，且互为独立信号。

改之前（实测）：**20** 个测试文件 / **12** 个接入段 / 未跑 **8** 个。
本任务加一个 ⇒ **21** / **13** / 未跑仍是 **8**。

**要改的是【五处】—— 用 `grep -n 'NEQ 12\|(12)\|of 20\|8 of the 20\|the 12 suites' run_tests.bat`
自己核一遍，别只改前三个**（本计划第一版只列了三个，漏掉的是两处**文本自述**）：

| 行 | 现在 | 改成 | 作用 |
|---|---|---|---|
| `:338` | `Summary: %TOTAL% of 20 suites run` | `of 21 suites run` | 打进日志的总数 |
| `:351` | `[NOT RUN] 8 of the 20 test_*.cpp` | `[NOT RUN] 8 of the 21 test_*.cpp` | 打进日志的未跑清单 |
| `:370` | `"the 12 suites that ran all passed"` | `"the 13 suites that ran all passed"` | **打进日志的自述** |
| `:378` | `the number of build sections (12)` | `build sections (13)` | 源码注释 |
| `:385` | `if %TOTAL% NEQ 12 (` | `if %TOTAL% NEQ 13 (` | ← **唯一影响行为的那个** |

各处的作用别混：

- `:338` 的 `20` 是**仓库里测试文件的总数**（20 → 21）。
- `:351` 的**两个数各自独立**：未跑数 21 − 13 = **8 不变**，总数 20 → 21。
  ⇒ 是 `8 of the 21`。⚠ 最容易手滑写成"7"（以为多接一个就少一个没跑）——
  新接的这个**同时也把分母加了一**，所以未跑数不动。
- `:370` **是打进日志的**：不改它，一次 13 段全绿的日志会写着"the 12 suites that ran
  all passed" —— 这正是本项目反复出现的"日志自述静默过期"（与 `Summary` 的 20/8 同类，
  也正是当期待办里想根治的那条）。
- `:385` 是**唯一影响行为的**，作用见 `:377-379`：每段恰好计一次。
  **不改它，新段一接上就会被误报 MISMATCH。**

同时把 `:347` 那句注释里的计算式更新成 `... = 21; sections wired in below = 13; 21 - 13 = 8`。

- [ ] **Step 4: 跑，确认 13 段全绿且没有 MISMATCH**

Run: `Touch_Client\tests\run_tests.bat`
Expected:
```
  Summary: 13 of 21 suites run - 13 OK, 0 FAILED
...
  Suites counted: 13    Exit code: 0
```
且**没有** `MISMATCH` 那一行。

⚠ 若出现 `MISMATCH: expected 13 counted suites, but counted N` ⇒ 你的新段没被计到
（检查它是不是漏了 `set /a PASSED+=1` / `set /a FAILED+=1`，或者被放在了
`set /a TOTAL=PASSED+FAILED` 之后）。

- [ ] **Step 5: 反证 —— 确认新段真的会红（负对照）**

这一条是这个项目的规矩："先确认它真的被构建/执行过"。
临时把 `build_force_tuning_test.bat` 里的源文件名改成不存在的名字，重跑 `run_tests.bat`。

Expected: 新的那一段打 `[FAIL: build error]`，Summary 变 `12 OK, 1 FAILED`，退出码非 0。
**看到这个才算新段真的接上了。** 然后**改回来**并重跑确认全绿。

- [ ] **Step 6: 提交**

```bash
git add Touch_Client/tests/run_tests.bat
git commit -m "test: 把 test_force_tuning 接进测试床 (12 段 → 13 段)

三个手工维护的数字一并更新 (其注释自己写明了'没有东西在算它们'):
  Summary 的 20 → 21, [NOT RUN] 的 8 of 20 → 8 of 21, MISMATCH 检查的 12 → 13。
漏掉最后一个会让新段被误报成 MISMATCH; 漏掉前两个会让日志静默地说少了一个套件。

负对照做过: 临时把构建脚本的源文件名改错 ⇒ 新段确实报 [FAIL: build error]
且退出码非 0, 然后改回。"
```

---

## 收尾

全部 Task 完成后：

- [ ] 跑 `Touch_Client\tests\run_tests.bat`：`Summary: 13 of 21 suites run - 13 OK, 0 FAILED`，退出码 0
- [ ] 编译 Release 通过
- [ ] **上机项（不在本计划内，需要机械臂）**：
      `RG|` 回读在真实 MATLAB + C++ 链路下走通 · 拖动滑条手上力确实变化 · Zero 按钮真的调零 ·
      被拒提示（在数值框里填 500）
- [ ] 更新记忆：把"增益可调"这件事与它的三个陷阱（`G|` 已占用 · 加载顺序 ·
      `run_tests.bat` 的手工数字）记进 `MEMORY.md`
