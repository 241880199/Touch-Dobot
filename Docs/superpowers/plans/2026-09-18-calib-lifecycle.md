# 标定文件生命周期与 CZ 符号自动判定 — 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让标定文件固定落点**并由实测（而非时间）判定其是否仍然可信**，同时让 CZ 符号约定完全由程序算出、不引入人工判断。

**Architecture:** `core/CalibStore` 只负责**位置**（从可执行文件推导 `calib\` 目录）；标定是否仍然可信改由**启动自检**——启动时引导摆 2~3 个姿态，用跨姿态极差实测判定，不通过则作废并要求重标；`PayloadCalibration` 用**外部锚点**（STL 估计的种子质心）加**余量判据**来确定符号；`BiasCheck` 用一个数据新鲜度标志替代粗暴的 `reset()`。

**Tech Stack:** C++17 / MSVC 2022 BuildTools / Win32 API (`GetModuleFileNameA`、`CreateDirectoryA`) / 无第三方库。

**规格:** `Docs/superpowers/specs/2026-09-18-calib-lifecycle-design.md`（**务必读 §7 的两轮修订**）

> ⚠ **本计划经过两轮设计修订，前面任务的文本保留原样只为记录演进。**
> **以最终态为准，不要照抄早期步骤。**
>
> | 早期文本 | 被谁取代 | 最终态 |
> |---|---|---|
> | Task 1/2 的 24h 有效期（`saved_at_unix` / `.expired` / `isFresh` / `resolve`） | **Task 8** 整体移除 | `CalibStore` 只管位置；负载不漂、**不做启动检查**；零偏会漂，由 Task 9 便宜地查 |
> | Task 3/4 交付的 `'i'` 人工兜底 | **Task 5** 移除 | 符号不引入人工判断 |
> | Task 3/5 的"物理质心为正"判据 | **Task 7** 替换 | 旧判据**永远选不出相反符号**（代数上恒等于 `sign(comCfg[2])`） |
> | Task 7 的"种子锚点 + 余量判据" | **Task 10** 替换 | **两次下发实测**：解算器给出两个候选，实机各下发一次、谁残余力矩小谁对。不依赖任何估计值 |
>
> **任务顺序：1 → 2 → 3 → 4 → 5 → 7 → 10 → 11 → 8 → 9 → 6。**
> （10 取代 7 的选择逻辑但复用它的候选计算；8/9/10 都动 `main.cpp`，必须串行；6 是文档，最后做。）
>
> 执行任一任务时，若其文本与本表冲突，**以本表和对应 Task 为准**。

## Global Constraints

- **文件保持 ASCII 或既有中文注释风格**；`.bat` 文件的额外约束见 `Touch_Client/build_and_run.bat` 顶部注释（`chcp 65001` + 非 ASCII 会 desync cmd 解析器）。
- **所有新文件用 LF 还是 CRLF 不强制**，但 `.bat` 必须 CRLF。
- **编译命令固定为：** `cmd.exe //c "D:\Projects\Touch\Touch_Client\build.bat"`（bash 下双斜杠），成功标志是输出 `Build OK.`。
- **完整构建是硬性要求**，不能只跑独立测试脚本：测试脚本不定义 `WIN32_LEAN_AND_MEAN`，真实项目定义 —— 两者编译环境不同，单测通过**不等于**项目能编。这一点已经咬过一次。
- **测试必须真的跑起来**：测试构建脚本只编译，不运行 exe。每步都要单独执行 `tests\<name>.exe`。
- **编译前确认 `Touch_Client.exe` 没有在运行**，否则链接报 `LNK1168`。检查：`tasklist //FI "IMAGENAME eq Touch_Client.exe"`。
- **不要写以反斜杠结尾的 `//` 注释** —— 它会延续到下一行，静默吃掉那一行。已经咬过一次。
- **`Config::ROBOT_PAYLOAD_SEED_CZ_MM` 是符号判定的外部锚点**（Task 7 起），不只是首次兜底值 —— 它的准确性开始承担判定责任。
- **不引入人工判断**：需要判断的地方一律由程序算（用户明确要求）。

---

### Task 1: CalibStore —— 标定目录解析与有效期判定

**Files:**
- Create: `Touch_Client/core/CalibStore.h`
- Create: `Touch_Client/core/CalibStore.cpp`
- Create: `Touch_Client/tests/test_calib_store.cpp`
- Create: `Touch_Client/tests/build_calib_store_test.bat`
- Modify: `Touch_Client/config/Config.h`（加常量）

**Interfaces:**
- Consumes: `Config::CALIB_MAX_AGE_SEC`
- Produces:
  - `bool CalibStore::isFresh(long savedAtUnix, long nowUnix, long maxAgeSec)`
  - `bool CalibStore::deriveDir(const char* exePath, char* out, size_t outSize)`
  - `const char* CalibStore::dir()`
  - `const char* CalibStore::fileFor(const char* name)`
  - `const char* CalibStore::resolve(const char* name)`

- [ ] **Step 1: 加配置常量**

在 `Touch_Client/config/Config.h` 的「力传感器标定参数」段末尾（`FORCE_CALIB_SAMPLE_TIME_S` 那行之后）加：

```cpp
    // ========== 标定文件有效期 ==========
    // 力传感器零偏随温度/时间漂移; 负载解算又依赖力数据。
    // 超过这个时长的标定一律作废, 启动时要求重新标定。
    const long CALIB_MAX_AGE_SEC = 24 * 3600;   // 24 小时
```

- [ ] **Step 2: 写失败的测试**

创建 `Touch_Client/tests/test_calib_store.cpp`：

```cpp
// CalibStore 单测: 路径推导 + 有效期判定。
// 不碰真实文件系统 —— 这两个都是纯函数。
#include "../core/CalibStore.h"
#include "../config/Config.h"
#include <cstdio>
#include <cstring>
#include <cmath>

static int g_passed = 0, g_failed = 0;

#define TEST(name) do { std::printf("  " #name "... "); } while(0)
#define CHECK(cond) do { if (!(cond)) { std::printf("FAIL: %s\n", #cond); g_failed++; return; } } while(0)
#define PASS() do { std::printf("PASS\n"); g_passed++; } while(0)

// 正常布局: exe 在 <repo>\Touch_Client\x64\Release\ 下, 上溯两级到 Touch_Client 目录
static void test_derive_dir_normal() {
    TEST(derive_dir_normal);
    char out[512];
    CHECK(CalibStore::deriveDir(
        "D:\\Projects\\Touch\\Touch_Client\\x64\\Release\\Touch_Client.exe", out, sizeof(out)));
    CHECK(strcmp(out, "D:\\Projects\\Touch\\Touch_Client\\calib\\") == 0);
    PASS();
}

// 只有一级: 不能再往上溯 (上溯不足时保持原样并仍以反斜杠结尾)
static void test_derive_dir_shallow() {
    TEST(derive_dir_shallow);
    char out[512];
    CHECK(CalibStore::deriveDir("C:\\a\\b.exe", out, sizeof(out)));
    CHECK(strcmp(out, "C:\\a\\calib\\") == 0);
    PASS();
}

// 反斜杠数量不足 -> 返回 false, 不越界
static void test_derive_dir_bad_input() {
    TEST(derive_dir_bad_input);
    char out[512];
    CHECK(!CalibStore::deriveDir("no_separators.exe", out, sizeof(out)));
    CHECK(!CalibStore::deriveDir(nullptr, out, sizeof(out)));
    PASS();
}

static void test_is_fresh_boundaries() {
    TEST(is_fresh_boundaries);
    const long DAY = 86400;
    CHECK( CalibStore::isFresh(1000, 1000 + DAY,      DAY));  // 刚好 24h -> 仍有效
    CHECK(!CalibStore::isFresh(1000, 1000 + DAY + 1,  DAY));  // 超 1s -> 过期
    CHECK( CalibStore::isFresh(1000, 1005,            DAY));
    CHECK(!CalibStore::isFresh(0,    1000,            DAY));  // 缺字段 -> 过期
    CHECK(!CalibStore::isFresh(-5,   1000,            DAY));  // 负值 -> 过期
    CHECK( CalibStore::isFresh(2000, 1000,            DAY));  // 时钟回拨 -> 当作刚保存
    PASS();
}

int main() {
    std::printf("=== CalibStore Tests ===\n");
    test_derive_dir_normal();
    test_derive_dir_shallow();
    test_derive_dir_bad_input();
    test_is_fresh_boundaries();
    std::printf("\n%d passed, %d failed\n", g_passed, g_failed);
    return g_failed ? 1 : 0;
}
```

- [ ] **Step 3: 写测试构建脚本**

创建 `Touch_Client/tests/build_calib_store_test.bat`（CRLF 行尾）：

```bat
@echo on
call "D:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvarsall.bat" x64
cd /d "D:\Projects\Touch\Touch_Client\tests"
cl /EHsc /std:c++17 /D_CRT_SECURE_NO_WARNINGS test_calib_store.cpp ..\core\CalibStore.cpp /Fe:test_calib_store.exe
echo BUILD_EXIT=%ERRORLEVEL%
```

- [ ] **Step 4: 跑测试确认失败**

Run: `cmd.exe /c "D:\Projects\Touch\Touch_Client\tests\build_calib_store_test.bat"`
Expected: 编译失败，报 `无法打开包括文件: "CalibStore.h"`（模块还没写）。

- [ ] **Step 5: 写头文件**

创建 `Touch_Client/core/CalibStore.h`：

```cpp
#pragma once
#include <cstddef>

// 标定文件的生命周期: 放哪、还能不能用。
//
// 为什么需要它: 三个标定文件原本都用相对路径 ("./payload_calib.json") 读写,
// 落在【当前工作目录】—— 从 Touch_Client\ 启动和从 x64\Release\ 启动会拿到
// 两份不同的文件。而且标定结果永不过期, 一份几天前的数据会被当成权威值。
// 路径解析和有效期判定放在同一个模块, 因为它们回答的是同一个问题:
// "该不该用这份标定"。
namespace CalibStore {

    // 纯函数: 由可执行文件全路径推出标定目录 (结尾带反斜杠)。
    // 规则: 去掉文件名, 再【最多】上溯两级 (停在盘符根), 拼上 "calib\"。
    //   ...\Touch_Client\x64\Release\Touch_Client.exe → ...\Touch_Client\calib\
    // 只有整个路径一个分隔符都没有时才返回 false (out 内容未定义)。
    // 注: "最多两级" 是评审时明确的契约 —— 原计划写的是"不足两级即 false",
    //     与 test_derive_dir_shallow 用例矛盾; 已裁决以测试为准。
    bool deriveDir(const char* exePath, char* out, size_t outSize);

    // 标定目录绝对路径, 结尾带反斜杠。不存在时创建。进程内缓存。
    const char* dir();

    // 拼接标定文件绝对路径。静态缓冲, 下次调用即失效。
    // 只拼路径, 不做任何新鲜度判定 —— 写入用这个。
    const char* fileFor(const char* name);

    // 解析一个标定文件是否可用。
    //   可用   → 返回绝对路径 (静态缓冲, 下次调用即失效)
    //   过期   → 改名为 <name>.expired, 打印醒目提示, 返回 nullptr
    //   不存在 → 返回 nullptr (静默; 首次启动没有标定文件是正常情况)
    // 判据见 isFresh: 没有 saved_at_unix 字段一律视为过期。
    const char* resolve(const char* name);

    // 纯函数, 便于单测。nowUnix 由调用方传入, 内部不读时钟。
    // savedAtUnix <= 0 (缺字段/解析失败) 判过期; 时钟回拨按 0 年龄处理。
    bool isFresh(long savedAtUnix, long nowUnix, long maxAgeSec);
}
```

- [ ] **Step 6: 写实现**

创建 `Touch_Client/core/CalibStore.cpp`：

```cpp
#include "CalibStore.h"
#include "../config/Config.h"
#include <cstdio>
#include <cstring>
#include <ctime>
#include <windows.h>

namespace CalibStore {

    bool deriveDir(const char* exePath, char* out, size_t outSize) {
        if (!exePath || !out || outSize == 0) return false;
        char buf[MAX_PATH];
        size_t n = strlen(exePath);
        if (n >= sizeof(buf)) return false;
        memcpy(buf, exePath, n + 1);

        // 去掉文件名
        char* p = strrchr(buf, '\\');
        if (!p) return false;              // 一个分隔符都没有: 推不出目录
        *p = '\0';

        // 再上溯两级。上溯停在盘符根 ("C:\a" -> 不再剥成 "C:"):
        // 盘符根没有可以承载 calib\ 的目录, 而且各种布局的标定文件会撞在一起。
        for (int up = 0; up < 2; up++) {
            char* q = strrchr(buf, '\\');
            if (!q || (q == buf + 2 && buf[1] == ':')) break;
            *q = '\0';
        }
        if (snprintf(out, outSize, "%s\\calib\\", buf) >= (int)outSize) return false;
        return true;
    }

    const char* dir() {
        static char s_dir[MAX_PATH] = {0};
        if (s_dir[0] != '\0') return s_dir;

        char exe[MAX_PATH] = {0};
        if (GetModuleFileNameA(NULL, exe, MAX_PATH) == 0 ||
            !deriveDir(exe, s_dir, sizeof(s_dir))) {
            // 理论不可达; 退回当前目录, 至少不崩
            snprintf(s_dir, sizeof(s_dir), ".\\calib\\");
        }
        CreateDirectoryA(s_dir, NULL);   // 已存在时返回 ERROR_ALREADY_EXISTS, 无害
        return s_dir;
    }

    const char* fileFor(const char* name) {
        static char s_buf[MAX_PATH];
        snprintf(s_buf, sizeof(s_buf), "%s%s", dir(), name);
        return s_buf;
    }

    bool isFresh(long savedAtUnix, long nowUnix, long maxAgeSec) {
        if (savedAtUnix <= 0) return false;      // 缺字段 / 解析失败 -> 过期
        long age = nowUnix - savedAtUnix;
        if (age < 0) age = 0;                    // 时钟回拨 -> 当作刚保存
        return age <= maxAgeSec;
    }

    // 极简 JSON 数值字段提取。只认本程序写出的格式 (顶层 "key": number),
    // 不引入第三方 JSON 库。
    static bool readLongField(const char* path, const char* key, long& out) {
        FILE* f = fopen(path, "r");
        if (!f) return false;
        char buf[4096];
        size_t n = fread(buf, 1, sizeof(buf) - 1, f);
        fclose(f);
        if (n == 0) return false;
        buf[n] = '\0';

        char pat[64];
        snprintf(pat, sizeof(pat), "\"%s\"", key);
        const char* p = strstr(buf, pat);
        if (!p) return false;
        p = strchr(p, ':');
        if (!p) return false;
        p++;
        while (*p == ' ' || *p == '\t') p++;
        char* end = nullptr;
        double v = strtod(p, &end);
        if (end == p) return false;
        out = (long)v;
        return true;
    }

    const char* resolve(const char* name) {
        const char* path = fileFor(name);

        FILE* probe = fopen(path, "r");
        if (!probe) return nullptr;     // 不存在: 静默
        fclose(probe);

        long savedAt = 0;
        const bool hasStamp = readLongField(path, "saved_at_unix", savedAt);
        const long now = (long)time(NULL);

        if (hasStamp && isFresh(savedAt, now, Config::CALIB_MAX_AGE_SEC)) {
            return path;
        }

        // 过期 —— 或没有时间戳 (明确决策: 一律作废)
        char newPath[MAX_PATH];
        snprintf(newPath, sizeof(newPath), "%s.expired", path);
        remove(newPath);                // 同名旧文件先清掉, 失败也无所谓
        const bool renamed = (rename(path, newPath) == 0);

        std::printf("\n");
        std::printf("[Calib] !! %s 已作废 — %s\n", name,
                    hasStamp ? "超过有效期" : "缺少 saved_at_unix 时间戳");
        if (hasStamp) {
            std::printf("[Calib] !!   保存于 %.1f 小时前 (上限 %.0f 小时)\n",
                        (double)(now - savedAt) / 3600.0,
                        (double)Config::CALIB_MAX_AGE_SEC / 3600.0);
        }
        if (renamed) std::printf("[Calib] !!   已改名为 %s.expired\n", name);
        else         std::printf("[Calib] !!   (改名失败, 文件保留原处)\n");
        return nullptr;
    }
}
```

- [ ] **Step 7: 跑测试确认通过**

Run: `cmd.exe /c "D:\Projects\Touch\Touch_Client\tests\build_calib_store_test.bat"`
然后: `cmd.exe /c "D:\Projects\Touch\Touch_Client\tests\test_calib_store.exe"`
Expected: `4 passed, 0 failed`

- [ ] **Step 8: 提交**

```bash
git add Touch_Client/core/CalibStore.h Touch_Client/core/CalibStore.cpp \
        Touch_Client/tests/test_calib_store.cpp Touch_Client/tests/build_calib_store_test.bat \
        Touch_Client/config/Config.h
git commit -m "feat(calib): add CalibStore for calibration file location and expiry"
```

> **关于测试覆盖面（评审后修正）:** 原计划把 `resolve()` 的覆盖推迟到 Task 2 Step 9 的实机启动验证。
> 评审指出这不行 —— 模块最承重的两条规则（改名 `.expired`、"无时间戳即作废"）就住在 `resolve()` 里，
> 而"计划授权的不测试"恰恰是这类策略以后被改坏而不自知的成因。**已裁决：现在就加自动化覆盖。**

### Task 1 补充：`resolve()` 的自动化覆盖（评审后追加）

**Files:**
- Modify: `Touch_Client/core/CalibStore.h`
- Modify: `Touch_Client/core/CalibStore.cpp`
- Modify: `Touch_Client/tests/test_calib_store.cpp`

**背景:** `resolve()` 的路径来自模块自己的 `dir()`（即测试 exe 的位置），所以没法指向临时目录。
加一个显式传目录的兄弟函数作为接缝，`resolve()` 退化成一行转发。

- [ ] **Step A1: 加可测接缝**

`CalibStore.h` 里，在 `const char* resolve(const char* name);` 之后加：

```cpp
    // 同 resolve, 但标定目录由调用方给出 —— 单测用临时目录走这个。
    // 生产代码用 resolve(name), 它等价于 resolveIn(dir(), name)。
    const char* resolveIn(const char* dirPath, const char* name);
```

`CalibStore.cpp` 里，把现有 `resolve()` 的函数体改名为 `resolveIn`，签名加 `dirPath` 参数，
第一行由 `const char* path = fileFor(name);` 改为：

```cpp
        static char s_path[MAX_PATH];
        snprintf(s_path, sizeof(s_path), "%s%s", dirPath, name);
        const char* path = s_path;
```

其余逻辑**逐字不动**（探针 `fopen`、`readLongField`、`isFresh` 判定、改名、打印提示）。

然后在它后面加转发：

```cpp
    const char* resolve(const char* name) { return resolveIn(dir(), name); }
```

- [ ] **Step A2: 写测试**

在 `Touch_Client/tests/test_calib_store.cpp` 加（并在文件顶部补 `#include <ctime>` 与 `#include <windows.h>`）：

```cpp
// 造一个样本标定文件; savedAt == 0 表示【不写】saved_at_unix 字段。
static void writeFixture(const char* dir, const char* name, long savedAt) {
    char path[512];
    snprintf(path, sizeof(path), "%s%s", dir, name);
    FILE* f = fopen(path, "w");
    if (!f) return;
    if (savedAt == 0) fprintf(f, "{ \"mass_kg\": 1.0 }\n");
    else fprintf(f, "{ \"version\": 2, \"saved_at_unix\": %ld, \"mass_kg\": 1.0 }\n", savedAt);
    fclose(f);
}

static bool fixtureExists(const char* dir, const char* name) {
    char path[512];
    snprintf(path, sizeof(path), "%s%s", dir, name);
    FILE* f = fopen(path, "r");
    if (f) { fclose(f); return true; }
    return false;
}

// resolve() 的两条承重策略: 无时间戳即作废、超期作废, 且都要改名 .expired。
// (注: 本用例会打印模块自己的 [Calib] !! 提示行 —— 那是被测的用户可见行为, 属预期。)
static void test_resolve_policy() {
    TEST(resolve_policy);
    const char* d = "test_calibstore_tmp\\";
    CreateDirectoryA("test_calibstore_tmp", NULL);
    const long now = (long)time(NULL);

    // 新鲜: 带时间戳且在有效期内 -> 可用, 不改名
    writeFixture(d, "fresh.json", now);
    CHECK(CalibStore::resolveIn(d, "fresh.json") != nullptr);
    CHECK(fixtureExists(d, "fresh.json"));

    // 缺 saved_at_unix -> 作废并改名
    writeFixture(d, "nostamp.json", 0);
    CHECK(CalibStore::resolveIn(d, "nostamp.json") == nullptr);
    CHECK(!fixtureExists(d, "nostamp.json"));
    CHECK(fixtureExists(d, "nostamp.json.expired"));

    // 超期 -> 作废并改名
    writeFixture(d, "stale.json", now - Config::CALIB_MAX_AGE_SEC - 60);
    CHECK(CalibStore::resolveIn(d, "stale.json") == nullptr);
    CHECK(!fixtureExists(d, "stale.json"));
    CHECK(fixtureExists(d, "stale.json.expired"));

    // 不存在 -> 静默返回 nullptr (无提示)
    CHECK(CalibStore::resolveIn(d, "missing.json") == nullptr);

    // 顺手钉住被 Config.h 规定的值 (reviewer Minor #4)
    CHECK(Config::CALIB_MAX_AGE_SEC == 86400);

    remove("test_calibstore_tmp\\fresh.json");
    remove("test_calibstore_tmp\\nostamp.json.expired");
    remove("test_calibstore_tmp\\stale.json.expired");
    remove("test_calibstore_tmp");
    PASS();
}
```

在 `main()` 里 `test_is_fresh_boundaries();` 之后加 `test_resolve_policy();`。

- [ ] **Step A3: 跑测试确认通过**

Run: `cmd.exe //c "D:\Projects\Touch\Touch_Client\tests\build_calib_store_test.bat"`
然后: `cmd.exe //c "D:\Projects\Touch\Touch_Client\tests\test_calib_store.exe"`
Expected: `5 passed, 0 failed`

- [ ] **Step A4: 回归确认**

`resolve()` 的行为对生产代码必须逐字不变（Task 2 要用）。确认 `CalibStore.h` 的
`resolve()` 声明还在、`dir()`/`fileFor()` 未改动。

- [ ] **Step A5: 提交**

```bash
git add Touch_Client/core/CalibStore.h Touch_Client/core/CalibStore.cpp Touch_Client/tests/test_calib_store.cpp
git commit -m "test(calib): cover resolve()'s void-and-rename policy"
```

---

### Task 2: 标定文件写入时间戳 + 读取端接入有效期

**Files:**
- Modify: `Touch_Client/force/PayloadCalibration.cpp`（`save` 加字段，`load` 加注释）
- Modify: `Touch_Client/force/ForceCalibration.cpp`（`saveToFile` 加字段）
- Modify: `Touch_Client/main.cpp:363,1035,1069,823,1095`
- Modify: `Touch_Client/Touch_Client.vcxproj`
- Modify: `Touch_Client/tests/test_payload_calibration.cpp`（加往返时间戳用例）

**Interfaces:**
- Consumes: `CalibStore::fileFor`、`CalibStore::resolve`（Task 1）
- Produces: `payload_calib.json` v2 与 `force_calib.json` v3 均含 `saved_at_unix`

- [ ] **Step 1: 写失败的测试**

在 `Touch_Client/tests/test_payload_calibration.cpp` 的 `test_save_load_roundtrip()` 之后加：

```cpp
// 落盘必须带 saved_at_unix —— 没有这个字段的文件会被 CalibStore 判为过期。
static void test_save_includes_timestamp() {
    TEST(save_includes_timestamp);
    const char* path = "test_payload_ts.json";
    PayloadCalibration::enabled = true;
    PayloadCalibration::massKg = 0.5;
    PayloadCalibration::comMm[0] = 0.0;
    PayloadCalibration::comMm[1] = 0.0;
    PayloadCalibration::comMm[2] = 80.0;
    CHECK(PayloadCalibration::save(path));

    FILE* f = fopen(path, "r");
    CHECK(f != nullptr);
    char buf[2048];
    size_t n = fread(buf, 1, sizeof(buf) - 1, f);
    fclose(f);
    buf[n] = '\0';
    // 字段必须存在, 且是 plausible 的 Unix 秒 (> 2026-01-01)
    CHECK(strstr(buf, "\"saved_at_unix\"") != nullptr);
    const char* p = strstr(buf, "\"saved_at_unix\"");
    const char* colon = strchr(p, ':');
    double ts = strtod(colon + 1, nullptr);
    CHECK(ts > 1767225600.0);
    CHECK(strstr(buf, "\"version\": 2") != nullptr);
    remove(path);
    PASS();
}
```

在 `main()` 里 `test_save_load_roundtrip();` 之后加 `test_save_includes_timestamp();`。
并确认文件顶部已 `#include <cstring>`（`strstr`）与 `#include <cstdio>`（`fopen`）。

- [ ] **Step 2: 跑测试确认失败**

Run: `cmd.exe /c "D:\Projects\Touch\Touch_Client\tests\build_payload_calibration_test.bat"`
然后: `cmd.exe /c "D:\Projects\Touch\Touch_Client\tests\test_payload_calibration.exe"`
Expected: `save_includes_timestamp... FAIL: strstr(buf, "\"saved_at_unix\"") != nullptr`

- [ ] **Step 3: PayloadCalibration::save 写时间戳**

在 `Touch_Client/force/PayloadCalibration.cpp` 的 `save()` 里，把 version 行改掉并加字段：

```cpp
        fprintf(f, "{\n");
        fprintf(f, "  \"version\": 2,\n");
        fprintf(f, "  \"saved_at_unix\": %ld,\n", (long)time(NULL));
        fprintf(f, "  \"mass_kg\": %.6g,\n", massKg);
```

并在文件顶部加 `#include <ctime>`。

- [ ] **Step 4: ForceCalibration::saveToFile 写时间戳**

在 `Touch_Client/force/ForceCalibration.cpp` 里，把整个 `saveToFile()` 替换为：

```cpp
bool saveToFile(const char* path, double massKg,
                const double biasForce[3], const double biasTorque[3])
{
    FILE* f = fopen(path, "w");
    if (!f) return false;
    fprintf(f, "{\n");
    fprintf(f, "  \"version\": 3,\n");
    // CalibStore 按这个字段判有效期; 缺了它整份标定会被判过期。
    fprintf(f, "  \"saved_at_unix\": %ld,\n", (long)time(NULL));
    fprintf(f, "  \"mass_kg\": %.6g,\n", massKg);
    fprintf(f, "  \"bias_force_n\": [%.6g, %.6g, %.6g],\n",
            biasForce[0], biasForce[1], biasForce[2]);
    fprintf(f, "  \"bias_torque_nm\": [%.6g, %.6g, %.6g]\n",
            biasTorque[0], biasTorque[1], biasTorque[2]);
    fprintf(f, "}\n");
    fclose(f);
    return true;
}
```

并在 `ForceCalibration.cpp` 顶部 include 区加 `#include <ctime>`。

- [ ] **Step 5: 跑测试确认通过**

Run: `cmd.exe /c "D:\Projects\Touch\Touch_Client\tests\build_payload_calibration_test.bat"`
然后: `cmd.exe /c "D:\Projects\Touch\Touch_Client\tests\test_payload_calibration.exe"`
Expected: `11 passed, 0 failed`

- [ ] **Step 6: 加入 vcxproj**

在 `Touch_Client/Touch_Client.vcxproj` 的 `core\AppState.cpp` 那一行之后加：

```xml
    <ClInclude Include="core\CalibStore.h" />
    <ClCompile Include="core\CalibStore.cpp" />
```

- [ ] **Step 7: main.cpp 的加载点改用 CalibStore::resolve**

`Touch_Client/main.cpp:1035` 附近，把 payload 加载块改成：

```cpp
        const char* payloadPath = CalibStore::resolve("payload_calib.json");
        if (payloadPath && PayloadCalibration::load(payloadPath)) {
            std::cout << "[Payload] Loaded payload_calib.json (mass=" << PayloadCalibration::massKg
                      << "kg, com=(" << PayloadCalibration::comMm[0] << ","
                      << PayloadCalibration::comMm[1] << "," << PayloadCalibration::comMm[2]
                      << ")mm, " << PayloadCalibration::poses << " poses, sign_z="
                      << (PayloadCalibration::comSignZ > 0 ? "+1" : "-1") << ")" << std::endl;
        } else {
            double m, c[3];
            PayloadCalibration::effective(m, c);
            std::cout << "[Payload] 无可用 payload_calib.json — 用种子值 mass=" << m
                      << "kg com=(" << c[0] << "," << c[1] << "," << c[2] << ")mm\n"
                      << "          实机标定: 启动后按 'm' 采多姿态 → 's' 求解" << std::endl;
        }
```

`Touch_Client/main.cpp:1069` 附近，force 加载块改成：

```cpp
        double massKg, biasF[3], biasM[3];
        const char* forcePath = CalibStore::resolve("force_calib.json");
        if (forcePath && ForceCalibration::loadFromFile(forcePath, massKg, biasF, biasM)) {
            double comZero[3] = {0};
            ForceCompensation::setCalibration(massKg, comZero, biasF, biasM);
            std::cout << "[Force] Loaded force_calib.json (mass=" << massKg
                      << "kg, bias=" << biasF[0] << "," << biasF[1] << "," << biasF[2] << "N)" << std::endl;
        } else {
            std::cout << "[Force] 无可用 force_calib.json — 按 'z' 调零。" << std::endl;
        }
```

`Touch_Client/main.cpp:1095` 附近，TCP 加载块改成：

```cpp
    const char* tcpPath = CalibStore::resolve("tcp_calib.json");
    if (tcpPath && TcpCalibration::load(tcpPath)) {
```

（其余不变。）

在 `main.cpp` 顶部 include 区加 `#include "core/CalibStore.h"`。

- [ ] **Step 8: 保存点改用 CalibStore::fileFor**

`Touch_Client/main.cpp:363` 与 `:823`：

```cpp
        if (!PayloadCalibration::save(CalibStore::fileFor("payload_calib.json"))) {
```
```cpp
        TcpCalibration::save(CalibStore::fileFor("tcp_calib.json"));
```

`Touch_Client/force/ForceCalibration.cpp:178` 与 `:309`：

```cpp
                ForceCalibration::saveToFile(CalibStore::fileFor("force_calib.json"), mass, g_biasForce, g_biasTorque);
```
```cpp
        ForceCalibration::saveToFile(CalibStore::fileFor("force_calib.json"), g_massKg, g_biasForce, g_biasTorque);
```

在 `ForceCalibration.cpp` 顶部加 `#include "../core/CalibStore.h"`。

- [ ] **Step 9: 编译并人工验证**

> ⚠ 这一步要验的是**两条不同的路径**：「目录里没有文件」（静默）和「文件过期」（告警 + 改名）。
> 只启动一次是验不全的 —— `calib\` 初始为空，走的是静默分支，**不会**打任何 `[Calib] !!`。

先确认客户端没在跑（否则链接报 `LNK1168`，不要去 kill 它）：
```bash
tasklist //FI "IMAGENAME eq Touch_Client.exe"
```
Run: `cmd.exe /c "D:\Projects\Touch\Touch_Client\build.bat"`
Expected: `Build OK.`

**9a. 空目录 -> 静默回退**

此时 `Touch_Client\calib\` 还不存在。启动：
```bash
cmd.exe /c "D:\Projects\Touch\Touch_Client\x64\Release\Touch_Client.exe" < /dev/null
```
Expected:
- `[Payload] 无可用 payload_calib.json — 用种子值 …`
- **不出现任何 `[Calib] !!` 行**（"首次启动没有标定文件"是正常情况，按设计静默）
- `ls -la Touch_Client/calib/` → 目录已创建且为空

**9b. 过期文件 -> 告警 + 改名**

往 `calib\` 放两份**没有 `saved_at_unix` 的旧格式文件**，模拟从旧版本升级上来：

```bash
cp Touch_Client/payload_calib.json Touch_Client/calib/payload_calib.json
printf '{ "version": 2, "mass_kg": 0.5, "bias_force_n": [0,0,0], "bias_torque_nm": [0,0,0] }\n' > Touch_Client/calib/force_calib.json
```

（`Touch_Client/payload_calib.json` 本来就没有 `saved_at_unix` 字段，直接拷即可。）

再启动一次。Expected:
- `[Calib] !! payload_calib.json 已作废 — 缺少 saved_at_unix 时间戳`
- `[Calib] !! force_calib.json 已作废 — 缺少 saved_at_unix 时间戳`
- `[Payload] 无可用 payload_calib.json — 用种子值 …`
- `[Force] 无可用 force_calib.json — 按 'z' 调零。`
- `ls Touch_Client/calib/` → 只剩 `payload_calib.json.expired`、`force_calib.json.expired`（原文件已消失）

- [ ] **Step 10: 提交**

```bash
git add Touch_Client/force/PayloadCalibration.cpp Touch_Client/force/ForceCalibration.cpp \
        Touch_Client/main.cpp Touch_Client/Touch_Client.vcxproj \
        Touch_Client/tests/test_payload_calibration.cpp
git commit -m "feat(calib): expire calibration files after 24h"
```

---

### Task 3: CZ 符号自动判定（求解器）

**Files:**
- Modify: `Touch_Client/force/PayloadCalibration.h`
- Modify: `Touch_Client/force/PayloadCalibration.cpp`
- Modify: `Touch_Client/tests/test_payload_calibration.cpp`

**Interfaces:**
- Consumes: 无（Task 1/2 独立）
- Produces:
  - `PayloadCalibration::Result` 新增 `signZ`、`cTrueZ[2]`、`signAmbiguous`
  - `PayloadCalibration::forcedSignZ`（`0` = 自动）
  - `PayloadCalibration::flipComSignZ()` 改为设置 `forcedSignZ`

- [ ] **Step 1: 写失败的测试**

在 `Touch_Client/tests/test_payload_calibration.cpp` 加：

```cpp
// 符号自动判定: 两个候选差 2·m_cfg·cz/m_true, 只有一个落在法兰下方。
static void test_auto_picks_physical_sign() {
    TEST(auto_picks_physical_sign);
    // 配置 0.660kg / com +80.4mm; 真值 0.409kg / com +67.9mm
    double cTrue[3] = {0.3, 0.3, 67.9};
    double cCfg[3]  = {0.0, 0.0, 80.4};
    double F[NP][3], M[NP][3];
    synthesize(0.409, cTrue, 0.660, cCfg, +1.0, F, M);   // 机械臂按 +1 解释

    PayloadCalibration::forcedSignZ = 0.0;
    PayloadCalibration::Result r;
    // 故意传入【错误】的 signZ=-1, 自动判定应当把它纠正回 +1
    CHECK(PayloadCalibration::solve(g_poses, F, M, NP, 0.660, cCfg, -1.0, r));
    CHECK(!r.signAmbiguous);
    CHECK(fabs(r.signZ - 1.0) < 1e-12);
    CHECK(r.cTrueZ[0] > 0.0);      // 候选 +1: 法兰下方, 物理
    CHECK(r.cTrueZ[1] < 0.0);      // 候选 -1: 法兰上方, 非物理
    CHECK(fabs(r.comMm[2] - 67.9) < 1e-6);
    PASS();
}

// forcedSignZ 覆盖自动判定 ('i' 键的兜底路径)
static void test_forced_sign_overrides_auto() {
    TEST(forced_sign_overrides_auto);
    double cTrue[3] = {0.3, 0.3, 67.9};
    double cCfg[3]  = {0.0, 0.0, 80.4};
    double F[NP][3], M[NP][3];
    synthesize(0.409, cTrue, 0.660, cCfg, +1.0, F, M);

    PayloadCalibration::forcedSignZ = -1.0;
    PayloadCalibration::Result r;
    CHECK(PayloadCalibration::solve(g_poses, F, M, NP, 0.660, cCfg, -1.0, r));
    CHECK(fabs(r.signZ - (-1.0)) < 1e-12);
    PayloadCalibration::forcedSignZ = 0.0;   // 复位, 别污染后面的用例
    PASS();
}
```

在 `main()` 里 `test_sign_convention();` 之后加这两行调用。

- [ ] **Step 2: 跑测试确认失败**

Run: `cmd.exe /c "D:\Projects\Touch\Touch_Client\tests\build_payload_calibration_test.bat"`
Expected: 编译失败，报 `Result` 没有成员 `signAmbiguous` / `PayloadCalibration::forcedSignZ` 未声明。

- [ ] **Step 3: 扩展头文件**

在 `Touch_Client/force/PayloadCalibration.h` 的 `struct Result` 里，`int poses;` 之后加：

```cpp
        // ===== CZ 符号自动判定 =====
        // signZ 不进 buildRows, 所以两种符号的拟合残差完全相同 —— 数据本身
        // 区分不了符号, 必须用外部判据: 工具挂在法兰下方 → 物理质心 Z 必须为正。
        double signZ;          // 实际选用的符号约定 (+1 / -1)
        double cTrueZ[2];      // {候选+1, 候选-1} 下的物理质心 Z (mm), 供人复核
        bool   signAmbiguous;  // 两个候选都合理或都不合理 → 沿用传入的 signZ
```

在 `extern double comSignZ;` 之后加：

```cpp
    // 手动强制符号约定: 0 = 自动判定 (默认), ±1 = 强制。
    // 'i' 键设置它; 开始新一批采集时清回 0。
    extern double forcedSignZ;
```

- [ ] **Step 4: 实现自动判定**

在 `Touch_Client/force/PayloadCalibration.cpp` 的全局状态区加：

```cpp
    double forcedSignZ = 0.0;
```

把 `solve()` 里从 `// ===== 换算绝对值 =====` 那一行起、**到 `// ===== 拟合残差 |A·x − b| (不是数据本身的量级) =====` 之前**的这一段替换为下面的代码。

> ⚠ 替换范围**不含**残差计算块 —— 那段要原样保留，它算的是 `out.rmsForceN` /
> `out.rmsMomentNm` / `out.poses`，紧接着才是 `return true;`。替换完确认这三行还在。

```cpp
        // ===== 换算绝对值 =====
        // signZ 不进 buildRows, 所以两种符号的拟合残差【完全相同】——
        // 数据本身区分不了符号, 必须用外部判据。
        // 判据: 工具挂在法兰下方 → 物理质心 Z 必须为正。
        // 两个候选相差 2·m_cfg·cz/m_true (本工具链约 259 mm), 一个必然非物理。
        double mTrue = mCfg + dm;
        if (!(mTrue > 0.01) || mTrue > 5.0) return false;   // 非物理

        // 注意: 下发值 cSend = cTrue/signZ 在两种符号下都可能是正的,
        // 所以判据必须看【物理】cTrue, 不能看下发值。
        double cTrueZ[2];
        for (int k = 0; k < 2; k++) {
            const double s = (k == 0) ? 1.0 : -1.0;
            const double pCfgZ = mCfg * (comCfg[2] * s) / 1000.0;
            cTrueZ[k] = (pCfgZ + dp[2]) / mTrue * 1000.0;
        }
        const bool plusOk  = (cTrueZ[0] > 0.0);
        const bool minusOk = (cTrueZ[1] > 0.0);

        double sChosen = signZ;
        bool   ambiguous = false;
        if (forcedSignZ != 0.0) {
            sChosen = forcedSignZ;                  // 人工覆盖优先
        } else if (plusOk != minusOk) {
            sChosen = plusOk ? 1.0 : -1.0;          // 恰好一个物理 -> 选它
        } else {
            ambiguous = true;                        // 都合理或都不合理 -> 沿用传入值
        }

        double cEff[3] = {comCfg[0], comCfg[1], comCfg[2] * sChosen};
        double pCfg[3] = {mCfg * cEff[0] / 1000.0,
                          mCfg * cEff[1] / 1000.0,
                          mCfg * cEff[2] / 1000.0};         // kg·m
        double pTrue[3] = {pCfg[0] + dp[0], pCfg[1] + dp[1], pCfg[2] + dp[2]};

        out.dm = dm;
        out.massKg = mTrue;
        out.signZ = sChosen;
        out.cTrueZ[0] = cTrueZ[0];
        out.cTrueZ[1] = cTrueZ[1];
        out.signAmbiguous = ambiguous;
        for (int i = 0; i < 3; i++) {
            double cTrue = pTrue[i] / mTrue * 1000.0;       // 物理质心 (mm)
            // 下发值按 sChosen 折算回去, 使机械臂的 c_eff 恰好等于 cTrue
            double cSend = (i == 2 && sChosen != 0.0) ? cTrue / sChosen : cTrue;
            out.comMm[i] = cSend;
            out.dc[i] = cSend - comCfg[i];
            if (fabs(cSend) > 500.0) return false;          // 超出 EnableRobot 的 ±500 量程
        }
```

- [ ] **Step 5: applyResult 写回符号；flipComSignZ 改为设置覆盖**

`applyResult()` 里加一行：

```cpp
        comSignZ = r.signZ;
```

把 `flipComSignZ()` 换成：

```cpp
    // 'i': 强制使用与当前约定相反的符号。求解器下次解算时不再自动判定。
    // 一批新采集开始时 (BiasCheck::reset) 会清回自动判定。
    void flipComSignZ() {
        forcedSignZ = (comSignZ >= 0.0) ? -1.0 : 1.0;
    }

    void clearForcedSignZ() {
        forcedSignZ = 0.0;
    }
```

并在 `PayloadCalibration.h` 的 `void flipComSignZ();` 之后声明：

```cpp
    // 恢复自动判定 (开始新一批采集时调用)
    void clearForcedSignZ();
```

- [ ] **Step 6: 跑测试确认通过**

Run: `cmd.exe /c "D:\Projects\Touch\Touch_Client\tests\build_payload_calibration_test.bat"`
然后: `cmd.exe /c "D:\Projects\Touch\Touch_Client\tests\test_payload_calibration.exe"`
Expected: `13 passed, 0 failed`

**若 `test_sign_convention` 失败**：那说明歧义回退没生效。该用例里两个候选都是正数（`+241.2` 与 `+80.4`），应走 `ambiguous` 分支并沿用传入的 `-1`。检查 `plusOk != minusOk` 这个条件写对了没有。

- [ ] **Step 7: 提交**

```bash
git add Touch_Client/force/PayloadCalibration.h Touch_Client/force/PayloadCalibration.cpp \
        Touch_Client/tests/test_payload_calibration.cpp
git commit -m "feat(force): pick the CZ sign convention automatically"
```

---

### Task 4: 符号判定输出 + 数据新鲜度不变式

**Files:**
- Modify: `Touch_Client/main.cpp`（`BiasCheck::reset` / `record` / `report` / `solveAndApply` / `'i'` 处理 / `'m'` 处理）

**Interfaces:**
- Consumes: `Result::signZ / cTrueZ / signAmbiguous`（Task 3）、`PayloadCalibration::clearForcedSignZ()`（Task 3）
- Produces: 无对外接口（终端行为）

- [ ] **Step 1: 给 BiasCheck 加数据新鲜度标志**

在 `Touch_Client/main.cpp` 的 `BiasCheck` 命名空间里，`static double bias[MAX_POSES][6];` 那行之后加：

```cpp
    // 数据只有在「机械臂配置 == 采集时的配置」时才可用于复验。
    // 求解下发了新负载 -> 已采数据作废 (拒绝 report 判定);
    // 但同一批数据在不同符号约定下的【重解释】始终合法 ('i' 键), 那时不改机械臂配置。
    static bool dataUnderCurrentPayload = true;
```

`reset()` 里加一行（放在 `count = 0;` 之前）：

```cpp
        dataUnderCurrentPayload = true;
        PayloadCalibration::clearForcedSignZ();   // 新一批采集 -> 恢复符号自动判定
```

`record()` 的 `if (sampling) { ... }` 检查之后、`if (count >= MAX_POSES)` 之前加：

```cpp
        // 上一批数据是在旧负载下采的 -> 从这里开始算新一批, 旧的全部丢弃
        if (!dataUnderCurrentPayload) {
            std::cout << "[BIAS] 上一批数据是在旧负载下采的, 已丢弃 — 开始新一批采集"
                      << std::endl;
            count = 0;
            dataUnderCurrentPayload = true;
            PayloadCalibration::clearForcedSignZ();
        }
```

- [ ] **Step 2: report() 拒绝判定旧数据**

`report()` 开头，`if (count < 3) { ... }` 之后加：

```cpp
        if (!dataUnderCurrentPayload) {
            std::cout << "\n[BIAS] 这批数据是在【旧负载】下采的, 不能用来复验当前参数。\n"
                      << "       请直接摆姿态按 SPACE 重新采集 (会开始新一批)。" << std::endl;
            return;
        }
```

- [ ] **Step 3: solveAndApply 展示符号判定 + 修掉两处被自动判定带偏的显示**

自动判定引入后，`solveAndApply()` 里有两处**原有**代码会显示错误或误导的信息。
它们不在 Task 3 的文件范围内，但成因是 Task 3 的改动（在此之前求解器不可能选出与传入值不同的符号），
所以在本步一并修掉。

**3a. 结果横幅打印的是旧约定。** `main.cpp:298-300` 现在把 `PayloadCalibration::comSignZ` 当作
"CZ 符号约定"打印。但 `comSignZ` 要到后面的 `applyResult()`（`main.cpp:363`）才更新，
所以当自动判定纠正了传入符号时，横幅显示的恰恰是求解器**刚刚覆盖掉**的旧约定。改为打印本次解算结果：

```cpp
        std::cout << "\n======================================================" << std::endl;
        std::cout << "  负载参数求解结果 (" << r.poses << " 个姿态, CZ 符号约定 "
                  << (r.signZ > 0 ? "+1" : "-1")
                  << (r.signAmbiguous ? " ⚠ 歧义, 沿用旧约定" : " (自动判定)") << ")" << std::endl;
        std::cout << "======================================================" << std::endl;
```

**3b. 质心判据测错了量。** `main.cpp:321` 是：

```cpp
        const bool   comZOk     = (r.comMm[2] > 0.0);   // 工具挂在法兰下方 → 质心 Z 应为正
```

注释说的是**物理**判据，代码测的却是**下发值** `r.comMm[2]`。两者在 `signZ = -1` 时符号相反
（`cSend = cTrue / signZ`），于是**正确求解的结果会被报成"符号可疑"**。
自动判定让这条分支无需人工干预就可能触发，所以必须改。替换为：

```cpp
        // 判据必须看【物理】质心: cSend = cTrue/signZ 在两种符号下都可能是正的,
        // 拿下发值判会在 signZ=-1 时把正确结果误报成"符号可疑"。
        const double cTrueZChosen = r.cTrueZ[r.signZ > 0.0 ? 0 : 1];
        const bool   comZOk       = (cTrueZChosen > 0.0);   // 工具挂在法兰下方 → 质心 Z 应为正
```

并把紧随其后的提示行改为打印物理值（顺带带上下发值便于对照）：

```cpp
        if (comZOk) {
            printf("    ✓ 物理质心 Z = %+.1f mm 在法兰下方 (下发 %+.1f mm), 偏心 |XY| = %.1f mm\n",
                   cTrueZChosen, r.comMm[2], comXY);
        } else {
            printf("    ⚠ 物理质心 Z = %+.1f mm 为负 — 工具挂在法兰下方, 应为正;"
                   " 符号约定可疑\n", cTrueZChosen);
        }
```

`comXY` 继续用 `r.comMm` 的 X/Y —— 这两个分量不参与符号折算，两种约定下相同。

**3c. 符号判定展示。** 在 `printf("  拟合残差: …)` 那一行之后插入（位置在 `// ===== 合理性评估 =====` 之前）：

```cpp
        // ===== CZ 符号约定判定 =====
        // signZ 不进线性系统, 两种符号的拟合残差完全相同 —— 数据区分不了,
        // 判据是"物理质心必须在法兰下方"。把两个候选都打出来, 便于人工复核。
        if (r.signAmbiguous) {
            std::cout << "  CZ 符号约定: ⚠ 无法判定 — 两个候选都落在同一侧, 沿用当前 "
                      << (PayloadCalibration::comSignZ > 0 ? "+1" : "-1") << std::endl;
        } else {
            std::cout << "  CZ 符号约定: 自动判定 → "
                      << (r.signZ > 0 ? "+1" : "-1") << std::endl;
        }
        printf("    · 候选 +1: 物理质心 Z = %+.1f mm  %s\n",
               r.cTrueZ[0], r.cTrueZ[0] > 0 ? "✓ 法兰下方" : "✗ 法兰上方 (非物理)");
        printf("    · 候选 -1: 物理质心 Z = %+.1f mm  %s\n",
               r.cTrueZ[1], r.cTrueZ[1] > 0 ? "✓ 法兰下方" : "✗ 法兰上方 (非物理)");
        if (r.signAmbiguous) {
            std::cout << "    (若结果不对, 按 'i' 强制用另一个符号)" << std::endl;
        }
```

- [ ] **Step 4: 下发后把数据标记为过期**

在 `solveAndApply()` 里，把 `RelayCore::instance().applyPayloadToRobot();` 之后的这一整段（`f7a5d46` 引入的）：

```cpp
        // 这批姿态是在【旧负载】下采的, 拿来复验新参数只会得出"FAIL"的假象
        // (曾经就是这样: 求解残差 0.06 N, 紧接着的报告却报 4.0 N)。
        // 直接清掉, 逼着重新采集 —— 复验必须用新负载下采的数据。
        reset();
        std::cout << "  已清空本次采集数据 (旧负载下采的, 不能用于复验)" << std::endl;
        std::cout << "  → 保持 'm' 模式, 直接摆姿态按 SPACE 重新采 3~4 个【复验】,"
                  << " 再看跨姿态极差" << std::endl;
        std::cout << "    若残差反而变大 → 说明机械臂解释 CZ 的符号与我们假设相反:"
                  << " 按 'i' 翻转 → 重新采集 → 再 's'" << std::endl;
        std::cout << std::endl;
    }
```

替换为：

```cpp
        // 这批姿态是在【旧负载】下采的。下发新负载之后:
        //   - 复验 (report) 必须拒绝它们, 否则会拿旧数据骂新参数 (曾经报出假 FAIL:
        //     求解残差 0.06 N, 紧接着的报告却报 |ΔF| = 4.0 N);
        //   - 但 'i' 的【重解释】仍然合法 —— 那不改机械臂配置, 只是换个符号看同一批
        //     测量, 不能把数据清掉 (清了 'i' 就没得重解了)。
        dataUnderCurrentPayload = false;
        std::cout << "  → 复验: 摆姿态按 SPACE 采集 (第一次 SPACE 会自动开新一批,"
                  << " 旧数据作废)" << std::endl;
        std::cout << "    若复验残差反而变大 → 按 'i' 用同一批数据换符号重解,"
                  << " **不必重新采集**" << std::endl;
        std::cout << std::endl;
    }
```

- [ ] **Step 5: 'i' 改为立即重解**

> ⚠ **本步已被 Task 5 取代。** 设计修订（2026-09-18）后 `'i'` 这整个人工覆盖被移除，
> 符号只由程序判定。保留原文只为忠实记录 Task 4 当时的实施内容；**最终态见 Task 5 Step 5**
> （删掉整块）。执行 Task 5 时不要被这里的旧代码误导。

把 `main.cpp` 里的 `'i'` 处理替换为：

```cpp
    if ((key == 'i' || key == 'I') && BiasCheck::mode) {
        PayloadCalibration::flipComSignZ();
        std::cout << "[BIAS] 强制 CZ 符号约定 → "
                  << (PayloadCalibration::forcedSignZ > 0 ? "+1" : "-1")
                  << " — 用同一批数据重解 (不需要重新采集)" << std::endl;
        BiasCheck::solveAndApply();
        return;
    }
```

> `solveAndApply()` 在 `count < 4` 时会自己拒绝并提示，这里不用重复检查。

- [ ] **Step 6: 'm' 的退出分支加注释**

`main.cpp` 的 `'m'` 处理里，把 `else` 分支改为（只是加注释说明 `report()` 会自行拒绝旧数据，逻辑不变）：

```cpp
        } else {
            BiasCheck::mode = false;
            // 别把柔顺状态带出模式
            RelayCore::instance().setDragMode(false);
            BiasCheck::report();   // 数据属旧负载时会自行拒绝判定
        }
```

- [ ] **Step 7: 编译**

先确认客户端没在跑：
```bash
tasklist //FI "IMAGENAME eq Touch_Client.exe"
```
Run: `cmd.exe /c "D:\Projects\Touch\Touch_Client\build.bat"`
Expected: `Build OK.`

- [ ] **Step 8: 实机验证（需要机械臂在线）**

> ⚠ **设计修订后 `'i'` 已不存在**（Task 5 移除）。符号完全由程序判定，判不出即判不合理。
> 本清单已按最终态更新——**不要**照着早期版本找 `'i'` 步骤。

启动客户端，按顺序验证：

1. `'m'` 进模式 → 摆 3~4 个姿态按 `SPACE` → `'m'` 退出
   Expected: 正常输出报告，**不**出现"旧负载"拒绝提示（还没求解过）

2. `'m'` 再进 → 摆 ≥4 个姿态（跨度≥30°，笔要有水平/朝上的）→ `'s'`
   Expected 求解输出里出现：
   ```
     负载参数求解结果 (N 个姿态, CZ 符号约定 +1 (自动判定))
     ...
     CZ 符号约定: 自动判定 → +1
       · 候选 +1: 物理质心 Z = +67.9 mm  ✓ 法兰下方
       · 候选 -1: 物理质心 Z = -191.6 mm ✗ 法兰上方 (非物理)
     ------------------------------------------------------
     合理性评估 (三条判据, 任一不满足即拒绝下发):
       ✓ 拟合残差 ... N ...
       ✓ 符号可判定 (候选 +1: ..., 候选 -1: ...)
       ✓ 物理质心 Z = ... mm 在法兰下方 (下发 ... mm), 偏心 |XY| = ... mm
     ------------------------------------------------------
     已保存 payload_calib.json (下次启动自动加载)
   ```
   **三条判据必须全是 ✓。** 只要有一条 ✗，就应看到
   `判定: ✗ 不合理 — 已【拒绝保存和下发】`，并且**不该**出现"已保存"那一行。
   > 重点核对：**横幅上的符号** 与 **判定块的"自动判定 →"** 必须是同一个值。

3. 紧接着按 `'m'` 退出
   Expected: 出现"这批数据是在【旧负载】下采的, 不能用来复验当前参数"（以前这里会假报 FAIL）

4. `'m'` 进 → 摆 3~4 个姿态按 `SPACE`
   Expected: 第一次 SPACE 时提示"上一批数据是在旧负载下采的, 已丢弃 — 开始新一批采集"

5. `'m'` 退出
   Expected: 报告基于新数据；`|ΔF|` 应显著小于求解前的 **4.0 N**

6. **验证闸门真会拦**（可选，但推荐）：
   按 `'m'` → 采 4 个姿态，**在每次 SPACE 后的 1 秒采样窗口内轻推机械臂**（制造脏数据）→ `'s'`
   Expected: 残差项 ✗，出现"已【拒绝保存和下发】"，且**没有**下发。
   连按 3 次 `'s'` 都应被拒，第 3 次后看到"已连续 3 次不合理, 停止求解"；
   此后按 `'s'` 只提示不再求解。按 `'m'` 重新进入后应恢复正常（计数清零）。

- [ ] **Step 9: 提交**

```bash
git add Touch_Client/main.cpp
git commit -m "feat(force): show the CZ sign decision and gate verification on fresh data"
```

---

### Task 5: 移除人工符号覆盖 + 合理性硬判据 + 连续失败升级

> **设计修订（2026-09-18，实机评审后）。** 用户否决了 `'i'` 这个人工兜底方向
> （"这种判断需要程序计算解决，不要引入人工判断"），并要求程序对结果合理性负责、
> 多次不合理要报错（"如果不合理需要再次采集，多次不合理需要报错"）。
> 详见 spec §3.5 / §3.6。**本任务会删除 Task 3 交付的一部分 API —— 那是有意的。**

**Files:**
- Modify: `Touch_Client/config/Config.h`
- Modify: `Touch_Client/force/PayloadCalibration.h`
- Modify: `Touch_Client/force/PayloadCalibration.cpp`
- Modify: `Touch_Client/main.cpp`
- Modify: `Touch_Client/tests/test_payload_calibration.cpp`

**Interfaces:**
- Consumes: `Result::signZ` / `cTrueZ[2]` / `signAmbiguous`（Task 3）
- Produces: 无新增对外接口；**删除** `PayloadCalibration::forcedSignZ`、`flipComSignZ()`、`clearForcedSignZ()`

- [ ] **Step 1: 加连续失败阈值常量**

`config/Config.h` 的 `CALIB_MAX_AGE_SEC` 之后加：

```cpp
    // 连续多少次"结果不合理"就打红字错误并停止接受求解。
    // 防的是操作者反复按 's' 却每次被拒的空转 —— 那种情况下问题在硬件/采集, 不在求解器。
    const int CALIB_MAX_CONSECUTIVE_FAILS = 3;
```

- [ ] **Step 2: 删掉人工符号覆盖（头文件）**

`force/PayloadCalibration.h` 里删除这三项：

```cpp
    // 手动强制符号约定: 0 = 自动判定 (默认), ±1 = 强制。
    // 'i' 键设置它; 开始新一批采集时清回 0。
    extern double forcedSignZ;
```
```cpp
    // 恢复自动判定 (开始新一批采集时调用)
    void clearForcedSignZ();
```
```cpp
    // CZ 符号约定 (由实机实测判定; 翻转后需重新采集再求解)
    void flipComSignZ();
```

（`flipComSignZ()` 的声明保留原文的注释行，一并删。）

**同时**给 `Result` 里 Task 3 新增的三个字段加默认初始化 —— 这是之前那颗"未初始化的地雷"的根因：

```cpp
        double signZ         = 1.0;    // 实际选用的符号约定 (+1 / -1)
        double cTrueZ[2]     = {0.0, 0.0};  // {候选+1, 候选-1} 下的物理质心 Z (mm)
        bool   signAmbiguous = true;   // 默认 true = 不可信, 不让漏填的 Result 看起来可用
```

- [ ] **Step 3: 删掉实现**

`force/PayloadCalibration.cpp`：删除 `double forcedSignZ = 0.0;`、`flipComSignZ()`、`clearForcedSignZ()` 三个定义。

`solve()` 里的符号选择去掉人工分支：

```cpp
        double sChosen = signZ;
        bool   ambiguous = false;
        if (plusOk != minusOk) {
            sChosen = plusOk ? 1.0 : -1.0;          // 恰好一个物理 -> 选它
        } else {
            ambiguous = true;                        // 都合理或都不合理 -> 程序判不了
        }
```

（原来是 `if (forcedSignZ != 0.0) {...} else if (plusOk != minusOk) {...} else {...}`，删掉第一个分支。）

- [ ] **Step 4: 测试删掉强制符号用例**

`tests/test_payload_calibration.cpp`：删除 `test_forced_sign_overrides_auto()` 整个函数，并从 `main()` 里删掉它的调用。用例数应为 **12**。

- [ ] **Step 5: main.cpp 删掉 `'i'` 处理**

删除整个：

```cpp
    if ((key == 'i' || key == 'I') && BiasCheck::mode) {
        ... 整块 ...
    }
```

- [ ] **Step 6: 合理性三判据**

在 `BiasCheck` 命名空间里加失败计数：

```cpp
    // 连续多少次求解被判"不合理"。达到 Config::CALIB_MAX_CONSECUTIVE_FAILS 后锁住 's'。
    static int  consecutiveFails = 0;
    static bool solveLocked = false;
```

`reset()` 里加（放在 `count = 0;` 之前）：

```cpp
        consecutiveFails = 0;
        solveLocked = false;
```

`solveAndApply()` 开头，在 `if (count < 4)` 之前加：

```cpp
        if (solveLocked) {
            std::cout << "\n[BIAS] !! 已连续 " << consecutiveFails << " 次判定结果不合理, 已停止求解。\n"
                      << "       [BIAS] !! 问题多半不在求解器 —— 请检查: 机械臂装夹是否松动 / "
                      << "力传感器是否受挤压 / 姿态覆盖是否足够。\n"
                      << "       [BIAS] !! 处理后按 'm' 重新采集 (计数会清零)。" << std::endl;
            return;
        }
```

把 `solveAndApply()` 里现有的判据块（`RMS_F_GOOD` … `if (!fitOk) { ... return; }`）整体替换为：

```cpp
        // ===== 合理性判据 =====
        // 任一命中即"不合理" -> 拒绝保存和下发, 机械臂保持原参数。
        // 绝不拿一个程序自己都判定为不可信的结果去配置机械臂。
        const double RMS_F_GOOD = 0.10;   // N — 到这个量级才说明模型与数据一致
        const double RMS_F_MAX  = 0.30;   // N — 超过即不合理 (实机: 约定对 ~0.06, 错 ~0.9)
        const bool   fitGood = (r.rmsForceN < RMS_F_GOOD);
        const bool   fitOk   = (r.rmsForceN < RMS_F_MAX);
        // 物理质心 (不是下发值): cSend = cTrue/signZ 在两种符号下都可能是正的
        const double cTrueZChosen = r.cTrueZ[r.signZ > 0.0 ? 0 : 1];
        const bool   signOk = !r.signAmbiguous;     // 程序必须能判出符号
        const bool   comZOk = (cTrueZChosen > 0.0); // 工具挂在法兰下方 -> 质心 Z 必须为正
        const double comXY  = sqrt(r.comMm[0] * r.comMm[0] + r.comMm[1] * r.comMm[1]);
        const bool   reasonable = fitOk && signOk && comZOk;

        std::cout << "------------------------------------------------------" << std::endl;
        std::cout << "  合理性评估 (三条判据, 任一不满足即拒绝下发):" << std::endl;
        if (fitGood) {
            printf("    ✓ 拟合残差 %.4f N ≈ 噪声本底 (< %.2f N)\n", r.rmsForceN, RMS_F_GOOD);
        } else if (fitOk) {
            printf("    ✓ 拟合残差 %.4f N 在容许范围内 (< %.2f N)\n", r.rmsForceN, RMS_F_MAX);
        } else {
            printf("    ✗ 拟合残差 %.4f N 超过阈值 %.2f N — 模型解释不了这批数据\n",
                   r.rmsForceN, RMS_F_MAX);
        }
        if (signOk) {
            printf("    ✓ 符号可判定 (候选 +1: %+.1f mm, 候选 -1: %+.1f mm)\n",
                   r.cTrueZ[0], r.cTrueZ[1]);
        } else {
            printf("    ✗ 符号无法判定 — 两个候选都在同一侧 (+1: %+.1f mm, -1: %+.1f mm)\n",
                   r.cTrueZ[0], r.cTrueZ[1]);
        }
        if (comZOk) {
            printf("    ✓ 物理质心 Z = %+.1f mm 在法兰下方 (下发 %+.1f mm), 偏心 |XY| = %.1f mm\n",
                   cTrueZChosen, r.comMm[2], comXY);
        } else {
            printf("    ✗ 物理质心 Z = %+.1f mm 不在法兰下方 — 工具装夹或符号有问题\n",
                   cTrueZChosen);
        }

        std::cout << "------------------------------------------------------" << std::endl;
        if (!reasonable) {
            consecutiveFails++;
            std::cout << "  判定: ✗ 不合理 — 已【拒绝保存和下发】, 机械臂负载参数保持原值 (第 "
                      << consecutiveFails << " 次)" << std::endl;
            std::cout << "  → 请按 'm' 重新采集 (姿态跨度≥30°, 笔要有水平/朝上的姿态), 再按 's'"
                      << std::endl;
            if (consecutiveFails >= Config::CALIB_MAX_CONSECUTIVE_FAILS) {
                solveLocked = true;
                std::cout << std::endl;
                std::cout << "  [BIAS] !! 已连续 " << consecutiveFails << " 次不合理, 停止求解。"
                          << std::endl;
                std::cout << "  [BIAS] !! 问题多半不在求解器 —— 请检查: 机械臂装夹是否松动 / "
                          << "力传感器是否受挤压 / 姿态覆盖是否足够。" << std::endl;
                std::cout << "  [BIAS] !! 处理后按 'm' 重新采集 (计数会清零)。" << std::endl;
            }
            std::cout << std::endl;
            return;
        }
        consecutiveFails = 0;   // 成功一次即清零
```

> ⚠ 注意：**质量修正幅度的告警已按用户要求删除**（"不报警，这也许是加装了别的设施，
> 正符合重新标定的作用"）。替换后不应再出现 `MASS_JUMP` / `massJump` / `dMassFrac`。
> 质量数值本身仍由上面的"质量:"行展示，只是不再做判断。

- [ ] **Step 7: 清掉三处被新语义带偏的文案**

`solveAndApply()` 横幅里的歧义后缀（`'i'` 已不存在）：

```cpp
                  << (r.signAmbiguous ? " ⚠ 判不出" : " (自动判定)") << ")" << std::endl;
```

3c 判定块里那行 `(若结果不对, 按 'i' 强制用另一个符号)` 整句删除。歧义时改为：

```cpp
        if (r.signAmbiguous) {
            std::cout << "    (两个候选都在同一侧, 程序判不出符号 —— 结果按不合理处理)"
                      << std::endl;
        }
```

`'m'` 模式进入帮助（现在含 `'i' 翻转 CZ 符号约定`）改为：

```cpp
            std::cout << "\n[BIAS] Mode ON — 笔尖悬空, 只改姿态(位置尽量不变),"
                      << " 每到一个姿态按 SPACE (采样 1s)\n"
                      << "       'd' 拖拽模式开关 (摆姿态用; 摆好一定要关掉再采样) /"
                      << " 'm' 退出并输出报告\n"
                      << "       's' 求解负载参数并下发 (符号与合理性由程序自动判定)"
                      << std::endl;
```

- [ ] **Step 8: 让 `record()` 复用 `reset()`**

`record()` 里新批次那段现在是三条重复语句（`count = 0; dataUnderCurrentPayload = true;` + 已删的 `clearForcedSignZ()`）。改为：

```cpp
        // 上一批数据是在旧负载下采的 -> 从这里开始算新一批。
        // 复用 reset(), 免得将来 reset() 加了字段而这里漏跟。
        if (!dataUnderCurrentPayload) {
            std::cout << "[BIAS] 上一批数据是在旧负载下采的, 已丢弃 — 开始新一批采集"
                      << std::endl;
            reset();
        }
```

- [ ] **Step 9: 编译并跑测试**

Run: `cmd.exe //c "D:\Projects\Touch\Touch_Client\tests\build_payload_calibration_test.bat"`
然后: `cmd.exe //c "D:\Projects\Touch\Touch_Client\tests\test_payload_calibration.exe"`
Expected: `12 passed, 0 failed`

Run: `cmd.exe //c "D:\Projects\Touch\Touch_Client\build.bat"`
Expected: `Build OK.`（**必须是完整构建**，独立测试脚本的编译环境与真实项目不同）

- [ ] **Step 10: 提交**

```bash
git add Touch_Client/config/Config.h Touch_Client/force/PayloadCalibration.h \
        Touch_Client/force/PayloadCalibration.cpp Touch_Client/main.cpp \
        Touch_Client/tests/test_payload_calibration.cpp
git commit -m "feat(force): decide the CZ sign in software only, and reject unreasonable solves"
```

---

### Task 7: 符号判定改为「种子锚点 + 余量判据」

> **第二轮设计修订（2026-09-18，终审后）。** 终审证明 Task 3/5 的判据
> **永远选不出与当前配置相反的符号**（详见 spec §7.2）——它只是"确认或拒绝"。
> 用户决定改用外部锚点，并追问"种子值做锚点是否存在错误估计的风险"。
> 余量判据就是为这个风险设的闸。

**Files:**
- Modify: `Touch_Client/config/Config.h`
- Modify: `Touch_Client/force/PayloadCalibration.cpp`
- Modify: `Touch_Client/tests/test_payload_calibration.cpp`

**Interfaces:**
- Consumes: `Result::cTrueZ[2]`（Task 3）、`Config::ROBOT_PAYLOAD_SEED_CZ_MM`
- Produces: `Config::SIGN_SEED_MARGIN_RATIO`；`solve()` 的符号选取改为锚点法（`Result` 字段不变）

- [ ] **Step 1: 加余量阈值常量**

`config/Config.h`，紧跟 `ROBOT_PAYLOAD_SEED_CZ_MM`（约 :63）之后加 —— **它是判定的锚点，不再是"仅首次兜底"**：

```cpp
    // ===== CZ 符号判定的锚点 =====
    // signZ 不进拟合方程 (两种符号残差完全相同), 所以必须外部判据。
    // 取法: 选离 ROBOT_PAYLOAD_SEED_CZ_MM 更近的那个候选。
    // 上面那个种子值因此从"仅首次兜底"升级为【判定的外部锚点】, 准确性开始承担判定责任
    // —— 换装差异大的工具后记得重跑 Hardware/tools/compute_payload.py。
    //
    // 余量判据: 只有选中的候选【显著】更近才采纳。
    //   选错的临界点是种子偏离真值超过两候选间距的一半
    //   (间距 = 2·m_cfg·cz/m_true, 实机 259.4 mm → 临界 129.7 mm;
    //    当前种子误差 12.5 mm, 余量约 10 倍)。
    //   这个比值就是防"种子本身失真"的闸: 不够显著就判不可判定, 而不是静默选错。
    const double SIGN_SEED_MARGIN_RATIO = 3.0;
```

- [ ] **Step 2: 写失败的测试**

在 `tests/test_payload_calibration.cpp` 加：

```cpp
// 符号判定: 按"离种子更近"选取, 且要求距离比 ≥ SIGN_SEED_MARGIN_RATIO。
// 真值 +1 时应纠正一个错误的传入符号 —— 这是旧判据【做不到】的
// (旧判据恒等于 sign(comCfg[2]), 只能确认不能翻转)。
static void test_sign_seed_anchor_picks_plus() {
    TEST(sign_seed_anchor_picks_plus);
    double cTrue[3] = {0.3, 0.3, 67.9};
    double cCfg[3]  = {0.0, 0.0, 80.4};   // 与种子同向
    double F[NP][3], M[NP][3];
    synthesize(0.409, cTrue, 0.660, cCfg, +1.0, F, M);

    PayloadCalibration::Result r;
    // 故意传入错误的 signZ=-1: 锚点应把它纠正回 +1
    CHECK(PayloadCalibration::solve(g_poses, F, M, NP, 0.660, cCfg, -1.0, r));
    CHECK(!r.signAmbiguous);
    CHECK(fabs(r.signZ - 1.0) < 1e-12);
    CHECK(fabs(r.comMm[2] - 67.9) < 1e-6);
    PASS();
}

// 真值 -1 的机器: 候选 +327.3 / +67.8 —— 【两个都是正的】,
// 旧判据在这里必然判歧义并永久拒绝; 锚点法应选 -1。
static void test_sign_seed_anchor_picks_minus() {
    TEST(sign_seed_anchor_picks_minus);
    double cTrue[3] = {0.3, 0.3, 67.9};
    double cCfg[3]  = {0.0, 0.0, 80.4};
    double F[NP][3], M[NP][3];
    synthesize(0.409, cTrue, 0.660, cCfg, -1.0, F, M);   // 机械臂按 -1 解释

    PayloadCalibration::Result r;
    CHECK(PayloadCalibration::solve(g_poses, F, M, NP, 0.660, cCfg, +1.0, r));
    CHECK(!r.signAmbiguous);
    CHECK(fabs(r.signZ - (-1.0)) < 1e-12);
    // 下发值按 signZ 折算回去, 使机械臂的 c_eff 恰好等于物理质心
    CHECK(fabs(r.comMm[2] - (-67.9)) < 1e-6);
    PASS();
}

// 余量不足 -> 判不可判定, 而不是静默选一个
static void test_sign_insufficient_margin_is_ambiguous() {
    TEST(sign_insufficient_margin_is_ambiguous);
    // 构造两个候选都与种子距离相近的场景: 种子本来就该在中点附近
    // 直接把种子当成锚点不可注入, 所以这里改为验证【距离比】这个纯判断
    CHECK(PayloadCalibration::seedMarginSufficient(10.0, 40.0));    // 比 4.0 -> 够
    CHECK(!PayloadCalibration::seedMarginSufficient(30.0, 40.0));   // 比 1.33 -> 不够
    CHECK( PayloadCalibration::seedMarginSufficient(0.0, 40.0));    // 锚点精确命中 -> 够
    CHECK(!PayloadCalibration::seedMarginSufficient(0.0, 0.0));     // 两候选重合 -> 退化, 不够
    PASS();
}
```

在 `main()` 里加这三个调用。

- [ ] **Step 3: 跑测试确认失败**

Run: `build_payload_calibration_test.bat` 然后跑 exe
Expected: 编译失败 —— `seedMarginSufficient` 未声明。

- [ ] **Step 4: 导出余量判据（纯函数，便于单测）**

`force/PayloadCalibration.h`，在 `bool solve(...)` 之前加：

```cpp
    // 纯函数: 余量是否足够采纳锚点选出的候选。
    // near = 选中候选到种子的距离, far = 另一个候选的距离。
    // 距离比必须 ≥ Config::SIGN_SEED_MARGIN_RATIO, 否则判不可判定。
    bool seedMarginSufficient(double near, double far);
```

- [ ] **Step 5: 实现锚点判定**

`force/PayloadCalibration.cpp` 加：

```cpp
    bool seedMarginSufficient(double near, double far) {
        // near == 0 是锚点的【最佳】情形 (候选与种子精确重合), 不是退化 —— 必须放行。
        // 真正退化的是两个候选【互相重合】: 那时 far 也接近 0, 锚点信息量为零。
        if (!(far > 0.0)) return false;      // 两候选重合 -> 无从判别
        if (!(near > 0.0)) return true;      // 锚点精确命中 -> 采纳
        return (far / near) >= Config::SIGN_SEED_MARGIN_RATIO;
    }
```

把 `solve()` 里选符号那段（`plusOk` / `minusOk` / `sChosen` / `ambiguous`）替换为：

```cpp
        // ===== 符号选取: 种子锚点 + 余量判据 =====
        // signZ 不进拟合方程, 两种符号的残差完全相同 -> 必须外部判据。
        // 旧判据 (看哪个候选为正) 在代数上恒等于 sign(comCfg[2]), 只能确认不能翻转
        // (X>0 时 minusOk 蕴含 plusOk, 非歧义分支永远是 +1), 因此在"配置符号本就错"
        // 时会把两个都为正的候选判成歧义、永久拒绝。改用外部锚点破环。
        const double seedZ = Config::ROBOT_PAYLOAD_SEED_CZ_MM;
        const double d0 = fabs(cTrueZ[0] - seedZ);
        const double d1 = fabs(cTrueZ[1] - seedZ);
        const double dNear = (d0 <= d1) ? d0 : d1;
        const double dFar  = (d0 <= d1) ? d1 : d0;

        double sChosen = signZ;
        bool   ambiguous = false;
        if (!seedMarginSufficient(dNear, dFar)) {
            ambiguous = true;                       // 余量不足 -> 不静默选
        } else {
            sChosen = (d0 <= d1) ? 1.0 : -1.0;
        }
```

后面原有的 `cEff` / `pCfg` / `pTrue` / `out.*` 换算**保持不变**，但**再加一道物理交叉检查** —— 锚点选中的候选必须为正：

```cpp
        if (!ambiguous) {
            const double cChosen = (sChosen > 0.0) ? cTrueZ[0] : cTrueZ[1];
            if (!(cChosen > 0.0)) ambiguous = true;   // 锚点与物理约束矛盾 -> 不可判定
        }
```

（放在 `sChosen` 定下来之后、`cEff` 计算之前。）

- [ ] **Step 6: 跑测试确认通过**

Run: `build_payload_calibration_test.bat` 然后跑 exe
Expected: **15 passed, 0 failed**（12 + 3 个新用例）

> `test_sign_convention` 现在会走锚点分支：它合成的是 −1 约定，候选 `+241.2 / +80.4`，
> 种子 80.4 离 **+80.4** 更近（距离 0 vs 160.8）→ 选 **−1** ✓
> 恰好也是该用例期望的 `comMm[2] = -80.4`。**若它挂了，先查锚点距离算对没有。**

- [ ] **Step 7: 完整构建**

Run: `cmd.exe //c "D:\Projects\Touch\Touch_Client\build.bat"`
Expected: `Build OK.`（**必须完整构建** —— 独立测试脚本的编译环境与真实项目不同）

- [ ] **Step 8: 提交**

```bash
git add Touch_Client/config/Config.h Touch_Client/force/PayloadCalibration.h \
        Touch_Client/force/PayloadCalibration.cpp Touch_Client/tests/test_payload_calibration.cpp
git commit -m "feat(force): break the CZ sign tie with an external anchor and a margin gate"
```

---

### Task 8: 移除 24h 时间闸门

> **第二轮设计修订。** §7.1：终审发现 TCP 过了闸门却从不写时间戳、每次启动被销毁。
> 用户决定不用补时间戳的办法，而是**换掉整个机制**（启动自检，见 Task 9）。
> 本任务只做**减法**：把时间闸门拆掉，`CalibStore` 退化为纯位置模块。

**Files:**
- Modify: `Touch_Client/core/CalibStore.h`
- Modify: `Touch_Client/core/CalibStore.cpp`
- Modify: `Touch_Client/config/Config.h`
- Modify: `Touch_Client/main.cpp`
- Modify: `Touch_Client/force/PayloadCalibration.cpp`
- Modify: `Touch_Client/force/ForceCalibration.cpp`
- Modify: `Touch_Client/tests/test_calib_store.cpp`
- Modify: `Touch_Client/tests/test_payload_calibration.cpp`
- ~~Delete: `Touch_Client/tests/build_calib_store_test.bat`~~ —— **不删**。Step 5 保留三个 `deriveDir` 用例，
  Step 6 还要用它跑那三个（条件不成立）。

> **⚠ 2026-09-19 控制器 pre-flight 更正（四处，均为事实性）。** 本节写于 ψ 自动求解之前，
> 有四处与今天的代码对不上：
> 1. **Step 4 里"删掉 main.cpp 里两处 `saved_at_unix` 相关的注释/提示"是空指令** ——
>    `grep -n "有效期\|过期\|expired\|AGE_SEC\|saved_at_unix" Touch_Client/main.cpp` **零命中**。
>    那些提示已在 Task 5 重写输出时一并消失，没有东西可删。
> 2. **Step 5/6 的 payload 用例数是过时的**：那是 15−1=14，而今天是 **21**（ψ 的扫描/一致性/
>    转置回归等用例在 Task 10 之后加的）。删掉 `test_save_includes_timestamp()` 后是 **20**。
>    `test_calib_store` 的 5−2=3 仍然正确。
> 3. **`PayloadCalibration.cpp:299-300` 有指向被删机制的注释**（"调用方必须先过
>    `CalibStore::resolve()`…由它挡掉缺 `saved_at_unix` 或已过期的文件"）。删了 `resolve` 之后
>    这两行就是悬空引用，必须一并改写（该文件已在 Files 清单里）。
> 4. **`version` 回退是惰性的**：`grep -n version` 在 `PayloadCalibration.cpp` / `ForceCalibration.cpp`
>    里只命中各自的 `save()` 那一行 —— **没有任何读取端校验 version**。所以回到 1/2 不会破坏兼容，
>    不必去找有没有别处依赖它。（`test_save_includes_timestamp` 是唯一断言 `"version": 2` 的地方，
>    而它正在被删除。）

- [ ] **Step 1: 删掉 CalibStore 的有效期部分**

`core/CalibStore.h`：删除 `resolveIn()`、`resolve()`、`isFresh()` 三个声明。
保留 `deriveDir()` / `dir()` / `fileFor()`，并把命名空间的头注释改为只讲位置：

```cpp
// 标定文件放哪。
//
// 为什么需要它: 三个标定文件原本都用相对路径 ("./payload_calib.json") 读写,
// 落在【当前工作目录】—— 从 Touch_Client\ 启动和从 x64\Release\ 启动会拿到
// 两份不同的文件。
//
// 注: 本模块【不管有效期】。标定是否仍然可信由启动自检实测判定 (见 main.cpp),
//     不用时间闸门 —— 时间只是代理指标, 而这里能直接测。
```

`core/CalibStore.cpp`：删除 `isFresh()`、`readLongField()`、`resolveIn()`、`resolve()` 四个定义。
保留 `deriveDir()` / `dir()` / `fileFor()`；`<ctime>` 与 `<cstdio>` 若不再需要可一并删。

- [ ] **Step 2: 删掉配置常量**

`config/Config.h`：删除 `CALIB_MAX_AGE_SEC`（连它的注释块）。

- [ ] **Step 3: 写入端去掉时间戳字段**

- `force/PayloadCalibration.cpp` 的 `save()`：删掉 `"saved_at_unix"` 那一行，`version` 回到 1
- `force/ForceCalibration.cpp` 的 `saveToFile()`：同上，`version` 回到 2

> 保留时间戳字段作为纯元数据也是一种选择，但既然不再有任何判定依赖它，
> 少一个字段就少一处会漂移的真相副本。删干净。

- [ ] **Step 4: 读取端改用 fileFor**

- `main.cpp`：三处 `CalibStore::resolve("xxx.json")` 改为 `CalibStore::fileFor("xxx.json")`，
  并把 `if (p && Xxx::load(p))` 改回 `if (Xxx::load(CalibStore::fileFor("xxx.json")))`
  （各解析器自己处理"文件不存在"）
- 删掉 `force/PayloadCalibration.cpp:299-300` 那两行指向 `resolve()` / `saved_at_unix` 的注释
  （见上方 pre-flight 第 3 条）

> 注意 `fileFor()` 返回 static 缓冲：**每个文件立即用完**，不要跨调用持有。
> 三处调用点今天都是 `const char* xxxPath = ...` 局部量（`main.cpp:1222` payload、
> `:1262` force、`:1289` tcp）。改成 `fileFor` 后**不要把指针单独存下来** —— 直接写进
> `if` 条件里一次用完。（`tcpPath` 尤其要小心：它今天活到 main 顶层、直到 GLUT 循环，
> 属既有隐患, 见终审 Minor #24。）

- [ ] **Step 5: 测试清理**

`tests/test_payload_calibration.cpp`：删除 `test_save_includes_timestamp()` 及其 `main()` 调用。
用例数回到 **20**（21 − 1；**不是原文写的 14**）。

`tests/test_calib_store.cpp`：删除 `test_is_fresh_boundaries()`、`test_resolve_policy()`、
`writeFixture()`、`fixtureExists()` 及各自的 `main()` 调用；保留 `deriveDir` 三个用例。
用例数 **3**。

- [ ] **Step 6: 跑测试 + 完整构建**

Run: `build_calib_store_test.bat` → exe → Expected `3 passed, 0 failed`
Run: `build_payload_calibration_test.bat` → exe → Expected `20 passed, 0 failed`
Run: `cmd.exe //c "D:\Projects\Touch\Touch_Client\build.bat"` → Expected `Build OK.`

- [ ] **Step 7: 清理现场**

`Touch_Client/calib/` 下现存两份 `.expired`（Task 2 验证时产生的）已无意义，
且新机制不再改名。删除：

```bash
rm -f Touch_Client/calib/*.expired
```

- [ ] **Step 8: 提交**

> **⚠ 2026-09-19 控制器更正: 不要用 `git add -A Touch_Client/`。** 工作区里有多份**未跟踪的
> 运行期产物**会被它一并扫进去 —— 已实地核对 `git status --porcelain`：
> `Touch_Client/alarms.log`、`Touch_Client/force_demo_log.csv`、`Touch_Client/robot_diagnostics.log`、
> `Touch_Client/calib/`（目录）、`Touch_Client/tests/_build_sa.bat` / `_build_sa.log` /
> `_hello.cpp` / `_rebuild_and_test.bat`。显式列出本任务改过的文件：

```bash
git add Touch_Client/core/CalibStore.h Touch_Client/core/CalibStore.cpp \
        Touch_Client/config/Config.h Touch_Client/main.cpp \
        Touch_Client/force/PayloadCalibration.cpp Touch_Client/force/ForceCalibration.cpp \
        Touch_Client/tests/test_calib_store.cpp Touch_Client/tests/test_payload_calibration.cpp
git commit -m "refactor(calib): drop the 24h expiry gate; CalibStore only owns location"
```

提交后 `git show --stat HEAD` 应恰好是这 8 个文件。

---

### Task 9: 启动零偏漂移检查

> **第二轮设计修订的第三次调整（2026-09-18）。** 用户推理：
> "既然不需要重新计算质心，那是否不需要在启动时重新计算质心等参数，而是作为可选项，
> 供有加装设备时的重新标定" —— **成立**，而且它暴露了原设计把两件事混在一起：
>
> | | 是否随时间漂移 | 失效时机 |
> |---|---|---|
> | **负载（质量 + 质心）** | **不漂** —— 工具的物理属性 | 只在硬件变化时 |
> | **力传感器零偏** | **会漂** —— 温度、时间 | 持续 |
>
> 所以：**负载不做任何启动检查**，按需重标（`'m'` → `'s'`，硬件变了才做）。
> 启动只**便宜地**查**零偏漂移**——那才是会漂的那个。
>
> **为什么单姿态就够**（而这个论证只对零偏成立）：**负载正确时残余力与姿态无关**，
> 所以在任意静止姿态读一次，"测量值 − 存储零偏"就是漂移量本身。
> 它验不了负载（负载错时残余是姿态相关的，恰好停在某个姿态可能读起来接近 0）——
> 但负载本来就不漂，不需要每次验。

**Files:**
- Modify: `Touch_Client/main.cpp`
- Modify: `Touch_Client/config/Config.h`

> **⚠ 2026-09-19 控制器 pre-flight 更正（三条，均已核实）。**
>
> 1. **Step 4 的构建路径是坏的字节，已修。** 原文 `"D:\Projects\Touch\Touch_Clientuild.bat"`
>    里那个 `\b` 被某个工具当转义符吃成了**字面的 0x08 退格字节**（`cat -A` 可见 `^H`）。
>    已改写回 `Touch_Client\build.bat`，并全文扫过：整份计划**只有这一处**控制字节。
>    注意这正是本计划 Global Constraints 里警告过的反斜杠吞字类问题。
> 2. **`filtered` 的语义核过了，本节假设成立。** `force/ForcePipeline.cpp:86` 是
>    `fd.filtered[i] = g_filters[i].step(fd.compensated[i])` —— 即"补偿后 → 低通"，
>    所以"任意静止姿态读 `filtered` 就是零偏漂移量"的论证有效。
>    （`AppState.h:117` 的注释只写"Butterworth 低通滤波输出"，没说清楚它是**补偿后**的 ——
>    别被那行注释误导。）
> 3. **`g_hasStoredZeroCalib` 的置位点**：`main.cpp` 加载 `force_calib.json` 的成功分支，
>    今天是 **`main.cpp:1261`** 的 `if (ForceCalibration::loadFromFile(CalibStore::fileFor("force_calib.json"), ...))`。
>    注意 Task 8 刚把这里从 `resolve()` 改成了 `fileFor()`，别的行号都会漂。

**Interfaces:**
- Consumes: `ForceCompensation` 已算好的 `appState.forceData.filtered`（补偿后读数）、`ForceCalibration::loadFromFile` 存入的零偏
- Produces: 无对外接口（启动期一次性检查）

- [ ] **Step 1: 加阈值常量**

`config/Config.h`，紧跟 `FORCE_RESIDUAL_DEADZONE_N` 之后：

```cpp
    // 启动零偏漂移检查的告警阈值 (N)。
    // 负载正确时残余力与姿态无关, 所以任意静止姿态下 "补偿后读数" 就是零偏漂移量。
    // 取 0.5 N: 明显高于死区 0.20 N 与噪声本底 (~0.05 N), 免得天天误报。
    const double FORCE_ZERO_DRIFT_WARN_N = 0.5;
```

- [ ] **Step 2: 一次性检查**

在 `main.cpp` 里加一个启动期状态（放在 `BiasCheck` 之外，因为它不属于采集模式）：

```cpp
// ===== 启动零偏漂移检查 =====
// 负载参数是工具的物理属性、不随时间漂移, 所以启动【不】验证负载 —— 它按需重标。
// 会漂的是力传感器零偏, 这里便宜地查它:
//   负载正确时残余力与姿态无关, 因此任意静止姿态读一次 "补偿后读数" 就是漂移量。
// 零操作负担: 不摆姿态、不阻断、不写盘。
static bool g_zeroCheckDone = false;
static DWORD g_zeroCheckStartMs = 0;
static double g_zeroCheckAccum[3] = {0, 0, 0};
static int g_zeroCheckCount = 0;

static void runZeroDriftCheck(bool hasStoredZero) {
    if (g_zeroCheckDone || !hasStoredZero) return;
    if (!g_noRobot) {
        DWORD now = GetTickCount();
        if (g_zeroCheckStartMs == 0) { g_zeroCheckStartMs = now; return; }

        AppState::ForceData fd;
        EnterCriticalSection(&appState.forceDataMutex);
        fd = appState.forceData;
        LeaveCriticalSection(&appState.forceDataMutex);

        // 启动后前 2s 让读数稳定, 之后取 1s 均值
        if (fd.isStale || now - g_zeroCheckStartMs < 2000) return;

        for (int i = 0; i < 3; i++) g_zeroCheckAccum[i] += fd.filtered[i];
        g_zeroCheckCount++;
        if (now - g_zeroCheckStartMs < 3000) return;

        // 定稿
        g_zeroCheckDone = true;
        if (g_zeroCheckCount < 10) return;   // 数据太少, 本次不作结论

        const double drift = sqrt(
            (g_zeroCheckAccum[0] / g_zeroCheckCount) * (g_zeroCheckAccum[0] / g_zeroCheckCount) +
            (g_zeroCheckAccum[1] / g_zeroCheckCount) * (g_zeroCheckAccum[1] / g_zeroCheckCount) +
            (g_zeroCheckAccum[2] / g_zeroCheckCount) * (g_zeroCheckAccum[2] / g_zeroCheckCount));

        if (drift > Config::FORCE_ZERO_DRIFT_WARN_N) {
            std::cout << "\n[Force] ⚠ 零偏漂移检查: 补偿后读数 " << drift
                      << " N, 超过阈值 " << Config::FORCE_ZERO_DRIFT_WARN_N << " N" << std::endl;
            std::cout << "[Force]   两种可能:" << std::endl;
            std::cout << "[Force]     · 零偏漂了 (温度/时间)  -> 按 'z' 重新调零" << std::endl;
            std::cout << "[Force]     · 硬件有变化 (加装/拆装) -> 按 'm' 采多姿态后 's' 重标负载"
                      << std::endl;
            std::cout << "[Force]   两种都不影响继续操作, 但建议尽快处理。" << std::endl;
        } else {
            std::cout << "[Force] 零偏漂移检查: 补偿后读数 " << drift
                      << " N, 正常 (< " << Config::FORCE_ZERO_DRIFT_WARN_N << " N)" << std::endl;
        }
    }
}
```

在 `idle()` 的 `if (!appState.isClosing)` 块里、`RelayCore::instance().pollForce()` 之后调用：

```cpp
            runZeroDriftCheck(g_hasStoredZeroCalib);
```

其中 `g_hasStoredZeroCalib` 是在启动加载 `force_calib.json` 时置位的文件级 bool（**新增**，与上面的状态放在一起）。

- [ ] **Step 3: 文案已被 Task 10 的修复接手 —— 本步作废**

原计划此处要修四处"为旧判据写的"文案，替换成 **"两个候选与种子的距离不够悬殊"**。

**但那是种子锚点（Task 7）的失效口径，而 Task 10 把锚点整个删掉了。** 照原文执行等于
用一个错的解释换掉另一个错的：post-Task-10 下 `signOk` 失败的真实原因是
**探针没测出来**（姿态倾角不足 / 两次残余分不开 / 力数据拿不到）。

这几处文案（`main.cpp` 的歧义横幅、`signOk` 的 ✗ 行、连续失败锁死后的提示）
**已由 Task 10 的修复一并改写**，见该修复的 Important 3。本步不再执行。

- [ ] **Step 4: 完整构建**

Run: `cmd.exe //c "D:\Projects\Touch\Touch_Client\build.bat"`
Expected: `Build OK.`

- [ ] **Step 5: 实机验证（交用户，不在本任务内执行）**

启动时应看到一行 `[Force] 零偏漂移检查: 补偿后读数 … N`。
漂移超阈时应看到 ⚠ 三条提示。

- [ ] **Step 6: 提交**

```bash
git add Touch_Client/config/Config.h Touch_Client/main.cpp
git commit -m "feat(force): check the stored zero for drift at startup, without blocking"
```

---

### Task 10: 符号改由「两次下发实测」定案

> **第三轮设计修订（2026-09-18）。** 用户连续两次追问推翻了 Task 7 的锚点方案：
> "理论上通过多个位置与力数据不就可以得到所有的坐标与力参数了吗？"
> "只需要计算力的零偏，为什么还需要种子？"
>
> **对。** 数据解出的是差值 `Δp = p* − p_r`，而 `p_r = m_cfg·(cx,cy,±cz)` 里的 `±`
> 是固件解释、**在差值里被消掉**，所以必须有数据之外的东西补上它。
> 种子是"估计"，而**另一次测量**才是"算出来"——这正是用户从一开始指向的方向。
>
> 而且比预想的便宜：**不需要重新采集一整轮**。同一个静止姿态下依次下发两个候选、
> 各读一次残余力矩即可——机械臂不动，差异纯粹来自符号。
> 信噪比悬殊：符号对时残余 ≈ 噪声本底 **0.003 N·m**，符号错时 `2·|p*_z|·g·sinθ ≈ 0.44 N·m`，**差 100 倍**。

**Files:**
- Modify: `Touch_Client/config/Config.h`
- Modify: `Touch_Client/force/PayloadCalibration.h`
- Modify: `Touch_Client/force/PayloadCalibration.cpp`
- Modify: `Touch_Client/relay/RelayCore.h`
- Modify: `Touch_Client/relay/RelayCore.cpp`
- Modify: `Touch_Client/main.cpp`
- Modify: `Touch_Client/tests/test_payload_calibration.cpp`

**Interfaces:**
- Consumes: `Result::cTrueZ[2]`（Task 3）
- Produces:
  - `Result::comCand[2][3]` —— `{候选+, 候选-}` 的**下发值**，供调用方实测裁决
  - `PayloadCalibration::applyResult(const Result&, int chosen)` —— 新增重载，按选定候选定案
  - `bool RelayCore::probePayloadResidual(double massKg, const double comMm[3], double& residualNm)`
  - **删除** `Config::SIGN_SEED_MARGIN_RATIO`、`PayloadCalibration::seedMarginSufficient()`

- [ ] **Step 1: 配置常量**

`config/Config.h`：删掉 `SIGN_SEED_MARGIN_RATIO`（连注释块），换成：

```cpp
    // ===== CZ 符号实测裁决 =====
    // 符号不再靠估计值猜: 同一个静止姿态下依次下发两个候选, 各读一次残余力矩,
    // 谁的小谁对 (符号对时 ≈ 噪声本底 0.003 N·m; 符号错时 ≈ 0.44 N·m, 差 100 倍)。
    const double SIGN_PROBE_MIN_RATIO    = 3.0;    // 胜者的残余必须比败者小这么多倍
    const double SIGN_PROBE_MAX_WIN_NM   = 0.05;   // 且胜者自身的残余要够小 (N·m)
    const int    SIGN_PROBE_SETTLE_MS    = 600;    // 重新使能后等待控制器采纳 + 读数刷新
    const int    SIGN_PROBE_AVG_MS       = 400;    // 每个候选的残余取样窗口
```

- [ ] **Step 2: `Result` 带出两个候选**

`force/PayloadCalibration.h` 的 `Result` 里，`comMm[3]` 之后加：

```cpp
        // 两个符号候选的【下发值】, 供调用方在实机上各下发一次、实测裁决。
        // solve() 本身不碰机器人, 所以它在 comMm 里放一个临时值,
        // 由调用方选定后再用 applyResult(r, chosen) 定案。
        double comCand[2][3];
```

`Result::signAmbiguous` 的语义改为 **"解算器无法自行定案"**，实测前恒为 `true`。

`solve()` 里把 Task 7 的"锚点 + 余量"选择整段替换为：

```cpp
        // ===== 两个候选都算出来, 不在解算器里选 =====
        // 符号不在数据里 (它只进入 p_cfg 的换算, 在 Δp 的差值里被消掉), 所以解算器
        // 只能把两种解释都给全, 由调用方在实机上各下发一次、看谁留下的力矩残余小。
        double sChosen = signZ;              // 临时值; 由调用方定案
        bool   ambiguous = true;             // 实测前恒为"未定案"

        for (int k = 0; k < 2; k++) {
            const double s = (k == 0) ? 1.0 : -1.0;
            double cEffK[3] = {comCfg[0], comCfg[1], comCfg[2] * s};
            double pCfgK[3] = {mCfg * cEffK[0] / 1000.0, mCfg * cEffK[1] / 1000.0,
                               mCfg * cEffK[2] / 1000.0};
            for (int i = 0; i < 3; i++) {
                double cSend = (pCfgK[i] + dp[i]) / mTrue * 1000.0;
                if (i == 2 && s != 0.0) cSend /= s;
                out.comCand[k][i] = cSend;
            }
        }
```

`out.comMm` / `out.dc` / `out.signZ` / `out.signAmbiguous` 仍按 `sChosen`/`ambiguous` 填
（临时值）；**`comCand` 才是调用方要用的**。

> **删掉** `seedMarginSufficient()` 的声明与定义，以及所有对 `SIGN_SEED_MARGIN_RATIO` 的引用。
> 同时删掉 Task 7 新增的三个锚点用例
> （`test_sign_seed_anchor_picks_plus` / `_minus` / `test_sign_insufficient_margin_is_ambiguous`），
> 换成 Step 3 的用例。

- [ ] **Step 3: 测试改为验证"两个候选确实算对"**

在 `tests/test_payload_calibration.cpp` 加：

```cpp
// 两个候选都必须算对: 一个是真值, 另一个是真值减间距。
// 符号本身由实机实测裁决, 解算器只负责把两种解释都给全。
static void test_both_candidates_computed() {
    TEST(both_candidates_computed);
    double cTrue[3] = {0.3, 0.3, 67.9};
    double cCfg[3]  = {0.0, 0.0, 80.4};
    double F[NP][3], M[NP][3];
    synthesize(0.409, cTrue, 0.660, cCfg, +1.0, F, M);

    PayloadCalibration::Result r;
    CHECK(PayloadCalibration::solve(g_poses, F, M, NP, 0.660, cCfg, +1.0, r));
    // 候选 k=0 (s=+1): cSend_z = cTrue_z = +67.9
    CHECK(fabs(r.comCand[0][2] - 67.9) < 1e-6);
    // 候选 k=1 (s=-1): pCfgK_z = -0.053064, cSend_z = (pCfgK_z + dp_z)/m/(-1) = +191.58
    //
    // ⚠ 初版计划此处写的是 -67.9 —— 【错的】。它把 pCfgK 当成了 +0.053064, 丢了那个负号。
    //   代入: dp_z = -0.0252929, pCfgK_z = 0.660*(-0.0804) = -0.053064
    //         (-0.053064 + -0.0252929)/0.409*1000 = -191.58, 再 /(-1) = +191.58
    //   推导见实施报告; 这条数值由实施者从第一性原理重算后纠正。
    CHECK(fabs(r.comCand[1][2] - 191.581663) < 1e-5);
    // solve() 不再自行定案
    CHECK(r.signAmbiguous);
    PASS();
}
```

在 `main()` 里替换掉被删的三个调用。

- [ ] **Step 4: RelayCore 的实测探针**

`relay/RelayCore.h`（公开区，`applyPayloadToRobot()` 之后）：

```cpp
    // 用给定负载重新使能, 等稳定后返回当前姿态的残余力矩模长 (N·m)。
    // 符号裁决用: 同一静止姿态下依次探两个候选, 谁留下的力矩残余小谁对。
    // 调用方必须保证机械臂【不动】(两次探针在同一姿态比较才有意义)。
    bool probePayloadResidual(double massKg, const double comMm[3], double& residualNm);
```

`relay/RelayCore.cpp`：

```cpp
bool RelayCore::probePayloadResidual(double massKg, const double comMm[3], double& residualNm) {
    if (!isRobotConnected()) return false;

    char cmd[128];
    snprintf(cmd, sizeof(cmd), "EnableRobot(%.3f,%.1f,%.1f,%.1f)",
             massKg, comMm[0], comMm[1], comMm[2]);
    robotSendEnable(cmd);
    Sleep(Config::SIGN_PROBE_SETTLE_MS);

    const DWORD t0 = GetTickCount();
    double sum[3] = {0, 0, 0};
    int n = 0;
    while (GetTickCount() - t0 < (DWORD)Config::SIGN_PROBE_AVG_MS) {
        AppState::ForceData fd;
        EnterCriticalSection(&appState.forceDataMutex);
        fd = appState.forceData;
        LeaveCriticalSection(&appState.forceDataMutex);
        if (!fd.isStale) {
            sum[0] += fd.filtered[3]; sum[1] += fd.filtered[4]; sum[2] += fd.filtered[5];
            n++;
        }
        Sleep(10);
    }
    if (n < 5) return false;                 // 数据太少, 不下结论
    const double m0 = sum[0] / n, m1 = sum[1] / n, m2 = sum[2] / n;
    residualNm = sqrt(m0 * m0 + m1 * m1 + m2 * m2);
    return true;
}
```

- [ ] **Step 5: solveAndApply 里做裁决**

在 `solveAndApply()` 的合理性判据**之前**插入：

```cpp
        // ===== 符号实测裁决 =====
        // solve() 只给出两种解释; 这里在【同一个静止姿态】下各下发一次, 谁留下的
        // 力矩残余小谁对。机械臂全程不动 —— 差异纯粹来自符号, 不是姿态。
        // 前提: 当前姿态要有足够倾角, 否则两者都≈0、分不开。
        double rProbe[2] = {0.0, 0.0};
        int    chosen = -1;
        bool   probeOk = true;
        {
            double p[6];
            EnterCriticalSection(&appState.robotPoseMutex);
            p[0] = appState.robotActualPose.x;  p[1] = appState.robotActualPose.y;
            p[2] = appState.robotActualPose.z;  p[3] = appState.robotActualPose.rx;
            p[4] = appState.robotActualPose.ry; p[5] = appState.robotActualPose.rz;
            LeaveCriticalSection(&appState.robotPoseMutex);
            double R[9];
            TcpCalibration::rpyToMatrix(p[3], p[4], p[5], R);
            const double sinTheta = sqrt(R[2] * R[2] + R[5] * R[5]);
            if (sinTheta < 0.5) {
                std::cout << "  ✗ 当前姿态倾角不足 (sinθ=" << sinTheta
                          << " < 0.5) — 两个符号分不开。" << std::endl;
                std::cout << "    请把笔摆到明显倾斜/水平再按 's'。" << std::endl;
                probeOk = false;
            }
        }
        if (probeOk) {
            for (int k = 0; k < 2; k++) {
                if (!RelayCore::instance().probePayloadResidual(r.massKg, r.comCand[k], rProbe[k])) {
                    std::cout << "  ✗ 符号探针失败 (力数据不足) — 恢复原配置" << std::endl;
                    probeOk = false;
                    break;
                }
            }
        }
        if (probeOk) {
            const int  win  = (rProbe[0] <= rProbe[1]) ? 0 : 1;
            const int  lose = 1 - win;
            const bool separated = (rProbe[lose] >= Config::SIGN_PROBE_MIN_RATIO * rProbe[win]);
            const bool small     = (rProbe[win] <= Config::SIGN_PROBE_MAX_WIN_NM);
            printf("  CZ 符号实测: 候选 +1 残余 %.4f N·m / 候选 -1 残余 %.4f N·m\n",
                   rProbe[0], rProbe[1]);
            if (separated && small) {
                chosen = win;
                std::cout << "  CZ 符号约定: 裁决 → " << (win == 0 ? "+1" : "-1")
                          << " (残余小 " << (rProbe[lose] / rProbe[win]) << " 倍)" << std::endl;
            } else {
                std::cout << "  CZ 符号约定: ✗ 实测分不开 — 结果按不合理处理" << std::endl;
            }
        }
        // ⚠ 这里【不】调用 applyResult —— 定案必须等合理性判据通过之后。
        //    否则判据一旦拒绝, 内存里的生效负载已经变了, 而机器人还停在探针留下的
        //    临时配置 (可能是败者 comCand[1]) 上, 那句"机械臂负载参数保持原值"就是假话。
        if (chosen < 0) {
            // 探针把机器人的配置改乱了, 恢复成当前生效值 (内存没动, 所以就是原值)
            RelayCore::instance().applyPayloadToRobot();
        }
```

`applyResult(const Result&, int chosen)` 是**新增重载**：按 `chosen` 选 `comCand` 填
`comMm` / `dc` / `signZ`，并置 `signAmbiguous = false`；原无参重载保留（供启动加载等用）。
**它只在接受路径上调用**，位置与原有的 `applyResult(r)` 相同（判据通过之后）。

合理性判据里的 `signOk` 改为 `(chosen >= 0)`。

**拒绝路径**（`判定: ✗ 不合理` 那一支）在 `return` 之前**必须加**：

```cpp
            // 探针动过机器人的配置, 拒绝时必须恢复 —— 否则"保持原值"是假话。
            // 内存未改 (applyResult 在判据之后才调), 所以这次下发就是原值。
            RelayCore::instance().applyPayloadToRobot();
```

> ⚠ 插入位置很关键：探针必须在合理性判据之前（判据要用 `chosen`），
> 而**定案必须在判据之后**。这两件事分居判据两侧——初版计划把定案也放在了判据之前，是错的。

- [ ] **Step 6: 跑测试 + 完整构建**

Run: `cmd.exe //c "D:\Projects\Touch\Touch_Client\tests\build_payload_calibration_test.bat"`
然后: `cmd.exe //c "D:\Projects\Touch\Touch_Client\tests\test_payload_calibration.exe"`
Expected: `13 passed, 0 failed`（15 − 3 个锚点用例 + 1 个新用例）

Run: `cmd.exe //c "D:\Projects\Touch\Touch_Client\build.bat"`
Expected: `Build OK.`（**必须完整构建**）

- [ ] **Step 7: 提交**

```bash
git add Touch_Client/config/Config.h Touch_Client/force/PayloadCalibration.h \
        Touch_Client/force/PayloadCalibration.cpp Touch_Client/relay/RelayCore.h \
        Touch_Client/relay/RelayCore.cpp Touch_Client/main.cpp \
        Touch_Client/tests/test_payload_calibration.cpp
git commit -m "feat(force): decide the CZ sign by measuring both candidates on the robot"
```

---

### Task 11: 标定结果落盘（`calib\calib_log.txt`）

> **用户提出（2026-09-18 首次实机测试后）。** 他测完想回看结果，**控制台已经滚掉了**。
>
> 这暴露了一个真实的缺口：整个标定流程**只往控制台打，什么都不留**。这既是操作者的
> 日常问题（想追溯只能靠回忆），也让"这次标定到底做了什么"在事后无从查证。
>
> 这与本分支的主题直接相关：**"让操作者看见标定到底做了什么"**。

> **⚠ 2026-09-19 控制器修订（用户确认）。** 本节原文写于 Task 10 之前，要求落盘
> `probe_plus1_Nm | probe_minus1_Nm | ratio | sign | comZ_sent_mm`。CZ 符号探针已在
> `4751382` 被**整体删除** —— `PayloadCalibration::Result` 里没有 `probe`、没有 `signZ`、
> 没有 `comCand`，`SIGN_PROBE_*` 也不存在了。原文 Step 5 那句"用来标定 `SIGN_PROBE_*`
> 常数"随之作废（该常数已删）。已按**当前模型**改写：列为模型参数全量，出口为**全部出口**。

**Files:**
- Modify: `Touch_Client/main.cpp`

**Interfaces:**
- Consumes: `CalibStore::fileFor()`、`PayloadCalibration::Result`
- Produces: 无对外接口；新增数据文件 `Touch_Client\calib\calib_log.txt`

- [ ] **Step 1: 落盘助手**

在 `main.cpp` 的 `BiasCheck` 命名空间里加（放在 `solveAndApply()` 之前）：

```cpp
    // 每次负载求解尝试都落一行 —— 控制台会滚掉, 而"这次标定到底做了什么"必须能追溯。
    // 一行一次尝试, 便于 grep 与表格工具直接读。
    //
    // 表头只在【文件不存在或为空】时写。不要用进程内 static 标志位: 那个标志每次启动都是
    // false, 会让 fopen 用 "w" 把历次记录整个截断 —— 与"可追溯"的立意在字面上相反。
    //
    // r 可为 nullptr: 求解【之前】就返回的出口 (已上锁 / 姿态数不足 / 正在采样) 没有 Result,
    // 此时各数值列记 "-"。
    static void logCalibAttempt(const char* outcome, const PayloadCalibration::Result* r)
    {
        // fileFor 返回 static 缓冲, 调用方必须立即拷贝 (头文件已注明)。
        char path[512];
        snprintf(path, sizeof(path), "%s", CalibStore::fileFor("calib_log.txt"));

        // 【只用 "a+" 打开, 永不截断。】不要先试 "r" 再决定 "w"/"a": "读不了"(被占用/权限)
        // 与"不存在"会被混为一谈, 而前者会走 "w" 把历次记录整个删掉 —— 毁的正是这个文件
        // 存在的理由。(Task 11 复审 Minor #50)
        FILE* f = fopen(path, "a+");
        if (!f) return;
        bool needHeader = true;
        if (fseek(f, 0, SEEK_END) == 0) needHeader = (ftell(f) == 0);
        if (needHeader) {
            fprintf(f, "# 负载标定尝试记录 (每次按 's' 一行)\n");
            fprintf(f, "# time | poses | rmsF_N | rmsM_Nm | psi_deg | dm_kg | mass_kg"
                       " | comZ_mm | outcome\n");
        }
        char sPoses[16], sRmsF[16], sRmsM[16], sPsi[16], sDm[16], sMass[16], sComZ[16];
        if (r) {
            snprintf(sPoses, sizeof(sPoses), "%d",    r->poses);
            snprintf(sRmsF,  sizeof(sRmsF),  "%.4f",  r->rmsForceN);
            snprintf(sRmsM,  sizeof(sRmsM),  "%.4f",  r->rmsMomentNm);
            snprintf(sPsi,   sizeof(sPsi),   "%+.1f", r->sensorYawDeg);
            snprintf(sDm,    sizeof(sDm),    "%+.4f", r->dm);
            snprintf(sMass,  sizeof(sMass),  "%.4f",  r->massKg);
            snprintf(sComZ,  sizeof(sComZ),  "%+.1f", r->comMm[2]);
        } else {
            char* cols[7] = {sPoses, sRmsF, sRmsM, sPsi, sDm, sMass, sComZ};
            for (int i = 0; i < 7; i++) strcpy(cols[i], "-");
        }
        char ts[24];
        const std::time_t now = std::time(nullptr);
        std::tm tmInfo;
        localtime_s(&tmInfo, &now);          // 与 RobotDiagnostics.cpp / SafetyPredictor.cpp 同一写法
        strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", &tmInfo);

        fprintf(f, "%s | %s | %s | %s | %s | %s | %s | %s | %s\n",
                ts, sPoses, sRmsF, sRmsM, sPsi, sDm, sMass, sComZ, outcome);
        fclose(f);
    }
```

时间戳用 `<ctime>` 的 `std::time` / `localtime_s` / `strftime`，**不要**用 `windows.h` 的
`SYSTEMTIME` / `GetLocalTime` —— 本仓库目前没有任何一处用它们（已 grep 确认），
为一行时间戳新增一个平台头不划算。`localtime_s` 同属 `<ctime>`，且是仓库既有写法
（`safety/RobotDiagnostics.cpp:28`、`safety/SafetyPredictor.cpp:438`）。`main.cpp` 已有
`<cstdio>`；需补 `#include <ctime>` 与 `#include <cstring>`。

- [ ] **Step 2: 在 `solveAndApply()` 的【每条】出口调用它**

| # | 出口 | 调用 | `outcome` |
|---|---|---|---|
| 1 | 已上锁（连续失败 ≥ `Config::CALIB_MAX_CONSECUTIVE_FAILS`） | `logCalibAttempt(outcome, nullptr)` | `REJECTED locked_out` |
| 2 | 姿态数 < 4 | `logCalibAttempt(outcome, nullptr)` | `REJECTED too_few_poses (count=%d)` |
| 3 | 正在采样 | `logCalibAttempt(outcome, nullptr)` | `REJECTED sampling_in_progress` |
| 4 | `solve()` 返回 false | `logCalibAttempt(outcome, nullptr)` | `REJECTED degenerate_or_nonphysical (count=%d)` |
| 5 | 拟合残差 ≥ 0.30 N（拒绝保存） | `logCalibAttempt(outcome, &r)` | `REJECTED rmsF=%.4f`（用 `r.rmsForceN`） |
| 6 | 成功 | `logCalibAttempt(outcome, &r)` | `DISPATCHED`；若 `force_calib.json` 或 `payload_calib.json` 落盘失败 → `DISPATCHED write_failed` |

> 出口 1~4 在求解之前/求解失败，没有可用的 `Result` → 传 `nullptr`，数值列记 `-`，
> 具体信息（如 `count`）写进 `outcome` 文本。
>
> 出口 6 的 `DISPATCHED write_failed`：如果落盘失败却只记 `DISPATCHED`，那"下次启动标定
> 没了"在日志里看起来像没发生过 —— 日志若不能反映这件事，追溯就是假的。该出口的调用点
> 因此要在**两处写入尝试之后**（用一个初值为 true 的 bool 累计两个 `save` 的结果）。
>
> 每条出口都要调用 —— 否则"为什么这批数据没被采纳"又一次只能靠回忆。

- [ ] **Step 3: 验证路径正确**

`CalibStore::fileFor()` 把文件放进 `calib\`（与三份标定文件同目录）。确认目录不存在时
`CalibStore::dir()` 会创建它（已有行为）。

**不要**用相对路径 —— 那正是本分支一开始要修的问题。

- [ ] **Step 4: 完整构建**

Run: `cmd.exe //c "D:\Projects\Touch\Touch_Client\build.bat"`
Expected: `Build OK.`

（Task 1 的教训：独立测试构建的编译环境与完整构建**不同**，"测试通过"不能替代"项目能编"。）

- [ ] **Step 5: 实机验证（交用户）**

按一次 `'s'`，确认 `Touch_Client\calib\calib_log.txt` 出现，且最后一行含
残差 / ψ / `dm` / 定案结果。

- [ ] **Step 6: 提交**

```bash
git add Touch_Client/main.cpp
git commit -m "feat(calib): persist every payload-solve attempt to calib\\calib_log.txt"
```

---

### Task 6: 文档修正与现场清理

> # ⛔ 2026-09-19 控制器警告：本节 Step 1 / Step 3 的**方向是反的**，不要照做。
>
> 本节写于**第二轮设计修订之前**，描述的是一个后来被推翻两次、又整体删除的世界。
> 执行前必须逐条重新定稿（由控制器在派单前完成，不由实施者自行判断）：
>
> - **Step 3 要"补 24h 有效期" —— 而 Task 8 刚刚把 24h 闸门删掉了。**
>   它附带的那整段文案（"有效期 24 小时…文件改名为 `<名字>.expired`…
>   没有 `saved_at_unix` 时间戳的文件一律视为过期"）**现在是假的**。
>   本节要做的不是**加**它，而是**把它从文档里删掉/改写**。
>   —— 这正是 Task 8 复审的 Minor #60：`Docs/调零与负载标定流程.md:46,48` 今天仍在宣称
>   这条已不存在的机制。**Task 8 复审者查过计划，认为"无任务认领"；实际是本节认领了，
>   但方向写反了。**
> - **Step 1 要重写 §3.5 去讲"物理质心 Z 必须为正"的符号判据 —— 该判据已不存在。**
>   它被 Task 7（种子锚点 + 余量）取代，再被 Task 10（两次下发实测）取代，
>   最后连**整个 CZ 符号探针**一起删除（`4751382`）。今天 `signZ` 是**纯显示/持久化约定**：
>   数据定不了它，**没有任何判据依赖它**，求解输出只把两个候选打出来"仅供复核，不影响结果"。
>   重写后的 §3.5 必须反映**这个**状态。
> - **Step 2（删键位表 `'i'` 行）大概率仍然正确** —— Task 5 确实删了人工符号覆盖。仍需对着
>   今天的文档核一遍（文档可能已被 Task 5/7/10 期间的改动修改过）。
> - **Step 4 起**（"结果不合理会怎样"等）也要重新核对：判据今天**只剩拟合残差一条**。
>
> **执行本节前，先跑一遍 `Docs/调零与负载标定流程.md` 与当前 `main.cpp` 实际输出的逐行对照。**

**Files:**
- Modify: `Docs/调零与负载标定流程.md`

- [ ] **Step 1: 重写 §3.5（`'i'` 已彻底移除，整节作废）**

Task 5 删掉了人工符号覆盖。`Docs/调零与负载标定流程.md` 的 §3.5 整节（标题为
`### 3.5 CZ 符号约定不对怎么办（'i'）`，到 `### ---` 之前的下一节为止）**用下面内容整体替换**：

````markdown
### 3.5 CZ 符号约定：由程序判定，不需要人工干预

下发 `centerZ` 时，机械臂内部可能按相反符号解释。**符号必须对**——假设反了，解出的修正量
不是把误差抵消掉而是翻倍。

**判定由程序完成**：`signZ` 只影响"Δp → 绝对质心"的换算，不进入拟合方程，所以两种符号的
**拟合残差完全相同**，数据本身区分不了。可行的是外部判据——两种符号各算一个物理质心候选，
工具挂在法兰下方 → **物理质心 Z 必须为正**，只有一个候选满足。

求解输出会把两个候选都打出来供你复核：

```
  CZ 符号约定: 自动判定 → +1
    · 候选 +1: 物理质心 Z = +67.9 mm  ✓ 法兰下方
    · 候选 -1: 物理质心 Z = -191.6 mm ✗ 法兰上方 (非物理)
```

**程序判不出符号时**（两个候选落在同一侧），该次结果按**不合理**处理并被拒绝——
不会猜一个符号下发。重新采集姿态再试。
````

- [ ] **Step 2: 删掉键位表里的 `'i'` 行**

键位表里删除整行：

```
| `i` | 在 `'m'` 模式下：翻转 CZ 符号约定 | 否 |
```

并把 `'s'` 那行的说明改为（符号已自动判定，不再是人工选择）：

```
| `s` | 在 `'m'` 模式下：**求解负载参数**并下发存盘（符号与合理性由程序自动判定） | 否 |
```

- [ ] **Step 3: 补 24h 有效期与新目录**

在 `Docs/调零与负载标定流程.md` §1 的表格之后加：

```markdown
### 标定文件存放位置与有效期

三个标定文件都放在 **`Touch_Client\calib\`**（由可执行文件位置反推，不随启动目录变化）。
目录不存在时自动创建。

**有效期 24 小时。** 力传感器零偏随温度和时间漂移，负载解算又依赖力数据，
所以超过 24h 的标定一律作废：启动时**不加载**（负载回退 `Config.h` 种子值、零偏清零），
文件改名为 `<名字>.expired` 保留，并在控制台打醒目提示。

> 没有 `saved_at_unix` 时间戳的文件**一律视为过期**——不接受来历不明的旧标定。
> 所以从旧版本升级上来时，原有的 `payload_calib.json` / `force_calib.json` 首次启动即作废，
> 需要重新标定。
```

- [ ] **Step 4: 补一节"结果不合理会怎样"**

在 §3.4 之后加：

````markdown
### 3.5b 程序会判定结果是否合理

每次按 `'s'`，程序不仅给数字，还会**判定这个结果能不能信**。三条判据，任一不满足即判为
**不合理**，此时**拒绝保存、拒绝下发**，机械臂保持原参数：

| 判据 | 含义 |
|------|------|
| 拟合残差 < 0.30 N | 模型解释得了这批数据 |
| 符号可判定 | 两个物理质心候选不在同一侧 |
| 物理质心 Z > 0 | 质心确实在法兰下方 |

**连续 3 次不合理**会打红字错误并停止接受 `'s'`，直到按 `'m'` 重新采集。
这是刻意的——连续失败通常意味着**问题不在求解器**，而在装夹松动、传感器受挤压、
或姿态覆盖不足。继续按 `'s'` 只是空转。

> **注意：质量修正幅度再大也不会被判不合理。** 从种子值出发的第一次标定本来就该大改
> （本项目首次解算就是 0.66 → 0.41 kg）。加装设施后重新标定正是这套流程的用途。
````

- [ ] **Step 5: 清理工作区里的旧标定文件**

这些文件没有时间戳，按新规则一律作废；留在原处只会和新目录混淆：

```bash
rm -f Touch_Client/payload_calib.json Touch_Client/force_calib.json
rm -rf Touch_Client/Touch_Client          # 误建的野目录, 只含 x64/ 构建产物
```

- [ ] **Step 6: 提交**

```bash
git add "Docs/调零与负载标定流程.md"
git commit -m "docs: document the calibration file lifecycle and the result-reasonableness gate"
```

---

## 完成判据

- `test_calib_store` 5/5、`test_payload_calibration` **12/12**（Task 5 删掉了强制符号用例）、`test_tcp_calibration` 7/7 全绿
- `build.bat` 输出 `Build OK.`
- 启动日志出现 `[Calib] !! ... 已作废` 与 `[Payload] 无可用 ... — 用种子值`
- 实机流程第 3 步（紧接着 `'s'` 后按 `'m'`）出现"旧负载"拒绝提示，第 5 步残差显著下降
- 代码里**不再存在** `'i'` 键、`forcedSignZ`、`flipComSignZ()`、`clearForcedSignZ()`、`massJump`
- 三条合理性判据任一不满足 → 拒绝保存和下发；连续 3 次 → 红字错误并锁住 `'s'`

---

## 追加提案（2026-09-19，控制器提出，**待用户定夺后才执行**）

### Task 12（提案）: `applyPayloadToRobot()` 改为「只写文件、下次启动再发机械臂」

> **来源：用户 2026-09-19 的实机观察。** 用户报告：昨天把负载设成 1.5 kg 时
> **"会导致机械臂迅速运动报错"**。也就是说，**改负载这个动作本身能让机械臂突动**。

**现状的风险点：**

`solveAndApply()` 成功路径的末尾会调用 `RelayCore::instance().applyPayloadToRobot()`
→ `enableRobotWithPayload()` → 下发 `EnableRobot(m, cx, cy, cz)`。
这是**运行中**改机械臂的负载，发生的时机是**操作者刚按完 `'s'`、手可能还在设备附近**。

讽刺的是**启动时那条路径反而有防护**（`relay/RelayCore.cpp:411-428`）：
使能前先 `SetCollisionLevel(0)` / `SetSafeSkin(0)` / `LoadSwitch(0)` 全关掉。
运行中这一发**什么防护都没有**。

**为什么它其实不必承担这个风险：**
代码里自己的注释已经写明它**不参与标定生效** ——
"这一发【不是】标定的必要步骤, 也【不是】标定结果生效的途径: 真正生效的是 [本地补偿]…
留着它只是让机械臂侧的显示值与我们对齐"（`relay/RelayCore.cpp:157-161`）。
**为了一个纯显示目的承担突动风险，不划算。**

**提案内容（待定稿）：**
1. `applyPayloadToRobot()` 不再在运行中调用 —— 只保留写盘（`payload_calib.json` 已经写了），
   机械臂侧的值由**下次启动**的 `RelayCore` 连接序列用新值使能时带上。
2. 求解输出里那句 `[参考] 已同步机械臂侧负载显示…` 相应改为
   `[参考] 机械臂侧负载将于下次启动更新`（**不要**再说"已同步"）。
3. 若用户仍想保留运行中同步的能力，则至少要在下发前明确提示操作者松手，
   并考虑复刻启动路径的 `SetCollisionLevel(0)` 等防护。

> **⚠ 与 [[2026-09-18-zeroing-and-payload]] 的结论的关系：** 用户 2026-09-19 重新提出了
> "能不能用 TCP 口 `Payload` 指令设定负载"这个问题 —— 那条结论当时下得偏强
> （把"手工试一次的 `Payload`"与"代码每次使能都发、带回读的 `EnableRobot`"并列成了
> "全无响应"；而 `git log -S'"Payload('` 显示 `Payload(weight,inertia)` **从未在代码里出现过**）。
> 而"1.5 kg 会导致机械臂突动"这条观察，**方向与"改不动"相反** ——
> 一个没被采纳的设定不可能让机械臂动。
> 用户已明确 **1.5 kg 太危险、不要再试**。若将来重开这条线，**必须用零运动测试**：
> 只回读 30004 @1168 Load / @1176 center，不看力、不让机械臂动，且步进要小（如 0.402 → 0.50）。
