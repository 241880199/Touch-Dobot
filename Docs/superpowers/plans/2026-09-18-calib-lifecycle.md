# 标定文件生命周期与 CZ 符号自动判定 — 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让标定文件固定落点、超 24h 自动作废，并让 CZ 符号约定从"人工三步试错"变成"自动判定 + 一键兜底"。

**Architecture:** 新增 `core/CalibStore` 承担标定文件的生命周期（放哪 + 还能不能用），成为唯一的路径与新鲜度判据来源；`PayloadCalibration` 在解算末尾对两种符号各算一个物理质心候选，按"必须在法兰下方"自动选；`BiasCheck` 用一个数据新鲜度标志替代粗暴的 `reset()`，同时满足"复验拦截"和"`'i'` 可重解"。

**Tech Stack:** C++17 / MSVC 2022 BuildTools / Win32 API (`GetModuleFileNameA`、`CreateDirectoryA`) / 无第三方库。

**规格:** `Docs/superpowers/specs/2026-09-18-calib-lifecycle-design.md`

## Global Constraints

- **文件保持 ASCII 或既有中文注释风格**；`.bat` 文件的额外约束见 `Touch_Client/build_and_run.bat` 顶部注释（`chcp 65001` + 非 ASCII 会 desync cmd 解析器）。
- **所有新文件用 LF 还是 CRLF 不强制**，但 `.bat` 必须 CRLF。
- **编译命令固定为：** `cmd.exe /c "D:\Projects\Touch\Touch_Client\build.bat"`，成功标志是输出 `Build OK.`。
- **测试必须真的跑起来**：测试构建脚本只编译，不运行 exe。每步都要单独执行 `tests\<name>.exe`。
- **编译前确认 `Touch_Client.exe` 没有在运行**，否则链接报 `LNK1168`。检查：`tasklist //FI "IMAGENAME eq Touch_Client.exe"`。
- **时间源用 `time(NULL)`（wall clock）**，不用 `GetTickCount`（开机计时，跨重启无效）。
- **24h = 86400 秒**，常量名 `Config::CALIB_MAX_AGE_SEC`。
- **没有 `saved_at_unix` 字段的文件一律视为过期**（明确决策，不做 mtime 回退）。

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
    // 规则: 去掉文件名 + 上溯两级, 再拼 "calib\"。
    //   ...\Touch_Client\x64\Release\Touch_Client.exe → ...\Touch_Client\calib\  (带结尾反斜杠)
    // 分隔符不足两级时返回 false (out 内容未定义)。
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

        // 去掉文件名 + 上溯两级
        for (int up = 0; up < 3; up++) {
            char* p = strrchr(buf, '\\');
            if (!p) return false;
            *p = '\0';
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

> **关于测试覆盖面:** `deriveDir` 与 `isFresh` 是纯函数，已单测。`dir()` / `fileFor()` /
> `resolve()` 要碰真实文件系统（`GetModuleFileNameA` 取的是测试 exe 自己的路径），
> 不做单测——**由 Task 2 Step 9 的实机启动验证**（旧文件被判过期 + 改名 + `calib\` 被创建）。

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

先确认客户端没在跑：
```bash
tasklist //FI "IMAGENAME eq Touch_Client.exe"
```
Run: `cmd.exe /c "D:\Projects\Touch\Touch_Client\build.bat"`
Expected: `Build OK.`

然后启动一次并确认目录被创建、旧文件被判过期：
```bash
cmd.exe /c "D:\Projects\Touch\Touch_Client\x64\Release\Touch_Client.exe" < /dev/null
```
Expected 输出中出现：
- `[Calib] !! payload_calib.json 已作废 — 缺少 saved_at_unix 时间戳`
- `[Calib] !! force_calib.json 已作废 — 缺少 saved_at_unix 时间戳`
- `[Payload] 无可用 payload_calib.json — 用种子值 …`

并确认 `Touch_Client\calib\` 目录已创建：
```bash
ls -la Touch_Client/calib/
```
Expected: 目录存在，内含 `payload_calib.json.expired`、`force_calib.json.expired`。

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

- [ ] **Step 3: solveAndApply 展示符号判定**

在 `solveAndApply()` 里，`printf("  拟合残差: …)` 那一行之后插入符号判定展示（位置在 `// ===== 合理性评估 =====` 之前）：

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

启动客户端，按顺序验证：

1. `'m'` 进模式 → 摆 3~4 个姿态按 `SPACE` → `'m'` 退出
   Expected: 正常输出报告，**不**出现"旧负载"拒绝提示（还没求解过）
2. `'m'` 再进 → 摆 ≥4 个姿态 → `'s'`
   Expected: 求解结果里出现
   ```
     CZ 符号约定: 自动判定 → +1        (或 -1)
       · 候选 +1: 物理质心 Z = ... mm  ✓ 法兰下方
       · 候选 -1: 物理质心 Z = ... mm  ✗ 法兰上方 (非物理)
   ```
   以及"旧数据已作废"提示
3. 紧接着按 `'m'` 退出
   Expected: **出现**"这批数据是在【旧负载】下采的, 不能用来复验当前参数" —— 这正是修复的目标（以前会假报 FAIL）
4. `'m'` 进 → 摆 3~4 个姿态按 `SPACE`
   Expected: 第一次 SPACE 时提示"上一批数据是在旧负载下采的, 已丢弃 — 开始新一批采集"
5. `'m'` 退出
   Expected: 报告基于新数据；`|ΔF|` 应显著小于求解前的 4.0 N
6. 若第 5 步残差反而变大：按 `'i'`
   Expected: 提示"强制 CZ 符号约定 → … 用同一批数据重解"，且**不需要**重新摆姿态就重新出结果

- [ ] **Step 9: 提交**

```bash
git add Touch_Client/main.cpp
git commit -m "feat(force): show the CZ sign decision and gate verification on fresh data"
```

---

### Task 5: 文档修正与现场清理

**Files:**
- Modify: `Docs/调零与负载标定流程.md`

- [ ] **Step 1: 修正"翻符号后旧数据作废"的错误说法**

在 `Docs/调零与负载标定流程.md` §3.5 里，把

```
符号反了就按 `'i'` 翻转约定，然后**重新 `'m'` 采集**再 `'s'` 求解（旧数据是在旧配置下采的，翻符号后作废）。符号对了，一次求解即收敛。
```

改为：

```
符号反了就按 `'i'`。**不需要重新采集** —— `signZ` 只影响解算结果的换算，不进入拟合方程，
那批数据反映的是机械臂当时的实际配置，换个符号解释依然有效。按 `'i'` 会直接用同一批数据重解。

判据由程序自动给出（见 §3.2 的输出）：两种符号各算一个物理质心候选，
工具挂在法兰下方 → 物理质心 Z 必须为正，只有一个候选满足。
两个候选都落在同一侧时程序会标"无法判定"并沿用当前约定，那时才需要 `'i'` 人工指定。
```

- [ ] **Step 2: 补 24h 有效期与新目录**

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

- [ ] **Step 3: 修正 `'i'` 的键位表说明**

在键位表里把 `'i'` 那一行改为：

```
| `i` | 在 `'m'` 模式下：**强制换一个 CZ 符号**并用同一批数据重解（自动判定失败时才需要） | 否 |
```

- [ ] **Step 4: 清理工作区里的旧标定文件**

这些文件没有时间戳，按新规则一律作废；留在原处只会和新目录混淆：

```bash
rm -f Touch_Client/payload_calib.json Touch_Client/force_calib.json
rm -rf Touch_Client/Touch_Client          # 误建的野目录, 只含 x64/ 构建产物
```

- [ ] **Step 5: 提交**

```bash
git add "Docs/调零与负载标定流程.md"
git commit -m "docs: correct the CZ sign guidance and document the 24h calibration expiry"
```

---

## 完成判据

- `test_calib_store` 4/4、`test_payload_calibration` 13/13、`test_tcp_calibration` 7/7 全绿
- `build.bat` 输出 `Build OK.`
- 启动日志出现 `[Calib] !! ... 已作废` 与 `[Payload] 无可用 ... — 用种子值`
- 实机流程第 3 步（紧接着 `'s'` 后按 `'m'`）出现"旧负载"拒绝提示，第 5 步残差显著下降
- `'i'` 无需重新采集即可重解
