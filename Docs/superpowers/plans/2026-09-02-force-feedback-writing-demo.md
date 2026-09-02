# 力反馈书写对照演示（软件侧）实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 为"机械臂末端软笔书写 'NJU'、有力反馈 vs 无力反馈对照实验"构建全部**设备无关的软件支持**——力反馈开关（MATLAB 界面控制）、力数据落盘、笔压稳定性分析、笔尖 TCP 偏移标定框架。

**Architecture:** 新增一条 MATLAB → C++ 反向命令通道（复用 :8888 TCP 连接，`FF|0`/`FF|1` 单行协议），C++ 侧用非阻塞 `select()` 在每帧轮询该连接并切换 `AppState::forceFeedbackEnabled`，`HapticCallback` 据此对触觉力整体门控（关闭时力归零、位置跟随保持）。力数据由 `pollForce()` 持续落盘 CSV（含 `ff_enabled` 标志列），事后由 MATLAB `force_analysis.m` 按该标志分组计算笔压稳定性指标。笔尖 TCP 偏移（法兰→笔尖）由新增 `TcpCalibration` 模块用 N 点最小二乘求解，结果存 `tcp_calib.json`。

**Tech Stack:** C++17（MSVC cl.exe / vcvarsall x64）、WinSock2、OpenHaptics HD API、MATLAB（tcpserver / uigridlayout / readtable）、独立 TDD 测试（test_*.exe + run_tests.bat）。

## Global Constraints

- C++17、Windows-only（依赖 `winsock2.h`、`CRITICAL_SECTION`），MSVC `cl /EHsc /std:c++17`。
- 协议约定：**行式**，`TYPE|payload`，以 `\n` 结尾（`\r` 会被剥除）。
- 配置单一数据源：`system_config.json` → `Config.h`（C++ 编译时常量）/ `relay_config.m`（MATLAB 运行时）。共享参数改 JSON，非共享参数才进 `Config.h`。
- 测试模式：独立 `test_*.cpp` → `test_*.exe`，`TEST/CHECK/PASS` 宏 + `g_passed/g_failed` 计数，`main` 返回 `g_failed > 0 ? 1 : 0`（0=通过）。构建脚本 `build_*_test.bat`，`run_tests.bat` 统一运行。
- 提交信息风格：`feat(scope): ...` / `test(scope): ...`。
- 力反馈开关**只**通过 MATLAB 界面触发，**不**设键盘快捷键（防误触，spec §4.1）。
- 本计划覆盖**全部设备无关的软件侧**（含 TCP 偏移标定框架 Task 7/8）。笔夹 OpenSCAD 建模（spec §3.2）与全部**实机调试**（TCP 实机标定执行、力反馈验证、A/B 演示）写入独立计划 `2026-09-02-force-feedback-writing-demo-hardware.md`。

---

## File Structure

| 文件 | 责任 | 本计划动作 |
|------|------|-----------|
| `Touch_Client/core/AppState.h` | 全局状态（触觉设备/位姿/力数据） | 修改：新增 `forceFeedbackEnabled` 原子标志 |
| `Touch_Client/haptic/HapticCallback.cpp` | 1kHz 触觉渲染回调 | 修改：力反馈整体门控 |
| `Touch_Client/relay/RelayCommandParser.h` / `.cpp` | 解析 MATLAB 反向命令（纯函数） | 新建 |
| `Touch_Client/relay/RelayCore.h` / `.cpp` | 与 MATLAB 中继站通信 | 修改：`pollRelayCommands()` + 命令分发 |
| `Touch_Client/force/ForceLogger.h` / `.cpp` | 力数据 CSV 落盘 | 新建 |
| `Touch_Client/config/Config.h` | 编译时常量 | 修改：新增 `FORCE_LOG_PATH` |
| `Touch_Client/main.cpp` | GLUT 主循环 | 修改：`idle()` 轮询反向命令 + TCP 标定按键 + 启动加载 |
| `Touch_Client/calibration/TcpCalibration.h` / `.cpp` | TCP 偏移求解 + 标定状态 + JSON 读写 | 新建 |
| `Touch_Client/tests/test_relay_command_parser.cpp` | 命令解析测试 | 新建 |
| `Touch_Client/tests/test_force_logger.cpp` | CSV 格式化测试 | 新建 |
| `Touch_Client/tests/build_relay_command_test.bat` | 命令解析测试构建脚本 | 新建 |
| `Touch_Client/tests/build_force_logger_test.bat` | CSV 格式化测试构建脚本 | 新建 |
| `Touch_Client/tests/test_tcp_calibration.cpp` | TCP 求解器测试 | 新建 |
| `Touch_Client/tests/build_tcp_calibration_test.bat` | TCP 求解器测试构建脚本 | 新建 |
| `Relay_Station/relay_gui.m` | MATLAB 可视化主入口 | 修改：力反馈开关 UI + 反向发送 |
| `Relay_Station/force_analysis.m` | 笔压稳定性分析脚本 | 新建 |

---

## Task 1: 力反馈开关状态 + HapticCallback 门控

**Files:**
- Modify: `Touch_Client/core/AppState.h`（在 `// ===== 力数据 =====` 小节之前插入标志）
- Modify: `Touch_Client/haptic/HapticCallback.cpp:146`

**Interfaces:**
- Consumes: 无（本计划第一个任务）。
- Produces: `AppState::forceFeedbackEnabled`（`std::atomic<bool>`，默认 `true`）。Task 3 写入，`HapticCallback` 读取。

**说明:** 本任务是单行门控，无独立单元测试（实际力输出需硬件）。验证方式为编译通过 + 逻辑审查；实机验证延后。

- [ ] **Step 1: 在 AppState.h 新增开关标志**

在 `AppState.h` 中，紧接 `// ===== 力数据 =====` 注释行**之前**插入：

```cpp
    // ===== 力反馈开关 (MATLAB relay_gui 反向命令控制) =====
    // true=渲染力反馈, false=力归零(位置跟随保持)。
    // 由 RelayCore::pollRelayCommands 写入 (主线程), haptic 回调线程读取 (1kHz)。
    std::atomic<bool> forceFeedbackEnabled{ true };
```

`<atomic>` 已在文件顶部 `#include <atomic>`（第 6 行），无需新增 include。

- [ ] **Step 2: 在 HapticCallback.cpp 对力反馈整体门控**

将 `HapticCallback.cpp` 第 146 行：

```cpp
        if (button1 || button2) {
```

改为：

```cpp
        if (app.forceFeedbackEnabled && (button1 || button2)) {
```

这是唯一改动：关闭时 `totalForce` 保持 `{0,0,0}`，第 204 行 `hdSetDoublev(HD_CURRENT_FORCE, totalForce)` 仍被调用（输出零力），位置跟随不受影响。此门控覆盖传感器力（8a）、虚拟约束力（8b）、姿态斥力（8c/8d），实现 spec §4.1"力归零、位置跟随保持"。

- [ ] **Step 3: 编译验证**

Run: `cd Touch_Client && build.bat`
Expected: 编译通过，无新增告警/错误。

- [ ] **Step 4: Commit**

```bash
git add Touch_Client/core/AppState.h Touch_Client/haptic/HapticCallback.cpp
git commit -m "feat(force-feedback): add enable flag gating haptic rendering"
```

---

## Task 2: RelayCommandParser（MATLAB 反向命令解析，TDD）

**Files:**
- Create: `Touch_Client/relay/RelayCommandParser.h`
- Create: `Touch_Client/relay/RelayCommandParser.cpp`
- Test: `Touch_Client/tests/test_relay_command_parser.cpp`
- Create: `Touch_Client/tests/build_relay_command_test.bat`

**Interfaces:**
- Consumes: 无。
- Produces: `RelayCommandParser::parse(const char* line) -> RelayCommandParser::Command`，其中 `enum class Command { None, ForceFeedbackOn, ForceFeedbackOff }`。Task 3 依赖此签名。

**协议:** 单行 `FF|0`（关）或 `FF|1`（开），允许前后空白与尾部 `\r\n`。未知/非法输入返回 `Command::None`。

- [ ] **Step 1: 写失败测试**

创建 `Touch_Client/tests/test_relay_command_parser.cpp`：

```cpp
// Standalone test: RelayCommandParser — MATLAB 反向命令解析
// Build: build_relay_command_test.bat
// Run: test_relay_command_parser.exe

#include <iostream>
#include "../relay/RelayCommandParser.h"

static int g_passed = 0, g_failed = 0;

#define TEST(name) do { std::cout << "  " << #name << "... "; } while(0)
#define PASS() do { std::cout << "PASS" << std::endl; g_passed++; } while(0)
#define CHECK(cond) do { if (!(cond)) { std::cout << "FAIL: " << #cond << std::endl; g_failed++; return; } } while(0)

using R = RelayCommandParser::Command;

static void test_ff_on() {
    TEST(ff_on);
    CHECK(RelayCommandParser::parse("FF|1") == R::ForceFeedbackOn);
    PASS();
}

static void test_ff_off() {
    TEST(ff_off);
    CHECK(RelayCommandParser::parse("FF|0") == R::ForceFeedbackOff);
    PASS();
}

static void test_ff_on_with_newline() {
    TEST(ff_on_with_newline);
    CHECK(RelayCommandParser::parse("FF|1\n") == R::ForceFeedbackOn);
    PASS();
}

static void test_ff_on_with_crlf() {
    TEST(ff_on_with_crlf);
    CHECK(RelayCommandParser::parse("FF|1\r\n") == R::ForceFeedbackOn);
    PASS();
}

static void test_ff_leading_whitespace() {
    TEST(ff_leading_whitespace);
    CHECK(RelayCommandParser::parse("  FF|0") == R::ForceFeedbackOff);
    PASS();
}

static void test_ff_invalid_digit() {
    TEST(ff_invalid_digit);
    CHECK(RelayCommandParser::parse("FF|2") == R::None);
    PASS();
}

static void test_ff_trailing_garbage() {
    TEST(ff_trailing_garbage);
    CHECK(RelayCommandParser::parse("FF|10") == R::None);
    PASS();
}

static void test_ff_missing_value() {
    TEST(ff_missing_value);
    CHECK(RelayCommandParser::parse("FF|") == R::None);
    PASS();
}

static void test_empty() {
    TEST(empty);
    CHECK(RelayCommandParser::parse("") == R::None);
    PASS();
}

static void test_null() {
    TEST(null);
    CHECK(RelayCommandParser::parse(nullptr) == R::None);
    PASS();
}

static void test_unknown_command() {
    TEST(unknown_command);
    CHECK(RelayCommandParser::parse("P|1,2,3") == R::None);
    PASS();
}

int main() {
    std::cout << "=== RelayCommandParser Tests ===" << std::endl;
    test_ff_on();
    test_ff_off();
    test_ff_on_with_newline();
    test_ff_on_with_crlf();
    test_ff_leading_whitespace();
    test_ff_invalid_digit();
    test_ff_trailing_garbage();
    test_ff_missing_value();
    test_empty();
    test_null();
    test_unknown_command();
    std::cout << std::endl;
    std::cout << g_passed << " passed, " << g_failed << " failed" << std::endl;
    return g_failed > 0 ? 1 : 0;
}
```

- [ ] **Step 2: 跑测试确认失败**

创建 `Touch_Client/tests/build_relay_command_test.bat`：

```bat
@echo on
call "D:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvarsall.bat" x64
cd /d "D:\Projects\Touch\Touch_Client\tests"
cl /EHsc /std:c++17 /D_CRT_SECURE_NO_WARNINGS /D_WINSOCK_DEPRECATED_NO_WARNINGS test_relay_command_parser.cpp ..\relay\RelayCommandParser.cpp /Fe:test_relay_command_parser.exe
echo BUILD_EXIT=%ERRORLEVEL%
```

Run: `cd Touch_Client/tests && build_relay_command_test.bat`
Expected: 编译失败（`RelayCommandParser.h` 不存在，`fatal error C1083`）。

- [ ] **Step 3: 写最小实现**

创建 `Touch_Client/relay/RelayCommandParser.h`：

```cpp
#pragma once

// MATLAB → C++ 反向命令解析 (单行, 以 '|' 分隔)
// 协议: "FF|0" 关力反馈, "FF|1" 开力反馈
namespace RelayCommandParser {
    enum class Command { None, ForceFeedbackOn, ForceFeedbackOff };

    // 解析一行命令。返回 Command::None 表示未知/空/非法输入。
    Command parse(const char* line);
}
```

创建 `Touch_Client/relay/RelayCommandParser.cpp`：

```cpp
#include "RelayCommandParser.h"
#include <cstring>

namespace RelayCommandParser {
    Command parse(const char* line) {
        if (line == nullptr) return Command::None;

        // 跳过前导空白
        while (*line == ' ' || *line == '\t' || *line == '\r' || *line == '\n') line++;
        if (*line == '\0') return Command::None;

        if (strncmp(line, "FF|", 3) != 0) return Command::None;

        const char* p = line + 3;
        while (*p == ' ' || *p == '\t') p++;
        if (*p != '0' && *p != '1') return Command::None;

        char val = *p++;
        // 值后只允许空白或行尾
        while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;
        if (*p != '\0') return Command::None;

        return (val == '1') ? Command::ForceFeedbackOn : Command::ForceFeedbackOff;
    }
}
```

- [ ] **Step 4: 跑测试确认通过**

Run: `cd Touch_Client/tests && build_relay_command_test.bat && test_relay_command_parser.exe`
Expected: `11 passed, 0 failed`，退出码 0。

- [ ] **Step 5: Commit**

```bash
git add Touch_Client/relay/RelayCommandParser.h Touch_Client/relay/RelayCommandParser.cpp \
        Touch_Client/tests/test_relay_command_parser.cpp Touch_Client/tests/build_relay_command_test.bat
git commit -m "feat(relay): add MATLAB reverse-command parser (FF| protocol)"
```

---

## Task 3: RelayCore 接收反向命令（非阻塞轮询 + 分发）

**Files:**
- Modify: `Touch_Client/relay/RelayCore.h`（公开方法 + 私有缓冲区/分发声明）
- Modify: `Touch_Client/relay/RelayCore.cpp`（实现 `pollRelayCommands` + `dispatchRelayCommand`，顶部 include）
- Modify: `Touch_Client/main.cpp:51`（`idle()` 中调用）

**Interfaces:**
- Consumes: `RelayCommandParser::parse`（Task 2）；`AppState::forceFeedbackEnabled`（Task 1）。
- Produces: `RelayCore::pollRelayCommands()`（无参无返回值，供 `main.cpp` 每帧调用）。

**说明:** 复用 `initRelayReporting()` 建立的 `m_relaySocket`（`127.0.0.1:8888`）。非阻塞 `select()` + `recv()`，逐行切分（`\n` 结束，剥 `\r`）。不新增线程。

- [ ] **Step 1: RelayCore.h 增加声明**

在 `RelayCore.h` 的 `// MATLAB GUI 上报` 小节（`sendConnectionHealth()` 之后、`reportDiagnostic(...)` 之前）加入：

```cpp
    // MATLAB → C++ 反向命令 (每帧调用, 非阻塞)
    void pollRelayCommands();
```

在私有区（`CRITICAL_SECTION m_relaySocketMutex;` 之后）加入：

```cpp
    // 反向命令接收缓冲 (行式协议, 逐行切分)
    char m_relayRecvBuf[256];
    int  m_relayRecvLen = 0;
    void dispatchRelayCommand(const char* line);
```

- [ ] **Step 2: RelayCore.cpp 实现**

在 `RelayCore.cpp` 顶部（`#include "FeedbackParser.h"` 等附近）加入：

```cpp
#include "RelayCommandParser.h"
```

在 `reportFeedback` 实现之后（或 `sendRelayUpdate` 附近）加入：

```cpp
void RelayCore::dispatchRelayCommand(const char* line) {
    using R = RelayCommandParser::Command;
    switch (RelayCommandParser::parse(line)) {
    case R::ForceFeedbackOn:
        appState.forceFeedbackEnabled = true;
        std::cout << "[Relay] Force feedback ENABLED (MATLAB command)" << std::endl;
        break;
    case R::ForceFeedbackOff:
        appState.forceFeedbackEnabled = false;
        std::cout << "[Relay] Force feedback DISABLED (MATLAB command)" << std::endl;
        break;
    case R::None:
    default:
        break;
    }
}

void RelayCore::pollRelayCommands() {
    EnterCriticalSection(&m_relaySocketMutex);
    SOCKET sock = m_relaySocket;
    LeaveCriticalSection(&m_relaySocketMutex);
    if (sock == INVALID_SOCKET) return;

    // 非阻塞检查可读数据
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(sock, &readfds);
    timeval tv = { 0, 0 };
    if (select(0, &readfds, nullptr, nullptr, &tv) <= 0) return;

    char tmp[256];
    int n = recv(sock, tmp, sizeof(tmp), 0);
    if (n <= 0) return;

    // 追加到接收缓冲, 逐行分发
    for (int i = 0; i < n; ++i) {
        char c = tmp[i];
        if (c == '\n') {
            m_relayRecvBuf[m_relayRecvLen] = '\0';
            dispatchRelayCommand(m_relayRecvBuf);
            m_relayRecvLen = 0;
        } else if (c != '\r' && m_relayRecvLen < (int)sizeof(m_relayRecvBuf) - 1) {
            m_relayRecvBuf[m_relayRecvLen++] = c;
        }
    }
}
```

> 注：`appState` 是 `extern` 全局（`AppState.h` 已声明），`RelayCore.cpp` 已在其它函数（如 `pollForce`）中使用 `appState`，无需新增声明。

- [ ] **Step 3: main.cpp 每帧调用**

在 `main.cpp` `idle()` 中，第 51 行 `glutPostRedisplay();` 之后、`if (_kbhit())` 之前，加入（**无条件**，因为 `initRelayReporting()` 在 `--no-robot` 模式下同样执行）：

```cpp
        // MATLAB → C++ 反向命令轮询 (力反馈开关等)
        RelayCore::instance().pollRelayCommands();
```

- [ ] **Step 4: 编译验证**

Run: `cd Touch_Client && build.bat`
Expected: 编译通过。

- [ ] **Step 5: Commit**

```bash
git add Touch_Client/relay/RelayCore.h Touch_Client/relay/RelayCore.cpp Touch_Client/main.cpp
git commit -m "feat(relay): poll MATLAB reverse commands to toggle force feedback"
```

---

## Task 4: MATLAB relay_gui 力反馈开关 UI

**Files:**
- Modify: `Relay_Station/relay_gui.m`（状态字段 + UI 控件 + 回调 + 反向发送）

**Interfaces:**
- Consumes: Task 3 的 C++ 侧接收（`FF|0`/`FF|1`）。
- Produces: MATLAB 侧 `S.ff_enabled` 状态 + 反向发送函数 `sendToClient(cmd)`。

**说明:** 开关放在"Force Output (Filtered -> Touch)"面板（`pnlFF`）标题下方，语义贴合（力反馈输出面板）。用 `uiswitch` 双态控件。反向发送用 `write(S.server, uint8(...), 'uint8')`。

- [ ] **Step 1: 新增状态字段**

在 `relay_gui.m` 的 `S` 结构体初始化区（`S.server = [];` 之后）加入：

```matlab
    S.ff_enabled = true;      % 力反馈开关状态 (A组=true / B组=false)
```

- [ ] **Step 2: 改造 pnlFF 面板为三行并插入开关**

将现有（约第 150-154 行）：

```matlab
    pnlFF = uigridlayout(glMid, [2 1]);
    pnlFF.RowHeight = {22, '1x'};
    pnlFF.Padding = [4 0 4 2];  pnlFF.RowSpacing = 0;
    pnlFF.BackgroundColor = clr.bg_panel;
    pnlFF.Layout.Row = 3;  pnlFF.Layout.Column = 1;
```

改为：

```matlab
    pnlFF = uigridlayout(glMid, [3 1]);
    pnlFF.RowHeight = {22, 26, '1x'};
    pnlFF.Padding = [4 0 4 2];  pnlFF.RowSpacing = 0;
    pnlFF.BackgroundColor = clr.bg_panel;
    pnlFF.Layout.Row = 3;  pnlFF.Layout.Column = 1;
```

在 `lblFFTitle`（其 `Layout.Row` 保持 1）与 `lblForceFilt` 之间，插入开关行：

```matlab
    % 力反馈开关 (A/B 对照实验)
    pnlFFToggle = uigridlayout(pnlFF, [1 2]);
    pnlFFToggle.ColumnWidth = {'1x', 60};
    pnlFFToggle.Padding = [0 0 0 0];  pnlFFToggle.RowSpacing = 0;  pnlFFToggle.ColumnSpacing = 4;
    pnlFFToggle.BackgroundColor = clr.bg_panel;
    pnlFFToggle.Layout.Row = 2;  pnlFFToggle.Layout.Column = 1;

    lblFFToggle = uilabel(pnlFFToggle, 'Text', 'Force Feedback (A/B switch)', ...
        'FontColor', clr.text_dim, 'FontSize', 9);
    lblFFToggle.Layout.Row = 1;  lblFFToggle.Layout.Column = 1;

    swFF = uiswitch(pnlFFToggle, 'Items', {'OFF', 'ON'}, 'Value', 'ON', ...
        'ValueChangedFcn', @(~,~) onForceFeedbackToggle(swFF.Value));
    swFF.Layout.Row = 1;  swFF.Layout.Column = 2;
```

将 `lblForceFilt` 的行号从 2 改为 3（原 `lblForceFilt.Layout.Row = 2;` → `= 3;`）：

```matlab
    lblForceFilt.Layout.Row = 3;  lblForceFilt.Layout.Column = 1;
```

- [ ] **Step 3: 新增回调与发送函数**

在嵌套函数区（如 `onServerConnection` 之后）加入：

```matlab
    function onForceFeedbackToggle(newVal)
        S.ff_enabled = strcmp(newVal, 'ON');
        if S.ff_enabled
            sendToClient('FF|1');
        else
            sendToClient('FF|0');
        end
    end

    function sendToClient(cmd)
        if ~isempty(S.server) && isvalid(S.server) && S.server.Connected
            try
                write(S.server, uint8([cmd newline]), 'uint8');
            catch e
                fprintf('[Relay] ERROR sending to client: %s\n', e.message);
            end
        end
    end
```

- [ ] **Step 4: 运行验证**

Run: `cd Relay_Station && matlab -r "relay_gui"`
Expected: 界面 "Force Output" 面板出现 `Force Feedback (A/B switch)` 开关，默认 `ON`。切换开关时命令行打印 `[Relay] ...`（若 C++ 客户端已连接并运行，其控制台打印 `Force feedback ENABLED/DISABLED`）。

- [ ] **Step 5: Commit**

```bash
git add Relay_Station/relay_gui.m
git commit -m "feat(relay): add MATLAB force-feedback toggle UI + reverse send"
```

---

## Task 5: 力数据落盘（ForceLogger CSV，TDD）

**Files:**
- Create: `Touch_Client/force/ForceLogger.h`
- Create: `Touch_Client/force/ForceLogger.cpp`
- Test: `Touch_Client/tests/test_force_logger.cpp`
- Create: `Touch_Client/tests/build_force_logger_test.bat`
- Modify: `Touch_Client/config/Config.h`（新增 `FORCE_LOG_PATH`）
- Modify: `Touch_Client/relay/RelayCore.cpp`（`initForceReader` 打开、`pollForce` 写、`shutdownForceReader` 关闭）

**Interfaces:**
- Consumes: `AppState::forceData.filtered[6]`（`pollForce` 内已有）、`AppState::forceFeedbackEnabled`（Task 1）、`robotActualPose`（`pollForce` 顶部已有局部 `pose[6]`）。
- Produces: `ForceLogger::open/close/log/formatLine`，供 `RelayCore` 调用；CSV 表头供 Task 6 的 `force_analysis.m` 读取。

**CSV 表头（固定顺序）:**
`t_ms,Fx,Fy,Fz,Mx,My,Mz,pose_x,pose_y,pose_z,pose_rx,pose_ry,pose_rz,ff_enabled`

**覆盖 spec §4.2 与 §4.3:** `Fx..Mz` 列 = 力数据落盘（§4.2），`pose_x..pose_rz` 列 = 笔尖轨迹持续记录（§4.3，每帧 `pollForce` 都会写一行，即轨迹采样）。两者合并进同一 CSV，避免两套文件。

- [ ] **Step 1: 写失败测试**

创建 `Touch_Client/tests/test_force_logger.cpp`：

```cpp
// Standalone test: ForceLogger — CSV 行格式化 (纯函数)
// Build: build_force_logger_test.bat
// Run: test_force_logger.exe

#include <iostream>
#include <cstring>
#include "../force/ForceLogger.h"

static int g_passed = 0, g_failed = 0;

#define TEST(name) do { std::cout << "  " << #name << "... "; } while(0)
#define PASS() do { std::cout << "PASS" << std::endl; g_passed++; } while(0)
#define CHECK(cond) do { if (!(cond)) { std::cout << "FAIL: " << #cond << std::endl; g_failed++; return; } } while(0)

static void test_format_basic() {
    TEST(format_basic);
    char buf[256];
    double f[6] = {1.0, -2.0, 3.5, 0.1, 0.2, 0.3};
    double p[6] = {100.0, 200.0, 300.0, 1.0, 2.0, 3.0};
    int n = ForceLogger::formatLine(buf, sizeof(buf), 1234, f, p, 1);
    CHECK(n > 0);
    CHECK(strcmp(buf, "1234,1.000,-2.000,3.500,0.100,0.200,0.300,100.000,200.000,300.000,1.000,2.000,3.000,1") == 0);
    PASS();
}

static void test_format_ff_disabled() {
    TEST(format_ff_disabled);
    char buf[256];
    double f[6] = {0,0,0,0,0,0};
    double p[6] = {0,0,0,0,0,0};
    ForceLogger::formatLine(buf, sizeof(buf), 0, f, p, 0);
    // 最后一个字段 (ff_enabled) 为 0
    CHECK(strstr(buf, ",0") != nullptr);
    CHECK(buf[strlen(buf)-1] == '0');
    PASS();
}

static void test_format_negative() {
    TEST(format_negative);
    char buf[256];
    double f[6] = {-5.5, -6.25, -7.125, -0.5, -0.75, -1.0};
    double p[6] = {-1.0, -2.0, -3.0, -4.0, -5.0, -6.0};
    ForceLogger::formatLine(buf, sizeof(buf), 999, f, p, 1);
    CHECK(strstr(buf, "-5.500,-6.250,-7.125") != nullptr);
    PASS();
}

static void test_format_truncation_returns_needed() {
    TEST(format_truncation_returns_needed);
    char small[16];
    double f[6] = {0,0,0,0,0,0};
    double p[6] = {0,0,0,0,0,0};
    int needed = ForceLogger::formatLine(small, sizeof(small), 123456789, f, p, 1);
    CHECK(needed >= (int)sizeof(small));  // 返回所需总长度 >= 缓冲, 说明会截断
    PASS();
}

int main() {
    std::cout << "=== ForceLogger Tests ===" << std::endl;
    test_format_basic();
    test_format_ff_disabled();
    test_format_negative();
    test_format_truncation_returns_needed();
    std::cout << std::endl;
    std::cout << g_passed << " passed, " << g_failed << " failed" << std::endl;
    return g_failed > 0 ? 1 : 0;
}
```

- [ ] **Step 2: 跑测试确认失败**

创建 `Touch_Client/tests/build_force_logger_test.bat`：

```bat
@echo on
call "D:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvarsall.bat" x64
cd /d "D:\Projects\Touch\Touch_Client\tests"
cl /EHsc /std:c++17 /D_CRT_SECURE_NO_WARNINGS /D_WINSOCK_DEPRECATED_NO_WARNINGS test_force_logger.cpp ..\force\ForceLogger.cpp /Fe:test_force_logger.exe
echo BUILD_EXIT=%ERRORLEVEL%
```

Run: `cd Touch_Client/tests && build_force_logger_test.bat`
Expected: 编译失败（`ForceLogger.h` 不存在）。

- [ ] **Step 3: 写最小实现**

创建 `Touch_Client/force/ForceLogger.h`：

```cpp
#pragma once
#include <cstddef>

// 力数据 CSV 落盘 (演示对照实验用)
// 始终记录, 每行含 ff_enabled 标志列, 事后按该标志分组分析。
namespace ForceLogger {
    // 打开 CSV 文件并写入表头。返回 false 表示失败。
    bool open(const char* path);

    // 关闭文件 (幂等)。
    void close();

    // 将一条采样格式化为 CSV 行 (纯函数, 便于单元测试)。
    // 返回写入字符数 (不含结尾 '\0'); 若 >= len 表示被截断。
    int formatLine(char* buf, size_t len,
                   unsigned long tMs,          // 相对启动时间戳 (ms)
                   const double force[6],      // 补偿+滤波后的 6 轴力/力矩 (N, Nm)
                   const double pose[6],       // 机器人末端位姿 x,y,z,rx,ry,rz
                   int ffEnabled);             // 1=力反馈开, 0=关

    // 写一条采样 (文件未打开时忽略)。
    void log(unsigned long tMs, const double force[6], const double pose[6], int ffEnabled);
}
```

创建 `Touch_Client/force/ForceLogger.cpp`：

```cpp
#include "ForceLogger.h"
#include <cstdio>

namespace {
    FILE* g_file = nullptr;
}

namespace ForceLogger {
    bool open(const char* path) {
        close();
        FILE* f = fopen(path, "w");
        if (!f) return false;
        g_file = f;
        fprintf(f, "t_ms,Fx,Fy,Fz,Mx,My,Mz,pose_x,pose_y,pose_z,pose_rx,pose_ry,pose_rz,ff_enabled\n");
        fflush(f);
        return true;
    }

    void close() {
        if (g_file) { fclose(g_file); g_file = nullptr; }
    }

    int formatLine(char* buf, size_t len,
                   unsigned long tMs, const double force[6],
                   const double pose[6], int ffEnabled) {
        return snprintf(buf, len,
            "%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d",
            tMs,
            force[0], force[1], force[2], force[3], force[4], force[5],
            pose[0], pose[1], pose[2], pose[3], pose[4], pose[5],
            ffEnabled);
    }

    void log(unsigned long tMs, const double force[6], const double pose[6], int ffEnabled) {
        if (!g_file) return;
        char buf[256];
        formatLine(buf, sizeof(buf), tMs, force, pose, ffEnabled);
        fprintf(g_file, "%s\n", buf);
    }
}
```

> 注：`snprintf` 需要 `<cstdio>`；MSVC 在 `_CRT_SECURE_NO_WARNINGS` 下直接可用（构建脚本已定义）。

- [ ] **Step 4: 跑测试确认通过**

Run: `cd Touch_Client/tests && build_force_logger_test.bat && test_force_logger.exe`
Expected: `4 passed, 0 failed`，退出码 0。

- [ ] **Step 5: Config.h 新增日志路径**

在 `Config.h` 的 `// ========== 诊断日志参数 ==========` 小节（`DIAGNOSTIC_LOG_PATH` 之后）加入：

```cpp
    constexpr const char* FORCE_LOG_PATH = "force_demo_log.csv"; // 力反馈演示落盘路径
```

- [ ] **Step 6: RelayCore 集成**

在 `RelayCore.cpp` 顶部加入：

```cpp
#include "../force/ForceLogger.h"
```

在 `initForceReader()` 中，`ForceCalibration::setDragModeCallback(calibDragMode);` 之后加入：

```cpp
    if (!ForceLogger::open(Config::FORCE_LOG_PATH)) {
        std::cerr << "[Force] Failed to open force log " << Config::FORCE_LOG_PATH << std::endl;
    }
```

在 `pollForce()` 中，第 1524 行 `LeaveCriticalSection(&app.forceDataMutex);` **之前**加入（此时 `pose` 局部数组、`app.forceData.filtered`、`now` 均在作用域内）：

```cpp
    // 落盘到 CSV (演示对照实验用, 含 ff_enabled 标志列)
    ForceLogger::log(now, app.forceData.filtered, pose,
                     appState.forceFeedbackEnabled ? 1 : 0);
```

在 `shutdownForceReader()` 中，`ForcePipeline::shutdown();` 之前加入：

```cpp
    ForceLogger::close();
```

- [ ] **Step 7: 编译验证**

Run: `cd Touch_Client && build.bat`
Expected: 编译通过。

- [ ] **Step 8: Commit**

```bash
git add Touch_Client/force/ForceLogger.h Touch_Client/force/ForceLogger.cpp \
        Touch_Client/tests/test_force_logger.cpp Touch_Client/tests/build_force_logger_test.bat \
        Touch_Client/config/Config.h Touch_Client/relay/RelayCore.cpp
git commit -m "feat(force): log compensated force + pose to CSV for A/B demo"
```

---

## Task 6: 笔压稳定性分析脚本（MATLAB）

**Files:**
- Create: `Relay_Station/force_analysis.m`

**Interfaces:**
- Consumes: Task 5 产出的 CSV（表头 `t_ms,...,Fz,...,ff_enabled`）。
- Produces: 指标表（`Fz_rms_N` / `Fz_var_N2` / `contact_segments` / `duration_s`）+ 两组并排 Fz 曲线对比图。

**说明:** 默认以 `Fz` 作为法向力（笔压）近似。若实机标定后法向力不是传感器 Z 轴，需在此脚本顶部调整 `AXIS`（预留说明）。接触阈值 `CONTACT_N` 为软垫稳定接触任务的落笔判定，实机可调。这是分析脚本，无单元测试，靠运行验证。

- [ ] **Step 1: 写分析脚本**

创建 `Relay_Station/force_analysis.m`：

```matlab
function force_analysis(csv_path)
%FORCE_ANALYSIS  力反馈 A/B 对照实验笔压稳定性分析
%  读取 C++ 端落盘的 CSV, 按 ff_enabled 分组, 计算笔压稳定性指标并绘图。
%
%  输入: csv_path — CSV 文件路径 (默认 'force_demo_log.csv')
%
%  指标 (每组): Fz RMS / 方差 / 接触段数(断线) / 书写时长
%  输出: 两组并排 Fz 曲线对比图 + 指标表
%
%  说明: 默认以 Fz 作为法向力(笔压)近似。若实机标定后法向力非传感器 Z 轴,
%        修改下方 AXIS 变量为对应列名 (如 'Fx'/'Fy')。

    if nargin < 1 || isempty(csv_path)
        csv_path = 'force_demo_log.csv';
    end
    if ~isfile(csv_path)
        error('force_analysis: 未找到文件 %s', csv_path);
    end

    % ===== 可调参数 =====
    AXIS      = 'Fz';   % 法向力(笔压)对应的列名
    CONTACT_N = 0.5;    % 接触阈值 (N) — 软垫稳定接触任务落笔判定, 实机可调
    % ====================

    % 读取 CSV (首行表头)
    T = readtable(csv_path);

    % 检查必需列
    required = {'t_ms', AXIS, 'ff_enabled'};
    for i = 1:numel(required)
        if ~any(strcmp(T.Properties.VariableNames, required{i}))
            error('force_analysis: 缺少列 %s', required{i});
        end
    end

    Fz = T.(AXIS);            % 法向力 (笔压)
    t  = T.t_ms / 1000;       % ms -> s
    ff = T.ff_enabled;        % 0=关, 1=开

    % 分组: 力反馈 ON 在前 (A 组), OFF 在后 (B 组)
    groups = sort(unique(ff), 'descend');

    results = table();
    fig = figure('Name', 'Force Feedback A/B 对比', 'Position', [100 100 1200 480]);

    for g = groups'
        idx = (ff == g);
        fg = Fz(idx); tg = t(idx);
        if isempty(fg), continue; end

        inContact = fg > CONTACT_N;
        % 接触段: 差分找上升沿 (抬笔 -> 落笔)
        edges = diff([false; inContact]);
        nContact = numel(find(edges == 1));

        % 接触期间的法向力统计
        fContact = fg(inContact);
        if isempty(fContact)
            rmsV = 0; varV = 0;
        else
            rmsV = sqrt(mean(fContact.^2));
            varV = var(fContact);
        end

        dur = max(tg) - min(tg);

        results = [results; table(g, rmsV, varV, nContact, dur, ...
            'VariableNames', {'ff_enabled','Fz_rms_N','Fz_var_N2','contact_segments','duration_s'})];

        % 绘图
        subplot(1, 2, find(groups == g));
        plot(tg, fg, 'LineWidth', 1.0); hold on;
        yline(CONTACT_N, 'r--', 'LineWidth', 1);
        xlabel('t (s)'); ylabel(sprintf('%s (N)', AXIS));
        if g == 1
            title('A 组: 力反馈 ON');
        else
            title('B 组: 力反馈 OFF');
        end
        grid on; hold off;
    end

    % 输出指标表
    fprintf('\n===== 力反馈 A/B 对照结果 =====\n');
    disp(results);
    fprintf('================================\n\n');
end
```

- [ ] **Step 2: 运行验证**

Run（有 CSV 时）：`cd Relay_Station && matlab -r "force_analysis('force_demo_log.csv')"`
Expected: 弹窗显示两组 Fz 曲线对比图，命令窗口打印指标表（含 `Fz_rms_N`、`Fz_var_N2`、`contact_segments`、`duration_s`）。
（无 CSV 时脚本应报错 `force_analysis: 未找到文件 ...`，属预期。）

- [ ] **Step 3: Commit**

```bash
git add Relay_Station/force_analysis.m
git commit -m "feat(analysis): add pen-pressure stability A/B analysis script"
```

---

## Task 7: TCP 偏移求解器 + 笔尖变换（纯函数，TDD）

**Files:**
- Create: `Touch_Client/calibration/TcpCalibration.h`
- Create: `Touch_Client/calibration/TcpCalibration.cpp`
- Test: `Touch_Client/tests/test_tcp_calibration.cpp`
- Create: `Touch_Client/tests/build_tcp_calibration_test.bat`

**Interfaces:**
- Consumes: 无（纯函数模块）。
- Produces: `TcpCalibration::rpyToMatrix(rx,ry,rz,R[9])`、`TcpCalibration::solve(poses[][6],n,offsetOut[3],rmsOut)`、`TcpCalibration::apply(pose[6],toolOffset[3],tipOut[3])`。Task 8 依赖 `solve`/`apply` 与状态字段。

**说明:** 求解法兰中心 → 笔尖的固定偏移（spec §4.5 的软件核心）。方法：笔尖对准固定参考点，机械臂多姿态记录法兰位姿（`GetPose` 的 x,y,z,rx,ry,rz），最小二乘求解 `p_i + R_i·t = 常数`。RPY→旋转矩阵约定 `R = Rz·Ry·Rx`（与 `test_self_collision.cpp`、FK 一致）。纯数学、可 TDD；实机采集与验证在独立硬件计划。

- [ ] **Step 1: 写失败测试**

创建 `Touch_Client/tests/test_tcp_calibration.cpp`：

```cpp
#define _USE_MATH_DEFINES
// Standalone test: TcpCalibration — RPY→矩阵 / 应用偏移 / N 点最小二乘求解
// Build: build_tcp_calibration_test.bat
// Run: test_tcp_calibration.exe

#include <iostream>
#include <cmath>
#include "../calibration/TcpCalibration.h"

static int g_passed = 0, g_failed = 0;

#define TEST(name) do { std::cout << "  " << #name << "... "; } while(0)
#define PASS() do { std::cout << "PASS" << std::endl; g_passed++; } while(0)
#define CHECK(cond) do { if (!(cond)) { std::cout << "FAIL: " << #cond << std::endl; g_failed++; return; } } while(0)

static void test_rpy_identity() {
    TEST(rpy_identity);
    double R[9];
    TcpCalibration::rpyToMatrix(0, 0, 0, R);
    CHECK(fabs(R[0]-1)<1e-9 && fabs(R[4]-1)<1e-9 && fabs(R[8]-1)<1e-9);
    CHECK(fabs(R[1])<1e-9 && fabs(R[3])<1e-9 && fabs(R[6])<1e-9);
    PASS();
}

static void test_rpy_rz90() {
    TEST(rpy_rz90);
    double R[9];
    TcpCalibration::rpyToMatrix(0, 0, M_PI/2, R);
    // Rz(90°): [[0,-1,0],[1,0,0],[0,0,1]]
    CHECK(fabs(R[0]-0)<1e-9 && fabs(R[1]+1)<1e-9);
    CHECK(fabs(R[3]-1)<1e-9 && fabs(R[4]-0)<1e-9);
    CHECK(fabs(R[8]-1)<1e-9);
    PASS();
}

static void test_apply_zero_offset() {
    TEST(apply_zero_offset);
    double pose[6] = {10, 20, 30, 0.1, 0.2, 0.3};
    double off[3] = {0,0,0};
    double tip[3];
    TcpCalibration::apply(pose, off, tip);
    CHECK(tip[0]==10 && tip[1]==20 && tip[2]==30);
    PASS();
}

static void test_apply_rotated_offset() {
    TEST(apply_rotated_offset);
    double pose[6] = {0,0,0, 0,0, M_PI/2};  // Rz 90°
    double off[3] = {1,0,0};
    double tip[3];
    TcpCalibration::apply(pose, off, tip);
    CHECK(fabs(tip[0]-0)<1e-9 && fabs(tip[1]-1)<1e-9 && fabs(tip[2]-0)<1e-9);
    PASS();
}

static void test_solve_recovers_offset() {
    TEST(solve_recovers_offset);
    double tTrue[3] = {5.0, -8.0, -150.0};
    double pTip[3] = {300.0, 100.0, 40.0};
    double rpy[4][3] = {
        {0.1, 0.2, 0.3},
        {-0.15, 0.05, 0.5},
        {0.25, -0.1, -0.2},
        {0.0, 0.3, -0.4}
    };
    double poses[4][6];
    for (int k = 0; k < 4; k++) {
        double R[9];
        TcpCalibration::rpyToMatrix(rpy[k][0], rpy[k][1], rpy[k][2], R);
        poses[k][0] = pTip[0] - (R[0]*tTrue[0]+R[1]*tTrue[1]+R[2]*tTrue[2]);
        poses[k][1] = pTip[1] - (R[3]*tTrue[0]+R[4]*tTrue[1]+R[5]*tTrue[2]);
        poses[k][2] = pTip[2] - (R[6]*tTrue[0]+R[7]*tTrue[1]+R[8]*tTrue[2]);
        poses[k][3] = rpy[k][0]; poses[k][4] = rpy[k][1]; poses[k][5] = rpy[k][2];
    }
    double off[3]; double rms;
    CHECK(TcpCalibration::solve(poses, 4, off, rms));
    CHECK(fabs(off[0]-tTrue[0]) < 1e-6);
    CHECK(fabs(off[1]-tTrue[1]) < 1e-6);
    CHECK(fabs(off[2]-tTrue[2]) < 1e-6);
    CHECK(rms < 1e-6);
    PASS();
}

static void test_solve_insufficient() {
    TEST(solve_insufficient);
    double poses[2][6] = {{0}};
    double off[3]; double rms;
    CHECK(!TcpCalibration::solve(poses, 2, off, rms));
    PASS();
}

static void test_solve_degenerate_same_orientation() {
    TEST(solve_degenerate_same_orientation);
    // 相同姿态 (R 全部相同) → dR=0 → 退化
    double poses[3][6] = {
        {0,0,0, 0,0,0},
        {10,0,0, 0,0,0},
        {20,0,0, 0,0,0}
    };
    double off[3]; double rms;
    CHECK(!TcpCalibration::solve(poses, 3, off, rms));
    PASS();
}

int main() {
    std::cout << "=== TcpCalibration Tests ===" << std::endl;
    test_rpy_identity();
    test_rpy_rz90();
    test_apply_zero_offset();
    test_apply_rotated_offset();
    test_solve_recovers_offset();
    test_solve_insufficient();
    test_solve_degenerate_same_orientation();
    std::cout << std::endl;
    std::cout << g_passed << " passed, " << g_failed << " failed" << std::endl;
    return g_failed > 0 ? 1 : 0;
}
```

- [ ] **Step 2: 跑测试确认失败**

创建 `Touch_Client/tests/build_tcp_calibration_test.bat`：

```bat
@echo on
call "D:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvarsall.bat" x64
cd /d "D:\Projects\Touch\Touch_Client\tests"
cl /EHsc /std:c++17 /D_CRT_SECURE_NO_WARNINGS /D_WINSOCK_DEPRECATED_NO_WARNINGS test_tcp_calibration.cpp ..\calibration\TcpCalibration.cpp /Fe:test_tcp_calibration.exe
echo BUILD_EXIT=%ERRORLEVEL%
```

Run: `cd Touch_Client/tests && build_tcp_calibration_test.bat`
Expected: 编译失败（`TcpCalibration.h` 不存在）。

- [ ] **Step 3: 写最小实现**

创建 `Touch_Client/calibration/TcpCalibration.h`：

```cpp
#pragma once

// 笔尖 TCP 偏移标定: 求解法兰中心 → 笔尖的固定偏移 (spec §4.5)。
// 方法: 笔尖对准固定参考点, 机械臂多姿态记录法兰位姿 (GetPose),
//      最小二乘求解偏移 t 使 p_i + R_i·t = 常数 (笔尖固定点)。
// 约定: RPY → 旋转矩阵采用 R = Rz(rz) * Ry(ry) * Rx(rx) (与 FK/姿态控制一致)。
namespace TcpCalibration {
    // RPY(弧度) → 3×3 旋转矩阵 (row-major)
    void rpyToMatrix(double rx, double ry, double rz, double R[9]);

    // 纯函数: 求解 TCP 偏移。
    // poses: n 个法兰位姿 [x,y,z,rx,ry,rz], n >= 3
    // offsetOut: 输出偏移 (法兰系, mm); rmsOut: 拟合残差 (mm)
    // 返回 false 表示退化 (姿态不足/共线/秩亏)
    bool solve(const double poses[][6], int n, double offsetOut[3], double& rmsOut);

    // 纯函数: 应用 TCP 偏移, 求笔尖世界坐标。
    // pose: 法兰位姿 [x,y,z,rx,ry,rz]; toolOffset: 法兰系偏移; tipOut: 输出笔尖坐标
    void apply(const double pose[6], const double toolOffset[3], double tipOut[3]);
}
```

创建 `Touch_Client/calibration/TcpCalibration.cpp`：

```cpp
#include "TcpCalibration.h"
#include <cmath>

namespace TcpCalibration {
    void rpyToMatrix(double rx, double ry, double rz, double R[9]) {
        double crx = cos(rx), srx = sin(rx);
        double cry = cos(ry), sry = sin(ry);
        double crz = cos(rz), srz = sin(rz);
        // R = Rz * Ry * Rx (row-major)
        R[0] = crz*cry;
        R[1] = crz*sry*srx - srz*crx;
        R[2] = crz*sry*crx + srz*srx;
        R[3] = srz*cry;
        R[4] = srz*sry*srx + crz*crx;
        R[5] = srz*sry*crx - crz*srx;
        R[6] = -sry;
        R[7] = cry*srx;
        R[8] = cry*crx;
    }

    void apply(const double pose[6], const double toolOffset[3], double tipOut[3]) {
        double R[9];
        rpyToMatrix(pose[3], pose[4], pose[5], R);
        tipOut[0] = pose[0] + R[0]*toolOffset[0] + R[1]*toolOffset[1] + R[2]*toolOffset[2];
        tipOut[1] = pose[1] + R[3]*toolOffset[0] + R[4]*toolOffset[1] + R[5]*toolOffset[2];
        tipOut[2] = pose[2] + R[6]*toolOffset[0] + R[7]*toolOffset[1] + R[8]*toolOffset[2];
    }

    bool solve(const double poses[][6], int n, double offsetOut[3], double& rmsOut) {
        if (n < 3) return false;

        double R0[9];
        rpyToMatrix(poses[0][3], poses[0][4], poses[0][5], R0);

        double AtA[9] = {0};
        double Atb[3] = {0};

        for (int k = 1; k < n; k++) {
            double Rk[9];
            rpyToMatrix(poses[k][3], poses[k][4], poses[k][5], Rk);
            double dR[9];
            for (int i = 0; i < 9; i++) dR[i] = Rk[i] - R0[i];
            double dp[3] = {
                poses[0][0] - poses[k][0],
                poses[0][1] - poses[k][1],
                poses[0][2] - poses[k][2]
            };
            // A^T A += dR^T dR ; A^T b += dR^T dp
            for (int r = 0; r < 3; r++) {
                for (int c = 0; c < 3; c++) {
                    double s = 0;
                    for (int i = 0; i < 3; i++) s += dR[i*3 + r] * dR[i*3 + c];
                    AtA[r*3 + c] += s;
                }
                double s = 0;
                for (int i = 0; i < 3; i++) s += dR[i*3 + r] * dp[i];
                Atb[r] += s;
            }
        }

        // 3×3 Gauss 消元 (部分主元)
        double A[3][3] = {
            {AtA[0], AtA[1], AtA[2]},
            {AtA[3], AtA[4], AtA[5]},
            {AtA[6], AtA[7], AtA[8]}
        };
        double b[3] = {Atb[0], Atb[1], Atb[2]};

        for (int col = 0; col < 3; col++) {
            int piv = col;
            for (int r = col + 1; r < 3; r++)
                if (fabs(A[r][col]) > fabs(A[piv][col])) piv = r;
            if (fabs(A[piv][col]) < 1e-12) return false;
            if (piv != col) {
                for (int c = 0; c < 3; c++) { double t = A[col][c]; A[col][c] = A[piv][c]; A[piv][c] = t; }
                double t = b[col]; b[col] = b[piv]; b[piv] = t;
            }
            double d = A[col][col];
            for (int c = col; c < 3; c++) A[col][c] /= d;
            b[col] /= d;
            for (int r = 0; r < 3; r++) {
                if (r == col) continue;
                double f = A[r][col];
                for (int c = col; c < 3; c++) A[r][c] -= f * A[col][c];
                b[r] -= f * b[col];
            }
        }

        offsetOut[0] = b[0];
        offsetOut[1] = b[1];
        offsetOut[2] = b[2];

        double sumSq = 0;
        for (int k = 1; k < n; k++) {
            double Rk[9];
            rpyToMatrix(poses[k][3], poses[k][4], poses[k][5], Rk);
            for (int i = 0; i < 3; i++) {
                double rv = 0;
                for (int c = 0; c < 3; c++) rv += (Rk[i*3 + c] - R0[i*3 + c]) * offsetOut[c];
                double e = rv - (poses[0][i] - poses[k][i]);
                sumSq += e * e;
            }
        }
        rmsOut = sqrt(sumSq / (n - 1));
        return true;
    }
}
```

- [ ] **Step 4: 跑测试确认通过**

Run: `cd Touch_Client/tests && build_tcp_calibration_test.bat && test_tcp_calibration.exe`
Expected: `7 passed, 0 failed`，退出码 0。

- [ ] **Step 5: Commit**

```bash
git add Touch_Client/calibration/TcpCalibration.h Touch_Client/calibration/TcpCalibration.cpp \
        Touch_Client/tests/test_tcp_calibration.cpp Touch_Client/tests/build_tcp_calibration_test.bat
git commit -m "feat(calibration): add TCP offset solver + pen-tip transform (N-point LSQ)"
```

---

## Task 8: TCP 标定状态 + 采集框架 + 键盘接线

**Files:**
- Modify: `Touch_Client/calibration/TcpCalibration.h`（追加状态/IO 声明）
- Modify: `Touch_Client/calibration/TcpCalibration.cpp`（追加状态全局 + `load/save/startCollect/cancelCollect`）
- Modify: `Touch_Client/main.cpp`（`#include` + `'t'`/SPACE/`'s'` 按键 + 启动加载）

**Interfaces:**
- Consumes: `TcpCalibration::solve`/`apply`（Task 7）、`appState.robotActualPose`（GetPose 位姿）。
- Produces: `TcpCalibration::enabled/offset[3]/rmsError/collectMode/collectCount/collectPose[][]` 与 `load/save/startCollect/cancelCollect`。文件 `tcp_calib.json`（实机计划运行时加载/验证）。

**说明:** 复用现有 `Calibration`/`CalibrationIO.cpp` 的采集+JSON 模式。按键 `'t'`（现未被占用）。与坐标标定 `Calibration::collectMode` 互斥（进入 TCP 采集时取消坐标采集）。

- [ ] **Step 1: 扩展 TcpCalibration.h**

在 `TcpCalibration.h` 的 `apply` 声明之后、`}` 之前，追加：

```cpp
    // ===== 标定状态 (存 tcp_calib.json) =====
    extern bool enabled;
    extern double offset[3];
    extern double rmsError;

    // ===== 采集状态 (笔尖对准固定点, 多姿态记录) =====
    extern bool collectMode;
    extern int  collectCount;
    static const int MAX_COLLECT_POSES = 50;
    extern double collectPose[MAX_COLLECT_POSES][6];

    bool load(const char* filepath);
    bool save(const char* filepath);
    void startCollect();
    void cancelCollect();
```

- [ ] **Step 2: 扩展 TcpCalibration.cpp**

在 `TcpCalibration.cpp` 顶部 `#include <cmath>` 之后追加 `#include <cstdio>`、`#include <cstring>`、`#include <cstdlib>`。

在 `namespace TcpCalibration {` 开头（`rpyToMatrix` 之前）追加状态全局：

```cpp
    // ===== 全局标定状态 =====
    bool enabled = false;
    double offset[3] = {0, 0, 0};
    double rmsError = 0.0;

    bool collectMode = false;
    int  collectCount = 0;
    double collectPose[MAX_COLLECT_POSES][6] = {{0}};
```

在 `namespace` 末尾（`solve` 实现之后、`}` 之前）追加 IO 与采集函数：

```cpp
    bool load(const char* filepath) {
        FILE* f = fopen(filepath, "r");
        if (!f) return false;
        char buf[1024];
        size_t n = fread(buf, 1, sizeof(buf) - 1, f);
        fclose(f);
        if (n == 0) return false;
        buf[n] = '\0';

        const char* p = strstr(buf, "\"offset\"");
        if (!p) return false;
        p = strchr(p, '[');
        if (!p) return false;
        p++;
        for (int i = 0; i < 3; i++) {
            char* end = nullptr;
            offset[i] = strtod(p, &end);
            if (end == p) return false;
            p = end;
            while (*p == ',' || *p == ' ' || *p == '\n' || *p == '\r' || *p == '\t') p++;
        }
        p = strstr(buf, "\"rmsError\"");
        if (p) {
            p = strchr(p, ':');
            if (p) rmsError = strtod(p + 1, nullptr);
        }
        enabled = true;
        return true;
    }

    bool save(const char* filepath) {
        FILE* f = fopen(filepath, "w");
        if (!f) return false;
        fprintf(f, "{\n");
        fprintf(f, "  \"offset\": [%.6g, %.6g, %.6g],\n", offset[0], offset[1], offset[2]);
        fprintf(f, "  \"rmsError\": %.6g\n", rmsError);
        fprintf(f, "}\n");
        fclose(f);
        return true;
    }

    void startCollect() {
        collectMode = true;
        collectCount = 0;
    }

    void cancelCollect() {
        collectMode = false;
        collectCount = 0;
    }
```

- [ ] **Step 3: main.cpp 增加 include 与按键处理**

在 `main.cpp` 顶部其它 include 附近（如 `#include "calibration/CalibrationSolver.h"` 之后）加入：

```cpp
#include "calibration/TcpCalibration.h"
```

在 `'c'/'C'` 处理块（约第 177-202 行，以 `return; }` 结束）之后，加入 `'t'/'T'` 切换：

```cpp
    // ===== TCP 偏移标定 ('t' key) =====
    // 't': 切换 TCP 标定采集模式 (笔尖对准固定点, 多姿态记录法兰位姿)
    if (key == 't' || key == 'T') {
        if (TcpCalibration::collectMode) {
            TcpCalibration::cancelCollect();
            std::cout << "\n[TCP-CALIB] Mode OFF" << std::endl;
        } else {
            if (Calibration::collectMode) Calibration::cancelCollect(); // 与坐标标定互斥
            TcpCalibration::startCollect();
            std::cout << "\n[TCP-CALIB] Mode ON — "
                      << "Keep pen TIP at a fixed point, reorient arm, "
                      << "press SPACE to record a pose, 's' to solve, 't' to exit" << std::endl;
        }
        return;
    }
```

在 SPACE 记录坐标标定点对块（约第 212-246 行，以 `return; }` 结束）之后，加入 TCP 位姿记录：

```cpp
    // Space during TCP calibration: 记录法兰位姿 (笔尖对准固定点)
    if (key == ' ' && TcpCalibration::collectMode) {
        int idx = TcpCalibration::collectCount;
        if (idx >= TcpCalibration::MAX_COLLECT_POSES) {
            std::cout << "[TCP-CALIB] Max " << TcpCalibration::MAX_COLLECT_POSES << " poses reached" << std::endl;
            return;
        }
        EnterCriticalSection(&appState.robotPoseMutex);
        double p[6] = {
            appState.robotActualPose.x, appState.robotActualPose.y, appState.robotActualPose.z,
            appState.robotActualPose.rx, appState.robotActualPose.ry, appState.robotActualPose.rz
        };
        LeaveCriticalSection(&appState.robotPoseMutex);

        for (int i = 0; i < 6; i++) TcpCalibration::collectPose[idx][i] = p[i];
        TcpCalibration::collectCount++;
        std::cout << "[TCP-CALIB] Pose " << TcpCalibration::collectCount << " recorded: ("
                  << p[0] << "," << p[1] << "," << p[2] << "," << p[3] << "," << p[4] << "," << p[5] << ")"
                  << std::endl;
        return;
    }
```

在 `'s'/'S'` 坐标标定求解块（约第 249-295 行，以 `return; }` 结束）之后，加入 TCP 求解：

```cpp
    // 's' during TCP calibration: 求解 TCP 偏移并保存
    if ((key == 's' || key == 'S') && TcpCalibration::collectMode) {
        if (TcpCalibration::collectCount < 3) {
            std::cout << "[TCP-CALIB] Need at least 3 poses, have "
                      << TcpCalibration::collectCount << std::endl;
            return;
        }
        double off[3]; double rms;
        if (!TcpCalibration::solve(TcpCalibration::collectPose, TcpCalibration::collectCount, off, rms)) {
            std::cout << "[TCP-CALIB] Solver failed — keep tip fixed, vary orientation" << std::endl;
            return;
        }
        for (int i = 0; i < 3; i++) TcpCalibration::offset[i] = off[i];
        TcpCalibration::rmsError = rms;
        TcpCalibration::enabled = true;
        TcpCalibration::save("tcp_calib.json");
        std::cout << "\n[TCP-CALIB] Solved! TCP offset = [" << off[0] << ", " << off[1] << ", " << off[2]
                  << "] mm, RMS = " << rms << " mm" << std::endl;
        std::cout << "[TCP-CALIB] Saved to tcp_calib.json" << std::endl;
        TcpCalibration::cancelCollect();
        return;
    }
```

- [ ] **Step 4: 启动加载 tcp_calib.json**

在 `main.cpp` 的 `if (Calibration::load("calibration.json")) { ... } else { ... }` 块（约第 536-542 行）之后加入：

```cpp
    // 6.6 加载 TCP 偏移标定 (如存在)
    if (TcpCalibration::load("tcp_calib.json")) {
        std::cout << "[TCP] Loaded tcp_calib.json (offset="
                  << TcpCalibration::offset[0] << "," << TcpCalibration::offset[1] << "," << TcpCalibration::offset[2]
                  << "mm, RMS=" << TcpCalibration::rmsError << "mm)" << std::endl;
    } else {
        std::cout << "[TCP] No tcp_calib.json — press 't' to calibrate TCP offset" << std::endl;
    }
```

- [ ] **Step 5: 编译验证**

Run: `cd Touch_Client && build.bat`
Expected: 编译通过（`TcpCalibration.cpp` 需加入项目构建——若 `build.bat` 显式列出源文件，需将 `calibration\TcpCalibration.cpp` 加入源文件清单；若为通配/目录扫描则自动包含，以实际构建脚本为准）。

- [ ] **Step 6: Commit**

```bash
git add Touch_Client/calibration/TcpCalibration.h Touch_Client/calibration/TcpCalibration.cpp Touch_Client/main.cpp
git commit -m "feat(calibration): add TCP offset collection + JSON persistence + keyboard wiring"
```

---

## 后续工作（独立计划，见硬件计划文档）

本软件计划到此覆盖**全部设备无关的软件侧**。以下内容写入独立计划 `Docs/superpowers/plans/2026-09-02-force-feedback-writing-demo-hardware.md`：

1. **笔夹 OpenSCAD 建模（spec §3.2）**：可调内径（V 型槽 + 侧向螺丝，~6-14mm）+ 笔杆滑动定位（短套筒 30-40mm），适配力传感器工具端法兰（4×M6 + Φ6 销）。
2. **实机调试**：TCP 偏移实机标定执行（跑 Task 8 采集、求解、与 FK 验证）、把 TCP 偏移应用到运动/轨迹记录、力反馈实机验证、Touch→Robot 坐标标定验证、安全边界标定、A/B 结项演示彩排。
