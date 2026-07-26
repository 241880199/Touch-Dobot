# MATLAB 全可视化重构 — 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将全部 2D 可视化从 C++ GLUT HUD 迁移到 MATLAB relay_gui，使用 uigridlayout 响应式布局 + STL 网格模型 3D 渲染。

**Architecture:** C++ 端删除 HudOverlay（~580行），新增 5 种 TCP 协议消息（S|/D|/L|/G|/B|）发送状态数据给 MATLAB。MATLAB 端重写 relay_gui.m，使用 uigridlayout 布局 + hgtransform STL 渲染 + 完整数据显示面板。

**Tech Stack:** MATLAB R2025b (uifigure/uigridlayout/uiaxes/hgtransform), C++17 (WinSock TCP), CR3 URDF kinematics

## Global Constraints

- MATLAB R2025b `uigridlayout` (R2018b+) — 必须用 `uifigure` 而非 `figure`
- STL 文件目录: `Touch_Client/models/cr3/*.STL`（相对于 MATLAB 脚本: `../Touch_Client/models/cr3/`）
- FK 参数与 C++ `RobotModel.h` 保持一致：J1_Z=136.0mm (标定值), J3_X=-274mm, J4_X=-230mm, J4_Z=128.3mm, J5_Y=-116mm, J6_Y=105mm
- TCP 端口 8888，消息以 `\n` 分隔，MATLAB `readline()` 逐行解析
- C++ 端保留精简 3D 视图（STL+地面+坐标轴+安全边界），不删 SceneRenderer
- 窗口最小尺寸 1000×600，支持自由缩放

---

### Task 1: MATLAB STL 二进制读取模块

**Files:**
- Create: `Relay_Station/+stl/loadBinaryStl.m`

**Interfaces:**
- Produces: `mesh = stl.loadBinaryStl(filepath)` → struct with fields `.vertices` (N×3), `.faces` (M×3), `.normals` (M×3), `.triangleCount`

- [ ] **Step 1: 创建 `+stl` 包目录和函数骨架**

```matlab
% Relay_Station/+stl/loadBinaryStl.m
function mesh = loadBinaryStl(filepath)
% LOADBINARYSTL 读取二进制 STL 文件
%   mesh = stl.loadBinaryStl(filepath)
%   返回 struct:
%     .vertices  — (N×3) 顶点坐标 (mm, 从 m 缩放)
%     .faces     — (M×3) 面索引 (1-based)
%     .normals   — (M×3) 面法线
%     .triangleCount — 三角面总数
    mesh = struct('vertices', [], 'faces', [], 'normals', [], 'triangleCount', 0);
    if ~isfile(filepath)
        warning('stl:fileNotFound', 'STL file not found: %s', filepath);
        return;
    end
    % TODO: implement binary read
end
```

- [ ] **Step 2: 实现二进制读取逻辑**

```matlab
% 替换函数体:
function mesh = loadBinaryStl(filepath)
    mesh = struct('vertices', [], 'faces', [], 'normals', [], 'triangleCount', 0);
    if ~isfile(filepath)
        warning('stl:fileNotFound', 'STL file not found: %s', filepath);
        return;
    end

    fid = fopen(filepath, 'rb');
    if fid < 0
        warning('stl:openFailed', 'Cannot open: %s', filepath);
        return;
    end

    % Skip 80-byte header
    fseek(fid, 80, 'bof');

    % Read triangle count (uint32)
    count = fread(fid, 1, 'uint32');
    if isempty(count) || count == 0
        fclose(fid);
        return;
    end

    % Each face: normal(3×float32) + v1(3×float32) + v2(3×float32) + v3(3×float32) + attr(2 bytes)
    % Read all float data at once
    raw = fread(fid, count * 12, 'float32');
    fclose(fid);

    if numel(raw) < count * 12
        warning('stl:truncated', 'File truncated: expected %d faces', count);
        return;
    end

    raw = reshape(raw, 12, count)';  % count×12 matrix

    % Extract normals (cols 1-3) and vertices (cols 4-12)
    mesh.normals = raw(:, 1:3);
    vertData = raw(:, 4:12);  % count×9: v1x,v1y,v1z,v2x,v2y,v2z,v3x,v3y,v3z

    % Reshape to (count*3)×3 vertex list
    vertData = vertData';  % 9×count
    mesh.vertices = reshape(vertData, 3, count*3)';  % (count*3)×3

    % Scale from meters to millimeters
    mesh.vertices = mesh.vertices * 1000;

    % Faces: 1-based indices (every 3 consecutive vertices form a face)
    mesh.faces = reshape(1:(count*3), 3, count)';
    mesh.triangleCount = count;
end
```

- [ ] **Step 3: 测试 — 加载一个 STL 文件验证**

在 MATLAB 命令窗口运行：
```matlab
addpath('D:/Projects/Touch/Relay_Station');
m = stl.loadBinaryStl('../Touch_Client/models/cr3/base_link.STL');
assert(m.triangleCount > 0, 'Triangle count should be > 0');
assert(size(m.vertices, 2) == 3, 'Vertices should be Nx3');
assert(size(m.faces, 2) == 3, 'Faces should be Mx3');
fprintf('base_link: %d triangles, %d vertices\n', m.triangleCount, size(m.vertices, 1));
```

- [ ] **Step 4: Commit**

```bash
git add Relay_Station/+stl/loadBinaryStl.m
git commit -m "feat(matlab): add STL binary loader module"
```

---

### Task 2: MATLAB FK 运动学模块

**Files:**
- Create: `Relay_Station/+fk/robotFk.m`
- Reference: `Relay_Station/relay_gui.m:380-425` (现有 computeFK 函数)

**Interfaces:**
- Produces: `joints = fk.robotFk(j1, j2, j3, j4, j5, j6)` → 7×3 matrix (base + J1~J6 world positions in mm)
- Produces: `T = fk.linkTransform(j1, j2, j3, j4, j5, j6, linkIdx)` → struct with `.T` (4×4 transform matrix) and `.T_world` (4×4 accumulated world transform) for link 0~6

- [ ] **Step 1: 提取并创建 FK 模块**

从 `relay_gui.m` 的 `computeFK()` 提取，并增加 `linkTransform()` 用于 hgtransform 矩阵更新：

```matlab
% Relay_Station/+fk/robotFk.m
function joints = robotFk(j1, j2, j3, j4, j5, j6)
% ROBOTFK CR3 正向运动学 (URDF 参数)
%   joints = fk.robotFk(j1..j6)  角度单位度, 返回 7×3 世界坐标 (mm)
    d2r = pi / 180;

    % URDF 关节参数 (mm) — 与 C++ RobotModel.h 及 relay_gui.m 保持一致
    j1_z  = 136.0;   % calibrated against real GetPose
    j3_x  = -274.0;
    j4_x  = -230.0;  j4_z = 128.3;
    j5_y  = -116.0;
    j6_y  = 105.0;

    % rpy 固定旋转 (弧度)
    j2_ry = pi/2;  j2_rx = pi/2;
    j4_rz = -pi/2;
    j5_rx = pi/2;
    j6_rx = -pi/2;

    T = eye(4);
    joints = zeros(7, 3);
    joints(1,:) = [0 0 0];  % base

    T = T * tr(0, 0, j1_z) * rotz(j1 * d2r);
    joints(2,:) = T(1:3,4)';

    T = T * roty(j2_ry) * rotx(j2_rx) * rotz(j2 * d2r);
    joints(3,:) = T(1:3,4)';

    T = T * tr(j3_x, 0, 0) * rotz(j3 * d2r);
    joints(4,:) = T(1:3,4)';

    T = T * tr(j4_x, 0, j4_z) * rotz(j4_rz) * rotz(j4 * d2r);
    joints(5,:) = T(1:3,4)';

    T = T * tr(0, j5_y, 0) * rotx(j5_rx) * rotz(j5 * d2r);
    joints(6,:) = T(1:3,4)';

    T = T * tr(0, j6_y, 0) * rotx(j6_rx) * rotz(j6 * d2r);
    joints(7,:) = T(1:3,4)';
end

function T = tr(x, y, z)
    T = eye(4); T(1:3,4) = [x; y; z];
end

function R = rotx(angle)
    c = cos(angle); s = sin(angle);
    R = [1 0 0 0; 0 c -s 0; 0 s c 0; 0 0 0 1];
end

function R = roty(angle)
    c = cos(angle); s = sin(angle);
    R = [c 0 s 0; 0 1 0 0; -s 0 c 0; 0 0 0 1];
end

function R = rotz(angle)
    c = cos(angle); s = sin(angle);
    R = [c -s 0 0; s c 0 0; 0 0 1 0; 0 0 0 1];
end
```

- [ ] **Step 2: 添加 linkTransform 函数**

在同一个文件中追加（在 `rotz` 函数之后）：

```matlab
function T = linkTransform(j1, j2, j3, j4, j5, j6, linkIdx)
% LINKTRANSFORM 返回指定 link 的世界变换矩阵 (用于 hgtransform)
%   T = fk.linkTransform(j1..j6, linkIdx)  linkIdx=0(base)~6
%   返回 4×4 齐次变换矩阵 (mm 单位, 与 STL 缩放一致)
    d2r = pi / 180;

    j1_z  = 136.0;  j3_x  = -274.0;
    j4_x  = -230.0; j4_z = 128.3;
    j5_y  = -116.0; j6_y = 105.0;

    j2_ry = pi/2;  j2_rx = pi/2;
    j4_rz = -pi/2;
    j5_rx = pi/2;
    j6_rx = -pi/2;

    T = eye(4);
    if linkIdx == 0, return; end  % base: identity

    T = T * tr(0, 0, j1_z) * rotz(j1 * d2r);
    if linkIdx == 1, return; end

    T = T * roty(j2_ry) * rotx(j2_rx) * rotz(j2 * d2r);
    if linkIdx == 2, return; end

    T = T * tr(j3_x, 0, 0) * rotz(j3 * d2r);
    if linkIdx == 3, return; end

    T = T * tr(j4_x, 0, j4_z) * rotz(j4_rz) * rotz(j4 * d2r);
    if linkIdx == 4, return; end

    T = T * tr(0, j5_y, 0) * rotx(j5_rx) * rotz(j5 * d2r);
    if linkIdx == 5, return; end

    T = T * tr(0, j6_y, 0) * rotx(j6_rx) * rotz(j6 * d2r);
    % linkIdx == 6
end
```

- [ ] **Step 3: 验证 — 与 relay_gui.m 现有 computeFK 对比**

在 MATLAB 命令窗口运行：
```matlab
addpath('D:/Projects/Touch/Relay_Station');
% 随机测试角度
ja = [10, -45, 30, 0, -15, 90];
j1_old = relay_gui_computeFK(ja(1),ja(2),ja(3),ja(4),ja(5),ja(6));  % extract old function first
j2_new = fk.robotFk(ja(1),ja(2),ja(3),ja(4),ja(5),ja(6));
assert(norm(j1_old - j2_new) < 1e-6, 'FK mismatch');
fprintf('FK validation PASSED\n');
```

- [ ] **Step 4: Commit**

```bash
git add Relay_Station/+fk/robotFk.m
git commit -m "feat(matlab): extract FK kinematics to +fk module"
```

---

### Task 3: C++ 新增协议消息发送

**Files:**
- Modify: `Touch_Client/relay/RelayCore.h:57-59` (新增 5 个函数声明)
- Modify: `Touch_Client/relay/RelayCore.cpp` (新增 5 个函数实现)

**Interfaces:**
- Consumes: `sendRelayUpdate(const char* msg)` (existing)
- Consumes: `appState.robotActualPose`, `appState.forceData`, `m_stateMachine`
- Produces: `sendSafetyStatus()`, `sendJointMargins()`, `sendSingularity()`, `sendCalibStatus()`
- Produces: `reportDiagnostic(RobotErrorCode code, double speedFactor, const char* reason)`

- [ ] **Step 1: RelayCore.h — 添加声明**

在 `RelayCore.h` 的 `reportCommand()` 声明之后（第 47 行后）添加：

```cpp
    // 新增: 发送状态数据到 MATLAB GUI (替代原 HudOverlay 显示)
    void sendSafetyStatus();
    void sendJointMargins();
    void sendSingularity();
    void sendCalibStatus();
    void reportDiagnostic(int errorCode, double speedFactor, const char* reason);
```

- [ ] **Step 2: RelayCore.cpp — 实现 sendSafetyStatus()**

在 `reportCommand()` 函数之后添加：

```cpp
void RelayCore::sendSafetyStatus() {
    const auto& sm = m_stateMachine;
    int state = static_cast<int>(sm.currentState());
    double spd = sm.speedFactor();
    int alarms = SafetyPredictor::instance().alarmCount();

    char buf[128];
    snprintf(buf, sizeof(buf), "S|%d,%.2f,%d", state, spd, alarms);
    sendRelayUpdate(buf);
}
```

- [ ] **Step 3: 实现 sendJointMargins()**

```cpp
void RelayCore::sendJointMargins() {
    static const double jlims[6][2] = {
        {-360, 360}, {-360, 360}, {-155, 155},
        {-360, 360}, {-360, 360}, {-360, 360}
    };

    auto& app = appState;
    EnterCriticalSection(&app.robotPoseMutex);
    double jvals[6] = {
        app.robotActualPose.j1, app.robotActualPose.j2,
        app.robotActualPose.j3, app.robotActualPose.j4,
        app.robotActualPose.j5, app.robotActualPose.j6
    };
    LeaveCriticalSection(&app.robotPoseMutex);

    char buf[128];
    snprintf(buf, sizeof(buf), "L|%.1f,%.1f,%.1f,%.1f,%.1f,%.1f",
        fmin(fabs(jvals[0] - jlims[0][0]), fabs(jlims[0][1] - jvals[0])),
        fmin(fabs(jvals[1] - jlims[1][0]), fabs(jlims[1][1] - jvals[1])),
        fmin(fabs(jvals[2] - jlims[2][0]), fabs(jlims[2][1] - jvals[2])),
        fmin(fabs(jvals[3] - jlims[3][0]), fabs(jlims[3][1] - jvals[3])),
        fmin(fabs(jvals[4] - jlims[4][0]), fabs(jlims[4][1] - jvals[4])),
        fmin(fabs(jvals[5] - jlims[5][0]), fabs(jlims[5][1] - jvals[5])));
    sendRelayUpdate(buf);
}
```

- [ ] **Step 4: 实现 sendSingularity()**

```cpp
void RelayCore::sendSingularity() {
    auto& app = appState;
    EnterCriticalSection(&app.robotPoseMutex);
    double x = app.robotActualPose.x;
    double y = app.robotActualPose.y;
    LeaveCriticalSection(&app.robotPoseMutex);

    double r_xy = sqrt(x * x + y * y);
    int singular = (r_xy < 30.0) ? 1 : 0;

    char buf[64];
    snprintf(buf, sizeof(buf), "G|%.1f,%d", r_xy, singular);
    sendRelayUpdate(buf);
}
```

- [ ] **Step 5: 实现 sendCalibStatus() 和 reportDiagnostic()**

```cpp
void RelayCore::sendCalibStatus() {
    char buf[64];
    snprintf(buf, sizeof(buf), "B|%d,%.2f",
        Calibration::enabled ? 1 : 0,
        Calibration::enabled ? Calibration::rmsError : -1.0);
    sendRelayUpdate(buf);
}

void RelayCore::reportDiagnostic(int errorCode, double speedFactor, const char* reason) {
    // Truncate reason to avoid buffer overflow
    char buf[256];
    snprintf(buf, sizeof(buf), "D|%d,%.2f,%.200s", errorCode, speedFactor,
        reason ? reason : "");
    sendRelayUpdate(buf);
}
```

- [ ] **Step 6: 在 RobotDiagnostics::logError 中触发 reportDiagnostic**

修改 `Touch_Client/safety/RobotDiagnostics.cpp` 的 `logError()` 函数，在日志写入后添加：

```cpp
// 在 logError() 函数末尾、return 之前添加:
RelayCore::instance().reportDiagnostic(
    static_cast<int>(error.code),
    error.speedFactor,
    errorCodeName(error.code));
```

需要在 `RobotDiagnostics.cpp` 顶部添加：
```cpp
#include "../relay/RelayCore.h"
```

- [ ] **Step 7: Commit**

```bash
git add Touch_Client/relay/RelayCore.h Touch_Client/relay/RelayCore.cpp Touch_Client/safety/RobotDiagnostics.cpp
git commit -m "feat(relay): add S|/L|/G|/B|/D| protocol message senders"
```

---

### Task 4: C++ main.cpp — 新增状态发送定时器

**Files:**
- Modify: `Touch_Client/main.cpp`

**Interfaces:**
- Consumes: `RelayCore::sendSafetyStatus()`, `sendJointMargins()`, `sendSingularity()`, `sendCalibStatus()`
- Consumes: `glutTimerFunc`

- [ ] **Step 1: 在 main.cpp 添加定时器回调函数**

在 `jointAngleTimer` 函数之后、`keyboard` 函数之前添加：

```cpp
void safetyStatusTimer(int) {
    if (!g_noRobot) {
        RelayCore::instance().sendSafetyStatus();
        RelayCore::instance().sendSingularity();
    }
    if (!appState.isClosing) {
        glutTimerFunc(200, safetyStatusTimer, 0);
    }
}

void jointMarginTimer(int) {
    if (!g_noRobot) {
        RelayCore::instance().sendJointMargins();
    }
    if (!appState.isClosing) {
        glutTimerFunc(500, jointMarginTimer, 0);
    }
}
```

- [ ] **Step 2: 在 init 序列中启动新定时器**

在 `main()` 函数中，现有的 `glutTimerFunc` 启动部分（约第 511 行），在 `jointAngleTimer` 启动之后添加：

```cpp
glutTimerFunc(1000, safetyStatusTimer, 0);   // 延迟1s启动, 之后每200ms
glutTimerFunc(1500, jointMarginTimer, 0);    // 延迟1.5s启动, 之后每500ms
```

- [ ] **Step 3: 在 init 成功时发送标定状态**

在 `main()` 的标定加载代码块（约第 503-508 行），`Calibration::load()` 之后添加：

```cpp
    // 上报标定状态到 MATLAB
    RelayCore::instance().sendCalibStatus();
```

- [ ] **Step 4: 在键盘标定求解成功后发送标定状态**

在 `keyboard()` 函数的 's' 键标定求解成功后（约第 270 行 `Calibration::cancelCollect()` 之前），添加：

```cpp
            RelayCore::instance().sendCalibStatus();
```

- [ ] **Step 5: Commit**

```bash
git add Touch_Client/main.cpp
git commit -m "feat(main): add safety/diagnostic status timers for MATLAB GUI"
```

---

### Task 5: MATLAB relay_gui.m 重写（核心任务）

**Files:**
- Rewrite: `Relay_Station/relay_gui.m`
- Create: (all code in one file as per design decision)

**Interfaces:**
- Consumes: `relay_config()` (existing)
- Consumes: `stl.loadBinaryStl()` (Task 1)
- Consumes: `fk.robotFk()`, `fk.linkTransform()` (Task 2)
- Consumes: TCP messages: `P|` `F|` `J|` `RP|` `C|` `S|` `L|` `G|` `B|` `D|`

**Layout structure:**
```
uigridlayout(4×3):
  Row 1 (fit, span 3): 顶部状态栏
  Row 2 (1x, 2 cols):  左栏 commands | 中栏 force raw | 右栏 3D (span rows 2-4)
  Row 3 (1x, 2 cols):  左栏 feedback | 中栏 force filt
  Row 4 (1x, 1 col):   (空)          | (空)           | 右栏 Robot State
```

Wait — the layout is tricky with uigridlayout. Let me think about this more carefully.

Actually, since the right column spans rows 2-4, and the left+center columns are in rows 2-3 (each split into top/bottom), I need nested grids.

Let me reconsider the layout:

```
Outer grid: 2 rows × 3 columns
  Row 1 (fit, span 3):   顶部状态栏
  Row 2 (1x, span 3):    主内容区

Main content (Row 2) inner grid: 1 row × 3 columns
  Col 1 (1x):  左栏 — inner grid 2×1 (commands / feedback)
  Col 2 (1x):  中栏 — inner grid 2×1 (force raw / force filt)
  Col 3 (2x):  右栏 — inner grid 3×1 (3D / Robot State / Safety)
```

This is cleaner. Let me use this structure.

Actually wait, I need to also handle the safety & diagnostics panel. Let me make it:

Right column inner grid: 3 rows
  Row 1 (3x): 3D view
  Row 2 (1x): Robot State  
  Row 3 (1x): Safety & Diagnostics

This matches the spec.

Let me write out the full relay_gui.m. This is going to be long but that's expected for the core task.

I'll structure the MATLAB code as:
1. Function header + config load + state init
2. STL loading + FK init
3. UI construction (uigridlayout hierarchy)
4. 3D scene init
5. Network init
6. Timer + update loop
7. Nested functions for data processing and display

Let me write this properly. Since this is the core deliverable and the most code, I'll be thorough.

Actually, for the plan format, I should break this into sub-steps. Each step should be 2-5 minutes. Let me break it down.<｜end▁of▁thinking｜>

<｜｜DSML｜｜tool_calls>
<｜｜DSML｜｜invoke name="Write">
<｜｜DSML｜｜parameter name="content" string="true"># MATLAB 全可视化重构 — 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将全部 2D 可视化从 C++ GLUT HUD 迁移到 MATLAB relay_gui，使用 uigridlayout 响应式布局 + STL 网格模型 3D 渲染。

**Architecture:** C++ 端删除 HudOverlay（~580行），新增 5 种 TCP 协议消息（S|/D|/L|/G|/B|）发送状态数据给 MATLAB。MATLAB 端重写 relay_gui.m，使用 uigridlayout 布局 + hgtransform STL 渲染 + 完整数据显示面板。

**Tech Stack:** MATLAB R2025b (uifigure/uigridlayout/uiaxes/hgtransform), C++17 (WinSock TCP), CR3 URDF kinematics

## Global Constraints

- MATLAB R2025b `uigridlayout` (R2018b+) — 必须用 `uifigure` 而非 `figure`
- STL 文件目录: `Touch_Client/models/cr3/*.STL`（相对于 MATLAB 脚本: `../Touch_Client/models/cr3/`）
- FK 参数与 C++ `RobotModel.h` 保持一致：J1_Z=136.0mm (标定值), J3_X=-274mm, J4_X=-230mm, J4_Z=128.3mm, J5_Y=-116mm, J6_Y=105mm
- TCP 端口 8888，消息以 `\n` 分隔，MATLAB `readline()` 逐行解析
- C++ 端保留精简 3D 视图（STL+地面+坐标轴+安全边界），不删 SceneRenderer
- 窗口最小尺寸 1000×600，支持自由缩放

---

### Task 1: MATLAB STL 二进制读取模块

**Files:**
- Create: `Relay_Station/+stl/loadBinaryStl.m`

**Interfaces:**
- Produces: `mesh = stl.loadBinaryStl(filepath)` → struct with fields `.vertices` (N×3), `.faces` (M×3), `.normals` (M×3), `.triangleCount`

- [ ] **Step 1: 创建 +stl 包目录和完整实现**

```matlab
% Relay_Station/+stl/loadBinaryStl.m
function mesh = loadBinaryStl(filepath)
% LOADBINARYSTL 读取二进制 STL 文件
%   mesh = stl.loadBinaryStl(filepath)
    mesh = struct('vertices', [], 'faces', [], 'normals', [], 'triangleCount', 0);
    if ~isfile(filepath)
        warning('stl:fileNotFound', 'STL file not found: %s', filepath);
        return;
    end
    fid = fopen(filepath, 'rb');
    if fid < 0, warning('stl:openFailed', 'Cannot open: %s', filepath); return; end
    fseek(fid, 80, 'bof');
    count = fread(fid, 1, 'uint32');
    if isempty(count) || count == 0, fclose(fid); return; end
    raw = fread(fid, count * 12, 'float32');
    fclose(fid);
    if numel(raw) < count * 12, warning('stl:truncated', 'File truncated'); return; end
    raw = reshape(raw, 12, count)';
    mesh.normals = raw(:, 1:3);
    vertData = raw(:, 4:12)';
    mesh.vertices = reshape(vertData, 3, count*3)' * 1000;  % m → mm
    mesh.faces = reshape(1:(count*3), 3, count)';
    mesh.triangleCount = count;
end
```

- [ ] **Step 2: 验证 STL 加载**

在 MATLAB 命令窗口：
```matlab
addpath('D:/Projects/Touch/Relay_Station');
m = stl.loadBinaryStl('../Touch_Client/models/cr3/base_link.STL');
assert(m.triangleCount > 0);
assert(size(m.vertices,2)==3 && size(m.faces,2)==3);
fprintf('OK: %d triangles\n', m.triangleCount);
```

- [ ] **Step 3: Commit**

```bash
git add Relay_Station/+stl/loadBinaryStl.m
git commit -m "feat(matlab): add STL binary loader module"
```

---

### Task 2: MATLAB FK 运动学模块

**Files:**
- Create: `Relay_Station/+fk/robotFk.m`

**Interfaces:**
- Produces: `joints = fk.robotFk(j1,j2,j3,j4,j5,j6)` → 7×3 world positions (mm)
- Produces: `T = fk.linkTransform(j1,j2,j3,j4,j5,j6, linkIdx)` → 4×4 transform for linkIdx 0~6

- [ ] **Step 1: 创建完整 FK 模块**

```matlab
% Relay_Station/+fk/robotFk.m
function joints = robotFk(j1, j2, j3, j4, j5, j6)
    d2r = pi / 180;
    j1_z=136.0; j3_x=-274.0; j4_x=-230.0; j4_z=128.3; j5_y=-116.0; j6_y=105.0;
    j2_ry=pi/2; j2_rx=pi/2; j4_rz=-pi/2; j5_rx=pi/2; j6_rx=-pi/2;
    T = eye(4); joints = zeros(7,3); joints(1,:)=[0 0 0];
    T=T*tr(0,0,j1_z)*rotz(j1*d2r); joints(2,:)=T(1:3,4)';
    T=T*roty(j2_ry)*rotx(j2_rx)*rotz(j2*d2r); joints(3,:)=T(1:3,4)';
    T=T*tr(j3_x,0,0)*rotz(j3*d2r); joints(4,:)=T(1:3,4)';
    T=T*tr(j4_x,0,j4_z)*rotz(j4_rz)*rotz(j4*d2r); joints(5,:)=T(1:3,4)';
    T=T*tr(0,j5_y,0)*rotx(j5_rx)*rotz(j5*d2r); joints(6,:)=T(1:3,4)';
    T=T*tr(0,j6_y,0)*rotx(j6_rx)*rotz(j6*d2r); joints(7,:)=T(1:3,4)';
end

function T = linkTransform(j1,j2,j3,j4,j5,j6,linkIdx)
    d2r=pi/180; j1_z=136.0; j3_x=-274.0; j4_x=-230.0; j4_z=128.3;
    j5_y=-116.0; j6_y=105.0;
    j2_ry=pi/2; j2_rx=pi/2; j4_rz=-pi/2; j5_rx=pi/2; j6_rx=-pi/2;
    T=eye(4); if linkIdx==0, return; end
    T=T*tr(0,0,j1_z)*rotz(j1*d2r); if linkIdx==1, return; end
    T=T*roty(j2_ry)*rotx(j2_rx)*rotz(j2*d2r); if linkIdx==2, return; end
    T=T*tr(j3_x,0,0)*rotz(j3*d2r); if linkIdx==3, return; end
    T=T*tr(j4_x,0,j4_z)*rotz(j4_rz)*rotz(j4*d2r); if linkIdx==4, return; end
    T=T*tr(0,j5_y,0)*rotx(j5_rx)*rotz(j5*d2r); if linkIdx==5, return; end
    T=T*tr(0,j6_y,0)*rotx(j6_rx)*rotz(j6*d2r);
end

function T=tr(x,y,z), T=eye(4); T(1:3,4)=[x;y;z]; end
function R=rotx(a), c=cos(a); s=sin(a); R=[1 0 0 0;0 c -s 0;0 s c 0;0 0 0 1]; end
function R=roty(a), c=cos(a); s=sin(a); R=[c 0 s 0;0 1 0 0;-s 0 c 0;0 0 0 1]; end
function R=rotz(a), c=cos(a); s=sin(a); R=[c -s 0 0;s c 0 0;0 0 1 0;0 0 0 1]; end
```

- [ ] **Step 2: 验证 FK 一致性**

```matlab
addpath('D:/Projects/Touch/Relay_Station');
ja = [10 -45 30 0 -15 90];
j = fk.robotFk(ja(1),ja(2),ja(3),ja(4),ja(5),ja(6));
assert(size(j,1)==7 && size(j,2)==3);
% Check T matrix is valid
T6 = fk.linkTransform(ja(1),ja(2),ja(3),ja(4),ja(5),ja(6), 6);
assert(all(size(T6)==[4 4]) && abs(det(T6(1:3,1:3))-1)<1e-6);
fprintf('FK validation OK\n');
```

- [ ] **Step 3: Commit**

```bash
git add Relay_Station/+fk/robotFk.m
git commit -m "feat(matlab): extract FK kinematics to +fk package"
```

---

### Task 3: C++ 新增协议消息发送

**Files:**
- Modify: `Touch_Client/relay/RelayCore.h` (add 5 declarations after line 47)
- Modify: `Touch_Client/relay/RelayCore.cpp` (add 5 implementations after reportCommand)
- Modify: `Touch_Client/safety/RobotDiagnostics.cpp` (call reportDiagnostic in logError)

**Interfaces:**
- Produces: `void sendSafetyStatus()`, `void sendJointMargins()`, `void sendSingularity()`, `void sendCalibStatus()`, `void reportDiagnostic(int,double,const char*)`

- [ ] **Step 1: RelayCore.h 添加声明**

在 `void reportCommand(const char* cmd);` 之后插入：

```cpp
    void sendSafetyStatus();
    void sendJointMargins();
    void sendSingularity();
    void sendCalibStatus();
    void reportDiagnostic(int errorCode, double speedFactor, const char* reason);
```

- [ ] **Step 2: RelayCore.cpp 添加 5 个实现**

在 `reportCommand()` 函数体之后（文件末尾 `shutdownForceReader` 之前）插入完整实现：

```cpp
void RelayCore::sendSafetyStatus() {
    const auto& sm = m_stateMachine;
    char buf[128];
    snprintf(buf, sizeof(buf), "S|%d,%.2f,%d",
        static_cast<int>(sm.currentState()),
        sm.speedFactor(),
        SafetyPredictor::instance().alarmCount());
    sendRelayUpdate(buf);
}

void RelayCore::sendJointMargins() {
    static const double jlims[6][2] = {
        {-360,360},{-360,360},{-155,155},{-360,360},{-360,360},{-360,360}};
    auto& app = appState;
    EnterCriticalSection(&app.robotPoseMutex);
    double jv[6]={app.robotActualPose.j1,app.robotActualPose.j2,
        app.robotActualPose.j3,app.robotActualPose.j4,
        app.robotActualPose.j5,app.robotActualPose.j6};
    LeaveCriticalSection(&app.robotPoseMutex);
    char buf[128];
    snprintf(buf,sizeof(buf),"L|%.1f,%.1f,%.1f,%.1f,%.1f,%.1f",
        fmin(fabs(jv[0]-jlims[0][0]),fabs(jlims[0][1]-jv[0])),
        fmin(fabs(jv[1]-jlims[1][0]),fabs(jlims[1][1]-jv[1])),
        fmin(fabs(jv[2]-jlims[2][0]),fabs(jlims[2][1]-jv[2])),
        fmin(fabs(jv[3]-jlims[3][0]),fabs(jlims[3][1]-jv[3])),
        fmin(fabs(jv[4]-jlims[4][0]),fabs(jlims[4][1]-jv[4])),
        fmin(fabs(jv[5]-jlims[5][0]),fabs(jlims[5][1]-jv[5])));
    sendRelayUpdate(buf);
}

void RelayCore::sendSingularity() {
    auto& app = appState;
    EnterCriticalSection(&app.robotPoseMutex);
    double x=app.robotActualPose.x, y=app.robotActualPose.y;
    LeaveCriticalSection(&app.robotPoseMutex);
    double r_xy=sqrt(x*x+y*y);
    char buf[64];
    snprintf(buf,sizeof(buf),"G|%.1f,%d",r_xy,(r_xy<30.0)?1:0);
    sendRelayUpdate(buf);
}

void RelayCore::sendCalibStatus() {
    char buf[64];
    snprintf(buf,sizeof(buf),"B|%d,%.2f",
        Calibration::enabled?1:0, Calibration::enabled?Calibration::rmsError:-1.0);
    sendRelayUpdate(buf);
}

void RelayCore::reportDiagnostic(int errorCode, double speedFactor, const char* reason) {
    char buf[256];
    snprintf(buf,sizeof(buf),"D|%d,%.2f,%.200s",errorCode,speedFactor,reason?reason:"");
    sendRelayUpdate(buf);
}
```

- [ ] **Step 3: RobotDiagnostics.cpp 触发诊断上报**

在文件顶部 `#include` 区域添加：
```cpp
#include "../relay/RelayCore.h"
```

在 `logError()` 函数中，日志写入后添加：
```cpp
    RelayCore::instance().reportDiagnostic(
        static_cast<int>(error.code), error.speedFactor, errorCodeName(error.code));
```

- [ ] **Step 4: Commit**

```bash
git add Touch_Client/relay/RelayCore.h Touch_Client/relay/RelayCore.cpp Touch_Client/safety/RobotDiagnostics.cpp
git commit -m "feat(relay): add S|/L|/G|/B|/D| protocol senders for MATLAB GUI"
```

---

### Task 4: C++ main.cpp — 新增定时器

**Files:**
- Modify: `Touch_Client/main.cpp`

- [ ] **Step 1: 添加定时器回调函数**

在 `jointAngleTimer()` 函数之后、`keyboard()` 函数之前插入：

```cpp
void safetyStatusTimer(int) {
    if (!g_noRobot) {
        RelayCore::instance().sendSafetyStatus();
        RelayCore::instance().sendSingularity();
    }
    if (!appState.isClosing) glutTimerFunc(200, safetyStatusTimer, 0);
}

void jointMarginTimer(int) {
    if (!g_noRobot) RelayCore::instance().sendJointMargins();
    if (!appState.isClosing) glutTimerFunc(500, jointMarginTimer, 0);
}
```

- [ ] **Step 2: 在 main() init 序列中启动定时器**

在 `glutTimerFunc(500, jointAngleTimer, 0);` 之后添加：

```cpp
    glutTimerFunc(1000, safetyStatusTimer, 0);
    glutTimerFunc(1500, jointMarginTimer, 0);
```

- [ ] **Step 3: 标定加载后发送状态**

在 `Calibration::load("calibration.json")` 成功分支内，`std::cout` 之后添加：
```cpp
        RelayCore::instance().sendCalibStatus();
```

- [ ] **Step 4: 标定求解成功后发送状态**

在 `keyboard()` 函数中 `Calibration::save("calibration.json")` 之后、`cancelCollect()` 之前：
```cpp
            RelayCore::instance().sendCalibStatus();
```

- [ ] **Step 5: Commit**

```bash
git add Touch_Client/main.cpp
git commit -m "feat(main): add safety/diagnostic status timers for MATLAB"
```

---

### Task 5: MATLAB relay_gui.m 重写 — UI 框架 + 状态初始化

**Files:**
- Rewrite: `Relay_Station/relay_gui.m`

- [ ] **Step 1: 创建函数骨架和全局状态**

```matlab
function relay_gui()
% Touch-Dobot Relay Station v4.0
% 全可视化 MATLAB GUI: STL 3D + uigridlayout + 完整数据面板
% C++ 端仅保留控制层, 所有显示由此 GUI 负责

    % ===== 全局状态 =====
    S = struct();
    S.running = true;
    S.cmd_log = cell(50,1);  S.cmd_idx = 0;
    S.fb_log  = cell(50,1);  S.fb_idx  = 0;
    S.touch_pos    = [0 0 0 0 0 0];
    S.robot_pos    = [300 0 200 0 0 0];
    S.robot_target = [300 0 200 0 0 0];
    S.joint_angles = [0 0 0 0 0 0];
    S.force_raw    = [0 0 0];
    S.force_filt   = [0 0 0];
    S.force_moment = [0 0 0];
    S.force_stale  = 0;
    S.touch_relay_delay = 0;
    S.packet_count = 0;
    S.last_time = tic;
    % 新增状态字段
    S.safety_state = 0;      S.safety_speed = 1.0;  S.safety_alarms = 0;
    S.joint_margins = [999 999 999 999 999 999];
    S.z_dist = 999;          S.singular = 0;
    S.calib_enabled = false; S.calib_rms = -1;
    S.diag_code = 0;         S.diag_spd = 1.0;  S.diag_reason = '';
    S.server = [];
```

- [ ] **Step 2: 加载配置 + 设置路径**

```matlab
    % ===== 加载配置 =====
    cfg = relay_config();

    % ===== 确保 +stl +fk 在路径中 =====
    scriptDir = fileparts(mfilename('fullpath'));
    if ~contains(path, fullfile(scriptDir, '+stl'))
        addpath(scriptDir);
    end
```

- [ ] **Step 3: 创建主窗口和颜色主题**

```matlab
    % ===== 颜色主题 =====
    clr = struct(...
        'bg_dark',  [0.10 0.12 0.16], 'bg_panel', [0.08 0.10 0.14], ...
        'border',   [0.20 0.30 0.50], 'text_on',  [0.90 0.94 1.00], ...
        'text_dim', [0.55 0.58 0.62], 'green',    [0.20 0.80 0.40], ...
        'red',      [0.95 0.25 0.25], 'blue',     [0.30 0.65 1.00], ...
        'orange',   [1.00 0.60 0.15], 'yellow',   [1.00 0.85 0.20]);

    % ===== 主窗口 =====
    fig = uifigure('Name', 'Touch-Dobot Relay Station', ...
                   'Position', [100 50 1400 850], ...
                   'Color', clr.bg_dark, ...
                   'CloseRequestFcn', @(~,~) onClose());
    fig.SizeChangedFcn = @(~,~) onResize();
```

- [ ] **Step 4: Commit**

```bash
git add Relay_Station/relay_gui.m
git commit -m "feat(gui): relay_gui v4.0 skeleton — state, config, colors, window"
```

---

### Task 6: MATLAB relay_gui — uigridlayout 布局

- [ ] **Step 1: 创建外层网格布局**

```matlab
    % ===== 外层网格: 2行 × 3列 =====
    g = uigridlayout(fig, [2 3]);
    g.RowHeight = {25, '1x'};            % 顶部 25px + 内容区 fill
    g.ColumnWidth = {'1x', '1x', '1.6x'}; % 左:中:右 = 1:1:1.6
    g.Padding = [2 2 2 2];
    g.RowSpacing = 2;
    g.ColumnSpacing = 2;
    g.BackgroundColor = clr.bg_dark;
```

- [ ] **Step 2: 创建顶部状态栏 (Row 1, span all columns)**

```matlab
    % ===== 顶部状态栏 =====
    pnlTop = uipanel(g, 'BackgroundColor', [0.06 0.08 0.12], 'BorderType', 'none');
    pnlTop.Layout.Row = 1;
    pnlTop.Layout.Column = [1 3];

    glTop = uigridlayout(pnlTop, [1 6]);
    glTop.ColumnWidth = {220, 280, 180, '1x', 150, 120};
    glTop.Padding = [4 1 4 1];
    glTop.BackgroundColor = [0.06 0.08 0.12];

    lblTitle = uilabel(glTop, 'Text', 'Touch-Dobot Relay Station', ...
        'FontColor', clr.blue, 'FontWeight', 'bold', 'FontSize', 11);
    lblDelay = uilabel(glTop, 'Text', 'Touch->Relay: -- ms', ...
        'FontColor', clr.green, 'FontSize', 10);
    lblIp = uilabel(glTop, 'Text', ['Robot IP: ' cfg.robot_ip], ...
        'FontColor', clr.text_dim, 'FontSize', 10);
    lblSpacer = uilabel(glTop, 'Text', '');
    lblState = uilabel(glTop, 'Text', '[--]', 'FontColor', clr.text_dim, ...
        'FontSize', 10, 'HorizontalAlignment', 'right');
    lblConn = uilabel(glTop, 'Text', 'C++ Client: OFFLINE', ...
        'FontColor', clr.red, 'FontSize', 10, 'HorizontalAlignment', 'right');
```

- [ ] **Step 3: 创建左栏和中栏的嵌套网格 (Row 2, Col 1 和 Col 2)**

```matlab
    % ===== 左栏 (Col 1): 指令 + 反馈 =====
    pnlLeft = uipanel(g, 'BackgroundColor', clr.bg_panel, 'BorderType', 'none');
    pnlLeft.Layout.Row = 2;  pnlLeft.Layout.Column = 1;
    glLeft = uigridlayout(pnlLeft, [2 1]);
    glLeft.RowHeight = {'1x', '1x'};
    glLeft.Padding = [1 1 1 1];  glLeft.RowSpacing = 2;
    glLeft.BackgroundColor = clr.bg_panel;

    % 指令面板
    pnlCmd = uipanel(glLeft, 'BackgroundColor', clr.bg_panel, 'BorderType', 'none');
    pnlCmd.Layout.Row = 1;  pnlCmd.Layout.Column = 1;
    lblCmdTitle = uilabel(pnlCmd, 'Text', 'Touch -> Robot (Commands)', ...
        'Position', [6 0 360 22], 'FontColor', clr.text_on, 'FontSize', 11, 'FontWeight', 'bold');
    lblCmd = uilabel(pnlCmd, 'Text', '(waiting for commands...)', ...
        'Position', [6 -350 360 350], 'FontColor', [0.30 0.85 0.50], ...
        'FontSize', 9, 'VerticalAlignment', 'top', 'FontName', 'Consolas');

    % 反馈面板
    pnlFb = uipanel(glLeft, 'BackgroundColor', clr.bg_panel, 'BorderType', 'none');
    pnlFb.Layout.Row = 2;  pnlFb.Layout.Column = 1;
    lblFbTitle = uilabel(pnlFb, 'Text', 'Robot -> Relay (Feedback)', ...
        'Position', [6 0 360 22], 'FontColor', clr.text_on, 'FontSize', 11, 'FontWeight', 'bold');
    lblFb = uilabel(pnlFb, 'Text', '(waiting for feedback...)', ...
        'Position', [6 -350 360 350], 'FontColor', clr.text_dim, ...
        'FontSize', 9, 'VerticalAlignment', 'top', 'FontName', 'Consolas');
```

- [ ] **Step 4: 创建中栏 (Col 2): 力数据**

```matlab
    % ===== 中栏 (Col 2): 力数据 =====
    pnlMid = uipanel(g, 'BackgroundColor', clr.bg_panel, 'BorderType', 'none');
    pnlMid.Layout.Row = 2;  pnlMid.Layout.Column = 2;
    glMid = uigridlayout(pnlMid, [2 1]);
    glMid.RowHeight = {'1x', '1x'};
    glMid.Padding = [1 1 1 1];  glMid.RowSpacing = 2;
    glMid.BackgroundColor = clr.bg_panel;

    % 原始力
    pnlFR = uipanel(glMid, 'BackgroundColor', clr.bg_panel, 'BorderType', 'none');
    pnlFR.Layout.Row = 1;  pnlFR.Layout.Column = 1;
    lblFRTitle = uilabel(pnlFR, 'Text', 'Force Sensor (Raw · 30004)', ...
        'Position', [6 0 360 22], 'FontColor', clr.text_on, 'FontSize', 11, 'FontWeight', 'bold');
    lblForceRaw = uilabel(pnlFR, 'Position', [10 -100 350 120], ...
        'Text', {'Awaiting force sensor data...', '', ...
                 'Fx:   0.00 N   Fy:   0.00 N   Fz:   0.00 N'}, ...
        'FontColor', clr.text_dim, 'FontSize', 10, ...
        'VerticalAlignment', 'top', 'FontName', 'Consolas');

    % 滤波力
    pnlFF = uipanel(glMid, 'BackgroundColor', clr.bg_panel, 'BorderType', 'none');
    pnlFF.Layout.Row = 2;  pnlFF.Layout.Column = 1;
    lblFFTitle = uilabel(pnlFF, 'Text', 'Force Output (Filtered -> Touch)', ...
        'Position', [6 0 360 22], 'FontColor', clr.text_on, 'FontSize', 11, 'FontWeight', 'bold');
    lblForceFilt = uilabel(pnlFF, 'Position', [10 -100 350 120], ...
        'Text', {'Filtered force for haptic feedback...', '', ...
                 'Fx:   0.00 N   Fy:   0.00 N   Fz:   0.00 N'}, ...
        'FontColor', clr.text_dim, 'FontSize', 10, ...
        'VerticalAlignment', 'top', 'FontName', 'Consolas');
```

- [ ] **Step 5: 创建右栏 (Col 3): 3D + Robot State + Safety**

```matlab
    % ===== 右栏 (Col 3): 3D + 状态 + 安全 =====
    pnlRight = uipanel(g, 'BackgroundColor', clr.bg_panel, 'BorderType', 'none');
    pnlRight.Layout.Row = 2;  pnlRight.Layout.Column = 3;
    glRight = uigridlayout(pnlRight, [3 1]);
    glRight.RowHeight = {'3x', '1x', '1x'};
    glRight.Padding = [1 1 1 1];  glRight.RowSpacing = 2;
    glRight.BackgroundColor = clr.bg_panel;

    % 3D 视图
    pnl3D = uipanel(glRight, 'BackgroundColor', clr.bg_panel, 'BorderType', 'none');
    pnl3D.Layout.Row = 1;  pnl3D.Layout.Column = 1;
    ax3d = uiaxes(pnl3D, 'BackgroundColor', [0.12 0.14 0.18], ...
        'XColor', clr.text_dim, 'YColor', clr.text_dim, 'ZColor', clr.text_dim, ...
        'Box', 'on', 'GridLineStyle', ':');
    title(ax3d, 'Digital Twin', 'Color', clr.text_on, 'FontSize', 11);
    xlabel(ax3d, 'X (mm)'); ylabel(ax3d, 'Y (mm)'); zlabel(ax3d, 'Z (mm)');
    hold(ax3d, 'on'); axis(ax3d, 'equal');
    ax3d.View = [45 30];
    xlim(ax3d, [0 500]); ylim(ax3d, [-250 250]); zlim(ax3d, [0 400]);

    % Robot State 面板
    pnlState = uipanel(glRight, 'BackgroundColor', clr.bg_panel, 'BorderType', 'none');
    pnlState.Layout.Row = 2;  pnlState.Layout.Column = 1;
    lblStateTitle = uilabel(pnlState, 'Text', 'Robot State', ...
        'Position', [6 2 600 22], 'FontColor', clr.text_on, 'FontSize', 11, 'FontWeight', 'bold');
    lblCoord = uilabel(pnlState, 'Position', [6 -100 600 125], ...
        'Text', 'Initializing...', 'FontColor', [0.70 0.85 0.50], ...
        'FontSize', 10, 'VerticalAlignment', 'top', 'FontName', 'Consolas');

    % Safety 面板
    pnlSafety = uipanel(glRight, 'BackgroundColor', clr.bg_panel, 'BorderType', 'none');
    pnlSafety.Layout.Row = 3;  pnlSafety.Layout.Column = 1;
    lblSafeTitle = uilabel(pnlSafety, 'Text', 'Safety & Diagnostics', ...
        'Position', [6 2 600 22], 'FontColor', clr.text_on, 'FontSize', 11, 'FontWeight', 'bold');
    lblSafety = uilabel(pnlSafety, 'Position', [6 -100 600 125], ...
        'Text', 'Safety: --', 'FontColor', clr.green, ...
        'FontSize', 10, 'VerticalAlignment', 'top', 'FontName', 'Consolas');
```

- [ ] **Step 6: Commit**

```bash
git add Relay_Station/relay_gui.m
git commit -m "feat(gui): uigridlayout — 3-column responsive layout with all panels"
```

---

### Task 7: MATLAB relay_gui — STL 加载与 3D 场景

- [ ] **Step 1: STL 加载 + hgtransform 初始化**

在 init3DScene() 和 3D 渲染相关代码（写入 relay_gui.m，在 UI 构建代码之后、timer 初始化之前）：

```matlab
    % ===== STL 模型加载 =====
    stlDir = fullfile(scriptDir, '..', 'Touch_Client', 'models', 'cr3');
    linkNames = {'base_link', 'Link1', 'Link2', 'Link3', 'Link4', 'Link5', 'Link6'};
    linkMesh = cell(1, 7);
    linkPatch = gobjects(1, 7);
    linkHg = gobjects(1, 7);
    stlLoaded = false;

    for i = 1:7
        stlPath = fullfile(stlDir, [linkNames{i} '.STL']);
        linkMesh{i} = stl.loadBinaryStl(stlPath);
        if linkMesh{i}.triangleCount > 0
            stlLoaded = true;
        end
    end

    % ===== 3D 场景初始化 =====
    % 地面网格
    [Xg, Yg] = meshgrid(0:50:500, -250:50:250);
    Zg = zeros(size(Xg));
    mesh(ax3d, Xg, Yg, Zg, 'FaceAlpha', 0.1, 'EdgeColor', [0.2 0.25 0.3], 'LineWidth', 0.5);

    % 坐标系
    quiver3(ax3d, 0,0,0, 80,0,0, 'r', 'LineWidth', 2, 'MaxHeadSize', 5);
    quiver3(ax3d, 0,0,0, 0,80,0, 'g', 'LineWidth', 2, 'MaxHeadSize', 5);
    quiver3(ax3d, 0,0,0, 0,0,80, 'b', 'LineWidth', 2, 'MaxHeadSize', 5);

    % 安全边界线框
    xL = [cfg.safe_x_min cfg.safe_x_max];
    yL = [cfg.safe_y_min cfg.safe_y_max];
    zL = [cfg.safe_z_min cfg.safe_z_max];
    plot3(ax3d, xL([1 1 2 2 1]), yL([1 2 2 1 1]), zL([1 1 1 1 1]), 'y--', 'LineWidth', 1);
    plot3(ax3d, xL([1 1 2 2 1]), yL([1 2 2 1 1]), zL([2 2 2 2 2]), 'y--', 'LineWidth', 1);
    for ii = 1:2
        for jj = 1:2
            plot3(ax3d, [xL(ii) xL(ii)], [yL(jj) yL(jj)], zL, 'y--', 'LineWidth', 1);
        end
    end

    % 创建 STL patch 对象 (如果加载成功) 否则 fallback 骨架模型
    if stlLoaded
        for i = 1:7
            linkPatch(i) = patch(ax3d, 'Faces', linkMesh{i}.faces, ...
                'Vertices', linkMesh{i}.vertices, ...
                'FaceColor', [0.25 0.28 0.32], 'EdgeColor', 'none', ...
                'FaceLighting', 'gouraud', 'AmbientStrength', 0.5);
            linkHg(i) = hgtransform(ax3d);
            linkPatch(i).Parent = linkHg(i);
        end
        % 添加光源
        light(ax3d, 'Position', [300 -300 400], 'Style', 'local');
    end

    % Touch 笔可视化对象
    touchPenBody = surface(ax3d, [], [], [], 'FaceColor', [0.35 0.38 0.42], ...
        'EdgeColor', 'none', 'Visible', 'off');
    touchPenTip = surface(ax3d, [], [], [], 'FaceColor', [1 0.15 0.1], ...
        'EdgeColor', 'none', 'FaceAlpha', 0.9, 'Visible', 'off');
    % 末端标记
    eeMarkerActual = surface(ax3d, [], [], [], 'FaceColor', [0.2 0.85 0.35], ...
        'EdgeColor', 'none', 'FaceAlpha', 0.8, 'Visible', 'off');
    eeMarkerTarget = line(ax3d, 0, 0, 0, 'Color', 'r', 'Marker', 'o', ...
        'MarkerSize', 10, 'LineWidth', 2, 'Visible', 'off');
```

- [ ] **Step 2: Commit**

```bash
git add Relay_Station/relay_gui.m
git commit -m "feat(gui): STL loading + hgtransform 3D scene initialization"
```

---

### Task 8: MATLAB relay_gui — 网络 + 定时器 + 主循环

- [ ] **Step 1: TCP 服务器初始化**

```matlab
    % ===== 网络初始化 =====
    function initNetwork()
        try
            S.server = tcpserver(cfg.listen_ip, cfg.relay_port);
            S.server.Timeout = cfg.timeout;
            S.server.ConnectionChangedFcn = @onServerConnection;
            fprintf('[Relay] TCP server listening on %s:%d\n', cfg.listen_ip, cfg.relay_port);
        catch e
            fprintf('[Relay] ERROR starting server: %s\n', e.message);
        end
    end

    function onServerConnection(src, ~)
        if src.Connected
            fprintf('[Relay] Touch client connected\n');
            lblConn.Text = 'C++ Client: CONNECTED';
            lblConn.FontColor = clr.green;
        else
            fprintf('[Relay] Touch client disconnected\n');
            lblConn.Text = 'C++ Client: OFFLINE';
            lblConn.FontColor = clr.red;
        end
    end
```

- [ ] **Step 2: 定时器**

```matlab
    % ===== 更新定时器 (20Hz) =====
    tmr = timer('Period', 0.05, 'ExecutionMode', 'fixedRate', ...
                'TimerFcn', @(~,~) updateDisplay(), ...
                'ErrorFcn', @(~,~) disp('Timer error'));
```

- [ ] **Step 3: updateDisplay 主循环**

```matlab
    function updateDisplay()
        if ~S.running, return; end
        try
            processNetworkData();
            update3DModel();
            updateTextPanels();
            drawnow limitrate;
        catch
        end
    end
```

- [ ] **Step 4: 启动入口**

```matlab
    % ===== 启动 =====
    initNetwork();
    start(tmr);
    fprintf('[Relay] GUI ready.\n');

    % ===== 清理 =====
    function onClose()
        S.running = false;
        stop(tmr); delete(tmr);
        if ~isempty(S.server) && isvalid(S.server), delete(S.server); end
        delete(fig);
        disp('[Relay] GUI closed.');
    end
end  % relay_gui()
```

- [ ] **Step 5: Commit**

```bash
git add Relay_Station/relay_gui.m
git commit -m "feat(gui): TCP server + 20Hz timer + update loop"
```

---

### Task 9: MATLAB relay_gui — processNetworkData (协议解析)

- [ ] **Step 1: 实现完整协议解析**

在 `updateDisplay()` 函数之前添加：

```matlab
    function processNetworkData()
        if isempty(S.server) || ~isvalid(S.server) || S.server.NumBytesAvailable == 0
            return;
        end
        try
            while S.server.NumBytesAvailable > 0
                raw = readline(S.server);
                if isempty(raw) || ismissing(raw), continue; end
                if isstring(raw), raw = char(raw); end
                if ~ischar(raw), continue; end
                msg = strtrim(raw);
                if isempty(msg), continue; end

                S.packet_count = S.packet_count + 1;

                % -- 现有协议 --
                if startsWith(msg, 'P|')
                    vals = sscanf(msg(3:end), '%f,%f,%f,%f,%f,%f');
                    if length(vals) == 6, S.touch_pos = vals'; S.robot_target = vals'; end
                elseif startsWith(msg, 'C|')
                    S.cmd_idx = mod(S.cmd_idx, 50) + 1;
                    S.cmd_log{S.cmd_idx} = msg(3:end);
                elseif startsWith(msg, 'F|')
                    vals = str2double(split(msg(3:end), ','));
                    if numel(vals) >= 7
                        S.force_raw = vals(1:3)'; S.force_filt = vals(1:3)';
                        S.force_moment = vals(4:6)'; S.force_stale = vals(7);
                    end
                elseif startsWith(msg, 'J|')
                    vals = sscanf(msg(3:end), '%f,%f,%f,%f,%f,%f');
                    if length(vals) == 6, S.joint_angles = vals'; end
                elseif startsWith(msg, 'RP|')
                    vals = sscanf(msg(3:end), '%f,%f,%f,%f,%f,%f');
                    if length(vals) == 6, S.robot_pos = vals'; end
                % -- 新协议 --
                elseif startsWith(msg, 'S|')
                    vals = sscanf(msg(3:end), '%d,%f,%d');
                    if length(vals) == 3
                        S.safety_state = vals(1); S.safety_speed = vals(2);
                        S.safety_alarms = vals(3);
                    end
                elseif startsWith(msg, 'L|')
                    vals = sscanf(msg(3:end), '%f,%f,%f,%f,%f,%f');
                    if length(vals) == 6, S.joint_margins = vals'; end
                elseif startsWith(msg, 'G|')
                    vals = sscanf(msg(3:end), '%f,%d');
                    if length(vals) == 2, S.z_dist = vals(1); S.singular = vals(2); end
                elseif startsWith(msg, 'B|')
                    vals = sscanf(msg(3:end), '%d,%f');
                    if length(vals) == 2
                        S.calib_enabled = (vals(1) == 1); S.calib_rms = vals(2);
                    end
                elseif startsWith(msg, 'D|')
                    parts = split(msg(3:end), ',');
                    if numel(parts) >= 2
                        S.diag_code = str2double(parts{1});
                        S.diag_spd  = str2double(parts{2});
                        if numel(parts) >= 3, S.diag_reason = strjoin(parts(3:end), ','); end
                    end
                end
            end
        catch
        end
        % 延迟统计
        t = toc(S.last_time);
        if t > 0.5
            S.touch_relay_delay = t * 1000 / max(S.packet_count, 1);
            S.packet_count = 0; S.last_time = tic;
        end
    end
```

- [ ] **Step 2: Commit**

```bash
git add Relay_Station/relay_gui.m
git commit -m "feat(gui): protocol parser — all 10 message types (P|/F|/J|/RP|/C|/S|/L|/G|/B|/D|)"
```

---

### Task 10: MATLAB relay_gui — update3DModel (STL + hgtransform)

- [ ] **Step 1: 3D 模型更新函数**

```matlab
    function update3DModel()
        ja = S.joint_angles;
        % 更新 STL 模型 (hgtransform)
        if stlLoaded
            for i = 0:6
                T = fk.linkTransform(ja(1),ja(2),ja(3),ja(4),ja(5),ja(6), i);
                % hgtransform Matrix is column-major 4x4
                linkHg(i+1).Matrix = T;
            end
        else
            % Fallback: 骨架模型 (复用原 computeFK 逻辑)
            joints = fk.robotFk(ja(1),ja(2),ja(3),ja(4),ja(5),ja(6));
            % 清除旧的 fallback 对象 (简化处理: 每帧重绘)
            delete(findobj(ax3d, 'Tag', 'fallback'));
            for i = 1:6
                plot3(ax3d, [joints(i,1) joints(i+1,1)], ...
                           [joints(i,2) joints(i+1,2)], ...
                           [joints(i,3) joints(i+1,3)], ...
                    'Color', [0.25 0.28 0.32], 'LineWidth', 6, 'Tag', 'fallback');
            end
            for i = 2:7
                [sx, sy, sz] = sphere(10);
                r = 6;
                surf(ax3d, sx*r+joints(i,1), sy*r+joints(i,2), sz*r+joints(i,3), ...
                    'FaceColor', [0.30 0.65 1.00], 'EdgeColor', 'none', ...
                    'FaceAlpha', 0.7, 'Tag', 'fallback');
            end
        end

        % Touch 笔可视化
        tp = S.touch_pos;
        if any(tp(1:3) ~= 0)
            [cx, cy, cz] = cylinder([2 1.5], 8);
            cz = cz * 40;
            set(touchPenBody, 'XData', cx+tp(1), 'YData', cy+tp(2), ...
                'ZData', cz+tp(3), 'Visible', 'on');
            [sx, sy, sz] = sphere(12);
            set(touchPenTip, 'XData', sx*4+tp(1), 'YData', sy*4+tp(2), ...
                'ZData', sz*4+tp(3), 'Visible', 'on');
        else
            set(touchPenBody, 'Visible', 'off');
            set(touchPenTip, 'Visible', 'off');
        end

        % 末端标记
        rp = S.robot_pos;
        if any(rp(1:3) ~= 0)
            [sx, sy, sz] = sphere(8);
            set(eeMarkerActual, 'XData', sx*8+rp(1), 'YData', sy*8+rp(2), ...
                'ZData', sz*8+rp(3), 'Visible', 'on');
        else
            set(eeMarkerActual, 'Visible', 'off');
        end
        rt = S.robot_target;
        if any(rt(1:3) ~= 0)
            set(eeMarkerTarget, 'XData', rt(1), 'YData', rt(2), 'ZData', rt(3), 'Visible', 'on');
        else
            set(eeMarkerTarget, 'Visible', 'off');
        end
    end
```

- [ ] **Step 2: Commit**

```bash
git add Relay_Station/relay_gui.m
git commit -m "feat(gui): STL hgtransform FK update + Touch pen + end effector markers"
```

---

### Task 11: MATLAB relay_gui — updateTextPanels (全部文本面板)

- [ ] **Step 1: 指令/反馈日志更新**

```matlab
    function updateTextPanels()
        % -- 指令日志 --
        lines = {};
        for i = 1:50
            idx = mod(S.cmd_idx - i + 50, 50) + 1;
            if ~isempty(S.cmd_log{idx}), lines{end+1} = S.cmd_log{idx}; end
        end
        if isempty(lines), lblCmd.Text = '(waiting for commands...)';
        else, lblCmd.Text = lines; end

        % -- 反馈日志 --
        lines = {};
        for i = 1:50
            idx = mod(S.fb_idx - i + 50, 50) + 1;
            if ~isempty(S.fb_log{idx}), lines{end+1} = S.fb_log{idx}; end
        end
        if isempty(lines), lblFb.Text = '(waiting for feedback...)';
        else, lblFb.Text = lines; end
```

- [ ] **Step 2: 力数据面板更新**

```matlab
        % -- 力数据 --
        fr = S.force_raw; ff = S.force_filt; mm = S.force_moment;
        lblForceRaw.Text = {
            sprintf('Raw:  Fx: %7.2f N  Fy: %7.2f N  Fz: %7.2f N', fr(1), fr(2), fr(3));
            sprintf('      Mx: %7.2f Nm My: %7.2f Nm Mz: %7.2f Nm', mm(1), mm(2), mm(3));
            ''};
        lblForceFilt.Text = {
            sprintf('Filt: Fx: %7.2f N  Fy: %7.2f N  Fz: %7.2f N', ff(1), ff(2), ff(3));
            ''};
        if S.force_stale
            lblForceRaw.Text{3} = '*** FORCE SENSOR OFFLINE ***';
            lblForceRaw.FontColor = [1.0 0.3 0.3];
            lblForceFilt.Text{2} = '*** FORCE SENSOR OFFLINE ***';
            lblForceFilt.FontColor = [1.0 0.3 0.3];
        else
            lblForceRaw.FontColor = clr.text_dim;
            lblForceFilt.FontColor = clr.text_dim;
        end
```

- [ ] **Step 3: Robot State 面板**

```matlab
        % -- Robot State --
        rp = S.robot_pos; rt = S.robot_target; ja = S.joint_angles;
        txActive = any(rt(1:3) ~= 0);
        lblCoord.Text = {
            sprintf('Position (mm):    X: %8.2f  (target: %8.2f)', rp(1), rt(1));
            sprintf('                   Y: %8.2f  (target: %8.2f)', rp(2), rt(2));
            sprintf('                   Z: %8.2f  (target: %8.2f)', rp(3), rt(3));
            sprintf('Orientation (deg): Rx: %7.2f  Ry: %7.2f  Rz: %7.2f', rp(4), rp(5), rp(6));
            '';
            sprintf('Joints (deg):  J1:%7.1f  J2:%7.1f  J3:%7.1f', ja(1:3));
            sprintf('               J4:%7.1f  J5:%7.1f  J6:%7.1f', ja(4:6));
            '';
            sprintf('Force (N):   Fx: %7.2f   Fy: %7.2f   Fz: %7.2f', ff(1), ff(2), ff(3));
            '';
            sprintf('TX: %s', ternary(txActive, 'ACTIVE', 'IDLE'))};
```

- [ ] **Step 4: Safety & Diagnostics 面板**

```matlab
        % -- Safety & Diagnostics --
        stateNames = {'RUNNING', 'WARN', 'DEGRADE', 'FATAL'};
        stateColors = {clr.green, clr.yellow, clr.orange, clr.red};
        st = S.safety_state + 1;
        if st < 1, st = 1; elseif st > 4, st = 4; end

        safetyLines = {};
        safetyLines{1} = sprintf('Safety: %s  |  Speed: %.1fx  |  Alarms: %d', ...
            stateNames{st}, S.safety_speed, S.safety_alarms);
        lblSafety.FontColor = stateColors{st};

        % 关节限位
        [minM, worstJ] = min(S.joint_margins);
        if minM < 15
            safetyLines{2} = sprintf('J%d near limit: %.1f deg margin', worstJ, minM);
            lblSafety.FontColor = clr.orange;
        else
            safetyLines{2} = sprintf('Joints: OK (min margin %.0f deg)', minM);
        end

        % 奇异位形
        if S.singular
            safetyLines{3} = sprintf('Z-axis dist: %.0f mm  !!SINGULAR!!', S.z_dist);
        else
            safetyLines{3} = sprintf('Z-axis dist: %.0f mm', S.z_dist);
        end

        % 标定
        if S.calib_enabled
            safetyLines{4} = sprintf('Calib: RMS=%.2f mm', S.calib_rms);
        else
            safetyLines{4} = 'Calib: not calibrated';
        end

        % 诊断
        if S.diag_code ~= 0
            safetyLines{5} = sprintf('Last Diag: code=%d speed=%.1f %s', ...
                S.diag_code, S.diag_spd, S.diag_reason);
        else
            safetyLines{5} = 'Diagnostics: (no errors)';
        end

        lblSafety.Text = safetyLines;

        % -- 延迟 + 状态 --
        lblDelay.Text = sprintf('Touch->Relay: %.1f ms', S.touch_relay_delay);
        lblState.Text = sprintf('[%s]  Spd: %.1fx', stateNames{st}, S.safety_speed);
        lblState.FontColor = stateColors{st};
    end
```

- [ ] **Step 5: 添加辅助函数 (ternary)**

在 relay_gui.m 文件末尾、`end` 之前添加：
```matlab
    function r = ternary(cond, tVal, fVal)
        if cond, r = tVal; else, r = fVal; end
    end
```

- [ ] **Step 6: Commit**

```bash
git add Relay_Station/relay_gui.m
git commit -m "feat(gui): all text panels — commands, feedback, force, state, safety, diagnostics"
```

---

### Task 12: C++ 删除 HudOverlay

**Files:**
- Delete: `Touch_Client/render/HudOverlay.h`
- Delete: `Touch_Client/render/HudOverlay.cpp`
- Modify: `Touch_Client/main.cpp` (remove #include and drawAll call)

- [ ] **Step 1: 删除文件**

```bash
rm Touch_Client/render/HudOverlay.h
rm Touch_Client/render/HudOverlay.cpp
```

- [ ] **Step 2: main.cpp 清理引用**

移除第 15 行：
```cpp
// #include "render/HudOverlay.h"  ← DELETE THIS LINE
```

在 `display()` 函数中（约第 83 行），移除：
```cpp
    // HudOverlay::drawAll();  ← DELETE THIS LINE
```

保留注释或直接删除。

- [ ] **Step 3: 编译验证**

```bash
cd Touch_Client && build.bat
```

预期编译通过，无 `HudOverlay` 相关错误。

- [ ] **Step 4: Commit**

```bash
git rm Touch_Client/render/HudOverlay.h Touch_Client/render/HudOverlay.cpp
git add Touch_Client/main.cpp
git commit -m "refactor(render): remove HudOverlay — all 2D UI moved to MATLAB GUI"
```

---

### Task 13: 联调与验证

- [ ] **Step 1: 启动 MATLAB GUI 测试 (--no-robot 模式)**

```bash
# Terminal 1: 启动 C++ (无机械臂模式)
cd D:/Projects/Touch/Touch_Client/x64/Release
./Touch_Client.exe --no-robot --no-touch
```

```matlab
% MATLAB: 启动 relay_gui
cd D:/Projects/Touch/Relay_Station
relay_gui
```

验证：
- 顶部状态栏显示 "C++ Client: CONNECTED"
- 3D 视图渲染 STL 模型（或 fallback 骨架）
- 窗口缩放时面板自适应

- [ ] **Step 2: 验证协议消息**

在 MATLAB 命令窗口监控消息接收：
```matlab
% 检查状态字段是否更新
% S| 消息应每 200ms 更新 S.safety_state
% L| 消息应每 500ms 更新 S.joint_margins
% G| 消息应每 200ms 更新 S.z_dist
```

- [ ] **Step 3: 实机联调 (需机械臂连接)**

```bash
# Terminal 1: C++ (完整模式)
cd D:/Projects/Touch/Touch_Client/x64/Release
./Touch_Client.exe
```

验证：
- 所有面板正确显示实时数据
- STL 模型随关节角度正确运动
- 安全状态变化时面板颜色正确切换
- 诊断事件触发时显示正确

- [ ] **Step 4: 确认旧 relay_gui.m 已备份**

```bash
git status  # 确认 relay_gui.m 已修改
git log --oneline -5  # 确认提交链完整
```

- [ ] **Step 5: Commit (如有微调)**

```bash
git add -A
git commit -m "chore: integration fixes after MATLAB GUI migration"
```
