# MATLAB 全可视化重构 — 设计文档

**日期:** 2026-07-26
**状态:** 设计完成，待审核

---

## 1. 动机与目标

### 问题

当前 C++ Touch_Client 使用 GLUT bitmap 字体 (`GLUT_BITMAP_8_BY_13` 等) 通过 `HudOverlay.cpp` 手绘全部 2D UI 面板。存在：

- 文字与板块边界不对应、重叠
- 文字被板块背景遮挡（z-order 问题）
- 固定分辨率点阵字体，无抗锯齿，可读性差
- 手动像素坐标定位，调整布局困难

### 目标

将**全部可视化职责**从 C++ 迁移到 MATLAB `relay_gui.m`， C++ 仅保留实时控制逻辑。

```
Before:                          After:
┌──────────────────────┐         ┌──────────────────────┐
│ C++: 控制 + 全部 UI   │         │ C++: 纯控制            │
│ MATLAB: 被动可视化     │         │ MATLAB: 全部可视化     │
└──────────────────────┘         └──────────────────────┘
```

---

## 2. 架构

### 组件边界

```
C++ Touch_Client                         MATLAB Relay_Station
══════════════════                       ═══════════════════════
Touch 力反馈设备 (OpenHaptics)           ✅ STL 数字孪生 (hgtransform)
机器人 TCP (29999/30003/30004)           ✅ 指令 / 反馈日志
IK → ServoP 运动控制                    ✅ 力传感器面板 (原始 + 滤波)
力反馈管线 (Butterworth)                 ✅ 安全状态 / 诊断事件
安全状态机 (OK→WARN→DEGRADE→FATAL)      ✅ 关节限位余量 / 奇异警告
标定求解 (Kabsch-Umeyama)               ✅ 标定状态 / 报警计数
看门狗 + 心跳 + NaN 防护                 ✅ 延迟统计

保留: 精简 3D 视图 (仅 STL + 地面网格)   TCP Server (:8888)
删除: HudOverlay (全部 2D 文本面板)      uigridlayout 响应式布局
新增: S|/D|/L|/G|/B| 协议消息发送
```

### 数据流

```
C++ RelayCore                  TCP :8888              MATLAB relay_gui
══════════════                 ════════              ════════════════
sendRelayUpdate("S|...") ──────────────────────────→ processNetworkData()
sendDiagnostic("D|...") ──────────────────────────→ updateSafetyPanel()
sendJointMargins("L|...") ────────────────────────→ updateSafetyPanel()
sendSingularity("G|...") ─────────────────────────→ updateSafetyPanel()
sendCalibStatus("B|...") ─────────────────────────→ updateSafetyPanel()
(现有 P|/F|/J|/RP| 保持不变) ────────────────────→  (现有处理不变)
```

---

## 3. TCP 协议扩展

所有消息以 `\n` 分隔，MATLAB `readline()` 逐行解析。

| 标识 | 格式 | 说明 | 发送频率 |
|------|------|------|----------|
| `P\|` | `P\|x,y,z,rx,ry,rz` | Touch 位姿 (保持不变) | ~30Hz |
| `F\|` | `F\|fx,fy,fz,mx,my,mz,stale` | 力传感器数据 (保持不变) | ~30Hz |
| `J\|` | `J\|j1,j2,j3,j4,j5,j6` | 关节角度 度 (保持不变) | ~5Hz |
| `RP\|` | `RP\|x,y,z,rx,ry,rz` | 机械臂实际位姿 (保持不变) | ~5Hz |
| `C\|` | `C\|cmd_text` | 指令日志 (保持不变) | 事件触发 |
| **`S\|`** | `S\|state,speedFactor,alarmCount` | **状态机**: state=OK(0)/WARN(1)/DEGRADE(2)/FATAL(3), speedFactor=0.0~1.0, alarmCount=报警累计 | 200ms |
| **`D\|`** | `D\|errorCode,speedFactor,reason` | **诊断事件**: errorCode=错误码, speedFactor=降速因子, reason=原因字符串 | 事件触发 |
| **`L\|`** | `L\|m1,m2,m3,m4,m5,m6` | **关节限位余量**: 每个关节距最近限位的角度(°) | 500ms |
| **`G\|`** | `G\|r_xy,singular_flag` | **奇异位形**: r_xy=末端距Z轴距离(mm), singular_flag=0/1 | 200ms |
| **`B\|`** | `B\|enabled,rmsError` | **标定状态**: enabled=0/1, rmsError=RMS误差(mm, -1=未启用) | 状态变化 |

### 通知协议

`D|` 消息仅在诊断事件发生时发送（状态机状态变化或新错误产生），而非定时轮询，避免冗余传输。

---

## 4. MATLAB GUI 设计

### 4.1 技术方案

- **布局**: `uigridlayout` (R2018b+)，自动响应窗口缩放
- **字体**: `Consolas` 等宽，TrueType 矢量渲染
- **3D 渲染**: `uiaxes` + `patch` (STL 网格) + `hgtransform` (FK 关节变换)
- **更新驱动**: `timer('Period', 0.05, 'ExecutionMode', 'fixedRate')` @ 20Hz

### 4.2 布局结构

```
uigridlayout(4行 × 3列, 窗口 1400×850, 最小 1000×600):

Row 1: 顶部状态栏 [固定 30px, 跨 3 列]
  └ 标题 | Touch→Relay 延迟 | IP | 状态灯

Row 2: 左栏 [Col 1, Row 2]          中栏 [Col 2, Row 2]        右栏 [Col 3, Row 2-4]
  ├ 指令日志 (Commands)              ├ 原始力 (Force Raw)        └ 3D 数字孪生 (STL)
  └ ...持续占据 Row 2...              └ ...占据 Row 2...            Row 2 权重 3

Row 3: 左栏 [Col 1, Row 3]          中栏 [Col 3, Row 3]        
  └ 反馈日志 (Feedback)              └ 滤波力 (Force Filt)       右栏 [Col 3, Row 3]
                                                               ├ Robot State (位姿 + 关节)
Row 4: (左栏/中栏无内容 Row 4)        (空)                        └ 权重 1
                                                               右栏 [Col 3, Row 4]
                                                               └ Safety & Diagnostics
                                                                  权重 1
```

列宽权重: `[1, 1, 1.6]`
行高权重: `[fit, 2, 2, 1, 1]` (Row 1 固定; Row 2-3 均分左中; Row 4 占右栏底部)

### 4.3 颜色主题

```matlab
bg_dark  = [0.10 0.12 0.16];    % 窗口背景
bg_panel = [0.08 0.10 0.14];    % 面板背景
border   = [0.20 0.30 0.50];    % 面板边框
text_on  = [0.90 0.94 1.00];    % 标题文字
text_dim = [0.55 0.58 0.62];    % 副文本
green    = [0.20 0.80 0.40];    % 正常/在线
red      = [0.95 0.25 0.25];    % 错误/离线
blue     = [0.30 0.65 1.00];    % 强调
orange   = [1.00 0.60 0.15];    % 警告
yellow   = [1.00 0.85 0.20];    % 降级
```

### 4.4 缩放行为

- 窗口最小尺寸: 1000×600 (由 `uigridlayout` 列/行最小尺寸约束)
- 三列按权重等比缩放
- 3D 视图自动适配 `uiaxes` 父容器尺寸
- 字体大小固定 (10-11px)，不随窗口缩放 — 保持可读性

### 4.5 面板详情

#### 顶部状态栏 (Row 1)
- 系统标题 "Touch-Dobot Relay Station" (蓝色, 粗体)
- Touch→Relay 延迟 (ms)
- Robot IP 地址
- C++ 客户端连接状态 (CONNECTED/OFFLINE, 绿色/红色)
- 状态机状态标签 [RUNNING]/[DEGRADED]/[FATAL] (颜色对应状态)
- 速度因子 (Speed: 1.0x)

#### 左栏上半: 指令日志 (Commands)
- 标题: "Touch → Robot (Commands)"
- 滚动环形缓冲 (50 条)
- 最新指令亮绿色，逐条变暗
- 空态: "(waiting for commands...)" 灰色

#### 左栏下半: 反馈日志 (Feedback)
- 标题: "Robot → Relay (Feedback)"
- 滚动环形缓冲 (50 条)
- 错误码非零显示红色
- 空态: "(waiting for feedback...)" 灰色

#### 中栏上半: 力传感器原始值 (Force Raw)
- 标题: "Force Sensor (Raw · 30004)"
- Fx/Fy/Fz 力和 Mx/My/Mz 力矩
- 传感器离线时红色警告 "*** NO DATA ***"

#### 中栏下半: 力反馈输出值 (Force Filtered)
- 标题: "Force Output (Filtered → Touch)"
- 滤波后力 + 触觉输出力
- 离线时警告 "Touch force output: ZERO (safety)"

#### 右栏: 3D 数字孪生
- CR3 机器人 STL 网格模型 (7 个部件)
- FK 驱动 `hgtransform` 层级变换
- 地面网格 + 坐标系箭头 + 安全边界线框 (保留现有)
- Touch 笔末端位置 (灰色圆柱 + 红球)
- 等距视角, 旋转/缩放/平移 (MATLAB 内置交互)

#### 右栏: Robot State
- 位姿: 实际 vs 目标 (X/Y/Z) 对比
- 姿态: Rx/Ry/Rz
- 关节角度: J1~J6
- 力数据摘要: Fx/Fy/Fz
- TX 传输状态: ACTIVE/IDLE

#### 右栏: Safety & Diagnostics
- 安全状态灯: OK (绿) / WARN (黄) / REJECT (红)
- 速度因子显示
- 报警历史计数
- Z 轴距离 + 奇异位形警告
- 关节限位余量 (最近关节 + 角度)
- 标定状态 (未标定/已标定 + RMS Error)
- 最新诊断事件 (错误码 + 原因)

---

## 5. STL 渲染

### 5.1 加载策略

- 启动时一次性加载 7 个 STL 文件 (~4.6MB 合计)
- 纯 MATLAB 二进制解析 (无需工具箱)
- 存储在 `+stl/loadBinaryStl.m` 作为独立模块

### 5.2 渲染策略

- 每个 link 创建 `patch('Faces', F, 'Vertices', V)` + `hgtransform` 父级
- 每帧只更新 `hgtransform.Matrix` (7 次 4×4 矩阵运算)，不触碰顶点数据
- 模型尺寸: STL/URDF 单位为米，场景单位为毫米 → 统一缩放 1000×

### 5.3 FK 参数

与 C++ 端 `RobotModel.cpp` 及 MATLAB 现有 `computeFK()` 保持一致:

| 参数 | 值 | 来源 |
|------|-----|------|
| J1_Z | 136.0 mm | URDF, 实机标定 |
| J3_X | -274.0 mm | URDF |
| J4_X | -230.0 mm | URDF |
| J4_Z | 128.3 mm | URDF |
| J5_Y | -116.0 mm | URDF |
| J6_Y | 105.0 mm | URDF |

---

## 6. C++ 改动

### 6.1 删除文件
- `Touch_Client/render/HudOverlay.h`
- `Touch_Client/render/HudOverlay.cpp`

### 6.2 修改文件

**`main.cpp`:**
- 移除 `#include "render/HudOverlay.h"`
- `display()` 中移除 `HudOverlay::drawAll()`
- 新增定时器: `safetyStatusTimer` (200ms), `jointMarginTimer` (500ms)

**`relay/RelayCore.h`:**
- 新增声明:
  - `void sendSafetyStatus();`
  - `void sendDiagnostic();`
  - `void sendJointMargins();`
  - `void sendSingularity();`
  - `void sendCalibStatus();`

**`relay/RelayCore.cpp`:**
- 新增实现: 组装 S|/D|/L|/G|/B| 协议字符串 → `sendRelayUpdate()`

### 6.3 保留的 3D 视图

C++ 端仅渲染:
- STL 模型 (7 个 link FK 层级)
- 地面网格 + 坐标系
- 安全边界线框

不渲染任何文字、面板、HUD。此视图作为无 MATLAB 时的独立调试备用。

---

## 7. 文件清单

| 文件 | 操作 | 说明 |
|------|------|------|
| `Relay_Station/relay_gui.m` | **重写** | uigridlayout + STL + 全面板 |
| `Relay_Station/relay_config.m` | 保持 | 无需改动 |
| `Relay_Station/+stl/loadBinaryStl.m` | **新增** | STL 二进制解析 |
| `Relay_Station/+fk/robotFk.m` | **新增** | FK 计算 (从 relay_gui 提取) |
| `Touch_Client/render/HudOverlay.h` | **删除** | |
| `Touch_Client/render/HudOverlay.cpp` | **删除** | |
| `Touch_Client/main.cpp` | **修改** | 移除 HUD + 新增定时器 |
| `Touch_Client/relay/RelayCore.h` | **修改** | 新增 5 个发送函数声明 |
| `Touch_Client/relay/RelayCore.cpp` | **修改** | 新增协议消息发送实现 |
| `Touch_Client/render/SceneRenderer.h` | 评估 | 可能需要移除 HUD 相关引用 |
| `Touch_Client/render/SceneRenderer.cpp` | 评估 | 同上 |

---

## 8. 风险与缓解

| 风险 | 缓解 |
|------|------|
| MATLAB `patch` + `hgtransform` 渲染性能不足 | 预加载 mesh, 不每帧复制顶点; 目标 20Hz, 实测可轻松达到 |
| 大量协议消息增加网络开销 | S\|/G\|/L\| 消息体 ~30-60 bytes, 5 种新消息总带宽 < 1KB/s |
| 窗口缩放时 3D 视图重绘卡顿 | `drawnow limitrate` 限制重绘频率; uiaxes 自动处理 resize |
| STL 文件路径变更导致加载失败 | 使用相对路径 + 启动时检查 + fallback 到骨架模型 |

---

## 9. 实现顺序

1. MATLAB: STL 加载模块 (`+stl/loadBinaryStl.m`)
2. MATLAB: FK 模块 (`+fk/robotFk.m`)  
3. MATLAB: 重写 `relay_gui.m` (uigridlayout + 全面板 + STL 渲染)
4. C++: 新增协议消息发送 (S|/D|/L|/G|/B|)
5. C++: 删除 `HudOverlay` + 清理 `main.cpp`
6. 联调测试
