# Touch-Dobot 力反馈系统：Touch 端代码重构设计

> 日期：2026-07-20 | 状态：待审批

## 一、背景与目标

### 当前状态
- Touch 端 C++ 代码经历 19 个迭代版本（test1 → test8.3）
- 最新版本 test8.3（~2249 行单文件）已实现 Touch → 中继站 → 机械臂的基本运动控制
- 存在**秒级跟随延迟**，代码高度耦合

### 重构目标
1. 模块化拆分：单文件 → 清晰模块结构
2. 时延优化：秒级 → <100ms
3. 历史代码清理：删除旧版本，新建 `Touch_Client/` 统一项目
4. 协议文档化：整理 Touch↔中继站通信协议

### 范围与边界
- **本次**：Touch 端 C++ 代码重构 + 时延优化 + 协议文档
- **后续**：中继站 MATLAB 代码、力反馈闭环

---

## 二、系统架构

### 整体数据流

```
Touch设备 ──1kHz──▶ PC-A (C++) ──TCP:8888──▶ PC-B (MATLAB中继站) ──TCP:29999/30003──▶ Dobot CR3
                      ▲                              │
                      │        反馈透传               │
                      └──────────────────────────────┘
```

### Touch 端内部数据流

```
HapticCallback (1kHz)
  → 读取原始坐标 → 坐标系转换 → 计算相对位移
  → 阈值过滤 → push 发送队列 → 3D 渲染更新

SenderThread (独立线程)
  → 队列取数据 → 安全边界检查 → 构造 ServoP 指令
  → 包裹中继协议 → TCP send（异步，不等待反馈）

TcpClientThread (独立线程)
  → 事件驱动 I/O → 接收反馈 → 解析 → 处理 PING/PONG

定时巡检 (300ms 间隔)
  → 查询 RobotMode → 检测报警 → 自动恢复
```

### 连接生命周期

```
启动 → InitTouch → InitOpenGL → ConnectRelay
  → ClearAlarm → EnableRobot → SetCP(100) → GetPose(基准)
  → 主循环(Render + Haptic + TcpEvent + Sender)
  → 关闭 → DisableRobot → CloseRelay → Cleanup
```

---

## 三、模块架构

### 目录结构

```
Codes/Touch_Client/
├── main.cpp                          # 入口，组装所有模块
├── config/
│   └── Config.h                      # 集中配置（IP/端口/阈值/颜色/安全边界）
├── haptic/
│   ├── HapticDevice.h/.cpp           # 设备初始化/关闭/回调句柄
│   └── HapticCallback.h/.cpp         # 1kHz 触觉回调（坐标读取/转换/入队）
├── render/
│   ├── SceneRenderer.h/.cpp          # 3D 场景（坐标轴/地板/光标/轨迹）
│   ├── CoordinateTable.h/.cpp        # 实时坐标表格 UI
│   ├── StatusDisplay.h/.cpp          # TCP 连接状态栏
│   └── LogoManager.h/.cpp            # Logo 加载与渲染
├── network/
│   ├── TcpClient.h/.cpp              # TCP 连接管理（连接/事件循环/断开）
│   ├── RelayProtocol.h/.cpp          # 中继站协议（打包/解包/反馈解析）
│   └── PongHandler.h/.cpp            # PING/PONG 时延测量
├── robot/
│   ├── RobotController.h/.cpp        # 机械臂控制入口（使能/报警/运动/状态）
│   ├── CommandBuilder.h/.cpp         # 指令构造器（ServoP/MovL等）
│   └── SafetyBoundary.h/.cpp         # 安全边界检查与钳位
├── core/
│   ├── AppState.h/.cpp               # 全局共享状态（线程安全）
│   ├── SenderQueue.h/.cpp            # 发送队列 + 独立发送线程
│   └── CoordinateTransform.h/.cpp    # 坐标系转换工具
└── utils/
    ├── RenderUtils.h/.cpp            # 渲染辅助函数
    └── MathUtils.h                   # clamp 等数学工具
```

### 模块依赖规则

- **Config** → 所有模块只读引用，不依赖任何模块
- **utils/** → 零依赖，纯工具函数
- **core/AppState** → 被上层模块共享，提供线程安全的状态读写
- **haptic/** → 依赖 core（AppState, SenderQueue, CoordinateTransform）
- **network/** → 依赖 core（AppState）, robot（CommandBuilder）
- **robot/** → 依赖 core（AppState）, network（RelayProtocol）
- **render/** → 依赖 core（AppState）, utils

---

## 四、坐标映射

### 映射关系（继承现有已验证逻辑，不做修改）

```cpp
// Touch 原始 (devicePos) → 机械臂右手系 (table)
adjustedPosTable.x =  devicePos[0];    // X → X
adjustedPosTable.y = -devicePos[2];    // Z(反转) → Y
adjustedPosTable.z =  devicePos[1];    // Y → Z

// 计算相对位移（按下按钮时刻的基准点）
delta = adjustedPosTable - basePoint;

// 叠加到机械臂用户坐标系（基座原点）的绝对基准位置
target = robotBase + delta;

// 安全边界钳位
target = clamp(target, SAFE_MIN, SAFE_MAX);
```

### 坐标系选择

- **机械臂坐标系**：用户坐标系（基座原点），`GetPose()` 默认返回此系坐标
- **运动接口**：`ServoP(X, Y, Z, Rx, Ry, Rz)` — 笛卡尔空间动态跟随，不排队

### 安全边界

- 边界参数在 `Config.h` 中可配置：`SAFE_X_MIN/MAX`, `SAFE_Y_MIN/MAX`, `SAFE_Z_MIN/MAX`
- 超限坐标钳位到最近边界值，输出警告日志
- 速度比例随距边界衰减：进入边界 20% 缓冲区时线性降速

### 姿态处理

- **当前阶段**：固定姿态，沿用 `GetPose()` 获取的 Rx/Ry/Rz，机械臂末端保持小臂竖直向下
- **预留接口**：`HapticCallback` 中预留以下读取点供后续迭代使用：
  - `hdGetDoublev(HD_CURRENT_TRANSFORM, ...)` — 读取 4×4 变换矩阵（含旋转）
  - `HD_DEVICE_BUTTON_2` — 第二个按钮（预留为姿态控制开关）
  - `AppState` 中预留目标 `rx/ry/rz` 字段
- **后续**：实现 Touch 笔平衡环旋转 → 机械臂末端姿态的增量映射

---

## 五、时延优化

### 优化点汇总

| # | 问题 | 对策 | 预期收益 |
|---|------|------|---------|
| 1 | `MovL` 队列执行 | 切换为 `ServoP`（直接覆盖，不排队） | **核心修复** |
| 2 | `sendToRelay` 同步等待反馈 | 发送与反馈解耦：send 后立即返回，TcpClient 异步处理反馈 | 消除阻塞 |
| 3 | `sendCoordinates` 每次调 `updateAlarmStatus` + `Sleep(150)` | 报警检查移出热路径，改为独立定时器每 300ms 巡检 | 消除 150ms+ 固定延迟 |
| 4 | 接收缓冲区 1024 字节 | 增大到 64KB | 减少数据丢失 |
| 5 | 发送队列无背压控制 | 容量上限 5，满时丢弃旧数据（保留最新） | 避免指令堆积 |

### 优化后热路径（每次坐标发送）

```
1. format ServoP cmd          ~0.01ms
2. wrap relay protocol        ~0.01ms
3. TCP send                   ~0.1ms
4. return (不等待)            ~0ms
──────────────────────────────────
   总计:                      ~0.12ms
```

### 预期指标

| 指标 | 优化前 | 优化后 |
|------|--------|--------|
| 单指令处理周期 | 150~500ms | ~0.12ms |
| 跟随延迟 | 秒级 | <100ms |
| 发送吞吐 | ~3 条/秒 | 理论上 8000+/秒，实际节流到 ~33Hz |

---

## 六、通信协议

### Touch ↔ 中继站协议

| 方向 | 格式 | 示例 |
|------|------|------|
| Touch → 中继站 | `{端口}\|{指令}` | `30003\|ServoP(200.00,0.00,150.00,0.00,0.00,0.00)` |
| | | `29999\|EnableRobot(0.5,0,0,0)` |
| | | `29999\|RobotMode()` |
| 中继站 → Touch | 透传机械臂原始反馈 | `0,{},ServoP(...);` |
| 时延探测 | `PING\|{序号}` | `PING\|1234` |
| 时延响应 | `PONG\|{序号}` | `PONG\|1234` |
| 异常通知 | `ROBOT_ARM_CLOSED` | — |

### 反馈判断

- 机械臂返回格式：`ErrorID,{data},CommandName();`
- `ErrorID == 0` → 成功
- `ErrorID != 0` → 失败，记录日志

---

## 七、清理计划

### 删除项

- `Codes/test1/` ~ `Codes/test7.3/` 全部旧版本源代码
- `Codes/test8.1/` ~ `Codes/test8.3/` 源代码（功能已迁移到新模块）
- `Codes/Debug/` 所有编译产物（.exe, .ilk, .pdb）
- `Codes/.vs/` Visual Studio 缓存

### 迁移项

- `pics/NINELAB.png` → `Touch_Client/pics/`
- `StbImage/stb_image.h` → `Touch_Client/vendor/stb_image.h`

### 新建项

- `Codes/Touch_Client/` — 重构后的唯一 Touch 端项目
- `Docs/protocol/Touch-Relay-Protocol.md` — 通信协议规范文档

---

## 八、文件清单（待实现）

| 文件 | 职责 | 预估行数 |
|------|------|---------|
| `Config.h` | 所有可配置常量 | ~80 |
| `MathUtils.h` | clamp 等模板工具 | ~15 |
| `CoordinateTransform.h/.cpp` | 坐标转换函数 | ~40 |
| `AppState.h/.cpp` | 全局状态 + 线程安全访问 | ~60 |
| `SenderQueue.h/.cpp` | 发送队列 + 发送线程 | ~120 |
| `HapticDevice.h/.cpp` | Touch 设备管理 | ~80 |
| `HapticCallback.h/.cpp` | 1kHz 触觉回调 | ~180 |
| `TcpClient.h/.cpp` | TCP 连接 + 事件循环 | ~250 |
| `RelayProtocol.h/.cpp` | 协议打包/解包 | ~60 |
| `PongHandler.h/.cpp` | PING/PONG 处理 | ~40 |
| `RobotController.h/.cpp` | 机械臂控制入口 | ~200 |
| `CommandBuilder.h/.cpp` | 指令构造 | ~50 |
| `SafetyBoundary.h/.cpp` | 安全边界检查 | ~40 |
| `SceneRenderer.h/.cpp` | 3D 场景渲染 | ~300 |
| `CoordinateTable.h/.cpp` | 坐标表格 UI | ~150 |
| `StatusDisplay.h/.cpp` | 状态栏 UI | ~100 |
| `LogoManager.h/.cpp` | Logo 渲染 | ~80 |
| `RenderUtils.h/.cpp` | 渲染工具 | ~60 |
| `main.cpp` | 入口 + GLUT 主循环 | ~180 |
| **合计** | | **~2085** |
