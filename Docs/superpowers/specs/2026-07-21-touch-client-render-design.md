# Touch_Client v3.0：单机数字孪生系统设计

> 状态：待审批 | 基于：v2.0 模块化重构

## 一、目标与范围

将当前 v2.0 双进程架构（C++ Touch_Client + MATLAB 中继站）合并为**单进程 C++ 程序**，增加**实时 3D 数字孪生渲染**。

### 核心变更

| 维度 | v2.0 (当前) | v3.0 (目标) |
|------|------------|------------|
| 进程数 | 2 (C++ exe + MATLAB) | 1 (C++ exe) |
| 网络跳数 | Touch → 中继站 → 机械臂 (2 跳) | Touch → Relay 层 → 机械臂 (0 网络跳) |
| 可视化 | 控制台文字输出 | 3D 数字孪生 (freeglut) |
| 中继逻辑 | MATLAB 独立进程 | C++ Relay 层 (进程内类) |
| 机械臂模型 | 无 | STL 网格 + 实时关节驱动 |

### 不做的

- 力反馈闭环（后续迭代）
- Touch 设备 3D 外形模型（只渲染光标球 + 轨迹）
- 姿态控制（保持固定姿态）

---

## 二、架构：三层解耦 + 单进程

```
┌──────────────────────────────────────────────────────────────┐
│                     Touch_Client.exe                          │
│                                                              │
│  ┌────────────┐     ┌──────────────┐     ┌──────────────┐   │
│  │  Touch 层   │────▶│   Relay 层   │────▶│   Robot 层   │───┼──▶ TCP:29999
│  │ (haptic/)   │     │  (relay/)    │     │  (robot/)    │───┼──▶ TCP:30003
│  │             │◀────│              │◀────│              │   │
│  └────────────┘     └──────┬───────┘     └──────────────┘   │
│                            │                                 │
│  ┌────────────┐           │                                 │
│  │  Render 层  │◀──────────┘                                 │
│  │ (render/)   │  读取 Relay 层的状态数据渲染 3D 场景          │
│  └────────────┘                                              │
└──────────────────────────────────────────────────────────────┘

规则：
- Touch 层和 Robot 层互不引用，所有通信经 Relay 层
- Relay 层持有所有数据处理管道（坐标转换/安全边界/协议适配/反馈解析）
- Render 层只读 Relay/AppState，不写
```

### 三层职责

**Touch 层** (`haptic/`) — 只跟设备打交道
- 初始化/关闭 Touch 设备
- 1kHz 回调读取原始坐标和按钮状态
- 将原始数据推入 Relay 层

**Relay 层** (`relay/`) — 所有业务逻辑
- ProtocolAdapter: Touch 语义 → Dobot TCP/IP 协议
- CoordinateTransform: 坐标系转换（从 core/ 迁入）
- SafetyBoundary: 安全边界检查（从 robot/ 迁入）
- FeedbackParser: 解析机械臂原始反馈，提取位姿/力/错误码
- IExtension: 扩展插件接口（预留力滤波、夹具控制等）

**Robot 层** (`robot/`) — 只跟机械臂通信
- RobotConnection: 双 TCP 端口管理（29999 + 30003）
- CommandBuilder: Dobot 指令字符串构造

**Render 层** (`render/`) — 只读，纯展示
- SceneRenderer: 3D 场景组装（地面/坐标轴/光标/轨迹/机器人）
- RobotModel: 机械臂运动学模型，驱动各连杆 STL 渲染
- HudOverlay: 2D 文字叠加（坐标表/指令/延迟/状态）
- StlLoader: STL 文件 → StlMesh 三角形网格

---

## 三、数据流

### 正向（Touch → 机械臂）

```
HapticCallback (1kHz)
  → 读取 devicePos[3] + 按钮状态
  → RelayCore.sendPosition(devicePos, buttonState)
      → CoordinateTransform: Touch 系 → 机械臂右手系
      → 计算 delta = current - basePoint
      → SafetyBoundary: 钳位 + 速度衰减
      → CommandBuilder: 构造 ServoP(cmd)
      → RobotConnection::sendMotion(cmd)
          → TCP:30003 send()
```

### 反向（机械臂 → Touch + 渲染）

```
RobotConnection 接收线程（独立）
  → TCP:30003 recv() → 机械臂反馈字符串
  → RelayCore.onRobotFeedback(raw)
      → FeedbackParser: 提取 ErrorID / 位姿 / 力数据
      → AppState: 更新 robotActualPos, robotTargetPos, latency
      → (预留) IExtension::onFeedback → ForceFilter
      → HapticDevice: (预留) 力渲染输出
```

### 状态查询（驱动 3D 模型）

```
定时器线程 (每 100ms)
  → RobotConnection::sendEnable("GetPose()")
  → FeedbackParser: 解析返回的 {x,y,z,rx,ry,rz}
  → AppState: 更新 robotActualPose → RobotModel 重绘
```

---

## 四、3D 数字孪生渲染

### 场景布局

```
┌─────────────────────────────────────────────┐
│  左上: 坐标数据面板                          │
│    Touch Raw:    ( 12.5, -8.3, 45.2)       │
│    Touch Mapped: ( 12.5, 45.2, -8.3)       │
│    Robot Target: (250.0, 30.0, 150.0)      │
│    Robot Actual: (248.7, 31.2, 149.5)      │
│    Delta:         (  1.3, -1.2,   0.5)     │
│                                            │
│  右上: 状态栏                               │
│    TCP 29999: CONNECTED                    │
│    TCP 30003: CONNECTED                    │
│    Robot Mode: ENABLED                     │
│    Latency: 3.2ms                          │
│                                            │
│  底部中央: 最后发送指令                      │
│    Last CMD: ServoP(250.0,30.0,150.0,...)  │
│                                            │
│  3D 场景 (主体)                             │
│    ┌─ 坐标系原点 (三色轴)                    │
│    ├─ 半透明地面网格                         │
│    ├─ Dobot CR3 机械臂模型 (STL 实体)       │
│    │   └─ 从底座到末端 6 个连杆              │
│    ├─ 目标位置标记 (红色线框)                │
│    ├─ 实际位置标记 (绿色实体)                │
│    ├─ Touch 光标球 (白色发光)                │
│    └─ 运动轨迹线 (青色)                      │
└─────────────────────────────────────────────┘
```

### 机械臂模型驱动

```
RobotModel 类:
  - 加载 7 个 STL (base + Link1~Link6)
  - 渲染时按运动链层级推进:
      glPushMatrix()
        drawBase()                           // 底座，世界坐标固定
        glRotatef(joint1, 0,0,1)             // J1: 绕 Z 旋转
        drawLink(1)
        glTranslatef(0, 0, link1_height)
        glRotatef(joint2, 0,1,0)             // J2: 绕 Y 旋转
        drawLink(2)
        // ...依次到 Link6
      glPopMatrix()
  - joint1~joint6 从 FeedbackParser 解析的 GetPose 数据更新
```

### 颜色约定

| 元素 | 颜色 |
|------|------|
| 机械臂实体 | 深灰金属色 (0.25, 0.28, 0.32) |
| 目标位置标记 | 红色半透明 (1.0, 0.3, 0.3, 0.5) |
| 实际位置标记 | 绿色实体 (0.3, 0.9, 0.4) |
| Touch 光标球 | 白色发光 (1.0, 1.0, 1.0, 0.9) |
| 轨迹线 | 青色 (0.25, 0.85, 1.0) |
| 安全边界 | 黄色线框 (1.0, 0.78, 0.28) |
| 地面网格 | 深灰半透明 |

---

## 五、STL 模型获取

### 来源

movensys 开源 ROS 2 项目 `movensys_manipulator_description` 包含 Dobot CR3 完整 STL：
- 仓库：`https://github.com/movensys/movensys_manipulator_description`
- 文件：`meshes/cr3a/` 目录下各连杆 STL + 底座/桌面/夹具
- 格式：Binary STL（三角形网格）

### 获取方式

```bash
# 方式一：sparse checkout（只拉模型文件，不拉整个 ROS 仓库）
git clone --depth 1 --filter=blob:none --sparse \
  https://github.com/movensys/movensys_manipulator_description.git cr3_models
cd cr3_models
git sparse-checkout set meshes/cr3a

# 方式二：直接下载（如果有 release 或 raw URL）
# 从 GitHub raw URL 逐个下载 STL 文件到 Codes/Touch_Client/models/
```

模型文件放入 `Codes/Touch_Client/models/cr3/` 目录。

### 备用方案

若无法获取 movensys STL，使用几何体近似：
- 底座：圆柱体 (gluCylinder)
- 大臂/小臂：长方体 (glutSolidCube 缩放)
- 末端：球体 (glutSolidSphere)
- 所有渲染通过 `IMeshRenderer` 接口，后续可替换为 STL 实现

---

## 六、模块文件清单

```
Codes/Touch_Client/
├── main.cpp                           # 改: 双端口直连 + GLUT 主循环
├── config/Config.h                    # 改: RELAY_PORT → ROBOT_IP
├── touch/                             # (原 haptic/，改名)
│   ├── HapticDevice.h/.cpp            # 保留，微调
│   └── HapticCallback.h/.cpp          # 改: 不再直连 TcpClient，调 RelayCore
├── relay/                             # 新增
│   ├── RelayCore.h/.cpp               # 数据流编排中枢
│   ├── ProtocolAdapter.h/.cpp         # Touch 语义 → Dobot 协议
│   ├── FeedbackParser.h/.cpp          # 机械臂反馈解析
│   └── IExtension.h                   # 扩展插件接口
├── robot/                             # 重构
│   ├── RobotConnection.h/.cpp         # 替代 TcpClient，双端口管理
│   ├── CommandBuilder.h               # 保留 (从 robot/ 迁入或原地)
│   ├── SafetyBoundary.h               # (迁到 relay/，此文件删除)
│   └── CoordinateTransform.h          # (迁到 relay/，从 core/ 搬)
├── render/                            # 全部新增
│   ├── StlMesh.h                      # 三角形网格数据结构
│   ├── StlLoader.h/.cpp               # STL 解析 (ASCII + Binary)
│   ├── RobotModel.h/.cpp              # 机械臂运动学 + 连杆渲染
│   ├── SceneRenderer.h/.cpp           # 3D 场景组装
│   └── HudOverlay.h/.cpp              # 2D HUD 文字叠加
├── core/
│   ├── AppState.h/.cpp                # 改: +机器人双连接状态 + 实际位姿
│   ├── SenderQueue.h/.cpp             # 保留
│   └── MathUtils.h                    # (原 utils/，合并到 core)
├── models/cr3/                        # STL 模型文件
│   ├── base.stl
│   ├── link1.stl ~ link6.stl
│   └── ...
└── vendor/stb_image.h                 # 保留 (Logo 用的)
```

### 删除的模块

| 文件 | 原因 |
|------|------|
| `network/RelayProtocol.h/.cpp` | 被 `ProtocolAdapter` 替代（不再需要 `端口\|指令` 封装） |
| `network/PongHandler.h/.cpp` | 无中继站，不需要 PING/PONG |
| `network/TcpClient.h/.cpp` | 被 `RobotConnection` 替代（双端口管理） |
| `utils/MathUtils.h` | 合并到 `core/MathUtils.h` |

---

## 七、build 环境

- 用户环境：Visual Studio 2022 Build Tools (MSBuild)，无完整 IDE
- 项目文件：`Touch_Client.vcxproj`（已有，需更新源文件列表）
- GLUT 依赖：SDK 自带 `glut32.lib` + `glut32.dll`（无需额外安装）
- 编译：`msbuild Touch_Client.vcxproj /p:Configuration=Release /p:Platform=x64`
- 运行：`start_system.bat`（已更新，自动拷贝 DLL）

---

## 八、不做的 & 预留

| 项目 | 状态 |
|------|------|
| 力反馈闭环渲染 | 预留 `IExtension` 接口 + `AppState` 力数据字段 |
| 姿态控制 | 预留 `AppState::targetRx/Ry/Rz`，当前固定 |
| Touch 设备外形 | 不做模型，只渲染光标球 |
| MATLAB 中继站 | 代码保留在 `Relay_Station/`，不做删除，作为备份参考 |

---

## 九、自检

- [x] 无 TBD/TODO 占位符
- [x] 架构数据流与文件结构一致
- [x] 三层解耦边界明确：Touch 层 ↔ Relay 层 ↔ Robot 层互不越界
- [x] 所有原有中继站功能（协议转换/数据处理/扩展性）在 Relay 层有对应实现
- [x] GLUT 配置基于现有 SDK 资源，无外部下载依赖
