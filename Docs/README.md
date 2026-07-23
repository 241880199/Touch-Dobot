# Touch-Dobot 远程控制系统

通过 Force Dimension Touch 力反馈设备远程控制 Dobot CR3 六轴机械臂。

```
Touch 设备 ──(OpenHaptics)──> Touch_Client (C++) ──(TCP:30003)──> CR3 机械臂
                                    │
                                    └──(TCP:8888)──> MATLAB GUI (监控)
```

---

## 目录结构

```
Touch-Dobot/
├── Touch_Client/        C++ Touch 端主程序 (OpenGL 可视化 + 触觉反馈)
│   ├── main.cpp         入口, GLUT 窗口 + 定时器
│   ├── config/          配置常量 (安全边界, 网络, 窗口)
│   ├── core/            全局状态 (AppState)
│   ├── haptic/          Touch 设备驱动 (HDAPI 1kHz 回调)
│   ├── relay/           中继逻辑 (坐标映射, 安全边界, TCP 协议)
│   ├── render/          3D 渲染 (机械臂模型, 光点, HUD)
│   ├── robot/           机械臂通信 + 运动学
│   ├── safety/          安全预判器 (四层防护)
│   └── models/cr3/      CR3 STL 模型文件 (7 个连杆)
├── Relay_Station/       MATLAB 中继站 (TCP 服务器 + GUI 监控)
│   ├── relay_main.m     主入口
│   ├── relay_gui.m      GUI 界面
│   └── relay_config.m   配置文件
├── OpenHaptics/         OpenHaptics SDK 3.5.0 (含 GLUT 3.2)
├── scripts/             构建/测试脚本
├── Docs/                文档
└── start_system.bat     一键启动 (编译 + 中继站 + Touch_Client)
```

---

## 环境依赖

| 组件 | 版本/路径 | 说明 |
|------|----------|------|
| Visual Studio | 2022 BuildTools | `D:\Program Files\Microsoft Visual Studio\2022\BuildTools\` |
| OpenHaptics SDK | 3.5.0 | `D:\Projects\Touch\OpenHaptics\Developer\3.5.0\` |
| MATLAB | R2020a+ | 仅中继站需要, 需安装 Instrument Control Toolbox |
| GLUT | 3.2 (随 OH SDK) | `OpenHaptics\...\utilities\include\GL\` |
| CR3 STL 模型 | `Touch_Client\models\cr3\` | base_link + Link1~Link6 (二进制 STL) |

## 快速开始

### 1. 仅编译 Touch_Client

```bat
cd Touch_Client
build.bat
```

输出: `x64\Release\Touch_Client.exe`

### 2. 无机械臂测试 (仅 Touch + 可视化)

```bat
x64\Release\Touch_Client.exe --no-robot
```

### 3. 无 Touch 测试 (仅机械臂连接)

```bat
x64\Release\Touch_Client.exe --no-touch
```

### 4. 一键启动完整系统

```bat
start_system.bat
```

自动执行: 启动 MATLAB 中继站 → 编译 Touch_Client → 复制 DLL → 运行

### 键盘操作

| 按键 | 功能 |
|------|------|
| Touch 按钮 1 按下 + 移动 | 增量控制机械臂运动 |
| `q` / `ESC` | 退出程序 |
| `e` | 手动触发脱困流程 |

---

## 配置说明

### 机械臂 IP (`Touch_Client/config/Config.h`)

```cpp
constexpr const char* ROBOT_IP = "192.168.101.11";
const int ENABLE_PORT = 29999;   // Dashboard 协议端口
const int MOTION_PORT = 30003;   // 运动控制端口
```

### 安全边界 (`Touch_Client/config/Config.h`)

```cpp
const double SAFE_X_MIN = -300.0, SAFE_X_MAX = 250.0;
const double SAFE_Y_MIN = -350.0, SAFE_Y_MAX = 250.0;
const double SAFE_Z_MIN = 140.0,  SAFE_Z_MAX = 500.0;
```

基于机械臂静止位姿 (~ -103, -153, 381) 设定。修改后需重新编译。

### 中继站配置 (`Relay_Station/relay_config.m`)

```matlab
cfg.robot_ip = '192.168.101.11';
cfg.relay_port = 8888;  % Touch_Client → MATLAB 上报端口
```

### MATLAB GUI 上报

Touch_Client 启动后自动连接 `127.0.0.1:8888`，实时上报机械臂位姿/关节角/指令日志。

---

## 安全系统 (四层)

| 层 | 检测项 | 失败动作 |
|----|--------|----------|
| 几何 | 圆柱奇异 (距 Z 轴 < 80mm) / 关节限位 < 10° | WARN 减速 |
| 1 | 工作半径 620mm / 用户安全边界 | REJECT 拒绝 |
| 2 | IK 数值逆解 (URDF 模型, 辅助判断) | WARN 减速 |
| 3 | 奇异位形 (雅可比条件数 > 500) | REJECT 拒绝 |
| 4 | 历史报警黑名单 (< 80mm) | WARN 减速 |

HUD 面板实时显示: 安全状态、Z 轴距离、关节限位余量、报警计数。

---

## 架构说明 (v3.0)

Touch_Client 是一个**单进程 C++ 桌面应用**，使用 freeglut + 即时模式 OpenGL 渲染。

### 线程模型

| 线程 | 频率 | 职责 |
|------|------|------|
| GLUT 主循环 | ~60Hz | 渲染 3D 场景 + 2D HUD |
| HDAPI 触觉回调 | 1kHz | 读取 Touch 位置/按钮, 写入力反馈 |
| GetPose 定时器 | 10Hz | 查询机械臂 Cartesian 位姿 |
| GetAngle 定时器 | 5Hz | 查询机械臂关节角 |
| Alarm 巡检定时器 | ~3Hz | 检测机械臂报警状态 |
| TCP 发送线程 | 按需 | 队列化发送运动指令 |

### 运动控制

- **增量模式**: Touch 每帧位移增量 → 累加目标坐标 → 安全预判 → ServoP
- 每帧最大增量: **4.5mm**，30Hz ServoP 频率
- 按钮按下时从当前机器人位姿开始累积，松开停止

### Touch→Robot 坐标映射 (`CoordinateTransform.h`)

```cpp
robot_x =  touch_x;
robot_y = -touch_z;
robot_z =  touch_y;
```

硬编码轴映射，未经实物标定。

### 已知限制

- URDF 运动学模型与 Dobot 控制器坐标系存在偏移 (~408mm, 含旋转分量)，IK 数值解不可靠
- 坐标映射为硬编码，X/Y/Z 方向可能与直觉不完全一致
- 力反馈通道已预留接口，尚未集成滤波和渲染

---

## 故障排查

### Touch_Client 启动失败
- 确认 Touch 设备 USB 已连接，驱动已安装
- 无 Touch 环境使用 `--no-touch` 参数
- 检查 `hd.dll` / `hdu.dll` / `glut32.dll` 是否在 exe 同目录

### 机械臂连接失败
- 确认机械臂 IP 可达 (`ping 192.168.101.11`)
- 无机械臂环境使用 `--no-robot` 参数
- 检查防火墙是否放行端口 29999/30003

### 安全边界持续触发
- 机械臂静止位姿可能已超出 `Config.h` 中的安全边界
- 根据实际 GetPose 读数调整 `SAFE_X/Y/Z_MIN/MAX`

### 编译错误: CALLBACK 宏重定义
- 已知警告 (glut.h vs Windows SDK)，不影响功能
