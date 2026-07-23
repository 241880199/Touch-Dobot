# Touch-Dobot 远程交互系统

## 项目简介
通过 Force Dimension Touch 设备远程控制 Dobot CR3 机械臂，实现双向力反馈闭环。
**核心技术栈：** C++ / OpenHaptics / MATLAB / TCP Socket / OpenGL

---

## 仓库目录结构

| 目录 | 说明 |
| :--- | :--- |
| **`Touch_Client/`** | Touch 端主程序 (C++)。Touch 设备控制、3D 可视化界面、力反馈渲染、TCP 通信。 |
| **`Relay_Station/`** | 中继站程序 (MATLAB)。TCP 服务器、坐标转换、机械臂指令生成、力反馈转发、时延监控。 |
| **`Robot_Control/`** | 机械臂底层控制模块。SDK 封装、运动控制接口、力/力矩读取。 |
| **`Algo_Sandbox/`** | 算法验证沙箱。坐标变换推导、滤波算法测试 (Python/Matlab)。 |
| **`Network_Shared/`** | 公共通信协议定义。TCP 消息格式规范、数据包结构。 |
| **`Docs/`** | 项目文档、技术资料。 |

---

## 系统架构

### 正向链路 (人 → 机械臂)
1. Touch 端读取原始坐标及姿态，通过 TCP 发送给中继站
2. 中继站接收坐标，调用坐标转换算法，映射为机械臂目标位姿
3. 生成运动指令 (ServoP)，通过 TCP 发送给机械臂控制器
4. 机械臂执行运动

### 反向链路 (机械臂 → 人)
1. 中继站通过 SDK 实时读取末端力/力矩数据
2. 滤波算法对原始力数据进行平滑处理
3. 处理后的力数据通过 TCP 发送给 Touch 端
4. Touch 程序调用 OpenHaptics SDK 驱动手柄产生阻力

---

## Touch_Client 当前进度 (v3.0)

### 已完成功能
- **Touch 设备集成**: OpenHaptics HDAPI 1kHz 触觉回调，位置读取 + 按钮状态
- **3D 可视化**: GLUT/OpenGL 渲染，透视等距视角，右手坐标系，CR3 STL 模型加载，末端光点指示器
- **机械臂通信**: 双端口 TCP (29999 Dashboard + 30003 Motion)，ServoP 增量运动控制
- **安全系统**: 四层防护 (工作半径 + 用户边界 + IK 辅助 + 奇异检测 + 报警历史)
- **HUD 界面**: 指令日志、反馈日志、坐标/关节角显示、安全状态、奇异位形预警
- **MATLAB GUI 上报**: TCP 位置/指令实时上报 (localhost:8888)
- **脱困流程**: 自动检测报警 → 拖拽模式 → 手动调整 → 重新使能

### 安全边界配置
- X: [-300, 250] mm
- Y: [-350, 250] mm
- Z: [140, 500] mm
- 工作半径: 620mm

### 已知限制
- URDF 运动学模型与 Dobot 控制器坐标系存在偏移 (约 408mm, 含旋转分量)，IK 数值解不可靠，已降级为辅助判断
- Touch→Robot 坐标映射为硬编码，未经实物标定

---

## 项目推进阶段

### 第一阶段: 环境搭建与单点验证 ✅
- 机械臂硬件连接，SDK 验证
- OpenHaptics 开发环境配置
- PC 间 TCP 通信验证

### 第二阶段: 核心模块开发 ✅
- Touch 坐标读取 + TCP 客户端
- MATLAB 中继站基础框架
- 坐标转换矩阵 + 滤波器初版

### 第三阶段: 闭环联调 (进行中)
- 中继站力数据读取与回传
- Touch 端力反馈渲染
- 通信联调
- 安全边界 + 滤波集成

### 第四阶段: 系统优化与测试
- 端到端延迟测试
- 力反馈增益调优
- 长时间运行稳定性

---

## 构建说明 (Touch_Client)

### 依赖
- Visual Studio 2022 Build Tools
- OpenHaptics SDK 3.5.0
- freeglut 3.2 (含于 OpenHaptics)

### 编译
```bat
cd Touch_Client
build_and_run.bat
```

### 命令行参数
- `--no-robot`: 无机械臂模式 (仅 Touch + 可视化)
- `--no-touch`: 无 Touch 模式 (仅机械臂连接)
- `q` / `ESC`: 退出
- `e`: 手动触发脱困

---

## 协作规范
1. **提交代码**: 不上传 `.exe`, `.obj`, `.o`, `build/` 等编译产物
2. **分支管理**: 日常开发使用独立分支，测试通过后合并 master
3. **文档维护**: 通信协议、算法推导及时更新到 `Docs/`
