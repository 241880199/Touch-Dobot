# Dobot 机械臂 TCP/IP 远程控制接口指令集 (V3)

## 端口说明
- **29999**：Dashboard 指令端口（设置、获取状态等）
- **30003**：运动指令端口
- **30004**：实时反馈端口（8ms）
- **30005**：实时反馈端口（200ms）
- **30006**：可配置实时反馈端口（默认50ms）

---

## 1. Dashboard 指令（29999端口）

### 1.1 控制相关指令

#### PowerOn
```text
PowerOn()
```
- **描述**：机械臂上电（六轴特有），约需10秒。
- **返回**：`ErrorID,{},PowerOn();`

#### EnableRobot
```text
EnableRobot(load, centerX, centerY, centerZ)
```
- **描述**：使能机械臂，执行运动前必须调用。
- **参数**：
  - `load` (double)：负载重量(kg)
  - `centerX` (double)：X方向偏心(mm)，范围-500~500
  - `centerY` (double)：Y方向偏心(mm)，范围-500~500
  - `centerZ` (double)：Z方向偏心(mm)，范围-500~500
- **说明**：可携带0、1或4个参数。
- **返回**：`ErrorID,{},EnableRobot(load,centerX,centerY,centerZ);`

#### DisableRobot
```text
DisableRobot()
```
- **描述**：下使能机器人。
- **返回**：`ErrorID,{},DisableRobot();`

#### ClearError
```text
ClearError()
```
- **描述**：清除报警，清除后需重新使能。
- **返回**：`ErrorID,{},ClearError();`

#### ResetRobot
```text
ResetRobot()
```
- **描述**：停止机器人，清空指令队列。
- **返回**：`ErrorID,{},ResetRobot();`

#### RunScript
```text
RunScript(projectName)
```
- **描述**：运行指定工程。
- **参数**：`projectName` (string) - 工程文件名
- **返回**：`ErrorID,{},RunScript(projectName);`

#### StopScript
```text
StopScript()
```
- **描述**：停止正在运行的工程。
- **返回**：`ErrorID,{},StopScript();`

#### PauseScript
```text
PauseScript()
```
- **描述**：暂停正在运行的工程。
- **返回**：`ErrorID,{},PauseScript();`

#### ContinueScript
```text
ContinueScript()
```
- **描述**：继续已暂停的工程。
- **返回**：`ErrorID,{},ContinueScript();`

#### EmergencyStop
```text
EmergencyStop()
```
- **描述**：紧急停止，下电并报警，需清除报警后重新上电。
- **返回**：`ErrorID,{},EmergencyStop();`

#### BrakeControl
```text
BrakeControl(axisID, value)
```
- **描述**：控制指定关节抱闸（六轴特有，控制器≥3.5.2）。
- **参数**：
  - `axisID` (int)：关节序号(1~6)
  - `value` (int)：0-锁死，1-松开
- **返回**：`ErrorID,{},BrakeControl(axisID,value);`

#### StartDrag
```text
StartDrag()
```
- **描述**：进入拖拽模式（六轴特有，≥3.5.2）。
- **返回**：`ErrorID,{},StartDrag();`

#### StopDrag
```text
StopDrag()
```
- **描述**：退出拖拽模式（六轴特有，≥3.5.2）。
- **返回**：`ErrorID,{},StopDrag();`

#### SetCollideDrag
```text
SetCollideDrag(status)
```
- **描述**：强制进入/退出拖拽模式（六轴特有，≥3.5.2）。
- **参数**：`status` (int) - 0强制退出，1强制进入
- **返回**：`ErrorID,{},SetCollideDrag(status);`

#### SetSafeSkin
```text
SetSafeSkin(status)
```
- **描述**：开启/关闭电子皮肤功能（CR系列特有）。
- **参数**：`status` (int) - 0关闭，1开启
- **返回**：`ErrorID,{},SetSafeSkin(status);`

#### Wait
```text
Wait(time)
```
- **描述**：指令队列延时（六轴≥3.5.5，四轴≥1.5.9）。
- **参数**：`time` (int) - 延时时间(ms)，范围(0,3600*1000]
- **返回**：`ErrorID,{},Wait(time);`

---

### 1.2 设置相关指令

#### SpeedFactor
```text
SpeedFactor(ratio)
```
- **描述**：设置全局速度比例（1~100），仅在本次TCP/IP控制生效。
- **参数**：`ratio` (int) - 1~100
- **返回**：`ErrorID,{},SpeedFactor(ratio);`

#### User
```text
User(index)
```
- **描述**：设置全局用户坐标系索引。
- **参数**：`index` (int) - 已标定坐标系索引
- **返回**：`ErrorID,{},User(index);`

#### Tool
```text
Tool(index)
```
- **描述**：设置全局工具坐标系索引。
- **参数**：`index` (int) - 已标定坐标系索引
- **返回**：`ErrorID,{},Tool(index);`

#### Payload (LoadSet)
```text
Payload(weight, inertia)
```
- **描述**：设置末端负载（六轴特有）。
- **参数**：
  - `weight` (double) - 负载重量(kg)
  - `inertia` (double) - 负载惯量(kg·m²)
- **返回**：`ErrorID,{},Payload(weight, inertia);`

#### LoadSwitch
```text
LoadSwitch(status)
```
- **描述**：开关负载设置（六轴特有）。
- **参数**：`status` (int) - 0关闭，1开启
- **返回**：`ErrorID,{},LoadSwitch(status);`

#### SetPayload
```text
SetPayload(weight, inertia)
```
- **描述**：设置末端负载（四轴特有）。
- **参数**：
  - `weight` (float) - 负载重量(kg)
  - `inertia` (float) - 负载惯量(kg·m²)（可选）
- **返回**：`ErrorID,{},SetPayload(weight, inertia);`

#### AccJ
```text
AccJ(R)
```
- **描述**：设置关节运动加速度比例（1~100），默认100。
- **参数**：`R` (int) - 1~100
- **返回**：`ErrorID,{},AccJ(R);`

#### AccL
```text
AccL(R)
```
- **描述**：设置直线/圆弧运动加速度比例（1~100），默认100。
- **参数**：`R` (int) - 1~100
- **返回**：`ErrorID,{},AccL(R);`

#### SpeedJ
```text
SpeedJ(R)
```
- **描述**：设置关节运动速度比例（1~100），默认100。
- **参数**：`R` (int) - 1~100
- **返回**：`ErrorID,{},SpeedJ(R);`

#### SpeedL
```text
SpeedL(R)
```
- **描述**：设置直线/圆弧运动速度比例（1~100），默认100。
- **参数**：`R` (int) - 1~100
- **返回**：`ErrorID,{},SpeedL(R);`

#### Arch
```text
Arch(Index)
```
- **描述**：设置Jump运动全局门型参数索引（四轴特有）。
- **参数**：`Index` (int) - 门型参数索引
- **返回**：`ErrorID,{},Arch(Index);`

#### CP
```text
CP(R)
```
- **描述**：设置平滑过渡比例（0~100），默认0（不平滑）。
- **参数**：`R` (unsigned int) - 0~100
- **返回**：`ErrorID,{},CP(R);`

#### SetArmOrientation
```text
SetArmOrientation(LorR, UorD, ForN, Config6)
```
- **描述**：设置目标点手系（六轴/M1 Pro特有）。
- **参数（六轴）**：
  - `LorR` (int) - 大臂朝向：1向前，-1向后
  - `UorD` (int) - 肘关节朝向：1向上，-1向下
  - `ForN` (int) - 腕关节翻转：1不翻转，-1翻转
  - `Config6` (int) - J6轴角度范围（如1表示[0,90]）
- **参数（M1 Pro）**：仅 `LorR` (0左手系，1右手系)
- **返回**：`ErrorID,{},SetArmOrientation(...);`

#### SetCollisionLevel
```text
SetCollisionLevel(level)
```
- **描述**：设置碰撞检测等级（0关闭，1~5灵敏度递增）。
- **参数**：`level` (int) - 0~5
- **返回**：`ErrorID,{},SetCollisionLevel(level);`

#### TCPSpeed
```text
TCPSpeed(vt)
```
- **描述**：设置绝对速度（mm/s），后续笛卡尔运动按此速度运行（六轴≥3.5.5）。
- **参数**：`vt` (unsigned int) - 0~100000 mm/s
- **返回**：`ErrorID,{},TCPSpeed(vt);`

#### TCPSpeedEnd
```text
TCPSpeedEnd()
```
- **描述**：关闭绝对速度设置（六轴≥3.5.5）。
- **返回**：`ErrorID,{},TCPSpeedEnd();`

---

### 1.3 计算与获取相关指令

#### RobotMode
```text
RobotMode()
```
- **描述**：获取机器人当前状态。
- **返回值**：见下表（ErrorID,{Value},RobotMode();）

| 值 | 状态 |
|----|------|
| 1 | 初始化 |
| 2 | 有抱闸松开 |
| 3 | 本体未上电 |
| 4 | 未使能 |
| 5 | 使能且空闲 |
| 6 | 拖拽模式 |
| 7 | 运行中 |
| 8 | 轨迹录制 |
| 9 | 有报警（优先级最高） |
| 10 | 暂停 |
| 11 | 点动中 |

#### HandleTrajPoints
```text
HandleTrajPoints(traceName)
```
- **描述**：预处理轨迹文件（六轴≥3.5.2），查询返回-3文件错误，-2文件不存在，-1未完成，0完成。
- **参数**：`traceName` (string) - 轨迹文件名（含后缀）
- **返回**：`ErrorID,{},HandleTrajPoints(traceName);`

#### GetTraceStartPose
```text
GetTraceStartPose(traceName)
```
- **描述**：获取轨迹拟合首个点位（笛卡尔坐标）（六轴≥3.5.2）。
- **参数**：`traceName` (string) - 轨迹文件名
- **返回**：`ErrorID,{x,y,z,a,b,c},GetTraceStartPose(traceName);`

#### GetPathStartPose
```text
GetPathStartPose(traceName)
```
- **描述**：获取轨迹复现首个点位（关节坐标）（六轴≥3.5.2）。
- **参数**：`traceName` (string) - 轨迹文件名
- **返回**：`ErrorID,{j1,j2,j3,j4,j5,j6},GetPathStartPose(traceName);`

#### PositiveSolution
```text
PositiveSolution(J1,J2,J3,J4,J5,J6,User,Tool)
```
- **描述**：正解运算，关节角→笛卡尔坐标（六轴特有）。
- **参数**：`J1~J6` (double) - 关节角度(度)，`User`/`Tool` (int) - 坐标系索引
- **返回**：`ErrorID,{x,y,z,a,b,c},PositiveSolution(...);`

#### InverseSolution
```text
InverseSolution(X,Y,Z,Rx,Ry,Rz,User,Tool,isJointNear,JointNear)
```
- **描述**：逆解运算，笛卡尔坐标→关节角（六轴特有）。
- **参数**：
  - `X,Y,Z,Rx,Ry,Rz` (double) - 目标位姿
  - `User/Tool` (int) - 坐标系索引
  - `isJointNear` (int) - 0使用当前角度就近选解，1使用JointNear
  - `JointNear` (string) - 就近参考关节坐标
- **返回**：`ErrorID,{J1,J2,J3,J4,J5,J6},InverseSolution(...);`

#### GetSixForceData
```text
GetSixForceData()
```
- **描述**：获取六维力数据（六轴特有）。
- **返回**：`ErrorID,{Fx,Fy,Fz,Mx,My,Mz},GetSixForceData();`

#### GetAngle
```text
GetAngle()
```
- **描述**：获取当前关节坐标。
- **返回**：`ErrorID,{J1,J2,J3,J4,J5,J6},GetAngle();`

#### GetPose
```text
GetPose(user, tool)
```
- **描述**：获取当前笛卡尔坐标。
- **参数**：`user`/`tool` (int) - 坐标系索引（可选，默认全局）
- **返回**：`ErrorID,{X,Y,Z,Rx,Ry,Rz},GetPose();`

#### GetErrorID
```text
GetErrorID()
```
- **描述**：获取当前报警错误码（六轴≥3.5.2）。
- **返回**：`ErrorID,{[id...],[id],...},GetErrorID();`

---

### 1.4 IO相关指令

#### DO
```text
DO(index, status)
```
- **描述**：设置数字输出（队列指令）。
- **参数**：`index` (int) - DO编号，`status` (int) - 0/1
- **返回**：`ErrorID,{},DO(index,status);`

#### DOExecute
```text
DOExecute(index, status)
```
- **描述**：设置数字输出（立即指令，六轴特有）。
- **参数**：同上
- **返回**：`ErrorID,{},DOExecute(index,status);`

#### DOGroup
```text
DOGroup(index1,value1,index2,value2,...)
```
- **描述**：设置多个数字输出（队列指令）。
- **参数**：成对的编号和状态
- **返回**：`ErrorID,{},DOGroup(...);`

#### ToolDO
```text
ToolDO(index, status)
```
- **描述**：设置末端数字输出（队列指令，六轴特有）。
- **返回**：`ErrorID,{},ToolDO(index,status);`

#### ToolDOExecute
```text
ToolDOExecute(index, status)
```
- **描述**：设置末端数字输出（立即指令，六轴特有）。
- **返回**：`ErrorID,{},ToolDOExecute(index,status);`

#### AO
```text
AO(index, value)
```
- **描述**：设置模拟输出（队列指令，六轴特有）。
- **参数**：`index` (int) - AO编号，`value` (double) - 输出值
- **返回**：`ErrorID,{},AO(index,value);`

#### AOExecute
```text
AOExecute(index, value)
```
- **描述**：设置模拟输出（立即指令，六轴特有）。
- **返回**：`ErrorID,{},AOExecute(index,value);`

#### DI
```text
DI(index)
```
- **描述**：获取DI端口状态。
- **返回**：`ErrorID,{value},DI(index);`

#### DIGroup
```text
DIGroup(index1,index2,...)
```
- **描述**：获取多个DI状态（六轴特有）。
- **返回**：`ErrorID,{value1,value2,...},DIGroup(...);`

#### ToolDI
```text
ToolDI(index)
```
- **描述**：获取末端DI状态（六轴特有）。
- **返回**：`ErrorID,{value},ToolDI(index);`

#### AI
```text
AI(index)
```
- **描述**：获取AI值（六轴特有）。
- **返回**：`ErrorID,{value},AI(index);`

#### ToolAI
```text
ToolAI(index)
```
- **描述**：获取末端AI值（六轴特有）。
- **返回**：`ErrorID,{value},ToolAI(index);`

---

### 1.5 Modbus相关指令（六轴≥3.5.2，四轴支持）

#### ModbusCreate
```text
ModbusCreate(ip, port, slave_id, isRTU)
```
- **描述**：创建Modbus主站，最多5个连接。
- **参数**：`ip` (string) - 从站IP；`port` (int) - 端口；`slave_id` (int)；`isRTU` (int) - 0 TCP，1 RTU
- **返回**：`ErrorID,{index},ModbusCreate(...);`

#### ModbusClose
```text
ModbusClose(index)
```
- **描述**：断开Modbus连接。
- **参数**：`index` (int) - 主站索引
- **返回**：`ErrorID,{},ModbusClose(index);`

#### GetInBits
```text
GetInBits(index, addr, count)
```
- **描述**：读取触点寄存器（离散输入）。
- **参数**：`addr` (int) - 起始地址，`count` (int) - 数量(1~16)
- **返回**：`ErrorID,{value1,...},GetInBits(...);`

#### GetInRegs
```text
GetInRegs(index, addr, count, valType)
```
- **描述**：读取输入寄存器，支持U16/U32/F32/F64。
- **参数**：`count` (1~4)，`valType` (string) - U16/U32/F32/F64
- **返回**：`ErrorID,{value1,...},GetInRegs(...);`

#### GetCoils
```text
GetCoils(index, addr, count)
```
- **描述**：读取线圈寄存器。
- **返回**：`ErrorID,{value1,...},GetCoils(...);`

#### SetCoils
```text
SetCoils(index, addr, count, valTab)
```
- **描述**：写入线圈寄存器。
- **参数**：`valTab` - 值列表，如`{1,0,1}`
- **返回**：`ErrorID,{},SetCoils(...);`

#### GetHoldRegs
```text
GetHoldRegs(index, addr, count, valType)
```
- **描述**：读取保持寄存器，支持U16/U32/F32/F64。
- **返回**：`ErrorID,{value1,...},GetHoldRegs(...);`

#### SetHoldRegs
```text
SetHoldRegs(index, addr, count, valTab, valType)
```
- **描述**：写入保持寄存器。
- **返回**：`ErrorID,{},SetHoldRegs(...);`

---

## 2. 运动指令（30003端口）

### 通用说明
- 坐标系参数`User`/`Tool`可选，默认使用全局坐标系。
- 速度/加速度参数`SpeedJ/SpeedL/AccJ/AccL`可选，默认使用全局设置。
- 不支持在指令中携带`CP`或`SYNC`参数。

---

### MovJ
```text
MovJ(X,Y,Z,Rx,Ry,Rz, User=index, Tool=index, Speed=R, AccJ=R)
```
- **描述**：关节运动至笛卡尔目标点。
- **参数**：`X,Y,Z,Rx,Ry,Rz` (double) - 目标位姿
- **返回**：`ErrorID,{},MovJ(...);`

### MovL
```text
MovL(X,Y,Z,Rx,Ry,Rz, User=index, Tool=index, Speed=R, AccL=R)
```
- **描述**：直线运动至笛卡尔目标点。
- **返回**：`ErrorID,{},MovL(...);`

### JointMovJ
```text
JointMovJ(J1,J2,J3,J4,J5,J6, SpeedJ=R, AccJ=R)
```
- **描述**：关节运动至关节目标点。
- **参数**：`J1~J6` (double) - 目标关节角度(度)
- **返回**：`ErrorID,{},JointMovJ(...);`

### MovLIO
```text
MovLIO(X,Y,Z,Rx,Ry,Rz, {Mode,Distance,Index,Status}, ..., SpeedL=R, AccL=R)
```
- **描述**：直线运动，运动过程中并行设置DO。
- **参数**：`{Mode,Distance,Index,Status}` - Mode:0百分比/1距离(mm)，Distance正负表示起/终点方向
- **返回**：`ErrorID,{},MovLIO(...);`

### MovJIO
```text
MovJIO(X,Y,Z,Rx,Ry,Rz, {Mode,Distance,Index,Status}, ..., SpeedJ=R, AccJ=R)
```
- **描述**：关节运动，并行设置DO。
- **返回**：`ErrorID,{},MovJIO(...);`

### Arc
```text
Arc(X1,Y1,Z1,Rx1,Ry1,Rz1, X2,Y2,Z2,Rx2,Ry2,Rz2, User=index, Tool=index, SpeedL=R, AccL=R)
```
- **描述**：圆弧插补，通过当前点、中间点(P1)、终点(P2)确定圆弧。
- **返回**：`ErrorID,{},Arc(...);`

### Circle3
```text
Circle3({X1,Y1,Z1,Rx1,Ry1,Rz1}, {X2,Y2,Z2,Rx2,Ry2,Rz2}, count, User=index, Tool=index)
```
- **描述**：整圆插补，经P1、P2回到起点（六轴≥3.5.5）。
- **参数**：`count` - 圈数
- **返回**：`ErrorID,{},Circle3(...);`

### ServoJ
```text
ServoJ(J1,J2,J3,J4,J5,J6, t, lookahead_time, gain)
```
- **描述**：关节空间动态跟随（六轴特有），建议33Hz调用。
- **参数**：`t` (float) - 运行时间(s)，默认0.1；`lookahead_time` (float) - 默认50；`gain` (float) - 默认500
- **返回**：无

### ServoP
```text
ServoP(X,Y,Z,Rx,Ry,Rz)
```
- **描述**：笛卡尔空间动态跟随（六轴特有），建议33Hz调用。
- **返回**：无

### MoveJog
```text
MoveJog(axisID, CoordType=typeValue, User=index, Tool=index)
```
- **描述**：点动，下发`MoveJog()`停止（六轴≥3.5.2，四轴≥1.5.6）。
- **参数**：`axisID` - 如`J1+`/`J1-`；`CoordType`可选坐标系
- **返回**：`ErrorID,{},MoveJog(...);`

### StartTrace
```text
StartTrace(traceName)
```
- **描述**：轨迹拟合运动（六轴≥3.5.2），需先运动至首个点位。
- **参数**：`traceName` (string) - 轨迹文件名
- **返回**：`ErrorID,{},StartTrace(traceName);`

### StartPath
```text
StartPath(traceName, const, cart)
```
- **描述**：轨迹复现运动（六轴≥3.5.2）。
- **参数**：`const` - 1匀速/0原速；`cart` - 1笛卡尔路径/0关节路径
- **返回**：`ErrorID,{},StartPath(...);`

### Sync
```text
Sync()
```
- **描述**：阻塞等待队列中最后一条指令执行完成。
- **返回**：`ErrorID,{},Sync();`

### RelMovJTool
```text
RelMovJTool(offsetX, offsetY, offsetZ, offsetRx, offsetRy, offsetRz, Tool, SpeedJ=R, AccJ=R, User=index)
```
- **描述**：沿工具坐标系相对关节运动（六轴≥3.5.2）。
- **返回**：`ErrorID,{},RelMovJTool(...);`

### RelMovTool
```text
RelMovTool(offsetX, offsetY, offsetZ, offsetRx, offsetRy, offsetRz, Tool, SpeedL=R, AccL=R, User=index)
```
- **描述**：沿工具坐标系相对直线运动（六轴≥3.5.2）。
- **返回**：`ErrorID,{},RelMovTool(...);`

### RelMovJUser
```text
RelMovJUser(offsetX, offsetY, offsetZ, offsetRx, offsetRy, offsetRz, User, SpeedJ=R, AccJ=R, Tool=index)
```
- **描述**：沿用户坐标系相对关节运动（六轴≥3.5.2，四轴≥1.5.6）。
- **返回**：`ErrorID,{},RelMovJUser(...);`

### RelMovLUser
```text
RelMovLUser(offsetX, offsetY, offsetZ, offsetRx, offsetRy, offsetRz, User, SpeedL=R, AccL=R, Tool=index)
```
- **描述**：沿用户坐标系相对直线运动（六轴≥3.5.2，四轴≥1.5.6）。
- **返回**：`ErrorID,{},RelMovLUser(...);`

### MovJExt
```text
MovJExt(Angle|Distance, SpeedE=50, AccE=50, Sync=1)
```
- **描述**：控制扩展轴运动（四轴特有）。
- **参数**：`Angle|Distance` (float) - 目标角度(度)或距离(mm)；`Sync` - 0异步/1同步
- **返回**：`ErrorID,{},MovJExt(...);`

### SyncAll
```text
SyncAll()
```
- **描述**：阻塞等待队列中**所有**指令（含扩展轴）执行完成（四轴特有）。
- **返回**：`ErrorID,{},SyncAll();`

---