好的，已将您提供的PDF文档内容转换为结构清晰的Markdown格式。

---

# Dobot机械臂TCP/IP远程控制接口文档 (V3)

## 目录

- [前言](#前言)
- [1. 概述](#1-概述)
- [2. Dashboard指令 (29999端口)](#2-dashboard指令-29999端口)
    - [2.1 控制相关指令](#21-控制相关指令)
    - [2.2 设置相关指令](#22-设置相关指令)
    - [2.3 计算和获取相关指令](#23-计算和获取相关指令)
    - [2.4 IO相关指令](#24-io相关指令)
    - [2.5 Modbus相关指令](#25-modbus相关指令)
- [3. 运动指令 (30003端口)](#3-运动指令-30003端口)
    - [3.1 指令列表](#31-指令列表)
- [4. 实时反馈信息 (30004/30005/30006端口)](#4-实时反馈信息-300043000530006端口)
- [5. 通用错误码](#5-通用错误码)

---

## 前言

### 目的

本手册介绍了Dobot工业机械臂控制柜V3版本TCP/IP二次开发接口及其使用方式，帮助用户了解和开发基于TCP/IP的机械臂控制软件。

### 读者对象

本手册适用于：
- 客户销售工程师
- 安装调测工程师
- 技术支持工程师

### 修订记录

| 时间 | 修订记录 |
| :--- | :--- |
| 2023/05/16 | 对应六轴控制器3.5.5和四轴控制器1.5.9版本。<br>新增 `Circle3`、`TCPSpeed`、`TCPSpeedEnd`、`Wait` 指令；<br>`GetPose`、`ServoJ` 指令新增可选参数；<br>`RobotMode` 返回值新增3的含义（本体未上电）；<br>新增30005端口可配置说明。 |
| 2023/04/03 | 增加运动指令通用说明中的使用限制。 |
| 2023/01/09 | 第一次发布。 |

---

## 1. 概述

基于TCP/IP通讯具有成本低、可靠性高、实用性强、性能高等特点，Dobot机器人在TCP/IP协议基础上，提供了丰富的接口用于与外部设备交互。

**支持版本**：
- 六轴机械臂：控制器版本 V3.5.1.19 及以上
- 四轴机械臂：控制器版本 V1.5.5.0 及以上

> **说明**：CR系列在TCP/IP控制模式下无法使用末端按键功能。

### 端口说明

Dobot工业机器人会开启以下服务器端口：

- **29999端口 (Dashboard)**：用于发送设置指令或主动获取机器人状态。
- **30003端口 (运动指令)**：用于发送机器人运动相关指令。
- **30004端口 (实时反馈)**：客户端每**8ms**接收一次机器人实时状态信息（1440字节）。
- **30005端口 (实时反馈)**：每**200ms**反馈一次机器人信息（1440字节）。
- **30006端口 (实时反馈)**：可配置的反馈端口（默认每50ms反馈，修改请联系技术支持）。

> **说明**：
> - 六轴控制器3.5.2+及四轴控制器V1.5.5+支持30004、30005、30006端口。
> - 六轴控制器3.5.1通过30003端口反馈状态信息。
> - 六轴控制器3.5.5+及四轴控制器1.5.9+可配置30005端口反馈周期。

### 消息格式

所有消息命令与应答均为**ASCII码格式（字符串形式）**。

**下发消息格式**：
由消息名称和参数组成，参数放在括号内，以英文逗号分隔，以右括号结束。指令不区分大小写。
```
CommandName(Param1,Param2,...)
```

**应答消息格式**：
```
ErrorID,{value1,...,valueN},CommandName(Param1,Param2,...);
```
- `ErrorID`：`0` 表示成功，非0表示错误（详见通用错误码）。
- `{value1,...,valueN}`：返回值，无返回值时为 `{}`。
- `CommandName(...)`：回显下发的命令。

**示例**：
```
下发: MovL(-500,100,200,150,0,90)
返回: 0,{},MovL(-500,100,200,150,0,90); // 成功

下发: Mov(-500,100,200,150,0,90)
返回: -10000,{},Mov(-500,100,200,150,0,90); // 命令不存在
```

> **说明**：本文档若无特殊说明，参数均以六轴机械臂为例，四轴机械臂需自行适配参数数量。

**获取DEMO**：
越疆提供了各种编程语言的二次开发DEMO，托管于Github，请自行获取。

---

## 2. Dashboard指令 (29999端口)

### 2.1 控制相关指令

#### PowerOn
- **原型**：`PowerOn()`
- **描述**：机械臂上电（六轴机械臂特有）。上电完成约需10秒，之后再进行使能操作。
- **返回**：`ErrorID,{},PowerOn();`
- **示例**：`PowerOn()`

#### EnableRobot
- **原型**：`EnableRobot(load, centerX, centerY, centerZ)`
- **描述**：使能机械臂。执行队列指令前必须先使能。
- **参数**：
    - `load` (double, 可选): 负载重量 (kg)，不能超范围。
    - `centerX` (double, 可选): X方向偏心距离 (mm)，范围 ±500。
    - `centerY` (double, 可选): Y方向偏心距离 (mm)，范围 ±500。
    - `centerZ` (double, 可选): Z方向偏心距离 (mm)，范围 ±500。
    - **参数组合**：可携带0、1、4个参数。
- **返回**：`ErrorID,{},EnableRobot(load,centerX,centerY,centerZ);`
- **示例**：
    - `EnableRobot()`：仅使能。
    - `EnableRobot(1.5)`：使能并设置负载1.5kg。
    - `EnableRobot(1.5,0,0,30.5)`：使能并设置负载和Z方向偏心。

#### DisableRobot
- **原型**：`DisableRobot()`
- **描述**：下使能机器人。
- **返回**：`ErrorID,{},DisableRobot();`
- **示例**：`DisableRobot()`

#### ClearError
- **原型**：`ClearError()`
- **描述**：清除机器人报警。清除后需重新使能才能下发运动指令。
- **返回**：`ErrorID,{},ClearError();`
- **示例**：`ClearError()`

#### ResetRobot
- **原型**：`ResetRobot()`
- **描述**：停止机器人，清空已规划的指令队列。
- **返回**：`ErrorID,{},ResetRobot();`
- **示例**：`ResetRobot()`

#### RunScript
- **原型**：`RunScript(projectName)`
- **描述**：运行指定工程。
- **参数**：`projectName` (string) - 工程文件名。
- **返回**：`ErrorID,{},RunScript(projectName);`
- **示例**：`RunScript(demo)`

#### StopScript
- **原型**：`StopScript()`
- **描述**：停止正在运行的工程。
- **返回**：`ErrorID,{},StopScript();`
- **示例**：`StopScript()`

#### PauseScript
- **原型**：`PauseScript()`
- **描述**：暂停正在运行的工程。
- **返回**：`ErrorID,{},PauseScript();`
- **示例**：`PauseScript()`

#### ContinueScript
- **原型**：`ContinueScript()`
- **描述**：继续已暂停的工程。
- **返回**：`ErrorID,{},ContinueScript();`
- **示例**：`ContinueScript()`

#### EmergencyStop
- **原型**：`EmergencyStop()`
- **描述**：紧急停止机械臂。急停后机械臂会下电并报警，需清除报警后才能重新上电和使能。
- **返回**：`ErrorID,{},EmergencyStop();`
- **示例**：`EmergencyStop()`

#### BrakeControl
- **原型**：`BrakeControl(axisID, value)`
- **描述**：控制指定关节的抱闸（六轴机械臂特有，控制器3.5.2+）。**仅能在下使能时控制**。
- **参数**：
    - `axisID` (int): 关节序号 (1~6)。
    - `value` (int): 0-抱闸锁死，1-松开抱闸。
- **返回**：`ErrorID,{},BrakeControl(axisID, value);`
- **示例**：`BrakeControl(1,1)` // 松开关节1抱闸

#### StartDrag
- **原型**：`StartDrag()`
- **描述**：机械臂进入拖拽模式（六轴特有，控制器3.5.2+）。报错状态下无法进入。
- **返回**：`ErrorID,{},StartDrag();`
- **示例**：`StartDrag()`

#### StopDrag
- **原型**：`StopDrag()`
- **描述**：机械臂退出拖拽模式（六轴特有，控制器3.5.2+）。报错状态下无法退出。
- **返回**：`ErrorID,{},StopDrag();`
- **示例**：`StopDrag()`

#### SetCollideDrag
- **原型**：`SetCollideDrag(status)`
- **描述**：强制进入或退出拖拽模式（六轴特有，控制器3.5.2+）。**报错状态下也可使用**。
- **参数**：`status` (int) - 0: 强制退出，1: 强制进入。
- **返回**：`ErrorID,{},SetCollideDrag(status);`
- **示例**：`SetCollideDrag(1)`

#### SetSafeSkin
- **原型**：`SetSafeSkin(status)`
- **描述**：开启或关闭电子皮肤功能（CR系列特有）。
- **参数**：`status` (int) - 0: 关闭，1: 开启。
- **返回**：`ErrorID,{},SetSafeSkin(status);`
- **示例**：`SetSafeSkin(1)`

#### Wait
- **原型**：`Wait(time)`
- **描述**：指令队列延时（六轴3.5.5+，四轴1.5.9+）。
- **参数**：`time` (int) - 延时时间 (ms)，范围 (0, 3600*1000]。
- **返回**：`ErrorID,{},Wait(time);`
- **示例**：`Wait(1000)`

---

### 2.2 设置相关指令

#### SpeedFactor
- **原型**：`SpeedFactor(ratio)`
- **描述**：设置全局速度比例 (1~100)。仅在本次TCP/IP控制模式中生效。
- **参数**：`ratio` (int) - 全局运动速度比例 (1~100)。
- **返回**：`ErrorID,{},SpeedFactor(ratio);`
- **示例**：`SpeedFactor(80)`

#### User
- **原型**：`User(index)`
- **描述**：设置全局用户坐标系。仅在本次TCP/IP控制模式中生效。
- **参数**：`index` (int) - 已标定的用户坐标系索引。
- **返回**：`ErrorID,{},User(index);` (索引不存在返回 -1)
- **示例**：`User(1)`

#### Tool
- **原型**：`Tool(index)`
- **描述**：设置全局工具坐标系。仅在本次TCP/IP控制模式中生效。
- **参数**：`index` (int) - 已标定的工具坐标系索引。
- **返回**：`ErrorID,{},Tool(index);` (索引不存在返回 -1)
- **示例**：`Tool(1)`

#### PayLoad (六轴)
- **原型**：`Payload(weight, inertia)`
- **描述**：设置机械臂末端负载（六轴独有）。别名 `LoadSet`。
- **参数**：
    - `weight` (double): 负载重量 (kg)。
    - `inertia` (double): 负载惯量 (kg·m²)。
- **返回**：`ErrorID,{},Payload(weight, inertia);`
- **示例**：`Payload(3, 0.4)`

#### LoadSwitch (六轴)
- **原型**：`LoadSwitch(status)`
- **描述**：开关负载设置（六轴独有），开启后可提高碰撞检测灵敏度。
- **参数**：`status` (int) - 0: 关闭，1: 开启。
- **返回**：`ErrorID,{},LoadSwitch(status);`
- **示例**：`LoadSwitch(1)`

#### SetPayload (四轴)
- **原型**：`SetPayload(weight, inertia)`
- **描述**：设置机械臂末端负载（四轴独有）。
- **参数**：
    - `weight` (float): 负载重量 (kg)。
    - `inertia` (float, 可选): 负载惯量 (kg·m²)。
- **返回**：`ErrorID,{},SetPayload(weight, inertia);`
- **示例**：`SetPayload(0.3)`

#### AccJ
- **原型**：`AccJ(R)`
- **描述**：设置关节运动方式的加速度比例 (1~100)。仅在本次TCP/IP模式中生效，默认100。
- **返回**：`ErrorID,{},AccJ(R);`
- **示例**：`AccJ(50)`

#### AccL
- **原型**：`AccL(R)`
- **描述**：设置直线/弧线运动方式的加速度比例 (1~100)。仅在本次TCP/IP模式中生效，默认100。
- **返回**：`ErrorID,{},AccL(R);`
- **示例**：`AccL(50)`

#### SpeedJ
- **原型**：`SpeedJ(R)`
- **描述**：设置关节运动方式的速度比例 (1~100)。仅在本次TCP/IP模式中生效，默认100。
- **返回**：`ErrorID,{},SpeedJ(R);`
- **示例**：`SpeedJ(50)`

#### SpeedL
- **原型**：`SpeedL(R)`
- **描述**：设置直线/弧线运动方式的速度比例 (1~100)。仅在本次TCP/IP模式中生效，默认100。
- **返回**：`ErrorID,{},SpeedL(R);`
- **示例**：`SpeedL(50)`

#### Arch (四轴)
- **原型**：`Arch(Index)`
- **描述**：设置Jump运动的全局门型参数索引（四轴独有）。默认0。
- **参数**：`Index` (int) - 门型参数索引。
- **返回**：`ErrorID,{},Arch(Index);`
- **示例**：`Arch(1)`

#### CP
- **原型**：`CP(R)`
- **描述**：设置平滑过渡比例 (0~100)，对Jump运动无效。仅在本次TCP/IP模式中生效，默认0。
- **参数**：`R` (unsigned int) - 平滑过渡比例 (0~100)。
- **返回**：`ErrorID,{},CP(R);`
- **示例**：`CP(50)`

#### SetArmOrientation
- **原型**：`SetArmOrientation(LorR, UorD, ForN, Config6)`
- **描述**：设置运动目标点的手系（六轴和M1Pro特有）。仅在本次TCP/IP模式中生效。
- **参数**：
    - **六轴**：4个参数。
        - `LorR` (int): 大臂朝向，1向前，-1向后。
        - `UorD` (int): 肘关节朝向，1向上，-1向下。
        - `ForN` (int): 腕关节翻转，1不翻转，-1翻转。
        - `Config6` (int): J6轴角度范围，-1:[0,-90]，-2:[-90,-180]，1:[0,90]，2:[90,180]。
    - **M1 Pro**：1个参数 (`LorR`)。
        - `LorR` (int): 0-左手系，1-右手系。
- **返回**：`ErrorID,{},SetArmOrientation(LorR, UorD, ForN, Config6);`
- **示例**：
    - `SetArmOrientation(1,1,-1,1)` // 六轴
    - `SetArmOrientation(1)` // M1 Pro

#### SetCollisionLevel
- **原型**：`SetCollisionLevel(level)`
- **描述**：设置碰撞检测等级 (0~5)。仅在本次TCP/IP模式中生效。
- **参数**：`level` (int) - 0关闭，1~5灵敏度递增。
- **返回**：`ErrorID,{},SetCollisionLevel(level);`
- **示例**：`SetCollisionLevel(1)`

#### TCPSpeed
- **原型**：`TCPSpeed(vt)`
- **描述**：设置绝对速度 (mm/s)。之后的笛卡尔运动指令以该速度运行，`SpeedL`失效。受全局速度限制。与焊接指令冲突时以焊接为准。（六轴3.5.5+）
- **参数**：`vt` (unsigned int) - 绝对速度 (mm/s)，[0, 100000]。
- **返回**：`ErrorID,{},TCPSpeed(vt);`
- **示例**：
    ```
    TCPSpeed(100)
    MovL(-500,100,200,150,0,90)
    ```

#### TCPSpeedEnd
- **原型**：`TCPSpeedEnd()`
- **描述**：关闭绝对速度设置，与`TCPSpeed`配合使用。（六轴3.5.5+）
- **返回**：`ErrorID,{},TCPSpeedEnd();`
- **示例**：
    ```
    TCPSpeed(100)
    MovL(-500,100,200,150,0,90)
    TCPSpeedEnd()
    MovL(500,100,200,150,0,90)
    ```

---

### 2.3 计算和获取相关指令

#### RobotMode
- **原型**：`RobotMode()`
- **描述**：获取机器人当前状态。
- **返回**：`ErrorID,{Value},RobotMode();`
- **Value取值**：
    - `1`: 初始化
    - `2`: 有抱闸松开
    - `3`: 本体未上电
    - `4`: 未使能（无抱闸松开）
    - `5`: 使能且空闲
    - `6`: 拖拽模式
    - `7`: 运行中
    - `8`: 轨迹录制模式
    - `9`: 有未清除报警（优先级最高）
    - `10`: 暂停状态
    - `11`: 点动中
- **示例**：`RobotMode()`

#### HandleTrajPoints
- **原型**：`HandleTrajPoints(traceName)`
- **描述**：预处理轨迹文件（六轴特有，控制器3.5.2+）。不带参数查询结果。
- **参数**：`traceName` (string) - 轨迹文件名（含后缀），文件路径：`/dobot/userdata/project/process/trajectory/`。
- **返回**：`ErrorID,{},HandleTrajPoints(traceName);` 或 `ErrorID,{},HandleTrajPoints();`
- **查询返回值**：-3文件内容错误，-2文件不存在，-1预处理未完成，0完成无错误，>0表示问题点位。
- **示例**：
    ```
    HandleTrajPoints(recv_string) // 轮询结果
    HandleTrajPoints() // 查询结果
    ```

#### GetTraceStartPose
- **原型**：`GetTraceStartPose(traceName)`
- **描述**：获取轨迹拟合的首个笛卡尔坐标点（六轴特有，控制器3.5.2+）。
- **参数**：`traceName` (string) - 轨迹文件名。
- **返回**：`ErrorID,{x,y,z,a,b,c},GetTraceStartPose(traceName);`
- **示例**：`GetTraceStartPose(recv_string)`

#### GetPathStartPose
- **原型**：`GetPathStartPose(traceName)`
- **描述**：获取轨迹复现的首个关节坐标点（六轴特有，控制器3.5.2+）。
- **参数**：`traceName` (string) - 轨迹文件名。
- **返回**：`ErrorID,{j1,j2,j3,j4,j5,j6},GetPathStartPose(traceName);`
- **示例**：`GetPathStartPose(recv_string)`

#### PositiveSolution
- **原型**：`PositiveSolution(J1,J2,J3,J4,J5,J6,User,Tool)`
- **描述**：正解运算，由关节角度计算笛卡尔坐标（六轴特有）。
- **参数**：`J1~J6` (double): 关节角度(°)，`User`, `Tool` (int): 坐标系索引。
- **返回**：`ErrorID,{x,y,z,a,b,c},PositiveSolution(...);`
- **示例**：
    ```
    PositiveSolution(0,0,-90,0,90,0,0,0)
    ```

#### InverseSolution
- **原型**：`InverseSolution(X,Y,Z,Rx,Ry,Rz,User,Tool,isJointNear,JointNear)`
- **描述**：逆解运算，由笛卡尔坐标计算关节角度（六轴特有）。
- **参数**：
    - `X,Y,Z,Rx,Ry,Rz` (double): 笛卡尔坐标。
    - `User`, `Tool` (int): 坐标系索引。
    - `isJointNear` (int, 可选): 0-根据当前角度就近选解，1-根据`JointNear`就近选解。
    - `JointNear` (string, 可选): 用于就近选解的关节坐标。
- **返回**：`ErrorID,{J1,J2,J3,J4,J5,J6},InverseSolution(...);`
- **示例**：
    ```
    // 根据当前角度选解
    InverseSolution(473,-141,469,-180,0,-90,0,0,0,0,0)
    // 根据指定关节坐标选解
    InverseSolution(473,-141,469,-180,0,-90,0,0,1,"0,0,-90,0,90,0")
    ```

#### GetSixForceData
- **原型**：`GetSixForceData()`
- **描述**：获取机械臂六维力数据原始值（六轴特有）。
- **返回**：`ErrorID,{Fx,Fy,Fz,Mx,My,Mz},GetSixForceData();`
- **示例**：`GetSixForceData()`

#### GetAngle
- **原型**：`GetAngle()`
- **描述**：获取当前位姿的关节坐标。
- **返回**：`ErrorID,{J1,J2,J3,J4,J5,J6},GetAngle();`
- **示例**：`GetAngle()`

#### GetPose
- **原型**：`GetPose(user, tool)`
- **描述**：获取当前位姿的笛卡尔坐标。
- **参数**：`user`, `tool` (int, 可选) - 坐标系索引，不传则使用全局坐标系。
- **返回**：`ErrorID,{X,Y,Z,Rx,Ry,Rz},GetPose();`
- **示例**：`GetPose()`

#### GetErrorID
- **原型**：`GetErrorID()`
- **描述**：获取当前报错的错误码（六轴3.5.2+，四轴支持）。
- **返回**：`ErrorID,{[id,...,id],[id],[id],[id],[id],[id],[id]},GetErrorID();`
    - 第一个数组：控制器/算法报警，无报警为[]（碰撞检测值为-2，电子皮肤为-3）。
    - 后六个数组：伺服报警信息（四轴为四个）。
- **示例**：`GetErrorID()`

---

### 2.4 IO相关指令

**通用说明**：
- **队列指令**：等待之前的指令队列执行完毕后再执行。
- **立即指令**：无视指令队列，立即执行。

#### DO
- **原型**：`DO(index, status)`
- **描述**：设置数字输出端口状态（队列指令）。
- **参数**：`index` (int) - DO编号，`status` (int) - 1:有信号，0:无信号。
- **返回**：`ErrorID,{},DO(index,status);`
- **示例**：`DO(1,1)`

#### DOExecute
- **原型**：`DOExecute(index, status)`
- **描述**：设置数字输出端口状态（立即指令，六轴特有）。
- **参数**：同`DO`。
- **返回**：`ErrorID,{},DOExecute(index,status);`
- **示例**：`DOExecute(1,1)`

#### DOGroup
- **原型**：`DOGroup(index1,value1,index2,value2,...,indexN,valueN)`
- **描述**：设置多个数字输出端口状态（队列指令）。
- **返回**：`ErrorID,{},DOGroup(...);`
- **示例**：`DOGroup(4,1,6,0,2,1,7,0)`

#### ToolDO
- **原型**：`ToolDO(index, status)`
- **描述**：设置末端数字输出端口状态（队列指令，六轴特有）。
- **返回**：`ErrorID,{},ToolDO(index,status);`
- **示例**：`ToolDO(1,1)`

#### ToolDOExecute
- **原型**：`ToolDOExecute(index, status)`
- **描述**：设置末端数字输出端口状态（立即指令，六轴特有）。
- **返回**：`ErrorID,{},ToolDOExecute(index,status);`
- **示例**：`ToolDOExecute(1,1)`

#### AO
- **原型**：`AO(index, value)`
- **描述**：设置模拟输出端口的值（队列指令，六轴特有）。
- **参数**：`index` (int) - AO编号，`value` (double) - 输出值。
- **返回**：`ErrorID,{},AO(index,value);`
- **示例**：`AO(1,2)`

#### AOExecute
- **原型**：`AOExecute(index, value)`
- **描述**：设置模拟输出端口的值（立即指令，六轴特有）。
- **返回**：`ErrorID,{},AOExecute(index,value);`
- **示例**：`AOExecute(1,2)`

#### DI
- **原型**：`DI(index)`
- **描述**：获取DI端口的状态。
- **参数**：`index` (int) - DI编号。
- **返回**：`ErrorID,{value},DI(index);` (value: 0/1)
- **示例**：`DI(1)`

#### DIGroup
- **原型**：`DIGroup(index1,index2,...,indexN)`
- **描述**：获取多个DI端口的状态（六轴特有）。
- **返回**：`ErrorID,{value1,...,valueN},DIGroup(...);`
- **示例**：`DIGroup(4,6,2,7)`

#### ToolDI
- **原型**：`ToolDI(index)`
- **描述**：获取末端DI端口的状态（六轴特有）。
- **返回**：`ErrorID,{value},ToolDI(index);` (value: 0/1)
- **示例**：`ToolDI(1)`

#### AI
- **原型**：`AI(index)`
- **描述**：获取AI端口的值（六轴特有）。
- **返回**：`ErrorID,{value},AI(index);`
- **示例**：`AI(1)`

#### ToolAI
- **原型**：`ToolAI(index)`
- **描述**：获取末端AI端口的值（六轴特有）。
- **返回**：`ErrorID,{value},ToolAI(index);`
- **示例**：`ToolAI(1)`

---

### 2.5 Modbus相关指令

> **支持**：六轴3.5.2+，四轴支持。

#### ModbusCreate
- **原型**：`ModbusCreate(ip, port, slave_id, isRTU)`
- **描述**：创建Modbus主站并连接从站，最多5个设备。
- **参数**：
    - `ip` (string): 从站IP。`127.0.0.1` 或 `0.0.0.1` 表示本机。
    - `port` (int): 从站端口。
    - `slave_id` (int): 从站ID。
    - `isRTU` (int, 可选): 0-TCP通信，1-RTU通信。
- **返回**：`ErrorID,{index},ModbusCreate(...);` (`index`为主站索引，0~4)
- **示例**：`ModbusCreate(127.0.0.1,60000,1,1)`

#### ModbusClose
- **原型**：`ModbusClose(index)`
- **描述**：断开Modbus连接，释放主站。
- **参数**：`index` (int) - 主站索引。
- **返回**：`ErrorID,{},ModbusClose(index);`
- **示例**：`ModbusClose(0)`

#### GetInBits
- **原型**：`GetInBits(index, addr, count)`
- **描述**：读取触点寄存器（离散输入）的值。
- **参数**：`index` (int), `addr` (int) - 起始地址, `count` (int) - 数量 [1,16]。
- **返回**：`ErrorID,{value1,...},GetInBits(...);`
- **示例**：`GetInBits(0,3000,5)`

#### GetInRegs
- **原型**：`GetInRegs(index, addr, count, valType)`
- **描述**：按指定类型读取输入寄存器值。
- **参数**：`addr` (int), `count` (int) - 数量 [1,4], `valType` (string, 可选): `U16`(默认), `U32`, `F32`, `F64`。
- **返回**：`ErrorID,{value1,...},GetInRegs(...);`
- **示例**：`GetInRegs(0,4000,3)`

#### GetCoils
- **原型**：`GetCoils(index, addr, count)`
- **描述**：读取线圈寄存器值。
- **参数**：同`GetInBits`。
- **返回**：`ErrorID,{value1,...},GetCoils(...);`
- **示例**：`GetCoils(0,1000,3)`

#### SetCoils
- **原型**：`SetCoils(index, addr, count, valTab)`
- **描述**：将值写入线圈寄存器。
- **参数**：`addr` (int), `count` (int) [1,16], `valTab` (string) - 值列表，如 `{1,0,1}`。
- **返回**：`ErrorID,{},SetCoils(...);`
- **示例**：`SetCoils(0,1000,3,{1,0,1})`

#### GetHoldRegs
- **原型**：`GetHoldRegs(index, addr, count, valType)`
- **描述**：按指定类型读取保持寄存器值。
- **参数**：同`GetInRegs`。
- **返回**：`ErrorID,{value1,...},GetHoldRegs(...);`
- **示例**：`GetHoldRegs(0,3095,1)`

#### SetHoldRegs
- **原型**：`SetHoldRegs(index, addr, count, valTab, valType)`
- **描述**：按指定类型将值写入保持寄存器。
- **参数**：`addr` (int), `count` (int) [1,4], `valTab` (string), `valType` (string, 可选)。
- **返回**：`ErrorID,{},SetHoldRegs(...);`
- **示例**：`SetHoldRegs(0,3095,2,{6000,300},U16)`

---

## 3. 运动指令 (30003端口)

### 通用说明

- **坐标系参数**：笛卡尔指令中的 `User` 和 `Tool` 可选参数指定坐标系，不传则使用全局坐标系。
- **速度参数**：`SpeedJ/SpeedL/AccJ/AccL` 可选参数指定本指令的速度/加速度比例。
    - **实际比例** = 指令参数 × 控制软件再现值 × 全局速率
    - 未指定则使用全局设置（见Dashboard设置指令）。
- **使用限制**：
    - 不支持在指令中携带 `CP` 参数，请使用 `CP(R)` 指令。
    - 不支持携带 `SYNC` 参数，请使用 `Sync()` 或 `SyncAll()`。

---

### 3.1 指令列表

#### MovJ
- **原型**：`MovJ(X,Y,Z,Rx,Ry,Rz,User=index,Tool=index,Speed=R,AccJ=R)`
- **描述**：关节运动至笛卡尔坐标目标点（非直线，所有关节同时完成）。
- **参数**：目标点 `X,Y,Z,Rx,Ry,Rz` (double)。
- **返回**：`ErrorID,{},MovJ(...);`
- **示例**：`MovJ(-500,100,200,150,0,90,AccJ=50)`

#### MovL
- **原型**：`MovL(X,Y,Z,Rx,Ry,Rz,User=index,Tool=index,Speed=R,AccL=R)`
- **描述**：直线运动至笛卡尔坐标目标点。
- **返回**：`ErrorID,{},MovL(...);`
- **示例**：`MovL(-500,100,200,150,0,90,SpeedL=60)`

#### JointMovJ
- **原型**：`JointMovJ(J1,J2,J3,J4,J5,J6,SpeedJ=R,AccJ=R)`
- **描述**：关节运动至关节坐标目标点。
- **参数**：目标点 `J1~J6` (double)。
- **返回**：`ErrorID,{},JointMovJ(...);`
- **示例**：`JointMovJ(0,0,-90,0,90,0,SpeedJ=60,AccJ=50)`

#### MovLIO
- **原型**：`MovLIO(X,Y,Z,Rx,Ry,Rz,{Mode,Distance,Index,Status},...,{Mode,Distance,Index,Status},User=index,Tool=index,SpeedL=R,AccL=R)`
- **描述**：直线运动并在运动过程中并行设置DO。
- **参数**：
    - 目标点：`X,Y,Z,Rx,Ry,Rz` (double)。
    - IO参数组 `{Mode,Distance,Index,Status}`：
        - `Mode` (int): 0-距离百分比，1-距离数值。
        - `Distance` (int): 正数表离起点，负数表离终点。Mode=0时范围(0,100]；Mode=1时单位mm。
        - `Index` (int): DO编号。
        - `Status` (int): 0/1。
- **返回**：`ErrorID,{},MovLIO(...);`
- **示例**：`MovLIO(-500,100,200,150,0,90,{0,50,1,0})`

#### MovJIO
- **原型**：`MovJIO(X,Y,Z,Rx,Ry,Rz,{Mode,Distance,Index,Status},...,{Mode,Distance,Index,Status},User=index,Tool=index,SpeedJ=R,AccJ=R)`
- **描述**：关节运动并在运动过程中并行设置DO。
- **参数**：同`MovLIO`。
- **返回**：`ErrorID,{},MovJIO(...);`
- **示例**：`MovJIO(-500,100,200,150,0,90,{0,50,1,0})`

#### Arc
- **原型**：`Arc(X1,Y1,Z1,Rx1,Ry1,Rz1,X2,Y2,Z2,Rx2,Ry2,Rz2,User=index,Tool=index,SpeedL=R,AccL=R)`
- **描述**：圆弧插补运动。通过当前位置、P1（中间点）、P2（目标点）确定圆弧。
- **参数**：P1点 `X1...Rz1`, P2点 `X2...Rz2`。
- **返回**：`ErrorID,{},Arc(...);`
- **示例**：`Arc(-350,-200,200,150,0,90,-300,-250,200,150,0,90)`

#### Circle3
- **原型**：`Circle3({X1,Y1,Z1,Rx1,Ry1,Rz1},{X2,Y2,Z2,Rx2,Ry2,Rz2},count,User=index,Tool=index)`
- **描述**：整圆插补运动，运动指定圈数后回到起点。通过当前位置、P1、P2确定圆。（六轴3.5.5+）
- **参数**：P1点 `{...}`, P2点 `{...}`, `count` (int) - 圈数。
- **返回**：`ErrorID,{},Circle3(...);`
- **示例**：`Circle3({-350,-200,200,150,0,90},{-300,-250,200,150,0,90},1,User=0,Tool=0)`

#### ServoJ
- **原型**：`ServoJ(J1,J2,J3,J4,J5,J6,t,lookahead_time,gain)`
- **描述**：基于关节空间的动态跟随（六轴独有）。建议33Hz调用频率。
- **参数**：
    - `J1~J6` (double): 目标关节角度。
    - `t` (float, 可选): 点位运行时间(s)，[0.02, 3600.0]，默认0.1。
    - `lookahead_time` (float, 可选): 提前量，[20.0, 100.0]，默认50。
    - `gain` (float, 可选): 位置比例增益，[200.0, 1000.0]，默认500。
- **返回**：无
- **示例**：
    ```
    ServoJ(0,0,-90,0,90,0,t=0.1,lookahead_time=50,gain=500)
    ServoJ(0,0,-89,0,90,0,t=0.1,lookahead_time=50,gain=500) // 寸动
    ```

#### ServoP
- **原型**：`ServoP(X,Y,Z,Rx,Ry,Rz)`
- **描述**：基于笛卡尔空间的动态跟随（六轴独有）。建议33Hz调用频率。
- **返回**：无
- **示例**：
    ```
    ServoP(-500,100,200,150,0,90)
    ServoP(-499,100,200,150,0,90) // X轴寸动
    ```

#### MoveJog
- **原型**：`MoveJog(axisID, CoordType=typeValue, User=index, Tool=index)`
- **描述**：点动机械臂。再次下发 `MoveJog()` 停止。（六轴3.5.2+，四轴1.5.6+）
- **参数**：`axisID` (string) - 如 `J1+`, `J2-`。也可下发任意非指定string停止。
- **返回**：`ErrorID,{},MoveJog(...);`
- **示例**：
    ```
    MoveJog(J2-)
    MoveJog() // 停止
    ```

#### StartTrace
- **原型**：`StartTrace(traceName)`
- **描述**：轨迹拟合运动。使用轨迹文件拟合路径运动。需先运动到首个点位。（六轴3.5.2+）
- **参数**：`traceName` (string) - 轨迹文件名。
- **返回**：`ErrorID,{},StartTrace(traceName);`
- **示例**：
    ```
    GetTraceStartPose(recv_string)
    MovJ(x,y,z,rx,ry,rz)
    StartTrace(recv_string)
    ```

#### StartPath
- **原型**：`StartPath(traceName, const, cart)`
- **描述**：轨迹复现运动。复现录制的轨迹。需先运动到首个点位。（六轴3.5.2+）
- **参数**：
    - `traceName` (string): 轨迹文件名。
    - `const` (int): 1-匀速复现，0-原速复现。
    - `cart` (int): 1-笛卡尔路径，0-关节路径。
- **返回**：`ErrorID,{},StartPath(...);`
- **示例**：
    ```
    GetPathStartPose(recv_string)
    JointMovJ(j1,j2,j3,j4,j5,j6)
    StartPath(recv_string,0,1)
    ```

#### Sync
- **原型**：`Sync()`
- **描述**：阻塞程序，等待队列中最后的指令执行完。
- **返回**：`ErrorID,{},Sync();`
- **示例**：
    ```
    MovL(...)
    Sync()
    RobotMode()
    ```

#### RelMovJTool
- **原型**：`RelMovJTool(offsetX, offsetY, offsetZ, offsetRx, offsetRy, offsetRz, Tool, SpeedJ=R, AccJ=R, User=index)`
- **描述**：沿工具坐标系进行相对关节运动。（六轴3.5.2+）
- **参数**：偏移量 `offsetX~offsetRz` (double)，`Tool` (int) - 工具坐标系索引。
- **返回**：`ErrorID,{},RelMovJTool(...);`
- **示例**：`RelMovJTool(10,10,10,0,0,0,0)`

#### RelMovTool
- **原型**：`RelMovTool(offsetX, offsetY, offsetZ, offsetRx, offsetRy, offsetRz, Tool, SpeedL=R, AccL=R, User=index)`
- **描述**：沿工具坐标系进行相对直线运动。（六轴3.5.2+）
- **返回**：`ErrorID,{},RelMovTool(...);`
- **示例**：`RelMovTool(10,10,10,0,0,0,0)`

#### RelMovJUser
- **原型**：`RelMovJUser(OffsetX,OffsetY,OffsetZ,OffsetRx,OffsetRy,OffsetRz,User,SpeedJ=R,AccJ=R,Tool=Index)`
- **描述**：沿用户坐标系进行相对关节运动。（六轴3.5.2+，四轴1.5.6+）
- **参数**：`User` (int) - 用户坐标系索引。
- **返回**：`ErrorID,{},RelMovJUser(...);`
- **示例**：`RelMovJUser(10,10,10,0,0,0,0)`

#### RelMovLUser
- **原型**：`RelMovLUser(OffsetX,OffsetY,OffsetZ,OffsetRx,OffsetRy,OffsetRz,User,SpeedL=R,AccL=R,Tool=Index)`
- **描述**：沿用户坐标系进行相对直线运动。（六轴3.5.2+，四轴1.5.6+）
- **返回**：`ErrorID,{},RelMovLUser(...);`
- **示例**：`RelMovLUser(10,10,10,0,0,0,0)`

#### RelJointMovJ
- **原型**：`RelJointMovJ(Offset1,...,Offset6,SpeedJ=R,AccJ=R)`
- **描述**：各关节进行相对关节运动。
- **参数**：`Offset1~Offset6` (double) - 各关节角度偏移量 (°)。
- **返回**：`ErrorID,{},RelJointMovJ(...);`
- **示例**：`RelJointMovJ(10,0,-10,0,0,0)`

#### MovJExt (四轴)
- **原型**：`MovJExt(Angle|Distance, SpeedE=50, AccE=50, Sync=1)`
- **描述**：控制滑轨（扩展轴）运动（四轴独有）。
- **参数**：
    - `Angle|Distance` (float): 目标角度(°)或距离(mm)，取决于扩展轴设置。
    - `SpeedE` (int, 可选): 速度比例 [1,100]，默认100。
    - `AccE` (int, 可选): 加速度比例 [1,100]，默认100。
    - `Sync` (int, 可选): 0-异步，1-同步。
- **返回**：`ErrorID,{},MovJExt(...);`
- **示例**：`MovJExt(300)`

#### SyncAll (四轴)
- **原型**：`SyncAll()`
- **描述**：阻塞程序，等待队列中**所有**指令（含扩展轴）执行完。（四轴独有）
- **返回**：`ErrorID,{},SyncAll();`
- **示例**：
    ```
    MovJ(...)
    MovJExt(...)
    SyncAll()
    RobotMode()
    ```

---

## 4. 实时反馈信息 (30004/30005/30006端口)

通过30004、30005、30006端口接收实时状态信息，数据包为**1440字节**，格式如下：

| 含义 | 数据类型 | 值数目 | 字节大小 | 字节位置 | 描述 |
| :--- | :--- | :--- | :--- | :--- | :--- |
| MessageSize | unsigned short | 1 | 2 | 0000 ~ 0001 | 消息字节总长度 |
| N/A | unsigned short | 3 | 6 | 0002 ~ 0007 | 保留位 |
| DigitalInputs | uint64 | 1 | 8 | 0008 ~ 0015 | 当前数字输入端子状态 |
| DigitalOutputs | uint64 | 1 | 8 | 0016 ~ 0023 | 当前数字输出端子状态 |
| RobotMode | uint64 | 1 | 8 | 0024 ~ 0031 | 机器人模式 |
| TimeStamp | uint64 | 1 | 8 | 0032 ~ 0039 | Unix时间戳 (ms) |
| N/A | uint64 | 1 | 8 | 0040 ~ 0047 | 保留位 |
| TestValue | uint64 | 1 | 8 | 0048 ~ 0055 | 0x0123456789ABCDEF |
| N/A | double | 1 | 8 | 0056 ~ 0063 | 保留位 |
| SpeedScaling | double | 1 | 8 | 0064 ~ 0071 | 速度比例 |
| N/A | double | 1 | 8 | 0072 ~ 0079 | 保留位 |
| VMain | double | 1 | 8 | 0080 ~ 0087 | 控制板电压 |
| VRobot | double | 1 | 8 | 0088 ~ 0095 | 机器人电压 |
| IRobot | double | 1 | 8 | 0096 ~ 0103 | 机器人电流 |
| N/A | double | 1 | 8 | 0104 ~ 0111 | 保留位 |
| N/A | double | 1 | 8 | 0112 ~ 0119 | 保留位 |
| N/A | double | 3 | 24 | 0120 ~ 0143 | 保留位 |
| N/A | double | 3 | 24 | 0144 ~ 0167 | 保留位 |
| N/A | double | 3 | 24 | 0168 ~ 0191 | 保留位 |
| QTarget | double | 6 | 48 | 0192 ~ 0239 | 目标关节位置 |
| QDTarget | double | 6 | 48 | 0240 ~ 0287 | 目标关节速度 |
| QDDTarget | double | 6 | 48 | 0288 ~ 0335 | 目标关节加速度 |
| ITarget | double | 6 | 48 | 0336 ~ 0383 | 目标关节电流 |
| MTarget | double | 6 | 48 | 0384 ~ 0431 | 目标关节扭矩 |
| QActual | double | 6 | 48 | 0432 ~ 0479 | 实际关节位置 |
| QDActual | double | 6 | 48 | 0480 ~ 0527 | 实际关节速度 |
| IActual | double | 6 | 48 | 0528 ~ 0575 | 实际关节电流 |
| ActualTCPForce | double | 6 | 48 | 0576 ~ 0623 | TCP传感器力值 |
| ToolVectorActual | double | 6 | 48 | 0624 ~ 0671 | TCP笛卡尔实际坐标值 |
| TCPSpeedActual | double | 6 | 48 | 0672 ~ 0719 | TCP笛卡尔实际速度值 |
| TCPForce | double | 6 | 48 | 0720 ~ 0767 | TCP力值 (通过关节电流计算) |
| ToolVectorTarget | double | 6 | 48 | 0768 ~ 0815 | TCP笛卡尔目标坐标值 |
| TCPSpeedTarget | double | 6 | 48 | 0816 ~ 0863 | TCP笛卡尔目标速度值 |
| MotorTemperatures | double | 6 | 48 | 0864 ~ 0911 | 关节温度 |
| JointModes | double | 6 | 48 | 0912 ~ 0959 | 关节控制模式 (8:位置, 10:力矩) |
| VActual | double | 6 | 48 | 0960 ~ 1007 | 关节电压 |
| HandType | char | 4 | 4 | 1008 ~ 1011 | 手系 |
| User | char | 1 | 1 | 1012 | 用户坐标系 |
| Tool | char | 1 | 1 | 1013 | 工具坐标系 |
| ... | ... | ... | ... | 1014 ~ 1037 | 运行标志、比例、状态等 (详见原文档) |
| Load | double | 1 | 8 | 1168 ~ 1175 | 末端负载重量 (kg) |
| CenterX/Y/Z | double | 3 | 24 | 1176 ~ 1199 | 负载偏心距离 (mm) |
| SixForceValue | double | 6 | 48 | 1304 ~ 1351 | 当前六维力数据原始值 |
| TargetQuaternion | double | 4 | 32 | 1352 ~ 1383 | 目标四元数 [qw,qx,qy,qz] |
| ActualQuaternion | double | 4 | 32 | 1384 ~ 1415 | 实际四元数 [qw,qx,qy,qz] |
| N/A | char | 124 | 124 | 1416 ~ 1440 | 保留位 |

> **注意**：完整字段请参考原文档第72-76页。

---

## 5. 通用错误码

| 错误码 | 描述 | 备注 |
| :--- | :--- | :--- |
| 0 | 无错误 | 下发成功 |
| -1 | 命令接收失败/执行失败 | |
| -10000 | 命令错误 | 下发的命令不存在 |
| -20000 | 参数数量错误 | |
| -30001 | 第一个参数类型错误 | 格式 `-30000` + 参数序号 |
| -30002 | 第二个参数类型错误 | 格式 `-30000` + 参数序号 |
| -40001 | 第一个参数范围错误 | 格式 `-40000` + 参数序号 |
| -40002 | 第二个参数范围错误 | 格式 `-40000` + 参数序号 |

---
**文档结束**