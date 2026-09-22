# 力反射增益 —— MATLAB 界面可调

日期：2026-09-22
状态：设计已定，待写实施计划

## 1. 要解决的问题

力反馈那条链路里，`Config::FORCE_REFLECTION_GAIN` 是"阻力大小"的总旋钮 —— 也就是用户说的
"阻力的大小再慢慢调"。但它现在是 `Config.h` 里的编译期 `const double`：

```
compensated ─ 死区(0.20N) ─ ×(3.3/200) ─ 硬夹(±3.3N) ─ 逐轴符号 ─ ×FORCE_REFLECTION_GAIN(=120) ─→ hapticOut
```

⇒ 每调一次手感都要改代码、重新编译、重启客户端。现场调参的成本高到没人愿意调。

要做的：**把这个增益变成能在 MATLAB 界面上实时改的值**，改完立刻生效、跨重启保留。

## 2. 范围

### 做
- **只做总增益 `FORCE_REFLECTION_GAIN` 一个系数**

### 不做（本次刻意）
- **逐轴系数**（X/Y/Z 三路各自的乘性系数）—— 用户选择"先只做总增益"
- **死区 `FORCE_RESIDUAL_DEADZONE_N` / 饱和上限 `FORCE_MAX_TOUCH_N`**
- **把增益写进 `force_demo_log.csv` 的列**。刻意不做，理由要留存：列数一变就撞上
  "读取器对未知列数**静默截断**"那个陷阱（见 `Docs/superpowers/plans/2026-09-22-offline-fixes.md`
  里收口项的陷阱一）。上一个计划收口时正是它差点让收口"看起来做完了却什么都没验证"。
  真要进日志，另开一个任务认真做。

## 3. 架构

新增模块 `force/ForceTuning.{h,cpp}`：

```cpp
namespace ForceTuning {
    double gain();          // atomic load —— 目标值；进程内所有人读到的都是它
    bool   setGain(double); // 校验 [0,300] → 过则 store + 标脏；返回是否被接受
    bool   loadFromFile();  // 只在 main.cpp 启动时调一次；读不到/非法 → 保持默认
    void   tick();          // 防抖落盘：值变过 且 静默 ≥1s 才写盘
}
```

- 值是 `static std::atomic<double> s_gain{ Config::FORCE_REFLECTION_GAIN };`
  **静态初值直接来自 `Config.h`，不读文件。**
- 落盘走 `CalibStore::fileFor("force_tuning.json")` → `<...>\Touch_Client\calib\force_tuning.json`。
  沿用已有的"标定目录"解析，**不新造一套路径规则**（这套规则当初就是为了治
  "从 `Touch_Client\` 启动和从 `x64\Release\` 启动会拿到两份文件"才建的）。
- 文件内容 `{"version":1, "reflection_gain":120.0}`。
- 读失败 / 缺文件 / 值越界或非有限数 → **保持默认 + 响亮打一句**，不静默回退。

### 为什么是独立模块，而不是把 `Config.h` 改成 `extern`

`Config.h` 的自述是"C++ 侧编译时常量"，全项目 5 处（含 4 个测试）直接读它。改成 `extern` 后
测试要额外链接，而且这个头文件会获得"运行时状态"的语义，与它自己的定位打架。

### 为什么不用 `AppState` 上的一个 atomic 字段

概念最少，但 `AppState.h` 是纯数据头，放不下"校验 + 落盘 + 读文件"；那部分最后要么塞进已经
2418 行的 `RelayCore.cpp`，要么另开模块 —— 兜一圈还是这个形状，只是值存了两处。

### 关键：测试不被现场调参污染

`gain()` 的**静态初值**就是 `Config::FORCE_REFLECTION_GAIN`，**不读文件**；
只有显式 `loadFromFile()`（只在 `main.cpp` 调一次）才改它。

否则"昨天现场拖到 200"会让测试红绿漂移 —— 这个坑项目里已经有过一次同形态的
（`test_force_pipeline.cpp:98` 那句注释："那两个常数被重调时它会一直绿"）。

⇒ **测试不调 `loadFromFile()`。** 这条要写进测试文件顶部。

### 接入点（三行）

| # | 位置 | 改动 |
|---|---|---|
| 1 | `ForcePipeline.cpp:133` | `Config::FORCE_REFLECTION_GAIN` → `ForceTuning::gain()` |
| 2 | `main.cpp` 启动处 | `ForceTuning::loadFromFile();` |
| 3 | `RelayCore::pollRelayCommands()` 开头 | `ForceTuning::tick();`（借现成的空闲循环当防抖心跳，不新起线程/定时器） |

`Config::FORCE_REFLECTION_GAIN = 120.0` **保留不动**，语义改成"出厂默认 / 兜底值"，
它那一大段注释补一句写清优先级：`force_tuning.json` > `Config.h`。
`ForceTuning.h` 里也要写同一句 —— **两处都要写**，否则下一个人改 `Config.h` 会"改了没反应"。

### 线程安全

`ForcePipeline::step` 在 ForceReader 线程（实测 122.9 Hz），命令来自 GLUT `idle()` 线程，
落盘也在 `idle()`。`atomic<double>` 的 load/store 就够 —— 只有一个标量，
不存在"读两个相关的量"那种需要锁的情形。

## 4. 协议

已占用的前缀（核过）：`B C D F FB G H J L P RP S W`。
⚠ **`G|` 已被占用**（`relay_gui.m:506`，安全预判的 z 距离 + 奇异标志）—— 不要用它。

新增只用 `RG|`：

| 方向 | 格式 | 例 |
|---|---|---|
| MATLAB → C++ | `RG\|<value>` | `RG\|200` |
| C++ → MATLAB | `RG\|<gain>,<min>,<max>,<ratio>,<satN>,<defGain>` | `RG\|120.0,0.0,300.0,1.98,1.67,120.0` |

- `gain` = 当前生效的**目标值**（不是斜坡的瞬时值，见 §6①）
- `ratio` = 净比例 = `(FORCE_MAX_TOUCH_N / FORCE_MAX_SENSOR_N) × gain`
- `satN` = 打顶阈值 = `FORCE_MAX_TOUCH_N / ratio`（传感器牛顿，超过它就不再区分强弱）
- `defGain` = 出厂默认 = `Config::FORCE_REFLECTION_GAIN`（"恢复默认"按钮发这个值）
- **范围、净比例、打顶阈值、默认值全部由 C++ 算好下发**，MATLAB 侧一个魔数都不用写：
  滑条上下限、错误提示里的数字、"净比例 ≈1.98:1"那行显示、饱和提示、"恢复默认"按下的目标值，
  全部来自这一条消息。否则这些数就要在"C++ 的校验"和"MATLAB 的滑条"各写一份然后漂移 ——
  这个项目对重复常量漂移有明确教训。

⚠ 字段有 6 个，是有意的：每多一个"MATLAB 自己算"的量，就多一处会漂的重复常量。
   `defGain` 就是自查时补上的 —— 界面草图里画了 `[Default]` 按钮，正文却从没定义它靠什么值。

`Z|` 也核过是空的（已占用前缀里没有 `Z`）。

### 自愈语义（本设计的重点）

`RG|` 是**唯一**的真值通道，两个方向共用。C++ 在三个时刻发回读：

1. 连接建立、以及每次自动重连成功后（挂在 `ensureRelayConnected` / `initRelayReporting` 上）
2. 每收到一条 `RG|` 命令、处理完之后 —— **不论接受还是拒绝，回的都是当前实际生效的目标值**
3. 启动完成时

⇒ MATLAB **永远不"记住"自己设过什么**，它只显示 C++ 说的数。
若它发了 500 被拒，回读仍是 120，滑条**自己弹回 120**，并用同一条消息里的 min/max 生成一句
日志说清原因。

⇒ **「GUI 显示的值 ≠ 实际生效的值」这个状态，在结构上无法存在。**

### 回读限频

拖动滑条可能产生 50+ 条/秒的命令。C++ **只在目标值真的变了、且距上次回读 ≥100ms** 才回读，
复用 `tick()` 所在的位置。否则几十条/秒的 `RG|` 会堆在 MATLAB 侧。

## 5. MATLAB 界面

位置：`pnlFF`（Force Output 那格，`glMid` 第 3 行）。它内部 grid 从 `[3 1]` 扩成 `[4 1]`，
`RowHeight` `{22, 26, '1x'}` → `{22, 26, 28, '1x'}`，插一行：

```
Reflection Gain              [≈1.98:1]   [传感器 >1.67N 即饱和]
[========|=========]  [ 120 ]  [Default]  [Zero]
   滑条            数值框   恢复默认    调零（再按=中止）
```

行为：

- 拖动 → `ValueChangingFcn` 实时发 `RG|<value>`（拖的过程手上就能感觉到变化）
- 松手、或数值框提交 → `ValueChangedFcn` 再发一次（幂等，保证最后一次一定到）
- `[Default]` → 发回读里的 `defGain`（**不硬编码 120**）
- `[Zero]` → 发 `Z|1`（见 §7）
- 收到回读 → 更新滑条与数值框
- **拖动进行中要挡住回写**：否则回读会和正在拖的滑块打架。
  `S.tuningDragging` 标志 —— 拖动中只更新文字（净比例/饱和提示），松手后由回读对齐。
- **还没收到过回读时滑条禁用**，显示"等待 C++" —— 不猜上下限。

## 6. 四项缓解（针对调参本身带来的风险）

### ① 增益斜坡

gain 从 120 拖到 300，手上力在一帧内变 2.5 倍。加斜率限制，让它 **0.25 秒内平滑到位**
（`FORCE_GAIN_RAMP_S = 0.25`，线性 slew；常数定义在 `Config.h`，与 `FORCE_GRADIENT_LIMIT` 并列）。

这不是新发明 —— `FORCE_GRADIENT_LIMIT = 50 N/frame` 就是这个项目已有的同类保护（防传感器尖峰），
只是它作用在 `filtered[]` 上、**管不到增益之后**。

**归属**：`ForceTuning` 只管**目标值**（校验/持久化）；斜坡是**信号处理**，
放 `ForcePipeline::step`，与 `FORCE_GRADIENT_LIMIT` 同类。

⇒ **回读报的是目标值，不是斜坡的瞬时值** —— 否则界面上的数字会自己动。

### ② 回读限频
见 §4。

### ③ 显示饱和阈值
见 §5。把"当前这一档还剩多少分辨力"变成屏幕上的一个数。

### ④ 启动横幅写值 + 来源

C++ 启动时打一行：

```
[Tuning] 力反射增益 = 200.0（来源：calib/force_tuning.json；Config.h 默认 120.0）
```

这条最便宜，也最直接地治掉"改了 `Config.h` 发现没反应"。

## 7. 调零按钮

`g_noRobot` 和 `cancelOtherCaptureModes` 都是 `main.cpp` 的 **file-static**，
`RelayCore` 拿不到。所以不能让 RelayCore 复制一份守卫链（那正是最忌讳的重复）。做法：

- `dispatchRelayCommand` 收到 `Z|` → 只置 `std::atomic<bool> m_forceZeroRequested`
- `main.cpp` 的 `idle()` 里（紧跟 `pollRelayCommands()` 那行）读标志 →
  调**与 `'z'` 键完全同一个函数**
- ⇒ 两条入口（键盘、MATLAB）汇成一条，守卫链只有一份
- 按钮语义**与 `'z'` 一致：再按一次 = 中止**。不在 GUI 里发明第二种语义。
- C++ 的处置文字（"调零中: 保持机械臂静止…"）走 `C|` 回显到 MATLAB 的命令日志面板。
  操作员在 MATLAB 前面**看不到 stdout**，不回显就等于什么都反馈不了。

## 8. 测试

新增 `tests/test_force_tuning.cpp` + `tests/build_force_tuning_test.bat`。

| 断言 | 为什么 |
|---|---|
| `setGain` 范围：0 与 300 接受；−0.1 / 300.1 / NaN / inf 拒绝**且值不变** | 边界 |
| **`setGain` 真的改变了 `ForcePipeline::step` 的输出** | 这条才是"旋钮真的接上了"的证据。只断言 `setGain` 的返回值不构成证据 |
| `gain()` 静态初值 == `Config::FORCE_REFLECTION_GAIN` | 防默认值悄悄漂走 |
| 落盘 → 读回，往返一致 | 持久化 |
| 文件损坏 / 缺文件 / 值越界 → 保持默认 | 回落路径 |
| 解析器：`RG\|200` 接受；`RG\|` / `RG\|abc` / `RG\|200x` / 未知前缀 拒绝 | 协议 |
| `RG\|500` 被拒后 `gain()` 仍为原值 | 拒收语义 |

⚠ **测试不调 `loadFromFile()`**（理由见 §3）。

⚠ 斜坡的影响：那条"真的改变了 `step` 的输出"要**跑够帧数收敛**（0.25s × 125Hz = 32 帧）。

`RelayCommandParser` 的改法（刻意保守）：

```cpp
enum class Command { None, ForceFeedbackOn, ForceFeedbackOff, SetReflectionGain, ForceZero };
Command parse(const char* line, double* valueOut = nullptr);
```

第二个参数带默认值 ⇒ **现有 12 条测试一个字都不用改**，照旧编译、照旧当回归网；
新命令另加测试。

**不**重构成通用的 `type|value` 结构 —— 那要重写现有测试，而收益是零（一共就三条命令）。

### 测试床接线

- `Touch_Client.vcxproj` 加 `force\ForceTuning.h` / `force\ForceTuning.cpp`
- `run_tests.bat` 加一个"先建再跑"的段

⚠ **`run_tests.bat` 有两个手工维护的数字必须跟着改**（`:346-347` 的注释自己写了
"这两个数是手工维护的，没有东西在算它们"）：

- `:338` 的 `of 20` → `of 21`
- `:351` 的 `8 of the 20` → `7 of the 21`

漏了就会变成"看着做完了、其实没验证"。

## 9. 风险，如实说

### 能管的（已按 §6 处置）
力跳变（① 斜坡）· 回读风暴（② 限频）· 饱和点不可见（③ 显示）· "改了没反应"（④ 横幅）

### 只能管、消不掉的
- **静止残余放大**。这不是缺陷，它就是"输出 = 输入 × 净比例"的定义本身。
  `comp_z` 曾到 −0.52 N ⇒ gain 300 时手上 **−2.6 N 常驻力**。
  **唯一**的软件解是让死区跟着增益缩放（gain 300 时死区 0.5 N），但那会吃掉真实的小笔压信号
  （实测笔压只有 0.3~1.15 N）⇒ **刻意不解**，用"界面提示 + 调零按钮（§7）"来管。
- **饱和点提前**。净比例 > 1 且总夹 ±3.3 N，必然有打顶点；提高总夹不安全。
  只能按 ③ 显示出来。

### 行为变更，要记账
MATLAB 断开后 C++ **保持最后的值**，不回退。所以"MATLAB 关了但 C++ 还在跑，手感还是上次调的"
是**期望行为**，不是故障。这条要写进注释。

### 代价如实记
拖动即生效，手上力会实时变化。总夹 ±3.3 N 一直在（`HapticCallback.cpp:228`），不会失控；
但从 120 拖到 300 的那一刻手上力会变成 2.5 倍（斜坡把它摊到 0.25 秒里）。

## 10. 优先级与落盘时机

- **优先级**：`calib/force_tuning.json` > `Config.h` 的 `120.0`
- 落盘时机：值变过 且 静默 ≥1s。**代价如实说：拖完立刻杀进程，最后 1 秒的改动会丢。**
