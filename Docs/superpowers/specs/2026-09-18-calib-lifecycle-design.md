# 标定文件生命周期与符号自动判定 — 设计

**日期:** 2026-09-18 · **分支:** `feat/pen-clamp-redesign`

## 背景

实机联调暴露了三个问题，它们互相纠缠：

1. **标定文件放哪不确定。** 三个标定文件都用相对路径读写（`"payload_calib.json"`），落在**当前工作目录**。exe 在 `x64\Release\`，但从 `Touch_Client\` 启动时文件写在 `Touch_Client\`。换个目录启动就换一份文件——烟测里已经出现过 `[Force] No calibration file`，而同一时刻 `Touch_Client\force_calib.json` 是存在的。

2. **标定结果永不失效。** 力传感器零偏随温度和时间漂移；负载解算又依赖力数据。一份几天前的标定会被当作权威值无限期使用。

3. **CZ 符号约定要靠人工三步试错。** 现有流程是：求解 → 复验 → 发现残差变大 → 按 `'i'` → **重新采集** → 再求解。而"重新采集"这一步是**不必要的**（见 §3.1），白跑一轮实机。

另外，`f7a5d46` 为修掉"复验假 FAIL"引入的 `reset()` 过于粗暴——它把数据清空，导致 `'i'` 重解无数据可用。本设计替换掉它（§4）。

## 目标

- 标定文件固定位置，不随 cwd 变化
- 超过 24h 的标定一律作废，启动时显眼提示
- CZ 符号自动判定；`'i'` 降级为兜底覆盖，且不再需要重新采集

**非目标：** 不改日志类文件（`force_demo_log.csv` / `alarms.log` / `robot_diagnostics.log`）的落点。

---

## 1. 标定文件位置

新增模块 `core/CalibStore.{h,cpp}`，单一职责：**标定文件的生命周期——放哪、还能不能用**。路径解析和有效期判定放在一起，因为它们回答的是同一个问题："该不该用这份标定"。

```cpp
namespace CalibStore {
    // 标定目录绝对路径, 结尾带反斜杠。不存在则创建。
    const char* dir();

    // 解析一个标定文件是否可用。
    // 可用  → 返回绝对路径 (静态缓冲, 进程内有效)
    // 过期  → 重命名为 <name>.expired, 打印醒目提示, 返回 nullptr
    // 不存在→ 返回 nullptr (静默, 首次启动是正常情况)
    const char* resolve(const char* name);

    // 纯函数, 便于单测: 年龄是否在有效期内
    bool isFresh(long savedAtUnix, long nowUnix, long maxAgeSec);
}
```

目录解析：

```
可执行文件 D:\Projects\Touch\Touch_Client\x64\Release\Touch_Client.exe
  GetModuleFileNameA
  → 去掉文件名、上溯两级 → D:\Projects\Touch\Touch_Client\
  → 拼 calib\                 （不存在则 mkdir）
```

结果缓存一次（进程内不变）。

### 调用点改造

把"读得到路径 + 新鲜"合并成一个判断，各个解析器只管解析：

```cpp
const char* p = CalibStore::resolve("payload_calib.json");
if (p && PayloadCalibration::load(p)) { /* 已标定 */ }
else { /* 回退种子值 */ }
```

| 文件:行 | 现在 | 改为 |
|---|---|---|
| `main.cpp:363` | `PayloadCalibration::save("payload_calib.json")` | `save(CalibStore::fileFor("payload_calib.json"))` |
| `main.cpp:823` | `TcpCalibration::save("tcp_calib.json")` | 同上 |
| `main.cpp:1035` | `PayloadCalibration::load("payload_calib.json")` | `CalibStore::resolve(...)` |
| `main.cpp:1069` | `ForceCalibration::loadFromFile("force_calib.json", ...)` | 同上 |
| `main.cpp:1095` | `TcpCalibration::load("tcp_calib.json")` | 同上 |
| `ForceCalibration.cpp:178,309` | `saveToFile("force_calib.json", ...)` | `saveToFile(CalibStore::fileFor(...), ...)` |

写入用 `fileFor(name)`（只拼路径，不做新鲜度判定）。

## 2. 24h 有效期

两个文件各加一个字段记录保存时刻：

- `payload_calib.json`：`version` 1 → 2，新增 `"saved_at_unix": <epoch 秒>`
- `force_calib.json`：`version` 2 → 3，新增同名字段

**判定规则：**

| 情况 | 处理 |
|---|---|
| 有 `saved_at_unix` | 按它算年龄 |
| 缺失该字段 | **视为过期** —— 明确决策：当前标定流程尚未确认正确，不接受任何来历不明的旧文件 |
| 年龄 > 24h（`Config::CALIB_MAX_AGE_SEC = 86400`） | 过期 |

**过期行为：**

1. 不加载 —— 负载回退 `Config.h` 种子值，力零偏清零
2. 文件重命名为 `<name>.expired`（保留不删，便于排查；同名覆盖）
3. 控制台显眼提示，含年龄与下一步按键：

```
[Calib] !! payload_calib.json 已过期 (37.2h > 24h) — 已作废并改名 .expired
[Calib] !! 负载参数回退种子值 mass=0.660kg com=(0.0,0.0,80.4)mm
[Calib] !! 重标: 按 'm' 采多姿态 → 's' 求解;  零偏: 按 'z'
```

时间源用 `time(NULL)`（wall clock）——有效期跨进程、跨重启，不能用 `GetTickCount`（开机计时）。

**关于现有文件的迁移：** 磁盘上现存的 `payload_calib.json` 和 `force_calib.json` 都没有 `saved_at_unix` 字段，因此**首次启动即判定过期**，与放在哪个目录无关。所以**不需要迁移**——按新规则它们本来就是死的。实施时把 cwd 里的旧文件直接删掉即可，避免和新目录混淆。

**已知代价：** wall clock 被回调或改时区会影响判定。对 24h 尺度的人工标定可接受，本次不处理。

## 3. CZ 符号自动判定

### 3.1 出发点：残差无法区分符号

从代码可验证：`signZ` **不进入 `buildRows`**——线性系统的未知量只有 `[Δm, Δp]`，`dg` 与符号无关。`signZ` 只作用于最后的换算：

```
pTrue = pCfg + dp,      pCfg = m_cfg · (cx, cy, cz·signZ)
cTrue = pTrue / m_true
cSend = (cTrue_x, cTrue_y, cTrue_z / signZ)
```

**推论一：两种符号的拟合残差完全相同。** 数据本身不能选符号，必须引入外部判据。

**推论二：翻符号不需要重新采集。** `dp` 由测量数据决定，反映机械臂**当时的实际配置**；翻符号只改变我们如何把它换算成绝对值。只要机械臂配置没变，同一批数据对新符号依然有效。文档里"翻符号后旧数据作废"是错的，要改。

### 3.2 判据：物理质心必须在法兰下方

两个候选的物理质心相差 `2·m_cfg·cz / m_true`。代入实测数据（`m_cfg=0.660`、`cz=80.4mm`、`m_true≈0.409`）是 **259 mm**：

| 候选符号 | 物理质心 Z |
|---|---|
| `+1` | `+67.9 mm` ✓ 法兰下方 |
| `-1` | `-191.6 mm` ✗ 法兰上方，非物理 |

工具挂在法兰下方，质心必须为正。**默认假设：本项目的工具链质心在法兰下方**（已确认）。换装到法兰上方的工具时此判据失效，用 `'i'` 覆盖。

### 3.3 实现

`solve()` 内部同时求两个候选，选 `cTrue[2] > 0` 的那个。`Result` 增加：

```cpp
double signZ;          // 实际选用的符号约定 (+1 / -1)
double cTrueZ[2];      // 两种候选下的物理质心 Z (mm), 供人复核
bool   signAmbiguous;  // 两个候选都合理或都不合理 → 沿用传入的 signZ 并告警
```

歧义时（`signAmbiguous = true`）沿用调用方传入的 `signZ`，并在输出里明确标出，**不静默选择**。

`applyResult()` 把 `r.signZ` 写回 `PayloadCalibration::comSignZ`，随 `payload_calib.json` 的 `com_sign_z` 持久化。

### 3.4 输出

```
  CZ 符号约定: 自动判定 → +1
    · 候选 +1: 物理质心 Z = +67.9 mm  ✓ 法兰下方
    · 候选 -1: 物理质心 Z = -191.6 mm ✗ 法兰上方 (非物理)
```

歧义时：

```
  CZ 符号约定: ⚠ 无法判定 — 两个候选都落在合理区间, 沿用当前 +1
    · 候选 +1: 物理质心 Z = +180.3 mm
    · 候选 -1: 物理质心 Z = +41.7 mm
    (若结果不对, 按 'i' 强制用另一个符号)
```

### 3.5 `'i'` 降级为兜底

`'i'` 强制采用与上次判定相反的符号，然后**立即用同一批数据重解**（走 `solveAndApply()`），不需要重新摆姿态。

防止自动判定覆盖掉人工选择：`PayloadCalibration` 增加覆盖状态

```cpp
double forcedSignZ = 0.0;   // 0 = 自动判定; ±1 = 强制
```

- `'i'` 设置 `forcedSignZ` 为上次 `signZ` 的反号，然后立刻重解
- `forcedSignZ` 在 `BiasCheck::reset()` 里清回 `0`——即"开始新一批采集"时恢复自动判定

## 4. 数据新鲜度不变式

替换 `f7a5d46` 的 `reset()`。核心不变式：

> **数据只有在「机械臂配置 == 采集时的配置」时才可用于复验。**
> 但同一批数据在不同符号约定下的**重解释**始终合法——那不改机械臂配置。

`BiasCheck` 增加：

```cpp
static bool dataUnderCurrentPayload = true;   // 数据是否都在当前生效负载下采
```

| 事件 | 动作 |
|---|---|
| `reset()`（进入 `'m'` 模式 / 开新批次） | 置 `true` |
| `record()` 时该标志为 `false` | 先 `reset()` 开新批次，再置 `true` |
| `solveAndApply()` 下发新负载后 | 置 `false` |
| `report()` 且 `count >= 3` 且标志为 `false` | **拒绝判定**，提示"这批数据是在旧负载下采的，请重新采集" |
| `solveAndApply()` 且标志为 `false` | 允许（`'i'` 重解路径），打印说明：本次是在重解释下发前的测量 |

这样同时满足两个需求：复验假 FAIL 被拦住，`'i'` 重解仍然可用。

## 5. 测试

| 测试 | 内容 |
|---|---|
| `CalibStore::isFresh` | 纯函数边界：刚好 24h、超 1 s、负数年龄、零值 |
| `CalibStore::dir/resolve` | 路径拼接正确；缺失字段判定为过期；目录自动创建 |
| 符号判定 | 合成数据下两种候选 Z 值符合预期；歧义场景置 `signAmbiguous` |
| 回归 | 现有 `test_payload_calibration` 10 项全绿 |

## 6. 影响范围

**新增：** `core/CalibStore.{h,cpp}`、`tests/test_calib_store.cpp`

**修改：**
- `main.cpp` —— 6 处路径调用 + 符号判定输出 + 数据新鲜度
- `force/PayloadCalibration.{h,cpp}` —— 符号双候选 / `forcedSignZ` / 保存时间戳
- `force/ForceCalibration.cpp` —— 路径 + 保存时间戳
- `config/Config.h` —— `CALIB_MAX_AGE_SEC`
- `Touch_Client.vcxproj` —— 加入新源文件
- `tests/test_payload_calibration.cpp` —— 符号判定用例
- `Docs/调零与负载标定流程.md` —— 修正"翻符号后旧数据作废"；补充 24h 规则与新目录

**不做：** 日志类文件落点不变。
