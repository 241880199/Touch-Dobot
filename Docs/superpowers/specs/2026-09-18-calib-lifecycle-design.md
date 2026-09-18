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

> ⚠ **本节机制已被 §7.1 取代，不要照此实现。** 终审发现它有一个 Critical 接缝
> （TCP 过了闸门却从不写时间戳，每次启动被销毁），用户决定改用「启动自检」。
> 保留本节只为记录演进过程。

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

### 3.5 `'i'` 移除：符号只由程序判定

**设计修订（2026-09-18，实机评审后）。** 初版设计给 `'i'` 留了人工兜底。用户明确否决了这个方向：

> `'i'` 应该是对数据符号的判断吧，这种判断需要程序计算解决，**不要引入人工判断**。

所以 `'i'` 连同 `forcedSignZ` / `flipComSignZ()` / `clearForcedSignZ()` 一并删除。符号**只**由 §3.2 的物理判据算出。

程序**算不出来**的情形（`signAmbiguous`，两个候选落在同一侧）不再"沿用旧约定"，而是**判为结果不合理**（§3.6）——由程序说"这批数据判不了符号"，让操作者重新采集，而不是让它猜一个。

> 顺带消除一个诚实的报告问题：初版 3a/3c 的展示代码把 `!signAmbiguous` 当作"自动判定"，人工强制时（`forcedSignZ != 0` 且 `ambiguous == false`）会把人工选择标成"自动"。移除人工模式后这个歧义不复存在。

### 3.6 合理性硬判据与连续失败升级

**设计修订（2026-09-18）。** 用户要求：

> `'m'` 采集足够数据计算后，需要程序给出对计算结果合理性的判断；如果不合理需要再次采集，**多次不合理需要报错**。

**三条判据，任一命中即"不合理"：**

| 判据 | 条件 | 原状 |
|---|---|---|
| 拟合残差超阈值 | `rmsForceN >= 0.30 N` | 已是硬闸（实机实测：约定对 ~0.06 N，错 ~0.9 N） |
| 符号无法判定 | `signAmbiguous` | 原为"沿用旧约定 + 告警" → 改为不合理 |
| 物理质心 Z ≤ 0 | `cTrueZChosen <= 0` | 原为告警 → 改为不合理 |

**明确排除：质量修正幅度。** 初版有「修正超过当前值 30% 就告警」，用户否决：

> 不报警，这也许是加装了别的设施，**正符合重新标定的作用**。

这是对的——`Config.h` 的种子值只是 STL 估计，第一次实机标定本来就该大改（本项目的首次解算就是 `0.66 → 0.41 kg`，-38%）。把大修正当异常，等于禁止重新标定发挥作用。

**不合理时的动作：一律拒绝保存和下发**，机械臂保持原参数。绝不拿一个程序自己都判定为不可信的结果去配置机械臂。

**连续失败升级：** 每次不合理记一笔；**连续 3 次**打红字错误，提示检查硬件/环境，**并停止接受新的 `'s'`**，直到重新进入 `'m'`（重新采集清零计数）。成功一次即清零。

> 它防的是"操作者反复按 `'s'` 却每次都被拒"的空转——那种情况下真正的问题不在求解器，而在硬件或采集方式，继续试只是浪费时间。

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
- `main.cpp` —— 6 处路径调用 + 符号判定输出 + 数据新鲜度 + 合理性三判据 + 连续失败升级
- `force/PayloadCalibration.{h,cpp}` —— 符号双候选 + 保存时间戳；**`Result` 三个新字段加默认初始化**（§3.6 之前那颗"未初始化的地雷"的根因）
- `force/ForceCalibration.cpp` —— 路径 + 保存时间戳
- `config/Config.h` —— `CALIB_MAX_AGE_SEC`；连续失败阈值 `CALIB_MAX_CONSECUTIVE_FAILS = 3`
- `Touch_Client.vcxproj` —— 加入新源文件
- `tests/test_payload_calibration.cpp` —— 符号判定用例
- `Docs/调零与负载标定流程.md` —— 修正"翻符号后旧数据作废"（§3.5 修订后此说完全作废）；补充 24h 规则与新目录；键位表删除 `'i'`

**删除（§3.5 修订）：** `'i'` 键处理、`PayloadCalibration::forcedSignZ` / `flipComSignZ()` / `clearForcedSignZ()`、`test_forced_sign_overrides_auto` 用例。

**不做：** 日志类文件落点不变。

### 第二轮修订追加（§7）

**删除：** `saved_at_unix` 字段（payload / force）、`Config::CALIB_MAX_AGE_SEC`、`CalibStore::isFresh()`、`.expired` 改名机制、`CalibStore::resolve()` 的有效期语义。

**保留并简化：** `CalibStore` 只负责**位置**——`dir()` / `fileFor()` 保留，`resolve()` 退化为"存在性检查 + 返回路径"或直接删除（调用方改用 `fileFor`）。

**新增：**
- `config/Config.h` —— `SIGN_SEED_MARGIN_RATIO = 3.0`（符号判定的余量判据）
- `PayloadCalibration::solve()` —— 符号改按「离种子更近 + 距离比 ≥ 3 + 选中的 c > 0」选取
- **启动自检** —— 启动时引导摆 2~3 个姿态，用跨姿态极差复用 `'m'` 复验的判据；不通过则作废并要求重标
- `main.cpp` —— 自检的启动流程接入 + 复用 `BiasCheck` 的采集/报告数学

**测试：** `test_calib_store` 删除 `isFresh` / 过期改名相关用例；新增符号锚点判定的用例（含余量不足 → 不可判定）。

**文档：** `Docs/调零与负载标定流程.md` 删除 24h 一节，改为描述启动自检。

---

## 7. 第二轮修订（2026-09-18，终审后）

终审（覆盖全部 15 个提交）报出 1 Critical + 2 Important，其中两条源于本设计本身：

### 7.1 有效期：用「实测一致性」取代 24h 时间闸门

**触发原因（Critical）：** `tcp_calib.json` 被 §1 的调用点表接进了 `CalibStore::resolve`，
但 §2 的时间戳清单只列了 payload / force —— 于是 TCP 偏移**过了闸门却从不写时间戳，
每次启动都被"过期"作废销毁**。根因是本设计 §1 与 §2 的接缝没对齐。

**用户决定的修法不是补时间戳，而是换掉整个机制：**

> 可以检查启动静止时标定后的数据是否与已有参数相同。

**移除：** `saved_at_unix` 字段、`CALIB_MAX_AGE_SEC` 时间闸门、`.expired` 改名机制。
**保留：** `CalibStore` 的**位置解析**（从可执行文件推导 `calib\` 目录）——它与有效期无关，仍然有价值。

**新机制 —— 启动自检：**

```
启动 → 使能 → 力数据流就绪
  → calib\ 下存在已标定的负载/零偏？
      否 → 用种子值，提示按 'm' 标定
      是 → 引导操作者摆 2~3 个姿态（跨度≥30°，笔朝下/水平/朝上），逐个 SPACE
           → 复用与 'm' 复验相同的跨姿态极差分析
              通过   → 标定继续有效，进入正常操作
              不通过 → **作废该标定**（回退种子值/清零零偏），要求重标
```

**为什么不是单姿态：** 质量误差体现在力通道（`|ΔF| = |Δm|·9.81`），单姿态可测；
但质心误差只体现在力矩通道，响应 ∝ `sinθ`，**静止单一姿态下它是定值，混在零偏里分不开**。
所以自检至少要 2~3 个姿态、且要覆盖倾角。

**已知代价（已与用户确认并接受）：** 每次启动都要摆姿态。
与直接跑 `'m'`→`'s'` 的工作量相当，区别是自检不求解、不写盘。

**C1 顺带解决：** TCP 偏移是纯几何量、不参与一致性检查，也就不再被任何闸门销毁。

### 7.2 符号判定：种子锚点 + 余量判据

**触发原因（Important）：** 终审证明 §3.2 的判据**永远选不出与当前配置相反的符号**。
代数上，令 `X = m_cfg·cz/1000`：

```
plusOk  ⟺ dp_z > −X
minusOk ⟺ dp_z > +X
X > 0 时 {dp_z > X} ⊂ {dp_z > −X}  ⟹  minusOk 蕴含 plusOk
⟹ 非歧义分支恒为「选 +1」，即 sChosen ≡ sign(comCfg[2])
```

**它只是"确认或拒绝"，不是"判定"。** 且失败模式很糟：若机器人真按 −1 解释而配置是 +1，
两候选变成 `+327.3 / +67.8`（**都是正的**）→ 判歧义 → 永久拒绝，
还提示去查"装夹松动 / 传感器受挤压 / 姿态覆盖不足"——**没一条是真正的原因**。

**用户问：** "理论上通过多个位置与力数据不就可以得到所有的坐标与力参数了吗？"

**答：** 数据能完全解出 `Δm` 与 `Δp`，但 `Δp = p真 − p配置` 是**差值**；
`signZ` 是"固件如何解释我们下发的 cz"，**在差值里被消掉**，再多姿态也不在数据里。
破环有两条路：外部锚点，或改变配置做两次测量。**采用前者。**

**新判据：**

```
候选 c(+1) / c(−1) 的物理质心 Z
  → 选离 Config::ROBOT_PAYLOAD_SEED_CZ_MM 更近的
  → 必须同时满足：
       · 距离比 ≥ SIGN_SEED_MARGIN_RATIO (= 3)
       · 选中的 c > 0  (物理约束交叉检查)
  → 任一不满足 → 符号不可判定 → 不合理 → 拒绝下发
```

**余量判据为什么必要：** 选错的临界点是种子偏离真值超过两候选间距的一半。
间距 `= 2·m_cfg·cz/m_true`，实机为 **259.4 mm** → 临界 **129.7 mm**；
当前种子 80.4 对真值 67.9，误差 **12.5 mm**，余量约 10 倍。
但换装差异大的工具后若不重跑 `compute_payload.py`，种子可能失真——余量判据就是为此设的闸。

实机两种情形都已验算：真值 +1 → 距离 `12.5 vs 272`（比 21.8）✓；
真值 −1 → `12.6 vs 246.9`（比 19.6）✓。

`Config::ROBOT_PAYLOAD_SEED_CZ_MM` 从"仅首次兜底"升级为**判定的外部锚点**，其准确性开始承担判定责任。
