# 22 条 Minor 收口（第一批：可离线验证的 9 条）—— 设计

**日期**：2026-09-30 · **分支**：`fix/offline-testbed-seams`
**清单**：`.superpowers/sdd/minor-findings-rollup.md`（2026-09-22 的 MATLAB 可调增益那一批的终审遗留）

## 0. 起点：先回核，再设计

清单是 **2026-09-22** 写的，此后 09-23 / 09-24 / 09-29 / 09-30 都落过改动。
**2026-09-30 逐条对着当前 HEAD 回核过一遍**（只读），结论：

| 结局 | 条数 | 明细 |
|---|---|---|
| **已关闭** | 2 | **20**（`sendToClient` 断线已出声：`relay_gui.m` 的 `notifyDropped`）· **22**（"运行失败分支"的负对照 **2026-09-24 已刻意做过** —— 把 `test_calib_store.cpp` 的 `main()` 改成 `return 1`，套件自报 3/0 而 harness 打 `[FAIL]`、`exit=1`；见 `Docs/superpowers/specs/2026-09-22-test-harness-state.md:335-377`） |
| **已不可恢复** | 1 | **21** —— 它指的是 harness Task 7 报告 `§7` 的 `@echo off` 计数，而那个路径现在是**按钮2 Task 7 的报告**；原报告在**未跟踪**的 `.superpowers/` 下，git 里没有 ⇒ 无法逐字核。**顺带记**：现行 `run_tests.bat` 的 `@echo off` 已是 **26 行 / 24 个判定块**，14/15 早已整体过期 ⇒ 这一条**按其自身的"计数必须来自重跑的命令"标准，已自动作废** |
| **作者当时即判"可接受"** | 3 | **3** · **6** · **10**（现状与描述一致，见 §3 的归宿表） |
| **仍成立、可动** | 15 | 1 · 2 · 5 · 7 · 8 · 9 · 11 · 12 · 13 · 14 · 15 · 16 · 17 · 18 · 19 |
| **回核后由控制方改判为「已缓解」** | 1 | **4** —— 回核把它记成"仍成立"，但**控制方亲自读原文**后改判：那一句**已经**给了算式、**且自己写明**"0.25 s 是此刻的结果、范围一变它就变" ⇒ "**静默**失效"不成立。见 §1 的说明 |

⇒ **本批做 8 条**；其余按 §3 逐条写明归宿。

> ⚠ **与用户最初批的口径差一条（第 4 条）**，我回核后**改判**，理由如下（请在设计评审时判定）：
> `Config.h:407-408` 现在写的是「走完 GAIN_MIN→GAIN_MAX 的时长 = (GAIN_MAX − GAIN_MIN) / 本值，
> **当前 ≈ 0.25 s。⚠ 0.25 s 是这个算式【此刻】的结果，不是设计目标 —— 本值或范围一变它就变。**」
> ⇒ 它**已经**给了算式、并**自己标明了**那个数字会随范围变 ⇒ 清单担心的"**静默**失效"这半**不成立**；
> 而且同一段上方刚说过"那两个数就是符号的副本，范围一改就静默过期"—— 与本处这个**带警告的派生值**
> 是两回事（派生值不是任何符号的副本）。⇒ **归宿 = 接受**（不改）。若你要求连这个数字也去掉，
> 那就是 9 条，加一行即可。

## 1. 本批范围（8 条）

`1` · `2` · `5` · `9` · `12` · `16` · `17` · `18`。

**唯一需要设计判断的是 12**（§2）；其余 7 条都是"如实化"，**不改任何可执行语义**。

## 2. ★ 第 12 条：把「三样状态 + 迁移」抽成可测单元（唯一的设计项）

### 2.1 问题（回核后的精确版）

`relay/GainReadbackPolicy.h` 已经把**判决**抽成了纯函数并有 5 格用例，但：

1. **判决够不到的那半仍无覆盖** —— 三样状态的**迁移**（`s_lastGainReportMs` / `s_lastSentGain` 的推进时机、
   `s_gainReportPending` 的置与**顺手清**）全在 `RelayCore.cpp:2794-2846` 里，而
   **`RelayCore.cpp` 不被任何测试编译**（`grep RelayCore tests/*.bat` 只命中 rem 注释）。
2. **调用点传的实参没人管** —— 判决测试只看**形参**。把某个调用点的 `force` 传错，
   5 格用例一个都不会红。**这正是 2026-09-22 真实发生的事故**（限频被传成 `force=true` ⇒ **静默死掉**，
   测试床一声不响）。

### 2.2 形状

新增 **header-only** 单元 `Touch_Client/relay/GainReadback.h`（照 `GainReadbackPolicy.h` 的先例：

无 .cpp、不读时钟、不碰 socket、不写文件）。

```cpp
namespace GainReadback {

// ★ 参数类型化：调用点不再传裸 bool。
//   今天的 `sendReflectionGain(bool force)` 在调用点读作 `sendReflectionGain(true)` —— 一个 bool
//   字面量，**看不出语义、编译器也管不着**。事故就长在这个形状上。
enum class SendMode {
    Forced,      // 重连/首次的强制回读；【被拒绝的增益改动】也必须走这一档（见下 ⚠）
    Throttled,   // 拖动洪水与 pollRelayCommands 的补发：走"值变了 + 距上次发送 ≥ 窗口"两道闸
};

// 判决 + 状态迁移，一步做完。时钟与时钟值都由调用方给 ⇒ 用例不必睡。
// 返回 true ⇒ 调用方【现在】应当真的把这条回读发出去。
class State {
public:
    bool beginSend(SendMode mode, double g, unsigned long nowMs);

    // pollRelayCommands 每帧问它：有没有被限频挡下、还欠 MATLAB 一条？
    bool pending() const;
    // ⚠ **【不提供】公开的 clearPending()**（2026-09-30 写计划时删掉，YAGNI）：
    //   那个标志【只】在判决/发送分支里被改（抽取前就是这样），**没有任何外部调用方需要清它**。
    //   加一个没人调的公开方法，正是复审会打回的形状。

    // 只读回显，供用例与诊断（不参与判决）。
    double lastSentGain() const;
    unsigned long lastReportMs() const;

private:
    // ⚠ 三个成员仍是 atomic —— 两个线程（GLUT idle 线程与 pollRelayCommands）不同步地碰它们，
    //   非原子对象上的不同步读写是 UB。**这条约束与今天逐字相同，抽取不改变它**，
    //   头文件里把"为什么"照抄下来（今天的理由写在 RelayCore.cpp:2760-2793，抽取后要跟着搬）。
    std::atomic<unsigned long> m_lastReportMs{0};
    std::atomic<double>        m_lastSentGain{0.0};   // 初值 0 = setGain 不接受的值 ⇒ 第一次 Throttled 一定发得出去（保守方向）
    std::atomic<bool>          m_pending{false};
};

} // namespace GainReadback
```

### 2.3 迁移（RelayCore 侧只剩"取值 / 组包 / 发送"）

```cpp
static GainReadback::State s_gainReadback;   // 存储期与今天相同（文件级 static）

void RelayCore::sendReflectionGain(GainReadback::SendMode mode) {
    const DWORD now = GetTickCount();
    const double g = ForceTuning::gain();
    if (!s_gainReadback.beginSend(mode, g, now)) return;   // 被挡下 ⇒ 标志已由 beginSend 管好
    ... snprintf 载荷（【逐字不动】）...
    sendRelayUpdate(buf);
}
```

- **`beginSend` 的迁移规则必须与今天逐条一致**（这是本条的硬要求，逐条写进用例）：
  - `Forced` ⇒ 一定发；`pending` 清；`lastReportMs` / `lastSentGain` 都推进。
  - `Throttled` + 值未变 ⇒ **不发，且【顺手清掉 pending】**（今天的 `:2808-2812`；理由：A→B 被挡后值又变回 A，
    留着标志会让 `pollRelayCommands` 每帧空转、标志从此失去意义）。
  - `Throttled` + 值变了 + 未到窗口 ⇒ 不发，**置 pending**，**且【不】推进 `lastReportMs`**
    （它记的是"上次**真的发出去**"的时刻）。
  - `Throttled` + 值变了 + 过了窗口 ⇒ 发，两个时刻/值都推进，pending 清。
  - ⚠ **"发"这一步的含义**：两个状态都落笔在**发送调用之前** ⇒ socket 恰在此刻失效时这条算"发过了"。
    这是**已知且能收敛**的（重连强制回读会把当前值原样再送一条）。
    **抽取不改变它** —— 头文件里把这句照抄，别让它变成"抽取时顺手改对"的牺牲品。

### 2.4 用例（新增 `tests/test_gain_readback_state.cpp` + `build_gain_readback_state_test.bat`，接进 `run_tests.bat`）

用**注入的 `nowMs`** 驱动，**不睡**。至少钉住：

| # | 格 | 期望 |
|---|---|---|
| 1 | 首次 `Throttled`（`lastSentGain` 初值 0） | **发**（保守方向） |
| 2 | `Forced` | **发**（且不看时刻） |
| 3 | `Throttled` 值未变 | 不发；`pending() == false`（**顺手清**） |
| 4 | `Throttled` 值变了但未到窗口 | 不发；`pending() == true`；**`lastReportMs` 不变** |
| 5 | 承接 4：过了窗口再来 | 发；`pending()` 清 |
| 6 | A→B（被挡）→ 值回到 A | 不发；**`pending() == false`**（第 3 格的场景化，今天注释点名的那个坑） |
| 7 | `Forced` 无视"值未变" | 发（**被拒绝的增益改动**那条路依赖它） |
| 8 | 时刻回绕（`now` 小于 `lastReportMs`） | 无符号相减 ⇒ 判"未到窗口"而不是"过了几十亿毫秒" |

**负对照（每条都要实测红）**：至少三条 —— ① 把"未变时清 pending"删掉 ⇒ 第 6 格红；
② 把 `Throttled` 的窗口判断去掉（恒定发）⇒ 第 4 格红；③ 把 `Forced` 也过一道值闸 ⇒ 第 7 格红。

### 2.5 ⚠ 这一条**不会**被自动化覆盖的部分（如实记账）

- `sendRelayUpdate(buf)` 那一句、以及 7 个字段的**顺序**仍然只靠人工审读
  （消费方在 MATLAB；`RelayCore.cpp` 仍不被任何测试编译）。
- 抽取**不新增**对 `RelayCore.cpp` 的覆盖 —— 它只是把**能被覆盖的那半**搬出去并测起来。
  ⇒ 报告里不许写成"`RelayCore` 从此有测试了"。

## 3. 其余 8 条的处置（本批）与另外 13 条的归宿

### 3.1 本批的其余 7 条（**不改可执行语义**）

> ⚠ **下表的 `file:line` 是「设计时」的，可能已被本批自己的提交移动。** 本批已落地的提交：
> `8f15048`（第 1/2/5/9/17 条）· `210168c` + `bc97e7f`（第 12 条）· `9a563dd`（第 16 条）·
> `8baf1d4`（第 18 条）· `83f52cb`（控制方订正，不是某条的修）。**要引用行号请回核当前值，别照抄本表。**
> 已回核订正的两处：第 1 条（`RelayCore.cpp` 的行号与 `SendMode` 写法）、第 16 条（用例名）。

| # | 文件 | 做法 |
|---|---|---|
| **1** | `main.cpp:4145-4146` | 句子仍过宽（"重连，或一次被**接受**的增益改动"）⇒ 改为"重连，或一次增益改动（**接受与拒绝都会回读**）"（依据：拒绝分支 `RelayCore.cpp:2864` 的 `sendReflectionGain(GainReadback::SendMode::Forced)`） |
| **2** | `main.cpp:2944-2954` | "三者之一"只断言结论 ⇒ 补自证："`canStart()` 的第四条失败路已在上面两个守卫里被排除"（`ForceCalibration.cpp:167-171,198`） |
| **5** | `ForceTuning.cpp:86-87` | `fopen` 失败把"不存在"与"打不开"混为一谈 ⇒ **只改措辞**（如实写明这是"没读到"，并指出启动横幅仍会打 `未采用 <path>`，操作员并非全盲）。⚠ **不改 API 加返回值**：清单自己也把它定性为措辞问题，且 `loadFromFile` 有多个调用方，加 `enum` 属扩面 |
| **9** | `test_force_pipeline.cpp:165-168` | 注释只说"两个字面量就是当前值" ⇒ 补上**双向**事实：**收窄**由本文件的 `CHECK(setGain(...))` 钉、**放宽**由兄弟套件 `test_force_tuning.cpp:91-94`（用字面量 `gain() == 100.0/300.0` 反向钉死两端点）钉 ⇒ 并**订正清单里"放宽仍全绿"那半句**（**控制方已亲自读该文件确认**：`GAIN_MIN` 若放宽到 50，那条 `fabs(gain() - 100.0) < 1e-9` 必红） |
| **12** | 见 §2 | ★ |
| **16** | `test_force_tuning.cpp` | ① 临时文件 `_tuning_test_tmp.json` 在 `CHECK` 早退时**泄漏到 `tests/`** ⇒ 加清理（用例出口无条件删；或改成 `setStorePathForTest` 指到系统临时目录下的唯一名）；② `test_missing_file_is_quiet_false`（`9a563dd` 后改名 `test_missing_file_returns_false`）的名字/注释声称 "quietly" 而**只断言返回 `false`** ⇒ **改名去掉 quiet（并对齐注释）**，不假装断言了 stderr |
| **17** | `test_relay_command_parser.cpp:96-98` | 注释引 `ForceTuning::GAIN_MIN/GAIN_MAX` 而该文件**故意不 include** ⇒ 把它写成**显式的"非编译器强制"依赖**（点名"改 `ForceTuning` 的名字要回来改这条注释"），或改引到 `test_force_tuning` 已钉住的那条断言 |
| **18** | `Touch_Client.vcxproj:138` | `core\JsonLite.h` 从 `<!-- force -->` 块挪到已存在的 `<!-- core -->` 块（纯分组，零构建影响 ⇒ **必须实测能编过**） |

### 3.2 本批**不做**、且已写明归宿的 14 条

| # | 归宿 | 理由 |
|---|---|---|
| **20** | ✅ **已关闭** | 回核确认已修 |
| **22** | ✅ **已关闭** | 回核确认 09-24 已刻意反证过 |
| **21** | ⛔ **作废（不可恢复）** | 所指报告不进 git；且现行 `run_tests.bat` 的计数已整体过期 |
| **4** | **接受（控制方回核后改判）** | 见 §1；不再是"静默"失效 |
| **3 · 6 · 10** | **接受**（作者原判） | 现状与描述一致；6 由重连强制回读自愈、10 由兄弟套件兜住不变式 |
| **7 · 8 · 11 · 13 · 14 · 15 · 19** | **第二批（上机 / 有 MATLAB 时）** | 全在 `Relay_Station/relay_gui.m`。<br>⛔ **2026-10-01 订正：「本机没有 MATLAB」是【错的】** —— 实测 `D:\Software\Matlab` 装着 **R2025b (25.2.0.3177638)**，`matlab -batch` 能跑。⇒ 当初那条"改了只能人工审读、不当作已改"的**前提不成立**；本仓自己的上机单里本来就写着 `matlab -nosplash -nodesktop -r "relay_gui"` ⇒ **同一份仓库自相矛盾**。**本组仍按"未改"记账**（因为确实还没改），但**现在有真正的验证路径了**。<br>其中 **13**（`AllowEmpty` 无 `isprop` 回退、老 MATLAB 构造期整窗起不来）与 **15**（清空后按 `[Default]` 逼不出回读）**是行为项**；**19** 在文件内**自相矛盾**（`:272` 声称完整显示 vs 清单说被截断）⇒ **必须上机看渲染才能判**。 |

### 3.3 清单文件本身的收口

`.superpowers/sdd/minor-findings-rollup.md` **末尾追加一张「22 条结局表」**：
每条一行 —— 结局（已修 / 已关闭 / 接受 / 第二批 / 作废）+ **一句依据**（含 file:line 或提交号）。
⇒ 收口的定义是**每条都有一个成立的归宿**，不是"全变成绿的"。

## 4. Global Constraints

- **除了 §2 的类型化（`bool` → `SendMode`）与 §2.3 的迁移，不改任何可执行语义**；
  尤其 **RS 载荷的 7 个字段顺序、`snprintf` 格式串、三样状态的初值与推进时机逐字不动**。
- **不许**顺手改：`GainReadbackPolicy.h` 的判决、`GAIN_REPORT_MIN_INTERVAL_MS`、`ForceTuning` 的取值/API、
  MATLAB 侧的 `relay_gui.m`（本批**一个字都不动**，见 §3.2）。
- 每步之后 **整床 exit 0 且 `Suites accounted: N of N`**；新增套件必须**同时**接进 `run_tests.bat`
  （build + run 两处）并让计数断言跟着变（本仓的"运行时数出 `test_*.cpp`"机制会自动算）。
- ⚠ **本仓最怕假绿**：新用例必须断言**具体值**；**每条新判据都要有一条负对照实测红**（§2.4 那三条是底线）。
- ⚠ **plan-mandated 也要审**：如发现本设计的某条与代码事实不符，**停下来记，不照抄**。
- C++ 注释中文；`.bat` 纯 ASCII。
- 新增的 `GainReadback.h` 与 `GainReadbackPolicy.h` 的**关系要写清**（一个管判决、一个管状态），
  免得后来人把两者合并或让判决绕过状态。

## 5. 明确不在本批
- `relay_gui.m` 的任何改动（7 条）。
- 任何"上机才能定"的判据（`RG|` 真实链路、0.25s 斜坡手感、`[Zero]` 回显、真鼠标拖动，
  见清单 §"已知的、非 Minor 但必须上机验证的"）。
- 第 3/6/10 条的"改法"（已判接受）。
- 第 21 条的抢救（不可恢复）。

## 6. 执行后：22 条结局表

收口的定义是**每条都有一个成立的归宿**，不是"全变成绿的"。

| # | 归宿 | 依据 |
|---|---|---|
| 1 | 已修（本批） | `8f15048` —— `main.cpp` 那句改成"**接受与拒绝都会回读**" |
| 2 | 已修（本批） | `8f15048` —— `main.cpp` 补自证：`canStart() ≡ !isRunning()`，由 `isForceCalibrating()` 守卫排除 |
| 3 | **接受** | `ForcePipeline.cpp:238`（`shutdown()` 归零 `g_gainRamp`）；与既有 `g_filters` 复位同类，作者原判可接受 |
| 4 | **接受**（**控制方回核后改判**） | `Config.h:407-408` 已给算式 + 自标"范围一变它就变" ⇒ 非**静默** |
| 5 | 已修（本批） | `8f15048` —— `ForceTuning.cpp` 把 `fopen` 失败的两义性写进注释 + 指向启动横幅的 `未采用 <path>` |
| 6 | **接受** | `GainReadback.h:64-70`（两样状态落笔在发送调用**之前**）⇒ 由重连的强制回读自愈；作者原判 |
| 7 | ✅ **已修（2026-10-01）** | `relay_gui.m` 两行启用/范围日志**去掉尾部的"当前 %.1f"**（它与同一拍下面的 ⚠ 会互相打脸）—— `1a48449`。⚠ 证据只有 `checkcode` **无语法错**，**运行时行为未验** |
| 8 | ✅ **已修（2026-10-01）** | `relay_gui.m` 把"队列里躺着的那条"改成**实际机制**（`SkipTooSoon` **压制** + 记一个 `pending` 标志，由下次轮询用**当时的当前值**补发，不是排队的旧值）—— `1a48449` |
| 9 | 已修（本批） | `8f15048` —— `test_force_pipeline.cpp:165-168` 补双向事实并订正"放宽仍全绿"那半句 |
| 10 | **接受** | 不变式由 `test_force_tuning.cpp:108` 的 `static_initial_value_is_legal` 兜住；作者原判 |
| 11 | ✅ **已修（2026-10-01）** | `relay_gui.m` 给"**C++ 只有一个拒收理由**"这个前提**补上钉住它的断言指针** —— `Touch_Client/tests/test_force_tuning.cpp` 的 `test_set_gain_bounds`（`CHECK(!setGain(99.9))/(300.1)/(nan)/(inf)`）—— `1a48449`。★ **写之前回代码核过**：`ForceTuning.cpp:39` 确实只有 `if (!inRange(v)) return false;` 一条拒收路 |
| 12 | 已修（本批） | `210168c`（`GainReadback.h`，三样状态 + 迁移抽成可测单元）+ `bc97e7f`（调用点换 `SendMode`）。**接线那层仍无自动化覆盖**（见 §2.5） |
| 13 | ✅ **已修（2026-10-01）** | `relay_gui.m` 把 `AllowEmpty`/`Placeholder` **从构造参数里拿出来**、改成构造后按 `isprop` 决定（构造参数一旦不支持就是**构造期抛错 ⇒ 整窗起不来**；改后老版本上窗口照常起、只退回"框里显示默认数"那一档）—— `1a48449` 之后的那次提交。<br>★ **证据**（`../evidence/2026-10-01-scripts/probe_allowempty_fallback.m` 实测）：**同一段代码形状跑两遍**（走支持分支 / 强制走回退分支）**两条都建得出控件** ✓；外加 `checkcode` 无语法错。<br>⚠ **边界**：本机 R2025b **支持** `AllowEmpty` ⇒ **复现不出老 MATLAB 的构造期失败**，那一条只有老版本上才能证 |
| 14 | ✅ **已修（2026-10-01）** | `relay_gui.m` 拖动/松手那一段**补注"不是保证"**（C++ 回读**按值变化门控** ⇒ 那条**可能根本不来**；是诊断缺口、不是承诺违背）—— `1a48449` |
| 15 | **第二批**（`relay_gui.m`，**行为项**） | `relay_gui.m:543`（`onGainDefault`）—— 清空后按 `[Default]` 逼不出回读 |
| 16 | 已修（本批） | `9a563dd` —— `main()` 出口 `atexit` 兜底清理 + 用例改名（`test_missing_file_returns_false`） |
| 17 | 已修（本批） | `8f15048` —— `test_relay_command_parser.cpp` 补"非编译器强制引用"告警 |
| 18 | 已修（本批） | `8baf1d4` —— `core\JsonLite.h` 归到 `<!-- core -->` |
| 19 | **第二批**（`relay_gui.m`） | 文件内自相矛盾（`relay_gui.m:272` 声称完整显示 vs 清单说被截断）⇒ **必须上机看渲染** |
| 20 | ✅ **已关闭** | `relay_gui.m:688-689,715` 的 `notifyDropped` / `tlog('DROP')` —— 断线已出声 |
| 21 | ⛔ **作废（不可恢复）** | 所指的 harness Task 7 报告不进 git；且现行 `Touch_Client/tests/run_tests.bat` 已有 **25** 处跟随 `call` 的 `@echo off`（`:65`–`:751`；含 `:1` 与 `:48` 散文里的字面量共 **27** 处），报告里那个计数已整体过期 |
| 22 | ✅ **已关闭** | "运行失败分支"的负对照 2026-09-24 已刻意做过（`Docs/superpowers/specs/2026-09-22-test-harness-state.md:335-377`） |

（合计：已修 **13** · 接受 **4** · 第二批 **2** · 已关闭 **2** · 作废 **1** = **22**。）

> ★ **2026-10-01 更新**：第 **7 · 8 · 11 · 14 · 13** 条已修 —— **已修 8 → 13，第二批 7 → 2**（余 **15 · 19**）。
> **13** 的做法：`AllowEmpty`/`Placeholder` 移出构造参数、改由 `isprop` 决定；
> ★ 并用 `../evidence/2026-10-01-scripts/probe_allowempty_fallback.m` **把回退路径真的走了一遍**
> （强制走"不支持"分支 ⇒ 控件照样建得出来）✓。
> ⚠ 但**本机 R2025b 支持 `AllowEmpty`** ⇒ **老 MATLAB 的构造期失败复现不出来**，那条只有老版本能证。
> ⚠ 这四条原先挂着的"**本机无 MATLAB ⇒ 改了只能人工审读**"是**错的**：本机装着 **R2025b (25.2.0.3177638)**、
> `matlab -batch` 能跑，`checkcode('relay_gui.m')` 也能跑。
> ★ 而且**第 13 条的原始描述自己就引用了本机 `VersionInfo.xml`** ⇒ **那句假前提是后来在设计文档里长出来的**，
> 还一路传到计划/rollup/提交说明（**转述链条**又一例）。
> ⚠ **证据边界如实记账**：这四条改动的验证**只有 `checkcode`（无语法错、无新增告警）**，
> **运行时行为（日志输出 / GUI 交互）没有验证** —— 那要真的把 `relay_gui` 跑起来看。

⚠ 依据要么是**提交号**、要么是 **file:line**；本表不含"应该已经修了"这类话。
