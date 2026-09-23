# 抖动诊断：量出"映射真正吃的那个量"（高优先级，第 1 步只取证）

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development to implement this plan task-by-task.

**Goal:** 拿到两个数 —— ①`hapticOut`（= 手感）在**帧率**上的逐轴 sd；②**残差超过死区的帧占比** —— 用来判定"抖动"是不是"噪声 1.27σ 对 0.20 N 死区"这个机制。

**Architecture:** 不加新探针键。**扩 `'n'` 探针**（它已经是"帧率噪声"的仪器，且已有单测的基础设施）：
在 `ForcePipeline` 里加**一个纯统计累加器**（可单测），由管线在每次 `step()` 时按**新帧**喂入，
`'n'` 按键时一次性打印结果。**不刷屏、不落盘、不改变任何控制行为。**

**Tech Stack:** C++17 / MSVC；测试走 `Touch_Client/tests/`（套件数运行时数出并断言，当前 22）。

## Global Constraints

- **不改变任何控制行为**：本计划只**读**已算出的量、只累加统计量。**不许**改死区、增益、滤波、限幅。
- **不许**按 1 kHz 回调节拍采样（采样保持会污染 sd —— 本项目栽过一次）。**只按帧**。
- `.bat` 纯 ASCII；C++ 注释中文；**回滚 = 一行**（新累加器放在开关后或直接可删，不接控制路径）。
- 每次提交后整床 **exit 0** 且 `Suites accounted: N of N`。

---

### Task 1: `JitterStats` 纯累加器 + 单测

**Files:**
- Create: `Touch_Client/force/JitterStats.h`（纯头文件，含内联实现）
- Create: `Touch_Client/tests/test_jitter_stats.cpp` + `build_jitter_stats_test.bat`
- Modify: `Touch_Client/tests/run_tests.bat`（接线一个段；套件数 22 → 23）

**Interfaces:**
- Produces:
  ```cpp
  struct JitterStats {                       // 只累加，不判断
      void reset();
      void addFrame(const double v[3]);      // 每个【新帧】调一次（不是每次回调）
      int    n() const;
      double mean(int axis) const;
      double sd(int axis) const;             // 样本 sd (÷ n-1)，n<2 ⇒ 0
      double fracAbove(int axis, double thr) const;   // |v[axis]| >= thr 的帧占比
  };
  ```

- [ ] **Step 1: 写失败测试**（真值用**构造序列**，不看实现）
  - 常量序列 ⇒ sd == 0；
  - 已知序列 `{1,2,3,4}` ⇒ mean 2.5、sd = √(5/3) ≈ 1.29099；
  - `fracAbove`：`{0.1, 0.25, -0.3, 0.2}`、thr = 0.2 ⇒ **3/4**（注意 `-0.3` 与边界 `0.2` 都算）；
  - n = 0 时不除零（mean/sd/frac 全 0）；
  - ⚠ 用**真正求值**的那个断言宏（本仓 `TEST` 是标签打印器，**会假绿** —— 2026-09-23 实测复现过）；
    照抄前先在目标文件里核宏的语义。
- [ ] **Step 2: 跑测试确认失败**（未声明）
- [ ] **Step 3: 实现**（两遍算法：先求均值再求偏差平方；`fracAbove` 用 `fabs`）
- [ ] **Step 4: 跑测试确认通过** + **整床**（exit 0、`N of N`）
- [ ] **Step 5: 提交**

---

### Task 2: 接进管线与 `'n'` 探针（只读、可删）

**★ 2026-09-23 按 Task 1 实现者的转交修正了两处（控制方裁定）：**

1. **Task 1 的累加器改成 Welford 在线式**（O(1) 内存、数值同样稳定、**接口不变**）。
   理由：两遍实现必须留全部样本 ⇒ 123 Hz 下 ~30 MB/小时，而它要被接在 `ForcePipeline::step()`
   （**控制路径**）里 ⇒ 帧循环里的 `push_back` 可能重分配，那是**没量过的时序特征变化** ✗。
   ⚠ **不要**给它加"样本数上限"来省内存 —— 那会**静默改变 sd 的含义** ✓（Task 1 实现者拒绝对，照此办理）。
2. ★★ **必须用【两个】实例，别用一个**（这是同一天发现的**静默量错**陷阱）：
   - `statsOut` ← 喂 **`hapticOut[0..2]`**（= 手感，判据 A 用它）；
   - `statsResidual` ← 喂 **补偿后残差**（判据 B 用它，`fracAbove(axis, FORCE_RESIDUAL_DEADZONE_N)`）。
   两者隔着**增益与映射**（`ForcePipeline.cpp` 那三行），**不可互换**；若把死区常数用在上
   `hapticOut` 上，**判据 B 会量在错的量上，而输出长得一模一样** ⇒ 分两段打印、**每段标明是哪个量**。

**Files:**
- Modify: `Touch_Client/force/JitterStats.h`（两遍 → Welford；接口不变）
- Modify: `Touch_Client/force/ForcePipeline.cpp`（每次 `step()` 末尾：把**这一帧**的
  `hapticOut[0..2]` 喂 `statsOut`；把**这一帧的补偿后残差**逐轴喂 `statsResidual`）
  ⚠ **必须确认"这一帧"的判据**（不要去数回调次数）—— 若 `step()` 本身按帧调，直接用它；
  否则用现有的帧新鲜度标志（`isStale` / 帧计数器），并**在报告里写明依据**。
- Modify: `Touch_Client/main.cpp`（`'n'` 的打印块里追加**两段**，形如：
  ```
  ===== A) 输出(手感)抖动 —— 自上次 'n' 起 N 帧 =====
    轴   mean        sd
    x   ...
  ===== B) 补偿后残差 vs 死区 0.20N —— 同一窗口 N 帧 =====
    轴   mean        sd          |残差|>=0.20N 的帧占比
    x   ...
  ```
  —— 数不需要漂亮，要的是真实；**N 必须打**（没有 N 的 sd 是半截信息，本项目有成文教训）。
  两段的标题必须写明是 **A) 输出** 还是 **B) 残差**。）
- [ ] **Step 1: 接线**（上面三处；**不新增按键**、不改任何阈值）
- [ ] **Step 2: 编译 + 负对照**（注入语法错误 ⇒ 必须报 `error C…`；随后还原重编。客户端**已退出**，可直接构建）
- [ ] **Step 3: 整床**（exit 0、`N of N`）
- [ ] **Step 4: 提交**（提交信息写明：这是**取证**、不是修复；回滚 = 删掉累加调用）

## 判据（这一步跑完怎么读）

- **预测 A**：输出的 sd 明显**大于** 11 Hz 落盘口量到的 **0.028 N**（我推算真实速率下 ≈0.15 N）
  ⇒ 证实"D2 用错了仪器"那条订正 ✓；
- **预测 B**：`|残差| >= 0.20 N` 的帧占比 **≈ 15~25%**（1.27σ 的高斯尾）⇒ **机制实锤** ✓；
- **若 B 远低于 15%** ⇒ 我的机制**不成立** ✗ ⇒ 回到"另有其因"，**别按机制去改死区**。

## 明确【不要】做的

- 不修抖动（本计划只取证）；不动 `FORCE_RESIDUAL_DEADZONE_N`；不动增益；不动任何滤波常数。
- 不按回调节拍累加（采样保持会污染 —— 本项目栽过）。
- 不把这段统计落盘成 CSV（`'n'` 那一路的输出已经够；落盘会引入新的 I/O 面）。
