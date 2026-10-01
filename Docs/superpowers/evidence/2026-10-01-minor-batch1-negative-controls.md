# Minor 收口第一批 —— 负对照证据（入库副本）

> 原件在 `.superpowers/sdd/minor-batch1-report.md` §D（该目录被 `.gitignore:30` 忽略 ⇒ 不进版控）。
> 本文件是它 §D 的**入库副本**，逐字复制，**未增删、未重构任何输出**。
> 复制范围 = §D 三条负对照的**描述 + 复现做法/命令 + 红原文 + 还原后复跑的全绿输出 + 基准哈希**。
> 唯一改动是版式（加标题层、把原件里的 `####` 提为 `###`）；文字与代码块一字未动。

---

### 关于「命令」的一点如实说明

原件 §D 对三条负对照只记了**共同的做法**（见下引文）与**共同的复现命令**（见文末 §H 引文），
**没有为每一条单记一条命令行**。这里照实复制，**未替它补造**任何命令。

原件 §D 原文（做法）：

> 每条：改坏实现 → 重建+跑 → 抄红 → **还原**（从改动前的备份副本 `cp` 回来，并 `sha256sum` 对齐）。

改动前基准哈希（三条共用）：

```
a4e9716d7f34d0367750a40990d1b9e3e435ebf76463ff718e78ac747d617de1 *relay/GainReadback.h
```

---

### 负对照 1：删掉 `SkipUnchanged` 分支里的 `m_pending.store(false);`

预期：格 3 红。**实测红原文：**

```
  first_throttled_sends... PASS
  forced_always_sends... PASS
  unchanged_clears_pending... FAIL: !s.pending()
  too_soon_sets_pending_without_advancing... PASS
  pending_resend_after_window... PASS
  forced_bypasses_unchanged... PASS
  clock_wraparound... PASS

6 passed, 1 failed
EXIT=1
```

✅ 与简报预期**逐字命中**（`FAIL: !s.pending()`）。还原后哈希 = 基准。

---

### 负对照 2：把 `SkipTooSoon` 整个并进 `Send`（删掉该 case，让它落到发送）

预期：格 4 红。**实测红原文：**

```
  first_throttled_sends... PASS
  forced_always_sends... PASS
  unchanged_clears_pending... FAIL: !s.beginSend(SendMode::Throttled, 130.0, 1010)
  too_soon_sets_pending_without_advancing... FAIL: !s.beginSend(SendMode::Throttled, 130.0, 1099)
  pending_resend_after_window... FAIL: !s.beginSend(SendMode::Throttled, 130.0, 1099)
  forced_bypasses_unchanged... PASS
  clock_wraparound... PASS

4 passed, 3 failed
EXIT=1
```

✅ 简报点名的**格 4** `FAIL: !s.beginSend(SendMode::Throttled, 130.0, 1099)` **命中**。
⚠ **一处比简报更宽**：简报说“格 4 红”，实测**格 3 / 格 4 / 格 5 一起红** —— 因为这三格都对
“被挡下的 too-soon 调用”断言 `!beginSend(...)`，同一处合并会让它们**同时**命中。这是**观察到的实况**，
不是缺陷（多一格报警只会更早暴露）。还原后哈希 = 基准。

---

### 负对照 3：在 `beginSend` 开头加 `mode = SendMode::Throttled;`（把 Forced 降级）

预期：格 2 与格 6 红。**实测红原文：**

```
  first_throttled_sends... PASS
  forced_always_sends... FAIL: s.beginSend(SendMode::Forced, 120.0, 1000)
  unchanged_clears_pending... PASS
  too_soon_sets_pending_without_advancing... PASS
  pending_resend_after_window... PASS
  forced_bypasses_unchanged... FAIL: s.beginSend(SendMode::Forced, 120.0, 1010)
  clock_wraparound... PASS

5 passed, 2 failed
EXIT=1
```

✅ 与简报预期**逐字命中**（格 2 + 格 6）。还原后哈希 = 基准。

---

### 还原后复跑全绿（Step 6 要求）

```
BUILD_EXIT=0
...
7 passed, 0 failed
EXIT=0
```

且 `sha256sum` 与基准逐字相同（`a4e9716d...d617de1`）⇒ 确认实现回到简报原文、无残留改动。

---

### 原始报告的复现命令（§H 原文，三条负对照共用）

```
# 单套件
cd /d/Projects/Touch/Touch_Client/tests
cmd //c ".\build_gain_readback_state_test.bat"   # 期待 BUILD_EXIT=0
cmd //c ".\test_gain_readback_state.exe"          # 期待 7 passed, 0 failed ; exit 0

# 整床
cmd //c ".\run_tests.bat"                          # 期待 exit 0 ; Suites accounted: 27 of 27 (ran 25 + not-run 2)
```

---

### 全绿基准（§C 原文，负对照所用实现的绿）

```
BUILD_EXIT=0
=== GainReadbackState Tests ===
  first_throttled_sends... PASS
  forced_always_sends... PASS
  unchanged_clears_pending... PASS
  too_soon_sets_pending_without_advancing... PASS
  pending_resend_after_window... PASS
  forced_bypasses_unchanged... PASS
  clock_wraparound... PASS

7 passed, 0 failed
EXIT=0
```
