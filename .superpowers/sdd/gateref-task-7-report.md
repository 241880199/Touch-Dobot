# Task 7 报告 —— 参考量"不可用"时 fail-closed

**分支** `feat/pen-clamp-redesign`  **提交** `cc7f10f` `feat(gate): 参考量不可用时 fail-closed, 并与"不一致"可区分`

---

## 0. 一句话

闸门多了一道**存在性守卫**: 参考量这一路**没有数据**时**拒绝**，并且报的是一个**独立状态**
`GuardState::REFERENCE_UNAVAILABLE`（不是 `INCONSISTENT`）。那个"退化的放行"没了；而
`test_payload_calibration` 那条红线断言**仍然红着**（判据 69 passed / 1 failed），
只是它的**描述**从"竟然放行了"改成了"参考量不可用"。

⚠ **性质：这是【防紧】，不是修一个正在发生的 bug。** 生产链路上 `RelayCore` 在同一次 30004
收帧、同一把 `forceDataMutex` 里一起填 `raw[] / tcpForce[] / sixForceRaw[] / sixForceOnline`
⇒ "通道其实有数但读数为零"**在实机目前不可达**；它只在回放 / 夹具路径出现（四份夹具没有
参考量那一路的列）。**本任务没有修任何实机正在发生的故障。**

---

## 1. 实现与判据依据（用了哪些**既有**信号，为什么够用）

新判据的唯一一份实现：`Touch_Client/force/ForceCompensation.cpp` 的 `guardReferenceAvailable`

```cpp
static inline bool guardReferenceAvailable(const AppState::ForceData& fd) {
    return !fd.isStale && (fd.sixForceOnline == 1);
}
```

| 信号 | 出处 | 为什么它够用 |
|---|---|---|
| `fd.sixForceOnline` | `AppState::ForceData`（30004 帧 `@1037`，`RelayCore` 每帧写） | **只有 `== 1` 才算"这一路在线"**。实机实测值就是 1（四份夹具文件头记 `sixForceOnline=1`，2026-09-19/20）；`ForceData` 初值 `-1` = **一帧都还没收到**（我核过：`RelayCore` 写的是 `static_cast<int>(static_cast<unsigned char>(buf[1037]))` ⇒ 值域 0..255，`-1` 只可能是"还没收到帧"）。取"正向确认"而不是"没说不在线就算在线"：无法确认时拒才是 fail-closed；而实机值就是 1，不会把正常工况判成不可用 |
| `fd.isStale` | 既有超时常量 `Config::FORCE_STALE_MS` 的落点（`RelayCore::pollForce` 用 `lastUpdateMs` 与它算出该标志，**同一把锁内、就在 `step()` 之前**） | 陈旧帧里的参考量是**上一次读数**，不是这一路的当前状态 ⇒ 同样不可用。**读这个标志而不是自己再算帧龄**：帧龄算法只有一份，库里再算一份会出现"闸门说新鲜、F| 组帧说陈旧"两个答案 |

⇒ 两个都是**布尔 / 枚举级的事实**，**没有新造任何数值门限**。
**这也是不许凭感觉取"零附近多大算零"的原因**：那种门限会把**"真的没有外力"**判成不可用 ——
而"真的没有外力"恰恰是**应当放行**的那一半。

### 位置与行为
- 判定放在 `step()` 的 **EMA 更新之前**（第 7b 步）：参考量不可用时那个差是拿"假设的 0"
  算出来的，喂进 EMA 就是**凭空造一个零进判据的状态里**（本项目最忌凭空造数）。跳过更新还让
  这层门一恢复就能接着用上一次的**真实**证据判（若一帧都没比过，播种标志仍是假）。
- 不可用 ⇒ `compensated[]` 保持第 2 步写的全零（fail-closed 的落点不变）。
- 新状态**不打逐通道对比表**（"没比过"不能说成"比过了且没问题"）；复报那一行改成
  "仍在拒绝：参考量不可用 + 六维力在线状态 = ? / 帧陈旧 = ?"。

---

## 2. TDD 证据

### RED

1. 先写用例 `test_guard_reference_unavailable_does_not_pass`（`test_force_compensation.cpp`），
   并注册进 `main()` 的显式调用列表。
2. 为取**改动前**的行为，把守卫临时改成 `return true;`（一行、带 `★ TEMP-RED` 标记，
   取自后立刻恢复）—— 其余代码不动，因此这确实是"改动前的闸门行为"。
3. 命令：`cmd.exe //c "D:\Projects\Touch\Touch_Client\tests\build_force_comp_test.bat"`
   （`BUILD_EXIT=0`），然后**单独执行** `test_force_compensation.exe`。

失败输出（原样抄）：

```
  guard_reference_unavailable_does_not_pass...
    FAIL (sixForceOnline=0 (机械臂自报不在线)): 参考量不可用【竟然放行了】—— 判据退化成了
    "本地输出是否在自己的容差内", 那个比较不携带任何信息。
```

`Results: 35 passed, 1 failed` —— 失败理由与预期一致（参考量为零且这一路没有数据时**放行**）。

### GREEN

- 装上守卫，重建 + 重跑：`Results: 36 passed, 0 failed`（基线 35/0 + 新增 1 条）。
- 之后又扩了两条**既有**用例（下面 §4），最终仍是 `36 passed, 0 failed`。

### 一条用例同时钉住两件事（不是两条）
`test_guard_reference_unavailable_does_not_pass` 里：
- **(甲)** 三种"没有数据"的驱动方式各验一遍（`sixForceOnline=0` / `= -1` / `isStale=true`，
  即越过 `FORCE_STALE_MS` 窗口）：**不许 `OK`**、状态名必须是 `REFERENCE_UNAVAILABLE`、
  `compensated[]` 全零、`isCalibrated == false`、错误码是新的那个且**不是** `INCONSISTENT`。
- **(乙) 正面对照**：帧新鲜 + 在线 + 参考量真的读到 1.25 N ⇒ **必须 `OK` 且数据真的过去**
  （否则"一律拒绝"的桩也能让 (甲) 全绿）。
- **(丙) 反面对照**：帧新鲜 + 在线 + 参考量读到 2.0 N（对不上）⇒ **`INCONSISTENT`**，且
  名字与错误码都与 (甲) 分得开。

⇒ **"不可用"与"不一致"在同一个用例里被直接对照**（同一个零读数，有数据 = 不一致，
没数据 = 不可用）。

---

## 3. "不可用"怎么与"不一致"区分 / 红线断言的下场

- **独立状态**：`GuardState::REFERENCE_UNAVAILABLE`（第 4 个值），经 `guardStateName()` 报出
  名字 `"REFERENCE_UNAVAILABLE"`；`guardReport().state` 同步。
- **独立错误码**：新增 `ERR_FORCE_REFERENCE_UNAVAILABLE`，**不复用** `ERR_FORCE_INCONSISTENT`
  （那个码的字面意思是"两边都读到了数、但对不上"，而这里**没有比过**；报成"对不上"是让日志里
  出现一个没发生过的事实，操作员照它去查下发会白忙）。`getSeverity` 归 `REJECT`（与另两个一致）。
  `RobotDiagnostics::ERROR_CODE_SLOTS` 25 → 26，`RobotError.h` 顶部计数注释同步（码按下标索引，
  新码只能加在末尾）。
- **对话文字也分开**（`setGuardState`）：原因与处置各三句 ——
  "去标定" / "去查负载参数有没有发进去" / **"去查参考量这一路为什么没有数据"**，
  并明写**不要**去做前两件（它们都以"存在一个可比的参考读数"为前提）。

### 那条红线断言：**仍然是 1 条失败**
- **改动前**基线（我实测复现）：`69 passed, 1 failed`，失败是
  `FAIL: 12:38 pose 1 竟然放行了 (state=OK)`。
- **改动后**：`69 passed, 1 failed`，**失败条数不变**，失败文本改为

```
    FAIL: 12:38 pose 1 不是 INCONSISTENT (state=REFERENCE_UNAVAILABLE)
      ⇒ 【参考量不可用】: 这份夹具没有【参考量那一路】的列, 所以判据那一侧没有数据 (Task 7 的存在性守卫把它择出来了)。
      ⇒ 【原断言仍然没有被验证】: 它问的是"两边【比过】之后 会不会拒绝", 而这里两边【没有比过】。这条【照旧算失败】—— 把它记成通过, 就是把一件未验证的事变绿。
      ⇒ 要它重新有判别力只能靠【重采一份带参考量那一路的列的夹具】(它是本计划的收口必做项), 不是靠改这里。
```

- 断言本身**一个字没改**（`if (rep.state != INCONSISTENT) { ...; g_failed++; return; }`），
  所以它**没有**被"改判成通过"。给它的只有**描述**：说清"现在为什么不是 `INCONSISTENT`"，
  以及**它仍然没有被验证**。**那条 0 不代表原断言通过。** 报告里也不说"全绿"。

⚠ **与计划书的一处冲突，我按派单走**：计划 Step 5 写的是 `test_payload_calibration` 变为
**70/0**；本次派单的硬约束写的是**那条红线必须保持红、计数必须是 1**。两者对"不许被计成
原断言满足"是一致的，但计数不同。我按**派单的硬约束**做（保持 69/1）—— 理由：把失败改判成
通过会让套件退出码变 0（看起来"全绿"），而那与用户明确否决过的"把未验证的东西变绿"只差一步。

---

## 4. 四处跨任务交互，逐条处理

1. **`ZeroDriftCheck::decide` 拿到新枚举值不会静默漏支**：`if (in.guard != GuardState::OK)`
   这一支**本来就兜住所有非 OK 状态**，所以新值的归宿是**有定义的**（不是掉进某个分支的
   空缺），结论仍是 **`NotDone`（【未做】）** —— 正确：三种情形下 `fd.filtered` 都是闸门
   置的 0，都是"给不出漂移数"。**但"该说的话"必须改**：它原来写"那一段分得开'没有可用模型'与
   '有模型但对不上'"，并且 ⚠ 那句只讲"对不上"的一种来源 —— 参考量不可用时这两句都成了误导
   （拿"对不上"的处置去忙是白忙）。现在 `decide` **按状态**给"为什么没查"：
   【没有可用模型】/【参考量不可用】/【有模型但与参考量对不上】，三种处置各说各的（共用
   `NotDone` 这一个 outcome）。`test_zero_drift_not_done_after_full_wait` 的原因数组从 2 个
   扩到 3 个（新值也走这一支，逐条断言不变）。
2. **`main.cpp` 调用侧的文字**：`gs != OK` 这一支**覆盖三种原因**了，结论一样（都是"没查"），
   所以**逻辑不变**；我在那段注释里写明"本函数不按原因分别处置，但该说的话不一样，那一段由
   `decide` 按状态给出"，并明写**别在这里再补一句笼统的"闸门在拒绝"** —— 那正是派单点出的
   "把原因说成拒绝就是撒谎"那一处。调用侧没有别的独立输出文字，所以 `【未做】` 那段的
   **全部**真实来源就是 `ZeroDriftCheck.h`，已改。
3. **没有 `static_cast<int>(guardState())` 比字面量**：我逐处核过 —— `RelayCore` 现在用的是
   `guardSt != GuardState::OK` 与 `guardErrorCode(guardSt)`；唯一剩下的 `static_cast<int>(guardSt)`
   是**变化检测的缓存**（`lastGuardSt`，与缓存值比、不与字面量比），新值照样流过，不构成
   记过账的那种缺陷。`guardStateName()` 加了新 case；`guardErrorCode` 的表加了新映射；
   两处 `switch` 都**没有 default**，但我要如实说：**编译器不会替我们发现漏配**
   （`.h` 里那段记账仍然成立），真正钉住它的是用例 —— 所以我也把
   `guard_error_code_mapping` 从三条映射扩到四条，并加了"三个码互不相同 + 与枚举数值索引无关
   + 名字对得上 + 严重度都是 REJECT"的断言。另外 `setGuardState` 里那个新的 `switch` 我**特意
   留了 default 并返回"未知状态"**（不是套用别人的话）—— 兜底也不许撒谎。
4. **票掩码一个字没动**：`g_guardVote` 仍是 `{true, true, false, false, false, false}`（用
   `git diff` 核过那一行没有出现在 diff 里）。`Fz` 不投票的依据被推翻那件事与**夹具重采**绑定，
   不属于本任务。

---

## 5. 两套计数 + 构建

| 套件 | 基线 | 现在 | 说明 |
|---|---|---|---|
| `test_force_compensation` | 35 passed / 0 failed | **36 passed / 0 failed**（exe 退出码 0） | +1 条新用例（Task 7）；另**扩充**了两条既有用例（漂移检查第三种原因、错误码第四行） |
| `test_payload_calibration` | 69 passed / 1 failed | **69 passed / 1 failed**（exe 退出码 1） | 那条红线**仍是唯一的那 1 条**，理由已改判为"参考量不可用" |

- 构建命令（原样）：`cmd.exe //c "D:\Projects\Touch\Touch_Client\build.bat"` ⇒ **`Build OK.`**
  （另有一条 `CALLBACK` 宏重定义的 C4005 警告，是既有的、与本次改动无关）。
- 两个套件都是**先构建、再单独执行 exe**（`run_tests.bat` 不跑 `test_payload_calibration`，
  我也没有改那两个 `.bat`）。
- `Touch_Client.exe` 未在运行（`tasklist` 核过），构建没有报"文件被占用"。
- **没有**因为跑测试而新增待提交文件：跑完 `git status` 与跑之前逐条一致（仓库根那个
  `calib/` 在本次会话开始前就是未跟踪状态，我没有动它，也没有提交任何标定文件）。
- **没有删除或放宽任何断言**：`git diff` 里两个测试文件**没有一行带 `CHECK`/`assert` 被删除**。

---

## 6. 我**没能复现**或与派单**不一致**的claim

1. **`sixForceOnline` 的 `-1` 不是"机械臂报的值是未知"，而是"我们一帧都还没收到"**。
   `RelayCore` 取的是 `buf[1037]` 的**无符号**字节 ⇒ 值域 0..255；`-1` 只出现在
   `AppState::ForceData` 的**类内初值**上。派单里"`-1` means unknown"这句话的**结论**
   （不可确认 ⇒ 不放行）不变，但**语义**要说准：它是"没有任何帧"，不是"臂说它不知道"。
2. **计划 Step 5 期望的 `70/0` 与派单硬约束的"必须保持 1 failed"冲突**（见 §3 末）。
   我按派单做，并在提交信息与本文里写明理由。
3. **一处派单没提但确实存在的涟漪（我修了）**：`test_payload_calibration` 里
   `test_runtime_compensation_*` 那类"喂一个与本地一致的参考量、再量模型输出"的用例，
   在旧代码上靠"闸门放行"才量得到；加了守卫之后它们会量到**闸门置的 0**，并以
   **口径自校不过**的样子红掉（我第一次重跑就撞上了：`68 passed, 2 failed`）。修法是给那三处
   夹具各加两行"帧新鲜 + 在线"（不是放宽断言），之后回到 69/1。
   这正是"两个套件都要**真的跑**、不能只看构建"的价值。

---

## 7. 改动文件

- `Touch_Client/force/ForceCompensation.h` —— 新状态 + 三种原因/三种处置的说明 + 错误码表注释
- `Touch_Client/force/ForceCompensation.cpp` —— `guardReferenceAvailable`、`step()` 第 7b 步、
  `guardStateName` / `guardErrorCode` 新项、`setGuardState` 的三路原因/处置 + 复报 + 不打表
- `Touch_Client/safety/RobotError.h` —— `ERR_FORCE_REFERENCE_UNAVAILABLE` + 严重度 + 名字 + 计数注释
- `Touch_Client/safety/RobotDiagnostics.h` —— `ERROR_CODE_SLOTS` 25 → 26
- `Touch_Client/relay/RelayCore.cpp` —— 两个码 → 三个码的注释（映射仍只有一处实现）
- `Touch_Client/force/ZeroDriftCheck.h` —— 【未做】那段按状态给出"为什么没查"（三种）
- `Touch_Client/main.cpp` —— 漂移检查调用侧注释：它现在覆盖三种原因、不要写笼统的"在拒绝"
- `Touch_Client/tests/test_force_compensation.cpp` —— 新用例 + `gateVisibleFrame()` 夹具事实 +
  两条既有用例扩充（漂移检查第三支、错误码第四条）
- `Touch_Client/tests/test_payload_calibration.cpp` —— 红线用例改判描述（仍计失败）、
  "没有比过就不打表"、三处喂参考量的夹具补"帧新鲜 + 在线"
- `Docs/superpowers/specs/2026-09-21-gate-reference-prereq.md` —— §9 记录决策/依据/边界；
  §8.3 那张表"两种原因"改成"三种"

---

## 8. 自审与遗留（如实）

- **一处分支没有用例覆盖**：`setGuardState` 里 `REFERENCE_UNAVAILABLE` 的**复报**那一行
  （需要"状态不变且距上次打印超过 `FORCE_GUARD_REPORT_MS` = 5 s"）。单测里连喂 8 帧都在 5 s 内，
  走不到；要覆盖得让用例睡 5 秒（不值得）。**状态跃迁**那条路径是用例覆盖到的
  （`test_payload_calibration` 的失败输出里就能看到那 9 行块）。
- **`guardReport().exceeded[]` 在"不可用"状态下仍按 `g_guardEma` 现算**（我没有把它清零）。
  理由：不可用时我**不更新** EMA，所以那个值是**上一次真实比较**的读数（不是造出来的数），
  而 `state` 字段就在旁边说明"这次没有比过"；生产侧目前**没有**消费者读 `exceeded`
  （只有用例读，且只在 `INCONSISTENT` 场景读）。**这一条是一个可以争论的取舍**，我按"少改"处理，
  写在这里供复审推翻。
- **实机未验**：`sixForceOnline == 1` 这个取值的实机依据是**夹具文件头的历史记录**，不是本次
  现场复测的。`--no-robot` 下不会走到这里（`pollForce` 只在机械臂模式调用）；**断链窗口**
  （还没有第一帧时）闸门会判"不可用"—— 方向是 fail-closed，但这是**行为变化**，现场第一次
  连臂时会看到新的拒绝原因文字。
- **本任务没有解决**的东西一字未动：`@720` 的符号约定未验证、夹具重采（收口必做项）、
  `A` 第三行/z 方向无判据、下发不持久、力矩自带偏置、Task 10 闭环、`Fz` 不投票依据被推翻
  之后的处置。**别把这份报告读成"闸门在新参考量上被验证过了"** —— 那条红线断言的存在
  就是"还没有"的机器可见记录。

---

# 附:复审修复 (2026-09-21) —— 提交 `a1bebda`

**补的是可诊断性与若干清理。守卫的判据、位置/顺序、投票掩码、全部容差一个字没动；
`test_payload_calibration` 那条红线断言（含它的文字）一字未改。**

## A. 两套计数与构建（改动前 / 改动后，都是在真实 exe 上跑的）

| 套件 | 改动前 | 改动后 | exe 退出码 (后) |
|---|---|---|---|
| `test_force_compensation` | 36 passed / 0 failed | **37 passed / 0 failed** | 0 |
| `test_payload_calibration` | 69 passed / 1 failed | **69 passed / 1 failed** | 1 |

- 「改动前」不是照抄本报告 §5：我先把**改动前**留在盘上的那两个 exe 各跑了一遍，实测复现
  36/0 与 69/1，再改代码、重编、重跑。**红线仍然恰好 1 条**，失败文字一字未变
  （`FAIL: 12:38 pose 1 不是 INCONSISTENT (state=REFERENCE_UNAVAILABLE)`），
  `git diff` 里 `test_payload_calibration.cpp` **没有任何改动**（本次一次都没碰它）。
- `test_force_compensation` 由 36 变 37 是**加了一条用例**（下面 D），不是把红改绿。
- 构建：`cmd.exe //c "D:\Projects\Touch\Touch_Client\build.bat"` ⇒ **`Build OK.`**
  （只剩既有的 `CALLBACK` 宏重定义 C4005 警告）。两套测试各自**先构建、再单独执行 exe**，
  没有用 `run_tests.bat`。
- `Touch_Client.exe` 未在运行（`tasklist` 核过，构建没有报文件被占用）。
- 跑完 `git status` 与跑之前逐条一致：**没有**新增待提交文件（测试的 stderr 捕获窗口用的是
  临时文件，用例自己 `remove` 掉；也没有留下任何标定文件）。
- 两套计数**不说"全绿"**：`test_payload_calibration` 仍然是红的（1 条，故意的）。

## B. 六条逐条

**Fix 1（Important，可诊断性）—— 已做。** 状态跃迁那段现在**当场打出本帧的两个可用性读数**
（`@1037` 在线状态、帧陈旧），措辞与 5 s 复报行一致（`@1037 = %d (只有 1 算在线)`, `帧陈旧 = %d`），
仍写在"要查哪两件事"那条处置里，没引入新词。缓冲区是现拼的（那段文字约 570 字节，留了三成余量；
`static` 缓冲区不足会静默截断，这点写在代码注释里）。

**Fix 2（Minor）—— 已做。** `ZeroDriftCheck.h` 的 `default` 改成显式 `case INCONSISTENT`
（文字与从前逐字相同），另加一个**照实说不明**的兜底：说原因不明、不给处置指引、并指向
`ForceCompensation` 那段现报的原因。**既有三种状态的输出一字未变。**

**Fix 3（Minor）—— 已做。** `ForceCompensation.cpp` 里"输出一律走 stderr"那段重复注释删掉一份。

**Fix 4（Minor）—— 已做。** `ForceCompensation.h` 里 `★ 都是空的` 改写为"
处置与上面两个【不同】(那两个动作在这里都没有依据)"——即它本来要说的意思。

**Fix 5（Minor）—— 选【写进声明处】，不清零。** 理由：清零会**丢信息**（恢复后接着用的就是
那一组 EMA），而 `state` 字段就在旁边，能把它说明白就够；这也与本文件"少改、且改动要留出处"
的一贯做法一致。注释写在 `exceeded[]` 的声明处并点名 `ema[]`，说明这两个数组里的值来自
**上一次真实比较**、不是为这个状态算的，并写明**要判"这次比过没有"只看 `state`**。
⚠ 一处**我按事实收紧了口径**：`tol[]` **不在**此列 —— 它每次现填，与"比较做没做过"无关，
所以没有把三个数组一起说成"留的是旧值"。

**Fix 6（Minor）—— 选【去掉】，不是"标记为不可达"。** 理由：留着它就是留一段**读起来像活路、
实际走不到**的文字，而这一类问题正是这一串提交一直在防的（枚举加值时编译器不会替我们发现漏配，
同一类）。去掉之后**判据没有变弱**：我把那条不变量（能进入该状态的 `setGuardState` 调用
全程序只有一处，就在写下这两个读数的几行下面；而复位它们的 `resetGuard()` 同时把状态置回
UNCALIBRATED ⇒ "本状态成立"与"没有本帧"不会同时发生）**写在了那两个读数的声明处和两个打印点**，
所以打印端拿到的永远是本帧的值。那个"是不是本帧"的标志随之删除（它已无读者）。

## C. 与派单不一致 / 没能复现的 claim

**没有**发现事实性错误 —— 派单描述的五处（值只在复报行、default 借词、重复注释、
"都是空的"、`exceeded[]` 留旧值）与"那一支到不了"这条推断，我都在代码里逐条对上了；
两套基线计数（36/0、69/1）也实测复现。只记两处**口头与代码的细微落差**（都不影响结论）：

1. "the first refusal print … tells the operator which two things to check" —— 那段**确实**
   列了两件要查的事（`30004 帧`、`六维力在线状态`），但**没有**说这两件事各自**为什么**
   可能导致不可用；我把值打出来时只加了值，没有改这两条的措辞（不新增判断）。
2. Fix 3 说的"an added copy plus the original" —— 两份是**逐字相同**的两行注释（各两行），
   只删了多出来的那一份，原文案一个字没改。

## D. Fix 1 有没有用例覆盖 —— 有，新增一条

`test_force_compensation` 新增 `guard_unavailable_first_refusal_prints_the_two_values`：
用 **stderr 捕获窗口**（`_dup`/`_dup2` 到临时文件，做法与 `test_payload_calibration` 的静音窗口
同源）把**状态跃迁那一次**的打印抓回来，三种 (online, stale) 组合
（`0/false`、`-1/false`、`1/true`）**各断言自己的两个值被印出来**，并断言抓到的那段
**不是复报行**。整段文字不做逐字比对（那样测的是措辞不是诊断能力），只按子串找那两个数。

- **红线证据（该用例不是摆设）**：临时把打印的两个值换成常量后重编重跑，它**按预期红掉**
  （`36 passed, 1 failed`，报"没有印出六维力在线状态 (找不到 \"@1037 = 0\")"），
  探针取自后立刻恢复，恢复后 `37 passed / 0 failed`。
- 复报那一行**仍然没有**用例覆盖（要睡 5 s 才走得到），这一点与 §8 的遗留一致；
  但本用例**同时断言了"抓到的不是复报行"**，所以"跃迁那一次就有值"这件事不再是靠人读代码。

## E. 一条不需要改动的行为变化（记录日志指纹）

新状态意味着**短暂的"还没有帧"窗口会每几秒报一次**（复报）+ 一段诊断帧；而从前的行为是
"断链期间可以拿陈旧数据安静地过去"。方向是 fail-closed，评审判定正确。**本次没有改任何日志
行为**，只把跃迁那一次的诊断值补全。⇒ 现场第一次连臂时会看到新的拒绝原因文字与两个读数。
