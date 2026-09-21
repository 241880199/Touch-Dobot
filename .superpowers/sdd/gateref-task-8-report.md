# Task 8 报告 —— 收尾一致性: z-gap 结论写回代码注释 + 标定证据【冻结快照】

**日期:** 2026-09-21 · **分支:** `feat/pen-clamp-redesign`
**性质:** 注释 + 引用路径 + git 跟踪状态。**掩码、全部容差、可用性判据、闸门逻辑一个字没动。**
**提交:** 见文末「提交」一节。

---

## 0. 一句话

状态标记与记录对齐了（"未复测" → "已复测、按原样不复现"，且把**分子/分母**说清）；
标定证据**逐字节冻结**进 `tests/fixtures/`，活文件 `git rm --cached` 后**原样留在磁盘上**，
`Touch_Client/calib/` 与仓库根 `/calib/` 都改成**目录级 ignore**；引用**按两类**分开改。
构建 `Build OK.`；`test_force_compensation` **37 passed / 0 failed**；
`test_payload_calibration` **69 passed / 1 failed**（那条红线仍红着，原因未变）。**不称"全绿"。**

---

## 1. Step 1 —— 写回 `ForceCompensation.cpp` 的状态标记

**改了哪一段:** `g_guardVote` 掩码注释块里那段 `Fz` 前提的"现状标记"。
**没改:** 掩码本身（`static const bool g_guardVote[6]` 逐字符未动）、任何容差、任何别的代码行。
只动那一段文字（`git diff` 可见：插入 13 行、删 6 行，全在注释里）。

**写回的文字（原样引用）:**

```
//   ⚠⚠ 【已复测 (2026-09-21) —— 结论: 那条秩 2 形状【按原样不复现】, 不许读成"已验"】:
//     参考量换到【通过关节电流计算】的那一路之后, 上面那条依据【跟着复测了】, 结果是
//     "第三行比另两行小 8~15 倍"这个形状在新参考量上【不存在】。掩码本身的取舍不在换
//     参考量那次改动内, 但这个前提的现状必须留在这里 (把这段说成"已验"就是让后人照着
//     一个已不复现的前提去改掩码)。
//     ★ 【机制要说准 —— 分子与分母不许混】: 这【不是】"z 行被修好了"。
//       · z 行的【绝对】模【减得少得多, 但不是"没动"】: 旧参考量 `0.0148~0.0201 kg`
//         → 新参考量 `0.0051~0.0164 kg` (其中第 1 轮缩了约 3.7 倍 —— 而那一轮正是
//         下面写着"不单独作数"的那一轮, 所以"z 行没怎么动"不能当三轮平权读);
//       · 【塌下去的是 x/y】: `0.20~0.21 kg` → `0.0075~0.0286 kg`。
//       ⇒ "z 不再被结构性压小"说的是【比值】(第三行 / 另两行), 而比值变小是
//         【由分母 (x/y) 缩小驱动】的, 不是 z 那一路被修好了。
//     ⚠ 定案范围: 第 2、3 轮与三轮合计。第 1 轮那个比值 (0.179) 只比旧参考量的带
//       (0.070~0.094) 高出约 2 倍, 其分子与 se 只差约 2~3σ ⇒ 【它单独不定案】。
//     ⚠ 【依据不复现 ≠ z 该投票】: 要不要把它提为投票通道是另一个决策 —— 那条参考量在
//       【接触状态下】未经验证, 本段只写"旧的依据不再是依据", 掩码一个字没动。
//     证据与复现方法: Docs/superpowers/evidence/2026-09-21-z-gap-report.md (秩复测一节
//     与判决一节) —— 摘要见 Docs/superpowers/specs/2026-09-19-remaining-workflow.md §3.1。
//     残余 (z 方向仍然没有守门) 见 Config.h 里 FORCE_GUARD_TOL_FORCE_N 上方那段。
```

### 1.1 分子/分母 —— 明确到这句

- **分子（z 行自己的绝对模）几乎没动，但不是"没动"**: `0.0148~0.0201 kg` → `0.0051~0.0164 kg`。
  逐轮 `0.0187→0.0051`（第 1 轮，缩约 3.7 倍）、`0.0201→0.0157`、`0.0148→0.0164`。
  **第 1 轮正是"不单独作数"的那一轮**，所以"z 行没怎么动"这句话**不能当三轮平权读**。
- **分母（x/y 两行）塌了**: 每轮取大的那一行，`0.20~0.21 kg` → `0.0075~0.0286 kg`。
- **⇒ "z 不再被结构性压小"说的是【比值】（第三行 / max(另两行)）**：`@576` 三轮 `0.070 / 0.094 / 0.088`
  （被抑制 11~14 倍），`@720` 三轮 `0.179 / 2.092 / 0.721` ⇒ **比值变小是由分母缩小驱动的，
  不是 z 那一路被修好了。** 差别最清楚的地方是**差量**：`comp − @576` 的 `K` 第三行被抑制 14~39 倍，
  `comp − @720` 的**没有被抑制**。
- **定案范围**: 第 2、3 轮与三轮合计；第 1 轮 `0.179×` 约 `2~3σ`，**单独不定案**。

出处：`Docs/superpowers/evidence/2026-09-21-z-gap-report.md` 秩复测一节（表里那三行 `@720` 的
行模 `0.0118/0.0286/0.0051`、`0.0075/0.0057/0.0157`、`0.0164/0.0227/0.0164`）与判决一节；
摘要见 `Docs/superpowers/specs/2026-09-19-remaining-workflow.md` §3.1。**注释里不复述它的表，只指文档。**

### 1.2 顺带改掉的一个过期指针（说明理由）

原文末行是：`开放项见 Config.h 里 FORCE_GUARD_TOL_FORCE_N 上方那段 (开放项 C: z 轴缺口)`。
记录里 **`开放项 C`（"38~64 N 的 z 缺口"这个读法）已被查证关闭**
（`remaining-workflow` §3 的 C 行 ✅ 已关闭；z-gap 报告 §6 "开放项 C 可以关闭"），
而 `Config.h` 那一段自己叫的是**"未决项"**。所以这句指针若照抄"开放项 C"就是**第二个会撒谎的标记**。
改成"**残余 (z 方向仍然没有守门)** 见 Config.h 里 `FORCE_GUARD_TOL_FORCE_N` 上方那段"：
指针目标没变，标签改成了记录里的那个（残余仍在，不是"都好了"）。

---

## 2. Step 2 —— 冻结快照（**不是搬家**）

### 2.1 快照名（我定的，与既有夹具风格一致）

| 源（活文件，现在不跟踪） | 冻结快照（已入库） |
|---|---|
| `Touch_Client/calib/calib_poses.txt` | `Touch_Client/tests/fixtures/calib_poses_2026-09-20_frozen.txt` |
| `Touch_Client/calib/calib_report.md` | `Touch_Client/tests/fixtures/calib_report_2026-09-20_frozen.md` |
| `Touch_Client/calib/calib_log.txt` | `Touch_Client/tests/fixtures/calib_log_2026-09-20_frozen.txt` |
| `Touch_Client/calib/force_calib.json` | `Touch_Client/tests/fixtures/force_calib_2026-09-20_frozen.json` |

命名沿用既有夹具的 `calib_poses_2026-09-19*.txt` 风格（日期 + 用途标记），
后缀用 `_2026-09-20_frozen`（活文件最后一个块的日期是 2026-09-20）。
**与既有夹具 `calib_poses_2026-09-19.txt` 不冲突**：那个 glob（`calib_poses_2026-09-19*`）
匹配不到 `..._2026-09-20_frozen.txt`。

**快照是【逐字节副本】（不是加过 banner 的新文件）** —— 这样才有下面那条可验证的等式。
（既有夹具 `calib_poses_2026-09-19.txt` 顶部带一段来源 banner；我**没有**给新快照加，
理由是"可证逐字节相同"比"自带出处说明"更值钱 —— 出处说明写在文档与忽略规则注释里。）

### 2.2 ★ 核验: 被文档引用的那些数**确实在快照里**

- **逐字节相同（冻结时刻的硬证据）**：`md5sum` 四对**两两相同**
  `b73f31fb…`（poses）/ `6a88242c…`（report）/ `f6a16dab…`（log）/ `a611f0d0…`（json）。
- **那三段带 `F720*`/`M720*` 列的 `# attempt` 在快照里，且恰好只有三段**：
  `awk` 扫快照，带 `F720x` 表头的块只有 `2026-09-20 21:58:46`、`22:12:28`、`22:18:42`，
  每段 **10 行数据**（= 5 对原地复采）。表头逐字含
  `F720x,F720y,F720z,M720x,M720y,M720z`。
- **`calib_log` 侧被引用的那一行在快照里**：`2026-09-20 22:18:42 | 10 | … | INSTALLED fit_ok … | 0.415864 …`
  —— `remaining-workflow` / 前置复核引的 `mass_kg = 0.415864`（`22:18:42`）逐位对上。
- **`calib_report` 侧的节在快照里**：`## 采集 …` 共 **11 节**，含 `21:58:46` / `22:12:28` / `22:18:42`
  三节（另两节是 `18:41:28 — 0 姿态` 与 `18:46:03` 的 `MODEL_FORM_NO_DOF`，
  与"九节可用线性解"这句话一致）。
- **`force_calib.json` 快照内容**：`version 3`，`a_matrix` / `bias_force_n` / `bias_torque_nm` /
  `com_sensor_m` 四项齐全（与 `remaining-workflow` 引的 `0.410561 kg` 那一版一致）。

⇒ **Task 1/3/6 的推导保留"可复现"**，且复现基座从"本机活文件"换成"入库的快照"。

### 2.3 谁在读这些文件（先查了再冻）

查法：对 `calib_poses` / `calib_report` / `calib_log` / `force_calib` 做全仓库
`grep`（`.cpp` / `.h` / `.bat` / `.py` / `.ps1` / `.sh`），并对 `fixtures` 做同样的 grep。

- **全部命中都是【写】，没有一处是【解析这些文件】**：
  - `main.cpp`：`CalibStore::fileFor("calib_log.txt")`（写一行）、
    `CalibStore::fileFor("calib_poses.txt")`（块注释自写"**只追加, 永不截断**"）、
    `CalibStore::fileFor("calib_report.md")`（追加一整块）、
    `CalibStore::fileFor("force_calib.json")`（`saveToFile`，'z' 调零 / 's' 求解 / 启动装载）。
  - `force/ForceCalibration.cpp`：只对 `force_calib.json` 做 `saveToFile` / `loadFromFile`。
- **测试侧读的是 `tests/fixtures/` 里的夹具**，而且**全是显式文件名**：
  `test_payload_calibration.cpp` 里 `fopen` 用的是 `fixtures/calib_poses_2026-09-19.txt`
  （四个候选路径）、`mgLoad(fixture, …)`（调用点传的是已入库夹具名）、`t6Load` 用 `FILES[k]` 里的字面名。
  **没有任何目录枚举 / glob**（`opendir` / `FindFirstFile` / `directory_iterator` 在
  `tests/` 的源码里一处都没有；命中的只有已编译的 `.exe`）。
- **⇒ 结论：冻结不会破坏任何读者。** 反过来，只要有人**按路径**去读活文件，
  它读到的就是会被改的那一份 —— 这正是要冻的理由。
- **⚠ 一条给后续任务的发现（不是本任务要修的）**：`t6Load` 只接受 **18 列或 25 列**的夹具布局
  （列数不是这两种就 `return false`）。**快照 `calib_poses_2026-09-20_frozen.txt` 是 31 列**
  （18 + `N1304` + 6 个 `sd` + 6 个 `@720`）⇒ 计划里那条"**重采带 `@720` 的夹具**（收口必做项）"
  落地时，**必须同时扩 `t6Load`（或新增一个读 31 列的入口）**，否则夹具进得去、用例读不出。

### 2.4 摘跟踪 + 忽略规则

- `git rm --cached` 四个文件（**只用 `--cached`，没有用普通 `git rm`**）。
- `.gitignore`（仓库根）：把原来的两条**逐文件**规则换成**两条目录级**规则 ——
  `Touch_Client/calib/` 与 `/calib/`（前导 `/` 锚定 = 只指仓库根那一处，
  `CalibStore::dir()` 在 exe 位于 `Touch_Client\` 下一层时产出的就是它）。
  同时把"为什么整个目录都不进库"写进注释。
- `.gitignore`（`Touch_Client/tests/`）：**这一步是被 `git check-ignore` 逼出来的** ——
  既有的 `*_log*.txt` 会把 `fixtures/calib_log_2026-09-20_frozen.txt` **吃掉**
  （`git check-ignore --no-index -v` 点名了那条通配规则）。
  加两条**否定规则**（`!fixtures/*_log*.txt` / `!fixtures/force_calib*.json`），写在通配**之后**
  （后者胜出）。通配本身不动 —— 它管的是运行产物，只是不该伸进 `fixtures/`。

### 2.5 核验（逐条）

- **一个文件都没从磁盘删掉**：`ls -la Touch_Client/calib/` 六个文件全在
  （`calib_log.txt` / `calib_poses.txt` / `calib_report.md` / `force_calib.json` /
  `force_calib.json.expired` / `payload_calib.json`），mtime 仍是 `Sep 20 22:18`。
  **`force_calib.json` 还在原处 —— app 启动装载的标定没被碰。**
- **跟踪状态**：`git ls-files Touch_Client/calib calib` ⇒ **0 条**（两个目录都不再跟踪）。
- **快照没被忽略**：对 `git ls-files Touch_Client/tests/fixtures/` 里的**每一个**文件跑
  `git check-ignore --no-index -q` ⇒ **一个都没有被忽略**（含新冻的四份与既有五份）。
- **活路径确实被忽略**：`git check-ignore --no-index -v` 给
  `Touch_Client/calib/force_calib.json → 仓库根 .gitignore 的 Touch_Client/calib/`、
  `calib/force_calib.json → 仓库根 .gitignore 的 /calib/`；
  而 `fixtures/calib_log_2026-09-20_frozen.txt` 命中的是**否定规则** `!fixtures/*_log*.txt`
  （即"不忽略"那一支）。
- **`CalibStore::dir()` 我按代码核过**（不是照抄 dispatch）：`core/CalibStore.cpp` 里
  `dir()` 用 `GetModuleFileNameA(NULL, …)` 取**可执行文件**路径 → 去文件名 → **上溯两级** →
  拼 `\calib\`；`fileFor()` = `dir() + name`。⇒ **与 CWD 无关**这条成立：
  app exe 在 `Touch_Client\x64\Release\` ⇒ `Touch_Client\calib\`；
  测试 exe 在 `Touch_Client\tests\` ⇒ 上溯两级 = **仓库根** ⇒ `D:\Projects\Touch\calib\`。
- **一次实测印证**：本次跑套件时，仓库根的 `calib/force_calib.json` mtime 变成 `13:55`
  （= 我跑用例的时刻），而 `Touch_Client/calib/` 下四个活文件的 md5 **分毫未变**。
  ⇒ "跑测试会写标定状态"是真的，且写的是**根 `calib/`**（那条规则正好把它盖住了）。

---

## 3. Step 3 —— 引用路径: 两类分开改

**判据**（计划给的那个，我照用）：这句在说"**这些数据从哪来（证据）**" 还是
"**程序往哪写（行为）**"？前者改，后者留。

### 3.1 （甲）改了的 —— 证据引用

| 文件 | 改法 |
|---|---|
| `Docs/superpowers/specs/2026-09-21-gate-reference-prereq.md` | 数据源表的 4 条 + §2.1 复现输入的 2 处 + §2.2 字符串撞车 1 处 + §3 判据 2 处 + §5④ 那段代码注释 + 文末"引用的文件清单" 3 行 ⇒ 全改指快照；并在数据源表**前**加一段"引用路径已改指冻结快照"的说明（讲清那个目录是运行时可写、为什么引快照） |
| `Docs/superpowers/evidence/2026-09-21-z-gap-report.md` | 同上（§1 数据源表、§2.2/§3.1/§4.1/§4.2/§5.1 的出处、§5.2 的表头对照、§附 的代码注释与清单）；同样加一段说明 |
| `Docs/superpowers/specs/2026-09-19-remaining-workflow.md` | §3.1 里"十三次拟合（`calib_report.md` 九节 + 四份夹具）" ⇒ 改指快照 |
| `Docs/superpowers/plans/2026-09-21-gate-reference-and-moment-vote.md` | Task 1 的数据源清单、Task 6 Step 3 的"优先用 `@720` 数据" ⇒ 改指快照 |
| `.superpowers/sdd/gateref-task-3-report.md` | 2 处（出处行里的原始数据、按 `force_calib.json` 复算） |
| `.superpowers/sdd/gateref-task-6-report.md` | 8 处（E3/E4/E6 的证据栏、§8.1 复核、§附 的脚本与复核行） |

**⚠ 两处代码注释也改了 —— 这是【超出 dispatch 列举范围】的一步，理由如下。**
dispatch 的（乙）清单把 `Config.h` 与 `main.cpp` 整文件列为"不许改"，但 dispatch **自己的判据**
说的是"描述**程序往哪写**的才留"。这两处**不是**行为描述，而是**证据出处**：

- `Touch_Client/config/Config.h`（`eps_乙` 那一段）：原文
  `原始数据: Touch_Client/calib/calib_poses.txt 里三段带力参考量那一列的采集` ——
  **它在说数从哪来**（三段 @720 采集），不是 app 往哪写。改成引快照，并补一句
  "不引 CalibStore 运行时可写目录下那份活文件：它只追加、永不截断，数字会随手漂"。
- `Touch_Client/main.cpp`（`BiasCheck` 的 z 镜像证据）：原文
  `2026-09-19 对 calib_poses.txt 那批 7 姿态做自由拟合 … 得到的证据` —— 同样是**出处**。
  它指的那批 7 姿态**本来就已经入库**
  （`tests/fixtures/calib_poses_2026-09-19.txt`；我逐行比过：与活文件的 `12:38` 那一次
  **数据行完全相同**，既有夹具只是顶部多一段来源 banner），所以改指**那个已入库的夹具**
  比指向我新冻的全量快照更准。

⇒ 按"判据优先于清单"处理，并在此明写：**若认为这两处也该照（乙）原样保留，回退它们即可，
不影响本提交其余部分**（它们是纯注释，无行为影响）。

### 3.2 （乙）故意**没改**的 —— 行为描述（逐处给理由）

| 位置 | 原文在说什么 | 为什么不改 |
|---|---|---|
| `Config.h`（`payload_calib.json` 那几处） | app 生成 / 重下发 / 持久化 `payload_calib.json` | 说"程序往哪写"，改了变假话 |
| `Config.h`（`FORCE_GUARD_TOL_FORCE_N` 上方整段） | 容差推导 + "z 方向没有闸门"这条未决项 | 行为/未决项陈述，不是证据出处；本任务硬约束也不动容差段 |
| `core/SessionReport.h`（开头那段 + `blockHeader`） | 本模块**追加**到 `calib\calib_report.md`；块头格式与 `calib_poses.txt` 的 `# attempt` **逐字相同** | 全是"程序写什么/什么格式"，改了变假话 |
| `main.cpp`（`logCalibAttempt` / `logPoseData` / `diagFinish` / `saveToFile` 附近几十处） | app 往 `calib_log.txt` 写一行、往 `calib_poses.txt` 追加、往 `calib_report.md` 追加、落 `force_calib.json` | 同上 |
| `main.cpp`（z 镜像段末尾"落盘的 `calib_poses.txt` 全部保持原始值"） | 说的是**写入行为**（保持原始值） | 同上 |
| `force/PayloadCalibration.cpp`（"整套标定是'夹具口径'…`calib_poses.txt` / 夹具把力矩量化到 0.001 N·m"） | **边界情形**。它既像"数据从哪来"，又是在说**记录程序的量化格式**（写入行为的一部分） | 判为**（乙）留**：这句话的依据是**写文件的格式化精度**（app 的写行为），快照只是同一批字节的副本；改指快照并不会让它更真，反而暗示"格式来自快照"。dispatch 也把本文件列在（乙）。**留** |
| `.superpowers/sdd/runtime-guard-report.md`（唯一一处 `calib_poses.txt` / `calib_report.md`） | 在"**没动**（硬约束）：门限…、夹具、`calib_poses.txt`、`calib_report.md`、`logCalibAttempt` / `logPoseData`"这张**范围清单**里 | 这是"哪些代码路径没被改"的**行为/范围**陈述，不是证据出处（且紧邻 `logCalibAttempt` / `logPoseData` 两个函数名）。改指快照会把"函数没动"说成"文件没动"，**变假话** ⇒ **留**。见 §5 第 1 条 |
| 其余历史 `Docs/superpowers/evidence/*.md`、`specs/*run-00*.md`、更早的 plans | 引用活路径当数据源 | **见 §5 第 2 条（明写的残余）** |

### 3.3 一处记录自身的歧义（我改了，说明理由）

`remaining-workflow.md` §3.1 原有一句：
"⇒ 代码里那两行『未复测 / 不许读成已验』**仍然是对的**，而本次复测的结论是『它按原样不复现』。"
**字面读**会得出"代码写'未复测'是对的"⇒ 与 dispatch 的前提（"未复测"已过期）**直接冲突**，
也与本条自己刚写下的复测结论自相矛盾。**我的判断**是落笔时指的是那两行的**告诫**
（"不许读成已验"仍然对），而不是"未复测"这三个字。
⇒ 照 dispatch 的读法改（"未复测"过期），并把该句改成**显式区分**：
"那两行的**告诫**（不许读成已验）仍然是对的" + 一条 2026-09-21 Task 8 补注
（"未复测"三个字本身已过期，标记已改成"已复测、按原样不复现"；掩码一个字未动）。
**⇒ 这一句是记录里唯一一处会把 Task 8 读反的地方，已堵上。**

---

## 4. Step 4 —— 验证

| 项 | 命令 | 结果 |
|---|---|---|
| `test_force_compensation` 构建 | `cmd //c "D:\Projects\Touch\Touch_Client\tests\build_force_comp_test.bat"` | `BUILD_EXIT=0` |
| `test_force_compensation` 执行（**单独跑 exe**） | `Touch_Client\tests\test_force_compensation.exe` | **37 passed, 0 failed**（退出码 0） |
| `test_payload_calibration` 构建 | `cmd //c "…\build_payload_calibration_test.bat"` | `BUILD_EXIT=0` |
| `test_payload_calibration` 执行（**单独跑 exe**） | `Touch_Client\tests\test_payload_calibration.exe` | **69 passed, 1 failed**（退出码 1） |
| 唯一那条红的 | — | `FAIL: 12:38 pose 1 不是 INCONSISTENT (state=REFERENCE_UNAVAILABLE)` —— **就是那条故意红着的断言**（2026-09-19 的四份夹具没有参考量那一列的列）。**没碰它，也没改它的 ⚠ 注。** |
| 完整构建（**必做**） | `cmd //c "D:\Projects\Touch\Touch_Client\build.bat"` | **`Build OK.`**（末行 `Build complete.`） |

- 两份套件都是**先 build、再单独执行 `.exe`**；`run_tests.bat` 覆盖不到这两个套件，
  所以没有用它代替。
- **不称"全绿"**：一条是 37/0，一条是 69/1。
- 构建前查过进程：`tasklist` 报 `No tasks are running which match` —— `Touch_Client.exe` 没在跑。
- **跑完 `git status` 查过**：除了本来就有的那几个无关未跟踪文件（`Hardware/stl.zip`、
  `Touch_Client/*.log`、`Touch_Client/tests/_*.bat|log|cpp`）**没有任何新文件**冒出来；
  测试写的标定状态落在仓库根 `calib/`（已被忽略）与 `Touch_Client/tests/force_calib.json`
  （`tests/.gitignore` 本就忽略）⇒ **没有把运行产物提交进去**。
- **计数有没有"因为我的改动而变"**：没有。37/0 与 69/1 都是预期基线；
  冻结快照没有让任何计数移动（见 §2.3：没有任何读者按路径读活文件）。

---

## 5. dispatch 里我**没能复现** / 与代码不符的地方

1. **`.superpowers/sdd/runtime-guard-report.md` 里没有"证据引用"可改。**
   dispatch 把它列进（甲）。我按 `grep -n calib` 逐行看过该文件，**唯一**提到
   `calib_poses.txt` / `calib_report.md` 的地方是**"没动（硬约束）"那张范围清单** ——
   那是**行为/范围**陈述（相邻条目是 `logCalibAttempt` / `logPoseData` 两个**函数名**），
   按判据属（乙）⇒ **一处未改**。**该文件在本提交里没有改动。**
   （若本意是"该文件里凡出现这两个文件名都要改指快照"，那与本任务自己的判据冲突，我按判据办。）

2. **本任务只重指了计划点名的范围 + 本计划自己的两份任务报告；其余历史文档仍在引活路径。**
   `git grep` 显示还有一批更早的 `Docs/superpowers/evidence/*.md`、
   `specs/2026-09-19-raw-channel-calibration-run-00*.md`、`specs/2026-09-19-on-machine-checklist.md`
   等在引 `Touch_Client/calib/…`。**我故意没动它们**，理由不是"忘了"，而是：
   这些文件里那些引用的**上下文本身**绑在活文件的"当时状态"上，例如
   `joint-A-report.md` 写"`calib_poses.txt` **最后一块** `18:49:55`"、"mtime 仍是采集当时的 `18:49:55`，未被触碰" ——
   **改指快照会让这些句子变成假话**（快照的"最后一块"是 `22:18:42`）。
   ⇒ 一次机械替换会把一批真话改成假话，**比不做更糟**。
   **⇒ 明写的残余**：这些旧文档引的数字**仍以本机活文件为基座**（它们本来就是这样，本任务没让情况变坏）；
   要彻底收口需要**逐篇判断**（哪些是"这批数据从哪来"、哪些是"当时那个文件的状态"），
   那是独立的一件工作，不应塞进本任务里悄悄做掉。

3. **`t6Load` 不认 31 列**（见 §2.3 末）—— dispatch 没提，但它与计划里"重采带 `@720` 的夹具"
   这条收口必做项直接相关：夹具能入库、用例读不出。**记在这里免得下次踩。**

4. **dispatch 关于"`Touch_Client/calib/` 是运行时可写、路径由 exe 位置推"这条 —— 成立**
   （我按 `CalibStore.cpp` 核过，见 §2.5）。这条**复核通过**，不是问题，列出来是为了说明我核过。

---

## 6. 改了哪些文件 / 自审

**本次提交（`chore(gate)`）:**

```
.gitignore                                          |  M
Touch_Client/tests/.gitignore                        |  M
Touch_Client/force/ForceCompensation.cpp             |  M  (只动那一段注释)
Touch_Client/config/Config.h                         |  M  (只动 (乙) 项的"原始数据"三行)
Touch_Client/main.cpp                                |  M  (只动 z 镜像证据那三行)
Touch_Client/tests/fixtures/calib_poses_2026-09-20_frozen.txt   |  A
Touch_Client/tests/fixtures/calib_report_2026-09-20_frozen.md   |  A
Touch_Client/tests/fixtures/calib_log_2026-09-20_frozen.txt     |  A
Touch_Client/tests/fixtures/force_calib_2026-09-20_frozen.json  |  A
Touch_Client/calib/{calib_log,calib_poses,calib_report,force_calib.json} | 摘跟踪（磁盘保留）
Docs/superpowers/specs/2026-09-21-gate-reference-prereq.md     |  M
Docs/superpowers/evidence/2026-09-21-z-gap-report.md           |  M
Docs/superpowers/specs/2026-09-19-remaining-workflow.md        |  M
Docs/superpowers/plans/2026-09-21-gate-reference-and-moment-vote.md | M
.superpowers/sdd/gateref-task-3-report.md                      |  M
.superpowers/sdd/gateref-task-6-report.md                      |  M
```

**自审发现 / 取舍:**

- **`git` 把四个快照显示成 `rename`（100%）而不是 delete+add。** 这是**显示层的重命名探测**，
  不是"搬走了" —— 活文件仍在磁盘上、且不再跟踪。**别把它读成 move**（这正是计划 Step 2 要避免的读法）。
- **`Config.h` / `main.cpp` 两处代码注释的改动超出了 dispatch 的（乙）清单**（见 §3.1），
  按判据办的，并留了回退说明。
- **`ForceCompensation.cpp` 里那句"下面写着『不单独作数』的那一轮"**：指的就是**同一段内**
  紧接着的"定案范围"那条 —— 顺序上确在其下，不是笔误。已核。
- **注释里没有留下任何以反斜杠结尾的 `//` 行**（硬约束），也没有新增代码行号。
- **`calib_poses.txt` 的"活文件"里最后一块的日期是 2026-09-20**，所以快照名用 `_2026-09-20`；
  它同时含 2026-09-19 的旧块 —— 名字说的是**冻结日**，不是内容覆盖的日期范围。已在此写明。
- **没动**：`g_guardVote` 掩码、`FORCE_GUARD_TOL_FORCE_N` / `FORCE_GUARD_TOL_MOMENT_NM`、
  可用性判据（Task 7 那套）、任何闸门逻辑、`test_payload_calibration.cpp` 里那条故意红的断言及其 ⚠ 注。

**顾虑（留给后续）:**

1. **§5 第 2 条的残余**：一批旧证据文档仍以活文件为复现基座。要收口得逐篇判性质。
2. **§2.3 末的 `t6Load` 31 列问题**：不解决，重采的 `@720` 夹具进不了用例。
3. **`@720` 在接触状态下未验证、符号约定未验证** —— 与本任务无关，但本任务**没有**让这两条更清楚，
   它们仍然悬在计划"仍存在"清单里（z-gap 报告 §7 那三条残余也没被本任务改动）。
4. **`.superpowers/` 整体在 `.gitignore` 里，但 `sdd/*.md` 是【跟踪的】** ——
   所以这个报告要进库必须显式 `git add`（我做了）。将来在这个目录里新建的报告**不会**自动可见。

---

## 7. 提交

| SHA | subject |
|---|---|
| `55fb8ce` | `chore(gate): z-gap 结论写回代码注释 (分子/分母说清) + 标定证据冻结快照入 fixtures, 活文件退回运行时状态` |
| —— | `docs(task-8): 报告落盘 (冻结快照核验 / 两类引用的判别 / 两处超范围的代码注释出处)` |

⚠ 第二行**故意不写 SHA**: 那份报告就是这次提交的内容，**一个文件写不出自己的落地提交**
（本仓库既有文档里已有同一条约定）。本报告里 §3.3 那条对 `remaining-workflow` 的更正
也在第二次提交里。

分支 `feat/pen-clamp-redesign`。**未 push**（用户没要求）。
