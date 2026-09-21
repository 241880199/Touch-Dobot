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
只动那一段文字（`git diff --numstat` 于本任务提交范围内实测：`Touch_Client/force/ForceCompensation.cpp`
**19 行插入、6 行删除**，全在注释里。
**⚠ 勘误（2026-09-21 复审修复）**：本报告初版写的是"插入 13 行、删 6 行"，**数错了**；
按 `git diff --numstat 55fb8ce^ 55fb8ce` 逐文件核过，该文件是 `19 / 6`。下面引用别的计数时也照此复核过。）

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
  **⚠ 口径（2026-09-21 复审补注，免得别的机器上的人被误导）**：这四个 md5 是**本机工作区里
  那两个 CRLF 副本**之间的比对结果（本仓库 `core.autocrlf=true` 且**没有 `.gitattributes`**，
  所以**入库的 blob 是 LF 归一过的**）。换一台机器（或换一种换行策略，如 `core.autocrlf=false` /
  `input`）**签出来的同一份内容会是 LF、md5 就不一样**。
  ⇒ 这条等式是**同一台机器、冻结时刻**对工作区副本的核验，**不是可移植的内容指纹**；
  **内容本身不受影响**，别把它当成"在别处也能复算出同一个数"。
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
- **⚠ 一条给后续任务的发现（不是本任务要修的）——【勘误：原写"用例读不出"，方向说反了】**

  **原写法（错，留作对照）**: "`t6Load` 只接受 **18 列或 25 列**的夹具布局（列数不是这两种就
  `return false`）… 否则夹具进得去、用例读不出。"

  **实际机制（按代码核过）**: 读取器解析一行的函数是 `mgSplitRow(q, c, 25)`，第三个参数是
  **上限**：它逐列解析、**到 25 列就停**，返回 `k = 25`。所以**一个 31 列的行返回的就是 `25`**；
  而两道守卫判的正是"是不是 18 / 是不是 25"（`t6Load` 一侧是"不是 18 也不是 25 才拒"，
  `mgLoad` 一侧是"不是 25 才拒"）⇒ **守卫被满足，行被收下，多出来的最后六列（`@720`）
  被【静默丢弃】**。**这不是"拒绝"，是"接受并截断" —— 比原写法说的更糟。**
  两处读取器用的是同一个解析函数与同一个上限，所以**这个洞是共用的**。

  **为什么这条要紧**: **快照 `calib_poses_2026-09-20_frozen.txt` 是 31 列**
  （18 + `N1304` + 6 个 `sd` + 6 个 `@720`）。计划里那条"**重采带 `@720` 的夹具**（收口必做项）"
  落地时，31 列的新夹具**会照常被读进来、套件会照样过**，而**参考量那一路的列从来没有被消费**：
  用例里参考量那一侧本来就是"现算"的（见 `test_payload_calibration.cpp` 里那条注释：
  "夹具只有 `@576` / `@1304` 两列, 没有参考量那一路的列 —— 所以这里只能现算"），
  于是重采之后那些用例**仍旧在现算参考量**，而**收口项会看起来做完了、其实什么也没被验证**。
  **那道自称"不静默跳行"的守卫，恰好察觉不到这种布局变化** —— 这正是让一份绿的套件失去意义的
  失效模式。

  **⇒ 真正的要求（写给重采那一天，必须显式做到两条）**:
  1. 读取器要改成**响亮拒绝它不认识的列数**：解析时**把上限抬高按真实列数判**，
     **高于支持上限就拒**（而不是像现在这样到上限就截断、然后拿被截断的列数去对守卫）；
  2. **用例必须去消费【真的】参考量那一列**，而不是像现在这样现算 —— 否则"夹具重采了"
     与"参考量真的被读进来了"这两件事之间没有任何强制联系。

  **本任务【没有】动任何测试代码，也不该动**：读取器的改动属于**重采那一次**，
  它需要**自己的一次复审**（它是判决链路的一部分）。
  这一条此前无人记录；本报告初版记下了它但**把方向写反了**，故在此更正。

### 2.4 摘跟踪 + 忽略规则

- `git rm --cached` 四个文件（**只用 `--cached`，没有用普通 `git rm`**）。
- `.gitignore`（仓库根）：把原来的两条**逐文件**规则换成**两条目录级**规则 ——
  `Touch_Client/calib/` 与 `/calib/`（前导 `/` 锚定 = 只指仓库根那一处，
  `CalibStore::dir()` 在 exe 位于 `Touch_Client\` 下一层时产出的就是它）。
  同时把"为什么整个目录都不进库"写进注释。
- `.gitignore`（`Touch_Client/tests/`）：**这一步是被 `git check-ignore` 逼出来的** ——
  既有的 `*_log*.txt` 会把 `fixtures/calib_log_2026-09-20_frozen.txt` **吃掉**
  （`git check-ignore --no-index -v` 点名了那条通配规则）。
  加否定规则 `!fixtures/*_log*.txt`，写在通配**之后**（后者胜出）。通配本身不动 ——
  它管的是运行产物，只是不该伸进 `fixtures/`。
  **⚠ 2026-09-21 复审修正**：当时还加了第二条 `!fixtures/force_calib*.json` —— **那一条是多余的，
  而且把"不忽略"的面白扩了一块**：上面那条精确规则只匹配名为 `force_calib.json` 的文件，
  **匹配不到快照名 `force_calib_2026-09-20_frozen.json`**，所以**快照本来就没被吃掉**；
  而这条宽通配会**顺带**把 `fixtures/force_calib.json` 这个**假设的运行时产物**也放回入库面。
  ⇒ 已收窄为 `!fixtures/*_frozen.json`（只给冻结快照一个显式的"这是入库证据"标记）。
  `git check-ignore --no-index -q` 逐条核过：9 份夹具**全不被忽略**（含 4 份新快照），
  而 `fixtures/force_calib.json` **重新被忽略**；两个活标定目录照旧被忽略。

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

3. **读取器对 31 列的行【静默截断】（见 §2.3 末）** —— dispatch 没提，但它与计划里
   "重采带 `@720` 的夹具"这条收口必做项直接相关，而且**方向与本报告初版写的相反**：
   不是"夹具能入库、用例读不出"，而是**读得进、守卫还照样过，多出的 `@720` 六列被静默丢掉**。
   ⇒ 重采时**必须先让读取器响亮拒绝不认识的列数，并让用例消费真的参考量列**，否则收口项会假完成。
   **记在这里免得下次踩。**

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
2. **§2.3 末的读取器 31 列问题**：不解决（改为响亮拒绝 + 用例消费真列），重采的 `@720` 夹具
   虽然读得进来，但它的参考量列**永远不被消费**，收口项会**看起来做完了**。
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

---

## 8. 2026-09-21 —— 复审修复（本节为**追加**，前面各节按需就地更正并留了对照）

**性质：纯文档 + 一条 `.gitignore`。** 没动任何 `.cpp` / `.h`，没删任何文件，
没改四份快照的内容或名字，没碰那条故意红的断言。⇒ **构建与两套测试不需要重跑**
（本次改动不影响编译或判决链路；Baseline `test_force_compensation` 37/0、
`test_payload_calibration` 69/1 保持不变，因为没有任何输入变了）。

### 8.1（Important）读取器那条事实**说反了** —— 已就地更正

**原写法（两处，错）**

- 本报告 §2.3 末：`t6Load` 只接受 18 列或 25 列的夹具布局（列数不是这两种就 `return false`）…
  **必须同时扩 `t6Load`**，否则**夹具进得去、用例读不出**。
- 计划（fixture 重采那一行）追加注：夹具【读取器】只认 18~25 列，而带 `@720` 的采集是 31 列
  ⇒ 重采时必须【同时扩读取器】，**否则新夹具读不进来**。

**实际行为（已按代码复现）**：读取器逐行调的是 `mgSplitRow(q, c, 25)`，第三个参数是**上限** ——
解析到第 25 列就**停**并返回 `25`。于是**31 列的行返回的就是 `25`**，而两处守卫判的正是列数
是不是 18 / 25（一处"不是 18 也不是 25 才拒"，一处"不是 25 才拒"）⇒ **守卫被满足、行被收下，
最后六列（`@720`）被静默丢弃**。**不是"拒绝"，是"接受并截断"。**

**改正后的写法（两处已改）**

- 本报告 §2.3 末 + §5 第 3 条 + §6 顾虑 2；计划 fixture 重采那一行。

**后果（为什么这是 Important）**：31 列的新夹具**读得进来、套件照样过**，而参考量那一路
**从来没被消费**（用例里参考量那一侧本来就是现算的）⇒ **收口项会看起来做完了、其实什么也没验证**。
那道自称"不静默跳行"的守卫**恰好察觉不到**这种布局变化。

**⇒ 收口项的真实要求（已写进计划该行，写清两条）**：重采夹具时**①** 读取器改成
**响亮拒绝它不认识的列数**（解析时把上限抬高、按真实列数判，**高于支持上限就拒**）；
**②** 用例必须去消费**真的**参考量那一列，而不是现算。

**⚠ 本任务【没有】改测试代码，也不该改**：读取器的改动属于**重采那一次**，
需要**它自己的复审**（它在判决链路上）。

### 8.2（Minor）改动行数勘误

`Touch_Client/force/ForceCompensation.cpp` 在本任务提交里是 **19 行插入 / 6 行删除**
（`git diff --numstat 55fb8ce^ 55fb8ce`），不是初版写的"插入 13 行、删 6 行"。§1 已就地更正。
同一次复核里另核了 §3.1 引用的两个计数（`gateref-task-3-report.md` 2 处、`gateref-task-6-report.md` 8 处）
—— 与 `--numstat` 的 `2/2`、`8/8` 一致，**无误**。

### 8.3（Minor）`.gitignore` 第二条否定规则收窄

原 `!fixtures/force_calib*.json` 是**多余的**、且白扩了不忽略的面积（理由见 §2.4 就地的补注）。
已改为 `!fixtures/*_frozen.json`。核验（`git check-ignore --no-index -q`，退出码 0=被忽略 / 1=不被忽略）：

| 路径 | 改前 | 改后 |
|---|---|---|
| `Touch_Client/tests/fixtures/` 下 9 份夹具（含 4 份快照） | 全不被忽略 | **全不被忽略**（逐份退出码 1） |
| `Touch_Client/tests/fixtures/force_calib.json`（假设的运行时产物） | **不被忽略**（被宽通配救回） | **被忽略**（退出码 0） |
| `Touch_Client/calib/force_calib.json` 与 `calib/force_calib.json` | 被忽略 | **被忽略**（退出码 0） |

### 8.4（Minor）md5 等式的口径补注

§2.2 的四个 md5 是**本机工作区 CRLF 副本**之间的比对（`core.autocrlf=true`、无 `.gitattributes`
⇒ 入库 blob 是 LF 归一过的）。已就地加一句口径说明：**这不是可移植的内容指纹**，
换机器/换换行策略算出的同一份内容 hash 不同，**内容本身不受影响**。
（dispatch 说"a fresh checkout materialises CRLF，所以不会在别处复现"——**结论对、理由不顺**：
`autocrlf=true` 的检出**恰好会**materialise CRLF 并复现这几个数；不复现的是 LF 形态
与换行策略不同的机器。已按可复现的版本落笔。）

### 8.5（Minor）计划里那行过期计数

计划 fixture 重采那一行原写 `test_force_compensation` 基线 **26/0**，已改为 **37/0**（当前实测）。

### 8.6（Minor）三处"数据从哪来"的引用回指快照

按判据（"这批数据从哪来"改、"程序往哪写/当时那个文件的状态"留）逐篇判过，**只改了三处**，
并在每处就地注明了快照名与"活文件已退回运行时不跟踪"。**动手前先核过数据确实在快照里**：
快照里对应块的**数据行**与冻结夹具**逐行相同**（12:38:19 / 15:25:23 / 15:30:03 / 15:33:38 四对，
行数 7 / 9 / 10 / 10，`diff` 无差异）。于是"改指快照"**不会把真话改成假话**。

| 文件 | 改的那一处 | 类别 |
|---|---|---|
| `Docs/superpowers/evidence/cs-bias-report.md` | §1.1 "数据: …四个块"那一行；§1.2 自校表里"7 姿态冻结夹具 vs 同名块"那一行 | 数据出处 / 同一份数据的比对 |
| `Docs/superpowers/evidence/moment-model-structure-report.md` | §1.1 "四次采集都取自 …"那一行 | 数据出处 |
| `Docs/superpowers/evidence/limit-recalibration-report.md` | §7.2 "`A_F` 是在 …那一块上解出来的"那一行 | 被引数字的出处 |

**故意仍留的（同文件内，逐条给理由）** —— 这些是"当时那个文件的状态"或行为/范围陈述，
改指快照会把真话改成假话：

- `limit-recalibration-report.md` 开头"没有碰 …；两份证据的 mtime 仍是采集当时的 `2026-09-19 18:49`"
  与 §末尾"生产代码 / 测试 / 夹具 / `calib_poses.txt` / `calib_report.md`: 未动" —— **状态与范围**陈述。
- `moment-model-structure-report.md` 里构建/运行命令注释中的"跑（默认读 `calib_poses.txt` 的四个块）"
  与"`git diff --stat …` 为空" —— 说的是**那个探针当时的行为**与改动范围。
- 三份文件里凡指**夹具口径 / 量化格式**的句子 —— 依据是**写文件的格式化精度**，不是数据来源。

**⇒ 仍按原样留着的其余历史文档一批**（见 §5 第 2 条）**未动**：本任务**没有**做机械替换。

### 8.7 就地更正过的地方（除本节外的清单）

- 本报告 §1（19/6）、§2.2（md5 口径）、§2.3 末（Fix 1）、§2.4（否定规则）、§5 第 3 条、§6 顾虑 2。
- 计划：fixture 重采那一行（Fix 1 的更正 + 37/0）。
- `.superpowers/sdd/final-minors-triage.md`（**未跟踪**的分诊稿）B1 行也有同一条说反了的事实，
  就地更正了 —— ⚠ **但它是未跟踪文件**，所以**不在本次提交里**（提交只含跟踪文件）。
- 三份证据报告（见 §8.6）。

### 8.8 收尾核验

- `git status`（跟踪面）：**只有本任务有意改的 6 个文件是 `M`**；其余 `??` 是开工前就有的无关未跟踪文件
  （`Hardware/stl.zip`、`Touch_Client/*.log` 等），**与本节无关**。
- **一个文件都没删**：`git status` 无 `D`；四份活标定文件仍在 `Touch_Client/calib/` 原处未被跟踪。
- **四份快照**：未在本节改动（`git diff --name-only` 里没有 `fixtures/` 任何文件）。
- **没有 `.cpp` / `.h` 进 diff**（`git diff --name-only` 过滤后为空）。
- **未跑构建/测试**：本次只碰文档与忽略规则，**不可能**影响编译或判决链路 ⇒ 按硬约束不必重跑。
  ⚠ **说清楚**：下面这两个数**不是本次实测**（本次没跑套件），而是上游任务记录的基线；
  本次没有任何输入变化可能移动它们 ⇒ 记为未变。**不称"全绿"。**
