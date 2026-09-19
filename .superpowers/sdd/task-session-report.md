# Task — 上机结果自动记录成文档 (`calib_report.md`)

- 日期: 2026-09-19
- 分支: `feat/pen-clamp-redesign`
- 依据: `.superpowers/sdd/task-session-report-brief.md` (硬要求 1~9)

---

## 1. 做了什么

`'s'` 求解时那一屏诊断现在**同时**落进 `Touch_Client/calib/calib_report.md`。
每次求解一个自描述块 (块头 + ```` ```text ```` 围栏里的逐字正文 + 机械臂自报负载尾节),
**只追加, 永不截断**。

### 改动清单

| 文件 | 行 | 内容 |
|---|---|---|
| `Touch_Client/core/SessionReport.h` | 新增, 175 行 | 块格式 (纯函数) + 追加写 + stderr 捕获窗口。header-only, 无新 .cpp/.obj |
| `Touch_Client/main.cpp` | 13 | `#include "core/SessionReport.h"` |
| `Touch_Client/main.cpp` | 26–30 | `#include <cstdarg> <string> <vector>` |
| `Touch_Client/main.cpp` | 846–1022 | **sink 本体** (`diagEmit` 865 / `diagEmitf` 873 / `DiagBuf`+`diagOut` 895–912 / `diagBegin` 915 / `diagFinish` 932 / `diagPayloadSection` 956) |
| `Touch_Client/main.cpp` | 1046 | `solveAndApply()` 的第一条语句 = `diagBegin()` |
| `Touch_Client/main.cpp` | 1069 | 姿态数不足的**提前 return 也落块** |
| `Touch_Client/main.cpp` | 1124 | "正在采样"的**提前 return 也落块** |
| `Touch_Client/main.cpp` | 1184–1207 | `fitRaw` 前后开 **stderr 捕获窗口**, 并回本出口 |
| `Touch_Client/main.cpp` | 1431 | 正常出口: `diagFinish(diagPayloadSection(decompOk, fit.cS, d.parity))` |
| `Touch_Client/main.cpp` | 求解路径上的 **81 处打印调用** | `std::cout` → `diagOut()`, `printf(` → `diagEmitf(` (见 §3) |
| `Touch_Client/Touch_Client.vcxproj` | +1 行 | `<ClInclude Include="core\SessionReport.h" />` |
| `Touch_Client/tests/test_session_report.cpp` | 新增, 358 行 | 11 个用例 |
| `Touch_Client/tests/build_session_report_test.bat` | 新增 | 沿用 `build_*.bat` 约定 |
| `Touch_Client/tests/fixtures/console_screen_2026-09-19_run001.txt` | 新增 | 真夹具: run-001 那次上机的整屏 (68 行) |

**没有动的东西** (逐条核过):
`main.cpp:1214–1478` 的 `#if 0` 停用块 (在 diff 里是上下文行, 一行未改);
`logCalibAttempt` / `logPoseData` 的函数体;
`calib_log.txt` / `calib_poses.txt` 的格式与调用时机;
任何阈值、模型、判决逻辑 (`fitRaw` 的入参与判决路径一个字节没变);
`comSignZ` / 下发路径 / 力反馈。

### 块长什么样

```
## 采集 2026-09-19 15:33:38 — 9 姿态 / 3 对

```text
重复姿态对 (3 对, ...): (pose 1, pose 2) ... —— 原地复采的复现性尺子就位
         力x: pair1 d=+0.0423 σ_rep=0.0299 | ... | 池化 σ_rep=0.0208 N
...
[Payload] 逐姿态残差 (力 N / 力矩 N·m / 力残差÷尺子):
            pose  1:   0.0111    0.0011   0.48
...
======================================================
  原始通道 (@1304 SixForceValue) 线性解 — 9 个姿态
...
  → 复验: 【就在 'm' 模式里】再按一次 'm' ...
```

### 机械臂自报负载 (30004 帧)
- @1168 Load        = 0.40400000000000003 kg
- @1176 CenterX/Y/Z = (0.30000000000000004, -0.10000000000000001, 68.699999999999989) mm
- c_s (本次解出, 传感器测量系; 原点 = 传感器的测量原点) = (-0.24399999999999999, 0.34900000000000003, 55.556000000000004) mm, |c_s| = 55.557...
  沿【工具轴】的分量 c_s_z = 55.556000000000004 mm (...)
  横向分量 c_s_x / c_s_y = -0.244 / 0.349 mm。parity = sign(det A) = -1
- d = cz_robot − c_s_z = 68.699999999999989 − 55.556000000000004 = 13.143999999999996 mm
  → 判据 0 < d < 31.5 mm: 【在范围内】✓ 测量原点落在传感器体内
- ⚠ 本节只是信息: 不参与任何接受/拒绝, 也不改变 fitRaw 的判决。
```
(上面是**格式示意**, 数值取自 run-001 的实机结果, 不是某次真实运行的产物 —— 见 §6。)

---

## 2. 同源 (硬要求 1) 是怎么保证的

**不是"再打印一遍到文件"**, 而是: 求解路径上的每一次打印都只经过一个出口 `diagEmit`,
它把**同一份字节**喂给两处:

```cpp
static void diagEmit(const char* s, size_t n) {
    fwrite(s, 1, n, stdout);      // 屏幕
    s_diagBody.append(s, n);      // 文档正文 —— 同一份字节, 同一个调用
}
```

落盘时 `SessionReport::block(...)` 把 `s_diagBody` **原样**夹进围栏 (`s += body;`, 不做任何
转义/换行/精度处理), 所以"文档里那一块 = 屏幕上那一屏"是**结构上**成立的, 不依赖两处格式串
是否写得一样。三条支撑:

1. **求解路径上不再有任何绕过 sink 的打印。** 原来的 81 处 `std::cout` / `printf` 全部换成
   `diagOut()` / `diagEmitf(`。这次替换是**纯文本替换**, 已证明只动了这两个标识符 (见 §5-4):
   格式串、实参、顺序一个字符没变 ⇒ 屏幕上的字节与改动前**逐字节相同**。
2. **回归闸门**: `test_solve_path_prints_only_through_the_sink` 直接读 `main.cpp`, 切出
   `solveAndApply` 的函数体, 断言里面没有 `std::cout` / `std::cerr` / 独立的 `printf(` /
   `fprintf(` / `fputs(` / `fwrite(` / `puts(` / `putchar(`。以后谁在求解路径上"顺手加个打印",
   文档就会比屏幕少一行 —— 这个测试会红。
   (`snprintf(` 是允许的: 它只往缓冲里写, 上屏的那一步仍然走 sink。)
3. **stderr 那一半也并进来了**: 库 (`fitRaw`) 的逐姿态残差表与 `[Payload]` 自检行原本直接
   `fprintf(stderr, ...)`, 不经过 sink —— 而它们是"为什么被拒"的唯一出处, 也在操作员屏幕上。
   做法: 调用 `fitRaw` 前后各一次 `_dup2` 把 fd 2 指向临时文件, 收完**先无条件还原**, 再把收到
   的字节**从同一个出口** `diagEmit(...)` 发出去。所以屏幕上的字节序列与块里的**仍然逐字相同**
   (屏幕顺序也保持: 前面 stdout 的行 → 库的 stderr 段 → 后面的 stdout 行, 与 run-001 记录下来的
   那一屏顺序一致)。
   失败时 (临时文件开不了 / 读不出来) **stderr 一个字节都不动**, 只在块尾照实写一句
   "这一段没能并入本块" —— 不许静默省略。

### 与硬要求 7 (全精度) 的冲突, 以及怎么处置的

要求 7 说"文档里凡是数值都写足 double 位数", 要求 1 说"文档那一块与屏幕逐字节一致"。
**正文那一半两者不可兼得** (正文就是屏幕的字节), 本实现让**要求 1 赢**:

- ```` ```text ```` 围栏里的正文 = 屏幕的逐字字节, 数值沿用控制台格式 (`%.3f` / `%.4g` 等),
  一个字都不改 —— 否则"同源"这条验收就废了, 而且会变成"改现有行为"。
- **文档自己新增的内容一律全精度**: 尾节里的 `@1168 Load` / `@1176 CenterX/Y/Z` / `c_s` 三个
  分量与模 / `d` 与它的减法算式, 全部用 `%.17g` (double 的往返精度)。
- 所以"读的人要靠它复算"这件事, 对**本次新增的判据量** (c_s / cz_robot / d) 是满足的;
  对正文里那些拟合量 (A / 奇异值 / σ) 仍受控制台格式限制 —— 这是上面那条取舍的直接后果,
  **照实记在这里**, 不当作"已满足"。

---

## 3. 打印调用是怎么改的 (以及为什么可以信任)

**没有手改任何一行打印语句。** 用一条正则对 `main.cpp` 的 858–1213 行 (旧行号, 即
`solveAndApply` 的活体部分) 做**两个纯文本替换**:

```
(?<![A-Za-z0-9_])printf(   →   diagEmitf(        (30 处; snprintf( 因前瞻被排除)
std::cout                  →   diagOut()         (51 处)
```

`diagOut()` 刻意取 9 个字符, 与 `std::cout` 等长 —— 多行 `<<` 链的对齐一格都没动。
多行 `printf` 的续行按 `diagEmitf(` 的左括号重新对齐 (只动行首空白)。

替换的**完备性**是量出来的, 不是看出来的 (见 §5-4): 替换后把两个标识符换回原名, 与改动前的
原文**逐字节相同**。也就是说控制台的输出不可能因为这次重构而变化。

`diagEmitf` 用 `vsnprintf` 先量长度: 超过 1 KB 就改用堆缓冲, **不设固定上限** (定长缓冲遇上
变长的数会静默截断, 而截断掉的正是落盘要保住的东西)。

---

## 4. 两处需要说明的判断 (与 brief 的措辞有出入的地方)

### 4.1 块是**每一次**按 `'s'` 都写, 不只是 `logPoseData()` 那一处

brief 要求 8: "与 `logPoseData()` 同一个调用点 (`count < 4` 之后的 `logPoseData();` 附近) ——
被拒绝的那几次同样要留下"。字面上的"同一个调用点"做不到"屏幕与块逐字一致", 因为
`logPoseData()` **之前**还有两行会打到屏幕上:

- `solveLocked` 的连续失败提示 (`main.cpp:1052`);
- `count < 4` 的那一行 `[BIAS] 求解至少需要 4 个姿态` (`main.cpp:1058`) ——
  它后面就直接 `return` 了, `logPoseData()` 根本不会被执行。

所以 `diagBegin()` 放在 `solveAndApply()` 的**第一条语句** (`main.cpp:1046`), 三个出口
(姿态数不足 1069 / 正在采样 1124 / 正常结束 1431) **各自落块**。这样:
"每一次按 `'s'` 在屏幕上出现的那一份字节" 与 "该次文档块里的正文" 对**每一次**都成立,
包括被拒的那几次 (brief 明令要留)。代价是: `count < 4` 这种**没有姿态数据**的尝试也会留下
一个短块 (正文一行, 尾节照实报"不可用")。我认为这比"控制台上打了、文档里没有"更符合要求 8
的本意, 但**这是一个偏离字面措辞的选择**, 记在这里。

### 4.2 `c_s` "沿工具轴的投影" 取哪个分量

- 取 **`RawFit::cS[2]`**, 即 c_s 在**本模型自己的传感器测量系**里的 z 分量, 乘 1000 得 mm。
- 依据: 该系由 `TcpCalibration::gravitySensorFrameAtYaw(pose, 0.0, g)` 定义 —— **psi 传 0**,
  而 psi 是绕 z 的旋转, 所以这个系的 z 轴**与 psi 无关**, 就是工具轴;
  `RawFit::cS` 与 `A·g` 同在力/力矩读数所在的那个系里, 于是 `cS[2]` 就是沿工具轴的分量。
  实机 run-001 的数也吻合这个读法: `c_s = (-0.244, +0.349, +55.556) mm` (横向分量很小),
  设计 §6b 的算式用的正是 `68.7 − 54.55` 这种**单值**相减。
- **我没有能定下来的一件事, 照实写**: 该系的 z 轴与工具轴的**指向**是否同向, **数据定不了**
  (`A` 是自由 3×3、含反射, 实机解出 `parity = −1`)。上面按"同向"相减 —— 若实际反向, `d` 会
  整体变负。这一点**写进了文档块本身** (读者能看到这个前提), 不是只写在任务报告里。
- 因此文档里 `c_s` **三个分量 + 模**都打了 (全精度), `d` 的算式也把两个操作数都打出来 ——
  读的人可以按另一种解释自己复算。

---

## 5. 验收命令与输出

### 5-1 构建

```
$ cmd.exe //c "D:\Projects\Touch\Touch_Client\build.bat"
  Touch_Client.vcxproj -> D:\Projects\Touch\Touch_Client\x64\Release\Touch_Client.exe
  Build OK.
  DLLs copied.
```

无新增编译告警 (仅原有两条: `MSB8004` 与 `glut.h`/`minwindef.h` 的 `CALLBACK` 重定义)。
**构建能写成 exe ⇒ 当时 `Touch_Client.exe` 没有在运行** (LNK1168 一次都没出现);
我全程**没有启动也没有结束**任何 `Touch_Client.exe` 进程。

### 5-2 原有用例不许动

```
$ cmd.exe //c "D:\Projects\Touch\Touch_Client\tests\build_payload_calibration_test.bat"   → BUILD_EXIT=0
$ tests\test_payload_calibration.exe | tail -2
45 passed, 0 failed
```

### 5-3 新用例

```
$ cmd.exe //c "D:\Projects\Touch\Touch_Client\tests\build_session_report_test.bat"        → BUILD_EXIT=0
$ tests\test_session_report.exe
=== SessionReport Tests ===
  block_header_format... PASS
  block_wraps_body_verbatim... PASS
  block_closes_fence_when_body_has_no_trailing_newline... PASS
  block_trailer_goes_after_the_fence... PASS
  block_wraps_a_real_console_screen_verbatim... PASS
  append_preserves_existing_bytes... PASS
  append_creates_missing_file... PASS
  append_on_unreadable_file_reports_failure_without_truncating... PASS
  solve_path_prints_only_through_the_sink... PASS
  stderr_capture_roundtrip_and_restore... PASS
  stderr_capture_begin_failure_leaves_stderr_alone... PASS

11 passed, 0 failed
```

按 brief 的验收条目逐个对上:

| brief 要求 | 用例 |
|---|---|
| 给一段固定诊断文本, 断言块把它**完整包住**、块头格式正确 | `block_wraps_body_verbatim` (断言正文的**偏移与长度**都对: `blk.compare(head.size(), body.size(), body) == 0`), `block_header_format` (逐字比 `## 采集 … — 10 姿态 / 5 对`) |
| 同上, 用**真**东西 | `block_wraps_a_real_console_screen_verbatim`: run-001 那次上机的整屏 (68 行, 含库打到 stderr 的 `[Payload]` 段), 逐字节比对 |
| 追加语义 —— 已有内容一个字节不动 | `append_preserves_existing_bytes` (先放两个旧块, 追加两次, 逐段比对 + 长度) |
| 追加语义的**底线** —— "读不了"不得退化成截断 | `append_on_unreadable_file_reports_failure_without_truncating`: 文件设只读 → `appendToFile` 返回 false, 而**历次记录一个字节都没少** |
| 拒绝的那一次**也**留下块 | `main.cpp:1069` 与 `main.cpp:1124` 两个提前 return 各自 `diagFinish(...)`; 判决失败的那次走 1431 的同一个出口 (它不在任何 `if (fitOk)` 里)。用 `solve_path_prints_only_through_the_sink` 之外的**读代码**说明: 三个出口 + 无第四处 return (range 内 `return;` 只有这两处, 已核) |
| stderr 段不丢 | `stderr_capture_roundtrip_and_restore` (收得回来 + fd 2 还原 + 临时文件删掉), `stderr_capture_begin_failure_leaves_stderr_alone` |

### 5-4 打印调用替换的完备性 (这条是量出来的)

改动前的那一份从 git 取回 (`git show <本次提交>^:Touch_Client/main.cpp > main.cpp.bak`), 然后:

```
$ awk 'NR>=858 && NR<=1213' main.cpp.bak | perl -pe 's/(?<![A-Za-z0-9_])printf\(/diagEmitf(/g; s/std::cout/diagOut()/g;' > a.txt
$ awk 'NR>=858 && NR<=1213' main.cpp     > b.txt
$ diff a.txt b.txt && echo "IDENTICAL: transformation is exactly the two substitutions"
IDENTICAL: transformation is exactly the two substitutions
```
(先按原文替换、再与改后文件对: 只有这两个标识符不同, 其余逐字节相同。
`std::cout` 计数 51→51、`printf(` 30→30、残留 `std::cout` 0、残留独立 `printf(` 0。)

---

## 6. 没能验证的地方 (照实说)

1. **没有在实机/真实进程上跑过一次 `'s'`。** 没有机械臂与触觉设备, 而且 brief 明令不许动
   正在运行的 `Touch_Client.exe`。所以"跑一次 `'s'`, 文档里那一块与屏幕上的必须一致"这条
   **端到端验收没有被执行过** —— 它是靠 §2 的三条结构/回归保证 + §5-3 的单测推出来的,
   不是实测出来的。
2. **`main.cpp` 里的部分代码不被任何测试覆盖**: `diagEmit` / `diagEmitf` / `DiagBuf` /
   `diagOut` / `diagBegin` / `diagFinish` / `diagPayloadSection`, 以及 `fitRaw` 外面那个
   stderr 捕获窗口的**接线**。能测的那一半 (块格式、追加语义、stderr 捕获机制本身) 已经测了;
   接线那一半只能读代码。`diagPayloadSection` 需要一个真 `appState` 才有内容, 单测里跑不了。
3. **块尾"机械臂自报负载"一节从没在真实数据上出现过**: `payloadEchoValid` 要有 30004 帧才会
   置 true。所以"@1168 / @1176 打印成什么样、d 算出来是几"只有格式示意 (§1 末), 没有实测样本。
   `不可用` 那条分支同样只能靠读代码。
4. **`calib_report.md` 这个文件是这次新引入的, 一次都还没被写过。** 目录 `calib/` 里的另两份
   日志照旧; 没往 git 里加任何运行期文件。
5. **stderr 捕获窗口的两个已知边角** (已写进 `SessionReport.h` 的注释):
   窗口内**别的线程**若写 stderr, 它的字节也会落进临时文件, 随后被一并打进 stdout 与文档块
   (屏幕不丢, 但这行会出现在块里; 本窗口只有 `fitRaw` 一次纯计算, 时长极短);
   窗口内 [Payload] 段的字节改由 stdout 出口发出, 若有人用 `2>file` 分开重定向, 那几行会跟着
   stdout 走 —— 交互式运行下两者都是同一个控制台, 没有差别。
6. **没有把新测试挂进 `tests/run_tests.bat`** (那是一份手工挑选的清单, 连
   `test_payload_calibration` / `test_calib_store` 都不在里面)。要挂的话是一行的事:
   照 `build_force_comp_test.bat` 那一段的样子加。

---

## 7. 一处顺带记下的设计取舍

`appendToFile` **保持 brief 要求的 `fopen(path, "a+")`**, 但紧接着做了一次
`_setmode(_fileno(f), _O_BINARY)`。理由: `"a+"` 的文本模式会把 `\n` 翻成 `\r\n`,
那么"文档里那一块"与"屏幕上的那一屏"就**不再逐字节相同**了 (而逐字节相同正是本任务要验的东西),
文件也会变成 CRLF (本仓库的 `Docs/**/*.md` 一律是 LF, 173 行里 0 个 `\r`)。
"只追加、永不截断"只由打开方式决定, 与二进制与否无关 —— 所以 `"a+"` 一个字没动,
只把流翻译关掉。`append_creates_missing_file` 里有一条
`CHECK(readWholeFile(path).find('\r') == std::string::npos)` 把这件事钉住。

---

# 8. 复审修复 (I1 / I2 / I3 + M3) — 2026-09-19

复审对 `beb5bcc` 提了三条 Important 与一条便宜的。三条都改在本报告所述的同一个特性里,
**没有动** `logCalibAttempt` / `logPoseData` / 两份日志格式 / `#if 0` 块 / 任何阈值 / 模型 /
任何判决路径, **也没有改控制台输出的任何一个字节**(下面 §8-5 核过)。

## 8-1 I1 — 尾节的判决不是数据定的, 而块里那句"反向会变负"算错了

**改成: 两种符号约定都打, 且整节一个勾都不出现。**

- 新增纯函数 `SessionReport::payloadDSection(cz_robot, c_s_z, fitOk)` (`core/SessionReport.h:67-124`,
  定义在 `89`): 约定一 (同向) `d = cz − c_s_z`, 约定二 (反向) `d = cz + c_s_z`, 两个数都用
  `%.17g` 打全精度, 各自照实跟一句 `【在范围内】` / `【在范围外】`, 再给一行
  `→ 判据 0 < d < 31.5 mm: 两种约定下结论【相同】/【相反】 —— 这一条要【成对读】… 本块【不给单一的勾】`。
- **一个 ✓/✗ 都不再出现**(连解释那句话里也不用这个字符), 所以 `grep ✓` 在这一节里必然为空 ——
  读者无法从块里摘出一个能被单独引用的"通过"。
- 算错的那句删了。`main.cpp:1014-1018` 现在是: 模型在 `g → −g, A → −A, c_s → −c_s` 下逐字不变,
  所以**只有 `|c_s_z|` 是数据定的、符号不是**; 反向的约定把 `cz − c_s_z` 变成 `cz + |c_s_z|` ——
  **那是另一个正数, 不是变负**(run-001: `68.700 + 55.556 = 124.256 mm`)。
- ⚠ **与 `aad9733` 的对齐**(评审在本修复进行中落的文档提交): 它把这条判据明确定义成
  **符号判别器**(上机操作单 §6 闸1: 恰好一支落在内 = 那就是正确的符号约定 / 两支都在内 =
  符号定不了, 不许猜 / 两支都在外 = 哪里错了, 不得下发), 并写明"实现侧由修复提交一并改为
  打印两种约定"。所以这一节的收尾句写成 **"要成对读 (用法见上机操作单 §6 闸1); 本块不给单一的
  勾, 别只认其中一支"** —— 而不是我自己先写过的"不得据它下发": 后者在"恰好一支落在内"那一格
  与刚改过的操作单**直接打架**, 而那条规矩管的是"要不要下发 cz"(Task 9 的事), 不属于本节
  "信息"的范畴。这里只把两个数摆出来, 不替读者裁定。
- run-001 的实数在这个实现下的实际输出(`fitOk = true`, 见 §8-7 的核验方式):

  ```
  - d = cz_robot − c_s_z, 【两种符号约定都算】= 13.143999999999984 / 124.256 mm
    (c_s 的 z 向【符号】不由数据决定: 模型在 g → −g, A → −A, c_s → −c_s 下逐字不变,
     反向的约定给的是另一个数、不是负数。所以这里【不】给单一的勾/叉 —— 整节一个
     勾都不出现, 谁也别想从块里摘出一个【通过】去。)
      约定一【同向, c_s_z = +55.556000000000004 mm】: d = 68.699999999999989 − 55.556000000000004 = 13.143999999999984 mm  【在范围内】
      约定二【反向, c_s_z = −55.556000000000004 mm】: d = 68.699999999999989 + 55.556000000000004 = 124.256 mm  【在范围外】
    → 判据 0 < d < 31.5 mm: 两种约定下结论【相反】 —— 这一条要【成对读】(用法见上机操作单 §6 闸1): 本块【不给单一的勾】,
      别只认其中一支。
  ```

  (这一段是用一个临时程序真的调 `payloadDSection` 打出来的, 编译产物落在系统 temp 里、已删,
  没有落进仓库; `13.143999999999984` 与评审文字里的 `13.143999999999996` 差在浮点字面量的
  取值上, 不是算法差异。)

## 8-2 I2 — 尾节不再对被拒的那一次下无条件结论

`diagPayloadSection` 增加第四个形参 `fitOk` (`main.cpp:969`), 三个出口都传 (`1081` / `1137` /
`1456`)。它**只被读来决定尾节那一行后面要不要跟一句限定**, `fitRaw` 的入参、阈值、判决与
`logCalibAttempt` 的 outcome 一个字节没动。

- `fitOk == false` 时, 两种约定的两行照旧全打(限定, 不是删掉), 末尾多一行:
  `（本行仅描述 d 的算术位置; 本次 fitRaw 为【拒绝】, 该结论不成立）`
- `fitOk == true` 时这一行不出现 —— 两个输出**逐字前缀相同**(测试把这条钉住了), 说明加的是尾巴,
  不是改了数。
- 两个提前 return(姿态数不足 / 正在采样)传 `false`: 那里 `fitRaw` 根本没跑, 且 `haveCs = false`
  时 d 那一节整段不打印, 所以它在那里不产生任何字(`main.cpp:1081` / `1137` 的注释里写明了)。

## 8-3 I3 — 收不回来的 stderr **不再被删掉**, 而且报告里那句错误陈述已纠正

`SessionReport::stderrCaptureEnd` (`core/SessionReport.h:213`): `_open` 读失败时**不再 `_unlink`**,
改成把路径记进 `st.leftoverPath` (`167`), 由 `stderrCaptureLeftoverPath()` (`177`) 交给调用方。
理由写在函数顶上: 那一句 `fflush(stderr)` 已经把字节推进文件了, 此刻盘上那份是**唯一的副本**
(它们没进控制台, 因为窗口里 fd 2 指着这个文件), 删掉就是"安静地销毁诊断"。
读成功时照旧删(字节已经在 `out` 里)。

调用方 (`main.cpp:1216-1226`) 的警告因此**带上了路径**: `原始字节没有丢, 它们还在临时文件里: <路径>`。

§2-3 那句 **"失败时 (临时文件开不了 / 读不出来) stderr 一个字节都不动" 是错的**, 本次纠正:
它对 `Begin` 失败成立, 对 `End` 读失败**不成立**(那时字节已经不在 stderr 上了)。正确说法是:
**窗口没搭起来时 stderr 一个字节都不动; 窗口搭起来之后字节就已经离开 stderr 了, 所以收不回来时
保证的是"副本还在盘上、路径报得出来", 不是"原地没动"。** (§2-3 那一段正文本次未改 ——
以本节为准。)

## 8-4 M3 — `DiagBuf::sync()`

`main.cpp:913-915` 覆盖 `sync()` 为 `fflush(stdout)`, 把 `std::endl` 的冲刷原样接回来
(从前是 `std::cout`, `sync_with_stdio` 默认 true 所以 endl 确实冲刷)。**只把字节推出去, 不改字节。**

## 8-5 验收命令与输出

```
$ cmd.exe //c "D:\Projects\Touch\Touch_Client\build.bat"
  Touch_Client.vcxproj -> D:\Projects\Touch\Touch_Client\x64\Release\Touch_Client.exe
  Build OK.
  DLLs copied.
  Models copied.
```
(与上次一样只有原有两条告警: `MSB8004` 与 `glut.h`/`minwindef.h` 的 `CALLBACK` 重定义;
没有出现 LNK1168 = 构建期间 `Touch_Client.exe` 没有在跑; 全程没有启动也没有结束任何进程。)

```
$ cmd.exe //c "D:\Projects\Touch\Touch_Client\tests\build_session_report_test.bat"   -> BUILD_EXIT=0
$ tests\test_session_report.exe
  block_header_format... PASS
  block_wraps_body_verbatim... PASS
  block_closes_fence_when_body_has_no_trailing_newline... PASS
  block_trailer_goes_after_the_fence... PASS
  block_wraps_a_real_console_screen_verbatim... PASS
  payload_d_section_prints_both_sign_conventions... PASS      ← 新增 (I1)
  payload_d_section_qualifies_a_rejected_fit... PASS          ← 新增 (I2)
  append_preserves_existing_bytes... PASS
  append_creates_missing_file... PASS
  append_on_unreadable_file_reports_failure_without_truncating... PASS
  solve_path_prints_only_through_the_sink... PASS
  stderr_capture_roundtrip_and_restore... PASS
  stderr_capture_begin_failure_leaves_stderr_alone... PASS
  stderr_capture_end_read_failure_keeps_the_bytes... PASS     ← 新增 (I3)
14 passed, 0 failed
```

```
$ cmd.exe //c "D:\Projects\Touch\Touch_Client\tests\build_payload_calibration_test.bat"  -> BUILD_EXIT=0
$ tests\test_payload_calibration.exe | tail -2
45 passed, 0 failed
```

新增的三个用例 (`tests/test_session_report.cpp`):

| 用例 | 钉住什么 |
|---|---|
| `payload_d_section_prints_both_sign_conventions` | 两支的 `%.17g` 数都在; 两个结论都打; 出现 `【相反】`; **`✓`/`✗` 一个都没有**; 有 `本块【不给单一的勾】` 与 `成对读`; **`变负` 字样不许再出现**(而 `cz + c_s > 0` 这一事实层面也断言了) |
| `payload_d_section_qualifies_a_rejected_fit` | `fitOk=false` 有 `本次 fitRaw 为【拒绝】`/`该结论不成立`; `fitOk=true` 没有; 且**前者逐字是后者的前缀**(限定是加上去的, 数一个没改) |
| `stderr_capture_end_read_failure_keeps_the_bytes` | 真跑那条失败路径(见下) |

## 8-6 I3 的"不删"是怎么验的 (不是只读代码)

`stderrCaptureEnd` 里那个 `_open` 失败不好造: 文件刚由我们自己创建, 而且 Windows 没有
"只写不可读"的属性。做法是**让它解析不到** —— 给 `Begin` 一个**相对路径**, 窗口开着的时候
`_chdir` 进临时目录, 那一句 `_open(相对路径)` 就落到别处去了:

1. `stderrCaptureBegin("test_session_report_tmp/cap_fail.tmp")` 成功 (fd 2 已指向它);
2. 往里写一行 `END-READ-FAIL 这一段收不回来, 但必须留在盘上` 并 `fflush(stderr)`
   —— **这一句就是把字节从 stderr 推到文件里的那一步**;
3. `_chdir(TMPDIR)`, 于是那个相对路径解析成 `…/test_session_report_tmp/test_session_report_tmp/…`
   → `_open` 返回 −1;
4. `stderrCaptureEnd` 返回 **false**(照实);
5. 断言: `stderrCaptureLeftoverPath() == 那个相对路径`; **`_access(path) == 0` (文件还在)**;
   **读回来的内容逐字等于第 2 步写进去的那一行**(⇒ 字节真的可捡回, 不是"也许还在");
   fd 2 已还原(再开一个窗口只收到新字节, 且那次成功后 leftover 为空串);
6. 测试自己把那个文件删掉(它本该留在盘上, 只有测试知道它是测试的产物)。

这条用例在**改之前会是红的**(旧代码 `_unlink` 掉了文件, `_access` 与内容断言都会失败)。

## 8-7 同源 (硬要求 1) 有没有被这次修改碰到

没有。`diagEmit` / `diagEmitf` 一个字节没动 (`main.cpp:875` / `883`), 求解路径上的打印调用一处未改;
本次改的三处(**尾节字符串**、`DiagBuf::sync()`、**stderr 捕获收尾**)里, 尾节本来就不上屏
(`diagFinish` 只写文件), `sync()` 只影响冲刷时机、不产生字节, 而 stderr 那一半只改了
"读失败之后删不删临时文件"与警告措辞。**控制台的字节输出与改动前逐字相同**,
所以"文档块正文 = 屏幕那一屏"这条仍然成立。

## 8-8 仍然没能验证的 (照实说)

1. 与 §6-1 相同: **端到端那条验收(真机跑一次 `'s'`)仍然没做** —— 没有设备, 也不许动
   正在运行的 `Touch_Client.exe`。I1/I2 这次是**纯函数单测**覆盖的(尾节是纯函数, 这一步现在
   真的能测了); 但 `fitOk` 从 `fitRaw` **接到** `diagPayloadSection` 的那一根线, 仍然只有读代码。
2. I1 的"两个约定到底哪一个对"**本次仍然没有解决**, 只是把这件事摆到台面上(这正是 I1 要的):
   块里现在给不出"通过", 判据本身也仍然不参与任何判决。
3. `payloadDSection` 里 `cz + c_s` 的那一支, **没有任何真实数据**证明机械臂报的 c_s 会落在哪一支;
   run-001 的数只是"两种约定下结论相反"的一个实例。
4. M3 的效果(`stdout` 被重定向时不憋到进程退出)**没有实测** —— 手上没有一个把 stdout 接进
   文件的运行实例; 断言只到"覆盖了 `sync()`, 且它调 `fflush(stdout)`"。

---

# 9. 追加 (2026-09-19 第二遍): 复审判为"上一次修改新引入"的三条 Minor —— N1 / N2 / N4

> 复审在 `5df1489` 上又标出三条 **Minor，且都判为"由 `5df1489` 新引入"**。本次【只改这三条】,
> 其余一律没碰: `logCalibAttempt` / `logPoseData` / `calib_log.txt` / `calib_poses.txt` 的格式、
> `#if 0` 块、任何阈值、模型、判决逻辑、`fitRaw` —— 一个字节没动。append-only (`"a+"`)、
> 块/围栏结构、"每次 `'s'` 都留一个块"这三条性质照旧。
>
> **硬要求 1(块正文 = 屏幕上那一份字节)**: **成功路径上一个字节没变**。三条修改里唯二会落到
> 控制台字节上的是 N1 与 N4, 而 N1 只改 `sync()` 的**返回值**(不发字节, `fflush` 照旧调),
> N4 只动**读失败那条分支**(见 §9-6: 那是本次要改掉的"半截被当成整段")。

## 9-1 N1 —— `sync()` 返回 −1 会把 ostream **永久**毒掉

**改前** (`main.cpp:913`): `int sync() override { return fflush(stdout) == 0 ? 0 : -1; }`

`std::ostream::flush()` 在 `pubsync()` 返回非 0 时 `setstate(badbit)`, 而异常掩码是默认值 ——
**不抛异常**。于是从那一次起, 每一个 `diagOut() << ...` 都变成**静默的空操作**, 而 `diagEmitf(...)`
那一路照旧在打: 屏幕与文档块**同时**缺行, 且没有任何提示。最容易撞上的场合就是
`Touch_Client.exe > log.txt` 跑到一半卷满 —— 第一次失败的 `fflush(stdout)` 把这一轮剩下的
ostream 诊断**永久**静音, 块的正文从此不全, 而块本身照样写成。这正是本模块开头点名的
"诊断被安静地藏起来"。

**改后** (`main.cpp:913-923`): `(void)fflush(stdout); return 0;` —— 冲刷照旧(**那才是 `sync()`
的本职**), 只是**不把结果当失败报**。字节已经由 `diagEmit` 的 `fwrite` 交给 C 流了, 这个返回值
唯一的作用是决定 badbit, 这里没有"失败"可报 (C 流的写失败另有 `ferror(stdout)` 这条出路)。

**这条没有单测** —— 理由见 §9-7 第 1 条 (造不出一次真的 `fflush` 失败)。改的是 2 行,
判据是"返回值恒为 0", 读代码即成立。

## 9-2 N2 —— 两个约定的标签把 `+`/`−` 写死在格式串里, 负的 `c_s_z` 会印成 `+−55.556`

`cS` 是法方程解出来的**原始解**, 没有任何符号归一 (`force/PayloadCalibration.cpp` 的 `cS` 直接
来自求解), 而这一节的论点恰恰是**符号不由数据定** —— 所以负的 `c_s_z` 与 run-001 的正值
**一样可能**。改前 (`core/SessionReport.h:107-113`) 两个标签写死了 `+%.17g` / `−%.17g`:
负值下会印出 `c_s_z = +-55.556…` 与 `c_s_z = −-55.556…`, **标签与紧挨着的那个数互相打架**,
而且正好发生在那段论证"符号未定"的文字里面。

**改后**: 两个约定的标签都写成 `c_s_z = %.17g mm` —— 符号**由数字自己带出来**;
约定一用 `c_s_z` 本身, 约定二用它取负 (反向就是**这一个数**)。**约定的区别(同向/反向)是真的,
标签保留**; 谁也不加修饰。`d` 的表达式与运算符、判据、`fitOk` 那一段限定一个字没动。

run-001 那一支 (`csZ = +55.556`) 现在印出:

```
    约定一【同向, c_s_z = 55.556000000000004 mm】: d = 68.699999999999989 − 55.556000000000004 = 13.143999999999984 mm  【在范围内】
    约定二【反向, c_s_z = -55.556000000000004 mm】: d = 68.699999999999989 + 55.556000000000004 = 124.256 mm  【在范围外】
```

负的那一支 (`csZ = −55.556`, 本次新增用例喂的) 现在印出 —— 两个数都带**自己的**符号:

```
    约定一【同向, c_s_z = -55.556000000000004 mm】: d = 68.699999999999989 − -55.556000000000004 = 124.256 mm  【在范围外】
    约定二【反向, c_s_z = 55.556000000000004 mm】: d = 68.699999999999989 + -55.556000000000004 = 13.143999999999984 mm  【在范围内】
```

(第二行那个 `+ -55.556…` 是**照实**的: 反向的 `d = cz + c_s_z`, 而这里的 `c_s_z` 是负的。
它与第一行的 `− -55.556…` 是同一条算术, 不是"变负"。`+`/`−` 在这里是**运算符**,
不是给数字贴的符号 —— 与本次去掉的那种"贴上去的符号"不是一回事。)

**单测** (新增 `test_payload_d_section_negative_cs_prints_no_fabricated_signs`): 喂
`csZ = −55.556000000000004`, 断言 1) `s` 里**不出现** `+-` 与 `−-`; 2) `= %.17g mm` 的
**两个**带符号值 (正负各一) 都在; 3) `约定一【同向` / `约定二【反向` 两个标签还在;
4) 两个 `d` 与"【相反】"照旧都在; 5) **正值那一支也照同一条规矩**(不许只在负值上打补丁)。

## 9-3 N4 —— 读到一半失败 ≠ EOF: 半截不许当成整段发出去

**改前** (`core/SessionReport.h:236-243`): `while ((n = _read(...)) > 0) {...}` 之后**无条件**
`_close(fd); _unlink(st.tmpPath);`。中途读失败 (I/O 错误 / 杀软正占着文件) 与"读完了"长得
一模一样: `out` 里是**半截**、`ok` 却是 **true**, 调用方 `diagEmit(errText…)` 把它当成整段发出去,
临时文件还被**删掉** —— 缺的尾巴两处都没有了, 而且**没有警告**。文件里那句新注释
"读不出来时【不删】"比代码宽 (代码只覆盖 `_open` 失败), 这一条就是把代码补到与注释一致。

**改后**: `n < 0` 与 `fd < 0` (**打不开**) **同等对待** —— `ok = false`、文件**不删**、
路径报给 `stderrCaptureLeftoverPath()`; 另外把**本调用已经追加进 `out` 的那半截退回去**
(`out->resize(outLen0)`)。"与 `_open` 失败同等对待"字面上就是这个意思: 那条路上 `out`
**一个字都不并入**, 所以调用方那句"这一段**没能并入**本块"才是真的 —— 否则它旁边会摆着
一段被截断的表, 读者会把它当成整张表 (正是本模块最忌讳的"安静地错")。字节**没丢**:
它们在不删的那个文件里, 路径照报, 人还能捡回来。

> ⚠ 这是本次唯一一处会让**控制台**字节与改前不同的地方 (只在读失败那条分支): 从前它会把
> 半截表打出去 + 一句"没能并入", 现在一个字都不打 + 那一句 + 路径。**正常路径 (读成功) 一个
> 字节没变**, 所以硬要求 1 不受影响。

## 9-4 N4 的单测: 那条路径**真的被走到了** (不是"理论上会失败")

新增 `test_stderr_capture_end_read_error_is_a_failure_not_eof`。

**先说为什么非注错不可**: Windows 上"打开了却读不出来"造不出来 —— 实测 (探针程序) 拿一个
目录去 `_open(_O_RDONLY)`, 结果是 `fd=-1 errno=13 (EACCES)`, 也就是**在 `_open` 那一步就被挡掉**,
落进的是上一条用例覆盖的那条老路; 只读文件照样读得出来。所以"中途读失败"这条路
**从前没有任何测试走到过**, 只能注错: 在测试 TU 里**先取到真的 `_read`**, 再用宏把
`SessionReport.h` 里出现的 `_read` 改名到一个转发函数 (`tests/test_session_report.cpp:22-49`),
置位后**先交出去 4 个字节**(制造"半截"), **再返回 −1**(读坏了)。**注错只在这个 TU 生效,
产品代码一个字节没动。**

**怎么知道它真的走了那条路** (三条计数断言, 先于其它断言):

| 断言 | 含义 |
|---|---|
| `g_fakeReadCalls > before` | 头文件里那一句 `_read` **确实**经过注错点 (宏接上了) |
| `g_fakeBytesServed == before + 4` | **半截真的进过 `out`** —— 旧代码里它就是这么出去的 |
| `g_fakeReadErrors == before + 1` | 那一次**确实返回了 −1** (读坏了, 不是 EOF) |

**然后才是行为断言**: `!ok` (**旧代码这里是 `true`**) / `out.empty()` (**旧代码这里是那 4 个
字节**) / fd 2 已还原 / `stderrCaptureLeftoverPath() == 那个路径` / `_access(path) == 0`
(**旧代码把文件 `_unlink` 掉了**) / **读回来的内容逐字等于写进去的那一行**(半截那 4 个字节也
在里面 ⇒ 字节真的可捡回)。

**红-绿都跑过**(不是"写完就绿"):

* 把 `SessionReport.h` 的两处修改**暂时撤掉**(`git stash push -- Touch_Client/core/SessionReport.h`),
  重新编译运行 → **`14 passed, 2 failed`**, 红的正是这一条与 N2 那一条:
  ```
    payload_d_section_negative_cs_prints_no_fabricated_signs... FAIL: s.find("+-") == std::string::npos
    stderr_capture_end_read_error_is_a_failure_not_eof... FAIL: !ok
  ```
  (N4 那条**先**过了三条计数断言才在 `!ok` 上红 —— 说明注错确实打中了那一句 `_read`,
  红的是行为, 不是"没注上"。)
* 恢复修改后重编 → **`16 passed, 0 failed`**。

## 9-5 验收命令与输出 (逐条照抄)

```
$ cmd.exe /c "D:\Projects\Touch\Touch_Client\build.bat"
  Touch_Client.vcxproj -> D:\Projects\Touch\Touch_Client\x64\Release\Touch_Client.exe
  Build OK.
  [2/2] Copying DLLs...   DLLs copied.
  [3/3] Copying models... Models copied.
  Build complete. Run: D:\Projects\Touch\Touch_Client\x64\Release\Touch_Client.exe

$ cmd.exe /c "D:\Projects\Touch\Touch_Client\tests\build_session_report_test.bat"
  ... /out:test_session_report.exe ... BUILD_EXIT=0
$ Touch_Client\tests\test_session_report.exe
  block_header_format... PASS
  block_wraps_body_verbatim... PASS
  block_closes_fence_when_body_has_no_trailing_newline... PASS
  block_trailer_goes_after_the_fence... PASS
  block_wraps_a_real_console_screen_verbatim... PASS
  payload_d_section_prints_both_sign_conventions... PASS
  payload_d_section_qualifies_a_rejected_fit... PASS
  payload_d_section_negative_cs_prints_no_fabricated_signs... PASS     ← 新增 (N2)
  append_preserves_existing_bytes... PASS
  append_creates_missing_file... PASS
  append_on_unreadable_file_reports_failure_without_truncating... PASS
  solve_path_prints_only_through_the_sink... PASS
  stderr_capture_roundtrip_and_restore... PASS
  stderr_capture_begin_failure_leaves_stderr_alone... PASS
  stderr_capture_end_read_failure_keeps_the_bytes... PASS
  stderr_capture_end_read_error_is_a_failure_not_eof... PASS          ← 新增 (N4)
  16 passed, 0 failed                          (改前 14 passed, 0 failed; +2 = 本次新增的两条)

$ cmd.exe /c "D:\Projects\Touch\Touch_Client\tests\build_payload_calibration_test.bat"
  ... BUILD_EXIT=0
$ Touch_Client\tests\test_payload_calibration.exe
  45 passed, 0 failed                          (与改前一致)
```

`build.bat` 全程没有 `LNK1168` ⇒ `Touch_Client.exe` 没在跑, 也不曾需要结束任何进程。

## 9-6 没碰的东西 (自查)

* 硬要求 1 的两半: `diagEmit` / `diagEmitf` / `diagBegin` / `diagFinish` 一个字没改;
  求解路径上的打印调用一处没改。
* `payloadDSection` 里 `d` 的算式与运算符、`0 < d < 31.5` 判据、`fitOk` 那段"该结论不成立"
  的限定: 没动。
* `appendToFile` 的 `"a+"` + `_O_BINARY`、块/围栏结构: 没动 (改的只是"读失败时删不删").
* `logCalibAttempt` / `logPoseData` / 两份日志格式 / `#if 0` 块 / 阈值 / 模型 / 判决 / `fitRaw`:
  一个字节没动。
* `Docs/superpowers/specs/2026-09-19-raw-channel-calibration-run-001.md` 与本文件 §7 里
  **引用的那两行旧输出**(带 `+`/`−` 的那两行)是**当时那一屏的记录**, 所以**没改** ——
  它们是"改前长什么样"的证据。改后的样子见 §9-2。

## 9-7 仍然没能验证的 (照实说)

1. **N1 没有单测, 也没有实测**。要造一次真的 `fflush(stdout)` 失败 (卷满 / 管道读端没了)
   才能把"毒掉之后静默空操作"这件事演出来 —— 手上造不出一个**确定性的**失败 (`> log.txt`
   加满盘会波及其它东西, 而且这台机器上没有可控的"写满"装置)。所以 N1 的判据只有: 返回值
   恒为 0 (读代码即成立) + `flush` 仍然在调。**"被毒掉之后确实不再 SilentNoop"这条没有跑过。**
2. 与 §6-1 / §8-8 第 1 条相同: **端到端那条验收 (真机跑一次 `'s'`, 比对屏幕与块)仍然没做** ——
   没有设备。
3. N2 的两个约定**哪一个对**, 本次仍然没有解决 (这条不归本次管), 只是把负值那一支也
   摆正了: 现在不管 `c_s_z` 是正是负, 标签旁边的数都是**那个约定的真值**。
4. 注错是**测试侧**的 (`#define _read`, 只在本 TU): 它证明的是 `stderrCaptureEnd` 在
   `_read` 返回 −1 时的行为。**产品里那条 `_read` 真的出 I/O 错误**的场合, 仍然只能靠
   "代码路径与单测一致"来推 —— 与 8-8 第 1 条同一个性质。
5. ⚠ 收尾时 `git stash list` 里留了一条 `stash@{0}`(WIP on feat/pen-clamp-redesign) —— 那是
   本次为做红-绿对比而 `git stash push -- Touch_Client/core/SessionReport.h` 留下的**同一份
   修改的副本**, 内容已经回到工作区 (用 `git show stash@{0}:… > 文件` 落的盘, 所以是 LF,
   没走 git 的 CRLF 转换)。**它是冗余的, 可以安全 `git stash drop`**;

---

# 附: 复审后的两处小修 (F1 / F2) — 2026-09-19

出发点 `126cadc`。**只改两条, 只碰三个文件**: `Touch_Client/main.cpp`、
`Touch_Client/core/SessionReport.h`、`Touch_Client/tests/test_session_report.cpp`。
判决逻辑 / 阈值 / 模型 / `fitRaw` / `payloadDSection` 的算式与判据 / `logCalibAttempt` /
`logPoseData` / 两份日志格式 / `#if 0` 块 / `"a+"` / 块与围栏结构: **一个字节没动**。

## F1 — 块里那句话替 `c_s_z` 假定了符号 (`main.cpp:1024-1034`)

原句: 「反向的约定把 cz_robot − c_s_z 变成 cz_robot + |c_s_z| —— 那是【另一个正数】, 不是变负
(run-001: 68.700 + 55.556 = 124.256 mm)」。它**只在解出的 `c_s_z > 0` 时成立** —— 而 `126cadc`
恰好把 `< 0` 那一支明写进测试 (`test_payload_d_section_negative_cs_prints_no_fabricated_signs`)。
反例: `c_s_z = −100`, `cz_robot = 68.7` ⇒ 约定一 `68.7 − (−100) = 168.7`、
约定二 `68.7 + (−100) = −31.3` —— **负数出现在另一支上**, 与那句话说的正好相反。

改法 (只换这一句, **不加新判决、不加 `✓`/`✗`**): 不再说某一支是多少, 只说两支的**关系**
「约定二 = 约定一 + 2·c_s_z (即 cz_robot − c_s_z 与 cz_robot + c_s_z): c_s_z 为正则约定二偏大,
为负则约定一偏大 —— 谁更大、谁落到零以下, 都随本次解出的符号反过来」。末尾「所以下面 d 的两支
都算、都打」原样保留 (它本来就是真的, 也没变)。

**这一段只进块、不进控制台 —— 这是核实的, 不是假设**: 它拼进 `diagPayloadSection` 的返回值,
该返回值只出现在三处 `diagFinish(diagPayloadSection(...))` (`main.cpp:1091 / 1147 / 1466`),
而 `diagFinish` 把这个字符串**只**交给 `SessionReport::block(...)` 落盘; 屏幕那一半是 `diagEmit`
(`fwrite` + `s_diagBody.append`), 正文里没有这一个字节。⇒ 硬要求 1 的"控制台与正文逐字节同源"
不受影响 (改的只是尾节, 尾节从来不进 `s_diagBody`)。

## F2 — "字节还在临时文件里"这个承诺, 撑不过下一次 `'s'`

`calib_stderr.tmp` 是**固定名字** (`main.cpp:1227`), 而 `stderrCaptureBegin` 从前用
`_O_CREAT | _O_TRUNC` 打开它。于是: 第 1 次 `'s'` 读失败 → 块尾写着"原始字节没有丢, 它们还在
临时文件里: …calib_stderr.tmp", 而那些字节**没进控制台** (窗口里 fd 2 正指着这个文件)、块里也
只落了一句"收不回来" —— 那个文件是**唯一副本**; 操作员最自然的反应 (再按一次 `'s'`) 把它截掉。
**承诺里的那份字节, 被下一次按键销毁。**

改法 (全在 `core/SessionReport.h`):

* 新增纯函数 `captureTmpPathFor(基名, pid, 序号)` = 基名 + `.<pid>_<序号>` + 扩展名, 序号每次
  `Begin` 递增 ⇒ **每次捕获一个专属名字**; 基名没有 `.`(或 `.` 只在目录名里)时后缀接在末尾。
* `stderrCaptureBegin` 改用 **`_O_CREAT | _O_EXCL`** (不再 `_O_TRUNC`): 撞名 (`errno == EEXIST`)
  就**换下一个序号重试**, 试满 64 个仍不行就**什么都不改地**返回 false (调用方照实报"窗口没搭
  起来")。⇒ 已有的残留**在文件系统这一层**就不可能被碰到, 连"pid 被复用"也不怕, 永不截断。
* `st.tmpPath` 从此存**真正建出来那个名字**, 而 `stderrCaptureLeftoverPath()` 报的就是它 ——
  **报给操作员的路径与盘上那个文件必然一致** (main.cpp 那句警告一个字没改, 它取的就是这个值;
  只加了一段说明"这个字符串是基名, 不是最终路径"的注释)。
* 成功路径照旧在 `stderrCaptureEnd` 里 `_unlink` **自己那一个**文件, 不留残骸。

## 验收 (逐条命令与输出)

```
$ cmd.exe /c "D:\Projects\Touch\Touch_Client\build.bat"
  Build OK.                      (无 error, 无 LNK1168 ⇒ Touch_Client.exe 没在跑, 没结束任何进程)

$ cmd.exe /c "D:\Projects\Touch\Touch_Client\tests\build_session_report_test.bat"
  BUILD_EXIT=0
$ Touch_Client\tests\test_session_report.exe
  18 passed, 0 failed             (改前 16 passed, 0 failed; +2 = 本次新增的两条)
    ├ test_stderr_capture_leftover_survives_a_second_begin   ← F2 的行为用例
    └ capture_tmp_path_is_unique_and_recognizable            ← F2 的命名规则 (纯函数)
  测试自己的临时目录收尾后为空 (残留 / 基名 / 派生名都清干净了 —— 见该用例末尾的 _unlink)

$ cmd.exe /c "D:\Projects\Touch\Touch_Client\tests\build_payload_calibration_test.bat"
  BUILD_EXIT=0
$ Touch_Client\tests\test_payload_calibration.exe
  45 passed, 0 failed             (与改前一致)
```

**F2 是怎么验的 —— 红-绿都做了**:

* 绿: 上面那条 `test_stderr_capture_leftover_survives_a_second_begin` 走完整条真实路径 ——
  第 1 次 `Begin`(基名) → 往 stderr 写一段 → 用"换工作目录"把 `End` 里那一句 `_open` 弄失效
  (真的走读失败那条路, 不是旁边断言一句) → 残留留在盘上、内容逐字节比过 → **再用同一个基名**
  `Begin` 第 2 次 → `End` 成功 → 断言残留**仍在、一个字节没少**, 且第 2 次收到的字节里没有第 1
  次的 (没串台), 且成功的这一次不留东西。
* 红 (证明这条用例真的挡得住旧行为): 把 `Begin` 临时改回"固定序号 + `_O_TRUNC`"重建再跑,
  得到 `stderr_capture_leftover_survives_a_second_begin... FAIL: _access(leftover1.c_str(), 0) == 0`
  / `17 passed, 1 failed`。旧行为比"截成 0 字节"还狠: 名字相同 ⇒ 第 2 次 `End` 成功时按自己的
  名字 `_unlink`, 把第 1 次的**唯一副本整个删掉**。改回后重跑 = `18 passed, 0 failed`。
  (红检用的是临时改写 + `cp` 备份还原, **没有用 `git stash`**, 所以没有给后续留下 stash 副本。)
* 另有两条既有用例跟着改了两行断言 (`stderrCaptureEnd` 读失败的两条): 原来断言
  `leftover == 基名`, 现在断言 `leftover` 以基名为前缀、**比基名长**、且**基名本身从来没被建过**
  (`_access(基名) != 0`) —— 这两条同时也是 F2 的回归闸门 (固定名字一旦回来就会红)。

## 没能验证的地方 (照实说)

1. **端到端那条仍然没做**: 没有设备, 不能真跑一次 `'s'` 去比对"屏幕那一屏"与"`calib_report.md`
   里那一块"逐字节相同 (与 §8-8 第 1 条、§9-7 第 2 条同一个缺口)。F1 的结论"这一段只进块"是
   读代码 + 结构核实的: `diagPayloadSection` 的返回值只有 `diagFinish` 一个去处, 而 `diagFinish`
   只把它交给 `appendToFile`。
2. F1 那句新文字的**算术**只在纸上与代码上核过 (约定二 − 约定一 = 2·c_s_z, 代 `c_s_z = ±100`
   两种符号各推一遍); 块里的这一句**没有单测**钉住 (它是 main.cpp 里的字面量, 而这个测试不能
   把 main.cpp 编进来 —— 与既有的"正文格式"用例同一个性质: 那句正文本身靠 `diagEmit` 的结构
   保证同源, 句子内容只有实机那次能看见)。
3. F2 的 `_O_EXCL` 重试: 单测走到的是"名字不同 ⇒ 不撞名"这条主路, **64 次全撞满**那条路
   (要盘上先摆 64 个同名文件) 没有单测, 也没有实测 —— 代码里它只是返回 false (什么都不改)。
4. ⚠ **本次没改、但同类的一处还在** (留给后续判): `core/SessionReport.h:106-108` 的
   `payloadDSection` 输出文本里也有一句「反向的约定给的是另一个数、**不是负数**」, 它同样只在
   `c_s_z > 0` 时成立 (`c_s_z = −100`、`cz_robot = 68.7` 时反向那一支 = `−31.3`)。它**不在
   F1 指的那两行里**, 而这一段是被 `126cadc` 连同它的两个用例一起定过的 (用例里断言了
   `find("变负") == npos`、`find("✓") == npos` 等), 单独改它属于**扩大本次范围**, 所以没动 ——
   但它会出现在同一个块的尾节里, 建议下一轮连它的用例一起收拾。

---

# 追加轮 (2026-09-19, HEAD `a3724e3` 之后) —— 上一轮 §「没能验证的地方」第 4 条的收尾

上一轮的报告末尾自己记着一条**同类未改项**: `core/SessionReport.h` 里那句
「反向的约定给的是另一个数、**不是负数**」。本轮就是把它连同**所有同类副本**一起收拾掉 ——
方法上刻意**不照抄上一轮给的行号**, 而是**按【主张】重新全库搜一遍** (这正是上一轮回
「只改了一处、没搜其它副本」的教训)。

## 1. 全文搜索 —— 命中清单 (按主张搜, 不按字符串搜)

主张 = 「反向的那个约定给的是另一个**正数** / **不会变负**」(即: 两支里反向那支必为正)。

搜法与词表: 全库 (含 `.cpp/.h/.hpp`、`tests/`、`Docs/`) 依次搜
`反向`、`另一个正数 / 另一个数 / 不是负数 / 变负 / 不是负`、`正数 / 为正 / 都是正`、
`变号 / 整体变号 / 偏大 / 取负 / 另一支`、`落到零以下 / 低于零 / 124.256 / 13.144`、
`约定一 / 约定二 / payloadDSection / 两种符号约定 / 成对读`、以及正则
`反向[^\n]{0,40}(正|负)|(正|负)[^\n]{0,30}反向`。逐条判读如下。

| # | 位置 | 形态 | 真的吗 | 处置 |
|---|---|---|---|---|
| 1 | `Touch_Client/core/SessionReport.h:107` | **输出文本** (经 `payloadDSection` → 块尾, **会落盘**) | ✗ **一般化为假** —— 只在 `c_s_z > 0` 时真 (`c_s_z = −100`、`cz_robot = 68.7` ⇒ 反向那支 = `−31.3`) | **改** |
| 2 | `Touch_Client/core/SessionReport.h:79-80` | 注释 (同一主张的"一般性"表述) | ✗ **一般化为假** —— 所引的 `cz_robot = +68.7, c_s_z = +55.556` 恰好让它是正的, 但文字读起来是通例 (上一轮把这句话**搬进了输出**) | **改** |
| 3 | `Touch_Client/main.cpp:1024-1028` | 注释 | ✓ **已经是对的** —— 它只是**引用**那句话并**限定**"只在 `c_s_z > 0` 时成立", 反面例子就在紧邻的下一行 | **不动** (上一轮 a3724e3 已修) |
| 4 | `Touch_Client/tests/test_session_report.cpp:194-195` | 注释 (用例头) | ✗ **一般化为假** —— 把"反向给的是另一个正数"当作对旧错的**更正**来陈述 | **改** (限定到本例) |
| 5 | `Touch_Client/tests/test_session_report.cpp:219-222` | 注释 + 断言 | 注释 ✗ 一般化为假; **断言本身 ✓** (`CHECK(czRobot + csZ > 0.0)` 断言的是 run-001 那一组数的**事实**) | **改注释, 留断言** (断言注释加"仅本例") |
| 6 | `Docs/superpowers/specs/2026-09-19-on-machine-checklist.md:121-127` | 文档 | ✓ **不是那句主张** —— 只给 run-001 的**实测**两个数及它们各自的 ✓/✗, 并明写"这一条要按两种符号约定各算一次" | **不动** (报备) |
| 7 | `Docs/superpowers/specs/2026-09-19-raw-channel-calibration-design.md:411-423` | 文档 | ✓ 同上 ("实测差着十倍", 例子级 + 判据表) | **不动** (报备) |

**结论: 主张的副本共 5 处 (全在 `.cpp/.h` 里), 其中 4 处为假、全部改掉; 1 处 (main.cpp) 上一轮已修。**
另有两处 `Docs/` 命中是**例子级**的实测记录, 不含那条一般化断言, 故不改。
其余 `反向` 命中 (`safety/EscalationTracker.h`、`relay/RelayCore.cpp`、
`calibration/TcpCalibration.h`、`force/PayloadCalibration.cpp`、`force/InertiaIdentification.h`、
`Hardware/tools/compute_payload.py` 等) 与 `d` 的符号无关, 未列。

## 2. 改了什么

**`core/SessionReport.h:79-82` (注释)** —— 改成只说**关系**:
> 反向的约定【不是】"把 d 整体变负"—— 两支差 `2·c_s_z`, 谁偏大、谁落到零以下都随 `c_s_z`
> 的符号翻。这里引的例子 (…) 算出 `68.700 + 55.556 = 124.256 mm`, 恰好仍是正数; 换成
> `c_s_z = −100` 就是 `68.700 + (−100) = −31.3 mm` —— 负数照样会出现在这一支上。

**`core/SessionReport.h:108-112` (输出文本, 这一处会落盘)** —— 原句
「…逐字不变,\n 反向的约定给的是另一个数、不是负数。所以这里【不】给单一的勾/叉…」
改为:
> (c_s 的 z 向【符号】不由数据决定: 模型在 `g → −g, A → −A, c_s → −c_s` 下逐字不变。
> 两支差 `2·c_s_z`, 即【约定二 = 约定一 + `2·c_s_z`】: `c_s_z` 为正时约定二偏大、为负时
> 约定一偏大, 谁落到零以下也跟着翻 (负的 `c_s_z` 下落到零以下的正是【反向】这一支)。
> 所以这里【不】给单一的勾/叉 —— 整节一个勾都不出现, …【通过】去。)

与 `main.cpp` 那段说的是**同一个关系**(约定二 = 约定一 + `2·c_s_z`)。**没有新判决、没有
引入任何 ✓/✗**, 也没有动两个 `d` 的算式 / 判据 / 阈值 / 标签 / 结论行 —— 改的只是
"这句解释在说什么"。

**`tests/test_session_report.cpp:194-197` / `:219-226` (注释)** —— 把"反向给的是另一个正数"
限定到 `c_s_z > 0` 的本例, 并写明"一般化的说法只在 `c_s_z > 0` 时真"。

## 3. 上一轮那个「哪个测试钉住了它」的问题 —— 答案: **没有测试钉住那句话**

上一轮的 §「没能验证的地方」第 2 条说的是 **main.cpp 里那句字面量**没有单测。
本轮这一处在 `payloadDSection` 里, **同样没有**: 最接近的是用例
`payload_d_section_prints_both_sign_conventions` 的 `CHECK(s.find("变负") == npos)` ——
但**旧句子里根本没有"变负"这两个字**(它写的是"不是负数"), 所以那条断言在**改前改后都绿**。
也就是说: 上一轮"只改一处"能溜过去, 一半的原因就在这里。

因此**把这个用例补成真正的回归闸门** (改后新增 3 条断言):
```cpp
CHECK(s.find("另一个正数") == std::string::npos);
CHECK(s.find("不是负数")   == std::string::npos);   // ← 旧句子会被它当场抓住
CHECK(s.find("2·c_s_z")    != std::string::npos);   // ← 关系在 (可搬走的真话)
```
原有的**性质**照旧全在: 两支的 `%.17g` 数值都在、`【在范围内】`/`【在范围外】`/`【相反】` 都在、
`✓`/`✗` 一个都没有、`本块【不给单一的勾】`/`成对读` 都在。

**红检 (照旧实测过, 不是声称)**: 把输出那句**临时改回旧文**再重编重跑 ⇒
`17 passed, 1 failed`, 红的正是
`payload_d_section_prints_both_sign_conventions... FAIL: s.find("不是负数") == std::string::npos`;
改回后重编重跑 ⇒ `18 passed, 0 failed`。负 `c_s_z` 的那条用例
(`..._negative_cs_prints_no_fabricated_signs`) 拿的是**同一个** `payloadDSection`, 所以新断言
对两支符号都成立 (它自己的 `+-`/`−-`/两个真值断言也照旧全绿)。

## 4. 硬要求 1 (控制台字节不变) —— 针对本次改动**重新核实过**, 不是照抄

* `payloadDSection(...)` 的返回值**只有三个去处**, 且三处都是 `diagFinish( diagPayloadSection(...) )`:
  `main.cpp:1097` (姿态数不足)、`:1153` (没有重复对)、`:1477` (正常收尾) —— 全库 grep `payloadDSection` 无第四处。
* `diagFinish` (`main.cpp:951-960`) 对 `trailer` 只做一件事: 交给
  `SessionReport::block(s_diagTs, …, s_diagBody, trailer)`; 而 `block` 的返回值只进
  `appendToFile` (失败时多打一行 `[BIAS] ⚠ 本次报告没落盘`, 与本次改动无关)。
* `block` (`core/SessionReport.h:57-67`) 把 `trailer` **接在收尾的 ``` 之后** ⇒ 它**不在**
  ```` ```text ```` 正文里。正文是 `s_diagBody`, 也就是 `diagEmit` 逐字节同时交给 stdout 的那一份。
* 屏幕那一半 (`diagEmit`/`diagEmitf`/`diagOut`) **从不接触** `payloadDSection` 的返回 —— 上面
  那三处调用点没有一处把它喂给 sink。
* 结构性回归闸门 `solve_path_prints_only_through_the_sink` (它断言 `solveAndApply` 体内没有
  `std::cout/std::cerr/fprintf/fputs/fwrite/puts/putchar`、且 `printf(` 前面必须是字母) 照旧绿。

⇒ 本次改动**只影响块尾 `### 机械臂自报负载` 那一节的解释文字**, 控制台上一个字节没变。

## 5. 验收命令与输出

* `cmd.exe //c "D:\Projects\Touch\Touch_Client\build.bat"` ⇒ `Build OK.`
  (无 `LNK1168`, 全程**没有结束任何进程**; 只有既有的 MSB8004 / C4005 警告)
* `tests\build_session_report_test.bat` (BUILD_EXIT=0) + `tests\test_session_report.exe`
  ⇒ **`18 passed, 0 failed`** (条数不变 —— 新增的是用例内断言, 不是新用例)
  * 红检 (旧句子) ⇒ `17 passed, 1 failed`, 见 §3
* `tests\build_payload_calibration_test.bat` (BUILD_EXIT=0) + `tests\test_payload_calibration.exe`
  ⇒ **`45 passed, 0 failed`**

## 6. 没能验证的地方 (照实说)

1. **端到端仍然没做**: 没有设备, 不能真跑一次 `'s'` 去比对"屏幕那一屏"与 `calib_report.md`
   里那一块 (与上一轮、与 brief §验收同一个缺口)。§4 的结论是**读代码 + grep 全部调用点**
   核实的, 不是实机比对。
2. 新句子的**算术**只在纸上与代码上核过: 约定二 = 约定一 + `2·c_s_z`, 代 `c_s_z = ±100`
   两种符号各推一遍 (`c_s_z = −100` ⇒ 约定一 `168.7` / 约定二 `−31.3`); 这条关系块里
   `main.cpp:1032` 已在说同一件事, **两处文字有重复** (没合并: 一处是 main.cpp 的字面量、
   一处是纯函数的输出, 合并会动到其中一处的结构)。
3. `Docs/` 里那两处**例子级**命中 (上表 #6/#7) 保留原样: 它们给的是 run-001 的**实测**两个数
   各自的 ✓/✗, 不是那条一般化断言 —— 我判断它们不需要改, 但这是我**判读**的, 没有第三方确认。
