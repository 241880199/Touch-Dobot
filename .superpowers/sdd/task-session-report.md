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
