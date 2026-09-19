# 复审遗留修复报告 —— 运行时一致性闸门 (Important 1 + Minor a/b/c/d)

基线: `1807abe`。范围: 只改注释与漂移检查的等待时钟; 闸门判据、容差、投票掩码、力矩门、
求解器、模型形式、`A_F`、`logCalibAttempt` / `logPoseData`、两个日志的列、`#if 0` 块、
冻结 fixture、`calib_poses.txt`、`calib_report.md` —— 一字未动。

---

## Important 1 —— 漂移检查承诺的兜底是死代码 (已修, 采用「独立时钟」方案)

**问题 (核实无误)**: `main.cpp` 里 `g_zeroCheckStartMs` 既当累计窗口起点、又当等待截止起点,
而拒绝分支每次都把它重开:

* `main.cpp:1888` `if (fd.isStale || now - g_zeroCheckStartMs < 2000) return;` —— 重开之后
  接下来 ~2 s 的帧都在这里返回, 所以拒绝分支大约每 2 s 才进一次 (与复审说的"elapsed ≈ 2 s"一致);
* 进到分支后 (`main.cpp:1898`) 用 `now - g_zeroCheckStartMs >= 60000` 判截止 —— 而那正是
  `1894` 那一版刚被重置的时钟 ⇒ **截止永远到不了**, 闸门一直拒绝时整段一行都不打印。

**修法 (选项 A: 独立时钟, 只设一次)**:

| 位置 | 改动 |
|---|---|
| `main.cpp:1850` | 新增 `static bool g_zeroCheckRefusalSeen = false;` |
| `main.cpp:1851` | 新增 `static DWORD g_zeroCheckFirstRefusalMs = 0;` —— 第一次被拒的时刻 |
| `main.cpp:1894-1897` | 只在 `!g_zeroCheckRefusalSeen` 时记一次 `g_zeroCheckFirstRefusalMs = now` |
| `main.cpp:1898` | 截止改用 `now - g_zeroCheckFirstRefusalMs >= ZERO_CHECK_GUARD_WAIT_MS` |
| `main.cpp:1916` | 重开窗口只动 `g_zeroCheckStartMs`, **不动**等待时钟 (加了注释说明) |

`g_zeroCheckFirstRefusalMs` 全文只在 `1894-1896` 被写一次 (已 `grep` 全文件确认), 因此从第一次
被拒起算满 60 s 仍不放行 ⇒ 打印【未做】并 `g_zeroCheckDone = true` 定稿。**行为等价于原注释
所承诺的那件事**。

**为什么选 A 而不是 B (删掉截止、无限重试)**: 选项 B 更"贴心"(修好负载参数后不必重启),
但它把"漂移检查没做过"这件事变成永久安静的 —— 而闸门自己的 stderr 是另一条通道, 漂移检查
在 stdout 上一句话都不说。本项目反复栽在"安静地不做/安静地错"上, 所以这里保留一个明确说法。
取舍已写进 `main.cpp:1867-1868` 的注释, 免得下一个维护者以为是漏写。

**措辞**: 删掉「通常是负载参数没发进机械臂」这个猜测 (真实的零偏漂移大到超出容差同样会让
本地模型与 @576 对不上, 而那句话会把操作员支到"去查下发"这一个动作上)。新文案
(`main.cpp:1900-1911`) 只陈述可核验的事实: 闸门拒绝时 `compensated`/`filtered` 是全 0、
'对不上'不止一种来源、逐通道表(有模型但不一致时才有)写着哪些通道超限、放行后重启即可。
第一行还把实际等待秒数打出来 (实测 ~62 s 触发: 2 s 稳定门 + 60 s 等待)。

**注释** (`main.cpp:1855-1869`) 已重写为描述**代码实际做的事**: 明确写出"两个时钟"、
"旧版拿窗口起点当等待起点 ⇒ 永远到不了 ⇒ 一行都不打印"、以及 A/B 两条路的取舍。

## Minor a —— 头文件声称的"编译期保证"不存在 (已修)

`ForceCompensation.h:100-109` 原说"加新 `GuardState` 忘了配错误码会在编译期炸掉 (C4715)"。
已改为与 `.cpp` 一致的表述。

**顺手核实出了一个新事实 (两边都错)**: 原 `.cpp:481-485` 说漏配时 **/W4 会给 C4062**。
`ForceCompensation.cpp:479-484` 与 `.h:101-105` 现在都写实测结果。

**实测 (临时探针 `tests/_w4062.cpp` + `_w4062.bat`, 跑完已删, 未提交)**: 一个 `enum class`
三个枚举值、switch 里只写两个 case、末尾兜底 `return 0;`:

```
===== /W1 =====   (无 C4062 / 无 C4715)
===== /W3 =====   (无)
===== /W4 =====   (无)
===== /Wall ===== _w4062.cpp(9): warning C4062: 未处理枚举 "E" 的开关中的枚举器 "E::C"
```

⇒ C4062 **默认关闭**, `/W1 /W3 /W4` 都不报, 只有 `/Wall` 报; 而 C4715 在**任何**级别都不出现
(末尾兜底 `return` 让缺失返回路径不存在)。即: 编译器**完全不会**替维护者发现漏配, 钉住这张表的
是 `test_force_compensation` 的 `guard_error_code_mapping` (`tests/test_force_compensation.cpp:310`,
三条映射逐条断言 + 与 `errorCodeName` 对上, 已确认存在)。

## Minor b —— "约束力也被切断"的旧说法 (已修, 含一处复审未点名的同源注释)

`RelayCore.cpp:1694-1698`: 原文「(haptic / 约束力 / F| 一起断)」已改为
「断开的是【传感器力那一条路】(filtered / hapticOut / F|); ⚠【虚拟约束力不断】——
HapticCallback.cpp:168 由位置现算」。
出处核实: `HapticCallback.cpp:168` `SafetyPredictor::instance().computeConstraintForce(forceRef, constraint)`,
其输入是 `robotPos` / `robotActualPose` (位置), 与 `compensated` 无关; 8c/8d 两条
(orientExtraForce / orientRepulsionForce) 同样是位置驱动 (`HapticCallback.cpp:175-192`)。

**⚠ 超出复审清单的一处**: 复审说这条说法"已在 ForceCompensation.cpp/.h 改过"—— `.h:68-72` 确实
改过了, 但 `.cpp:610` (不一致分支的注释) **还原样留着**「haptic / 约束力 / F| 随之断开」。
那正是本轮要消灭的那类注释 (描述了一个不存在的行为), 所以一并改掉
(`ForceCompensation.cpp:610-612`)。若认为越界, 可单独回退这两行, 与本轮其余改动无耦合。

## Minor c —— `GUARD_MIN_DET_RATIO` 的措辞 (核实: **已在本分支修过, 无需再动**)

`ForceCompensation.cpp:441-447` 现文案为「接近完全秩亏 …… 该比值对'标量质量 x 正交'恒为
0.19245, 本门限约等于条件数 354」, 并带一行"别再说'至少一个力方向没有模型'"。`git blame`
显示 `441-443 / 446-447` 出自 `1807abe` ⇒ 已是复审要求的措辞, 未改动。

## Minor d —— `setCalibration` 只校验 `A` 的有限性 (核实: **已在本分支修过, 无需再动**)

`ForceCompensation.cpp:329-350` 已对 `biasForce` / `biasTorque` / `comSensor` 三个数组逐个
`std::isfinite` 校验, 再走共用的 `modelUsable(A,...)`; 与 `ForceCalibration.cpp:498-514`
(装载路径对同样三个数组逐个判) 对称。`git blame` 显示这一段出自 `1807abe` ⇒ 未改动。

---

## 构建与测试

构建: `Touch_Client/build.bat` ⇒ `Build OK.` + `Build complete.`
(命令里必须临时清掉 `NoDefaultCurrentDirectoryInExePath`, 否则 cmd 拒绝执行当前目录下的 .bat;
用 bash 的 `env -u NoDefaultCurrentDirectoryInExePath cmd //c build.bat`, 不涉及反斜杠。)

| 套件 | 命令 | 结果 |
|---|---|---|
| test_force_compensation | `tests/build_force_comp_test.bat` + 运行 | **23 passed, 0 failed** |
| test_payload_calibration | `tests/build_payload_calibration_test.bat` + 运行 | **49 passed, 0 failed** |
| test_session_report | `tests/build_session_report_test.bat` + 运行 | **20 passed, 0 failed** |
| run_tests.bat (全套) | `tests/run_tests.bat` | 11 个 exe 全绿: 5/7/8/28/15/18/27/**23**/11/4/7 passed, 0 failed |

`test_safety_core` 已知间歇性失败在本次复现过 **1 次**: 第一次跑 `run_tests.bat` 时
`Results: 7 passed, 1 failed` (`FAIL: fabs(sm.speedFactor() - 0.3) < 0.01`); 单独连跑 3 次
均 `8 passed, 0 failed`, 之后整轮 `run_tests.bat` 也是 8/0。与本次改动无关 (未碰 SafetyCore)。

## 未能验证的部分 (照实说)

* **【未做】那条分支没有跑起来过**: `runZeroDriftCheck` 是 `main.cpp` 里的 static 函数, 且必须
  真机 + 闸门持续拒绝才能触发; 本环境无机械臂, 也不允许启停进程。只能静态核对:
  等待时钟全文只在 `main.cpp:1894-1896` 写一次, 不存在别处重置; 触发时刻 = 启动后约 62 s。
  **实机复验方法**: 保持当前"闸门每帧拒绝"的状态启动, 60 s 后 stdout 应出现
  `[Force] 零偏漂移检查: 【未做】—— 一致性闸门从第一次拒绝起已 60 s 一直在拒绝`。
* C4062 的探针是**等价最小复现**, 不是本项目文件的直接编译; 结论 (默认关闭、只有 /Wall 报)
  是编译器行为, 与具体 TU 无关, 但严格说未在 `ForceCompensation.cpp` 本体上验证过。
* 未跑实机, 未验证闸门在真机上的放行/拒绝时序 —— 本轮没有碰判据, 不需要。

## 未纳入本次提交的既有改动 (提交前就在工作区, 与本任务无关)

` M .superpowers/sdd/task-9-report.md`, ` M Docs/superpowers/plans/2026-09-18-calib-lifecycle.md`,
以及未跟踪的 `Hardware/stl.zip` / `Touch_Client/alarms.log` / `Touch_Client/calib/` /
`Touch_Client/force_demo_log.csv` / `Touch_Client/robot_diagnostics.log` /
`Touch_Client/tests/_build_sa.bat` / `_build_sa.log` / `_hello.cpp` / `_rebuild_and_test.bat` /
`calib/`。本次提交只含下面四个源文件 + 本报告。
