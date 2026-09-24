# 2026-09-24 的分析脚本（报告里那些数是怎么算出来的）

> 报告 = `../2026-09-24-jitter-report.md` 与记忆 `2026-09-24-button2-diagnosis`。
> 这些脚本**硬编码了日志路径**（今晚的两份原始日志见下表），直接 `python <脚本>` 即可复算。
> 输出文件 `_*_out.txt` 是当时 capture 的 stdout —— **报告里的数可以直接对上去**。

## 原始数据（脚本读的就是这些）

| 日志 | 在哪 |
|---|---|
| MATLAB 旁路日志（`P\|`/`J\|`/`RP\|`/`ServoJ`） | `Relay_Station/_matlab_session_20260923.log` · `_20260924.log` |
| 力日志（`filtered` + pose + acc） | `Touch_Client/force_demo_log.csv`（09-23）· `Touch_Client/x64/Release/force_demo_log.csv`（09-24） |

## 脚本清单

| 脚本 | 算什么 | 报告/记忆里对应的数 |
|---|---|---|
| `_analyze_btn2_log.py` | 第一版：从日志挖 `P\|`/`J\|` 序列找运动段 | （被下一个取代，留作过程） |
| `_analyze_btn2_bursts.py` | `ServoJ` 连续段 + 配对 `P\|`/`J\|`/`RP\|`/命令增量 | 4 段 ServoJ（23:09:00…23:11:58）与逐段差 |
| `_analyze_btn2_purity.py` | 逐步增量的旋转在**器件系**的轴 | "自转 = 轴稳定在器件 Z，\|Z\|≈0.95" |
| `_analyze_btn2_axes.py` | 按主导轴切时间线 | 三动作各一根轴（Z/X/Y 的身份） |
| `_analyze_btn2_beforeafter.py` | 现行 vs 候选映射的**串扰比** | **3.75 → 0.85**（自转）、1.22→0.80、0.78→0.91 |
| `_analyze_stylus_range.py` | 笔杆角度取值域 + 跨接缝计数 | Rx[−84,60] · Ry[−70,61] · Rz[−171,109]；\|Rz\|>170 仅 4 条 |
| `_analyze_axes_principal.py` | 段内**加权求和**求主轴 | ⚠ **这个估计器是坏的**（来回拧会让轴向分量自相抵消）⇒ 结论已被下一个取代 |
| `_analyze_force_during_joint.py` | 落盘力按静/动分组 | 09-23：静止 \|F\| 0.33 / 运动 0.33 max 2.03 |
| `_align_force_to_wallclock.py` | 首次对齐 `t_ms` → 墙钟 | ⚠ **失败**（姿势歧义）—— 留作过程，被下一个取代 |
| `_align2.py` | 序列对齐（滑动偏移 + 指纹评分） | 中位位姿偏差 **0.01 mm**，覆盖 23:11:14–23:22:29 |
| `_analyze_inertia_residual.py` | `filtered` 对 `R^T·acc` 最小二乘 | **R² 全负** ⇒ 排除惯量项（\|acc\|≈0.1 ⇒ ~0.07 N） |
| `_analyze_gravity_lag.py` | 每步 `\|ΔF g\|` vs 实测 `\|F\|` | **corr = +0.727**；0.591/0.591 · 1.041/1.046 · 0.670/0.691 · 1.555/1.656 ⇒ ③ 的根因 |
| `_analyze_20260924.py` | 今晚日志分段 + 段内轴 + 命令增量 | 今晚只有 1 段 58.9 s（1259 帧） |
| `_analyze_20260924_axes.py` | **角速度法**（0.3 s 基线）求轴 | 今晚的轴三元组：X 偏名义 5° · Y 10° · Z 6~12° |
| `_analyze_20260924_signs.py` | ΔR 旋转向量 vs 命令增量 | 参照姿态 `(−58.93,11.83,−6.41)°` 下的逐段对照 |
| `_analyze_20260924_sensitivity.py` | **灵敏度矩阵**（绕器件轴 10° ⇒ 各关节几度） | "自转同时喂三关节（合计 1.55）"那张表；也是"X→J4 本来就是对的"的来源 |

## ⚠ 两个别乱跑的

- **`_patch_tests.py`** —— 它**已经把补丁打进 `tests/test_button2_joint.cpp` 了**（一次性）。
  **再跑一次会二次套用/失败**；它是留档（说明当时怎么改的），不是工具。
- `_analyze_axes_principal.py` 与 `_align_force_to_wallclock.py` 是**被取代的失败版本**，
  **它们的结论不要引用**（各自被下一个脚本否掉）。
