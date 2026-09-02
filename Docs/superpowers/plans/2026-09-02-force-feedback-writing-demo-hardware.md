# 力反馈书写对照演示（硬件 + 实机联调）实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 完成结项演示所需的**笔夹 3D 打印建模**（硬件，可离线做）与**全部实机联调**（TCP 偏移实机标定、力反馈验证、A/B 对照演示彩排）。

**Architecture:** 分两部分。**Part A（夹具）** 用 OpenSCAD 参数化建模笔夹（可调内径 V 型槽 + 侧向螺丝 + 短套筒滑动定位），离线渲染导出 STL，打印前做尺寸审查。**Part B（实机联调）** 依赖设备与夹具：回装设备 → 装夹具/传感器/笔 → 复验坐标标定 → 力反馈最小验证（先写几笔看 A/B 差异）→ TCP 偏移实机标定并应用 → 完整 A/B 演示彩排。

**Tech Stack:** OpenSCAD（参数化建模）、3D 打印（STL）、既有 C++ 系统（Touch_Client）+ MATLAB relay_gui + CR3 机械臂。

## Global Constraints

- 夹具必须安装在**力传感器（KWR75B）工具端法兰（下方）**，而非机械臂法兰中心——否则笔尖接触力不经过传感器、无法被测到（spec §3.1）。
- 笔杆直径（~6-14mm）与长度**待实测** → 夹持必须**可调内径 + 笔杆滑动定位**（spec §3.2）。
- 力反馈开关只在 MATLAB 界面，无键盘快捷键（防误触）。
- 落笔/抬笔由 Touch Z 轴自然控制，不设自动抬 Z（spec §2.4.5）。
- 本计划的任务依赖实机、依赖前一个软件计划（`2026-09-02-force-feedback-writing-demo.md`）的 Task 1-8 已落地。执行前确认该软件计划已合并到 `master`。

---

## File Structure

| 文件 | 责任 | 动作 |
|------|------|------|
| `Hardware/pen_clamp.scad` | 笔夹 OpenSCAD 参数化模型 | 新建 |
| `Hardware/pen_clamp.stl` | 导出打印件 | 生成（不入库亦可，视仓库约定） |
| `Hardware/README.md` | 待实测参数记录 + 打印/装配说明 | 新建 |
| （既有）`Touch_Client/relay/RelayCore.cpp` | `pollForce` 落盘处（Task 6 应用偏移） | 修改（实机验证后） |
| （既有）`Touch_Client/calibration/TcpCalibration.*` | TCP 偏移求解（软件计划 Task 7/8 已建） | 读取 |

---

## Part A — 笔夹夹具（离线可做）

### Task 1: OpenSCAD 笔夹参数化建模

**Files:**
- Create: `Hardware/pen_clamp.scad`
- Create: `Hardware/README.md`

**说明:** 上端配合力传感器工具端法兰（4×M6 + Φ6 销），下端可调内径夹持笔杆。**待实测参数**（`BC_D`/`DOWEL_R`/`DOWEL_ANG`）必须从 KWR75B 尺寸图或实物测量填入后才能打印。

- [ ] **Step 1: 写 OpenSCAD 脚本**

创建 `Hardware/pen_clamp.scad`：

```openscad
// 笔夹 — Touch-Dobot 书写演示
// 安装: 力传感器(KWR75B) 工具端法兰下方 (4×M6 + Φ6 销通用接口)
// 夹持: V 型槽 + 侧向螺丝顶紧 (可调内径 6~14mm), 笔杆滑动定位 (短套筒)

/* ===== 待实测参数 (从 KWR75B 尺寸图/实物测量填入后再打印) ===== */
BC_D      = 50;   // 工具端法兰 4×M6 孔分布圆直径 (mm)
DOWEL_R    = 12;  // Φ6 定位销孔距中心径向距离 (mm)
DOWEL_ANG  = 45;  // Φ6 销孔相对 M6 孔的角度 (deg)
/* ============================================================= */

/* ===== 已定参数 ===== */
FLANGE_T     = 8;     // 上端法兰板厚 (mm)
M6_CLEAR     = 6.5;   // M6 螺钉过孔直径 (mm)
DOWEL_D      = 6.0;   // Φ6 定位销配合孔直径 (mm)
SLEEVE_LEN   = 35;    // 夹持段长度 (mm) — 短套筒, 笔杆可滑动
PEN_MAX_D    = 14;    // 最大笔杆直径 (mm)
CLAMP_SCREW_D = 4.0;  // 侧向顶紧螺丝直径 (M4) — 攻丝或嵌铜螺母
WALL         = 4;     // 壁厚 (mm)
FLANGE_D     = BC_D + 18;

$fn = 128;

// ===== 上端法兰板 (接传感器工具端) =====
module top_flange() {
    difference() {
        cylinder(d = FLANGE_D, h = FLANGE_T);
        for (a = [0 : 90 : 270])
            rotate([0, 0, a]) translate([BC_D/2, 0, -1])
                cylinder(d = M6_CLEAR, h = FLANGE_T + 2);
        rotate([0, 0, DOWEL_ANG]) translate([DOWEL_R, 0, -1])
            cylinder(d = DOWEL_D, h = FLANGE_T + 2);
    }
}

// ===== 夹持主体: V 型槽 + 侧向顶紧螺丝 =====
module clamp_body() {
    body_d = PEN_MAX_D + 2*WALL + CLAMP_SCREW_D + 4;
    difference() {
        union() {
            // 主体圆柱 (自法兰板下沿向下延伸)
            translate([0, 0, -SLEEVE_LEN])
                cylinder(d = body_d, h = SLEEVE_LEN);
        }
        // 中心笔杆通孔 (略大于最大笔径, 靠 V 槽 + 螺丝夹紧)
        translate([0, 0, -SLEEVE_LEN - 1])
            cylinder(d = PEN_MAX_D + 0.5, h = SLEEVE_LEN + 2);
        // V 型槽: 在 +X 侧开一个 90° V 口, 让侧向螺丝把笔顶向 V 槽对侧
        rotate([0, 0, 0])
            translate([PEN_MAX_D/2, 0, -SLEEVE_LEN - 1])
                rotate([0, 90, 0])
                    linear_extrude(height = SLEEVE_LEN + 2)
                        polygon(points = [[0, -body_d/2 - 1], [0, body_d/2 + 1], [-body_d, 0]]);
    }
    // 侧向顶紧螺丝孔 (水平贯穿到笔孔, 与 V 槽对侧)
    rotate([0, 0, 0])
        translate([-(body_d/2 - CLAMP_SCREW_D - 1), 0, -SLEEVE_LEN/2])
            rotate([0, 90, 0])
                cylinder(d = CLAMP_SCREW_D, h = body_d - 2*CLAMP_SCREW_D - 2);
}

// ===== 装配 =====
top_flange();
translate([0, 0, -FLANGE_T]) clamp_body();
```

> 注：`clamp_body()` 的具体 V 槽角度/螺丝位置需结合选定笔杆直径**试打印后微调**。上表是可用起点，非一次性成品。

- [ ] **Step 2: 写 README 记录待测参数**

创建 `Hardware/README.md`：

```markdown
# 笔夹 (pen_clamp) 说明

## 待实测参数 (打印前必填)
| 参数 | 含义 | 当前占位值 | 来源 |
|------|------|-----------|------|
| BC_D | 4×M6 孔分布圆直径 | 50 mm | KWR75B 尺寸图 |
| DOWEL_R | Φ6 销孔径向距离 | 12 mm | KWR75B 尺寸图 |
| DOWEL_ANG | Φ6 销相对 M6 孔角度 | 45° | KWR75B 尺寸图 |

## 装配顺序
1. 法兰板 4×M6 螺钉 + Φ6 销固定到力传感器工具端。
2. 笔杆自下而上穿过套筒，滑动到合适位置（笔尖伸出下方合适长度）。
3. 侧向螺丝顶紧，笔垂直纸面、无晃动。

## 换笔
松开侧向螺丝 → 抽出旧笔 → 插入新笔滑动到同一笔尖位置 → 重新顶紧。
笔尖位置一致则无需重标 TCP 偏移。
```

- [ ] **Step 3: Commit**

```bash
git add Hardware/pen_clamp.scad Hardware/README.md
git commit -m "feat(hardware): add parameterized pen clamp OpenSCAD model"
```

---

### Task 2: 渲染导出 STL + 打印前尺寸审查

**说明:** 无代码改动，交付一份可打印 STL。审查要点是尺寸合理性（能否夹住笔、法兰孔位是否匹配）。

- [ ] **Step 1: 填充待实测参数**

用 KWR75B 尺寸图/实物测量填 `BC_D`/`DOWEL_R`/`DOWEL_ANG`，重新 `render`。

- [ ] **Step 2: 渲染导出 STL**

Run: `openscad -o Hardware/pen_clamp.stl Hardware/pen_clamp.scad`
Expected: 生成 STL，无报错。

- [ ] **Step 3: 尺寸审查**

- 4×M6 过孔分布圆与传感器工具端法兰一致（用打印件或 CAD 测量核对）。
- 中心笔孔能容纳选定笔杆（~6-14mm），侧向螺丝孔与笔孔连通。
- 打印预览检查 V 槽与螺丝顶紧方向正确。

- [ ] **Step 4: 提交 STL（若仓库约定纳入）**

```bash
git add Hardware/pen_clamp.stl
git commit -m "feat(hardware): add rendered pen clamp STL"
```

---

## Part B — 实机联调（依赖设备 + 已完成软件）

### Task 3: 设备回装 + 夹具/传感器/笔安装

- [ ] **Step 1: 机械臂 + 传感器 + 夹具 + 笔 装配**

按 `Hardware/README.md` 装配顺序安装，确认笔尖在传感器正下方、垂直纸面、无晃动。

- [ ] **Step 2: 上电 + 连接自检**

- PC 以太网 `192.168.101.100` ↔ CR3 控制柜 LAN 口（`192.168.101.11`）网线直连。
- 启动 `Relay_Station` MATLAB `relay_gui`；再启动 `Touch_Client\x64\Release\Touch_Client.exe`。
- 确认 MATLAB 界面 `C++ Client: CONNECTED`，C++ 控制台打印 `[Relay] GUI reporting connected`。
- 确认 `enable/motion/force` 三连接正常（MATLAB 连接监视面板三点绿）。

- [ ] **Step 3: 安全自检**

- 按下 Touch 按钮 1 移动机械臂，确认跟随、安全边界钳位正常。
- 确认力传感器 `F|` 数据流（MATLAB 力面板有非零读数）。
- 记录本次实测的环境数据（纸面高度、安全边界是否需要扩缩）。

**验收:** 机械臂可控、力数据可达、MATLAB 可视化正常。

---

### Task 4: Touch→Robot 坐标标定实机复验

- [ ] **Step 1: 重新采集标定点**

按 `main.cpp` 键盘流程：`'c'` 进入坐标采集 → 对准标记点 → `SPACE` 记录 ≥4 点 → `'s'` 求解并保存 `calibration.json`。

- [ ] **Step 2: 验证残差**

Run: 查看控制台 `[CALIB] Solved! RMS error = X mm`
验收: RMS < 5mm（书写精度量级；越大说明 Touch 坐标与机器人坐标对不上，需排查）。

- [ ] **Step 3: 记录结果**

把 RMS、采集点数记入 `Hardware/README.md` 的"实机标定记录"节。

---

### Task 5: 力反馈最小验证（先写几笔看 A/B 差异）

**说明:** 在正式演示前先做最小验证：确认"稳定接触 + 软垫"策略可行、A/B 差异肉眼可见。若差异被噪声淹没，回 spec §6 调整（软垫刚度、接触阈值）。

- [ ] **Step 1: 铺软垫 + 固定纸张**

纸下垫海绵/软垫，把接触刚度降到 1~5 N/mm。

- [ ] **Step 2: A 组试写（力反馈 ON）**

MATLAB 界面开关保持 `ON` → 操作者写简单笔画 → 观察力面板与手感。

- [ ] **Step 3: B 组试写（力反馈 OFF）**

切换 MATLAB 开关到 `OFF` → 同样内容再写 → 对比手感与笔迹。

- [ ] **Step 4: 定性判断**

验收：A 组笔迹更稳、笔压更可控；B 组明显更难（忽轻忽重/易断线）。若差异不明显，按 spec §6 调整软垫或任务。

---

### Task 6: TCP 偏移实机标定 + 应用

**说明:** 使用软件计划 Task 8 已落地的采集/求解框架。先在实机采集求解偏移，再用 FK 验证模式核对 RPY 约定，最后把偏移应用到轨迹记录（`pollForce` 落盘）。

- [ ] **Step 1: 采集 TCP 位姿**

在纸面固定一个尖点参考，笔尖对准 → `'t'` 进入 TCP 采集 → 变换机械臂姿态（笔尖保持不动）→ `SPACE` 记录 ≥4 个法兰位姿 → `'s'` 求解保存 `tcp_calib.json`。

- [ ] **Step 2: 核对求解结果合理性**

验收：`offset` 量级 ~笔长 + 夹持段 + 传感器高度（约 100-200mm 的 -Z 方向为主），`RMS` 残差小（<2mm 理想）。

- [ ] **Step 3: FK 一致性验证 RPY 约定**

用既有 `'v'` FK 验证模式（`main.cpp`）或对比 `Kinematics::computeJointPositions`，确认 `TcpCalibration::rpyToMatrix` 的 `Rz·Ry·Rx` 约定与 `GetPose` 返回的 RPY 一致。**若不一致，修正 `TcpCalibration.cpp` 的 `rpyToMatrix` 约定并重跑 Task 7 测试。**

- [ ] **Step 4: 把偏移应用到落盘轨迹（软件改动，实机验证后提交）**

在 `RelayCore.cpp` 的 `pollForce()` 落盘处，用 `TcpCalibration::apply` 把记录的 `pose` 换成笔尖坐标（当 `TcpCalibration::enabled`）：

```cpp
    // 落盘: 优先记录笔尖世界坐标 (TCP 偏移已标定时)
    double logPose[6];
    if (TcpCalibration::enabled) {
        double tip[3];
        TcpCalibration::apply(pose, TcpCalibration::offset, tip);
        logPose[0] = tip[0]; logPose[1] = tip[1]; logPose[2] = tip[2];
        logPose[3] = pose[3]; logPose[4] = pose[4]; logPose[5] = pose[5]; // 姿态同法兰
    } else {
        for (int i = 0; i < 6; i++) logPose[i] = pose[i];
    }
    ForceLogger::log(now, app.forceData.filtered, logPose,
                     appState.forceFeedbackEnabled ? 1 : 0);
```

（需 `#include "../calibration/TcpCalibration.h"`；原 `ForceLogger::log(..., pose, ...)` 改为 `logPose`。）

- [ ] **Step 5: 编译 + 实机验证**

Run: `cd Touch_Client && build.bat`，重启客户端，确认落盘 CSV 的 `pose_x/y/z` 变为笔尖坐标。

- [ ] **Step 6: Commit**

```bash
git add Touch_Client/relay/RelayCore.cpp
git commit -m "feat(force): log pen-tip pose when TCP offset is calibrated"
```

---

### Task 7: 完整 A/B 结项演示彩排

- [ ] **Step 1: 预检**

- `calibration.json`、`tcp_calib.json`、`force_calib.json` 均已加载（看启动日志）。
- 纸张/软垫就位，手机/相机架好用于拍字迹。

- [ ] **Step 2: A 组（力反馈 ON）**

1. 确认 MATLAB 开关 `ON`。
2. 操作者写 "NJU"，C++ 全程落盘 CSV。
3. 拍照字迹。

- [ ] **Step 3: B 组（力反馈 OFF）**

1. 切换 MATLAB 开关 `OFF`。
2. 同内容再写 "NJU"。
3. 拍照字迹。

- [ ] **Step 4: 数据分析**

Run: `cd Relay_Station && matlab -r "force_analysis('force_demo_log.csv')"`
验收：输出 A/B 两组 `Fz_rms_N`、`Fz_var_N2`、`contact_segments`，A 组稳定性指标优于 B 组；生成并排对比图。

- [ ] **Step 5: 产出结项材料**

- 两张字迹照片并排。
- 力曲线对比图 + 量化指标表（RMS/方差/断线次数）。
- 整理进结项报告。

- [ ] **Step 6: 记录实测数据**

把最终指标、阈值、软垫参数记入 `Hardware/README.md` 的"演示记录"节。

---

## 风险与回退（实机）

| 风险 | 应对 |
|------|------|
| 夹具尺寸不匹配法兰 | 按 `README` 待测参数核对，必要时重打印 |
| A/B 差异不明显 | 软垫降刚度、调 `CONTACT_N` 阈值、换更软笔 |
| TCP 求解 RPY 约定与 GetPose 不符 | 用 FK 验证模式核对，修正 `rpyToMatrix` 并重跑 Task 7 测试 |
| 力反馈开关反向命令未生效 | 检查 MATLAB `write()` 是否发到已连接客户端、C++ `pollRelayCommands` 是否每帧调用 |
