# 笔夹重设计 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把 `Hardware/pen_clamp` 从「V 型块短套筒」重写为「轴向顶紧 + 双光轴可滑导向 + 三档自定心夹头」，满足毛笔笔径 6~16mm、笔长 170~300mm、夹持点可调、笔尾顶传感器直传力。

**Architecture:** 单文件 `Hardware/pen_clamp.scad` 参数化建模，`PART` 数字选择器渲染不同零件；导出各零件 STL 到 `Hardware/stl/`；更新 `Hardware/README.md`。无软件代码改动。

**Tech Stack:** OpenSCAD（参数化建模）、OpenSCAD CLI（`C:\Program Files\OpenSCAD\openscad.exe`）、3D 打印。

## Global Constraints

- 力传递：笔尾端（笔顶）**轴向顶住**传感器 Φ31.5 凸台面；夹头只扶正+防落，**不承书写力**。
- 传感器接口（已确认）：4×M6 @Φ50、2×Φ6 销 @R25/45°、Φ31.5 凸台高 **7mm**。
- 笔夹法兰厚 **6mm**（凸台 7mm 凸出 1mm，笔尾干净顶到）；`FLANGE_D` 扩到 **80mm** 给导向柱耳座留位。
- 笔杆直径 **6~16mm**（夹头三档 6-10 / 10-13 / 13-16）；笔长 **170~300mm**。
- 导向柱用**市售 Φ8 光轴**（标准件，非打印），两根 + 底部横梁成门架。
- 夹头用 **3D 打印件**（不做金属夹头）。
- 每次装笔都「顶到位」，笔尖相对传感器高度随笔长变 → **TCP 偏移需重标**（复用既有 `TcpCalibration`，本计划不改软件）。
- 坐标约定：+Z 朝上（传感器方向），-Z 朝下（纸面方向）；法兰顶面在 z=0，法兰体在 z∈[0, FLANGE_T]，导向柱/滑块/夹头在 z<0。

---

## File Structure

| 文件 | 责任 | 动作 |
|------|------|------|
| `Hardware/pen_clamp.scad` | 参数表 + 全部模块 + `PART` 选择器 | 重写（本计划核心） |
| `Hardware/stl/` | 各零件 STL 导出目录 | 新建 |
| `Hardware/README.md` | 参数/装配/换笔/待测记录 | 重写 |
| （废弃）`Hardware/pen_clamp.stl` | 旧单件 STL | 删除（被 `stl/` 多件替代）|

### 零件清单（打印件 6 + 标准件 5）

| 件 | 打印/采购 | 数量 |
|---|---|---|
| 顶部法兰（含导向柱耳座）| 打印 | 1 |
| 滑动导向块 | 打印 | 1 |
| 底部横梁 | 打印 | 1 |
| 分档夹头（三瓣锥套）| 打印 | 3（孔径 8 / 11.5 / 14.5）|
| 锁紧环（锥面压环）| 打印 | 1 |
| Φ8 光轴 | 采购 | 2（长 250mm）|
| M4×16 内六角（锁紧环）| 采购 | 2 |
| M4×10 内六角（块锁高）| 采购 | 1 |
| M6 螺栓 + Φ6 销（接传感器）| 采购 | 4 + 2（复用现款）|

---

## Task 1: 参数表 + 顶部法兰（含导向柱耳座）

**Files:**
- Create: `Hardware/pen_clamp.scad`（参数表 + `top_flange()` + `PART` 选择器骨架）

**Interfaces:**
- Produces: 全部参数常量；模块 `top_flange()`；`PART` 数值约定（1=assembly 2=flange 3=block 4=tiebar 5=collet_s 6=collet_m 7=collet_l 8=ring）。

- [ ] **Step 1: 写参数表 + `top_flange()` + 件选择骨架**

创建 `Hardware/pen_clamp.scad`：

```openscad
// 笔夹 — Touch-Dobot 书写演示（轴向顶紧 + 双光轴可滑导向 + 三档自定心夹头）
// 安装: 力传感器(KWR75B) 工具端法兰 (4×M6 + 2×Φ6销 + Φ31.5凸台高7mm)
// 力传递: 笔尾顶住 Φ31.5 凸台面 → 书写力沿笔轴直传传感器；夹头只扶正+防落。

/* ===== 传感器接口 (KWR75B, 已确认) ===== */
BC_D      = 50;   // 4×M6 分度圆直径 (mm)
DOWEL_R   = 25;   // 2×Φ6 销孔径向距离 (mm)
DOWEL_ANG = 45;   // 销孔相对 X/Y 轴角度 (deg)
BOSS_D    = 32;   // Φ31.5 凸台过孔直径 (mm), 留 0.5 余量
BOSS_H    = 7;    // 凸台高度 (mm) = 31.5(总高) - 24.5(法兰安装高)
M6_CLEAR  = 6.5;  // M6 螺栓过孔 (mm)
DOWEL_D   = 6.0;  // Φ6 销配合孔 (mm)
FLANGE_T  = 6;    // 法兰板厚 (mm) — 凸台凸出 1mm, 笔尾干净顶到
FLANGE_D  = 80;   // 法兰外径 (mm) — 扩到 80 给导向柱耳座留位

/* ===== 笔适配 ===== */
PEN_MIN_D = 6;    // 最小笔杆直径 (mm)
PEN_MAX_D = 16;   // 最大笔杆直径 (mm)
PEN_MIN_L = 170;  // 最短笔总长 (mm)
PEN_MAX_L = 300;  // 最长笔总长 (mm)

/* ===== 导向柱 (2×Φ8 市售光轴) ===== */
ROD_D    = 8;     // 光轴直径 (mm)
ROD_L    = 250;   // 光轴长度 (mm) — 覆盖夹持点离传感器 ~100~210mm + 余量
ROD_R    = 32;    // 光轴心距中心 (mm) — 避让凸台孔 R16 与 M6/销孔 R25
ROD_ANG  = 22.5;  // 光轴相对 +X 轴角度 (deg) — 落在 M6(0°) 与销(45°) 中间
EAR_LEN  = 16;    // 耳座伸出法兰下表面的长度 (mm)

/* ===== 夹头 (三瓣锥套) ===== */
COLLET_BORE_S = 8;     // 档1 孔径: 适配 6-10mm 笔
COLLET_BORE_M = 11.5;  // 档2 孔径: 适配 10-13mm 笔
COLLET_BORE_L = 14.5;  // 档3 孔径: 适配 13-16mm 笔
COLLET_HEAD_D    = 24;   // 夹头头段外径 (mm)
COLLET_HEAD_L    = 8;    // 夹头头段长度 (mm)
COLLET_FLANGE_D  = 28;   // 夹头顶部挡圈外径 (mm)
COLLET_FINGER_L  = 16;   // 三瓣工作段长度 (mm)
COLLET_FINGER_TOP= 22;   // 三瓣顶端外径 (mm)
COLLET_FINGER_BOT= 25;   // 三瓣底端外径 (mm) — 下宽上窄, 压环上移即收紧
FINGERS = 3;             // 瓣数

/* ===== 锁紧环 (锥面压环) ===== */
RING_OD      = 32;   // 环外径 (mm)
RING_L       = 12;   // 环高 (mm)
RING_ID_TOP  = 22.5; // 环内锥上端 (小) 直径 (mm)
RING_ID_BOT  = 25.5; // 环内锥下端 (大) 直径 (mm)
RING_BOLT_R  = 20;   // 2×M4 螺栓孔心距中心 (mm)
RING_BOLT_D  = 4.5;  // M4 螺栓过孔 (mm)

/* ===== 滑动导向块 ===== */
BLOCK_W      = 88;   // 块宽 (mm) — 方形板, 覆盖对角线两光轴(±32@22.5°)与其耳座
BLOCK_D      = 88;   // 块深 (mm) — 与宽同, 做成方形以避开对角线干涉
BLOCK_H      = 20;   // 块高 (mm)
BLOCK_LOCK_D = 4.2;  // 锁高 M4 螺丝过孔 (mm)

$fn = 96;

// ===== 顶部法兰 (接传感器工具端) + 导向柱耳座 =====
module top_flange() {
    difference() {
        union() {
            // 主体圆盘: z ∈ [0, FLANGE_T]
            cylinder(d = FLANGE_D, h = FLANGE_T);
            // 两个耳座: 从法兰下表面向下伸 EAR_LEN, 夹住光轴上端
            for (a = [ROD_ANG, ROD_ANG + 180])
                rotate([0, 0, a])
                    translate([ROD_R, 0, -EAR_LEN])
                        cylinder(d = ROD_D + 8, h = FLANGE_T + EAR_LEN);
        }
        // 中央 Φ31.5 凸台过孔
        cylinder(d = BOSS_D, h = FLANGE_T + 1);
        // 4×M6 过孔 (螺栓自下往上穿入传感器)
        for (a = [0 : 90 : 270])
            rotate([0, 0, a]) translate([BC_D/2, 0, -1])
                cylinder(d = M6_CLEAR, h = FLANGE_T + 2);
        // 2×Φ6 定位销孔
        for (a = [DOWEL_ANG, DOWEL_ANG + 180])
            rotate([0, 0, a]) translate([DOWEL_R, 0, -1])
                cylinder(d = DOWEL_D, h = FLANGE_T + 2);
        // 耳座上的光轴孔: 自下往上盲孔, 深 EAR_LEN (留 2mm 底)
        for (a = [ROD_ANG, ROD_ANG + 180])
            rotate([0, 0, a])
                translate([ROD_R, 0, -EAR_LEN - 1])
                    cylinder(d = ROD_D + 0.3, h = EAR_LEN + 2);
    }
}

/* ===== 件选择 (PART 数值) ===== */
PART = 1; // 1=assembly 2=flange 3=block 4=tiebar 5=collet_s 6=collet_m 7=collet_l 8=ring

// 其余模块在后续 Task 中定义; 此处先只渲染法兰
if (PART == 2) top_flange();
```

- [ ] **Step 2: 渲染法兰验证无报错**

Run: `"/c/Program Files/OpenSCAD/openscad.exe" -o /tmp/flange.stl -D PART=2 Hardware/pen_clamp.scad`

Expected: 退出码 0，无 "WARNING/ERROR"，生成非空 `/tmp/flange.stl`。

- [ ] **Step 3: 预览检查耳座避让**

在 OpenSCAD GUI 打开预览，核对：两耳座落在 M6 孔(0/90/180/270)与销孔(45/225)之间，耳座与最近 M6/销孔**壁间 ≥2mm**（若不足，增大 `ROD_R` 或 `FLANGE_D`，勿硬凑）。

- [ ] **Step 4: Commit**

```bash
git add Hardware/pen_clamp.scad
git commit -m "feat(hardware): rewrite pen clamp - params + top flange with rod ears"
```

---

## Task 2: 滑动导向块 + 底部横梁 + 光轴占位

**Files:**
- Modify: `Hardware/pen_clamp.scad`（追加 `slide_block()`、`tie_bar()`、`guide_rod()`）

**Interfaces:**
- Consumes: Task 1 的参数（`ROD_R`/`ROD_ANG`/`ROD_D`/`ROD_L`、`BLOCK_*`、`COLLET_*`、`RING_BOLT_R`）。
- Produces: `slide_block()`、`tie_bar()`、`guide_rod()`。

- [ ] **Step 1: 追加三个模块**

在 `Hardware/pen_clamp.scad` 末尾（`if (PART == 2)` 之前）插入：

```openscad
// ===== 光轴占位 (装配视图用, 非打印件) =====
module guide_rod() {
    cylinder(d = ROD_D, h = ROD_L);
}

// ===== 滑动导向块: 两光轴过孔 + 中央夹头座 + 锁高螺孔 + 锁紧环螺栓孔 =====
module slide_block() {
    // 中央夹头座: 头段 Φ24 滑配, 挡圈 Φ28 沉孔落在块顶面
    socket_d = COLLET_HEAD_D + 0.5;      // 24.5
    flange_d = COLLET_FLANGE_D + 0.5;    // 28.5
    difference() {
        union() {
            // 主体: z ∈ [-BLOCK_H, 0]
            translate([-BLOCK_W/2, -BLOCK_D/2, -BLOCK_H])
                cube([BLOCK_W, BLOCK_D, BLOCK_H]);
            // 两光轴套筒: 外径 ROD_D+8, 沿柱向加高
            for (a = [ROD_ANG, ROD_ANG + 180])
                rotate([0, 0, a])
                    translate([ROD_R, 0, -BLOCK_H])
                        cylinder(d = ROD_D + 8, h = BLOCK_H);
        }
        // 两光轴滑配孔 (Φ8.3)
        for (a = [ROD_ANG, ROD_ANG + 180])
            rotate([0, 0, a])
                translate([ROD_R, 0, -BLOCK_H - 1])
                    cylinder(d = ROD_D + 0.3, h = BLOCK_H + 2);
        // 中央夹头座 (上下贯通, 笔从下穿过)
        cylinder(d = socket_d, h = BLOCK_H + 1, center = true);
        // 夹头挡圈沉孔 (顶面下 2mm)
        translate([0, 0, -2]) cylinder(d = flange_d, h = 3);
        // 锁高 M4 螺孔: 沿径向顶住其中一根光轴 (开在 +X 侧)
        rotate([0, 0, ROD_ANG])
            translate([ROD_R + ROD_D/2 + 1, 0, -BLOCK_H/2])
                rotate([0, 90, 0])
                    cylinder(d = BLOCK_LOCK_D, h = ROD_D + 4);
        // 2×M4 锁紧环螺栓孔: 自底面上钻 (锁紧环从下方用螺栓拉紧)
        for (dx = [-RING_BOLT_R, RING_BOLT_R])
            translate([dx, 0, -BLOCK_H - 1])
                cylinder(d = RING_BOLT_D, h = 12);
    }
}

// ===== 底部横梁: 连接两光轴下端, 成门架 =====
module tie_bar() {
    bar_h = 10;
    difference() {
        translate([-BLOCK_W/2, -BLOCK_D/2, 0])
            cube([BLOCK_W, BLOCK_D, bar_h]);
        for (a = [ROD_ANG, ROD_ANG + 180])
            rotate([0, 0, a])
                translate([ROD_R, 0, -1])
                    cylinder(d = ROD_D + 0.3, h = bar_h + 2);
    }
}

// 件选择
if (PART == 3) slide_block();
if (PART == 4) tie_bar();
```

- [ ] **Step 2: 渲染块与横梁验证无报错**

Run:
```bash
"/c/Program Files/OpenSCAD/openscad.exe" -o /tmp/block.stl -D PART=3 Hardware/pen_clamp.scad
"/c/Program Files/OpenSCAD/openscad.exe" -o /tmp/tiebar.stl -D PART=4 Hardware/pen_clamp.scad
```

Expected: 均退出码 0，生成非空 STL。

- [ ] **Step 3: 预览检查块孔位**

核对：两光轴孔与法兰耳座孔**同轴**（`ROD_R`/`ROD_ANG` 一致）；中央夹头座与两光轴孔不干涉；锁高螺孔确实顶到光轴；2×M4 螺栓孔与夹头座、光轴套筒都不干涉。

- [ ] **Step 4: Commit**

```bash
git add Hardware/pen_clamp.scad
git commit -m "feat(hardware): add slide block + tie bar + rod placeholder"
```

---

## Task 3: 分档自定心夹头 + 锁紧环

**Files:**
- Modify: `Hardware/pen_clamp.scad`（追加 `split_collet(bore)`、`clamp_ring()`）

**Interfaces:**
- Consumes: Task 1 的 `COLLET_*`/`RING_*`/`FINGERS` 参数。
- Produces: `split_collet(bore_d)`（参数化孔径）、`clamp_ring()`。

- [ ] **Step 1: 追加夹头与锁紧环模块**

在 `Hardware/pen_clamp.scad` 末尾（件选择之前）插入：

```openscad
// ===== 三瓣自定心夹头: 头段(挡圈) + 三瓣锥面工作段, 中央通孔 =====
module split_collet(bore_d) {
    difference() {
        union() {
            // 头段: z ∈ [-COLLET_HEAD_L, 0]
            translate([0, 0, -COLLET_HEAD_L])
                cylinder(d = COLLET_HEAD_D, h = COLLET_HEAD_L);
            // 挡圈: 顶面薄盘, 落在块顶
            translate([0, 0, -2])
                cylinder(d = COLLET_FLANGE_D, h = 2);
            // 三瓣工作段: 下宽上窄锥体, z ∈ [-(HEAD+FINGER), -HEAD]
            translate([0, 0, -COLLET_HEAD_L - COLLET_FINGER_L])
                cylinder(d1 = COLLET_FINGER_BOT, d2 = COLLET_FINGER_TOP, h = COLLET_FINGER_L);
        }
        // 中央通孔 (笔杆)
        cylinder(d = bore_d, h = COLLET_HEAD_L + COLLET_FINGER_L + 2, center = true);
        // 三瓣缝: 沿周向 120° 均布, 从工作段底贯穿到头段下沿
        for (i = [0 : FINGERS - 1]) {
            rotate([0, 0, i * 360/FINGERS])
                translate([0, 0, -COLLET_HEAD_L - COLLET_FINGER_L - 1])
                    cube([0.8, COLLET_FINGER_BOT + 2, COLLET_FINGER_L + 2], center = true);
        }
    }
}

// ===== 锁紧环: 内锥(上小下大) + 2×M4 过孔, 自下套上三瓣、螺栓拉向块 =====
module clamp_ring() {
    difference() {
        cylinder(d = RING_OD, h = RING_L);
        // 内锥: 上端小径, 下端大径
        translate([0, 0, -0.01])
            cylinder(d1 = RING_ID_TOP, d2 = RING_ID_BOT, h = RING_L + 0.02);
        // 2×M4 螺栓过孔
        for (dx = [-RING_BOLT_R, RING_BOLT_R])
            translate([dx, 0, -1])
                cylinder(d = RING_BOLT_D, h = RING_L + 2);
    }
}

// 件选择
if (PART == 5) split_collet(COLLET_BORE_S);
if (PART == 6) split_collet(COLLET_BORE_M);
if (PART == 7) split_collet(COLLET_BORE_L);
if (PART == 8) clamp_ring();
```

- [ ] **Step 2: 渲染三档夹头与锁紧环验证无报错**

Run:
```bash
"/c/Program Files/OpenSCAD/openscad.exe" -o /tmp/collet_s.stl -D PART=5 Hardware/pen_clamp.scad
"/c/Program Files/OpenSCAD/openscad.exe" -o /tmp/collet_m.stl -D PART=6 Hardware/pen_clamp.scad
"/c/Program Files/OpenSCAD/openscad.exe" -o /tmp/collet_l.stl -D PART=7 Hardware/pen_clamp.scad
"/c/Program Files/OpenSCAD/openscad.exe" -o /tmp/ring.stl -D PART=8 Hardware/pen_clamp.scad
```

Expected: 均退出码 0，生成非空 STL。

- [ ] **Step 3: 预览检查锥面配合**

核对：锁紧环内锥上端(22.5) < 夹头三瓣底端(25) < 环内锥下端(25.5)——保证环能从三瓣自由端套入、上移时收紧；三瓣缝(0.8mm)贯穿且不切到挡圈。

- [ ] **Step 4: Commit**

```bash
git add Hardware/pen_clamp.scad
git commit -m "feat(hardware): add 3-size self-centering split collet + clamp ring"
```

---

## Task 4: 装配视图 + 导出全部 STL

**Files:**
- Modify: `Hardware/pen_clamp.scad`（追加 `assembly()`）
- Create: `Hardware/stl/`（导出目录）

**Interfaces:**
- Consumes: Task 1-3 全部模块与参数。
- Produces: `assembly()`；`Hardware/stl/` 下 7 个 STL。

- [ ] **Step 1: 追加装配视图**

在件选择之前插入（示意笔用 `PEN_MAX_D` 中位 ~11mm 圆杆占位，顶到凸台面）：

```openscad
// ===== 装配视图: 法兰 + 双光轴 + 块(代表高度) + 夹头 + 锁紧环 + 笔占位 =====
module assembly() {
    top_flange();
    for (a = [ROD_ANG, ROD_ANG + 180])
        rotate([0, 0, a]) translate([ROD_R, 0, -EAR_LEN])
            guide_rod();
    // 块与夹头放在代表高度: 夹持点离传感器 ~150mm
    block_z = -150;
    translate([0, 0, block_z]) slide_block();
    translate([0, 0, block_z]) split_collet(COLLET_BORE_M);
    translate([0, 0, block_z - RING_L]) clamp_ring();
    // 笔占位: 从纸面(≈-260) 顶到凸台面(≈-1)
    translate([0, 0, -260]) cylinder(d = 11, h = 259);
}

if (PART == 1) assembly();
```

- [ ] **Step 2: 渲染装配视图验证无报错**

Run: `"/c/Program Files/OpenSCAD/openscad.exe" -o /tmp/assembly.stl -D PART=1 Hardware/pen_clamp.scad`

Expected: 退出码 0。预览核对：笔占位贯穿夹头中央、顶端到凸台过孔处；夹头+锁紧环在块下方同轴；双光轴穿法兰耳座与块、横梁。

- [ ] **Step 3: 导出全部零件 STL**

Run:
```bash
mkdir -p Hardware/stl
cd Hardware
OSC="/c/Program Files/OpenSCAD/openscad.exe"
"$OSC" -o stl/flange.stl  -D PART=2 pen_clamp.scad
"$OSC" -o stl/block.stl   -D PART=3 pen_clamp.scad
"$OSC" -o stl/tiebar.stl  -D PART=4 pen_clamp.scad
"$OSC" -o stl/collet_s.stl -D PART=5 pen_clamp.scad
"$OSC" -o stl/collet_m.stl -D PART=6 pen_clamp.scad
"$OSC" -o stl/collet_l.stl -D PART=7 pen_clamp.scad
"$OSC" -o stl/ring.stl    -D PART=8 pen_clamp.scad
ls -la stl/
```

Expected: `stl/` 下 7 个 STL，每个 >10KB。

- [ ] **Step 4: 删除旧单件 STL 并提交**

```bash
git rm Hardware/pen_clamp.stl
git add Hardware/pen_clamp.scad Hardware/stl/
git commit -m "feat(hardware): add assembly view + export per-part STLs"
```

---

## Task 5: 尺寸审查 + 更新 README + 提交

**Files:**
- Modify: `Hardware/README.md`（整篇重写为新设计）

**Interfaces:**
- Consumes: 全部已定参数。
- Produces: 新 README（参数表/装配/换笔/待测记录）。

- [ ] **Step 1: 尺寸审查清单逐项核对**

用 OpenSCAD 预览或打印件核对（打印前人工确认）：
- [ ] 4×M6 过孔与传感器工具端 4-M6 螺纹孔对齐（BC_D=50）。
- [ ] 2×Φ6 销孔与传感器 2-Φ6 销对齐（R25/45°）。
- [ ] 中央 Φ32 过孔套入 Φ31.5 凸台，凸台高 7mm 凸出法兰(6mm)下表面 1mm。
- [ ] 两光轴孔（法兰耳座 / 块 / 横梁）三者**同轴**。
- [ ] 三档夹头孔径 8 / 11.5 / 14.5 覆盖 6-10 / 10-13 / 13-16mm，锥面与锁紧环匹配。
- [ ] 笔从夹头中央穿过、顶端能顶到凸台面。

- [ ] **Step 2: 重写 README**

覆盖 `Hardware/README.md` 为：

```markdown
# 笔夹 (pen_clamp) 说明

## 设计原理
- 力传递：笔尾端（笔顶）轴向顶住传感器 Φ31.5 凸台面，书写力沿笔轴直传传感器。
- 夹头只扶正笔杆（共轴）并轻夹防落，**不承书写力**。
- 双 Φ8 光轴 + 滑动块：适配笔长、夹持点沿柱可调。
- 三档自定心夹头：适配笔杆直径 6~16mm，换档只换夹头。

## 传感器接口参数 (已确认)
| 参数 | 值 | 依据 |
|------|-----|------|
| BC_D | 50 mm | GB/T 14468.1-50-4-M6 |
| DOWEL_R | 25 mm | 实测确认 |
| DOWEL_ANG | 45° | 外形参数 §6 |
| 凸台直径/高度 | Φ31.5 / 7 mm | 高度=31.5-24.5，打印前卡尺复核 |

## 笔适配范围
| 项 | 值 |
|----|-----|
| 笔杆直径 | 6~16mm（夹头 6-10 / 10-13 / 13-16 三档）|
| 笔总长 | 170~300mm |
| 夹持点 | 沿柱可调（落在手握位，离笔尖 50~120mm）|

## 零件清单
见 `Hardware/stl/`：`flange` / `block` / `tiebar` / `collet_s|m|l` / `ring`。
采购件：Φ8 光轴×2(250mm)、M4×16 内六角×2、M4×10 内六角×1、M6 螺栓×4、Φ6 销×2。

## 装配顺序
1. 法兰板中心 Φ32 过孔套进传感器 Φ31.5 凸台。
2. 2×Φ6 销定位 + 4×M6 螺栓自下往上拧入传感器。
3. 两光轴压入法兰耳座（可点胶），套上滑动块，底部横梁固定光轴下端。
4. 按笔径选夹头档，夹头从块顶放入中央座。
5. 锁紧环自下套上夹头三瓣，2×M4 螺栓拧入块底面（先不收紧）。

## 装笔
1. 笔从下方穿入夹头中央孔。
2. 向上推笔，直到笔顶顶住传感器凸台面。
3. 下滑动块使夹头落在手握位 → 拧紧块侧 M4 锁高螺丝。
4. 拧紧锁紧环 2×M4 螺栓轻夹笔（只扶正+防落，勿死拧）。

## 换笔
松锁紧环螺栓 → 抽旧笔 → 换档(如需) → 插新笔顶到位 → 锁高 → 轻夹。
每次装笔都「顶到位」，笔尖相对传感器高度随笔长变 → **TCP 偏移需重标**（复用 TcpCalibration 流程）。

## 实机标定记录
（待填：坐标标定 RMS/点数、TCP 偏移 RMS/offset、凸台高度实测值）

## 演示记录
（待填：最终指标、接触阈值、软垫参数）
```

- [ ] **Step 3: Commit**

```bash
git add Hardware/README.md
git commit -m "docs(hardware): rewrite pen clamp README for axial-abutment design"
```

---

## 风险与回退

| 风险 | 应对 |
|------|------|
| 耳座与 M6/销孔干涉 | Task 1 Step 3 预览核对，增大 `ROD_R`/`FLANGE_D` |
| 夹头锥面与锁紧环不匹配（打印公差）| 夹头内径留余量，靠三瓣收缩适配；砂纸修锥面 |
| 三瓣缝 0.8mm 过窄打印粘连 | 调大缝宽到 1.0~1.2mm（`cube` 的 0.8 改）|
| 笔顶非平面顶不住 | 凸台面与笔顶间加 Φ8~16 平垫片 |
| 抬笔笔脱离传感器 | 夹头轻夹已足够；仍晃则传感器侧加轻弹簧顶柱 |

## 交付验收

- [ ] 7 个 STL 全部导出、无报错、尺寸审查通过。
- [ ] 3 档夹头能分别夹稳 6-10 / 10-13 / 13-16mm 圆杆且同轴。
- [ ] 装笔「顶到位」后，笔尾确实顶到凸台面、笔杆与传感器共轴。
- [ ] README 记录完整（参数/装配/换笔/待测）。
