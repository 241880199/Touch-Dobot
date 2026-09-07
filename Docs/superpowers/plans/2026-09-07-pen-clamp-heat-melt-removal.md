# 笔夹去热熔 + 修锥面 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把笔夹硬件的 3 处 M4 热熔嵌件改为六角螺母嵌槽，并把锁紧环内锥反转为与三瓣平行（线接触→全锥面贴合）。

**Architecture:** 整体架构不变（法兰/块/横梁/三档夹头/锁紧环五件）。改动集中在单文件 `Hardware/pen_clamp.scad` 的参数与 `slide_block()`/新增 `hex_nut_pocket()` 模块，外加两份文档（README、采购单）与两个 STL 重导。锁紧环 `clamp_ring()` 模块体无需改（已由参数驱动）。

**Tech Stack:** OpenSCAD 2021.01（CLI 在 `/c/Program Files/OpenSCAD/openscad.exe`）、3D 打印件 + 标准紧固件。

## Global Constraints

- 分支：`feat/pen-clamp-redesign`（不在 master 上直接改）。
- 力传递路径与传感器接口不动：笔尾顶 Φ31.5 凸台直传力；M6@Φ50 / 销 @R25/45° / 凸台 Φ31.5 不变。
- 三档夹头孔径（8 / 11.5 / 14.5mm）不动，`split_collet()` 不改。
- 六角螺母槽对边 `M4_NUT_AF = 7.0`、深 `M4_NUT_H = 3.5`（螺母 GB/T 6170：s=7 / m=3.2）。
- 锁紧环锥面须平行于三瓣（锥度 3/16，下窄上宽），`RING_L = 6`。
- OpenSCAD 的 `PART` 选择器是文件内硬编码（第 89 行 `PART = 1;`），`-D PART=N` 会被覆盖，导出单件须临时改该行再渲染，导出后改回 `1`。

---

## File Structure

| 文件 | 责任 |
|---|---|
| `Hardware/pen_clamp.scad` | 唯一几何源：参数 + 新增 `hex_nut_pocket()` + 改 `slide_block()` |
| `Hardware/stl/ring.stl` | 锁紧环打印件（重导） |
| `Hardware/stl/block.stl` | 滑动导向块打印件（重导） |
| `Hardware/README.md` | 紧固件处理说明 + BOM + 已知限制更新 |
| `Hardware/采购单.md` | 采购件清单（删热熔、加螺母、改 M4×16） |

其余 `flange.stl` / `tiebar.stl` / `collet_s|m|l.stl` 不重导（相关模块未改）。

---

### Task 1: 修锥面 — 锁紧环参数反转 + 缩短，重导 ring.stl

**Files:**
- Modify: `Hardware/pen_clamp.scad:42-48`（锁紧环参数块）
- Modify: `Hardware/pen_clamp.scad:89`（临时 `PART=8`，导出后改回）
- Produce: `Hardware/stl/ring.stl`

**Interfaces:**
- Produces: `RING_L = 6`、`RING_ID_BOT = 22.2`、`RING_ID_TOP = 23.325`。`clamp_ring()` 已引用这三个参数（`cylinder(d1=RING_ID_BOT, d2=RING_ID_TOP, h=RING_L)`），无需改模块体。

- [ ] **Step 1: 改锁紧环参数**

把 `Hardware/pen_clamp.scad` 第 42-48 行：

```openscad
/* ===== 锁紧环 (锥面压环) ===== */
RING_OD      = 32;   // 环外径 (mm)
RING_L       = 12;   // 环高 (mm)
RING_ID_TOP  = 22.5; // 环内锥上端 (小) 直径 (mm)
RING_ID_BOT  = 25.5; // 环内锥下端 (大) 直径 (mm)
RING_BOLT_R  = 20;   // 2×M4 螺栓孔心距中心 (mm)
RING_BOLT_D  = 4.5;  // M4 螺栓过孔 (mm)
```

改为：

```openscad
/* ===== 锁紧环 (锥面压环) ===== */
RING_OD      = 32;   // 环外径 (mm)
RING_L       = 6;    // 环高 (mm) — 缩短换夹紧行程 (锥面平行后 16-6=10mm)
RING_ID_BOT  = COLLET_FINGER_BOT + 0.2;  // 环内锥下端 (小) 直径 — 套过自由端 Φ22 留 0.2
RING_ID_TOP  = RING_ID_BOT + (COLLET_FINGER_TOP - COLLET_FINGER_BOT) * RING_L / COLLET_FINGER_L;  // 环内锥上端 (大) — 与三瓣锥平行
RING_BOLT_R  = 20;   // 2×M4 螺栓孔心距中心 (mm)
RING_BOLT_D  = 4.5;  // M4 螺栓过孔 (mm)
```

（`COLLET_FINGER_L=16`、`COLLET_FINGER_TOP=25`、`COLLET_FINGER_BOT=22` 在第 36-38 行，先于本块定义，可前向引用。）

- [ ] **Step 2: 验证参数值（方向正确）**

推导值应为 `RING_ID_BOT = 22.2`、`RING_ID_TOP = 22.2 + 3*6/16 = 23.325`，即 **22.2 < 23.325（下窄上宽）**，与原「25.5 > 22.5（下宽上窄）」相反。可临时在第 48 行后加一行 `echo(RING_ID_BOT=RING_ID_BOT, RING_ID_TOP=RING_ID_TOP);` 渲染确认后删除。

- [ ] **Step 3: 导出 ring.stl**

把第 89 行 `PART = 1;` 临时改为 `PART = 8;`，运行：

```bash
"/c/Program Files/OpenSCAD/openscad.exe" -o Hardware/stl/ring.stl Hardware/pen_clamp.scad
```

Expected: 退出码 0，`ring.stl` 更新（尺寸在数十~数百 KB，非 0 字节）。

- [ ] **Step 4: 复核环 bbox**

新环 bbox 应约为 `x∈[-26.25, 26.25]`（耳外伸）、`y∈[-16, 16]`、`z∈[0, 6]`（高 6mm）。用 OpenSCAD 打开 `ring.stl` 或切片软件确认：**高度从 12mm 变 6mm**，内锥下口小（≈Φ22.2）、上口大（≈Φ23.3）。

- [ ] **Step 5: 改回 PART 并提交**

把第 89 行改回 `PART = 1;`，确认没有残留 `echo` 调试行，然后：

```bash
git add Hardware/pen_clamp.scad Hardware/stl/ring.stl
git commit -m "fix(hardware): flip ring taper to parallel collet + shorten for surface contact"
```

---

### Task 2: 去热熔 — 六角螺母嵌槽 + 改 slide_block，重导 block.stl

**Files:**
- Modify: `Hardware/pen_clamp.scad:54`（加螺母参数）、`:96` 前（加 `hex_nut_pocket()`）、`:122-132`（改 `slide_block()`）
- Modify: `Hardware/pen_clamp.scad:89`（临时 `PART=3`）
- Produce: `Hardware/stl/block.stl`

**Interfaces:**
- Consumes: `M4_NUT_AF`、`M4_NUT_H`（本任务定义）；`BLOCK_W`、`BLOCK_D`、`BLOCK_H`、`ROD_R`、`ROD_ANG`、`ROD_D`、`RING_BOLT_R`、`RING_BOLT_D`、`BLOCK_LOCK_D`（已存在）。
- Produces: `module hex_nut_pocket(af=M4_NUT_AF, h=M4_NUT_H)`（六角棱柱，底在 z=0 向上 h）。

- [ ] **Step 1: 加螺母参数**

把第 54 行：

```openscad
BLOCK_LOCK_D = 4.5;  // 锁高 M4 过孔 (mm) — 4.5 匹配 M4 热熔嵌件, 与块底螺栓孔一致
```

改为：

```openscad
BLOCK_LOCK_D = 4.5;  // 锁高 M4 过孔 (mm)

/* ===== M4 六角螺母嵌槽 (去热熔) ===== */
M4_NUT_AF = 7.0;   // 六角槽对边 (mm) — 与 M4 螺母 s=7 等值, 打印缩水呈过盈卡住
M4_NUT_H  = 3.5;   // 六角槽深 (mm) — 螺母 m=3.2 + 0.3
```

- [ ] **Step 2: 加 `hex_nut_pocket()` 模块**

在 `// ===== 滑动导向块: ... =====` 注释（原第 96 行）之前插入：

```openscad
// ===== M4 六角螺母嵌槽: 六角棱柱 (对边 af, 高 h), 底在 z=0 向上 =====
module hex_nut_pocket(af = M4_NUT_AF, h = M4_NUT_H) {
    cylinder(r = af / sqrt(3), h = h, $fn = 6);
}

```

（`af/√3` 是外接圆半径，使 `$fn=6` 六边形的对边距 = af。）

- [ ] **Step 3: 改 `slide_block()` 的锁高孔 + 锁紧环螺栓孔**

把 `slide_block()` 内（原第 122-132 行）：

```openscad
        // 锁高 M4 螺孔: 沿 +X 从块侧面钻入, 在 +22.5° 光轴的 Y 位置顶住杆身锁高。
        //   原径向孔沿 22.5° 从 r=30 到 42, 但块侧面沿 22.5° 在 r≈45, 孔埋块内不可达 → 改轴向 +X 直达块面
        lock_y   = ROD_R * sin(ROD_ANG);                       // 光轴中心 Y = 12.63
        lock_len = BLOCK_W/2 - ROD_R*cos(ROD_ANG) + ROD_D/2;  // 从 +X 面钻到刚过光轴内缘
        translate([BLOCK_W/2 - lock_len/2, lock_y, -BLOCK_H/2])
            rotate([0, 90, 0])
                cylinder(d = BLOCK_LOCK_D, h = lock_len);
        // 2×M4 锁紧环螺栓孔: 自底面上钻 (锁紧环从下方用螺栓拉紧)
        for (dx = [-RING_BOLT_R, RING_BOLT_R])
            translate([dx, 0, -BLOCK_H - 1])
                cylinder(d = RING_BOLT_D, h = 12);
```

改为：

```openscad
        // 锁高 M4 螺孔: 沿 +X 从块侧面钻入, 在 +22.5° 光轴的 Y 位置顶住杆身锁高。
        //   原径向孔沿 22.5° 从 r=30 到 42, 但块侧面沿 22.5° 在 r≈45, 孔埋块内不可达 → 改轴向 +X 直达块面
        lock_y   = ROD_R * sin(ROD_ANG);                       // 光轴中心 Y = 12.63
        lock_len = BLOCK_W/2 - ROD_R*cos(ROD_ANG) + ROD_D/2;  // 从 +X 面钻到刚过光轴内缘
        translate([BLOCK_W/2 - lock_len/2, lock_y, -BLOCK_H/2])
            rotate([0, 90, 0])
                cylinder(d = BLOCK_LOCK_D, h = lock_len);
        // 锁高 M4 六角螺母槽: +X 面开口 (沿 -X 沉入块内 3.5)
        translate([BLOCK_W/2, lock_y, -BLOCK_H/2])
            rotate([0, -90, 0])
                hex_nut_pocket();
        // 2×M4 锁紧环螺栓孔: 通孔贯穿块高 + 块底面六角螺母槽 (开口朝下)
        for (dx = [-RING_BOLT_R, RING_BOLT_R]) {
            translate([dx, 0, -BLOCK_H - 1])
                cylinder(d = RING_BOLT_D, h = BLOCK_H + 2);
            translate([dx, 0, -BLOCK_H])
                hex_nut_pocket();
        }
```

- [ ] **Step 4: 导出 block.stl**

把第 89 行 `PART = 1;` 临时改为 `PART = 3;`，运行：

```bash
"/c/Program Files/OpenSCAD/openscad.exe" -o Hardware/stl/block.stl Hardware/pen_clamp.scad
```

Expected: 退出码 0，`block.stl` 更新（非 0 字节）。

- [ ] **Step 5: 复核 block bbox**

块 bbox 应仍为 `x∈[-42,42]`、`y∈[-24,24]`、`z∈[-20,0]`（84×48×20，六角槽/通孔是减法不改变外轮廓）。用 OpenSCAD/切片确认：+X 面出现六角槽（y=12.63、z=−10 处）、底面 ±20 出现两处六角槽 + 通孔。

- [ ] **Step 6: 改回 PART 并提交**

把第 89 行改回 `PART = 1;`，然后：

```bash
git add Hardware/pen_clamp.scad Hardware/stl/block.stl
git commit -m "fix(hardware): replace heat-melt inserts with captured hex nuts"
```

---

### Task 3: 文档 — 更新 README.md 与 采购单.md

**Files:**
- Modify: `Hardware/README.md`
- Modify: `Hardware/采购单.md`

**Interfaces:** 无代码接口，纯文档。

- [ ] **Step 1: 更新 README 零件清单行**

`Hardware/README.md` 第 26 行：

```markdown
采购件：Φ8 光轴×2(250mm)、M4×20 内六角×2、M4×10 内六角×1、M6×16 螺栓×4（按传感器孔深复核）、Φ6 销×2。
```

改为：

```markdown
采购件：Φ8 光轴×2(250mm)、M4×16 内六角×2、M4×10 顶丝×1、M4 六角螺母×3、M6×16 螺栓×4（按传感器孔深复核）、Φ6 销×2。
```

- [ ] **Step 2: 重写 README「紧固件处理」段**

把 `Hardware/README.md` 第 35-45 行（`## 紧固件处理（打印后必做，否则无法锁紧）` 到该节结束）整体替换为：

```markdown
## 紧固件处理（六角螺母嵌槽，免热熔/免攻丝）
以下螺纹位**不用热熔、不用攻丝**——打印件上预留 M4 六角螺母嵌槽，把标准 **M4 六角螺母（GB/T 6170，对边 7mm / 厚 3.2mm）** 压入即可（槽对边 7.0mm，打印缩水后呈轻微过盈卡住）：

| 位置 | 嵌槽 | 用途 | 装配 |
|------|------|------|------|
| 块底面 2×M4 螺母槽 | 开口朝下，深 3.5 | 接收锁紧环螺栓、把环上拉夹紧 | 螺母压入槽（过紧锉刀微扩、过松点胶）；螺栓自下穿过锁紧环耳拧入 |
| 块侧面 1×M4 螺母槽 | +X 面开口，深 3.5 | 锁高顶丝吃丝、尖端顶光轴 | 螺母压入；顶丝拧入、尖端顶光轴锁高 |
| 锁紧环 2×M4 过孔 | `RING_BOLT_D=4.5` | 螺栓杆身穿过（不需螺纹） | 保持原样 |

- **最终紧固件 BOM**：M4×16 内六角×2（锁紧环→块底）+ M4×10 顶丝×1（块侧锁高）+ M4 六角螺母×3 + M6×16 螺栓×4（按传感器孔深复核）+ Φ6 销×2。
```

- [ ] **Step 3: 删除 README 线接触「已知限制」并更新夹紧范围**

删掉 `Hardware/README.md`「已知限制 / 打印前待测」里第 64 行的整条「锥面配合为线接触（锥角不匹配）」bullet（该问题已修复）。

把第 66 行里 `各档实际可夹范围 S≈6.1–10 / M≈9.6–13 / L≈12.6–16` 的 `S≈6.1` 改为 `S≈6.3`（环全锥面夹紧 ~1.7mm 后最细端由 6.1→6.3），并在该条末尾补一句「锁紧环改全锥面贴合后闭合量由环行程 ~1.7mm 决定」。

- [ ] **Step 4: 更新采购单（删热熔、加螺母、改 M4×16）**

`Hardware/采购单.md`：

1. 第 10 行 `M4 × 20` 改 `M4 × 16`，备注改 `环 6 + 螺母 3.2 ≈ 9.2mm 吃深`。
2. 第 12 行整行（`热熔嵌件 | M4 × 6 × Φ4.5（黄铜） | 3 | ...`）替换为：

```markdown
| 4 | 六角螺母 | M4（GB/T 6170，对边 7mm / 厚 3.2mm） | 3 | 块底 2 孔 + 块侧 1 孔 嵌槽吃丝 | 压入打印件六角槽，免热熔免攻丝 |
```

3. 删除第 16-20 行整段「### 热熔嵌件选型说明」。
4. 「四、工具」表删除第 2 行 `| 2 | 热熔枪（嵌件烙铁头） | 压入 M4 热熔嵌件 |`，并把原第 3 行「卡尺」的序号改为 `2`。

- [ ] **Step 5: 提交**

```bash
git add Hardware/README.md Hardware/采购单.md
git commit -m "docs(hardware): update README + BOM for captive-nut scheme"
```

---

## Self-Review

**1. Spec 覆盖：**
- 去热熔（3 处→六角螺母嵌槽）：Task 2（参数+模块+slide_block）✅、Task 3（README/采购单）✅
- 修锥面（环锥反转 + RING_L=6）：Task 1 ✅
- 六角槽对边 7.0 / 深 3.5（过盈卡住）：Task 2 Step 1 ✅
- M4×20→M4×16、顶丝不动、删热熔枪：Task 3 ✅
- 重导 block.stl / ring.stl：Task 1 + Task 2 ✅
- 非目标（不改三档夹头/传感器接口/力路径）：Global Constraints ✅

**2. 占位符：** 无 TBD/TODO，所有代码块完整。

**3. 一致性：** `hex_nut_pocket` 签名在 Task 2 定义并被 Task 2 调用；`RING_ID_BOT/TOP` 派生公式在 Task 1 定义且 `clamp_ring()` 已引用；派生值 22.2 / 23.325 与 spec §4.2 一致。
