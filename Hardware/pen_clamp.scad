// 笔夹 — Touch-Dobot 书写演示（轴向顶紧 + 双光轴可滑导向 + 三档自定心夹头）
// 安装: 力传感器(KWR75B) 工具端法兰 (4×M6 + 2×Φ6销 + Φ31.5凸台高7mm)
// 力传递: 笔尾顶住 Φ31.5 凸台面 → 书写力沿笔轴直传传感器；夹头只扶正+防落。

/* ===== 传感器接口 (KWR75B, 已确认) ===== */
BC_D      = 50;   // 4×M6 分度圆直径 (mm)
DOWEL_R   = 25;   // 2×Φ6 销孔径向距离 (mm)
DOWEL_ANG = 45;   // 销孔相对 X/Y 轴角度 (deg)
BOSS_D    = 32;   // Φ31.5 凸台过孔直径 (mm), 留 0.5 余量
BOSS_H    = 7;    // 凸台高度 (mm) = 31.5(总高) - 24.5(法兰安装高) (仅文档, 不参与几何)
M6_CLEAR  = 6.5;  // M6 螺栓过孔 (mm)
DOWEL_D   = 6.0;  // Φ6 销配合孔 (mm)
FLANGE_T  = 6;    // 法兰板厚 (mm) — 凸台凸出 1mm, 笔尾干净顶到
FLANGE_D  = 84;   // 法兰外径 (mm) — 扩到 84 给导向柱耳座留位 (耳外缘 R41)

/* ===== 笔适配 ===== */
PEN_MIN_D = 6;    // 最小笔杆直径 (mm)
PEN_MAX_D = 16;   // 最大笔杆直径 (mm)
PEN_MIN_L = 170;  // 最短笔总长 (mm)
PEN_MAX_L = 300;  // 最长笔总长 (mm)

/* ===== 导向柱 (2×Φ8 市售光轴) ===== */
ROD_D    = 8;     // 光轴直径 (mm)
ROD_L    = 250;   // 光轴长度 (mm) — 覆盖夹持点离传感器 ~100~210mm + 余量
ROD_R    = 33;    // 光轴心距中心 (mm) — 避让凸台孔 R16 与 M6/销孔 R25 (耳-M6/销壁间隙 ≥2.5mm)
ROD_ANG  = 22.5;  // 光轴相对 +X 轴角度 (deg) — 落在 M6(0°) 与销(45°) 中间
EAR_LEN  = 16;    // 耳座伸出法兰下表面的长度 (mm)

/* ===== 夹头 (三瓣锥套) ===== */
COLLET_BORE_S = 8;     // 档1 孔径: 适配 6-10mm 笔
COLLET_BORE_M = 11.5;  // 档2 孔径: 适配 10-13mm 笔
COLLET_BORE_L = 14.5;  // 档3 孔径: 适配 13-16mm 笔
COLLET_HEAD_D    = 24;   // 夹头头段外径 (mm)
COLLET_HEAD_L    = 20;   // 夹头头段长度 (mm) — 须=块高 BLOCK_H, 使三瓣整段垂到块下供锁紧环套夹
COLLET_FLANGE_D  = 28;   // 夹头顶部挡圈外径 (mm)
COLLET_FINGER_L  = 16;   // 三瓣工作段长度 (mm)
COLLET_FINGER_TOP= 25;   // 三瓣顶端(与头段连接处)外径 (mm) — 上宽, 压环上移收紧
COLLET_FINGER_BOT= 22;   // 三瓣底端(自由端)外径 (mm) — 下窄, 便于压环套入
FINGERS = 3;             // 瓣数
FINGER_GAP = 2.0;        // 三瓣缝宽 (mm) — 决定可收紧量 3·gap/π; 2.0 使各档覆盖到最细笔且打印不粘连

/* ===== 锁紧环 (锥面压环) ===== */
RING_OD      = 32;   // 环外径 (mm)
RING_L       = 6;    // 环高 (mm) — 缩短换夹紧行程 (锥面平行后 16-6=10mm)
RING_ID_BOT  = COLLET_FINGER_BOT + 0.2;  // 环内锥下端 (小) 直径 — 套过自由端 Φ22 留 0.2
RING_ID_TOP  = RING_ID_BOT + (COLLET_FINGER_TOP - COLLET_FINGER_BOT) * RING_L / COLLET_FINGER_L;  // 环内锥上端 (大) — 与三瓣锥平行
RING_BOLT_R  = 20;   // 2×M4 螺栓孔心距中心 (mm)
RING_BOLT_D  = 4.5;  // M4 螺栓过孔 (mm)

/* ===== 滑动导向块 ===== */
BLOCK_W      = 84;   // 块宽 (mm) — 矩形, 覆盖对角线两光轴(±33@22.5°)与其 Φ16 耳座(外缘 X≈±38.5)
BLOCK_D      = 48;   // 块深 (mm) — 光轴 Y 向只到 ±20.6, 48 足够(原 88×88 方形 Y 向浪费 ~23mm/侧)
BLOCK_H      = 20;   // 块高 (mm)
BLOCK_LOCK_D = 4.5;  // 锁高 M4 过孔 (mm)

/* ===== M4 六角螺母嵌槽 (去热熔) ===== */
M4_NUT_AF = 7.0;   // 六角槽对边 (mm) — 与 M4 螺母 s=7 等值, 打印缩水呈过盈卡住
M4_NUT_H  = 3.5;   // 六角槽深 (mm) — 螺母 m=3.2 + 0.3

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

// ===== 光轴占位 (装配视图用, 非打印件) =====
module guide_rod() {
    cylinder(d = ROD_D, h = ROD_L);
}

// ===== M4 六角螺母嵌槽: 六角棱柱 (对边 af, 高 h), 底在 z=0 向上 =====
module hex_nut_pocket(af = M4_NUT_AF, h = M4_NUT_H) {
    cylinder(r = af / sqrt(3), h = h, $fn = 6);
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
        // 中央夹头座 (上下贯通, 笔从下穿过) — 通孔贯穿块体全高, 否则夹头三瓣与笔杆被块体挡住
        translate([0, 0, -BLOCK_H - 1])
            cylinder(d = socket_d, h = BLOCK_H + 2);
        // 夹头挡圈沉孔 (顶面下 2mm)
        translate([0, 0, -2]) cylinder(d = flange_d, h = 3);
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
            // 三瓣工作段: 下窄上宽锥体, z ∈ [-(HEAD+FINGER), -HEAD]
            translate([0, 0, -COLLET_HEAD_L - COLLET_FINGER_L])
                cylinder(d1 = COLLET_FINGER_BOT, d2 = COLLET_FINGER_TOP, h = COLLET_FINGER_L);
        }
        // 中央通孔 (笔杆): 贯穿头段+三瓣全高 (从三瓣底端 z=-HEAD-FINGER 顶到 z=+2)
        translate([0, 0, -COLLET_HEAD_L - COLLET_FINGER_L])
            cylinder(d = bore_d, h = COLLET_HEAD_L + COLLET_FINGER_L + 2);
        // 三瓣缝: 沿周向 120° 均布的单条径向缝, 自中心至外缘贯穿工作段, 不切头段/挡圈
        max_d = max(COLLET_FINGER_TOP, COLLET_FINGER_BOT);
        for (i = [0 : FINGERS - 1]) {
            rotate([0, 0, i * 360/FINGERS])
                translate([max_d/4, 0, -COLLET_HEAD_L - COLLET_FINGER_L/2])
                    cube([max_d/2 + 1, FINGER_GAP, COLLET_FINGER_L], center = true);
        }
    }
}

// ===== 锁紧环: 内锥(上小下大) + 2×M4 过孔, 自下套上三瓣、螺栓拉向块 =====
module clamp_ring() {
    ear_d = RING_BOLT_D + 8;  // 螺栓耳直径: Φ4.5 过孔两侧各留 ~4mm 壁
    difference() {
        union() {
            // 锥环主体
            cylinder(d = RING_OD, h = RING_L);
            // 两螺栓耳: 环体外径 RING_OD 只到 R16, 螺栓孔在 RING_BOLT_R=R20, 外伸耳承载 M4 过孔
            for (dx = [-RING_BOLT_R, RING_BOLT_R])
                translate([dx, 0, 0])
                    cylinder(d = ear_d, h = RING_L);
        }
        // 内锥: 上端小径 (RING_ID_TOP), 下端大径 (RING_ID_BOT)
        translate([0, 0, -0.01])
            cylinder(d1 = RING_ID_BOT, d2 = RING_ID_TOP, h = RING_L + 0.02);
        // 2×M4 螺栓过孔
        for (dx = [-RING_BOLT_R, RING_BOLT_R])
            translate([dx, 0, -1])
                cylinder(d = RING_BOLT_D, h = RING_L + 2);
    }
}

// ===== 装配视图: 法兰 + 双光轴 + 底部横梁 + 块 + 夹头 + 锁紧环 + 笔占位 =====
module assembly() {
    top_flange();
    // 双光轴: guide_rod 局部 +z 朝上, 故整体下移 ROD_L 使其自耳座向下垂 (z∈[-EAR_LEN-ROD_L, -EAR_LEN])
    for (a = [ROD_ANG, ROD_ANG + 180])
        rotate([0, 0, a]) translate([ROD_R, 0, -EAR_LEN - ROD_L])
            guide_rod();
    // 底部横梁: 连接两光轴下端, 成门架
    translate([0, 0, -EAR_LEN - ROD_L]) tie_bar();
    // 块与夹头放在代表高度: 夹持点离传感器 ~150mm
    block_z = -150;
    translate([0, 0, block_z]) slide_block();
    translate([0, 0, block_z]) split_collet(COLLET_BORE_M);
    // 锁紧环: 包在三瓣手指段 (z∈[-182,-170], 头段=20 使手指整段在块底 z=-170 之下) 而非头段
    translate([0, 0, block_z - COLLET_HEAD_L - RING_L]) clamp_ring();
    // 笔占位: 从纸面(≈-260) 顶到凸台面(≈-1)
    translate([0, 0, -260]) cylinder(d = (PEN_MIN_D + PEN_MAX_D)/2, h = 259);
}

// 件选择
if (PART == 1) assembly();
if (PART == 5) split_collet(COLLET_BORE_S);
if (PART == 6) split_collet(COLLET_BORE_M);
if (PART == 7) split_collet(COLLET_BORE_L);
if (PART == 8) clamp_ring();

if (PART == 3) slide_block();
if (PART == 4) tie_bar();
if (PART == 2) top_flange();
