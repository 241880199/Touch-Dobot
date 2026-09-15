// 笔夹 — Touch-Dobot 书写演示（轴向顶紧 + 双光轴可滑导向 + 自定心夹头 · 固定毛笔 Φ8×220mm）
// 安装: 力传感器(KWR75B) 工具端法兰 (4×M6 + 2×Φ6销 + Φ31.5凸台高7mm)
// 力传递: 笔尾顶住 Φ31.5 凸台面 → 书写力沿笔轴直传传感器；夹头只扶正+防落。

/* ===== 传感器接口 (KWR75B, 已确认) ===== */
BC_D      = 50;   // 4×M6 分度圆直径 (mm)
DOWEL_R   = 25;   // 2×Φ6 销孔径向距离 (mm)
DOWEL_ANG = 45;   // 销孔相对 X/Y 轴角度 (deg)
BOSS_D    = 32;   // Φ31.5 凸台过孔直径 (mm), 留 0.5 余量
BOSS_H    = 7;    // 凸台高度 (mm) = 31.5(总高) - 24.5(法兰安装高) (仅文档, 不参与几何)
M6_CLEAR  = 6.5;  // M6 螺栓过孔 (mm)
DOWEL_D   = 6.2;  // Φ6 销过孔 (mm) — FDM 缩水后 ~6.0 滑配 Φ6 m6 销 (原 6.0 缩水后 ~5.6 塞不进)
FLANGE_T  = 6;    // 法兰板厚 (mm) — 凸台凸出 1mm, 笔尾干净顶到
FLANGE_D  = 86;   // 法兰外径 (mm) — 光轴 R31 后耳Φ22 外缘 R42, 留 1mm; 并消除锁高螺母槽与光轴孔重叠

/* ===== 笔适配 (固定毛笔) ===== */
PEN_D = 8;    // 毛笔杆直径 (mm) — 0.8cm, 匹配夹头孔径 COLLET_BORE=8
PEN_L = 220;  // 毛笔杆长 (mm) — 22cm

/* ===== 导向柱 (2×Φ8 市售光轴) ===== */
ROD_D    = 8;     // 光轴直径 (mm)
ROD_L    = 190;   // 光轴长度 (mm) — 笔总长 220(杆188+出锋32); 45#软轴 Φ8×190 截断即可, 不打孔(耳座顶丝锁死)
ROD_R    = 31;    // 光轴心距中心 (mm) — 落空档(-45°)回缩到 31, 消除锁高螺母槽(内壁x26.4)与光轴孔(+面x26.07)的 1.08mm 重叠; 耳-凸台 4, 环-光轴 4, 耳-M6头 6.1
ROD_ANG  = -45;   // 光轴相对 +X 轴角度 (deg) — 落空档(135°/315°, 无 M6/销), 彻底避开螺丝孔
EAR_LEN  = 16;    // 耳座伸出法兰下表面的长度 (mm)
EAR_D    = ROD_D + 14; // 耳座外径 (mm) — 加粗到 22 容纳 M4 螺母槽 + 径向顶丝 (原 16); 落空档后无 M6 冲突
EAR_LOCK_D = 4.5;      // 耳座径向锁光轴 M4 顶丝过孔 (mm)
EAR_LOCK_H = 8;        // 顶丝轴距耳座底面高度 (mm) — 落在 16mm 盲孔中段

/* ===== 夹头 (三瓣锥套) ===== */
COLLET_BORE   = 8;   // 夹头孔径 (mm) — 适配 Φ8 毛笔
COLLET_HEAD_D    = 24;   // 夹头头段外径 (mm)
COLLET_HEAD_L    = 20;   // 夹头头段长度 (mm) — 须=块高 BLOCK_H, 使三瓣整段垂到块下供锁紧环套夹
COLLET_FLANGE_D  = 28;   // 夹头顶部挡圈外径 (mm)
COLLET_FINGER_L  = 16;   // 三瓣工作段长度 (mm)
COLLET_FINGER_TOP= 25;   // 三瓣顶端(与头段连接处)外径 (mm) — 上宽 (环为宽松直孔, 不再靠锥面夹紧)
COLLET_FINGER_BOT= 22;   // 三瓣底端(自由端)外径 (mm) — 下窄 (便于环从自由端套入)
FINGERS = 3;             // 瓣数
FINGER_GAP = 2.0;        // 三瓣缝宽 (mm) — 决定可收紧量 3·gap/π; 2.0 使各档覆盖到最细笔且打印不粘连

/* ===== 锁紧环 (径向顶丝压环: 宽松直孔 + 3×M4 顶丝压缩三瓣) ===== */
RING_OD      = 46;    // 环外径 (mm) — 容纳径向螺母槽 + 竖直螺栓孔; ≥44.5 才能包住 Φ4.5 孔@R20 (原 32)
RING_ID      = 25.5;  // 环内孔 (mm, 直孔宽松) — 手指顶 Φ25 + 0.5; 不做锥面, 夹紧由径向顶丝承担
RING_L       = 10;    // 环高 (mm) — 容纳径向 M4 螺母 (AF7)
RING_BOLT_R  = 20;    // 2×M4 竖直螺栓孔心距中心 (mm) — 只固定环位置, 不承夹紧
RING_BOLT_D  = 4.5;   // M4 竖直螺栓过孔 (mm)
// 径向顶丝 (3×M4, 一瓣一颗)
RADIAL_ANG   = [30, 150, 270];  // 顶丝角度 (deg) — 对准三瓣中心(60/180/300 转 -30°), 避开竖直螺栓 0°/180°
RADIAL_D     = 4.5;   // M4 顶丝过孔 (mm)
RADIAL_H     = 5;     // 顶丝轴距环顶面深度 (mm) = 环高一半
// 径向螺母槽 (顶面放螺母 + 外壁挡螺母, 同锁高螺母套路)
NUT_WALL     = 2;     // 外壁厚 (mm) — 挡螺母不外脱
NUT_SLOT_R   = 3.6;   // 槽径向厚 (mm) — M4 螺母厚 3.2 + 0.4
NUT_SLOT_W   = 7.4;   // 槽周向宽 (mm) — M4 螺母对边 7 + 0.4
NUT_SLOT_D   = 8.5;   // 槽轴向深 (mm, 自顶面) — 螺母 AF7 + 余量

/* ===== 滑动导向块 ===== */
BLOCK_W      = 64;   // 块宽 (mm) — 方形, 覆盖对角光轴(-45°/135°)与其 Φ16 套筒(外缘 |x|=|y|≈31.3)
BLOCK_D      = 64;   // 块深 (mm) — 方形 64×64 (光轴对角对称; 面积≈原 86×48 矩形, 不增料)
BLOCK_H      = 20;   // 块高 (mm)
BLOCK_LOCK_D = 4.5;  // 锁高 M4 顶丝过孔 (mm) — 顶丝自 +X 穿过 +X 壁与块体、拧入顶面放入的 M4 螺母, 尖端顶光轴

/* ===== M4 六角螺母嵌槽 (去热熔) ===== */
M4_NUT_AF = 7.4;   // 六角槽对边 (mm) — M4 螺母 s=7 + 0.4 打印缩水余量 (原 7.0 压不进; 块顶槽靠螺栓张力压住, 无需过盈)
M4_NUT_H  = 3.6;   // 六角槽深 (mm) — 螺母 m=3.2 + 0.4

$fn = 96;

// ===== 顶部法兰 (接传感器工具端) + 导向柱耳座 =====
module top_flange() {
    difference() {
        union() {
            // 主体圆盘: z ∈ [0, FLANGE_T]
            cylinder(d = FLANGE_D, h = FLANGE_T);
            // 两个耳座: 从法兰下表面向下伸 EAR_LEN, 夹住光轴上端 (加粗到 EAR_D 容纳径向锁光轴顶丝)
            for (a = [ROD_ANG, ROD_ANG + 180])
                rotate([0, 0, a])
                    translate([ROD_R, 0, -EAR_LEN])
                        cylinder(d = EAR_D, h = FLANGE_T + EAR_LEN);
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
        // 耳座上的光轴孔: 自下往上盲孔, 深 EAR_LEN (光轴端落在法兰底面 z=0) + 径向锁光轴顶丝/螺母槽
        for (a = [ROD_ANG, ROD_ANG + 180])
            rotate([0, 0, a])
                translate([ROD_R, 0, -EAR_LEN]) {
                    cylinder(d = ROD_D + 0.3, h = EAR_LEN);  // 光轴盲孔
                    ear_lock();                               // 顶丝孔 + 螺母槽 (锁光轴, 免攻丝)
                }
    }
}

// ===== 耳座径向锁光轴: M4 顶丝自外圆旋入螺母、尖端顶光轴压坑锁死 (同锁高顶丝套路) =====
module ear_lock() {
    ear_r    = EAR_D/2;              // 耳座外半径 (11)
    rod_r    = ROD_D/2;              // 光轴半径 (4)
    hole_len = ear_r + 0.5 - rod_r;  // 顶丝过孔长: 外圆外 0.5 到光轴面
    // 顶丝过孔 (径向沿 X): Φ4.5 贯穿外壁 + 螺母槽 + 内壁, 直达光轴面
    translate([(rod_r + ear_r + 0.5)/2, 0, EAR_LOCK_H])
        rotate([0, 90, 0])
            cylinder(d = EAR_LOCK_D, h = hole_len, center = true);
    // M4 螺母槽: 自耳座顶面(z=FLANGE_T+EAR_LEN)开口向下, 螺母自顶面放入; 外壁 2mm 挡螺母 (Φ4.5<7mm 不外脱)
    translate([ear_r - NUT_WALL - M4_NUT_H, -M4_NUT_AF/2, EAR_LOCK_H - M4_NUT_AF/2])
        cube([M4_NUT_H, M4_NUT_AF, (FLANGE_T + EAR_LEN) - EAR_LOCK_H + M4_NUT_AF/2]);
}

/* ===== 件选择 (PART 数值) ===== */
PART = 1; // 1=assembly 2=flange 3=block 5=collet 8=ring

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
        // 锁高 M4 顶丝 + 顶面放入的 M4 螺母(卡在 +X 壁内侧): 顶丝自 +X 拧入螺母、尖端顶住杆身锁高。
        //   螺母自块顶面放入(同锁紧环螺母), 拧紧把螺母往 +X 顶、被 +X 壁(Φ4.5 过孔 < 螺母 7mm)挡住不外脱 —— 免攻丝免热熔。
        //   原径向孔沿 22.5° 从 r=30 到 42, 但块侧面沿 22.5° 在 r≈45, 孔埋块内不可达 → 改轴向 +X 直达块面
        lock_y   = ROD_R * sin(ROD_ANG);                       // 光轴中心 Y = -23.33 (锁 +X 侧 315° 光轴)
        lock_len = BLOCK_W/2 - ROD_R*cos(ROD_ANG) + ROD_D/2;  // 从 +X 面钻到刚过光轴内缘
        // 顶丝过孔: Φ4.5 贯穿 +X 壁 + 螺母槽 + 块体, 直达光轴面 (不攻丝)
        translate([BLOCK_W/2 - lock_len/2, lock_y, -BLOCK_H/2])
            rotate([0, 90, 0])
                cylinder(d = BLOCK_LOCK_D, h = lock_len);
        // 锁高螺母槽: 顶面开口矩形槽 (X M4_NUT_H × Y M4_NUT_AF), 螺母自顶面放入; +X 壁留 2mm 挡螺母
        // 槽底 = 顶丝中心(-BLOCK_H/2) 再下半螺母对边(M4_NUT_AF/2), 使螺母落位中心对准顶丝 (原 /sqrt(3) 多深 0.77mm)
        translate([BLOCK_W/2 - M4_NUT_H - 2, lock_y - M4_NUT_AF/2, -BLOCK_H/2 - M4_NUT_AF/2])
            cube([M4_NUT_H, M4_NUT_AF, BLOCK_H/2 + M4_NUT_AF/2]);
        // 2×M4 锁紧环螺栓孔: 通孔贯穿块高 + 块顶面六角螺母沉孔(开口向上, 拧紧把螺母压向槽底台阶)
        // 螺栓 M4×35 自下穿过锁紧环耳(6)+块(20)+螺母(3.2)≈29.2mm, 头在环下、螺母在块顶
        for (dx = [-RING_BOLT_R, RING_BOLT_R]) {
            translate([dx, 0, -BLOCK_H - 1])
                cylinder(d = RING_BOLT_D, h = BLOCK_H + 2);
            translate([dx, 0, -M4_NUT_H])
                hex_nut_pocket();
        }
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

// ===== 锁紧环: 宽松直孔圆盘 + 2×M4 竖直螺栓过孔(固定) + 3×M4 径向顶丝(夹紧) =====
module clamp_ring() {
    r_out    = RING_OD/2;                        // 23
    r_slot   = r_out - NUT_WALL - NUT_SLOT_R/2;  // 螺母槽中心半径 19.2
    hole_in  = RING_ID/2 - 2;                    // 径向孔内端 (伸入内孔 2mm)
    hole_out = r_out + 1.5;                      // 径向孔外端 (伸出外圆 1.5mm, 保证穿透)
    difference() {
        // 主体圆盘 (z∈[0,RING_L], 顶面在 z=RING_L)
        cylinder(d = RING_OD, h = RING_L);
        // 内孔 (宽松直孔)
        translate([0, 0, -0.01])
            cylinder(d = RING_ID, h = RING_L + 0.02);
        // 2×M4 竖直螺栓过孔 (只固定环, 不承夹紧)
        for (dx = [-RING_BOLT_R, RING_BOLT_R])
            translate([dx, 0, -0.01])
                cylinder(d = RING_BOLT_D, h = RING_L + 0.02);
        // 3×径向顶丝过孔: 贯穿外壁 + 螺母槽 + 内实体, 内伸入内孔、外伸出外圆
        for (th = RADIAL_ANG)
            rotate([0, 0, th])
                translate([(hole_in + hole_out)/2, 0, RING_L - RADIAL_H])
                    rotate([0, 90, 0])
                        cylinder(d = RADIAL_D, h = hole_out - hole_in, center = true);
        // 3×径向螺母槽 (顶面放入, 外壁挡螺母): 矩形槽自环顶面 z=RING_L 向下开
        for (th = RADIAL_ANG)
            rotate([0, 0, th])
                translate([r_slot, 0, RING_L - NUT_SLOT_D/2])
                    cube([NUT_SLOT_R, NUT_SLOT_W, NUT_SLOT_D], center = true);
    }
}

// ===== 装配视图: 法兰 + 双光轴 + 块 + 夹头 + 锁紧环 + 笔占位 =====
module assembly() {
    top_flange();
    // 双光轴: guide_rod 局部 +z 朝上, 故整体下移 ROD_L 使光轴插入耳座盲孔(顶到 z=0)后向下垂 (z∈[-ROD_L, 0])
    for (a = [ROD_ANG, ROD_ANG + 180])
        rotate([0, 0, a]) translate([ROD_R, 0, -ROD_L])
            guide_rod();
    // 块与夹头放在代表高度: 夹持点离传感器 ~150mm
    block_z = -150;
    translate([0, 0, block_z]) slide_block();
    translate([0, 0, block_z]) rotate([0, 0, -30]) split_collet(COLLET_BORE);  // 转 -30°: 三瓣中心(60/180/300)→(30/150/270) 对准径向顶丝
    // 锁紧环: 宽松直孔圆盘套在三瓣上段 (环顶贴块底 z=-170), 3×径向顶丝从侧面旋入压三瓣
    translate([0, 0, block_z - COLLET_HEAD_L - RING_L]) clamp_ring();
    // 笔占位: 从纸面(≈-221) 顶到凸台面(≈-1), 固定 Φ8×220 毛笔
    translate([0, 0, -PEN_L - 1]) cylinder(d = PEN_D, h = PEN_L);
}

// 件选择
if (PART == 1) assembly();
if (PART == 5) split_collet(COLLET_BORE);
if (PART == 8) clamp_ring();

if (PART == 3) slide_block();
if (PART == 2) top_flange();
