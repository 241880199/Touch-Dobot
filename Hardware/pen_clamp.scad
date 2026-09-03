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
