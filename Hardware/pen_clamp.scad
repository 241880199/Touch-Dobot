// 笔夹 — Touch-Dobot 书写演示
// 安装: 力传感器(KWR75B) 工具端法兰下方 (4×M6 螺纹孔 + 2×Φ6 销 + Φ31.5 定心凸台)
// 夹持: 抱紧式开口套筒 (纵向开缝 + 横向螺丝抱紧笔杆), 笔杆滑动定位

/* ===== 待实测参数 (KWR75B 工具端法兰 + 实际用笔, 打印前用卡尺复核) ===== */
BC_D      = 50;   // 工具端法兰 4×M6 螺纹孔分布圆直径 (mm)  [确认: GB/T 14468.1-50-4-M6 分度圆 Φ50]
DOWEL_R    = 25;  // 2×Φ6 定位销孔距中心径向距离 (mm)      [确认: 落在 Φ50 分度圆上, R=25]
DOWEL_ANG  = 45;  // Φ6 销孔相对 M6 孔的角度 (deg)          [已确认: 相对 X/Y 轴 45°]
PEN_D      = 10;  // 实际用笔笔杆直径 (mm) — 用卡尺量实际笔, 换笔就改这里 [待实测]
/* ============================================================= */

/* ===== 已定参数 ===== */
FLANGE_T      = 8;     // 上端法兰板厚 (mm)
M6_CLEAR      = 6.5;   // M6 螺钉过孔直径 (mm) — 螺栓自下往上穿入传感器 4-M6 螺纹孔
DOWEL_D       = 6.0;   // Φ6 定位销配合孔直径 (mm)
BOSS_D        = 32.0;  // 顶部定心凸台(Φ31.5)过孔直径 (mm), 留 0.5 余量
SLEEVE_LEN    = 35;    // 夹持段长度 (mm) — 短套筒, 笔杆可滑动
CLAMP_SCREW_D = 4.0;   // 抱紧螺丝直径 (M4) — 螺丝+螺母 (或一侧攻丝)
BORE_CLEAR    = 0.4;   // 笔孔余量 (mm) — 笔径 + 0.4, 靠开缝压缩抱紧
SLOT_W        = 2.0;   // 纵向开缝宽度 (mm) — 决定抱紧压缩量
FLANGE_D      = BC_D + 18;

$fn = 128;

// ===== 上端法兰板 (接传感器工具端) =====
module top_flange() {
    difference() {
        cylinder(d = FLANGE_D, h = FLANGE_T);
        // 4×M6 过孔 (螺栓自下而上穿入传感器 4-M6 螺纹孔)
        for (a = [0 : 90 : 270])
            rotate([0, 0, a]) translate([BC_D/2, 0, -1])
                cylinder(d = M6_CLEAR, h = FLANGE_T + 2);
        // 2×Φ6 定位销孔 (互为 180°, 相对 M6 孔偏 45°)
        for (a = [DOWEL_ANG, DOWEL_ANG + 180])
            rotate([0, 0, a]) translate([DOWEL_R, 0, -1])
                cylinder(d = DOWEL_D, h = FLANGE_T + 2);
        // 中心定心凸台过孔 (Φ31.5 凸台)
        cylinder(d = BOSS_D, h = FLANGE_T + 2);
    }
}

// ===== 夹持主体: 抱紧式开口套筒 (collet) =====
module clamp_body() {
    body_d  = 40;                           // 主体外径 (mm) — 须大于 BOSS_D(32), 才能与法兰板底面环带搭接
    bore_d  = PEN_D + BORE_CLEAR;           // 笔孔直径 (实测笔径 + 余量)
    screw_x = (bore_d/2 + body_d/2) / 2;    // 抱紧螺丝 X 位置 (开缝径向中点)
    difference() {
        // 主体圆柱 (自法兰板下沿向下延伸, 顶部上探 1mm 与法兰板搭接成一体)
        translate([0, 0, -SLEEVE_LEN])
            cylinder(d = body_d, h = SLEEVE_LEN + 1);
        // 中心笔杆通孔 (直径 = 实测笔径 + 余量)
        translate([0, 0, -SLEEVE_LEN - 1])
            cylinder(d = bore_d, h = SLEEVE_LEN + 2);
        // 纵向开缝 (从笔孔中心到 +X 外壁, 沿 Z 全长) — 抱紧时缝闭合、孔径缩小
        translate([0, -SLOT_W/2, -SLEEVE_LEN - 1])
            cube([body_d/2 + 1, SLOT_W, SLEEVE_LEN + 2]);
        // 抱紧螺丝孔 (M4, 沿 Y 横穿开缝): 螺丝+螺母拧紧 → +Y/-Y 两半靠拢 → 缝闭合抱紧笔杆
        translate([screw_x, body_d/2 + 1, -SLEEVE_LEN/2])
            rotate([90, 0, 0])
                cylinder(d = CLAMP_SCREW_D, h = body_d + 2);
    }
}

// ===== 装配 =====
top_flange();
clamp_body();
