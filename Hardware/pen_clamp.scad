// 笔夹 — Touch-Dobot 书写演示
// 安装: 力传感器(KWR75B) 工具端法兰下方 (4×M6 螺纹孔 + 2×Φ6 销 + Φ31.5 定心凸台)
// 夹持: V 型槽 + 侧向螺丝顶紧 (可调内径 6~14mm), 笔杆滑动定位 (短套筒)

/* ===== 待实测参数 (KWR75B 工具端法兰, 打印前用卡尺复核) ===== */
BC_D      = 50;   // 工具端法兰 4×M6 螺纹孔分布圆直径 (mm)  [确认: GB/T 14468.1-50-4-M6 分度圆 Φ50]
DOWEL_R    = 25;  // 2×Φ6 定位销孔距中心径向距离 (mm)      [确认: 落在 Φ50 分度圆上, R=25]
DOWEL_ANG  = 45;  // Φ6 销孔相对 M6 孔的角度 (deg)          [已确认: 相对 X/Y 轴 45°]
/* ============================================================= */

/* ===== 已定参数 ===== */
FLANGE_T     = 8;     // 上端法兰板厚 (mm)
M6_CLEAR     = 6.5;   // M6 螺钉过孔直径 (mm) — 螺栓自下往上穿入传感器 4-M6 螺纹孔
DOWEL_D      = 6.0;   // Φ6 定位销配合孔直径 (mm)
BOSS_D       = 32.0;  // 顶部定心凸台(Φ31.5)过孔直径 (mm), 留 0.5 余量
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
