// 笔夹 — Touch-Dobot 书写演示
// 安装: 力传感器(KWR75B) 工具端法兰下方 (4×M6 螺纹孔 + 2×Φ6 销 + Φ31.5 定心凸台)
// 夹持: V 型块 + 顶紧螺丝 (螺丝行程适应 6~14mm 笔径), 笔杆滑动定位

/* ===== 待实测参数 (KWR75B 工具端法兰, 打印前用卡尺复核) ===== */
BC_D      = 50;   // 工具端法兰 4×M6 螺纹孔分布圆直径 (mm)  [确认: GB/T 14468.1-50-4-M6 分度圆 Φ50]
DOWEL_R    = 25;  // 2×Φ6 定位销孔距中心径向距离 (mm)      [确认: 落在 Φ50 分度圆上, R=25]
DOWEL_ANG  = 45;  // Φ6 销孔相对 M6 孔的角度 (deg)          [已确认: 相对 X/Y 轴 45°]
/* ============================================================= */

/* ===== 笔径范围 (可调, 由螺丝行程覆盖) ===== */
PEN_MIN_D   = 6;    // 最小笔径 (mm) — 决定螺丝行程
PEN_MAX_D   = 14;   // 最大笔径 (mm) — 决定 V 槽深度
/* =========================================== */

/* ===== 已定参数 ===== */
FLANGE_T      = 8;     // 上端法兰板厚 (mm)
M6_CLEAR      = 6.5;   // M6 螺钉过孔直径 (mm) — 螺栓自下往上穿入传感器 4-M6 螺纹孔
DOWEL_D       = 6.0;   // Φ6 定位销配合孔直径 (mm)
BOSS_D        = 32.0;  // 顶部定心凸台(Φ31.5)过孔直径 (mm), 留 0.5 余量
SLEEVE_LEN    = 35;    // 夹持段长度 (mm) — 笔杆可滑动
CLAMP_SCREW_D = 5.0;   // 顶紧螺丝直径 (M5) — 攻丝或嵌螺母
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

// ===== 夹持主体: V 型块 + 顶紧螺丝 =====
module clamp_body() {
    // V 槽几何 (90° V): 深度须容纳 Φ14 笔底 + 螺丝余量
    //   Φ14 笔底距 apex = (PEN_MAX_D/2)*(√2+1) ≈ 16.9, 再加螺丝余量 → v_depth ≈ 20
    v_depth  = PEN_MAX_D/2 * (sqrt(2) + 1) + 3;
    v_apex_y = v_depth * 0.6;                       // V apex 位置 (深入上壁)
    body_w   = 2 * (v_depth + 4);                   // 主体 X 宽 (V 底宽 2*v_depth + 侧壁)
    body_h   = 2 * (v_depth + 6);                   // 主体 Y 高 (V 深 + 上下壁)
    screw_reach = body_h/2 + v_apex_y - v_depth;    // 螺丝孔长 (下壁贯穿到 V 槽底)

    difference() {
        // 主体矩形块 (顶部上探 1mm 与法兰板搭接成一体)
        translate([-body_w/2, -body_h/2, -SLEEVE_LEN])
            cube([body_w, body_h, SLEEVE_LEN + 1]);

        // V 型槽 (90°, apex 在上 +Y, 开口朝下 -Y): 沿 Z 挤出, 笔靠 V 面
        translate([0, 0, -SLEEVE_LEN - 1])
            linear_extrude(height = SLEEVE_LEN + 2)
                polygon(points = [[0, v_apex_y],
                                  [-v_depth, v_apex_y - v_depth],
                                  [ v_depth, v_apex_y - v_depth]]);

        // 顶紧螺丝孔 (M5, 沿 Y 从下壁贯穿到 V 槽底): 螺丝顶笔入 V 槽
        translate([0, -body_h/2, -SLEEVE_LEN/2])
            rotate([-90, 0, 0])
                cylinder(d = CLAMP_SCREW_D, h = screw_reach + 1);
    }
}

// ===== 装配 =====
top_flange();
clamp_body();
