// Standalone test: Button2Mapping —— 按钮2 姿态映射的纯函数（真旋转合成）
// Build: build_button2_mapping_test.bat
// Run: test_button2_mapping.exe
//
// 【判据一律用【矩阵】比，不用 RPY 比】—— RPY 有表示歧义（同一个旋转有无穷多组 RPY，
//   且本机末端常年贴着 rx≈±180 的接缝，2026-09-23 实测 |rx|>170 占 85.8%），矩阵没有。
//
// 【这套用例针对的是"算术"，不是"表"】：现有那张硬编码矩阵 M 逐行验算成立
//   （ω_robot = M·ω_dev，见 Button2Mapping.h 里的推导表），所以错的不在映射表，
//   而在算术 —— 把 Euler 角的【差】当旋转向量、再把三个角【逐分量加到】参照姿态上。
//   用例 ⑦（大角度）就是那个病根的照妖镜：旧写法在这里会差几十度。
//
// ⚠ 本文件【不碰 socket、不碰 OpenHaptics 运行时】：Button2Mapping.h 是纯函数，
//   这正是把映射抽出来的目的（与 FrameLayout.h / ZeroDriftCheck.h 同一套做法）。

#include <iostream>
#include <cmath>

#include "../relay/Button2Mapping.h"
#include "../relay/CoordinateTransform.h"
#include "../calibration/TcpCalibration.h"

static int g_passed = 0, g_failed = 0;

// ⚠ 宏的语义是【逐文件】的，不是仓库级的 —— 照抄别的文件之前先读它自己那三行。
//   本文件这三个与 test_frame_layout.cpp 逐字一致：TEST 只是【标签打印器】（不求值、
//   不计失败），所以断言一律写 CHECK。2026-09-23 实测过这个坑：在 test_frame_layout.cpp
//   里把断言写成 TEST(表达式)，即使把映射表改错也照样印 "12 passed, 0 failed" 并退出 0。
#define TEST(name) do { std::cout << "  " << #name << "... "; } while(0)
#define PASS() do { std::cout << "PASS" << std::endl; g_passed++; } while(0)
#define CHECK(cond) do { if (!(cond)) { std::cout << "FAIL: " << #cond << std::endl; g_failed++; return; } } while(0)

using namespace TcpCalibration;

// ===== 期望值一侧的构造工具（不是被测代码）=====

// 轴角 → 矩阵（Rodrigues，轴须为单位向量）
static void axisR(double ax, double ay, double az, double thDeg, double R[9]) {
    double th = thDeg * 3.14159265358979323846 / 180.0, c = cos(th), s = sin(th), t = 1 - c;
    R[0]=t*ax*ax+c;    R[1]=t*ax*ay-s*az; R[2]=t*ax*az+s*ay;
    R[3]=t*ax*ay+s*az; R[4]=t*ay*ay+c;    R[5]=t*ay*az-s*ax;
    R[6]=t*ax*az-s*ay; R[7]=t*ay*az+s*ax; R[8]=t*az*az+c;
}
static void rotY(double thDeg, double R[9]) { axisR(0, 1, 0, thDeg, R); }
static void identity(double R[9]) {
    R[0]=1; R[1]=0; R[2]=0; R[3]=0; R[4]=1; R[5]=0; R[6]=0; R[7]=0; R[8]=1;
}
static void mul3(const double A[9], const double B[9], double C[9]) {
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) {
        double v=0; for (int k=0;k<3;k++) v += A[i*3+k]*B[k*3+j]; C[i*3+j]=v; }
}
static double angDeg(const double A[9], const double B[9]) {   // 两旋转矩阵夹角（度）
    double T[9]; for (int i=0;i<3;i++) for (int j=0;j<3;j++) T[i*3+j]=A[0*3+i]*B[0*3+j]+A[1*3+i]*B[1*3+j]+A[2*3+i]*B[2*3+j];
    double tr = T[0]+T[4]+T[8]; double c = (tr-1)/2; if (c>1) c=1; if (c<-1) c=-1;
    return acos(c) * 180.0 / 3.14159265358979323846;
}
// 被测函数的返回值（RPY 度）→ 矩阵，供上面那些【矩阵比】用。
static void targetM(const double ref[3], const double refS[3], const double cur[3], double R[9]) {
    Vec3 t = button2OrientationTarget(ref, refS, cur); rpyToMatrix(t.x, t.y, t.z, R);
}

// ===== 用例 =====

// ① 笔杆不动 ⇒ 逐位返回参照（本函数没有位置入参 ⇒ 平移天然不参与）
static void test_no_motion_returns_reference() {
    TEST(no_motion_returns_reference);
    double ref[3] = {-173.0, -22.0, -118.0}, refS[3] = {-20.0, 12.0, 30.0};
    Vec3 t = button2OrientationTarget(ref, refS, refS);
    double Rt[9], Rr[9]; rpyToMatrix(t.x, t.y, t.z, Rt); rpyToMatrix(ref[0], ref[1], ref[2], Rr);
    CHECK(angDeg(Rt, Rr) < 1e-6);
    PASS();
}

// ②③ 左右摆：sz 减/增 θ ⇒ 基座系绕 +Y/−Y
static void test_left_right_is_yaw_about_base_Y() {
    TEST(left_right_is_yaw_about_base_Y);
    double ref[3] = {0,0,0}, refS[3] = {0,0,0};
    double L[3] = {0, 0, -30.0}, R[3] = {0, 0, +30.0};
    double RL[9], RR[9], Ry_p[9], Ry_m[9], I[9];
    targetM(ref, refS, L, RL);
    // ⚠ 简报里【漏了这一行】：只算 L 的期望，而下面却 CHECK 了 RR。RR 是未初始化的 9 个
    //   double，那样写要么读到栈上的垃圾（/O2 下还可能"碰巧"绿），要么让整条断言变成噪声。
    //   补上它是为了【实现简报自己写下的那条期望】——"右摆 = 绕 −Y 转 30°"，不是改符号。
    targetM(ref, refS, R, RR);
    rotY( +30.0, Ry_p); rotY(-30.0, Ry_m); identity(I);
    CHECK(angDeg(RL, Ry_p) < 1e-6);      // 左摆 = 绕 +Y 转 30°
    CHECK(angDeg(RR, Ry_m) < 1e-6);      // 右摆 = 绕 −Y 转 30°
    PASS();
}

// ④⑤ 前后摆：sx 增/减 θ ⇒ 基座系绕 +X/−X
static void test_fore_aft_is_pitch_about_base_X() {
    TEST(fore_aft_is_pitch_about_base_X);
    double ref[3] = {0,0,0}, refS[3] = {0,0,0};
    double F[3] = {+30.0, 0, 0}, B[3] = {-30.0, 0, 0};
    double RF[9], RB[9], Rx_p[9], Rx_m[9];
    targetM(ref, refS, F, RF); targetM(ref, refS, B, RB);
    axisR(1,0,0, +30.0, Rx_p); axisR(1,0,0, -30.0, Rx_m);
    CHECK(angDeg(RF, Rx_p) < 1e-6);     // 前摆 = 绕 +X 转 30°
    CHECK(angDeg(RB, Rx_m) < 1e-6);     // 后摆 = 绕 −X 转 30°
    PASS();
}

// ⑥ 自转：sy 增 θ ⇒ 基座系绕 Z 转 θ（= 工具 roll ⇒ 关节上就是 J6，见实现的注释）
static void test_twist_is_roll_about_base_Z() {
    TEST(twist_is_roll_about_base_Z);
    double ref[3] = {0,0,0}, refS[3] = {0,0,0};
    double T1[3] = {0, +30.0, 0}, T2[3] = {0, -30.0, 0};
    double R1[9], R2[9], Rz_p[9], Rz_m[9];
    targetM(ref, refS, T1, R1); targetM(ref, refS, T2, R2);
    axisR(0,0,1, +30.0, Rz_p); axisR(0,0,1, -30.0, Rz_m);
    CHECK(angDeg(R1, Rz_p) < 1e-6);
    CHECK(angDeg(R2, Rz_m) < 1e-6);
    PASS();
}

// ⑦ ★ 大角度（旧实现真正翻车的地方）：90° 的左右摆仍必须【精确】是绕 +Y 转 90°。
//    旧写法把 Euler 差当旋转向量、还在参照上逐分量加 ⇒ 这里会差几十度。
static void test_large_tilt_is_exact() {
    TEST(large_tilt_is_exact);
    double ref[3] = {-173.0, -22.0, -118.0};      // 用本机真实的贴接缝姿态
    double refS[3] = {-20.0, 12.0, 30.0};
    double L90[3] = {refS[0], refS[1], refS[2] - 90.0};   // 左摆 90°（sz −90）
    double Rr[9], Rt[9], Ry90[9], want[9];
    rpyToMatrix(ref[0], ref[1], ref[2], Rr);
    targetM(ref, refS, L90, Rt);
    axisR(0,1,0, +90.0, Ry90);
    mul3(Ry90, Rr, want);                          // 期望 = R_y(90°) · R_ref
    CHECK(angDeg(Rt, want) < 1e-6);
    PASS();
}

// ⑧ 限幅按【旋转角】：超过 ORIENT_MAX_OFFSET_DEG 的输入 ⇒ 目标与参照的夹角恰为该上限
static void test_offset_is_clamped_by_rotation_angle() {
    TEST(offset_is_clamped_by_rotation_angle);
    double ref[3] = {0,0,0}, refS[3] = {0,0,0};
    double L170[3] = {0, 0, -170.0};               // 左摆 170°，超过上限 150°
    double Rr[9], Rt[9];
    rpyToMatrix(ref[0], ref[1], ref[2], Rr);
    targetM(ref, refS, L170, Rt);
    CHECK(fabs(angDeg(Rt, Rr) - 150.0) < 1e-3);    // 夹到 ORIENT_MAX_OFFSET_DEG
    PASS();
}

// ⑨ 【简报 8 条之外，实现者追加】矩阵 → RPY 的万向锁分支（cos(ry)≈0）。
//    为什么必须有用例：这条分支是实现里【我自己加的】（简报只说了"最后提取一次 RPY"），
//    它要么被覆盖，要么就是一段"看着防住了、其实没跑过"的代码 —— 本仓库最怕后者。
//    怎么构造：目标 = R_y(+90°)·R_z(180°)（左摆 90°，参照 rz=180°）—— 它恰好落在
//    cos(ry_target)=0 上，自由变量是 rx 与 rz 的【组合量】（此处 rz + rx = 180°）。
//
//    ⚠ 2026-09-23 实测（负对照）：把这条分支换成"不特判"的朴素提取（rx=atan2(R[7],R[8])、
//      rz=atan2(R[3],R[0])），【矩阵往返仍然是精确的、这条 angDeg 断言照样绿】——
//      因为 cos(90°) 在 double 里是 6.12e-17 而不是 0，那几个"零"其实带着比例信息。
//      朴素版给出 (63.43°, −90°, 116.57°)，本实现给出 (0, −90°, −180°)：同一个旋转的
//      两组等价代表。⇒ 所以能区分二者的【不是矩阵】，而是【取哪一组代表】。
//      第二条断言钉住的就是这件事：cos(ry)=0 时必须取规范代表 rx = 0，否则"返回哪一组"
//      取决于舍入噪声 —— 那是表示歧义，正是本函数一路用矩阵比想避开的东西。
static void test_gimbal_lock_picks_a_canonical_representative() {
    TEST(gimbal_lock_picks_a_canonical_representative);
    double ref[3] = {0, 0, 180.0}, refS[3] = {0,0,0};
    double L90[3] = {0, 0, -90.0};
    double Rz180[9], Ry90[9], want[9], Rt[9];
    axisR(0,0,1, 180.0, Rz180); axisR(0,1,0, 90.0, Ry90);
    mul3(Ry90, Rz180, want);
    CHECK(fabs(want[6]) > 0.999999);          // 自检：这个期望值【确实】在万向锁上
    Vec3 t = button2OrientationTarget(ref, refS, L90);
    rpyToMatrix(t.x, t.y, t.z, Rt);
    CHECK(angDeg(Rt, want) < 1e-6);           // ① 往返必须精确（两组代表都满足）
    CHECK(fabs(t.x) < 1e-9);                  // ② 且必须是规范代表（朴素版在这里给 63.43°）
    PASS();
}

// ⑩ 【同上，实现者追加】万向锁的另一侧（ry = +90°）。
//    ⑨ 只走到 ry = −90° 那一支（它的目标 row2 是 +1）。两支的公式不同（组合量一个是
//    rz − rx、另一个是 rz + rx），只测一支等于留下另一支的符号没人验过 —— 而这类符号错
//    在仓库里出现的方式一向是"静默偏几十度"。这里取的目标 R_y(90°)·R_x(90°) 恰好
//    落在 ry = +90°，且 rz = −90°（不是 0）⇒ 若把 rz 的公式写成 −atan2(...)，往返立刻差 180°。
static void test_gimbal_lock_plus90_side_is_exact() {
    TEST(gimbal_lock_plus90_side_is_exact);
    double ref[3] = {90.0, 0, 0}, refS[3] = {0,0,0};
    double L90[3] = {0, 0, -90.0};
    double Rx90[9], Ry90[9], want[9], Rt[9];
    axisR(1,0,0, 90.0, Rx90); axisR(0,1,0, 90.0, Ry90);
    mul3(Ry90, Rx90, want);
    CHECK(want[6] < -0.999999);               // 自检：确实落在 ry = +90° 那一侧
    CHECK(fabs(want[5]) > 0.9);               // 自检：rz/组合量确实非平凡（否则测不出符号）
    Vec3 t = button2OrientationTarget(ref, refS, L90);
    rpyToMatrix(t.x, t.y, t.z, Rt);
    CHECK(angDeg(Rt, want) < 1e-6);
    CHECK(fabs(t.x) < 1e-9);
    PASS();
}

int main() {
    std::cout << "--- Button2Mapping (按钮2 姿态映射：真旋转合成) ---" << std::endl;
    test_no_motion_returns_reference();
    test_left_right_is_yaw_about_base_Y();
    test_fore_aft_is_pitch_about_base_X();
    test_twist_is_roll_about_base_Z();
    test_large_tilt_is_exact();
    test_offset_is_clamped_by_rotation_angle();
    test_gimbal_lock_picks_a_canonical_representative();
    test_gimbal_lock_plus90_side_is_exact();
    std::cout << "\nResults: " << g_passed << " passed, " << g_failed << " failed" << std::endl;
    return g_failed == 0 ? 0 : 1;
}
