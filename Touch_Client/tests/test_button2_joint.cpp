// Standalone test: Button2Joint —— 按钮2 关节空间映射的纯函数（笔杆 Euler 增量 → 关节增量）
// Build: build_button2_joint_test.bat
// Run: test_button2_joint.exe
//
// 【这套用例的判据分三层】
//   ① 结构层（**与符号无关**，本方案的核心）：只动笔杆一个角 ⇒ **恰好一个关节**动，
//      且 J1/J2/J3 **逐位不变**。映射表整个改错（Rx→J6 之类）在这里就红。
//   ② 数值层（**经 Config 的符号常数表达**）：幅度/死区/限幅的度数与边界。
//      ⚠ 这里【一个 +1 都不写死】—— 符号由 `Config::BTN2_J4/J5/J6_SIGN` 决定，
//        而计划 Task 3 上机会把它们钉成 ±1。写死了 +1 的话，Task 3 一改符号就满堂红，
//        而红的是测试不是实现。
//   ③ 契约层：NaN/Inf 守卫、签名里没有位置入参（编译期）。
//
// ⚠ 本文件【不碰 socket、不碰 OpenHaptics 运行时、不碰 appState】：Button2Joint 是纯函数，
//   这正是把它抽出来的目的（与 Button2Mapping.h / FrameLayout.h / JitterStats.h 同一套做法）。
//   ⇒ 本套件只链接 `../relay/Button2Joint.cpp` 一个翻译单元，且**不需要**任何 /I 路径。

#include <iostream>
#include <cmath>
#include <limits>      // ⑥ 用 quiet_NaN / infinity

#include "../relay/Button2Joint.h"
#include "../config/Config.h"

static int g_passed = 0, g_failed = 0;

// ⚠ 宏的语义是【逐文件】的，不是仓库级的 —— 照抄别的文件之前先读它自己那三行。
//   本文件这三个与 test_button2_mapping.cpp 逐字一致：TEST 只是【标签打印器】（不求值、
//   不计失败），所以断言一律写 CHECK。2026-09-23 实测复现过这个坑：把断言写成
//   TEST(表达式) 时，即使被断言的映射整个改错也照样印 "N passed, 0 failed" 并退出 0。
#define TEST(name) do { std::cout << "  " << #name << "... "; } while(0)
#define PASS() do { std::cout << "PASS" << std::endl; g_passed++; } while(0)
#define CHECK(cond) do { if (!(cond)) { std::cout << "FAIL: " << #cond << std::endl; g_failed++; return; } } while(0)

// ===== ③ 编译期钉子：签名里【没有】位置入参 =====
// "笔杆平移 ⇒ 机械臂不动"在本方案里是**结构性**保证：函数根本没有位置分量可吃
// （第二道保证在 RelayCore：它不会把位置并进笔杆偏移）。结构性质运行期测不出来，
// 所以钉在类型上 —— 谁给函数加第 5 个（位置）参数，下面这行**立刻编译不过**。
// ⚠ 参数里的 `const double[6]` 会退化成 `const double*`，所以这里写指针形式。
typedef void (*Btn2JointFn)(const double*, const double*, const double*, double*);
static Btn2JointFn g_pinnedSignature = &button2JointTarget;

// ===== 用例 =====

// ① ★ 结构判据（本方案的核心，且与符号无关）：一次只动笔杆一根轴 ⇒ 恰好一个关节动。
//    三条都写全（Rx/Rz/Ry），因为"只查一条"放过的是整个映射表：Rx→J4 对而 Rz→J5 错、
//    或 Rz/Ry 对调的写法，都能在只查一条时全绿。
static void test_each_stylus_axis_moves_exactly_one_joint() {
    TEST(each_stylus_axis_moves_exactly_one_joint);
    const double ref[6] = {10, 20, 30, 40, 50, 60};
    const double refS[3] = {0, 0, 0};
    double out[6];

    // (a) 前后摆：笔杆 Rx +10 ⇒ 只有 J4 动
    {
        const double cur[3] = {+10.0, 0, 0};
        button2JointTarget(ref, refS, cur, out);
        CHECK(fabs(out[3] - ref[3]) > 1e-6);            // J4 动了
        CHECK(fabs(out[4] - ref[4]) < 1e-9);            // J5 没动
        CHECK(fabs(out[5] - ref[5]) < 1e-9);            // J6 没动
        CHECK(fabs(out[0] - ref[0]) < 1e-9);            // J1 逐位不变
        CHECK(fabs(out[1] - ref[1]) < 1e-9);            // J2 逐位不变
        CHECK(fabs(out[2] - ref[2]) < 1e-9);            // J3 逐位不变
    }

    // (b) 左右摆：笔杆 Rz −10 ⇒ 只有 J5 动
    {
        const double cur[3] = {0, 0, -10.0};
        button2JointTarget(ref, refS, cur, out);
        CHECK(fabs(out[4] - ref[4]) > 1e-6);            // J5 动了
        CHECK(fabs(out[3] - ref[3]) < 1e-9);            // J4 没动
        CHECK(fabs(out[5] - ref[5]) < 1e-9);            // J6 没动
        CHECK(fabs(out[0] - ref[0]) < 1e-9);
        CHECK(fabs(out[1] - ref[1]) < 1e-9);
        CHECK(fabs(out[2] - ref[2]) < 1e-9);
    }

    // (c) 自转：笔杆 Ry +10 ⇒ 只有 J6 动
    {
        const double cur[3] = {0, +10.0, 0};
        button2JointTarget(ref, refS, cur, out);
        CHECK(fabs(out[5] - ref[5]) > 1e-6);            // J6 动了
        CHECK(fabs(out[3] - ref[3]) < 1e-9);            // J4 没动
        CHECK(fabs(out[4] - ref[4]) < 1e-9);            // J5 没动
        CHECK(fabs(out[0] - ref[0]) < 1e-9);
        CHECK(fabs(out[1] - ref[1]) < 1e-9);
        CHECK(fabs(out[2] - ref[2]) < 1e-9);
    }
    PASS();
}

// ② 笔杆不动 ⇒ 逐位等于参照（六个关节都不动）
//    ⚠ 用【非零参照笔杆】: refS = cur 而 refS 全 0 时，"偏移为 0"与"根本没读笔杆"两种
//      实现长得一样。非零参照下后者会算出非零偏移 ⇒ 能红。
static void test_no_motion_returns_reference() {
    TEST(no_motion_returns_reference);
    const double ref[6] = {-30.5, 12.25, 0.0, 88.125, -140.0, 179.5};
    const double refS[3] = {-20.0, 12.0, 30.0};
    double out[6] = {0, 0, 0, 0, 0, 0};
    button2JointTarget(ref, refS, refS, out);
    for (int i = 0; i < 6; i++) {
        CHECK(fabs(out[i] - ref[i]) < 1e-12);
    }
    PASS();
}

// ③ 编译期：签名里没有位置入参（见上面的函数指针钉子）+ 运行期走一遍那个指针。
//    运行期这一半只是让钉子"被用掉"（否则它是没人碰的声明）；它也在功能上覆盖
//    "笔杆只转不平移"的马虎版：调用只有姿态两个数组，没有第三个位置数组可传。
static void test_signature_has_no_position_input() {
    TEST(signature_has_no_position_input);
    const double ref[6] = {1, 2, 3, 4, 5, 6};
    const double refS[3] = {0, 0, 0};
    const double cur[3] = {+10.0, 0, 0};
    double viaPin[6] = {0, 0, 0, 0, 0, 0}, viaDirect[6] = {0, 0, 0, 0, 0, 0};
    g_pinnedSignature(ref, refS, cur, viaPin);          // 经"四参数"类型调用
    button2JointTarget(ref, refS, cur, viaDirect);
    for (int i = 0; i < 6; i++) {
        CHECK(fabs(viaPin[i] - viaDirect[i]) < 1e-12);
    }
    PASS();
}

// ④ 偏移限幅：笔杆转 170°（> ORIENT_MAX_OFFSET_DEG = 150）⇒ 关节增量**恰为** 150°。
//    ⚠ 符号按 Config::BTN2_J4_SIGN 表达（Task 3 改符号后本用例仍应绿）。
//    ⚠ 这条断的是【关节增量】，不是末端姿态的任何量 —— 常数在这里换了语义，
//      见 Button2Joint.h「单位变了」那一段。
static void test_oversized_stylus_rotation_is_clamped_per_joint() {
    TEST(oversized_stylus_rotation_is_clamped_per_joint);
    const double ref[6] = {0, 0, 0, 40, 50, 60};
    const double refS[3] = {0, 0, 0};
    double out[6];

    // (a) 前后摆 170° ⇒ J4 增量 = 符号 × 150
    {
        const double cur[3] = {+170.0, 0, 0};
        button2JointTarget(ref, refS, cur, out);
        CHECK(fabs(out[3] - ref[3]) > 1e-6);
        CHECK(fabs(fabs(out[3] - ref[3]) - Config::ORIENT_MAX_OFFSET_DEG) < 1e-9);
        CHECK(fabs((out[3] - ref[3]) - Config::BTN2_J4_SIGN * Config::ORIENT_MAX_OFFSET_DEG) < 1e-9);
        CHECK(fabs(out[4] - ref[4]) < 1e-9);            // 限幅只碰它自己那一路
        CHECK(fabs(out[5] - ref[5]) < 1e-9);
    }

    // (b) 反向：前后摆 −170° ⇒ J4 增量 = 符号 × (−150)
    {
        const double cur[3] = {-170.0, 0, 0};
        button2JointTarget(ref, refS, cur, out);
        CHECK(fabs((out[3] - ref[3]) + Config::BTN2_J4_SIGN * Config::ORIENT_MAX_OFFSET_DEG) < 1e-9);
    }

    // (c) 三个轴同时超限 ⇒ 三路各自夹到 150（逐关节夹，不是合成量）
    {
        const double cur[3] = {+170.0, -170.0, +170.0};
        button2JointTarget(ref, refS, cur, out);
        CHECK(fabs(fabs(out[3] - ref[3]) - Config::ORIENT_MAX_OFFSET_DEG) < 1e-9);   // Rx -> J4
        CHECK(fabs(fabs(out[4] - ref[4]) - Config::ORIENT_MAX_OFFSET_DEG) < 1e-9);   // Rz -> J5
        CHECK(fabs(fabs(out[5] - ref[5]) - Config::ORIENT_MAX_OFFSET_DEG) < 1e-9);   // Ry -> J6
    }
    PASS();
}

// ⑤ 死区：小于 ORIENT_DEADZONE_DEG 的偏移 ⇒ 六个关节**逐位不变**；
//      而**恰好等于**门限 ⇒ 放行（`>=` 语义，与 RelayCore 的 axisGate 逐字一致）。
//    ⚠ 后半条是【边界判据】：只测"0.01 不动"的话，把门限写成 `>`（即恰好等于时被吞掉）
//      这个错**测不出来**，而它与现有实现语义不符 ⇒ 那正是实现与 RelayCore 分家的地方。
static void test_deadzone_swallows_tiny_offset_and_keeps_the_boundary() {
    TEST(deadzone_swallows_tiny_offset_and_keeps_the_boundary);
    const double ref[6] = {10, 20, 30, 40, 50, 60};
    const double refS[3] = {0, 0, 0};
    const double dz = Config::ORIENT_DEADZONE_DEG;
    double out[6];

    // (a) 三轴各偏 0.01°（< 0.05）⇒ 逐位不变
    {
        const double cur[3] = {+0.01, -0.01, +0.01};
        button2JointTarget(ref, refS, cur, out);
        for (int i = 0; i < 6; i++) {
            CHECK(fabs(out[i] - ref[i]) < 1e-12);
        }
    }

    // (b) 自检：0.01° 确实【小于】门限（否则 (a) 什么都没证明）
    CHECK(0.01 < dz);

    // (c) 恰好等于门限 ⇒ 放行，且增量恰为 符号×（那一根源轴的偏移）
    //     ⚠ 三根源轴**取不同正负**（Rx +dz / Ry −dz / Rz +dz）：这样"哪根源轴接到哪个关节"
    //       在这一条里也是可区分的 —— 若 Rz 与 Ry 被接反，cur[1]≠cur[2] ⇒ 符号对不上 ⇒ 红。
    {
        const double cur[3] = {+dz, -dz, +dz};      // cur[0]=Rx, cur[1]=Ry, cur[2]=Rz
        button2JointTarget(ref, refS, cur, out);
        CHECK(fabs((out[3] - ref[3]) - Config::BTN2_J4_SIGN * (+dz)) < 1e-12);   // Rx -> J4
        CHECK(fabs((out[4] - ref[4]) - Config::BTN2_J5_SIGN * (+dz)) < 1e-12);   // Rz -> J5
        CHECK(fabs((out[5] - ref[5]) - Config::BTN2_J6_SIGN * (-dz)) < 1e-12);   // Ry -> J6
    }
    PASS();
}

// ⑥ 【实现者追加】NaN/Inf 守卫 —— 覆盖【两处入参】× 三个分量 × {NaN, Inf}，
//    外加一条：`refJoints` 自己非有限时**原样传出去**（钉住契约的准确边界，
//    与 Button2Joint.h 的 NaN 那一段逐句对应）。
//    【为什么非补不可】没有这条用例时，"守卫到底覆盖了哪些位置"只能靠读代码；
//      而 `refJoints` 非有限这一支**最容易写错成"也退回去"**（那是自相矛盾的兜底：
//      退回去的还是同一个 NaN）⇒ 必须把**现状**钉住，且注明它是刻意的。
static void test_nonfinite_guard_covers_all_argument_positions() {
    TEST(nonfinite_guard_covers_all_argument_positions);
    const double nan_v = std::numeric_limits<double>::quiet_NaN();
    const double inf_v = std::numeric_limits<double>::infinity();
    const double ref[6] = {10, 20, 30, 40, 50, 60};
    const double refS[3] = {-20.0, 12.0, 30.0};
    const double cur[3] = {-10.0, 20.0, 30.0};
    double out[6];

    // (a)(b) refStylus / curStylus 非有限 ⇒ 六个关节【逐位退回参照】（= 臂原地保持）
    for (int arg = 0; arg < 2; arg++) {
        for (int axis = 0; axis < 3; axis++) {
            for (int kind = 0; kind < 2; kind++) {
                double a1[3] = {refS[0], refS[1], refS[2]};
                double a2[3] = {cur[0],  cur[1],  cur[2]};
                double* injected = (arg == 0) ? a1 : a2;
                injected[axis] = (kind == 0 ? nan_v : inf_v);
                button2JointTarget(ref, a1, a2, out);
                for (int i = 0; i < 6; i++) {
                    CHECK(fabs(out[i] - ref[i]) < 1e-12);
                }
            }
        }
    }

    // (c) refJoints 非有限 ⇒ **原样传到同一个位置**（本函数不保证返回值有限；刻意的现状）
    for (int axis = 0; axis < 6; axis++) {
        for (int kind = 0; kind < 2; kind++) {
            double a0[6] = {ref[0], ref[1], ref[2], ref[3], ref[4], ref[5]};
            a0[axis] = (kind == 0 ? nan_v : inf_v);
            button2JointTarget(a0, refS, cur, out);
            CHECK(!std::isfinite(out[axis]));           // 那个非有限值还在【同一个位置】
        }
    }
    PASS();
}

// ⑦ 【实现者追加】三轴同时动 + 大角度 ⇒ 各走各的，**没有交叉耦合、没有大角度误差**。
//    【为什么非补不可】① 是一次只动一根轴，它对"某两轴交换了影响"之外的错误有分辨力，
//      但**测不出**"三轴同时偏时互相串扰"这一类（例如把三个增量加在同一个关节上、
//      或先合成再分解）。而旧路（末端 RPY）正是**大角度下才翻车**（90° 摆幅差 149°）。
//      新路是逐轴 1:1 相加，**大角度不引入任何误差** —— 这条用例把这个性质写死，
//      也顺带把"新路绕开了整层姿态算术"这件事变成可执行的证据。
static void test_three_axes_at_once_are_independent_at_large_angles() {
    TEST(three_axes_at_once_are_independent_at_large_angles);
    const double ref[6] = {-173.0, -22.0, -118.0, 100.0, -100.0, 179.0};
    const double refS[3] = {-20.0, 12.0, 30.0};
    // 三轴同时偏，且都远大于死区、都小于限幅（此处 120° 上限内）
    const double dRx = +120.0, dRy = -60.0, dRz = +45.0;
    const double cur[3] = {refS[0] + dRx, refS[1] + dRy, refS[2] + dRz};
    double out[6];
    button2JointTarget(ref, refS, cur, out);
    CHECK(fabs((out[3] - ref[3]) - Config::BTN2_J4_SIGN * dRx) < 1e-9);   // 前后摆 -> J4
    CHECK(fabs((out[4] - ref[4]) - Config::BTN2_J5_SIGN * dRz) < 1e-9);   // 左右摆 -> J5
    CHECK(fabs((out[5] - ref[5]) - Config::BTN2_J6_SIGN * dRy) < 1e-9);   // 自转   -> J6
    CHECK(fabs(out[0] - ref[0]) < 1e-12);
    CHECK(fabs(out[1] - ref[1]) < 1e-12);
    CHECK(fabs(out[2] - ref[2]) < 1e-12);
    PASS();
}

int main() {
    std::cout << "--- Button2Joint (按钮2 关节空间映射：笔杆 Euler 增量 -> 关节增量) ---" << std::endl;
    test_each_stylus_axis_moves_exactly_one_joint();
    test_no_motion_returns_reference();
    test_signature_has_no_position_input();
    test_oversized_stylus_rotation_is_clamped_per_joint();
    test_deadzone_swallows_tiny_offset_and_keeps_the_boundary();
    test_nonfinite_guard_covers_all_argument_positions();
    test_three_axes_at_once_are_independent_at_large_angles();
    std::cout << "\nResults: " << g_passed << " passed, " << g_failed << " failed" << std::endl;
    return g_failed == 0 ? 0 : 1;
}
