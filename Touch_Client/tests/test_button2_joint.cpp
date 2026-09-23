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
//      ⚠ 但"期望值不写死符号"不等于"符号常数没人管"——**恰恰相反**：所有期望值都经
//        这三个常数表达 ⇒ 它们被改成任何值都全绿（含 0.5，即把增益塞进符号位）。
//        ⇒ ⑨ 单独把这三个常数的**取值域**钉死在 {+1, −1}。两句话不矛盾，各管一层。
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

// ④ 偏移限幅：笔杆转过**上限的 2 倍**（> ORIENT_MAX_OFFSET_DEG）⇒ 关节增量**恰为**上限。
//    ⚠ 符号按 Config::BTN2_J4_SIGN 表达（Task 3 改符号后本用例仍应绿）。
//    ⚠ 这条断的是【关节增量】，不是末端姿态的任何量 —— 常数在这里换了语义，
//      见 Button2Joint.h「单位变了」那一段。
//    ⚠ 【输入从常数导出，别硬编码 170】原版写死 `+170.0`。那在"上限今天 = 150"时成立，
//      但**上限一变就红，而且红得像"钳位坏了"**（其实是常数被调大了）—— 又一处"常数是脆的"。
//      取 `2 × 上限`：既【一定】超限，又不必知道上限的现值（上限翻倍/减半都仍成立）。
//    ⚠ 断言仍写成"恰为 ORIENT_MAX_OFFSET_DEG"（**不是**"= 输入"）—— 后者会在"没夹"时
//      也红不了（输入 = 输出），等于对常数恒真，把这条变成空转。
static void test_oversized_stylus_rotation_is_clamped_per_joint() {
    TEST(oversized_stylus_rotation_is_clamped_per_joint);
    const double ref[6] = {0, 0, 0, 40, 50, 60};
    const double refS[3] = {0, 0, 0};
    const double overCap = 2.0 * Config::ORIENT_MAX_OFFSET_DEG;   // 一定超过上限
    double out[6];

    // 自检：输入确实**超过**上限。上限若被改成 0 或负数，这一句先红 ——
    //   否则下面三条会因为"根本没超"而变成空转（2×0 = 0 不超 0）。
    CHECK(overCap > Config::ORIENT_MAX_OFFSET_DEG);

    // (a) 前后摆 +2×上限 ⇒ J4 增量 = 符号 × 上限
    {
        const double cur[3] = {+overCap, 0, 0};
        button2JointTarget(ref, refS, cur, out);
        CHECK(fabs(out[3] - ref[3]) > 1e-6);
        CHECK(fabs(fabs(out[3] - ref[3]) - Config::ORIENT_MAX_OFFSET_DEG) < 1e-9);
        CHECK(fabs((out[3] - ref[3]) - Config::BTN2_J4_SIGN * Config::ORIENT_MAX_OFFSET_DEG) < 1e-9);
        CHECK(fabs(out[4] - ref[4]) < 1e-9);            // 限幅只碰它自己那一路
        CHECK(fabs(out[5] - ref[5]) < 1e-9);
    }

    // (b) 反向：前后摆 −2×上限 ⇒ J4 增量 = 符号 × (−上限)
    {
        const double cur[3] = {-overCap, 0, 0};
        button2JointTarget(ref, refS, cur, out);
        CHECK(fabs((out[3] - ref[3]) + Config::BTN2_J4_SIGN * Config::ORIENT_MAX_OFFSET_DEG) < 1e-9);
    }

    // (c) 三个轴同时超限 ⇒ 三路各自夹到上限（逐关节夹，不是合成量）
    {
        const double cur[3] = {+overCap, -overCap, +overCap};
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

    // (c) refJoints 非有限 ⇒ **原样传到同一个位置**，且**只影响那一路** ——
    //     其余五路必须【有限】且【逐位等于"同等入参、refJoints 全有限"时的结果】。
    //    ⚠ 原版只断言被注入那一路 `!isfinite` ⇒ "**把六路全写 NaN**"照样全绿（复审实测）。
    //    ⚠ 复审给的修法原文是"其余五路 …… `== ref[i]`"。这里**没有逐字照写**，因为
    //      在本用例的入参下那句对 J4/J6 **恒不成立**：`refS ≠ cur`（(a)(b) 刻意如此 ⇒
    //      偏移非零），正常路径上 out[3] = ref[3] + SIGN·(cur[0]−refS[0])、out[5] 同理，
    //      它们本来就 ≠ ref[i] ⇒ 照写会让**正确实现恒红**（那会把测试变成新的假红源）。
    //      ⇒ 改断"逐位等于基线"。判别力与复审的意图相同、且更强：基线对
    //        「六路全 NaN」与「非有限时退回参照」**两种**错法都红（后者下 out[3] 会退回 40）。
    double base[6];
    button2JointTarget(ref, refS, cur, base);           // 同等入参、refJoints 全有限
    for (int axis = 0; axis < 6; axis++) {
        for (int kind = 0; kind < 2; kind++) {
            double a0[6] = {ref[0], ref[1], ref[2], ref[3], ref[4], ref[5]};
            a0[axis] = (kind == 0 ? nan_v : inf_v);
            button2JointTarget(a0, refS, cur, out);
            for (int i = 0; i < 6; i++) {
                if (i == axis) {
                    CHECK(!std::isfinite(out[i]));      // 那个非有限值还在【同一个位置】
                } else {
                    CHECK(std::isfinite(out[i]));       // 其余五路【有限】
                    CHECK(out[i] == base[i]);           // 且**逐位**等于基线（没被污染）
                }
            }
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

// ⑧ 【复审追加】死区是【逐轴】的，不是按笔杆偏移的**矢量模长**聚合的。
//    【为什么非补不可】①⑤⑦ 里三个轴的偏移**要么全过门、要么全不过门**（⑤(a) 全不过、
//      ⑤(c) 全恰好过、①⑦ 全远过）⇒ 把 `axisGate` 换成
//      `sqrt(dRx²+dRy²+dRz²) >= dz` 这种**聚合**门，整个套件仍然全绿（复审实测 7/7 绿）。
//      聚合门的后果不是"抖"而是**漏门**：一根轴远越界时（+5.0），它会把另一根只有
//      0.04° 的轴**一起放行** —— 而那 0.04° 正是设计上要吞掉的那一档（⑤(a) 判的就是它）。
//    【这条怎么区分】★ 必须在**同一次调用**里两件事同时出现：
//      一根轴远越界（+5.0 ⇒ J4 应动 SIGN×5.0）、另一根轴**刚好在门限之下**
//      （0.8×dz ⇒ J5 应**逐位**等于参照）。逐轴门下两句都成立；聚合门下一句红（J5 拿到 0.04）。
//      ⚠ 拆成两次调用就区分不了 —— 那正是 ⑤ 已有的形状，也正是它漏掉这个错的原因。
static void test_deadzone_is_per_axis_not_by_magnitude() {
    TEST(deadzone_is_per_axis_not_by_magnitude);
    const double ref[6] = {11.0, 22.0, 33.0, 44.0, 55.0, 66.0};
    const double refS[3] = {0, 0, 0};
    const double dz = Config::ORIENT_DEADZONE_DEG;
    // 门限**之下**但**非零**，且从常数导出（写死 0.04 会在 dz 被调小时变成恒真 —— 见 ④ 的注释）
    const double under = 0.8 * dz;
    double out[6];

    // 自检：这一条的三个前提本身要成立，否则它什么都没证明
    CHECK(dz > 0.0);                       // 门限为正 ⇒ under > 0 且 "5.0 ≥ dz" 才有意义
    CHECK(under > 0.0);                    // ★ 非零 —— 聚合门才"有东西可以顺便放行"
    CHECK(under < dz);                     // ★ 确实在门限【之下】
    CHECK(5.0 >= dz);                      // 另一根轴确实【远】在门限之上
    CHECK(5.0 < Config::ORIENT_MAX_OFFSET_DEG);   // 且不被限幅吃掉 ⇒ 差值应当【恰为】5.0

    // 一次调用：cur[0]=Rx=+5.0（远越界 ⇒ J4）· cur[1]=Ry=0（⇒ J6）· cur[2]=Rz=+under（门限之下 ⇒ J5）
    const double cur[3] = {+5.0, 0.0, +under};
    button2JointTarget(ref, refS, cur, out);

    // (a) 远越界那一根：逐轴门与聚合门**都**放行 ⇒ 单看它区分不了两种门（幅度也钉住：±1e-6）
    CHECK(fabs((out[3] - ref[3]) - Config::BTN2_J4_SIGN * 5.0) < 1e-6);
    // (b) ★ 判别力所在：门限之下那一根必须**逐位**等于参照。
    //     用 `==` 而不是 `< 1e-9`：0.04 一旦被放行，差值就是 0.04（远大于 1e-9），
    //     但逐位相等是这里真正的契约（没放行就是原样搬参照，不该有任何浮点痕迹）。
    CHECK(out[4] == ref[4]);               // Rz -> J5：0.04 被吞掉
    // (c) 第三根是 0，且三根之间没有串扰；J1~J3 逐位不变
    CHECK(out[5] == ref[5]);
    CHECK(out[0] == ref[0]);
    CHECK(out[1] == ref[1]);
    CHECK(out[2] == ref[2]);
    PASS();
}

// ⑨ 【复审追加】`BTN2_J4/J5/J6_SIGN` 必须是**符号**，不是增益。
//    【为什么非补不可】本文件所有期望值都**经由这三个常数**写（刻意的，见文件头 ②：
//      Task 3 改符号不该让用例红）。代价是：把某个常数改成 `0.5`（= 把**增益**塞进符号位）
//      整套用例**照样全绿** —— 因为 `out[3]−ref[3] == SIGN·Δ` 这句里 SIGN 出现在两边，
//      它就是实现本身，对 SIGN 的任何取值都成立。
//    ⚠★ 而这正是**计划 Task 3 最可能犯的错**：它的活就是"加逐轴增益"，最容易顺手写进
//      这三个常数里（Button2Joint.h 的「本函数不施加逐轴增益」那一段正是为此写的）。
//      ⇒ 需要一条【不经过实现】的断言，把这三个常数的**取值域**钉死在 {+1, −1}。
static void test_sign_constants_are_unit_signs() {
    TEST(sign_constants_are_unit_signs);
    // 误差 1e-12（与复审给的判据一致）。写成 `fabs(fabs(x) − 1.0) < 1e-12` 而不是
    //   `x == 1.0 || x == -1.0`：后者只认字面量写法，前者认"数值上等于 ±1"这个语义。
    CHECK(fabs(fabs(Config::BTN2_J4_SIGN) - 1.0) < 1e-12);
    CHECK(fabs(fabs(Config::BTN2_J5_SIGN) - 1.0) < 1e-12);
    CHECK(fabs(fabs(Config::BTN2_J6_SIGN) - 1.0) < 1e-12);
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
    test_deadzone_is_per_axis_not_by_magnitude();       // ⑧ 复审追加
    test_sign_constants_are_unit_signs();               // ⑨ 复审追加
    std::cout << "\nResults: " << g_passed << " passed, " << g_failed << " failed" << std::endl;
    return g_failed == 0 ? 0 : 1;
}
