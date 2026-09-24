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
//
// ⚠★ 2026-09-23 fix2 订正（**原文是错的，别按它读**）：原文写"本套件只链接
//   `../relay/Button2Joint.cpp` 一个翻译单元，且**不需要**任何 /I 路径"。
//   抽出 `isTrustworthyJointRef` 之后它要用 `Kinematics::isWithinJointLimits`，而那条链会走到
//   `<HDU/hduVector.h>` ⇒ 本套件**多链一个 `../robot/Kinematics.cpp`，并且需要两个 /I 路径**。
//   ⚠ 变的只是**构建图的形状**，函数本身仍然纯（Kinematics 的 FK/IK 也是纯算术，不碰 Win32）；
//     即"不需要 /I"这句话现在**只在'不碰 OpenHaptics 调用'那个意义下**还成立。
//     理由与完整的依赖链写在 `build_button2_joint_test.bat` 的注释里（那里是改的时候第一个要读的）。
// ⚠ `#include "../robot/Kinematics.h"` 是给 ⑫ 用的（期望值从限位常数**导出**，不写死数字）——
//   它同样是编译期依赖，同样需要那两个 /I。

#include <iostream>
#include <cmath>
#include <limits>      // ⑥ 用 quiet_NaN / infinity

#include "../relay/Button2Joint.h"
#include "../config/Config.h"
#include "../robot/Kinematics.h"   // ⑫：关节限位常数（给 isTrustworthyJointRef 导出期望值）
// ★ 2026-09-24：⑦ 与 ⑱ 要用 `TcpCalibration::rpyToMatrix` **造输入**（把"绕器件某轴转 δ"
//   翻成欧拉三元组）。它也是欧拉约定的唯一真相源 ⇒ 测试侧不复写一份约定。
//   ⚠ 这也是构建脚本要多链 `../calibration/TcpCalibration.cpp` 的原因。
#include "../calibration/TcpCalibration.h"   // ⑫：关节限位常数（给 isTrustworthyJointRef 导出期望值）

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
// ===== 测试侧的【输入构造器】（不是被测逻辑）=====
// 造出"在 ref 姿态上再绕【器件某根轴】转 deg"所对应的欧拉三元组。
// ⚠ 它只用到 `TcpCalibration::rpyToMatrix`（欧拉约定的唯一真相源）+ 一个绕单轴的
//   旋转矩阵 + 一个矩阵→ZYX 欧拉的反解。**被测那一段（ΔR → 旋转向量 → 关节）不在这里。**
// ⚠ 它自带一个可复核的前提：调用方可以用 rpyToMatrix 把造出来的三元组还原成矩阵、与
//   `R_ref·R_axis(δ)` 对比（⑦ 与 ⑱ 都这么自检）—— 构造器错了会让下游用例变成空转，
//   所以"构造成功"这件事必须被断言，不能只靠"我看着对"。
static void bodyAxisMatrix(int axis, double deg, double R[9]) {
    const double k = 3.14159265358979323846 / 180.0;
    const double a = deg * k, ca = cos(a), sa = sin(a);
    for (int i = 0; i < 9; ++i) R[i] = 0.0;
    if (axis == 0)      { R[0] = 1; R[4] = ca; R[5] = -sa; R[7] = sa;  R[8] = ca; }
    else if (axis == 1) { R[0] = ca; R[2] = sa; R[4] = 1; R[6] = -sa; R[8] = ca; }
    else                { R[0] = ca; R[1] = -sa; R[3] = sa; R[4] = ca; R[8] = 1; }
}

static void matMul3(const double A[9], const double B[9], double out[9]) {
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) {
            double s = 0.0;
            for (int k = 0; k < 3; ++k) s += A[r * 3 + k] * B[k * 3 + c];
            out[r * 3 + c] = s;
        }
}

// 矩阵 → ZYX 欧拉（度）。与 HapticCallback 的提取式同构（那里是列主序下标）。
static void matToZyRpy(const double R[9], double rpy[3]) {
    double sy = -R[6];
    if (sy > 1.0) sy = 1.0;
    if (sy < -1.0) sy = -1.0;
    const double ry = asin(sy);
    double rx, rz;
    if (fabs(cos(ry)) > 1e-9) { rx = atan2(R[7], R[8]); rz = atan2(R[3], R[0]); }
    else                      { rx = atan2(-R[7], R[4]); rz = 0.0; }
    const double k = 180.0 / 3.14159265358979323846;
    rpy[0] = rx * k; rpy[1] = ry * k; rpy[2] = rz * k;
}

static void bodyAxisEuler(const double refRpy[3], int axis, double deg, double outRpy[3]) {
    double Rref[9], Rax[9], Rc[9];
    TcpCalibration::rpyToMatrix(refRpy[0], refRpy[1], refRpy[2], Rref);
    bodyAxisMatrix(axis, deg, Rax);
    matMul3(Rref, Rax, Rc);          // Rc = R_ref · R_axis(deg)
    matToZyRpy(Rc, outRpy);
}



// ① ★ 结构判据（本方案的核心，且与符号无关）：一次只动笔杆一根轴 ⇒ 恰好一个关节动。
//    三条都写全，因为"只查一条"放过的是整个映射表：对而对、或两路对调的写法，
//    都能在只查一条时全绿。
//    ⚠★ 2026-09-24：**源轴换成了"器件轴"**。在 refS = 全 0（= 单位矩阵）时，
//      `cur = (δ,0,0)` ⇒ R_cur = Rx(δ) ⇒ rv = (δ,0,0) ⇒ **器件 X**；
//      `cur = (0,δ,0)` ⇒ Ry(δ) ⇒ **器件 Y**；`cur = (0,0,δ)` ⇒ Rz(δ) ⇒ **器件 Z**。
//      ⇒ 与从前"逐欧拉差(Rx→J4 · Rz→J5 · Ry→J6)"相比，**Y 与 Z 两路的期望互换了** ——
//        这是刻意的规格变更（现场实测：左右摆=器件 Y、自转=器件 Z），不是笔误。
static void test_each_stylus_axis_moves_exactly_one_joint() {
    TEST(each_stylus_axis_moves_exactly_one_joint);
    const double ref[6] = {10, 20, 30, 40, 50, 60};
    const double refS[3] = {0, 0, 0};
    double out[6];

    // (a) 前后摆 = 绕【器件 X】转 +10 ⇒ 只有 J4 动（这一路从前就是对的：
    //     R·Rx(δ) 只是把最内层那个欧拉角加 δ ⇒ 绕器件 X ≡ ΔRx 是恒等式）
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

    // (b) 左右摆 = 绕【器件 Y】转 −10 ⇒ 只有 J5 动
    {
        const double cur[3] = {0, -10.0, 0};
        button2JointTarget(ref, refS, cur, out);
        CHECK(fabs(out[4] - ref[4]) > 1e-6);            // J5 动了
        CHECK(fabs(out[3] - ref[3]) < 1e-9);            // J4 没动
        CHECK(fabs(out[5] - ref[5]) < 1e-9);            // J6 没动
        CHECK(fabs(out[0] - ref[0]) < 1e-9);
        CHECK(fabs(out[1] - ref[1]) < 1e-9);
        CHECK(fabs(out[2] - ref[2]) < 1e-9);
    }

    // (c) 自转 = 绕【器件 Z】转 +10 ⇒ 只有 J6 动
    {
        const double cur[3] = {0, 0, +10.0};
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

// ④ 偏移限幅：笔杆转过的角度**超过上限** ⇒ 关节增量**恰为**上限。
//    ⚠★ 2026-09-24 换了实现之后，本用例的**输入构造**必须跟着换：
//      真 ΔR 的旋转向量**量程天然 ≤180°**（取的是主值旋转）⇒ 原版那句
//      `overCap = 2 × 上限 = 300°` 会被折成 −60°，**根本超不了限** ⇒ 用例会变成假红。
//      现在取 `θ = (180 + 上限)/2`：在 上限 < 180 的前提下它【一定】> 上限且 ≤ 180。
//      ⚠ 前提 `上限 < 180` 由下面的自检钉住；上限若被提到 ≥180，这条用例**本就不可构造**，
//        那时该做的是重新设计这条用例，而不是让它悄悄变绿。
//    ⚠ 断言仍写成"恰为 ORIENT_MAX_OFFSET_DEG"（不是"= 输入"）—— 后者在"没夹"时也红不了。
static void test_oversized_stylus_rotation_is_clamped_per_joint() {
    TEST(oversized_stylus_rotation_is_clamped_per_joint);
    const double ref[6] = {0, 0, 0, 40, 50, 60};
    const double refS[3] = {0, 0, 0};
    const double cap = Config::ORIENT_MAX_OFFSET_DEG;
    const double overCap = (180.0 + cap) * 0.5;      // 一定 > cap，且 <= 180
    double out[6];

    CHECK(cap < 180.0);                              // ★ 前提：否则"超限"不可构造
    CHECK(overCap > cap);                            // 输入确实超过上限
    CHECK(overCap <= 180.0 + 1e-9);                  // 且仍在旋转向量的量程内

    // (a) 器件 X 转过 overCap ⇒ J4 增量 = 符号 × 上限
    {
        const double cur[3] = {+overCap, 0, 0};
        button2JointTarget(ref, refS, cur, out);
        CHECK(fabs(out[3] - ref[3]) > 1e-6);
        CHECK(fabs(fabs(out[3] - ref[3]) - cap) < 1e-6);
        CHECK(fabs((out[3] - ref[3]) - Config::BTN2_J4_SIGN * cap) < 1e-6);
        CHECK(fabs(out[4] - ref[4]) < 1e-9);            // 限幅只碰它自己那一路
        CHECK(fabs(out[5] - ref[5]) < 1e-9);
    }

    // (b) 反向 ⇒ J4 增量 = 符号 × (−上限)
    {
        const double cur[3] = {-overCap, 0, 0};
        button2JointTarget(ref, refS, cur, out);
        CHECK(fabs((out[3] - ref[3]) + Config::BTN2_J4_SIGN * cap) < 1e-6);
    }

    // (c) 器件 Y 与 Z 两路同样夹住（各自单独来一遍 —— 旋转向量**不可能**让三轴同时超限，
    //     因为 |rv| ≤ 180 而 3×cap 已 > 180 ⇒ 原版那句"三轴同时超限"在新语义下不可构造）。
    {
        const double curY[3] = {0, -overCap, 0};
        button2JointTarget(ref, refS, curY, out);
        CHECK(fabs(fabs(out[4] - ref[4]) - cap) < 1e-6);
        CHECK(fabs(out[3] - ref[3]) < 1e-9);
        const double curZ[3] = {0, 0, +overCap};
        button2JointTarget(ref, refS, curZ, out);
        CHECK(fabs(fabs(out[5] - ref[5]) - cap) < 1e-6);
        CHECK(fabs(out[4] - ref[4]) < 1e-9);
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

    // (c) 恰好等于门限 ⇒ 放行，且**单轴**时增量恰为 符号×偏移（单轴是精确的：
    //     refS 全 0、cur 只有一维 ⇒ R_cur = R_axis(dz) ⇒ rv 就只有那一维 = dz）。
    {
        const double cur[3] = {+dz, 0, 0};
        button2JointTarget(ref, refS, cur, out);
        CHECK(fabs((out[3] - ref[3]) - Config::BTN2_J4_SIGN * (+dz)) < 1e-12);   // 器件 X -> J4
    }
    {
        const double cur[3] = {0, -dz, 0};
        button2JointTarget(ref, refS, cur, out);
        CHECK(fabs((out[4] - ref[4]) - Config::BTN2_J5_SIGN * (-dz)) < 1e-12);   // 器件 Y -> J5
    }
    {
        const double cur[3] = {0, 0, +dz};
        button2JointTarget(ref, refS, cur, out);
        CHECK(fabs((out[5] - ref[5]) - Config::BTN2_J6_SIGN * (+dz)) < 1e-12);   // 器件 Z -> J6
    }

    // (d) 三轴同时、取不同正负、都**远高于**门限 ⇒ 三路都放行，且各自落在**正确的一侧**。
    //     ⚠ 这里【不能】断言精确值，也【不能】把三轴取在门限上：
    //       三个欧拉角同时动时，真 ΔR 的旋转向量与 (ΔRx,ΔRy,ΔRz) 相差 O(θ²) ——
    //       θ = 10° 时每个分量可差 ~1°。而 θ = dz = 0.05° 时那一项虽只有 1e-4 量级，
    //       却足以把某个分量**推到门限之下** ⇒ 被门吞掉、期望值恒不成立（2026-09-24 实测）。
    //       ⇒ "恰好等于门限"这个边界由 (c) 用**单轴**精确钉住（单轴时 rv 就是输入本身）。
    //     ⚠ 本条仍有判别力：若 Y 与 Z 两路被接反，下面两个符号断言会同时红。
    {
        const double dbig = 10.0;
        const double cur[3] = {+dbig, -dbig, +dbig};
        button2JointTarget(ref, refS, cur, out);
        const double d3 = out[3] - ref[3], d4 = out[4] - ref[4], d5 = out[5] - ref[5];
        CHECK(d3 * Config::BTN2_J4_SIGN > 0.0);        // 器件 X 正向 ⇒ J4 落在 SIGN 那一侧
        CHECK(d4 * Config::BTN2_J5_SIGN < 0.0);        // 器件 Y 负向
        CHECK(d5 * Config::BTN2_J6_SIGN > 0.0);        // 器件 Z 正向
        CHECK(fabs(fabs(d3) - dbig) < 3.0);            // 量级同阶（容差见上）
        CHECK(fabs(fabs(d4) - dbig) < 3.0);
        CHECK(fabs(fabs(d5) - dbig) < 3.0);
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

// ⑦ ★★ 2026-09-24 重写：**倾斜参照下的三根器件轴**（这是现场那个症状的回归用例）。
//    【为什么换掉原来那条】旧版的期望值是"欧拉角差 ⇒ 关节"的逐分量等式 —— 那正是被推翻的语义
//      （`cur = refS + (Δx,Δy,Δz)` 在欧拉图上加减，与"绕器件某轴转"不是一回事）。
//    【现在断什么】把参照放在**多轴倾斜**的姿态（现场那个按下姿态），然后**一次只绕一根器件轴**转：
//        ΔR = R_ref^T · (R_ref · R_axis(δ)) = R_axis(δ)  ⇒ 旋转向量**恰好**只有那一维
//      ⇒ 期望值可以写成【解析解】：那一根轴转 δ ⇒ 那一个关节转 SIGN·δ，另两个**逐位不动**。
//    ★ 判别力：旧实现在这个姿态下把"自转 10°"送成 J4 +1.52 / J5 +5.48 / J6 +8.50（离线复算），
//      本用例对它**必红** —— 它不是"换个写法", 它是把现场症状钉住。
//    ⚠ 输入用本地的 bodyAxisEuler() 构造（测试侧的输入构造器），并用 rpyToMatrix 复核构造成功。
static void test_three_axes_at_once_are_independent_at_large_angles() {
    TEST(three_axes_at_once_are_independent_at_large_angles);
    const double ref[6] = {-173.0, -22.0, -118.0, 100.0, -100.0, 179.0};
    // 现场那个按下姿态（2026-09-24 实测日志里的 burst 起点）
    const double refS[3] = {-58.93, 11.83, -6.41};
    const double deltas[3] = {+40.0, -30.0, +50.0};   // 都远大于死区、都小于限幅
    const int    jmap[3]   = {3, 4, 5};               // 器件 X/Y/Z -> J4/J5/J6
    const double signs[3]  = {Config::BTN2_J4_SIGN, Config::BTN2_J5_SIGN, Config::BTN2_J6_SIGN};
    double out[6];

    for (int axis = 0; axis < 3; axis++) {
        double cur[3];
        bodyAxisEuler(refS, axis, deltas[axis], cur);

        // 自检：构造出的欧拉三元组确实还原成 R_ref · R_axis(δ)（否则下面等于在空转）
        {
            double Ra[9], Rb[9], Rax[9], expect[9];
            TcpCalibration::rpyToMatrix(refS[0], refS[1], refS[2], Ra);
            TcpCalibration::rpyToMatrix(cur[0],  cur[1],  cur[2],  Rb);
            bodyAxisMatrix(axis, deltas[axis], Rax);
            matMul3(Ra, Rax, expect);
            double err = 0.0;
            for (int i = 0; i < 9; ++i) { const double d = Rb[i] - expect[i]; err += d * d; }
            CHECK(err < 1e-18);          // ★ 构造器自检（构造错了会让下面三条变成空转）
        }

        button2JointTarget(ref, refS, cur, out);
        for (int k = 0; k < 3; ++k) {
            const int j = jmap[k];
            if (k == axis) {
                CHECK(fabs((out[j] - ref[j]) - signs[k] * deltas[axis]) < 1e-6);
            } else {
                CHECK(fabs(out[j] - ref[j]) < 1e-9);     // 另两个关节**没动**
            }
        }
        CHECK(fabs(out[0] - ref[0]) < 1e-12);
        CHECK(fabs(out[1] - ref[1]) < 1e-12);
        CHECK(fabs(out[2] - ref[2]) < 1e-12);
    }
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
    // ⚠ 容差 1e-2 而【不是】1e-6：2026-09-24 换成真 ΔR 之后，输入里 X 与 Z 两轴混在一起，
    //   rv.x 与欧拉差 ΔRx 相差 O(5°×0.04°) ≈ 3.5e-3° ⇒ 1e-6 会恒红。
    //   判别力不在这句上：真正区分逐轴门与聚合门的是下面那句 `out[4] == ref[4]`。
    CHECK(fabs((out[3] - ref[3]) - Config::BTN2_J4_SIGN * 5.0) < 1e-2);
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

// ⑩ 【2026-09-23 fix2 追加】`isTrustworthyJointRef` 的**正常**输入必须被接受。
//    四条用例（⑩~⑬）合起来才是这个判据的完整规格：接受什么、拒绝什么、以及**故意不覆盖什么**。
//    【为什么非补不可】`RelayCore.cpp` 不被任何测试编译 ⇒ I1 那道门原来是"读 + 上机"。
//      而"抽出来"这件事的**全部价值**就在于这四条能跑。
//    ⚠ 本条的参照里**故意留了一个恰好 0 的关节**（J1/J5）：那是**合法**的关节角
//      （J1 归零是常态），而"六位**恰好**全 0"才是指纹 ⇒ 这条同时是"子句一不是按轴 OR 判的"
//      的判别器（把 `&&` 写成 `||` 会在这里红，见 ⑫）。
static void test_trustworthy_ref_is_accepted() {
    TEST(trustworthy_ref_is_accepted);
    // 一个含两个恰好 0 的**正常**参照：都在关节限位内（J3 的 ±155 是所有轴里最紧的）
    const double ref[6] = {0.0, -20.0, 30.0, 40.0, 0.0, 60.0};
    CHECK(isTrustworthyJointRef(ref));
    PASS();
}

// ⑪ 六位**恰好**全 0 ⇒ 拒绝（"`GetAngle()` 从未解析成功"的指纹）。
//    ⚠ 这是 I1 的**存在理由**：那种参照 FK 出来 = (0, −233.3, 756)，**过得了位置门**
//      ⇒ 不拒就是 `ServoJ(0,…,0)` 把臂开向模型零位。
static void test_all_zero_ref_is_rejected() {
    TEST(all_zero_ref_is_rejected);
    const double zeros[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    CHECK(!isTrustworthyJointRef(zeros));
    PASS();
}

// ⑫ 越关节限位 ⇒ 拒绝。**逐轴**各试一遍 ⇒ "只查了某一个下标"的实现会红。
//    ⚠ 期望值从 `Kinematics` 的限位常数**导出**（不写死 200）：限位表是唯一真相源，
//      写死数字会在限位被调整时让用例变成"红得像判据坏了"。
//    ⚠ 顺带钉住另一半：**恰好落在限位边界上**是**可信**的（`isWithinJointLimits` 用
//      `>` / `<`，是闭区间）—— 这一句是 ⑫ 与 ⑬ 之间的边界，别把它读成"越界"。
static void test_out_of_limits_ref_is_rejected() {
    TEST(out_of_limits_ref_is_rejected);
    const double limsMin[6] = {Kinematics::J1_MIN, Kinematics::J2_MIN, Kinematics::J3_MIN,
                               Kinematics::J4_MIN, Kinematics::J5_MIN, Kinematics::J6_MIN};
    const double limsMax[6] = {Kinematics::J1_MAX, Kinematics::J2_MAX, Kinematics::J3_MAX,
                               Kinematics::J4_MAX, Kinematics::J5_MAX, Kinematics::J6_MAX};
    const double ref[6] = {0.0, -20.0, 30.0, 40.0, 0.0, 60.0};

    for (int i = 0; i < 6; i++) {
        // 自检：本轴的行程非零，否则"上界 +1"既不越界也没意义（那条轴会变成空转）
        CHECK(limsMax[i] > limsMin[i]);
        {
            double over[6] = {ref[0], ref[1], ref[2], ref[3], ref[4], ref[5]};
            over[i] = limsMax[i] + 1.0;
            CHECK(!isTrustworthyJointRef(over));
        }
        {
            double under[6] = {ref[0], ref[1], ref[2], ref[3], ref[4], ref[5]};
            under[i] = limsMin[i] - 1.0;
            CHECK(!isTrustworthyJointRef(under));
        }
        // 恰好在边界上 ⇒ 可信（闭区间）
        {
            double atMax[6] = {ref[0], ref[1], ref[2], ref[3], ref[4], ref[5]};
            atMax[i] = limsMax[i];
            CHECK(isTrustworthyJointRef(atMax));
        }
    }
    PASS();
}

// ⑬ ★【边界用例】NaN 参照被本判据**接受** —— 断的是**现状**，而且它是**刻意**的现状。
//    【为什么非写不可（而不是"顺手修好"）】`isWithinJointLimits` 对 NaN 逐条比较全为 false
//      ⇒ 它**返回 true**（放行）；而"恰好全 0"对 NaN 也为假 ⇒ 本函数对 NaN 返回 **true**。
//      接住 NaN 的是**调用方的 FK 门**（FK(NaN) ⇒ NaN ⇒ 位置入口第一道守卫 REJECT），
//      那条**不在**本函数的职责内。
//      ⇒ 这条用例把"谁负责哪一段"写成可执行的：谁要在本函数里补一个 `isfinite` 守卫
//        （看起来像"顺手修好"），**这条用例立刻红** —— 而那次改动会拿走 FK 门那一层
//        与这里的分工，且**没有任何东西**在背面接着。见 Button2Joint.h 的 NaN 那一段。
//    ⚠ 与 ⑥ 的关系：⑥ 钉的是 `button2JointTarget`（笔杆或参照非有限 ⇒ 六个关节退回参照），
//      本条钉的是**另一个函数**的另一条契约 ⇒ 两者不重复。
static void test_nan_ref_is_accepted_by_this_predicate() {
    TEST(nan_ref_is_accepted_by_this_predicate);
    const double nan_v = std::numeric_limits<double>::quiet_NaN();
    // 全 NaN：`isWithinJointLimits` 逐条为 false ⇒ 放行；全 0 子句也为假 ⇒ 本判据判"可信"
    {
        const double allNan[6] = {nan_v, nan_v, nan_v, nan_v, nan_v, nan_v};
        CHECK(isTrustworthyJointRef(allNan));
    }
    // 只坏一路 ⇒ 同样放行（NaN 不会被"恰好全 0"或"越限位"任一条捞住）
    {
        const double oneNan[6] = {0.0, -20.0, nan_v, 40.0, 0.0, 60.0};
        CHECK(isTrustworthyJointRef(oneNan));
    }
    // ⚠ Inf **会**被捞住（`Inf > J*_MAX` 为真 ⇒ 越限位 ⇒ 拒绝）—— 与 NaN 不同，一并钉住，
    //   免得下一个人从"NaN 被放行"推出"非有限一律被放行"。
    {
        const double inf_v = std::numeric_limits<double>::infinity();
        const double oneInf[6] = {0.0, -20.0, inf_v, 40.0, 0.0, 60.0};
        CHECK(!isTrustworthyJointRef(oneInf));
    }
    PASS();
}

// ⑭ ★【判别力用例】五路恰好 0 + 第六路是极小非零 ⇒ **可信**（"六位**恰好**全 0"的边界）。
//    【为什么非补不可】把子句一的 `&&` 写成 `||`（"任一为 0 就判坏"）是个**很容易犯**的写法
//      （六个 `== 0.0` 的链子读起来像"有零就是坏"）；而它会让 ⑩（含两个 0）与 ⑪（全 0）
//      一起绿 —— ⑩⑪ 的组合**区分不了** `&&` 与 `||`。这条把边界钉在**恰好**上：
//      `||` 下 `0.001` 之外的五个 0 会让它被拒 ⇒ 红。
//    ⚠ 0.001° 是**合法**关节角（J1 恰好停在零点附近本就常见）⇒ 不是"太刁钻的输入"。
static void test_five_zeros_and_a_tiny_value_is_still_trustworthy() {
    TEST(five_zeros_and_a_tiny_value_is_still_trustworthy);
    const double nearly[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.001};
    CHECK(isTrustworthyJointRef(nearly));
    // 自检：那一位**确实非零**（否则本用例退化成 ⑪ 的翻版，什么都不区分）
    CHECK(nearly[5] != 0.0);
    PASS();
}

// ⑮ `clampJointStep`：偏移为 0 ⇒ **逐位不变**（且不许有浮点痕迹）。
//    语义上是"按下按钮2 的第一帧"：`prev` 被种子化成参照本身、期望也等于参照 ⇒ 本帧不走。
//    ⚠ 用 `==` 而不是 `< 1e-9`：契约是"没动就是原样搬"，不该有任何浮点痕迹。
static void test_clamp_step_zero_offset_changes_nothing() {
    TEST(clamp_step_zero_offset_changes_nothing);
    const double prev[6] = {-173.0, -22.0, -118.0, 100.0, -100.0, 179.0};
    double out[6] = {99, 99, 99, 99, 99, 99};
    clampJointStep(prev, prev, Config::ORIENT_MAX_STEP_DEG, out);
    for (int i = 0; i < 6; i++) CHECK(out[i] == prev[i]);
    PASS();
}

// ⑯ 单步限幅：大偏移下**逐轴**夹到 `maxStep`（不是夹合成量、不是永久截断成 maxStep）。
//    ⚠ 输入从常数导出（`4 × maxStep`），不写死 17.5 —— 与 ④ 同一个理由（常数是脆的）。
static void test_clamp_step_limits_a_large_offset_per_axis() {
    TEST(clamp_step_limits_a_large_offset_per_axis);
    const double step = Config::ORIENT_MAX_STEP_DEG;
    CHECK(step > 0.0);                       // 自检：负/零会让下面三句全部失去意义
    const double huge = 4.0 * step;          // 一定超过单步上限
    const double prev[6] = {10.0, 20.0, 30.0, 40.0, 50.0, 60.0};
    double out[6];

    // (a) 六轴**同向**各差 huge ⇒ 每轴恰好走一步，且**方向**是朝目标的（不是反向）
    {
        double desired[6];
        for (int i = 0; i < 6; i++) desired[i] = prev[i] + huge;
        clampJointStep(prev, desired, step, out);
        for (int i = 0; i < 6; i++) CHECK(fabs(out[i] - (prev[i] + step)) < 1e-12);
    }

    // (b) 负方向 ⇒ 同样夹到 −step
    {
        double desired[6];
        for (int i = 0; i < 6; i++) desired[i] = prev[i] - huge;
        clampJointStep(prev, desired, step, out);
        for (int i = 0; i < 6; i++) CHECK(fabs(out[i] - (prev[i] - step)) < 1e-12);
    }

    // (c) ★ 别名：调用方就是**原地**把 `j` 同时当 `desired` 与 `out` 传进来的
    //     （`RelayCore.cpp` 的 `clampJointStep(m_btn2JointCmd, j, …, j)`）。若实现成"先算完
    //     再整体写回"之外的顺序（或先把 out 清零），这里会红。
    {
        double j[6] = {10.0 + huge, 20.0 - huge, 30.0 + huge,
                       40.0 - huge, 50.0 + huge, 60.0 - huge};
        clampJointStep(prev, j, step, j);
        CHECK(fabs(j[0] - (prev[0] + step)) < 1e-12);
        CHECK(fabs(j[1] - (prev[1] - step)) < 1e-12);
        CHECK(fabs(j[2] - (prev[2] + step)) < 1e-12);
        CHECK(fabs(j[3] - (prev[3] - step)) < 1e-12);
        CHECK(fabs(j[4] - (prev[4] + step)) < 1e-12);
        CHECK(fabs(j[5] - (prev[5] - step)) < 1e-12);
    }

    // (d) 在步长**之内**的偏移 ⇒ 一步到位（限幅不该拖慢正常小移动）
    {
        const double small = 0.5 * step;
        double desired[6];
        for (int i = 0; i < 6; i++) desired[i] = prev[i] + small;
        clampJointStep(prev, desired, step, out);
        for (int i = 0; i < 6; i++) CHECK(fabs(out[i] - desired[i]) < 1e-12);
    }
    PASS();
}

// ⑰ ★【累加器不会永久截断】大目标在多帧里**慢慢跟上**，最终**恰好**到达。
//    【为什么非补不可】⑯ 只证明"单步被夹住"。而"夹取"与"永久截断"在**单帧**里长得一样 ——
//      把累加式换成"把目标直接夹到 `prev ± step` 就发"（= 不推进积分器）会**逐帧结果相同**
//      但对大偏移**永远走不到**（每帧都从同一个 prev 夹同一个量）⇒ 只有**多帧**能区分。
//      RPY 路径 2026-09-21 那次改造就是为这个（见 `Config.h` 的 `ORIENT_DEADZONE_DEG` 历史段）。
//    ⚠ 这是本函数与调用方**分工**的用例：本函数无状态，累加器（`prev` 的推进）由调用方持有
//      ⇒ 这里就照调用方的形状**自己推**那个积分器（每帧把上一帧的输出当下一帧的 prev）。
static void test_accumulator_reaches_a_large_target_over_several_frames() {
    TEST(accumulator_reaches_a_large_target_over_several_frames);
    const double step = Config::ORIENT_MAX_STEP_DEG;
    CHECK(step > 0.0);
    const double prev0[6] = {0.0, 10.0, -20.0, 30.0, -40.0, 50.0};
    // 目标偏移 20 步 ⇒ 不设累加的话永远走不到；设了就到得了（帧数与步长都从常数导出）
    const int    frames = 20;
    double desired[6], cmd[6];
    for (int i = 0; i < 6; i++) {
        desired[i] = prev0[i] + frames * step;
        cmd[i] = prev0[i];                    // 积分器种子 = 参照本身（与 onButton2Press 一致）
    }

    // 前 frames-1 帧：每帧**恰好**推进一个 step（不是更多、也不是更少）
    for (int f = 0; f < frames - 1; f++) {
        clampJointStep(cmd, desired, step, cmd);
        for (int i = 0; i < 6; i++) {
            CHECK(fabs(cmd[i] - (prev0[i] + (f + 1) * step)) < 1e-9);
        }
    }

    // 最后一帧：**恰好**到达目标（且没有过冲）
    clampJointStep(cmd, desired, step, cmd);
    for (int i = 0; i < 6; i++) CHECK(fabs(cmd[i] - desired[i]) < 1e-9);

    // 再走一帧：目标已到 ⇒ 不再变化（这条抓"每帧无条件加一个 step"那种写法）
    clampJointStep(cmd, desired, step, cmd);
    for (int i = 0; i < 6; i++) CHECK(fabs(cmd[i] - desired[i]) < 1e-9);
    PASS();
}

// ⑱ ★ 跨 ±180 接缝（2026-09-24 新实现特有的性质，旧实现没有）。
//    【场景】`Rz(+179°) → Rz(−179°)`：物理上只转了 **2°**，但逐欧拉角作差是
//      `−179 − (+179) = −358°` ⇒ 被限幅夹到 −150 ⇒ **关节一路冲向限位**（本仓记过
//      "180 → −179.9 的数值跳被照字面解释成 J6 转一整圈"那次关节超速）。
//    【现在断什么】真 ΔR 走矩阵，绕接缝无关 ⇒ 期望值是 **+2°**，不是 ±150°。
//    ⚠ 这条对新实现是**结构性的**（不是调参调出来的）；旧实现在这里必红。
static void test_crossing_the_pm180_seam_is_only_two_degrees() {
    TEST(crossing_the_pm180_seam_is_only_two_degrees);
    const double ref[6] = {0, 0, 0, 10, 20, 30};
    const double refS[3] = {0, 0, +179.0};
    const double cur[3]  = {0, 0, -179.0};
    double out[6];
    button2JointTarget(ref, refS, cur, out);
    CHECK(fabs((out[5] - ref[5]) - Config::BTN2_J6_SIGN * (+2.0)) < 1e-6);   // 器件 Z -> J6，+2°
    CHECK(fabs(out[3] - ref[3]) < 1e-9);
    CHECK(fabs(out[4] - ref[4]) < 1e-9);
    CHECK(fabs(out[5] - ref[5]) < 150.0);            // ★ 绝不能被夹成 ±150
    PASS();
}


int main() {
    std::cout << "--- Button2Joint (按钮2 关节空间映射：笔杆姿态增量 -> 关节增量) ---" << std::endl;
    test_each_stylus_axis_moves_exactly_one_joint();
    test_no_motion_returns_reference();
    test_signature_has_no_position_input();
    test_oversized_stylus_rotation_is_clamped_per_joint();
    test_deadzone_swallows_tiny_offset_and_keeps_the_boundary();
    test_nonfinite_guard_covers_all_argument_positions();
    test_three_axes_at_once_are_independent_at_large_angles();
    test_deadzone_is_per_axis_not_by_magnitude();       // ⑧ 复审追加
    test_sign_constants_are_unit_signs();               // ⑨ 复审追加
    test_trustworthy_ref_is_accepted();                 // ⑩ 2026-09-23 fix2：I1 的判据（抽出后）
    test_all_zero_ref_is_rejected();                    // ⑪
    test_out_of_limits_ref_is_rejected();               // ⑫
    test_nan_ref_is_accepted_by_this_predicate();       // ⑬ ★ 边界（刻意放行 NaN）
    test_five_zeros_and_a_tiny_value_is_still_trustworthy();   // ⑭ ★ 恰好全 0 的边界
    test_clamp_step_zero_offset_changes_nothing();      // ⑮ M2 的判据（抽出后）
    test_clamp_step_limits_a_large_offset_per_axis();   // ⑯
    test_accumulator_reaches_a_large_target_over_several_frames();  // ⑰ ★ 不会永久截断
    test_crossing_the_pm180_seam_is_only_two_degrees();   // ⑱ 2026-09-24 换实现后新增（跨 ±180）
    std::cout << "\nResults: " << g_passed << " passed, " << g_failed << " failed" << std::endl;
    return g_failed == 0 ? 0 : 1;
}
