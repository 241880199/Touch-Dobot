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

// ⚠★ 2026-09-29 修复轮 F1：这里**删掉**了一份 `matToRpyZYX` —— 它与本文件上面 :87 的
//   `matToZyRpy` 是"同一约定两份实现"（本仓最忌的一种缺陷）。全部调用点
//   （⑳/⑳b/⑳c/⑳d/㉑）已改走 `matToZyRpy`；本文件里矩阵→欧拉现在**只有这一份实现**。
//   【为什么当初会误加】派单简报断言"本仓没有矩阵→欧拉"，于是照着这层意思新写了一份 ——
//     而那句对**生产代码**成立、对**本测试文件**不成立（`matToZyRpy` 早就在 :87，⑦ 用它造
//     输入并自检过）。真实差别只有**万向锁那一支**的取舍（`atan2(-R[5],R[4])` vs
//     `atan2(-R[7],R[4])`）；**非退化支路**上两者逐字相同 —— 而本组用例全部落在非退化支路
//     （最大也只是 ±58.93° 这类小角度、|ry| ≪ 90）。
//   ⚠ 等价性**不靠**上面那句"逐字相同"的口头结论：下面这条往返断言把它实测钉住。
//     「为什么不直接删了不管」的答案就在这条用例里 —— 换过去若不等价，它必红。
//   ⚠ `matToZyRpy` 本身**一个字都没改**（既有用例 ⑦ 依赖它构造输入）。
static void test_mat_to_rpy_roundtrip_on_task2_inputs() {
    TEST(⑳x F1 等价性：matToZyRpy(rpyToMatrix(x)) 回到 x（⑳/㉑ 实际用到的输入，非退化）);
    // (1) ⑳ 实际喂进去的两个 refStylus：全 0 与现场那个按下姿态
    const double refs[2][3] = {{0, 0, 0}, {-58.93, 11.83, -6.41}};
    for (int q = 0; q < 2; ++q) {
        double R[9], back[3];
        TcpCalibration::rpyToMatrix(refs[q][0], refs[q][1], refs[q][2], R);
        matToZyRpy(R, back);
        for (int i = 0; i < 3; ++i)
            CHECK(std::fabs(back[i] - refs[q][i]) < 1e-9);   // 往返回到原值
    }
    // (2) ⑳ 用 Rx(10)、㉑ 用 Rz(20) 造 curStylus —— 两条构造路径各走一遍（矩阵→欧拉→矩阵）
    const double devs[2][3] = {{10, 0, 0}, {0, 0, 20}};
    for (int q = 0; q < 2; ++q) {
        double Rs[9], Rd[9], Rsc[9], back[3];
        TcpCalibration::rpyToMatrix(0, 0, 0, Rs);
        button2RotVecToMatDeg(devs[q], Rd);
        button2Mat3Mul(Rs, Rd, Rsc);
        matToZyRpy(Rsc, back);                        // 与 ⑳/㉑ 里构造 curStylus 的同一句
        double R2[9];
        TcpCalibration::rpyToMatrix(back[0], back[1], back[2], R2);
        for (int i = 0; i < 9; ++i)
            CHECK(std::fabs(R2[i] - Rsc[i]) < 1e-9);  // 反解回同一个矩阵
    }
    PASS();
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


// ⑲ 3×3 纯算术：旋转向量↔矩阵 的往返 + 一个【具体数】的已知值 + 乘/转置/求逆。
//    ⚠ 不用"动了就算过"的断言：这里钉的是具体数值。
//    ⚠★ 2026-09-29 评审修复轮：原来四个子块挤在【一个】`static void` 里，而 `CHECK` 的失败路径是
//      `return;` ⇒ (a) 一红，(b)(c)(d) **根本不会跑**（一个红遮住三个结论；实测：突变 `R[2]` 后
//      只印出 ⑲(a) 的 FAIL，⑲(b)(c)(d) 连标签都没打）。现拆成【四个】独立函数，与本文件其余
//      18 条用例同一形状：一条红只停它自己。
//    ⚠ 为什么是 `static void` 函数而不是直接写进 `main()`：`CHECK` 展开成 `return;`，而 `return;`
//      在 `int main()` 里**不是合法 C++**（2026-09-29 实测 MSVC C2561「"main": 函数必须返回值」）。
//      包成 void 函数后失败只退回本用例，main 照常 `return g_failed == 0 ? 0 : 1` ⇒ 真红会让
//      整套件 exit 非 0（而 run_tests.bat 判的正是退出码）。
//    ⚠★ 每个子块末尾都必须有 `PASS()`：`PASS()` 是本文件里【唯一】给 `g_passed` 计数的宏。
//      没有它时，"跑绿"与"把这四条全删掉"的输出**逐字相同**、`Results:` 的计数也不涨（评审实测：一直是 18）。
//
// ⑲(a) 往返：rotVecDeg ∘ rotVecToMatDeg 还原原向量
static void test_rotvec_matrix_roundtrip() {
    TEST(⑲(a) 往返: rotVecDeg∘rotVecToMatDeg 还原原向量);
    const double cases[4][3] = {{0,0,0}, {10,0,0}, {0,-25,0}, {12,-7,3}};
    for (int c = 0; c < 4; ++c) {
        double R[9], rv[3];
        button2RotVecToMatDeg(cases[c], R);
        button2RotVecDegForTest(R, rv);        // 复用 Button2Joint.cpp 里的实现（经头文件薄壳暴露）
        for (int i = 0; i < 3; ++i)
            CHECK(std::fabs(rv[i] - cases[c][i]) < 1e-9);
    }
    PASS();
}

// ⑲(b) 绕 X 转 90° 的具体矩阵
static void test_rotvec_matrix_specific_rx90() {
    TEST(⑲(b) 绕 X 转 90° 的具体矩阵);
    const double rv[3] = {90, 0, 0};
    double R[9];
    button2RotVecToMatDeg(rv, R);
    // Rx(90) = [[1,0,0],[0,0,-1],[0,1,0]]（行主序）
    const double exp[9] = {1,0,0, 0,0,-1, 0,1,0};
    for (int i = 0; i < 9; ++i) CHECK(std::fabs(R[i] - exp[i]) < 1e-9);
    PASS();
}

// ⑲(c) 乘与转置：A·Aᵀ == I（A 是旋转）
static void test_rotvec_mat3_mul_and_transpose() {
    TEST(⑲(c) 乘与转置：A·Aᵀ == I（A 是旋转）);
    const double rv[3] = {12, -7, 3};
    double A[9], At[9], P[9];
    button2RotVecToMatDeg(rv, A);
    button2Mat3T(A, At);
    button2Mat3Mul(A, At, P);
    for (int i = 0; i < 9; ++i)
        CHECK(std::fabs(P[i] - (i % 4 == 0 ? 1.0 : 0.0)) < 1e-9);
    PASS();
}

// ⑲(d) 求逆：A·A⁻¹ == I；奇异矩阵返回 false
static void test_rotvec_mat3_inverse() {
    TEST(⑲(d) 求逆：A·A⁻¹ == I；奇异矩阵返回 false);
    const double rv[3] = {12, -7, 3};
    double A[9], Ai[9], P[9];
    button2RotVecToMatDeg(rv, A);
    CHECK(button2Mat3Inv(A, Ai));
    button2Mat3Mul(A, Ai, P);
    for (int i = 0; i < 9; ++i)
        CHECK(std::fabs(P[i] - (i % 4 == 0 ? 1.0 : 0.0)) < 1e-9);
    const double sing[9] = {1,2,3, 2,4,6, 0,0,1};   // 第 1、2 行线性相关
    CHECK(!button2Mat3Inv(sing, Ai));
    PASS();
}

// ============================================================================
//  ⑳ ~ ㉑ 2026-09-29 Task 2：`button2OrientTarget`（摆动锁基座 + 自转绕自身轴）
// ============================================================================
// 【这一组用例在测什么，以及为什么是这个形状】
//   旧路（`button2JointTarget`，本文件其余 18 条钉着）把"一根器件轴"直接喂给"一个关节"。
//   新路先算出**想要的末端朝向**（摆动在**基座系**里、自转绕**末端自身**轴），再由 Task 3 解 J4/J5/J6。
//   本文件这一组只测**目标朝向那一半**（它是纯算术）；解关节那一半在 Task 3。
//
// ⚠ 三条判据的形状是刻意的，别"顺手改成更直观的写法"：
//   ① **不去查 FK 的列**。要断言"前摆 ⇒ 末端往基座 +Y 摆"，最省事的写法是拿 `Kinematics`
//      算出末端轴、看它的 tip 往哪挪 —— 但那要先认定"末端系的哪一列是笔尖指向"。
//      那是**未核实的前提**（计划自审时抓到的）⇒ 改成对**这次姿态变化本身**取旋转向量：
//      `D = outR · refRᵀ`。绕基座 +X 正转 θ **就是**"末端轴（朝下时）往 +Y 摆"那件事的
//      坐标无关说法 ⇒ 只断言 D 的旋转向量沿基座 X、大小 = 输入角。**与位姿无关、与符号约定无关。**
//   ② **期望值经 Config 表达，一个 ±1 都不写死**（与文件头 ② 同一条规矩）：摆动两路用
//      `BTN2_TILT_SIGN_X/Y`、自转用 `BTN2_ROLL_SIGN`。⇒ Task 3 上机翻符号不会让用例红。
//   ③ **φ 一律经入参传**（`Config::BTN2_TILT_PHI_DEG`，只有 ⑳d 传 90.0）。原因见头文件：
//      编译期常数在运行期改不了 ⇒ 写不出有牙齿的对照。

// ⑳ ★【验收标准本身】器件前摆 ⇒ 末端往基座 +Y 摆 —— **任意位姿下都成立**（这正是本次要修的）。
//    【为什么是三个位姿 × 两个参考笔杆姿态】"摆动方向随姿态跑"这个病只在**非正位**上显形
//      （正位下"左乘"与"右乘"退化到同一件事）⇒ 只测一个正位，坏实现照样绿。
//    【为什么参考笔杆姿态要有第二个】要看的是"**与参考笔杆姿态无关**"（映射吃的是 ΔR = R_refᵀ·R_cur，
//      不是绝对姿态）⇒ 参考取 {-58.93, 11.83, -6.41}（现场那个按下姿态）再走一遍。
//    ⚠★ 2026-09-29 修复轮 F2：本用例原来把**两条**断言挤在【一个】 `static void` 里，而 `CHECK`
//      的失败路径是 `return;` ⇒ 排在前面的"逐元素等于 `Mw·refR`"一红，**后面那条真正的验收判据
//      根本不会跑**（一个红遮住一个结论）。现拆成 **⑳(a) / ⑳(b) 两个独立函数**：两条断言各自
//      失败、各带自己的 `PASS()`。**断言内容一个字都没有放宽**，只是把它们分开放。
//      ⚠ 为什么是 `static void` 而不是直接写进 `main()`：`CHECK` 展开成 `return;`，而 `return;`
//        在 `int main()` 里不是合法 C++（同 ⑲ 2026-09-29 实测那条 C2561）。
//      ⚠ 拆开后**两个函数各自持有**那份"`refR` 得是旋转"的自检：少了它，函数二可能在坏夹具上
//        静默通过（又是一次"看起来跑了但其实空转"）⇒ 自检必须跟着断言走，不能只留一份。
//
// ⑳(a) 展开式断言：`outR` **逐元素**等于 `Mw·refR`（`Mw = rotVecToMatDeg(SX·10, 0, 0)`）。
//      这是"实现与期望的**精确值**相等"那一条 —— 最紧，也最容易被实现自己背书（期望值经 SX 表达）。
static void test_tilt_target_equals_world_rotation_times_ref() {
    TEST(⑳(a) 前摆 ⇒ outR 逐元素等于 Mw·refR（三个位姿 × 两个参考笔杆姿态）);
    const double poses[3][6] = {
        {0,0,0,0,-90,0}, {30,-60,45,20,-70,10}, {-120,40,-30,90,-45,180}
    };
    const double refs[2][3] = {{0,0,0}, {-58.93, 11.83, -6.41}};
    for (int p = 0; p < 3; ++p) {
        double T[4][4]; Kinematics::composeTransform(poses[p], T);
        double refR[9];
        for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) refR[r*3+c] = T[r][c];
        // 自检：refR 得是**旋转**，否则下面那组期望值不再是"一次世界系旋转"（空转）。
        {
            double rt[9], pr[9], e = 0.0;
            button2Mat3T(refR, rt);
            button2Mat3Mul(refR, rt, pr);
            for (int i = 0; i < 9; ++i) e += std::fabs(pr[i] - (i % 4 == 0 ? 1.0 : 0.0));
            CHECK(e < 1e-12);
        }
        for (int q = 0; q < 2; ++q) {
            // 造输入：绕【器件 X】转 10° ⇒ R_cur = R_refStylus · Rx(10)
            double Rs[9], Rx[9], Rsc[9];
            TcpCalibration::rpyToMatrix(refs[q][0], refs[q][1], refs[q][2], Rs);
            const double a10[3] = {10, 0, 0};
            button2RotVecToMatDeg(a10, Rx);
            button2Mat3Mul(Rs, Rx, Rsc);
            double curStylus[3]; matToZyRpy(Rsc, curStylus);

            double outR[9];
            button2OrientTarget(refR, refs[q], curStylus, Config::BTN2_TILT_PHI_DEG, outR);

            // 期望 = rotVecToMatDeg(SX·10, 0, 0) · refR  ← 经 SX 表达，【不写死符号】
            double Mw[9], expR[9];
            const double w[3] = { Config::BTN2_TILT_SIGN_X * 10.0, 0.0, 0.0 };
            button2RotVecToMatDeg(w, Mw);
            button2Mat3Mul(Mw, refR, expR);
            for (int i = 0; i < 9; ++i) CHECK(std::fabs(outR[i] - expR[i]) < 1e-9);
        }
    }
    PASS();
}

// ⑳(b) ★【与实现无关的验收判据】这次姿态变化的【世界系旋转向量】必须**沿基座 X 轴**
//      （即"往 +Y 摆"的那根轴）：Y、Z 分量 < 1e-9，且 |X| 分量**恰为 10°**。
//      ⚠ 这条**不查实现算出来的任何中间量** —— 它只问"末端朝向变了多少、绕哪根轴变"，
//        符号约定、乘法次序、坐标约定都改不动它 ⇒ 正是拆开前会被 ⑳(a) 遮住的那条。
//      ⚠ 拆开后它自带一份"refR 是旋转"的自检（与 ⑳(a) 的那份同因不同用）：
//        没有它，坏夹具下 `D = outR·refRᵀ` 不再是"这次变化的世界系旋转"（空转）。
static void test_tilt_moves_tip_toward_base_plus_y_in_any_pose() {
    TEST(⑳(b) 前摆 ⇒ 末端 tip 往基座 +Y 摆（世界系旋转向量沿基座 X，三个位姿 × 两个参考笔杆姿态）);
    const double poses[3][6] = {
        {0,0,0,0,-90,0}, {30,-60,45,20,-70,10}, {-120,40,-30,90,-45,180}
    };
    const double refs[2][3] = {{0,0,0}, {-58.93, 11.83, -6.41}};
    for (int p = 0; p < 3; ++p) {
        double T[4][4]; Kinematics::composeTransform(poses[p], T);
        double refR[9];
        for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) refR[r*3+c] = T[r][c];
        // 自检：refR 得是**旋转**，否则下面 D = outR·refRᵀ 不再是"这次变化的世界系旋转"（空转）。
        {
            double rt[9], pr[9], e = 0.0;
            button2Mat3T(refR, rt);
            button2Mat3Mul(refR, rt, pr);
            for (int i = 0; i < 9; ++i) e += std::fabs(pr[i] - (i % 4 == 0 ? 1.0 : 0.0));
            CHECK(e < 1e-12);
        }
        for (int q = 0; q < 2; ++q) {
            // 造输入：绕【器件 X】转 10° ⇒ R_cur = R_refStylus · Rx(10)
            double Rs[9], Rx[9], Rsc[9];
            TcpCalibration::rpyToMatrix(refs[q][0], refs[q][1], refs[q][2], Rs);
            const double a10[3] = {10, 0, 0};
            button2RotVecToMatDeg(a10, Rx);
            button2Mat3Mul(Rs, Rx, Rsc);
            double curStylus[3]; matToZyRpy(Rsc, curStylus);

            double outR[9];
            button2OrientTarget(refR, refs[q], curStylus, Config::BTN2_TILT_PHI_DEG, outR);

            // ★ 与实现无关的【验收判据】：这次姿态变化的【世界系旋转向量】必须
            //   **沿基座 X 轴**（即"往 +Y 摆"的那根轴），其余两分量必须是 0，大小恰为 10°。
            double refT[9], D[9];
            button2Mat3T(refR, refT);
            button2Mat3Mul(outR, refT, D);              // D = outR · refRᵀ
            double rvw[3]; button2RotVecDegForTest(D, rvw);
            CHECK(std::fabs(rvw[1]) < 1e-9);            // 无基座 Y 分量
            CHECK(std::fabs(rvw[2]) < 1e-9);            // 无基座 Z 分量
            CHECK(std::fabs(std::fabs(rvw[0]) - 10.0) < 1e-9);   // 大小 = 10°（方向经 SX 表达）
        }
    }
    PASS();
}

// ⑳b 死区：**低于**门限的分量必须归 0；**恰好等于**门限必须放行（`>=` 而不是 `>`）。
//    【为什么后半条非有不可】只测"0.005 不动"的话，把门限写成 `>`（恰好等于时被吞掉）
//      这个错**测不出来** —— 而它与旧实现 `axisGate` 的语义不符 ⇒ 那正是实现与接线分家的地方。
//    ⚠★ 实测过一件事再写这条：这里的**输入构造**（rotVecToMatDeg → 欧拉 → rpyToMatrix）
//      绕了一圈三角函数，理论上可能把 `rv[0]` 压到门限**之下一个 ulp** ⇒ 正确实现也会红
//      （"掷硬币"式的假红）。上机前先用探针实测：`rv[0] == dz` **逐位相等**、diff = 0.000e+00
//      ⇒ 这条不存在那个脆弱性（探针记录在 task-2-report.md）。
static void test_deadzone_swallows_below_and_admits_exactly_at_threshold() {
    TEST(⑳b 死区：低于门限的分量必须归 0；恰好等于门限必须放行);
    const double refR[9] = {1,0,0, 0,1,0, 0,0,1};
    const double refStylus[3] = {0,0,0};
    CHECK(Config::ORIENT_DEADZONE_DEG > 0.0);       // 自检：门限非正 ⇒ 下面两条同义
    {   // 低于门限 ⇒ 输出 == refR（一点没动）
        double Rs[9], Rx[9], Rsc[9], cur[3], outR[9];
        TcpCalibration::rpyToMatrix(0,0,0, Rs);
        const double a[3] = { Config::ORIENT_DEADZONE_DEG * 0.5, 0, 0 };
        button2RotVecToMatDeg(a, Rx); button2Mat3Mul(Rs, Rx, Rsc); matToZyRpy(Rsc, cur);
        button2OrientTarget(refR, refStylus, cur, Config::BTN2_TILT_PHI_DEG, outR);
        for (int i = 0; i < 9; ++i) CHECK(std::fabs(outR[i] - refR[i]) < 1e-12);
    }
    {   // 恰好等于门限 ⇒ 必须放行（与 `>` 相反）
        double Rs[9], Rx[9], Rsc[9], cur[3], outR[9];
        TcpCalibration::rpyToMatrix(0,0,0, Rs);
        const double a[3] = { Config::ORIENT_DEADZONE_DEG, 0, 0 };
        button2RotVecToMatDeg(a, Rx); button2Mat3Mul(Rs, Rx, Rsc); matToZyRpy(Rsc, cur);
        button2OrientTarget(refR, refStylus, cur, Config::BTN2_TILT_PHI_DEG, outR);
        // ⚠ 2026-09-29 Task 3（复审 M3）：这里原来多一行 `button2Mat3T(outR, outT);` —— 它的
        //   结果 `outT` **算完没人读**（下面只用 `refT`）⇒ 删掉那一行连同它的声明。这不是
        //   行为改变：`outT` 从未参与任何断言。
        double D[9], refT[9];
        button2Mat3T(refR, refT); button2Mat3Mul(outR, refT, D);
        double rv2[3]; button2RotVecDegForTest(D, rv2);
        CHECK(std::fabs(std::fabs(rv2[0]) - Config::ORIENT_DEADZONE_DEG) < 1e-9);
    }
    PASS();
}

// ⑳c 偏移限幅：超 150° ⇒ 旋转角被夹到 150°，且**【轴不变】**。
//    ⚠★ 这条必须分两半，因为**(a) 单独根本区分不了两种实现**（这是写之前先算出来的）：
//      "整体缩比"（正确）与"逐分量 clamp"（错误）在**纯单轴**输入下给出**同一个结果** ——
//      只有一个分量非零、而它正是要缩的那一根 ⇒ 两者都得 (150,0,0)。
//      ⇒ (b) 用**两根非零分量**的旋转向量（|rv| = √(170²+20²) ≈ 171.2° > 150°）：
//        整体缩比 ⇒ (SX·170k, SY·20k, 0)（**方向不变**，k = 150/|rv|）；
//        逐分量夹 ⇒ (SX·150, SY·20, 0)（**方向被夹歪**，且幅度也不再是 150）。
//      负对照（把缩比换成逐分量 clamp）**只在 (b) 变红**，见 task-2-report.md。
static void test_offset_cap_scales_rotation_angle_and_keeps_the_axis() {
    TEST(⑳c 偏移限幅：超 150° ⇒ 夹到 150° 且【轴不变】);
    const double refR[9] = {1,0,0, 0,1,0, 0,0,1};
    const double refStylus[3] = {0,0,0};
    const double cap = Config::ORIENT_MAX_OFFSET_DEG;
    CHECK(cap > 0.0 && cap < 180.0);                // 自检：否则"超限"不可构造

    // (a) 纯器件 X 轴转 170°（> cap）⇒ 世界系旋转向量 = 基座 X × 符号 × cap，另两维恰为 0
    {
        double Rs[9], Ra[9], Rsc[9], cur[3], outR[9];
        TcpCalibration::rpyToMatrix(0,0,0, Rs);
        const double a[3] = {170.0, 0.0, 0.0};
        button2RotVecToMatDeg(a, Ra); button2Mat3Mul(Rs, Ra, Rsc); matToZyRpy(Rsc, cur);
        button2OrientTarget(refR, refStylus, cur, Config::BTN2_TILT_PHI_DEG, outR);
        double refT[9], D[9];
        button2Mat3T(refR, refT); button2Mat3Mul(outR, refT, D);
        double rvw[3]; button2RotVecDegForTest(D, rvw);
        CHECK(std::fabs(rvw[1]) < 1e-9);
        CHECK(std::fabs(rvw[2]) < 1e-9);
        CHECK(std::fabs(std::fabs(rvw[0]) - cap) < 1e-9);
    }

    // (b) ★【轴不变】的判别器：两根非零分量、模长 > cap
    {
        const double a[3] = {170.0, 20.0, 0.0};
        const double n = std::sqrt(170.0*170.0 + 20.0*20.0);
        const double k = cap / n;
        double Rs[9], Ra[9], Rsc[9], cur[3], outR[9];
        TcpCalibration::rpyToMatrix(0,0,0, Rs);
        button2RotVecToMatDeg(a, Ra); button2Mat3Mul(Rs, Ra, Rsc); matToZyRpy(Rsc, cur);
        button2OrientTarget(refR, refStylus, cur, Config::BTN2_TILT_PHI_DEG, outR);
        double refT[9], D[9];
        button2Mat3T(refR, refT); button2Mat3Mul(outR, refT, D);
        double rvw[3]; button2RotVecDegForTest(D, rvw);
        // 期望：方向不变、幅度缩到 cap ⇒ rv_new = 原方向 × cap = (SX·170k, SY·20k, 0)
        CHECK(std::fabs(rvw[0] - Config::BTN2_TILT_SIGN_X * 170.0 * k) < 1e-9);
        CHECK(std::fabs(rvw[1] - Config::BTN2_TILT_SIGN_Y *  20.0 * k) < 1e-9);
        CHECK(std::fabs(rvw[2]) < 1e-9);
        // 自检：本条与 (a) 确实不同（若 n ≤ cap，(b) 就退化成 (a)，判别力归零）
        CHECK(n - cap > 1.0);
    }
    PASS();
}

// ⑳d ★ φ 不是死常数：φ=90° 时"器件左右摆"必须映到【绕基座 Z 转】。
//    今天 φ=0 ⇒ 左右摆映到绕基座 Y。把 φ 传成 90° 时，(0,cosφ,sinφ) = (0,0,1)
//    ⇒ 世界系旋转向量必须【沿基座 Z】且大小仍是 10°。**这一条专治"常数没被用上"**。
//    ⚠★ 本仓栽过"常数写进去但没人读"，而那种缺陷**在默认值下完全看不出来** —— 这里默认值
//      正是 0（今天就是 0）⇒ 不测 φ≠0 就等于没测这个常数。
//    【负对照（保证红，不依赖任何假设）】把实现里的 φ 恒置 0（即把入参丢掉）⇒ rvw[2] 变 ≈0、
//      rvw[1] 变 ±10 ⇒ 下面第三句必红。实测记录见 task-2-report.md。
static void test_phi_is_not_a_dead_constant() {
    TEST(⑳d ★ φ 不是死常数：φ=90° 时"左右摆"映到绕基座 Z 转);
    const double refR[9] = {1,0,0, 0,1,0, 0,0,1};
    const double refStylus[3] = {0,0,0};
    double Rs[9], Ry10[9], Rsc[9], cur[3], outR[9];
    TcpCalibration::rpyToMatrix(0,0,0, Rs);
    const double a[3] = {0, 10, 0};                       // 器件 Y 轴 +10°（左右摆）
    button2RotVecToMatDeg(a, Ry10);
    button2Mat3Mul(Rs, Ry10, Rsc);
    matToZyRpy(Rsc, cur);
    button2OrientTarget(refR, refStylus, cur, 90.0, outR);   // ← φ = 90

    double refT[9], D[9];
    button2Mat3T(refR, refT);
    button2Mat3Mul(outR, refT, D);                        // 世界系增量
    double rvw[3]; button2RotVecDegForTest(D, rvw);
    CHECK(std::fabs(rvw[0]) < 1e-9);                      // 无基座 X 分量
    CHECK(std::fabs(rvw[1]) < 1e-9);                      // 无基座 Y 分量
    CHECK(std::fabs(std::fabs(rvw[2]) - 10.0) < 1e-9);    // 大小 = 10°（方向由 SY 定，故取绝对值）
    PASS();
}

// ㉑ 自转 ⇒ **末端轴指向不变**、只绕【末端自身轴】拧 ROLL_SIGN·20°。
//    【判据 1 为什么是"第三列逐位不变"】`outR = R_tilt · Rz(roll)` 里 `Rz` 的第三列恰是 (0,0,1)
//      ⇒ `outR` 的第三列 == `R_tilt` 的第三列。20° 的输入下 `R_tilt == refR`（摆动那两路
//      的分量都是 0）⇒ 第三列必须**逐位**等于 `refR` 的第三列。
//      ⚠ 这一句正是"右乘 vs 左乘"的判别器：左乘 `Rz·refR` 会把第三列一起转走 ⇒ 必红。
//    【判据 2】`outR·outRᵀ == I` —— 顺带证明 `outR` 还是旋转矩阵（连乘没有把它乘坏）。
//    【判据 3】`rotvec(outR·refRᵀ)` 必须恰为 `refR` 的第三列（归一化）× ROLL_SIGN·20°。
static void test_roll_only_spins_around_the_styluses_own_axis() {
    TEST(㉑ 自转 ⇒ 末端轴指向不变、绕自身轴转 ROLL_SIGN·20°);
    const double poses[2][6] = {{0,0,0,0,-90,0}, {30,-60,45,20,-70,10}};
    for (int p = 0; p < 2; ++p) {
        double T[4][4]; Kinematics::composeTransform(poses[p], T);
        double refR[9];
        for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) refR[r*3+c] = T[r][c];
        const double refStylus[3] = {0, 0, 0};
        double Rs[9], Rz20[9], Rsc[9];
        TcpCalibration::rpyToMatrix(0,0,0, Rs);
        const double a20[3] = {0, 0, 20};
        button2RotVecToMatDeg(a20, Rz20);
        button2Mat3Mul(Rs, Rz20, Rsc);
        double curStylus[3]; matToZyRpy(Rsc, curStylus);

        double outR[9];
        button2OrientTarget(refR, refStylus, curStylus, Config::BTN2_TILT_PHI_DEG, outR);

        // 判据1：接近轴（第 3 列）逐位不变
        for (int r = 0; r < 3; ++r) CHECK(std::fabs(outR[r*3+2] - refR[r*3+2]) < 1e-9);
        // 判据2：outR·outRᵀ 应 ≈ I（两矩阵都正交）—— 顺带证明 outR 是旋转
        double outT[9], D[9];
        button2Mat3T(outR, outT);
        button2Mat3Mul(outR, outT, D);
        for (int i = 0; i < 9; ++i)
            CHECK(std::fabs(D[i] - (i % 4 == 0 ? 1.0 : 0.0)) < 1e-9);
        // 判据3：outR·refRᵀ 是绕"接近轴"转 ROLL_SIGN·20° 的旋转
        double Rs2[9], dR[9];
        button2Mat3T(refR, Rs2);
        button2Mat3Mul(outR, Rs2, dR);
        double rvd[3]; button2RotVecDegForTest(dR, rvd);
        double ax[3] = {refR[2], refR[5], refR[8]};      // 接近轴（第三列）
        double n = std::sqrt(ax[0]*ax[0]+ax[1]*ax[1]+ax[2]*ax[2]);
        CHECK(std::fabs(n - 1.0) < 1e-12);               // 自检：refR 正交 ⇒ 第三列是单位向量
        for (int i = 0; i < 3; ++i) ax[i] /= n;
        for (int i = 0; i < 3; ++i)
            CHECK(std::fabs(rvd[i] - ax[i] * Config::BTN2_ROLL_SIGN * 20.0) < 1e-6);
    }
    PASS();
}

// ============================================================================
//  ㉒ ~ ㉔ 2026-09-29 Task 3：`button2SolveWrist`（3×3 角雅可比 + FK 回代 + 阻尼 + 自验门）
// ============================================================================
// 【这一组在测什么】给定"想要的末端朝向"（Task 2 的 `button2OrientTarget` 产出的那一半），
//   解出 J4/J5/J6 使 FK(J1..J6) 的旋转 = 目标。**判据的形状**（照简报，且刻意如此）：
//     · ㉒ 恒等：目标 = 参照姿态 ⇒ 解 = 参照（初值即解，一步都不走）；
//     · ㉓ ★ **跨位姿 FK 回验** —— 这是"解对了"的**直接证据**，且【不依赖任何符号常数】
//       （三个 `BTN2_*_SIGN` 都不进这条：它只问"FK(解) 到不到得了目标"）；
//     · ㉔ 腕部奇异位姿上的**不变量**：返回 true ⇒ FK(解) 必须**真的**等于目标。
//   ⚠ ㉓ 拆成 (a)/(b) 两个独立函数：`CHECK` 的失败路径是 `return;` ⇒ 挤在一个函数里时，
//     排在前面的断言一红，"FK 回验"这条最重要的判据**根本不会跑**（一个红遮住一个结论）。
//     与 ⑲ / ⑳ 2026-09-29 的教训同一条。
//   ⚠ 每条都是 `static void` + 末尾 `PASS()`：`CHECK` 展开成 `return;`（不能写进 `int main()`，
//     那里 `return;` 不是合法 C++；漏 `PASS()` 则跑绿与"整条删掉"输出逐字相同）。

// 测试侧【读数器】：从关节角取 FK 旋转（行主序 9 元）。不是被测逻辑。
static void fkR(const double j[6], double R[9]) {
    double T[4][4]; Kinematics::composeTransform(j, T);
    for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) R[r*3+c] = T[r][c];
}
// 两个旋转之间的夹角（度）：|rotvec(A·Bᵀ)|。只经 `rotVecDeg` 的薄壳，不复写约定。
static double angBetweenDeg(const double A[9], const double B[9]) {
    double Bt[9], D[9]; button2Mat3T(B, Bt); button2Mat3Mul(A, Bt, D);
    double rv[3]; button2RotVecDegForTest(D, rv);
    return std::sqrt(rv[0]*rv[0] + rv[1]*rv[1] + rv[2]*rv[2]);
}

// 测试侧【随机可达目标】构造器：给定 RNG 状态，造出 `ref[6]` 与"在 FK(ref) 上左乘一个小旋转"
//   得到的目标 `Rt`。返回 false = 这一轮抽到退化轴（调用方跳过，且**不再**多消耗 RNG）。
//   ⚠ 判据依赖它的两条性质：① 目标是**附近可达**的（≤ ~26°）；② 位姿遍布 J4/J5/J6 全域。
//     它若退化成"目标恒等于参照"，㉓ 就成了空转 —— ㉒ 的存在正是不让这一层悄悄发生。
static bool makeRandomReachTarget(unsigned& seed, double ref[6], double Rt[9]) {
    auto rnd = [&seed]() { seed = seed * 1103515245u + 12345u;
                           return (double)((seed >> 16) & 0x7FFF) / 32767.0; };
    ref[0] = rnd()*720-360; ref[1] = rnd()*720-360; ref[2] = rnd()*310-155;
    ref[3] = rnd()*720-360; ref[4] = rnd()*720-360; ref[5] = rnd()*720-360;
    double Rr[9]; fkR(ref, Rr);
    const double ax[3] = { rnd()*2-1, rnd()*2-1, rnd()*2-1 };
    double n = std::sqrt(ax[0]*ax[0]+ax[1]*ax[1]+ax[2]*ax[2]);
    if (n < 1e-6) return false;
    const double sv[3] = { ax[0]/n*15*rnd(), ax[1]/n*15*rnd(), ax[2]/n*15*rnd() };
    double Md[9]; button2RotVecToMatDeg(sv, Md); button2Mat3Mul(Md, Rr, Rt);
    return true;
}

// ㉒ 目标 = 参考姿态 ⇒ 误差为 0 ⇒ 初值即解（一轮不走），解 = 参照（逐位）。
//    【为什么非有不可】它把"解出来了"这件事与"目标就在原地"这个平凡情形钉在一起：
//      若 ㉓ 的输入构造坏了（目标 ≈ 参照），只会让 ㉓ 变松 —— ㉒ 单独把平凡解的形状测掉。
static void test_solve_wrist_identity_target_returns_reference() {
    TEST(㉒ 目标 = 参考姿态 ⇒ 解 = 参考（恒等）);
    const double poses[3][6] = {
        {0,0,0,0,-90,0}, {30,-60,45,20,-70,10}, {-120,40,-30,90,-45,180}
    };
    for (int p = 0; p < 3; ++p) {
        double R[9]; fkR(poses[p], R);
        double out[6] = {9,9,9,9,9,9};
        CHECK(button2SolveWrist(poses[p], R, Config::ORIENT_MAX_STEP_DEG, out));
        for (int i = 0; i < 6; ++i) CHECK(std::fabs(out[i] - poses[p][i]) < 1e-6);
    }
    PASS();
}

// ㉓(a) ★★【本任务最重要的一条】跨位姿 FK 回验：解得出来 ⇒ FK(解) 必须真的等于目标。
//    【为什么它是"直接证据"】它只问"我把解代回 FK，末端朝向对不对"，**不碰任何符号常数**
//      （`BTN2_TILT_SIGN_*` / `BTN2_ROLL_SIGN` 都不进这条）—— 即使三个符号全反，只要解出的
//      q 能让 FK 命中目标，这条就绿。符号的账由 Task 2 那一组与上机去结。
//    ⚠ 判据写在循环**里**逐次断（而不是累加成 ok 再断）：第一个不满足的样本立刻红，且报出的是
//      那句本质判据，不是一个 "ok=189" 的聚合数。
//    ⚠ `solved >= 190` 是**防空转**：若实现恒返回 false，循环里那句一次都不会执行 ⇒ 全绿。
static void test_solve_wrist_fk_roundtrip_across_random_poses() {
    TEST(㉓(a) ★ 跨位姿 FK 回验：随机 200 组，解出来后 FK(解) 必须等于目标（< 0.05°）);
    // ★ 值域钉（照 ⑨ 的招数：一条**不经过实现**的断言）：`BTN2_WRIST_TOL_DEG` **同时**是
    //   `button2SolveWrist` 的收敛门限（`Button2Joint.cpp` 两处）与**本文件**的"到位"容差
    //   （㉓(a) / ㉔ / ㉕(a) 三处都用它）⇒ 把它放宽（例如 5.0）会让【全套用例照绿】，
    //   而交付精度静默变差 —— 没有任何别的断言会响。这一行挡的就是那个；
    //   0.1 的理由：比今天的 0.05 宽一倍（给调参留一点余地），又把它钉在"度"的小量级上。
    CHECK(Config::BTN2_WRIST_TOL_DEG <= 0.1);
    unsigned seed = 20260929u;
    int solved = 0;
    for (int t = 0; t < 200; ++t) {
        double ref[6], Rt[9];
        if (!makeRandomReachTarget(seed, ref, Rt)) continue;          // 抽到退化轴：跳过
        double out[6];
        if (!button2SolveWrist(ref, Rt, Config::ORIENT_MAX_STEP_DEG, out)) continue;  // 拒发：跳过
        ++solved;
        double Ro[9]; fkR(out, Ro);
        // ★ 本任务最重要的判据：解出来了 ⇒ FK(解) 必须真的到得了目标。
        CHECK(angBetweenDeg(Rt, Ro) < Config::BTN2_WRIST_TOL_DEG);
    }
    CHECK(solved >= 190);   // 容许 ~5% 因腕部奇异/不可达被拒，但绝大多数必须解得出来
    PASS();
}

// ㉓(b) 解腕【只】动 J4/J5/J6：J1/J2/J3 必须**逐位**不变（用 `==`，不是"接近"）。
//    【为什么与 (a) 分开】这是本任务第二条不变量，而 `CHECK` 的失败是 `return;` ⇒ 与 (a) 同函数
//      时会被 (a) 的 FK 断言遮住（或反过来）。两条各带自己的 `PASS()`，各自失败。
static void test_solve_wrist_keeps_base_joints_across_random_poses() {
    TEST(㉓(b) 跨位姿：解腕只动 J4/J5/J6，J1/J2/J3 逐位不变);
    unsigned seed = 20260929u;   // 与 ㉓(a) 同种子 ⇒ 跑的是同一批位姿
    int solved = 0;
    for (int t = 0; t < 200; ++t) {
        double ref[6], Rt[9];
        if (!makeRandomReachTarget(seed, ref, Rt)) continue;
        double out[6];
        if (!button2SolveWrist(ref, Rt, Config::ORIENT_MAX_STEP_DEG, out)) continue;
        ++solved;
        for (int i = 0; i < 3; ++i) CHECK(out[i] == ref[i]);
    }
    CHECK(solved >= 190);
    PASS();
}

// ㉔ 腕部奇异位姿上的**不变量**：**返回 true ⇒ FK(解) 必须真的等于目标**。
//    ★★ 这条【不能】写成"奇异位姿必须拒发"。理由两条，都是实测/算术：
//      ① 写死"必须拒发"= 让用例替实现背书（在 ref 附近腕部未必解不出）；
//      ② 更要命的是它**结构上无法变红** —— 不收敛会被末尾自验门转成 `false`，而失败分支
//         断言的是 `out == ref` ⇒ 删掉自验门后它照旧绿（本轮复审实测证伪，见 task-3-report.md）。
//      ⇒ 断言的只能是【不变量】：**返回 true ⇒ 解真的到得了目标**。这也是末尾自验门存在的
//        唯一理由（它把"没解出来"翻译成 `false`，而不是一个看起来像解的值）。
//    ⚠ 失败分支断言"out 逐位退回参照"是**另一条**契约（与 `button2Mat3Inv` 同款），一并钉住。
// ⚠ 2026-09-29 Task 3 复审 M-8：函数名从 `..._on_wrist_singularities` 改成现在这个 ——
//   原名说"腕部奇异位姿：可达/不可达"，而 c2 被拒的原因是**步长预算**、**不是**奇异
//   （实测把 `maxStepDeg` 换成 30° 它就解得出来，见下面那段注释）⇒ 名字按事实写。
static void test_solve_wrist_invariant_holds_and_budget_refusal_is_exercised() {
    TEST(㉔ 不变量：返回 true ⇒ FK(解) 必须真的等于目标（可达若干 + 一组因【步长预算】被拒）);
    // 三组输入**刻意覆盖两个分支**（缺一类就有个负对照打不到，见下面 ⚠）：
    //   c0/c1 —— 奇异位姿 + 附近可达目标（60°）⇒ 解应当出来 ⇒ 走 else 分支（把 FK 回验执行到）；
    //   c2    —— 奇异位姿 + 极端目标（179°）⇒ **步长预算**内解不出来 ⇒ 走 fail 分支（把"退回参照"执行到）。
    //     ⚠ c2 被拒的机制是【24 轮 × 3°/轮 = 72° 的步长预算】，**不是**腕部奇异的几何不可达 ——
    //       实测：把 `maxStepDeg` 换成 30° 后 c2 **解得出来**（三组全 ok=1，FK 回验仍绿）。
    //       这里只断言"返回 true ⇒ FK 到得了目标"这条不变量，**不**断言"该位姿必须被拒"
    //       （后者会让用例替实现背书，且结构上无法变红）。
    // ⚠ 为什么两类【必须】都在（这是本轮实测出来的，不是推的）：
    //   · 若只有"解得出来"的位姿：删末尾自验门**不会**让本用例红 —— 收敛在循环里就发生了，
    //     自验门**没被走到**（主对照空转）；
    //   · 若只有"解不出来"的位姿："在 return true 前把解弄错"那个兜底对照**不会**让本用例红
    //     —— 它走 fail 分支，够不到 return true。
    //   两个负对照各需要一类 ⇒ 两类都要在。哪一组解出来/解不出来的探针记录见 task-3-report.md。
    // ★★ 2026-09-29 Task 3 复审 I-1：上面那条"两类都要在"以前**只写在注释里** —— 那是**空的**：
    //   c2 落进 fail 分支靠的是步长预算（`BTN2_WRIST_MAX_ITER × maxStepDeg`），而 `maxStepDeg`
    //   是**明确可调的入参**、`BTN2_WRIST_MAX_ITER` 是普通常数 ⇒ 谁把任一个调大，c2 就解得出来
    //   ⇒ fail 分支（连同对照 D 的目标）**静默失效**、对照 A 也不再能红本用例，而**没有任何东西会报**。
    //   下面 `failedCount` / `solvedCount` 就是给这条"空转"上的守卫。
    //   ⚠ 2026-09-29 Task 4 订正：这句以前写的是"（改 `>= 99` 会立刻红，见 report 的负对照）"——
    //     **那条对照已被本轮复审判定为"不够"**：它只证明"这行会被执行、且能失败"，而本仓的
    //     标准是**对照必须复现"守卫存在的那个场景"**。`>= 99` 是把门限人为抬到不可能满足，
    //     它连"入参被调大 ⇒ 失败分支静默失效"这条真实路径都没碰到（改回去仍是原值）。
    //   ⇒ 本守卫**真正**对应的对照是：**把 c0/c1 的 `maxStepDeg` 临时调小**（例如 0.5°/轮，
    //     预算 24×0.5 = 12°），使三组输入**全落 `!ok`** ⇒ `failedCount = 3 >= 1` 照样通过、
    //     六条 `out == ref` 也照样通过，而 `solvedCount = 0 >= 2` **必须红**。
    //     这正是上面那段描述的"成功分支空转"场景的复现，而不是一个人造门限。
    const double refs[3][6] = {{0,0,0,0,0,0}, {0,0,0,0,180,0}, {0,0,0,0,0,0}};
    const double bigs[3][3] = {{0,60,0}, {0,0,60}, {0,179,0}};
    int failedCount = 0;   // ★ 抗空转：走 fail 分支的输入个数（见上面 I-1 那段）
    int solvedCount = 0;   // ★ 抗空转：走 else 分支的输入个数 —— 与 failedCount **成对**，
                           //   两条一起才叫"两个分支都真的走过"（见下面守卫处的 I-1 补充）
    for (int c = 0; c < 3; ++c) {
        double Rr[9]; fkR(refs[c], Rr);
        double Md[9], Rt[9];
        button2RotVecToMatDeg(bigs[c], Md);
        button2Mat3Mul(Md, Rr, Rt);                              // 目标 = 世界系旋转 · FK(ref)
        double out[6] = {9,9,9,9,9,9};                           // 哨兵：失败时必须被覆盖成 ref
        const bool ok = button2SolveWrist(refs[c], Rt, Config::ORIENT_MAX_STEP_DEG, out);
        if (!ok) {
            ++failedCount;
            for (int i = 0; i < 6; ++i) CHECK(out[i] == refs[c][i]);   // 失败 ⇒ 必须原样退回参照
        } else {
            ++solvedCount;
            double Ro[9]; fkR(out, Ro);
            CHECK(angBetweenDeg(Rt, Ro) < Config::BTN2_WRIST_TOL_DEG); // ★ 真的不变量（**能红**）
            for (int i = 0; i < 3; ++i) CHECK(out[i] == refs[c][i]);   // 且 J1/J2/J3 仍不动
        }
    }
    // ★ 抗空转守卫（两条，成对；缺一条就有一个静默失效方向没人管）：
    //   · `failedCount >= 1` 防的是【失败分支空转】：它靠的是"有一组输入解不出来"，
    //     而那取决于 maxStepDeg 与 BTN2_WRIST_MAX_ITER ⇒ 谁把它们调大，本条就红着告诉你
    //     "失败分支死了"（对照 D 的目标随之失效），而不是静默空转。
    //   · `solvedCount >= 2` 防的是【成功分支空转】——**本轮复审 I-1 补的那条**：
    //     如果 c0/c1 **也不再可解**（同一条"入参可调"的路：把预算调小，或把 `bigs` 那两组
    //     的目标调大），三组输入会**全落进 `!ok`** ⇒ `failedCount = 3 >= 1` 照样通过、
    //     六条 `out == ref` 也照样通过；而 `else` 里那句 `angBetweenDeg(Rt, Ro) < TOL`
    //     是本用例**唯一能红的断言**，它一次都不会被执行 ⇒ 本用例在【一条 FK 回验都没跑】
    //     的情况下变绿，**连主对照 A（删末尾自验门）也不再能把它染红**。
    //     要求 `>= 2`（不是 `>= 1`）是因为 c0 与 c1 是**两组不同的**输入，两组都必须真的
    //     走到那条 FK 回验；只要求 1 会漏掉"其中一组退化成不可解"的情形。
    CHECK(failedCount >= 1);
    CHECK(solvedCount >= 2);
    PASS();
}

// ============================================================================
//  ㉕ 2026-09-29 Task 4：对外入口 `button2OrientJointTarget`（Task 2 + Task 3 拼起来）
// ============================================================================
// ⚠ 这三个用例**必须在 `fkR` / `angBetweenDeg` 的定义（上面 :1131/:1136）之后** ——
//   它们定义在文件中间，㉕(a) 要用；放在更靠前的位置就得自己处理声明顺序（**不许**为了
//   省事再写第二份实现）。放在文件末尾（㉔ 之后、main 之前）天然满足这一条。

// 签名里【没有位置入参】是编译期性质，用函数指针类型钉住（与旧函数用例③同一招）。
// ★ 2026-09-29 整支终审 I-1：返回类型从 `void` 改成 `Btn2JointResult` ⇒ 这个钉子**如设计那样
//   先编译不过**（改动当场实测，原文存 `.superpowers/sdd/task-final-B-report.md`）：
//     error C2440: “初始化”: 无法从“Btn2JointResult (__cdecl *)(const double [],const double
//     [],const double [],double [])”转换为“Btn2OrientSig”
//   ⇒ 这一行**同步**成新签名（钉子照旧只钉"没有位置入参"这件事，返回类型只是跟着走）。
typedef Btn2JointResult (*Btn2OrientSig)(const double[6], const double[3], const double[3], double[6]);

// ㉕(a) ★ 端到端：任意参照位姿 + 任意小摆动 ⇒ FK(解) 必须真的到得了目标。
//    【它与 ㉓(a) 的分工】㉓(a) 喂的是"**直接构造的**目标矩阵"；这一条喂的是"**笔杆角度**"，
//      中间隔着 Task 2 的整条流水线 ⇒ 它把 Task 2 与 Task 3 **串起来**验：
//      **不依赖 任何符号常数**（它比的是"Task 2 算出来的目标"与"Task 3 解出来的朝向"，
//      两边都走同一套常数 ⇒ 符号全翻也绿；符号的账由 Task 2 那一组与上机去结）。
//    ⚠ 判据写法分两种，别混：三条 `out[i] == ref[i]`（J1/J2/J3 保持）**写在循环里** ——
//      第一个不满足的样本立刻红，报出的是本质判据；而 FK 匹配那一条恰恰是**累加**的
//      （循环里 `if (...) ++ok;`，循环外才 `CHECK(ok >= 55)`）。
//    ⚠ `ok >= 55` 是**防空转**：若入口恒返回参照，60 组里能对上目标的只会是碰巧，多半全落。
//      容许少数位姿真的解不出来（那正是自验门该拒的）。**实测余量：ok = 58/60（只差 3 才到限）**
//      —— 余量薄，但种子固定 ⇒ 不 flake；若 `Config` 里步长预算/收敛门限那几个常数被调紧，
//      这条会先红，那正是它该说的话。
//    ⚠★ 2026-09-29 Task 4 负对照实测（简报 Step 4(a) 那条**不红**，别照抄它当证据）：
//      把入口**第一句** `outJoints[0] = refJoints[0];` 改成 `+ 1.0` ⇒ **整套仍 36/0 全绿**。
//      原因：`button2SolveWrist` 自己第一句写 `out[i] = ref[i]`、成功路径又再写
//      `out[0..2] = ref[0..2]` ⇒ 入口的三条出口**每一条**都会把这三个下标重写成参照
//      （详见 `Button2Joint.cpp` 那段 ⚠★）。⇒ **本用例"J1/J2/J3 保持"这条断言能红的对照是**：
//      把求解失败的回退 `outJoints[i] = refJoints[i];` 改成 `= 0.0` ⇒ 立刻红在
//      `FAIL: out[0] == ref[0]`（实测 35/1）。那条才是这条不变量在本组合里真正被执行到的路径。
static void test_orient_joint_end_to_end_reaches_target() {
    TEST(㉕(a) ★ 端到端：任意参照位姿 + 任意小摆动 ⇒ FK(解) 必须真的到得了目标);
    unsigned seed = 20260929u;
    auto rnd = [&seed]() { seed = seed*1103515245u + 12345u; return (double)((seed>>16)&0x7FFF)/32767.0; };
    int ok = 0, ran = 0;
    int okOk = 0;   // ★ I-1：报告 `Ok` 的组数（见循环末尾与它那三条断言）
    for (int t = 0; t < 60; ++t) {
        double ref[6] = { rnd()*720-360, rnd()*720-360, rnd()*310-155,
                          rnd()*720-360, rnd()*720-360, rnd()*720-360 };
        // 笔杆从"单位姿态"转到"绕器件某轴 ≤ ~26°"（rv 三分量各在 ±15° ⇒ 模长上限 15√3 ≈ 26°）
        //   ——用测试侧的 matToZyRpy 造输入（别再写第二份！）。与 makeRandomReachTarget 的措辞一致。
        double Rs[9], Rd[9], Rsc[9], cur[3];
        TcpCalibration::rpyToMatrix(0,0,0, Rs);
        const double rv[3] = { rnd()*30-15, rnd()*30-15, rnd()*30-15 };
        button2RotVecToMatDeg(rv, Rd);
        button2Mat3Mul(Rs, Rd, Rsc);
        matToZyRpy(Rsc, cur);
        const double refStylus[3] = {0,0,0};

        double out[6];
        const Btn2JointResult res = button2OrientJointTarget(ref, refStylus, cur, out);
        ++ran;
        // J1/J2/J3 必须逐位不变（**三种结局都成立**：Ok 写参照、另两档也写参照 ⇒ 无条件的）
        CHECK(out[0] == ref[0]);
        CHECK(out[1] == ref[1]);
        CHECK(out[2] == ref[2]);
        // 目标朝向（用同一套纯函数算一遍）与 FK(解) 必须一致
        double refR[9]; fkR(ref, refR);
        double tgtR[9];
        button2OrientTarget(refR, refStylus, cur, Config::BTN2_TILT_PHI_DEG, tgtR);
        double outR[9]; fkR(out, outR);
        if (angBetweenDeg(tgtR, outR) < Config::BTN2_WRIST_TOL_DEG) ++ok;
        // ★ 2026-09-29 整支终审 I-1：返回值必须与"FK 真的到得了目标"**逐组一致** ——
        //   报 `Ok` 的组数必须**恰好**等于 FK 命中目标的组数。两边都会变红的方向是分开的：
        //     · 少报（解出来了却回 `SolveFailed`）⇒ `okOk < ok`；
        //     · 多报（没解出来却回 `Ok`）⇒ `okOk > ok`（这一条正是本条要防的"无声失败"）。
        //   ⚠ 不写成循环里的 `CHECK(res == Ok)`：那会**取消**本用例原有的容忍度
        //     （实测 60 组里有 2 组本来就解不出来、由自验门拒掉 —— 见上面的 `ok >= 55`）。
        if (res == Btn2JointResult::Ok) ++okOk;
    }
    CHECK(ran == 60);
    CHECK(ok >= 55);      // 容许少数位姿真的解不出来（那正是自验门该拒的）
    CHECK(okOk >= 55);    // 同上：报告成功的组数也要够多（防空转）
    CHECK(okOk == ok);    // ★ I-1：返回值与"真的到得了目标"必须逐组一致
    PASS();
}

// ㉕(b) 笔杆不动 ⇒ 六关节逐位不动（含"输入退化成恒等"这一档）；NaN/Inf ⇒ 六位全回参照。
//    【为什么"不动"这一档是本质的】笔杆不动 ⇒ rv = 0 ⇒ 目标 = 参照姿态 ⇒ 解 = 参照。
//      它是入口的**恒等情形**，也是"手停住时臂就该停住"这条手感要求的实现路径。
//    ⚠ 三段（不动 / NaN 在 curStylus / Inf 在 refStylus）**刻意覆盖守卫的两个入参位置**：
//      只测一边会漏掉"另一半守卫写反了"（旧函数 ⑥ 因同一理由也分了三段）。
static void test_orient_joint_identity_and_nan() {
    TEST(㉕(b) 笔杆不动 ⇒ 六关节逐位不动；NaN/Inf ⇒ 六位全回参照);
    const double ref[6] = {10, -20, 30, 40, -50, 60};
    double out[6];
    {   // 不动
        const double s[3] = {0,0,0};
        // ★ I-1：笔杆不动 ⇒ 目标就是参照 ⇒ 恒等求解**成功**，必须报 `Ok`（**不是** `SolveFailed`）。
        //   ⚠ 这一条与本文件新增的 ㉕(d) 是一对：**两者的 `outJoints` 逐位相同**（都是参照）
        //     ⇒ 返回值是**唯一**能分开"没动"与"没解出来"的东西（这正是 I-1 要修的那件事）。
        CHECK(button2OrientJointTarget(ref, s, s, out) == Btn2JointResult::Ok);
        for (int i = 0; i < 6; ++i) CHECK(out[i] == ref[i]);
    }
    // ⚠★ 【本段的牙齿在 I-1 之后换了来源 —— 这条历史仍值得读，别按旧结论照抄】
    //    I-1 之前，本段的断言**只有一条**"六位逐位等于参照"，而那条**没有**牙齿：实测
    //    （`task-4-report.md` §3.3）把守卫那个 `if (stylusBad) { … return; }` **整块删掉**，
    //    本段**照旧全绿** —— 因为不进守卫也照样落回 `out == ref`：`rpyToMatrix(NaN,…)` ⇒
    //    非有限 `dR` ⇒ `rotVecDeg` 落进**最后一个退化分支** ⇒ `rv = {0,0,0}` ⇒ `ω = 0` ⇒
    //    `targetR == refR` ⇒ 求解器在恒等目标上**第 0 轮就 break** ⇒ `out == ref`。
    //    当时给它牙齿的只能是"改守卫**写入的值**"那条对照（报告 §3.4）。
    //  ★ 2026-09-29 整支终审 I-1 之后：本段两档**各多了一条返回值断言**
    //    （`== StylusUntrustworthy`）⇒ **删掉守卫这一支现在会当场红**：删了之后 NaN 输入会
    //    一路走到底、在恒等目标上**成功**（上面刚推导过）⇒ 返回 `Ok` ⇒ 本段 `FAIL`。
    //    ⚠ 那条对照**实测过**（原文见 `.superpowers/sdd/task-final-B-report.md` §4 负对照 B：
    //      `FAIL: … == Btn2JointResult::StylusUntrustworthy`，37 passed / 1 failed），不是推断
    //      —— 这正是"含糊的返回值把诊断成本推给操作员"的反面：有了它，
    //      "守卫在不在"这件事第一次有了自动化证据。
    //    ⚠ 别把牙齿归给"求解失败回退"那条对照：它红的是**㉕(a)**（`FAIL: out[0] == ref[0]`，
    //      报告 §3.2），而本段这两档输入在守卫在时就**已返回**；就算守卫被删也是恒等目标、
    //      第 0 轮 break ⇒ 成功 —— **这条回退根本不在本段的路线上**（它现在只会经返回值显形）。
    {   // NaN 在 curStylus
        const double s0[3] = {0,0,0};
        const double sN[3] = {0, std::numeric_limits<double>::quiet_NaN(), 0};
        // ★ I-1：守卫那条出口必须报 `StylusUntrustworthy`（**不是** `SolveFailed`：
        //   守卫在求解**之前**就返回了 ⇒ 报成求解失败会把操作员指到错的地方去）。
        CHECK(button2OrientJointTarget(ref, s0, sN, out) == Btn2JointResult::StylusUntrustworthy);
        for (int i = 0; i < 6; ++i) CHECK(out[i] == ref[i]);
    }
    {   // Inf 在 refStylus
        const double sI[3] = {std::numeric_limits<double>::infinity(), 0, 0};
        const double s0[3] = {0,0,0};
        CHECK(button2OrientJointTarget(ref, sI, s0, out) == Btn2JointResult::StylusUntrustworthy);
        for (int i = 0; i < 6; ++i) CHECK(out[i] == ref[i]);
    }
    PASS();
}

// ㉕(c) 签名里没有位置入参（编译期性质）。
//    【为什么这条必须单独存在】"笔杆平移 ⇒ 机械臂不动"在本方案里是**结构性**保证：函数根本
//      没有位置分量可吃。结构性质运行期测不出来 ⇒ 只能钉在类型上：谁给入口加第 5 个（位置）
//      参数，下面这行**立刻编译不过**。`fp != nullptr` 那个断言只是让这条用例在运行期也有
//      一个 PASS 可数（真正干活的是**这一行的编译**）。
static void test_orient_joint_signature_has_no_position_input() {
    TEST(㉕(c) 签名里没有位置入参（编译期性质）);
    Btn2OrientSig fp = &button2OrientJointTarget;
    CHECK(fp != nullptr);
    PASS();
}

// ㉕(d) ★ 2026-09-29 整支终审 I-1：**求解失败**必须能被调用方看见（返回值），且六位逐位回参照。
//    【为什么非有不可】这正是 I-1 要修的缺陷：三条出口**都只写六个数**、而返回值原来根本不存在
//      ⇒ 现场"臂不再跟手、往按下位姿收回去"时**控制台一声不响**，谁也分不出是没解出来。
//    【怎么造出"解不出来"的输入 —— 实测出来的机制，不是猜的】笔杆绕器件 X 转 150° ⇒
//      ΔR 的旋转角 = 150° ⇒ 目标朝向离参照 **150°**；而求解器一轮最多走
//      `Config::ORIENT_MAX_STEP_DEG`（现 3°）、最多 `Config::BTN2_WRIST_MAX_ITER`（现 24）轮
//      ⇒ **步长预算 72° < 150°** ⇒ 解不到，自验门判 false。实测探针（记录见 report）：
//      同一个目标把 `maxStepDeg` 换成 12° 就**解得出来** ⇒ 拒发的原因是**预算**，
//      **不是**几何不可达 ✓（这两句话的来源见 report，不是推的）。
//    ⚠ 150° 正是 `Config::ORIENT_MAX_OFFSET_DEG` 的**上限**（限幅只在 `th > 150` 时才缩比
//      ⇒ 150 原样保留）⇒ 这是**入口能造出的最远目标**。用字面量 150 而不是那个常数：
//      常数被调小时目标会跟着缩、反而落回预算内 ⇒ 那会让本用例静默失去意义。
//    ⚠ 这个输入**挂在两个预算常数上**（与 ㉔ 的 c2 同一类）：谁把 `ORIENT_MAX_STEP_DEG`
//      或 `BTN2_WRIST_MAX_ITER` 调到 24×step ≥ 150，本用例会红着说"这条输入不再解不出来"
//      —— 那正是它该说的话（否则下面的失败分支会**静默**变成空转）。**别**为了让它绿去改 150。
//    ⚠ `refStylus`/`curStylus` 全为**有限**值 ⇒ 本用例走的**不是** NaN 守卫那条路
//      （那一条由 ㉕(b) 钉）⇒ 两档失败在返回值上被分开断言。
static void test_orient_joint_solve_failure_is_visible_and_falls_back() {
    TEST(㉕(d) ★ 求解失败 ⇒ 返回 `SolveFailed` 且六位逐位回参照（I-1）);
    // 与 ㉕(b) 同一个参照：本用例与它**只差在返回值上**（两边的 `outJoints` 都是参照）
    //   ⇒ 一对用例合起来证明"返回值是唯一能分开'没动'与'没解出来'的东西"。
    const double ref[6] = {10, -20, 30, 40, -50, 60};
    const double s0[3] = {0, 0, 0};
    const double sFar[3] = {150.0, 0, 0};   // 绕器件 X 转 150°（见上面那段：超出步长预算）
    double out[6] = {9,9,9,9,9,9};          // 哨兵：失败时必须被覆盖成参照
    const Btn2JointResult res = button2OrientJointTarget(ref, s0, sFar, out);
    CHECK(res == Btn2JointResult::SolveFailed);
    for (int i = 0; i < 6; ++i) CHECK(out[i] == ref[i]);   // 逐位（`==`，不是"接近"）
    // 另一组参照（全 0 的腕部姿态是奇异位姿，是"腕部求解"最容易被拒的地方）
    const double ref0[6] = {0,0,0,0,0,0};
    double out0[6] = {9,9,9,9,9,9};
    CHECK(button2OrientJointTarget(ref0, s0, sFar, out0) == Btn2JointResult::SolveFailed);
    for (int i = 0; i < 6; ++i) CHECK(out0[i] == ref0[i]);
    PASS();
}

// ㉖ ★ 端到端：纯自转（器件 Z 转 20°）⇒ 【只动 J6】。
//    【为什么非补不可】㉑ 只断言**目标朝向**的第三列不变、㉕(a) 只断言 `FK(解) ≈ 目标`
//      —— 两者都**没有**断言"解出来的关节只动了 J6"。而球腕下"右乘 Rz(roll) ⇒ 只动 J6"
//      是可离线证明的，也是用户定下的规格（"自转沿用原来的 J6 方案"）⇒ 需要一条端到端断言
//      把它钉住：喂一个**纯自转**的笔杆输入，经 `button2OrientJointTarget` 走完整条流水线
//      （Task 2 的目标朝向 + Task 3 的腕部求解），断言解出来的 J4/J5 几乎不动、
//      而 J6 恰好是 `BTN2_ROLL_SIGN × 20°`。
//    【为什么要有第二个（非正位）参照】正位上"只动 J6"最容易成立；第二个取**非正位**的参照
//      关节姿态 + **非零**参照笔杆姿态（现场那个按下姿态）⇒ 只测正位的话，"自转随姿态散到
//      别的关节上"这个病**测不出来**（与 ⑦ 存在的理由同一条）。
//    【容差怎么选】实测两组的残差：J4 ≤ 1.9e-10°、J5 ≤ 2.9e-11°、J6 ≤ 5.6e-10°（度）；
//      而真实的串扰是**度级**的（旧的逐欧拉路把自转 10° 送成 J4 +1.52 / J5 +5.48，
//      见 ⑦ 的说明）⇒ 取 `kTolDeg = 1e-6`：比实测残差大 4 个数量级、比任何真实串扰小
//      6 个数量级 ⇒ 能干净区分"只动 J6"与"三个都动"，且不挂在任何单个测量值上。
//    ⚠ 期望值经 `Config::BTN2_ROLL_SIGN` 表达，【一个 ±1 都不写死】（与文件头 ② 同一条规矩）。
static void test_roll_end_to_end_moves_only_j6() {
    TEST(㉖ ★ 端到端：纯自转（器件 Z 转 20°）⇒ 只动 J6);
    const double kTolDeg = 1e-6;      // 见上面【容差怎么选】
    const double poses[2][6] = {
        {0,0,0,0,-90,0}, {30,-60,45,20,-70,10}
    };
    const double stylus[2][3] = {
        {0,0,0}, {-58.93, 11.83, -6.41}
    };
    const double rollDeg = 20.0;
    for (int p = 0; p < 2; ++p) {
        // 造输入：绕【器件 Z】转 rollDeg ⇒ curStylus = R_stylus · Rz(rollDeg)
        //   （用测试侧既有的 bodyAxisEuler —— 它内部走 matToZyRpy，别再写第二份矩阵→欧拉）
        double cur[3];
        bodyAxisEuler(stylus[p], 2, rollDeg, cur);

        double out[6];
        button2OrientJointTarget(poses[p], stylus[p], cur, out);

        // ★ 本用例的核心判据：J4/J5 几乎不动（容差见上），J6 恰为 ROLL_SIGN × 20°
        CHECK(std::fabs(out[3] - poses[p][3]) < kTolDeg);
        CHECK(std::fabs(out[4] - poses[p][4]) < kTolDeg);
        CHECK(std::fabs((out[5] - poses[p][5]) - Config::BTN2_ROLL_SIGN * rollDeg) < kTolDeg);
        // J1/J2/J3 逐位不变
        CHECK(out[0] == poses[p][0]);
        CHECK(out[1] == poses[p][1]);
        CHECK(out[2] == poses[p][2]);
        // 自检：解真的成立（FK(解) ≈ 目标）。若求解器被拒，out 会退回参照 ⇒ 上面 J6 那条本就会红；
        //   这一句把"目标与 FK 的来源是同一个量"一并说清楚，并排除"坏夹具导致空转"。
        double refR[9]; fkR(poses[p], refR);
        double tgtR[9];
        button2OrientTarget(refR, stylus[p], cur, Config::BTN2_TILT_PHI_DEG, tgtR);
        double outR[9]; fkR(out, outR);
        CHECK(angBetweenDeg(tgtR, outR) < Config::BTN2_WRIST_TOL_DEG);
    }
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
    test_rotvec_matrix_roundtrip();                        // ⑲(a) 2026-09-29 Task 1：3×3 纯算术地基
    test_rotvec_matrix_specific_rx90();                    // ⑲(b)
    test_rotvec_mat3_mul_and_transpose();                  // ⑲(c)
    test_rotvec_mat3_inverse();                            // ⑲(d)
    test_mat_to_rpy_roundtrip_on_task2_inputs();           // ⑳x 修复轮 F1：矩阵→欧拉统一后的往返自检
    test_tilt_target_equals_world_rotation_times_ref();    // ⑳(a) 2026-09-29 Task 2：姿态目标（展开式断言）
    test_tilt_moves_tip_toward_base_plus_y_in_any_pose();  // ⑳(b) ★ 验收判据（F2 拆开后独立失败）
    test_deadzone_swallows_below_and_admits_exactly_at_threshold();  // ⑳b
    test_offset_cap_scales_rotation_angle_and_keeps_the_axis();      // ⑳c
    test_phi_is_not_a_dead_constant();                     // ⑳d ★ 专治"常数没被用上"
    test_roll_only_spins_around_the_styluses_own_axis();   // ㉑
    test_solve_wrist_identity_target_returns_reference();        // ㉒ 2026-09-29 Task 3：腕部求解
    test_solve_wrist_fk_roundtrip_across_random_poses();         // ㉓(a) ★ 跨位姿 FK 回验
    test_solve_wrist_keeps_base_joints_across_random_poses();    // ㉓(b) 只动腕
    test_solve_wrist_invariant_holds_and_budget_refusal_is_exercised();   // ㉔ ★ 不变量（能红）
    test_orient_joint_end_to_end_reaches_target();                 // ㉕(a) 2026-09-29 Task 4：端到端
    test_orient_joint_identity_and_nan();                          // ㉕(b)
    test_orient_joint_signature_has_no_position_input();           // ㉕(c) ★ 编译期钉子
    test_orient_joint_solve_failure_is_visible_and_falls_back();   // ㉕(d) ★ I-1：求解失败可见
    test_roll_end_to_end_moves_only_j6();                          // ㉖ ★ 端到端：纯自转 ⇒ 只动 J6

    std::cout << "\nResults: " << g_passed << " passed, " << g_failed << " failed" << std::endl;
    return g_failed == 0 ? 0 : 1;
}
