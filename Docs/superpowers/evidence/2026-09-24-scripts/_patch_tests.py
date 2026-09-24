# -*- coding: utf-8 -*-
"""把 test_button2_joint.cpp 的期望值改到新语义（真 ΔR 器件系旋转向量）。
自下而上替换，每段先校验锚点，不符就中止 —— 不硬改。"""
import io, sys

P = r'D:\Projects\Touch\Touch_Client\tests\test_button2_joint.cpp'
src = io.open(P, encoding='utf-8', newline='').read()
L = src.split('\n')


def blk(a, b):
    """1-indexed 闭区间"""
    return '\n'.join(L[a - 1:b])


def must(a, b, needle, what):
    got = blk(a, b)
    if needle not in got:
        print('!! 中止：%s 的 %d..%d 里找不到锚点 %r' % (what, a, b, needle))
        print('   实际前 200 字：', got[:200].replace('\n', ' | '))
        sys.exit(1)


# ---------- ⑧ 337：远越界那一路的容差（混合入参下 rv 与欧拉差有 O(θ²) 差别）----------
must(337, 337, 'Config::BTN2_J4_SIGN * 5.0', '⑧(a)')
L[336] = ("    // ⚠ 容差 1e-2 而【不是】1e-6：2026-09-24 换成真 ΔR 之后，输入里 X 与 Z 两轴混在一起，\n"
          "    //   rv.x 与欧拉差 ΔRx 相差 O(5°×0.04°) ≈ 3.5e-3° ⇒ 1e-6 会恒红。\n"
          "    //   判别力不在这句上：真正区分逐轴门与聚合门的是下面那句 `out[4] == ref[4]`。\n"
          "    CHECK(fabs((out[3] - ref[3]) - Config::BTN2_J4_SIGN * 5.0) < 1e-2);")

# ---------- ⑦ 282..304：三轴同时 → 改成"倾斜参照下逐器件轴"（★ 现场回归）----------
must(282, 304, 'test_three_axes_at_once_are_independent_at_large_angles', '⑦')
must(282, 304, 'const double dRx = +120.0', '⑦ 尾部')
NEW7 = '''// ⑦ ★★ 2026-09-24 重写：**倾斜参照下的三根器件轴**（这是现场那个症状的回归用例）。
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
}'''
L[281:304] = NEW7.split('\n')

# ---------- ⑤ 209..218：门限边界（(c) 拆成"逐轴精确"+"混合轴映射"）----------
must(209, 218, '恰好等于门限', '⑤(c)')
NEW5 = '''    // (c) 恰好等于门限 ⇒ 放行，且**单轴**时增量恰为 符号×偏移（单轴是精确的：
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

    // (d) 三轴同时**恰好等于**门限、取不同正负 ⇒ 三路都放行、且各自的符号对得上。
    //     ⚠ 这里容差是 1e-3 而不是 1e-12：真 ΔR 的旋转向量与逐欧拉差相差 O(θ²)
    //       （θ = dz = 0.05° ⇒ 约 4e-5°），三轴混合时那一项就会出现。**不是实现不准**。
    //     ⚠ 它仍然有判别力：若 Y 与 Z 两路被接反，(d) 里 J5/J6 的符号会同时翻。
    {
        const double cur[3] = {+dz, -dz, +dz};
        button2JointTarget(ref, refS, cur, out);
        CHECK(fabs((out[3] - ref[3]) - Config::BTN2_J4_SIGN * (+dz)) < 1e-3);   // 器件 X -> J4
        CHECK(fabs((out[4] - ref[4]) - Config::BTN2_J5_SIGN * (-dz)) < 1e-3);   // 器件 Y -> J5
        CHECK(fabs((out[5] - ref[5]) - Config::BTN2_J6_SIGN * (+dz)) < 1e-3);   // 器件 Z -> J6
    }'''
L[208:218] = NEW5.split('\n')

# ---------- ④ 137..184：限幅（旋转向量量程 ≤180，输入要按这个量程造）----------
must(137, 184, 'test_oversized_stylus_rotation_is_clamped_per_joint', '④')
NEW4 = '''// ④ 偏移限幅：笔杆转过的角度**超过上限** ⇒ 关节增量**恰为**上限。
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
}'''
L[136:184] = NEW4.split('\n')

# ---------- ① 58..103：三根器件轴的对应（Y/Z 两路与从前互换）----------
must(58, 103, 'each_stylus_axis_moves_exactly_one_joint', '①')
must(79, 101, '左右摆', '①(b)(c)')
NEW1 = '''// ① ★ 结构判据（本方案的核心，且与符号无关）：一次只动笔杆一根轴 ⇒ 恰好一个关节动。
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
}'''
L[57:103] = NEW1.split('\n')

# ---------- 新增 ⑱（跨 ±180）：插在 int main() 之前 ----------
i_main = next(i for i, l in enumerate(L) if l.strip().startswith('int main()'))
NEWBLOCK = '''// ===== 测试侧的【输入构造器】（不是被测逻辑）=====
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

'''
HELPERS, TEST18 = NEWBLOCK.split('// ⑱ ★ 跨 ±180')
TEST18 = '// ⑱ ★ 跨 ±180' + TEST18

# 构造器必须在【第一个用到它的用例】之前（C++ 要求先声明）⇒ 插在"===== 用例 ====="之后
i_use = next(i for i, l in enumerate(L) if '===== 用例 =====' in l)
L[i_use + 1:i_use + 1] = HELPERS.split('\n')

# ⑱ 放在 int main() 之前
i_main = next(i for i, l in enumerate(L) if l.strip().startswith('int main()'))
L[i_main:i_main] = TEST18.split('\n')

# ---------- main：登记新用例 + 换掉过期的横幅 ----------
i_main = next(i for i, l in enumerate(L) if l.strip().startswith('int main()'))
for i in range(i_main, min(i_main + 25, len(L))):
    if 'test_accumulator_reaches_a_large_target_over_several_frames();' in L[i]:
        L[i] = (L[i] + '\n    test_crossing_the_pm180_seam_is_only_two_degrees();'
                       '   // ⑱ 2026-09-24 换实现后新增（跨 ±180）')
        break
for i in range(i_main, min(i_main + 5, len(L))):
    if '笔杆 Euler 增量' in L[i]:
        L[i] = '    std::cout << "--- Button2Joint (按钮2 关节空间映射：笔杆姿态增量 -> 关节增量) ---" << std::endl;'
        break

io.open(P, 'w', encoding='utf-8', newline='').write('\n'.join(L))
print('patch ok')
