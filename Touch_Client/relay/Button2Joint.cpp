#include "Button2Joint.h"
#include "../config/Config.h"
// ⚠ 2026-09-23 Task 2 修复轮：`isTrustworthyJointRef` 要用 `Kinematics::isWithinJointLimits`
//   ⇒ 这里多了一条依赖。它**不是** Win32/socket/appState —— Kinematics 的 FK/IK 也是纯算术，
//   本文件仍然"纯"。但这条链会走到 `CoordinateTransform.h` → `<HDU/hduVector.h>`
//   ⇒ **构建图的形状变了**（测试用的 .bat 要多给两个 /I、多链一个 .cpp），
//     见 `tests/build_button2_joint_test.bat` 的注释与 `Button2Joint.h` 的「依赖变了」那一段。
//   ⚠ 用**现成的** `isWithinJointLimits` 而不是在这里重抄一遍关节限位：那是**唯一真相源**
//     （`Kinematics.h` 的 `J*_MIN/MAX`），重抄一份就是两处会各自漂移的量。
#include "../robot/Kinematics.h"
// ★ 2026-09-24：欧拉->旋转矩阵的【唯一真相源】（R = Rz·Ry·Rx，输入为度）。
//   ⚠ 别在本文件里再展开一份矩阵：那就是"同一个约定两份实现"——本项目的老毛病
//     （`Config.h:732` 就写着两边必须同一个约定）。旧路 `relay/Button2Mapping.cpp`
//     用的也是它，有先例；代价是测试构建要多链一个 `calibration/TcpCalibration.cpp`
//     （见 `tests/build_button2_joint_test.bat`）。它只依赖 cmath/cstdio/cstring/cstdlib。
#include "../calibration/TcpCalibration.h"

// ============================================================================
//  笔杆 姿态增量（真 ΔR 的器件系旋转向量，2026-09-24 换） → 关节增量（纯函数）
// ============================================================================
// 逐行对照 Button2Joint.h 的「管线」那一节：偏移 → 逐轴死区 → 逐轴符号 → 逐关节限幅
// → 一一对应加到参照上。
//
// 映射表（**标定结论**，来源见头文件；这里重复一遍是因为它就是本文件的核心四行）：
//     R_ref = rpyToMatrix(refStylus)   R_cur = rpyToMatrix(curStylus)   （ZYX，输入为度）
//     ΔR  = R_ref^T · R_cur        ← 【体系】增量：取转置再右乘 ⇒ 轴表达在**器件系**
//     rv  = 旋转向量(ΔR)            ← 方向×角（度），|rv| ≤ 180
//     out[3] = ref[3] + clamp(SIGN_J4 * gate(rv.x))     // 器件 X（前后摆）
//     out[4] = ref[4] + clamp(SIGN_J5 * gate(rv.y))     // 器件 Y（左右摆）
//     out[5] = ref[5] + clamp(SIGN_J6 * gate(rv.z))     // 器件 Z（自转）
//     out[0..2] = ref[0..2]                             // J1/J2/J3 无条件保持
//
// ⚠ **为什么不再逐欧拉角作差**（2026-09-23 现场 + 2026-09-24 离线复算）:
//   三个动作在器件系里各是一根真实旋转轴（实测 自转≈器件 Z、|Z|≈0.99；前后摆≈X；左右摆≈Y），
//   而 ZYX 欧拉角的**三个分量会一起变**（同一个自转实测 ΔRy +12.11 / ΔRz −9.65 / ΔRx −1.43）
//   ⇒ 逐分量映射必然送三个关节。在按下姿态 (−58.93, 11.83, −6.41)° 下算出的增益矩阵（度/度）:
//       绕器件 X 转 10° ⇒ J4 +1.00 · J5  0.00 · J6  0.00
//       绕器件 Y 转 10° ⇒ J4 −0.22 · J5 −0.89 · J6 +0.50
//       绕器件 Z 转 10° ⇒ J4 +0.15 · J5 +0.55 · J6 +0.85   ⇒ 自转同时喂三关节（合计 1.55）
//   ⚠ 而 `绕器件 X 转 δ` ≡ `ΔRx = δ` 是【恒等式】（R·Rx(δ) 只把最内层欧拉角加 δ）
//     ⇒ X→J4 那一路的结构**本来就对**，坏的是另两路；这也是"J4 的符号可以沿用 09-23
//     那句判断"的依据（换成旋转向量后它的输入逐位不变）。
//   ⚠ 顺带消掉两个【数值】隐患（与耦合无关，是逐欧拉差特有的）:
//     ① 跨 ±180：ZYX 下 `Rz(179)→Rz(−179)` 的逐欧拉差是 −358°（被限幅夹到 −150 ⇒ 关节冲限位），
//        而真 ΔR 是 +2°（用例⑰钉住）；
//   ⚠★★ **但这条免疫力被上游架空了（2026-09-24 晚核出）**：`RelayCore` 侧算的偏移是
//     【欧拉分量相减】(`current − m_orientRefStylus`)，**而且那道低通也做在欧拉域**上。
//     笔杆 Rz 跨 ±180 时（实测到过 −171°，离接缝 9°）偏移会跳 ±360 ⇒ 低通把这一跳
//     **抹成 ~0.25 s 的斜坡**再喂进来 ⇒ 本函数收到的是一串**看起来平滑的假偏移**，
//     照样被 ±150 夹住、照样按 3°/帧 爬 ⇒ **冲限位那一幕依旧可能发生**。
//     ⇒ 真修法是把低通/死区搬到**旋转向量域**（那里没有接缝），那是 RelayCore 侧的重构，
//       **不在本函数职责内**，已记账。**别把本条读成"跨接缝已经安全了"。**
//     ② 万向锁：HapticCallback 在 |ry|→90° 时硬切换另一支提取并把 rz 强制置 0 ⇒ 逐欧拉差会
//        跳掉整个累积的 rz；真 ΔR 从矩阵算，与提取支路无关。
//
// ⚠ 旋转向量的量程天然 ≤180°（取主值旋转）⇒ ±150° 那个限幅只在"接近转半圈"时才咬得住。
//   这**不是**放松保护：越界由调用方的关节限位闸与 FK 位置门管（RelayCore.cpp 那三处 return）。

namespace {

// 逐轴响应门限：与 RelayCore 姿态块里的 `axisGate` **逐字同语义**（`>=` 门限才放行）。
// ⚠ 别改成 `>`：那会把"恰好等于门限"这一档吞掉，与旧路径分家（用例⑤(c) 钉住）。
inline double axisGate(double d) {
    return (fabs(d) >= Config::ORIENT_DEADZONE_DEG) ? d : 0.0;
}

// 逐关节偏移限幅。⚠ 夹的是**关节增量**，不是绝对关节角（见头文件「单位变了」）。
inline double clampJointDelta(double d) {
    if (d >  Config::ORIENT_MAX_OFFSET_DEG) return  Config::ORIENT_MAX_OFFSET_DEG;
    if (d < -Config::ORIENT_MAX_OFFSET_DEG) return -Config::ORIENT_MAX_OFFSET_DEG;
    return d;
}

// 三个笔杆分量是否都有限（NaN/Inf 守卫用）
inline bool allFinite3(const double v[3]) {
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

const double kR2D = 180.0 / 3.14159265358979323846;

// out = A^T * B（A、B、out 都是行主序 3x3 的 9 元）
// ★ 这一步就是"绕笔杆【自身】轴"：用 R_ref 的转置把增量搬到器件系，
//   于是"自转"表现为一根固定的轴（器件 Z），而不是随姿态变化的欧拉分量组合。
inline void matTmul(const double A[9], const double B[9], double out[9]) {
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            double s = 0.0;
            for (int k = 0; k < 3; ++k) s += A[k * 3 + r] * B[k * 3 + c];
            out[r * 3 + c] = s;
        }
    }
}

// 旋转矩阵 -> 旋转向量（方向 x 角，单位：度）。
// ⚠ 两个退化分支必须显式处理，否则 (≈0)/(≈0) 会给出 NaN 或离谱的轴：
//   · θ ≈ 0    ⇒ 全 0（没有转；这也是"笔杆不动 ⇒ 六关节不动"的实现路径）
//   · θ ≈ 180° ⇒ sinθ ≈ 0，反对称部分解不出轴 ⇒ 改用【对角线法】：
//                 R[i][i] = 2a_i² - 1 ⇒ |a_i| = sqrt((R_ii+1)/2)；取最大那一维为主、
//                 符号取正（(a,π) 与 (-a,π) 是同一个旋转），其余维由 R[i][j] = 2·a_i·a_j 解出。
//                 主值旋转的角上限就是 180° ⇒ 这个分支是【可达】的，不是理论摆设。
inline void rotVecDeg(const double R[9], double rv[3]) {
    const double tr = R[0] + R[4] + R[8];
    double c = (tr - 1.0) * 0.5;
    if (c > 1.0) c = 1.0;
    if (c < -1.0) c = -1.0;
    const double th = acos(c);                    // 弧度, 属于 [0, π]
    if (th < 1e-9) {                              // θ ≈ 0
        rv[0] = rv[1] = rv[2] = 0.0;
        return;
    }
    const double s = sin(th);
    if (s > 1e-6) {                               // 常规分支
        // ⚠ 分母里的 2 不能省：R32−R23 = 2·u1·sinθ（不是 u1·sinθ）。
        //   漏了它就是整体放大 2 倍 —— 2026-09-24 实测漏过一次，只有断言**具体数值**
        //   的用例抓住了它（`> 1e-6` 那种"动了就算过"的断言全瞎过）。
        const double k = th / (2.0 * s) * kR2D;
        rv[0] = (R[7] - R[5]) * k;
        rv[1] = (R[2] - R[6]) * k;
        rv[2] = (R[3] - R[1]) * k;
        return;
    }
    const double a0 = sqrt((R[0] + 1.0) * 0.5);   // θ ≈ 180°：对角线法
    const double a1 = sqrt((R[4] + 1.0) * 0.5);
    const double a2 = sqrt((R[8] + 1.0) * 0.5);
    double ax[3] = {0.0, 0.0, 0.0};
    if (a0 >= a1 && a0 >= a2 && a0 > 1e-9) {
        ax[0] = a0; ax[1] = R[1] / (2.0 * a0); ax[2] = R[2] / (2.0 * a0);
    } else if (a1 >= a2 && a1 > 1e-9) {
        ax[1] = a1; ax[0] = R[1] / (2.0 * a1); ax[2] = R[5] / (2.0 * a1);
    } else if (a2 > 1e-9) {
        ax[2] = a2; ax[0] = R[2] / (2.0 * a2); ax[1] = R[5] / (2.0 * a2);
    } else {                                      // 只可能出现在非旋转矩阵上
        rv[0] = rv[1] = rv[2] = 0.0;
        return;
    }
    const double n = (sqrt(ax[0] * ax[0] + ax[1] * ax[1] + ax[2] * ax[2]) > 1e-12)
                     ? sqrt(ax[0] * ax[0] + ax[1] * ax[1] + ax[2] * ax[2]) : 1.0;
    for (int i = 0; i < 3; ++i) rv[i] = th * kR2D * ax[i] / n;
}

}   // namespace

// 仅供测试：把文件内的 rotVecDeg 暴露出来（不改实现，只转发）
// ⚠ 必须放在匿名命名空间【之外】才有外部链接 —— 测试（test_button2_joint.cpp 的 ⑲(a)）
//   要拿 `button2RotVecToMatDeg` 造出的矩阵**经同一个 rotVecDeg** 还原，才能证明
//   "旋转向量 ↔ 矩阵"这一对在**同一份实现**下自洽。声明在 Button2Joint.h。
void button2RotVecDegForTest(const double R[9], double rv[3]) { rotVecDeg(R, rv); }

void button2JointTarget(const double refJoints[6],
                        const double refStylus[3],
                        const double curStylus[3],
                        double outJoints[6]) {
    // ---- 第一句就写 J1/J2/J3，且【无条件】--------------------------------------
    // 本方案的"保持性"就是这三行：它们在上面、在任何分支之前，且下面**任何分支都不许再写
    // 这三个下标** ⇒ 无论后面怎么早退/守卫，J1~J3 都逐位等于参照。
    // （用例①②⑥ 各从一个方向钉它：不动 / 单轴动 / 入参非有限。）
    outJoints[0] = refJoints[0];
    outJoints[1] = refJoints[1];
    outJoints[2] = refJoints[2];

    // ---- NaN/Inf 守卫 ---------------------------------------------------------
    // 笔杆侧的任一分量非有限 ⇒ 三根关节**都不动**（退回参照 = 臂原地保持）。
    // ⚠ 只守笔杆侧：`refJoints` 若非有限，本函数**原样传出去**（没有安全值可退，见头文件
    //   的契约那一段）—— 用例⑥(c) 断言的就是这个现状。
    // ⚠ 守卫放在这里（而不是在算完增量之后）：`gate(NaN)` 会因为 `fabs(NaN) >= dz` 为假而
    //   返回 0.0，看起来"也没事" —— 但那是**靠巧合**：NaN 比较的性质一变（或有人把门限写法
    //   换成 `!(< dz)`）就会静默地把 NaN 放进 ServoJ。显式守卫与那个巧合无关。
    if (!allFinite3(refStylus) || !allFinite3(curStylus)) {
        outJoints[3] = refJoints[3];
        outJoints[4] = refJoints[4];
        outJoints[5] = refJoints[5];
        return;
    }

    // ---- 三路各自：偏移 → 死区 → 符号 → 限幅 ---------------------------------
    // 每路只用**它自己那一根**笔杆轴 ⇒ 结构上一一对应、无交叉耦合（用例①⑦）。
    // ★★ 2026-09-24：这里从"逐欧拉角作差"换成【真 ΔR 的器件系旋转向量】。理由与实测见
    //   Button2Joint.h 的「为什么不再逐欧拉角作差」那一节（三条：耦合、跨 ±180、万向锁）。
    //   欧拉约定走唯一真相源 rpyToMatrix；ΔR = R_ref^T · R_cur 把增量表达在**器件系**
    //   ⇒ "自转"落成一根固定的轴（器件 Z），不再散到三个欧拉分量上。
    double Rref[9], Rcur[9], dR[9];
    TcpCalibration::rpyToMatrix(refStylus[0], refStylus[1], refStylus[2], Rref);
    TcpCalibration::rpyToMatrix(curStylus[0],   curStylus[1],   curStylus[2],   Rcur);
    matTmul(Rref, Rcur, dR);

    double rv[3];
    rotVecDeg(dR, rv);

    const double dJ4 = Config::BTN2_J4_SIGN * axisGate(rv[0]);   // 器件 X（前后摆）
    const double dJ5 = Config::BTN2_J5_SIGN * axisGate(rv[1]);   // 器件 Y（左右摆）
    const double dJ6 = Config::BTN2_J6_SIGN * axisGate(rv[2]);   // 器件 Z（自转）

    outJoints[3] = refJoints[3] + clampJointDelta(dJ4);
    outJoints[4] = refJoints[4] + clampJointDelta(dJ5);
    outJoints[5] = refJoints[5] + clampJointDelta(dJ6);
}

// ============================================================================
//  按钮2 姿态目标（2026-09-29 Task 2）：摆动锁基座 + 自转绕自身轴
// ============================================================================
// 契约、流水线（①~⑤）、以及"φ 为什么是入参"、"两个常数语义被重定"、"非有限入参没有守卫"
// 这几条**全部**写在 `Button2Joint.h` 上；这里只写"为什么正好是这几行"。
//
// ⚠ 左边三次连乘**各自一块独立缓冲**（`Mw` / `Rtilt` / `Mr` / `outR`）—— 不是风格，是契约：
//   `button2Mat3Mul` **别名不安全**（见头文件），`out` 与 `A`/`B` 同一块内存会算出一团糊。
//   摆动与自转那两处【左乘/右乘】的分工见头文件；对调会被 ⑳ / ㉑ 各自抓住（Task 2 Step 6 的负对照）。
void button2OrientTarget(const double refR[9], const double refStylus[3],
                         const double curStylus[3], double phiDeg, double outR[9]) {
    // ---- ①ΔR：器件系增量（与旧路 `button2JointTarget` 逐字同一条式子）----------------
    double Rref[9], Rcur[9], RrefT[9], dR[9];
    TcpCalibration::rpyToMatrix(refStylus[0], refStylus[1], refStylus[2], Rref);
    TcpCalibration::rpyToMatrix(curStylus[0],   curStylus[1],   curStylus[2],   Rcur);
    button2Mat3T(Rref, RrefT);
    button2Mat3Mul(RrefT, Rcur, dR);                    // ΔR = R_refᵀ · R_cur（器件系）

    double rv[3];
    rotVecDeg(dR, rv);                                  // 直接调文件内的 `rotVecDeg`

    // ---- ② 逐分量死区（`>=` 门限才放行，与旧实现 axisGate **逐字同语义**）--------------
    // ⚠ 写 `if (!(std::fabs(rv[i]) >= dz))` 而**不是** `if (std::fabs(rv[i]) < dz)`：
    //   后者对 NaN 会**放行**（NaN < dz 为假 ⇒ 不归零），前者对 NaN 会归零。两者在有限输入下
    //   完全等价，本条只是把 NaN 的落点钉在"归零"这一侧。`>` 语义（恰好等于门限时吞掉）
    //   由用例 ⑳b 的第二半钉住 —— 换成 `>` 那条用例必红（Task 2 的负对照之一）。
    const double dz = Config::ORIENT_DEADZONE_DEG;
    for (int i = 0; i < 3; ++i)
        if (!(std::fabs(rv[i]) >= dz)) rv[i] = 0.0;

    // ---- ③ 整体偏移限幅：ΔR 的【旋转角】θ > 150° ⇒ 把 ΔR【整体缩比】到 150°（轴不变）----
    // ⚠ 缩比做在**分量**上（三个分量同一个 k）⇒ 方向不变。**不许换成逐分量 clamp**：
    //   逐分量夹会改变方向（某一根分量先到顶、别的还在长），末端就朝错的方向走。
    //   ⚠ 这条负对照只有在**多分量**输入下才会红：纯单轴输入时"整体缩比"与"逐分量夹"
    //   给出**同一个结果**（只有一个分量、且它就是要缩的那根）⇒ 用例 ⑳c 因此分 (a)(b) 两半。
    // ⚠ 用**未经死区的** `dR` 重算一次 θ（死区改的是 `rv`；而"这次偏移有多大"必须按原值判）。
    {
        double rvAll[3];
        rotVecDeg(dR, rvAll);
        const double th = std::sqrt(rvAll[0]*rvAll[0] + rvAll[1]*rvAll[1] + rvAll[2]*rvAll[2]);
        // ⚠ 2026-09-29 Task 3（复审 M1）：这里原来还挂着一句 `&& th > 1e-12` —— 它**不可达**
        //   （`th > 150` 已蕴含 `th > 1e-12`），读起来像个除零守卫、其实不是（下面除的是 `th`，
        //   而 `th > 150` 早已排除 `th ≈ 0`）⇒ 删掉。这个条件今天只剩一个子句。
        if (th > Config::ORIENT_MAX_OFFSET_DEG) {
            const double k = Config::ORIENT_MAX_OFFSET_DEG / th;
            for (int i = 0; i < 3; ++i) rv[i] *= k;      // 缩比在【分量】上做：三个分量同一个 k ⇒ 轴不变
        }
    }

    // ---- ④ 摆动：【左乘】= 在基座系里摆 ------------------------------------------------
    // 旋量 = Rx(φ)·(SX·rv[0], SY·rv[1], 0)：φ 是**入参**（见头文件那两个理由）。
    // ⚠ `cos(phi)*t1` / `sin(phi)*t1` 用的是 `t1 = SY*rv[1]`（**已折符号**），顺序不能换 ——
    //   先折符号再进 M，与"先 M 再折符号"在 φ≠0 时**不是**同一件事（M 会把 y 分量转到 y/z 上，
    //   而在转了之后按 y 折符号就折错轴了）。
    const double D2R0 = 3.14159265358979323846 / 180.0;
    const double phi = phiDeg * D2R0;
    const double t0 = Config::BTN2_TILT_SIGN_X * rv[0];
    const double t1 = Config::BTN2_TILT_SIGN_Y * rv[1];
    const double w[3] = { t0, std::cos(phi) * t1, std::sin(phi) * t1 };
    double Mw[9], Rtilt[9];
    button2RotVecToMatDeg(w, Mw);
    button2Mat3Mul(Mw, refR, Rtilt);                    // 左乘 = 基座系里摆

    // ---- ⑤ 自转：【右乘】= 绕【新的】末端自身轴 -----------------------------------------
    // ⚠ 右乘 `Rtilt · Rz` 而不是 `Rz · Rtilt`：右乘绕的是**摆动之后**的末端轴（"自身"轴），
    //   左乘绕的是参照姿态的轴 —— 两者只在 rtilt 与 refR 同轴时才等价。
    //   对调会被 ㉑ 抓住（它的判据 1 就是"接近轴（第三列）逐位不变"）。
    const double roll = Config::BTN2_ROLL_SIGN * rv[2];
    double Mr[9];
    const double rollAxis[3] = { 0.0, 0.0, roll };      // MSVC 的 C++ 不支持 C99 复合字面量
    button2RotVecToMatDeg(rollAxis, Mr);
    button2Mat3Mul(Rtilt, Mr, outR);
}

// ============================================================================
//  按钮2 腕部求解（2026-09-29 Task 3）：给定"想要的末端朝向"，解 J4/J5/J6
// ============================================================================
// 契约、算法要点、失败语义（失败 ⇒ out 逐位退回 ref）全部写在 `Button2Joint.h`；这里只写
// 【为什么正好是这几行】以及几处容易写错的地方。
//
// ⚠ **为什么是角雅可比 `J[3+i][3+j]`**：`Kinematics::jacobian`（Kinematics.cpp:272 起）填的是
//   `J[0..2][i] = z_i × (p_ee − p_i)`（线速度）与 `J[3..5][i] = z_i`（**角速度**，各关节 z 轴
//   在世界系）。我们要的是"腕三个关节（下标 3/4/5）的角速度 → 世界系角速度" ⇒ 取**行 3..5、
//   列 3..5** 的 3×3 块。这是本任务与 FK 回代配对的全部数学。
//
// ⚠ **误差为什么右乘 `Rqᵀ`**：`D = targetR · Rqᵀ` 是"把**当前**末端朝向转到**目标**所需的世界系
//   旋转"（左乘世界系旋转 ⇒ 与上面那 3×3 角雅可比的表达坐标系一致）。`rotVecDeg(D)` 就是那一下
//   的旋转向量（度）—— FK 回代每一轮都重算 Rq，所以这是**牛顿法**、不是一次线性外推。
//
// ⚠ **单位自洽**：`Jw` 无量纲（z 轴是单位向量），`e` 与 `dq` 都是**度** —— 两者按同一比例
//   (度↔弧度) 缩放，比值不变 ⇒ 直接用度算 `dq` 是对的（不必先转弧度）。**别"顺手转弧度"**：
//   转了还要转回来，多两处能写错的地方。
//
// ⚠ **`Jw·Jwᵀ + λ²I` 为什么一定可逆**：加的是 λ²I（λ ≥ 1e-6 ⇒ λ² ≥ 1e-12）⇒ 特征值都 ≥ 1e-12。
//   而 `button2Mat3Inv` 的失败门是 `|det| < 1e-12` ⇒ **理论上**它可能仍判失败（det 是三个特征值
//   之积，λ 极小时确实可能压到门限下）。那一支 `return false` 是**真实的**（不是理论摆设）：
//   它正是"雅可比退化"这条失败路径的执行点（`out` 此刻已是 ref）。
bool button2SolveWrist(const double ref[6], const double targetR[9], double maxStepDeg, double out[6]) {
    // ---- 失败契约：第一句就把 out 退回参照 --------------------------------------------
    // 成功时下面会覆盖 out[3..5]（out[0..2] 本就等于 ref）；任何一条失败路径（雅可比退化、
    // 迭代不收敛、末尾自验不过）都**不再动 out** ⇒ 自动满足"失败 ⇒ out 逐位退回 ref"。
    // ⚠ 放在最前、且**所有**失败支路都不写 out —— 这样成功/失败的分叉只由返回值表达，
    //   与 `button2Mat3Inv` 的失败契约同一条纪律（"结果只能看返回值，别把陈旧值当结果"）。
    for (int i = 0; i < 6; ++i) out[i] = ref[i];

    double q[3] = { ref[3], ref[4], ref[5] };   // 腕角初值 = 参照
    double lam = Config::BTN2_WRIST_DAMP;       // λ 初值（阻尼下限）
    double prevErr = 0.0;
    bool havePrev = false;
    bool converged = false;
    int iter = 0;

    for (; iter < Config::BTN2_WRIST_MAX_ITER; ++iter) {
        const double joints[6] = { ref[0], ref[1], ref[2], q[0], q[1], q[2] };

        // FK：当前末端旋转 Rq（取齐次阵的左上 3×3）与角雅可比 Jw
        double T[4][4];
        Kinematics::composeTransform(joints, T);
        double Rq[9];
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c) Rq[r * 3 + c] = T[r][c];

        double J[6][6];
        Kinematics::jacobian(joints, J);
        double Jw[9];                            // 行 3..5、列 3..5（见上面第一段说明）
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c) Jw[r * 3 + c] = J[3 + r][3 + c];

        // 世界系误差 e = rotvec(targetR · Rqᵀ)（度）
        double RqT[9], D[9], e[3];
        button2Mat3T(Rq, RqT);
        button2Mat3Mul(targetR, RqT, D);
        rotVecDeg(D, e);                         // 直接调本文件匿名命名空间里的实现
        const double err = std::sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);
        if (err < Config::BTN2_WRIST_TOL_DEG) { converged = true; break; }

        // λ 自适应：这一轮比上一轮更接近目标 ⇒ 减半（更信雅可比）；否则加倍（更信阻尼）。
        //   ⚠ 与 `Kinematics::inverse` 同一形状（那里 λ 夹 [0.001, 10]；这里按计划夹
        //     [1e-6, 1.0]，且用 λ² —— 见头文件的算法那一节）。
        if (havePrev) { if (err < prevErr) lam *= 0.5; else lam *= 2.0; }
        havePrev = true;
        prevErr = err;
        if (lam < 1e-6) lam = 1e-6;
        if (lam > 1.0) lam = 1.0;

        // M = Jw·Jwᵀ + λ²·I
        double M[9];
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c) {
                double s = 0.0;
                for (int k = 0; k < 3; ++k) s += Jw[r * 3 + k] * Jw[c * 3 + k];
                M[r * 3 + c] = s;
            }
        const double l2 = lam * lam;
        M[0] += l2; M[4] += l2; M[8] += l2;

        double Minv[9];
        // 失败 ⇒ out 已是 ref（第一句写过），直接退。
        if (!button2Mat3Inv(M, Minv)) return false;

        // y = Minv · e；dq = Jwᵀ · y（阻尼最小二乘的一步）
        double y[3];
        for (int r = 0; r < 3; ++r)
            y[r] = Minv[r * 3 + 0] * e[0] + Minv[r * 3 + 1] * e[1] + Minv[r * 3 + 2] * e[2];
        double dq[3];
        for (int i = 0; i < 3; ++i)
            dq[i] = Jw[0 * 3 + i] * y[0] + Jw[1 * 3 + i] * y[1] + Jw[2 * 3 + i] * y[2];

        // 逐轴夹到 ±maxStepDeg，再累加。
        //   ⚠ 次序照抄 `clampJointStep`：**先比上界、再比下界**（两条独立 if，不是 else-if）——
        //     `maxStepDeg` 为负时两条都会执行到，结果 = |maxStepDeg|（荒谬但与原处一致）。
        for (int i = 0; i < 3; ++i) {
            if (dq[i] >  maxStepDeg) dq[i] =  maxStepDeg;
            if (dq[i] < -maxStepDeg) dq[i] = -maxStepDeg;
            q[i] += dq[i];
        }
    }

    // ---- 末尾自验（★ 这条不变量唯一能被抓住的地方）------------------------------------
    // 迭代跑满 `MAX_ITER` 而没在循环里判收敛时，**再算一次** FK 与误差；仍不达门限 ⇒ false。
    //   ⚠ 循环里那句 `err < TOL` 已经判过一次收敛 ⇒ 走到这里时 q 是"最后一次迭代后"的值，
    //     必须重新评估（`err` 是上一轮的，不是当前 q 的）。
    if (!converged) {
        const double joints[6] = { ref[0], ref[1], ref[2], q[0], q[1], q[2] };
        double T[4][4];
        Kinematics::composeTransform(joints, T);
        double Rq[9];
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c) Rq[r * 3 + c] = T[r][c];
        double RqT[9], D[9], e[3];
        button2Mat3T(Rq, RqT);
        button2Mat3Mul(targetR, RqT, D);
        rotVecDeg(D, e);
        const double err = std::sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);
        if (err >= Config::BTN2_WRIST_TOL_DEG) return false;   // out 仍是 ref
        converged = true;
    }

    // ---- 成功：只覆盖腕三关节（J1/J2/J3 保持参照）---------------------------------------
    out[0] = ref[0]; out[1] = ref[1]; out[2] = ref[2];
    out[3] = q[0];   out[4] = q[1];   out[5] = q[2];
    return true;
}

// ============================================================================
//  接线层的两条纯判据（从 RelayCore.cpp 抽出来 —— 那里不被任何测试编译）
// ============================================================================
// 契约、边界（尤其是 NaN 与负数 maxStep 这两处"刻意照抄"）全部写在 Button2Joint.h 上；
// 这里只写"为什么是这几行"。**两处都是逐字搬运**：任何"顺手改进"都是没有用例背书的行为改变。
// ⚠ 判据本身与"接线有没有接对"是两件事：控制流（三处 `return`）留在 `RelayCore.cpp`
//   ⇒ 那一段**仍然没有自动化用例**，抽出来的只有算术。

bool isTrustworthyJointRef(const double ref[6]) {
    // 子句一：六位**恰好**全 0（"`GetAngle()` 从未解析成功"的指纹）。
    //   ⚠ 写全六个下标而**不写循环**：与 `onButton2Press` 里那三处逐字段拷贝同一风格，
    //     而且它判的是**恰好**而不是"接近" —— 换成 1e-9 之类的容差会把"机械臂真的停在
    //     零位附近"判成坏参照，而那是**合法位姿**（臂就该停在那儿按着不动）。
    const bool allZero =
        (ref[0] == 0.0 && ref[1] == 0.0 && ref[2] == 0.0 &&
         ref[3] == 0.0 && ref[4] == 0.0 && ref[5] == 0.0);

    // 子句二：越关节限位 ⇒ 不可信。用现成函数（`Kinematics.cpp:457`；`tests/test_kinematics.cpp`
    //   322-337 有用例）而不是在这里重抄限位表 —— 那份表是唯一真相源，重抄会各自漂移。
    //   ⚠ 它对 NaN 返回 **true**（逐条比较全为 false）—— 这是本函数对 NaN 放行的**唯一**
    //     原因，见头文件那一段。别在这里补 NaN 判断（理由同样写在头文件）。
    return !(allZero || !Kinematics::isWithinJointLimits(ref));
}

void clampJointStep(const double prev[6], const double desired[6], double maxStep, double out[6]) {
    // 逐轴：本帧要走的量 → 夹到 ±maxStep → 加到"上一帧已下发"上。
    //   ⚠ 次序照抄原处：**先比上界、再比下界**。两条都用独立的 `if`（**不是** else-if）——
    //     `maxStep` 为负时两条都会被执行到（见头文件的"未定义意图"），换成 else-if 就变了。
    //   ⚠ 先读后写（同一轮里读完 `prev[i]`/`desired[i]` 才写 `out[i]`）⇒ 调用方原地传 `j`
    //     当 `out` 是安全的（用例⑯(c)）。
    for (int i = 0; i < 6; ++i) {
        double w = desired[i] - prev[i];
        if (w >  maxStep) w =  maxStep;
        if (w < -maxStep) w = -maxStep;
        out[i] = prev[i] + w;
    }
}
