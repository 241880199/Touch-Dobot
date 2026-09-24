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
