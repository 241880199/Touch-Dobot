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

// ============================================================================
//  笔杆 Euler 增量 → 关节增量（纯函数）
// ============================================================================
// 逐行对照 Button2Joint.h 的「管线」那一节：偏移 → 逐轴死区 → 逐轴符号 → 逐关节限幅
// → 一一对应加到参照上。
//
// 映射表（**标定结论**，来源见头文件；这里重复一遍是因为它就是本文件的核心四行）：
//     out[3] = ref[3] + clamp(SIGN_J4 * gate(cur[0] − ref[0]))     // 笔杆 Rx（前后摆）
//     out[4] = ref[4] + clamp(SIGN_J5 * gate(cur[2] − ref[2]))     // 笔杆 Rz（左右摆）
//     out[5] = ref[5] + clamp(SIGN_J6 * gate(cur[1] − ref[1]))     // 笔杆 Ry（自转）
//     out[0..2] = ref[0..2]                                       // J1/J2/J3 无条件保持
//
// ⚠ 注意 out[4] 吃的是 **cur[2]**（Rz）、out[5] 吃的是 **cur[1]**（Ry）—— 索引是**错开**的。
//   这不是笔误：映射就是"源轴 ≠ 目标轴"，而且这两路**最容易在重构时被顺手写整齐**。
//   用例①(b)(c) 与⑦对"只有它自己动"逐条断言 ⇒ 写整齐了立刻红。

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
    const double dJ4 = Config::BTN2_J4_SIGN * axisGate(curStylus[0] - refStylus[0]);  // Rx
    const double dJ5 = Config::BTN2_J5_SIGN * axisGate(curStylus[2] - refStylus[2]);  // Rz
    const double dJ6 = Config::BTN2_J6_SIGN * axisGate(curStylus[1] - refStylus[1]);  // Ry

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
