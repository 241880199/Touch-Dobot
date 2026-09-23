#include "Button2Joint.h"
#include "../config/Config.h"
#include <cmath>

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
