#include "Button2Mapping.h"
#include "../calibration/TcpCalibration.h"
#include "../config/Config.h"
#include <cmath>

// ============================================================================
//  真旋转合成（替换"Euler 差当旋转向量 + 逐分量相加"）：
//    ΔR_dev  = R(cur_stylus) · R(ref_stylus)ᵀ        // 笔杆相对按下点的旋转（器件系）
//    ΔR_rob  = M · ΔR_dev · Mᵀ                        // 同一张 M 做相似变换（旋转的坐标变换）
//    R_tgt   = ΔR_rob · R(ref_robot)                  // 左乘 = 在【基座系】里倾斜（规格说的就是基座方向）
//    ⇒ 最后才提取一次 RPY
//  角度限幅：把 ΔR_rob 的【旋转角】夹到 ORIENT_MAX_OFFSET_DEG，再重新合成
//    （旧写法夹的是 RPY 的三个分量 —— 那是表示量，不是旋转量；本改动顺带修掉这一点）
//  「笔杆自转 ⇒ J6」：自转是绕【笔杆自身轴】的旋转 ⇒ ΔR_rob 里含一个 tool-roll 分量；
//    工具轴 = J6 轴时，IK 会用 J6 实现它 ⇒ 规格第 4 条由此自然满足（上机要核 J6 读数，见 Task 4）。
//
//  ★ 2026-09-23 现场+推导结论：现有那张硬编码矩阵【不是】病根 —— `ω_robot = M·ω_dev`
//    逐行验算成立（见 Button2Mapping.h 的推导表）。病根是【算术】：drx/dry/drz 是 Euler
//    **角之差**，不是旋转向量的分量；三者又各自被**逐分量加到**参照姿态上。
//    两者都只在小角度下近似成立。
//
//  为什么 M 与 Mᵀ 夹在中间是对的：ΔR_dev 是【器件系】里描述的一个旋转，要变成【基座系】
//    里描述同一个物理旋转，就是相似变换 M·ΔR_dev·Mᵀ（M 是 det=+1 的正交阵 ⇒ Mᵀ=M⁻¹，
//    相似变换保持"旋转"这个性质不变，不会引入镜像）。M 本身【不是】rotation vector 的
//    换基那么简单 —— 那正是旧写法在做的、也是它只在小角度下成立的原因。
// ============================================================================

namespace {

const double PI  = 3.14159265358979323846;
const double D2R = PI / 180.0;
const double R2D = 180.0 / PI;

// ---- 3×3 小工具（只在本文件内用；不扩散到公共头，见计划 Step 3）----
void mul3(const double A[9], const double B[9], double C[9]) {
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            double v = 0.0;
            for (int k = 0; k < 3; k++) v += A[i * 3 + k] * B[k * 3 + j];
            C[i * 3 + j] = v;
        }
}

void transpose3(const double A[9], double T[9]) {
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) T[i * 3 + j] = A[j * 3 + i];
}

// 轴角 → 矩阵（Rodrigues；轴须为单位向量，角度为弧度）。
// 与旧路径的区别：这里重建的是一个【旋转】，不是把三个角塞进 RPY 的三个槽。
void axisAngleToMat(const double axis[3], double thRad, double R[9]) {
    const double c = cos(thRad), s = sin(thRad), t = 1.0 - c;
    const double x = axis[0], y = axis[1], z = axis[2];
    R[0] = t*x*x + c;     R[1] = t*x*y - s*z; R[2] = t*x*z + s*y;
    R[3] = t*x*y + s*z;   R[4] = t*y*y + c;   R[5] = t*y*z - s*x;
    R[6] = t*x*z - s*y;   R[7] = t*y*z + s*x; R[8] = t*z*z + c;
}

// 矩阵 → (单位轴, 角度/弧度)。
// ⚠ 走四元数（Shepperd 的分支法）而【不是】 (R−Rᵀ)/(2·sinθ)：后者在 θ→0 与 θ→180° 时
//   都要除以一个趋于 0 的量。这里 θ→0 与 θ→180° 各自走不同的分支，分母恒为 O(1)。
// ⚠ 约定 qw ≥ 0（取"短弧"那一支）。取反会让轴反向 + 角度变成 2π−θ，两者描述的旋转
//   在 θ<180° 时相同，但 θ 会算成 300 多度 ⇒ 限幅会误判成"超限"。
void matToAxisAngle(const double R[9], double axis[3], double& thRad) {
    double qw, qx, qy, qz;
    const double tr = R[0] + R[4] + R[8];
    if (tr > 0.0) {
        double s = sqrt(tr + 1.0) * 2.0;            // s = 4·qw
        qw = 0.25 * s;
        qx = (R[7] - R[5]) / s;
        qy = (R[2] - R[6]) / s;
        qz = (R[3] - R[1]) / s;
    } else if (R[0] > R[4] && R[0] > R[8]) {
        double s = sqrt(1.0 + R[0] - R[4] - R[8]) * 2.0;   // s = 4·qx
        qw = (R[7] - R[5]) / s;
        qx = 0.25 * s;
        qy = (R[1] + R[3]) / s;
        qz = (R[2] + R[6]) / s;
    } else if (R[4] > R[8]) {
        double s = sqrt(1.0 + R[4] - R[0] - R[8]) * 2.0;   // s = 4·qy
        qw = (R[2] - R[6]) / s;
        qx = (R[1] + R[3]) / s;
        qy = 0.25 * s;
        qz = (R[5] + R[7]) / s;
    } else {
        double s = sqrt(1.0 + R[8] - R[0] - R[4]) * 2.0;   // s = 4·qz
        qw = (R[3] - R[1]) / s;
        qx = (R[2] + R[6]) / s;
        qy = (R[5] + R[7]) / s;
        qz = 0.25 * s;
    }
    // 归一化（分支里的除法已经保证 |q|≈1，这一步是收掉浮点误差）
    double n = sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
    if (n <= 0.0) { axis[0] = 1.0; axis[1] = 0.0; axis[2] = 0.0; thRad = 0.0; return; }
    qw /= n; qx /= n; qy /= n; qz /= n;
    if (qw < 0.0) { qw = -qw; qx = -qx; qy = -qy; qz = -qz; }
    if (qw > 1.0) qw = 1.0;
    thRad = 2.0 * acos(qw);

    const double sh = sqrt(1.0 - qw * qw);         // = |sin(θ/2)| ≥ 0（qw ≥ 0 时 θ/2 ∈ [0, π/2]）
    if (sh < 1e-12) {
        // θ ≈ 0：轴无意义（角度为 0 的旋转与轴无关）。给一个确定的轴，免得返回未初始化值。
        axis[0] = 1.0; axis[1] = 0.0; axis[2] = 0.0;
    } else {
        axis[0] = qx / sh; axis[1] = qy / sh; axis[2] = qz / sh;
    }
}

// 矩阵 → RPY（度），与 TcpCalibration::rpyToMatrix 的约定互为逆（R = Rz·Ry·Rx）。
//   R[6] = −sin(ry) ⇒ ry = atan2(−R[6], |cos(ry)|) ∈ [−90, 90]
//   rx = atan2(R[7], R[8]) · rz = atan2(R[3], R[0])
void matToRpy(const double R[9], double rpy[3]) {
    const double sry = -R[6];
    const double cry = sqrt(R[0] * R[0] + R[3] * R[3]);      // = |cos(ry)|
    if (cry > 1e-9) {
        rpy[0] = atan2(R[7], R[8]) * R2D;
        rpy[1] = atan2(sry, cry) * R2D;
        rpy[2] = atan2(R[3], R[0]) * R2D;
    } else {
        // 万向锁（cos(ry)≈0，ry=±90°）：rx 与 rz 只有组合量可定。
        // 不特判的话 atan2(0,0) 会返回 0 的【和】，还原出来的矩阵与目标相差 (rz−rx) 那样大
        // —— 一个静默几十度的错。取 rx = 0 作为自由变量的规范选择（rz 吸收全部组合量）。
        if (sry > 0.0) {                       // ry = +90°：组合量 = rz − rx
            rpy[1] =  90.0;
            rpy[2] = atan2( R[5], R[4]) * R2D;
        } else {                               // ry = −90°：组合量 = rz + rx
            rpy[1] = -90.0;
            rpy[2] = atan2(-R[5], R[4]) * R2D;
        }
        rpy[0] = 0.0;
    }
}

bool allFinite(const double v[3]) {
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

}  // namespace

Vec3 button2OrientationTarget(const double refRobotRpy[3],
                              const double refStylusRpy[3],
                              const double curStylusRpy[3]) {
    // NaN/Inf 守卫：三个入参【任一】非有限 ⇒ 返回参照姿态（"不倾斜"），让按下按钮2 的那一刻
    // 姿态成为安全落点。否则非有限值会顺着下面的矩阵乘法把返回值污染成 NaN，再经 RelayCore
    // 下发出去 —— 那时现场看到的是"机械臂乱动"，而不是"哪里算错了"。
    // ⚠⚠ 2026-09-23 订正（Task 3 附加要求③）：这里原先写着"**任何**非有限的输入都会…"——
    //   **说过头了**。`refRobotRpy` 自己非有限时，本函数**照样把它原样返回**（就是下面这行
    //   return 的那三个分量），即它**不**保证"返回值一定有限"。这不是 bug 而是没有更好的选择：
    //   那一刻没有"安全值"可退（0 是一个真实姿态，机械臂会真的转过去），所以这一层只能由
    //   **调用者**负责。RelayCore 的参照 (`m_orientRefRobot`) 抄自实际姿态，它若已经是 NaN，
    //   新旧两条路径都会把 NaN 传下去 —— 本函数不是那条分界线。
    //   该行为已被用例钉住（tests/test_button2_mapping.cpp ⑫，覆盖三个入参位置），
    //   所以"注释说的"和"代码做的"不会再各自漂。
    //   另：RelayCore 的姿态块里还有一道 NaN 守卫，但它守的是 drx/dry/drz（笔杆偏移），
    //   **不覆盖**参照本身；两道是刻意的，代价只是一次比较。
    if (!allFinite(refRobotRpy) || !allFinite(refStylusRpy) || !allFinite(curStylusRpy)) {
        return Vec3(refRobotRpy[0], refRobotRpy[1], refRobotRpy[2]);
    }

    // ① 三个姿态 → 旋转矩阵（入参都是度；rpyToMatrix 的约定是 R = Rz·Ry·Rx）
    double R_refS[9], R_curS[9], R_refR[9];
    TcpCalibration::rpyToMatrix(refStylusRpy[0], refStylusRpy[1], refStylusRpy[2], R_refS);
    TcpCalibration::rpyToMatrix(curStylusRpy[0], curStylusRpy[1], curStylusRpy[2], R_curS);
    TcpCalibration::rpyToMatrix(refRobotRpy[0],  refRobotRpy[1],  refRobotRpy[2],  R_refR);

    // ② 笔杆相对按下点的旋转（器件系）：ΔR_dev = R(cur) · R(ref)ᵀ
    //    正交阵的逆就是转置；用转置而不是求逆，免得引入"看着像求逆其实解了个病态方程"的风险。
    double M[9], Mt[9];
    touchToRobotMatrix(M);                 // 与平移路径【同一张】表（Task 1 抽出来的）
    transpose3(M, Mt);

    double R_refS_T[9], dR_dev[9], tmp[9], dR_rob[9];
    transpose3(R_refS, R_refS_T);
    mul3(R_curS, R_refS_T, dR_dev);

    // ③ 同一个物理旋转在【基座系】里的表示：相似变换
    mul3(M, dR_dev, tmp);
    mul3(tmp, Mt, dR_rob);

    // ④ 限幅：按【旋转角】夹，不按 RPY 的三个分量夹（那是表示量，不是旋转量）
    //    θ ≤ 上限时【原样保留】dR_rob（不做"提取角度再重建"的往返，省掉一次精度损失）。
    double dR_use[9];
    for (int i = 0; i < 9; i++) dR_use[i] = dR_rob[i];
    double axis[3], thRad = 0.0;
    matToAxisAngle(dR_rob, axis, thRad);
    const double limitRad = Config::ORIENT_MAX_OFFSET_DEG * D2R;
    if (thRad > limitRad) axisAngleToMat(axis, limitRad, dR_use);

    // ⑤ 左乘参照姿态：在【基座系】里倾斜（规格说的"尖端朝某方向"就是基座方向）
    double R_tgt[9];
    mul3(dR_use, R_refR, R_tgt);

    // ⑥ 只在最后提取一次 RPY
    double rpy[3];
    matToRpy(R_tgt, rpy);
    return Vec3(rpy[0], rpy[1], rpy[2]);
}
