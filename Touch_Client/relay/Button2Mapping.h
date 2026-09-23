#pragma once

#include "CoordinateTransform.h"   // Vec3 + touchToRobotMatrix

// ============================================================================
//  按钮2 姿态映射：笔杆姿态 → 机械臂目标姿态（纯函数，可单测）
// ============================================================================
//
// 【为什么要有这个文件】按钮2 的规格是四条：
//     笔杆平移 ⇒ 机械臂不动；笔尖左(右)摆 ⇒ 尖端向 −X(+X) 倾；
//     笔尖前(后)摆 ⇒ 尖端向 +Y(−Y) 倾；笔杆自转 ⇒ J6 转。
//   旧实现把「笔杆 Euler 角之差」【当成旋转向量】用（robot_dR = M·(drx,dry,drz)），
//   再把三个角【逐分量加到】按下时的参照姿态上。两者都只在小角度下近似成立，
//   而实际摆幅 ±50~80°、且本机末端姿态常年贴着 rx≈±180（RPY 表示的接缝）。
//
// ---- 推导表（本计划的依据；用坐标变换矩阵 M 逐行反推并验算过）----
// Touch 标准器件系 = X 右 / Y 上 / Z 朝用户；M（relay/CoordinateTransform.h 的
// touchToRobotMatrix）给出 robot = M·dev ⇒
//     dev +X → robot +X · dev +Y → robot +Z · dev +Z → robot −Y。
// 笔尖指 dev −Y（笔朝下）⇒ 用 M 反推每次"物理摆动"在基座系里是绕哪根轴：
//
// | 物理动作 | 器件系旋转 | 基座系旋转 | 体现在器件 Euler 上 |
// |---|---|---|---|
// | 笔尖左摆（尖端向 −X） | 绕 −Z | 绕 +Y | `sz` 减小 |
// | 笔尖右摆（尖端向 +X） | 绕 +Z | 绕 −Y | `sz` 增大 |
// | 笔尖前摆（尖端向 +Y） | 绕 +X | 绕 +X | `sx` 增大 |
// | 笔尖后摆（尖端向 −Y） | 绕 −X | 绕 −X | `sx` 减小 |
// | 笔杆自转 | 绕 ±Y | 绕 ±Z | `sy` ± |
//
// ★ 这张表与现有硬编码矩阵【逐行一致】（robot_dRx=+drx / robot_dRy=−drz /
//   robot_dRz=+dry 正好就是 ω_robot = M·ω_dev 的三行）⇒ **错的不在"表"，在"算术"**。
//   所以本文件【不动】M 的任何一项，也不动任何符号。
//
// ⚠ 调用约定：三个入参都是【度】；返回值也是【度】。参照姿态 refRobotRpy 是按下
//   按钮2 那一刻的机械臂 GetPose 的 Rx/Ry/Rz，与 TcpCalibration::rpyToMatrix 的
//   约定一致（R = Rz·Ry·Rx）。
// ⚠ 本函数【没有位置入参】：平移分量在结构上就进不来，这是"笔杆平移时机械臂不动"
//   的第一道保证（第二道在 RelayCore 的姿态块里）。
Vec3 button2OrientationTarget(const double refRobotRpy[3],
                              const double refStylusRpy[3],
                              const double curStylusRpy[3]);
