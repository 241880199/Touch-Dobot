#pragma once
#include "../relay/CoordinateTransform.h"

namespace SingularityAvoidance {

// ===== Mode 1: Position control → optimal orientation =====
// Given a target Cartesian position and the current robot joint angles,
// compute the orientation (Rx,Ry,Rz in degrees) that maximizes manipulability
// and keeps the arm away from singular configurations.
// Returns the optimal orientation. If IK fails, returns a zero Vec3
// (caller should fall back to current orientation).
Vec3 optimizeOrientation(const Vec3& targetPos, const double currentJoints[6]);

// ===== Mode 2: Orientation control → damped delta + TCP micro-adjust + directional repulsion =====
// Given the current accumulated orientation target, the user's orientation
// delta (in robot-frame degrees), the frozen TCP position, and current
// joint angles, produce:
//   - return value: damped orientation delta (may be smaller than input)
//   - tcpAdjustOut: TCP position micro-adjustment (mm), zero if safe
//   - repulsionOut: task-space directional repulsion force (N), zero if safe
//     This force opposes the user's rotation toward the wrist singular direction.
// Also sends W| warnings when damping or adjustment is active.
Vec3 dampOrientationMotion(const Vec3& targetOrient, const Vec3& deltaOrient,
                           const Vec3& currentTcp, const double currentJoints[6],
                           Vec3& tcpAdjustOut, Vec3& repulsionOut);

// ===== Mode 3: Combined position+orientation → selective damping =====
// Given the user's 6-DOF delta (position + orientation) and current joint
// angles, damp components along singular directions. Output is the damped
// delta — caller adds it to the current target.
// Also sends W| warnings when any direction is significantly damped.
void dampFullCommand(const Vec3& userDeltaPos, const Vec3& userDeltaOrient,
                     const double currentJoints[6],
                     Vec3& dampedDeltaPos, Vec3& dampedDeltaOrient);

// ===== Send warning to MATLAB relay station =====
// level: 0=INFO, 1=WARN, 2=CRITICAL
// type: "shoulder", "wrist", "elbow", "joint", "tcp_adjust", "full_damp"
// message: human-readable description
// suggestion: actionable advice for the operator
// param1, param2: key metrics for display
void sendWarning(int level, const char* type, const char* message,
                 const char* suggestion, double param1, double param2);

} // namespace SingularityAvoidance
