#pragma once
#include "../relay/CoordinateTransform.h"

// Lightweight self-collision detection for the CR3 6-DOF robot.
// Uses FK joint positions to check distances between non-adjacent links.
// Primary focus: wrist assembly (J4-J5-J6) vs upper arm / base during orientation changes.

namespace SelfCollision {

    struct Result {
        bool warning;            // approaching collision — consider slowing
        bool reject;             // too close — block motion
        double minDistMm;        // minimum distance found between any checked pair
        int    pairA;            // joint index of closer link (0=base, 1..6=J1..J6)
        int    pairB;            // joint index of farther link
    };

    // Thresholds (mm)
    const double WARN_DIST_MM   = 80.0;   // warning: wrist approaching upper arm / base
    const double REJECT_DIST_MM = 40.0;   // reject: risk of actual contact

    // Check self-collision risk given FK joint positions [0..6] (base, J1..J6).
    // Returns the minimum distance across all checked non-adjacent pairs.
    Result check(const Vec3 positions[7]);

} // namespace SelfCollision
