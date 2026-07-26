#include "SelfCollision.h"
#include <cmath>
#include <algorithm>

namespace SelfCollision {

// Squared distance between two 3D points
static inline double distSq(const Vec3& a, const Vec3& b) {
    double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    return dx*dx + dy*dy + dz*dz;
}

// Minimum distance from point P to line segment AB (mm)
static double pointToSegmentDist(const Vec3& P, const Vec3& A, const Vec3& B) {
    double abx = B.x - A.x, aby = B.y - A.y, abz = B.z - A.z;
    double abSq = abx*abx + aby*aby + abz*abz;
    if (abSq < 1e-9) {
        // Degenerate segment: A ≈ B → point-to-point
        return sqrt(distSq(P, A));
    }
    // Project P onto line AB, clamp to [0,1]
    double t = ((P.x - A.x)*abx + (P.y - A.y)*aby + (P.z - A.z)*abz) / abSq;
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;
    double cx = A.x + t*abx, cy = A.y + t*aby, cz = A.z + t*abz;
    return sqrt(distSq(P, Vec3(cx, cy, cz)));
}

Result check(const Vec3 positions[7]) {
    Result r;
    r.warning = false;
    r.reject = false;
    r.minDistMm = 1e9;
    r.pairA = -1;
    r.pairB = -1;

    // Helper: evaluate a point-to-segment pair, update min if closer
    auto evalPointSeg = [&](const Vec3& P, const Vec3& A, const Vec3& B, int idxP, int idxSeg) {
        double d = pointToSegmentDist(P, A, B);
        if (d < r.minDistMm) {
            r.minDistMm = d;
            r.pairA = idxP;
            r.pairB = idxSeg;
        }
    };

    // Helper: evaluate a point-to-point pair
    auto evalPointPoint = [&](const Vec3& P, const Vec3& Q, int idxP, int idxQ) {
        double d = sqrt(distSq(P, Q));
        if (d < r.minDistMm) {
            r.minDistMm = d;
            r.pairA = idxP;
            r.pairB = idxQ;
        }
    };

    // Key non-adjacent collision pairs for the CR3:
    //
    //   positions[0] = base origin
    //   positions[1] = J1 (base rotation top)
    //   positions[2] = J2 (shoulder)
    //   positions[3] = J3 (elbow)
    //   positions[4] = J4 (wrist roll)
    //   positions[5] = J5 (wrist pitch)  ← wrist center
    //   positions[6] = J6 (TCP)
    //
    // During orientation changes, the wrist assembly (J4-J5-J6) rotates.
    // Most likely collisions: wrist hitting the upper arm or base column.

    // Wrist center (J5) vs shoulder link (J1→J2)
    evalPointSeg(positions[5], positions[1], positions[2], 5, 1);

    // Wrist center (J5) vs elbow link (J2→J3)
    evalPointSeg(positions[5], positions[2], positions[3], 5, 2);

    // TCP (J6) vs shoulder link (J1→J2)
    evalPointSeg(positions[6], positions[1], positions[2], 6, 1);

    // TCP (J6) vs elbow link (J2→J3)
    evalPointSeg(positions[6], positions[2], positions[3], 6, 2);

    // Wrist (J5) vs base column top (J1) — tool dipping below base
    evalPointPoint(positions[5], positions[1], 5, 1);

    // TCP (J6) vs base origin — extreme reach-under
    evalPointPoint(positions[6], positions[0], 6, 0);

    if (r.minDistMm < REJECT_DIST_MM) {
        r.reject = true;
        r.warning = true;
    } else if (r.minDistMm < WARN_DIST_MM) {
        r.warning = true;
    }

    return r;
}

} // namespace SelfCollision
