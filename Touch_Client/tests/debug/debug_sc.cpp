#define _USE_MATH_DEFINES
#include <cstdio>
#include <cmath>
#include "../../relay/CoordinateTransform.h"
#include "../../robot/Kinematics.h"
#include "../../safety/SelfCollision.h"

int main() {
    double configs[][6] = {
        {0, -90, -150, 0, -90, 0},
        {0, -90, -155, 0, -90, 0},
        {0, -80, -155, 0, -90, 0},
        {0, -70, -155, 0, -90, 0},
        {0, -60, -155, 0, -90, 0},
        {0, -85, -150, 0, 90, 0},
        {0, -85, -150, 0, -45, 0},
        {30, -70, -155, 0, 90, 0},
    };
    for (int i = 0; i < 8; i++) {
        Vec3 pos[7];
        Kinematics::computeJointPositions(configs[i], pos);
        auto r = SelfCollision::check(pos);
        printf("J=(%.0f,%.0f,%.0f,%.0f,%.0f,%.0f) minDist=%.1fmm warn=%d J%d/J%d\n",
            configs[i][0],configs[i][1],configs[i][2],configs[i][3],configs[i][4],configs[i][5],
            r.minDistMm, r.warning, r.pairA, r.pairB);
        printf("  J2=(%.0f,%.0f,%.0f) J3=(%.0f,%.0f,%.0f) J5=(%.0f,%.0f,%.0f)\n",
            pos[2].x,pos[2].y,pos[2].z, pos[3].x,pos[3].y,pos[3].z, pos[5].x,pos[5].y,pos[5].z);
    }
    return 0;
}
