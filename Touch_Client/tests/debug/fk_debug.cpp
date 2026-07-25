#include <iostream>
#include <cmath>
#define M_PI 3.14159265358979323846
#include "../relay/CoordinateTransform.h"
#include "../robot/Kinematics.h"

int main() {
    double configs[][6] = {
        {0, 0, 0, 0, 0, 0},
        {45, 0, 0, 0, 0, 0},
        {0, 45, 0, 0, 0, 0},
        {0, 0, 45, 0, 0, 0},
        {0, 0, 0, 45, 0, 0},
        {0, 0, 0, 0, 45, 0},
        {0, 0, 0, 0, 0, 45},
        {10, -20, 30, -15, 25, -10},
        {90, 0, 0, 0, 0, 0},
    };
    
    for (int i = 0; i < 9; i++) {
        Vec3 p = Kinematics::forwardPosition(configs[i]);
        std::cout << "config " << i << ": (" 
                  << configs[i][0] << "," << configs[i][1] << "," << configs[i][2] << ","
                  << configs[i][3] << "," << configs[i][4] << "," << configs[i][5] 
                  << ") -> (" << p.x << "," << p.y << "," << p.z << ")" << std::endl;
    }
    return 0;
}
