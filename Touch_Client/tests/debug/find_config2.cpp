#include <iostream>
#include <cmath>
#define M_PI 3.14159265358979323846
#include "../relay/CoordinateTransform.h"
#include "../robot/Kinematics.h"

int main() {
    // Now vary J4, J5 too to get wrist extension
    double configs[][6] = {
        {0, -45, 90, 0, 90, 0},     // J5=90, wrist bent
        {0, -45, 90, 0, -90, 0},    
        {0, -30, 60, 0, -45, 0},    
        {0, -60, 120, 0, -60, 0},   
        {0, -90, 150, 0, -30, 0},   
        {0, -45, 90, 45, 90, 0},    // J4 rotation too
        {0, -45, 90, -45, 90, 0},   
        {0, -30, 90, 90, 60, 0},    
        {0, -60, 60, 0, 0, 0},      
        {0, -90, 90, 0, 0, 0},      
    };
    
    for (int i = 0; i < 10; i++) {
        double j[6];
        memcpy(j, configs[i], sizeof(j));
        Vec3 p0 = Kinematics::forwardPosition(j);
        double r0 = sqrt(p0.x*p0.x + p0.y*p0.y);
        
        j[0] = 90;
        Vec3 p90 = Kinematics::forwardPosition(j);
        double dxy = sqrt(pow(p90.x-p0.x,2) + pow(p90.y-p0.y,2));
        
        std::cout << "config[" << i << "]: r0=" << r0 
                  << " dxy(J1)=" << dxy
                  << " p0=(" << p0.x << "," << p0.y << "," << p0.z << ")" << std::endl;
    }
    return 0;
}
