#include <iostream>
#include <cmath>
#define M_PI 3.14159265358979323846
#include "../relay/CoordinateTransform.h"
#include "../robot/Kinematics.h"

int main() {
    // Try various configs to find ones where J1 rotation causes significant XY change
    double j2_vals[] = {-80, -70, -60, -45, -30, -20};
    double j3_vals[] = {150, 140, 120, 100, 90, 60, 45};
    
    for (int i2 = 0; i2 < 6; i2++) {
    for (int i3 = 0; i3 < 7; i3++) {
        double j[6] = {0, j2_vals[i2], j3_vals[i3], 0, 0, 0};
        Vec3 p0 = Kinematics::forwardPosition(j);
        
        j[0] = 90;
        Vec3 p90 = Kinematics::forwardPosition(j);
        
        double dxy = sqrt(pow(p90.x-p0.x,2) + pow(p90.y-p0.y,2));
        double r0 = sqrt(p0.x*p0.x + p0.y*p0.y);
        
        if (r0 > 100 && dxy > 50) {
            std::cout << "J2=" << j2_vals[i2] << " J3=" << j3_vals[i3] 
                      << " r0=" << r0 << " dxy(J1)=" << dxy 
                      << " p0=(" << p0.x << "," << p0.y << "," << p0.z << ")" << std::endl;
        }
    }}
    return 0;
}
