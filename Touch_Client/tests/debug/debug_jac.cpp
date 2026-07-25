#include <iostream>
#include <cmath>
#define M_PI 3.14159265358979323846
#include "../relay/CoordinateTransform.h"
#include "../robot/Kinematics.h"

int main() {
    double j[6] = {0, -30, 60, 0, 30, 0};
    Vec3 p0 = Kinematics::forwardPosition(j);
    std::cout << "FK at [0,-30,60,0,30,0]: (" << p0.x << ", " << p0.y << ", " << p0.z << ")" << std::endl;
    
    // Check J1 rotation effect
    double j0[6] = {0, -30, 60, 0, 0, 0};
    double j90[6] = {90, -30, 60, 0, 0, 0};
    Vec3 p_j0 = Kinematics::forwardPosition(j0);
    Vec3 p_j90 = Kinematics::forwardPosition(j90);
    std::cout << "J1=0:  (" << p_j0.x << ", " << p_j0.y << ", " << p_j0.z << ")" << std::endl;
    std::cout << "J1=90: (" << p_j90.x << ", " << p_j90.y << ", " << p_j90.z << ")" << std::endl;
    std::cout << "dXY: " << fabs(p_j90.x-p_j0.x) << ", " << fabs(p_j90.y-p_j0.y) << std::endl;
    
    // Check different configs
    double j1[6] = {0, -30, 60, 0, 0, 0};
    double j2[6] = {45, -30, 60, 0, 0, 0};
    Vec3 p1 = Kinematics::forwardPosition(j1);
    Vec3 p2 = Kinematics::forwardPosition(j2);
    double d12 = sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2));
    std::cout << "d12 (J1=0 vs J1=45): " << d12 << std::endl;
    
    // Jacobian debug
    double J[6][6];
    Kinematics::jacobian(j, J);
    double d2r = M_PI / 180.0;
    std::cout << "\nJacobian @ [0,-30,60,0,30,0] (mm/deg):" << std::endl;
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 6; c++) {
            std::cout << J[r][c]*d2r << "\t";
        }
        std::cout << std::endl;
    }
    
    // Numerical Jacobian
    std::cout << "\nNumerical Jacobian (mm/deg):" << std::endl;
    double eps = 0.01;
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 6; c++) {
            double jp[6], jm[6];
            memcpy(jp, j, sizeof(jp));
            memcpy(jm, j, sizeof(jm));
            jp[c] += eps;
            jm[c] -= eps;
            Vec3 pp = Kinematics::forwardPosition(jp);
            Vec3 pm = Kinematics::forwardPosition(jm);
            double num = (c==0) ? (pp.x-pm.x)/(2*eps) : (c==1) ? (pp.y-pm.y)/(2*eps) : 0;
            if (r == 0) num = (pp.x-pm.x)/(2*eps);
            else if (r == 1) num = (pp.y-pm.y)/(2*eps);
            else num = (pp.z-pm.z)/(2*eps);
            std::cout << num << "\t";
        }
        std::cout << std::endl;
    }
    
    return 0;
}
