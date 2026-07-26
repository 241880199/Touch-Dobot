#pragma once
#include "../core/AppState.h"

// Motion estimator: tracks tool velocity & acceleration from position history
// Uses 5-point ring buffer for central-difference acceleration estimation
class MotionEstimator {
public:
    MotionEstimator();
    void update(double x, double y, double z, double dt);
    void getState(double vel[3], double acc[3]) const;
    bool isStill() const;
    void reset();
private:
    static const int BUF_SIZE = 5;
    double m_posBuf[5][3];
    int m_idx;
    int m_count;
    double m_vel[3];
    double m_accRaw[3];
    double m_accFiltered[3];
    // Butterworth2-style 10Hz LPF for acceleration
    double m_lpfB0, m_lpfB1, m_lpfB2, m_lpfA1, m_lpfA2;
    double m_lpfX1[3], m_lpfX2[3], m_lpfY1[3], m_lpfY2[3];
};

namespace ForceCompensation {
    // Call once at startup — loads calibration file, initializes filters
    void init();

    // Call at ~125Hz (from ForceReader thread) or ~30Hz (from pollForce)
    // fd.raw[] must be fresh; poseRxyz = {X,Y,Z,Rx,Ry,Rz} in mm & deg from GetPose()
    // Writes fd.compensated[] (6-axis compensated force)
    void step(AppState::ForceData& fd, const double poseRxyz[6]);

    // Set calibration parameters (called after calibration completes or file load)
    void setCalibration(double massKg, const double comSensor[3],
                        const double biasForce[3], const double biasTorque[3]);

    // Check if calibration is active
    bool isCalibrated();

    // Call on shutdown
    void shutdown();
}
