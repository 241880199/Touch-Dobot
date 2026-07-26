#pragma once

// Force sensor calibration — static bias + motion mass estimation
namespace ForceCalibration {

    enum class State {
        IDLE,
        TARE,      // 2s static collection for force/torque bias
        MOTION,    // User moves robot; record F vs a to fit mass
        SOLVE,     // Fit mass + apply results
        DONE,      // Success
        ABORTED    // User interrupt or safety trip
    };

    // Start calibration
    bool start();

    // Abort immediately
    void abort();

    // SPACE: start/stop sampling in TARE/MOTION phases
    void confirmPose();

    // Drag mode callback (called on state transitions)
    void setDragModeCallback(void (*cb)(bool enable));

    bool isRunning();
    bool isDone();
    State currentState();
    const char* statusText();

    // Called each frame from pollForce (~30Hz)
    bool update(double dt, const double raw[6], const double pose[6]);

    // Persistence
    bool saveToFile(const char* path, double massKg,
                    const double biasForce[3], const double biasTorque[3]);
    bool loadFromFile(const char* path, double& massKg,
                      double biasForce[3], double biasTorque[3]);

} // namespace ForceCalibration
