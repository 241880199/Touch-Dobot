#pragma once

// Gaussian elimination solver for Ax = b (linear least squares via normal equations)
class GaussSolver {
public:
    // Solve overdetermined system via normal equations: A^T A x = A^T b
    // A is (nRows x nCols) row-major, b is (nRows x 1)
    // x is (nCols x 1) output; residualRms = ||Ax - b|| / sqrt(nRows)
    // Returns false if singular or degenerate
    static bool solve(int nRows, const double A[], const double b[],
                      int nCols, double x[], double& residualRms);
private:
    static bool gaussElim(int n, double A[], double b[], double x[]);
};

// Force sensor calibration — multi-pose automatic sweep
namespace ForceCalibration {

    enum class State {
        IDLE,
        TARE,      // 2s still collection for initial bias estimate
        MOVE,      // Moving to target orientation
        SETTLE,    // 0.5s wait for vibration decay
        SAMPLE,    // 0.5s data collection
        SOLVE,     // Normal equations -> extract params
        VERIFY,    // Check residual
        DONE,      // Success
        ABORTED    // User interrupt or safety trip
    };

    // Start calibration (must be called from main thread when not transmitting)
    bool start();

    // Abort immediately (called from safety handlers or user interrupt)
    void abort();

    // Confirm current pose reached (user presses SPACE in manual MOVE mode)
    void confirmPose();

    bool isRunning();
    bool isDone();
    State currentState();
    const char* statusText();

    // Called each frame (~125Hz from ForceReader) to drive state machine
    // Returns true when calibration is complete (DONE or ABORTED)
    bool update(double dt, const double raw[6], const double pose[6]);

    // Persistence — explicit parameter version
    bool saveToFile(const char* path, double residualRms,
                    double massKg, const double comSensor[3],
                    const double biasForce[3], const double biasTorque[3]);

    // Convenience: save current internal calibration state to path
    bool saveToFile(const char* path);

    bool loadFromFile(const char* path, double& massKg, double comSensor[3],
                      double biasForce[3], double biasTorque[3], double& residualRms);

} // namespace ForceCalibration
