#pragma once
#include "../core/AppState.h"

// 2nd-order Butterworth lowpass filter (biquad form)
// One instance per channel, zero-phase initialization
class Butterworth2 {
public:
    Butterworth2();
    void reset();
    double step(double input);
    // coefficients — public so init() can set them
    double b0, b1, b2, a1, a2;
private:
    double x1, x2, y1, y2;       // delay states
};

namespace ForcePipeline {
    // Call once: initialize filter coefficients
    void init();

    // Call at 30Hz: raw -> filtered -> hapticOut (writes into fd under caller's mutex)
    void step(AppState::ForceData& fd);

    // Call on shutdown
    void shutdown();
}
