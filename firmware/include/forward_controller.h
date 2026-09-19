#ifndef ANR_FORWARD_CONTROLLER_H
#define ANR_FORWARD_CONTROLLER_H

#include <Arduino.h>
#include "types.h"

struct ForwardControllerConfig {
    // Linear speed while moving toward the waypoint (percent).
    uint8_t  forwardPct;
    // Linear speed while correcting heading only (percent).
    uint8_t  pivotPct;
    // Maximum absolute heading error treated as "aligned".
    float    alignThresholdRad;
    // Proportional gain for heading correction (percent / rad).
    float    headingGain;
    // Maximum steer offset (percent) applied to the inside wheel.
    float    maxSteerOffset;
};

class ForwardController {
public:
    ForwardController();

    void begin(const ForwardControllerConfig& cfg);

    MotorCommand compute(float headingErrRad, float distanceM) const;

    void setConfig(const ForwardControllerConfig& cfg) { _cfg = cfg; }
    const ForwardControllerConfig& config() const { return _cfg; }

private:
    ForwardControllerConfig _cfg;
};

#endif // ANR_FORWARD_CONTROLLER_H