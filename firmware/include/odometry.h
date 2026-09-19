#ifndef ANR_ODOMETRY_H
#define ANR_ODOMETRY_H

#include <Arduino.h>

struct Pose {
    float x;       // metres
    float y;       // metres
    float theta;   // radians, CCW positive
};

class Odometry {
public:
    Odometry();

    void reset();
    void update(int32_t leftTicks, int32_t rightTicks, float dtSec);

    const Pose& pose() const { return _pose; }
    Pose& pose() { return _pose; }
    float distanceTravelledM() const { return _distanceM; }
    float headingDeg() const { return _pose.theta * 57.2957795f; }

    // Per-wheel ground speed for PID speed control.
    float speedLeftMps()  const { return _speedLeftMps; }
    float speedRightMps() const { return _speedRightMps; }

    // Milliseconds since the last encoder pulse was consumed by update().
    // Used by the Pi-side bridge to detect a stalled pose stream and by
    // the firmware's own telemetry to surface encoder health.
    uint32_t encoderAgeMs() const { return millis() - _lastUpdateMs; }
    bool     encodersFresh(uint32_t maxAgeMs) const { return encoderAgeMs() <= maxAgeMs; }

private:
    Pose    _pose;
    float   _distanceM;
    float   _speedLeftMps;
    float   _speedRightMps;
    int32_t _lastLeftTicks;
    int32_t _lastRightTicks;
    uint32_t _lastUpdateMs;
};

#endif // ANR_ODOMETRY_H