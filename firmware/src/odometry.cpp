#include "odometry.h"
#include "config.h"

Odometry::Odometry()
    : _pose{0.0f, 0.0f, 0.0f}, _distanceM(0.0f),
      _speedLeftMps(0.0f), _speedRightMps(0.0f),
      _lastLeftTicks(0), _lastRightTicks(0), _lastUpdateMs(0) {}

void Odometry::reset() {
    _pose = {0.0f, 0.0f, 0.0f};
    _distanceM = 0.0f;
    _speedLeftMps = 0.0f;
    _speedRightMps = 0.0f;
    _lastLeftTicks = 0;
    _lastRightTicks = 0;
    _lastUpdateMs = millis();
}

void Odometry::update(int32_t leftTicks, int32_t rightTicks, float dtSec) {
    _lastUpdateMs = millis();
    // Convert encoder ticks to wheel arc lengths.
    float dL = static_cast<float>(leftTicks)  * METERS_PER_TICK;
    float dR = static_cast<float>(rightTicks) * METERS_PER_TICK;

    // Differential-drive kinematics.
    float ds     = 0.5f * (dR + dL);
    float dTheta = (dR - dL) / WHEEL_BASE_M;

    // Update pose using the exact arc update (no small-angle approximation).
    if (fabsf(dTheta) < 1e-6f) {
        _pose.x += ds * cosf(_pose.theta);
        _pose.y += ds * sinf(_pose.theta);
    } else {
        float theta_new = _pose.theta + dTheta;
        float radius = ds / dTheta;
        _pose.x += radius * (sinf(theta_new) - sinf(_pose.theta));
        _pose.y += -radius * (cosf(theta_new) - cosf(_pose.theta));
        _pose.theta = theta_new;
    }

    // Normalize theta to [-pi, pi].
    while (_pose.theta >  3.14159265f) _pose.theta -= 2.0f * 3.14159265f;
    while (_pose.theta < -3.14159265f) _pose.theta += 2.0f * 3.14159265f;

    _distanceM += fabsf(ds);

    // Wheel speeds in m/s for the PID controller.
    if (dtSec > 0.0f) {
        _speedLeftMps  = dL / dtSec;
        _speedRightMps = dR / dtSec;
    }

    _lastLeftTicks  = leftTicks;
    _lastRightTicks = rightTicks;
}