#ifndef ANR_PID_H
#define ANR_PID_H

#include <Arduino.h>

class PIDController {
public:
    PIDController();

    void begin(float kp, float ki, float kd, float outputMax, float integralMax);
    void reset();

    // Run one step.  setpoint and measurement use the same units.
    float update(float setpoint, float measurement, float dtSec);

    // Telemetry (for diagnostics)
    float integral() const { return _integral; }
    float lastError() const { return _lastError; }

private:
    float _kp, _ki, _kd;
    float _outputMax;
    float _integralMax;
    float _integral;
    float _lastError;
    bool  _initialized;
};

#endif // ANR_PID_H