#include "pid.h"

PIDController::PIDController()
    : _kp(0), _ki(0), _kd(0),
      _outputMax(100.0f), _integralMax(60.0f),
      _integral(0.0f), _lastError(0.0f), _initialized(false) {}

void PIDController::begin(float kp, float ki, float kd, float outputMax, float integralMax) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _outputMax = outputMax;
    _integralMax = integralMax;
    reset();
}

void PIDController::reset() {
    _integral = 0.0f;
    _lastError = 0.0f;
    _initialized = false;
}

float PIDController::update(float setpoint, float measurement, float dtSec) {
    if (dtSec <= 0.0f) return 0.0f;

    float error = setpoint - measurement;
    _integral += error * dtSec;

    if (_integral > _integralMax)  _integral = _integralMax;
    if (_integral < -_integralMax) _integral = -_integralMax;

    float derivative = 0.0f;
    if (_initialized) {
        derivative = (error - _lastError) / dtSec;
    } else {
        _initialized = true;
    }
    _lastError = error;

    float output = (_kp * error) + (_ki * _integral) + (_kd * derivative);

    if (output > _outputMax)  output = _outputMax;
    if (output < -_outputMax) output = -_outputMax;

    return output;
}