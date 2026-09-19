#ifndef ANR_MOTOR_H
#define ANR_MOTOR_H

#include <Arduino.h>

class MotorDriver {
public:
    MotorDriver(int rpwm, int lpwm, int r_en, int l_en);

    void begin();
    void setSpeed(int8_t percent); // -100..+100
    void stop();
    int8_t currentSpeed() const { return _speedPct; }

private:
    int _rpwm, _lpwm, _r_en, _l_en;
    int8_t _speedPct;
};

#endif // ANR_MOTOR_H