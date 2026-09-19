#include "motor.h"
#include "config.h"

MotorDriver::MotorDriver(int rpwm, int lpwm, int r_en, int l_en)
    : _rpwm(rpwm), _lpwm(lpwm), _r_en(r_en), _l_en(l_en), _speedPct(0) {}

void MotorDriver::begin() {
    pinMode(_rpwm, OUTPUT);
    pinMode(_lpwm, OUTPUT);
    pinMode(_r_en, OUTPUT);
    pinMode(_l_en, OUTPUT);

    ledcAttachPin(_rpwm, 0);
    ledcAttachPin(_lpwm, 1);

    ledcSetup(0, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
    ledcSetup(1, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);

    digitalWrite(_r_en, HIGH);
    digitalWrite(_l_en, HIGH);

    ledcWrite(0, 0);
    ledcWrite(1, 0);
}

void MotorDriver::setSpeed(int8_t percent) {
    if (percent > 100) percent = 100;
    if (percent < -100) percent = -100;
    _speedPct = percent;

    uint16_t duty = (uint16_t)(abs((int)percent) * PWM_MAX / 100);

    if (percent >= 0) {
        ledcWrite(0, duty);
        ledcWrite(1, 0);
    } else {
        ledcWrite(0, 0);
        ledcWrite(1, duty);
    }
}

void MotorDriver::stop() {
    _speedPct = 0;
    ledcWrite(0, 0);
    ledcWrite(1, 0);
}
