#ifndef ANR_ESTOP_H
#define ANR_ESTOP_H

#include <Arduino.h>

class EmergencyStop {
public:
    EmergencyStop();

    void begin();
    bool update();             // call frequently; returns true when pressed
    bool pressed() const { return _pressed; }
    uint32_t lastChangeMs() const { return _lastChangeMs; }

private:
    bool     _pressed;
    bool     _rawState;
    uint32_t _lastChangeMs;
    uint32_t _lastDebounceMs;
};

#endif // ANR_ESTOP_H