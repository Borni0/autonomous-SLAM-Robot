#ifndef ANR_TOF_H
#define ANR_TOF_H

#include <Arduino.h>
#include <Wire.h>
#include "config.h"

class ToFSensor {
public:
    ToFSensor(int xshutPin, TwoWire& bus = Wire);

    bool begin();
    void update();

    bool     ok() const { return _ok; }
    uint16_t distanceMm() const { return _distanceMm; }
    bool     stale() const { return (millis() - _lastUpdateMs) > TOF_TIMEOUT_MS; }
    uint32_t lastUpdateMs() const { return _lastUpdateMs; }

private:
    int _xshutPin;
    TwoWire& _bus;
    bool _ok;
    uint16_t _distanceMm;
    uint32_t _lastUpdateMs;
};

#endif // ANR_TOF_H