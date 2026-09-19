#include "tof.h"
#include "config.h"
#include <Adafruit_VL53L0X.h>

static Adafruit_VL53L0X g_sensor;

ToFSensor::ToFSensor(int xshutPin, TwoWire& bus)
    : _xshutPin(xshutPin), _bus(bus), _ok(false), _distanceMm(0), _lastUpdateMs(0) {}

bool ToFSensor::begin() {
    if (_xshutPin >= 0) {
        pinMode(_xshutPin, OUTPUT);
        digitalWrite(_xshutPin, HIGH);
        delay(20);
    }

    _ok = g_sensor.begin();
    if (_ok) {
        g_sensor.startRangeContinuous(50);
        _lastUpdateMs = millis();
    }
    return _ok;
}

void ToFSensor::update() {
    if (!_ok) {
        _distanceMm = 0;
        return;
    }

    if (g_sensor.timeoutOccurred()) {
        _ok = false;
        _distanceMm = 0;
        return;
    }

    if (!g_sensor.isRangeComplete()) {
        return;
    }

    uint16_t mm = g_sensor.readRangeResult();
    _distanceMm = mm;
    _lastUpdateMs = millis();
}