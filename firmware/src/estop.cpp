#include "estop.h"
#include "config.h"

EmergencyStop::EmergencyStop()
    : _pressed(false), _rawState(false),
      _lastChangeMs(0), _lastDebounceMs(0) {}

void EmergencyStop::begin() {
    pinMode(ESTOP_PIN, INPUT_PULLUP);

    bool raw = digitalRead(ESTOP_PIN);
    bool active = ESTOP_ACTIVE_LOW ? !raw : raw;
    _rawState = active;
    _pressed = active;
    _lastChangeMs = millis();
    _lastDebounceMs = millis();
}

bool EmergencyStop::update() {
    uint32_t now = millis();
    bool raw = digitalRead(ESTOP_PIN);
    bool active = ESTOP_ACTIVE_LOW ? !raw : raw;

    if (active != _rawState) {
        _rawState = active;
        _lastDebounceMs = now;
    }

    if ((now - _lastDebounceMs) >= ESTOP_DEBOUNCE_MS && _pressed != _rawState) {
        _pressed = _rawState;
        _lastChangeMs = now;
    }

    return _pressed;
}