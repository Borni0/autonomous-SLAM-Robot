#include "encoder.h"

Encoder::Encoder(int pin)
    : _pin(pin), _delta(0), _total(0) {}

void Encoder::begin() {
    pinMode(_pin, INPUT_PULLUP);
    attachInterruptArg(digitalPinToInterrupt(_pin),
                       &Encoder::isrThunk,
                       this,
                       RISING);
}

int32_t Encoder::consumeTicks() {
    int32_t v = _delta;
    _delta = 0;
    return v;
}

int32_t Encoder::totalTicks() const {
    return _total;
}

void IRAM_ATTR Encoder::isrThunk(void* arg) {
    static_cast<Encoder*>(arg)->onPulse();
}

void IRAM_ATTR Encoder::onPulse() {
    _delta++;
    _total++;
}