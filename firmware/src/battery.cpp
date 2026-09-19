#include "battery.h"
#include "config.h"

BatteryMonitor::BatteryMonitor()
    : _voltage(0.0f), _status(BatteryStatus::OK), _lastUpdateMs(0) {}

void BatteryMonitor::begin() {
    analogReadResolution(12); // ESP32-S3 default, but set explicitly
    analogSetPinAttenuation(BAT_ADC_PIN, ADC_11db);
    _voltage = 0.0f;
    _status = BatteryStatus::OK;
    _lastUpdateMs = 0;
}

BatteryStatus BatteryMonitor::update(uint32_t now) {
    if ((now - _lastUpdateMs) < BAT_PERIOD_MS) return _status;
    _lastUpdateMs = now;

    uint32_t sum = 0;
    for (uint16_t i = 0; i < BAT_SAMPLES; ++i) {
        sum += analogReadMilliVolts(BAT_ADC_PIN);
    }
    float avgMv = static_cast<float>(sum) / static_cast<float>(BAT_SAMPLES);
    _voltage = (avgMv / 1000.0f) * BAT_DIVIDER_RATIO;

    if (_voltage <= BAT_CRITICAL_V)        _status = BatteryStatus::CRITICAL;
    else if (_voltage <= BAT_LOW_V)        _status = BatteryStatus::WARNING;
    else                                   _status = BatteryStatus::OK;

    return _status;
}