#ifndef ANR_BATTERY_H
#define ANR_BATTERY_H

#include <Arduino.h>

enum class BatteryStatus : uint8_t {
    OK,
    WARNING,
    CRITICAL
};

class BatteryMonitor {
public:
    BatteryMonitor();

    void begin();
    BatteryStatus update(uint32_t now);

    float voltage() const { return _voltage; }
    BatteryStatus status() const { return _status; }

private:
    float          _voltage;
    BatteryStatus  _status;
    uint32_t       _lastUpdateMs;
};

#endif // ANR_BATTERY_H
