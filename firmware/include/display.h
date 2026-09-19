#ifndef ANR_DISPLAY_H
#define ANR_DISPLAY_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "navigation.h"
#include "odometry.h"
#include "battery.h"
#include "types.h"

class Display {
public:
    Display();

    bool begin();
    void showSplash();
    void showStatus(NavMode mode,
                    ControlMode ctrl,
                    uint16_t distanceMm,
                    int8_t leftPct,
                    int8_t rightPct,
                    const Pose& pose,
                    float distanceM,
                    float batteryV,
                    BatteryStatus battery,
                    bool tofOk,
                    bool estop);
    void showFault(const char* message);

private:
    Adafruit_SSD1306 _oled;
    bool _ok;
};

#endif // ANR_DISPLAY_H