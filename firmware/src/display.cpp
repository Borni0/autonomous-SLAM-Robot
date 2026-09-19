#include "display.h"
#include "config.h"

static const char* modeText(NavMode m) {
    switch (m) {
        case NavMode::INIT:           return "INIT";
        case NavMode::MOVING:         return "MOVING";
        case NavMode::SLOWING:        return "SLOWING";
        case NavMode::OBSTACLE_STOP:  return "STOPPED";
        case NavMode::AVOIDING_LEFT:  return "AVOID-L";
        case NavMode::AVOIDING_RIGHT: return "AVOID-R";
        case NavMode::RECOVERING:     return "RECOVER";
        case NavMode::FAULT:          return "FAULT";
    }
    return "?";
}

Display::Display()
    : _oled(SCREEN_W, SCREEN_H, &Wire, OLED_RESET_PIN), _ok(false) {}

bool Display::begin() {
    _ok = _oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
    if (_ok) {
        _oled.clearDisplay();
        _oled.display();
    }
    return _ok;
}

void Display::showSplash() {
    if (!_ok) return;
    _oled.clearDisplay();
    _oled.setTextSize(1);
    _oled.setTextColor(SSD1306_WHITE);
    _oled.setCursor(0, 0);
    _oled.println(F("ANR Phase 2"));
    _oled.println(F("GPS-free Indoor Bot"));
    _oled.println(F("Booting..."));
    _oled.display();
}

void Display::showStatus(NavMode mode,
                         ControlMode ctrl,
                         uint16_t distanceMm,
                         int8_t leftPct,
                         int8_t rightPct,
                         const Pose& pose,
                         float distanceM,
                         float batteryV,
                         BatteryStatus battery,
                         bool tofOk,
                         bool estop) {
    if (!_ok) return;
    _oled.clearDisplay();
    _oled.setTextSize(1);
    _oled.setTextColor(SSD1306_WHITE);
    _oled.setCursor(0, 0);
    _oled.print(ctrl == ControlMode::AUTO ? "AUTO" : "MANU");
    _oled.print(F(":"));
    _oled.println(modeText(mode));

    _oled.setCursor(0, 10);
    if (estop) {
        _oled.println(F("** ESTOP ACTIVE **"));
    } else if (!tofOk) {
        _oled.println(F("ToF: ERROR"));
    } else {
        _oled.print(F("D: "));
        _oled.print(distanceMm);
        _oled.println(F(" mm"));
    }

    _oled.setCursor(0, 22);
    _oled.print(F("L:"));
    _oled.print(leftPct);
    _oled.setCursor(64, 22);
    _oled.print(F("R:"));
    _oled.print(rightPct);

    _oled.setCursor(0, 34);
    _oled.print(F("Trav:"));
    _oled.print(distanceM, 2);
    _oled.println(F(" m"));

    _oled.setCursor(0, 46);
    _oled.print(F("Bat: "));
    _oled.print(batteryV, 1);
    _oled.print(F(" V"));
    if (battery == BatteryStatus::WARNING)   _oled.print(F(" LOW"));
    if (battery == BatteryStatus::CRITICAL)  _oled.print(F(" CRIT"));

    _oled.display();
}

void Display::showFault(const char* message) {
    if (!_ok) return;
    _oled.clearDisplay();
    _oled.setTextSize(2);
    _oled.setTextColor(SSD1306_WHITE);
    _oled.setCursor(0, 0);
    _oled.println(F("FAULT"));
    _oled.setTextSize(1);
    _oled.setCursor(0, 24);
    while (*message) _oled.print(*message++);
    _oled.display();
}