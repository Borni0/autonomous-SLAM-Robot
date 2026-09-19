#include "telemetry.h"
#include <Arduino.h>
#include "config.h"

// portMUX-based guard so Serial.printf from emitPose/emitTelemetry never
// interleaves bytes with debug printf from the obstacle FSM.
static portMUX_TYPE g_telemetryMux = portMUX_INITIALIZER_UNLOCKED;

static inline void serialPrintf(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    portENTER_CRITICAL(&g_telemetryMux);
    char buf[160];
    int n = vsnprintf(buf, sizeof(buf), fmt, args);
    if (n > 0) {
        if (n >= (int)sizeof(buf)) n = sizeof(buf) - 1;
        Serial.write((const uint8_t*)buf, n);
    }
    portEXIT_CRITICAL(&g_telemetryMux);
    va_end(args);
}

namespace telemetry {

void begin() {
    // Nothing to do — the mux is statically initialised. Hook left here
    // so callers can extend with buffered writers later.
}

void emitPose(const Odometry& odom) {
    const Pose& p = odom.pose();
    float vL = odom.speedLeftMps();
    float vR = odom.speedRightMps();
    uint32_t age = odom.encoderAgeMs();

    serialPrintf("P x=%.4f y=%.4f theta=%.4f vL=%.3f vR=%.3f age=%lu\n",
                 p.x, p.y, p.theta, vL, vR,
                 static_cast<unsigned long>(age));
}

void emitTelemetry(const BatteryMonitor& battery,
                   bool estopActive,
                   const ToFSensor& tof,
                   NavMode mode) {
    serialPrintf("T bat=%.2f estop=%d tof=%u mode=%d\n",
                 battery.voltage(),
                 estopActive ? 1 : 0,
                 static_cast<unsigned>(tof.distanceMm()),
                 static_cast<int>(mode));
}

}  // namespace telemetry
