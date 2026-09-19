#ifndef ANR_TELEMETRY_H
#define ANR_TELEMETRY_H

#include <Arduino.h>
#include "odometry.h"
#include "battery.h"
#include "tof.h"

// =============================================================================
// ROS 2 telemetry
//
// Emits two line-oriented records to the ESP32's USB serial:
//
//   P x=<m> y=<m> theta=<rad> vL=<m/s> vR=<m/s> age=<ms>
//   T bat=<volts> estop=<0|1> tof=<mm> mode=<0|1|2>
//
// The Pi's esp32_bridge parses these directly. emitPose is called at the
// control-loop rate; emitTelemetry is throttled by TELEMETRY_RATE_MS in
// main.cpp.
//
// Writes are guarded by a portMUX spinlock so the Serial buffer cannot
// interleave with human-readable debug printf from elsewhere (e.g. the
// obstacle FSM logging). Single-byte printf is safe; printf with a long
// format string is what this guard protects.
// =============================================================================

namespace telemetry {

enum class NavMode : uint8_t {
    ROS2 = 0,
    AUTO = 1,
    FAULT = 2,
};

// Initialise the module (call once from setup()).
void begin();

// Emit one pose record. Safe to call at the control-loop rate; the
// divider lives in main.cpp.
void emitPose(const Odometry& odom);

// Emit one telemetry record (5 Hz throttled in main.cpp).
void emitTelemetry(const BatteryMonitor& battery,
                   bool estopActive,
                   const ToFSensor& tof,
                   NavMode mode);

}  // namespace telemetry

#endif  // ANR_TELEMETRY_H
