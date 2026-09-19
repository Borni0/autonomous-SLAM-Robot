#ifndef ANR_TYPES_H
#define ANR_TYPES_H

#include <Arduino.h>

enum class ControlMode : uint8_t {
    AUTO,
    MANUAL,
    ROS2  // Transparent driver for the Raspberry Pi (Nav2 + LiDAR).
};

struct ManualCommand {
    int8_t left;
    int8_t right;
    bool   fresh;
};

struct MotorCommand {
    int8_t left;
    int8_t right;
};

#endif // ANR_TYPES_H