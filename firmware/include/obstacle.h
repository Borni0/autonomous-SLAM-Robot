#ifndef ANR_OBSTACLE_H
#define ANR_OBSTACLE_H

#include <Arduino.h>

enum class ObstacleState : uint8_t {
    CLEAR,
    SLOW,
    BLOCKED,
    SENSOR_FAULT
};

class ObstacleDetector {
public:
    ObstacleDetector();

    // Process a new ToF reading.
    void update(uint16_t distanceMm, bool sensorOk, bool stale);

    ObstacleState state() const { return _state; }

    // Hysteresis-based thresholds for obstacle state.
    static constexpr uint16_t EMERGENCY_STOP_MM = 250;
    static constexpr uint16_t SLOW_DOWN_MM      = 500;
    static constexpr uint16_t CLEAR_MM          = 700;

private:
    ObstacleState _state;
};

#endif // ANR_OBSTACLE_H