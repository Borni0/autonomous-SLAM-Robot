#include "obstacle.h"

ObstacleDetector::ObstacleDetector()
    : _state(ObstacleState::CLEAR) {}

void ObstacleDetector::update(uint16_t distanceMm, bool sensorOk, bool stale) {
    if (!sensorOk || stale) {
        _state = ObstacleState::SENSOR_FAULT;
        return;
    }

    switch (_state) {
        case ObstacleState::CLEAR:
            if (distanceMm <= EMERGENCY_STOP_MM) _state = ObstacleState::BLOCKED;
            else if (distanceMm <= SLOW_DOWN_MM)  _state = ObstacleState::SLOW;
            break;
        case ObstacleState::SLOW:
            if (distanceMm <= EMERGENCY_STOP_MM) _state = ObstacleState::BLOCKED;
            else if (distanceMm >= CLEAR_MM)     _state = ObstacleState::CLEAR;
            break;
        case ObstacleState::BLOCKED:
            if (distanceMm >= CLEAR_MM) _state = ObstacleState::SLOW;
            else if (distanceMm >= SLOW_DOWN_MM) _state = ObstacleState::SLOW;
            break;
        case ObstacleState::SENSOR_FAULT:
            if (sensorOk && !stale) _state = ObstacleState::CLEAR;
            break;
    }
}
