#ifndef ANR_NAVIGATION_H
#define ANR_NAVIGATION_H

#include <Arduino.h>
#include "obstacle.h"
#include "types.h"

enum class NavMode : uint8_t {
    INIT,
    MOVING,
    SLOWING,
    OBSTACLE_STOP,
    AVOIDING_LEFT,
    AVOIDING_RIGHT,
    RECOVERING,
    WAYPOINT_NAV,
    WAYPOINT_REACHED,
    FAULT
};

class Navigation {
public:
    Navigation();

    void begin();
    void reset();

    // AUTO update (local avoidance, no waypoint).
    MotorCommand updateAUTO(ObstacleState obs,
                            bool sensorOk,
                            bool sensorStale,
                            uint32_t now);

    // AUTO update with active waypoint steering.
    MotorCommand updateWaypoint(ObstacleState obs,
                                bool sensorOk,
                                bool sensorStale,
                                float headingErrRad,
                                float distanceToTargetM,
                                uint32_t now);

    MotorCommand applyManual(const ManualCommand& cmd,
                             bool estop,
                             bool sensorOk,
                             bool sensorStale);

    NavMode mode() const { return _mode; }
    uint32_t modeChangedMs() const { return _modeChangedMs; }

private:
    NavMode   _mode;
    uint32_t  _modeChangedMs;
    int8_t    _manualLeft;
    int8_t    _manualRight;

    void transition(NavMode next, uint32_t now);
};

#endif // ANR_NAVIGATION_H