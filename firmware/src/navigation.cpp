#include "navigation.h"
#include "config.h"
#include "forward_controller.h"

static bool g_preferLeft = true;
static ForwardController g_forward;

Navigation::Navigation()
    : _mode(NavMode::INIT), _modeChangedMs(0),
      _manualLeft(0), _manualRight(0) {}

void Navigation::begin() {
    transition(NavMode::INIT, millis());
}

void Navigation::reset() {
    transition(NavMode::MOVING, millis());
}

void Navigation::transition(NavMode next, uint32_t now) {
    _mode = next;
    _modeChangedMs = now;
}

MotorCommand Navigation::updateAUTO(ObstacleState obs,
                                    bool sensorOk,
                                    bool sensorStale,
                                    uint32_t now) {

    MotorCommand cmd{0, 0};

    switch (_mode) {

        case NavMode::INIT:
            transition(NavMode::MOVING, now);
            cmd = {0, 0};
            break;

        case NavMode::MOVING:
            if (!sensorOk || sensorStale) {
                transition(NavMode::FAULT, now);
                cmd = {0, 0};
                break;
            }
            switch (obs) {
                case ObstacleState::CLEAR:
                    cmd = {(int8_t)MOTOR_NORMAL_PCT, (int8_t)MOTOR_NORMAL_PCT};
                    break;
                case ObstacleState::SLOW:
                    transition(NavMode::SLOWING, now);
                    cmd = {(int8_t)MOTOR_SLOW_PCT, (int8_t)MOTOR_SLOW_PCT};
                    break;
                case ObstacleState::BLOCKED:
                    transition(NavMode::OBSTACLE_STOP, now);
                    cmd = {0, 0};
                    break;
                case ObstacleState::SENSOR_FAULT:
                    transition(NavMode::FAULT, now);
                    cmd = {0, 0};
                    break;
            }
            break;

        case NavMode::SLOWING:
            cmd = {(int8_t)MOTOR_SLOW_PCT, (int8_t)MOTOR_SLOW_PCT};
            if (obs == ObstacleState::CLEAR) {
                transition(NavMode::MOVING, now);
            } else if (obs == ObstacleState::BLOCKED) {
                transition(NavMode::OBSTACLE_STOP, now);
                cmd = {0, 0};
            } else if (!sensorOk || sensorStale || obs == ObstacleState::SENSOR_FAULT) {
                transition(NavMode::FAULT, now);
                cmd = {0, 0};
            }
            break;

        case NavMode::OBSTACLE_STOP:
            cmd = {0, 0};
            if (obs == ObstacleState::BLOCKED) {
                if (g_preferLeft) {
                    g_preferLeft = false;
                    transition(NavMode::AVOIDING_LEFT, now);
                } else {
                    g_preferLeft = true;
                    transition(NavMode::AVOIDING_RIGHT, now);
                }
            } else if (obs == ObstacleState::SLOW) {
                transition(NavMode::SLOWING, now);
                cmd = {(int8_t)MOTOR_SLOW_PCT, (int8_t)MOTOR_SLOW_PCT};
            } else if (obs == ObstacleState::CLEAR) {
                transition(NavMode::RECOVERING, now);
            }
            break;

        case NavMode::AVOIDING_LEFT:
            cmd = {-MOTOR_SLOW_PCT, MOTOR_SLOW_PCT};
            if ((now - _modeChangedMs) >= AVOID_TURN_MS) {
                transition(NavMode::RECOVERING, now);
            }
            break;

        case NavMode::AVOIDING_RIGHT:
            cmd = {MOTOR_SLOW_PCT, -MOTOR_SLOW_PCT};
            if ((now - _modeChangedMs) >= AVOID_TURN_MS) {
                transition(NavMode::RECOVERING, now);
            }
            break;

        case NavMode::RECOVERING:
            cmd = {(int8_t)MOTOR_SLOW_PCT, (int8_t)MOTOR_SLOW_PCT};
            if ((now - _modeChangedMs) >= AVOID_CLEAR_MS) {
                transition(NavMode::MOVING, now);
            }
            if (obs == ObstacleState::BLOCKED) {
                transition(NavMode::OBSTACLE_STOP, now);
                cmd = {0, 0};
            }
            break;

        case NavMode::WAYPOINT_NAV:
        case NavMode::WAYPOINT_REACHED:
            // Caller should use updateWaypoint(); fall back to a safe stop.
            cmd = {0, 0};
            break;

        case NavMode::FAULT:
            cmd = {0, 0};
            break;
    }

    return cmd;
}

MotorCommand Navigation::updateWaypoint(ObstacleState obs,
                                        bool sensorOk,
                                        bool sensorStale,
                                        float headingErrRad,
                                        float distanceToTargetM,
                                        uint32_t now) {
    MotorCommand cmd{0, 0};

    if (!sensorOk || sensorStale) {
        transition(NavMode::FAULT, now);
        return cmd;
    }

    if (obs == ObstacleState::BLOCKED) {
        transition(NavMode::OBSTACLE_STOP, now);
        return cmd;
    }

    if (obs == ObstacleState::SLOW) {
        transition(NavMode::SLOWING, now);
        cmd = {(int8_t)MOTOR_SLOW_PCT, (int8_t)MOTOR_SLOW_PCT};
        return cmd;
    }

    if (distanceToTargetM <= 0.0f) {
        transition(NavMode::WAYPOINT_REACHED, now);
        return cmd;
    }

    if (_mode != NavMode::WAYPOINT_NAV) {
        transition(NavMode::WAYPOINT_NAV, now);
    }

    ForwardControllerConfig cfg = g_forward.config();
    cfg.forwardPct = (obs == ObstacleState::SLOW) ? MOTOR_SLOW_PCT : MOTOR_NORMAL_PCT;
    g_forward.setConfig(cfg);
    cmd = g_forward.compute(headingErrRad, distanceToTargetM);
    return cmd;
}

MotorCommand Navigation::applyManual(const ManualCommand& cmd,
                                     bool estop,
                                     bool sensorOk,
                                     bool sensorStale) {
    _manualLeft  = cmd.left;
    _manualRight = cmd.right;

    if (estop) {
        _mode = NavMode::FAULT;
        return {0, 0};
    }

    if (!sensorOk || sensorStale) {
        return {0, 0};
    }

    return {cmd.left, cmd.right};
}