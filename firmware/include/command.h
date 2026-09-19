#ifndef ANR_COMMAND_H
#define ANR_COMMAND_H

#include <Arduino.h>
#include "types.h"
#include "config.h"

enum class CommandEvent : uint8_t {
    NONE,
    RESET_HEADING,
    ADD_WAYPOINT,
    LIST_WAYPOINTS,
    CLEAR_WAYPOINTS
};

struct WaypointRequest {
    float x;
    float y;
};

class CommandLine {
public:
    CommandLine();

    void begin();
    void poll();

    ManualCommand consume();
    ControlMode mode() const { return _mode; }
    uint32_t lastCommandMs() const { return _lastCommandMs; }
    bool ros2Active() const { return _mode == ControlMode::ROS2; }

    CommandEvent takeEvent(WaypointRequest& request);

private:
    char        _buffer[CMD_BUFFER_SIZE];
    uint8_t     _index;
    ControlMode _mode;
    int8_t      _lastLeft;
    int8_t      _lastRight;
    uint32_t    _lastCommandMs;
    CommandEvent _event;
    WaypointRequest _waypointRequest;

    void execute(const char* line);
};

#endif // ANR_COMMAND_H
