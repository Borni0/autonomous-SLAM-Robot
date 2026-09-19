#ifndef ANR_WAYPOINT_H
#define ANR_WAYPOINT_H

#include <Arduino.h>
#include "odometry.h"

struct Waypoint {
    float    x;          // metres
    float    y;          // metres
    float    headingRad; // desired final heading
    uint32_t id;         // serial number
    bool     active;
};

struct WaypointStatus {
    bool     hasTarget;
    bool     reached;
    float    distanceM;     // straight-line distance to target
    float    headingErrRad; // signed heading error, [-pi, pi]
    uint32_t currentId;
};

class WaypointQueue {
public:
    WaypointQueue();

    void begin();

    // Push a new waypoint at the head of the queue.
    bool push(const Waypoint& wp);

    // Drop the current waypoint and mark it reached.
    void complete();

    // Returns the next planned waypoint without removing it.
    Waypoint current() const { return _queue[_head]; }

    // Returns true if a waypoint is being pursued.
    bool active() const;

    // Evaluate progress against the latest pose.
    WaypointStatus update(const Pose& pose, float toleranceM);

    // Drop all waypoints (e.g. on reset).
    void clear();

    uint8_t size() const { return _count; }

private:
    static constexpr uint8_t MAX_WAYPOINTS = 8;

    Waypoint _queue[MAX_WAYPOINTS];
    uint8_t  _head;
    uint8_t  _count;
    uint32_t _nextId;
};

#endif // ANR_WAYPOINT_H