#include "waypoint.h"

WaypointQueue::WaypointQueue()
    : _head(0), _count(0), _nextId(0) {}

void WaypointQueue::begin() {
    _head = 0;
    _count = 0;
    _nextId = 1;
}

bool WaypointQueue::push(const Waypoint& wp) {
    if (_count >= MAX_WAYPOINTS) return false;
    Waypoint copy = wp;
    copy.id = _nextId++;
    copy.active = true;

    uint8_t slot = _head + _count;
    if (slot >= MAX_WAYPOINTS) slot -= MAX_WAYPOINTS;
    _queue[slot] = copy;
    _count++;
    return true;
}

void WaypointQueue::complete() {
    if (_count == 0) return;
    _head = (_head + 1) % MAX_WAYPOINTS;
    _count--;
}

bool WaypointQueue::active() const {
    return _count > 0;
}

WaypointStatus WaypointQueue::update(const Pose& pose, float toleranceM) {
    WaypointStatus st;
    st.hasTarget = (_count > 0);
    st.reached = false;
    st.distanceM = 0.0f;
    st.headingErrRad = 0.0f;
    st.currentId = 0;

    if (!st.hasTarget) return st;

    const Waypoint& target = _queue[_head];
    st.currentId = target.id;

    float dx = target.x - pose.x;
    float dy = target.y - pose.y;
    st.distanceM = sqrtf(dx * dx + dy * dy);

    float desired = atan2f(dy, dx);
    float err = desired - pose.theta;
    while (err >  3.14159265f) err -= 2.0f * 3.14159265f;
    while (err < -3.14159265f) err += 2.0f * 3.14159265f;
    st.headingErrRad = err;

    if (st.distanceM <= toleranceM) {
        st.reached = true;
    }

    return st;
}

void WaypointQueue::clear() {
    _head = 0;
    _count = 0;
}