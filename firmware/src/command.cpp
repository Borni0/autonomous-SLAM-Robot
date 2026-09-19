#include "command.h"
#include "config.h"
#include <Arduino.h>

CommandLine::CommandLine()
    : _index(0),
      _mode(ROS2_DEFAULT_ACTIVE ? ControlMode::ROS2 : ControlMode::AUTO),
      _lastLeft(0), _lastRight(0), _lastCommandMs(0),
      _event(CommandEvent::NONE), _waypointRequest{0.0f, 0.0f} {}

void CommandLine::begin() {
    _index = 0;
    _mode = ROS2_DEFAULT_ACTIVE ? ControlMode::ROS2 : ControlMode::AUTO;
    _lastLeft = 0;
    _lastRight = 0;
    _lastCommandMs = 0;
    _event = CommandEvent::NONE;
    _waypointRequest = {0.0f, 0.0f};

    Serial.println(F("[ANR] Serial interface ready"));
    Serial.println(F("  ROS2 ON | ROS2 OFF  -- transparent driver for the Pi"));
    Serial.println(F("  STOP                -- freeze motors"));
    Serial.println(F("  M L=40 R=40         -- manual wheel percent (only in MANUAL/ROS2 mode)"));
    Serial.println(F("  AUTO | RESET"));
    Serial.println(F("  GOTO X=1.0 Y=0.5 | WPLS | WPCLR"));
    Serial.println(F("  Telemetry out:  P x= y= theta= vL= vR= age=  (50 Hz)"));
    Serial.println(F("                  T bat= estop= tof= mode=      (5 Hz)"));
}

void CommandLine::poll() {
    while (Serial.available() > 0) {
        char c = static_cast<char>(Serial.read());

        if (c == '\r') continue;

        if (c == '\n') {
            if (_index > 0) {
                _buffer[_index] = '\0';
                execute(_buffer);
                _index = 0;
            }
            continue;
        }

        if (_index < (CMD_BUFFER_SIZE - 1)) {
            _buffer[_index++] = c;
        } else {
            _index = 0;
        }
    }
}

void CommandLine::execute(const char* line) {
    String s(line);
    s.trim();
    if (s.length() == 0) return;

    if (s.equalsIgnoreCase("AUTO")) {
        _mode = ControlMode::AUTO;
        _lastCommandMs = millis();
        Serial.println(F("[ANR] Mode: AUTO"));
        return;
    }
    if (s.equalsIgnoreCase("STOP")) {
        _mode = ControlMode::MANUAL;
        _lastLeft = 0;
        _lastRight = 0;
        _lastCommandMs = millis();
        Serial.println(F("[ANR] Mode: MANUAL (STOP)"));
        return;
    }
    if (s.equalsIgnoreCase("ROS2 ON") || s.equalsIgnoreCase("ROS2ON")) {
        // Enter transparent driver mode: own FSM is bypassed, M L=… R=…
        // and STOP go straight to the motors. Used by the Pi's
        // esp32_bridge immediately after opening the serial port.
        _mode = ControlMode::ROS2;
        _lastLeft = 0;
        _lastRight = 0;
        _lastCommandMs = millis();
        Serial.println(F("[ANR] ROS2 mode engaged"));
        return;
    }
    if (s.equalsIgnoreCase("ROS2 OFF") || s.equalsIgnoreCase("ROS2OFF")) {
        _mode = ControlMode::AUTO;
        _lastLeft = 0;
        _lastRight = 0;
        _lastCommandMs = millis();
        Serial.println(F("[ANR] ROS2 mode disengaged -> AUTO"));
        return;
    }
    if (s.equalsIgnoreCase("RESET")) {
        _mode = ControlMode::AUTO;
        _lastLeft = 0;
        _lastRight = 0;
        _event = CommandEvent::RESET_HEADING;
        _lastCommandMs = millis();
        Serial.println(F("[ANR] Heading reset"));
        return;
    }

    if (s.startsWith("M ") || s.startsWith("m ")) {
        // If we were in AUTO and a manual command arrives (e.g. teleop
        // serial test) promote to MANUAL; in ROS2 mode, stay in ROS2 so
        // M L=… R=… keeps working without bouncing through MANUAL.
        if (_mode != ControlMode::ROS2) _mode = ControlMode::MANUAL;
        int lIdx = s.indexOf("L=");
        int rIdx = s.indexOf("R=");
        if (lIdx >= 0) _lastLeft = static_cast<int8_t>(s.substring(lIdx + 2).toInt());
        if (rIdx >= 0) _lastRight = static_cast<int8_t>(s.substring(rIdx + 2).toInt());
        _lastCommandMs = millis();
        Serial.printf("[ANR] %s L=%d R=%d\n",
                      _mode == ControlMode::ROS2 ? "ROS2" : "MANUAL",
                      _lastLeft, _lastRight);
        return;
    }

    if (s.startsWith("GOTO ") || s.startsWith("goto ")) {
        int xIdx = s.indexOf("X=");
        int yIdx = s.indexOf("Y=");
        if (xIdx < 0 || yIdx < 0) {
            Serial.println(F("[ANR] GOTO syntax: GOTO X=1.0 Y=0.5"));
            return;
        }
        _waypointRequest.x = s.substring(xIdx + 2).toFloat();
        _waypointRequest.y = s.substring(yIdx + 2).toFloat();
        _mode = ControlMode::AUTO;
        _event = CommandEvent::ADD_WAYPOINT;
        _lastCommandMs = millis();
        Serial.printf("[ANR] GOTO X=%.2f Y=%.2f queued\n", _waypointRequest.x, _waypointRequest.y);
        return;
    }

    if (s.equalsIgnoreCase("WPLS")) {
        _event = CommandEvent::LIST_WAYPOINTS;
        _lastCommandMs = millis();
        return;
    }

    if (s.equalsIgnoreCase("WPCLR")) {
        _event = CommandEvent::CLEAR_WAYPOINTS;
        _lastCommandMs = millis();
        Serial.println(F("[ANR] Clear waypoints"));
        return;
    }

    Serial.println(F("[ANR] ? Unknown command"));
}

ManualCommand CommandLine::consume() {
    ManualCommand cmd;
    cmd.left = _lastLeft;
    cmd.right = _lastRight;
    cmd.fresh = (millis() - _lastCommandMs) < REMOTE_TIMEOUT_MS;
    return cmd;
}

CommandEvent CommandLine::takeEvent(WaypointRequest& request) {
    CommandEvent e = _event;
    request = _waypointRequest;
    _event = CommandEvent::NONE;
    return e;
}