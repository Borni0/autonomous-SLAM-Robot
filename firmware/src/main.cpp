// =============================================================================
// Autonomous Indoor Navigation Robot - Phase 4
// Main entry point - transparent ROS 2 motor/encoder/IMU-free driver
// =============================================================================
//
// Phase 4 changed the firmware's role:
//
//   * The IMU/MPU6050 + heading_fusion code was removed.
//   * When the Raspberry Pi is connected and sends `ROS2 ON`, the
//     firmware's own avoidance FSM, waypoint nav and reactive obstacle
//     stops are BYPASSED. The Pi's Nav2 / A3 LiDAR is the only
//     authority on where to go.
//   * The firmware now streams two telemetry records over USB serial:
//       `P x= y= theta= vL= vR= age=`  at ~50 Hz
//       `T bat= estop= tof= mode=`      at ~5 Hz
//     The Pi's esp32_bridge parses those and publishes /odom,
//     /battery_state, /estop and /tof/range.
//
//   * The standalone behaviour (AUTO + waypoint + ToF avoidance) is
//     still available when ROS2 mode is OFF, so the robot remains
//     usable with just a USB serial cable and no Pi.

#include <Arduino.h>
#include <Wire.h>

#include "config.h"
#include "motor.h"
#include "encoder.h"
#include "odometry.h"
#include "tof.h"
#include "obstacle.h"
#include "navigation.h"
#include "display.h"
#include "estop.h"
#include "battery.h"
#include "pid.h"
#include "command.h"
#include "waypoint.h"
#include "telemetry.h"

static MotorDriver      leftMotor(M1_RPWM, M1_LPWM, M1_R_EN, M1_L_EN);
static MotorDriver      rightMotor(M2_RPWM, M2_LPWM, M2_R_EN, M2_L_EN);
static Encoder          encLeft(ENC_LEFT_PIN);
static Encoder          encRight(ENC_RIGHT_PIN);
static Odometry         odom;
static ToFSensor        tof(TOF_XSHUT_PIN);
static ObstacleDetector obstacle;
static Navigation       nav;
static Display          display;
static EmergencyStop    estop;
static BatteryMonitor   battery;
static PIDController    pidLeft;
static PIDController    pidRight;
static CommandLine      cmdline;
static WaypointQueue    waypointQueue;

static const float WAYPOINT_TOLERANCE_M = 0.10f;

static uint32_t lastControlUs = 0;
static uint32_t lastHeartbeatMs = 0;
static uint32_t lastTelemetryMs = 0;
static uint16_t poseStreamDivider = 0;
static bool     systemFault = false;
static bool     estopState = false;
// ROS2 mode: when true the firmware behaves as a transparent driver for
// the Pi. Initial value is taken from ROS2_DEFAULT_ACTIVE so a bare
// ESP32 (no Pi attached) boots into the safe standalone AUTO mode.
// Once ROS2 ON is seen on the serial port we stay in ROS2 mode until
// the user explicitly sends ROS2 OFF (mirrors cmdline.ros2Active()).
static bool     ros2Active = ROS2_DEFAULT_ACTIVE;
// Hardware-level ToF safety override: independent of nav mode, the front
// ToF can drop both motors to zero for EMERGENCY_HOLD_MS. The override
// only fires while the ToF is fresh; the FSM in navigation.cpp is the
// fallback in AUTO mode.
static uint32_t tofHoldUntilMs = 0;

static float percentToLinearMps(int8_t pct) {
    return (static_cast<float>(pct) / 100.0f) * 0.30f; // ASSUMPTION
}

static telemetry::NavMode currentTelemetryMode() {
    if (systemFault) return telemetry::NavMode::FAULT;
    if (ros2Active)  return telemetry::NavMode::ROS2;
    return telemetry::NavMode::AUTO;
}

static void enterFault(const char* reason) {
    systemFault = true;
    leftMotor.stop();
    rightMotor.stop();
    Serial.printf("[ANR] FAULT: %s\n", reason);
    display.showFault(reason);
}

static void processCommandEvent() {
    WaypointRequest wp;
    switch (cmdline.takeEvent(wp)) {
        case CommandEvent::RESET_HEADING:
            odom.reset();
            nav.reset();
            break;
        case CommandEvent::ADD_WAYPOINT: {
            Waypoint w;
            w.x = wp.x;
            w.y = wp.y;
            w.headingRad = 0.0f;
            w.active = true;
            if (!waypointQueue.push(w)) {
                Serial.println(F("[ANR] Waypoint queue full (max 8)"));
            }
            break;
        }
        case CommandEvent::LIST_WAYPOINTS:
            Serial.printf("[ANR] Waypoints: %u queued\n", waypointQueue.size());
            break;
        case CommandEvent::CLEAR_WAYPOINTS:
            waypointQueue.clear();
            break;
        case CommandEvent::NONE:
        default:
            break;
    }
}

void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(200);
    Serial.println(F("[ANR] Booting..."));

    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW);

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ_HZ);

    leftMotor.begin();
    rightMotor.begin();
    encLeft.begin();
    encRight.begin();

    if (!display.begin()) {
        Serial.println(F("[ANR] OLED init failed"));
    }
    display.showSplash();

    if (!tof.begin()) {
        enterFault("ToF sensor not detected");
        return;
    }

    battery.begin();

    if (PID_ENABLED) {
        pidLeft.begin(PID_KP,  PID_KI,  PID_KD,  PID_OUTPUT_MAX, PID_INTEGRAL_MAX);
        pidRight.begin(PID_KP, PID_KI, PID_KD, PID_OUTPUT_MAX, PID_INTEGRAL_MAX);
    }

    estop.begin();
    cmdline.begin();
    waypointQueue.begin();

    odom.reset();
    nav.begin();

    telemetry::begin();

    Serial.println(F("[ANR] Ready"));
    if (ros2Active) {
        Serial.println(F("[ANR] ROS2 mode ACTIVE at boot"));
    }
}

void loop() {
    uint32_t nowMs = millis();

    // --- Emergency stop ---
    estopState = estop.update();
    if (estopState) {
        leftMotor.stop();
        rightMotor.stop();
        systemFault = true;
    }

    // --- Serial command interface ---
    cmdline.poll();
    // Mirror cmdline's mode into the loop-local ros2Active flag so the
    // nav branch below has a single source of truth.
    ros2Active = cmdline.ros2Active();
    processCommandEvent();

    // --- Battery monitor ---
    battery.update(nowMs);

    if (systemFault) {
        if ((nowMs - lastHeartbeatMs) >= HEARTBEAT_MS) {
            lastHeartbeatMs = nowMs;
            digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
        }
        display.showStatus(nav.mode(),
                           cmdline.mode(),
                           tof.distanceMm(),
                           0, 0,
                           odom.pose(),
                           odom.distanceTravelledM(),
                           battery.voltage(),
                           battery.status(),
                           tof.ok(),
                           estopState);
        if ((nowMs - lastTelemetryMs) >= TELEMETRY_RATE_MS) {
            lastTelemetryMs = nowMs;
            telemetry::emitPose(odom);
            telemetry::emitTelemetry(battery, estopState, tof,
                                     currentTelemetryMode());
        }
        return;
    }

    // --- Control loop timing ---
    uint32_t nowUs = micros();
    if ((nowUs - lastControlUs) < CONTROL_DT_US) return;
    float dtSec = (nowUs - lastControlUs) / 1000000.0f;
    lastControlUs = nowUs;

    // --- Sensors ---
    tof.update();
    battery.update(nowMs);

    // --- Encoders + odometry ---
    int32_t lTicks = encLeft.consumeTicks();
    int32_t rTicks = encRight.consumeTicks();
    odom.update(lTicks, rTicks, dtSec);

    // --- Obstacle evaluation (used by AUTO FSM and ToF safety override) ---
    obstacle.update(tof.distanceMm(), tof.ok(), tof.stale());

    // --- Waypoint update (only used when not in ROS2 mode) ---
    WaypointStatus wpStatus{false, false, 0.0f, 0.0f, 0};
    if (!ros2Active) {
        wpStatus = waypointQueue.update(odom.pose(), WAYPOINT_TOLERANCE_M);
        if (wpStatus.reached) {
            Serial.printf("[ANR] Waypoint %u reached\n", wpStatus.currentId);
            waypointQueue.complete();
        }
    }

    // --- Hardware-level ToF safety override (always active) ---
    if (tof.ok() && !tof.stale()
        && tof.distanceMm() > 0
        && tof.distanceMm() <= EMERGENCY_STOP_MM
        && nowMs >= tofHoldUntilMs) {
        // Latch the override for EMERGENCY_HOLD_MS so we don't
        // immediately re-arm under a still-too-close ToF.
        tofHoldUntilMs = nowMs + EMERGENCY_HOLD_MS;
        leftMotor.stop();
        rightMotor.stop();
    }
    bool tofBraking = (nowMs < tofHoldUntilMs);

    // --- Decide motor command ---
    MotorCommand cmd{0, 0};

    if (ros2Active) {
        // Transparent driver mode: forward the most recent `M L=… R=…`
        // straight to the motors. STOP zeroes them. The nav FSM and
        // waypoint queue are NOT consulted.
        ManualCommand manual = cmdline.consume();
        if (manual.fresh) {
            cmd = {manual.left, manual.right};
        } else {
            // Watchdog: no M command for >= REMOTE_TIMEOUT_MS -> STOP.
            cmd = {0, 0};
        }
    } else if (cmdline.mode() == ControlMode::MANUAL) {
        ManualCommand manual = cmdline.consume();
        if (manual.fresh) {
            cmd = nav.applyManual(manual, estopState, tof.ok(), tof.stale());
        } else {
            cmd = {0, 0};
        }
    } else if (waypointQueue.active()) {
        cmd = nav.updateWaypoint(obstacle.state(),
                                 tof.ok(),
                                 tof.stale(),
                                 wpStatus.headingErrRad,
                                 wpStatus.distanceM,
                                 nowMs);
    } else {
        cmd = nav.updateAUTO(obstacle.state(), tof.ok(), tof.stale(), nowMs);
    }

    // --- Apply ToF safety override on top of whatever the nav layer said ---
    if (tofBraking) {
        cmd.left = 0;
        cmd.right = 0;
    }

    int8_t cmdLeftPct = cmd.left;
    int8_t cmdRightPct = cmd.right;

    if (PID_ENABLED && !estopState && !tofBraking) {
        float setL = percentToLinearMps(cmdLeftPct);
        float setR = percentToLinearMps(cmdRightPct);
        float measL = odom.speedLeftMps();
        float measR = odom.speedRightMps();

        float outL = pidLeft.update(setL, measL, dtSec);
        float outR = pidRight.update(setR, measR, dtSec);

        if (outL >  PID_OUTPUT_MAX) outL =  PID_OUTPUT_MAX;
        if (outL < -PID_OUTPUT_MAX) outL = -PID_OUTPUT_MAX;
        if (outR >  PID_OUTPUT_MAX) outR =  PID_OUTPUT_MAX;
        if (outR < -PID_OUTPUT_MAX) outR = -PID_OUTPUT_MAX;

        leftMotor.setSpeed((int8_t)outL);
        rightMotor.setSpeed((int8_t)outR);
    } else {
        leftMotor.setSpeed(cmdLeftPct);
        rightMotor.setSpeed(cmdRightPct);
    }

    // --- Stream pose + telemetry to the Pi ---
    // POSE_STREAM_DIVIDER == 1 -> emit every control tick (~50 Hz).
    // Larger values let the user throttle to 25 Hz, 12.5 Hz, etc.
    if (POSE_STREAM_DIVIDER <= 1 || ++poseStreamDivider >= POSE_STREAM_DIVIDER) {
        poseStreamDivider = 0;
        telemetry::emitPose(odom);
    }
    if ((nowMs - lastTelemetryMs) >= TELEMETRY_RATE_MS) {
        lastTelemetryMs = nowMs;
        telemetry::emitTelemetry(battery, estopState, tof,
                                 currentTelemetryMode());
    }

    // --- Heartbeat ---
    if ((nowMs - lastHeartbeatMs) >= HEARTBEAT_MS) {
        lastHeartbeatMs = nowMs;
        digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
    }

    // --- OLED refresh ---
    display.showStatus(nav.mode(),
                       cmdline.mode(),
                       tof.distanceMm(),
                       leftMotor.currentSpeed(),
                       rightMotor.currentSpeed(),
                       odom.pose(),
                       odom.distanceTravelledM(),
                       battery.voltage(),
                       battery.status(),
                       tof.ok(),
                       estopState);

    // --- Safety checks ---
    if (estopState) {
        leftMotor.stop();
        rightMotor.stop();
        systemFault = true;
        return;
    }
    if (battery.status() == BatteryStatus::CRITICAL) {
        enterFault("Battery CRITICAL");
        return;
    }
    if (!tof.ok() || tof.stale()) {
        enterFault("ToF stale/fault");
        return;
    }
    if (obstacle.state() == ObstacleState::SENSOR_FAULT) {
        enterFault("Obstacle sensor fault");
        return;
    }
}
