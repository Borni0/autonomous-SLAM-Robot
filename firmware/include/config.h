#ifndef ANR_CONFIG_H
#define ANR_CONFIG_H

// =============================================================================
// Autonomous Indoor Navigation Robot - Phase 1
// Global Configuration
// =============================================================================
//
// All assumptions in this file are clearly marked.  Review and adjust every
// entry marked "ASSUMPTION" before powering the hardware on.  The robot will
// refuse to operate (FAULT state) until every ASSUMPTION is satisfied.
//
// ----------------------------------------------------------------------------
// VERIFIED HARDWARE SPECS
// ----------------------------------------------------------------------------
// Motor model:        <FILL IN>
// Encoder type:       single-channel (Hall), RISING-edge counted
// Encoder voltage:    3.3 V (ESP32 GPIO)
// Encoder PPR:        11 pulses per motor shaft revolution  -> see ENCODER_PPR
// Gear ratio:         30 : 1  (motor : wheel)                -> see GEAR_RATIO
// Wheel diameter:     64 mm (radius 32 mm)                   -> see WHEEL_RADIUS_M
// Wheel track width:  150 mm (centre-to-centre)              -> see WHEEL_BASE_M
//
// To re-measure the chassis:
//   1. Lift the rover on blocks so both wheels spin freely.
//   2. Open `make firmware-monitor` and command `M L=20 R=20`.
//   3. Read the final `Left Count` / `Right Count`, divide by 60 s, and
//      compare against METERS_PER_TICK for a sanity check.
//   4. Measure the wheel-to-wheel distance with calipers and update
//      WHEEL_BASE_M.
// ----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// Robot Geometry (calibrate before first run)
// -----------------------------------------------------------------------------
static constexpr float WHEEL_RADIUS_M        = 0.032f;   // 64 mm wheel diameter
static constexpr float WHEEL_BASE_M          = 0.150f;   // distance between wheels
static constexpr int   ENCODER_PPR           = 11;       // pulses per revolution (single channel)
static constexpr int   GEAR_RATIO            = 30;       // motor : wheel gear ratio
static constexpr float ENCODER_TICKS_PER_REV = static_cast<float>(ENCODER_PPR) * 2.0f * static_cast<float>(GEAR_RATIO);
static constexpr float METERS_PER_TICK       = (2.0f * 3.14159265f * WHEEL_RADIUS_M) / ENCODER_TICKS_PER_REV;

// -----------------------------------------------------------------------------
// Obstacle thresholds (VL53L0X mm)
// -----------------------------------------------------------------------------
static constexpr uint16_t EMERGENCY_STOP_MM  = 250;
static constexpr uint16_t SLOW_DOWN_MM       = 500;
static constexpr uint16_t CLEAR_MM           = 700;
static constexpr uint16_t TOF_TIMEOUT_MS     = 200;

// -----------------------------------------------------------------------------
// Motor / speed
// -----------------------------------------------------------------------------
static constexpr uint16_t PWM_FREQUENCY_HZ   = 20000;
static constexpr uint8_t  PWM_RESOLUTION_BITS = 8;
static constexpr uint16_t PWM_MAX            = 255;
static constexpr uint8_t  MOTOR_NORMAL_PCT   = 60;
static constexpr uint8_t  MOTOR_SLOW_PCT     = 35;
static constexpr uint16_t CONTROL_LOOP_MS    = 20;
static constexpr uint32_t CONTROL_DT_US      = CONTROL_LOOP_MS * 1000UL;

// -----------------------------------------------------------------------------
// PID speed controller (Phase 2)
// -----------------------------------------------------------------------------
static constexpr float PID_KP               = 1.6f;
static constexpr float PID_KI               = 4.0f;
static constexpr float PID_KD               = 0.05f;
static constexpr float PID_OUTPUT_MAX       = 100.0f; // percent
static constexpr float PID_INTEGRAL_MAX     = 60.0f;
static constexpr bool  PID_ENABLED          = true;   // ASSUMPTION: enable by default

// -----------------------------------------------------------------------------
// Avoidance manoeuvre timing
// -----------------------------------------------------------------------------
static constexpr uint16_t AVOID_TURN_MS      = 350;
static constexpr uint16_t AVOID_CLEAR_MS     = 800;
static constexpr uint32_t HEARTBEAT_MS       = 250;

// -----------------------------------------------------------------------------
// ROS 2 mode (transparent driver for Nav2 + LiDAR on the Raspberry Pi)
// -----------------------------------------------------------------------------
// Default = false so the firmware is safe to power up with no Pi attached.
// The Pi's esp32_bridge sends `ROS2 ON` shortly after opening the serial
// port. While ROS2 mode is active, the firmware's own navigation FSM is
// bypassed: `M L=… R=…` and `STOP` are passed straight to the motors.
// Hardware e-stop still wins.
static constexpr bool     ROS2_DEFAULT_ACTIVE       = false;
static constexpr uint16_t POSE_STREAM_DIVIDER       = 1;  // 1 = every control tick (~50 Hz)
static constexpr uint16_t TELEMETRY_RATE_MS         = 200; // 5 Hz telemetry line
// ToF safety override: even in ROS 2 mode, if the front ToF sees an
// obstacle within EMERGENCY_STOP_MM the firmware zeros both motors for
// EMERGENCY_HOLD_MS as a hardware-level last-resort brake.
static constexpr uint16_t EMERGENCY_HOLD_MS         = 250;
// Maximum allowed ms between successive `P` lines on the Pi side before
// it should warn that pose telemetry is degrading.
static constexpr uint16_t POSE_STALE_MS             = 250;

// -----------------------------------------------------------------------------
// I2C bus (shared by VL53L0X and OLED)
// -----------------------------------------------------------------------------
static constexpr int I2C_SDA_PIN             = 8;      // ASSUMPTION
static constexpr int I2C_SCL_PIN             = 9;      // ASSUMPTION
static constexpr uint32_t I2C_FREQ_HZ        = 400000UL;

// OLED
static constexpr uint8_t  OLED_ADDR          = 0x3C;   // ASSUMPTION
static constexpr int      OLED_RESET_PIN     = -1;
static constexpr uint16_t SCREEN_W           = 128;
static constexpr uint16_t SCREEN_H           = 64;

// VL53L0X uses default 0x29 on this bus.

// -----------------------------------------------------------------------------
// BTS7960 motor driver pin mapping
// Each motor uses RPWM (forward), LPWM (reverse), and enable.
// -----------------------------------------------------------------------------
// Left motor (BTS7960 #1)
static constexpr int M1_RPWM = 4;
static constexpr int M1_LPWM = 5;
static constexpr int M1_R_EN = 6;
static constexpr int M1_L_EN = 7;

// Right motor (BTS7960 #2)
static constexpr int M2_RPWM = 10;
static constexpr int M2_LPWM = 11;
static constexpr int M2_R_EN = 12;
static constexpr int M2_L_EN = 13;

// -----------------------------------------------------------------------------
// Encoders (interrupt-capable pins, pull-up)
//
// Verified: single-channel Hall effect encoders (RISING-edge counted).
// For quadrature (direction-sensing) encoders, add the B-channel pins
// here and adapt encoder.cpp to use a four-state quadrature decoder.
// -----------------------------------------------------------------------------
static constexpr int ENC_LEFT_PIN            = 14;
static constexpr int ENC_RIGHT_PIN           = 15;

// -----------------------------------------------------------------------------
// XSHUT for VL53L0X (optional, pulled high to keep sensor enabled)
// -----------------------------------------------------------------------------
static constexpr int TOF_XSHUT_PIN           = 16;

// -----------------------------------------------------------------------------
// Status LED (on-board)
// -----------------------------------------------------------------------------
static constexpr int STATUS_LED_PIN          = 18;

// -----------------------------------------------------------------------------
// Emergency stop (Phase 2)
// ASSUMPTION: active-LOW button to GND with internal pull-up enabled.
// Replace with the actual GPIO you wire the button to.
// -----------------------------------------------------------------------------
static constexpr int ESTOP_PIN               = 17;
static constexpr bool ESTOP_ACTIVE_LOW       = true;
static constexpr uint8_t ESTOP_DEBOUNCE_MS   = 30;

// -----------------------------------------------------------------------------
// Buzzer (optional, Phase 2)
// Set to -1 to disable.
// -----------------------------------------------------------------------------
static constexpr int BUZZER_PIN              = -1;

// -----------------------------------------------------------------------------
// Battery monitor (Phase 2)
// ASSUMPTION: voltage divider (R1=100k to Vbat, R2=33k to GND) on BAT_ADC_PIN.
// ratio = Vbat / Vadc = (R1 + R2) / R2 = 4.0303
// Adjust to match the actual resistor values on your hardware.
// -----------------------------------------------------------------------------
static constexpr int      BAT_ADC_PIN         = 1;       // ASSUMPTION: free ADC1 pin; do not share with motor PWM
static constexpr float    BAT_DIVIDER_RATIO   = 4.0303f;
static constexpr float    BAT_LOW_V           = 10.5f;    // 3S LiPo - adjust for your pack
static constexpr float    BAT_CRITICAL_V      = 9.6f;
static constexpr uint16_t BAT_SAMPLES         = 16;
static constexpr uint32_t BAT_PERIOD_MS       = 1000;

// -----------------------------------------------------------------------------
// Serial command interface (Phase 2)
// -----------------------------------------------------------------------------
static constexpr uint32_t SERIAL_BAUD         = 115200;
static constexpr uint16_t CMD_DEBOUNCE_MS     = 50;
static constexpr uint8_t  CMD_BUFFER_SIZE     = 64;

// -----------------------------------------------------------------------------
// Remote control mode duration
// After this many ms without input, the robot returns to AUTO.
// -----------------------------------------------------------------------------
static constexpr uint32_t REMOTE_TIMEOUT_MS   = 3000;

#endif // ANR_CONFIG_H