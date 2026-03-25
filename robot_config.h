#pragma once

#include <Arduino.h>

namespace RobotConfig {

// =========================
// Pin Mapping (ESP32 + Wokwi)
// =========================
constexpr uint8_t X_STEP_PIN = 18;
constexpr uint8_t X_DIR_PIN = 19;
constexpr uint8_t X_ENABLE_PIN = 5;

// Both Y-axis A4988 drivers should share these logical STEP/DIR signals.
constexpr uint8_t Y_STEP_PIN = 16;
constexpr uint8_t Y_DIR_PIN = 17;
constexpr uint8_t Y_ENABLE_PIN = 4;

constexpr uint8_t X_LIMIT_LEFT_PIN = 12;
constexpr uint8_t X_LIMIT_RIGHT_PIN = 13;

constexpr uint8_t ULTRASONIC_TRIG_PIN = 10;
constexpr uint8_t ULTRASONIC_ECHO_PIN = 11;

constexpr uint8_t CAMERA_TRIGGER_PIN = 2;

// =========================
// Electrical Behavior
// =========================
constexpr bool LIMIT_SWITCH_ACTIVE_LOW = true;
constexpr bool DRIVER_ENABLE_ACTIVE_LOW = true;

// Set these to true if a motor moves opposite to expectation.
constexpr bool INVERT_X_DIRECTION = false;
constexpr bool INVERT_Y_DIRECTION = false;

// =========================
// Motion Parameters
// =========================
constexpr long STEP_DISTANCE_X = 230;
constexpr long STEP_DISTANCE_Y = 310;

constexpr float X_MAX_SPEED_STEPS_PER_SEC = 320.0F;
constexpr float X_ACCEL_STEPS_PER_SEC2 = 500.0F;

constexpr float Y_MAX_SPEED_STEPS_PER_SEC = 260.0F;
constexpr float Y_ACCEL_STEPS_PER_SEC2 = 420.0F;

// =========================
// Camera & Sensing
// =========================
constexpr uint32_t CAMERA_EXPOSURE_MS = 500;

// End-of-panel condition: distance larger than threshold means a gap.
constexpr float PANEL_GAP_THRESHOLD_CM = 30.0F;

constexpr uint8_t PANEL_GAP_CONFIRM_READINGS = 3;

// Ultrasonic polling cadence and timeout protection.
constexpr uint32_t ULTRASONIC_POLL_INTERVAL_MS = 250;
constexpr uint32_t ULTRASONIC_TIMEOUT_US = 30000;

// Optional startup wait for Serial monitor attachment.
constexpr uint32_t SERIAL_STARTUP_WAIT_MS = 1500;

}  // namespace RobotConfig
