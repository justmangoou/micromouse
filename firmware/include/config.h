/**
 * @file    config.h
 * @brief   Robot constants: wheel and encoder geometry, direction fixes, wheel speed loop gains, timing, start pose.
 *          They mirror sim/config.py; replace them with the measured values (SETUP.md, section 5).
 */
#pragma once

#include <stdint.h>
#include "math_util.h"

namespace cfg {

// USART1 PA9/PA10: debug console or HC-05, shared with printf.
// Match the module's configured baud before using Bluetooth.
constexpr uint32_t UART1_BAUD_RATE = 460800;

constexpr float WHEEL_RADIUS_CM = 1.6f;
constexpr float WHEEL_TRACK_CM = 9.0f;        // wheel separation
constexpr float COUNTS_PER_REV = 420.0f; // x4 hardware counts per wheel revolution (after the gearbox); measure on the robot
constexpr float CM_PER_COUNT = 2.0f * math::PI * WHEEL_RADIUS_CM / COUNTS_PER_REV;

// Direction fixes found during bring-up: +1 or -1 so that "forward" counts up and drives forward.
constexpr int ENC_SIGN_L = 1;
constexpr int ENC_SIGN_R = 1;
constexpr int MOTOR_SIGN_L = 1;
constexpr int MOTOR_SIGN_R = 1;
constexpr float GYRO_SIGN = 1.0f;       // +1 when positive gyro-z is a CCW (left) turn

constexpr float V_FULL_CM_S = 62.2f;      // wheel speed at full duty, cm/s (measure it: SETUP.md step 6)

// Wheel speed loop: duty = feed-forward (target / V_FULL) + PID on the speed error. Gains are a starting
// point only; tune on the car. Set USE_SPEED_LOOP = false for the open-loop behaviour the sim assumes.
constexpr bool USE_SPEED_LOOP = true;
constexpr float SPEED_KP = 0.004f;      // duty per cm/s
constexpr float SPEED_KI = 0.02f;
constexpr float SPEED_CORR_LIMIT = 0.3f;

// Timing
constexpr unsigned SPEED_LOOP_HZ = 200;     // wheel speed loop: one TIM11 interrupt per period (board.cpp)
constexpr float SPEED_LOOP_DT_S = 1.0f / SPEED_LOOP_HZ;
constexpr unsigned CONTROL_PERIOD_MS = 33;  // ToF timing budget, navigation and odometry step
constexpr unsigned GYRO_PERIOD_MS = 5;      // gyro read, averaged over each control step
constexpr unsigned CALIBRATION_STEPS = 60; // ~2 s locked on the line, gyro bias estimation
constexpr float RUN_TIME_LIMIT_S = 240.0f;

// Start pose: behind the start line, facing +y (cm, rad)
constexpr float START_X = 30.0f;
constexpr float START_Y = -8.0f;
constexpr float START_TH = 1.57079633f;

}  // namespace cfg
