/**
 * @file    config.h
 * @brief   Board pin assignments and robot constants shared by the run modes: geometry, direction fixes, wheel speed
 *          loop gains, timing. They mirror sim/config.py; replace them with the measured values (SETUP.md, section 5).
 *          Mode settings live with the mode: src/course.cpp (start pose, finish).
 */
#pragma once

#include <stdint.h>
#include "math_util.h"
#include "odometry.h"
#include "tof_layout.h"

// Board wiring: Blue Pill STM32F103C8 (schematic in the repo root). Verify against the physical wiring.

// Motor A = left, motor B = right (TB6612-style driver). Direction pins are plain GPIO, speed is PWM:
//   A: AIN1 PB1, AIN2 PB0, PWMA PB14 = TIM1_CH2N      B: BIN1 PB10, BIN2 PB11, PWMB PB13 = TIM1_CH1N
// Encoders (quadrature, hardware): left ENLB PA0 / ENLA PA1 on TIM2, right ENRA PA6 / ENRB PA7 on TIM3.
// I2C1: PB6 SCL, PB7 SDA. USART1: PA9 TX, PA10 RX. SWD: PA13 / PA14 (JTAG is released for PB3, PB4, PA15).
// Buttons on PB9 and PB8, status LED PC13. The wheel speed loop runs from TIM4.

#define MOTOR_A_IN1_PORT GPIOB
#define MOTOR_A_IN1_PIN  GPIO_PIN_1
#define MOTOR_A_IN2_PORT GPIOB
#define MOTOR_A_IN2_PIN  GPIO_PIN_0
#define MOTOR_B_IN1_PORT GPIOB
#define MOTOR_B_IN1_PIN  GPIO_PIN_10
#define MOTOR_B_IN2_PORT GPIOB
#define MOTOR_B_IN2_PIN  GPIO_PIN_11

// VL53L0X shutdown lines. Five are wired (senL PB4, senL1 PB3, senF PA15, senR1 PA12, senR PA11) but only three
// sensors are used: right 45 deg, front, left 45 deg. Sensor k must match TOF_LAYOUT.sensors[k]. The two unused
// lines are driven low in gpioInit() so those sensors stay in reset and do not answer at the default address.
#define TOF_XSHUT_PORTS \
    { GPIOA, GPIOA, GPIOB }
#define TOF_XSHUT_PINS \
    { GPIO_PIN_12, GPIO_PIN_15, GPIO_PIN_3 }
#define TOF_UNUSED_XSHUT_A_PINS (GPIO_PIN_11)  // senR
#define TOF_UNUSED_XSHUT_B_PINS (GPIO_PIN_4)   // senL

// Buttons, active low with pull-ups: USER_BTN (PB9) starts the run, USER_BTN2 (PB8) spare.
#define BUTTON_PORT  GPIOB
#define BUTTON_PIN   GPIO_PIN_9
#define BUTTON2_PORT GPIOB
#define BUTTON2_PIN  GPIO_PIN_8
// Status LED (PC13, active low)
#define LED_PORT GPIOC
#define LED_PIN  GPIO_PIN_13

namespace cfg {

// Physical ToF layout. Indices match the XSHUT wiring above.
constexpr int TOF_COUNT = 3;
constexpr int TOF_RIGHT_45 = 0, TOF_FRONT = 1, TOF_LEFT_45 = 2;
constexpr float TOF_ANGLE_DEG[TOF_COUNT] = {-45.0f, 0.0f, 45.0f};
// Mount position in the body frame (x forward, y left), mm. Placeholder values: measure them on the robot.
constexpr float TOF_MOUNT_MM[TOF_COUNT][2] = {{60.0f, -30.0f}, {65.0f, 0.0f}, {60.0f, 30.0f}};
constexpr int TOF_FOV_RAYS = 5;
constexpr float TOF_FOV_DEG[TOF_FOV_RAYS] = {-12.0f, -6.0f, 0.0f, 6.0f, 12.0f};
// Per-sensor range offset in micrometres, from env:test_tof_cal (target at a known distance). TOF_OFFSET_UNSET keeps the
// sensor's factory offset.
constexpr int32_t TOF_OFFSET_UNSET = INT32_MIN;
// Calibrated 2026-11-21 with a flat card at 100 mm (R1, F, L1): 50/50 valid readings each, mean 99/99/99 mm.
// R1 and L1 were also checked at 300 mm (299 and 300). F was replaced and recalibrated: its offset belongs to that unit.
constexpr int32_t TOF_OFFSET_UM[TOF_COUNT] = {31000, 38000, 34000};
constexpr float TOF_MIN_MM = 30.0f;
constexpr float TOF_MAX_MM = 1200.0f;

static_assert(TOF_COUNT > 0 && TOF_COUNT <= tof::MAX_SENSORS, "ToF sensor count exceeds layout capacity");
static_assert(TOF_FOV_RAYS > 0 && TOF_FOV_RAYS <= tof::MAX_FOV_RAYS, "ToF ray count exceeds layout capacity");

constexpr tof::Layout makeTofLayout() {
    tof::Layout layout{};
    layout.count = TOF_COUNT;
    layout.rayCount = TOF_FOV_RAYS;
    layout.minMm = TOF_MIN_MM;
    layout.maxMm = TOF_MAX_MM;
    for (int i = 0; i < TOF_COUNT; ++i)
        layout.sensors[i] = {TOF_MOUNT_MM[i][0], TOF_MOUNT_MM[i][1], math::toRadian(TOF_ANGLE_DEG[i])};
    for (int i = 0; i < TOF_FOV_RAYS; ++i) layout.rays[i] = math::toRadian(TOF_FOV_DEG[i]);
    return layout;
}
constexpr tof::Layout TOF_LAYOUT = makeTofLayout();

// USART1 PA9/PA10: printf text to a wired USB-serial adapter (transmit only).
constexpr uint32_t UART1_BAUD_RATE = 115200;

constexpr float WHEEL_RADIUS_MM = 16.5f;   // measured: 33 mm wheel diameter
constexpr float WHEEL_TRACK_MM  = 82.0f;   // measured: wheel separation
constexpr float COUNTS_PER_REV  = 270.0f;  // measured by hand over 10 turns: left 271.2, right 269.7 (x4 counts per wheel revolution)
constexpr float MM_PER_COUNT    = 2.0f * math::PI * WHEEL_RADIUS_MM / COUNTS_PER_REV;

// Direction fixes found during bring-up: +1 or -1 so that "forward" counts up and drives forward.
constexpr int ENC_SIGN_L   = -1;  // bench: left motor is mounted reversed, so forward counts down on the raw left encoder
constexpr int ENC_SIGN_R   = -1;  // found on the bench: motor B forward counts down on the right encoder
constexpr int MOTOR_SIGN_L = -1;  // bench: a positive command to motor A turns the left wheel backward
constexpr int MOTOR_SIGN_R = 1;
constexpr float GYRO_SIGN  = 1.0f;       // +1 when positive gyro-z is a CCW (left) turn
// Gyro scale error (the MPU6050's is up to +-3 %, which is 2.7 deg on a 90 deg turn). Spin in place for 10 turns
// (SETUP.md step 8): GYRO_SCALE = 3600 / the angle the odometry reported in degrees. 1.0 until measured.
constexpr float GYRO_SCALE = 1.0f;

// Odometry filter (lib/base/odometry.h). The weight and slip threshold are ports of FusedOdometry.ALPHA and SLIP_RATE
// in sim/robot.py; sim/tools/check_firmware.py compares them.
constexpr float ODOM_GYRO_WEIGHT = 0.98f;        // weight of the gyro rate in the fused yaw rate
constexpr float ODOM_SLIP_RATE_RAD_S = 1.0f;     // encoder/gyro disagreement above which the encoders are ignored
constexpr nav::OdomParams ODOM = {WHEEL_RADIUS_MM, WHEEL_TRACK_MM, COUNTS_PER_REV, ODOM_GYRO_WEIGHT,
                                  ODOM_SLIP_RATE_RAD_S};

// Gyro offset tracking while standing still with the motors off (lib/base/odometry.h GyroBiasTracker).
constexpr nav::GyroBiasParams GYRO_BIAS = {
    10.0f,   // s, time constant
    0.035f,  // rad/s (2 deg/s): faster than this is motion, not bias
    0.5f,    // s of continuous stillness before the first correction
};

constexpr float V_FULL_MM_S = 622.0f;      // wheel speed at full duty, mm/s (measure it: SETUP.md step 6)

// Wheel speed loop: duty = feed-forward (target / V_FULL) + PID on the speed error. Gains are a starting
// point only; tune on the car.
// Motor model measured with src/test_step.cpp, 2026-11-21 (on the floor, 0.15-0.35 duty): speed = gain * (duty - offset), so
// the feed-forward is target / gain + offset. The wheels-off gain is ~2900; V_FULL_MM_S above is only the navigation's speed
// scale and was 4.5x too low as a feed-forward, which left the PID saturated at its limit all run (the wobble).
// The right wheel is ~3% faster at the same duty, hence the separate values.
constexpr float MOTOR_FF_MM_S_L = 2725.0f, MOTOR_FF_MM_S_R = 2690.0f;  // mm/s per unit duty
constexpr float MOTOR_FF_OFFSET_L = 0.068f, MOTOR_FF_OFFSET_R = 0.065f; // duty at which the wheel just starts to move
// Gains from a host simulation of that plant (loaded response time 0.1-0.2 s): speed error 122 -> ~45 mm/s RMS on a 300/150/350 mm/s
// step sequence. The old 0.001 / 0.01 were chosen against a feed-forward that was 4.5x off.
constexpr float SPEED_KP = 0.0006f;      // duty per mm/s
constexpr float SPEED_KI = 0.003f;       // duty per mm/s per second; the integral term is limited to SPEED_CORR_LIMIT (pid.h)
constexpr float SPEED_CORR_LIMIT = 0.3f;
constexpr float SPEED_TARGET_DEADBAND_MM_S = 10.0f;  // PID bypass below this target magnitude; feed-forward remains
// One encoder count per 5 ms tick is 77 mm/s, so the raw speed is a staircase and Kp turns each step into a duty jump
// (the wiggle). Average the counts over 4 ticks (19 mm/s per count), then filter lightly.
constexpr int SPEED_WINDOW_TICKS = 4;
constexpr float SPEED_FILTER_ALPHA = 0.5f;

// Timing
constexpr unsigned SPEED_LOOP_HZ = 200;     // wheel speed loop: one TIM4 interrupt per period (board.cpp)
constexpr float SPEED_LOOP_DT_S = 1.0f / SPEED_LOOP_HZ;
constexpr unsigned CONTROL_PERIOD_MS = 33;  // ToF timing budget, navigation and odometry step
constexpr unsigned GYRO_PERIOD_MS = 5;      // gyro read, averaged over each control step
constexpr uint16_t GYRO_CALIBRATION_SAMPLES = 400; // ~2 s stationary at 200 Hz
constexpr uint32_t GYRO_SETTLE_MS = 2000;   // after power-on: hands off and let the chip settle before calibrating
// The calibration is rejected (and retried while the robot stands still) if the samples spread more than this (the robot
// was touched or moving) or the offset is larger than a gyro's plausible zero-rate error.
constexpr float GYRO_CAL_MAX_STD_RAD_S = 0.01f;   // 0.6 deg/s; a still MPU6050 is ~0.002
constexpr float GYRO_CAL_MAX_BIAS_RAD_S = 0.1f;   // 5.7 deg/s
constexpr float RUN_TIME_LIMIT_S = 240.0f;
// A replan runs the planner inside one control step. On the F103 (no FPU) that blocks the loop for ~150 ms, so the step
// after it may be this old before it counts as a timing fault. Only the course mode raises the limit, and only then.
constexpr uint32_t STALL_MAX_AGE_MS = 500;

}  // namespace cfg
