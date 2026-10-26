/**
 * @file    config.h
 * @brief   Board pin assignments and robot constants shared by both navigation modes: geometry, direction fixes, wheel speed
 *          loop gains, timing. They mirror sim/config.py; replace them with the measured values (SETUP.md, section 5).
 *          Mode settings live with the mode: src/course.cpp (start pose, finish) and src/maze.cpp (maze tuning).
 */
#pragma once

#include <stdint.h>
#include "math_util.h"
#include "odometry.h"
#include "tof_layout.h"

// Board wiring: Black Pill STM32F411CE. Verify against the physical wiring.

// Motor 1 = left, motor 2 = right. Each H-bridge has two PWM inputs (A, B): TIM3 CH1..CH4, AF2.
//   MT1A PA6 (CH1), MT1B PA7 (CH2), MT2A PB0 (CH3), MT2B PB1 (CH4)

// Encoders (quadrature): CL1 = left on TIM2 PA0 / PA1 (AF1), CL2 = right on TIM4 PB6 / PB7 (AF2)
// I2C1: PB8 SCL, PB9 SDA (AF4). USART1: PA9 TX, PA10 RX (AF7). SWD: PA13 / PA14.

// VL53L0X shutdown lines. Sensor k must match TOF_LAYOUT.sensors[k]: 0..3 = right edge,
// right middle, left middle, left edge. Swap pins here if the wiring differs. One entry per sensor, cfg::TOF_COUNT in all.
#define TOF_XSHUT_PORTS \
    { GPIOA, GPIOB, GPIOB, GPIOB }
#define TOF_XSHUT_PINS \
    { GPIO_PIN_15, GPIO_PIN_3, GPIO_PIN_4, GPIO_PIN_5 }

// User buttons USER_BTN (PA2) and USER_BTN2 (PA3), active low with pull-ups.
#define BUTTON_PORT  GPIOA
#define BUTTON_PIN   GPIO_PIN_2
#define BUTTON2_PORT GPIOA
#define BUTTON2_PIN  GPIO_PIN_3
// Status LED L_STATUS (PC13, active low)
#define LED_PORT GPIOC
#define LED_PIN  GPIO_PIN_13

// User LEDs L1 (PA8) and L2 (PB15), active high
#define L1_PORT GPIOA
#define L1_PIN  GPIO_PIN_8
#define L2_PORT GPIOB
#define L2_PIN  GPIO_PIN_15

// Switches SW1..SW3 (PB12, PB13, PB14), pulled up, active low
#define SW_PORT GPIOB
#define SW_PINS \
    { GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14 }

// Battery sense BAT: PA4 = ADC1_IN4
#define BAT_PORT    GPIOA
#define BAT_PIN     GPIO_PIN_4
#define BAT_CHANNEL ADC_CHANNEL_4

namespace cfg {

// Physical ToF layout. Indices match the XSHUT wiring above.
constexpr int TOF_COUNT = 4;
constexpr int TOF_RIGHT_EDGE = 0, TOF_RIGHT_MIDDLE = 1, TOF_LEFT_MIDDLE = 2, TOF_LEFT_EDGE = 3;
constexpr float TOF_ANGLE_DEG[TOF_COUNT] = {-60.0f, -20.0f, 20.0f, 60.0f};
constexpr float TOF_MOUNT_MM[TOF_COUNT][2] = {{60.0f, -45.0f}, {60.0f, -20.0f}, {60.0f, 20.0f}, {60.0f, 45.0f}};
constexpr int TOF_FOV_RAYS = 5;
constexpr float TOF_FOV_DEG[TOF_FOV_RAYS] = {-12.0f, -6.0f, 0.0f, 6.0f, 12.0f};
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

// USART1 PA9/PA10: comms packets (remote.h) to an HC-05 or a wired adapter; printf travels as Log packets.
// 9600 is the HC-05 factory baud; if you change the module with AT+UART, change this to match.
constexpr uint32_t UART1_BAUD_RATE = 9600;
constexpr uint32_t TELEMETRY_PERIOD_MS = 100;  // ~30 bytes per packet: 10 Hz fits in a third of 9600 baud

constexpr float WHEEL_RADIUS_MM = 16.0f;
constexpr float WHEEL_TRACK_MM  = 90.0f;   // wheel separation
constexpr float COUNTS_PER_REV  = 420.0f;  // x4 hardware counts per wheel revolution (after the gearbox); measure on the robot
constexpr float MM_PER_COUNT    = 2.0f * math::PI * WHEEL_RADIUS_MM / COUNTS_PER_REV;

// Direction fixes found during bring-up: +1 or -1 so that "forward" counts up and drives forward.
constexpr int ENC_SIGN_L   = 1;
constexpr int ENC_SIGN_R   = 1;
constexpr int MOTOR_SIGN_L = 1;
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
constexpr float SPEED_KP = 0.0004f;      // duty per mm/s
constexpr float SPEED_KI = 0.002f;
constexpr float SPEED_CORR_LIMIT = 0.3f;
constexpr float SPEED_TARGET_DEADBAND_MM_S = 10.0f;  // PID bypass below this target magnitude; feed-forward remains

// Timing
constexpr unsigned SPEED_LOOP_HZ = 200;     // wheel speed loop: one TIM11 interrupt per period (board.cpp)
constexpr float SPEED_LOOP_DT_S = 1.0f / SPEED_LOOP_HZ;
constexpr unsigned CONTROL_PERIOD_MS = 33;  // ToF timing budget, navigation and odometry step
constexpr unsigned GYRO_PERIOD_MS = 5;      // gyro read, averaged over each control step
constexpr uint16_t GYRO_CALIBRATION_SAMPLES = 400; // ~2 s stationary at 200 Hz
constexpr float RUN_TIME_LIMIT_S = 240.0f;

// Manual mode (env:manual): wheel speeds come from the host's Drive packets.
constexpr float MANUAL_MAX_MM_S = 300.0f;        // clamp on each wheel; raise it once the robot is on the floor and tuned
constexpr uint32_t MANUAL_TIMEOUT_MS = 300;      // wheels stop when no Drive packet arrives for this long

}  // namespace cfg
