/**
 * @file    config.h
 * @brief   The robot: board wiring, geometry, sensors, motor model, control gains and timing.
 *
 * Sections:  1. wiring   2. ToF sensors   3. wheels and encoders   4. odometry and gyro
 *            5. wheel speed control   6. timing
 * Course-specific settings (start pose, finish, how the Navigator drives) are in course_params.h. Where a number comes
 * from (measurements, tests) is in docs/DECISIONS.md. Units: mm, mm/s, rad, s unless the name says otherwise.
 */
#pragma once

#include <stdint.h>
#include "math_util.h"
#include "odometry.h"
#include "tof_layout.h"

// =====================================================================================================================
// 1. Wiring: Blue Pill (STM32F103C8)
// =====================================================================================================================
// Motor A = left, motor B = right (TB6612-style driver). Direction pins are plain GPIO, speed is PWM:
//   A: AIN1 PB1, AIN2 PB0, PWMA PB14 = TIM1_CH2N      B: BIN1 PB10, BIN2 PB11, PWMB PB13 = TIM1_CH1N
// Encoders (quadrature, counted in hardware): left PA0 / PA1 on TIM2, right PA6 / PA7 on TIM3.
// I2C1 (IMU and ToF): PB6 SCL, PB7 SDA. USART1 (printf): PA9 TX. SWD: PA13 / PA14 (JTAG pins freed for GPIO).
// Buttons PB9 (KEY: start) and PB8, status LED PC13. The wheel speed loop runs from TIM4.

#define MOTOR_A_IN1_PORT GPIOB
#define MOTOR_A_IN1_PIN  GPIO_PIN_1
#define MOTOR_A_IN2_PORT GPIOB
#define MOTOR_A_IN2_PIN  GPIO_PIN_0
#define MOTOR_B_IN1_PORT GPIOB
#define MOTOR_B_IN1_PIN  GPIO_PIN_10
#define MOTOR_B_IN2_PORT GPIOB
#define MOTOR_B_IN2_PIN  GPIO_PIN_11

// VL53L0X shutdown (XSHUT) lines, in sensor order: right 45 deg, front, left 45 deg, right 90 deg (socket "senR", PA11).
// Five sensor sockets are wired; the one unused line is held low (boardInit) so that sensor stays in reset.
#define TOF_XSHUT_PORTS \
    { GPIOA, GPIOA, GPIOB, GPIOA }
#define TOF_XSHUT_PINS \
    { GPIO_PIN_12, GPIO_PIN_15, GPIO_PIN_3, GPIO_PIN_11 }
#define TOF_UNUSED_XSHUT_B_PINS (GPIO_PIN_4)   // socket "senL"

// Buttons (active low, pull-ups) and the status LED (active low).
#define BUTTON_PORT  GPIOB
#define BUTTON_PIN   GPIO_PIN_9
#define BUTTON2_PORT GPIOB
#define BUTTON2_PIN  GPIO_PIN_8
#define LED_PORT GPIOC
#define LED_PIN  GPIO_PIN_13

namespace cfg {

constexpr uint32_t UART1_BAUD_RATE = 115200;  // printf to a wired USB-serial adapter (transmit only)

// =====================================================================================================================
// 2. ToF sensors (VL53L0X)
// =====================================================================================================================
// TOF_COUNT = the sensors of the navigation layout (TOF_LAYOUT): right 45, front, left 45. TOF_SENSORS = all sensors on
// the robot: the 4th, TOF_RIGHT_90, looks sideways to the right. It is not safety-critical: the navigation layout, the
// health checks and the fault logic only see the first TOF_COUNT; a missing or unreliable 4th reads TOF_MAX_MM.
constexpr int TOF_COUNT = 3;
constexpr int TOF_SENSORS = 4;
constexpr int TOF_RIGHT_45 = 0, TOF_FRONT = 1, TOF_LEFT_45 = 2, TOF_RIGHT_90 = 3;
constexpr float TOF_ANGLE_DEG[TOF_SENSORS] = {-45.0f, 0.0f, 45.0f, -90.0f};   // counter-clockwise from straight ahead
// x forward, y left, from the wheel axle midpoint. TOF_RIGHT_90, from the PCB: socket senR's pins centred 39 mm ahead of the
// axle (the motor clamp holes) and 33 mm right, plus ~2 mm for the module standing in it. Check with a ruler.
constexpr float TOF_MOUNT_MM[TOF_SENSORS][2] = {{60.0f, -30.0f}, {65.0f, 0.0f}, {60.0f, 30.0f}, {39.0f, -35.0f}};
// Each sensor's field of view is modelled as five rays.
constexpr int TOF_FOV_RAYS = 5;
constexpr float TOF_FOV_DEG[TOF_FOV_RAYS] = {-12.0f, -6.0f, 0.0f, 6.0f, 12.0f};
// Range offset per sensor, micrometres, measured with env:test_tof_cal (flat card at 100 mm). TOF_OFFSET_UNSET keeps the
// sensor's factory value. An offset belongs to one physical sensor: recalibrate when a sensor is replaced.
constexpr int32_t TOF_OFFSET_UNSET = INT32_MIN;
constexpr int32_t TOF_OFFSET_UM[TOF_SENSORS] = {31000, 38000, 34000, TOF_OFFSET_UNSET};  // right 90: not calibrated yet
constexpr float TOF_MIN_MM = 30.0f;    // readings are clamped to this range; TOF_MAX_MM also means "nothing in range"
constexpr float TOF_MAX_MM = 1200.0f;

static_assert(TOF_COUNT > 0 && TOF_COUNT <= tof::MAX_SENSORS && TOF_SENSORS > TOF_COUNT && TOF_RIGHT_90 == TOF_COUNT, "ToF sensor count exceeds layout capacity");
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

// =====================================================================================================================
// 3. Wheels and encoders
// =====================================================================================================================
constexpr float WHEEL_RADIUS_MM = 16.5f;   // 33 mm wheels
constexpr float WHEEL_TRACK_MM  = 82.0f;   // distance between the wheels
constexpr float COUNTS_PER_REV  = 270.0f;  // encoder counts per wheel turn (x4 decoding), measured over 10 turns
constexpr float MM_PER_COUNT    = 2.0f * math::PI * WHEEL_RADIUS_MM / COUNTS_PER_REV;

// Direction fixes (+1 or -1) so that "forward" counts up and drives forward on both sides.
constexpr int ENC_SIGN_L   = -1;
constexpr int ENC_SIGN_R   = -1;
constexpr int MOTOR_SIGN_L = -1;  // the left motor is mounted mirrored
constexpr int MOTOR_SIGN_R = 1;

// =====================================================================================================================
// 4. Odometry and gyro (MPU6050)
// =====================================================================================================================
constexpr float GYRO_SIGN  = 1.0f;   // +1 when a positive gyro z is a counter-clockwise (left) turn
constexpr float GYRO_SCALE = 1.0f;   // scale correction (env:test_gyro_scale measured 1800.3 for 1800 deg)

// Heading = 98 % gyro + 2 % encoders; when they disagree by more than 1 rad/s the wheels slip and are ignored.
constexpr float ODOM_GYRO_WEIGHT = 0.98f;
constexpr float ODOM_SLIP_RATE_RAD_S = 1.0f;
constexpr nav::OdomParams ODOM = {WHEEL_RADIUS_MM, WHEEL_TRACK_MM, COUNTS_PER_REV, ODOM_GYRO_WEIGHT,
                                  ODOM_SLIP_RATE_RAD_S};

// The gyro's zero offset drifts: it is re-learned whenever the car stands still with the motors off.
constexpr nav::GyroBiasParams GYRO_BIAS = {
    10.0f,   // s, time constant
    0.035f,  // rad/s (2 deg/s): faster than this is motion, not offset
    0.5f,    // s of continuous stillness before the first correction
};

// Start-up calibration: hands off for GYRO_SETTLE_MS, then average GYRO_CALIBRATION_SAMPLES. The result is rejected (and
// retried while the car stands still) if the samples spread too much (it moved) or the offset is implausibly large.
constexpr uint32_t GYRO_SETTLE_MS = 2000;
constexpr uint16_t GYRO_CALIBRATION_SAMPLES = 400;   // ~2 s at 200 Hz
// Again before every run, in the start countdown: wait for the hand to leave the car, then average these.
constexpr uint32_t GYRO_RECAL_DELAY_MS = 1000;
constexpr uint16_t GYRO_RECAL_SAMPLES = 300;         // ~1.5 s
constexpr float GYRO_CAL_MAX_STD_RAD_S = 0.01f;      // 0.6 deg/s; a still MPU6050 is ~0.002
constexpr float GYRO_CAL_MAX_BIAS_RAD_S = 0.1f;      // 5.7 deg/s

// =====================================================================================================================
// 5. Wheel speed control (200 Hz, lib/base/speed_controller.h)
// =====================================================================================================================
// Navigation speed scale: the Navigator's wheel commands are fractions of this. Not the motors' real top speed.
// The Navigator's speed scale (wheel command 1.0). 680 laps the map in ~5.9 s (floor runs without the block, no hits).
constexpr float V_FULL_MM_S = 680.0f;

// Measured motor model (src/test_step.cpp, car on the floor): speed = gain * (duty - offset). The controller's
// feed-forward is its inverse, duty = target / gain + offset. The right wheel is ~3 % faster, so each side has its own.
constexpr float MOTOR_FF_MM_S_L = 2725.0f, MOTOR_FF_MM_S_R = 2690.0f;     // mm/s per unit duty
constexpr float MOTOR_FF_OFFSET_L = 0.068f, MOTOR_FF_OFFSET_R = 0.065f;   // duty at which the wheel starts to turn

// PI correction on top of the feed-forward, tuned in a host simulation of that model.
constexpr float SPEED_KP = 0.0006f;                  // duty per mm/s
constexpr float SPEED_KI = 0.003f;                   // duty per mm/s per second
constexpr float SPEED_CORR_LIMIT = 0.3f;             // the PI correction (and its integral) stays within +-0.3 duty
constexpr float SPEED_TARGET_DEADBAND_MM_S = 10.0f;  // below this target only the feed-forward acts

// Speed measurement: one encoder count per 5 ms tick is 77 mm/s, so counts are summed over 4 ticks (19 mm/s per count)
// and then low-pass filtered.
constexpr int SPEED_WINDOW_TICKS = 4;
constexpr float SPEED_FILTER_ALPHA = 0.5f;

// =====================================================================================================================
// 6. Timing
// =====================================================================================================================
constexpr unsigned SPEED_LOOP_HZ = 200;              // wheel speed loop: TIM4 interrupt
constexpr float SPEED_LOOP_DT_S = 1.0f / SPEED_LOOP_HZ;
constexpr unsigned CONTROL_PERIOD_MS = 33;           // control step: ToF timing budget, odometry, navigation
constexpr unsigned GYRO_PERIOD_MS = 5;               // gyro reads, averaged over each control step
constexpr float RUN_TIME_LIMIT_S = 240.0f;
// A route plan runs inside one control step and blocks it (~150 ms without an FPU). The sensor snapshot after it may be
// this old without counting as a timing fault.
constexpr uint32_t STALL_MAX_AGE_MS = 500;

}  // namespace cfg
