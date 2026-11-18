/**
 * @file    motor.cpp
 * @brief   Motor PWM and wheel control implementation. TIM4 runs the speed
 *          loop at cfg::SPEED_LOOP_HZ; TIM2/TIM3 decode encoders in hardware.
 */
#include "motor.h"

#include "board.h"
#include "config.h"
#include "speed_controller.h"
#include "reliability.h"
#include <cmath>

void Motor::setDuty(float u) {
    u *= motorSign_;
    if (u > 1.0f) u = 1.0f;
    if (u < -1.0f) u = -1.0f;
    uint32_t top = __HAL_TIM_GET_AUTORELOAD(pwm_) + 1;  // full period of the PWM timer
    uint32_t pulse = (uint32_t)((u < 0 ? -u : u) * top);
    HAL_GPIO_WritePin(in1Port_, in1Pin_, u > 0.0f ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(in2Port_, in2Pin_, u < 0.0f ? GPIO_PIN_SET : GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(pwm_, channel_, pulse);
}

Motor::Motor(TIM_HandleTypeDef* pwm, uint32_t channel, GPIO_TypeDef* in1Port, uint16_t in1Pin,
             GPIO_TypeDef* in2Port, uint16_t in2Pin, TIM_HandleTypeDef* encoder,
             const speed_control::SpeedController& controller, int motorSign, int encoderSign)
    : pwm_(pwm), channel_(channel), in1Port_(in1Port), in2Port_(in2Port), in1Pin_(in1Pin), in2Pin_(in2Pin),
      motorSign_(motorSign),
      encoder_(encoder), encoderSign_(encoderSign), controller_(controller) {}

uint16_t Motor::counter() const { return static_cast<uint16_t>(__HAL_TIM_GET_COUNTER(encoder_)); }

void Motor::init() {
    const uint32_t irqState = __get_PRIMASK();
    __disable_irq();
    enable(false);
    controller_.reset();
    lastLoop_ = lastOdometry_ = counter();
    __set_PRIMASK(irqState);
}

void Motor::enable(bool on) {
    // Reset shared controller state without a speed-loop interrupt interleaving.
    const uint32_t irqState = __get_PRIMASK();
    __disable_irq();
    enabled_ = on;
    if (!on) {
        controller_.stop();
        setDuty(0.0f);
    }
    __set_PRIMASK(irqState);
}

void Motor::setTarget(float mmPerSec) {
    // Disabled motors retain a zero target so the PID cannot wind up.
    controller_.update_target(enabled_ ? mmPerSec : 0.0f);
}

void Motor::tick() {
    // Hardware counts continuously, including while disabled. Keep speed warm.
    uint16_t now = counter();
    int32_t counts = speed_control::counterDelta(now, lastLoop_) * encoderSign_;
    lastLoop_ = now;
    float duty = controller_.update_counts(counts);
    if (enabled_) setDuty(duty);
}

int32_t Motor::takeCounts() {
    // Separate history from tick(): no shared counter state or interrupt lock.
    uint16_t now = counter();
    int32_t delta = speed_control::counterDelta(now, lastOdometry_) * encoderSign_;
    lastOdometry_ = now;
    return delta;
}

namespace {

reliability::CommandLease gLease;
volatile uint32_t gExpiries = 0;
volatile bool gEnabled = false;

speed_control::SpeedController makeController() {
    return speed_control::SpeedController(
        cfg::V_FULL_MM_S, cfg::MM_PER_COUNT, cfg::SPEED_LOOP_DT_S,
        speed_control::Pid(cfg::SPEED_KP, cfg::SPEED_KI, 0.0f, 200.0f, cfg::SPEED_CORR_LIMIT),
        cfg::SPEED_TARGET_DEADBAND_MM_S);
}

// Motor A (left): PWMA = TIM1_CH2N. Motor B (right): PWMB = TIM1_CH1N.
Motor gLeft(&htim1, TIM_CHANNEL_2, MOTOR_A_IN1_PORT, MOTOR_A_IN1_PIN, MOTOR_A_IN2_PORT, MOTOR_A_IN2_PIN, &htim2,
            makeController(), cfg::MOTOR_SIGN_L, cfg::ENC_SIGN_L);
Motor gRight(&htim1, TIM_CHANNEL_1, MOTOR_B_IN1_PORT, MOTOR_B_IN1_PIN, MOTOR_B_IN2_PORT, MOTOR_B_IN2_PIN, &htim3,
             makeController(), cfg::MOTOR_SIGN_R, cfg::ENC_SIGN_R);

}  // namespace

void motorInit() {
    gLeft.init();
    gRight.init();
}

void motorEnable(bool on) {
    const uint32_t irqState = __get_PRIMASK();
    __disable_irq();
    gEnabled = on;
    gLeft.enable(on);
    gRight.enable(on);
    if (on) gLease.publish(HAL_GetTick());
    else gLease.clear();
    __set_PRIMASK(irqState);
}

void motorSetTargets(float vLeft, float vRight) {
    const uint32_t irqState = __get_PRIMASK();
    __disable_irq();
    const uint32_t now = HAL_GetTick();
    if (gEnabled && gLease.expired(now)) {
        motorEnable(false);
        ++gExpiries;
    } else if (!std::isfinite(vLeft) || !std::isfinite(vRight)) {
        motorEnable(false);
    } else if (gEnabled) {
        gLeft.setTarget(vLeft);
        gRight.setTarget(vRight);
        gLease.publish(now);
    }
    __set_PRIMASK(irqState);
}

void motorTick() {
    if (gEnabled && gLease.expired(HAL_GetTick())) {
        motorEnable(false);
        ++gExpiries;
    }
    gLeft.tick();
    gRight.tick();
}

uint32_t motorCommandExpiries() { return gExpiries; }
bool motorsEnabled() { return gEnabled; }

void motorTakeCounts(int32_t* left, int32_t* right) {
    *left = gLeft.takeCounts();
    *right = gRight.takeCounts();
}
