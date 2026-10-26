/**
 * @file    motor.cpp
 * @brief   Motor PWM and wheel control implementation. TIM11 runs the speed
 *          loop at cfg::SPEED_LOOP_HZ; TIM2/TIM4 decode encoders in hardware.
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
    __HAL_TIM_SET_COMPARE(pwm_, channelA_, u > 0.0f ? pulse : 0);
    __HAL_TIM_SET_COMPARE(pwm_, channelB_, u < 0.0f ? pulse : 0);
}

Motor::Motor(TIM_HandleTypeDef* pwm, uint32_t channelA, uint32_t channelB,
             TIM_HandleTypeDef* encoder, const speed_control::SpeedController& controller,
             int motorSign, int encoderSign)
    : pwm_(pwm), channelA_(channelA), channelB_(channelB), motorSign_(motorSign),
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

Motor gLeft(&htim3, TIM_CHANNEL_1, TIM_CHANNEL_2, &htim2, makeController(), cfg::MOTOR_SIGN_L, cfg::ENC_SIGN_L);
Motor gRight(&htim3, TIM_CHANNEL_3, TIM_CHANNEL_4, &htim4, makeController(), cfg::MOTOR_SIGN_R, cfg::ENC_SIGN_R);

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
