/**
 * @file    motor.cpp
 * @brief   Motor PWM and wheel control implementation. TIM11 runs the speed
 *          loop at cfg::SPEED_LOOP_HZ; TIM2/TIM4 decode encoders in hardware.
 */
#include "motor.h"

#include "board.h"
#include "config.h"
#include "pins.h"
#include "speed_controller.h"
#include "speed_estimator.h"

void Motor::setDuty(float u) {
    u *= sign_;
    if (u > 1.0f) u = 1.0f;
    if (u < -1.0f) u = -1.0f;
    uint32_t top = __HAL_TIM_GET_AUTORELOAD(pwm_) + 1;  // full period of the PWM timer
    uint32_t pulse = (uint32_t)((u < 0 ? -u : u) * top);
    __HAL_TIM_SET_COMPARE(pwm_, channelA_, u > 0.0f ? pulse : 0);
    __HAL_TIM_SET_COMPARE(pwm_, channelB_, u < 0.0f ? pulse : 0);
}

namespace {

class Wheel {
public:
    Wheel(const Motor& motor, const nav::SpeedController& controller, TIM_HandleTypeDef* encoder, int encoderSign)
        : motor_(motor),
          controller_(controller),
          speed_(cfg::CM_PER_COUNT, cfg::SPEED_LOOP_DT_S),
          encoder_(encoder),
          encoderSign_(encoderSign) {}

    void init() { lastLoop_ = lastOdometry_ = counter(); }

    void enable(bool on) {
        enabled_ = on;
        if (!on) {
            target_ = 0;
            controller_.reset();
            motor_.setDuty(0.0f);
        }
    }

    void setTarget(float cmPerSec) { target_ = cmPerSec; }

    // Speed loop, interrupt context. The timer counts the pulses on its own; this only reads the position, also while
    // disabled so the speed estimate is warm when the wheel is enabled.
    void tick() {
        uint16_t now = counter();
        float speed = speed_.update(nav::counterDelta(now, lastLoop_) * encoderSign_);
        lastLoop_ = now;
        if (!enabled_) return;
        motor_.setDuty(controller_.update(target_, speed, cfg::SPEED_LOOP_DT_S));
    }

    // Main loop. Counts since the last call, from the same hardware counter but with its own "last" value, so it
    // shares no state with tick() and needs no interrupt lock.
    int32_t takeCounts() {
        uint16_t now = counter();
        int32_t d = nav::counterDelta(now, lastOdometry_) * encoderSign_;
        lastOdometry_ = now;
        return d;
    }

private:
    uint16_t counter() const { return (uint16_t)__HAL_TIM_GET_COUNTER(encoder_); }

    Motor motor_;
    nav::SpeedController controller_;
    nav::SpeedEstimator speed_;
    TIM_HandleTypeDef* encoder_;
    int encoderSign_;

    volatile bool enabled_ = false;
    volatile float target_ = 0;  // cm/s
    uint16_t lastLoop_ = 0;      // counter at the last tick()
    uint16_t lastOdometry_ = 0;  // counter at the last takeCounts()
};

nav::SpeedController makeController() {
    return nav::SpeedController(cfg::V_FULL_CM_S, cfg::USE_SPEED_LOOP,
                                nav::Pid(cfg::SPEED_KP, cfg::SPEED_KI, 0.0f, 20.0f, cfg::SPEED_CORR_LIMIT));
}

Wheel gLeft(Motor(&htim3, TIM_CHANNEL_1, TIM_CHANNEL_2, cfg::MOTOR_SIGN_L), makeController(), &htim2, cfg::ENC_SIGN_L);
Wheel gRight(Motor(&htim3, TIM_CHANNEL_3, TIM_CHANNEL_4, cfg::MOTOR_SIGN_R), makeController(), &htim4, cfg::ENC_SIGN_R);

}  // namespace

void wheelInit() {
    gLeft.enable(false);  // motors stopped, duty 0
    gRight.enable(false);
    gLeft.init();
    gRight.init();
}

void wheelEnable(bool on) {
    gLeft.enable(on);
    gRight.enable(on);
}

void wheelSetTargets(float vLeft, float vRight) {
    gLeft.setTarget(vLeft);
    gRight.setTarget(vRight);
}

void wheelTick() {
    gLeft.tick();
    gRight.tick();
}

void wheelTakeCounts(int32_t* left, int32_t* right) {
    *left = gLeft.takeCounts();
    *right = gRight.takeCounts();
}
