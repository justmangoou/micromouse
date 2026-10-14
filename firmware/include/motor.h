/**
 * @file    motor.h
 * @brief   Motor PWM driver and paired wheel control with hardware encoders,
 *          speed estimation and PID.
 *
 * @par Notes
 *   - Forward = PWM on A and B low, reverse = the other way, zero = both low (coast).
 */
#pragma once

#include "stm32f4xx_hal.h"

class Motor {
public:
    // pwm: a timer already running in PWM mode on `channelA` and `channelB`. sign: +1 or -1 to fix a reversed motor.
    Motor(TIM_HandleTypeDef* pwm, uint32_t channelA, uint32_t channelB, int sign = 1)
        : pwm_(pwm), channelA_(channelA), channelB_(channelB), sign_(sign) {}

    // u in [-1, 1], forward positive (after the sign fix). Duty 1 = the timer's full period.
    void setDuty(float u);

private:
    TIM_HandleTypeDef* pwm_;
    uint32_t channelA_;
    uint32_t channelB_;
    int sign_;
};

// TIM2/TIM4 count encoder pulses in hardware. wheelTick() runs the speed
// estimate and PID at cfg::SPEED_LOOP_HZ; odometry reads counters separately.
void wheelInit();
void wheelEnable(bool on);
void wheelSetTargets(float vLeft, float vRight);  // cm/s, forward positive
void wheelTick();                                  // once per speed loop period (interrupt context)
void wheelTakeCounts(int32_t* left, int32_t* right);  // counts since the last call, from the hardware counters
