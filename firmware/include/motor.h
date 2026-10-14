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
#include "speed_controller.h"

class Motor {
public:
    // Timers are configured by boardInit(). Signs fix motor/encoder polarity.
    Motor(TIM_HandleTypeDef* pwm, uint32_t channelA, uint32_t channelB,
          TIM_HandleTypeDef* encoder, const speed_control::SpeedController& controller,
          int motorSign = 1, int encoderSign = 1);

    void init();
    void enable(bool on);
    void setTarget(float mmPerSec);
    void tick();             // speed-loop interrupt; samples the hardware encoder
    int32_t takeCounts();    // main loop; independent odometry counter history

private:
    void setDuty(float duty);  // internal PWM output, clamped to [-1, 1]
    uint16_t counter() const;

    TIM_HandleTypeDef* pwm_;
    uint32_t channelA_, channelB_;
    int motorSign_;
    TIM_HandleTypeDef* encoder_;
    int encoderSign_;
    speed_control::SpeedController controller_;
    volatile bool enabled_ = false;
    uint16_t lastLoop_ = 0;
    uint16_t lastOdometry_ = 0;
};

// TIM2/TIM4 count encoder pulses in hardware. motorTick() runs the speed
// estimate and PID at cfg::SPEED_LOOP_HZ; odometry reads counters separately.
void motorInit();
void motorEnable(bool on);
void motorSetTargets(float vLeft, float vRight);  // mm/s, forward positive
void motorTick();                                  // once per speed loop period (interrupt context)
void motorTakeCounts(int32_t* left, int32_t* right);  // counts since the last call, from the hardware counters
