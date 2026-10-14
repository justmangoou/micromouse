/**
 * @file    pid.h
 * @brief   PID with integral clamp and output limit.
 */
#pragma once

namespace speed_control {

class Pid {
public:
    float kp = 0, ki = 0, kd = 0;
    float integralLimit = 0;  // 0 = unlimited
    float outputLimit = 0;    // 0 = unlimited

    Pid() {}
    Pid(float kp_, float ki_, float kd_, float integralLimit_, float outputLimit_)
        : kp(kp_), ki(ki_), kd(kd_), integralLimit(integralLimit_), outputLimit(outputLimit_) {}

    float update(float setpoint, float measured, float dt);

    // Forget the accumulated integral and the last error.
    void reset() { integral_ = prevError_ = 0; }

private:
    float integral_ = 0, prevError_ = 0;
};

}  // namespace speed_control
