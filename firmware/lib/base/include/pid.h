/**
 * @file    pid.h
 * @brief   PID with integral clamp and output limit.
 */
#pragma once

#include <algorithm>

namespace speed_control {

class Pid {
public:
    float kp = 0, ki = 0, kd = 0;
    float integralLimit = 0;  // 0 = unlimited
    float outputLimit = 0;    // 0 = unlimited

    Pid() {}
    Pid(float kp_, float ki_, float kd_, float integralLimit_, float outputLimit_)
        : kp(kp_), ki(ki_), kd(kd_), integralLimit(integralLimit_), outputLimit(outputLimit_) {}

    float update(float setpoint, float measured, float dt) {
        const float err = setpoint - measured;

        integral_ += err * dt;
        if (integralLimit > 0) integral_ = std::clamp(integral_, -integralLimit, integralLimit);

        const float deriv = dt > 0 ? (err - prevError_) / dt : 0.0f;
        prevError_ = err;

        const float out = kp * err + ki * integral_ + kd * deriv;
        return outputLimit > 0 ? std::clamp(out, -outputLimit, outputLimit) : out;
    }

    // Forget the accumulated integral and the last error.
    void reset() { integral_ = prevError_ = 0; }

private:
    float integral_ = 0, prevError_ = 0;
};

}  // namespace speed_control
