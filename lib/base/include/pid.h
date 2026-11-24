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
        // Anti-windup: the integral term alone may not exceed the output limit. Without this it winds up while the output
        // is saturated (to ki * integralLimit, far beyond outputLimit) and keeps the output pinned for a long time after
        // the error changes sign: the wheel was driven to a stop for ~1 s in a real run.
        float limit = integralLimit;
        if (outputLimit > 0 && ki > 0) {
            const float fromOutput = outputLimit / ki;
            limit = limit > 0 ? std::min(limit, fromOutput) : fromOutput;
        }
        if (limit > 0) integral_ = std::clamp(integral_, -limit, limit);

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
