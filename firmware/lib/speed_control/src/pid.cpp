/**
 * @file    pid.cpp
 * @brief   PID implementation. See pid.h.
 */
#include "pid.h"

#include <algorithm>

namespace speed_control {

float Pid::update(float setpoint, float measured, float dt) {
    const float err = setpoint - measured;

    integral_ += err * dt;
    if (integralLimit > 0) integral_ = std::clamp(integral_, -integralLimit, integralLimit);

    const float deriv = dt > 0 ? (err - prevError_) / dt : 0.0f;
    prevError_ = err;

    const float out = kp * err + ki * integral_ + kd * deriv;
    return outputLimit > 0 ? std::clamp(out, -outputLimit, outputLimit) : out;
}

}  // namespace speed_control
