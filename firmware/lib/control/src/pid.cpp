/**
 * @file    pid.cpp
 * @brief   PID implementation. See pid.h.
 */
#include "pid.h"

#include <algorithm>

namespace nav {

float Pid::update(float setpoint, float measured, float dt) {
    float err = setpoint - measured;
    integral_ += err * dt;
    if (integralLimit > 0) integral_ = std::clamp(integral_, -integralLimit, integralLimit);
    float deriv = dt > 0 ? (err - prevError_) / dt : 0.0f;
    prevError_ = err;
    float out = kp * err + ki * integral_ + kd * deriv;
    if (outputLimit > 0) out = std::clamp(out, -outputLimit, outputLimit);
    return out;
}

}  // namespace nav
