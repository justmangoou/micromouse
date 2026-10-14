/**
 * @file    pid.h
 * @brief   PID with integral clamp and output limit. The sim's Pid in sim/robot.py is a line-for-line port of it.
 */
#pragma once

namespace control {

class Pid {
public:
    // Gains and limits are public so they can be tuned in place, e.g. pid.kp = 0.004f.
    float kp = 0, ki = 0, kd = 0;
    float integralLimit = 0;  // 0 = unlimited
    float outputLimit = 0;    // 0 = unlimited

    Pid() {}
    Pid(float kp_, float ki_, float kd_, float integralLimit_, float outputLimit_)
        : kp(kp_), ki(ki_), kd(kd_), integralLimit(integralLimit_), outputLimit(outputLimit_) {}

    // Forget the accumulated integral and the last error.
    void reset() { integral_ = prevError_ = 0; }

    float update(float setpoint, float measured, float dt);

private:
    float integral_ = 0, prevError_ = 0;
};

}  // namespace control
