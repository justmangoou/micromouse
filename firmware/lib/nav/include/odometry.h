/**
 * @file    odometry.h
 * @brief   Encoder + gyro complementary filter. Port of FusedOdometry in sim/robot.py.
 */
#pragma once

#include <stdint.h>

#include "math_util.h"

namespace nav {

struct OdomParams {
    float wheelRadius = 16.0f;   // mm
    float track = 90.0f;         // mm, wheel separation
    float countsPerRev = 420.0f;
};

class FusedOdometry {
public:
    static constexpr float GYRO_WEIGHT = 0.98f;     // weight of the gyro rate in the fused yaw rate
    static constexpr float SLIP_RATE = 1.0f;   // rad/s encoder/gyro disagreement above which encoders are ignored

    explicit FusedOdometry(const OdomParams& p = OdomParams()) : p_(p) {}

    void reset(const math::Pose& pose);

    // dcL/dcR: encoder counts since the last call (forward positive);
    // gyro: offset-corrected yaw rate, rad/s, CCW positive. Calibrate in the sensor driver.
    void update(int32_t dcL, int32_t dcR, float gyro, float dt);

    math::Pose pose{0, 0, 0};
    float v = 0;      // forward speed, mm/s
    float omega = 0;  // fused yaw rate, rad/s
    float ds = 0;     // distance travelled in the last step, mm
    float dth = 0;    // heading change in the last step, rad

private:
    OdomParams p_;
};

}  // namespace nav
