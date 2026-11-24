/**
 * @file    odometry.h
 * @brief   Encoder + gyro complementary filter. Port of FusedOdometry in sim/robot.py. Holds no robot or tuning
 *          numbers: the caller supplies all of them (firmware: cfg::ODOM and cfg::GYRO_BIAS in config.h).
 */
#pragma once

#include <stdint.h>

#include "math_util.h"

namespace nav {

struct OdomParams {
    float wheelRadius;   // mm
    float track;         // mm, wheel separation
    float countsPerRev;
    float gyroWeight;    // weight of the gyro rate in the fused yaw rate, [0, 1]
    float slipRate;      // rad/s encoder/gyro disagreement above which the encoders are ignored
};

class FusedOdometry {
public:
    explicit FusedOdometry(const OdomParams& p) : p_(p) {}

    void reset(const math::Pose& pose);

    // dcL/dcR: encoder counts since the last call (forward positive);
    // gyro: offset-corrected yaw rate, rad/s, CCW positive. Calibrate in the sensor driver.
    void update(int32_t dcL, int32_t dcR, float gyro, float dt);

    void updateEncoders(int32_t dcL, int32_t dcR, float dt);

    math::Pose pose{0, 0, 0};
    float v = 0;      // forward speed, mm/s
    float omega = 0;  // fused yaw rate, rad/s
    float ds = 0;     // distance travelled in the last step, mm
    float dth = 0;    // heading change in the last step, rad

private:
    OdomParams p_;
};

// Follows the gyro's zero-rate offset while the robot is standing still. The bias measured at start-up drifts as the
// chip warms up, and a drifting bias is heading error that grows with time: 0.1 deg/s is 6 degrees a minute.
struct GyroBiasParams {
    float timeConstantS;  // how fast the bias follows the measured offset
    float maxRate;        // rad/s: a mean rate above this is motion, not bias
    float stillS;         // continuous stillness required before the first correction
};

class GyroBiasTracker {
public:
    explicit GyroBiasTracker(const GyroBiasParams& p) : p_(p) {}

    // rate: the step's mean offset-corrected rate, in the sensor frame and units the bias is kept in, rad/s.
    // stationary: wheels not moving and motors disabled. dt: seconds since the previous call.
    // Returns the amount to add to the bias (0 when not still), independent of the step rate.
    float update(float rate, bool stationary, float dt);
    void reset() { stillS_ = 0; }

private:
    GyroBiasParams p_;
    float stillS_ = 0;
};

}  // namespace nav
