/**
 * @file    odometry.cpp
 * @brief   Fused odometry implementation. Gyro readings are calibrated by the sensor driver; encoders are ignored when
 *          they disagree with the gyro by more than OdomParams::slipRate (wheel slip).
 */
#include "odometry.h"

#include <math.h>

namespace nav {

void FusedOdometry::reset(const math::Pose& p) {
    pose = p;
    v = omega = ds = dth = 0.0f;
}

void FusedOdometry::update(int32_t dcL, int32_t dcR, float gyro, float dt) {
    if (!std::isfinite(dt) || dt <= 0) return;
    const float mmPerCount = 2.0f * math::PI * p_.wheelRadius / p_.countsPerRev;
    float dl = dcL * mmPerCount;
    float dr = dcR * mmPerCount;
    float wGyro = gyro;
    float wEnc = (dr - dl) / p_.track / dt;
    float w = fabsf(wEnc - wGyro) > p_.slipRate ? wGyro : p_.gyroWeight * wGyro + (1.0f - p_.gyroWeight) * wEnc;
    ds = 0.5f * (dl + dr);
    float mid = pose.th + 0.5f * w * dt;
    pose.x += ds * cosf(mid);
    pose.y += ds * sinf(mid);
    pose.th += w * dt;
    v = ds / dt;
    omega = w;
    dth = w * dt;
}

void FusedOdometry::updateEncoders(int32_t l, int32_t r, float dt) {
    if (!std::isfinite(dt) || dt <= 0) return;
    float rate = (r - l) * (2.0f * math::PI * p_.wheelRadius / p_.countsPerRev) / p_.track / dt;
    update(l, r, rate, dt);
}

float GyroBiasTracker::update(float rate, bool stationary, float dt) {
    if (!stationary || !std::isfinite(rate) || fabsf(rate) > p_.maxRate || !std::isfinite(dt) || dt <= 0) {
        stillS_ = 0;
        return 0.0f;
    }
    stillS_ += dt;
    if (stillS_ < p_.stillS) return 0.0f;
    float gain = dt / p_.timeConstantS;
    return (gain > 1.0f ? 1.0f : gain) * rate;
}

}  // namespace nav
