/**
 * @file    odometry.cpp
 * @brief   Fused odometry implementation. The gyro bias is estimated while stationary; the encoders are ignored when
 *          they disagree with the gyro by more than SLIP_RATE (wheel slip).
 */
#include "odometry.h"

#include <math.h>

namespace nav {

void FusedOdometry::reset(const math::Pose& p, float gyroBias) {
    pose = p;
    bias = gyroBias;
    v = omega = ds = dth = 0.0f;
}

void FusedOdometry::update(int32_t dcL, int32_t dcR, float gyro, float dt, bool calibrating) {
    const float cmPerCount = 2.0f * math::PI * p_.wheelRadius / p_.countsPerRev;
    float dl = dcL * cmPerCount;
    float dr = dcR * cmPerCount;
    if (calibrating || (dcL == 0 && dcR == 0)) bias += 0.05f * (gyro - bias);
    float wGyro = gyro - bias;
    float wEnc = (dr - dl) / p_.track / dt;
    float w = fabsf(wEnc - wGyro) > SLIP_RATE ? wGyro : GYRO_WEIGHT * wGyro + (1.0f - GYRO_WEIGHT) * wEnc;
    ds = 0.5f * (dl + dr);
    float mid = pose.th + 0.5f * w * dt;
    pose.x += ds * cosf(mid);
    pose.y += ds * sinf(mid);
    pose.th += w * dt;
    v = ds / dt;
    omega = w;
    dth = w * dt;
}

}  // namespace nav
