/**
 * @file    speed_estimator.h
 * @brief   Wheel speed from a hardware quadrature counter: a wrap-safe count delta per control period, converted to
 *          cm/s and smoothed by a first-order filter. Pure logic, no hardware, so it runs in the host tests.
 */
#pragma once

#include <stdint.h>

namespace control {

// Counts moved between two reads of a free-running 16-bit counter. Right as long as fewer than 32768 counts pass
// between the reads (the wheel makes ~2800 counts/s at full speed, so that is more than 11 s).
inline int32_t counterDelta(uint16_t now, uint16_t last) { return static_cast<int16_t>(static_cast<uint16_t>(now - last)); }

class SpeedEstimator {
public:
    // cmPerCount: wheel travel per encoder count. dt: control period in s. alpha: filter weight of the newest sample.
    SpeedEstimator(float cmPerCount, float dt, float alpha = 0.4f) : cmPerCount_(cmPerCount), dt_(dt), alpha_(alpha) {}

    // Counts since the last call (forward positive). Returns the filtered speed in cm/s.
    float update(int32_t counts) {
        speed_ += alpha_ * (counts * cmPerCount_ / dt_ - speed_);
        return speed_;
    }

    float speed() const { return speed_; }
    void reset() { speed_ = 0; }

private:
    float cmPerCount_, dt_, alpha_;
    float speed_ = 0;
};

}  // namespace control
