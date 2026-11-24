/**
 * @file    speed_controller.h
 * @brief   Filtered encoder speed and feed-forward plus PID wheel control. Pure logic, no
 *          hardware, so it runs in the host tests.
 *
 * @par Behaviour
 *   - duty = target / vFull + ffOffset (signed like the target), plus the PID output. vFull is the motor's measured speed at
 *     full duty and ffOffset the duty at which the wheel just starts to turn (src/test_step.cpp measures both).
 *   - With |target| below targetDeadband (default 10 mm/s), only feed-forward is used and the PID is reset, so the
 *     integral does not wind up while the wheel is meant to stand still.
 */
#pragma once

#include <stdint.h>

#include "pid.h"

namespace speed_control {

// Signed travel between 16-bit counter reads, valid for fewer than 32768 counts between reads.
inline int32_t counterDelta(uint16_t now, uint16_t last) {
    return static_cast<int16_t>(static_cast<uint16_t>(now - last));
}

class SpeedController {
public:
    // vFull: full-duty speed (mm/s). mmPerCount: encoder travel. dt: fixed period (s).
    // targetDeadband: PID bypass threshold (mm/s). alpha: newest speed sample filter weight.
    // window: ticks (1..MAX_WINDOW) the speed is averaged over before the filter. One encoder count is
    // mmPerCount/dt of speed (~77 mm/s at 200 Hz on this car), so a window of 1 hands the PID a speed that jumps by
    // that much every tick; a window of 4 cuts the step to a quarter.
    static constexpr int MAX_WINDOW = 8;
    SpeedController(float vFull, float mmPerCount, float dt, const Pid& pid,
                    float targetDeadband = 10.0f, float alpha = 0.4f, int window = 1, float ffOffset = 0.0f)
        : vFull_(vFull), pid_(pid), mmPerCount_(mmPerCount), dt_(dt), alpha_(alpha),
          targetDeadband_(targetDeadband), ffOffset_(ffOffset), window_(window < 1 ? 1 : window > MAX_WINDOW ? MAX_WINDOW : window) {}

    // Store the navigation target in mm/s; consumed on the next control tick.
    void update_target(float mmPerSec) { target_ = mmPerSec; }

    // Signed encoder counts since the last fixed-period tick. Returns unclamped duty.
    float update_counts(int32_t counts) {
        const float target = target_;
        sum_ += counts - hist_[pos_];
        hist_[pos_] = counts;
        pos_ = (pos_ + 1) % window_;
        speed_ += alpha_ * ((sum_ * mmPerCount_ / (window_ * dt_)) - speed_);
        float feedForward = target / vFull_;

        if (target > -targetDeadband_ && target < targetDeadband_) {
            pid_.reset();
            return feedForward;
        }

        feedForward += target > 0 ? ffOffset_ : -ffOffset_;
        return feedForward + pid_.update(target, speed_, dt_);
    }

    float speed() const { return speed_; }

    // Clear the command and PID history, preserving measured speed while coasting.
    // Caller must exclude control ticks for stop() and reset().
    void stop() {
        target_ = 0;
        pid_.reset();
    }

    // Full initialization, including the speed estimate.
    void reset() {
        stop();
        speed_ = 0;
        for (int& h : hist_) h = 0;
        sum_ = 0;
        pos_ = 0;
    }

private:
    float vFull_;
    Pid pid_;
    float mmPerCount_, dt_, alpha_;
    float targetDeadband_;
    float ffOffset_;
    int window_;
    int hist_[MAX_WINDOW] = {};
    int32_t sum_ = 0;
    int pos_ = 0;
    float speed_ = 0;
    volatile float target_ = 0;  // main-loop writes, control-interrupt reads
};

}  // namespace speed_control
