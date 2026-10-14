/**
 * @file    speed_controller.h
 * @brief   Filtered encoder speed and feed-forward plus PID wheel control. Pure logic, no
 *          hardware, so it runs in the host tests.
 *
 * @par Behaviour
 *   - duty = target / vFull, plus the PID output.
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
    SpeedController(float vFull, float mmPerCount, float dt, const Pid& pid,
                    float targetDeadband = 10.0f, float alpha = 0.4f)
        : vFull_(vFull), pid_(pid), mmPerCount_(mmPerCount), dt_(dt), alpha_(alpha),
          targetDeadband_(targetDeadband) {}

    // Store the navigation target in mm/s; consumed on the next control tick.
    void update_target(float mmPerSec) { target_ = mmPerSec; }

    // Signed encoder counts since the last fixed-period tick. Returns unclamped duty.
    float update_counts(int32_t counts);

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
    }

private:
    float vFull_;
    Pid pid_;
    float mmPerCount_, dt_, alpha_;
    float targetDeadband_;
    float speed_ = 0;
    volatile float target_ = 0;  // main-loop writes, control-interrupt reads
};

}  // namespace speed_control
