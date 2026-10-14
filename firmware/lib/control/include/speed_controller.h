/**
 * @file    speed_controller.h
 * @brief   Wheel speed controller: a feed-forward from the target speed plus a PID on the speed error. Pure logic, no
 *          hardware, so it runs in the host tests.
 *
 * @par Behaviour
 *   - duty = target / vFull, plus the PID output when the speed loop is on.
 *   - With a target of about zero (|target| < 1 cm/s) only the feed-forward is used and the PID is reset, so the
 *     integral does not wind up while the wheel is meant to stand still.
 */
#pragma once

#include "pid.h"

namespace nav {

class SpeedController {
public:
    // vFull: wheel speed at full duty (cm/s). useSpeedLoop = false gives open-loop (feed-forward only) control.
    SpeedController(float vFull, bool useSpeedLoop, const Pid& pid)
        : vFull_(vFull), useSpeedLoop_(useSpeedLoop), pid_(pid) {}

    void reset() { pid_.reset(); }

    // Target and measured speed in cm/s, dt in s. Returns a duty (not clamped; the motor driver clamps).
    float update(float target, float measured, float dt);

private:
    float vFull_;
    bool useSpeedLoop_;
    Pid pid_;
};

}  // namespace nav
