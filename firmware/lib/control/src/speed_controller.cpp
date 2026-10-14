/**
 * @file    speed_controller.cpp
 * @brief   Speed controller implementation. See speed_controller.h.
 */
#include "speed_controller.h"

namespace nav {

namespace {
constexpr float TARGET_DEADBAND = 1.0f;  // cm/s: below this the wheel is meant to stand still
}

float SpeedController::update(float target, float measured, float dt) {
    float feedForward = target / vFull_;
    if (!useSpeedLoop_) return feedForward;
    if (target > -TARGET_DEADBAND && target < TARGET_DEADBAND) {
        pid_.reset();
        return feedForward;
    }
    return feedForward + pid_.update(target, measured, dt);
}

}  // namespace nav
