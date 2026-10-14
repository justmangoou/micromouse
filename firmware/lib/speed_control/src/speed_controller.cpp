/**
 * @file    speed_controller.cpp
 * @brief   Speed controller implementation. See speed_controller.h.
 */
#include "speed_controller.h"

namespace speed_control {

float SpeedController::update_counts(int32_t counts) {
    const float target = target_;
    speed_ += alpha_ * ((counts * mmPerCount_ / dt_) - speed_);
    float feedForward = target / vFull_;

    if (target > -targetDeadband_ && target < targetDeadband_) {
        pid_.reset();
        return feedForward;
    }

    return feedForward + pid_.update(target, speed_, dt_);
}

}  // namespace speed_control
