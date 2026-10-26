#include "micromouse.h"

#include <algorithm>
#include <cmath>

namespace nav {

float Micromouse::angle(Direction direction) {
    return math::PI / 2 - static_cast<int>(direction) * math::PI / 2;
}

bool Micromouse::start(float t) {
    if (!layout_.valid() || prm_.leftSensor < 0 || prm_.leftSensor >= layout_.count ||
        prm_.rightSensor < 0 || prm_.rightSensor >= layout_.count || prm_.leftSensor == prm_.rightSensor) {
        stop();
        return false;
    }
    for (int k : {prm_.leftSensor, prm_.rightSensor}) {
        if (cosf(layout_.sensors[k].angle) <= 0 ||
            prm_.cellSize / 2 < layout_.sensors[k].x + prm_.stopClearance) {
            stop();
            return false;
        }
    }
    for (float value : {t, prm_.cellSize, prm_.track, prm_.speed, prm_.minSpeed, prm_.maxTurnRate,
                        prm_.headingGain, prm_.crossTrackGain, prm_.positionTolerance, prm_.headingTolerance,
                        prm_.maxCrossTrack, prm_.wallTolerance, prm_.openMargin, prm_.stopClearance,
                        prm_.settleTime, prm_.observationTimeout, prm_.moveTimeout, prm_.runTimeout}) {
        if (!std::isfinite(value)) { stop(); return false; }
    }
    if (!maze_.reset(prm_.width, prm_.height) || !std::isfinite(prm_.cellSize) ||
        prm_.track <= 0 || prm_.minSpeed <= 0 || prm_.speed < prm_.minSpeed ||
        prm_.maxTurnRate <= 0 || prm_.headingGain <= 0 || prm_.crossTrackGain < 0 ||
        prm_.wallTolerance < 0 || prm_.openMargin <= prm_.wallTolerance || prm_.stopClearance < 0 ||
        prm_.maxCrossTrack <= prm_.positionTolerance || prm_.positionTolerance <= 0 ||
        prm_.positionTolerance >= prm_.cellSize / 2 || prm_.headingTolerance <= 0 ||
        prm_.settleTime < 0 || prm_.observationTimeout <= 0 || prm_.moveTimeout <= 0 || prm_.runTimeout <= 0) {
        stop();
        return false;
    }
    cell_ = next_ = {0, 0};
    heading_ = Direction::North;
    arrived_ = paused_ = alignOnResume_ = false;
    runStart_ = t;
    enter(State::Observing, t);
    return true;
}

void Micromouse::pause(float t) {
    if (!paused_ && state_ != State::Done && state_ != State::Fault) {
        paused_ = true;
        pausedAt_ = t;
    }
}

bool Micromouse::resume(float t) {
    if (!paused_ || !std::isfinite(t) || t < pausedAt_ || t - runStart_ > prm_.runTimeout) return false;
    phaseStart_ += t - pausedAt_;
    paused_ = false;
    alignOnResume_ = true;
    return true;
}

Passage Micromouse::observe(const float ranges[]) const {
    // Project each inner beam's hit onto the forward axis of a cardinal pose.
    // Both must agree; ambiguous measurements leave the passage unknown.
    float left = layout_.sensors[prm_.leftSensor].x + ranges[prm_.leftSensor] * cosf(layout_.sensors[prm_.leftSensor].angle);
    float right = layout_.sensors[prm_.rightSensor].x + ranges[prm_.rightSensor] * cosf(layout_.sensors[prm_.rightSensor].angle);
    float boundary = prm_.cellSize / 2;
    if (left <= boundary + prm_.wallTolerance && right <= boundary + prm_.wallTolerance) return Passage::Wall;
    if (left >= boundary + prm_.openMargin && right >= boundary + prm_.openMargin) return Passage::Open;
    return Passage::Unknown;
}

void Micromouse::step(const math::Pose& pose, const float ranges[],
                      const bool reliable[], float t, float wheelSpeed[2]) {
    wheelSpeed[0] = wheelSpeed[1] = 0;
    if (state_ == State::Done || state_ == State::Fault || paused_) return;
    if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.th) ||
        !std::isfinite(t) || t < runStart_ || t - runStart_ > prm_.runTimeout ||
        t - phaseStart_ > (state_ == State::Observing ? prm_.observationTimeout : prm_.moveTimeout)) {
        stop();
        return;
    }
    float th = angle(heading_);
    float error = math::wrap(th - pose.th);
    if (alignOnResume_) {
        if (fabsf(error) > prm_.headingTolerance) {
            float rate = std::clamp(prm_.headingGain * error, -prm_.maxTurnRate, prm_.maxTurnRate);
            wheelSpeed[0] = -rate * prm_.track / 2;
            wheelSpeed[1] = rate * prm_.track / 2;
            return;
        }
        alignOnResume_ = false;
    }
    if (state_ == State::Settling) {
        if (t - phaseStart_ < prm_.settleTime) return;
        if (arrived_) {
            cell_ = next_;
            arrived_ = false;
            if (maze_.isGoal(cell_)) {
                enter(State::Done, t);
                return;
            }
        }
        enter(State::Observing, t);
        return;
    }
    if (state_ == State::Turning) {
        if (fabsf(error) <= prm_.headingTolerance) {
            enter(State::Settling, t);
            return;
        }
        float rate = std::clamp(prm_.headingGain * error, -prm_.maxTurnRate, prm_.maxTurnRate);
        wheelSpeed[0] = -rate * prm_.track / 2;
        wheelSpeed[1] = rate * prm_.track / 2;
        return;
    }
    // Invalid/no-target/stale ranges must not mark an edge open or permit motion.
    for (int k : {prm_.leftSensor, prm_.rightSensor}) {
        if (!reliable[k] || !std::isfinite(ranges[k]) || ranges[k] < layout_.minMm || ranges[k] >= layout_.maxMm) {
            if (state_ == State::Advancing) stop();
            return;  // while observing, wait for a measurement until timeout
        }
    }
    if (state_ == State::Observing) {
        if (maze_.isGoal(cell_)) { enter(State::Done, t); return; }
        if (fabsf(error) > prm_.headingTolerance) { enter(State::Turning, t); return; }
        Passage passage = observe(ranges);
        if (passage == Passage::Unknown) return;
        maze_.setPassage(cell_, heading_, passage);
        Direction direction;
        if (!maze_.nextMove(cell_, heading_, next_, direction)) { stop(); return; }
        if (direction != heading_) {
            heading_ = direction;
            enter(State::Turning, t);
        } else if (maze_.passage(cell_, direction) == Passage::Open) {
            enter(State::Advancing, t);
        }
        return;
    }

    float dx = pose.x - (next_.x + 0.5f) * prm_.cellSize;
    float dy = pose.y - (next_.y + 0.5f) * prm_.cellSize;
    float remaining = -(dx * cosf(th) + dy * sinf(th));
    float cross = -dx * sinf(th) + dy * cosf(th);
    if (fabsf(cross) > prm_.maxCrossTrack || remaining < -prm_.positionTolerance) { stop(); return; }
    if (hypotf(dx, dy) <= prm_.positionTolerance) {
        maze_.setPassage(cell_, heading_, Passage::Open);
        arrived_ = true;
        enter(State::Settling, t);
        return;
    }
    float clearance = fminf(ranges[prm_.rightSensor] * cosf(layout_.sensors[prm_.rightSensor].angle), ranges[prm_.leftSensor] * cosf(layout_.sensors[prm_.leftSensor].angle));
    if (clearance <= prm_.stopClearance || fabsf(error) > 0.5f) { stop(); return; }
    float speed = std::clamp(2.0f * fmaxf(remaining, 0.0f), prm_.minSpeed, prm_.speed);
    float rate = std::clamp(prm_.headingGain * error - prm_.crossTrackGain * cross,
                           -prm_.maxTurnRate, prm_.maxTurnRate);
    wheelSpeed[0] = speed - rate * prm_.track / 2;
    wheelSpeed[1] = speed + rate * prm_.track / 2;
}

}  // namespace nav
