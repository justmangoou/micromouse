/**
 * @file    navigator.h
 * @brief   Path following with unmapped-block detection and replanning. Port of AStarDriver in sim/course/planner.py. Uses
 *          only what the car has: its pose estimate, the 4 ToF readings, the stored map and its own speed.
 */
#pragma once

#include "geometry.h"
#include "planner.h"
#include "tof_layout.h"

namespace nav {

struct NavParams {
    float vFull = 622.0f;       // mm/s at full duty; the firmware passes cfg::V_FULL_MM_S
    float track = 90.0f;        // mm, wheel separation; the firmware passes cfg::WHEEL_TRACK_MM
    float speed = 0.85f;       // fraction of vFull on a straight, clear path
    float lookahead = 100.0f;
    float caution = 0.6f;      // speed factor where the unmapped random blocks can be (x > 1050 mm)
    float alertSpeed = 0.5f;   // speed factor for a moment after a new unmapped block shows up
};

class Navigator {
public:
    explicit Navigator(const NavParams& p = NavParams()) : prm_(p) {}

    // map: rows of (x, y, angle) for the static blocks mapped in run 1.
    void init(const float (*map)[3], int n);

    // Plan from the start pose. Call while the car is locked on the line (the clock has not started).
    void start(const math::Pose& est, float t);

    // The car was lifted and put back: plan again from where it is.
    void reposition(const math::Pose& est, float t);

    // One control step. tof: distances in mm. duty: wheel commands in [-1, 1] (left, right).
    void step(const math::Pose& est, float speed, const float tof[tof::COUNT], float t, float duty[2]);

    float progress() const { return path_.n > 0 ? path_.s[idx_] / path_.s[path_.n - 1] : 0.0f; }
    const Path& path() const { return path_; }
    const Planner& planner() const { return planner_; }
    int replans() const { return replans_; }
    int unmappedBlocks() const { return planner_.numExtras(); }

private:
    void replan(math::Vec2 from, float t);
    void scan(const math::Pose& e, const float tof[tof::COUNT], float t);
    void track(const math::Pose& e, float speed, float t, float duty[2]);

    NavParams prm_;
    Planner planner_;
    Path path_;
    int idx_ = 0;
    uint8_t streak_[tof::COUNT] = {0};
    uint8_t hits_[MAX_EXTRA] = {0};
    float lastPlanT_ = 0.0f;
    float alertUntil_ = -1.0f;
    bool needReplan_ = false;
    int replans_ = 0;
};

}  // namespace nav
