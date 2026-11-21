/**
 * @file    navigator.h
 * @brief   Path following with unmapped-block detection and replanning. Port of AStarDriver in sim/course/planner.py. Uses
 *          only what the car has: its pose estimate, the configured ToF readings, the stored map and its own speed.
 */
#pragma once

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include "geometry.h"
#include "planner.h"
#include "tof_layout.h"

namespace nav {

struct NavParams {
    float vFull = 622.0f;       // mm/s at full duty; the firmware passes cfg::V_FULL_MM_S
    float track = 90.0f;        // mm, wheel separation; the firmware passes cfg::WHEEL_TRACK_MM
    float speed = 0.88f;       // fraction of vFull on a straight, clear path
    float lookahead = 100.0f;
    float lookaheadGain = 0.15f;  // mm of extra lookahead per mm/s of speed: a longer look at speed damps the steering wobble
    float caution = 0.6f;      // speed factor where the unmapped random blocks can be (x > 1050 mm)
    float unmappedMinX = 1000.0f;  // detections left of this are never new blocks: that part of the arena is fully mapped,
                                   // so a short reading there is pose or mount error, and a false block can close the route
    // Random (unmapped) blocks: how many can exist, and where. A new block is accepted only if its estimated centre is
    // inside [unmappedMinX, unmappedMaxX] in x and above unmappedMinY, and only while fewer than maxExtras are known.
    float unmappedMaxX = 1.0e9f;
    float unmappedMinY = -1.0e9f;
    int maxExtras = MAX_EXTRA;
    // A short reading is ignored if its hit point is closer than this to a wall / to a mapped block: pose and map error make
    // those read short. The data-driven background mask (setBackground) is the better filter, so keep these small.
    // Only blocks that matter: a short reading counts only if its hit point is within pathReach of the planned path in the next
    // pathAhead mm. A block off the path costs a replan and gains nothing. 1e9 = no such test.
    float pathReach = 1.0e9f;
    float pathAhead = 700.0f;
    float rangeTol = 0.08f;    // a reading is "short" if it is more than 40 mm + rangeTol * (predicted range) below the map prediction
    int confirmSteps = 2;      // consecutive control steps (33 ms each) a reading must stay short before it counts as a block
    float wallMargin = 150.0f;
    float mapMargin = 150.0f;
    bool softProfile = false;  // use sqrt of the path's speed factor: slows down less in tight gaps and on curves
    float finishY = -1.0e9f;      // below this y (the run in to the finish line) the speed never drops under finishSpeed
    float finishSpeed = 0.4f;     // fraction of vFull
    float alertSpeed = 0.5f;   // speed factor for a moment after a new unmapped block shows up
};

class Navigator {
public:
    explicit Navigator(const tof::Layout& layout, const NavParams& p = NavParams())
        : prm_(p), layout_(layout) {}

    // map: rows of (x, y, angle) for the static blocks mapped in run 1.
    void init(const float (*map)[3], int n);

    // Plan from the start pose. Call while the car is locked on the line (the clock has not started).
    bool start(const math::Pose& est, float t);

    // The car was lifted and put back: plan again from where it is.
    bool reposition(const math::Pose& est, float t);
    bool configurationValid() const;
    bool hasRoute() const { return path_.n > 1; }

    // One control step. tof: layout.count distances in mm. duty: wheel commands in [-1, 1] (left, right).
    void step(const math::Pose& est, float speed, const float tof[], float t, float duty[2]);

    // Pose correction against the map. Nudges `est` (in place) when the short ToF readings explain a mapped block better
    // from a slightly different pose: a hill-climb over +-20 mm / +-2 deg steps, at most every 100 ms. Readings that
    // the map cannot explain (unmapped blocks, floor) are ignored. True when the pose was changed. The total
    // correction is bounded, so a bad run of readings cannot drag the pose far.
    bool localize(math::Pose& est, const float tof[], float t);

    // Steering angle to the lookahead target (rad) from the last track() call, for the run log.
    float lastAlpha() const { return lastAlpha_; }
    // A step at time t will replan (a block was just found): the planner blocks the loop, so hold the motor lease first.
    bool replanDue(float t) const { return needReplan_ && t - lastPlanT_ > 0.2f; }

    // Baseline of what the ToF sensors read from each pose on a clean run (tools/make_baseline.py -> include/baseline.h). Where the
    // baseline covers the current pose, a reading is "short" when it is baseMargin mm below the shortest clean-run reading there:
    // static clutter (pit corner, T14's real place, walls) is in the baseline, so only something new on the path stands out.
    struct BaselineEntry { int16_t cx, cy; int8_t bin; uint8_t r[3]; };
    struct Baseline {
        const BaselineEntry* e = nullptr;
        int n = 0;
        float cell = 30.0f, bin = 0.15f;
        float margin = 100.0f;  // mm
    };
    void setBaseline(const Baseline& b) { base_ = b; }
    // Shortest clean-run range per sensor near pose e (x, y within one cell, heading within one bin). False if nothing is near.
    bool baselineAt(const math::Pose& e, float out[3]) const {
        if (!base_.e) return false;
        int cx = (int)lroundf(e.x / base_.cell), cy = (int)lroundf(e.y / base_.cell), b = (int)lroundf(e.th / base_.bin);
        int bins = (int)lroundf(6.2831853f / base_.bin);
        int found = 0;
        for (int k = 0; k < 3; ++k) out[k] = 1.0e9f;
        for (int i = 0; i < base_.n; ++i) {
            const BaselineEntry& q = base_.e[i];
            int db = q.bin - b;
            while (db > bins / 2) db -= bins;
            while (db < -bins / 2) db += bins;
            if (abs(q.cx - cx) > 1 || abs(q.cy - cy) > 1 || abs(db) > 1) continue;
            ++found;
            for (int k = 0; k < 3; ++k) { float v = q.r[k] * 5.0f; if (v < out[k]) out[k] = v; }
        }
        return found >= 2;
    }

    // Is p within `reach` of the planned path between the car and `ahead` mm further on?
    bool nearPathAhead(math::Vec2 p, float reach, float ahead) const {
        if (path_.n < 2) return false;
        float s0 = path_.s[idx_];
        for (int i = idx_; i < path_.n && path_.s[i] <= s0 + ahead; ++i)
            if (math::distance(path_.p[i], p) <= reach) return true;
        return false;
    }

    // Places where the sensors return hits on a run with no random block that the map does not explain (the pit's corners and
    // walls, static things off the map). A reading whose hit point is on a marked cell is background, never a random block.
    // Generated by tools/make_background.py into include/background.h; null = no background.
    struct Background {
        const uint8_t* bits = nullptr;
        float x0 = 0, y0 = 0, cell = 40;
        int nx = 0, ny = 0;
    };
    void setBackground(const Background& b) { bg_ = b; }
    bool isBackground(math::Vec2 p) const {
        if (!bg_.bits) return false;
        int ix = (int)floorf((p.x - bg_.x0) / bg_.cell), iy = (int)floorf((p.y - bg_.y0) / bg_.cell);
        if (ix < 0 || iy < 0 || ix >= bg_.nx || iy >= bg_.ny) return false;
        int i = iy * bg_.nx + ix;
        return (bg_.bits[i >> 3] >> (i & 7)) & 1;
    }
    // Consecutive control steps sensor k has read shorter than the map predicts (for tuning and tests).
    int unexplainedStreak(int k) const { return streak_[k]; }

    float progress() const { return path_.n > 0 ? path_.s[idx_] / path_.s[path_.n - 1] : 0.0f; }
    const Path& path() const { return path_; }
    const Planner& planner() const { return planner_; }
    int replans() const { return replans_; }
    int unmappedBlocks() const { return planner_.numExtras(); }

private:
    bool replan(math::Vec2 from, float t);
    void scan(const math::Pose& e, const float tof[], float t);
    void track(const math::Pose& e, float speed, float t, float duty[2]);

    float locCost(const math::Pose& e, const float tof[], const bool use[], int* used) const;

    NavParams prm_;
    tof::Layout layout_;
    Planner planner_;
    Path path_;
    Background bg_;
    Baseline base_;
    int idx_ = 0;
    float lastAlpha_ = 0.0f;
    uint8_t streak_[tof::MAX_SENSORS] = {0};
    uint8_t hits_[MAX_EXTRA] = {0};
    float lastPlanT_ = 0.0f;
    float alertUntil_ = -1.0f;
    bool needReplan_ = false;
    int replans_ = 0;
    float lastLocT_ = -1.0f;
    math::Vec2 locOffset_ = {0, 0};  // total x/y correction applied so far
    float locTurn_ = 0.0f;           // total heading correction applied so far
};

}  // namespace nav
