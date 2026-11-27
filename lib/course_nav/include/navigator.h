/**
 * @file    navigator.h
 * @brief   Drives the planned route and reacts to a block that is not on the map.
 *
 * Every control step (30 Hz) the Navigator
 *   1. checks the ToF readings for a block that is not on the map (detectBlocks),
 *   2. plans a new route around it if one was found (A*, see planner.h),
 *   3. follows the route with pure pursuit and returns the two wheel commands (track).
 *
 * It uses only what the car has: its pose estimate, the ToF readings, the stored map and its own speed.
 *
 * Random-block detection, in one paragraph: a reading is "short" when it is well below what the sensor should see from
 * here. Where the car has a reference (the log of a run without the block, see Baseline) that is what it read near this
 * pose, so static things the map lacks cancel out; elsewhere it is the map (blocks + walls). A sensor that stays short
 * for a few steps, and whose block would stand on the route ahead, places a block there and the route is planned again.
 */
#pragma once

#include <stdint.h>

#include "geometry.h"
#include "planner.h"
#include "tof_layout.h"

namespace nav {

// Which short readings count as a new block. Defaults accept everything; the course sets the real limits.
struct DetectParams {
    int maxBlocks = MAX_EXTRA;     // how many random blocks the course can have
    float minHitX = 1000.0f;       // hits left of this are ignored: that part of the arena is fully mapped
    float maxCentreX = 1.0e9f;     // a block centre beyond this is outside the arena
    float minCentreY = -1.0e9f;    // a block centre below this is on the finish tape
    float pathReach = 1.0e9f;      // the hit must be this close to the route ahead (1e9 = anywhere)
    float pathAhead = 700.0f;      // ... looking this far along the route, mm
    float centreReach = 1.0e9f;    // the block's estimated centre must be this close to the route ahead
    float minCentreX = -1.0e9f;    // a centre estimate left of this is not a block (it is in the forbidden middle) ...
    float clampCentreX = -1.0e9f;  // ... and an accepted one is moved right to at least this: no block can be further left
    int confirmSteps = 2;          // control steps a reading must stay short
    // Against the map (no baseline here):
    float rangeTol = 0.08f;        // short = more than 40 mm + rangeTol * predicted range below the prediction
    float wallMargin = 150.0f;     // hits this close to a wall are ignored (pose error makes walls read short)
    float mapMargin = 150.0f;      // hits this close to a mapped block are ignored (same reason)
    // Against the baseline:
    float baselineMargin = 100.0f; // short = more than this below the reference run's reading, mm
};

struct NavParams {
    float vFull = 622.0f;          // speed scale: the wheel commands are fractions of this, mm/s
    float track = 90.0f;           // wheel separation, mm
    float speed = 0.88f;           // cruise speed on a clear straight, fraction of vFull
    float lookahead = 100.0f;      // pure-pursuit lookahead at standstill, mm
    float lookaheadGain = 0.15f;   // extra lookahead per mm/s of speed (longer at speed = less weaving)
    bool softProfile = false;      // use sqrt of the route's speed factor (slows less in gaps and bends)
    float caution = 0.6f;          // speed factor right of cautionX, while the block may be ahead
    float alertSpeed = 0.5f;       // speed factor for 0.8 s after a new block is found
    float finishY = -1.0e9f;       // below this y the car never drives slower than finishSpeed
    float finishSpeed = 0.4f;      // fraction of vFull
    // Trust: once the one random block is confirmed and behind the car, nothing unknown is left, so the right wing's
    // caution is relaxed. "Behind" assumes the right wing is driven toward -y. 0 support = never trust.
    int trustSupport = 0;          // short readings that must agree on the block
    float trustPassMm = 150.0f;    // the car must be this far past the block's centre
    float trustedCaution = 1.0f;   // replaces `caution` from then on
    // The finish lane between T16 and T17 (a dodge ends there) is only 200 mm wide and the estimate drifts. There a side
    // sensor reading closer than laneClose moves the estimate sideways, so the car steers away. 0 = off.
    float laneClose = 0.0f;        // mm
    float laneStep = 3.0f;         // mm per control step
    float laneLimit = 80.0f;       // total correction per run, mm
    float cautionX = 1050.0f;      // `caution` applies right of this, mm
    // A new block closer than this (car centre to block centre) is too near to turn around beside it: brake and back
    // off for backSteps control steps before planning. 0 = off.
    float backRange = 0.0f;
    int backSteps = 10;
    float backDuty = 0.45f;
    DetectParams detect;
};

// The log of a run without the random block, as the run record stores it in flash (src/run_record.cpp): one record per
// control step. Little endian: int16 x at byte 2, y at 4 (mm), heading at 6 (mrad); the three ranges in 5 mm units from
// byte 13.
struct Baseline {
    static constexpr int RECORD_BYTES = 16;
    const uint8_t* records = nullptr;
    int count = 0;
};

class Navigator {
public:
    explicit Navigator(const tof::Layout& layout, const NavParams& params = NavParams())
        : params_(params), layout_(layout) {}

    // map: rows of (x, y, angle) of the blocks known before the run.
    void init(const float (*map)[3], int n);
    void setBaseline(const Baseline& baseline) { baseline_ = baseline; }
    void setParams(const NavParams& params) { params_ = params; }  // between runs, before start()
    bool configurationValid() const;

    // Plan from the start pose (before the clock starts). Forgets blocks found on an earlier run.
    bool start(const math::Pose& pose, float t);
    // The car was lifted and put back: plan again from where it is.
    bool reposition(const math::Pose& pose, float t);

    // One control step. tof: one range per sensor, mm. speed: the car's speed, mm/s. duty: wheel commands in [-1, 1]
    // (left, right), zero when there is no route.
    void step(const math::Pose& pose, float speed, const float tof[], float t, float duty[2]);

    // Pose correction against the mapped blocks (currently OFF on the car, see docs/DECISIONS.md). Nudges `pose` when the
    // short ToF readings fit the map better from a slightly different pose: a hill-climb over +-20 mm / +-2 deg steps,
    // at most every 100 ms, with a bounded total correction. True when the pose was changed.
    bool localize(math::Pose& pose, const float tof[], float t);

    bool hasRoute() const { return path_.n > 1; }
    // True when the next step will plan again (a block was just found). Planning blocks the control loop for a while.
    bool replanDue(float t) const { return needReplan_ && t - lastPlanT_ > REPLAN_SPACING_S; }

    // Side-sensor correction in the finish lane (NavParams::laneClose). Call before step(); true when `pose` was moved.
    bool keepLane(math::Pose& pose, const float tof[]);

    // For the run log, tools and tests:
    float progress() const { return path_.n > 0 ? path_.s[idx_] / path_.s[path_.n - 1] : 0.0f; }
    float lastAlpha() const { return lastAlpha_; }        // steering angle to the lookahead point, rad
    int shortStreak(int sensor) const { return streak_[sensor]; }  // steps this sensor has read short
    const Path& path() const { return path_; }
    const Planner& planner() const { return planner_; }
    int replans() const { return replans_; }
    int unmappedBlocks() const { return planner_.numExtras(); }
    int blockSupport() const { return support_; }  // short readings that agreed with the first found block
    bool blockPassed() const { return passed_; }   // trusted and behind the car: driving with trustedCaution

private:
    static constexpr float REPLAN_SPACING_S = 0.2f;  // plan at most this often for detected blocks

    bool replan(math::Vec2 from, float t);

    // Random-block detection
    void detectBlocks(const math::Pose& pose, const float tof[], float t);
    float predictedRange(const math::Pose& pose, int sensor) const;
    bool baselineAt(const math::Pose& pose, float shortest[3]) const;
    bool isNewBlockHit(math::Vec2 hit, bool baselineCovered) const;
    float distanceToRoute(math::Vec2 p) const;  // to the route ahead, DetectParams::pathAhead along it
    bool nearRouteAhead(math::Vec2 p, float reach) const { return distanceToRoute(p) <= reach; }
    void addOrMergeBlock(math::Vec2 centre, float t);
    void updateTrust(const math::Pose& pose);

    // Route following
    void track(const math::Pose& pose, float speed, float t, float duty[2]);
    float targetSpeed(const math::Pose& pose, float alpha, float t) const;

    // Pose correction
    float localizeCost(const math::Pose& pose, const float tof[], const bool use[]) const;

    math::Vec2 sensorOrigin(const math::Pose& pose, int sensor) const;

    NavParams params_;
    tof::Layout layout_;
    Planner planner_;
    Path path_;
    Baseline baseline_;
    int idx_ = 0;                // path point nearest the car
    float lastAlpha_ = 0.0f;
    uint8_t streak_[tof::MAX_SENSORS] = {0};
    uint8_t hits_[MAX_EXTRA] = {0};  // detections merged into each found block
    uint16_t support_ = 0;
    float laneShifted_ = 0.0f;
    int backing_ = 0;  // control steps of reversing still to do
    bool passed_ = false;
    float lastPlanT_ = 0.0f;
    float alertUntil_ = -1.0f;
    bool needReplan_ = false;
    int replans_ = 0;
    float lastLocalizeT_ = -1.0f;
    math::Vec2 locOffset_ = {0, 0};  // total x/y correction applied so far
    float locTurn_ = 0.0f;           // total heading correction applied so far
};

}  // namespace nav
