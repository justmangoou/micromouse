#pragma once

#include "floodfill.h"
#include "math_util.h"
#include "tof_layout.h"

namespace nav {

struct MicromouseParams {
    int leftSensor = 0, rightSensor = 1;  // forward-facing sensors in the supplied layout
    int width = 16, height = 16;
    float cellSize = 180.0f;       // mm; start at southwest cell center, facing north
    float track = 90.0f;
    float speed = 120.0f;          // mm/s
    float minSpeed = 40.0f;
    float maxTurnRate = 1.5f;     // rad/s
    float headingGain = 3.0f;
    float crossTrackGain = 0.03f;  // rad/s per mm of lateral error
    float positionTolerance = 8.0f;
    float headingTolerance = 0.05f;
    float maxCrossTrack = 30.0f;
    float wallTolerance = 10.0f;   // wall plane relative to cell boundary
    float openMargin = 20.0f;
    float stopClearance = 30.0f;   // forward clearance from inner sensor mounts
    float settleTime = 0.2f;
    float observationTimeout = 1.0f;
    float moveTimeout = 6.0f;
    float runTimeout = 240.0f;
};

// Hardware-free maze exploration and cell motion. Commands are wheel speeds
// in mm/s; pose comes from calibrated encoder/gyro odometry. Inspect an unknown
// edge head-on before crossing it; never infer a side wall from an oblique ray.
class Micromouse {
public:
    enum class State { Observing, Turning, Advancing, Settling, Done, Fault };
    explicit Micromouse(const tof::Layout& layout, const MicromouseParams& p = {})
        : prm_(p), layout_(layout) {}
    math::Pose startPose() const { return {prm_.cellSize / 2, prm_.cellSize / 2, math::PI / 2}; }
    bool start(float t);
    // ranges and reliable each contain layout.count entries in layout order.
    void step(const math::Pose& pose, const float ranges[],
              const bool reliable[], float t, float wheelSpeed[2]);
    void pause(float t);
    bool resume(float t);
    bool paused() const { return paused_; }
    void stop() { state_ = State::Fault; }
    State state() const { return state_; }
    Cell cell() const { return cell_; }
    Direction heading() const { return heading_; }
    const FloodFill& maze() const { return maze_; }

private:
    static float angle(Direction direction);
    void enter(State state, float t) { state_ = state; phaseStart_ = t; }
    Passage observe(const float ranges[]) const;
    MicromouseParams prm_;
    tof::Layout layout_;
    FloodFill maze_;
    State state_ = State::Done;
    Cell cell_{0, 0}, next_{0, 0};
    Direction heading_ = Direction::North;
    bool arrived_ = false;
    bool paused_ = false, alignOnResume_ = false;
    float pausedAt_ = 0;
    float phaseStart_ = 0, runStart_ = 0;
};

}  // namespace nav
