/**
 * @file    planner.h
 * @brief   Global planner: clearance-weighted A* on a 20 mm grid, string pulling, corner smoothing, resampling.
 *          All memory is static (~36 KB, or ~10 KB with NAV_LOW_MEMORY), no allocation.
 */
#pragma once

#include <stdint.h>

#include "geometry.h"

namespace nav {

// Memory profile. The default is the full-resolution planner (~46 KB with the navigator's
// path). A target with 20 KB of RAM (the Blue Pill) builds with -DNAV_LOW_MEMORY: 40 mm cells, shorter path
// and open list, ~13 KB. Same algorithm, coarser grid; the string pulling and smoothing recover most of the resolution.
#ifdef NAV_LOW_MEMORY
constexpr float GRID_RES = 40.0f;     // grid cell, mm
constexpr int MAX_PATH = 200;         // PATH_STEP spacing -> 4000 mm
constexpr int MAX_COARSE = 128;
constexpr int HEAP_CAP = 200;
constexpr float PATH_STEP = 20.0f;
constexpr float CHECK_STEP = 20.0f;   // spacing of the clearance samples along a shortcut (blocks are 100 mm wide)
constexpr int PROFILE_STRIDE = 2;     // speed profile: clearance and edge terms at every 2nd path point, interpolated
#else
constexpr float GRID_RES = 20.0f;     // grid cell, mm
constexpr int MAX_PATH = 420;         // 15 mm spacing -> 6300 mm
constexpr int MAX_COARSE = 400;
// Open-list entries. The longest search in the host tests (unmapped blocks, replanning) peaks at 104, so this is
// ~10x headroom. When it fills, new entries are dropped, which can only cost path optimality.
constexpr int HEAP_CAP = 1024;
constexpr float PATH_STEP = 15.0f;
constexpr float CHECK_STEP = 10.0f;
constexpr int PROFILE_STRIDE = 1;
#endif
constexpr int GRID_NX = static_cast<int>(1800.0f / GRID_RES) + 1;  // x = 0 .. 1800 mm
constexpr int GRID_NY = static_cast<int>(1400.0f / GRID_RES) + 1;  // y = -200 .. 1200 mm
constexpr int GRID_CELLS = GRID_NX * GRID_NY;
constexpr int MAX_MAP = 24;       // mapped (static) blocks
constexpr int MAX_EXTRA = 8;      // unmapped blocks found by the ToF at run time

constexpr float R_ROBOT = 90.0f;   // min clearance from a block surface for the centre line
constexpr float R_PRUNE = 110.0f;   // shortcuts must keep this much clearance
// Stay this far inside the tape. A wheel sits track/2 + wheel_half_w = 50 mm off the centreline, so this must
// clear that plus *real* margin for dead-reckoning drift, not just a few mm. But the 600 mm-wide corridor (with
// R_ROBOT clearance from obstacles also eating into it) cannot support a static margin that covers the full drift
// budget -- above ~80 mm, the planner starts failing to find a route at all. So R_EDGE only buys back the wheel
// geometry plus a modest cushion; EDGE_SLOW_START (below) is what actually keeps the car off the tape when it has
// drifted further than that, by slowing it down near the boundary so pure pursuit has more time to correct.
constexpr float R_EDGE = 70.0f;
constexpr float EDGE_SLOW_START = 220.0f;  // mm of edge margin at which the speed factor starts dropping below 1
constexpr float DISC_R = 50.0f;     // an unmapped block is a disc around its estimated centre, its half-width. Larger
                                    // (it was 110) closed the 22 cm gap below T14 and sent the car round over the top
constexpr float CENTERING = 6.0f;

// The gap between T15 (above) and T16 (below), 200 mm. With the random block low in the first corridor of the right
// wing the route goes through it instead of back up and round T15: down to the gap, turn on the spot, straight through,
// round into the finish lane (Planner::gate, Navigator: GATE_*).
constexpr float GATE_Y = 300.0f;             // the gap's centre line
constexpr float GATE_MAX_BLOCK_Y = 150.0f;   // only for a block centred this low (it leaves the gap's mouth free)
constexpr float GATE_X0 = 1220.0f, GATE_X1 = 1330.0f;  // the car is in the corridor ...
constexpr float GATE_MIN_RUN = 50.0f;        // ... at least this far above the gap
constexpr float GATE_MAX_START_Y = 700.0f;
constexpr float GATE_EXIT_X = 1600.0f, GATE_EXIT_R = 100.0f;  // the turn into the lane: round T16's corner (1500, 200)
constexpr float GATE_CLEAR = 75.0f;           // every leg keeps this from the blocks (the gap leaves 100)
constexpr float GATE_FACE_X1 = 1500.0f;      // T16's right-hand face (its top face spans x 1400-1500 at y 200)
// true: stop above the gap and turn on the spot (Navigator measures there). false: curve into it without stopping.
constexpr bool GATE_TURN_ON_SPOT = true;
constexpr float GATE_ENTRY_R = 90.0f;        // the curve into the gap, at most

constexpr math::Vec2 GOAL = {1320.0f, -120.0f};  // only the straight-line fallback; A* ends anywhere on the finish line

struct Path {
    math::Vec2 p[MAX_PATH];
    float s[MAX_PATH];   // arc length
    float v[MAX_PATH];   // speed factor 0.35 .. 1 (curvature and clearance)
    int n = 0;
};

class Planner {
public:
    // Static map from run 1. Precomputes the clearance field (a few tens of ms).
    void setMap(const Block* blocks, int n);
    // Same, from rows of (x, y, angle): no temporary block array (RAM and stack are tight on small targets).
    void setMap(const float (*rows)[3], int n);

    void clearExtras() { nExtras_ = 0; }
    int numExtras() const { return nExtras_; }
    bool gated() const { return gated_; }  // the last plan goes through the T15-T16 gap
    int addExtra(math::Vec2 centre);  // -1 if full
    math::Vec2 extra(int i) const { return extras_[i]; }
    void setExtra(int i, math::Vec2 c) { extras_[i] = c; }

    // Plan from `start` to the finish line. False if no route exists even with the smallest margin.
    bool plan(math::Vec2 start, Path& out);
    // Fallback: straight line start -> goal.
    void straight(math::Vec2 start, Path& out);

    float clearance(math::Vec2 p) const;                   // blocks + unmapped discs
    float mapClearance(math::Vec2 p) const;                // mapped blocks only
    float clearanceUpTo(math::Vec2 p, float cap) const;    // exact below cap; any value >= cap above it. Skips far blocks
    bool clearAtLeast(math::Vec2 p, float rad) const;
    bool segmentFree(math::Vec2 a, math::Vec2 b, float rad) const;

    int numBlocks() const { return nBlocks_; }
    const Block& block(int i) const { return blocks_[i]; }

private:
    void buildBase();
    int astar(math::Vec2 start, float radius);
    void buildWork();
    void finish(Path& out, int nCoarse);
    bool gate(math::Vec2 start, Path& out);

    Block blocks_[MAX_MAP];
    int nBlocks_ = 0;
    math::Vec2 extras_[MAX_EXTRA];
    int nExtras_ = 0;
    bool gated_ = false;

    uint8_t base_[GRID_CELLS];   // static clearance, 2.5 mm units (0 off the track)
    uint8_t work_[GRID_CELLS];   // base_ with the unmapped discs applied
    uint8_t mult8_[256];     // clearance -> cost multiplier in 1/8 units
    uint8_t par_[GRID_CELLS];
    uint8_t closed_[(GRID_CELLS + 7) / 8];
    uint32_t heap_[HEAP_CAP];
    int heapN_ = 0;

    // A* cost-so-far and the path post-processing buffers are never live together: A* is finished (and only par_
    // is still read) by the time coarse is written, and finish() needs no A* state. Inside finish() the heading
    // buffer hu is only needed after the path has been resampled, when coarse is no longer read, so it shares
    // coarse's memory.
    struct Finish {
        union {
            math::Vec2 coarse[MAX_COARSE];
            float hu[2 * MAX_COARSE];
        };
        math::Vec2 tmpA[MAX_COARSE];
        math::Vec2 tmpB[MAX_COARSE];
    };
    union Scratch {
        uint16_t g[GRID_CELLS];
        Finish f;
    } scratch_;
    static_assert(MAX_PATH <= 2 * MAX_COARSE, "the heading buffer shares the coarse path's memory");
};

}  // namespace nav
