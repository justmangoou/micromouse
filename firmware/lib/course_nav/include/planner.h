/**
 * @file    planner.h
 * @brief   Global planner: clearance-weighted A* on a 20 mm grid, string pulling, corner smoothing, resampling. Port
 *          of AStarDriver._plan in sim/course/planner.py. All memory is static (~36 KB), no allocation.
 */
#pragma once

#include <stdint.h>

#include "geometry.h"

namespace nav {

constexpr float GRID_RES = 20.0f;      // grid cell, mm
constexpr int GRID_NX = 91;           // x = 0 .. 1800 mm
constexpr int GRID_NY = 71;           // y = -200 .. 1200 mm
constexpr int GRID_CELLS = GRID_NX * GRID_NY;
constexpr int MAX_MAP = 24;       // mapped (static) blocks
constexpr int MAX_EXTRA = 8;      // unmapped blocks found by the ToF at run time
constexpr int MAX_PATH = 420;     // 15 mm spacing -> 6300 mm
constexpr int MAX_COARSE = 400;
// Open-list entries. The longest search in the host tests (unmapped blocks, replanning) peaks at 104, so this is
// ~10x headroom. When it fills, new entries are dropped, which can only cost path optimality.
constexpr int HEAP_CAP = 1024;

constexpr float R_ROBOT = 70.0f;   // min clearance from a block surface for the centre line
constexpr float R_PRUNE = 90.0f;   // shortcuts must keep this much clearance
// Stay this far inside the tape. A wheel sits track/2 + wheel_half_w = 50 mm off the centreline, so this must
// clear that plus *real* margin for dead-reckoning drift, not just a few mm. But the 600 mm-wide corridor (with
// R_ROBOT clearance from obstacles also eating into it) cannot support a static margin that covers the full drift
// budget -- above ~80 mm, the planner starts failing to find a route at all. So R_EDGE only buys back the wheel
// geometry plus a modest cushion; EDGE_SLOW_START (below) is what actually keeps the car off the tape when it has
// drifted further than that, by slowing it down near the boundary so pure pursuit has more time to correct.
constexpr float R_EDGE = 70.0f;
constexpr float EDGE_SLOW_START = 220.0f;  // mm of edge margin at which the speed factor starts dropping below 1
constexpr float DISC_R = 110.0f;    // an unmapped block is a disc around its estimated centre. Bigger than the
                                    // block's own 50 mm half-width: the single-ray near-face + half-block-behind
                                    // centre estimate is biased for oblique hits, so this buys back margin for that
constexpr float PATH_STEP = 15.0f;
constexpr float CENTERING = 6.0f;

constexpr math::Vec2 GOAL = {1500.0f, -120.0f};

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

    void clearExtras() { nExtras_ = 0; }
    int numExtras() const { return nExtras_; }
    int addExtra(math::Vec2 centre);  // -1 if full
    math::Vec2 extra(int i) const { return extras_[i]; }
    void setExtra(int i, math::Vec2 c) { extras_[i] = c; }

    // Plan from `start` to the finish line. False if no route exists even with the smallest margin.
    bool plan(math::Vec2 start, Path& out);
    // Fallback: straight line start -> goal.
    void straight(math::Vec2 start, Path& out);

    float clearance(math::Vec2 p) const;                   // blocks + unmapped discs
    float mapClearance(math::Vec2 p) const;                // mapped blocks only
    bool clearAtLeast(math::Vec2 p, float rad) const;
    bool segmentFree(math::Vec2 a, math::Vec2 b, float rad) const;

    int numBlocks() const { return nBlocks_; }
    const Block& block(int i) const { return blocks_[i]; }

private:
    int astar(math::Vec2 start, float radius);
    void buildWork();
    void finish(Path& out, int nCoarse);

    Block blocks_[MAX_MAP];
    int nBlocks_ = 0;
    math::Vec2 extras_[MAX_EXTRA];
    int nExtras_ = 0;

    uint8_t base_[GRID_CELLS];   // static clearance, 2.5 mm units (0 off the track)
    uint8_t work_[GRID_CELLS];   // base_ with the unmapped discs applied
    uint8_t mult8_[256];     // clearance -> cost multiplier in 1/8 units
    uint8_t par_[GRID_CELLS];
    uint8_t closed_[(GRID_CELLS + 7) / 8];
    uint32_t heap_[HEAP_CAP];
    int heapN_ = 0;

    // A* cost-so-far and the path post-processing buffers are never live together: A* is finished (and only par_
    // is still read) by the time coarse_ is written, and finish() needs no A* state.
    struct Finish {
        math::Vec2 coarse[MAX_COARSE];
        math::Vec2 tmpA[MAX_COARSE];
        math::Vec2 tmpB[MAX_COARSE];
        float hu[MAX_PATH];
    };
    union Scratch {
        uint16_t g[GRID_CELLS];
        Finish f;
    } scratch_;
    static_assert(sizeof(Finish) <= sizeof(uint16_t) * GRID_CELLS, "Finish buffers must fit in the A* cost array");
};

}  // namespace nav
