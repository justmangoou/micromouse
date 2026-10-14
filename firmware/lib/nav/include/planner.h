/**
 * @file    planner.h
 * @brief   Global planner: clearance-weighted A* on a 2 cm grid, string pulling, corner smoothing, resampling. Port
 *          of AStarDriver._plan in sim/course/planner.py. All memory is static (~60 KB), no allocation.
 */
#pragma once

#include <stdint.h>

#include "geometry.h"

namespace nav {

constexpr float GRID_RES = 2.0f;      // grid cell, cm
constexpr int GRID_NX = 91;           // x = 0 .. 180
constexpr int GRID_NY = 71;           // y = -20 .. 120
constexpr int GRID_CELLS = GRID_NX * GRID_NY;
constexpr int MAX_MAP = 24;       // mapped (static) blocks
constexpr int MAX_EXTRA = 8;      // unmapped blocks found by the ToF at run time
constexpr int MAX_PATH = 420;     // 1.5 cm spacing -> 630 cm
constexpr int MAX_COARSE = 400;
constexpr int HEAP_CAP = 6144;

constexpr float R_ROBOT = 7.0f;   // min clearance from a block surface for the centre line
constexpr float R_PRUNE = 9.0f;   // shortcuts must keep this much clearance
// Stay this far inside the tape. A wheel sits WHEEL_TRACK/2 + wheel_half_w = 5.0 cm off the centreline, so this must
// clear that plus *real* margin for dead-reckoning drift, not just a few mm. But the 60 cm-wide corridor (with
// R_ROBOT clearance from obstacles also eating into it) cannot support a static margin that covers the full drift
// budget -- above ~8 cm, the planner starts failing to find a route at all. So R_EDGE only buys back the wheel
// geometry plus a modest cushion; EDGE_SLOW_START (below) is what actually keeps the car off the tape when it has
// drifted further than that, by slowing it down near the boundary so pure pursuit has more time to correct.
constexpr float R_EDGE = 7.0f;
constexpr float EDGE_SLOW_START = 22.0f;  // cm of edge margin at which the speed factor starts dropping below 1
constexpr float DISC_R = 11.0f;    // an unmapped block is a disc around its estimated centre. Bigger than the
                                    // block's own 5 cm half-width: the single-ray near-face + half-block-behind
                                    // centre estimate is biased for oblique hits, so this buys back margin for that
constexpr float PATH_STEP = 1.5f;
constexpr float CENTERING = 6.0f;

constexpr math::Vec2 GOAL = {150.0f, -12.0f};

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

    uint8_t base_[GRID_CELLS];   // static clearance, 0.25 cm units (0 off the track)
    uint8_t work_[GRID_CELLS];   // base_ with the unmapped discs applied
    uint8_t mult8_[256];     // clearance -> cost multiplier in 1/8 units
    uint16_t g_[GRID_CELLS];
    uint8_t par_[GRID_CELLS];
    uint8_t closed_[(GRID_CELLS + 7) / 8];
    uint32_t heap_[HEAP_CAP];
    int heapN_ = 0;
    math::Vec2 coarse_[MAX_COARSE];
    math::Vec2 tmpA_[MAX_COARSE];
    math::Vec2 tmpB_[MAX_COARSE];
    float hu_[MAX_PATH];
};

}  // namespace nav
