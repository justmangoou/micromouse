/**
 * @file    geometry.h
 * @brief   Arena geometry shared by the planner, the navigator and the host tests: blocks, ray-casting and the track.
 *          Units: mm, rad. Mirrors sim/config.py and sim/course/arena.py. Generic helpers (math::Vec2, math::Pose,
 *          math::wrap) are in math_util.h.
 */
#pragma once

#include <math.h>

#include "math_util.h"

namespace nav {

constexpr float BLOCK_HALF = 50.0f;  // blocks are 100 x 100 x 100 mm
constexpr float BLOCK_HALF_DIAG = 70.711f;
constexpr float TAPE_HALF = 10.0f;
constexpr float ARENA_W = 1800.0f;
constexpr float PAD_Y = -200.0f;     // floor below the start / finish line
constexpr float ARENA_H = 1200.0f;

// A 100 mm square block: centre and rotation. cos/sin are cached because every query needs them.
struct Block {
    float x = 0, y = 0, ang = 0, c = 1, s = 0;
    Block() {}
    Block(float x_, float y_, float ang_) : x(x_), y(y_), ang(ang_), c(cosf(ang_)), s(sinf(ang_)) {}
};

// Distance from p to the surface of the block (0 inside).
float blockClearance(math::Vec2 p, const Block& b);

// Distance along the ray (o, unit d) to the block, or maxRange.
float rayBlock(math::Vec2 o, math::Vec2 d, const Block& b, float maxRange);

// On the inverted-U track. margin > 0 dilates it (tape), < 0 erodes it.
bool insideTrack(math::Vec2 p, float margin);

// How many mm of margin this point (assumed on the track) still has before insideTrack would call it off-track --
// i.e. its distance to the tape boundary. Found by bisection on insideTrack rather than deriving the U-shaped
// polygon's edges by hand (it is a union of 3 rectangles, not one convex shape). Port of arena.edge_margin.
float edgeMargin(math::Vec2 p, float hi = 300.0f, int iters = 10);

}  // namespace nav
