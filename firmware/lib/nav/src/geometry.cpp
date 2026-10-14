/**
 * @file    geometry.cpp
 * @brief   Geometry implementation: distance to a block, ray against a block (slab method in the block's frame) and
 *          the track test.
 */
#include "geometry.h"

namespace nav {

float blockClearance(math::Vec2 p, const Block& b) {
    float rx = p.x - b.x, ry = p.y - b.y;
    float lx = b.c * rx + b.s * ry;
    float ly = -b.s * rx + b.c * ry;
    float dx = fmaxf(fabsf(lx) - BLOCK_HALF, 0.0f);
    float dy = fmaxf(fabsf(ly) - BLOCK_HALF, 0.0f);
    return sqrtf(dx * dx + dy * dy);
}

float rayBlock(math::Vec2 o, math::Vec2 d, const Block& b, float maxRange) {
    float rx = o.x - b.x, ry = o.y - b.y;
    float ox = b.c * rx + b.s * ry;
    float oy = -b.s * rx + b.c * ry;
    float dx = b.c * d.x + b.s * d.y;
    float dy = -b.s * d.x + b.c * d.y;
    if (fabsf(dx) < 1e-9f) dx = 1e-9f;
    if (fabsf(dy) < 1e-9f) dy = 1e-9f;
    float tx1 = (-BLOCK_HALF - ox) / dx, tx2 = (BLOCK_HALF - ox) / dx;
    float ty1 = (-BLOCK_HALF - oy) / dy, ty2 = (BLOCK_HALF - oy) / dy;
    float tmin = fmaxf(fminf(tx1, tx2), fminf(ty1, ty2));
    float tmax = fminf(fmaxf(tx1, tx2), fmaxf(ty1, ty2));
    float t0 = fmaxf(tmin, 0.0f);
    if (tmax >= t0) return fminf(t0, maxRange);
    return maxRange;
}

bool insideTrack(math::Vec2 p, float m) {
    // left column, top band, right column (x0, y0, x1, y1)
    static const float r[3][4] = {
        {0.0f, PAD_Y, 60.0f, ARENA_H},
        {0.0f, 60.0f, ARENA_W, ARENA_H},
        {120.0f, PAD_Y, ARENA_W, ARENA_H},
    };
    for (int i = 0; i < 3; i++) {
        if (p.x >= r[i][0] - m && p.x <= r[i][2] + m && p.y >= r[i][1] - m && p.y <= r[i][3] + m) return true;
    }
    return false;
}

float edgeMargin(math::Vec2 p, float hi, int iters) {
    float lo = 0.0f;
    for (int i = 0; i < iters; i++) {
        float mid = 0.5f * (lo + hi);
        if (insideTrack(p, -mid)) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    return lo;
}

}  // namespace nav
