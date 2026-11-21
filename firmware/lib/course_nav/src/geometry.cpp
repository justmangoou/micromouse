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

float wallClearance(math::Vec2 p) {
    static const float w[6][4] = {
        {0.0f, 0.0f, 0.0f, ARENA_H}, {ARENA_W, 0.0f, ARENA_W, ARENA_H}, {0.0f, ARENA_H, ARENA_W, ARENA_H},
        {600.0f, 0.0f, 600.0f, 600.0f}, {1200.0f, 0.0f, 1200.0f, 600.0f}, {600.0f, 600.0f, 1200.0f, 600.0f},
    };
    float best = 1.0e9f;
    for (int i = 0; i < 6; i++) {
        float ex = w[i][2] - w[i][0], ey = w[i][3] - w[i][1];
        float t = ((p.x - w[i][0]) * ex + (p.y - w[i][1]) * ey) / (ex * ex + ey * ey);
        t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
        best = fminf(best, hypotf(p.x - (w[i][0] + t * ex), p.y - (w[i][1] + t * ey)));
    }
    return best;
}

float rayWalls(math::Vec2 o, math::Vec2 d, float maxRange) {
    // wall segments: x0, y0, x1, y1
    static const float w[6][4] = {
        {0.0f, 0.0f, 0.0f, ARENA_H},            // left
        {ARENA_W, 0.0f, ARENA_W, ARENA_H},      // right
        {0.0f, ARENA_H, ARENA_W, ARENA_H},      // top
        {600.0f, 0.0f, 600.0f, 600.0f},         // pit, left
        {1200.0f, 0.0f, 1200.0f, 600.0f},       // pit, right
        {600.0f, 600.0f, 1200.0f, 600.0f},      // pit, top (its bottom is open to the start / finish floor)
    };
    float best = maxRange;
    for (int i = 0; i < 6; i++) {
        float ex = w[i][2] - w[i][0], ey = w[i][3] - w[i][1];
        float den = d.x * ey - d.y * ex;
        if (fabsf(den) < 1e-9f) continue;  // parallel
        float qx = w[i][0] - o.x, qy = w[i][1] - o.y;
        float t = (qx * ey - qy * ex) / den;   // distance along the ray
        float u = (qx * d.y - qy * d.x) / den;  // position along the wall, 0..1
        if (t >= 0.0f && t < best && u >= 0.0f && u <= 1.0f) best = t;
    }
    return best;
}

bool insideTrack(math::Vec2 p, float m) {
    // left column, top band, right column (x0, y0, x1, y1)
    static const float r[3][4] = {
        {0.0f, PAD_Y, 600.0f, ARENA_H},
        {0.0f, 600.0f, ARENA_W, ARENA_H},
        {1200.0f, PAD_Y, ARENA_W, ARENA_H},
    };
    for (int i = 0; i < 3; i++) {
        if (p.x >= r[i][0] - m && p.x <= r[i][2] + m && p.y >= r[i][1] - m && p.y <= r[i][3] + m) return true;
    }
    return false;
}

float edgeMargin(math::Vec2 p, float hi, int iters) {
    (void)iters;
    // insideTrack(p, -m) holds while m <= min distance to the sides of at least one of the three rectangles, so the
    // margin is the best of those: the same answer as bisecting on insideTrack, without ten rounds of float tests.
    static const float r[3][4] = {
        {0.0f, PAD_Y, 600.0f, ARENA_H},
        {0.0f, 600.0f, ARENA_W, ARENA_H},
        {1200.0f, PAD_Y, ARENA_W, ARENA_H},
    };
    float best = 0.0f;
    for (int i = 0; i < 3; i++) {
        float m = fminf(fminf(p.x - r[i][0], r[i][2] - p.x), fminf(p.y - r[i][1], r[i][3] - p.y));
        if (m > best) best = m;
    }
    return best < hi ? best : hi;
}

}  // namespace nav
