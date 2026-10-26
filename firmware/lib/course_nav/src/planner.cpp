/**
 * @file    planner.cpp
 * @brief   Planner implementation: A* with integer costs (octile heuristic, lazy-deletion heap), string pulling with
 *          a coarse stride, Chaikin smoothing, 15 mm resampling and the speed profile.
 */
#include "planner.h"

#include <string.h>

#include <algorithm>

namespace nav {

namespace {

inline float cellX(int ix) { return ix * GRID_RES; }
inline float cellY(int iy) { return PAD_Y + iy * GRID_RES; }

// goal cells: x in [1350, 1650], y <= -120 mm
constexpr int GOAL_X0 = 68, GOAL_X1 = 82, GOAL_Y1 = 4;

inline bool isGoal(int ix, int iy) { return ix >= GOAL_X0 && ix <= GOAL_X1 && iy <= GOAL_Y1; }

inline int heuristic(int ix, int iy) {
    int dx = 0;
    if (ix < GOAL_X0) dx = GOAL_X0 - ix;
    else if (ix > GOAL_X1) dx = ix - GOAL_X1;
    int dy = iy > GOAL_Y1 ? iy - GOAL_Y1 : 0;
    int hi = dx > dy ? dx : dy, lo = dx > dy ? dy : dx;
    return 10 * hi + 4 * lo;  // octile distance with straight = 10, diagonal = 14
}

}  // namespace

void Planner::setMap(const Block* blocks, int n) {
    nBlocks_ = n > MAX_MAP ? MAX_MAP : n;
    for (int i = 0; i < nBlocks_; i++) blocks_[i] = blocks[i];
    nExtras_ = 0;
    for (int iy = 0; iy < GRID_NY; iy++) {
        for (int ix = 0; ix < GRID_NX; ix++) {
            math::Vec2 p = {cellX(ix), cellY(iy)};
            uint8_t q = 0;
            if (insideTrack(p, -R_EDGE)) {
                float c = mapClearance(p);
                q = (uint8_t)std::clamp(c * 0.4f, 0.0f, 255.0f);
            }
            base_[iy * GRID_NX + ix] = q;
        }
    }
    for (int q = 0; q < 256; q++) {
        float c = q * 2.5f;
        float t = std::clamp((140.0f - c) / 140.0f, 0.0f, 1.0f);
        mult8_[q] = (uint8_t)(8.0f * (1.0f + CENTERING * t * t) + 0.5f);
    }
}

int Planner::addExtra(math::Vec2 centre) {
    if (nExtras_ >= MAX_EXTRA) return -1;
    extras_[nExtras_] = centre;
    return nExtras_++;
}

float Planner::mapClearance(math::Vec2 p) const {
    float best = 1e4f;
    for (int i = 0; i < nBlocks_; i++) best = fminf(best, blockClearance(p, blocks_[i]));
    return best;
}

float Planner::clearance(math::Vec2 p) const {
    float best = mapClearance(p);
    for (int i = 0; i < nExtras_; i++) best = fminf(best, math::distance(p, extras_[i]) - DISC_R);
    return best;
}

bool Planner::clearAtLeast(math::Vec2 p, float rad) const {
    for (int i = 0; i < nBlocks_; i++) {
        const Block& b = blocks_[i];
        float dx = p.x - b.x, dy = p.y - b.y;
        float lim = rad + BLOCK_HALF_DIAG;
        if (dx * dx + dy * dy > lim * lim) continue;  // too far to matter
        if (blockClearance(p, b) < rad) return false;
    }
    for (int i = 0; i < nExtras_; i++) {
        if (math::distance(p, extras_[i]) - DISC_R < rad) return false;
    }
    return true;
}

bool Planner::segmentFree(math::Vec2 a, math::Vec2 b, float rad) const {
    float len = math::distance(a, b);
    int n = (int)(len / 10.0f);
    if (n < 2) n = 2;
    for (int k = 0; k <= n; k++) {
        float t = (float)k / n;
        math::Vec2 p = math::lerp(a, b, t);
        if (!clearAtLeast(p, rad) || !insideTrack(p, -R_EDGE)) return false;
    }
    return true;
}

void Planner::buildWork() {
    memcpy(work_, base_, sizeof(work_));
    const float reach = DISC_R + 150.0f;
    for (int e = 0; e < nExtras_; e++) {
        math::Vec2 c = extras_[e];
        int x0 = std::clamp((int)((c.x - reach) / GRID_RES), 0, GRID_NX - 1), x1 = std::clamp((int)((c.x + reach) / GRID_RES) + 1, 0, GRID_NX - 1);
        int y0 = std::clamp((int)((c.y - reach - PAD_Y) / GRID_RES), 0, GRID_NY - 1);
        int y1 = std::clamp((int)((c.y + reach - PAD_Y) / GRID_RES) + 1, 0, GRID_NY - 1);
        for (int iy = y0; iy <= y1; iy++) {
            for (int ix = x0; ix <= x1; ix++) {
                float d = math::distance({cellX(ix), cellY(iy)}, c) - DISC_R;
                uint8_t q = (uint8_t)std::clamp(d * 0.4f, 0.0f, 255.0f);
                uint8_t& w = work_[iy * GRID_NX + ix];
                if (q < w) w = q;
            }
        }
    }
}

namespace {

// binary min-heap on (f << 16 | index)
inline void heapPush(uint32_t* h, int& n, int cap, uint32_t key) {
    if (n >= cap) return;  // full: drop (only costs optimality)
    int i = n++;
    while (i > 0) {
        int parent = (i - 1) / 2;
        if (h[parent] <= key) break;
        h[i] = h[parent];
        i = parent;
    }
    h[i] = key;
}

inline uint32_t heapPop(uint32_t* h, int& n) {
    uint32_t top = h[0];
    uint32_t last = h[--n];
    int i = 0;
    while (true) {
        int l = 2 * i + 1;
        if (l >= n) break;
        int r = l + 1;
        int c = (r < n && h[r] < h[l]) ? r : l;
        if (h[c] >= last) break;
        h[i] = h[c];
        i = c;
    }
    if (n > 0) h[i] = last;
    return top;
}

const int8_t NEIGHBOR_STEPS[8][2] = {{-1, -1}, {0, -1}, {1, -1}, {-1, 0}, {1, 0}, {-1, 1}, {0, 1}, {1, 1}};

}  // namespace

// Returns the number of points written to scratch_.f.coarse (cell centres, start replaced by `start`), 0 on failure.
int Planner::astar(math::Vec2 start, float radius) {
    const uint8_t rq = (uint8_t)ceilf(radius * 0.4f);
    memset(scratch_.g, 0xFF, sizeof(scratch_.g));
    memset(par_, 0xFF, sizeof(par_));
    memset(closed_, 0, sizeof(closed_));
    heapN_ = 0;

    int sx = std::clamp((int)lroundf(start.x / GRID_RES), 0, GRID_NX - 1);
    int sy = std::clamp((int)lroundf((start.y - PAD_Y) / GRID_RES), 0, GRID_NY - 1);
    int s = sy * GRID_NX + sx;
    scratch_.g[s] = 0;
    heapPush(heap_, heapN_, HEAP_CAP, ((uint32_t)heuristic(sx, sy) << 16) | (uint32_t)s);

    while (heapN_ > 0) {
        int i = (int)(heapPop(heap_, heapN_) & 0xFFFF);
        if (closed_[i >> 3] & (1 << (i & 7))) continue;
        closed_[i >> 3] |= (uint8_t)(1 << (i & 7));
        int ix = i % GRID_NX, iy = i / GRID_NX;
        if (isGoal(ix, iy) && work_[i] >= rq) {
            int n = 0;
            int cur = i;
            while (true) {
                if (n >= MAX_COARSE) return 0;
                scratch_.f.coarse[n++] = {cellX(cur % GRID_NX), cellY(cur / GRID_NX)};
                if (cur == s) break;
                int d = par_[cur];
                cur -= NEIGHBOR_STEPS[d][1] * GRID_NX + NEIGHBOR_STEPS[d][0];
            }
            for (int a = 0, b = n - 1; a < b; a++, b--) {
                math::Vec2 t = scratch_.f.coarse[a];
                scratch_.f.coarse[a] = scratch_.f.coarse[b];
                scratch_.f.coarse[b] = t;
            }
            scratch_.f.coarse[0] = start;
            return n;
        }
        for (int d = 0; d < 8; d++) {
            int nx = ix + NEIGHBOR_STEPS[d][0], ny = iy + NEIGHBOR_STEPS[d][1];
            if (nx < 0 || nx >= GRID_NX || ny < 0 || ny >= GRID_NY) continue;
            int j = ny * GRID_NX + nx;
            if (work_[j] < rq) continue;
            int base = (NEIGHBOR_STEPS[d][0] != 0 && NEIGHBOR_STEPS[d][1] != 0) ? 14 : 10;
            uint32_t cost = (uint32_t)(base * (mult8_[work_[i]] + mult8_[work_[j]])) >> 4;
            uint32_t ng = (uint32_t)scratch_.g[i] + cost;
            if (ng > 0xFFFE) continue;
            if (ng < scratch_.g[j]) {
                scratch_.g[j] = (uint16_t)ng;
                par_[j] = (uint8_t)d;
                uint32_t f = ng + (uint32_t)heuristic(nx, ny);
                if (f > 0xFFFF) f = 0xFFFF;
                heapPush(heap_, heapN_, HEAP_CAP, (f << 16) | (uint32_t)j);
            }
        }
    }
    return 0;
}

bool Planner::plan(math::Vec2 start, Path& out) {
    out.n = 0;
    buildWork();
    static const float radii[3] = {R_ROBOT, R_ROBOT - 10.0f, 45.0f};  // squeeze through if the preferred margin is gone
    int n = 0;
    for (int r = 0; r < 3 && n == 0; r++) n = astar(start, radii[r]);
    if (n == 0) return false;
    finish(out, n);
    return true;
}

void Planner::straight(math::Vec2 start, Path& out) {
    scratch_.f.coarse[0] = start;
    scratch_.f.coarse[1] = GOAL;
    finish(out, 2);
}

// scratch_.f.coarse[0..n) -> pruned -> smoothed -> extended -> resampled into `out`
void Planner::finish(Path& out, int n) {
    // string pulling: drop waypoints while a straight shortcut keeps the clearance
    math::Vec2* pr = scratch_.f.tmpA;
    int np = 0;
    pr[np++] = scratch_.f.coarse[0];
    int i = 0;
    while (i < n - 1) {
        int j = n - 1;
        // coarse stride first, then refine to the farthest visible point
        while (j > i + 1 && !segmentFree(scratch_.f.coarse[i], scratch_.f.coarse[j], R_PRUNE)) j -= (j - i > 12 ? 8 : 1);
        if (j < i + 1) j = i + 1;
        int refine = (j + 8 < n - 1) ? j + 8 : n - 1;
        for (int k = refine; k > j; k--) {
            if (segmentFree(scratch_.f.coarse[i], scratch_.f.coarse[k], R_PRUNE)) {
                j = k;
                break;
            }
        }
        if (np >= MAX_COARSE / 4) {  // too many corners to smooth safely: keep the rest as is
            pr[np++] = scratch_.f.coarse[n - 1];
            break;
        }
        pr[np++] = scratch_.f.coarse[j];
        i = j;
    }

    // Chaikin corner cutting (twice), kept only while the result stays clear of the blocks
    math::Vec2* cur = pr;
    math::Vec2* alt = scratch_.f.tmpB;
    for (int it = 0; it < 2; it++) {
        if (np < 2 || 2 * np > MAX_COARSE) break;
        int m = 0;
        alt[m++] = cur[0];
        for (int k = 0; k < np - 1; k++) {
            alt[m++] = {0.75f * cur[k].x + 0.25f * cur[k + 1].x, 0.75f * cur[k].y + 0.25f * cur[k + 1].y};
            alt[m++] = {0.25f * cur[k].x + 0.75f * cur[k + 1].x, 0.25f * cur[k].y + 0.75f * cur[k + 1].y};
        }
        alt[m++] = cur[np - 1];
        bool ok = true;
        for (int k = 0; k < m && ok; k++) ok = clearAtLeast(alt[k], R_ROBOT - 5.0f);
        if (!ok) break;
        math::Vec2* t = cur;
        cur = alt;
        alt = t;
        np = m;
    }

    // run through the finish line along the last direction
    if (np + 2 > MAX_COARSE) np = MAX_COARSE - 2;
    math::Vec2 last = cur[np - 1];
    math::Vec2 d = {0.0f, -1.0f};
    if (np > 1) {
        float dx = last.x - cur[np - 2].x, dy = last.y - cur[np - 2].y;
        float len = fmaxf(hypotf(dx, dy), 1e-5f);
        d = {dx / len, dy / len};
    }
    cur[np++] = {last.x + d.x * 150.0f, last.y + d.y * 150.0f};
    cur[np++] = {last.x + d.x * 300.0f, last.y + d.y * 300.0f};

    // resample at PATH_STEP
    float total = 0.0f;
    for (int k = 0; k < np - 1; k++) total += math::distance(cur[k], cur[k + 1]);
    int m = 0, seg = 0;
    float segStart = 0.0f;
    for (float s = 0.0f; s < total && m < MAX_PATH; s += PATH_STEP) {
        while (seg < np - 2 && s > segStart + math::distance(cur[seg], cur[seg + 1])) {
            segStart += math::distance(cur[seg], cur[seg + 1]);
            seg++;
        }
        float len = fmaxf(math::distance(cur[seg], cur[seg + 1]), 1e-5f);
        float t = std::clamp((s - segStart) / len, 0.0f, 1.0f);
        out.p[m] = math::lerp(cur[seg], cur[seg + 1], t);
        out.s[m] = s;
        m++;
    }
    out.n = m;

    // speed factor: slow in tight turns and near blocks
    scratch_.f.hu[0] = 0.0f;
    float prev = 0.0f;
    for (int k = 0; k < m - 1; k++) {
        float h = atan2f(out.p[k + 1].y - out.p[k].y, out.p[k + 1].x - out.p[k].x);
        if (k == 0) {
            scratch_.f.hu[0] = h;
        } else {
            scratch_.f.hu[k] = scratch_.f.hu[k - 1] + math::wrap(h - prev);
        }
        prev = h;
    }
    if (m > 1) scratch_.f.hu[m - 1] = scratch_.f.hu[m - 2];
    const int kw = 5;
    for (int k = 0; k < m; k++) {
        int a = k + kw < m - 1 ? k + kw : m - 1;
        int b = k - kw > 0 ? k - kw : 0;
        float turn = fabsf(scratch_.f.hu[a] - scratch_.f.hu[b]);
        float clr = clearance(out.p[k]);
        float edgeClr = edgeMargin(out.p[k]);
        float vTurn = std::clamp(1.0f / (1.0f + 2.0f * turn), 0.35f, 1.0f);
        float vClr = std::clamp(0.6f + 0.4f * (clr - R_ROBOT) / 60.0f, 0.6f, 1.0f);
        float vEdge = std::clamp(0.5f + 0.5f * (edgeClr - R_EDGE) / EDGE_SLOW_START, 0.5f, 1.0f);
        out.v[k] = vTurn * vClr * vEdge;
    }
}

}  // namespace nav
