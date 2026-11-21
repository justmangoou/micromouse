/**
 * @file    navigator.cpp
 * @brief   Navigator implementation: unmapped-block detection (ToF against the distance the map predicts), replan
 *          triggers and pure-pursuit tracking with speed scaling.
 */
#include "navigator.h"
#include <cmath>
#include <cstdlib>

namespace nav {

bool Navigator::configurationValid() const {
    return layout_.valid() && std::isfinite(prm_.vFull) && prm_.vFull > 0 &&
           std::isfinite(prm_.track) && prm_.track > 0 &&
           std::isfinite(prm_.speed) && prm_.speed > 0 && prm_.speed <= 1 &&
           std::isfinite(prm_.lookahead) && prm_.lookahead > 0 &&
           std::isfinite(prm_.caution) && prm_.caution > 0 && prm_.caution <= 1 &&
           std::isfinite(prm_.alertSpeed) && prm_.alertSpeed > 0 && prm_.alertSpeed <= 1;
}

void Navigator::init(const float (*map)[3], int n) {
    planner_.setMap(map, n);
}

bool Navigator::replan(math::Vec2 from, float t) {
    path_.n = 0;
    bool ok = configurationValid() && std::isfinite(from.x) && std::isfinite(from.y) &&
              std::isfinite(t) && planner_.plan(from, path_) && path_.n > 1;
    if (!ok && planner_.numExtras() > 0 && configurationValid()) {
        // Detected blocks closed the route. They are estimates (a ToF ray, a map that is off by centimetres), so a
        // route through the mapped course is a better bet than stopping.
        planner_.clearExtras();
        ok = planner_.plan(from, path_) && path_.n > 1;
    }
    if (!ok) path_.n = 0;
    lastPlanT_ = t;
    needReplan_ = false;
    idx_ = 0;
    replans_++;
    return ok;
}

bool Navigator::start(const math::Pose& est, float t) {
    planner_.clearExtras();
    for (auto& streak : streak_) streak = 0;
    for (int i = 0; i < MAX_EXTRA; i++) hits_[i] = 0;
    alertUntil_ = -1.0f;
    lastLocT_ = -1.0f;
    locOffset_ = {0, 0};
    locTurn_ = 0.0f;
    replans_ = 0;
    path_.n = 0;
    return replan({est.x, est.y}, t);
}

bool Navigator::reposition(const math::Pose& est, float t) { return replan({est.x, est.y}, t); }

// A reading much shorter than the map predicts is a block that is not in the map.
void Navigator::scan(const math::Pose& e, const float tof[], float t) {
    float c = cosf(e.th), s = sinf(e.th);
    float baseRange[3];
    const bool covered = baselineAt(e, baseRange);
    for (int k = 0; k < layout_.count; k++) {
        math::Vec2 o = {e.x + c * layout_.sensors[k].x - s * layout_.sensors[k].y, e.y + s * layout_.sensors[k].x + c * layout_.sensors[k].y};
        float expected = layout_.maxMm;
        for (int f = 0; f < layout_.rayCount; f++) {
            float a = e.th + layout_.sensors[k].angle + layout_.rays[f];
            math::Vec2 d = {cosf(a), sinf(a)};
            for (int b = 0; b < planner_.numBlocks(); b++) {
                float r = rayBlock(o, d, planner_.block(b), layout_.maxMm);
                if (r < expected) expected = r;
            }
        }
        // The arena and pit walls are in no block list. Walls are checked over a +-20 deg fan around the sensor axis, wider than
        // the sensor's field of view: with a few cm of pose drift a wall corner (the pit's top-right one, seen from (1040,790))
        // is inside the real field of view but outside the predicted rays, and would otherwise read as a new block.
        for (int f = -2; f <= 2; f++) {
            float a = e.th + layout_.sensors[k].angle + f * 0.175f;
            float w = rayWalls(o, {cosf(a), sinf(a)}, layout_.maxMm);
            if (w < expected) expected = w;
        }
        float z = tof[k];
        bool unexplained = z < expected - (40.0f + prm_.rangeTol * expected) && z < 800.0f;
        if (covered) unexplained = z < baseRange[k] - base_.margin && z < 800.0f && z >= 40.0f;  // the clean runs saw this much from here
        streak_[k] = unexplained ? (uint8_t)(streak_[k] < 250 ? streak_[k] + 1 : 250) : 0;
        if (streak_[k] < prm_.confirmSteps) continue;

        float a = e.th + layout_.sensors[k].angle;
        math::Vec2 d = {cosf(a), sinf(a)};
        math::Vec2 hit = {o.x + z * d.x, o.y + z * d.y};
        if (hit.x < prm_.unmappedMinX) continue;           // the left part of the arena is fully mapped
        if (prm_.pathReach < 1.0e8f && !nearPathAhead(hit, prm_.pathReach, prm_.pathAhead)) continue;  // not on our way: no reason to replan
        if (!covered && wallClearance(hit) < prm_.wallMargin) continue;          // a wall or wall corner seen through pose error (the pit corner reads like a block)
        if (isBackground(hit)) continue;                    // a known static return seen on clean runs (pit corner, walls)
        if (!covered && planner_.mapClearance(hit) < prm_.mapMargin) continue;  // a mapped block seen through pose and map error (real course is +-5-8 cm off)
        math::Vec2 centre = {o.x + (z + BLOCK_HALF) * d.x, o.y + (z + BLOCK_HALF) * d.y};  // near face seen, centre is behind

        if (centre.x > prm_.unmappedMaxX || centre.y < prm_.unmappedMinY) continue;  // outside the arena, or on the finish tape

        bool merged = false;
        for (int i = 0; i < planner_.numExtras(); i++) {
            math::Vec2 ex = planner_.extra(i);
            float dist = math::distance(ex, centre);
            // the near-face+half-block-behind centre estimate drifts a lot with viewing angle (seen up to ~190 mm
            // between two detections of the same block), so this has to be generous or MAX_EXTRA fills with ghost
            // duplicates of the same 1-2 real blocks
            if (dist < 180.0f) {
                if (hits_[i] < 250) hits_[i]++;
                if (dist > 20.0f && hits_[i] < 6) {
                    planner_.setExtra(i, math::lerp(ex, centre, 0.3f));
                    needReplan_ = true;
                }
                merged = true;
                break;
            }
        }
        if (!merged) {
            if (planner_.numExtras() >= prm_.maxExtras) continue;  // the course has that many random blocks, no more
            int i = planner_.addExtra(centre);
            if (i >= 0) {
                hits_[i] = 1;
                needReplan_ = true;
                alertUntil_ = t + 0.8f;
            }
        }
    }
}

// Predicted range of sensor k from pose e: the nearest mapped block over the sensor's outer and centre rays.
static float predictRange(const Planner& pl, const tof::Layout& L, const math::Pose& e, int k) {
    float c = cosf(e.th), sn = sinf(e.th);
    math::Vec2 o = {e.x + c * L.sensors[k].x - sn * L.sensors[k].y, e.y + sn * L.sensors[k].x + c * L.sensors[k].y};
    float best = L.maxMm;
    for (int f = 0; f < L.rayCount; f += (L.rayCount > 2 ? L.rayCount / 2 : 1)) {
        float a = e.th + L.sensors[k].angle + L.rays[f];
        math::Vec2 d = {cosf(a), sinf(a)};
        for (int b = 0; b < pl.numBlocks(); b++) {
            const Block& bl = pl.block(b);
            float dx = bl.x - o.x, dy = bl.y - o.y;
            if (dx * dx + dy * dy > 350.0f * 350.0f) continue;
            float r = rayBlock(o, d, bl, L.maxMm);
            if (r < best) best = r;
        }
    }
    return best;
}

static constexpr float LOC_MAX_MM = 220.0f;     // beyond this the floor and the sensor's compression spoil the reading
static constexpr float LOC_GATE_MM = 70.0f;     // a reading further than this from the map's prediction is not a mapped block
static constexpr float LOC_STEP_MM = 20.0f, LOC_STEP_RAD = 0.035f;
static constexpr float LOC_LIMIT_MM = 120.0f, LOC_LIMIT_RAD = 0.17f;

// Sum of squared range errors over the sensors in `use`, from pose e.
float Navigator::locCost(const math::Pose& e, const float tof[], const bool use[], int* used) const {
    float cost = 0;
    int n = 0;
    for (int k = 0; k < layout_.count; k++) {
        if (!use[k]) continue;
        float r = predictRange(planner_, layout_, e, k);
        float err = tof[k] - r;
        if (r > LOC_MAX_MM + 60.0f) err = 60.0f;  // the map says nothing there
        if (err > 60.0f) err = 60.0f;
        if (err < -60.0f) err = -60.0f;
        cost += err * err;
        n++;
    }
    if (used) *used = n;
    return cost;
}

bool Navigator::localize(math::Pose& est, const float tof[], float t) {
    if (t - lastLocT_ < 0.1f) return false;
    lastLocT_ = t;
    bool use[tof::MAX_SENSORS] = {false};
    int any = 0;
    for (int k = 0; k < layout_.count; k++) {
        float z = tof[k];
        if (!(z >= layout_.minMm && z <= LOC_MAX_MM)) continue;
        float r = predictRange(planner_, layout_, est, k);
        if (r > LOC_MAX_MM + 40.0f || fabsf(z - r) > LOC_GATE_MM) continue;  // not explained by a mapped block
        use[k] = true;
        any++;
    }
    if (!any) return false;
    float base = locCost(est, tof, use, nullptr);
    if (base < 100.0f) return false;  // already within a few mm
    float best = base;
    math::Pose bp = est;
    for (int ix = -1; ix <= 1; ix++)
        for (int iy = -1; iy <= 1; iy++)
            for (int it = -1; it <= 1; it++) {
                if (!ix && !iy && !it) continue;
                math::Pose c = {est.x + ix * LOC_STEP_MM, est.y + iy * LOC_STEP_MM, est.th + it * LOC_STEP_RAD};
                float nx = locOffset_.x + ix * LOC_STEP_MM, ny = locOffset_.y + iy * LOC_STEP_MM;
                float nt = locTurn_ + it * LOC_STEP_RAD;
                if (fabsf(nx) > LOC_LIMIT_MM || fabsf(ny) > LOC_LIMIT_MM || fabsf(nt) > LOC_LIMIT_RAD) continue;
                float cost = locCost(c, tof, use, nullptr);
                // a step must be a clear improvement, and the smaller step wins a tie
                cost += (abs(ix) + abs(iy) + abs(it)) * 25.0f;
                if (cost < best) { best = cost; bp = c; }
            }
    if (best >= base * 0.85f) return false;
    locOffset_.x += bp.x - est.x;
    locOffset_.y += bp.y - est.y;
    locTurn_ += bp.th - est.th;
    est = bp;
    return true;
}

void Navigator::track(const math::Pose& e, float speed, float t, float duty[2]) {
    math::Vec2 pos = {e.x, e.y};
    int hi = idx_ + 80 < path_.n ? idx_ + 80 : path_.n;
    float best = 1e9f;
    int bi = idx_;
    for (int i = idx_; i < hi; i++) {
        float d = math::distance(path_.p[i], pos);
        if (d < best) {
            best = d;
            bi = i;
        }
    }
    idx_ = bi;
    if (best > 250.0f && t - lastPlanT_ > 0.5f) {  // far off the line: plan again from here
        if (!replan(pos, t)) return;
        best = 0.0f;
    }

    float look = prm_.lookahead + prm_.lookaheadGain * fabsf(speed);
    int j = idx_;
    while (j < path_.n - 1 && path_.s[j] < path_.s[idx_] + look) j++;
    math::Vec2 tgt = path_.p[j];
    float dist = fmaxf(math::distance({e.x, e.y}, tgt), 10.0f);
    float alpha = math::wrap(atan2f(tgt.y - e.y, tgt.x - e.x) - e.th);
    lastAlpha_ = alpha;
    if (fabsf(alpha) > 1.2f) {  // facing the wrong way: pivot first
        float sg = alpha > 0 ? 1.0f : -1.0f;
        duty[0] = -0.45f * sg;
        duty[1] = 0.45f * sg;
        return;
    }
    float kappa = 2.0f * sinf(alpha) / dist;
    float pv = prm_.softProfile ? sqrtf(path_.v[idx_]) : path_.v[idx_];
    float v = prm_.speed * prm_.vFull * pv * fmaxf(0.35f, cosf(alpha));
    if (e.x > 1050.0f) v *= prm_.caution;
    if (t < alertUntil_) v *= prm_.alertSpeed;
    float vMin = 0.22f * prm_.vFull;  // the slowdown factors multiply; keep the motors out of the deadband
    if (v < vMin) v = vMin;
    if (e.y < prm_.finishY && v < prm_.finishSpeed * prm_.vFull) v = prm_.finishSpeed * prm_.vFull;  // run in to the line, no crawl
    float vl = v * (1.0f - kappa * prm_.track / 2.0f), vr = v * (1.0f + kappa * prm_.track / 2.0f);
    float ul = vl / prm_.vFull, ur = vr / prm_.vFull;
    float m = fmaxf(1.0f, fmaxf(fabsf(ul), fabsf(ur)));
    duty[0] = ul / m;
    duty[1] = ur / m;
}

void Navigator::step(const math::Pose& est, float speed, const float tof[], float t, float duty[2]) {
    duty[0] = duty[1] = 0;
    if (!hasRoute()) return;
    if (!std::isfinite(est.x) || !std::isfinite(est.y) || !std::isfinite(est.th) ||
        !std::isfinite(speed) || !std::isfinite(t)) { path_.n = 0; return; }
    for (int i = 0; i < layout_.count; ++i)
        if (!std::isfinite(tof[i])) { path_.n = 0; return; }
    scan(est, tof, t);
    if (needReplan_ && t - lastPlanT_ > 0.2f) replan({est.x, est.y}, t);
    if (hasRoute()) track(est, speed, t, duty);
}

}  // namespace nav
