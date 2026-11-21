/**
 * @file    navigator.cpp
 * @brief   Navigator: random-block detection, replanning and pure-pursuit route following. See navigator.h.
 */
#include "navigator.h"

#include <cmath>
#include <cstdlib>

namespace nav {

namespace {

// Random-block detection
constexpr float ALERT_DURATION_S = 0.8f;      // slow down this long after a new block is found
constexpr float SHORT_MARGIN_MM = 40.0f;      // fixed part of "short" against the map
constexpr float MAX_BLOCK_RANGE_MM = 800.0f;  // longer readings are too far to matter (and less reliable)
constexpr float MIN_RANGE_MM = 40.0f;         // the sensors clamp at 30 mm: shorter readings carry no position
constexpr float WALL_FAN_STEP_RAD = 0.175f;   // walls are checked at -2..+2 steps (+-20 deg) around the sensor axis:
constexpr int WALL_FAN_HALF_STEPS = 2;        // wider than the sensor's field of view, so pose error cannot hide a wall
constexpr float MERGE_RADIUS_MM = 180.0f;     // two detections this close are the same block (the centre estimate
                                              // moves a lot with the viewing angle)
constexpr int REFINE_HITS = 6;                // the first detections of a block move its estimate ...
constexpr float REFINE_MIN_SHIFT_MM = 20.0f;  // ... when they disagree by more than this ...
constexpr float REFINE_WEIGHT = 0.3f;         // ... by this fraction of the difference

// Route following
constexpr int NEAREST_SEARCH_POINTS = 80;     // the nearest route point is searched this far ahead
constexpr float OFF_ROUTE_MM = 250.0f;        // further than this from the route: plan again from here
constexpr float OFF_ROUTE_REPLAN_S = 0.5f;    // ... at most this often
constexpr float PIVOT_ALPHA_RAD = 1.2f;       // target more than this far to the side: turn on the spot first
constexpr float PIVOT_DUTY = 0.45f;
constexpr float MIN_HEADING_FACTOR = 0.35f;   // speed factor cos(alpha) never drops below this
constexpr float CAUTION_X_MM = 1050.0f;       // params.caution applies right of this
constexpr float MIN_SPEED_FRACTION = 0.22f;   // of vFull: keeps the motors out of their dead band

// Pose correction (localize)
constexpr float LOC_MAX_MM = 220.0f;          // beyond this the floor and the sensor's compression spoil the reading
constexpr float LOC_GATE_MM = 70.0f;          // a reading further than this from the prediction is not a mapped block
constexpr float LOC_ERR_CAP_MM = 60.0f;
constexpr float LOC_STEP_MM = 20.0f, LOC_STEP_RAD = 0.035f;
constexpr float LOC_LIMIT_MM = 120.0f, LOC_LIMIT_RAD = 0.17f;
constexpr float LOC_PERIOD_S = 0.1f;

}  // namespace

bool Navigator::configurationValid() const {
    const NavParams& p = params_;
    return layout_.valid() && std::isfinite(p.vFull) && p.vFull > 0 &&
           std::isfinite(p.track) && p.track > 0 &&
           std::isfinite(p.speed) && p.speed > 0 && p.speed <= 1 &&
           std::isfinite(p.lookahead) && p.lookahead > 0 &&
           std::isfinite(p.caution) && p.caution > 0 && p.caution <= 1 &&
           std::isfinite(p.alertSpeed) && p.alertSpeed > 0 && p.alertSpeed <= 1;
}

void Navigator::init(const float (*map)[3], int n) { planner_.setMap(map, n); }

bool Navigator::start(const math::Pose& pose, float t) {
    planner_.clearExtras();
    for (auto& streak : streak_) streak = 0;
    for (auto& hits : hits_) hits = 0;
    alertUntil_ = -1.0f;
    lastLocalizeT_ = -1.0f;
    locOffset_ = {0, 0};
    locTurn_ = 0.0f;
    replans_ = 0;
    path_.n = 0;
    return replan({pose.x, pose.y}, t);
}

bool Navigator::reposition(const math::Pose& pose, float t) { return replan({pose.x, pose.y}, t); }

bool Navigator::replan(math::Vec2 from, float t) {
    path_.n = 0;
    bool ok = configurationValid() && std::isfinite(from.x) && std::isfinite(from.y) && std::isfinite(t) &&
              planner_.plan(from, path_) && path_.n > 1;
    if (!ok && planner_.numExtras() > 0 && configurationValid()) {
        // The detected blocks closed every route. They are estimates, so the mapped course is a better bet than stopping.
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

void Navigator::step(const math::Pose& pose, float speed, const float tof[], float t, float duty[2]) {
    duty[0] = duty[1] = 0;
    if (!hasRoute()) return;
    if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.th) || !std::isfinite(speed) ||
        !std::isfinite(t)) {
        path_.n = 0;
        return;
    }
    for (int i = 0; i < layout_.count; ++i)
        if (!std::isfinite(tof[i])) {
            path_.n = 0;
            return;
        }
    detectBlocks(pose, tof, t);
    if (replanDue(t)) replan({pose.x, pose.y}, t);
    if (hasRoute()) track(pose, speed, t, duty);
}

math::Vec2 Navigator::sensorOrigin(const math::Pose& pose, int sensor) const {
    const float c = cosf(pose.th), s = sinf(pose.th);
    const tof::Mount& m = layout_.sensors[sensor];
    return {pose.x + c * m.x - s * m.y, pose.y + s * m.x + c * m.y};
}

// ---------------------------------------------------------------------------------------------------------------------
// Random-block detection
// ---------------------------------------------------------------------------------------------------------------------

void Navigator::detectBlocks(const math::Pose& pose, const float tof[], float t) {
    const DetectParams& d = params_.detect;
    float shortest[3];
    const bool covered = baselineAt(pose, shortest);  // clean runs were recorded here

    for (int k = 0; k < layout_.count; k++) {
        // 1. Is this reading short?
        const float z = tof[k];
        bool isShort;
        if (covered) {
            isShort = z < shortest[k] - d.baselineMargin && z < MAX_BLOCK_RANGE_MM && z >= MIN_RANGE_MM;
        } else {
            const float expected = predictedRange(pose, k);
            isShort = z < expected - (SHORT_MARGIN_MM + d.rangeTol * expected) && z < MAX_BLOCK_RANGE_MM;
        }

        // 2. Has it been short long enough?
        streak_[k] = isShort ? (uint8_t)(streak_[k] < 250 ? streak_[k] + 1 : 250) : 0;
        if (streak_[k] < d.confirmSteps) continue;

        // 3. Is the hit point a plausible new block in our way?
        const math::Vec2 o = sensorOrigin(pose, k);
        const float a = pose.th + layout_.sensors[k].angle;
        const math::Vec2 dir = {cosf(a), sinf(a)};
        const math::Vec2 hit = {o.x + z * dir.x, o.y + z * dir.y};
        if (!isNewBlockHit(hit, covered)) continue;

        // 4. The sensor sees the near face: the centre is half a block further along the ray.
        const math::Vec2 centre = {o.x + (z + BLOCK_HALF) * dir.x, o.y + (z + BLOCK_HALF) * dir.y};
        if (centre.x > d.maxCentreX || centre.y < d.minCentreY) continue;
        addOrMergeBlock(centre, t);
    }
}

// What the sensor should read according to the map: the nearest mapped block over the sensor's rays, or the nearest
// wall over a wider fan.
float Navigator::predictedRange(const math::Pose& pose, int sensor) const {
    const math::Vec2 o = sensorOrigin(pose, sensor);
    const float axis = pose.th + layout_.sensors[sensor].angle;
    float expected = layout_.maxMm;
    for (int f = 0; f < layout_.rayCount; f++) {
        const float a = axis + layout_.rays[f];
        const math::Vec2 dir = {cosf(a), sinf(a)};
        for (int b = 0; b < planner_.numBlocks(); b++) expected = fminf(expected, rayBlock(o, dir, planner_.block(b), layout_.maxMm));
    }
    for (int f = -WALL_FAN_HALF_STEPS; f <= WALL_FAN_HALF_STEPS; f++) {
        const float a = axis + f * WALL_FAN_STEP_RAD;
        expected = fminf(expected, rayWalls(o, {cosf(a), sinf(a)}, layout_.maxMm));
    }
    return expected;
}

// The shortest clean-run reading of each sensor near this pose (x, y within one cell, heading within one bin). False when
// fewer than two baseline cells are near: then the map is the reference.
bool Navigator::baselineAt(const math::Pose& pose, float shortest[3]) const {
    if (!baseline_.entries) return false;
    const int cx = (int)lroundf(pose.x / baseline_.cell);
    const int cy = (int)lroundf(pose.y / baseline_.cell);
    const int bin = (int)lroundf(pose.th / baseline_.bin);
    const int bins = (int)lroundf(6.2831853f / baseline_.bin);
    int found = 0;
    for (int k = 0; k < 3; ++k) shortest[k] = 1.0e9f;
    for (int i = 0; i < baseline_.count; ++i) {
        const BaselineEntry& e = baseline_.entries[i];
        int dBin = e.bin - bin;
        while (dBin > bins / 2) dBin -= bins;
        while (dBin < -bins / 2) dBin += bins;
        if (abs(e.cx - cx) > 1 || abs(e.cy - cy) > 1 || abs(dBin) > 1) continue;
        ++found;
        for (int k = 0; k < 3; ++k) shortest[k] = fminf(shortest[k], e.range5mm[k] * 5.0f);
    }
    return found >= 2;
}

bool Navigator::isNewBlockHit(math::Vec2 hit, bool baselineCovered) const {
    const DetectParams& d = params_.detect;
    if (hit.x < d.minHitX) return false;                                    // the mapped part of the arena
    if (d.pathReach < 1.0e8f && !nearRouteAhead(hit)) return false;         // not in our way: no reason to replan
    if (baselineCovered) return true;                                       // the baseline already cancels walls and blocks
    if (wallClearance(hit) < d.wallMargin) return false;                    // a wall, seen through pose error
    if (planner_.mapClearance(hit) < d.mapMargin) return false;             // a mapped block, seen through pose/map error
    return true;
}

bool Navigator::nearRouteAhead(math::Vec2 p) const {
    if (path_.n < 2) return false;
    const DetectParams& d = params_.detect;
    const float s0 = path_.s[idx_];
    for (int i = idx_; i < path_.n && path_.s[i] <= s0 + d.pathAhead; ++i)
        if (math::distance(path_.p[i], p) <= d.pathReach) return true;
    return false;
}

void Navigator::addOrMergeBlock(math::Vec2 centre, float t) {
    for (int i = 0; i < planner_.numExtras(); i++) {
        const math::Vec2 known = planner_.extra(i);
        const float dist = math::distance(known, centre);
        if (dist >= MERGE_RADIUS_MM) continue;
        // Another detection of a block we already have: the first few refine its position.
        if (hits_[i] < 250) hits_[i]++;
        if (dist > REFINE_MIN_SHIFT_MM && hits_[i] < REFINE_HITS) {
            planner_.setExtra(i, math::lerp(known, centre, REFINE_WEIGHT));
            needReplan_ = true;
        }
        return;
    }
    if (planner_.numExtras() >= params_.detect.maxBlocks) return;  // the course has no more random blocks
    const int i = planner_.addExtra(centre);
    if (i < 0) return;
    hits_[i] = 1;
    needReplan_ = true;
    alertUntil_ = t + ALERT_DURATION_S;
}

// ---------------------------------------------------------------------------------------------------------------------
// Route following: pure pursuit
// ---------------------------------------------------------------------------------------------------------------------

void Navigator::track(const math::Pose& pose, float speed, float t, float duty[2]) {
    // Where on the route are we?
    const math::Vec2 pos = {pose.x, pose.y};
    const int end = idx_ + NEAREST_SEARCH_POINTS < path_.n ? idx_ + NEAREST_SEARCH_POINTS : path_.n;
    float nearest = 1e9f;
    int nearestIdx = idx_;
    for (int i = idx_; i < end; i++) {
        const float dist = math::distance(path_.p[i], pos);
        if (dist < nearest) {
            nearest = dist;
            nearestIdx = i;
        }
    }
    idx_ = nearestIdx;
    if (nearest > OFF_ROUTE_MM && t - lastPlanT_ > OFF_ROUTE_REPLAN_S) {
        if (!replan(pos, t)) return;
    }

    // The lookahead point: further ahead at speed.
    const float look = params_.lookahead + params_.lookaheadGain * fabsf(speed);
    int j = idx_;
    while (j < path_.n - 1 && path_.s[j] < path_.s[idx_] + look) j++;
    const math::Vec2 target = path_.p[j];
    const float dist = fmaxf(math::distance(pos, target), 10.0f);
    const float alpha = math::wrap(atan2f(target.y - pose.y, target.x - pose.x) - pose.th);
    lastAlpha_ = alpha;

    if (fabsf(alpha) > PIVOT_ALPHA_RAD) {  // the target is behind or far to the side: turn on the spot first
        const float sign = alpha > 0 ? 1.0f : -1.0f;
        duty[0] = -PIVOT_DUTY * sign;
        duty[1] = PIVOT_DUTY * sign;
        return;
    }

    // Pure pursuit: drive the circle arc through the lookahead point (curvature kappa), at the target speed.
    const float kappa = 2.0f * sinf(alpha) / dist;
    const float v = targetSpeed(pose, alpha, t);
    const float vLeft = v * (1.0f - kappa * params_.track / 2.0f), vRight = v * (1.0f + kappa * params_.track / 2.0f);
    const float uLeft = vLeft / params_.vFull, uRight = vRight / params_.vFull;
    const float scale = fmaxf(1.0f, fmaxf(fabsf(uLeft), fabsf(uRight)));  // keep both wheels within full duty
    duty[0] = uLeft / scale;
    duty[1] = uRight / scale;
}

// Cruise speed, times the route's speed factor here (slow near blocks, edges and bends), times cos(alpha) (slow while
// turning toward the route), with the course's slow-downs and a minimum.
float Navigator::targetSpeed(const math::Pose& pose, float alpha, float t) const {
    const NavParams& p = params_;
    const float routeFactor = p.softProfile ? sqrtf(path_.v[idx_]) : path_.v[idx_];
    float v = p.speed * p.vFull * routeFactor * fmaxf(MIN_HEADING_FACTOR, cosf(alpha));
    if (pose.x > CAUTION_X_MM) v *= p.caution;
    if (t < alertUntil_) v *= p.alertSpeed;
    const float vMin = MIN_SPEED_FRACTION * p.vFull;
    if (v < vMin) v = vMin;
    if (pose.y < p.finishY && v < p.finishSpeed * p.vFull) v = p.finishSpeed * p.vFull;  // no crawling onto the line
    return v;
}

// ---------------------------------------------------------------------------------------------------------------------
// Pose correction (localize)
// ---------------------------------------------------------------------------------------------------------------------

// Predicted range for pose correction: the nearest mapped block within 350 mm over the sensor's centre and outer rays.
static float localizeRange(const Planner& planner, const tof::Layout& layout, math::Vec2 origin, float axis) {
    float best = layout.maxMm;
    const int stride = layout.rayCount > 2 ? layout.rayCount / 2 : 1;
    for (int f = 0; f < layout.rayCount; f += stride) {
        const float a = axis + layout.rays[f];
        const math::Vec2 dir = {cosf(a), sinf(a)};
        for (int b = 0; b < planner.numBlocks(); b++) {
            const Block& block = planner.block(b);
            const float dx = block.x - origin.x, dy = block.y - origin.y;
            if (dx * dx + dy * dy > 350.0f * 350.0f) continue;
            best = fminf(best, rayBlock(origin, dir, block, layout.maxMm));
        }
    }
    return best;
}

// Sum of squared range errors (each capped) over the sensors in `use`.
float Navigator::localizeCost(const math::Pose& pose, const float tof[], const bool use[]) const {
    float cost = 0;
    for (int k = 0; k < layout_.count; k++) {
        if (!use[k]) continue;
        const float r = localizeRange(planner_, layout_, sensorOrigin(pose, k), pose.th + layout_.sensors[k].angle);
        float err = tof[k] - r;
        if (r > LOC_MAX_MM + LOC_ERR_CAP_MM) err = LOC_ERR_CAP_MM;  // the map says nothing there
        err = fmaxf(-LOC_ERR_CAP_MM, fminf(LOC_ERR_CAP_MM, err));
        cost += err * err;
    }
    return cost;
}

bool Navigator::localize(math::Pose& pose, const float tof[], float t) {
    if (t - lastLocalizeT_ < LOC_PERIOD_S) return false;
    lastLocalizeT_ = t;

    // Use only the sensors whose reading the map explains (a mapped block close by).
    bool use[tof::MAX_SENSORS] = {false};
    int used = 0;
    for (int k = 0; k < layout_.count; k++) {
        const float z = tof[k];
        if (!(z >= layout_.minMm && z <= LOC_MAX_MM)) continue;
        const float r = localizeRange(planner_, layout_, sensorOrigin(pose, k), pose.th + layout_.sensors[k].angle);
        if (r > LOC_MAX_MM + 40.0f || fabsf(z - r) > LOC_GATE_MM) continue;
        use[k] = true;
        used++;
    }
    if (!used) return false;

    const float base = localizeCost(pose, tof, use);
    if (base < 100.0f) return false;  // already within a few mm

    // Try the 26 neighbouring poses one step away; keep the best if it is a clear improvement.
    float best = base;
    math::Pose bestPose = pose;
    for (int ix = -1; ix <= 1; ix++)
        for (int iy = -1; iy <= 1; iy++)
            for (int it = -1; it <= 1; it++) {
                if (!ix && !iy && !it) continue;
                const math::Pose c = {pose.x + ix * LOC_STEP_MM, pose.y + iy * LOC_STEP_MM, pose.th + it * LOC_STEP_RAD};
                const float nx = locOffset_.x + ix * LOC_STEP_MM, ny = locOffset_.y + iy * LOC_STEP_MM;
                const float nt = locTurn_ + it * LOC_STEP_RAD;
                if (fabsf(nx) > LOC_LIMIT_MM || fabsf(ny) > LOC_LIMIT_MM || fabsf(nt) > LOC_LIMIT_RAD) continue;
                const float cost = localizeCost(c, tof, use) + (abs(ix) + abs(iy) + abs(it)) * 25.0f;  // smaller steps win ties
                if (cost < best) {
                    best = cost;
                    bestPose = c;
                }
            }
    if (best >= base * 0.85f) return false;
    locOffset_.x += bestPose.x - pose.x;
    locOffset_.y += bestPose.y - pose.y;
    locTurn_ += bestPose.th - pose.th;
    pose = bestPose;
    return true;
}

}  // namespace nav
