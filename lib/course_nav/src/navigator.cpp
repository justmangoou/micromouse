/**
 * @file    navigator.cpp
 * @brief   Navigator: random-block detection, replanning, pure-pursuit route following and drift correction.
 *          See navigator.h.
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
constexpr float SIDE_MAX_NEW_MM = 350.0f;     // a side sensor further than this cannot say where a new block is
constexpr float FRONT_SPOT_MM = 40.0f;         // how far the front sensor's spot may move and still be the same thing
constexpr float SPOT_HEADING_SIN = -0.96f;      // ... checked only while driving nearly straight down (within 16 deg)
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
constexpr float REAR_REACH_MM = 90.0f;        // backing up stops when this point behind the car's centre ...
constexpr float REAR_CLEAR_MM = 75.0f;        // ... comes this close to a mapped block
constexpr float ALIGNED_RAD = 0.3f;           // after backing up: turn until the target is this close to straight ahead
constexpr float MIN_HEADING_FACTOR = 0.35f;   // speed factor cos(alpha) never drops below this
constexpr float RIGHT_WING_X_MM = 1050.0f;    // right of this: the block can be passed, softProfile has no floor
constexpr float LEFT_MIN_FACTOR = 0.50f;       // softProfile, left of RIGHT_WING_X_MM: the route's factor is at least this
constexpr float MIN_SPEED_FRACTION = 0.22f;   // of vFull: keeps the motors out of their dead band

// Drift correction in the finish lane (keepLane)
constexpr float LANE_MIN_X_MM = 1450.0f;      // the lane between T16 and T17 ...
constexpr float LANE_MAX_Y_MM = 600.0f;       // ... from here down to the finish line
constexpr float LANE_HEADING_SIN = -0.9f;     // only while driving nearly straight down

// Reference run (baselineAt)
constexpr int NEAR_MM = 45;                   // a record this close in x and y ...
constexpr int NEAR_MRAD = 220;                // ... and in heading was taken "here"
constexpr int FRONT_KEEP = 24;                // front readings kept for the median
constexpr int FRONT_SPURIOUS_5MM = 16;        // 80 mm: nothing is this close ahead on the route

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
           std::isfinite(p.trustedCaution) && p.trustedCaution > 0 && p.trustedCaution <= 1 &&
           std::isfinite(p.alertSpeed) && p.alertSpeed > 0 && p.alertSpeed <= 1;
}

void Navigator::init(const float (*map)[3], int n) { planner_.setMap(map, n); }

bool Navigator::start(const math::Pose& pose, float t) {
    planner_.clearExtras();
    for (auto& streak : streak_) streak = 0;
    for (auto& hits : hits_) hits = 0;
    support_ = 0;
    laneShifted_ = 0.0f;
    spot_ = {0, 0};
    align_ = false;
    backing_ = 0;
    passed_ = false;
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
    // Decided before detection: a block found in this step is planned around in the next one. The firmware extends the
    // motors' command only when replanDue() said so before the step; planning without that stops the motors.
    const bool planNow = replanDue(t);
    if (!passed_) detectBlocks(pose, tof, t);  // after that, a replan could only cost time
    updateTrust(pose);
    // A new block very close: brake and back off first (NavParams::backRange). This comes before the replan, which holds
    // the last wheel command for ~0.1 s.
    // There is no sensor behind: the map says when to stop reversing.
    if (backing_ > 0 && planner_.mapClearance({pose.x - REAR_REACH_MM * cosf(pose.th),
                                              pose.y - REAR_REACH_MM * sinf(pose.th)}) < REAR_CLEAR_MM)
        backing_ = 0;
    if (backing_ > 0) {
        --backing_;
        duty[0] = duty[1] = -params_.backDuty;
        return;
    }
    if (needReplan_ && planner_.numExtras() > 0 && hits_[0] == 1 &&
        math::distance({pose.x, pose.y}, planner_.extra(0)) < params_.backRange) {
        backing_ = params_.backSteps - 1;
        align_ = true;
        hits_[0] = 2;  // back off only once for this block
        duty[0] = duty[1] = -params_.backDuty;
        return;
    }
    if (planNow) replan({pose.x, pose.y}, t);
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
    const bool covered = baselineAt(pose, shortest);  // the reference run passed here

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
        if (!isShort) continue;
        // The front sensor throws bursts of false readings. A real block stays put as the car closes in; a burst does not.
        if (fabsf(layout_.sensors[k].angle) < 0.1f && sinf(pose.th) < SPOT_HEADING_SIN) {
            const math::Vec2 from = sensorOrigin(pose, k);
            const math::Vec2 at = {from.x + z * cosf(pose.th), from.y + z * sinf(pose.th)};
            if (streak_[k] == 1 || math::distance(at, spot_) > FRONT_SPOT_MM) {
                streak_[k] = 1;
                spot_ = at;
            }
        }

        // 3. Where would the block be? The sensor sees the near face, so the centre is half a block further than the
        // reading. It sees a cone, though, not a line: of the directions in the cone, take the one that puts the block
        // closest to the route (a corner caught by a 45-degree sensor otherwise lands 10-20 cm off).
        const math::Vec2 o = sensorOrigin(pose, k);
        math::Vec2 hit = o, centre = o;
        float offRoute = 2.0e9f;
        for (int f = 0; f < layout_.rayCount; ++f) {
            const float a = pose.th + layout_.sensors[k].angle + layout_.rays[f];
            const math::Vec2 dir = {cosf(a), sinf(a)};
            const math::Vec2 c = {o.x + (z + BLOCK_HALF) * dir.x, o.y + (z + BLOCK_HALF) * dir.y};
            const float away = distanceToRoute(c);
            if (away < offRoute) {
                offRoute = away;
                centre = c;
                hit = {o.x + z * dir.x, o.y + z * dir.y};
            }
        }
        // Trust in a block already found: any short reading that lands on it, on the route or not (once the route goes
        // around the block, the new-block filters below reject further readings of it).
        if (planner_.numExtras() > 0 && support_ < 60000 && math::distance(planner_.extra(0), centre) < MERGE_RADIUS_MM)
            ++support_;

        // 4. Is the hit point a plausible new block in our way? A 45-degree sensor's far reading is not: one at 420 mm put a
        // block 18 cm from where it was, and nothing corrected it later.
        if (fabsf(layout_.sensors[k].angle) > 0.1f && z > SIDE_MAX_NEW_MM) continue;
        if (streak_[k] < d.confirmSteps || !isNewBlockHit(hit, covered)) continue;
        if (centre.x > d.maxCentreX || centre.x < d.minCentreX || centre.y < d.minCentreY) continue;
        // A side sensor catches a block's corner, which puts the estimate too far toward the forbidden middle.
        centre.x = fmaxf(centre.x, d.clampCentreX);
        if (d.centreReach < 1.0e8f && !nearRouteAhead(centre, d.centreReach)) continue;  // not blocking the way
        // Something right beside the car: a side sensor this close places the block far better than the guess above, which
        // pulls it toward the route (a block just beside the route was put 10 cm low and driven into).
        if (planner_.numExtras() == 0)
            for (int j = 0; j < layout_.count; ++j) {
                const float angle = layout_.sensors[j].angle;
                if (fabsf(angle) < 0.1f || streak_[j] == 0 || tof[j] >= 80.0f) continue;
                const math::Vec2 from = sensorOrigin(pose, j);
                const math::Vec2 dir = {cosf(pose.th + angle), sinf(pose.th + angle)};
                const math::Vec2 side = {from.x + (tof[j] + BLOCK_HALF) * dir.x, from.y + (tof[j] + BLOCK_HALF) * dir.y};
                const math::Vec2 face = {from.x + tof[j] * dir.x, from.y + tof[j] * dir.y};
                if (math::distance(side, centre) < 200.0f && planner_.mapClearance(face) > 80.0f && side.x >= d.clampCentreX &&
                    side.x <= d.maxCentreX && side.y >= d.minCentreY)
                    centre = side;
            }
        addOrMergeBlock(centre, t);
    }
}

// Latches once the block is well supported and the car is past it in the right wing. A run where the block is never
// seen, or seen only briefly, keeps the cautious speed.
void Navigator::updateTrust(const math::Pose& pose) {
    if (passed_ || params_.trustSupport <= 0 || planner_.numExtras() == 0 || support_ < params_.trustSupport) return;
    passed_ = pose.x > RIGHT_WING_X_MM && pose.y < planner_.extra(0).y - params_.trustPassMm;
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

// What each sensor read near this pose in the reference run: its shortest reading, in mm. The front sensor throws random
// 30-80 mm readings on open floor, and one of them in the reference would hide a real block there: where its shortest is
// that small, its median is used instead. A single stray further out is enough too, so for the front sensor the second
// shortest reading counts. False when fewer than two records are near: then the map is the reference.
// Integer work only: it scans the whole log.
bool Navigator::baselineAt(const math::Pose& pose, float shortest[3]) const {
    if (!baseline_.records) return false;
    const int px = (int)pose.x, py = (int)pose.y, pth = (int)(pose.th * 1000.0f);
    const auto at = [](const uint8_t* p) { return (int)(int16_t)(p[0] | (p[1] << 8)); };
    uint8_t front[FRONT_KEEP];  // front readings in rising order
    int found = 0, kept = 0, low[3] = {255, 255, 255}, low2[3] = {255, 255, 255};
    const uint8_t* r = baseline_.records;
    for (int i = 0; i < baseline_.count; ++i, r += Baseline::RECORD_BYTES) {
        if (abs(at(r + 2) - px) > NEAR_MM || abs(at(r + 4) - py) > NEAR_MM) continue;
        int dth = (at(r + 6) - pth) % 6283;
        if (dth > 3141) dth -= 6283;
        if (dth < -3141) dth += 6283;
        if (abs(dth) > NEAR_MRAD) continue;
        ++found;
        for (int k = 0; k < 3; ++k) {
            const int v = r[13 + k];
            if (v < low[k]) { low2[k] = low[k]; low[k] = v; } else if (v < low2[k]) low2[k] = v;
        }
        if (kept < FRONT_KEEP) {
            int j = kept++;
            for (; j > 0 && front[j - 1] > r[14]; --j) front[j] = front[j - 1];
            front[j] = r[14];
        }
    }
    if (found < 2) return false;
    for (int k : {1}) low[k] = low2[k];  // one stray reading does not set the expectation
    if (low[1] < FRONT_SPURIOUS_5MM) low[1] = front[kept / 2];
    for (int k = 0; k < 3; ++k) shortest[k] = low[k] * 5.0f;
    return true;
}

bool Navigator::isNewBlockHit(math::Vec2 hit, bool baselineCovered) const {
    const DetectParams& d = params_.detect;
    if (hit.x < d.minHitX) return false;                                    // the mapped part of the arena
    if (d.pathReach < 1.0e8f && !nearRouteAhead(hit, d.pathReach)) return false;         // not in our way: no reason to replan
    if (baselineCovered) return true;                                       // the baseline already cancels walls and blocks
    if (wallClearance(hit) < d.wallMargin) return false;                    // a wall, seen through pose error
    if (planner_.mapClearance(hit) < d.mapMargin) return false;             // a mapped block, seen through pose/map error
    return true;
}

float Navigator::distanceToRoute(math::Vec2 p) const {
    float best = 1.0e9f;
    if (path_.n < 2) return best;
    const float end = path_.s[idx_] + params_.detect.pathAhead;
    for (int i = idx_; i < path_.n && path_.s[i] <= end; ++i) best = fminf(best, math::distance(path_.p[i], p));
    return best;
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

    if (align_ && fabsf(alpha) < ALIGNED_RAD) align_ = false;
    if (align_ || fabsf(alpha) > PIVOT_ALPHA_RAD) {  // the target is behind or far to the side: turn on the spot first
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
    float baseFactor = path_.v[idx_];
    if (p.softProfile && pose.x <= RIGHT_WING_X_MM) {
        baseFactor = fmaxf(LEFT_MIN_FACTOR, baseFactor);
    }
    const float routeFactor = p.softProfile ? sqrtf(baseFactor) : baseFactor;
    float v = p.speed * p.vFull * routeFactor * fmaxf(MIN_HEADING_FACTOR, cosf(alpha));
    if (pose.x > p.cautionX) v *= passed_ ? p.trustedCaution : p.caution;
    if (t < alertUntil_) v *= p.alertSpeed;
    const float vMin = MIN_SPEED_FRACTION * p.vFull;
    if (v < vMin) v = vMin;
    if (pose.y < p.finishY && v < p.finishSpeed * p.vFull) v = p.finishSpeed * p.vFull;  // no crawling onto the line
    return v;
}

// Driving down (-y), the right-hand sensor looks toward -x (T16) and the left-hand one toward +x (T17). Something close on
// one side means the car is nearer to it than the estimate says.
bool Navigator::keepLane(math::Pose& pose, const float tof[]) {
    const NavParams& p = params_;
    if (p.laneClose <= 0 || planner_.numExtras() == 0 || pose.x < LANE_MIN_X_MM || pose.y > LANE_MAX_Y_MM ||
        pose.y < 0.0f || sinf(pose.th) > LANE_HEADING_SIN)
        return false;
    const bool right = tof[0] < p.laneClose, left = tof[layout_.count - 1] < p.laneClose;
    if (right == left) return false;
    const float step = right ? -p.laneStep : p.laneStep;  // the estimate moves toward what is close
    if (fabsf(laneShifted_ + step) > p.laneLimit) return false;
    laneShifted_ += step;
    pose.x += step;
    return true;
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
