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
// Across the pit: before the bend by T14 the right 45-degree sensor looks down the first corridor (x 1200-1400). A block
// there reads far shorter than run 1 did; it is then put in the corridor and the route goes round over T15 at once,
// instead of driving down the corridor to find it close up. Floor logs: 5 of 8 corridor blocks seen from x 1030-1050,
// none on 506 clean pairs or 9 runs with the block elsewhere. Simulated (block placed where seen): 90th percentile
// time 7.3 -> 5.4 s, T15 touches 7 -> 1 of 2634.
constexpr float ACROSS_X0_MM = 990.0f, ACROSS_X1_MM = 1120.0f, ACROSS_MIN_Y_MM = 700.0f;
constexpr float ACROSS_TH0 = -0.95f, ACROSS_TH1 = -0.2f;   // heading toward the bend
constexpr float ACROSS_MIN_MM = 380.0f, ACROSS_MAX_MM = 820.0f;
constexpr float ACROSS_MARGIN_MM = 250.0f;   // shorter than run 1's shortest from here by this much ...
constexpr int ACROSS_STEPS = 2;              // ... this many steps in a row
constexpr float ACROSS_HIT_X0 = 1100.0f, ACROSS_HIT_X1 = 1400.0f;  // the centre ray's end (floor logs: 1120-1340)
constexpr float ACROSS_MAX_Y_MM = 470.0f;    // higher up, the front sensor finds it in time (and the route passes there)
constexpr float ACROSS_FRONT_CLEAR_MM = 300.0f;  // and nothing close ahead: a block at the bend itself is the front's
constexpr float ACROSS_REPLACE_X_MM = 1250.0f;   // a block found up close before here replaces the guess
constexpr float ACROSS_X_MM = 1290.0f;       // put in the corridor's middle ...
constexpr float ACROSS_MIN_Y_BLOCK_MM = 100.0f;  // ... at the seen height, at least this
constexpr float FRONT_STRAIGHT_MARGIN_MM = 60.0f;  // "short" for the front sensor while driving straight down
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

// The T15-T16 gap (Planner::gate)
constexpr float GATE_STOP_MM = 15.0f;         // turn on the spot once this close above the gap's centre line
constexpr float GATE_BRAKE = 2.5f;            // approaching it: at most this x the distance left, mm/s per mm
constexpr float GATE_ALIGNED_RAD = 0.05f;     // turned far enough ...
constexpr int GATE_SETTLE_STEPS = 2;          // ... for this many steps
constexpr float GATE_TURN_GAIN = 0.3f;        // wheel command per rad of heading still to turn
constexpr float GATE_TURN_MIN = 0.08f;
constexpr int GATE_LOOK_STEPS = 6;            // standing still, the 45-degree sensors measure T15 against T16
constexpr float GATE_SEE_MM = 250.0f;
constexpr float GATE_MAX_SHIFT_MM = 60.0f;
constexpr float GATE_SHIFT_STEP_MM = 4.0f;
constexpr float GATE_EDGE_FAR_MM = 300.0f;    // the straight-right reading jumps past this: T16's far edge (x 1500) just went by
constexpr float GATE_EDGE_SPAN_MM = 70.0f;    // ... trusted only this close to where the map puts that moment
constexpr float TOF_HALF_FOV_TAN = 0.21f;     // tan(12 deg): the edge of the sensor's cone leaves the face first
constexpr float GATE_LOOKAHEAD_MM = 70.0f;    // through the gap and round T16: a short lookahead cuts the bend less ...
constexpr float GATE_SPEED_MM_S = 320.0f;     // ... and at most this speed until heading down the lane
// The straight-right sensor against T16's top face, in the gap
constexpr float SIDE_SEE_MM = 160.0f;         // a face this close ...
constexpr float SIDE_GATE_MM = 45.0f;         // ... and the reading within this of the map's
constexpr float SIDE_GAIN = 0.3f;             // the pose moves this share of the difference ...
constexpr float SIDE_STEP_MM = 4.0f;          // ... at most this much per step
constexpr float SIDE_LIMIT_MM = 70.0f;        // ... and in all, per run

// Drift correction in the finish lane (keepLane)
constexpr float LANE_MIN_X_MM = 1450.0f;      // the lane between T16 and T17 ...
constexpr float LANE_MAX_Y_MM = 600.0f;       // ... from here down to the finish line
constexpr float LANE_HEADING_SIN = -0.9f;     // only while driving nearly straight down

// Reference run (baselineAt)
constexpr int NEAR_MM = 45;                   // a record this close in x and y ...
constexpr int NEAR_MRAD = 220;                // ... and in heading was taken "here"
constexpr int BEND_NEAR_MM = 20;                // ... but for the front sensor in the bend by T14, this close
constexpr int BEND_X0_MM = 1050, BEND_X1_MM = 1200, BEND_MIN_Y_MM = 650;
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
    gate_ = Gate::Off;
    acrossStreak_ = 0;
    acrossOnly_ = false;
    sideShifted_ = 0.0f;
    lastRight90_ = 1200.0f;
    gateEdge_ = false;
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
    if (gate_ != Gate::Off && path_.n > 1) {  // committed to the gap: its route is laid out to the finish
        needReplan_ = false;
        lastPlanT_ = t;
        return true;
    }
    path_.n = 0;
    bool ok = configurationValid() && std::isfinite(from.x) && std::isfinite(from.y) && std::isfinite(t) &&
              planner_.plan(from, path_) && path_.n > 1;
    if (!ok && planner_.numExtras() > 0 && configurationValid()) {
        // The detected blocks closed every route. They are estimates, so the mapped course is a better bet than stopping.
        planner_.clearExtras();
        ok = planner_.plan(from, path_) && path_.n > 1;
    }
    if (!ok) path_.n = 0;
    gate_ = !ok || !planner_.gated() ? Gate::Off : GATE_TURN_ON_SPOT ? Gate::Approach : Gate::Through;
    gateCorner_ = 0;
    while (gate_ == Gate::Approach && gateCorner_ + 1 < path_.n && fabsf(path_.p[gateCorner_ + 1].x - path_.p[0].x) < 1.0f)
        ++gateCorner_;
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
    if (gate_ == Gate::Approach && pose.y <= GATE_Y + GATE_STOP_MM) {
        gate_ = Gate::Turn;
        gateSteps_ = 0;
    }
    if (gate_ == Gate::Turn || gate_ == Gate::Look) {
        gateTurn(pose, duty);
        return;
    }
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
            // Front sensor driving straight down a corridor: its reading is steady, a smaller margin finds a block sooner.
            const bool straight = fabsf(layout_.sensors[k].angle) < 0.1f && sinf(pose.th) < SPOT_HEADING_SIN;
            isShort = z < shortest[k] - (straight ? FRONT_STRAIGHT_MARGIN_MM : d.baselineMargin) && z < MAX_BLOCK_RANGE_MM &&
                      z >= MIN_RANGE_MM;
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
        addOrMergeBlock(centre, t, false, pose.x);
    }

    if (covered && planner_.numExtras() == 0) {
        const float z = tof[0];
        const bool zone = pose.x > ACROSS_X0_MM && pose.x < ACROSS_X1_MM && pose.y > ACROSS_MIN_Y_MM && pose.th > ACROSS_TH0 &&
                          pose.th < ACROSS_TH1;
        const math::Vec2 o = sensorOrigin(pose, 0);
        const float a = pose.th + layout_.sensors[0].angle;
        const math::Vec2 hit = {o.x + z * cosf(a), o.y + z * sinf(a)};  // the cone's centre: y fits, x lands short
        const bool seen = zone && z > ACROSS_MIN_MM && z < ACROSS_MAX_MM && shortest[0] - z > ACROSS_MARGIN_MM &&
                          hit.x > ACROSS_HIT_X0 && hit.x < ACROSS_HIT_X1 && hit.y < ACROSS_MAX_Y_MM &&
                          tof[1] > ACROSS_FRONT_CLEAR_MM;
        acrossStreak_ = seen ? (uint8_t)(acrossStreak_ + 1) : 0;
        if (acrossStreak_ >= ACROSS_STEPS) addOrMergeBlock({ACROSS_X_MM, fmaxf(hit.y, ACROSS_MIN_Y_BLOCK_MM)}, t, true, pose.x);
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
    const bool inBend = px > BEND_X0_MM && px < BEND_X1_MM && py > BEND_MIN_Y_MM;
    int found = 0, kept = 0, low[3] = {255, 255, 255}, low2[3] = {255, 255, 255};
    const uint8_t* r = baseline_.records;
    for (int i = 0; i < baseline_.count; ++i, r += Baseline::RECORD_BYTES) {
        if (abs(at(r + 2) - px) > NEAR_MM || abs(at(r + 4) - py) > NEAR_MM) continue;
        int dth = (at(r + 6) - pth) % 6283;
        if (dth > 3141) dth -= 6283;
        if (dth < -3141) dth += 6283;
        if (abs(dth) > NEAR_MRAD) continue;
        ++found;
        // In the bend down into the first corridor run 1's front reading changes fast (T15's corner comes and goes): only
        // records right here count for it, or one from a few cm on hides a block at the bend (a floor run hit one).
        const bool frontHere = !inBend || (abs(at(r + 2) - px) <= BEND_NEAR_MM && abs(at(r + 4) - py) <= BEND_NEAR_MM);
        for (int k = 0; k < 3; ++k) {
            if (k == 1 && !frontHere) continue;
            const int v = r[13 + k];
            if (v < low[k]) { low2[k] = low[k]; low[k] = v; } else if (v < low2[k]) low2[k] = v;
        }
        if (kept < FRONT_KEEP && frontHere) {
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

void Navigator::addOrMergeBlock(math::Vec2 centre, float t, bool across, float x) {
    for (int i = 0; i < planner_.numExtras(); i++) {
        const math::Vec2 known = planner_.extra(i);
        const float dist = math::distance(known, centre);
        if (dist >= MERGE_RADIUS_MM) continue;
        // Another detection of a block we already have: the first few refine its position.
        if (!across) acrossOnly_ = false;
        if (hits_[i] < 250) hits_[i]++;
        if (dist > REFINE_MIN_SHIFT_MM && hits_[i] < REFINE_HITS) {
            planner_.setExtra(i, math::lerp(known, centre, REFINE_WEIGHT));
            needReplan_ = true;
        }
        return;
    }
    // The across-the-pit look only guessed: a block then found up close before the corridor is the real one.
    if (planner_.numExtras() > 0 && acrossOnly_ && !across && x < ACROSS_REPLACE_X_MM) {
        planner_.setExtra(0, centre);
        acrossOnly_ = false;
        hits_[0] = 1;
        needReplan_ = true;
        alertUntil_ = t + ALERT_DURATION_S;
        return;
    }
    if (planner_.numExtras() >= params_.detect.maxBlocks) return;  // the course has no more random blocks
    const int i = planner_.addExtra(centre);
    if (i < 0) return;
    hits_[i] = 1;
    acrossOnly_ = across;
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
    const bool gapBend = gate_ == Gate::Through && sinf(pose.th) > LANE_HEADING_SIN;
    const float look = gapBend ? GATE_LOOKAHEAD_MM : params_.lookahead + params_.lookaheadGain * fabsf(speed);
    int j = idx_;
    while (j < path_.n - 1 && path_.s[j] < path_.s[idx_] + look) j++;
    if (gate_ == Gate::Approach && j > gateCorner_) j = gateCorner_;  // stop at the gap, do not cut into it
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
    // Keep both wheels within full speed; the boosted left side may exceed vFull by that factor (the motors have far more).
    const float limit = pose.x <= params_.leftBoostX                                 ? params_.leftBoost
                        : planner_.numExtras() > 0 && pose.x > RIGHT_WING_X_MM ? params_.foundBoost
                                                                               : 1.0f;
    const float scale = fmaxf(1.0f, fmaxf(fabsf(uLeft), fabsf(uRight)) / limit);
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
    if (pose.x <= p.leftBoostX) v *= p.leftBoost;
    // There is only one block: once it is on the map there is nothing left to slow down for.
    if (pose.x > p.cautionX) v *= passed_ || planner_.numExtras() > 0 ? p.trustedCaution : p.caution;
    if (planner_.numExtras() > 0 && pose.x > RIGHT_WING_X_MM) v *= p.foundBoost;
    if (t < alertUntil_) v *= p.alertSpeed;
    if (gate_ == Gate::Approach) v = fminf(v, GATE_BRAKE * (pose.y - GATE_Y));
    if (gate_ == Gate::Through && sinf(pose.th) > LANE_HEADING_SIN) v = fminf(v, GATE_SPEED_MM_S);
    const float vMin = MIN_SPEED_FRACTION * p.vFull;
    if (v < vMin) v = vMin;
    if (pose.y < p.finishY && v < p.finishSpeed * p.vFull) v = p.finishSpeed * p.vFull;  // no crawling onto the line
    return v;
}

// What a sensor should read off the mapped blocks within 300 mm (no walls): cheap enough for a search.
float Navigator::nearRange(const math::Pose& pose, int sensor) const {
    const math::Vec2 o = sensorOrigin(pose, sensor);
    float best = layout_.maxMm;
    for (int b = 0; b < planner_.numBlocks(); b++) {
        const Block& blk = planner_.block(b);
        if (math::distance(o, {blk.x, blk.y}) > 300.0f) continue;
        for (int f = 0; f < layout_.rayCount; f++) {
            const float a = pose.th + layout_.sensors[sensor].angle + layout_.rays[f];
            best = fminf(best, rayBlock(o, {cosf(a), sinf(a)}, blk, layout_.maxMm));
        }
    }
    return best;
}

// Approaching the gap: turn on the spot to face +x, then stand still while gateLook measures.
void Navigator::gateTurn(const math::Pose& pose, float duty[2]) {
    duty[0] = duty[1] = 0.0f;
    if (gate_ != Gate::Turn) return;
    const float err = math::wrap(-pose.th);
    if (fabsf(err) < GATE_ALIGNED_RAD) {
        if (++gateSteps_ >= GATE_SETTLE_STEPS) {
            gate_ = Gate::Look;
            gateSteps_ = gateSeen_ = 0;
            gateDiff_ = 0.0f;
        }
        return;
    }
    gateSteps_ = 0;
    float u = fmaxf(-PIVOT_DUTY, fminf(PIVOT_DUTY, GATE_TURN_GAIN * err));
    if (fabsf(u) < GATE_TURN_MIN) u = err > 0 ? GATE_TURN_MIN : -GATE_TURN_MIN;
    duty[0] = -u;
    duty[1] = u;
}

// Facing the gap, the left 45-degree sensor sees T15 and the right one T16. Their difference says how far off the
// centre line the car stands, whatever both sensors' common range error: the y offset whose predicted difference fits
// best is applied to the pose.
bool Navigator::gateLook(math::Pose& pose, const float tof[]) {
    const float r = tof[0], l = tof[layout_.count - 1];
    if (r < GATE_SEE_MM && l < GATE_SEE_MM) {
        gateDiff_ += l - r;
        ++gateSeen_;
    }
    if (++gateSteps_ < GATE_LOOK_STEPS) return false;
    gate_ = Gate::Through;
    if (gateSeen_ < GATE_LOOK_STEPS / 2) return false;
    // The y offset whose predicted difference fits best. Only blocks near the sensors and no walls: the full prediction
    // over 31 offsets cost the chip ~0.2 s, past the motors' command lease (it stopped a floor run).
    const float measured = gateDiff_ / gateSeen_;
    float best = 1.0e9f, shift = 0.0f;
    for (float dy = -GATE_MAX_SHIFT_MM; dy <= GATE_MAX_SHIFT_MM; dy += GATE_SHIFT_STEP_MM) {
        const math::Pose c = {pose.x, pose.y + dy, pose.th};
        const float miss = fabsf(nearRange(c, layout_.count - 1) - nearRange(c, 0) - measured);
        if (miss < best) {
            best = miss;
            shift = dy;
        }
    }
    if (fabsf(shift) >= GATE_MAX_SHIFT_MM) return false;  // at the limit: implausible, trust the odometry
    pose.y += shift;
    return shift != 0.0f;
}

// The straight-right sensor against the face of a mapped block: the car is as much further from it than the estimate
// says as the reading is longer than the map's.
bool Navigator::sideFix(math::Pose& pose, float right90) {
    const float c = cosf(pose.th), s = sinf(pose.th);
    const math::Vec2 m = params_.right90Mount;
    const math::Vec2 from = {pose.x + c * m.x - s * m.y, pose.y + s * m.x + c * m.y};
    const math::Vec2 dir = {s, -c};  // heading - 90 degrees
    float expected = layout_.maxMm;
    for (int b = 0; b < planner_.numBlocks(); b++) expected = fminf(expected, rayBlock(from, dir, planner_.block(b), layout_.maxMm));
    const float err = right90 - expected;
    if (expected > SIDE_SEE_MM || fabsf(err) > SIDE_GATE_MM) return false;
    onFace_ = true;
    const float step = fmaxf(-SIDE_STEP_MM, fminf(SIDE_STEP_MM, SIDE_GAIN * err));
    if (fabsf(sideShifted_ + step) > SIDE_LIMIT_MM) return false;
    sideShifted_ += step;
    pose.x -= step * dir.x;
    pose.y -= step * dir.y;
    return true;
}

// Driving down (-y), the right-hand sensor looks toward -x (T16) and the left-hand one toward +x (T17). Something close on
// one side means the car is nearer to it than the estimate says.
bool Navigator::keepLane(math::Pose& pose, const float tof[], float right90) {
    const NavParams& p = params_;
    if (gate_ == Gate::Look) return gateLook(pose, tof);
    bool moved = false;
    const bool inGap = gate_ == Gate::Through && cosf(pose.th) > 0.97f;
    const bool wasOnFace = onFace_;
    onFace_ = false;
    if (inGap && !gateEdge_ && wasOnFace && right90 > GATE_EDGE_FAR_MM) {
        // Leaving T16's top face: the cone's forward edge last touched it at x 1500, h = the last reading ahead.
        gateEdge_ = true;
        const math::Vec2 m = params_.right90Mount;
        const float x = GATE_FACE_X1 - m.x - lastRight90_ * TOF_HALF_FOV_TAN;
        if (fabsf(x - pose.x) < GATE_EDGE_SPAN_MM) {
            pose.x = x;
            moved = true;
        }
    }
    lastRight90_ = right90;
    if (inGap) moved |= sideFix(pose, right90);
    if (p.laneClose <= 0 || planner_.numExtras() == 0 || pose.x < LANE_MIN_X_MM || pose.y > LANE_MAX_Y_MM ||
        pose.y < 0.0f || sinf(pose.th) > LANE_HEADING_SIN)
        return moved;
    const bool right = tof[0] < p.laneClose, left = tof[layout_.count - 1] < p.laneClose;
    if (right == left) return moved;
    const float step = right ? -p.laneStep : p.laneStep;  // the estimate moves toward what is close
    if (fabsf(laneShifted_ + step) > p.laneLimit) return moved;
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
