/**
 * @file    navigator.cpp
 * @brief   Navigator implementation: unmapped-block detection (ToF against the distance the map predicts), replan
 *          triggers and pure-pursuit tracking with speed scaling.
 */
#include "navigator.h"
#include <cmath>

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
    Block blocks[MAX_MAP];
    if (n > MAX_MAP) n = MAX_MAP;
    for (int i = 0; i < n; i++) blocks[i] = Block(map[i][0], map[i][1], map[i][2]);
    planner_.setMap(blocks, n);
}

bool Navigator::replan(math::Vec2 from, float t) {
    path_.n = 0;
    bool ok = configurationValid() && std::isfinite(from.x) && std::isfinite(from.y) &&
              std::isfinite(t) && planner_.plan(from, path_) && path_.n > 1;
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
    replans_ = 0;
    path_.n = 0;
    return replan({est.x, est.y}, t);
}

bool Navigator::reposition(const math::Pose& est, float t) { return replan({est.x, est.y}, t); }

// A reading much shorter than the map predicts is a block that is not in the map.
void Navigator::scan(const math::Pose& e, const float tof[], float t) {
    float c = cosf(e.th), s = sinf(e.th);
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
        float z = tof[k];
        bool unexplained = z < expected - (40.0f + 0.08f * expected) && z < 800.0f;
        streak_[k] = unexplained ? (uint8_t)(streak_[k] < 250 ? streak_[k] + 1 : 250) : 0;
        if (streak_[k] < 2) continue;

        float a = e.th + layout_.sensors[k].angle;
        math::Vec2 d = {cosf(a), sinf(a)};
        math::Vec2 hit = {o.x + z * d.x, o.y + z * d.y};
        if (planner_.mapClearance(hit) < 50.0f) continue;  // a mapped block seen through a small pose error
        math::Vec2 centre = {o.x + (z + BLOCK_HALF) * d.x, o.y + (z + BLOCK_HALF) * d.y};  // near face seen, centre is behind

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
            int i = planner_.addExtra(centre);
            if (i >= 0) {
                hits_[i] = 1;
                needReplan_ = true;
                alertUntil_ = t + 0.8f;
            }
        }
    }
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

    float look = prm_.lookahead + 0.15f * fabsf(speed);
    int j = idx_;
    while (j < path_.n - 1 && path_.s[j] < path_.s[idx_] + look) j++;
    math::Vec2 tgt = path_.p[j];
    float dist = fmaxf(math::distance({e.x, e.y}, tgt), 10.0f);
    float alpha = math::wrap(atan2f(tgt.y - e.y, tgt.x - e.x) - e.th);
    if (fabsf(alpha) > 1.2f) {  // facing the wrong way: pivot first
        float sg = alpha > 0 ? 1.0f : -1.0f;
        duty[0] = -0.45f * sg;
        duty[1] = 0.45f * sg;
        return;
    }
    float kappa = 2.0f * sinf(alpha) / dist;
    float v = prm_.speed * prm_.vFull * path_.v[idx_] * fmaxf(0.35f, cosf(alpha));
    if (e.x > 1050.0f) v *= prm_.caution;
    if (t < alertUntil_) v *= prm_.alertSpeed;
    float vMin = 0.22f * prm_.vFull;  // the slowdown factors multiply; keep the motors out of the deadband
    if (v < vMin) v = vMin;
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
