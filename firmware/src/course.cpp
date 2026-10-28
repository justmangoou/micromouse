/**
 * @file    course.cpp
 * @brief   Block-course mode (env:course_nav): plan with A* on the stored map, follow the path, replan around
 *          blocks the map does not have, stop over the finish line.
 */
#include <cmath>

#include "board.h"
#include "config.h"
#include "map_data.h"
#include "motor.h"
#include "navigator.h"
#include "robot.h"

namespace {

// Start pose: behind the start line, facing +y (mm, rad)
constexpr float START_X = 300.0f;
constexpr float START_Y = -80.0f;
constexpr float START_TH = 1.57079633f;

// Finish: the whole car below the line in the right-hand column, most of the path done
constexpr float FINISH_Y = -90.0f;
constexpr float FINISH_X_MIN = 1250.0f, FINISH_X_MAX = 1750.0f;
constexpr float FINISH_MIN_PROGRESS = 0.8f;

nav::NavParams navParams() {
    nav::NavParams p;
    p.vFull = cfg::V_FULL_MM_S;
    p.track = cfg::WHEEL_TRACK_MM;
    return p;
}

nav::Navigator gNav(cfg::TOF_LAYOUT, navParams());  // ~36 KB of planner arrays, static on purpose

bool atFinish(const math::Pose& p) {
    return p.y < FINISH_Y && p.x > FINISH_X_MIN && p.x < FINISH_X_MAX && gNav.progress() > FINISH_MIN_PROGRESS;
}

}  // namespace

void runMode(const StartupStatus&) {
    using namespace reliability;
    gNav.init(MAP_BLOCKS, MAP_BLOCK_COUNT);
    nav::FusedOdometry odo(cfg::ODOM);
    odo.reset({START_X, START_Y, START_TH});
    Supervisor supervisor;
    if (!gNav.configurationValid()) supervisor.latch(Fault::Configuration);
    bool planned = false;
    bool routeReady = false;
    uint32_t lastPlan = HAL_GetTick() - RETRY_MS;
    unsigned pressed = 0;
    SensorStep s;

    while (true) {
        waitStep(s);
        float target[2] = {};  // wheel targets this step, mm/s
        RemoteCommands remote = pollRemote();
        State before = supervisor.state();
        if (before != State::Done && before != State::Fault && !updateOdometry(odo, s)) supervisor.latch(Fault::Pose);
        Fault issue = sensorIssue(s, (1u << TOF_COUNT) - 1, !supervisor.started());
        if (before == State::Running && !motorsEnabled()) issue = Fault::CommandExpired;
        if (issue != Fault::None) routeReady = false;
        if (issue == Fault::None && (!routeReady || !gNav.hasRoute())) issue = Fault::NoRoute;
        pressed = buttonPressed() ? pressed + 1 : 0;
        supervisor.update(s.completedAt, issue, s.countsL == 0 && s.countsR == 0, pressed >= 3 || remote.start);
        if (supervisor.state() != State::Fault && supervisor.started() && (s.completedAt - supervisor.startedAt()) * 0.001f > cfg::RUN_TIME_LIMIT_S)
            supervisor.done();
        if (remote.stop) supervisor.done();

        if (supervisor.state() != State::Running) motorEnable(false);
        if (supervisor.state() == State::Recovering) {
            if (recoverSensors(s)) {
                supervisor.update(HAL_GetTick(), Fault::Sensor, false, false);
            } else if (sensorIssue(s, (1u << TOF_COUNT) - 1, !supervisor.started()) == Fault::None && !routeReady &&
                       s.completedAt - lastPlan >= RETRY_MS) {
                lastPlan = s.completedAt;
                ++gRuntime.recoveryAttempts;
                float planTime = supervisor.started() ? (s.completedAt - supervisor.startedAt()) * 0.001f : 0;
                if (!planned) { routeReady = gNav.start(odo.pose, planTime); planned = true; }
                else routeReady = gNav.reposition(odo.pose, planTime);
            }
        } else if (supervisor.state() == State::Running) {
            if (before != State::Running) motorEnable(true);
            float t = (s.completedAt - supervisor.startedAt()) * 0.001f;
            float duty[2] = {};
            gNav.step(odo.pose, odo.v, s.tof, t, duty);
            Fault after = sensorIssue(s, (1u << TOF_COUNT) - 1, !supervisor.started());
            if (!gNav.hasRoute()) after = Fault::NoRoute;
            if (!motorsEnabled()) after = Fault::CommandExpired;
            if (!std::isfinite(duty[0]) || !std::isfinite(duty[1])) {
                supervisor.latch(Fault::Configuration);
                motorEnable(false);
            } else if (after != Fault::None) {
                motorEnable(false);
                routeReady = false;
                supervisor.update(HAL_GetTick(), after, false, false);
            } else if (atFinish(odo.pose)) {
                motorEnable(false);
                supervisor.done();
            } else {
                target[0] = duty[0] * cfg::V_FULL_MM_S;
                target[1] = duty[1] * cfg::V_FULL_MM_S;
                motorSetTargets(target[0], target[1]);
            }
        }
        ledStatusSet(supervisor.state() == State::Running ||
                     (supervisor.state() == State::Ready && (s.completedAt / 500) % 2 == 0));
        reportRuntime(supervisor, s, odo, target);
    }
}
