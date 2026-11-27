/**
 * @file    course.cpp
 * @brief   Block-course mode (env:course_nav): drive the planned route to the finish line, going around the one random
 *          block if it is in the way.
 *
 * A run, as the operator sees it:
 *   power on -> sensors up, gyro calibrated (main.cpp) -> LED ready:
 *       slow even blink: run 1, no random block on the course. A clean finish records the reference.
 *       double blink:    run 2, a reference exists; the random block is looked for and avoided.
 *   Run 1 is learned anew after every power-on or reset (or hold KEY for 2 s during the countdown).
 *   press KEY -> 3 s countdown (LED blinks fast; the run record's flash slot is erased meanwhile). 1 s after KEY is let
 *       go the gyro offset is measured again (LED solid, ~1.5 s, hands off)
 *   run: 30 Hz control step (odometry -> Navigator -> wheel targets), the 200 Hz speed loop drives the wheels
 *   over the finish line -> stop, LED solid. Carry it back to the start, press KEY: the next run starts.
 *
 * Safety: reliability::Supervisor decides whether the car may move. A sensor, timing or pose problem stops the motors
 * (state Recovering) until the car has stood still and healthy for a moment; it then continues. Some faults latch
 * until reset. The LED shows the fault number (statusLed).
 */
#include <cmath>

#include "board.h"
#include "config.h"
#include "course_params.h"
#include "map_data.h"
#include "motor.h"
#include "navigator.h"
#include "robot.h"
#include "run_record.h"

namespace {

using reliability::Fault;
using reliability::State;
using reliability::Supervisor;

constexpr uint32_t COUNTDOWN_MS = 3000;          // time to take your hands away after pressing KEY
constexpr unsigned PRESS_STEPS = 3;              // KEY held this many control steps (~0.1 s) is a press
constexpr uint32_t REPLAN_LEASE_MS = 600;        // how long the motors keep their command while a replan blocks the loop
constexpr uint8_t FAULT_RESET_DURING_RUN = 8;    // LED code for "the chip reset in the middle of a run"

constexpr unsigned CLEAR_STEPS = 60;             // KEY held this long (2 s) forgets the reference

nav::Navigator gNav(cfg::TOF_LAYOUT, course::navParams(false));  // ~36 KB of planner arrays: static on purpose

// Everything the run loop keeps between control steps.
struct Course {
    Supervisor supervisor;
    nav::FusedOdometry odometry{cfg::ODOM};
    bool planned = false;      // a route has been planned since the last start
    bool routeReady = false;   // ... and is valid
    uint32_t lastPlanAt = 0;   // recovery planning is retried at most every RETRY_MS
    unsigned pressed = 0;      // control steps KEY has been held
    bool waitRelease = false;  // KEY must be let go before it starts anything again
    bool reachedFinish = false;
    bool hasReference = false;  // run 2 mode
    bool countingDown = false;
    uint32_t startAt = 0;      // end of the countdown
    bool gyroDue = false;      // the gyro has not been recalibrated in this countdown yet
    bool autoStart = false;    // re-armed after a finished run: start the countdown as soon as ready
    uint8_t ledFault = 0;      // the last fault, shown until the next run starts
    State previousState = State::Recovering;
};

// ---- Helpers ----------------------------------------------------------------------------------------------------------

// Run 1 or run 2: with a reference learned the Navigator looks for the random block against it.
void applyMode(Course& c) {
    const nav::Baseline reference = runrec::reference();
    c.hasReference = reference.records != nullptr;
    gNav.setParams(course::navParams(c.hasReference));
    gNav.setBaseline(reference);
}

// The course's sensor check. The floor is open, so "nothing in range" is a normal ToF reading: a sensor only has to be
// alive (a range or a no-target result). Missing, stale, bus-error and not-ready readings are faults.
Fault checkSensors(const SensorStep& s, bool beforeFirstStart) {
    const Fault f = sensorIssue(s, 0, beforeFirstStart);  // gyro, timing, start-up checks
    if (f != Fault::None) return f;
    for (int i = 0; i < TOF_COUNT; ++i) {
        const reliability::RangeStatus st = s.ranges[i].status;
        if (st != reliability::RangeStatus::Valid && st != reliability::RangeStatus::Invalid) return Fault::Sensor;
    }
    return Fault::None;
}

bool atFinish(const math::Pose& p) {
    return p.y < course::FINISH_Y && p.x > course::FINISH_X_MIN && p.x < course::FINISH_X_MAX &&
           gNav.progress() > course::FINISH_MIN_PROGRESS;
}

float secondsSinceStart(const Course& c, uint32_t now) { return (now - c.supervisor.startedAt()) * 0.001f; }

// Status LED: solid = running or finished, slow blink = ready for run 1, double blink = ready for run 2, fast blink = countdown, N short flashes then a pause =
// fault N (1 sensor, 2 timing, 3 command expired, 4 no route, 5 pose, 6 configuration, 7 timeout, 8 reset during a run).
bool statusLed(Course& c, uint32_t now) {
    const Supervisor& sup = c.supervisor;
    if (c.countingDown) return (now / 100) % 2 == 0;
    if (sup.state() == State::Running) {
        c.ledFault = 0;
        return true;
    }
    if (sup.started() && sup.fault() != Fault::None) c.ledFault = static_cast<uint8_t>(sup.fault());
    const uint8_t code = sup.fault() != Fault::None ? static_cast<uint8_t>(sup.fault()) : c.ledFault;
    if (code != 0) {  // N flashes of 150 ms, 150 ms apart, then a 1.2 s pause
        const uint32_t period = code * 300u + 1200u, t = now % period;
        return t < code * 300u && (t % 300u) < 150u;
    }
    if (sup.state() == State::Done) return true;
    if (c.hasReference) return now % 1200 < 120 || (now % 1200 >= 240 && now % 1200 < 360);
    return (now / 500) % 2 == 0;
}

// ---- The parts of one control step ------------------------------------------------------------------------------------

// The run finished and the car has been carried back to the start line: a KEY press re-arms it. The next run starts as
// soon as it is ready again (standing still, route planned). A fault stays latched until reset.
void rearmIfPressed(Course& c, State before) {
    if (before != State::Done || c.pressed < PRESS_STEPS) return;
    c.supervisor = Supervisor();
    c.odometry.reset({course::START_X, course::START_Y, course::START_TH});
    c.routeReady = false;
    c.planned = false;
    c.countingDown = false;
    c.autoStart = true;
}

// KEY starts the countdown; when it ends the run starts from the start line with a fresh plan. Returns true on the step
// the run starts. `issue` becomes NoRoute if the start plan fails.
bool handleStart(Course& c, const SensorStep& s, Fault& issue) {
    if (c.supervisor.state() != State::Ready) c.countingDown = false;
    if (c.countingDown && c.pressed >= CLEAR_STEPS) {  // KEY still held: forget the reference, back to run 1
        runrec::clearReference();
        applyMode(c);
        c.countingDown = false;
        c.waitRelease = true;
    }
    if (c.pressed == 0) c.waitRelease = false;
    if (!c.waitRelease && !c.countingDown && c.supervisor.state() == State::Ready &&
        (c.pressed >= PRESS_STEPS || c.autoStart)) {
        c.autoStart = false;
        c.countingDown = true;
        c.startAt = s.completedAt + COUNTDOWN_MS;
        c.gyroDue = true;
        runrec::prepare();
    }
    const bool startNow = c.countingDown && static_cast<int32_t>(s.completedAt - c.startAt) >= 0;
    if (c.countingDown && !startNow) runrec::eraseStep();
    if (!startNow) return false;

    // A fresh start: whatever the wheels or the gyro did while the car was being placed is dropped.
    c.countingDown = false;
    c.reachedFinish = false;
    applyMode(c);  // a run 1 that just finished has left a reference
    c.odometry.reset({course::START_X, course::START_Y, course::START_TH});
    c.routeReady = gNav.start(c.odometry.pose, 0.0f);
    c.planned = true;
    noteStall();  // the plan blocked the loop
    runrec::startRun();
    if (issue == Fault::None && !c.routeReady) issue = Fault::NoRoute;
    return true;
}

// In the countdown, once KEY has been let go for a moment: measure the gyro's offset again with the car untouched.
// Blocks the loop; the start is pushed back if less than 200 ms of the countdown would be left.
void recalibrateIfDue(Course& c, const SensorStep& s) {
    if (!c.countingDown || !c.gyroDue || c.pressed != 0 ||
        s.completedAt - (c.startAt - COUNTDOWN_MS) < cfg::GYRO_RECAL_DELAY_MS)
        return;
    c.gyroDue = false;
    ledStatusSet(true);  // solid: hands off
    recalibrateGyro();
    if (static_cast<int32_t>(c.startAt - HAL_GetTick()) < 200) c.startAt = HAL_GetTick() + 200;
}

// Stopped by a fault: try to recover the sensors, and plan again once they are healthy.
void recoverStep(Course& c, const SensorStep& s) {
    if (recoverSensors(s)) {
        c.supervisor.update(HAL_GetTick(), Fault::Sensor, false, false);
        return;
    }
    const bool healthy = checkSensors(s, !c.supervisor.started()) == Fault::None;
    if (!healthy || c.routeReady || s.completedAt - c.lastPlanAt < reliability::RETRY_MS) return;
    c.lastPlanAt = s.completedAt;
    ++gRuntime.recoveryAttempts;
    const float t = c.supervisor.started() ? secondsSinceStart(c, s.completedAt) : 0;
    if (!c.planned) {
        c.routeReady = gNav.start(c.odometry.pose, t);
        c.planned = true;
    } else {
        c.routeReady = gNav.reposition(c.odometry.pose, t);
    }
    noteStall();  // the plan blocked the loop: the next step is not a timing fault
}

// Running: one Navigator step and the wheel targets. targetMmS is set for the run log.
void driveStep(Course& c, SensorStep& s, State before, float targetMmS[2]) {
    if (before != State::Running) motorEnable(true);
    const float t = secondsSinceStart(c, s.completedAt);
    float duty[2] = {};
    const int replansBefore = gNav.replans();
    // A replan runs the planner inside this step and blocks the loop: keep the motors' command alive meanwhile.
    if (gNav.replanDue(t)) motorExtendLease(REPLAN_LEASE_MS);
    // gNav.localize(c.odometry.pose, s.tof, t);  // pose correction: off, see docs/DECISIONS.md
    gNav.keepLane(c.odometry.pose, s.tof);
    gNav.step(c.odometry.pose, c.odometry.v, s.tof, t, duty);
    if (gNav.replans() != replansBefore) {
        // The sensor snapshot is now older than usual. That is expected after a plan, not a fault.
        s.maxAgeMs = cfg::STALL_MAX_AGE_MS;
        noteStall();
        if (!motorsEnabled()) motorEnable(true);
    }

    Fault issue = checkSensors(s, !c.supervisor.started());
    if (!gNav.hasRoute()) issue = Fault::NoRoute;
    if (!motorsEnabled()) issue = Fault::CommandExpired;
    if (!std::isfinite(duty[0]) || !std::isfinite(duty[1])) {
        c.supervisor.latch(Fault::Configuration);
        motorEnable(false);
    } else if (issue != Fault::None) {
        motorEnable(false);
        c.routeReady = false;
        c.supervisor.update(HAL_GetTick(), issue, false, false);
    } else if (atFinish(c.odometry.pose)) {
        motorEnable(false);
        c.reachedFinish = true;
        c.supervisor.done();
    } else {
        targetMmS[0] = duty[0] * cfg::V_FULL_MM_S;
        targetMmS[1] = duty[1] * cfg::V_FULL_MM_S;
        motorSetTargets(targetMmS[0], targetMmS[1]);
    }
}

// Run log, LED, trace, and saving the record when the run has ended.
void recordStep(Course& c, const SensorStep& s, const float targetMmS[2]) {
    Supervisor& sup = c.supervisor;
    runrec::markRunning(sup.state() == State::Running);
    if (sup.state() == State::Running && sup.started())
        runrec::logStep(s.completedAt - sup.startedAt(), c.odometry.pose, gNav.lastAlpha(), targetMmS, s.tof);
    ledStatusSet(statusLed(c, s.completedAt));
    reportRuntime(sup, s);
    runrec::traceStep(sup, gNav, c.odometry.pose, s.completedAt);
    if (c.previousState == State::Running && sup.state() != State::Running) {  // stopped: finished, faulted or recovering
        motorEnable(false);
        noteStall();
    }
    if (runrec::saveIfEnded(sup, c.reachedFinish)) {
        noteStall();  // the flash write blocks the loop for a moment
        applyMode(c);
    }
    c.previousState = sup.state();
}

}  // namespace

void runMode(const StartupStatus&) {
    static Course c;
    if (runrec::boot()) c.ledFault = FAULT_RESET_DURING_RUN;
    gNav.init(MAP_BLOCKS, MAP_BLOCK_COUNT);
    applyMode(c);
    c.odometry.reset({course::START_X, course::START_Y, course::START_TH});
    if (!gNav.configurationValid()) c.supervisor.latch(Fault::Configuration);
    c.lastPlanAt = HAL_GetTick() - reliability::RETRY_MS;
    SensorStep s;

    while (true) {
        waitStep(s);
        float targetMmS[2] = {};
        const State before = c.supervisor.state();

        // 1. Pose and health
        if (before != State::Done && before != State::Fault && !updateOdometry(c.odometry, s))
            c.supervisor.latch(Fault::Pose);
        Fault issue = checkSensors(s, !c.supervisor.started());
        if (before == State::Running && !motorsEnabled()) issue = Fault::CommandExpired;
        if (issue != Fault::None) c.routeReady = false;
        if (issue == Fault::None && (!c.routeReady || !gNav.hasRoute())) issue = Fault::NoRoute;

        // 2. Operator: re-arm after a finished run, start the countdown and the run
        c.pressed = buttonPressed() ? c.pressed + 1 : 0;
        rearmIfPressed(c, before);
        const bool startNow = handleStart(c, s, issue);
        // The start plan blocked the loop, so this step's sensor snapshot is old. Expected, not a timing fault (without
        // this the run opened with ~0.9 s of fault recovery before the wheels moved).
        if (startNow) s.maxAgeMs = cfg::STALL_MAX_AGE_MS;

        // 3. Let the supervisor decide the state; stop at the time limit
        c.supervisor.update(s.completedAt, issue, s.countsL == 0 && s.countsR == 0, startNow);
        if (c.supervisor.state() != State::Fault && c.supervisor.started() &&
            secondsSinceStart(c, s.completedAt) > cfg::RUN_TIME_LIMIT_S)
            c.supervisor.done();

        // 4. Act on the state
        if (c.supervisor.state() != State::Running) motorEnable(false);
        if (c.supervisor.state() == State::Recovering) recoverStep(c, s);
        else if (c.supervisor.state() == State::Running) driveStep(c, s, before, targetMmS);

        // 5. Record
        recordStep(c, s, targetMmS);
        recalibrateIfDue(c, s);  // last: the snapshot above is old once this has run
    }
}
