/**
 * @file    maze.cpp
 * @brief   Micromouse mode (env:micromouse): explore a 16 x 16 maze cell by cell with flood fill until a centre cell.
 *          Start at the southwest cell centre, facing north.
 */
#include <stdio.h>
#include <cmath>

#include "board.h"
#include "config.h"
#include "micromouse.h"
#include "motor.h"
#include "robot.h"

namespace {

// Maze tuning. Measure the cell size for your maze.
nav::MicromouseParams mazeParams() {
    nav::MicromouseParams p;
    p.leftSensor = cfg::TOF_LEFT_MIDDLE;
    p.rightSensor = cfg::TOF_RIGHT_MIDDLE;
    p.width = 16;
    p.height = 16;
    p.cellSize = 180.0f;            // mm
    p.track = cfg::WHEEL_TRACK_MM;
    p.speed = 120.0f;               // mm/s
    p.minSpeed = 40.0f;             // mm/s
    p.maxTurnRate = 1.5f;          // rad/s
    p.headingGain = 3.0f;
    p.crossTrackGain = 0.03f;       // rad/s per mm of lateral error
    p.positionTolerance = 8.0f;    // mm
    p.headingTolerance = 0.05f;    // rad
    p.wallTolerance = 10.0f;        // mm, wall plane relative to the cell boundary
    p.openMargin = 20.0f;           // mm
    p.stopClearance = 30.0f;        // mm, forward clearance from the inner sensor mounts
    p.maxCrossTrack = 30.0f;        // mm
    p.settleTime = 0.2f;           // s
    p.observationTimeout = 1.0f;   // s
    p.moveTimeout = 6.0f;          // s
    p.runTimeout = cfg::RUN_TIME_LIMIT_S;
    return p;
}

nav::Micromouse gMouse(cfg::TOF_LAYOUT, mazeParams());

}  // namespace

void runMode(const StartupStatus&) {
    using namespace reliability;
    constexpr unsigned required = (1u << cfg::TOF_RIGHT_MIDDLE) | (1u << cfg::TOF_LEFT_MIDDLE);
    nav::FusedOdometry odo(cfg::ODOM);
    odo.reset(gMouse.startPose());
    Supervisor supervisor;
    bool mouseStarted = false;
    unsigned pressed = 0;
    SensorStep s;

    while (true) {
        waitStep(s);
        float target[2] = {};  // wheel targets this step, mm/s
        RemoteCommands remote = pollRemote();
        State before = supervisor.state();
        if (before != State::Done && before != State::Fault && !updateOdometry(odo, s)) supervisor.latch(Fault::Pose);
        Fault issue = sensorIssue(s, required, !supervisor.started());
        if (before == State::Running && !motorsEnabled()) issue = Fault::CommandExpired;
        pressed = buttonPressed() ? pressed + 1 : 0;
        supervisor.update(s.completedAt, issue, s.countsL == 0 && s.countsR == 0, pressed >= 3 || remote.start);
        float t = supervisor.started() ? (s.completedAt - supervisor.startedAt()) * 0.001f : 0;
        if (supervisor.state() != State::Fault && supervisor.started() && t > cfg::RUN_TIME_LIMIT_S) supervisor.done();
        if (remote.stop) supervisor.done();

        if (supervisor.state() != State::Running) {
            motorEnable(false);
            if (mouseStarted) gMouse.pause(t);
        }
        if (supervisor.state() == State::Recovering) {
            if (recoverSensors(s)) supervisor.update(HAL_GetTick(), Fault::Sensor, false, false);
        } else if (supervisor.state() == State::Running) {
            if (before != State::Running) {
                if (!mouseStarted) odo.reset(gMouse.startPose());
                bool ok = mouseStarted ? gMouse.resume(t) : gMouse.start(t);
                if (!ok) supervisor.latch(Fault::Configuration);
                else { mouseStarted = true; motorEnable(true); }
            }
            if (supervisor.state() == State::Running) {
                bool reliable[TOF_COUNT];
                for (int i = 0; i < TOF_COUNT; ++i) reliable[i] = s.ranges[i].status == RangeStatus::Valid;
                gMouse.step(odo.pose, s.tof, reliable, t, target);
                if (!std::isfinite(target[0]) || !std::isfinite(target[1])) supervisor.latch(Fault::Configuration);
                else if (gMouse.state() == nav::Micromouse::State::Done) supervisor.done();
                else if (gMouse.state() == nav::Micromouse::State::Fault) supervisor.latch(Fault::Pose);
                if (supervisor.state() == State::Running) motorSetTargets(target[0], target[1]);
                else {
                    motorEnable(false);
                    target[0] = target[1] = 0;
                }
            }
        }
        ledStatusSet(supervisor.state() == State::Running ||
                     (supervisor.state() == State::Ready && (s.completedAt / 500) % 2 == 0));
        reportRuntime(supervisor, s, odo, target);
    }
}
