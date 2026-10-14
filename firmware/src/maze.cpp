/**
 * @file    maze.cpp
 * @brief   Micromouse mode (env:micromouse): explore a 16 x 16 maze cell by cell with flood fill until a centre cell.
 *          Start at the southwest cell centre, facing north.
 */
#include <stdio.h>

#include "board.h"
#include "config.h"
#include "micromouse.h"
#include "motor.h"
#include "robot.h"

namespace {

constexpr uint32_t TOF_MAX_AGE_MS = 150;  // an older range is not evidence of anything

// Maze tuning. Measure the cell size for your maze.
nav::MicromouseParams mazeParams() {
    nav::MicromouseParams p;
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

nav::Micromouse gMouse(mazeParams());

}  // namespace

void runMode(const StartupStatus& status) {
    bool ready = status.calibrated && status.tofUp == TOF_COUNT;
    printf("maze %s: place at the southwest cell centre facing north\r\n", ready ? "ready" : "NOT READY");

    nav::FusedOdometry odo(odometryParams());
    odo.reset(gMouse.startPose());
    bool running = false;
    unsigned pressed = 0;
    uint32_t runStart = 0;
    SensorStep s;

    while (true) {
        waitStep(s);
        bool reliable[TOF_COUNT];
        for (int k = 0; k < TOF_COUNT; ++k) reliable[k] = tofReliable(k, TOF_MAX_AGE_MS);

        if (running) {
            if (s.gyroSamples == 0 || s.dt <= 0 || s.dt > TOF_MAX_AGE_MS * 0.001f) {
                gMouse.stop();
            } else {
                odo.update(s.countsL, s.countsR, s.gyro, s.dt);
            }
            float target[2];
            gMouse.step(odo.pose, s.tof, reliable, (s.now - runStart) * 0.001f, target);
            motorSetTargets(target[0], target[1]);

            auto state = gMouse.state();
            if (state == nav::Micromouse::State::Done || state == nav::Micromouse::State::Fault) {
                motorEnable(false);
                running = ready = false;
                nav::Cell cell = gMouse.cell();
                printf("maze %s at %d,%d\r\n", state == nav::Micromouse::State::Done ? "goal" : "FAULT", cell.x, cell.y);
            }
            ledSet(running);
        } else if (ready) {
            ledSet((s.now / 500) % 2 == 0);
            pressed = buttonPressed() ? pressed + 1 : 0;
            if (pressed >= 3 && s.gyroSamples > 0 && reliable[tof::RIGHT_MIDDLE] && reliable[tof::LEFT_MIDDLE]) {
                odo.reset(gMouse.startPose());  // start from the cell pose only once the button is pressed
                runStart = s.now;
                running = ready = gMouse.start(0.0f);
                motorEnable(running);
                printf("maze %s\r\n", running ? "run" : "invalid configuration");
            }
        } else {
            ledSet(false);
        }
    }
}
