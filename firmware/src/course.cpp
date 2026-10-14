/**
 * @file    course.cpp
 * @brief   Block-course mode (env:obstacle_avoidance): plan with A* on the stored map, follow the path, replan around
 *          blocks the map does not have, stop over the finish line.
 */
#include <stdio.h>

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

nav::Navigator gNav(navParams());  // ~65 KB of planner arrays, static on purpose

bool atFinish(const math::Pose& p) {
    return p.y < FINISH_Y && p.x > FINISH_X_MIN && p.x < FINISH_X_MAX && gNav.progress() > FINISH_MIN_PROGRESS;
}

// One CSV line per step: time (ms), x, y (mm), heading (mrad), speed (mm/s), left and right duty (%), ToF (mm).
void logStep(uint32_t ms, const nav::FusedOdometry& odo, const float duty[2], const float tof[TOF_COUNT]) {
    printf("%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n", (unsigned long)ms, (int)(odo.pose.x), (int)(odo.pose.y),
           (int)(odo.pose.th * 1000), (int)(odo.v), (int)(duty[0] * 100), (int)(duty[1] * 100), (int)(tof[0]),
           (int)(tof[1]), (int)(tof[2]), (int)(tof[3]));
}

}  // namespace

void runMode(const StartupStatus& status) {
    enum class State { Ready, Running, Done };

    printf("block course: %d map blocks\r\n", MAP_BLOCK_COUNT);
    gNav.init(MAP_BLOCKS, MAP_BLOCK_COUNT);
    nav::FusedOdometry odo(odometryParams());
    odo.reset({START_X, START_Y, START_TH});

    if (status.calibrated) {
        uint32_t t0 = HAL_GetTick();
        gNav.start(odo.pose, 0.0f);
        printf("planned %d points, %d replans, %lu ms\r\n", gNav.path().n, gNav.replans(),
               (unsigned long)(HAL_GetTick() - t0));
    }

    State state = status.calibrated ? State::Ready : State::Done;
    unsigned pressed = 0;
    uint32_t runStart = 0;
    float gyro = 0;  // last good yaw rate, held through a step without a gyro sample
    SensorStep s;

    while (true) {
        waitStep(s);
        if (s.gyroSamples > 0) gyro = s.gyro;
        float dt = s.dt < 0.005f ? cfg::CONTROL_PERIOD_MS * 0.001f : s.dt;
        odo.update(s.countsL, s.countsR, gyro, dt);

        switch (state) {
            case State::Ready:
                ledSet((s.now / 500) % 2 == 0);
                pressed = buttonPressed() ? pressed + 1 : 0;
                if (pressed >= 3) {
                    runStart = s.now;
                    motorEnable(true);
                    printf("run\r\n");
                    state = State::Running;
                }
                break;

            case State::Running: {
                float t = (s.now - runStart) * 0.001f;
                float duty[2];
                gNav.step(odo.pose, odo.v, s.tof, t, duty);
                motorSetTargets(duty[0] * cfg::V_FULL_MM_S, duty[1] * cfg::V_FULL_MM_S);
                ledSet(true);
                logStep(s.now - runStart, odo, duty, s.tof);

                if (atFinish(odo.pose) || t > cfg::RUN_TIME_LIMIT_S) {
                    motorEnable(false);
                    printf("done at %d ms, %d unmapped blocks, %d replans\r\n", (int)(t * 1000), gNav.unmappedBlocks(),
                           gNav.replans());
                    state = State::Done;
                }
                break;
            }

            case State::Done:
                ledSet(false);
                break;
        }
    }
}
