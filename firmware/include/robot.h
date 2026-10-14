/**
 * @file    robot.h
 * @brief   What main.cpp shares with the navigation mode it was built with (course.cpp or maze.cpp): the start-up
 *          result and a sensor loop that hands over one control step at a time.
 */
#pragma once

#include <stdint.h>

#include "odometry.h"
#include "tof_array.h"

struct StartupStatus {
    bool imuOk = false;
    int tofUp = 0;            // sensors that came up, out of TOF_COUNT
    bool calibrated = false;  // gyro bias measured while standing still
};

// One control step (cfg::CONTROL_PERIOD_MS).
struct SensorStep {
    uint32_t now = 0;               // HAL tick, ms
    float dt = 0;                   // s since the previous step
    int32_t countsL = 0, countsR = 0;  // encoder counts since the previous step, forward positive
    float gyro = 0;                 // mean yaw rate over the step, rad/s, CCW positive; 0 if gyroSamples == 0
    unsigned gyroSamples = 0;
    float tof[TOF_COUNT] = {};      // mm
};

// Waits for the next control step, reading the gyro every cfg::GYRO_PERIOD_MS meanwhile.
void waitStep(SensorStep& step);

nav::OdomParams odometryParams();

// The navigation mode's run loop. Never returns.
[[noreturn]] void runMode(const StartupStatus& status);
