/**
 * @file    robot.h
 * @brief   What main.cpp (start-up, sensors) offers the run mode (course.cpp): the start-up result, a sensor loop
 *          that hands over one control step at a time, and the shared health checks.
 */
#pragma once

#include <stdint.h>

#include "odometry.h"
#include "tof_array.h"

struct StartupStatus {
    bool imuOk = false;
    int tofUp = 0;            // navigation sensors that came up, out of TOF_COUNT
    bool calibrated = false;  // gyro bias measured while standing still
};

// One control step (cfg::CONTROL_PERIOD_MS).
struct SensorStep : reliability::SensorSnapshot<cfg::TOF_COUNT> {
    int32_t countsL = 0, countsR = 0;  // signed counts since the previous encoder sample
    float tof[TOF_SENSORS] = {};      // clamped ranges in mm: [0..TOF_COUNT) the navigation sensors, then the extras (cfg::TOF_RIGHT_90)
};

// Waits for the next control step, reading the gyro every cfg::GYRO_PERIOD_MS meanwhile.
void waitStep(SensorStep& step);

// The navigation mode's run loop. Never returns.
[[noreturn]] void runMode(const StartupStatus& status);

struct RuntimeStatus {
    reliability::State state = reliability::State::Recovering;
    reliability::Fault fault = reliability::Fault::Sensor;
    uint32_t recoveryAttempts = 0;
    uint32_t commandExpiries = 0;
    uint32_t completedAt = 0;
    uint32_t gyroAt = 0;
    reliability::RangeStatus rangeStatus[TOF_COUNT] = {};
    uint32_t rangeTime[TOF_COUNT] = {};
    uint32_t rangeFailures[TOF_COUNT] = {};
};
extern volatile RuntimeStatus gRuntime;

// Shared health/recovery policy; requiredMask selects the mode's required ToF sensors.
reliability::Fault sensorIssue(const SensorStep& step, unsigned requiredMask, bool initialStart = true);
bool updateOdometry(nav::FusedOdometry& odo, const SensorStep& step);
bool recoverSensors(const SensorStep& step);
// Call after a long blocking computation (a route plan): the next step may be older than the usual limit without
// being a timing fault.
void noteStall();

// Measure the gyro's offset again (blocks ~1.5 s, motors off, car still). False when rejected: the old offset stays.
bool recalibrateGyro();

// Updates gRuntime (state, faults and sensor health, readable over ST-Link).
void reportRuntime(const reliability::Supervisor& supervisor, const SensorStep& step);
