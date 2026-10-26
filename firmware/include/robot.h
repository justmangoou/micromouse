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
struct SensorStep : reliability::SensorSnapshot<cfg::TOF_COUNT> {
    int32_t countsL = 0, countsR = 0;  // signed counts since the previous encoder sample
    float tof[TOF_COUNT] = {};        // clamped ranges for navigation, in mm
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

// Host commands received since the last call (remote.h); each one is acknowledged.
struct RemoteCommands {
    bool start = false;  // treat like a button press
    bool stop = false;   // end the run: Supervisor::done(), motors off
    bool drive = false;  // a Drive packet arrived; left/right are the latest (mm/s), unclamped
    float left = 0, right = 0;
};
RemoteCommands pollRemote();

// Updates gRuntime and sends telemetry every cfg::TELEMETRY_PERIOD_MS. target: wheel targets this step, mm/s.
void reportRuntime(const reliability::Supervisor& supervisor, const SensorStep& step, const nav::FusedOdometry& odo,
                   const float target[2]);
