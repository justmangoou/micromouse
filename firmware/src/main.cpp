/**
 * @file    main.cpp
 * @brief   Start-up and sensor loop shared by both navigation modes. The mode itself is course.cpp (block course, A*)
 *          or maze.cpp (16 x 16 maze, flood fill); platformio.ini builds one of them per environment.
 *
 * @par Flow
 *   power on -> sensors up -> gyro calibration (keep the robot still) -> runMode(): wait for the button, then run at
 *   30 Hz (ToF, odometry, navigation) while the 200 Hz TIM11 interrupt closes the wheel speed loop.
 */
#include <stdio.h>
#include <cmath>

#include "board.h"
#include "config.h"
#include "i2c_bus.h"
#include "motor.h"
#include "mpu6050.h"
#include "remote.h"
#include "robot.h"

namespace {

I2cBus gI2c(&hi2c1);  // shared by the IMU and the ToF sensors
Mpu6050 gImu(gI2c);

StartupStatus gStartup;
uint32_t gLastGyro = 0;
uint32_t gLastRecovery = 0;
bool gLoopStarted = false;
uint32_t gNextGyro, gNextStep, gLastStep;  // HAL ticks, ms
uint32_t gLastTelemetry = 0;
nav::GyroBiasTracker gBiasTracker(cfg::GYRO_BIAS);

bool due(uint32_t now, uint32_t at) { return static_cast<int32_t>(now - at) >= 0; }  // wrap-safe

// Fixed rate. After a stall, skip the missed periods instead of running them back to back.
uint32_t nextTime(uint32_t at, uint32_t now, uint32_t period) {
    at += period;
    return due(now, at) ? now + period : at;
}

}  // namespace

void waitStep(SensorStep& step) {
    if (!gLoopStarted) {
        gNextGyro = gNextStep = gLastStep = HAL_GetTick();
        gLoopStarted = true;
    }
    float gyroSum = 0;
    unsigned gyroN = 0;
    while (true) {
        uint32_t now = HAL_GetTick();
        if (due(now, gNextGyro)) {
            gNextGyro = nextTime(gNextGyro, now, cfg::GYRO_PERIOD_MS);
            if (auto g = gImu.readGyroZ()) {
                gyroSum += cfg::GYRO_SIGN * cfg::GYRO_SCALE * *g;
                gyroN++;
                gLastGyro = HAL_GetTick();
            }
        }
        if (due(now, gNextStep)) {
            gNextStep = nextTime(gNextStep, now, cfg::CONTROL_PERIOD_MS);
            break;
        }
    }
    step.now = HAL_GetTick();
    step.dt = (step.now - gLastStep) * 0.001f;
    gLastStep = step.now;
    motorTakeCounts(&step.countsL, &step.countsR);
    step.gyroAt = gLastGyro;
    step.gyroSamples = gyroN;
    step.gyro = gyroN > 0 ? gyroSum / gyroN : 0.0f;
    // Standing still with the motors off: follow the gyro's offset instead of trusting the start-up value forever.
    if (gStartup.imuOk && gStartup.calibrated && gyroN > 0) {
        float sensorRate = step.gyro / (cfg::GYRO_SIGN * cfg::GYRO_SCALE);
        gImu.adjustGyroBiasZ(gBiasTracker.update(sensorRate, step.countsL == 0 && step.countsR == 0 && !motorsEnabled(), step.dt));
    }
    tofRead(step.tof);
    step.completedAt = HAL_GetTick();
    for (int i = 0; i < TOF_COUNT; ++i) step.ranges[i] = tofReading(i, step.completedAt);
}

int main() {
    boardInit();
    motorInit();
    speedLoopStart();

    StartupStatus& status = gStartup;
    status.imuOk = gImu.init();
    status.tofUp = tofInit(gI2c);
    printf("\r\nimu %s, tof %d/%d up\r\ngyro calibration: keep the robot still\r\n", status.imuOk ? "ok" : "FAILED",
           status.tofUp, TOF_COUNT);

    ledStatusSet(true);
    status.calibrated = status.imuOk && gImu.calibrateGyroZ(cfg::GYRO_CALIBRATION_SAMPLES, cfg::GYRO_PERIOD_MS);
    printf("gyro calibration %s, bias %d mdeg/s\r\n", status.calibrated ? "ok" : "FAILED",
           (int)(cfg::GYRO_SIGN * gImu.gyroBiasZ() * 57295.78f));

    int32_t left, right;
    motorTakeCounts(&left, &right);  // drop whatever the wheels moved during start-up
    gLastRecovery = HAL_GetTick();
    runMode(status);
}

volatile RuntimeStatus gRuntime;

reliability::Fault sensorIssue(const SensorStep& s, unsigned mask, bool initialStart) {
    return s.issue(HAL_GetTick(), mask, gStartup.imuOk && (!initialStart || gStartup.tofUp == TOF_COUNT), gStartup.calibrated, cfg::TOF_MIN_MM, cfg::TOF_MAX_MM);
}

bool updateOdometry(nav::FusedOdometry& odo, const SensorStep& s) {
    if (!reliability::encoderGapSafe(s.dt, cfg::MM_PER_COUNT, cfg::V_FULL_MM_S)) return false;
    if (s.dt > 0) {
        if (s.gyroSamples && std::isfinite(s.gyro) && reliability::fresh(s.completedAt, s.gyroAt) &&
            s.dt <= reliability::MAX_AGE_MS * 0.001f)
            odo.update(s.countsL, s.countsR, s.gyro, s.dt);
        else odo.updateEncoders(s.countsL, s.countsR, s.dt);
    }
    return std::isfinite(odo.pose.x) && std::isfinite(odo.pose.y) && std::isfinite(odo.pose.th);
}

bool recoverSensors(const SensorStep& s) {
    static reliability::Qualification stationary;
    bool still = stationary.update(s.completedAt, gStartup.imuOk, s.countsL == 0 && s.countsR == 0);
    if (motorsEnabled() || s.completedAt - gLastRecovery < reliability::RETRY_MS) return false;
    bool imuRetry = !gStartup.imuOk || s.gyroSamples == 0;
    bool tofRetry = tofNeedsRecovery(s.completedAt);
    bool calibrationRetry = !gStartup.calibrated && still;
    if (!imuRetry && !tofRetry && !calibrationRetry) return false;
    gLastRecovery = s.completedAt;
    ++gRuntime.recoveryAttempts;
    // Never reinitialize the shared bus while motion is enabled.
    if (!gI2c.recover()) return true;
    if (imuRetry) gStartup.imuOk = gImu.init();
    if (tofRetry) gStartup.tofUp = tofInit(gI2c);
    // Retry startup calibration only; retain an established bias across outages.
    if (calibrationRetry && gStartup.imuOk)
        gStartup.calibrated = gImu.calibrateGyroZ(cfg::GYRO_CALIBRATION_SAMPLES, cfg::GYRO_PERIOD_MS);
    stationary.reset();
    return true;  // Caller must collect a new snapshot before qualifying recovery.
}

RemoteCommands pollRemote() {
    RemoteCommands commands;
    comms::Packet p;
    while (remoteReceive(p)) {
        switch (p.type) {
            case comms::Type::Start: commands.start = true; break;
            case comms::Type::Stop: commands.stop = true; break;
            case comms::Type::Ping: break;
            case comms::Type::Drive:
                // Streamed at 10 Hz or more: no acknowledgement, telemetry shows the effect.
                if (comms::unpackDrive(p, commands.left, commands.right)) commands.drive = true;
                continue;
            default: continue;  // not a command: no acknowledgement
        }
        const uint8_t ack[comms::ACK_SIZE] = {static_cast<uint8_t>(p.type), p.seq};
        remoteSend(comms::Type::Ack, ack, sizeof(ack));
    }
    return commands;
}

static_assert(TOF_COUNT == comms::TELEMETRY_RANGES, "telemetry carries one range per ToF sensor");

void reportRuntime(const reliability::Supervisor& supervisor, const SensorStep& s, const nav::FusedOdometry& odo,
                   const float target[2]) {
    if (gRuntime.state != supervisor.state() || gRuntime.fault != supervisor.fault()) {
        printf("state %u fault %u\r\n", (unsigned)supervisor.state(), (unsigned)supervisor.fault());
    }
    gRuntime.state = supervisor.state();
    gRuntime.fault = supervisor.fault();
    gRuntime.commandExpiries = motorCommandExpiries();
    gRuntime.completedAt = s.completedAt;
    gRuntime.gyroAt = s.gyroAt;
    for (int i = 0; i < TOF_COUNT; ++i) {
        gRuntime.rangeStatus[i] = s.ranges[i].status;
        gRuntime.rangeTime[i] = s.ranges[i].timestamp;
        gRuntime.rangeFailures[i] = s.ranges[i].failures;
    }

    if (s.completedAt - gLastTelemetry < cfg::TELEMETRY_PERIOD_MS) return;
    gLastTelemetry = s.completedAt;
    comms::Telemetry t;
    t.ms = s.completedAt;
    t.x = odo.pose.x;
    t.y = odo.pose.y;
    t.th = odo.pose.th;
    t.v = odo.v;
    t.target[0] = target[0];
    t.target[1] = target[1];
    for (int i = 0; i < TOF_COUNT; ++i) {
        t.range[i] = s.tof[i];
        if (s.ranges[i].status == reliability::RangeStatus::Valid) t.validRanges |= 1u << i;
    }
    t.state = static_cast<uint8_t>(supervisor.state());
    t.fault = static_cast<uint8_t>(supervisor.fault());
    uint8_t payload[comms::TELEMETRY_SIZE];
    remoteSend(comms::Type::Telemetry, payload, comms::pack(t, payload));
}
