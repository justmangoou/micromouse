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

#include "board.h"
#include "config.h"
#include "i2c_bus.h"
#include "motor.h"
#include "mpu6050.h"
#include "robot.h"

namespace {

I2cBus gI2c(&hi2c1);  // shared by the IMU and the ToF sensors
Mpu6050 gImu(gI2c);

bool gLoopStarted = false;
uint32_t gNextGyro, gNextStep, gLastStep;  // HAL ticks, ms

bool due(uint32_t now, uint32_t at) { return static_cast<int32_t>(now - at) >= 0; }  // wrap-safe

// Fixed rate. After a stall, skip the missed periods instead of running them back to back.
uint32_t nextTime(uint32_t at, uint32_t now, uint32_t period) {
    at += period;
    return due(now, at) ? now + period : at;
}

}  // namespace

nav::OdomParams odometryParams() {
    nav::OdomParams p;
    p.wheelRadius = cfg::WHEEL_RADIUS_MM;
    p.track = cfg::WHEEL_TRACK_MM;
    p.countsPerRev = cfg::COUNTS_PER_REV;
    return p;
}

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
                gyroSum += cfg::GYRO_SIGN * *g;
                gyroN++;
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
    step.gyroSamples = gyroN;
    step.gyro = gyroN > 0 ? gyroSum / gyroN : 0.0f;
    tofRead(step.tof);
}

int main() {
    boardInit();
    motorInit();
    speedLoopStart();

    StartupStatus status;
    status.imuOk = gImu.init();
    status.tofUp = tofInit(gI2c);
    printf("\r\nimu %s, tof %d/%d up\r\ngyro calibration: keep the robot still\r\n", status.imuOk ? "ok" : "FAILED",
           status.tofUp, TOF_COUNT);

    ledSet(true);
    status.calibrated = status.imuOk && gImu.calibrateGyroZ(cfg::GYRO_CALIBRATION_SAMPLES, cfg::GYRO_PERIOD_MS);
    printf("gyro calibration %s, bias %d mdeg/s\r\n", status.calibrated ? "ok" : "FAILED",
           (int)(cfg::GYRO_SIGN * gImu.gyroBiasZ() * 57295.78f));

    int32_t left, right;
    motorTakeCounts(&left, &right);  // drop whatever the wheels moved during start-up
    runMode(status);
}
