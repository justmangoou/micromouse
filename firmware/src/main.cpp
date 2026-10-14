/**
 * @file    main.cpp
 * @brief   Micromouse firmware, run 2 (autonomous). Black Pill, STM32 HAL via PlatformIO.
 *
 * @par Flow
 *   power on -> calibrate the gyro while locked on the line -> plan on the stored map -> wait for the button
 *   -> run (30 Hz: ToF, odometry, navigation; 200 Hz interrupt: wheel speed loop) -> stop over the finish line
 */
#include "nav_mode.h"

#if defined(LED_BLINK_TEST)

#include "board.h"

int main() {
    boardInit();

    while (true) {
        ledSet(true);
        HAL_Delay(500);
        ledSet(false);
        HAL_Delay(500);
    }
}

#elif NAV_MODE == NAV_MICROMOUSE

#include <stdio.h>
#include "board.h"
#include "floodfill.h"
#include "motor.h"

namespace {
nav::FloodFill gMaze;
}

int main() {
    boardInit();
    wheelInit();
    wheelEnable(false);
    nav::Cell current{0, 0};
    nav::Direction heading = nav::Direction::North;
    nav::Cell next;
    nav::Direction direction;
    printf("\r\nmicromouse: flood fill ready; cell sensing and motion required\r\n");
    if (gMaze.nextMove(current, heading, next, direction)) {
        printf("first target: %d,%d heading %d\r\n", next.x, next.y, static_cast<int>(direction));
    }
    // Supply wall observations and execute cell moves before enabling motors.
    while (true) {
        ledSet(true);
        HAL_Delay(500);
        ledSet(false);
        HAL_Delay(500);
    }
}

#else  // NAV_OBSTACLE_AVOIDANCE

#include <stdio.h>

#include "board.h"
#include "i2c_bus.h"
#include "config.h"
#include "map_data.h"
#include "mpu6050.h"
#include "navigator.h"
#include "odometry.h"
#include "tof_array.h"
#include "motor.h"

namespace {

enum class State { Calibrating, Ready, Running, Done };

nav::Navigator gNav;  // ~65 KB of planner arrays, static on purpose
I2cBus gI2c(&hi2c1);  // shared by the IMU (and any later I2C device)
Mpu6050 gImu(gI2c);

nav::OdomParams odomParams() {
    nav::OdomParams p;
    p.wheelRadius = cfg::WHEEL_RADIUS_CM;
    p.track = cfg::WHEEL_TRACK_CM;
    p.countsPerRev = cfg::COUNTS_PER_REV;
    return p;
}

}  // namespace

int main() {
    boardInit();
    printf("\r\nmicromouse: %d map blocks\r\n", MAP_BLOCK_COUNT);

    wheelInit();
    speedLoopStart();

    bool imuOk = gImu.init();
    int tofUp = tofInit(gI2c);
    printf("imu %s, tof %d/%d up\r\n", imuOk ? "ok" : "FAILED", tofUp, TOF_COUNT);

    gNav.init(MAP_BLOCKS, MAP_BLOCK_COUNT);

    nav::FusedOdometry odo(odomParams());
    odo.reset({cfg::START_X, cfg::START_Y, cfg::START_TH});

    State state = State::Calibrating;
    unsigned calibSteps = 0, pressed = 0;
    uint32_t nextGyro = HAL_GetTick(), nextCtrl = HAL_GetTick(), lastCtrl = HAL_GetTick(), runStart = 0;
    float gyroSum = 0, gyroLast = 0;
    int gyroN = 0;
    float tof[TOF_COUNT];

    while (true) {
        uint32_t now = HAL_GetTick();

        if (now >= nextGyro) {
            nextGyro += cfg::GYRO_PERIOD_MS;
            if (auto g = gImu.readGyroZ()) {
                gyroSum += cfg::GYRO_SIGN * *g;
                gyroN++;
            }
        }

        if (now < nextCtrl) continue;
        nextCtrl += cfg::CONTROL_PERIOD_MS;
        float dt = (now - lastCtrl) * 0.001f;
        lastCtrl = now;
        if (dt < 0.005f) dt = cfg::CONTROL_PERIOD_MS * 0.001f;

        int32_t dcL, dcR;
        wheelTakeCounts(&dcL, &dcR);
        if (gyroN > 0) gyroLast = gyroSum / gyroN;
        gyroSum = 0;
        gyroN = 0;
        tofRead(tof);

        odo.update(dcL, dcR, gyroLast, dt, state == State::Calibrating);
        math::Pose est = odo.pose;

        switch (state) {
            case State::Calibrating:
                ledSet((calibSteps / 5) % 2 == 0);
                if (++calibSteps >= cfg::CALIBRATION_STEPS) {
                    odo.pose = {cfg::START_X, cfg::START_Y, cfg::START_TH};  // keep the bias, drop any drift while locked
                    uint32_t t0 = HAL_GetTick();
                    gNav.start(odo.pose, 0.0f);
                    printf("bias %d mdeg/s, planned %d points, %d replans, %lu ms\r\n", (int)(odo.bias * 57295.78f),
                           gNav.path().n, gNav.replans(), (unsigned long)(HAL_GetTick() - t0));
                    nextCtrl = HAL_GetTick() + cfg::CONTROL_PERIOD_MS;
                    state = imuOk ? State::Ready : State::Done;
                }
                break;

            case State::Ready:
                ledSet(((now / 500) % 2) == 0);
                pressed = buttonPressed() ? pressed + 1 : 0;
                if (pressed >= 3) {
                    runStart = now;
                    wheelEnable(true);
                    printf("run\r\n");
                    state = State::Running;
                }
                break;

            case State::Running: {
                float t = (now - runStart) * 0.001f;
                float duty[2];
                gNav.step(est, odo.v, tof, t, duty);
                wheelSetTargets(duty[0] * cfg::V_FULL_CM_S, duty[1] * cfg::V_FULL_CM_S);
                ledSet(true);

                printf("%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n", (unsigned long)(now - runStart), (int)(est.x * 10),
                       (int)(est.y * 10), (int)(est.th * 1000), (int)(odo.v * 10), (int)(duty[0] * 100),
                       (int)(duty[1] * 100), (int)(tof[0] * 10), (int)(tof[1] * 10), (int)(tof[2] * 10),
                       (int)(tof[3] * 10));

                bool atFinish = est.y < -9.0f && est.x > 125.0f && est.x < 175.0f && gNav.progress() > 0.8f;
                if (atFinish || t > cfg::RUN_TIME_LIMIT_S) {
                    wheelEnable(false);
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

#endif  // LED_BLINK_TEST
