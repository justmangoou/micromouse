/**
 * @file    imu_test.cpp
 * @brief   Stationary MPU6050 check (env:imu_test). Probes the chip, calibrates the gyro bias (keep the board still
 *          for about 2 s), then prints at 10 Hz as plain text for `pio device monitor`: yaw rate in deg/s and the
 *          yaw angle integrated from it. Rotate the board by hand: yaw should follow, and sit still at rest (slow
 *          drift = bias). The same numbers are in gImuTest for ST-Link. Fast status LED blink = chip not found.
 */
#include <stdio.h>

#include "board.h"
#include "i2c_bus.h"
#include "mpu6050.h"

struct ImuTestStatus {
    uint32_t stage;  // 0 reset, 1 found, 2 init done, 3 calibrated, 4 running
    float biasDegPerS;
    float rateDegPerS;
    float yawDeg;
    uint32_t samples;
    uint32_t readFailures;
};
volatile ImuTestStatus gImuTest = {};

namespace {

I2cBus gI2c(&hi2c1);
Mpu6050 gImu(gI2c);

constexpr float RAD_TO_DEG = 57.2957795f;

[[noreturn]] void fail(const char* what) {
    while (true) {
        printf("FAIL %s (bus failures=%lu lastErr=0x%lX)\n", what, (unsigned long)gI2c.failures(),
               (unsigned long)gI2c.lastError());
        for (int i = 0; i < 20; ++i) {
            ledStatusSet((HAL_GetTick() / 100) % 2 == 0);
            HAL_Delay(50);
        }
    }
}

}  // namespace

int main() {
    boardInit();
    HAL_Delay(200);  // let the terminal attach
    printf("\nMPU6050 test\n");

    if (!gI2c.probe(0x68)) fail("no ACK at 0x68 (check wiring / AD0)");
    gImuTest.stage = 1;
    auto who = gI2c.readReg(0x68, 0x75);  // WHO_AM_I, 0x68 on a genuine part
    printf("WHO_AM_I = 0x%02X\n", who ? *who : 0xFF);
    if (!gImu.init()) fail("init");
    gImuTest.stage = 2;

    printf("calibrating, keep still...\n");
    if (!gImu.calibrateGyroZ()) fail("calibration");
    gImuTest.stage = 3;
    gImuTest.biasDegPerS = gImu.gyroBiasZ() * RAD_TO_DEG;
    printf("bias = %.3f deg/s\n", gImuTest.biasDegPerS);
    gImuTest.stage = 4;

    float yaw = 0;
    uint32_t last = HAL_GetTick(), lastPrint = last;
    while (true) {
        uint32_t now = HAL_GetTick();
        if (now - last >= 5) {
            float dt = (now - last) * 1e-3f;
            last = now;
            if (auto g = gImu.readGyroZ()) {
                float rate = *g * RAD_TO_DEG;
                yaw += rate * dt;
                gImuTest.rateDegPerS = rate;
                gImuTest.yawDeg = yaw;
                ++gImuTest.samples;
            } else {
                ++gImuTest.readFailures;
            }
        }
        if (now - lastPrint >= 100) {
            lastPrint = now;
            ledStatusSet((now / 500) % 2 == 0);
            printf("rate %8.2f deg/s   yaw %8.2f deg   fails %lu\n", gImuTest.rateDegPerS, gImuTest.yawDeg,
                   (unsigned long)gImuTest.readFailures);
        }
    }
}
