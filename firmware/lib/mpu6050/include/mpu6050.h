/**
 * @file    mpu6050.h
 * @brief   MPU6050 gyro-z driver over I2C (STM32 HAL). The bus is passed in, so the library does not depend on the
 *          board setup.
 */
#pragma once

#include <optional>
#include "i2c_bus.h"

class Mpu6050 {
public:
    // addr7: 0x68 with AD0 low, 0x69 with AD0 high
    explicit Mpu6050(I2cBus& bus, uint8_t addr7 = 0x68) : bus_(bus), addr7_(addr7) {}

    // Reset, wake, 42 Hz low-pass, 200 Hz sample rate, +-500 deg/s. False if the chip does not answer.
    bool init();

    // Blocking calibration: keep the robot stationary for the entire sample window.
    // A failed read aborts and preserves the previous offset. Samples are >=5 ms
    // apart to match the configured 200 Hz sample rate. Call after init().
    bool calibrateGyroZ(uint16_t samples = 400, uint32_t samplePeriodMs = 5);
    float gyroBiasZ() const { return gyroBiasZ_; }  // sensor-frame rad/s

    // Offset-corrected yaw rate in rad/s, positive = counter-clockwise for a chip mounted flat, top up. Flip the sign in the caller
    // if your mounting differs. Empty if the I2C read failed.
    std::optional<float> readGyroZ();

private:
    bool write(uint8_t reg, uint8_t value);
    std::optional<float> readRawGyroZ();

    I2cBus& bus_;
    uint8_t addr7_;
    float gyroBiasZ_ = 0.0f;
    bool initialized_ = false;
};
