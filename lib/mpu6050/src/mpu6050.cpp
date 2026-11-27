/**
 * @file    mpu6050.cpp
 * @brief   MPU6050 gyro-z driver, implementation. See mpu6050.h.
 */
#include "mpu6050.h"
#include "math_util.h"

#include <cmath>
#include <optional>

namespace {

// Registers: RM-MPU-6000A-00, sections 4.2–4.4, 4.20, 4.30, 4.34.
// https://cdn.sparkfun.com/datasheets/Sensors/Accelerometers/RM-MPU-6000A.pdf
constexpr uint8_t REG_SMPLRT_DIV   = 0x19;
constexpr uint8_t REG_CONFIG       = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG  = 0x1B;
constexpr uint8_t REG_GYRO_ZOUT_H  = 0x47;
constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;
constexpr uint8_t REG_WHO_AM_I     = 0x75;

// WHO_AM_I stays 0x68 regardless of AD0 (bus address 0x68 or 0x69).
constexpr uint8_t WHO_AM_I_VAL     = 0x68;

// Scale factors: ±1000 dps -> 32.8 LSB/(deg/s) -> rad/s direct factor. Not ±500: a turn on the spot passes 600 deg/s,
// and the clipped readings put the heading 5-10 deg wrong for the rest of the run.
constexpr float SCALE_1000DPS      = 32.8f;
constexpr float GYRO_Z_TO_RAD_S    = math::toRadian(1.0f) / SCALE_1000DPS;

} // namespace

bool Mpu6050::write(uint8_t reg, uint8_t val) {
    return bus_.writeReg(addr7_, reg, val);
}

bool Mpu6050::init() {
    initialized_ = false;
    auto id = bus_.readReg(addr7_, REG_WHO_AM_I);
    if (!id || *id != WHO_AM_I_VAL)
        return false;

    // Reset device
    if (!write(REG_PWR_MGMT_1, 0x80)) return false;
    HAL_Delay(100);

    // CLKSEL = 1: PLL with X gyro reference (steadier than the internal 8 MHz oscillator)
    if (!write(REG_PWR_MGMT_1, 0x01)) return false;

    // DLPF config: 42 Hz bandwidth, 1 kHz internal sample base
    if (!write(REG_CONFIG, 0x03)) return false;

    // Sample rate = 1kHz / (1 + 4) = 200 Hz
    if (!write(REG_SMPLRT_DIV, 0x04)) return false;

    // Gyro full scale: ±1000 deg/s (FS_SEL = 2 -> bits [4:3] = 10 -> 0x10)
    if (!write(REG_GYRO_CONFIG, 0x10)) return false;

    HAL_Delay(10);
    initialized_ = true;
    return true;
}

bool Mpu6050::calibrateGyroZ(uint16_t samples, uint32_t samplePeriodMs, float maxStd, float maxBias) {
    if (!initialized_ || samples == 0 || samplePeriodMs < 5) return false;
    const uint32_t start = HAL_GetTick();
    const uint32_t limitMs = 2u * samples * samplePeriodMs + 500u;
    float mean = 0.0f, m2 = 0.0f;  // Welford running mean and sum of squared deviations
    for (uint32_t i = 0; i < samples; i++) {
        if (i > 0) HAL_Delay(samplePeriodMs);
        if (HAL_GetTick() - start > limitMs) return false;
        auto sample = readRawGyroZ();
        if (!sample) return false;
        const float delta = *sample - mean;
        mean += delta / static_cast<float>(i + 1);
        m2 += delta * (*sample - mean);
    }
    if (maxStd > 0.0f && samples > 1 && std::sqrt(m2 / static_cast<float>(samples - 1)) > maxStd) return false;
    if (maxBias > 0.0f && std::fabs(mean) > maxBias) return false;
    gyroBiasZ_ = mean;
    return true;
}

std::optional<float> Mpu6050::readGyroZ() {
    auto raw = readRawGyroZ();
    if (!raw) return std::nullopt;
    return *raw - gyroBiasZ_;
}

std::optional<float> Mpu6050::readRawGyroZ() {
    if (!initialized_) return std::nullopt;
    uint8_t buf[2];
    if (!bus_.read(addr7_, REG_GYRO_ZOUT_H, buf, 2)) {
        return std::nullopt;
    }

    const int16_t raw_z = static_cast<int16_t>((buf[0] << 8) | buf[1]);
    return static_cast<float>(raw_z) * GYRO_Z_TO_RAD_S;
}
