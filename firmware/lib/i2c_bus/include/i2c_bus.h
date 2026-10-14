/**
 * @file    i2c_bus.h
 * @brief   Register-style read/write helpers over one STM32 HAL I2C peripheral, shared by every I2C driver.
 *
 * @par References (from memory: check against your copies)
 *   - RM0383, STM32F411xC/E reference manual, I2C chapter (transfer sequences, BUSY / AF / BERR flags)
 *   - ES0287, STM32F411xC/E errata sheet (I2C peripheral limitations; BUSY can stay set after a bus error)
 *
 * @par Notes
 *   - Blocking and polled. Call it from the main loop only, never from an interrupt: the ToF sensors and the IMU
 *     share this bus, and nothing here is reentrant.
 *   - Addresses are 7-bit (0x68, not 0xD0); the shift for the HAL is done here.
 *   - Register addresses are 8 bit. Multi-byte reads use the device's auto-increment.
 *   - A failed transfer is reported by the return value; lastStatus()/lastError() say why.
 */
#pragma once

#include <optional>
#include "stm32f4xx_hal.h"

class I2cBus {
public:
    explicit I2cBus(I2C_HandleTypeDef* handle, uint32_t timeoutMs = 10) : h_(handle), timeoutMs_(timeoutMs) {}

    bool writeReg(uint8_t addr7, uint8_t reg, uint8_t value);
    bool write(uint8_t addr7, uint8_t reg, const uint8_t* data, uint16_t len);

    bool read(uint8_t addr7, uint8_t reg, uint8_t* data, uint16_t len);
    std::optional<uint8_t> readReg(uint8_t addr7, uint8_t reg);

    // Read-modify-write: reg = (reg & ~mask) | (value & mask).
    bool updateReg(uint8_t addr7, uint8_t reg, uint8_t mask, uint8_t value);

    // True if a device acknowledges its address.
    bool probe(uint8_t addr7);

    // Re-initialise the peripheral with its current settings (clears BUSY and error flags). It does not free a bus
    // held low by a slave: that needs clock pulses on SCL, which this does not do.
    bool recover();

    HAL_StatusTypeDef lastStatus() const { return lastStatus_; }
    uint32_t lastError() const { return lastError_; }  // HAL_I2C_ERROR_* bits of the last failed transfer
    uint32_t failures() const { return failures_; }

private:
    bool done(HAL_StatusTypeDef st);

    I2C_HandleTypeDef* h_;
    uint32_t timeoutMs_;
    HAL_StatusTypeDef lastStatus_ = HAL_OK;
    uint32_t lastError_ = 0;
    uint32_t failures_ = 0;
};
