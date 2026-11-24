/**
 * @file    i2c_bus.cpp
 * @brief   I2cBus implementation: thin wrappers over HAL_I2C_Mem_Read/Write plus failure bookkeeping. See i2c_bus.h.
 */
#include "i2c_bus.h"

namespace {

inline uint16_t hal(uint8_t addr7) { return (uint16_t)(addr7 << 1); }

}  // namespace

bool I2cBus::done(HAL_StatusTypeDef st) {
    lastStatus_ = st;
    if (st == HAL_OK) return true;
    lastError_ = HAL_I2C_GetError(h_);
    failures_++;
    return false;
}

bool I2cBus::write(uint8_t addr7, uint8_t reg, const uint8_t* data, uint16_t len) {
    // the HAL takes a non-const pointer but does not modify the data of a write
    return done(HAL_I2C_Mem_Write(h_, hal(addr7), reg, I2C_MEMADD_SIZE_8BIT, const_cast<uint8_t*>(data), len, timeoutMs_));
}

bool I2cBus::writeReg(uint8_t addr7, uint8_t reg, uint8_t value) { return write(addr7, reg, &value, 1); }

bool I2cBus::read(uint8_t addr7, uint8_t reg, uint8_t* data, uint16_t len) {
    return done(HAL_I2C_Mem_Read(h_, hal(addr7), reg, I2C_MEMADD_SIZE_8BIT, data, len, timeoutMs_));
}

std::optional<uint8_t> I2cBus::readReg(uint8_t addr7, uint8_t reg) {
    uint8_t v;
    if (!read(addr7, reg, &v, 1)) return std::nullopt;
    return v;
}

bool I2cBus::updateReg(uint8_t addr7, uint8_t reg, uint8_t mask, uint8_t value) {
    auto cur = readReg(addr7, reg);
    if (!cur) return false;
    return writeReg(addr7, reg, (uint8_t)((*cur & ~mask) | (value & mask)));
}

bool I2cBus::probe(uint8_t addr7) { return done(HAL_I2C_IsDeviceReady(h_, hal(addr7), 2, timeoutMs_)); }

bool I2cBus::recover() {
    if (HAL_I2C_DeInit(h_) != HAL_OK) return done(HAL_ERROR);
    return done(HAL_I2C_Init(h_));
}
