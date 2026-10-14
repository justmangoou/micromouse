/**
 * @file    vl53l0x_impl.cpp
 * @brief   Implements the I2C functions ST's VL53L0X API expects. See vl53l0x_impl.h.
 */
#include "stm32f4xx_hal.h"
#include "vl53l0x_platform.h"
#include "vl53l0x_impl.h"

namespace {

I2cBus* gBus = nullptr;

inline uint8_t addr7(VL53L0X_DEV dev) { return (uint8_t)(dev->I2cDevAddr >> 1); }

inline VL53L0X_Error result(bool ok) { return ok ? VL53L0X_ERROR_NONE : VL53L0X_ERROR_CONTROL_INTERFACE; }

}  // namespace

void VL53L0X_SetBus(I2cBus* bus) { gBus = bus; }

extern "C" {

// One task, one bus: nothing to lock.
VL53L0X_Error VL53L0X_LockSequenceAccess(VL53L0X_DEV Dev) { return VL53L0X_ERROR_NONE; }
VL53L0X_Error VL53L0X_UnlockSequenceAccess(VL53L0X_DEV Dev) { return VL53L0X_ERROR_NONE; }

VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t* pdata, uint32_t count) {
    if (!gBus || count > 0xFFFF) return VL53L0X_ERROR_CONTROL_INTERFACE;
    return result(gBus->write(addr7(Dev), index, pdata, (uint16_t)count));
}

VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t* pdata, uint32_t count) {
    if (!gBus || count > 0xFFFF) return VL53L0X_ERROR_CONTROL_INTERFACE;
    return result(gBus->read(addr7(Dev), index, pdata, (uint16_t)count));
}

VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV Dev, uint8_t index, uint8_t data) {
    if (!gBus) return VL53L0X_ERROR_CONTROL_INTERFACE;
    return result(gBus->writeReg(addr7(Dev), index, data));
}

// The sensor's multi-byte registers are big endian.
VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV Dev, uint8_t index, uint16_t data) {
    uint8_t b[2] = {(uint8_t)(data >> 8), (uint8_t)data};
    return VL53L0X_WriteMulti(Dev, index, b, 2);
}

VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t data) {
    uint8_t b[4] = {(uint8_t)(data >> 24), (uint8_t)(data >> 16), (uint8_t)(data >> 8), (uint8_t)data};
    return VL53L0X_WriteMulti(Dev, index, b, 4);
}

VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV Dev, uint8_t index, uint8_t* data) {
    return VL53L0X_ReadMulti(Dev, index, data, 1);
}

VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV Dev, uint8_t index, uint16_t* data) {
    uint8_t b[2];
    VL53L0X_Error st = VL53L0X_ReadMulti(Dev, index, b, 2);
    if (st == VL53L0X_ERROR_NONE) *data = (uint16_t)((b[0] << 8) | b[1]);
    return st;
}

VL53L0X_Error VL53L0X_RdDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t* data) {
    uint8_t b[4];
    VL53L0X_Error st = VL53L0X_ReadMulti(Dev, index, b, 4);
    if (st == VL53L0X_ERROR_NONE) *data = ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8) | b[3];
    return st;
}

// reg = (reg & AndData) | OrData
VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV Dev, uint8_t index, uint8_t AndData, uint8_t OrData) {
    uint8_t v;
    VL53L0X_Error st = VL53L0X_RdByte(Dev, index, &v);
    if (st != VL53L0X_ERROR_NONE) return st;
    return VL53L0X_WrByte(Dev, index, (uint8_t)((v & AndData) | OrData));
}

// Used inside the API's wait loops (boot, calibration).
VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV Dev) {
    HAL_Delay(2);
    return VL53L0X_ERROR_NONE;
}

}  // extern "C"
