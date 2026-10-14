/**
 * @file    vl53l0x_impl.h
 * @brief   STM32 platform layer for ST's VL53L0X API, on top of the shared I2cBus.
 *
 * ST's API calls a fixed set of I2C functions (VL53L0X_WrByte, VL53L0X_ReadMulti, ...; declared in vl53l0x_platform.h)
 * and leaves them to the platform. This library defines them. The API's Dev handle has no bus field, and this board has
 * one I2C bus, so the bus is set once with VL53L0X_SetBus().
 *
 * Device addresses follow ST's convention: VL53L0X_Dev_t::I2cDevAddr is the 8-bit form (0x52 at power-on = 7-bit 0x29).
 */
#pragma once

#include "i2c_bus.h"

// Call before any VL53L0X_* function. `bus` must outlive every sensor.
void VL53L0X_SetBus(I2cBus* bus);
