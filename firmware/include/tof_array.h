/**
 * @file    tof_array.h
 * @brief   Four VL53L0X on one I2C bus, told apart at start-up through their XSHUT lines. See tof_array.cpp for how
 *          the readings are taken.
 */
#pragma once

#include "i2c_bus.h"
#include "tof_layout.h"

constexpr int TOF_COUNT = tof::COUNT;  // the layout lives in lib/nav/include/tof_layout.h

// Bring the sensors up (ST API, blocking, once) and keep `bus` for the run-time reads. Returns how many of the four
// came up. A sensor that failed always reads "no target". `bus` must outlive the sensors.
int tofInit(I2cBus& bus);

// Latest distances in mm, clamped to [30, 1200]. Call once per control step.
void tofRead(float mm[TOF_COUNT]);

bool tofOk(int index);

// A good range (not an invalid/no-target result), no older than maxAgeMs.
bool tofReliable(int index, uint32_t maxAgeMs);
