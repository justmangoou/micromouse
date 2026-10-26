/**
 * @file    tof_array.h
 * @brief   VL53L0X array on one I2C bus, told apart at start-up through their XSHUT lines. See tof_array.cpp for how
 *          the readings are taken.
 */
#pragma once

#include "i2c_bus.h"
#include "config.h"
#include "reliability.h"

constexpr int TOF_COUNT = cfg::TOF_COUNT;  // physical layout and wiring live in config.h

// Bring the sensors up (ST API, blocking, once) and keep `bus` for the run-time reads. Returns how many sensors
// came up. A sensor that failed is held in XSHUT and reports Missing status. `bus` must outlive the sensors.
int tofInit(I2cBus& bus);

// Latest distances in mm, clamped to the limits in config.h. Call once per control step.
// These values alone do not establish health; check the snapshot status and age.
void tofRead(float mm[TOF_COUNT]);

bool tofOk(int index);

// A good range (not an invalid/no-target result), no older than maxAgeMs.
bool tofReliable(int index, uint32_t maxAgeMs);

// Snapshot at the supplied completion time, without another bus transaction.
reliability::RangeReading tofReading(int index, uint32_t now);
bool tofNeedsRecovery(uint32_t now);
