/**
 * @file    tof_array.h
 * @brief   VL53L0X array on one I2C bus, told apart at start-up through their XSHUT lines. See tof_array.cpp for how
 *          the readings are taken.
 */
#pragma once

#include "i2c_bus.h"
#include "config.h"
#include "reliability.h"

constexpr int TOF_COUNT = cfg::TOF_COUNT;      // navigation sensors: the only ones that can cause a sensor fault
constexpr int TOF_SENSORS = cfg::TOF_SENSORS;  // all sensors on the robot (the extras follow the navigation ones)

// Bring the sensors up (ST API, blocking, once) and keep `bus` for the run-time reads. Returns how many of the
// TOF_COUNT navigation sensors came up (the extra sensors are optional and not counted). A sensor that failed is held in XSHUT and reports Missing status. `bus` must outlive the sensors.
int tofInit(I2cBus& bus);

// Latest distances in mm, clamped to the limits in config.h. Call once per control step.
// These values alone do not establish health; check the snapshot status and age. Extra sensors (index >= TOF_COUNT)
// read cfg::TOF_MAX_MM unless their latest range is good and fresh.
void tofRead(float mm[TOF_SENSORS]);

bool tofOk(int index);

// A good range (not an invalid/no-target result), no older than maxAgeMs.
bool tofReliable(int index, uint32_t maxAgeMs);

// Snapshot at the supplied completion time, without another bus transaction.
reliability::RangeReading tofReading(int index, uint32_t now);
// Only the navigation sensors ask for recovery.
bool tofNeedsRecovery(uint32_t now);
