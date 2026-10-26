/**
 * @file    tof_array.cpp
 * @brief   VL53L0X array on one I2C bus. Start-up goes through ST's API; run-time reads are a direct burst read of
 *          the result registers (3 bus transactions per sensor instead of ~8 through the API).
 *
 * @par References (from memory: check against your copies)
 *   - VL53L0X datasheet, DS11555
 *   - VL53L0X API user manual, UM2039
 *   - the register layout below is the one ST's VL53L0X_GetRangingMeasurementData reads (see
 *     lib/vl53l0x_api/src/vl53l0x_api.c and VL53L0X_get_pal_range_status in vl53l0x_api_core.c)
 */
#include "tof_array.h"

#include "board.h"
#include "config.h"
#include "i2c_bus.h"
#include "vl53l0x_api.h"
#include "vl53l0x_impl.h"

namespace {

VL53L0X_Dev_t gDev[TOF_COUNT];
bool gOk[TOF_COUNT];
float gLast[TOF_COUNT];
bool gReliable[TOF_COUNT];
uint32_t gRangeTime[TOF_COUNT];
I2cBus* gBus = nullptr;
uint32_t gFailStreak[TOF_COUNT] = {};
uint32_t gFailures[TOF_COUNT] = {};
reliability::RangeStatus gStatus[TOF_COUNT];
uint32_t gInitTime = 0;

GPIO_TypeDef* const XSHUT_PORT[] = TOF_XSHUT_PORTS;
const uint16_t XSHUT_PIN[] = TOF_XSHUT_PINS;
static_assert(sizeof(XSHUT_PORT) / sizeof(XSHUT_PORT[0]) == TOF_COUNT && sizeof(XSHUT_PIN) / sizeof(XSHUT_PIN[0]) == TOF_COUNT,
              "config.h needs one XSHUT port and pin per sensor in cfg::TOF_COUNT");

inline uint8_t newAddress(int i) { return (uint8_t)(0x60 + 2 * i); }  // 8-bit form; 7-bit addresses start at 0x30
inline uint8_t addr7(int i) { return (uint8_t)(newAddress(i) >> 1); }

// Result registers. A burst read from 0x13 gives:
//   b[0]      RESULT_INTERRUPT_STATUS (bits 2:0 non-zero = a new sample is ready)
//   b[1]      RESULT_RANGE_STATUS (bits 6:3 = device range status, 11 = good ranging)
//   b[7..8]   peak signal rate, 9.7 fixed point (Mcps = raw / 128)
//   b[11..12] range in mm, big endian (ST's default has no linearity gain or fractional mode)
// SYSTEM_INTERRUPT_CLEAR is pulsed 1 then 0, as VL53L0X_ClearInterruptMask does.
constexpr uint8_t REG_INT_CLEAR = 0x0B;
constexpr uint8_t REG_RESULT = 0x13;
constexpr int RESULT_LEN = 13;
constexpr uint8_t STATUS_GOOD = 11;
constexpr uint16_t MIN_SIGNAL_RAW = 13;  // ~0.1 Mcps; below this the return is too weak to trust
constexpr int RECOVER_AFTER_FAILURES = 10;

enum class Sample { NotReady, Valid, NoTarget, BusError };

Sample readSample(int i, float* range) {
    uint8_t b[RESULT_LEN];
    if (!gBus->read(addr7(i), REG_RESULT, b, RESULT_LEN)) return Sample::BusError;
    if ((b[0] & 0x07) == 0) return Sample::NotReady;

    if (!gBus->writeReg(addr7(i), REG_INT_CLEAR, 0x01) ||
        !gBus->writeReg(addr7(i), REG_INT_CLEAR, 0x00)) return Sample::BusError;

    uint8_t status = (b[1] & 0x78) >> 3;
    uint16_t signal = (uint16_t)((b[7] << 8) | b[8]);
    uint16_t rangeMm = (uint16_t)((b[11] << 8) | b[12]);
    if (status != STATUS_GOOD || signal < MIN_SIGNAL_RAW || rangeMm >= cfg::TOF_MAX_MM) return Sample::NoTarget;
    *range = static_cast<float>(rangeMm);
    return Sample::Valid;
}

bool bringUp(int i) {
    VL53L0X_Dev_t* d = &gDev[i];
    d->I2cDevAddr = 0x52;  // power-on default (0x29 in 7-bit form)
    d->comms_type = 1;
    d->comms_speed_khz = 400;

    uint8_t vhv = 0, phase = 0, apertureSpads = 0;
    uint32_t refSpadCount = 0;

    // The bundled ST API's WaitDeviceBooted always returns NOT_IMPLEMENTED.
    // tofInit already waits 10 ms after XSHUT; verify the model before setup.
    uint8_t model = 0;
    if (VL53L0X_RdByte(d, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &model) != VL53L0X_ERROR_NONE || model != 0xEE)
        return false;
    if (VL53L0X_DataInit(d) != VL53L0X_ERROR_NONE) return false;
    if (VL53L0X_SetDeviceAddress(d, newAddress(i)) != VL53L0X_ERROR_NONE) return false;
    d->I2cDevAddr = newAddress(i);
    if (VL53L0X_StaticInit(d) != VL53L0X_ERROR_NONE) return false;
    if (VL53L0X_PerformRefCalibration(d, &vhv, &phase) != VL53L0X_ERROR_NONE) return false;
    if (VL53L0X_PerformRefSpadManagement(d, &refSpadCount, &apertureSpads) != VL53L0X_ERROR_NONE) return false;
    if (VL53L0X_SetDeviceMode(d, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING) != VL53L0X_ERROR_NONE) return false;
    if (VL53L0X_SetMeasurementTimingBudgetMicroSeconds(d, cfg::CONTROL_PERIOD_MS * 1000) != VL53L0X_ERROR_NONE) return false;
    return VL53L0X_StartMeasurement(d) == VL53L0X_ERROR_NONE;
}

}  // namespace

int tofInit(I2cBus& bus) {
    gBus = &bus;
    VL53L0X_SetBus(&bus);
    gInitTime = HAL_GetTick();
    for (int i = 0; i < TOF_COUNT; i++) {
        HAL_GPIO_WritePin(XSHUT_PORT[i], XSHUT_PIN[i], GPIO_PIN_RESET);
        gOk[i] = false;
        gLast[i] = cfg::TOF_MAX_MM;
        gReliable[i] = false;
        gRangeTime[i] = 0;
        gFailStreak[i] = 0;
        gStatus[i] = reliability::RangeStatus::Missing;
    }
    HAL_Delay(10);
    int up = 0;
    for (int i = 0; i < TOF_COUNT; i++) {  // one at a time: they all wake up at the same address
        HAL_GPIO_WritePin(XSHUT_PORT[i], XSHUT_PIN[i], GPIO_PIN_SET);
        HAL_Delay(10);
        gOk[i] = bringUp(i);
        if (gOk[i]) {
            up++;
            gStatus[i] = reliability::RangeStatus::NotReady;
        } else {
            HAL_GPIO_WritePin(XSHUT_PORT[i], XSHUT_PIN[i], GPIO_PIN_RESET);
        }
    }
    return up;
}

void tofRead(float mm[TOF_COUNT]) {
    for (int i = 0; i < TOF_COUNT; i++) {
        if (gOk[i]) {
            float v;
            switch (readSample(i, &v)) {
                case Sample::Valid:
                    gLast[i] = v;
                    gReliable[i] = true;
                    gStatus[i] = reliability::RangeStatus::Valid;
                    gRangeTime[i] = HAL_GetTick();
                    gFailStreak[i] = 0;
                    break;
                case Sample::NoTarget:  // invalid measurements are not evidence of clear space
                    gReliable[i] = false;
                    gLast[i] = cfg::TOF_MAX_MM;
                    gStatus[i] = reliability::RangeStatus::Invalid;
                    gRangeTime[i] = HAL_GetTick();
                    gFailStreak[i] = 0;
                    break;
                case Sample::NotReady:  // keep the previous value until the next measurement completes
                    gFailStreak[i] = 0;
                    break;
                case Sample::BusError:
                    gReliable[i] = false;
                    gStatus[i] = reliability::RangeStatus::BusError;
                    ++gFailures[i];
                    if (gFailStreak[i] < UINT32_MAX) ++gFailStreak[i];
                    break;
            }
        }
        float v = gLast[i];
        mm[i] = v < cfg::TOF_MIN_MM ? cfg::TOF_MIN_MM : (v > cfg::TOF_MAX_MM ? cfg::TOF_MAX_MM : v);
    }
}

bool tofOk(int index) { return index >= 0 && index < TOF_COUNT && gOk[index]; }

bool tofReliable(int index, uint32_t maxAgeMs) {
    return index >= 0 && index < TOF_COUNT && gOk[index] && gReliable[index] &&
           HAL_GetTick() - gRangeTime[index] <= maxAgeMs;
}

reliability::RangeReading tofReading(int index, uint32_t now) {
    if (index < 0 || index >= TOF_COUNT) return {};
    auto status = gStatus[index];
    if ((status == reliability::RangeStatus::Valid || status == reliability::RangeStatus::Invalid) &&
        !reliability::fresh(now, gRangeTime[index])) status = reliability::RangeStatus::Stale;
    return {gLast[index], gRangeTime[index], status, gFailures[index]};
}

bool tofNeedsRecovery(uint32_t now) {
    for (int i = 0; i < TOF_COUNT; ++i) {
        if (!gOk[i] || gFailStreak[i] >= RECOVER_AFTER_FAILURES) return true;
        if (gStatus[i] == reliability::RangeStatus::NotReady && now - gInitTime >= 1000) return true;
        if (gOk[i] && now - gInitTime >= 1000 &&
            (gStatus[i] == reliability::RangeStatus::Valid || gStatus[i] == reliability::RangeStatus::Invalid) &&
            now - gRangeTime[i] >= 1000) return true;
    }
    return false;
}
