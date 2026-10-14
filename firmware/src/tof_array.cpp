/**
 * @file    tof_array.cpp
 * @brief   Four VL53L0X on one I2C bus. Start-up goes through ST's API; run-time reads are a direct burst read of
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
#include "i2c_bus.h"
#include "pins.h"
#include "vl53l0x_api.h"
#include "vl53l0x_impl.h"

namespace {

VL53L0X_Dev_t gDev[TOF_COUNT];
bool gOk[TOF_COUNT];
float gLast[TOF_COUNT];
I2cBus* gBus = nullptr;
int gFailStreak = 0;  // consecutive failed bus reads, any sensor

GPIO_TypeDef* const XSHUT_PORT[] = TOF_XSHUT_PORTS;
const uint16_t XSHUT_PIN[] = TOF_XSHUT_PINS;
static_assert(sizeof(XSHUT_PORT) / sizeof(XSHUT_PORT[0]) == TOF_COUNT && sizeof(XSHUT_PIN) / sizeof(XSHUT_PIN[0]) == TOF_COUNT,
              "pins.h needs one XSHUT port and pin per sensor in tof::COUNT");

constexpr float MAX_CM = 120.0f;
constexpr float MIN_CM = 3.0f;

inline uint8_t newAddress(int i) { return (uint8_t)(0x60 + 2 * i); }  // 8-bit form, 7-bit 0x30..0x34
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

Sample readSample(int i, float* cm) {
    uint8_t b[RESULT_LEN];
    if (!gBus->read(addr7(i), REG_RESULT, b, RESULT_LEN)) return Sample::BusError;
    if ((b[0] & 0x07) == 0) return Sample::NotReady;

    gBus->writeReg(addr7(i), REG_INT_CLEAR, 0x01);
    gBus->writeReg(addr7(i), REG_INT_CLEAR, 0x00);

    uint8_t status = (b[1] & 0x78) >> 3;
    uint16_t signal = (uint16_t)((b[7] << 8) | b[8]);
    uint16_t mm = (uint16_t)((b[11] << 8) | b[12]);
    if (status != STATUS_GOOD || signal < MIN_SIGNAL_RAW || mm >= 1200) return Sample::NoTarget;
    *cm = mm / 10.0f;
    return Sample::Valid;
}

bool bringUp(int i) {
    VL53L0X_Dev_t* d = &gDev[i];
    d->I2cDevAddr = 0x52;  // power-on default (0x29 in 7-bit form)
    d->comms_type = 1;
    d->comms_speed_khz = 400;

    uint8_t vhv = 0, phase = 0, apertureSpads = 0;
    uint32_t refSpadCount = 0;

    if (VL53L0X_WaitDeviceBooted(d) != VL53L0X_ERROR_NONE) return false;
    if (VL53L0X_DataInit(d) != VL53L0X_ERROR_NONE) return false;
    if (VL53L0X_SetDeviceAddress(d, newAddress(i)) != VL53L0X_ERROR_NONE) return false;
    d->I2cDevAddr = newAddress(i);
    if (VL53L0X_StaticInit(d) != VL53L0X_ERROR_NONE) return false;
    if (VL53L0X_PerformRefCalibration(d, &vhv, &phase) != VL53L0X_ERROR_NONE) return false;
    if (VL53L0X_PerformRefSpadManagement(d, &refSpadCount, &apertureSpads) != VL53L0X_ERROR_NONE) return false;
    if (VL53L0X_SetDeviceMode(d, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING) != VL53L0X_ERROR_NONE) return false;
    if (VL53L0X_SetMeasurementTimingBudgetMicroSeconds(d, 33000) != VL53L0X_ERROR_NONE) return false;
    return VL53L0X_StartMeasurement(d) == VL53L0X_ERROR_NONE;
}

}  // namespace

int tofInit(I2cBus& bus) {
    gBus = &bus;
    VL53L0X_SetBus(&bus);
    gFailStreak = 0;
    for (int i = 0; i < TOF_COUNT; i++) {
        HAL_GPIO_WritePin(XSHUT_PORT[i], XSHUT_PIN[i], GPIO_PIN_RESET);
        gOk[i] = false;
        gLast[i] = MAX_CM;
    }
    HAL_Delay(10);
    int up = 0;
    for (int i = 0; i < TOF_COUNT; i++) {  // one at a time: they all wake up at the same address
        HAL_GPIO_WritePin(XSHUT_PORT[i], XSHUT_PIN[i], GPIO_PIN_SET);
        HAL_Delay(10);
        gOk[i] = bringUp(i);
        if (gOk[i]) up++;
    }
    return up;
}

void tofRead(float cm[TOF_COUNT]) {
    for (int i = 0; i < TOF_COUNT; i++) {
        if (gOk[i]) {
            float v;
            switch (readSample(i, &v)) {
                case Sample::Valid:
                    gLast[i] = v;
                    gFailStreak = 0;
                    break;
                case Sample::NoTarget:  // anything but a good ranging counts as "nothing in range"
                    gLast[i] = MAX_CM;
                    gFailStreak = 0;
                    break;
                case Sample::NotReady:  // keep the previous value until the next measurement completes
                    gFailStreak = 0;
                    break;
                case Sample::BusError:
                    if (++gFailStreak >= RECOVER_AFTER_FAILURES) {
                        gBus->recover();
                        gFailStreak = 0;
                    }
                    break;
            }
        }
        float v = gLast[i];
        cm[i] = v < MIN_CM ? MIN_CM : (v > MAX_CM ? MAX_CM : v);
    }
}

bool tofOk(int index) { return gOk[index]; }
