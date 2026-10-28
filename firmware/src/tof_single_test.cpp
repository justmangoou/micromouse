/**
 * @file    tof_single_test.cpp
 * @brief   Stationary bring-up of ONE VL53L0X (env:tof_single_test), through ST's API in single-shot mode. Sensor
 *          TOF_TEST_INDEX is woken through its XSHUT line (the others stay in reset), so only that sensor needs to
 *          be wired. Prints range in mm about 5 times a second as plain text for `pio device monitor`; the same
 *          numbers are in gTofSingle for ST-Link. The status LED blinks on every good range; fast blink = init failed.
 */
#include <stdio.h>

#include "board.h"
#include "config.h"
#include "i2c_bus.h"
#include "vl53l0x_api.h"
#include "vl53l0x_impl.h"

#ifndef TOF_TEST_INDEX
#define TOF_TEST_INDEX 0
#endif

struct TofSingleStatus {
    uint32_t stage;       // 0 reset, 1 model id read, 2 init done, 3 ranging
    int32_t apiError;     // last VL53L0X_Error
    uint32_t model;       // identification model id, expect 0xEE
    uint32_t rangeMm;     // last range
    uint32_t rangeStatus; // device range status, 0 = good
    uint32_t good;
    uint32_t bad;
    uint32_t busFailures;
};
volatile TofSingleStatus gTofSingle = {};

namespace {

I2cBus gI2c(&hi2c1);
VL53L0X_Dev_t gDev;

GPIO_TypeDef* const XSHUT_PORT[] = TOF_XSHUT_PORTS;
const uint16_t XSHUT_PIN[] = TOF_XSHUT_PINS;

[[noreturn]] void fail(const char* what, VL53L0X_Error err) {
    gTofSingle.apiError = err;
    while (true) {
        printf("FAIL %s err=%d model=0x%02lX bus failures=%lu lastErr=0x%lX\n", what, (int)err,
               (unsigned long)gTofSingle.model, (unsigned long)gI2c.failures(), (unsigned long)gI2c.lastError());
        for (int i = 0; i < 20; ++i) {
            ledStatusSet((HAL_GetTick() / 100) % 2 == 0);
            HAL_Delay(50);
        }
    }
}

#define CHECK(call) do { VL53L0X_Error e_ = (call); if (e_ != VL53L0X_ERROR_NONE) fail(#call, e_); } while (0)

}  // namespace

int main() {
    boardInit();
    HAL_Delay(200);  // let the terminal attach
    printf("\nVL53L0X single test, sensor %d\n", TOF_TEST_INDEX);

    VL53L0X_SetBus(&gI2c);
    for (size_t i = 0; i < sizeof(XSHUT_PIN) / sizeof(XSHUT_PIN[0]); ++i)
        HAL_GPIO_WritePin(XSHUT_PORT[i], XSHUT_PIN[i], GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(XSHUT_PORT[TOF_TEST_INDEX], XSHUT_PIN[TOF_TEST_INDEX], GPIO_PIN_SET);
    HAL_Delay(10);

    gDev.I2cDevAddr = 0x52;  // power-on default (0x29 7-bit)
    gDev.comms_type = 1;
    gDev.comms_speed_khz = 400;

    uint8_t model = 0;
    CHECK(VL53L0X_RdByte(&gDev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &model));
    gTofSingle.model = model;
    if (model != 0xEE) fail("model id (expect 0xEE)", VL53L0X_ERROR_CONTROL_INTERFACE);
    gTofSingle.stage = 1;

    uint8_t vhv = 0, phase = 0, aperture = 0;
    uint32_t refSpads = 0;
    CHECK(VL53L0X_DataInit(&gDev));
    CHECK(VL53L0X_StaticInit(&gDev));
    CHECK(VL53L0X_PerformRefCalibration(&gDev, &vhv, &phase));
    CHECK(VL53L0X_PerformRefSpadManagement(&gDev, &refSpads, &aperture));
    CHECK(VL53L0X_SetDeviceMode(&gDev, VL53L0X_DEVICEMODE_SINGLE_RANGING));
    gTofSingle.stage = 2;
    printf("init ok: vhv=%u phase=%u refSpads=%lu aperture=%u\n", vhv, phase, (unsigned long)refSpads, aperture);
    gTofSingle.stage = 3;

    while (true) {
        VL53L0X_RangingMeasurementData_t m;
        VL53L0X_Error e = VL53L0X_PerformSingleRangingMeasurement(&gDev, &m);
        if (e != VL53L0X_ERROR_NONE) {
            gTofSingle.apiError = e;
            ++gTofSingle.bad;
            printf("error %d (bus failures=%lu)\n", (int)e, (unsigned long)gI2c.failures());
        } else {
            gTofSingle.rangeMm = m.RangeMilliMeter;
            gTofSingle.rangeStatus = m.RangeStatus;
            if (m.RangeStatus == 0) {
                ++gTofSingle.good;
                ledStatusSet(gTofSingle.good % 2);
                printf("%5u mm\n", m.RangeMilliMeter);
            } else {
                ++gTofSingle.bad;
                printf("  --  mm (status %u)\n", m.RangeStatus);
            }
        }
        gTofSingle.busFailures = gI2c.failures();
        HAL_Delay(200);
    }
}
