/**
 * @file    test_tof_cal.cpp
 * @brief   Offset calibration of every VL53L0X (env:test_tof_cal). Needs a flat, matt grey or white target (a card)
 *          held square to the sensor at exactly CAL_MM from the sensor window. For each sensor in turn (the others
 *          stay in reset): hold the target, press KEY, keep it still while the LED is on (about 8 s); the tool runs ST's offset calibration, then prints 50 checking
 *          readings. At the end it prints the TOF_OFFSET_UM line to paste into include/config.h. Plain text on USART1
 *          (115200). Do it once per sensor and again if one is replaced.
 */
#include <stdio.h>

#include "board.h"
#include "config.h"
#include "i2c_bus.h"
#include "tof_array.h"
#include "vl53l0x_api.h"
#include "vl53l0x_impl.h"

// -DCAL_ONLY=n calibrates only sensor n (after replacing one); default -1 = all in order.
#ifndef CAL_ONLY
#define CAL_ONLY -1
#endif
#ifndef CAL_MM
#define CAL_MM 100
#endif

// Results for ST-Link (no serial adapter needed): read gTofCal after each sensor.
struct TofCalStatus {
    uint32_t sensor;      // the sensor being calibrated
    uint32_t doneMask;    // bit i set when sensor i finished
    int32_t offsetUm[TOF_COUNT];
    int32_t mean[TOF_COUNT], lo[TOF_COUNT], hi[TOF_COUNT], valid[TOF_COUNT];  // mm, the 50 checking readings
    int32_t error[TOF_COUNT];  // 0 ok, 1 bring-up failed, else the API error
};
volatile TofCalStatus gTofCal = {};

namespace {

I2cBus gI2c(&hi2c1);
VL53L0X_Dev_t gDev;
int32_t gOffsetUm[TOF_COUNT];
bool gDone[TOF_COUNT];

GPIO_TypeDef* const XSHUT_PORT[] = TOF_XSHUT_PORTS;
const uint16_t XSHUT_PIN[] = TOF_XSHUT_PINS;

void waitKey() {
    while (!buttonPressed()) HAL_Delay(20);
    HAL_Delay(50);
    while (buttonPressed()) HAL_Delay(20);
}

bool bringUp(int index) {
    for (size_t i = 0; i < TOF_COUNT; ++i) HAL_GPIO_WritePin(XSHUT_PORT[i], XSHUT_PIN[i], GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(XSHUT_PORT[index], XSHUT_PIN[index], GPIO_PIN_SET);
    HAL_Delay(10);
    gDev.I2cDevAddr = 0x52;
    gDev.comms_type = 1;
    gDev.comms_speed_khz = 400;
    uint8_t model = 0, vhv, phase, aperture;
    uint32_t refSpads;
    if (VL53L0X_RdByte(&gDev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &model) != VL53L0X_ERROR_NONE || model != 0xEE)
        return false;
    return VL53L0X_DataInit(&gDev) == VL53L0X_ERROR_NONE && VL53L0X_StaticInit(&gDev) == VL53L0X_ERROR_NONE &&
           VL53L0X_PerformRefCalibration(&gDev, &vhv, &phase) == VL53L0X_ERROR_NONE &&
           VL53L0X_PerformRefSpadManagement(&gDev, &refSpads, &aperture) == VL53L0X_ERROR_NONE;
}

void check(int index) {
    int n = 0, lo = 9999, hi = 0;
    long sum = 0;
    for (int k = 0; k < 50; ++k) {
        VL53L0X_RangingMeasurementData_t m;
        if (VL53L0X_PerformSingleRangingMeasurement(&gDev, &m) != VL53L0X_ERROR_NONE || m.RangeStatus != 0) continue;
        ++n;
        sum += m.RangeMilliMeter;
        if (m.RangeMilliMeter < lo) lo = m.RangeMilliMeter;
        if (m.RangeMilliMeter > hi) hi = m.RangeMilliMeter;
    }
    gTofCal.valid[index] = n;
    gTofCal.lo[index] = lo;
    gTofCal.hi[index] = hi;
    gTofCal.mean[index] = n ? static_cast<int32_t>(sum / n) : 0;
    if (n == 0) printf("  sensor %d: no valid readings\n", index);
    else printf("  sensor %d after calibration: mean %ld mm, min %d, max %d, %d/50 valid (target %d)\n", index,
                sum / n, lo, hi, n, CAL_MM);
}

void fastBlink() {
    for (int k = 0; k < 20; ++k) ledStatusSet(k % 2 == 0), HAL_Delay(100);
    ledStatusSet(false);
}

}  // namespace

int main() {
    boardInit();
    HAL_Delay(200);
    VL53L0X_SetBus(&gI2c);
    printf("\nToF offset calibration, target at %d mm. KEY = go.\n", CAL_MM);
    for (int i = 0; i < TOF_COUNT; ++i) {
        if (CAL_ONLY >= 0 && i != CAL_ONLY) continue;
        printf("Sensor %d: hold the flat target %d mm square in front of it, press KEY\n", i, CAL_MM);
        for (int k = 0; k < 3; ++k) ledStatusSet(false), HAL_Delay(300), ledStatusSet(true), HAL_Delay(300);  // ready blinks
        waitKey();
        ledStatusSet(true);  // LED on = keep the target still; it goes off when this sensor is done
        HAL_Delay(1000);     // time to settle the target after pressing KEY
        gTofCal.sensor = i;
        if (!bringUp(i)) { gTofCal.error[i] = 1; printf("  sensor %d: bring-up FAILED\n", i); fastBlink(); continue; }
        int32_t um = 0;
        VL53L0X_Error e = VL53L0X_PerformOffsetCalibration(&gDev, (FixPoint1616_t)CAL_MM << 16, &um);
        if (e != VL53L0X_ERROR_NONE) { gTofCal.error[i] = e; printf("  sensor %d: calibration error %d\n", i, (int)e); fastBlink(); continue; }
        gOffsetUm[i] = um;
        gDone[i] = true;
        gTofCal.offsetUm[i] = um;
        printf("  sensor %d offset %ld um\n", i, (long)um);
        check(i);
        gTofCal.doneMask |= 1u << i;
        ledStatusSet(false);
    }
    printf("\nPaste into include/config.h:\nconstexpr int32_t TOF_OFFSET_UM[TOF_COUNT] = {");
    for (int i = 0; i < TOF_COUNT; ++i) {
        if (gDone[i]) printf("%ld", (long)gOffsetUm[i]);
        else if (CAL_ONLY >= 0) printf("(unchanged)");
        else printf("TOF_OFFSET_UNSET");
        printf(i + 1 < TOF_COUNT ? ", " : "};\n");
    }
    while (true) {
        ledStatusSet((HAL_GetTick() / 500) % 2 == 0);
        HAL_Delay(100);
    }
}
