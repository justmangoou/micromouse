#include <stdio.h>

#include "board.h"
#include "i2c_bus.h"
#include "tof_array.h"

// Volatile so a debugger can inspect live results without a serial adapter.
struct TofTestStatus {
    uint32_t phase;  // 1 = initializing, 2 = reading
    uint32_t samples;
    uint32_t sensorsUp;
    uint8_t addresses[128];  // I2C ACK by 7-bit address, after initialization
    uint8_t initialized[TOF_SENSORS];
    uint8_t reliable[TOF_SENSORS];
    float mm[TOF_SENSORS];
};
volatile TofTestStatus gTofTest = {};

int main() {
    boardInit();  // PWM starts at zero; the motor speed interrupt stays disabled.
    I2cBus bus(&hi2c1);
    gTofTest.phase = 1;
    gTofTest.sensorsUp = tofInit(bus);
    for (int address = 8; address < 120; ++address)
        gTofTest.addresses[address] = bus.probe(address);
    for (int i = 0; i < TOF_SENSORS; ++i) gTofTest.initialized[i] = tofOk(i);
    printf("\r\nToF test: %lu/%d navigation sensors initialized (sensor %d, right 90, is optional: %s)\r\n",
           (unsigned long)gTofTest.sensorsUp, TOF_COUNT, cfg::TOF_RIGHT_90, tofOk(cfg::TOF_RIGHT_90) ? "up" : "missing");
    gTofTest.phase = 2;
    while (true) {
        float mm[TOF_SENSORS];
        tofRead(mm);
        bool anyValid = false;
        for (int i = 0; i < TOF_SENSORS; ++i) {
            gTofTest.mm[i] = mm[i];
            gTofTest.reliable[i] = tofReliable(i, 200);
            anyValid |= gTofTest.reliable[i];
        }
        ++gTofTest.samples;
        ledStatusSet(anyValid);
        if (gTofTest.samples % 10 == 0) {
            for (int i = 0; i < TOF_SENSORS; ++i)
                printf("sensor %d: %d mm (%s)  ", i, (int)mm[i],
                       gTofTest.reliable[i] ? "valid" : "no valid range");
            printf("\r\n");
        }
        HAL_Delay(50);
    }
}
