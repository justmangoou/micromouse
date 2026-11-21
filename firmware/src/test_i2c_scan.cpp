/**
 * @file    test_i2c_scan.cpp
 * @brief   Bus scan (env:test_i2c_scan): probes every 7-bit address repeatedly: 20 rounds with every ToF sensor held in reset, then with all of them awake, and
 *          counts the ACKs per address in gScan.hits (read through ST-Link). A healthy MPU6050 shows 0x68 (or 0x69) on
 *          all rounds.
 */
#include "board.h"
#include "i2c_bus.h"

struct ScanStatus {
    uint32_t rounds;
    uint32_t hits[128];
};
volatile ScanStatus gScan = {};

int main() {
    boardInit();  // every XSHUT line is driven low here
    I2cBus bus(&hi2c1, 5);
    HAL_Delay(100);
    while (true) {
        for (uint8_t a = 1; a < 120; ++a)
            if (bus.probe(a)) ++gScan.hits[a];
        ++gScan.rounds;
        if (gScan.rounds == 20) {  // phase 2: wake every ToF sensor (they all answer at 0x29, which is fine for a scan)
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_15, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3 | GPIO_PIN_4, GPIO_PIN_SET);
            HAL_Delay(20);
            for (auto& h : gScan.hits) h = 0;  // hits now count phase 2 only
        }
        ledStatusSet((gScan.rounds / 5) % 2 == 0);
    }
}
