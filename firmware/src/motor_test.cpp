/**
 * @file    motor_test.cpp
 * @brief   Open-loop motor, encoder and button check (env:motor_test). No I2C, no HC-05, no speed loop. Wheels off the
 *          ground. Write gMotorTest.dutyA / dutyB (per mille, -1000..1000; A = left, B = right) and gMotorTest.runMs
 *          through ST-Link; both motors run for runMs and then stop on their own. The encoder counts (signed, since
 *          start) and the button levels are always live. The status LED blinks while idle and stays on while running.
 */
#include "board.h"
#include "config.h"

struct MotorTestStatus {
    int32_t dutyA, dutyB;  // per mille, set by the host
    uint32_t runMs;        // set by the host: run this long, then stop; the firmware clears it
    int32_t countsL, countsR;  // quadrature counts since reset (x4)
    uint32_t button, button2;  // 1 = pressed
    uint32_t tick;
};
volatile MotorTestStatus gMotorTest = {};

namespace {

void drive(TIM_HandleTypeDef* pwm, uint32_t channel, GPIO_TypeDef* p1, uint16_t n1, GPIO_TypeDef* p2, uint16_t n2,
           int32_t perMille) {
    if (perMille > 1000) perMille = 1000;
    if (perMille < -1000) perMille = -1000;
    uint32_t top = __HAL_TIM_GET_AUTORELOAD(pwm) + 1;
    uint32_t mag = static_cast<uint32_t>(perMille < 0 ? -perMille : perMille) * top / 1000;
    HAL_GPIO_WritePin(p1, n1, perMille > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(p2, n2, perMille < 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(pwm, channel, mag);
}

void setMotors(int32_t a, int32_t b) {
    drive(&htim1, TIM_CHANNEL_2, MOTOR_A_IN1_PORT, MOTOR_A_IN1_PIN, MOTOR_A_IN2_PORT, MOTOR_A_IN2_PIN, a);
    drive(&htim1, TIM_CHANNEL_1, MOTOR_B_IN1_PORT, MOTOR_B_IN1_PIN, MOTOR_B_IN2_PORT, MOTOR_B_IN2_PIN, b);
}

}  // namespace

int main() {
    boardInit();  // TIM2 / TIM3 count the encoders, TIM1 PWM at zero duty
    uint16_t lastL = __HAL_TIM_GET_COUNTER(&htim2), lastR = __HAL_TIM_GET_COUNTER(&htim3);
    uint32_t runUntil = 0;
    bool running = false;
    int32_t sumL = 0, sumR = 0;
    while (true) {
        uint32_t now = HAL_GetTick();
        uint16_t l = __HAL_TIM_GET_COUNTER(&htim2), r = __HAL_TIM_GET_COUNTER(&htim3);
        sumL += static_cast<int16_t>(l - lastL);
        sumR += static_cast<int16_t>(r - lastR);
        lastL = l;
        lastR = r;
        gMotorTest.countsL = sumL;
        gMotorTest.countsR = sumR;
        gMotorTest.button = buttonPressed();
        gMotorTest.button2 = button2Pressed();
        gMotorTest.tick = now;

        if (!running && gMotorTest.runMs > 0) {
            running = true;
            runUntil = now + gMotorTest.runMs;
            setMotors(gMotorTest.dutyA, gMotorTest.dutyB);
        }
        if (running && static_cast<int32_t>(now - runUntil) >= 0) {
            running = false;
            gMotorTest.runMs = 0;
            setMotors(0, 0);
        }
        ledStatusSet(running || (now / 500) % 2 == 0);
        HAL_Delay(1);
    }
}
