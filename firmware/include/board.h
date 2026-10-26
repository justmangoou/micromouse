/**
 * @file    board.h
 * @brief   Board bring-up: clock, GPIO, UART, I2C, ADC, PWM and encoder timers, the speed loop timer, LEDs, button
 *          and switches. Exposes the HAL handles the other modules use.
 */
#pragma once

#include "stm32f4xx_hal.h"

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;  // PA9/PA10, HC-05 or wired; owned by remote.h after boardInit()
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim2;  // left encoder (hardware quadrature, 32-bit timer run as 16-bit)
extern TIM_HandleTypeDef htim3;  // motor PWM
extern TIM_HandleTypeDef htim4;  // right encoder (hardware quadrature)
extern TIM_HandleTypeDef htim11;  // speed loop interrupt (cfg::SPEED_LOOP_HZ)

// Clock (100 MHz from the 25 MHz crystal), GPIO, UART, I2C, ADC, PWM, encoders. The speed loop timer is started
// separately, after motorInit().
enum class BoardFault { None, Clock, Uart, I2c, Adc, Pwm, Encoder, ControlTimer };
extern volatile BoardFault gBoardFault;
void boardInit();
void speedLoopStart();

void ledStatusSet(bool on);
void led1Set(bool on);  // L1
void led2Set(bool on);  // L2
bool buttonPressed();   // USER_BTN, PA2; raw active-low input
bool button2Pressed();  // USER_BTN2, PA3; raw active-low input
bool switchOn(int index);  // SW1..SW3 as 0..2, true = closed (pulled to ground)
uint16_t batteryRaw();     // BAT pin, 12-bit ADC counts (the divider ratio is a property of the board)
