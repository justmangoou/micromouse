/**
 * @file    board.h
 * @brief   Board bring-up (Blue Pill STM32F103): clock, GPIO, UART, I2C, PWM and encoder timers, the speed loop timer,
 *          status LED and buttons. Exposes the HAL handles the other modules use.
 */
#pragma once

#include "stm32f1xx_hal.h"

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;  // PA9/PA10, wired adapter; printf output (syscalls.cpp)
extern TIM_HandleTypeDef htim1;  // motor PWM (CH1N = motor B, CH2N = motor A)
extern TIM_HandleTypeDef htim2;  // left encoder (hardware quadrature)
extern TIM_HandleTypeDef htim3;  // right encoder (hardware quadrature)
extern TIM_HandleTypeDef htim4;  // speed loop interrupt (cfg::SPEED_LOOP_HZ)

// Clock (72 MHz from the 8 MHz crystal), GPIO, UART, I2C, PWM, encoders. The speed loop timer is started
// separately, after motorInit().
enum class BoardFault { None, Clock, Uart, I2c, Pwm, Encoder, ControlTimer };
extern volatile BoardFault gBoardFault;
void boardInit();
void speedLoopStart();

void ledStatusSet(bool on);
bool buttonPressed();   // USER_BTN, PB9; raw active-low input
bool button2Pressed();  // USER_BTN2, PB8; raw active-low input
