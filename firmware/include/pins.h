/**
 * @file    pins.h
 * @brief   Black Pill (STM32F411CE) pin plan, as set in firmware.ioc. Check against your wiring before powering.
 *
 * @par References
 *   - STM32F411xC/xE datasheet, DS10314: pinout and the alternate-function table
 *   (Document numbers from memory: check against your copies.)
 */
#pragma once

#include "stm32f4xx_hal.h"

// Motor 1 = left, motor 2 = right. Each H-bridge has two PWM inputs (A, B): TIM3 CH1..CH4, AF2.
//   MT1A PA6 (CH1), MT1B PA7 (CH2), MT2A PB0 (CH3), MT2B PB1 (CH4)

// Encoders (quadrature): CL1 = left on TIM2 PA0 / PA1 (AF1), CL2 = right on TIM4 PB6 / PB7 (AF2)
// I2C1: PB8 SCL, PB9 SDA (AF4). USART1: PA9 TX, PA10 RX (AF7). SWD: PA13 / PA14.

// VL53L0X shutdown lines. Sensor k must be the one at tof::ANGLE_DEG[k] (lib/nav/include/tof_layout.h): 0..3 = right edge,
// right middle, left middle, left edge. Swap pins here if the wiring differs. One entry per sensor, tof::COUNT in all.
#define TOF_XSHUT_PORTS \
    { GPIOA, GPIOB, GPIOB, GPIOB }
#define TOF_XSHUT_PINS \
    { GPIO_PIN_15, GPIO_PIN_3, GPIO_PIN_4, GPIO_PIN_5 }

// Start button USER_BTN (PA2, active low), status LED L_STATUS (PC13, active low)
#define PIN_BUTTON_PORT GPIOA
#define PIN_BUTTON_PIN GPIO_PIN_2
#define PIN_LED_PORT GPIOC
#define PIN_LED_PIN GPIO_PIN_13

// User LEDs L1 (PA8) and L2 (PB15), active high
#define PIN_L1_PORT GPIOA
#define PIN_L1_PIN GPIO_PIN_8
#define PIN_L2_PORT GPIOB
#define PIN_L2_PIN GPIO_PIN_15

// Switches SW1..SW3 (PB12, PB13, PB14), pulled up, active low
#define SW_PORT GPIOB
#define SW_PINS \
    { GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14 }

// Battery sense BAT: PA4 = ADC1_IN4
#define PIN_BAT_PORT GPIOA
#define PIN_BAT_PIN GPIO_PIN_4
#define PIN_BAT_CHANNEL ADC_CHANNEL_4
