/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum SolveMode
{
    FLOOD_FILL = 0,
    WALL_FOLLOWING = 1
} SolveMode;

typedef enum State
{
    IDLE,
    MOVING,
    DELAY_FOR_TURNING,
    TURNING,
} State;

typedef enum Direction
{
    NORTH = 0,
    EAST  = 1,
    SOUTH = 2,
    WEST  = 3,
} Direction;

typedef enum
{
    LEFT     = 0,
    RIGHT    = 1,
    FORWARD  = 2,
    BACKWARD = 3,
} RelativeDirection;

extern I2C_HandleTypeDef hi2c2;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

extern UART_HandleTypeDef huart2;

extern State state;

extern volatile Direction curr_dir, prev_dir;
extern volatile uint16_t dist[3];

extern volatile uint32_t update_prev_time;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
RelativeDirection Convert_Direction(Direction dir);

Direction Convert_RelativeDirection(RelativeDirection dir);

Direction Direction_Opposite(Direction dir);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define VL53L0X_R_GPIO1_Pin GPIO_PIN_14
#define VL53L0X_R_GPIO1_GPIO_Port GPIOC
#define VL53L0X_R_GPIO1_EXTI_IRQn EXTI15_10_IRQn
#define VL53L0X_R_XSHUT_Pin GPIO_PIN_15
#define VL53L0X_R_XSHUT_GPIO_Port GPIOC
#define VL53L0X_F_GPIO1_Pin GPIO_PIN_5
#define VL53L0X_F_GPIO1_GPIO_Port GPIOA
#define VL53L0X_F_GPIO1_EXTI_IRQn EXTI9_5_IRQn
#define VL53L0X_F_XSHUT_Pin GPIO_PIN_6
#define VL53L0X_F_XSHUT_GPIO_Port GPIOA
#define BUTTON_Pin GPIO_PIN_12
#define BUTTON_GPIO_Port GPIOB
#define BUTTON_EXTI_IRQn EXTI15_10_IRQn
#define VL53L0X_L_XSHUT_Pin GPIO_PIN_14
#define VL53L0X_L_XSHUT_GPIO_Port GPIOB
#define VL53L0X_L_GPIO1_Pin GPIO_PIN_15
#define VL53L0X_L_GPIO1_GPIO_Port GPIOB
#define VL53L0X_L_GPIO1_EXTI_IRQn EXTI15_10_IRQn
#define MOTOR_STBY_Pin GPIO_PIN_8
#define MOTOR_STBY_GPIO_Port GPIOA
#define MOTOR_R_M1_Pin GPIO_PIN_11
#define MOTOR_R_M1_GPIO_Port GPIOA
#define MOTOR_R_M2_Pin GPIO_PIN_12
#define MOTOR_R_M2_GPIO_Port GPIOA
#define MOTOR_L_C1_Pin GPIO_PIN_15
#define MOTOR_L_C1_GPIO_Port GPIOA
#define MOTOR_L_C2_Pin GPIO_PIN_3
#define MOTOR_L_C2_GPIO_Port GPIOB
#define MOTOR_R_PWM_Pin GPIO_PIN_4
#define MOTOR_R_PWM_GPIO_Port GPIOB
#define MOTOR_L_PWM_Pin GPIO_PIN_5
#define MOTOR_L_PWM_GPIO_Port GPIOB
#define MOTOR_R_C1_Pin GPIO_PIN_6
#define MOTOR_R_C1_GPIO_Port GPIOB
#define MOTOR_R_C2_Pin GPIO_PIN_7
#define MOTOR_R_C2_GPIO_Port GPIOB
#define MOTOR_L_M1_Pin GPIO_PIN_8
#define MOTOR_L_M1_GPIO_Port GPIOB
#define MOTOR_L_M2_Pin GPIO_PIN_9
#define MOTOR_L_M2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define VL53L0X_L_ADDR   0x30 << 1
#define VL53L0X_R_ADDR   0x31 << 1
#define VL53L0X_F_ADDR   0x32 << 1

#define VL53L0X_L_OFFSET -7
#define VL53L0X_R_OFFSET 5
#define VL53L0X_F_OFFSET -45

#define WALL_THRESHOLD 110
#define F_WALL_THRESHOLD 40

#define WHEEL_RADIUS 2.25F // Centimeters
#define ROBOT_BASE_RADIUS 3.25F // Centimeters
#define ENCODER_TICKS_PER_REV

#define MOTOR_L true
#define MOTOR_R false
#define BASE_SPD_PWR 40

// TIM3
#define MOTOR_L_PWM_CHANNEL TIM_CHANNEL_2
#define MOTOR_R_PWM_CHANNEL TIM_CHANNEL_1
// TIM2
#define MOTOR_L_C1          TIM_CHANNEL_1
#define MOTOR_L_C2          TIM_CHANNEL_2
// TIM4
#define MOTOR_R_C1          TIM_CHANNEL_1
#define MOTOR_R_C2          TIM_CHANNEL_2

#define l_dist dist[0]
#define r_dist dist[1]
#define f_dist dist[2]

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
