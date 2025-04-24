/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdbool.h"
#include "string.h"

#include "maze.h"
#include "sensor.h"
#include "controller.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUTTON_LONG_PRESS_TIME 2000

#define BUTTON_FLAG 0x01
#define VL53L0X_L_FLAG 0x02
#define VL53L0X_R_FLAG 0x04
#define VL53L0X_F_FLAG 0x08
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
SolveMode mode = WALL_FOLLOWING;
State state = IDLE;

volatile uint8_t flags = 0;
VL53L0XData l_dist_data, r_dist_data, f_dist_data;
Direction curr_dir, prev_dir = NORTH;
volatile uint16_t dist[3];

uint32_t g_lastTime = 0;

// FLOODFILL
// Maze maze;
// MazeStack maze_stack;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

static void MX_GPIO_Init(void);

static void MX_TIM2_Init(void);

static void MX_TIM3_Init(void);

static void MX_TIM4_Init(void);

static void MX_I2C2_Init(void);

static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
static void I2C_Scanner(void);

static void VL53L0X_InitAll(void);

static void Controller_Test(void);

static bool IsWall(RelativeDirection dir);

bool is_reversing(Direction new_dir) {
    return new_dir == prev_dir;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_I2C2_Init();
    MX_USART2_UART_Init();
    /* USER CODE BEGIN 2 */
    VL53L0X_InitAll();
    MPU6050_Init();
    I2C_Scanner();
    Controller_Init();

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    // Calculating the delta time between loop
    const uint32_t now = HAL_GetTick();
    const uint32_t dt = now - g_lastTime;
    const float dts = (float) dt / 1000.0f;

    HAL_GPIO_WritePin(MOTOR_STBY_GPIO_Port, MOTOR_STBY_Pin, GPIO_PIN_SET);

    char msg[50];

    while (1) {
        if (flags & BUTTON_FLAG) {
            static volatile bool button_pressed = true;
            // static volatile uint32_t button_pressed_start = 0;
            // static volatile uint32_t button_pressed_duration = 0;

            if (button_pressed) {
                // button_pressed_duration = HAL_GetTick() - button_pressed_start;
                button_pressed = false;

                if (state == IDLE) {
                    state = MOVING;
                    HAL_GPIO_WritePin(MOTOR_STBY_GPIO_Port, MOTOR_STBY_Pin, GPIO_PIN_SET);
                    HAL_TIM_PWM_Start(&htim3, MOTOR_L_PWM_CHANNEL);
                    HAL_TIM_PWM_Start(&htim3, MOTOR_R_PWM_CHANNEL);
                } else {
                    state = IDLE;
                    HAL_GPIO_WritePin(MOTOR_STBY_GPIO_Port, MOTOR_STBY_Pin, GPIO_PIN_RESET);
                    HAL_TIM_PWM_Stop(&htim3, MOTOR_L_PWM_CHANNEL);
                    HAL_TIM_PWM_Stop(&htim3, MOTOR_R_PWM_CHANNEL);
                }
            } else {
                button_pressed = true;
                // button_pressed_start = HAL_GetTick();
            }

            flags &= ~BUTTON_FLAG;
            continue;
        }

        if (state == IDLE) {
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
            HAL_Delay(500);
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
            HAL_Delay(500);
            continue;
        }

        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

        if (state == TURNING) {
            Controller_UpdateTurning(dts);
            continue;
        }

        if (flags & VL53L0X_L_FLAG) {
            VL53L0X_SetAddr(VL53L0X_L_ADDR);
            l_dist = VL53L0X_ReadRangeContinuousMillimeters(&l_dist_data) + VL53L0X_L_OFFSET;
            VL53L0X_ClearInterrupt();
            flags &= ~VL53L0X_L_FLAG;
            // sprintf(msg, "Left Dist: %d\n", l_dist);
            // HAL_UART_Transmit(&huart2, (uint8_t *) msg, strlen(msg), HAL_MAX_DELAY);
        }
        if (flags & VL53L0X_R_FLAG) {
            VL53L0X_SetAddr(VL53L0X_R_ADDR);
            r_dist = VL53L0X_ReadRangeContinuousMillimeters(&r_dist_data) + VL53L0X_R_OFFSET;
            VL53L0X_ClearInterrupt();
            flags &= ~VL53L0X_R_FLAG;
            // sprintf(msg, "Right Distance: %d\n", r_dist);
            // HAL_UART_Transmit(&huart2, (uint8_t *) msg, strlen(msg), HAL_MAX_DELAY);
        }
        if (flags & VL53L0X_F_FLAG) {
            VL53L0X_SetAddr(VL53L0X_F_ADDR);
            f_dist = VL53L0X_ReadRangeContinuousMillimeters(&f_dist_data) + VL53L0X_F_OFFSET;
            VL53L0X_ClearInterrupt();
            flags &= ~VL53L0X_F_FLAG;
            // sprintf(msg, "Front Distance: %d\n", f_dist);
            // HAL_UART_Transmit(&huart2, (uint8_t *) msg, strlen(msg), HAL_MAX_DELAY);
        }

        // if (IsGoal()) {
        //     state = IDLE;
        //     Controller_Reset();
        //     continue;
        // }

        sprintf(msg, "L R F Wall: %d %d %d\n", IsWall(LEFT), IsWall(RIGHT), IsWall(FORWARD));
        HAL_UART_Transmit(&huart2, (uint8_t *) msg, strlen(msg), HAL_MAX_DELAY);

        if (mode == WALL_FOLLOWING) {
            // Priority: left > forward > right > (avoid back)
            if (!IsWall(LEFT)) {
                Direction next_dir = Convert_RelativeDirection(LEFT);
                if (Convert_Direction(next_dir) != BACKWARD) {
                    Controller_Turn(LEFT);
                    prev_dir = curr_dir;
                    curr_dir = next_dir;
                    // sprintf(msg, "Prev Dir: %d, New Dir: %d\n", prev_dir, curr_dir);
                    // HAL_UART_Transmit(&huart2, (uint8_t *) msg, strlen(msg), HAL_MAX_DELAY);
                }
            } else if (!IsWall(FORWARD)) {
                if (IsWall(RIGHT) && IsWall(LEFT)) {
                    Controller_UpdateCentering(dts);
                } else {
                    Controller_MoveForward();
                }
            } else if (!IsWall(RIGHT)) {
                    Direction next_dir = Convert_RelativeDirection(RIGHT);
                    if (Convert_Direction(next_dir) != BACKWARD) {
                        Controller_Turn(RIGHT);
                        prev_dir = curr_dir;
                        curr_dir = next_dir;
                        // sprintf(msg, "Prev Dir: %d, New Dir: %d\n", prev_dir, curr_dir);
                        // HAL_UART_Transmit(&huart2, (uint8_t *) msg, strlen(msg), HAL_MAX_DELAY);
                    }
                // } else {
                //     // Nowhere to go — turn around
                //     Direction next_dir = Convert_RelativeDirection(BACKWARD);
                //     Controller_Turn(BACKWARD);
                //     prev_dir = curr_dir;
                //     curr_dir = next_dir;
                // }
            }
        }
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void) {
    /* USER CODE BEGIN I2C2_Init 0 */

    /* USER CODE END I2C2_Init 0 */

    /* USER CODE BEGIN I2C2_Init 1 */

    /* USER CODE END I2C2_Init 1 */
    hi2c2.Instance = I2C2;
    hi2c2.Init.ClockSpeed = 100000;
    hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c2.Init.OwnAddress1 = 0;
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2 = 0;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C2_Init 2 */

    /* USER CODE END I2C2_Init 2 */
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void) {
    /* USER CODE BEGIN TIM2_Init 0 */

    /* USER CODE END TIM2_Init 0 */

    TIM_Encoder_InitTypeDef sConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM2_Init 1 */

    /* USER CODE END TIM2_Init 1 */
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 65535;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
    sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = 0;
    sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = 0;
    if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM2_Init 2 */

    /* USER CODE END TIM2_Init 2 */
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void) {
    /* USER CODE BEGIN TIM3_Init 0 */

    /* USER CODE END TIM3_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    /* USER CODE BEGIN TIM3_Init 1 */

    /* USER CODE END TIM3_Init 1 */
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 2;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 999;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM3_Init 2 */

    /* USER CODE END TIM3_Init 2 */
    HAL_TIM_MspPostInit(&htim3);
}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void) {
    /* USER CODE BEGIN TIM4_Init 0 */

    /* USER CODE END TIM4_Init 0 */

    TIM_Encoder_InitTypeDef sConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM4_Init 1 */

    /* USER CODE END TIM4_Init 1 */
    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 0;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 65535;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
    sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = 0;
    sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = 0;
    if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM4_Init 2 */

    /* USER CODE END TIM4_Init 2 */
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void) {
    /* USER CODE BEGIN USART2_Init 0 */

    /* USER CODE END USART2_Init 0 */

    /* USER CODE BEGIN USART2_Init 1 */

    /* USER CODE END USART2_Init 1 */
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 9600;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN USART2_Init 2 */

    /* USER CODE END USART2_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* USER CODE BEGIN MX_GPIO_Init_1 */

    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, LED_Pin | VL53L0X_R_XSHUT_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, VL53L0X_F_XSHUT_Pin | MOTOR_STBY_Pin | MOTOR_R_M1_Pin | MOTOR_R_M2_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, VL53L0X_L_XSHUT_Pin | MOTOR_L_M1_Pin | MOTOR_L_M2_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pins : LED_Pin VL53L0X_R_XSHUT_Pin */
    GPIO_InitStruct.Pin = LED_Pin | VL53L0X_R_XSHUT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pin : VL53L0X_R_GPIO1_Pin */
    GPIO_InitStruct.Pin = VL53L0X_R_GPIO1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(VL53L0X_R_GPIO1_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : VL53L0X_F_GPIO1_Pin */
    GPIO_InitStruct.Pin = VL53L0X_F_GPIO1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(VL53L0X_F_GPIO1_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : VL53L0X_F_XSHUT_Pin MOTOR_STBY_Pin MOTOR_R_M1_Pin MOTOR_R_M2_Pin */
    GPIO_InitStruct.Pin = VL53L0X_F_XSHUT_Pin | MOTOR_STBY_Pin | MOTOR_R_M1_Pin | MOTOR_R_M2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : BUTTON_Pin */
    GPIO_InitStruct.Pin = BUTTON_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : VL53L0X_L_XSHUT_Pin MOTOR_L_M1_Pin MOTOR_L_M2_Pin */
    GPIO_InitStruct.Pin = VL53L0X_L_XSHUT_Pin | MOTOR_L_M1_Pin | MOTOR_L_M2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pin : VL53L0X_L_GPIO1_Pin */
    GPIO_InitStruct.Pin = VL53L0X_L_GPIO1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(VL53L0X_L_GPIO1_GPIO_Port, &GPIO_InitStruct);

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

    /* USER CODE BEGIN MX_GPIO_Init_2 */

    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void I2C_Scanner(void) {
    char msg[64];
    for (uint8_t addr = 1; addr < 128; addr++) {
        if (HAL_I2C_IsDeviceReady(&hi2c2, addr << 1, 1, HAL_MAX_DELAY) == HAL_OK) {
            sprintf(msg, "I2C device found at address 0x%02X\n", addr);
            HAL_UART_Transmit(&huart2, (uint8_t *) msg, strlen(msg), HAL_MAX_DELAY);
        }
    }
    sprintf(msg, "I2C scan complete.\n");
    HAL_UART_Transmit(&huart2, (uint8_t *) msg, strlen(msg), HAL_MAX_DELAY);
}

static void VL53L0X_Config(const uint8_t addr, const bool front) {
    VL53L0X_Init(true, addr);
    VL53L0X_SetSignalRateLimit(10);
    VL53L0X_SetVcselPulsePeriod(VcselPeriodPreRange, 10);
    VL53L0X_SetVcselPulsePeriod(VcselPeriodFinalRange, 14);
    VL53L0X_SetMeasurementTimingBudget(300 * 1000UL);
    VL53L0X_StartContinuous(0);
}

static void VL53L0X_InitAll(void) {
    HAL_GPIO_WritePin(VL53L0X_L_XSHUT_GPIO_Port, VL53L0X_L_XSHUT_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(VL53L0X_R_XSHUT_GPIO_Port, VL53L0X_R_XSHUT_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(VL53L0X_F_XSHUT_GPIO_Port, VL53L0X_F_XSHUT_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);

    HAL_GPIO_WritePin(VL53L0X_L_XSHUT_GPIO_Port, VL53L0X_L_XSHUT_Pin, GPIO_PIN_SET);
    HAL_Delay(10);
    VL53L0X_Config(VL53L0X_L_ADDR, false);

    HAL_GPIO_WritePin(VL53L0X_R_XSHUT_GPIO_Port, VL53L0X_R_XSHUT_Pin, GPIO_PIN_SET);
    HAL_Delay(10);
    VL53L0X_Config(VL53L0X_R_ADDR, false);

    HAL_GPIO_WritePin(VL53L0X_F_XSHUT_GPIO_Port, VL53L0X_F_XSHUT_Pin, GPIO_PIN_SET);
    HAL_Delay(10);
    VL53L0X_Config(VL53L0X_F_ADDR, true);
}

static bool IsWall(const RelativeDirection dir) {
    switch (dir) {
        case FORWARD:
            return f_dist < F_WALL_THRESHOLD;
        case LEFT:
            return l_dist < WALL_THRESHOLD;
        case RIGHT:
            return r_dist < WALL_THRESHOLD;
        default:
            return false;
    }
}

static bool IsGoal(void) {
    if (mode == WALL_FOLLOWING) {
        return !IsWall(FORWARD) && !IsWall(LEFT) && !IsWall(RIGHT);
    }
}

void HAL_GPIO_EXTI_Callback(const uint16_t GPIO_Pin) {
    if (GPIO_Pin == BUTTON_Pin) flags |= BUTTON_FLAG;
    if (GPIO_Pin == VL53L0X_L_GPIO1_Pin) flags |= VL53L0X_L_FLAG;
    if (GPIO_Pin == VL53L0X_R_GPIO1_Pin) flags |= VL53L0X_R_FLAG;
    if (GPIO_Pin == VL53L0X_F_GPIO1_Pin) flags |= VL53L0X_F_FLAG;
}

RelativeDirection Convert_Direction(const Direction dir) {
    switch (prev_dir) {
        case NORTH:
            if (dir == NORTH)
                return FORWARD;
            if (dir == EAST)
                return RIGHT;
            if (dir == SOUTH)
                return BACKWARD;
            return LEFT;
        case EAST:
            if (dir == NORTH)
                return LEFT;
            if (dir == EAST)
                return FORWARD;
            if (dir == SOUTH)
                return RIGHT;
            return BACKWARD;
        case SOUTH:
            if (dir == NORTH)
                return BACKWARD;
            if (dir == EAST)
                return LEFT;
            if (dir == SOUTH)
                return FORWARD;
            return RIGHT;
        case WEST:
            if (dir == NORTH)
                return RIGHT;
            if (dir == EAST)
                return BACKWARD;
            if (dir == SOUTH)
                return LEFT;
            return FORWARD;
    }

    return FORWARD;
}

Direction Convert_RelativeDirection(const RelativeDirection dir) {
    switch (curr_dir) {
        case NORTH:
            if (dir == FORWARD) return NORTH;
            if (dir == RIGHT) return EAST;
            if (dir == BACKWARD)return SOUTH;
            if (dir == LEFT) return WEST;
            break;

        case EAST:
            if (dir == FORWARD) return EAST;
            if (dir == RIGHT) return SOUTH;
            if (dir == BACKWARD)return WEST;
            if (dir == LEFT) return NORTH;
            break;
        case SOUTH:
            if (dir == FORWARD) return SOUTH;
            if (dir == RIGHT) return WEST;
            if (dir == BACKWARD)return NORTH;
            if (dir == LEFT) return EAST;
            break;

        case WEST:
            if (dir == FORWARD) return WEST;
            if (dir == RIGHT) return NORTH;
            if (dir == BACKWARD)return EAST;
            if (dir == LEFT) return SOUTH;
            break;
    }

    return NORTH;
}

/* TEST FUNCTION ------*/
void Controller_Test(void) {
    // Test Left
    Controller_SetSpeed(MOTOR_L, 25);
    HAL_Delay(1000);
    Controller_SetSpeed(MOTOR_L, -25);
    HAL_Delay(1000);
    Controller_SetSpeed(MOTOR_L, 0);

    // Test Right
    Controller_SetSpeed(MOTOR_R, 25);
    HAL_Delay(1000);
    Controller_SetSpeed(MOTOR_R, -25);
    HAL_Delay(1000);
    Controller_SetSpeed(MOTOR_R, 0);

    // Test both
    HAL_Delay(1000);
    Controller_SetSpeed(MOTOR_L, 25);
    Controller_SetSpeed(MOTOR_R, 25);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
