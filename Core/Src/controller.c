#include "stdlib.h"
#include "stdint.h"
#include "stdbool.h"
#include "stm32f1xx_hal.h"

#include "main.h"
#include "controller.h"
#include "sensor.h"

int_least8_t g_LSpdPwr = 0, g_RSpdPwr = 0;

VL53L0XData shared;

/* INTERNAL FUNCTION DECLARATIONS ------------------------------------*/
static void process_pid(float input, float dt);

/* PUBLIC FUNCTIONS --------------------------------------------------*/
void Controller_Init(void)
{
}

void Controller_Reset(void)
{
    PID.SetPoint = PID.Error = PID.AccumError = PID.PrevError = 0.0f;
    g_LSpdPwr = g_RSpdPwr = 0;

    HAL_GPIO_WritePin(MOTOR_L_M1_GPIO_Port, MOTOR_L_M1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_L_M2_GPIO_Port, MOTOR_L_M2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_R_M1_GPIO_Port, MOTOR_R_M1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_R_M2_GPIO_Port, MOTOR_R_M2_Pin, GPIO_PIN_RESET);
    HAL_Delay(1000);
}

void Controller_Update(const float dt)
{
    switch (state)
    {
    case MOVING:
        process_pid((float)lDist - (float)rDist, dt);
        break;
    case TURNING:
        process_pid(yaw, dt);
        break;
    default:
        break;
    }

    if (PID.Error > 0.5F)
    {
        Controller_SetSpeed(MOTOR_L, (int16_t)PID.Output);
        Controller_SetSpeed(MOTOR_R, (int16_t)-PID.Output);
    }
    else if (PID.Error < -0.5F)
    {
        Controller_SetSpeed(MOTOR_L, (int16_t)-PID.Output);
        Controller_SetSpeed(MOTOR_R, (int16_t)PID.Output);
    }
    else
    {
        Controller_Reset();
        state = state == TURNING ? MOVING : IDLE;
    }
}

void Controller_Turn(RelativeDirection dir)
{
    Controller_Reset();
    state = TURNING;

    switch (dir)
    {
    case LEFT:
        PID.SetPoint = -90;
        break;
    case RIGHT:
        PID.SetPoint = 90;
        break;
    case BACKWARD:
        PID.SetPoint = -180;
        break;
    default:
        break;
    }
}

void Controller_SetSpeed(const bool motor, const int_least8_t speed_power)
{
    const uint16_t speed = abs(speed_power) * MOTOR_MAX_SPEED / 100;

    if (motor == MOTOR_L)
    {
        if (speed_power > 0)
        {
            HAL_GPIO_WritePin(MOTOR_L_M1_GPIO_Port, MOTOR_L_M1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(MOTOR_L_M2_GPIO_Port, MOTOR_L_M2_Pin, GPIO_PIN_SET);
        }
        else
        {
            HAL_GPIO_WritePin(MOTOR_L_M1_GPIO_Port, MOTOR_L_M1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(MOTOR_L_M2_GPIO_Port, MOTOR_L_M2_Pin, GPIO_PIN_RESET);
        }

        g_LSpdPwr = speed_power;
        __HAL_TIM_SET_COMPARE(&htim3, MOTOR_L_PWM_CHANNEL, speed);
    }
    else
    {
        if (speed_power > 0)
        {
            HAL_GPIO_WritePin(MOTOR_R_M1_GPIO_Port, MOTOR_R_M1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(MOTOR_R_M2_GPIO_Port, MOTOR_R_M2_Pin, GPIO_PIN_SET);
        }
        else
        {
            HAL_GPIO_WritePin(MOTOR_R_M1_GPIO_Port, MOTOR_R_M1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(MOTOR_R_M2_GPIO_Port, MOTOR_R_M2_Pin, GPIO_PIN_RESET);
        }

        g_RSpdPwr = speed_power;
        __HAL_TIM_SET_COMPARE(&htim3, MOTOR_R_PWM_CHANNEL, speed);
    }
}

/* INTERNAL FUNCTIONS ------------------------------------------------*/
static void process_pid(const float input, const float dt)
{
    PID.Error = PID.SetPoint - input;
    PID.AccumError += PID.Error * dt;

    PID.Output = PID.Kp * PID.Error + PID.Ki * PID.AccumError + PID.Kd * (PID.Error - PID.PrevError) / dt;
    PID.PrevError = PID.Error;
}
