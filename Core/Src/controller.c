#include "stdio.h"
#include "stdlib.h"
#include "stdint.h"
#include "stdbool.h"
#include "string.h"
#include "tgmath.h"

#include "stm32f1xx_hal.h"
#include "main.h"
#include "controller.h"

#include "sensor.h"

float l_spd_pwr = 0, r_spd_pwr = 0;
float yaw_on_turn = 0.0F;

struct {
    double Kp, Ki, Kd;

    double SetPoint;
    double Output;
    double Error, AccumError, PrevError;
} PID;

/* INTERNAL FUNCTION DECLARATIONS ------------------------------------*/
static void process_pid(float input, float dt);

/* PUBLIC FUNCTIONS --------------------------------------------------*/
void Controller_Init(void)
{
    HAL_TIM_PWM_Start(&htim3, MOTOR_L_PWM_CHANNEL);
    HAL_TIM_PWM_Start(&htim3, MOTOR_R_PWM_CHANNEL);
}

void Controller_Reset(void)
{
    PID.SetPoint = PID.Error = PID.AccumError = PID.PrevError = 0.0F;
    l_spd_pwr = r_spd_pwr = 0;

    HAL_GPIO_WritePin(MOTOR_L_M1_GPIO_Port, MOTOR_L_M1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_L_M2_GPIO_Port, MOTOR_L_M2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_R_M1_GPIO_Port, MOTOR_R_M1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_R_M2_GPIO_Port, MOTOR_R_M2_Pin, GPIO_PIN_RESET);
    HAL_Delay(1000);
}

void Controller_MoveForward(void) {
    if (f_dist < 60) {
        Controller_Reset();
        return;
    }

    Controller_SetSpeed(MOTOR_L, BASE_SPD_PWR);
    Controller_SetSpeed(MOTOR_R, BASE_SPD_PWR);
}

void Controller_UpdateTurning(const float dt)
{
    const float angle = MPU6050_GetZAngle();
    yaw_on_turn += angle * dt;
    process_pid(yaw_on_turn, dt);

    const float speed = BASE_SPD_PWR + PID.Output;

    char msg[50];
    sprintf(msg, "PID: %f %f %f\n", PID.Error, PID.Output, speed);
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);

    if (PID.Error < -2.0F)
    {
        Controller_SetSpeed(MOTOR_L, speed);
        Controller_SetSpeed(MOTOR_R, -speed);
    }
    else if (PID.Error > 2.0F)
    {
        Controller_SetSpeed(MOTOR_L, -speed);
        Controller_SetSpeed(MOTOR_R, speed);
    }
    else
    {
        Controller_Reset();
        state = MOVING;
    }
}

void Controller_UpdateCentering(const float dt) {
    if (f_dist < F_WALL_THRESHOLD) {
        Controller_Reset();
        return;
    }

    float error = 0;

    const bool has_left = l_dist < WALL_THRESHOLD;
    const bool has_right = r_dist < WALL_THRESHOLD;

    if (has_left && has_right) {
        // Centering: stay between walls
        error = (float)l_dist - (float)r_dist;
    } else if (has_left) {
        // Left wall-following: target fixed distance
        error = (float)l_dist - WALL_THRESHOLD;
    } else if (has_right) {
        // Right wall-following: target fixed distance
        error = WALL_THRESHOLD - (float)r_dist;
    } else {
        // No wall to follow
        error = 0;
    }

    process_pid(error, dt);

    if (fabs(PID.Error) > 2.0F)
    {
        Controller_SetSpeed(MOTOR_L, BASE_SPD_PWR + PID.Output);
        Controller_SetSpeed(MOTOR_R, BASE_SPD_PWR - PID.Output);
    }
}

void Controller_Turn(RelativeDirection dir)
{
    HAL_Delay(1100);
    Controller_Reset();
    yaw_on_turn = 0;
    state = TURNING;

    PID.Kp = TURN_KP;
    PID.Ki = TURN_KI;
    PID.Kd = TURN_KD;

    switch (dir)
    {
    case LEFT:
        PID.SetPoint = 90;
        break;
    case RIGHT:
        PID.SetPoint = -90;
        break;
    case BACKWARD:
        PID.SetPoint = 180;
        break;
    default:
        break;
    }
}

void Controller_SetSpeed(const bool motor, const float speed_power)
{
    const float speed = fabs(speed_power) * MOTOR_MAX_SPEED / 107;

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

        l_spd_pwr = speed_power;
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

        r_spd_pwr = speed_power;
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
