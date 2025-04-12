#include "stdlib.h"
#include "stdint.h"
#include "stdbool.h"
#include "stm32f1xx_hal.h"

#include "main.h"
#include "controller.h"
#include "sensor.h"

uint_least8_t g_State = 0, g_IsWall = 0;
uint8_t g_FSensor = 0, g_LSensor = 0, g_RSensor = 0;
int16_t g_LMotorSpd = 0, g_RMotorSpd = 0;
float g_Yaw = 0.0F;

VL53L0XData shared;

/* INTERNAL FUNCTION DECLARATIONS ------------------------------------*/
static void turn_off_motors(void);

static void handle_moving(float dt);
static void handle_turn(float dt);

static void process_pid(float input, float dt);

/* PUBLIC FUNCTIONS --------------------------------------------------*/
void Controller_Init(void) {
    MPU6050_Init();
    VL53L0X_Init(true);
}

void Controller_Update(const float dt) {
    VL53L0X_SetAddr(VL53L0X_L_ADDR);
    g_LSensor = VL53L0X_ReadRangeSingleMillimeters(&shared) - VL53L0X_L_OFFSET;
    VL53L0X_SetAddr(VL53L0X_R_ADDR);
    g_RSensor = VL53L0X_ReadRangeSingleMillimeters(&shared) - VL53L0X_R_OFFSET;
    VL53L0X_SetAddr(VL53L0X_F_ADDR);
    g_FSensor = VL53L0X_ReadRangeSingleMillimeters(&shared) - VL53L0X_F_OFFSET;

    switch (g_State) {
        case STATE_MOVING:
            handle_moving(dt);
            break;
        case STATE_TURNING:
            handle_turn(dt);
            break;
        case STATE_IDLE:
        default:
            break;
    }
}

void Controller_Reset() {
    turn_off_motors();
    PID.SetPoint = PID.Error = PID.AccumError = PID.PrevError = 0.0f;

    g_Yaw = 0.0F;
    g_LSensor = g_RSensor = g_FSensor = 0;
    g_LMotorSpd = g_RSensor = 0;
}

void Controller_Turn(RelativeDirection dir) {
    Controller_Reset();
    g_State = STATE_TURNING;

    switch (dir) {
        case LEFT:
            PID.SetPoint = -90;
            break;
        case RIGHT:
            PID.SetPoint = 90;
            break;
        case LEFT_BACK:
            PID.SetPoint = -180;
            break;
        case RIGHT_BACK:
            PID.SetPoint = 180;
            break;
        default:
            break;
    }
}

bool Controller_IsWall(const RelativeDirection dir) {
    switch (dir) {
        case LEFT:
            return g_IsWall & 0x01;
        case RIGHT:
            return g_IsWall >> 1 & 0x01;
        case FRONT:
            return g_IsWall >> 2 & 0x01;
        default:
            return false;
    }
}

void Controller_SetSpeed(const bool motor, const int16_t speed) {
    const uint16_t abs_speed = abs(speed);

    if (motor == MOTOR_L) {
        if (speed > 0) {
            HAL_GPIO_WritePin(MOTOR_L_M1_GPIO_Port, MOTOR_L_M1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(MOTOR_L_M2_GPIO_Port, MOTOR_L_M2_Pin, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(MOTOR_L_M1_GPIO_Port, MOTOR_L_M1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(MOTOR_L_M2_GPIO_Port, MOTOR_L_M2_Pin, GPIO_PIN_RESET);
        }

        g_LMotorSpd = speed;
        __HAL_TIM_SET_COMPARE(&htim3, MOTOR_L_PWM_CHANNEL, abs_speed);
    } else {
        if (speed > 0) {
            HAL_GPIO_WritePin(MOTOR_R_M1_GPIO_Port, MOTOR_R_M1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(MOTOR_R_M2_GPIO_Port, MOTOR_R_M2_Pin, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(MOTOR_R_M1_GPIO_Port, MOTOR_R_M1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(MOTOR_R_M2_GPIO_Port, MOTOR_R_M2_Pin, GPIO_PIN_RESET);
        }

        g_RMotorSpd = speed;
        __HAL_TIM_SET_COMPARE(&htim3, MOTOR_R_PWM_CHANNEL, abs_speed);
    }
}


void Controller_Test(void) {
    Controller_SetSpeed(MOTOR_L, 500);
}

/* INTERNAL FUNCTIONS ------------------------------------------------*/
static void turn_off_motors(void) {
    HAL_GPIO_WritePin(MOTOR_L_M1_GPIO_Port, MOTOR_L_M1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_L_M2_GPIO_Port, MOTOR_L_M2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_R_M1_GPIO_Port, MOTOR_R_M1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_R_M2_GPIO_Port, MOTOR_R_M2_Pin, GPIO_PIN_RESET);
    HAL_Delay(1000);
}

static void handle_moving(const float dt) {
    process_pid((float)g_LSensor - (float)g_RSensor, dt);

    if (PID.Error > 0.5F) {
        Controller_SetSpeed(MOTOR_L, (int16_t) PID.Output);
        Controller_SetSpeed(MOTOR_R, (int16_t) -PID.Output);
    } else if (PID.Error < -0.5F) {
        Controller_SetSpeed(MOTOR_L, (int16_t) -PID.Output);
        Controller_SetSpeed(MOTOR_R, (int16_t) PID.Output);
    } else {
        Controller_Reset();
        g_State = STATE_IDLE;
    }
}

static void handle_turn(const float dt) {
    g_Yaw += MPU6050_GetZAngle() * dt;
    process_pid(g_Yaw, dt);

    if (PID.Error > 0.5F) {
        Controller_SetSpeed(MOTOR_L, (int16_t) PID.Output);
        Controller_SetSpeed(MOTOR_R, (int16_t) PID.Output);
    } else if (PID.Error < -0.5F) {
        Controller_SetSpeed(MOTOR_L, (int16_t) -PID.Output);
        Controller_SetSpeed(MOTOR_R, (int16_t) -PID.Output);
    } else {
        Controller_Reset();
        g_State = STATE_MOVING;
    }
}

static void process_pid(const float input, const float dt) {
    PID.Error = PID.SetPoint - input;
    PID.AccumError += PID.Error * dt;

    PID.Output = PID.Kp * PID.Error + PID.Ki * PID.AccumError + PID.Kd * (PID.Error - PID.PrevError) / dt;
    PID.PrevError = PID.Error;
}