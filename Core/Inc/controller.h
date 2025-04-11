//
// Created by Tuan Nguyen on 9/4/25.
//

#ifndef CONTROLLER_H
#define CONTROLLER_H

#define MOTOR_L 0
#define MOTOR_R 1

#define MOTOR_L_PWM_CHANNEL TIM_CHANNEL_2
#define MOTOR_L_M_PIN_TYPE GPIOB
#define MOTOR_L_M1_PIN_NUM GPIO_PIN_0
#define MOTOR_L_M2_PIN_NUM GPIO_PIN_1
#define MOTOR_R_PWM_CHANNEL TIM_CHANNEL_1
#define MOTOR_R_M_PIN_TYPE GPIOA
#define MOTOR_R_M1_PIN_NUM GPIO_PIN_5
#define MOTOR_R_M2_PIN_NUM GPIO_PIN_4

const float TURN_KP = 0.1F, TURN_KD = 0.01F, TURN_KI = 0.5F;
const float CENTERING_KP = 0.1F, CENTERING_KD = 0.01F, CENTERING_KI = 0.5F, CENTERING_SET_POINT = 0;

struct {
    double Kp, Ki, Kd;

    double SetPoint;
    double Output;
    double Error, AccumError, PrevError;
} PID;

enum {
    STATE_IDLE = 0,
    STATE_MOVING = 1,
    STATE_TURNING = 2,
} State;

typedef enum Direction {
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3,
} Direction;

typedef enum {
    LEFT = 0,
    RIGHT = 1,
    LEFT_BACK = 2,
    RIGHT_BACK = 3,
    FRONT = 4,
} RelativeDirection;

void Controller_Init(void);

void Controller_Update(float dt);

void Controller_Turn(RelativeDirection dir);

bool Controller_IsWall(RelativeDirection dir);

#endif //CONTROLLER_H
