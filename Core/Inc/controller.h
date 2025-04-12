//
// Created by Tuan Nguyen on 9/4/25.
//

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "stdbool.h"

#define MOTOR_L 0
#define MOTOR_R 1

#define TURN_KP 0.1F
#define TURN_KD 0.01F
#define TURN_KI 0.5F
#define CENTERING_KP 0.1F
#define CENTERING_KD 0.01F
#define CENTERING_KI 0.5F

extern struct {
    double Kp, Ki, Kd;

    double SetPoint;
    double Output;
    double Error, AccumError, PrevError;
} PID;

extern enum {
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

void Controller_SetSpeed(bool motor, int16_t speed);

void Controller_Test(void);

#endif //CONTROLLER_H
