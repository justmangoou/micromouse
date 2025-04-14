//
// Created by Tuan Nguyen on 9/4/25.
//

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "stdbool.h"
#include "main.h"

#define MOTOR_MAX_SPEED 999

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

void Controller_Init(void);

void Controller_Reset(void);

void Controller_Update(float dt);

void Controller_Turn(RelativeDirection dir);

void Controller_SetSpeed(bool motor, int16_t speed);

void Controller_Test(void);

#endif //CONTROLLER_H
