//
// Created by Tuan Nguyen on 9/4/25.
//

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "stdbool.h"
#include "main.h"

#define MOTOR_MAX_SPEED 999

#define TURN_KP 0.1F
#define TURN_KI 0.0F
#define TURN_KD 0.8F
#define CENTERING_KP 2.6F
#define CENTERING_KI 0.05F
#define CENTERING_KD 0.8F
void Controller_Init(void);

void Controller_Reset(void);

void Controller_MoveForward(void);

void Controller_UpdateTurning(float dt);

void Controller_UpdateCentering(float dt);

void Controller_Turn(RelativeDirection dir);

void Controller_SetSpeed(bool motor, float speed_power);

#endif //CONTROLLER_H
