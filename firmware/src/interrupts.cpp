/**
 * @file    interrupts.cpp
 * @brief   Interrupt handlers: SysTick (the HAL tick) and the speed loop timer (TIM4).
 */
#include "board.h"
#include "motor.h"

extern "C" {

void SysTick_Handler(void) { HAL_IncTick(); }

// Update flag only: no HAL callback layer in between, this runs 200 times a second.
void TIM4_IRQHandler(void) {
    if (__HAL_TIM_GET_FLAG(&htim4, TIM_FLAG_UPDATE)) {
        __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);
        motorTick();
    }
}

}
