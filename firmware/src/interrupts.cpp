/**
 * @file    interrupts.cpp
 * @brief   Interrupt handlers: SysTick (the HAL tick) and the speed loop timer (TIM11).
 */
#include "board.h"
#include "motor.h"

extern "C" {

void SysTick_Handler(void) { HAL_IncTick(); }

// TIM11 shares its vector with TIM1's trigger/commutation event, which is not used. Update flag only: no HAL callback
// layer in between, this runs 200 times a second.
void TIM1_TRG_COM_TIM11_IRQHandler(void) {
    if (__HAL_TIM_GET_FLAG(&htim11, TIM_FLAG_UPDATE)) {
        __HAL_TIM_CLEAR_FLAG(&htim11, TIM_FLAG_UPDATE);
        motorTick();
    }
}

}
