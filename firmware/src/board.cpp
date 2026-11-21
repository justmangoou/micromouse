/**
 * @file    board.cpp
 * @brief   Board bring-up for the Blue Pill (STM32F103C8). Clock: 8 MHz HSE through the PLL (x9) to 72 MHz, APB1 36 MHz
 *          (its timers run at 72 MHz), APB2 72 MHz. TIM1 PWM at 20 kHz on the complementary outputs CH1N/CH2N, TIM2/TIM3
 *          as x4 quadrature encoders, TIM4 as the wheel speed loop interrupt. The F103 has no FPU: floats are software.
 *
 * @par References
 *   - RM0008, STM32F10xxx reference manual: clock tree, timer encoder interface, I2C, USART, AFIO
 *   - DS5319, STM32F103x8/xB datasheet: pin definitions and default alternate functions
 *   (Document numbers from memory: check against your copies.)
 */
#include "board.h"

#include "config.h"

I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart1;
TIM_HandleTypeDef htim1, htim2, htim3, htim4;

volatile BoardFault gBoardFault = BoardFault::None;

namespace {

[[noreturn]] void halt(BoardFault fault);

void clockConfig() {
    RCC_OscInitTypeDef osc = {};
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_ON;
    osc.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLMUL = RCC_PLL_MUL9;  // 72 MHz SYSCLK
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) halt(BoardFault::Clock);

    RCC_ClkInitTypeDef clk = {};
    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV2;  // 36 MHz, timers on APB1 run at 72 MHz
    clk.APB2CLKDivider = RCC_HCLK_DIV1;  // 72 MHz
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK) halt(BoardFault::Clock);
}

void gpioInit() {
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_AFIO_REMAP_SWJ_NOJTAG();  // SWD stays; PB3, PB4 and PA15 become plain GPIO (ToF shutdown lines)

    GPIO_InitTypeDef g = {};

    // plain outputs, low at start: motor direction pins, and every ToF shutdown line, the unused ones included, so a
    // sensor that is not in use stays in reset instead of answering at the default I2C address
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Pin = MOTOR_A_IN1_PIN | MOTOR_A_IN2_PIN | MOTOR_B_IN1_PIN | MOTOR_B_IN2_PIN | GPIO_PIN_3 | GPIO_PIN_4;
    HAL_GPIO_WritePin(GPIOB, g.Pin, GPIO_PIN_RESET);
    HAL_GPIO_Init(GPIOB, &g);
    g.Pin = GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_15;
    HAL_GPIO_WritePin(GPIOA, g.Pin, GPIO_PIN_RESET);
    HAL_GPIO_Init(GPIOA, &g);

    // status LED (active low): off
    g.Pin = LED_PIN;
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
    HAL_GPIO_Init(LED_PORT, &g);

    // buttons with pull-up
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    g.Pin = BUTTON_PIN | BUTTON2_PIN;
    HAL_GPIO_Init(BUTTON_PORT, &g);

    // PWM pins: TIM1 CH1N on PB13, CH2N on PB14 (default mapping)
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    g.Pin = GPIO_PIN_13 | GPIO_PIN_14;
    HAL_GPIO_Init(GPIOB, &g);

    // Hardware encoder inputs: timer inputs, never GPIO/EXTI edge counting. TIM2 on PA0/PA1, TIM3 on PA6/PA7.
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    g.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_6 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOA, &g);

    // I2C1 on PB6 (SCL) / PB7 (SDA): open drain, external 4.7 k pull-ups
    g.Mode = GPIO_MODE_AF_OD;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    g.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOB, &g);

    // USART1 on PA9 (TX) / PA10 (RX)
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_NOPULL;
    g.Pin = GPIO_PIN_9;
    HAL_GPIO_Init(GPIOA, &g);
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    g.Pin = GPIO_PIN_10;
    HAL_GPIO_Init(GPIOA, &g);
}

void uartInit() {
    __HAL_RCC_USART1_CLK_ENABLE();
    huart1.Instance = USART1;
    huart1.Init.BaudRate = cfg::UART1_BAUD_RATE;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    if (HAL_UART_Init(&huart1) != HAL_OK) halt(BoardFault::Uart);
}

// A reset in the middle of a transfer leaves a slave holding SDA (and sometimes SCL) low, and the bus stays stuck until
// something clocks the transaction out. Do it by hand before the peripheral takes the pins: up to nine SCL pulses until
// SDA is released, then a STOP.
void i2cBusClear() {
    GPIO_InitTypeDef g = {};
    g.Mode = GPIO_MODE_OUTPUT_OD;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    g.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    HAL_GPIO_WritePin(GPIOB, g.Pin, GPIO_PIN_SET);
    HAL_GPIO_Init(GPIOB, &g);
    auto wait = [] { for (volatile int i = 0; i < 400; ++i) {} };  // ~5 us at 72 MHz
    wait();
    for (int i = 0; i < 9 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_RESET; ++i) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); wait();
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); wait();
    }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); wait();  // STOP: SDA rises while SCL is high
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); wait();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET); wait();
    g.Mode = GPIO_MODE_AF_OD;
    HAL_GPIO_Init(GPIOB, &g);
}

void i2cInit() {
    i2cBusClear();
    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_I2C1_FORCE_RESET();  // clears a BUSY flag left by an earlier reset
    __HAL_RCC_I2C1_RELEASE_RESET();
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 400000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) halt(BoardFault::I2c);
}

// 20 kHz PWM on TIM1 (72 MHz / 3600), duty 0 .. 3600. The driver's PWMA / PWMB pins are the complementary outputs
// CH2N / CH1N: with only CHxNE set the pin follows OCxREF, so the polarity is plain active high.
void pwmInit() {
    __HAL_RCC_TIM1_CLK_ENABLE();
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 3599;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) halt(BoardFault::Pwm);

    TIM_OC_InitTypeDef oc = {};
    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = 0;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    oc.OCIdleState = TIM_OCIDLESTATE_RESET;
    oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    for (uint32_t ch : {TIM_CHANNEL_1, TIM_CHANNEL_2}) {
        if (HAL_TIM_PWM_ConfigChannel(&htim1, &oc, ch) != HAL_OK) halt(BoardFault::Pwm);
        if (HAL_TIMEx_PWMN_Start(&htim1, ch) != HAL_OK) halt(BoardFault::Pwm);  // also sets MOE
    }
}

// PWM was initialized with zero duty before this can happen, so the motors stay stopped.
[[noreturn]] void halt(BoardFault fault) {
    gBoardFault = fault;
    __disable_irq();
    if (htim1.Instance) htim1.Instance->CCR1 = htim1.Instance->CCR2 = 0;
    while (true) {}
}

void encoderInit(TIM_HandleTypeDef* h, TIM_TypeDef* inst) {
    h->Instance = inst;
    h->Init.Prescaler = 0;
    h->Init.CounterMode = TIM_COUNTERMODE_UP;
    h->Init.Period = 0xFFFF;
    h->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    h->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    TIM_Encoder_InitTypeDef e = {};
    e.EncoderMode = TIM_ENCODERMODE_TI12;  // x4 counting
    e.IC1Polarity = TIM_ICPOLARITY_RISING;
    e.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    e.IC1Prescaler = TIM_ICPSC_DIV1;
    e.IC1Filter = 6;
    e.IC2Polarity = TIM_ICPOLARITY_RISING;
    e.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    e.IC2Prescaler = TIM_ICPSC_DIV1;
    e.IC2Filter = 6;
    // Start without capture interrupts: the timer decodes both phases and
    // updates CNT autonomously. TIM4 only samples CNT for wheel control.
    if (HAL_TIM_Encoder_Init(h, &e) != HAL_OK) halt(BoardFault::Encoder);
    __HAL_TIM_SET_COUNTER(h, 0);
    if (HAL_TIM_Encoder_Start(h, TIM_CHANNEL_ALL) != HAL_OK) halt(BoardFault::Encoder);
}

}  // namespace

void boardInit() {
    if (HAL_Init() != HAL_OK) halt(BoardFault::Clock);
    clockConfig();
    gpioInit();
    uartInit();
    i2cInit();
    pwmInit();
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    encoderInit(&htim2, TIM2);
    encoderInit(&htim3, TIM3);
}

// Wheel speed loop: TIM4 update interrupt at cfg::SPEED_LOOP_HZ. TIM4 is a plain 16-bit timer here (its pins are not
// used); 72 MHz / 72 = 1 MHz count rate.
void speedLoopStart() {
    constexpr uint32_t kCountHz = 1000000;
    constexpr uint32_t kPeriod = kCountHz / cfg::SPEED_LOOP_HZ;
    static_assert(kPeriod >= 2 && kPeriod <= 65536, "SPEED_LOOP_HZ does not fit TIM4's 16-bit counter");

    __HAL_RCC_TIM4_CLK_ENABLE();
    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 71;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = kPeriod - 1;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim4) != HAL_OK) halt(BoardFault::ControlTimer);
    HAL_NVIC_SetPriority(TIM4_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
    if (HAL_TIM_Base_Start_IT(&htim4) != HAL_OK) halt(BoardFault::ControlTimer);
}

void ledStatusSet(bool on) { HAL_GPIO_WritePin(LED_PORT, LED_PIN, on ? GPIO_PIN_RESET : GPIO_PIN_SET); }

bool buttonPressed() { return HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN) == GPIO_PIN_RESET; }
bool button2Pressed() { return HAL_GPIO_ReadPin(BUTTON2_PORT, BUTTON2_PIN) == GPIO_PIN_RESET; }
