/**
 * @file    board.cpp
 * @brief   Board bring-up implementation. Clock: 25 MHz HSE through the PLL to 100 MHz, APB1 50 MHz (its timers run
 *          at 100 MHz), APB2 100 MHz. TIM3 PWM at 20 kHz (4 channels), TIM2/TIM4 as x4 quadrature encoders, TIM11 as the wheel
 *          speed loop interrupt. TIM1, TIM5, TIM9 and TIM10 are free.
 *
 * @par References
 *   - RM0383, STM32F411xC/E reference manual: clock tree, timer encoder interface, I2C, USART
 *   - DS10314, STM32F411xC/xE datasheet: alternate-function mapping
 *   (Document numbers from memory: check against your copies.)
 */
#include "board.h"

#include "pins.h"
#include "config.h"
#include "hc05.h"

I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart1;
Hc05 bluetooth(&huart1);
ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim2, htim3, htim4, htim11;

namespace {

void clockConfig() {
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitTypeDef osc = {};
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_ON;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM = HSE_VALUE / 1000000U;  // 1 MHz PLL input
    osc.PLL.PLLN = 200;                   // 200 MHz VCO
    osc.PLL.PLLP = RCC_PLLP_DIV2;         // 100 MHz SYSCLK
    osc.PLL.PLLQ = 4;
    HAL_RCC_OscConfig(&osc);

    RCC_ClkInitTypeDef clk = {};
    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV2;  // 50 MHz, timers on APB1 run at 100 MHz
    clk.APB2CLKDivider = RCC_HCLK_DIV1;  // 100 MHz
    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_3);
}

void gpioInit() {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef g = {};

    // plain outputs, low at start: ToF shutdown (L, ML, MR, R), user LEDs L1, L2
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | PIN_L2_PIN;
    HAL_GPIO_WritePin(GPIOB, g.Pin, GPIO_PIN_RESET);
    HAL_GPIO_Init(GPIOB, &g);
    g.Pin = GPIO_PIN_15 | PIN_L1_PIN;
    HAL_GPIO_WritePin(GPIOA, g.Pin, GPIO_PIN_RESET);
    HAL_GPIO_Init(GPIOA, &g);

    // status LED (active low): off
    g.Pin = PIN_LED_PIN;
    HAL_GPIO_WritePin(PIN_LED_PORT, PIN_LED_PIN, GPIO_PIN_SET);
    HAL_GPIO_Init(PIN_LED_PORT, &g);

    // button and switches with pull-up
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    g.Pin = PIN_BUTTON_PIN;
    HAL_GPIO_Init(PIN_BUTTON_PORT, &g);
    g.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14;
    HAL_GPIO_Init(SW_PORT, &g);

    // battery sense: analog
    g.Mode = GPIO_MODE_ANALOG;
    g.Pull = GPIO_NOPULL;
    g.Pin = PIN_BAT_PIN;
    HAL_GPIO_Init(PIN_BAT_PORT, &g);

    // PWM pins: TIM3 CH1/CH2 on PA6/PA7, CH3/CH4 on PB0/PB1
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = GPIO_AF2_TIM3;
    g.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOA, &g);
    g.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    HAL_GPIO_Init(GPIOB, &g);

    // Hardware encoder inputs: alternate function, never GPIO/EXTI edge counting.
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Alternate = GPIO_AF1_TIM2;
    g.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    HAL_GPIO_Init(GPIOA, &g);
    g.Alternate = GPIO_AF2_TIM4;
    g.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOB, &g);

    // I2C1 on PB8/PB9: open drain, external 4.7 k pull-ups
    g.Mode = GPIO_MODE_AF_OD;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = GPIO_AF4_I2C1;
    g.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    HAL_GPIO_Init(GPIOB, &g);

    // USART1 on PA9/PA10
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_PULLUP;
    g.Alternate = GPIO_AF7_USART1;
    g.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    HAL_GPIO_Init(GPIOA, &g);
}

void uartInit() {
    __HAL_RCC_USART1_CLK_ENABLE();
    huart1.Instance = USART1;
    huart1.Init.BaudRate = cfg::UART1_BAUD_RATE;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);
}

void i2cInit() {
    __HAL_RCC_I2C1_CLK_ENABLE();
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 400000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&hi2c1);
}

constexpr uint32_t kPwmChannels[] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4};

// 20 kHz PWM on TIM3 (100 MHz / 5000), duty 0 .. 5000, four channels: two per motor
void pwmInit() {
    __HAL_RCC_TIM3_CLK_ENABLE();
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 4999;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&htim3);

    TIM_OC_InitTypeDef oc = {};
    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = 0;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    for (uint32_t ch : kPwmChannels) {
        HAL_TIM_PWM_ConfigChannel(&htim3, &oc, ch);
        HAL_TIM_PWM_Start(&htim3, ch);
    }
}

// battery sense on ADC1 IN4 (PA4), single software-triggered conversion
void adcInit() {
    __HAL_RCC_ADC1_CLK_ENABLE();
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;  // 25 MHz
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    HAL_ADC_Init(&hadc1);

    ADC_ChannelConfTypeDef c = {};
    c.Channel = PIN_BAT_CHANNEL;
    c.Rank = 1;
    c.SamplingTime = ADC_SAMPLETIME_144CYCLES;  // a resistor divider is a high-impedance source
    HAL_ADC_ConfigChannel(&hadc1, &c);
}

// PWM was initialized with zero duty before this can happen, so the motors stay stopped.
[[noreturn]] void halt() {
    __disable_irq();
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
    // updates CNT autonomously. TIM5 only samples CNT for wheel control.
    if (HAL_TIM_Encoder_Init(h, &e) != HAL_OK) halt();
    __HAL_TIM_SET_COUNTER(h, 0);
    if (HAL_TIM_Encoder_Start(h, TIM_CHANNEL_ALL) != HAL_OK) halt();
}

}  // namespace

void boardInit() {
    HAL_Init();
    clockConfig();
    gpioInit();
    uartInit();
    i2cInit();
    adcInit();
    pwmInit();
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();
    encoderInit(&htim2, TIM2);
    encoderInit(&htim4, TIM4);
}

// Wheel speed loop: TIM11 update interrupt at cfg::SPEED_LOOP_HZ. TIM11 is a plain 16-bit timer on APB2 (100 MHz), the
// simplest one the F411 has, so the 32-bit and multi-channel timers stay free. 100 MHz / 100 = 1 MHz count rate.
void speedLoopStart() {
    constexpr uint32_t kCountHz = 1000000;
    constexpr uint32_t kPeriod = kCountHz / cfg::SPEED_LOOP_HZ;
    static_assert(kPeriod >= 2 && kPeriod <= 65536, "SPEED_LOOP_HZ does not fit TIM11's 16-bit counter");

    __HAL_RCC_TIM11_CLK_ENABLE();
    htim11.Instance = TIM11;
    htim11.Init.Prescaler = 99;
    htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim11.Init.Period = kPeriod - 1;
    htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim11);
    HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
    HAL_TIM_Base_Start_IT(&htim11);
}

void ledSet(bool on) { HAL_GPIO_WritePin(PIN_LED_PORT, PIN_LED_PIN, on ? GPIO_PIN_RESET : GPIO_PIN_SET); }

void led1Set(bool on) { HAL_GPIO_WritePin(PIN_L1_PORT, PIN_L1_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET); }

void led2Set(bool on) { HAL_GPIO_WritePin(PIN_L2_PORT, PIN_L2_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET); }

bool switchOn(int index) {
    static const uint16_t pins[3] = SW_PINS;
    return HAL_GPIO_ReadPin(SW_PORT, pins[index]) == GPIO_PIN_RESET;
}

uint16_t batteryRaw() {
    HAL_ADC_Start(&hadc1);
    uint16_t v = 0;
    if (HAL_ADC_PollForConversion(&hadc1, 2) == HAL_OK) v = (uint16_t)HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return v;
}

bool buttonPressed() { return HAL_GPIO_ReadPin(PIN_BUTTON_PORT, PIN_BUTTON_PIN) == GPIO_PIN_RESET; }
