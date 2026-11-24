/**
 * @file    test_step.cpp
 * @brief   Motor step response (env:test_step, wheels OFF THE GROUND; env:test_step_floor, ON THE FLOOR with ~1.5 m of
 *          clear straight in front, only three low duty levels so it travels about 1 m). Open loop, no speed loop. Press KEY: after 2 s both
 *          wheels are stepped through duty 0.2 .. 1.0 (0.7 s on, 0.5 s off each) while the encoder counts are recorded
 *          every 5 ms. Read gStep over ST-Link afterwards (tools/read_step.py). The result gives the full-duty speed, the
 *          dead-band (lowest duty that moves the wheel) and the response time of each motor, for the speed PID.
 *          Positive duty = forward for both wheels (the signs in config.h are applied). Status LED: blinks idle, fast
 *          while waiting the 2 s, solid while running, slow blink when finished.
 */
#include "board.h"
#include "config.h"

constexpr int LEVELS = 7;  // slots in the record; STEP_LEVELS says how many are run
constexpr int ON_MS = 700, OFF_MS = 500, SAMPLE_MS = 5;
constexpr int N = LEVELS * (ON_MS + OFF_MS) / SAMPLE_MS;  // 1680
#ifdef STEP_FLOOR
constexpr int16_t kLevels[LEVELS] = {150, 250, 350, 0, 0, 0, 0};
constexpr int STEP_LEVELS = 3;
#else
constexpr int16_t kLevels[LEVELS] = {200, 300, 400, 500, 600, 800, 1000};
constexpr int STEP_LEVELS = 7;
#endif
constexpr int RUN_SAMPLES = STEP_LEVELS * (ON_MS + OFF_MS) / SAMPLE_MS;

struct StepTest {
    uint32_t state;   // 0 idle, 1 waiting, 2 running, 3 done
    uint32_t n;       // samples recorded
    uint32_t used;    // levels run (the rest of level[] is zero)
    int16_t level[LEVELS];  // duty per mille
    uint16_t ms[N];   // time since the start of the steps
    int8_t dL[N];     // counts since the previous sample, forward positive
    int8_t dR[N];
};
volatile StepTest gStep = {};

namespace {

void drive(TIM_HandleTypeDef* pwm, uint32_t channel, GPIO_TypeDef* p1, uint16_t n1, GPIO_TypeDef* p2, uint16_t n2,
           int32_t perMille) {
    if (perMille > 1000) perMille = 1000;
    if (perMille < -1000) perMille = -1000;
    uint32_t top = __HAL_TIM_GET_AUTORELOAD(pwm) + 1;
    uint32_t mag = static_cast<uint32_t>(perMille < 0 ? -perMille : perMille) * top / 1000;
    HAL_GPIO_WritePin(p1, n1, perMille > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(p2, n2, perMille < 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(pwm, channel, mag);
}

void setForward(int32_t perMille) {  // both wheels, forward positive
    drive(&htim1, TIM_CHANNEL_2, MOTOR_A_IN1_PORT, MOTOR_A_IN1_PIN, MOTOR_A_IN2_PORT, MOTOR_A_IN2_PIN, cfg::MOTOR_SIGN_L * perMille);
    drive(&htim1, TIM_CHANNEL_1, MOTOR_B_IN1_PORT, MOTOR_B_IN1_PIN, MOTOR_B_IN2_PORT, MOTOR_B_IN2_PIN, cfg::MOTOR_SIGN_R * perMille);
}

int8_t clamp8(int32_t v) { return static_cast<int8_t>(v > 127 ? 127 : v < -128 ? -128 : v); }

}  // namespace

int main() {
    boardInit();  // PWM at zero, encoders counting, speed loop not started
    for (int i = 0; i < LEVELS; ++i) gStep.level[i] = kLevels[i];
    gStep.used = STEP_LEVELS;

    while (true) {
        gStep.state = 0;
        while (!buttonPressed()) {
            ledStatusSet((HAL_GetTick() / 500) % 2 == 0);
            HAL_Delay(20);
        }
        gStep.state = 1;
        gStep.n = 0;
        uint32_t t0 = HAL_GetTick();
        while (HAL_GetTick() - t0 < 2000) {
            ledStatusSet((HAL_GetTick() / 100) % 2 == 0);
            HAL_Delay(10);
        }
        ledStatusSet(true);
        gStep.state = 2;
        uint16_t lastL = __HAL_TIM_GET_COUNTER(&htim2), lastR = __HAL_TIM_GET_COUNTER(&htim3);
        t0 = HAL_GetTick();
        uint32_t next = SAMPLE_MS;
        int32_t current = -1;
        while (gStep.n < RUN_SAMPLES) {
            uint32_t el = HAL_GetTick() - t0;
            if (el < next) continue;
            int level = static_cast<int>(el) / (ON_MS + OFF_MS);
            bool on = static_cast<int>(el) % (ON_MS + OFF_MS) < ON_MS;
            int32_t duty = (on && level < STEP_LEVELS) ? kLevels[level] : 0;
            if (duty != current) {
                setForward(duty);
                current = duty;
            }
            uint16_t l = __HAL_TIM_GET_COUNTER(&htim2), r = __HAL_TIM_GET_COUNTER(&htim3);
            uint32_t i = gStep.n;
            gStep.ms[i] = static_cast<uint16_t>(el);
            gStep.dL[i] = clamp8(cfg::ENC_SIGN_L * static_cast<int16_t>(l - lastL));
            gStep.dR[i] = clamp8(cfg::ENC_SIGN_R * static_cast<int16_t>(r - lastR));
            lastL = l;
            lastR = r;
            gStep.n = i + 1;
            next += SAMPLE_MS;
        }
        setForward(0);
        gStep.state = 3;
        while (buttonPressed()) HAL_Delay(20);
        while (!buttonPressed()) {  // done: slow blink; press KEY to run again
            ledStatusSet((HAL_GetTick() / 1000) % 2 == 0);
            HAL_Delay(20);
        }
        while (buttonPressed()) HAL_Delay(20);
    }
}
