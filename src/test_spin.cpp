/**
 * @file    test_spin.cpp
 * @brief   Spin-in-place check of the gyro against the encoders (env:test_spin). Keep the robot still for the first 2 s
 *          (gyro calibration). Then write gSpin.dutyA / dutyB (per mille; A = left, B = right) and gSpin.runMs through
 *          ST-Link: the wheels run open loop for runMs and stop (or stop once the gyro yaw reaches targetDeg). With speedMmS > 0 the real wheel speed loop runs instead of open-loop duty. A left (counter-clockwise) turn is dutyA < 0, dutyB > 0.
 *          gyroYawDeg integrates the gyro (raw sign, cfg::GYRO_SCALE applied); encYawDeg is the yaw the encoders imply
 *          from cfg::ENC_SIGN_* and the wheel geometry. Positive = counter-clockwise. Write resetYaw = 1 to zero both.
 */
#include "board.h"
#include "config.h"
#include "i2c_bus.h"
#include "motor.h"
#include "mpu6050.h"

struct SpinStatus {
    int32_t dutyA, dutyB;
    uint32_t runMs;      // set by the host; cleared by the firmware when the run ends
    uint32_t resetYaw;   // set by the host
    uint32_t stage;      // 1 IMU found, 2 calibrated, 3 running
    float gyroYawDeg;
    float encYawDeg;
    int32_t countsL, countsR;  // signed with cfg::ENC_SIGN_*, so forward counts up
    float rateDegPerS;
    uint32_t readFailures;
    float targetDeg;     // set by the host: stop early once |gyro yaw| reaches this (0 = time only)
    float speedMmS;      // set by the host: > 0 runs the real wheel speed loop (left -v, right +v) instead of open-loop duty
    uint32_t straight;   // set by the host with speedMmS: 1 = both wheels forward at speedMmS (a straight line)
    float targetMm;      // set by the host: stop once the mean encoder distance reaches this (0 = off)
    uint32_t buttonRuns; // number of button-started runs completed
    uint32_t resetFlags; // RCC_CSR bits 31..24 at boot: 27 = power-on / brown-out, 26 = reset pin, 28 = software, 29 = watchdog
    uint32_t buttonMode; // what the button starts: 0 = turn in place by turnTargetDeg (default), 1 = straight 200 mm
    float turnTargetDeg; // button turn: target gyro yaw, degrees, positive = left (default 90)
    float turnErrDeg;    // button turn: target minus gyro yaw when it stopped
    uint32_t turnTimeMs; // button turn: how long it took
};
volatile SpinStatus gSpin = {};

namespace {

// Button turn: the wheels run +-v with v proportional to the angle left to turn, so the robot slows down as it arrives
// and stops without coasting past the target. It stops once it is within TURN_TOL_DEG and nearly stationary; if it
// overshoots it turns back.
constexpr float TURN_KP = 4.0f;        // mm/s per degree of error
constexpr float TURN_V_MAX = 180.0f;   // mm/s per wheel (~250 deg/s of pivot; the gyro range is 500)
constexpr float TURN_V_MIN = 60.0f;    // below this the wheels stall on the floor
constexpr float TURN_TOL_DEG = 1.5f;
constexpr float TURN_RATE_TOL = 12.0f; // deg/s that counts as stopped
constexpr uint32_t TURN_SETTLE_MS = 150;
constexpr uint32_t TURN_TIMEOUT_MS = 5000;

I2cBus gI2c(&hi2c1);
Mpu6050 gImu(gI2c);

void drive(uint32_t channel, GPIO_TypeDef* p1, uint16_t n1, GPIO_TypeDef* p2, uint16_t n2, int32_t perMille) {
    if (perMille > 1000) perMille = 1000;
    if (perMille < -1000) perMille = -1000;
    uint32_t top = __HAL_TIM_GET_AUTORELOAD(&htim1) + 1;
    uint32_t mag = static_cast<uint32_t>(perMille < 0 ? -perMille : perMille) * top / 1000;
    HAL_GPIO_WritePin(p1, n1, perMille > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(p2, n2, perMille < 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim1, channel, mag);
}

void setMotors(int32_t a, int32_t b) {
    drive(TIM_CHANNEL_2, MOTOR_A_IN1_PORT, MOTOR_A_IN1_PIN, MOTOR_A_IN2_PORT, MOTOR_A_IN2_PIN, a * cfg::MOTOR_SIGN_L);
    drive(TIM_CHANNEL_1, MOTOR_B_IN1_PORT, MOTOR_B_IN1_PIN, MOTOR_B_IN2_PORT, MOTOR_B_IN2_PIN, b * cfg::MOTOR_SIGN_R);
}

[[noreturn]] void fail() {
    while (true) {
        ledStatusSet((HAL_GetTick() / 100) % 2 == 0);
        HAL_Delay(50);
    }
}

}  // namespace

int main() {
    gSpin.resetFlags = RCC->CSR >> 24;  // why the chip last reset
    RCC->CSR |= RCC_CSR_RMVF;
    boardInit();
    motorInit();
    speedLoopStart();
    HAL_Delay(200);
    if (!gImu.init()) fail();
    gSpin.stage = 1;
    if (!gImu.calibrateGyroZ()) fail();
    gSpin.stage = 2;

    constexpr float RAD_TO_DEG = 57.2957795f;
    uint16_t lastL = __HAL_TIM_GET_COUNTER(&htim2), lastR = __HAL_TIM_GET_COUNTER(&htim3);
    uint32_t last = HAL_GetTick(), runUntil = 0;
    bool running = false;
    float gyroYaw = 0;
    int32_t sumL = 0, sumR = 0;
    gSpin.stage = 3;
    gSpin.turnTargetDeg = 90.0f;
    bool turning = false, settling = false;
    uint32_t turnStart = 0, settleAt = 0;
    while (true) {
        uint32_t now = HAL_GetTick();
        if (now - last >= 5) {
            float dt = (now - last) * 1e-3f;
            last = now;
            if (auto g = gImu.readGyroZ()) {
                float rate = *g * RAD_TO_DEG * cfg::GYRO_SCALE;
                gyroYaw += rate * dt;
                gSpin.rateDegPerS = rate;
            } else {
                ++gSpin.readFailures;
            }
            uint16_t l = __HAL_TIM_GET_COUNTER(&htim2), r = __HAL_TIM_GET_COUNTER(&htim3);
            sumL += cfg::ENC_SIGN_L * static_cast<int16_t>(l - lastL);
            sumR += cfg::ENC_SIGN_R * static_cast<int16_t>(r - lastR);
            lastL = l;
            lastR = r;
        }
        if (gSpin.resetYaw) {
            gSpin.resetYaw = 0;
            gyroYaw = 0;
            sumL = sumR = 0;
        }
        gSpin.gyroYawDeg = gyroYaw;
        gSpin.countsL = sumL;
        gSpin.countsR = sumR;
        gSpin.encYawDeg = (sumR - sumL) * cfg::MM_PER_COUNT / cfg::WHEEL_TRACK_MM * RAD_TO_DEG;

        // Untethered run: press the button, 3 s later the robot turns in place by turnTargetDeg (or, with buttonMode 1,
        // drives straight 200 mm at 200 mm/s). The results stay in gSpin for ST-Link to read afterwards. Pull the debug
        // cable during the countdown.
        static bool countdown = false;
        static uint32_t startAt = 0;
        static bool wasPressed = false;
        bool pressed = buttonPressed();
        if (!running && !turning && !countdown && pressed && !wasPressed) {
            countdown = true;
            startAt = now + 3000;
        }
        wasPressed = pressed;
        if (countdown) {
            ledStatusSet((now / 100) % 2 == 0);
            if (static_cast<int32_t>(now - startAt) >= 0) {
                countdown = false;
                gyroYaw = 0;
                sumL = sumR = 0;
                if (gSpin.buttonMode == 0) {
                    turning = true;
                    settling = false;
                    turnStart = now;
                    gSpin.turnErrDeg = 0;
                    gSpin.turnTimeMs = 0;
                    motorEnable(true);
                } else {
                    gSpin.speedMmS = 200.0f;
                    gSpin.straight = 1;
                    gSpin.targetMm = 200.0f;
                    gSpin.targetDeg = 0;
                    gSpin.runMs = 4000;
                    ++gSpin.buttonRuns;
                }
            }
        }
        if (turning) {
            float err = gSpin.turnTargetDeg - gyroYaw;
            float mag = err < 0 ? -err : err;
            float rate = gSpin.rateDegPerS < 0 ? -gSpin.rateDegPerS : gSpin.rateDegPerS;
            bool arrived = mag < TURN_TOL_DEG && rate < TURN_RATE_TOL;
            if (!arrived) settling = false;
            else if (!settling) {
                settling = true;
                settleAt = now;
            }
            if ((settling && now - settleAt >= TURN_SETTLE_MS) || now - turnStart >= TURN_TIMEOUT_MS) {
                turning = false;
                motorEnable(false);
                gSpin.turnErrDeg = err;
                gSpin.turnTimeMs = now - turnStart;
                ++gSpin.buttonRuns;
            } else {
                float v = TURN_KP * mag;
                if (v > TURN_V_MAX) v = TURN_V_MAX;
                if (v < TURN_V_MIN) v = TURN_V_MIN;
                if (mag < TURN_TOL_DEG) v = 0;  // inside the tolerance: let it settle
                float side = err > 0 ? 1.0f : -1.0f;  // positive error = turn left: left wheel back, right wheel forward
                motorSetTargets(-side * v, side * v);
            }
        }
        if (!running && gSpin.runMs > 0) {
            running = true;
            runUntil = now + gSpin.runMs;
            if (gSpin.speedMmS > 0) motorEnable(true);
            else setMotors(gSpin.dutyA, gSpin.dutyB);
        }
        if (running && gSpin.speedMmS > 0) motorSetTargets(gSpin.straight ? gSpin.speedMmS : -gSpin.speedMmS, gSpin.speedMmS);
        float meanMm = 0.5f * (sumL + sumR) * cfg::MM_PER_COUNT;
        bool reached = running && ((gSpin.targetDeg > 0 && (gyroYaw >= gSpin.targetDeg || gyroYaw <= -gSpin.targetDeg)) ||
                                   (gSpin.targetMm > 0 && meanMm >= gSpin.targetMm));
        if (running && (reached || static_cast<int32_t>(now - runUntil) >= 0)) {
            running = false;
            gSpin.runMs = 0;
            if (gSpin.speedMmS > 0) motorEnable(false);
            setMotors(0, 0);
        }
        if (!countdown) ledStatusSet(running || turning || (now / 500) % 2 == 0);
    }
}
