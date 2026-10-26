/**
 * @file    manual.cpp
 * @brief   Manual mode (env:manual): the host sets both wheel speeds with Drive packets (tools/console.py). No
 *          navigation. The same health rules apply as in the other modes, so a sensor, timing or pose fault stops the
 *          wheels; only the ToF sensors are optional here.
 *
 * @par Flow
 *   Ready -> start (button or Start packet) -> Running: the wheels follow the latest Drive packet, clamped to
 *   cfg::MANUAL_MAX_MM_S, and stop when none arrives for cfg::MANUAL_TIMEOUT_MS. Stop disarms; the robot requalifies and
 *   is Ready again after 0.5 s standing still. A fault stays latched until reset.
 */
#include <cmath>

#include "board.h"
#include "config.h"
#include "motor.h"
#include "robot.h"

namespace {

float clampSpeed(float v) {
    if (!std::isfinite(v)) return 0;
    return v > cfg::MANUAL_MAX_MM_S ? cfg::MANUAL_MAX_MM_S : v < -cfg::MANUAL_MAX_MM_S ? -cfg::MANUAL_MAX_MM_S : v;
}

}  // namespace

void runMode(const StartupStatus&) {
    using namespace reliability;
    nav::FusedOdometry odo(cfg::ODOM);
    odo.reset({0, 0, 0});
    Supervisor supervisor;
    unsigned pressed = 0;
    float command[2] = {};  // latest Drive request, mm/s
    uint32_t driveAt = 0;
    bool driven = false;
    SensorStep s;

    while (true) {
        waitStep(s);
        float target[2] = {};  // wheel targets this step, mm/s
        RemoteCommands remote = pollRemote();
        if (remote.drive) {
            command[0] = clampSpeed(remote.left);
            command[1] = clampSpeed(remote.right);
            driveAt = s.completedAt;
            driven = true;
        }
        State before = supervisor.state();
        if (before != State::Done && before != State::Fault && !updateOdometry(odo, s)) supervisor.latch(Fault::Pose);
        Fault issue = sensorIssue(s, 0, false);
        if (before == State::Running && !motorsEnabled()) issue = Fault::CommandExpired;
        pressed = buttonPressed() ? pressed + 1 : 0;
        supervisor.update(s.completedAt, issue, s.countsL == 0 && s.countsR == 0, pressed >= 3 || remote.start);
        if (remote.stop || supervisor.state() == State::Done) {
            if (supervisor.state() != State::Fault) supervisor = Supervisor();  // disarm and requalify
            driven = false;
            command[0] = command[1] = 0;
        }

        if (supervisor.state() != State::Running) {
            motorEnable(false);
            driven = false;  // a Drive sent before arming must not move the wheels on arming
        } else {
            bool fresh = driven && s.completedAt - driveAt <= cfg::MANUAL_TIMEOUT_MS;
            if (before != State::Running) motorEnable(true);
            if (fresh) {
                target[0] = command[0];
                target[1] = command[1];
            }
            motorSetTargets(target[0], target[1]);
        }
        if (supervisor.state() == State::Recovering) {
            if (recoverSensors(s)) supervisor.update(HAL_GetTick(), Fault::Sensor, false, false);
        }
        ledStatusSet(supervisor.state() == State::Running ||
                     (supervisor.state() == State::Ready && (s.completedAt / 500) % 2 == 0));
        reportRuntime(supervisor, s, odo, target);
    }
}
