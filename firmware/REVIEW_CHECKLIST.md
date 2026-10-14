# Firmware manual review

Tick each box yourself after checking the code or hardware. All boxes start unchecked; earlier automated test results do not count as your manual review.

Work through libraries in the order below: shared helpers → sensor drivers → speed control → navigation → integration. Record questions beside an item before marking it complete.

Use mm for distance, mm/s for speed, radians for angles, and seconds for control calculations. Check every conversion at a module boundary.

## 1. I2C bus — `lib/i2c_bus`
- [ ] Hardware: confirm device acknowledgements and a failed/disconnected-device response.

## 2. MPU6050 — `lib/mpu6050`

- [ ] Read the README, header, implementation, and relevant datasheet registers.
- [ ] Trace initialization: identity → reset/wake → filter → sample rate → gyro range.
- [ ] Check signed raw gyro decoding and conversion to rad/s.
- [ ] Check stationary calibration, sample spacing, and offset subtraction exactly once.
- [ ] Check failed reads and failed recalibration preserve the intended state.
- [ ] Hardware: stationary readings stay near zero; rotation sign matches robot heading.
- [ ] Review complete; questions resolved or recorded.

## 3. HC-05 — `lib/hc05`

- [ ] Read the README, header, and implementation.
- [ ] Check host UART baud/8N1 agrees with the module configuration and wiring.
- [ ] Check byte/text transmit, receive timeout behavior, and error reporting.
- [ ] Check AT command validation/CRLF handling and module AT-mode requirements.
- [ ] Check shared UART use with telemetry/printf and blocking time in the main loop.
- [ ] Hardware: send and receive known data; verify AT responses if AT mode is used.
- [ ] Review complete; questions resolved or recorded.

## 4. Speed control — `lib/speed_control`

Software review completed: target/count flow, constructor settings and units, signed counter wraparound, filtering, PID history and limits, deadband, stop/reset, and motor integration checked. TIM11 and the estimator both use the configured 200 Hz period (5 ms). Stop/reset operations in `Motor` exclude interrupts; disabled motors keep estimating speed with zero duty. All 13 host tests pass, including speed-control tests.

Caller requirements: finite inputs, positive `vFull` and `dt`, nonnegative deadband, filter weight in [0, 1], and fewer than 32,768 counts of movement between samples. Deadband retains feed-forward. Integral clamping bounds accumulation but does not fully prevent windup when the final motor duty saturates. Actual interrupt timing and control tuning still require hardware verification.

- [ ] Hardware: forward/reverse speed tracking and zero duty while the wheel coasts.

## 5. Navigation — `lib/nav`

- [ ] Read the README and confirm the two navigation modes and shared units.
- [ ] `tof_layout`: compare sensor positions, angles, ordering, and limits with `LAYOUT.JPG`.
- [ ] `odometry`: check count-to-distance conversion, gyro sign, fusion, and reset pose.
- [ ] `geometry`: check arena boundaries, obstacle shapes, and clearance calculations.
- [ ] `floodfill`: check cell bounds, goals, wall updates, and unknown/known route behavior.
- [ ] `micromouse`: trace observing → turning → advancing → settling → done/fault.
- [ ] `micromouse`: check cell-center steering, front-wall detection, stale/invalid ranges, and timeouts.
- [ ] Record that side-wall centering is currently absent; decide separately whether to add it.
- [ ] `planner`: check grid bounds, robot clearance, path generation, and no-path behavior.
- [ ] `navigator`: check obstacle mapping, replanning, path following, and wheel-speed output.
- [ ] Check failures and goal completion result in the intended stop behavior in each mode.
- [ ] Hardware: verify pose and sensor signs before running either navigation mode.
- [ ] Review complete; questions resolved or recorded.

## 6. Firmware integration

- [ ] `board.cpp` / `pins.h` / `firmware.ioc`: compare pin assignments and timer configuration.
- [ ] Check TIM3 hardware PWM channels/frequency and TIM2/TIM4 encoder mode.
- [ ] Check TIM11 control period and interrupt work; sensor I/O stays outside the interrupt.
- [ ] `motor.cpp`: check direction signs, PWM duty limits, counter histories, disable, and initialization.
- [ ] `tof_array.cpp`: check XSHUT sequencing, address assignment, freshness, and range status handling.
- [ ] `main.cpp`: check startup, stationary IMU calibration, and readiness checks.
- [ ] `maze.cpp` / `course.cpp`: check start conditions, targets, faults, completion, and stopping.
- [ ] `config.h`: confirm physical dimensions, encoder counts, full-duty speed, and tuning values.
- [ ] `platformio.ini`: confirm mode selection builds the intended application.

## 7. Final verification

- [ ] Run `pio test -e native`; inspect failures and confirm all tests pass.
- [ ] Run `pio run -e micromouse -e obstacle_avoidance`; inspect errors and warnings.
- [ ] Confirm selected firmware mode before flashing.
- [ ] Test motor direction and encoders with wheels raised.
- [ ] Test stopping, sensor failure handling, and low-speed straight/turn behavior.
- [ ] Test the selected navigation mode on the course/maze.
- [ ] Record final tuning values and unresolved issues below.

## Review notes

| Library/file | Finding or question | Next action | Resolved |
| --- | --- | --- | --- |
| | | | |
