# Real-car setup (Black Pill, STM32F411CE)

How to get from the simulator (`sim/`) to a car that behaves like it. Everything here follows the
constants in `sim/config.py`, the rules in `docs/Luat thi Nhung.pdf`, and the firmware in
`firmware/` (a PlatformIO project on the STM32 HAL; it replaced the old Embassy Rust skeleton).

**Status.** The simulator, the planner and the localisation exist in `sim/`. The firmware in `firmware/` is a new
PlatformIO project for the Black Pill (the old Embassy Rust skeleton was removed; it is still in git history).
It **builds** and its navigation code passes the host tests, but it has **not been run on hardware yet**. Section 3
lists what is implemented and what is not. Items marked **TODO** do not exist yet. Pin assignments below are a
proposal, not something already wired. Items marked **check** are from my knowledge of the board and the tools,
not verified against your hardware.

## 1. Hardware

| Part | Spec the sim assumes | Notes |
|---|---|---|
| MCU | Black Pill, **STM32F411CE**: Cortex-M4F @ 100 MHz, hardware FPU (single precision), 128 KB RAM, 512 KB flash | buy the F411, not the F401 variant (64 KB RAM). WeAct boards usually have a 25 MHz crystal — **check** yours |
| Motors | 2 × N20 gear motor with magnetic encoder, ~400 rpm at the wheel on 7.4 V | |
| Driver | H-bridge with two PWM inputs per motor (A, B), no direction or standby pins | |
| Wheels | Ø 32 mm (radius 1.6 cm), wheel centres 9.0 cm apart, axle at body centre | |
| Distance | 4 × VL53L0X | one XSHUT line each, for address assignment |
| IMU | MPU6050 (gyro-z is what the sim uses) | |
| Battery | 2S LiPo 7.4 V, 3.3 V and 5 V regulators | log the voltage; the sim varies motor speed by 0.85–1.0 |
| Wireless | UART/SPI/I2C module (HC-05, ESP, nRF24…) | the rules require SPI, I2C or UART; USB is only for development |
| Extras | start button, LED, alignment jig for the start line | the board's own KEY button and LED can be the start button and the status LED |

Body: **≤ 15 × 10 cm** (checked with the frame in stage 1). The sim uses 12.5 × 10 cm.

VL53L0X mounting (body frame, x forward, y left, cm), angle from heading, CCW positive:

| # | angle | x | y |
|---|---|---|---|
| 1 | −60° (right edge) | 6.0 | −4.5 |
| 2 | −20° | 6.0 | −2.0 |
| 3 | +20° | 6.0 | +2.0 |
| 4 | +60° (left edge) | 6.0 | +4.5 |

Measure the real positions and angles with a ruler and protractor. The firmware defines the layout once, in
`firmware/include/config.h`; put the numbers there, and in `TOF_MOUNT` / `TOF_ANGLES` in `sim/config.py`, then run
`python -m sim.tools.check_firmware` to confirm the two agree.
A 1° error is enough to shift the map-matching correction.

## 2. Pin plan (proposal — check against your board before powering)

The F4 assigns functions per pin through alternate functions (AF), so each pin below was chosen to have the
function listed. Reserved: PA11/PA12 are USB, PA13/PA14 are SWD, PB11 is not usable on the 48-pin F411
(it is a core-voltage capacitor pin), PB2 is BOOT1. The start button is on PA2 and the status LED on PC13 (**check**).

| Function | Pins |
|---|---|
| Motor PWM (TIM3, 2 channels per H-bridge, no direction pins) | left MT1A/MT1B PA6/PA7 (CH1/CH2), right MT2A/MT2B PB0/PB1 (CH3/CH4) |
| Left encoder CL1 (TIM2, quadrature) | PA0 / PA1 |
| Right encoder CL2 (TIM4, quadrature) | PB6 / PB7 |
| I2C1 @ 400 kHz | SCL PB8, SDA PB9, 4.7 kΩ pull-ups to 3.3 V |
| VL53L0X XSHUT (L, ML, MR, R) | PA15, PB3, PB4, PB5 |
| MPU6050 | I2C address 0x68 (AD0 to GND) |
| Wireless / log | USART1 PA9 (TX) / PA10 (RX) |
| Start button USER_BTN | PA2 (active low) |
| Status LED L_STATUS | PC13 (active low) |
| LEDs L1, L2 | PA8, PB15 |
| Switches SW1..SW3 | PB12, PB13, PB14 (pulled up, active low) |
| Battery sense BAT | PA4 (ADC1_IN4) |

Timers: TIM2 (left) and TIM4 (right) decode the encoders in hardware, TIM3 makes the motor PWM, and TIM11 (a plain 16-bit APB2
timer, the simplest the F411 has since it lacks TIM6/TIM7) fires the 200 Hz wheel speed loop. TIM1, TIM5, TIM9 and TIM10 are free.

## 3. Firmware: PlatformIO project on the STM32 HAL (`firmware/`)

`platform = ststm32`, `framework = stm32cube` (ST's HAL and CMSIS, not the Arduino core), board `blackpill_f411ce`,
ST-Link upload. Hard-float is forced through `fpu_flags.py` because the ststm32 builder does not pass it to the linker.

```
firmware/
  platformio.ini  one environment per navigation mode (micromouse, course_nav) plus native host tests
  include/    config.h (robot constants), pins.h (pin plan), board.h, motor.h, tof_array.h, robot.h,
              map_data.h (generated), stm32f4xx_hal_conf.h
  src/        main.cpp     start-up and the sensor loop, shared by both modes
              course.cpp   block-course mode: start pose, finish line, A* navigator (env:course_nav)
              maze.cpp     maze mode: maze tuning, flood-fill explorer (env:micromouse)
              board.cpp (clock, GPIO, UART, I2C, ADC, PWM, encoder timers, speed loop timer), motor.cpp (motors and the
              wheel speed loop), tof_array.cpp (4x VL53L0X), remote.cpp (comms packets over USART1),
              interrupts.cpp, syscalls.cpp
  lib/        each library has its own include/ and src/
    base/       math, odometry, PID and wheel speed control, reliability, ToF layout: pure logic, host-tested
    comms/      packet protocol for the HC-05 link (telemetry, logs, start/stop): pure logic, host-tested
    course_nav/ arena geometry, A* planner and navigator (block course)
    micromouse/ flood fill and maze explorer
    i2c_bus/    shared register read/write helpers over the HAL I2C
    mpu6050/    gyro-z driver, uses an I2cBus
    vl53l0x_api/ ST VL53L0X API, portable core only (see its README)
    vl53l0x_impl/ the API's I2C functions implemented on I2cBus
  test/       host unit tests
  tools/      console.py: host console for the comms link
```

```
pio run -e micromouse -t upload          # maze mode (the default environment), flash over ST-Link
pio run -e course_nav -t upload  # block-course mode
python tools/console.py PORT --baud 9600    # telemetry, logs, start/stop over USART1 (wired or HC-05)
pio test -e native                       # host tests of lib/, no hardware needed
python -m sim.course.export_map          # regenerate include/map_data.h and the test reference from the sim
```

**What it does.** Both modes start the same way (`main.cpp`): sensors up, then ~2 s of gyro bias calibration with the
robot standing still. Then the mode waits for the KEY button (or `start` from the console) and runs at 30 Hz (4 ToF readings, fused odometry,
navigation) while a 200 Hz timer interrupt (TIM11) closes the wheel speed loop; the encoders are decoded by hardware.
The block-course mode plans on the stored map (`include/map_data.h`) and stops over the finish line. Both modes send
telemetry at 10 Hz: pose, speed, wheel targets, the four ToF distances, state and fault (`lib/comms/README.md`);
`tools/console.py --csv` records it. `stop` from the console ends a run. `pio run -e manual` builds a third mode where
`drive L R` in the console sets the wheel speeds (mm/s) with a 300 ms dead-man timeout. The maze mode explores from the southwest cell, facing north, until it reaches a centre cell.

**Implemented and checked.**
- Builds clean for the F411. Block course: about 47 KB flash (9 %) and 50 KB RAM (38 %, the planner's static arrays
  are ~36 KB). Maze: about 42 KB flash and 7 KB RAM. The ELF is hard-float (VFPv4-D16).
- `lib/course_nav` owns the course A* planner and navigator; `lib/micromouse` owns maze exploration.
  `lib/base` holds the hardware-independent shared code: math types, odometry, the PID and speed estimate,
  reliability supervision, and the generic ToF layout. `pio test -e native` passes 44 tests:
  comms framing and telemetry, flood fill, geometry, odometry, the wheel speed controller and estimator, the planner against the Python planner's
  path length on the same map, a closed-loop course run with two unmapped blocks, and closed-loop maze runs with fault
  checks.

**Not done / not verified.**
- **Nothing has run on hardware.** The drivers (PWM, encoders, I2C, MPU6050, VL53L0X bring-up) are untested.
  Expect to fix direction signs (`ENC_SIGN_*`, `MOTOR_SIGN_*`, `GYRO_SIGN` in `config.h`), the clock settings if your
  crystal is not 25 MHz, and the pins.
- **No map-matching localisation.** The firmware uses encoder + gyro odometry only, and the sim now defaults to the
  same (`localize=False` in `MicromouseEnv`) rather than the particle filter, so its numbers reflect this. Measured
  in the sim: about 1.8–2.7 cm mean pose error dead-reckoning only, against about 1 cm with the particle filter
  (`localize=True`, still available for comparison but not what the car runs). The 1-D line localiser (section 7)
  is the planned replacement.
- The wheel speed PID gains in `config.h` are placeholders; `V_FULL_CM_S` must come from your speed curve (step 6).
- The map in `map_data.h` is the official map (`config.STATIC_BLOCKS`, read off the rules diagram to ~5 cm). Replace it with your run-1 scan if that is more accurate.
- The VL53L0X run-time reads skip ST's API: one burst read of the result registers plus the interrupt clear (3 bus
  transactions per sensor instead of ~8). A reading counts only if the device range status is 11 and the signal rate
  is above ~0.1 Mcps; anything else is "nothing in range". ST's sigma and other limit checks are not applied. Start-up
  still uses the ST API.
- No wireless module code (the rules need SPI, I2C or UART for it; USART1 is the debug console for now).
- The planner takes a measured time at power-on (printed over the UART); I have not measured it on the chip.
- The vendored VL53L0X API is ST's package with its Windows-only files removed; the STM32 side of it is `lib/vl53l0x_impl`.

## 4. Bring-up checklist

Do these in order and write down the numbers. Each one fills in a constant in `sim/config.py` (section 5).

1. **Flash and log.** Build and upload with `pio run -t upload` from `firmware/` over SWD with an ST-Link. If you have
   no probe, the F411 has a built-in USB DFU bootloader (hold BOOT0, reset, then upload with `upload_protocol = dfu`). Confirm the
   serial output with `python tools/console.py PORT` (the `tof_test` build prints plain text for `pio device monitor`).
2. **Motors.** Wheels off the ground. Check direction for + and − duty and the PWM frequency (above audible, ~20 kHz).
   Sweep duty from 0 up and find the duty where each wheel starts to turn (the **deadband**; the sim uses 0.06).
3. **Encoders.** Turn each wheel by hand exactly 10 revolutions and read the counts. Counts / 10 = counts per rev
   (the sim uses 420). Check the sign: forward must count up on both wheels.
4. **IMU.**
   - Set the gyro to ±500 °/s (65.5 LSB per °/s) and the low-pass filter to ~42 Hz.
   - Let the car sit still for 30 s and record the bias and the noise. The sim assumes σ 0.3 °/s per reading and a
     per-episode bias of σ 0.4 °/s.
   - Warm it up for 5–10 minutes first; bias drifts with temperature.
5. **VL53L0X.**
   - Bring the sensors up one at a time through XSHUT and give each a unique I2C address.
   - Run the offset and crosstalk calibration with the cover or window you will actually use.
   - Check readings against a ruler at 5, 10, 20, 50, 100 cm on a foam block.
   - Record the noise (the sim: σ = max 0.5 cm, 3 %), the minimum reliable range (3 cm in the sim), and what happens
     with no target.
   - Timing budget: 33 ms (the sim reads once per 33 ms, with one step of latency).
   - Stagger the measurements so the sensors do not interfere with each other.
6. **Speed curve.** Drive straight on the floor at several duties. Record speed against duty, the time constant
   (the sim uses 60 ms), and the maximum speed (the sim: ~67 cm/s at full duty). Check at both a full and a
   half-empty battery.
7. **Wheel PID.** Close the loop on wheel speed using a port of `pid.rs`, with feed-forward and battery-voltage
   compensation.
8. **Odometry tests**, on the real floor:
   - 2 m straight: measure the end offset.
   - Spin in place 10 turns: gives the gyro scale (`GYRO_SCALE` in `config.h`: 3600 divided by the degrees the
     odometry reported) and the track width. The gyro offset is re-estimated whenever the robot stands still with the
     motors off, so warm-up drift while waiting for the button does not accumulate.
   - A 50 cm square driven 5 times each way (UMBmark): separates a wheel-radius error from a track error.
9. **Closed loop.** Run a recorded line from the start jig and measure the final error against the finish mark.

## 5. Mapping real measurements to the sim

Edit `sim/config.py`, then re-run `python -m sim.course.evaluate --astar` and compare the sim's
pose error with what you measured.

| Sim constant | Measure |
|---|---|
| `WHEEL_R`, `TRACK` | steps 3 and 8 |
| `CPR` | step 3 |
| `OMEGA_MAX`, `MOTOR_TAU`, `MOTOR_DEADBAND` | steps 2 and 6 |
| `GYRO_NOISE`, `GYRO_BIAS_SIGMA`, `GYRO_BIAS_WALK` | step 4 |
| `TOF_MOUNT`, `TOF_ANGLES`, `TOF_MIN/MAX` (also `config.h` in the firmware), noise in `robot.py` | steps 1 and 5 |
| `ACCEL_SLIP_*`, `SLIP_WALK` | compare wheel odometry against a tape-measure ground truth over several runs |
| `START_SIGMA` | repeat start-jig placements and measure the spread |

If the sim's pose error is much better than the real one, the sim is too optimistic. Widen the noise until they
match before trusting any score.

## 6. Arena

| Item | Spec |
|---|---|
| Floor | 180 × 120 cm, 60 × 60 cm tiles |
| Tape | black, ~2 cm wide, along the outer edge and the U (the U centre is the 60 × 60 cm forbidden cell) |
| Start line | green, left column, y = 0 |
| Finish line | checkered, right column, y = 0 |
| Blocks | 10 × 10 × 10 cm foam, fixed static layout from the layout sheet, plus 1 random (cell 4 or 5) in run 2 |

The sim's coordinates: origin at the bottom-left corner of the floor, x right, y up (cm), the car starts
at (30, −8) facing +y. Convert the layout sheet to these coordinates before using it.

## 7. Algorithms on the MCU

The F411 has an FPU and 128 KB of RAM, so most of the sim's algorithms fit. The timings below are my estimates
from operation counts, not measurements on the chip.

| Piece | Where |
|---|---|
| Encoder + gyro fused odometry | MCU, cheap. Port `FusedOdometry` from `sim/robot.py` |
| A* path planning | Either works. **Offline on the laptop** is the safest first version: export a waypoint table (a few hundred `f32` or `i16` points) — **TODO**: there is no export script yet. **On the MCU** also fits now: the sim's 2 cm grid (~6,500 cells, ~19 KB of cost, parent and flag arrays) is about 15 % of RAM, and with the FPU a plan should take a few tens of ms. Slice it across main-loop iterations, or run it in an RTOS task, so the control loop is never blocked. Initial planning can use the 30 s lock before you press start |
| Path tracking | MCU, pure pursuit (a few multiplies per step) |
| Map-matching localisation | the 150-particle filter is possible but heavy: roughly 30k ray-box tests per update, which I estimate at 10–30 ms, so ~10–15 Hz, or fewer particles / one ray per beam for 30 Hz. The 1-D "line" localiser from the teach-and-repeat idea (position along the stored line, matched to stored ToF signatures) is much cheaper. Pick after you have measured the real cost |
| Unmapped (random) blocks | compare each ToF reading with the expected distance along the stored path; a reading much shorter than expected means a new block, so slow down and replan or detour locally |

Budget the loop: 30 Hz for ToF and control, gyro read at 200 Hz, encoders as hardware quadrature timers
read at 100 Hz or faster.

## 8. Event-day procedure (from the rules)

- Stage 1 (run 1): hand-driven scan, up to 3 minutes, no points, collisions free, as many repeats as you like.
  Time you do not use is added to run 2, but the learned driver finishes in ~10–20 s so the time is not
  needed. Use the whole slot for data: block positions, odometry checks, ToF noise.
- Between runs you may change the code and the car until your turn in stage 2.
- Stage 2 (run 2): 30 s to place the car behind the start line and lock it (no code upload or touching after that),
  then press the button on "Chạy!". Up to 4 minutes autonomous. The random block is drawn after the car is locked.
- Anything that touches the car after the button press counts as a reposition (−10).
- Check beforehand with the teacher whether autonomous code may run in run 1.

## 9. Acceptance before trusting it on the floor

- Pose error at the finish of a lap: within about 2 cm over 10 repeats from the jig.
- On a practice layout with the random block: finish rate 10/10, and the clean rate as high as possible.
  The sim's A* driver reaches 9–15 clean runs out of 20 depending on the layout.
- Log every practice run (pose, ToF, wheel speed) and replay it against the sim to see where they differ.
