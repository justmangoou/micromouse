# Block-course robot: firmware

A small two-wheeled robot drives from the start line to the finish line of a block course (17 known blocks, see
`include/map_data.h`). One extra block that is not on the map is put in its way; the robot detects it with its distance
sensors and drives around it.

**Hardware:** Blue Pill (STM32F103C8: 72 MHz, 20 KB RAM, 64 KB flash, no FPU), two DC motors with quadrature encoders,
an MPU6050 gyro, three VL53L0X time-of-flight distance sensors (right 45°, front, left 45°), a start button (KEY) and a
status LED.

## A run

1. Power on and keep the robot still for ~4 s: sensors start, the gyro is calibrated. The LED then blinks slowly: ready.
2. Press KEY: 3 s countdown (fast blink), then the robot drives the course and stops past the finish line (LED solid).
3. Carry it back to the start and press KEY again for the next run.

Run 1 is without the random block (ready LED: slow even blink). Once it finishes cleanly, its log is the reference and
the next runs look for the block (ready LED: double blink). The robot forgets it at power-off or reset, so the first
run after switching on always learns again (or hold KEY for 2 s during the countdown).

A fault stops the motors and the LED flashes its number (`src/course.cpp`, `statusLed`). The two runs are kept in
flash and can be read afterwards (`tools/read_run.py`).

## How it works

Two loops run at the same time:

```
every 33 ms (main loop, src/course.cpp)                          every 5 ms (timer interrupt, src/motor.cpp)
  read gyro, encoders, 3 ToF ranges          (main.cpp)            measure each wheel's speed from its encoder
  pose = odometry(encoders, gyro)            (lib/base/odometry)   duty = feed-forward(target) + PI(target - speed)
  Navigator.step(pose, ToF):                 (lib/course_nav)      PWM to the motor
    - a block not on the map?  -> plan a new route (A*)
    - pure pursuit along the route -> wheel speed targets  ---->   (targets)
  safety supervisor: may the car move?       (lib/base/reliability)
  record the step to flash                   (src/run_record.cpp)
```

| Part | What it does | Where |
|---|---|---|
| Odometry | Position from the wheel encoders, heading mainly from the gyro (98 %). The gyro's zero offset is re-learned whenever the car stands still, and measured again in each start countdown. | `lib/base/include/odometry.h` |
| Route planning | A* on a 40 mm grid that prefers cells far from blocks; then shortcuts, corner smoothing and a speed factor per point (slow near blocks, near the edge and in bends). | `lib/course_nav/src/planner.cpp` |
| Route following | Pure pursuit: steer on the circle arc through a point a little ahead on the route; that point is further ahead at speed. | `Navigator::track` in `lib/course_nav/src/navigator.cpp` |
| Random-block detection | Run 2 only. A reading much shorter than expected, for 5 steps, whose block would stand on the route ahead. "Expected" is what run 1 read near the same pose (kept in flash until a restart), or the map where run 1 never was. | `Navigator::detectBlocks`, `src/run_record.cpp` |
| Dodging | Slower where the block can only be seen late; backs off if it appears very close; side sensors correct drift in the narrow finish lane. | `include/course_params.h`, `Navigator` |
| Wheel speed control | Feed-forward from a measured motor model plus a PI correction with anti-windup. | `lib/base/include/speed_controller.h`, `pid.h` |
| Safety | A supervisor allows motion only while sensors are fresh and healthy; a fault stops the motors until the car has stood still and healthy again. The motors also stop on their own if the main loop stops sending commands for 150 ms. | `lib/base/include/reliability.h` |

## Code map

```
include/
  config.h          the robot: wiring, sensors, motor model, gains, timing
  course_params.h   the course: start, finish, how the Navigator drives it
  map_data.h        the 17 known blocks
src/
  main.cpp          start-up, sensor loop, health checks
  course.cpp        the run: start button, countdown, drive, finish, faults, LED
  run_record.cpp    run 1 (the reference) and run 2 in flash
  motor.cpp         motors and the 200 Hz wheel speed loop
  board.cpp         clocks, pins, timers, I2C (STM32 HAL)
  tof_array.cpp     the three VL53L0X sensors
  test_*.cpp        bench firmware, one per platformio environment (motor step test, ToF calibration, ...)
lib/
  base/             maths, odometry, PID, speed controller, safety supervisor (no hardware: tested on the PC)
  course_nav/       geometry, A* planner, Navigator (no hardware: tested on the PC)
  i2c_bus/, mpu6050/, vl53l0x_impl/   drivers
  vl53l0x_api/      ST's VL53L0X API (vendor code, unchanged)
test/               PC tests: pio test -e native
tools/              read and analyse runs on the PC (see tools/README.md)
data/               logged runs (see data/README.md)
docs/DECISIONS.md   why every tuned number has its value
docs/report/        project report (Typst): typst compile docs/report/report.typ docs/report/report.pdf
```

## Build, test, run

```
pio run -e course_nav -t upload     # build and flash over ST-Link
pio test -e native                  # PC tests (also: -e native_lowmem, the planner settings of the robot)
python3 tools/read_run.py           # read both runs from the robot (or tools/analyze.py)
typst compile docs/report/report.typ docs/report/report.pdf   # build the project report
```

Coordinates throughout: mm, x to the right along the start line, y forward; angles in rad, counter-clockwise.
