# Base

Pure C++ shared by both navigation modes: math, odometry, PID/speed control,
sensor reliability, and ToF layout. Nothing here touches HAL, I2C, or a specific
navigation mode; everything runs under `pio test -e native`.

Units throughout: mm, mm/s, radians, seconds. Heading is counter-clockwise from +X.

## Math — [math_util.h](include/math_util.h)

Float `PI`, `Vec2`, `Pose`, distance, point interpolation, angle wrapping, and
scalar/array degree-to-radian conversion. `wrap()` returns angles in [-π, π];
non-finite inputs return NaN.

## Odometry — [odometry.h](include/odometry.h)

Differential-drive pose estimation. Construct `nav::FusedOdometry` with an
`OdomParams`: wheel radius, wheel track, counts per revolution, gyro weight and slip
threshold. The library has no defaults; firmware passes `cfg::ODOM` from `config.h`. `update()` fuses signed encoder
deltas with calibrated gyro yaw rate; `updateEncoders()` explicitly uses
encoder-only heading integration during an outage. Both take elapsed seconds.
`reset()` establishes a known pose.

`GyroBiasTracker` (parameters `cfg::GYRO_BIAS`) follows the gyro's zero-rate offset while the robot stands
still with the motors off; its time constant and stillness time are in seconds, independent of the step rate.

Encoder fallback cannot correct wheel slip or relocalize a lifted robot. Firmware
rejects sampling gaps that could alias the 16-bit encoder delta before updating pose.

## Speed control — [speed_controller.h](include/speed_controller.h), [pid.h](include/pid.h)

Wheel speed estimation and PID control in namespace `speed_control`.

1. `update_target(mmPerSec)` stores the desired speed.
2. Each fixed-period tick, `update_counts(counts)` converts encoder travel to mm/s, smooths the speed, and returns duty:
   `duty = target / vFull + PID(target - speed)`.
3. `Motor` clamps duty to [-1, 1] and applies it through TIM3 hardware PWM.

`speed()` returns filtered speed. `stop()` clears the target and PID history while preserving measured speed; `reset()` also clears measured speed. Exclude control interrupts while calling either. Motor disable immediately zeros PWM and keeps encoder updates running.

Tune full-duty speed, PID gains, and target deadband in [config.h](../../include/config.h). Inside the deadband, PID is reset and only feed-forward remains. Filter weight `alpha` defaults to 0.4.

`counterDelta()` handles 16-bit encoder wraparound, provided movement between reads is less than 32,768 counts in either direction.

## Reliability — [reliability.h](include/reliability.h)

Health checks, recovery supervision, command lease, and encoder sampling-gap checks.

`SensorSnapshot<N>` holds acquisition/gyro timestamps and N range statuses and
values. Call `issue()` with required sensor mask, initialization/calibration state,
and distance limits supplied by the caller. Missing, invalid, stale, and bus-error
ranges remain distinct from valid data. Required ranges must be finite and within
[minMm, maxMm); mask bits outside the snapshot are configuration errors.

`Supervisor` owns Ready, Running, Recovering, Done, and Fault transitions. Recovery
requires 500 ms of healthy samples and stationary encoders, then automatically
resumes a started run. Configuration and pose faults latch until reset. Firmware
retries sensors/planning at most once per second with motors disabled and keeps
the original run deadline during recovery. Initial calibration may be retried;
an established gyro bias is preserved across runtime outages.

`CommandLease` expires after 150 ms. The paired motor implementation publishes both
targets and lease under one IRQ lock; TIM11 checks expiry every 5 ms and disables
both motors. Late commands cannot refresh an already expired lease. Zero PWM
coasts; it does not actively brake.

Inspect `gRuntime` through ST-Link for state, fault, recovery count, command expiry
count, and range status/timestamps/failure counts. `gBoardFault` identifies fatal
peripheral initialization failures. Host tests exercise logic only; hardware
stopping, failed sensor isolation, and reconnect timing require bench checks.

## ToF layout — [tof_layout.h](include/tof_layout.h)

`tof::Layout` describes sensor mounts (x forward, y left in mm, CCW angle in
radians), sensor count, angular ray offsets, and usable distance limits. It has no
physical mounting defaults, sensor ordering, board wiring, or HAL dependency.
Storage supports up to 8 sensors and 9 rays per sensor; `count` and `rayCount`
select the active entries. `valid()` checks counts, finite geometry, and limits.

Each navigation module receives a layout explicitly. The actual robot's angles,
mounts, four-sensor ordering, five cone rays, and distance limits live in
`include/config.h` as `cfg::TOF_LAYOUT`, beside its XSHUT wiring.

The simulator retains its own experimental layout. Run
`python -m sim.tools.check_firmware` from the project root to compare defaults.
