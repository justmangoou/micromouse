# Speed control

Hardware-independent wheel speed estimation and PID control in namespace `speed_control`.

1. `update_target(mmPerSec)` stores the desired speed.
2. Each fixed-period tick, `update_counts(counts)` converts encoder travel to mm/s, smooths the speed, and returns duty:
   `duty = target / vFull + PID(target - speed)`.
3. `Motor` clamps duty to [-1, 1] and applies it through TIM3 hardware PWM.

`speed()` returns filtered speed. `stop()` clears the target and PID history while preserving measured speed; `reset()` also clears measured speed. Exclude control interrupts while calling either. Motor disable immediately zeros PWM and keeps encoder updates running.

Distances use mm, speeds mm/s, and the fixed tick period seconds. Tune full-duty speed, PID gains, and target deadband in [config.h](../../include/config.h). Inside the deadband, PID is reset and only feed-forward remains. Filter weight `alpha` defaults to 0.4.

`counterDelta()` handles 16-bit encoder wraparound, provided movement between reads is less than 32,768 counts in either direction.
