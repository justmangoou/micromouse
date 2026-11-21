# Why the numbers are what they are

The code says *what* each setting does. This file records *why* it has its value: the measurement or test behind it,
and what was tried and rejected. Newest topics first within each section.

Tools mentioned: `src/test_*.cpp` (bench firmware, one `env:test_*` each in `platformio.ini`), `tools/read_run.py`
(read a run from the robot), `tools/replay_run.cpp` (replay a logged run on the host), `pio test -e native` (host tests).

---

## Wheel speed control (`include/config.h` section 5)

**Problem.** The car wobbled, overshot to ~170 % of the target speed after starting, and once stood still for 0.8 s
with both wheels commanded forward.

**Measurement** (`src/test_step.cpp`, duty steps 0.15–1.0):

| | full-duty speed | response (63 %) | notes |
|---|---|---|---|
| wheels off the ground | ~2900 mm/s | 40–50 ms | left and right within 1 % |
| on the floor, 0.15–0.35 duty | speed ≈ 2700 × (duty − 0.065) | 190–270 ms | right wheel ~3 % faster |

**Cause.** The feed-forward assumed 622 mm/s at full duty (4.5× too low). The PI correction then spent the whole run
pinned at its ±0.3 limit, which gives the oscillation. Its integral also wound up far past that limit, so after a
target change it held the output negative for about a second: that was the stall.

**Fix.**
- Feed-forward from the measured model, per wheel: `duty = target / gain + offset` (`MOTOR_FF_*`).
- Anti-windup: the integral term may not exceed the output limit (`lib/base/include/pid.h`).
- Gains `Kp 0.0006`, `Ki 0.003`, chosen in a host simulation of the measured plant: speed error 122 → ~45 mm/s RMS on a
  300/150/350 mm/s step sequence. On the car the error went from 145 to ~50–65 mm/s RMS.
- Speed measured over 4 ticks (one count per 5 ms tick is 77 mm/s; over 4 ticks 19 mm/s).

`V_FULL_MM_S` (622) stayed as the Navigator's speed *scale*; only the controller uses the motor model.

## Speed of the run (`include/course_params.h`, `lib/course_nav/src/planner.cpp`)

- **Cruise speed 0.88** (~550 mm/s). The host closed-loop tests overlap a block at 0.90 and above, so going faster
  needs better tracking (pose correction), not a bigger number.
- **Speed profile.** The route's speed factor used to be clearance × edge × bend. Where all three were mildly low the
  car crawled at ~170 mm/s. Now the worse of clearance and edge counts, times the bend factor (planned time 12.2 →
  11.1 s). Faster combinations (`sqrt`, power 0.75) made a host closed-loop test overlap a block.
- **Bend factor 1.75** (was 2.0, which slowed a gentle bend to 45 %). 1.5 made the low-memory closed-loop test
  overlap a block.
- **Tight gaps.** Full speed from 130 mm clearance down to 0.4 at 95 mm. Before, the car entered the 200 mm gap
  between T16 and T17 (the detour route) at ~480 mm/s and touched T17; now ~250 mm/s. Costs ~0.6 s on the normal route.
- **Lookahead gain 0.22.** 0.15 weaved on the long straight; 0.30 cut the corner at T9.
- Run times on the car: 11.7 s (start) → 10.7 → 8.9 → 8.5 s.

## Route (`lib/course_nav/src/planner.cpp`)

- **Straight finish.** Any crossing of the finish line wins. The A* goal used to be x 1350–1650, which made the route
  swing east past T16's corner at the end, and a clean run scraped T16. The goal is now x 1260–1750: the route goes
  straight down whichever corridor it takes.
- **Shortcut bug.** In the low-memory build the shortcut ("string pulling") step stopped after 32 corners and jumped
  straight to the end without a clearance check. That line passed T16 at 65 mm centre-to-face (a wheel at 15 mm) and
  the car touched T16. Now the remaining A* points are kept. Regression test: `test_planned_path_keeps_clear_of_blocks`.
- **The pit wall is not a "keep away" obstacle**, only a hard edge. Counting it like a block would centre the car in
  the corridor beside the pit, but the planner then needs 90 mm on both sides of a 200 mm corridor and takes the long
  way round.

## Map (`include/map_data.h`)

- **T14.** The car kept touching T14 and the ToF saw "unknown blocks" near it. Fitting a 100 mm square to the ToF hits
  of five clean runs put the physical T14 at (1270, 800), 5 cm below the drawing; it was moved back to (1250, 850).
  A 20 mm map shift added earlier as a work-around was removed.
- **T11** is 10 mm lower than drawn so the route keeps off its corner.
- Fitting the other blocks the same way gave scattered 1–6 cm offsets. Those hit points come from the dead-reckoned
  pose, which drifts by a similar amount, so they were not trusted and the map was not changed.

## ToF sensors (`include/config.h` section 2)

- **Offsets** (`env:test_tof_cal`, flat card at 100 mm): right +31 mm, front +38 mm, left +34 mm. Uncalibrated, all
  three read ~3 cm short. Checked at 300 mm: right 299, left 300, front 297–305. The front sensor was replaced once;
  its offset belongs to that unit.
- **Long range.** The front sensor reads ~10–20 % short at 400–500 mm (375–435 where 475 is expected). The
  map-based detection therefore allows 40 mm + 8 % and ignores readings beyond 800 mm.

## Random-block detection (`lib/course_nav/src/navigator.cpp`, `include/course_params.h`)

The course has one block that is not on the map, in the right wing, placed in the car's way.

1. **First version** (any reading much shorter than the map at x > 1000): a run with no block found 8 "blocks" and
   replanned 36 times (21.6 s instead of 9.9 s). They were the arena wall, the pit's walls and the finish tape, none
   of which were in the map.
2. **Walls added** to the prediction (`rayWalls`), checked over ±20° around the sensor axis: a few cm of pose error
   otherwise hides a wall corner from the prediction but not from the sensor.
3. **Ignore hits near walls and mapped blocks** (150 mm). This stopped the remaining false blocks, but made the
   corridor beside the pit and the gap under T14 blind, exactly where a block blocks our route. Most false blocks
   there turned out to be the misplaced T14.
4. **Only blocks on our route** matter: the hit must be within 150 mm of the route in the next 700 mm.
5. **Baseline from clean runs** (`tools/make_baseline.py` → `include/baseline.h`). The car drives the same route every
   run, so for each pose cell the baseline keeps the shortest reading any clean run saw. Static things the map lacks
   cancel out. A reading 100 mm shorter than that, for 3 steps (0.1 s), is a block. Where the baseline covers the pose,
   the wall / map-block margins of step 3 are not needed.
   - Checked by leaving each clean run out of the baseline and replaying it: no false blocks.
   - A block in the corridor mouth was found 0.1–1 s before contact (replayed and on the car).
6. **Replan stall.** Planning blocks the control loop longer than the 150 ms motor command lease. The motors switched
   off, the supervisor raised CommandExpired and the car stopped for ~1.2 s. The lease is now extended to 600 ms
   just before a replan.

**Limits.** A block that stays hidden behind T14 until the last moment is seen late; that is the sensor's view. The
baseline must be rebuilt (a few clean runs) when the course or a sensor changes. The estimated block centre comes from
the near face only and can be 10 cm off; the planner keeps 110 mm from it.

## Pose correction: `Navigator::localize` (off)

It corrects the pose against the mapped blocks seen by the ToF. On the car it once pulled the pose by ~5 cm in one step,
because some blocks stood a few cm off the map (T14 by 5 cm). It stays off until the map is checked. It is tested on the
host (`test_localisation_against_drift_and_a_shifted_map`). Turning it on is the most promising way to drive faster.

## Gyro start-up (`include/config.h` section 4)

2 s settle after power-on (hands off), then a 2 s average. A calibration is rejected if the samples spread more than
0.01 rad/s (the car moved) or the offset exceeds 0.1 rad/s, and retried while the car stands still.

## Run log (`src/run_record.cpp`)

Two 5 KB flash slots, the older one overwritten, so the last two runs survive power-off. The 16-byte records (30 Hz,
9.5 s) store speeds and ranges in 5 mm(/s) units, small enough for two slots below the program.

## Removed

- The HC-05 Bluetooth link (`lib/comms`, `remote.cpp`, `tools/console.py`) and the host-driven manual mode.
- A "background mask" of known false-block places, replaced by the baseline.
