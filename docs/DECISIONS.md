# Why the numbers are what they are

The code says *what* each setting does. This file records *why* it has its value: the measurement or test behind it,
and what was tried and rejected. Older entries in a section explain how the current values were reached.

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

`V_FULL_MM_S` is the Navigator's speed *scale* only; the controller uses the motor model.

## Speed of the run (`include/course_params.h`, `lib/course_nav/src/planner.cpp`)

**Current:** `V_FULL_MM_S` 680, `speed` 1.0, lookahead 85 mm + 0.18 x speed. Three floor runs without the block took
5.86-5.90 s (from 8.6 s), no hits. Chosen in a host simulation with the measured motor model. About 0.9 s of the gain
was a bug: the start plan made the first sensor snapshot look stale, so every run opened with fault recovery.

**Left side floor stays 0.5.** 0.7 saved ~0.35 s (simulation: no touch; floor run 1: 5.83 to 5.50 s) but the car
touched blocks much more often on the floor, so it was put back.

Earlier history, at `V_FULL_MM_S` 622:

- **Cruise speed 0.88** (~550 mm/s). The host closed-loop tests overlap a block at 0.90 and above, so going faster
  needs better tracking (pose correction), not a bigger number.
- **Speed profile.** The route's speed factor used to be clearance × edge × bend. Where all three were mildly low the
  car crawled at ~170 mm/s. Now the worse of clearance and edge counts, times the bend factor (planned time 12.2 →
  11.1 s). Faster combinations (`sqrt`, power 0.75) made a host closed-loop test overlap a block.
- **Bend factor 1.75** (was 2.0, which slowed a gentle bend to 45 %). 1.5 made the low-memory closed-loop test
  overlap a block.
- **Tight gaps.** Full speed from 130 mm clearance down to 0.4 at 95 mm. Before, the car entered the 200 mm gap
  between T16 and T17 (the detour route) at ~480 mm/s and touched T17; now ~250 mm/s. Costs ~0.6 s on the normal route.
- **Lookahead gain** was 0.22: 0.15 weaved on the long straight; 0.30 cut the corner at T9.
- Run times on the car: 11.7 s (start) → 10.7 → 8.9 → 8.5 s.

## Route (`lib/course_nav/src/planner.cpp`)

- **Straight finish.** Any crossing of the finish line wins. The A* goal used to be x 1350–1650, which made the route
  swing east past T16's corner at the end, and a clean run scraped T16. The goal is now x 1260–1750: the route goes
  straight down whichever corridor it takes.
- **Shortcut bug.** In the low-memory build the shortcut ("string pulling") step stopped after 32 corners and jumped
  straight to the end without a clearance check. That line passed T16 at 65 mm centre-to-face (a wheel at 15 mm) and
  the car touched T16. Now the remaining A* points are kept. Regression test: `test_planned_path_keeps_clear_of_blocks`.
- **Straight finish lane** (x = 1600). The grid route swung 1640 → 1600 → 1640 between T15, T17 and T16; the car cut
  that bend, drifted a further ~2 cm, and passed T17 within about 5 mm. In simulation the typical gap to T17 went from
  42 to 48 mm and the gap to T15 from 66 to 52 mm.
- **Lane at x = 1585 and earlier side correction** (from y 600, under 90 mm, 4 mm per step, 60 mm at most). On the
  floor the car kept ending up 2-4 cm toward T17 there (map checked with a ruler: right within 1 cm). In a simulation
  with that drift added (8000 drives), failures fell from 16.9 % to 5.6 % and T17 hits from 1074 to 98; T15 hits rose
  from 107 to 176. Slowing down there instead (`trustedCaution` 0.65) cost 0.3 s and did not help.
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
5. **Reference run.** Run 1 (no block) is logged to flash; a clean finish (no fault, log not cut off) makes it the
   reference until power-off, a reset, or KEY held 2 s during a countdown: each session learns anew (it was kept across
   power cycles at first). Run 2 compares each reading with what run 1 read near the same
   pose, so static things the map lacks cancel out. (It replaced a `baseline.h` generated on the PC.)
6. **Replan stall.** Planning blocks the control loop longer than the 150 ms motor command lease. The motors switched
   off, the supervisor raised CommandExpired and the car stopped for ~1.2 s. The lease is now extended to 600 ms
   just before a replan. A block found and planned around within one step got no extension (the warning is asked for
   before the step): the motors cut out and the car rolled into the block. Detection now plans in the following step.

With a single reference run, the first settings found phantom blocks in 25 of 36 replays of clean logs. What stopped
that, and what floor tests with the block changed:

- Side sensor readings beyond 350 mm do not place a block. A 420 mm one put a floor-test block 18 cm from where it was
  (the car brushed it). On the logs only the two runs with a far side detection changed, both to where the later readings
  agree; in simulation estimates over 10 cm off fell from 2.0 to 0.3 %, at +0.2 s average. Correcting the position from
  later, closer readings was tried and made it worse.
- The block must stand on the planned route (`centreReach` 60 mm): the teacher tends to block run 1's path.
- A sensor must stay short for 5 steps (`confirmSteps`).
- A reading places the block anywhere in the sensor's cone, so the direction closest to the route is used. Projecting
  along the sensor's axis put a block seen by a 45 deg sensor 10-20 cm off, and it was rejected.
- Estimates from x 1160 are accepted and moved to x 1250 (`clampCentreX`): a real block's estimate reached 1190.
- The front sensor throws random 30-80 mm readings on open floor. One in the reference hid a real block, so there the
  reference's median is used (`Navigator::baselineAt`). Its mounting is worth checking. A single 140 mm stray in an
  otherwise clear lane then hid a block for all but 4 of the 5 steps needed, so the second shortest front reading is
  used: on the logs that block is found 25 cm out, every earlier detection stays, and false blocks are unchanged.

Replays of logged runs: no false block among the clean runs at the current speed (0 of 12 pairs), 8 of 30 among older,
slower ones. In simulation a block 8-12 cm *beside* the path is still touched in 10-25 % of runs.

## Dodging the block (`planner.h`, `include/course_params.h`)

- **`DISC_R` 50** (the block's half-width). At 110 the gap below T14 looked closed and the car looped over the top.
- **Face the route after backing up.** Driving off while still turning, the car swung 3-6 cm inside the planned route
  and touched T14. In a replay of that moment (motor model) the gap went from 12-30 mm to 38-45 mm.
- **Blocks on the finish line count** (`minCentreY` 0; it was 60, against the finish tape). A block by the line was seen
  from 500 mm away and rejected, its estimated centre being y 12. On the logs false blocks did not change (7 of 90 clean
  pairs); at -20 they rose to 13.
- **Stop reversing before a mapped block.** Backing off from a block in front of T15, the car's rear corner hit T14.
  Reversing stops when the point 90 mm behind the centre is within 75 mm of a mapped block: in a replay of that moment
  the gap to T14 went from 0 to 31 mm, five other back-off moments unchanged. Shorter back-off everywhere (5 steps) was
  tried and hit T12 in two of them.
- **The front sensor's spot must stay put** (40 mm). A burst of false readings (125, 95, 105, 115, 140 mm from
  "nothing") became a ghost block; turning back for it, the car drove into the real block beside the path. On the logs the
  ghost is gone, all 20 real detections are unchanged, false blocks 9 of 132 as before.
  Checked only when heading nearly straight down: in turns the hit point of a real block moves, and applying it
  everywhere changed half of the simulated runs with a block 50-70 cm up.
- **A close side reading places the block.** A block just beside run 1's path was seen late; the guess pulled it toward
  the route, 10 cm low, and the car drove into it. With a side sensor under 80 mm the block is placed from that reading:
  on 22 logged runs only that one (corrected) and one other (by 1 cm) move; no change in 4000 simulated drives.
- **Slow approach** (`caution` 0.65 from x 950, until the block is passed): just after the bend by T14 it can only be
  seen close up.
- **Back off** (`backRange` 200 mm): a block found that close is reversed from before turning. 220-260 mm backed into
  T14 in simulation. The floor hit that led to this was not reproduced in simulation.
- **Finish lane** between T16 and T17 (200 mm): side readings under `laneClose` move the pose estimate sideways
  (`Navigator::keepLane`). Shifting the planned lane instead hit T17 in simulation.
- **Trust:** with at least 3 readings agreeing and the car 150 mm past the block, `trustedCaution` replaces `caution`
  and detection stops.
- **Tried and rejected:** going between T15 and T16 (80 mm required clearance instead of 90). On the floor it hit more.
- The planner keeps 90 mm from a block's surface on a 40 mm grid, so a 200 mm gap has no cell wide enough.

## Pose correction: `Navigator::localize` (off)

It corrects the pose against the mapped blocks seen by the ToF. On the car it once pulled the pose by ~5 cm in one step,
because some blocks stood a few cm off the map (T14 by 5 cm). It stays off until the map is checked. It is tested on the
host (`test_localisation_against_drift_and_a_shifted_map`). Turning it on is the most promising way to drive faster.

## Gyro (`include/config.h` section 4, `lib/mpu6050`)

- **Range ±1000 deg/s.** At ±500 the turn on the spot (~640 deg/s) clipped, and 5-10 deg of heading were lost toward the
  side of the turn: the car then hit T16 after right turns and T17 after left ones.
- **Scale** is 1: five turns by hand-off test (`env:test_gyro_scale`) counted 1800.3 for 1800.
- **Offset** is measured again in every start countdown.

At power-on: 2 s settle (hands off), then a 2 s average. A calibration is rejected if the samples spread more than
0.01 rad/s (the car moved) or the offset exceeds 0.1 rad/s, and retried while the car stands still.

## Run log (`src/run_record.cpp`)

Two 6 KB flash slots: run 1 (the reference) and the latest run 2. The 16-byte records (30 Hz, 11.7 s per slot) store
ranges in 5 mm units and speeds in 8 mm/s units (up to 1016 mm/s). The program must stay under 52 KB
(`flash_guard.ld` fails the link otherwise); linking newlib's float `sqrtf` / `fmodf` without their double-precision
error wrappers made room. About 600 bytes are left.

## Removed

- The HC-05 Bluetooth link (`lib/comms`, `remote.cpp`, `tools/console.py`) and the host-driven manual mode.
- A "background mask" of known false-block places, replaced by the reference run.
- The compiled `include/baseline.h` and `tools/make_baseline.py`, replaced by the reference run.
