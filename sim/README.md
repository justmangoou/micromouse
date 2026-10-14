# Micromouse (xe dò mê cung) simulator + drivers

Simulates run 2 (the scored, autonomous run) of *Luật thi xe dò mê cung*:
180×120 cm floor, inverted-U track marked by black tape, the 17 fixed 10 cm blocks of the official map (rules update 12/10/2026) plus one
random block on 1 of 4 spots in the right corner cell / finish cell, 240 s limit.

## Layout

```
sim/
  config.py  robot.py  geometry.py    shared: constants, car + sensor + odometry model, ray casts / collision
  course/                             the 180x120 obstacle course (rules in docs/Luat thi Nhung.pdf)
    arena.py env.py planner.py localization.py policy.py train.py evaluate.py render.py export_map.py
  maze/                               classic 16x16 micromouse maze, flood-fill driver (no training)
    world.py driver.py run.py render.py
  tools/draw_tof_layout.py            top-view picture of the sensor layout
  tools/check_firmware.py             fails if the firmware's copies of the sim defaults (ToF layout, wheel geometry, gains) drift
  checkpoints/                        policy.npz (current), archive/ (older layouts), logs/
```

## Run

```
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt

# obstacle course: A* planner, learned policy, hand-written baseline
.venv/bin/python -m sim.course.evaluate --astar --episodes 20 --gif docs/media/astar.gif
.venv/bin/python -m sim.course.evaluate --episodes 40 --gif docs/media/policy.gif
.venv/bin/python -m sim.course.evaluate --baseline --episodes 20
# train (evolution strategies; add --resume <file.npz> to continue, --out to choose where it saves)
.venv/bin/python -m sim.course.train --generations 300 --layouts 5 --out sim/checkpoints/policy.npz
# regenerate firmware/include/map_data.h and the host-test reference from the sim
.venv/bin/python -m sim.course.export_map

# 16x16 maze (search, then speed run)
.venv/bin/python -m sim.maze --seed 1 --episodes 12
.venv/bin/python -m sim.maze --seed 1 --gif docs/media/maze.gif
```

`--gif` renders the best episode of the batch (`--pick worst|first` for the others).

## Robot model (`config.py`, `robot.py`)

The sim is kept in step with `firmware/`, not just plausible-looking: each piece below is a port of the
actual C++ it mirrors, same gains, same structure, checked by hand against the firmware source.

| Part | Model |
|---|---|
| Wheel speed loop | The policy's action is a **target wheel speed** (fraction of `V_FULL`), not raw PWM. It is closed into PWM by `Robot.drive_to_target`: feed-forward (`target / V_FULL`) + PID on an IIR-filtered (α 0.4), encoder-quantised measured speed — a line-for-line port of `lib/control/speed_controller.cpp` + `pid.cpp`, same gains (`SPEED_KP 0.004`, `SPEED_KI 0.02`, integral clamp 20, output clamp 0.3, 1 cm/s target deadband that resets the integral). That duty then drives the motor physics: first-order wheel speed (τ 60 ms), deadband, battery/gain spread, 420 counts/rev |
| 4× VL53L0X at -60°, -20°, 20°, 60° (edge pair angled out, middle pair nearly forward) | each sensor's 25° cone cast as 5 rays, 3–120 cm, σ = max(0.5 cm, 3 %), 1 % dropouts, one-step latency, ~1° / 3 mm mounting error per episode |
| IMU | MPU6050 gyro-z: bias (with random walk), scale error, noise |
| Fused odometry | Encoder + gyro complementary filter (gyro bias calibrated while locked on the line, encoders ignored when they disagree with the gyro, i.e. slip) — ports `lib/estimation/odometry.cpp` |
| Pose estimate (`env.est`) | **Dead reckoning only by default** (`localize=False`): `firmware/lib/estimation` has nothing but `FusedOdometry` today — no EKF, no map-matching. `localization.py`'s 150-particle filter still exists and can be turned on with `localize=True` for comparison, but it is not what the car actually runs |

Body 12.5×10 cm (limit is 15×10). The tape is **not** seen by any sensor, so staying on the track
relies on the estimated pose.

Errors the estimator and drivers do not know about: start pose error (1 %: 0.3 cm, 0.1 cm, 0.9°), wheel
radius / track / gain spread, wheel traction random walk, up to 8 % slip while accelerating hard, gyro
bias drift, ToF latency, sensor mounting error. The physics/speed-loop tick (`SUBSTEPS = 4`, ~8.3 ms) is
coarser than the firmware's real 200 Hz (5 ms) wheel loop — a reasonable approximation, not exact.

**Finding from porting the PID verbatim:** `Robot.place()` (used on the initial drop and on every
Điều 5 reposition) resets wheel speed to zero but must also reset the PID's integral and the measured-speed
filter — without that, a car stuck against a block (repeatedly repositioned) winds the integral up to its
clamp and stays pinned there. The firmware has the same gap: `src/course.cpp` never calls `wheelEnable(false)` on a
mid-run reposition, so `Wheel`'s PID integral is never reset either. Worth fixing there too.

## Rules implemented (`course/env.py`)

Obstacle +10 / random +15 (the whole car past the last dividing line of the block's part, Điều 4; car inside the map; block undisturbed),
finish +40 (whole car past the line), speed (240 − t)/6, clean +10, wheel fully outside the tape −5,
block displaced > 2 cm −5 and its points lost, reposition −10. Leaving the map (including cutting
through the U) or being stuck for 4 s forces a reposition near the last passed block, as in Điều 5.
Contact and tape touching are free.

## Drivers

**A* + pure pursuit (`course/planner.py`)** – plans on the stored static map (clearance-weighted A*, string
pulling, corner smoothing), tracks with pure pursuit on the corrected (now: dead-reckoned) pose, and
compares the ToF readings with what the map predicts. A reading much shorter than expected is an unmapped
(random) block: it is added as an obstacle, the car slows down and replans. It slows in the right-hand
cells where the random blocks can be, and now also near the track edge (`EDGE_SLOW_START`) -- see below.
It uses only the car's own pose estimate, ToF, map and wheel speed.

**Wheel-out margin.** The planned path has to clear the tape by `R_EDGE` (7 cm): a wheel sits
`TRACK/2 + wheel_half_w` = 5.0 cm off the centreline, so anything less doesn't even cover the robot's own
geometry. But the 60 cm-wide corridor (with `R_ROBOT` clearance from obstacles also eating into it) cannot
support a much bigger *static* margin -- above ~8 cm the planner starts failing to find a route at all.
Since real wheel-out mostly comes from dead-reckoning drift (several cm on a longer run, not tracking lag
-- confirmed by logging true-vs-planned deviation at wheel-out events), the fix that actually helps is
`EDGE_SLOW_START`: the path's speed profile drops as low as 0.5x within 22 cm of the boundary, giving pure
pursuit more time to correct before the margin runs out, the same way it already slows near obstacles.

**Unmapped-block detection.** The single-ray "near face + half block behind" centre estimate used when a
block isn't on the map drifts a lot with viewing angle (as much as ~19 cm between two detections of the
same physical block from different angles). With denser obstacles this showed up as a real bug: the merge
threshold for treating two detections as "the same block" (8 cm) was tighter than that drift, so repeated
detections of the same random block created ghost duplicates that filled `MAX_EXTRA` and crowded out
the real one. Fixed by widening the merge threshold to 18 cm (safely under the 17 cm minimum spacing
between genuinely different blocks, so it can't falsely merge two real obstacles) and the avoidance disc
(`DISC_R`) from 8 to 11 cm, to buy back margin for the estimate's own imprecision. Both fixes are mirrored
in `firmware/lib/nav` (same constants, same reasoning in the comments there).

**Learned MLP (`course/policy.py`, `course/train.py`)** – 12→32→32→2 tanh network trained with antithetic evolution
strategies (numpy only, parallel). Reward = official score deltas + a small progress term. Random block spot and sensor noise per episode. Its output is now a target-speed fraction (see above), so a checkpoint trained before
the wheel-speed-loop change is not compatible; retrain from scratch.

Results on the course (40 episodes, seed 1000+, dead-reckoning pose, ToF at ±60° / ±20°; the maximum score on this
map is 274):

| Driver | Mean score | Finish rate | Mean finish time | Clean runs |
|---|---|---|---|---|
| Learned MLP (`checkpoints/policy.npz`, 60°/20°, 600 generations) | 259.2 | 100% | 6.9 s | 19/40 |
| A* + pure pursuit | 258.3 | 100% | 18.6 s | 24/40 |
| Hand-written baseline | 203.2 | 100% | 27.8 s | 0/40 |

"Pose error" and "pose error without map correction" in `evaluate` output are identical: with `localize=False` the
estimate *is* the raw dead-reckoning odometry.

## Maze (`maze/`)

A random 16×16 maze with 18 cm cells and 1.2 cm walls: a perfect maze plus 22 extra openings, with the 2×2 centre goal
reached through a single door. The car is the course car with a 9 cm long body (the 12.5 cm one cannot turn in place in a
16.8 cm corridor); it still has 4 ToF, the encoder/gyro odometry and the same motor model. The driver is flood fill: it
starts knowing only the outer wall, reads walls from the sensors while it drives, searches start → centre → start until the
shortest path consists of cells it has seen, then speed-runs it. No training.

What it took to make it hold together (the sim's odometry under-reads distance by a few percent and the gyro drifts):
- wall tests use the median of the last 3 readings (a 1 % dropout would otherwise hide a wall), and a front wall must make
  *both* middle sensors short (one alone is a side wall);
- stop and brake on the measured range to a front wall, not on odometry;
- a side wall starting or ending sits on a cell boundary, which re-syncs the position along the corridor;
- the edge sensors' wall distance vs distance driven gives the yaw to the corridor, which holds the heading estimate and
  retunes the gyro bias.

Results (30 mazes): 28 solved at 60°/20°, 29 at 90°/30°; mean search ~250 s, speed run ~35 s at 32 cm/s (faster makes it
lose its position). `set_layout(edge, mid)` in `maze/driver.py` re-derives every threshold for another sensor layout.

## Sensor layout study

Same car, same noise; "clean" = finished with no wheel out, no displaced block, no reposition.

| ToF edge / middle | Course, learned policy: mean score, clean of 40 | Maze: solved of 30 |
|---|---|---|
| ±10° / ±30° | 257.5, 20 | not tested (front walls are hidden behind the side walls) |
| **±60° / ±20° (current)** | **259.2, 19** (after 300 more generations; 256.2, 18 before) | **28** |
| ±60° / ±30° | 251.3, 14 | 27 |
| ±70° / ±20° | 246.4, 12 | not tested |
| ±90° / ±30° | 245.3, 13 | 29 |
| ±45° / ±15° | not trained | 0 (wall edges no longer line up with the cell boundary) |

For the A* driver (clean runs of 40) the sensor layout matters much less: 16–30 across 28 layouts, best at ±90°/±30°.
The learned policy prefers forward-looking sensors, the maze driver prefers sideways ones; ±60°/±20° is within noise of
the best on both. Differences of a few clean runs are inside the run-to-run noise.

## Assumptions to check against the real event

- The static layout is the official map, read off the diagram to the nearest 5 cm (`config.STATIC_BLOCKS`). The 4 random
  spots are not marked in the diagram; `config.RANDOM_SPOTS` holds placeholders. The 6 / 3 dividing-line parts
  per tile (Điều 4) are my reading: 10 cm bands on straight tiles, 30° fans from the inside corner on corner tiles. A map built by hand-driving run 1 will have odometry drift that this sim does not model
  (mapping error here is 0.5 cm / 2°).
- After a reposition the odometry is re-seeded at the placement pose (±1.5 cm, ±2°). A restarted car would
  zero it, so the firmware needs a known-pose restart to match.
- Blocks are light foam: pushed by the car with only a mild slowdown.
- The particle filter (150 particles, 5 rays per sensor) is too heavy for the STM32 as written, and the firmware
  does not have it yet regardless — see "Pose estimate" above.
