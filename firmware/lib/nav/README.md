# Navigation

Firmware distances use millimeters, speeds use mm/s, and headings use radians.
Floating-point arithmetic is retained. The simulator uses centimeters; its
export and comparison tools convert at the firmware boundary. Course CSV logs
continue to report mm, mm/s, and mrad without additional distance scaling.

`Navigator` follows the existing block-obstacle course using A*. For cell-based
micromouse mazes, use `FloodFill` from `floodfill.h`:

```cpp
nav::FloodFill maze; // 16 x 16, four center goals, unknown edges traversable
nav::Cell current{0, 0};
nav::Direction heading = nav::Direction::North;
maze.setPassage(current, nav::Direction::North, nav::Passage::Open);
maze.setPassage(current, nav::Direction::East, nav::Passage::Wall);

nav::Cell next;
nav::Direction direction;
if (maze.nextMove(current, heading, next, direction)) {
    // Turn to direction and move to next with your cell motion controller.
    // Update current and heading after completing the move, then observe again.
}
```

Coordinates increase east (+x) and north (+y). Dimensions can be reset to any
rectangle from 1 x 1 to 16 x 16. Boundary edges are always walls. Each observation
updates both cells sharing the edge; queries automatically recompute shortest
cell distances after changes. Equal shortest routes prefer straight, left,
right, then back. A goal or unreachable cell returns no move.

Use `setGoals(&start, 1)` for a return run, `setCenterGoals()` to restore center
targets, and `setMode(nav::MazeMode::KnownOnly)` to restrict routes to observed
open passages. The planner uses fixed storage and no heap allocation.

Odometry lives here too (`odometry.h`), shared by navigation modes.

Each navigation mode is one source file in `src/`: `course.cpp` runs the
block-course navigator, `maze.cpp` runs `Micromouse` to turn, inspect passages,
and advance one cell at a time until reaching a center goal. Both share the
start-up (IMU calibration) and sensor loop in `main.cpp`. All sensors must initialize; the start button
requires good inner ToF readings and a gyro sample before enabling motors.

Place the robot at the southwest cell center facing north. Set maze dimensions,
cell size, speed, turn rate, wall thresholds and motion tolerances in
`mazeParams()` at the top of `src/maze.cpp` before running. Defaults are a 16 x 16 maze with 180 mm cells
and 120 mm/s forward speed. This mode uses its own cell-center start pose rather
than the block-course start pose in `src/course.cpp`.

The inner +/-20 degree sensors inspect edges head-on, accounting for their
mounts. Both must agree on a wall or open passage. The planner may route through
unknown edges, but the motion controller turns to inspect them before crossing.
The oblique outer sensors are not used to infer side walls. Invalid, no-target
and stale measurements are not evidence of an open passage; missing reliable
inner ranges stop motion or time out an observation. A long open corridor with
no usable range return may therefore require different sensing before running.

Heading uses the calibrated gyro and encoders. Advancement uses the estimated
pose, heading correction and lateral error correction; it does not relocalize
against walls. Cell coordinates are committed after completing and settling a
move. Sensor loss, excessive pose error and timeouts stop the motors. This is
an initial exploration implementation: no persistent map, return run, speed run,
or wall-based drift correction. Validate tuning on hardware before a full maze.

Build or flash one mode with:

```sh
pio run -e obstacle_avoidance
pio run -e obstacle_avoidance -t upload
pio run -e micromouse
pio run -e micromouse -t upload
```

`platformio.ini` builds `maze.cpp` or `course.cpp` per environment; the linker
drops the navigation code the other mode would use. `micromouse` is the default
environment for a plain `pio run`; change `default_envs` to switch.

Run host tests with `pio test -e native`.
