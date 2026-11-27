# Course navigation

Block-course geometry, clearance-weighted A*, path smoothing, pure-pursuit
tracking, and discovery of unmapped obstacles. Distances are mm, speeds mm/s,
angles radians, and wheel commands fractions of full-duty speed in [-1, 1].
Storage is fixed; the planner arrays occupy approximately 36 KB. The A* open list holds 1024 entries (peak
observed in the host tests: 104); when full it drops entries, which can only cost path optimality. The A* cost array and
the path post-processing buffers share memory, so `plan()` and `straight()` clobber neither a live search nor a
returned `Path`; do not call planner internals concurrently.
This library has no dependency on micromouse navigation, firmware configuration,
or HAL. Course dimensions and finish geometry are specific to this project.

Construct `Navigator(layout, params)` with the robot's `tof::Layout`. The layout
is copied into the navigator and validated with its tuning parameters. Supply
`layout.count` ranges to `step()` in sensor order. Obstacle projection uses the
supplied mounts, angles, ray offsets, and maximum distance. Firmware owns health
and freshness checks before calling navigation.

`init()` installs the stored map. `start()` clears discovered obstacles for a new
run and plans its initial route. `reposition()` replans from the retained pose
without clearing discovered obstacles. Both return route success. Failed planning
clears the route; `hasRoute()` becomes false and `step()` returns zero commands.
There is no automatic straight-line fallback.

Random-block detection (`Navigator::detectBlocks`) compares each ToF reading with what it should be: the shortest
reading a run without the block recorded near the same pose (`setBaseline`), or the map's blocks and
walls where there is no baseline. `DetectParams` holds its limits.

The course's settings (start, finish, `NavParams`) live in `include/course_params.h`, the stored map in
`include/map_data.h`, and sensor geometry/wiring in `include/config.h`.
Build with `pio run -e course_nav`; run host logic tests with
`pio test -e native`.

## Memory profiles

The planner is all static memory. Two profiles, chosen at compile time:

| | default | `-DNAV_LOW_MEMORY` |
|---|---|---|
| Grid | 20 mm cells (91 x 71) | 40 mm cells (46 x 36) |
| Path / coarse path / open list | 420 / 400 / 1024 | 200 / 160 / 200 |
| Path spacing, clearance sample spacing | 15 mm, 10 mm | 20 mm, 20 mm |
| Speed profile | every path point | every second point, interpolated |
| RAM (planner + path) | ~46 KB | ~13 KB |

The default is the planner as first prototyped in the (now removed) Python simulator. The low-memory profile is what `env:course_nav` builds for the Blue Pill
(20 KB RAM, no FPU); it has the same algorithm on a coarser grid. `pio test -e native` and `pio test -e native_lowmem`
run the same host tests against both. On the Blue Pill a route plan takes about 145 ms and blocks the control loop for
that long (`src/course.cpp` handles the step that replans: see `noteStall()` in `include/robot.h`); a normal navigator
step takes under 20 ms. `env:test_planner` measures this on the board.

