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

Firmware tuning/start/finish settings live in `src/course.cpp`, the stored map in
`include/map_data.h`, and sensor geometry/wiring in `include/config.h`.
Build with `pio run -e course_nav`; run host logic tests with
`pio test -e native`.
