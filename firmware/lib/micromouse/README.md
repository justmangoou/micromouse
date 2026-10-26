# Micromouse navigation

Hardware-independent maze exploration: `FloodFill` plans routes through a cell
maze, and `Micromouse` controls observing, turning, advancing, and settling.
Distances are mm, wheel commands mm/s, angles radians, and time seconds.
This library has no dependency on the course planner, board headers, or HAL.

Construct `Micromouse(layout, params)` with a `tof::Layout` and the indices of
its two forward-facing sensors in `MicromouseParams::leftSensor/rightSensor`.
Both sensors must be distinct, in bounds, and face forward. Provide arrays of
`layout.count` distances and reliability flags to `step()` in the same order.
Each instance copies its layout; changing the caller's layout does not change it.
The firmware supplies `cfg::TOF_LAYOUT` and its inner-sensor indices.

The inner beams inspect an edge head-on; both must agree before recording a wall
or open passage. Mount offsets and angles come from the supplied layout. Invalid,
no-target, and stale readings do not establish an open passage. A long corridor
without a usable range return may therefore prevent motion.

`start()` resets the maze and motion state. `pause()/resume()` preserve cell,
heading, movement phase, and maze knowledge, exclude recovery time from phase
timeouts, and align heading before continuing. The overall run deadline includes
recovery. Excessive pose error and motion timeouts enter Fault with zero commands.
There is no wall-based relocalization, persistent map, return run, or speed run.

`FloodFill` supports rectangular mazes up to 16 x 16. Coordinates increase east
(+x) and north (+y); the start is the southwest cell center facing north. Boundary
edges are walls. Edge observations update both adjacent cells; route ties prefer
straight, left, right, then back. Explore mode may plan through unknown edges,
which the motion controller inspects before crossing. KnownOnly mode requires
observed openings. `setGoals()` and `setCenterGoals()` choose route goals.

Firmware tuning lives in `src/maze.cpp`; physical geometry and sensor wiring live
in `include/config.h`. Build with `pio run -e micromouse` and run host logic tests
with `pio test -e native`.
