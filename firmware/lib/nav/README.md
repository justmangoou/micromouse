# Navigation

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

Select `NAV_MODE` in `main.cpp` via `nav_mode.h`: `NAV_MICROMOUSE` or
`NAV_OBSTACLE_AVOIDANCE`. The obstacle-avoidance mode runs the block-course
navigator. The micromouse entry point initializes flood fill and reports its
first cell target with motors disabled. Maze sensing, cell
localization and turn/advance motor control must supply and execute the discrete
maze API; raw ToF values are not interpreted by this planner.

Build or flash one mode with:

```sh
pio run -e obstacle_avoidance
pio run -e obstacle_avoidance -t upload
pio run -e micromouse
pio run -e micromouse -t upload
```

Each mode excludes the other planner's sources. `nav_sources.py` selects sources
for these environments; native tests build both. The default `blackpill_f411ce`
environment keeps the LED bring-up test. Change `default_envs` in
`platformio.ini` to your preferred navigation environment for plain `pio run`.

Run host tests with `pio test -e native`.
