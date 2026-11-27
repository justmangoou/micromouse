# Tools (run on the PC)

| Tool | What it does |
|---|---|
| `analyze.py` | Reads both run logs from the robot, saves them under `data/<date-time>/`, prints a comparison and draws them on the map. Also takes saved CSVs. |
| `plot_run_map.py` | Draws the course map for `analyze.py`; run on its own, it redraws the figures in `docs/media/`. |
| `read_run.py` | Reads the two run slots over ST-Link and prints them; `--csv x` saves `x_run1.csv`, `x_run2.csv`. |
| `replay_run.cpp` | Replays a saved run through the block detection, with another run's CSV as the reference. A run without a random block must find none. Build command in the file header. |
| `read_step.py` | Reads the motor step test (`env:test_step`, `env:test_step_floor`) and prints each wheel's speed per duty and response time. |

## A session

1. Run 1, no random block (LED: slow even blink). A clean finish makes it the reference until power-off or reset.
2. Run 2, with the block (LED: double blink). Detection compares against run 1. Nothing needs a PC in between.
3. `python3 tools/analyze.py` with the ST-Link connected.

Switching off or resetting forgets the reference (the logs stay readable); so does holding KEY for 2 s during the
countdown.
