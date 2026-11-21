# Tools (run on the PC)

| Tool | What it does |
|---|---|
| `read_run.py` | Reads the last two runs from the robot's flash over ST-Link and prints them; `--csv run` saves each run's 30 Hz log as `run_<n>.csv`. |
| `replay_run.cpp` | Replays a saved run through the random-block detection with the course's own settings and prints the blocks it would find. A run without a random block must find none. Build command in the file header. |
| `make_baseline.py` | Builds `include/baseline.h` (what the ToF sensors read from each spot on clean runs) from saved clean runs. Rebuild it after the course or a sensor changes. |
| `read_step.py` | Reads the motor step test (`env:test_step`, `env:test_step_floor`) and prints each wheel's speed per duty and response time. |

## Typical session

```
python3 tools/read_run.py --csv data/clean_runs/run          # after two clean runs (no random block)
python3 tools/make_baseline.py data/clean_runs/*.csv > include/baseline.h
pio run -e course_nav -t upload
# after a run with the random block:
python3 tools/read_run.py --csv /tmp/block
/tmp/replay_run /tmp/block_<n>.csv                           # when and where it was found
```
