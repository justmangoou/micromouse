# Logged runs

Positions are the robot's own estimate, not a measurement. Columns: time, pose (x, y, heading), steering angle,
wheel targets and measured speeds, and the three distance readings.

| Folder | What's in it |
|---|---|
| `block_runs/` | Current setup, with the extra block. `reference.csv` is run 1 (no block), shared by all. Each `HHMMSS.csv` is one run 2 on 2026-11-26, with its `.txt` summary. |
| `speed_tests/` | Fast driving profile. Runs 2-4 without a block; runs 6-8 with a block but detection off. |
| `old_setup/` | Earlier, slower setup: runs 18-19 with a block, 21-22 without. |
| `report/` | The four runs quoted in the project report. |

Detected block position in `block_runs/` (mm):

| Run | Block | Log length |
|---|---|---|
| 131149 | none detected | 6.4 s |
| 132002 | none detected | 6.4 s |
| 132827 | (1282, 492) | 10.4 s |
| 133519 | (1278, 533) | 10.4 s |
| 134210 | (1278, 570) | 8.8 s |
| 135959 | (1250, 113) | 10.5 s |
| 140636 | (1250, 663) | 8.9 s |
| 140850 | (1250, 661) | 8.8 s |

`tools/analyze.py` saves each new reading in its own date-time folder here. Draw any saved runs with
`python3 tools/analyze.py data/block_runs/reference.csv data/block_runs/140850.csv`.
