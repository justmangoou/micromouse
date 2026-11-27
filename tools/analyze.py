"""Read the robot's two run logs, compare them and draw them on the map.

    python3 tools/analyze.py                         # read the robot over ST-Link, save to data/<date-time>/
    python3 tools/analyze.py run1.csv run2.csv       # analyse saved logs instead
Needs matplotlib and numpy (requirements.txt). Positions are the robot's own estimate, not a measurement.
"""
import datetime, os, subprocess, sys

import numpy as np

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
from plot_run_map import MAP_BLOCKS, draw_runs, load_run  # noqa: E402


def stats(d):
    t, x, y = d["ms"] / 1000, d["x"], d["y"]
    v = (d["measL"] + d["measR"]) / 2
    start = int(np.argmax(np.hypot(x - x[0], y - y[0]) > 5))
    gap, tag = min((np.hypot(np.maximum(abs(x - bx) - 50, 0), np.maximum(abs(y - by) - 50, 0)).min(), tag)
                   for bx, by, tag in MAP_BLOCKS)
    err = np.sqrt((((d["tgtL"] - d["measL"]) ** 2 + (d["tgtR"] - d["measR"]) ** 2) / 2)[start:].mean())
    finished = y[-1] < -70 and x[-1] > 1215
    return (f"{t[-1]:.2f} s{'' if finished else ' (log ends before the finish)'}, "
            f"{np.hypot(np.diff(x), np.diff(y)).sum():.0f} mm, mean {v[start:].mean():.0f} / peak {v.max():.0f} mm/s, "
            f"speed error {err:.0f} mm/s, closest {gap:.0f} mm to {tag}")


def main():
    paths = sys.argv[1:]
    if not paths:
        out = os.path.join(HERE, "..", "data", datetime.datetime.now().strftime("%Y%m%d-%H%M%S"))
        os.makedirs(out)
        text = subprocess.run([sys.executable, os.path.join(HERE, "read_run.py"), "--csv", os.path.join(out, "log")],
                              capture_output=True, text=True, check=True).stdout
        with open(os.path.join(out, "summary.txt"), "w") as f:
            f.write(text)
        for line in text.splitlines():  # the slot headers: result, replans, blocks found, faults
            if line.startswith("===") or "replans" in line or "blocks found" in line:
                print(line)
        paths = [p for p in (os.path.join(out, "log_run1.csv"), os.path.join(out, "log_run2.csv")) if os.path.exists(p)]
    else:
        out = os.path.dirname(os.path.abspath(paths[0]))
    runs = [(os.path.basename(p), d) for p in paths if (d := load_run(p))]
    if not runs:
        sys.exit("no run logs")
    for name, d in runs:
        print(f"{name}: {stats(d)}")
    png = os.path.join(out, "runs.png")
    draw_runs([(os.path.splitext(name)[0], d) for name, d in runs], png)
    print("map:", os.path.normpath(png))


if __name__ == "__main__":
    main()
