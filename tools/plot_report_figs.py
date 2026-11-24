"""Figures for docs/report/report.typ, drawn from the two newest logged runs in data/ (flash_run_9.csv, flash_run_10.csv).

    .venv/bin/python tools/plot_report_figs.py
"""
import csv
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "..", "data")
OUT = os.path.join(HERE, "..", "docs", "report", "fig")


def load(name):
    with open(os.path.join(DATA, name)) as f:
        rows = list(csv.DictReader(f))
    return {k: [float(r[k]) for r in rows] for k in rows[0]}


def main():
    run = load("flash_run_10.csv")
    other = load("flash_run_9.csv")
    t = [m / 1000 for m in run["ms"]]
    t9 = [m / 1000 for m in other["ms"]]

    fig, ax = plt.subplots(3, 1, figsize=(7.2, 7.0), sharex=True)
    avg = lambda d: [(a + b) / 2 for a, b in zip(d["measL"], d["measR"])]
    tgt = lambda d: [(a + b) / 2 for a, b in zip(d["tgtL"], d["tgtR"])]
    ax[0].plot(t, tgt(run), color="#9ca3af", lw=1.2, label="target (mean of wheels)")
    ax[0].plot(t, avg(run), color="#1d4ed8", lw=1.6, label="measured (mean of wheels)")
    ax[0].plot(t9, avg(other), color="#d97706", lw=1.0, ls="--", label="run 9, measured")
    ax[0].set_ylabel("speed [mm/s]")
    ax[0].legend(frameon=False, fontsize=8, ncol=3, loc="upper center")
    ax[0].set_ylim(0, 800)

    ax[1].plot(t, [a / 1000 for a in run["alpha_mrad"]], color="#1d4ed8", lw=1.4)
    ax[1].axhline(0, color="#9ca3af", lw=0.6)
    ax[1].set_ylabel("steering angle α [rad]")

    for key, name, c in (("tofR", "right 45°", "#047857"), ("tofF", "front", "#b91c1c"), ("tofL", "left 45°", "#6d28d9")):
        ax[2].plot(t, run[key], lw=1.2, label=name, color=c)
    ax[2].set_ylabel("ToF range [mm]")
    ax[2].set_xlabel("time since start [s]")
    ax[2].legend(frameon=False, fontsize=8, ncol=3, loc="upper center")
    ax[2].set_ylim(0, 1400)
    for a in ax:
        a.grid(alpha=0.25)
        for s in ("top", "right"):
            a.spines[s].set_visible(False)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "run10_signals.png"), dpi=200)


def blocks():
    import re
    txt = open(os.path.join(HERE, "..", "include", "map_data.h")).read()
    return [(float(a), float(b)) for a, b, _ in re.findall(r"\{([\d.]+)f, ([\d.]+)f, ([\d.]+)f\}", txt)]


def speed_map():
    from matplotlib.collections import LineCollection
    from matplotlib.patches import Rectangle
    run = load("flash_run_10.csv")
    pts = list(zip(run["x"], run["y"]))
    segs = [[pts[i], pts[i + 1]] for i in range(len(pts) - 1)]
    v = [(a + b) / 2 for a, b in zip(run["measL"], run["measR"])]
    fig, ax = plt.subplots(figsize=(7.2, 5.4))
    ax.add_patch(Rectangle((600, 0), 600, 600, fc="#fee2e2", ec="#111827", lw=2, hatch="//", alpha=0.6))
    for i, (bx, by) in enumerate(blocks()):
        ax.add_patch(Rectangle((bx - 50, by - 50), 100, 100, fc="#cbd5e1", ec="#1f2937", lw=1))
        ax.text(bx, by, f"T{i + 1}", ha="center", va="center", fontsize=7)
    lc = LineCollection(segs, array=v[:-1], cmap="viridis", linewidths=4)
    lc.set_clim(150, 620)
    ax.add_collection(lc)
    cb = fig.colorbar(lc, ax=ax, fraction=0.04, pad=0.02)
    cb.set_label("speed [mm/s]")
    ax.plot([0, 600], [0, 0], color="#16a34a", lw=3, ls="--")
    ax.plot([1200, 1800], [0, 0], color="black", lw=3, ls=":")
    ax.set_xlim(0, 1800); ax.set_ylim(-150, 1200); ax.set_aspect("equal")
    ax.set_xlabel("x [mm]"); ax.set_ylabel("y [mm]")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "speed_map.png"), dpi=200)


def tuning():
    fig, ax = plt.subplots(figsize=(6.4, 3.0))
    t = [11.7, 10.7, 8.9, 8.5]
    ax.bar(range(4), t, color="#1d4ed8", width=0.55)
    for i, x in enumerate(t):
        ax.text(i, x + 0.15, f"{x} s", ha="center", fontsize=10)
    ax.set_xticks(range(4)); ax.set_xticklabels(["start", "step 2", "step 3", "step 4"], fontsize=9)
    ax.set_ylabel("run time [s]"); ax.set_ylim(0, 13.5)
    for sp in ("top", "right"):
        ax.spines[sp].set_visible(False)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "tuning.png"), dpi=200)


if __name__ == "__main__":
    main()
    speed_map()
    tuning()
