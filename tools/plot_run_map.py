#!/usr/bin/env python3
"""Course map drawing, shared with analyze.py, and the figures used in docs/.

    python3 tools/plot_run_map.py        # redraw docs/media/*.png
Needs matplotlib and numpy (requirements.txt).
"""
import csv
import os
import re

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle, Polygon, Rectangle, Wedge

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
MEDIA = os.path.join(ROOT, "docs", "media")

SURFACE, INK, MUTED, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e7e6e2"
WALL, BLOCK, BLOCK_EDGE, PIT = "#3a3936", "#d9d8d3", "#8a8984", "#efeeea"
SERIES = ("#2a78d6", "#eb6834", "#1baf7a", "#eda100")  # blue, orange, aqua, yellow

plt.rcParams.update({
    "font.size": 11, "text.color": INK, "axes.labelcolor": MUTED, "xtick.color": MUTED, "ytick.color": MUTED,
    "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
})


def _map_blocks():
    with open(os.path.join(ROOT, "include", "map_data.h")) as f:
        rows = re.findall(r"\{([\d.]+)f, ([\d.]+)f, [\d.]+f\}", f.read())
    return [(float(x), float(y), f"T{i + 1}") for i, (x, y) in enumerate(rows)]


MAP_BLOCKS = _map_blocks()


def draw_course_arena(ax):
    """The 1800 x 1200 mm course: walls, pit, start and finish lines, the mapped blocks."""
    ax.add_patch(Rectangle((600, 0), 600, 600, fc=PIT, ec="none", zorder=1))
    wall = dict(color=WALL, lw=3, solid_capstyle="projecting", zorder=3)
    ax.plot([0, 0, 1800, 1800], [-150, 1200, 1200, -150], **wall)
    ax.plot([600, 600, 1200, 1200], [0, 600, 600, 0], **wall)
    ax.plot([0, 600], [0, 0], color=MUTED, lw=1.5, ls=(0, (5, 4)), zorder=2)
    for i in range(24):  # checkered finish line
        for j in range(2):
            if (i + j) % 2 == 0:
                ax.add_patch(Rectangle((1200 + i * 25, -12 + j * 12), 25, 12, fc=WALL, ec="none", zorder=2))
    for x, label in ((300, "START"), (1500, "FINISH")):
        ax.text(x, -125, label, ha="center", va="center", color=MUTED, fontsize=10, fontweight="bold")
    for x, y, tag in MAP_BLOCKS:
        ax.add_patch(Rectangle((x - 50, y - 50), 100, 100, fc=BLOCK, ec=BLOCK_EDGE, lw=1, zorder=4))
        ax.text(x, y, tag, ha="center", va="center", fontsize=8.5, color=INK, zorder=5)
    ax.set_xlim(-30, 1830)
    ax.set_ylim(-170, 1230)
    ax.set_aspect("equal")
    ax.set_xticks(range(0, 1801, 300))
    ax.set_yticks(range(0, 1201, 300))
    ax.tick_params(length=0, labelsize=9)
    for spine in ax.spines.values():
        spine.set_visible(False)


def clean_axes(ax):
    ax.grid(axis="y", color=GRID, lw=0.8)
    ax.set_axisbelow(True)
    ax.tick_params(length=0)
    for side in ("top", "right", "left"):
        ax.spines[side].set_visible(False)
    ax.spines["bottom"].set_color(BLOCK_EDGE)


def load_run(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    return {k: np.array([float(r[k]) for r in rows]) for k in rows[0]} if rows else None


def draw_runs(runs, path, title=None, block=None):
    """runs: (label, data) pairs. Saves the paths on the map above the speed along each run."""
    fig, (arena, speed) = plt.subplots(2, 1, figsize=(9, 10), height_ratios=[2.5, 1], layout="constrained")
    draw_course_arena(arena)
    if block:
        arena.add_patch(Rectangle((block[0] - 50, block[1] - 50), 100, 100, fc="none", ec=INK, lw=1.8,
                                  ls=(0, (3, 2)), zorder=6))
        arena.annotate("extra block\n(estimated)", (block[0] - 50, block[1]), (900, 450), fontsize=10,
                       ha="center", va="center", arrowprops=dict(arrowstyle="-", color=MUTED, lw=1), zorder=9)
    for (label, d), colour in zip(runs, SERIES):
        arena.plot(d["x"], d["y"], color=colour, lw=2.4, solid_capstyle="round", zorder=8, label=label)
        arena.plot(d["x"][-1], d["y"][-1], "o", ms=8, color=colour, mec=SURFACE, mew=1.5, zorder=9)
        dist = np.r_[0, np.cumsum(np.hypot(np.diff(d["x"]), np.diff(d["y"])))]
        speed.plot(dist / 1000, (d["measL"] + d["measR"]) / 2, color=colour, lw=2, label=label)
    arena.legend(loc="center", bbox_to_anchor=(900, 170), bbox_transform=arena.transData, frameon=False, fontsize=9.5)
    if title:
        arena.set_title(title, loc="left", fontsize=13, fontweight="bold")
    clean_axes(speed)
    speed.axhline(0, color=BLOCK_EDGE, lw=0.8)
    speed.set_xlabel("distance driven (m)")
    speed.set_ylabel("speed (mm/s)")
    fig.savefig(path, dpi=130)
    plt.close(fig)


def draw_detection(path):
    """Schematic of Navigator::detectBlocks: a short reading, and where in the sensor's cone the block is placed."""
    fig, ax = plt.subplots(figsize=(8, 7.4), layout="constrained")
    seen, expected, route_x = 350, 560, 70

    ax.add_patch(Rectangle((route_x - 60, 0), 120, 640, fc=SERIES[0], alpha=0.10, ec="none"))
    ax.plot([route_x, route_x], [0, 640], color=SERIES[0], lw=2.4)
    ax.add_patch(Wedge((0, 0), expected, 78, 102, fc="none", ec=MUTED, lw=1, ls=(0, (4, 3))))
    ax.add_patch(Wedge((0, 0), seen, 78, 102, fc=SERIES[1], alpha=0.14, ec="none"))
    for radius, colour, style in ((seen, SERIES[1], "-"), (expected, MUTED, (0, (4, 3)))):
        a = np.radians(np.linspace(78, 102, 40))
        ax.plot(radius * np.cos(a), radius * np.sin(a), color=colour, lw=2, ls=style)

    centres = [((seen + 50) * np.cos(a), (seen + 50) * np.sin(a)) for a in np.radians([102, 96, 90, 84, 78])]
    chosen = min(centres, key=lambda c: abs(c[0] - route_x))
    for c in centres:
        if c != chosen:
            ax.add_patch(Circle(c, 7, fc=SURFACE, ec=MUTED, lw=1.4, zorder=5))
    ax.add_patch(Rectangle((chosen[0] - 50, chosen[1] - 50), 100, 100, fc="none", ec=INK, lw=1.8, zorder=6))
    ax.add_patch(Circle(chosen, 8, fc=INK, ec=SURFACE, zorder=7))

    ax.add_patch(Polygon([(-45, -95), (45, -95), (45, -5), (0, 20), (-45, -5)], fc=BLOCK, ec=WALL, lw=1.5))
    ax.text(0, -55, "robot", ha="center", va="center")

    def note(text, xy, xytext):
        ax.annotate(text, xy, xytext, fontsize=10.5, va="center", ha="left" if xytext[0] > xy[0] else "right",
                    arrowprops=dict(arrowstyle="-", color=BLOCK_EDGE, lw=1))

    note("run 1 read this far\nfrom the same spot", (-95, 550), (-175, 590))
    note("now the reading\nis much shorter", (-60, 343), (-175, 300))
    note("5 possible centres:\nreading + 50 mm,\nacross the ±12° cone", centres[0], (-175, 450))
    note("planned path\nand 60 mm each side", (route_x + 55, 180), (190, 180))
    note("chosen: the one\nclosest to the path", (chosen[0] + 50, chosen[1]), (190, chosen[1]))

    ax.set_xlim(-360, 370)
    ax.set_ylim(-110, 650)
    ax.set_aspect("equal")
    ax.axis("off")
    fig.savefig(path, dpi=130)
    plt.close(fig)


def main():
    os.makedirs(MEDIA, exist_ok=True)
    runs = os.path.join(ROOT, "data", "block_runs")
    draw_runs([("run 1: no extra block", load_run(os.path.join(runs, "reference.csv"))),
               ("run 2: goes around it", load_run(os.path.join(runs, "140850.csv")))],
              os.path.join(MEDIA, "block_run.png"), block=(1250, 661))
    draw_detection(os.path.join(MEDIA, "detection.png"))
    print("wrote", os.path.normpath(MEDIA))


if __name__ == "__main__":
    main()
