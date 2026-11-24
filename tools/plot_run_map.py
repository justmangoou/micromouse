#!/usr/bin/env python3
"""
Generate high-resolution map and run diagnostic plots from flash-extracted run logs.
Supports individual clean run plots as well as comparison plots.
"""
import csv
import math
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

# Coordinates and Arena configuration (in mm)
ARENA_W = 1800.0
ARENA_H = 1200.0
CELL = 600.0
PAD = 160.0  # mm below start/finish line
BLOCK_SIZE = 100.0  # 10 cm x 10 cm

MAP_BLOCKS = [
    (50.0, 150.0, "T1"),
    (550.0, 250.0, "T2"),
    (300.0, 350.0, "T3"),
    (550.0, 450.0, "T4"),
    (50.0, 550.0, "T5"),
    (50.0, 750.0, "T6"),
    (350.0, 900.0, "T7"),
    (450.0, 1150.0, "T8"),
    (650.0, 750.0, "T9"),
    (750.0, 650.0, "T10"),
    (850.0, 990.0, "T11"),
    (950.0, 650.0, "T12"),
    (1150.0, 1150.0, "T13"),
    (1250.0, 850.0, "T14"),
    (1450.0, 450.0, "T15"),
    (1450.0, 150.0, "T16"),
    (1750.0, 250.0, "T17"),
]

def load_run(path):
    rows = []
    if not os.path.exists(path):
        return rows
    with open(path, "r") as f:
        reader = csv.DictReader(f)
        for r in reader:
            rows.append({
                "ms": int(r["ms"]),
                "x": float(r["x"]),
                "y": float(r["y"]),
                "th_rad": float(r["th_mrad"]) / 1000.0,
                "tgtL": float(r["tgtL"]),
                "tgtR": float(r["tgtR"]),
                "measL": float(r["measL"]),
                "measR": float(r["measR"]),
                "tofR": float(r["tofR"]),
                "tofF": float(r["tofF"]),
                "tofL": float(r["tofL"]),
            })
    return rows

def draw_course_arena(ax):
    # Floor background
    ax.set_facecolor("#f8fafc")
    
    # Outer floor patch
    floor = patches.Rectangle((0, -PAD), ARENA_W, ARENA_H + PAD, fc="#ffffff", ec="none", zorder=0)
    ax.add_patch(floor)
    
    # Center cutout / pit [600, 1200] x [0, 600]
    pit = patches.Rectangle((CELL, 0), CELL, CELL, fc="#f8fafc", ec="#cbd5e1", lw=1.5, hatch="///", zorder=1)
    ax.add_patch(pit)
    
    # Grid lines (minor: every 100 mm, major: every 600 mm)
    for x in np.arange(0, ARENA_W + 1, 100):
        is_major = (x % CELL == 0)
        ax.plot([x, x], [-PAD, ARENA_H], color="#94a3b8" if is_major else "#e2e8f0",
                lw=1.2 if is_major else 0.6, ls="-" if is_major else ":", zorder=1)
    for y in np.arange(-PAD, ARENA_H + 1, 100):
        is_major = (y >= 0 and y % CELL == 0)
        ax.plot([0, ARENA_W], [y, y], color="#94a3b8" if is_major else "#e2e8f0",
                lw=1.2 if is_major else 0.6, ls="-" if is_major else ":", zorder=1)
        
    # Boundary black tape walls
    tape_kw = dict(color="#0f172a", lw=4.5, solid_capstyle="butt", zorder=3)
    ax.plot([0, 0, ARENA_W, ARENA_W], [-PAD, ARENA_H, ARENA_H, -PAD], **tape_kw)
    ax.plot([CELL, CELL, 2 * CELL, 2 * CELL], [0, CELL, CELL, 0], **tape_kw)
    
    # Start Line (XUẤT PHÁT) at y=0, x in [0, CELL]
    ax.plot([0, CELL], [0, 0], color="#16a34a", lw=3.8, ls="--", zorder=3)
    ax.text(CELL / 2, -50, "XUẤT PHÁT (START)", color="#15803d", fontsize=11, fontweight="bold",
            ha="center", va="center", zorder=3)
    
    # Finish Line (ĐÍCH) checkered line at y=0, x in [2*CELL, ARENA_W]
    n_checks = 24
    check_w = (ARENA_W - 2 * CELL) / n_checks
    check_h = 24.0
    for i in range(n_checks):
        for j in range(2):
            if (i + j) % 2 == 0:
                ax.add_patch(patches.Rectangle((2 * CELL + i * check_w, -check_h / 2 + j * check_h / 2),
                                               check_w, check_h / 2, fc="#0f172a", ec="none", zorder=3))
            else:
                ax.add_patch(patches.Rectangle((2 * CELL + i * check_w, -check_h / 2 + j * check_h / 2),
                                               check_w, check_h / 2, fc="#ffffff", ec="none", zorder=3))
    ax.text(2 * CELL + CELL / 2, -50, "ĐÍCH (FINISH)", color="#b91c1c", fontsize=11, fontweight="bold",
            ha="center", va="center", zorder=3)

    # Cell labels (Ô1 .. Ô5) matching official layout positions
    cells = [
        ("Ô1", 60, 40),
        ("Ô2", 60, 620),
        ("Ô3", 660, 1140),
        ("Ô4", 1240, 1140),
        ("Ô5", 1260, 40)
    ]
    for name, cx, cy in cells:
        ax.text(cx, cy, name, color="#475569", fontsize=11, fontweight="bold",
                ha="center", va="center", zorder=2,
                bbox=dict(boxstyle="round,pad=0.25", facecolor="#ffffff", edgecolor="#cbd5e1", alpha=0.9))

    # Static blocks T1..T17
    half_b = BLOCK_SIZE / 2
    for bx, by, tag in MAP_BLOCKS:
        ax.add_patch(patches.Rectangle((bx - half_b + 4, by - half_b - 4), BLOCK_SIZE, BLOCK_SIZE,
                                       fc="#94a3b8", ec="none", alpha=0.35, zorder=4))
        ax.add_patch(patches.Rectangle((bx - half_b, by - half_b), BLOCK_SIZE, BLOCK_SIZE,
                                       fc="#cbd5e1", ec="#1e293b", lw=2.0, joinstyle="round", zorder=5))
        ax.text(bx, by, tag, color="#0f172a", fontsize=9.0, fontweight="bold",
                ha="center", va="center", zorder=6)

    # Coordinates
    for x in range(0, int(ARENA_W) + 1, 200):
        ax.text(x, ARENA_H + 25, f"{x}", color="#64748b", fontsize=8.5, ha="center", va="bottom", zorder=3)
    ax.text(ARENA_W / 2, ARENA_H + 55, "180 cm (1800 mm)", color="#334155", fontsize=10, fontweight="bold", ha="center")
    
    for y in range(0, int(ARENA_H) + 1, 200):
        ax.text(-25, y, f"{y}", color="#64748b", fontsize=8.5, ha="right", va="center", zorder=3)
    ax.text(-55, ARENA_H / 2, "120 cm\n(1200 mm)", color="#334155", fontsize=10, fontweight="bold",
            ha="center", va="center", rotation=90)

    ax.set_xlim(-85, ARENA_W + 85)
    ax.set_ylim(-PAD - 20, ARENA_H + 90)
    ax.set_aspect("equal")
    ax.axis("off")


def plot_single_run_map(run_data, run_num, out_path, color="#1d4ed8", glow_color="#60a5fa"):
    fig, ax = plt.subplots(figsize=(14, 10.5), facecolor="#f8fafc")
    fig.subplots_adjust(left=0.04, right=0.96, top=0.93, bottom=0.03)
    draw_course_arena(ax)

    if run_data:
        x = np.array([r["x"] for r in run_data])
        y = np.array([r["y"] for r in run_data])
        t = np.array([r["ms"] / 1000.0 for r in run_data])
        th = np.array([r["th_rad"] for r in run_data])
        duration = t[-1]

        # Pit badge for this run
        pit_info = [
            f"RUN #{run_num} DETAILS",
            "────────────────────────",
            f"• Duration: {duration:.2f} s",
            f"• Records: {len(run_data)} (30 Hz)",
            f"• Start: ({x[0]:.0f}, {y[0]:.0f}) mm",
            f"• Finish: ({x[-1]:.0f}, {y[-1]:.0f}) mm",
            f"• Status: FINISHED",
            f"• Replans: 2 | Extras: 0"
        ]
        ax.text(CELL + CELL / 2, CELL / 2, "\n".join(pit_info),
                color="#1e293b", fontsize=10.0, fontweight="semibold", family="monospace",
                ha="center", va="center", zorder=4,
                bbox=dict(boxstyle="round,pad=0.6", facecolor="#ffffff", edgecolor="#94a3b8", alpha=0.95))

        # Trajectory
        ax.plot(x, y, color=glow_color, lw=6.5, alpha=0.35, zorder=7)
        ax.plot(x, y, color=color, lw=3.2, alpha=0.95, zorder=8,
                label=f"Run #{run_num} Trajectory ({duration:.2f}s, {len(run_data)} pts)")

        # Start marker
        ax.scatter([x[0]], [y[0]], color="#16a34a", s=180, ec="#ffffff", lw=2.5, zorder=10)
        ax.annotate(f"Start ({x[0]:.0f}, {y[0]:.0f})", (x[0], y[0]), xytext=(x[0] - 110, y[0] - 50),
                    fontsize=9.5, fontweight="bold", color="#15803d",
                    arrowprops=dict(arrowstyle="->", color="#15803d", lw=1.5), zorder=11)

        # Finish marker
        ax.scatter([x[-1]], [y[-1]], color="#dc2626", s=180, ec="#ffffff", lw=2.5, zorder=10)
        ax.annotate(f"Finish ({x[-1]:.0f}, {y[-1]:.0f})\nTime: {duration:.2f}s", (x[-1], y[-1]),
                    xytext=(x[-1] - 80, y[-1] - 60),
                    fontsize=9.5, fontweight="bold", color="#b91c1c",
                    arrowprops=dict(arrowstyle="->", color="#b91c1c", lw=1.5), zorder=11)

        # Time checkpoints every 1 second
        cur_sec = math.ceil(t[0])
        t_max = math.floor(t[-1])
        for s in range(cur_sec, t_max + 1):
            idx = np.argmin(np.abs(t - s))
            px, py, pth = x[idx], y[idx], th[idx]
            ax.scatter([px], [py], color=color, s=45, ec="#ffffff", lw=1.2, zorder=9)
            dx = 30 * np.cos(pth)
            dy = 30 * np.sin(pth)
            ax.arrow(px, py, dx, dy, head_width=18, head_length=15, fc=color, ec=color, zorder=9)
            ax.text(px + 22, py + 16, f"{s}s", fontsize=8.5, fontweight="bold", color="#1e3a8a",
                    bbox=dict(boxstyle="round,pad=0.18", facecolor="#dbeafe", edgecolor="#93c5fd", alpha=0.9),
                    zorder=10)

    ax.legend(loc="lower right", bbox_to_anchor=(0.98, 0.05), frameon=True,
              facecolor="#ffffff", edgecolor="#cbd5e1", fontsize=10, framealpha=0.95)
    plt.suptitle(f"MICROMOUSE OBSTACLE COURSE - RUN #{run_num} (STANDALONE)",
                 fontsize=15, fontweight="bold", color="#0f172a", y=0.97)

    plt.savefig(out_path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved Run #{run_num} map to: {out_path}")


def plot_side_by_side_comparison(run9, run10, out_path):
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(24, 9.5), facecolor="#f8fafc")
    fig.subplots_adjust(left=0.02, right=0.98, top=0.91, bottom=0.03, wspace=0.05)

    for ax, run_data, run_num, col, glow_col in [
        (ax1, run9, 9, "#d97706", "#fde68a"),
        (ax2, run10, 10, "#1d4ed8", "#93c5fd")
    ]:
        draw_course_arena(ax)
        if run_data:
            x = np.array([r["x"] for r in run_data])
            y = np.array([r["y"] for r in run_data])
            t = np.array([r["ms"] / 1000.0 for r in run_data])
            th = np.array([r["th_rad"] for r in run_data])
            dur = t[-1]

            pit_info = [
                f"RUN #{run_num} SUMMARY",
                "─────────────────────",
                f"• Time: {dur:.2f} s",
                f"• Records: {len(run_data)}",
                f"• Final X: {x[-1]:.0f} mm",
                f"• Status: FINISHED"
            ]
            ax.text(CELL + CELL / 2, CELL / 2, "\n".join(pit_info),
                    color="#1e293b", fontsize=9.5, fontweight="semibold", family="monospace",
                    ha="center", va="center", zorder=4,
                    bbox=dict(boxstyle="round,pad=0.5", facecolor="#ffffff", edgecolor="#94a3b8", alpha=0.95))

            ax.plot(x, y, color=glow_col, lw=6.0, alpha=0.35, zorder=7)
            ax.plot(x, y, color=col, lw=3.0, alpha=0.95, zorder=8, label=f"Run #{run_num} ({dur:.2f}s)")
            ax.scatter([x[0]], [y[0]], color="#16a34a", s=140, ec="#fff", lw=2, zorder=10)
            ax.scatter([x[-1]], [y[-1]], color="#dc2626", s=140, ec="#fff", lw=2, zorder=10)

            for s in range(math.ceil(t[0]), math.floor(t[-1]) + 1):
                idx = np.argmin(np.abs(t - s))
                ax.scatter([x[idx]], [y[idx]], color=col, s=35, ec="#fff", lw=1.0, zorder=9)
                ax.text(x[idx] + 20, y[idx] + 14, f"{s}s", fontsize=8, fontweight="bold", color="#1e293b", zorder=10)

            ax.legend(loc="lower right", bbox_to_anchor=(0.98, 0.05), fontsize=9.5)
            ax.set_title(f"Slot {'A' if run_num==9 else 'B'}: Run #{run_num} ({dur:.2f}s)", fontsize=13, fontweight="bold", pad=8)

    plt.suptitle("COMPARISON OF LAST 2 RUNS FROM FLASH MEMORY (RUN #9 vs RUN #10)",
                 fontsize=15, fontweight="bold", color="#0f172a", y=0.97)
    plt.savefig(out_path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved side-by-side comparison to: {out_path}")


if __name__ == "__main__":
    # Run from anywhere; reads and writes next to the repository's data/ folder.
    ws_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "data")

    run9 = load_run(os.path.join(ws_dir, "flash_run_9.csv"))
    run10 = load_run(os.path.join(ws_dir, "flash_run_10.csv"))

    plot_single_run_map(run10, 10, os.path.join(ws_dir, "course_run_10_standalone.png"), color="#1d4ed8", glow_color="#60a5fa")
    plot_single_run_map(run9, 9, os.path.join(ws_dir, "course_run_9_standalone.png"), color="#d97706", glow_color="#fde68a")
    plot_side_by_side_comparison(run9, run10, os.path.join(ws_dir, "course_last_2_runs_comparison.png"))
