"""Top-view picture of the ToF layout in sim/config.py.

    python -m sim.tools.draw_tof_layout [docs/media/tof_layout.png]
"""
import sys
import numpy as np, matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Wedge, Circle
from .. import config as C

fig, ax = plt.subplots(figsize=(9, 6.2))
L, W = C.BODY_L, C.BODY_W
ax.add_patch(Rectangle((-L / 2, -W / 2), L, W, fc="#dfe6f7", ec="#2a3350", lw=2, zorder=3))
for sgn in (1, -1):  # wheels (left wheel at +y)
    ax.add_patch(Rectangle((-C.WHEEL_R, sgn * C.TRACK / 2 - C.WHEEL_HALF_W), 2 * C.WHEEL_R, 2 * C.WHEEL_HALF_W, fc="#222", zorder=4))
ax.annotate("", xy=(L / 2 + 1.5, 0), xytext=(0, 0), arrowprops=dict(arrowstyle="->", color="#2a63e8", lw=2), zorder=6)
ax.text(L / 2 + 1.8, -0.9, "front", color="#2a63e8", fontsize=11, ha="left")
ax.plot([0], [0], "k+", ms=10, zorder=6)

names = ["right edge", "right middle", "left middle", "left edge"]
cols = ["#d94b3d", "#e8a317", "#e8a317", "#d94b3d"]
R = 30.0
for k in range(4):
    mx, my = C.TOF_MOUNT[k]
    a = C.TOF_ANGLES[k]
    ax.add_patch(Wedge((mx, my), R, np.degrees(a) - 12.5, np.degrees(a) + 12.5, fc=cols[k], alpha=0.22, ec=cols[k], lw=0.8, zorder=1))
    ax.plot([mx, mx + R * np.cos(a)], [my, my + R * np.sin(a)], color=cols[k], lw=1.4, ls="--", zorder=2)
    ax.add_patch(Circle((mx, my), 0.55, fc=cols[k], ec="k", zorder=7))
    ang = np.degrees(a)
    ax.text(mx + (R + 5.0) * np.cos(a), my + (R + 5.0) * np.sin(a), f"{names[k]}\n{ang:+.0f}°", color=cols[k], fontsize=10, ha="center", va="center", fontweight="bold")
# angle arcs
for a in np.radians([60, 20, -20, -60]):
    pass
ax.plot([6, 6 + 14], [4.5, 4.5], color="#888", lw=0.8, ls=":")
ax.plot([6, 6 + 14], [-4.5, -4.5], color="#888", lw=0.8, ls=":")
ax.plot([6, 6 + 14], [2, 2], color="#888", lw=0.8, ls=":")
ax.plot([6, 6 + 14], [-2, -2], color="#888", lw=0.8, ls=":")
fig.text(0.5, 0.02, f"body {L:g} x {W:g} cm, sensors at x = {C.TOF_MOUNT[0,0]:g} cm forward; edge pair y = ±{C.TOF_MOUNT[0,1]:g} cm, middle pair y = ±{C.TOF_MOUNT[1,1]:g} cm (estimated)\ncones drawn 25° wide (VL53L0X), 30 cm long", ha="center", fontsize=9, color="#444")
ax.set_aspect("equal"); ax.set_xlim(-10, 48); ax.set_ylim(-44, 44)
ax.set_title("ToF layout: " + ", ".join(f"{np.degrees(a):+.0f}°" for a in C.TOF_ANGLES) + " (top view)", fontsize=13)
ax.axis("off")
fig.savefig(sys.argv[1] if len(sys.argv) > 1 else "tof_layout.png", dpi=110, bbox_inches="tight")
