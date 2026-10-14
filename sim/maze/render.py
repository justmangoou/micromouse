"""Matplotlib rendering of a maze run."""
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.collections import PatchCollection
from matplotlib.patches import Polygon, Rectangle

from .. import config as C
from ..geometry import obb_corners
from . import world as M
from .world import CELL, GOAL, MAZE_BODY_L, MAZE_TOF_MOUNT, N

BG, FLOOR, WALL = "#1b1e23", "#dcdcd6", "#2b2f36"
PHASE_COLOR = {"search to centre": "#2a63e8", "search back to start": "#8a63d9", "speed run": "#d94b3d", "done": "#d94b3d"}


def animate_maze(env, frames, path, stride=8, fps=25):
    fig, ax = plt.subplots(figsize=(7, 7.4), facecolor=BG)
    ax.set_facecolor(BG)
    ax.add_patch(Rectangle((0, 0), N * CELL, N * CELL, color=FLOOR, zorder=0))
    for gx, gy in GOAL:
        ax.add_patch(Rectangle((gx * CELL, gy * CELL), CELL, CELL, color="#e8c9c2", zorder=1))
    ax.add_patch(Rectangle((0, 0), CELL, CELL, color="#bfe3cf", zorder=1))
    ax.add_collection(PatchCollection([Rectangle(r[:2], r[2] - r[0], r[3] - r[1]) for r in env.rects], color=WALL, zorder=3))
    body = Polygon(np.zeros((4, 2)), closed=True, fc="#9fb4e8", ec="#2a3350", lw=1, zorder=6)
    nose = ax.plot([], [], color="#2a63e8", lw=2.5, zorder=7)[0]
    rays = [ax.plot([], [], color="#e8a317", lw=1, alpha=0.9, zorder=5)[0] for _ in M.MAZE_TOF_ANGLES]
    ax.add_patch(body)
    trail = {p: ax.plot([], [], color=c, lw=1.2, alpha=0.9, zorder=4)[0] for p, c in PHASE_COLOR.items()}
    pts = {p: [] for p in PHASE_COLOR}
    hud = ax.text(0.01, 1.01, "", transform=ax.transAxes, va="bottom", color="w", family="monospace", fontsize=10)
    ax.set_xlim(-3, N * CELL + 3)
    ax.set_ylim(-3, N * CELL + 3)
    ax.set_aspect("equal")
    ax.axis("off")

    def update(i):
        f = frames[i]
        x, y, th = f["pose"]
        body.set_xy(obb_corners(x, y, th, MAZE_BODY_L / 2, C.BODY_W / 2))
        nose.set_data([x + 3 * np.cos(th), x + 6 * np.cos(th)], [y + 3 * np.sin(th), y + 6 * np.sin(th)])
        c, s = np.cos(th), np.sin(th)
        for k, ray in enumerate(rays):
            ox, oy = MAZE_TOF_MOUNT[k] @ np.array([[c, s], [-s, c]]) + [x, y]
            a = th + M.MAZE_TOF_ANGLES[k]
            ray.set_data([ox, ox + f["tof"][k] * np.cos(a)], [oy, oy + f["tof"][k] * np.sin(a)])
        pts[f["phase"]].append((x, y))
        trail[f["phase"]].set_data(*zip(*pts[f["phase"]]))
        hud.set_text(f"t {f['t']:6.1f}s   {f['phase']}")
        return []

    idx = list(range(0, len(frames), stride))
    FuncAnimation(fig, lambda k: update(idx[k]), frames=len(idx), interval=1000 / fps, blit=False).save(path, writer=PillowWriter(fps=fps))
    plt.close(fig)
