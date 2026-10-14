"""Matplotlib rendering of a recorded rollout."""
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Polygon, Rectangle

from .. import config as C
from ..geometry import obb_corners

BG, FLOOR, TAPE = "#1b1e23", "#dcdcd6", "#15171a"


def _draw_arena(ax):
    ax.set_facecolor(BG)
    ax.add_patch(Rectangle((0, -C.PAD), C.ARENA_W, C.ARENA_H + C.PAD, color=FLOOR, zorder=0))
    ax.add_patch(Rectangle((C.CELL, 0), C.CELL, C.CELL, fc="#e8c9c2", ec="none", hatch="///", zorder=1))
    grid = dict(color="#b5b5ae", lw=0.6, zorder=1)
    for x in (C.CELL, 2 * C.CELL):
        ax.plot([x, x], [-C.PAD, C.ARENA_H], **grid)
    ax.plot([0, C.ARENA_W], [C.CELL, C.CELL], **grid)
    tape = dict(color=TAPE, lw=3.2, solid_capstyle="butt", zorder=2)
    ax.plot([0, 0, C.ARENA_W, C.ARENA_W], [0, C.ARENA_H, C.ARENA_H, 0], **tape)
    ax.plot([C.CELL, C.CELL, 2 * C.CELL, 2 * C.CELL], [0, C.CELL, C.CELL, 0], **tape)
    ax.plot([0, C.CELL], [0, 0], color="#1a9c6b", lw=2.5, ls=(0, (3, 2)), zorder=2)
    n = 12
    w = (C.FINISH_X[1] - C.FINISH_X[0]) / n
    for i in range(n):
        ax.add_patch(Rectangle((C.FINISH_X[0] + i * w, -1.2 + (i % 2) * 1.2), w, 1.2, color="k", zorder=2))
    ax.set_xlim(-12, C.ARENA_W + 12)
    ax.set_ylim(-C.PAD - 4, C.ARENA_H + 8)
    ax.set_aspect("equal")
    ax.axis("off")


def animate(frames, path=None, stride=2, fps=15, show=False):
    fig, ax = plt.subplots(figsize=(8.4, 6.0), facecolor=BG)
    _draw_arena(ax)
    n_obs = len(frames[0]["centers"])
    boxes = [Polygon(np.zeros((4, 2)), closed=True, ec="#444", lw=1.2, zorder=3) for _ in range(n_obs)]
    for b in boxes:
        ax.add_patch(b)
    body = Polygon(np.zeros((4, 2)), closed=True, fc="#9fb4e8", ec="#2a3350", lw=1.2, zorder=5)
    wheels = [Polygon(np.zeros((4, 2)), closed=True, fc="#222", zorder=5) for _ in range(2)]
    nose = ax.plot([], [], color="#2a63e8", lw=3, zorder=6)[0]
    rays = [ax.plot([], [], color="#e8a317", lw=1, alpha=0.8, zorder=4)[0] for _ in C.TOF_ANGLES]
    true_path = ax.plot([], [], color="#2a63e8", lw=1.2, alpha=0.8, zorder=3, label="true path")[0]
    dr_path = ax.plot([], [], color="#8a8f99", lw=1.0, ls=":", zorder=3, label="encoder+gyro only")[0]
    odom_path = ax.plot([], [], color="#d94b3d", lw=1.0, ls="--", alpha=0.9, zorder=3, label="map-corrected odom")[0]
    for p in (body, *wheels):
        ax.add_patch(p)
    hud = ax.text(0.01, 0.99, "", transform=ax.transAxes, va="top", color="w", family="monospace", fontsize=10)
    ax.legend(loc="lower center", ncol=3, frameon=False, labelcolor="w", fontsize=8)
    tp, op, dp = [], [], []

    def update(i):
        f = frames[i]
        x, y, th = f["pose"]
        body.set_xy(obb_corners(x, y, th, C.BODY_L / 2, C.BODY_W / 2))
        for side, w in enumerate(wheels):
            off = (C.TRACK / 2) * (1 if side == 0 else -1)
            w.set_xy(obb_corners(x - np.sin(th) * off, y + np.cos(th) * off, th, C.WHEEL_R, C.WHEEL_HALF_W))
        nose.set_data([x + 3 * np.cos(th), x + 6 * np.cos(th)], [y + 3 * np.sin(th), y + 6 * np.sin(th)])
        c, s = np.cos(th), np.sin(th)
        for k, ray in enumerate(rays):
            ox, oy = C.TOF_MOUNT[k] @ np.array([[c, s], [-s, c]]) + [x, y]
            a = th + C.TOF_ANGLES[k]
            ray.set_data([ox, ox + f["tof"][k] * np.cos(a)], [oy, oy + f["tof"][k] * np.sin(a)])
        for j, b in enumerate(boxes):
            b.set_xy(obb_corners(*f["centers"][j], f["angles"][j], 5, 5))
            b.set_facecolor("#e8a317" if f["is_random"][j] else "#9a9a95")
            b.set_edgecolor("#d94b3d" if f["disturbed"][j] else ("#1a9c6b" if f["passed"][j] else "#444"))
        tp.append((x, y))
        op.append(f["odom"][:2])
        dp.append(f["dead_reckoning"][:2])
        dr_path.set_data(*zip(*dp))
        true_path.set_data(*zip(*tp))
        odom_path.set_data(*zip(*op))
        err = np.hypot(f["odom"][0] - x, f["odom"][1] - y)
        hud.set_text(f"t {f['t']:6.1f}s   score {f['score']:6.1f}   passed {int(f['passed'].sum())}/{n_obs}   odom err {err:4.1f} cm")
        return []

    idx = list(range(0, len(frames), stride))
    anim = FuncAnimation(fig, lambda k: update(idx[k]), frames=len(idx), interval=1000 / fps, blit=False)
    if path:
        anim.save(path, writer=PillowWriter(fps=fps))
    if show:
        plt.show()
    plt.close(fig)
