"""Matplotlib rendering of a recorded rollout."""
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Polygon, Rectangle

from .. import config as C
from ..geometry import obb_corners

BG, FLOOR, TAPE, INK = "#f4f4f1", "#f1f1ee", "#15171a", "#2b2e33"


def _draw_arena(ax):
    ax.set_facecolor(BG)
    ax.add_patch(Rectangle((0, -C.PAD), C.ARENA_W, C.ARENA_H + C.PAD, color=FLOOR, zorder=0))
    ax.add_patch(Rectangle((C.CELL, 0), C.CELL, C.CELL, fc="#f4efec", ec="#e6c9c2", lw=0, hatch="///", zorder=1))
    plt.rcParams["hatch.linewidth"] = 0.8
    for x in np.arange(0, C.ARENA_W + 1, 10):  # 10 cm grid, heavier every 60 cm cell
        ax.plot([x, x], [-C.PAD, C.ARENA_H], color="#a9a9a2" if x % C.CELL == 0 else "#d3d3cd",
                lw=0.9 if x % C.CELL == 0 else 0.5, zorder=1)
    for y in np.arange(-C.PAD + 10, C.ARENA_H + 1, 10):
        ax.plot([0, C.ARENA_W], [y, y], color="#a9a9a2" if y % C.CELL == 0 else "#d3d3cd",
                lw=0.9 if y % C.CELL == 0 else 0.5, zorder=1)
    tape = dict(color=TAPE, lw=3.6, solid_capstyle="butt", zorder=2)
    ax.plot([0, 0, C.ARENA_W, C.ARENA_W], [0, C.ARENA_H, C.ARENA_H, 0], **tape)
    ax.plot([C.CELL, C.CELL, 2 * C.CELL, 2 * C.CELL], [0, C.CELL, C.CELL, 0], **tape)
    ax.plot([0, C.CELL], [0, 0], color="#1a9c6b", lw=2.8, ls=(0, (3, 2)), zorder=2)
    n = 24
    w = (C.FINISH_X[1] - C.FINISH_X[0]) / n
    for i in range(n):
        ax.add_patch(Rectangle((C.FINISH_X[0] + i * w, -1.4 + (i % 2) * 1.4), w, 1.4, color="k", zorder=2))
    lab = dict(color="#8a8d92", fontsize=9, fontweight="bold", zorder=2, ha="center", va="center")
    for name, x, y in (("Ô1", 6, 4), ("Ô2", 6, 62), ("Ô3", 66, 114), ("Ô4", 124, 114), ("Ô5", 126, 4)):
        ax.text(x, y, name, **lab)
    ax.text(30, -C.PAD / 2 - 2, "XUẤT PHÁT", color=INK, fontsize=11, fontweight="bold", ha="center", va="center")
    ax.text(150, -C.PAD / 2 - 2, "ĐÍCH", color=INK, fontsize=11, fontweight="bold", ha="center", va="center")
    tk = dict(color="#7b7e83", fontsize=6.5, ha="center", va="bottom")
    for x in range(0, int(C.ARENA_W) + 1, 20):
        ax.text(x, C.ARENA_H + 2, str(x), **tk)
    for y in range(0, int(C.ARENA_H) + 1, 20):
        ax.text(-2.5, y, str(y), color="#7b7e83", fontsize=6.5, ha="right", va="center")
    ax.set_xlim(-12, C.ARENA_W + 8)
    ax.set_ylim(-C.PAD - 4, C.ARENA_H + 16)
    ax.set_aspect("equal")
    ax.axis("off")


def animate(frames, path=None, stride=2, fps=15, show=False):
    fig, ax = plt.subplots(figsize=(8.4, 6.0), facecolor=BG)
    fig.subplots_adjust(0.01, 0.01, 0.99, 0.99)
    _draw_arena(ax)
    n_obs = len(frames[0]["centers"])
    boxes = [Polygon(np.zeros((4, 2)), closed=True, ec="#444", lw=1.6, joinstyle="round", zorder=3) for _ in range(n_obs)]
    for j, b in enumerate(boxes):
        ax.add_patch(b)
    tags = [ax.text(0, 0, "R" if frames[0]["is_random"][j] else f"T{j + 1}", fontsize=6.5, fontweight="bold",
                    color=INK, ha="center", va="center", zorder=4) for j in range(n_obs)]
    body = Polygon(np.zeros((4, 2)), closed=True, fc="#9fb4e8", ec="#2a3350", lw=1.2, zorder=5)
    wheels = [Polygon(np.zeros((4, 2)), closed=True, fc="#222", zorder=5) for _ in range(2)]
    nose = ax.plot([], [], color="#2a63e8", lw=3, zorder=6)[0]
    rays = [ax.plot([], [], color="#e8a317", lw=1, alpha=0.8, zorder=4)[0] for _ in C.TOF_ANGLES]
    true_path = ax.plot([], [], color="#2a63e8", lw=1.6, alpha=0.9, zorder=3, label="true path")[0]
    dr_path = ax.plot([], [], color="#8a8f99", lw=1.0, ls=":", zorder=3, label="encoder+gyro only")[0]
    odom_path = ax.plot([], [], color="#d94b3d", lw=1.0, ls="--", alpha=0.9, zorder=3, label="map-corrected odom")[0]
    for p in (body, *wheels):
        ax.add_patch(p)
    hud = ax.text(0.01, 0.99, "", transform=ax.transAxes, va="top", color=INK, family="monospace", fontsize=10)
    ax.legend(loc="lower center", ncol=3, frameon=False, labelcolor=INK, fontsize=8)
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
            b.set_facecolor("#f0b84a" if f["is_random"][j] else "#cfd1d0")
            tags[j].set_position(f["centers"][j])
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
