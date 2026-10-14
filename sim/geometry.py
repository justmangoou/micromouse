"""Geometry shared by the course and the maze: oriented boxes, collision, ray casts."""
import numpy as np

from . import config as C


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def obb_corners(cx, cy, th, hl, hw):
    c, s = np.cos(th), np.sin(th)
    local = np.array([[hl, hw], [-hl, hw], [-hl, -hw], [hl, -hw]])
    return local @ np.array([[c, s], [-s, c]]) + [cx, cy]


def sat_mtv(a, b):
    """Translation that pushes rectangle b out of rectangle a, or None if they are disjoint."""
    best, best_axis = np.inf, None
    for poly in (a, b):
        e = poly[1] - poly[0], poly[2] - poly[1]
        for ex, ey in e:
            ax = np.array([-ey, ex]) / np.hypot(ex, ey)
            pa, pb = a @ ax, b @ ax
            overlap = min(pa.max(), pb.max()) - max(pa.min(), pb.min())
            if overlap <= 0:
                return None
            if overlap < best:
                best, best_axis = overlap, ax
    if (b.mean(0) - a.mean(0)) @ best_axis < 0:
        best_axis = -best_axis
    return best_axis * best


def raycast(origins, dirs, centers, angles, half=C.OBSTACLE_SIZE / 2, max_range=C.TOF_MAX):
    """Distance along each ray to the nearest oriented box. origins/dirs (R,2), centers (N,2)."""
    if len(centers) == 0:
        return np.full(len(origins), max_range)
    c, s = np.cos(angles), np.sin(angles)
    rel = origins[:, None, :] - centers[None]
    ox = c * rel[..., 0] + s * rel[..., 1]
    oy = -s * rel[..., 0] + c * rel[..., 1]
    dx = c * dirs[:, None, 0] + s * dirs[:, None, 1]
    dy = -s * dirs[:, None, 0] + c * dirs[:, None, 1]
    dx = np.where(np.abs(dx) < 1e-9, 1e-9, dx)
    dy = np.where(np.abs(dy) < 1e-9, 1e-9, dy)
    tx1, tx2 = (-half - ox) / dx, (half - ox) / dx
    ty1, ty2 = (-half - oy) / dy, (half - oy) / dy
    tmin = np.maximum(np.minimum(tx1, tx2), np.minimum(ty1, ty2))
    tmax = np.minimum(np.maximum(tx1, tx2), np.maximum(ty1, ty2))
    hit = (tmax >= np.maximum(tmin, 0.0))
    t = np.where(hit, np.maximum(tmin, 0.0), np.inf)
    return np.minimum(t.min(1), max_range)


def raycast_rects(origins, dirs, rects, max_range):
    d = np.where(np.abs(dirs) < 1e-9, 1e-9, dirs)
    t1 = (rects[None, :, :2] - origins[:, None, :]) / d[:, None, :]
    t2 = (rects[None, :, 2:] - origins[:, None, :]) / d[:, None, :]
    tmin = np.minimum(t1, t2).max(-1)
    tmax = np.maximum(t1, t2).min(-1)
    t = np.where(tmax >= np.maximum(tmin, 0.0), np.maximum(tmin, 0.0), np.inf)
    return np.minimum(t.min(1), max_range)
