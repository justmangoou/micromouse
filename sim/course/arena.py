"""Arena geometry: track region, obstacle layouts, ray casting, collision, centreline progress."""
from collections import deque
from dataclasses import dataclass

import numpy as np

from .. import config as C


def inside_track(pts, margin=0.0):
    """True where a point lies on the track. `margin` > 0 dilates it (tape), < 0 erodes it. `margin` can be a
    scalar or one value per point (array of shape (N,))."""
    pts = np.atleast_2d(pts)
    margin = np.asarray(margin)
    if margin.ndim > 0:
        margin = margin[:, None]  # broadcast against the (N, n_rects) comparisons below, one margin per point
    r = C.TRACK_RECTS
    ok = (
        (pts[:, None, 0] >= r[None, :, 0] - margin)
        & (pts[:, None, 0] <= r[None, :, 2] + margin)
        & (pts[:, None, 1] >= r[None, :, 1] - margin)
        & (pts[:, None, 1] <= r[None, :, 3] + margin)
    )
    return ok.any(1)


def edge_margin(pts, hi=30.0, iters=10):
    """How many cm of margin each point (assumed on the track) still has before `inside_track` would call it
    off-track -- i.e. its distance to the tape boundary. Found by bisection on `inside_track` rather than by
    deriving the U-shaped polygon's edges by hand (it is a union of 3 rectangles, not one convex shape)."""
    pts = np.atleast_2d(pts)
    lo, hi_arr = np.zeros(len(pts)), np.full(len(pts), hi)
    for _ in range(iters):
        mid = 0.5 * (lo + hi_arr)
        ok = inside_track(pts, -mid)
        lo = np.where(ok, mid, lo)
        hi_arr = np.where(ok, hi_arr, mid)
    return lo


# --- centreline progress -----------------------------------------------------
_P = C.CENTERLINE
_D = np.diff(_P, axis=0)
_LEN = np.hypot(_D[:, 0], _D[:, 1])
_DIR = _D / _LEN[:, None]
_CUM = np.concatenate([[0.0], np.cumsum(_LEN)])
TRACK_LEN = float(_CUM[-1])


def progress(pts):
    """Arc length along the centreline and signed lateral offset (left of travel > 0)."""
    pts = np.atleast_2d(pts)
    rel = pts[:, None, :] - _P[None, :-1, :]
    t = np.clip((rel * _DIR[None]).sum(-1), 0.0, _LEN[None])
    proj = _P[None, :-1, :] + t[..., None] * _DIR[None]
    k = ((pts[:, None, :] - proj) ** 2).sum(-1).argmin(1)
    i = np.arange(len(pts))
    cross = _DIR[k, 0] * rel[i, k, 1] - _DIR[k, 1] * rel[i, k, 0]
    return _CUM[k] + t[i, k], cross


def point_at(s):
    s = np.clip(s, 0.0, TRACK_LEN)
    return np.array([np.interp(s, _CUM, _P[:, 0]), np.interp(s, _CUM, _P[:, 1])])


def tangent_at(s):
    k = min(np.searchsorted(_CUM, np.clip(s, 0.0, TRACK_LEN), side="right") - 1, len(_DIR) - 1)
    return float(np.arctan2(_DIR[k, 1], _DIR[k, 0]))


# --- obstacle layout ------------------------------------------------------------
@dataclass
class Layout:
    centers: np.ndarray  # (N,2)
    angles: np.ndarray  # (N,)
    is_random: np.ndarray  # (N,) bool  (only present in the scored run)


def sample_layout(rng, with_random=True):
    """The official map: 17 fixed blocks, plus (run 2) one random block on 1 of 4 spots."""
    centers = C.STATIC_BLOCKS.copy()
    is_random = np.zeros(len(centers), bool)
    if with_random:
        centers = np.vstack([centers, C.RANDOM_SPOTS[rng.integers(len(C.RANDOM_SPOTS))]])
        is_random = np.append(is_random, True)
    return Layout(centers, np.zeros(len(centers)), is_random)


# --- dividing lines (Dieu 4) ----------------------------------------------------
# Each 60x60 tile is cut into parts by lines painted on the floor; a part counts once the whole car is past its
# last line. Straight tiles (1, 3, 5): 6 parts of 10 cm, lines square to the direction of travel. Corner tiles (2, 4):
# 3 parts, lines fanning out from the inside corner of the U.
def _section_lines():
    """[(point, forward normal, tile)] for every line, in travel order. A point is past a line when
    (p - point) . normal > 0."""
    lines = []
    for k in range(1, 7):  # tile 1, driving +y
        lines.append((np.array([30.0, 10.0 * k]), np.array([0.0, 1.0]), 0))
    for deg in (150, 120, 90):  # tile 2: inside corner (60, 60), sweeping from the +y side to the +x side
        a = np.radians(deg)
        lines.append((np.array([60.0, 60.0]), np.array([np.sin(a), -np.cos(a)]), 1))
    for k in range(1, 7):  # tile 3, driving +x
        lines.append((np.array([60.0 + 10.0 * k, 90.0]), np.array([1.0, 0.0]), 2))
    for deg in (60, 30, 0):  # tile 4: inside corner (120, 60)
        a = np.radians(deg)
        lines.append((np.array([120.0, 60.0]), np.array([np.sin(a), -np.cos(a)]), 3))
    for k in range(1, 7):  # tile 5, driving -y
        lines.append((np.array([150.0, 60.0 - 10.0 * k]), np.array([0.0, -1.0]), 4))
    return lines


SECTION_LINES = _section_lines()
SEC_POINT = np.array([p for p, _, _ in SECTION_LINES])
SEC_NORMAL = np.array([n for _, n, _ in SECTION_LINES])
N_SECTIONS = len(SECTION_LINES)


def block_section(centers):
    """Index of the part (= the line that must be crossed) of each block, from its centre."""
    out = np.zeros(len(centers), int)
    for i, (x, y) in enumerate(centers):
        if x < 60:
            if y < 60:
                out[i] = min(int(y // 10), 5)
            else:
                ang = np.degrees(np.arctan2(y - 60.0, x - 60.0))  # 90..180, first part is the one nearest tile 1
                out[i] = 6 + min(int((180.0 - ang) // 30), 2)
        elif x < 120:
            out[i] = 9 + min(int((x - 60.0) // 10), 5)
        elif y >= 60:
            ang = np.degrees(np.arctan2(y - 60.0, x - 120.0))  # 0..90, first part is the one nearest tile 3
            out[i] = 15 + min(int((90.0 - ang) // 30), 2)
        else:
            out[i] = 18 + min(int((60.0 - y) // 10), 5)
    return out


def section_progress():
    """Centreline arc length at which each line is crossed (for placing the car just after a line). The half-plane
    test is only meaningful once the previous line is behind the car, so each search starts at the previous result."""
    ss = np.arange(0.0, TRACK_LEN, 0.5)
    pts = np.stack([point_at(s) for s in ss])
    out, lo = [], 0
    for p, n in zip(SEC_POINT, SEC_NORMAL):
        lo += int(((pts[lo:] - p) @ n >= 0).argmax())
        out.append(ss[lo])
    return np.array(out)
