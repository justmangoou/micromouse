"""Global A* on the run-1 map, pure-pursuit tracking, replanning around blocks the map does not contain.

The driver only uses what the real car has: its own pose estimate, the 4 ToF readings, the stored
static map and its own wheel speed. No ground truth.
"""
import heapq

import numpy as np

from .. import config as C
from ..geometry import raycast
from .arena import edge_margin, inside_track

RES = 2.0  # planning grid, cm
HALF = C.OBSTACLE_SIZE / 2
R_ROBOT = 7.0  # min clearance from a block surface for the planned centre line
R_PRUNE = 9.0  # shortcuts must keep this much clearance
# Stay this far inside the tape. A wheel sits TRACK/2 + wheel_half_w = 5.0 cm off the centreline, so this must clear
# that plus *real* margin for dead-reckoning drift, not just a few mm. But the 60 cm-wide corridor (with R_ROBOT
# clearance from obstacles also eating into it) cannot support a static margin that covers the full drift budget --
# above ~8 cm, A* starts failing to find a route at all. So R_EDGE only buys back the wheel geometry plus a modest
# cushion; EDGE_SLOW (below) is what actually keeps the car off the tape when it has drifted further than that, by
# slowing it down near the boundary so pure pursuit has more time to correct.
R_EDGE = 7.0
R_EDGE_MIN = 5.5  # last resort when no route exists at R_EDGE: wheel extent (5.0 cm) + 0.5 cm, zero room for drift
EDGE_SLOW_START = 22.0  # cm of edge margin at which the speed factor starts dropping below 1.0
DISC_R = 11.0  # an unmapped block is modelled as a disc around its estimated centre. Bigger than the block's own
# 5 cm half-width: the single-ray near-face + half-block-behind centre estimate is biased for oblique hits (seen
# several cm off for a glancing angle), so this buys back margin for that on top of the true geometry
GOAL = np.array([150.0, -12.0])
V_FULL = C.V_FULL  # cm/s at full duty, mid-life battery (also the firmware's target-speed normalisation)
V_MIN = 0.22 * V_FULL

XS = np.arange(0.0, C.ARENA_W + RES / 2, RES)
YS = np.arange(-C.PAD, C.ARENA_H + RES / 2, RES)
_GX, _GY = np.meshgrid(XS, YS)
CELL_PTS = np.stack([_GX.ravel(), _GY.ravel()], 1)
EDGE_DIST = edge_margin(CELL_PTS, iters=8)  # distance from each grid point to the tape (0 off-track)
TRACK_OK = {e: inside_track(CELL_PTS, -e) for e in (R_EDGE, R_EDGE_MIN)}
# (obstacle clearance, edge margin) rungs, tried in order. Relax the obstacle squeeze first, the tape margin last.
RUNGS = ((R_ROBOT, R_EDGE), (R_ROBOT - 1.0, R_EDGE), (4.5, R_EDGE), (4.5, R_EDGE_MIN))


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def block_clearance(pts, centers, angles):
    """Distance from points to the surface of oriented square blocks (0 inside)."""
    if len(centers) == 0:
        return np.full(len(pts), 1e3)
    c, s = np.cos(angles), np.sin(angles)
    rel = pts[:, None, :] - centers[None]
    lx = c * rel[..., 0] + s * rel[..., 1]
    ly = -s * rel[..., 0] + c * rel[..., 1]
    return np.hypot(np.maximum(np.abs(lx) - HALF, 0), np.maximum(np.abs(ly) - HALF, 0)).min(1)


class AStarDriver:
    def __init__(self, speed=0.85, lookahead=10.0, centering=6.0, caution=0.6, alert_speed=0.5):
        self.speed, self.lookahead, self.centering = speed, lookahead, centering
        self.caution = caution  # speed factor where the unmapped random blocks can be (right-hand cells)
        self.alert_speed = alert_speed  # speed factor for a moment after a new unmapped block shows up

    # --- setup ------------------------------------------------------------
    def reset(self, env):
        self.env = env
        self.map_c, self.map_a = env.loc.map_centers, env.loc.map_angles
        self.static_clear = block_clearance(CELL_PTS, self.map_c, self.map_a)
        self.spot_clear = block_clearance(CELL_PTS, C.RANDOM_SPOTS, np.zeros(len(C.RANDOM_SPOTS)))
        self.extra = []  # [centre, n_hits] of blocks the ToF saw but the map does not have
        self.streak = np.zeros(len(C.TOF_ANGLES), int)
        self.n_repos = env.n_repos
        self.path = None
        self.replans = 0
        self.edge_used = R_EDGE
        self.no_route = False
        self.need_replan = False
        self.alert_until = -1.0
        self._plan(np.array(env.est[:2]))

    # --- clearance ------------------------------------------------------------
    def _clearance(self, pts):
        clr = block_clearance(pts, self.map_c, self.map_a)
        for c, _ in self.extra:
            clr = np.minimum(clr, np.hypot(pts[:, 0] - c[0], pts[:, 1] - c[1]) - DISC_R)
        return clr

    def _segment_free(self, a, b, rad):
        n = max(int(np.hypot(*(b - a))), 2)
        pts = a + (b - a) * np.linspace(0, 1, n + 1)[:, None]
        return bool((self._clearance(pts) >= rad).all() and inside_track(pts, -self.edge_used).all())

    # --- planning ------------------------------------------------------------
    def _astar(self, start, radius, edge):
        clear = self.static_clear.copy()
        for c, _ in self.extra:
            clear = np.minimum(clear, np.hypot(CELL_PTS[:, 0] - c[0], CELL_PTS[:, 1] - c[1]) - DISC_R)
        free = ((clear >= radius) & TRACK_OK[edge]).reshape(len(YS), len(XS))
        # prefer the middle of a gap: crowding a block or the tape both cost extra
        # (the 4 spots a random block can appear on are soft obstacles too: avoid them when there is room)
        room = np.minimum(np.minimum(clear, EDGE_DIST + (R_ROBOT - R_EDGE)), self.spot_clear)
        mult = (1.0 + self.centering * np.clip((16.0 - room) / 16.0, 0, 1) ** 2).reshape(free.shape)
        free_l, mult_l = free.tolist(), mult.tolist()
        ny, nx = free.shape
        si = (int(np.clip(round((start[1] + C.PAD) / RES), 0, ny - 1)), int(np.clip(round(start[0] / RES), 0, nx - 1)))

        def is_goal(r, c):
            return YS[r] <= GOAL[1] and 135.0 <= XS[c] <= 165.0

        def h(r, c):
            dx = max(135.0 - XS[c], 0.0, XS[c] - 165.0)
            return float(np.hypot(dx, max(YS[r] - GOAL[1], 0.0)))

        steps = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
        g = {si: 0.0}
        came = {}
        heap = [(h(*si), 0.0, si)]
        while heap:
            _, gc, cur = heapq.heappop(heap)
            if gc > g.get(cur, np.inf):
                continue
            if is_goal(*cur) and free_l[cur[0]][cur[1]]:
                path = [cur]
                while path[-1] in came:
                    path.append(came[path[-1]])
                return np.array([[XS[c], YS[r]] for r, c in path[::-1]])
            for dr, dc in steps:
                r, c = cur[0] + dr, cur[1] + dc
                if not (0 <= r < ny and 0 <= c < nx) or not free_l[r][c]:
                    continue
                ng = gc + (1.4142 if dr and dc else 1.0) * RES * 0.5 * (mult_l[r][c] + mult_l[cur[0]][cur[1]])
                if ng < g.get((r, c), np.inf):
                    g[(r, c)] = ng
                    came[(r, c)] = cur
                    heapq.heappush(heap, (ng + h(r, c), ng, (r, c)))
        return None

    def _plan(self, start):
        coarse = None
        for radius, edge in RUNGS:  # squeeze through if the preferred margins do not exist
            coarse = self._astar(start, radius, edge)
            if coarse is not None:
                self.edge_used = edge
                break
        self.last_plan_t = self.env.t
        self.need_replan = False
        if coarse is None:
            if self.path is not None:
                return  # keep following the last good path
            # No route even at the loosest margins. A straight line to the goal would run off the track (the middle of
            # the U is not floor), so say so loudly instead of silently driving into the tape forever.
            self.edge_used = R_EDGE_MIN
            self.no_route = True
            coarse = np.array([start, GOAL])
        self.replans += 1
        pts = np.vstack([start, coarse[1:]])
        # string pulling: drop waypoints while the straight shortcut keeps clearance
        keep, i = [pts[0]], 0
        while i < len(pts) - 1:
            j = len(pts) - 1
            while j > i + 1 and not self._segment_free(pts[i], pts[j], R_PRUNE):
                j -= 1
            keep.append(pts[j])
            i = j
        pts = np.array(keep)
        for _ in range(2):  # Chaikin corner cutting, kept only if it stays clear
            q = 0.75 * pts[:-1] + 0.25 * pts[1:]
            r = 0.25 * pts[:-1] + 0.75 * pts[1:]
            sm = np.vstack([pts[:1], np.column_stack([q, r]).reshape(-1, 2), pts[-1:]])
            if (self._clearance(sm) >= R_ROBOT - 0.5).all():
                pts = sm
        d = pts[-1] - pts[-2] if len(pts) > 1 else np.array([0.0, -1.0])
        if np.hypot(*d) < 1e-3:  # replanning from inside the goal cell collapses the route to one point: no direction
            d = np.array([0.0, -1.0])  # to extrapolate, so just run straight down through the finish line
        d = d / np.hypot(*d)
        pts = np.vstack([pts, pts[-1] + d * 15.0, pts[-1] + d * 30.0])  # run through the finish line
        seg = np.hypot(*np.diff(pts, axis=0).T)
        s = np.concatenate([[0.0], np.cumsum(seg)])
        grid = np.arange(0.0, s[-1], 1.5)
        self.path = np.column_stack([np.interp(grid, s, pts[:, 0]), np.interp(grid, s, pts[:, 1])])
        self.path_s = grid
        heading = np.unwrap(np.arctan2(*np.diff(self.path, axis=0).T[::-1]))
        heading = np.concatenate([heading, heading[-1:]])
        n, k = len(heading), 5
        turn = np.abs(heading[np.minimum(np.arange(n) + k, n - 1)] - heading[np.maximum(np.arange(n) - k, 0)])
        clr = self._clearance(self.path)
        edge_clr = edge_margin(self.path)
        v_turn = np.clip(1.0 / (1.0 + 2.0 * turn), 0.35, 1.0)
        v_block = np.clip(0.6 + 0.4 * (clr - R_ROBOT) / 6.0, 0.6, 1.0)
        # slow well before the tape, not just where R_EDGE already forces the centreline away from it: dead-reckoning
        # drift (a few cm, more on a longer run) is what actually pushes the car over the line, not tracking lag, so
        # giving pure pursuit a slower, longer look near the boundary is what keeps the wheels inside it.
        v_edge = np.clip(0.5 + 0.5 * (edge_clr - self.edge_used) / EDGE_SLOW_START, 0.5, 1.0)
        self.path_v = v_turn * v_block * v_edge
        self.idx = 0

    # --- unmapped blocks ----------------------------------------------------
    def _scan(self):
        env = self.env
        x, y, th = env.est
        c, s = np.cos(th), np.sin(th)
        origins = C.TOF_MOUNT @ np.array([[c, s], [-s, c]]) + [x, y]
        ang = (th + C.TOF_ANGLES)[:, None] + C.TOF_FOV[None]
        dirs = np.stack([np.cos(ang), np.sin(ang)], -1).reshape(-1, 2)
        exp = raycast(np.repeat(origins, len(C.TOF_FOV), 0), dirs, self.map_c, self.map_a).reshape(len(C.TOF_ANGLES), -1).min(1)
        z = env.tof
        unexplained = (z < exp - (4.0 + 0.08 * exp)) & (z < 80.0)
        self.streak = np.where(unexplained, self.streak + 1, 0)
        for k in np.flatnonzero(self.streak >= 2):
            a = th + C.TOF_ANGLES[k]
            d = np.array([np.cos(a), np.sin(a)])
            hit = origins[k] + z[k] * d
            if block_clearance(hit[None], self.map_c, self.map_a)[0] < 5.0:
                continue  # a mapped block seen through a small pose error, not a new one
            centre = origins[k] + (z[k] + HALF) * d  # sensor sees the near face; the block centre is half a block behind
            for e in self.extra:
                if np.hypot(*(e[0] - centre)) < 18.0:  # the near-face+half-block-behind centre estimate
                # drifts a lot with viewing angle (seen up to ~19 cm between two detections of the same block), so
                # this has to be generous or MAX_EXTRA fills with ghost duplicates of the same 1-2 real blocks
                    e[1] += 1
                    if np.hypot(*(e[0] - centre)) > 2.0 and e[1] < 6:
                        e[0] = 0.7 * e[0] + 0.3 * centre
                        self.need_replan = True
                    break
            else:
                self.extra.append([centre, 1])
                self.need_replan = True
                self.alert_until = env.t + 0.8

    # --- control ------------------------------------------------------------
    def __call__(self, obs):
        env = self.env
        if env.n_repos != self.n_repos:  # the car was lifted and put back: plan from where it is now
            self.n_repos = env.n_repos
            self._plan(np.array(env.est[:2]))
        self._scan()
        pos = np.array(env.est[:2])
        if self.need_replan and env.t - self.last_plan_t > 0.2:
            self._plan(pos)
        lo = self.idx
        hi = min(lo + 80, len(self.path))
        self.idx = lo + int(np.hypot(*(self.path[lo:hi] - pos).T).argmin())
        if np.hypot(*(self.path[self.idx] - pos)) > 25.0 and env.t - self.last_plan_t > 0.5:
            self._plan(pos)
        x, y, th = env.est
        look = self.lookahead + 0.15 * abs(env.odom.v)
        j = min(int(np.searchsorted(self.path_s, self.path_s[self.idx] + look)), len(self.path) - 1)
        tgt = self.path[j]
        dist = max(float(np.hypot(*(tgt - pos))), 1.0)
        alpha = wrap(np.arctan2(tgt[1] - y, tgt[0] - x) - th)
        if abs(alpha) > 1.2:  # facing the wrong way: pivot first
            return np.array([-0.45, 0.45]) * np.sign(alpha)
        kappa = 2.0 * np.sin(alpha) / dist
        v = self.speed * V_FULL * self.path_v[self.idx] * max(0.35, np.cos(alpha))
        if x > 105.0:
            v *= self.caution
        if env.t < self.alert_until:
            v *= self.alert_speed
        v = max(v, V_MIN)  # the slowdown factors multiply; keep the motors out of the deadband
        vl, vr = v * (1 - kappa * C.TRACK / 2), v * (1 + kappa * C.TRACK / 2)
        u = np.array([vl, vr]) / V_FULL
        return u / max(1.0, np.abs(u).max())
