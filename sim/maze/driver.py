"""Flood-fill driver for the maze: learns walls from the 4 ToF sensors while it searches, then speed-runs the shortest
path. Layout-dependent thresholds are derived by `set_layout` (called at import for the default layout)."""
from collections import deque

import numpy as np

from .. import config as C
from ..geometry import raycast_rects, wrap
from . import world
from .world import CELL, DIRS, GOAL, HEADING, MAZE_TOF_MOUNT, N, START, WALL_T

R_EDGE, R_MID_R, L_MID, L_EDGE = 0, 1, 2, 3  # indices into the 4 readings

# Layout-dependent (set_layout fills these in): edge sensor reading when centred, its reading -> lateral distance factor,
# wall / no-wall threshold for it, the front-wall threshold for the middle pair, and the angle those use for ranging.
WALL_SIDE = 8.0  # edge sensor reads less than this: side wall present
WALL_FRONT = 8.5  # BOTH middle sensors read less than this: front wall present (one alone is a side wall)
SIDE_REF = 3.9  # edge sensor reading when centred
EDGE_SIN = 1.0  # lateral wall distance = edge reading * EDGE_SIN
MID_RANGE_ANGLE = np.radians(18.0)  # a front wall is ranged by the nearest ray of the middle cone
USE_EDGE_FIX = True
SENSE_AT = 16.0  # cm travelled since the last centre at which the cell ahead is read
KP_LAT = 0.05  # rad of heading offset per cm of lateral error
EDGE_END_PAST, EDGE_START_PAST = 4.6, 0.6  # (set_layout can replace them)  # cm the edge sensor is past the boundary when a wall end / start is seen
YAW_GAIN = 0.08
BIAS_GAIN = 0.01  # rad/s of bias correction per rad of heading error, per step
FRONT_AT_CENTRE = CELL / 2 - WALL_T / 2 - MAZE_TOF_MOUNT[3, 0]  # sensor-to-front-wall distance at the cell centre: 3.9 cm
V_APPROACH = 12.0  # search runs slow down for the last 6 cm of each cell, where the front wall is read
V_SEARCH, V_FAST, ACCEL = 22.0, 32.0, 350.0  # cm/s, cm/s, cm/s^2 (braking profile)


def set_layout(edge_deg, mid_deg, end_past=None, start_past=None):
    """Sensor layout: edge pair at +-edge_deg, middle pair at +-mid_deg from the heading. Derives the wall thresholds from
    ray casts in an ideal corridor, so the driver needs no hand tuning per layout. `end_past` / `start_past` are the
    wall-edge position fixes (see EDGE_END_PAST); call calibrate_events() for a new layout and pass the results."""
    global WALL_SIDE, WALL_FRONT, SIDE_REF, EDGE_SIN, MID_RANGE_ANGLE, EDGE_END_PAST, EDGE_START_PAST
    angles = np.radians([-edge_deg, -mid_deg, mid_deg, edge_deg])
    world.MAZE_TOF_ANGLES = angles  # the maze car reads these when it is reset
    h = WALL_T / 2
    rects = np.array([[-300, -h, 300, h], [-300, CELL - h, 300, CELL + h]])  # corridor along +x, centre y = 9
    origins = MAZE_TOF_MOUNT + [0.0, CELL / 2]
    ang = angles[:, None] + C.TOF_FOV[None]
    dirs = np.stack([np.cos(ang), np.sin(ang)], -1).reshape(-1, 2)
    ref = raycast_rects(np.repeat(origins, len(C.TOF_FOV), 0), dirs, rects, C.TOF_MAX).reshape(4, -1).min(1)
    SIDE_REF = float(ref[3])
    EDGE_SIN = float((CELL / 2 - h - MAZE_TOF_MOUNT[3, 1]) / SIDE_REF)  # lateral distance / reading
    WALL_SIDE = SIDE_REF + 4.0
    MID_RANGE_ANGLE = float(max(np.radians(mid_deg - 12.0), 0.0))
    # front wall one 2 cm before the centre, seen by the nearest ray of the middle cone; threshold 1.4x that
    WALL_FRONT = 1.4 * (FRONT_AT_CENTRE + 2.0) / np.cos(MID_RANGE_ANGLE)
    if end_past is not None:
        EDGE_END_PAST, EDGE_START_PAST = end_past, start_past


set_layout(60, 20, 1.78, -4.64)  # default layout; the two numbers are the wall-edge position fixes measured for it


class FloodFillDriver:
    """Knows only the outer wall and the start cell's walls; learns the rest from the 4 ToF while driving."""

    def __init__(self, max_rounds=4):
        self.kv, self.kh = np.zeros((N + 1, N), bool), np.zeros((N, N + 1), bool)
        self.kv[0, :] = self.kv[N, :] = self.kh[:, 0] = self.kh[:, N] = True
        self.kv[1, 0] = True
        self.visited = np.zeros((N, N), bool)
        self.cell, self.d, self.s = START, 0, 0.0
        self.targets, self.phase, self.vmax = GOAL, "search to centre", V_SEARCH
        self.state, self.rounds, self.max_rounds = "SENSE0", 0, max_rounds
        self.next_d, self.stop, self.decided = 0, False, False
        self.turn_ok = 0
        self.travel, self.trk = 0.0, deque(maxlen=20)  # distance driven and edge readings, for the wall-yaw estimate
        self.edge_prev = None
        self.hist = []  # last 3 readings: wall tests use the median, so one dropped reading cannot hide a wall
        self.log = []  # (phase, t) at every phase change

    # known walls
    def wall(self, cell, d):
        i, j = cell
        return (self.kh[i, j + 1], self.kv[i + 1, j], self.kh[i, j], self.kv[i, j])[d]

    def _set(self, cell, d, v):
        i, j = cell
        if d == 0:
            self.kh[i, j + 1] = v
        elif d == 1:
            self.kv[i + 1, j] = v
        elif d == 2:
            self.kh[i, j] = v
        else:
            self.kv[i, j] = v

    def flood(self, targets):
        dist = np.full((N, N), 10**6, int)
        q = list(targets)
        for t in q:
            dist[t] = 0
        for c in q:
            for d, (di, dj) in enumerate(DIRS):
                n = (c[0] + di, c[1] + dj)
                if 0 <= n[0] < N and 0 <= n[1] < N and not self.wall(c, d) and dist[n] > dist[c] + 1:
                    dist[n] = dist[c] + 1
                    q.append(n)
        return dist

    def path_is_known(self):
        dist, c, d = self.flood(GOAL), START, 0
        while c not in GOAL:
            if not self.visited[c]:
                return False
            best = min((k for k in range(4) if not self.wall(c, k)), key=lambda k: (dist[c[0] + DIRS[k][0], c[1] + DIRS[k][1]], k != d))
            c, d = (c[0] + DIRS[best][0], c[1] + DIRS[best][1]), best
        return True

    def _sense(self, c, d, tof):
        # a reading of ~120 cm cannot be a real open side or front in a maze (the next wall is always closer): it is a
        # dropped reading, so keep what is already believed rather than erase a wall
        if tof[L_EDGE] < 100:
            self._set(c, (d + 3) % 4, tof[L_EDGE] < WALL_SIDE)
        if tof[R_EDGE] < 100:
            self._set(c, (d + 1) % 4, tof[R_EDGE] < WALL_SIDE)
        if self._front_far(tof) < 100:
            self._set(c, d, self._front_far(tof) < WALL_FRONT)
        self.kv[0, :] = self.kv[N, :] = self.kh[:, 0] = self.kh[:, N] = True  # the boundary is known, never erase it
        self.kv[1, 0] = True
        self.visited[c] = True

    def _front_far(self, tof):
        """The farther of the two middle readings, over the last two steps: below WALL_FRONT only when both sensors
        have kept seeing a wall straight ahead (one alone sees a side wall, one sample can be a dropout)."""
        h = self.hist[-2:]
        return max(max(x[R_MID_R], x[L_MID]) for x in h)

    def _decide(self, c, d):
        """Pick the exit of cell c (entered heading d) with the lowest flood value."""
        dist = self.flood(self.targets)
        opts = [k for k in range(4) if not self.wall(c, k)]
        self.next_d = min(opts, key=lambda k: (dist[c[0] + DIRS[k][0], c[1] + DIRS[k][1]], k != d, k)) if opts else d
        self.stop = self.next_d != d or c in self.targets
        self.decided = True

    def _arrived_at_target(self, t):
        """At a cell centre, stopped. Returns True when the whole job is done."""
        if self.cell not in self.targets:
            return False
        if self.phase == "search to centre":
            self.phase, self.targets = "search back to start", (START,)
        elif self.phase == "search back to start":
            self.rounds += 1
            if self.path_is_known() or self.rounds >= self.max_rounds:
                self.phase, self.targets, self.vmax = "speed run", GOAL, V_FAST
            else:
                self.phase, self.targets = "search to centre", GOAL
        elif self.phase == "speed run":
            self.phase = "done"
            self.log.append((self.phase, t))
            return True
        self.log.append((self.phase, t))
        self._decide(self.cell, self.d)
        return False

    def __call__(self, env):
        tof, odom, T = env.tof, env.odom, C.V_FULL
        self.hist = (self.hist + [tof])[-3:]
        med = np.median(self.hist, axis=0)
        if self.state == "DONE":
            return np.zeros(2)
        if self.state == "SENSE0":  # standing on the start cell
            self._sense(self.cell, self.d, med)
            self._decide(self.cell, self.d)
            self.log.append((self.phase, env.t))
            self.state = "TURN"
        if self.state == "TURN":
            self.trk.clear()
            self.edge_prev = None
            err = wrap(HEADING[self.next_d] - odom.th)
            self.turn_ok = self.turn_ok + 1 if abs(err) < 0.04 and abs(odom.omega) < 0.4 else 0
            if self.turn_ok >= 3:
                self.d, self.s, self.state, self.decided, self.turn_ok = self.next_d, 0.0, "RUN", False, 0
                return np.zeros(2)
            w = np.clip(4.0 * err, -6.0, 6.0)
            w = np.sign(w) * max(abs(w), 1.5) if abs(err) > 0.04 else 0.0
            return np.array([-w, w]) * C.TRACK / 2 / T

        # RUN: one cell at a time. The cell ahead is read shortly before its centre (a known cell in the speed run is
        # planned at once, so there is a whole cell to brake in).
        self.s += odom.ds
        ahead = (self.cell[0] + DIRS[self.d][0], self.cell[1] + DIRS[self.d][1])
        if not self.decided:
            if self.vmax == V_FAST and self.visited[ahead]:
                self._decide(ahead, self.d)
            elif self.s >= SENSE_AT:
                self._sense(ahead, self.d, med)
                self._decide(ahead, self.d)
        elif self.s >= SENSE_AT - 1.0 and not self.wall(ahead, self.d) and self._front_far(tof) < WALL_FRONT:
            self._set(ahead, self.d, True)  # the wall ahead shows up late (it is only seen square-on from close by)
            self._decide(ahead, self.d)
        # A side wall starts or ends exactly on a cell boundary, so seeing it do so (edge sensors, 4.5 cm ahead of the
        # axle) pins down the position along the corridor. Measured in this sim: an ending wall is seen once the sensor
        # is ~4.6 cm past the boundary (its cone still catches the wall behind), a starting one ~0.6 cm past.
        walls_now = (med[L_EDGE] < WALL_SIDE, med[R_EDGE] < WALL_SIDE)
        if USE_EDGE_FIX and self.edge_prev is not None and abs(odom.omega) < 0.4:
            for was, now in zip(self.edge_prev, walls_now):
                if was != now:
                    seen = self.s + MAZE_TOF_MOUNT[0, 0]  # sensor position along the travel, from this cell's centre
                    true_at = (EDGE_END_PAST if was else EDGE_START_PAST)
                    k = np.round((seen - true_at - CELL / 2) / CELL)
                    fix = CELL / 2 + k * CELL + true_at - MAZE_TOF_MOUNT[0, 0]
                    if abs(fix - self.s) < 6.0:
                        self.s += 0.6 * (fix - self.s)
        self.edge_prev = walls_now
        rem_front = None
        if self.decided and self.stop and self.wall(ahead, self.d) and self.s >= CELL - 6.0 and max(tof[R_MID_R], tof[L_MID]) < 10.0:
            # a front wall is the one thing that pins down where the car is along the cell: the stop point (cell
            # centre) is FRONT_AT_CENTRE from it, measured square-on from the middle pair. Brake and stop on that range
            # rather than on the odometry, which reads a few percent short and piles up along a straight.
            rem_front = 0.5 * (tof[R_MID_R] + tof[L_MID]) * np.cos(MID_RANGE_ANGLE) - FRONT_AT_CENTRE - odom.v * C.CTRL_DT
        if max(tof[R_MID_R], tof[L_MID]) < 3.4 and max(med[R_MID_R], med[L_MID]) < 3.4 and self.s < CELL - 5.0:
            # touching a wall that the map says is open: believe the sensors, stop and re-plan from this cell
            self._set(self.cell, self.d, True)
            self._decide(self.cell, self.d)
            self.state, self.turn_ok, self.decided = "TURN", 0, False
            return np.zeros(2)
        by_range = rem_front is not None and rem_front <= 0.3
        if self.decided and (by_range or self.s >= CELL - (0.3 if not self.stop else 0.4)):
            self.cell, self.s, self.decided = ahead, 0.0 if by_range else self.s - CELL, False
            if self.stop:
                self.state, self.turn_ok = "TURN", 0
                if self.cell in self.targets and self._arrived_at_target(env.t):
                    self.state = "DONE"
                    return np.zeros(2)
                self.decided = False
                return np.zeros(2)
        v = self.vmax
        if self.decided and self.stop:
            v = min(v, np.sqrt(2 * ACCEL * max(CELL - self.s if rem_front is None else rem_front, 0.0)) + 5.0)
        if self.vmax == V_SEARCH and self.s >= CELL - 6.0:
            v = min(v, V_APPROACH)
        self.travel += odom.ds
        self.trk.append((self.travel, tof[L_EDGE], tof[R_EDGE]))
        psi = self._wall_yaw()
        if psi is not None and abs(odom.omega) < 0.4 and odom.v > 8.0:
            e_h = wrap(HEADING[self.d] + psi - odom.th)  # the walls are the heading reference
            odom.th += YAW_GAIN * e_h
            odom.bias -= BIAS_GAIN * e_h  # a steady heading error is gyro bias drift: retune the bias too
        e = self._lateral(tof)
        w = 6.0 * wrap(HEADING[self.d] + np.clip(KP_LAT * e, -0.35, 0.35) - odom.th)
        return np.array([v - w * C.TRACK / 2, v + w * C.TRACK / 2]) / T

    def _wall_yaw(self):
        """Car yaw relative to the corridor (left +, rad). An edge sensor's wall distance changes by sin(yaw) per cm
        driven, so the least-squares slope of that distance over the last ~14 cm gives the yaw, on the sides where the
        wall stayed in view the whole window. Unbiased (mounting offsets cancel), and the long window keeps it to ~2 deg."""
        if len(self.trk) < self.trk.maxlen:
            return None
        a = np.array(self.trk)
        x = a[:, 0] - a[:, 0].mean()
        if a[-1, 0] - a[0, 0] < 8.0:
            return None
        out = []
        for col, sgn in ((1, -1.0), (2, 1.0)):
            if (a[:, col] < WALL_SIDE).all():
                out.append(sgn * np.arcsin(np.clip(EDGE_SIN * np.dot(x, a[:, col] - a[:, col].mean()) / np.dot(x, x), -0.5, 0.5)))
        return float(np.mean(out)) if out else None

    def _lateral(self, tof):
        """Lateral error in cm, + = the car should move left."""
        left, right = tof[L_EDGE] < WALL_SIDE, tof[R_EDGE] < WALL_SIDE
        if left and right:
            return 0.5 * (tof[L_EDGE] - tof[R_EDGE]) * EDGE_SIN
        if left:
            return (tof[L_EDGE] - SIDE_REF) * EDGE_SIN
        if right:
            return (SIDE_REF - tof[R_EDGE]) * EDGE_SIN
        return 0.0
