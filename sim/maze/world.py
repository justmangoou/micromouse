"""16x16 micromouse maze world: maze generation, wall geometry, the maze car (a shorter body than the course car) and the
stepping environment. The sensor layout is set from `driver.set_layout` (it overwrites MAZE_TOF_ANGLES here)."""
import numpy as np

from .. import config as C
from ..geometry import obb_corners, raycast_rects, sat_mtv
from ..robot import FusedOdometry, Robot

N, CELL, WALL_T = 16, 18.0, 1.2
GOAL = ((7, 7), (7, 8), (8, 7), (8, 8))
START = (0, 0)
DIRS = ((0, 1), (1, 0), (0, -1), (-1, 0))  # N E S W
HEADING = np.array([np.pi / 2, 0.0, -np.pi / 2, np.pi])

MAZE_TOF_ANGLES = np.radians([-60.0, -20.0, 20.0, 60.0])  # replaced by set_layout()
# The 12.5 cm body of the obstacle course cannot turn in place in a 16.8 cm corridor (its corners sweep r = 8.0 cm against
# 8.4 cm of room), so the maze uses a 9 cm long body. Overall width stays 10 cm (wheel to wheel).
MAZE_BODY_L = 9.0
MAZE_TOF_MOUNT = np.array([[4.5, -4.5], [4.5, -2.0], [4.5, 2.0], [4.5, 4.5]])


def make_maze(rng, loops=22):
    """wv[i, j]: vertical wall on line x = i*18 in row j. wh[i, j]: horizontal wall on line y = j*18 in column i."""
    wv, wh = np.ones((N + 1, N), bool), np.ones((N, N + 1), bool)

    def carve(a, b):
        (i, j), (k, m) = a, b
        if k != i:
            wv[max(i, k), j] = False
        else:
            wh[i, max(j, m)] = False

    seen = np.zeros((N, N), bool)
    for g in GOAL:
        seen[g] = True  # the centre is kept out of the maze and attached afterwards, through a single door
    stack = [START]
    seen[START] = True
    while stack:
        i, j = stack[-1]
        nb = [
            (i + di, j + dj) for di, dj in DIRS
            if 0 <= i + di < N and 0 <= j + dj < N and not seen[i + di, j + dj] and ((i, j), (i + di, j + dj)) not in
            (((0, 0), (1, 0)), ((1, 0), (0, 0)))  # the start cell has only the north exit
        ]
        if nb:
            nxt = nb[rng.integers(len(nb))]
            carve((i, j), nxt)
            seen[nxt] = True
            stack.append(nxt)
        else:
            stack.pop()
    for a in GOAL:  # open the 2x2 centre
        for b in GOAL:
            if abs(a[0] - b[0]) + abs(a[1] - b[1]) == 1:
                carve(a, b)
    doors = [(g, (g[0] + di, g[1] + dj)) for g in GOAL for di, dj in DIRS if (g[0] + di, g[1] + dj) not in GOAL]
    carve(*doors[rng.integers(len(doors))])  # the one entrance
    perimeter = {frozenset(d) for d in doors}
    walls = [("v", i, j) for i in range(1, N) for j in range(N) if wv[i, j] and (i, j) != (1, 0)
             and frozenset(((i - 1, j), (i, j))) not in perimeter] + \
            [("h", i, j) for i in range(N) for j in range(1, N) if wh[i, j] and frozenset(((i, j - 1), (i, j))) not in perimeter]
    for k in rng.choice(len(walls), loops, replace=False):  # a few loops, so there is more than one route
        t, i, j = walls[k]
        (wv if t == "v" else wh)[i, j] = False
    return wv, wh


def wall_rects(wv, wh):
    h = WALL_T / 2
    out = []
    for i, j in zip(*np.nonzero(wv)):
        out.append([i * CELL - h, j * CELL - h, i * CELL + h, (j + 1) * CELL + h])
    for i, j in zip(*np.nonzero(wh)):
        out.append([i * CELL - h, j * CELL - h, (i + 1) * CELL + h, j * CELL + h])
    return np.array(out)


class MazeRobot(Robot):
    def body_corners(self):
        return obb_corners(self.x, self.y, self.th, MAZE_BODY_L / 2, C.BODY_W / 2)

    def reset(self, rng, pose):
        super().reset(rng, pose)
        self.tof_angles = MAZE_TOF_ANGLES + rng.normal(0.0, C.TOF_MOUNT_ANGLE_SIGMA, 4)
        self.tof_mount = MAZE_TOF_MOUNT + rng.normal(0.0, C.TOF_MOUNT_POS_SIGMA, (4, 2))

    def read_tof_rects(self, rects):
        c, s = np.cos(self.th), np.sin(self.th)
        origins = self.tof_mount @ np.array([[c, s], [-s, c]]) + [self.x, self.y]
        ang = (self.th + self.tof_angles)[:, None] + C.TOF_FOV[None]
        dirs = np.stack([np.cos(ang), np.sin(ang)], -1).reshape(-1, 2)
        d = raycast_rects(np.repeat(origins, len(C.TOF_FOV), 0), dirs, rects, C.TOF_MAX)
        d = d.reshape(4, -1).min(1)
        d = d + self.rng.normal(0.0, np.maximum(0.5, 0.03 * d))
        d[self.rng.random(4) < 0.01] = C.TOF_MAX  # occasional dropped reading
        d = np.clip(d, C.TOF_MIN, C.TOF_MAX)
        out = d if self._tof_prev is None else self._tof_prev  # reading is one step old
        self._tof_prev = d
        return out


class MazeEnv:
    def __init__(self):
        self.robot, self.odom = MazeRobot(), FusedOdometry()

    def reset(self, seed):
        self.rng = np.random.default_rng(seed)
        self.wv, self.wh = make_maze(self.rng)
        self.rects = wall_rects(self.wv, self.wh)
        self.rect_c = 0.5 * (self.rects[:, :2] + self.rects[:, 2:])
        self.rect_h = 0.5 * (self.rects[:, 2:] - self.rects[:, :2])
        start = (CELL / 2, CELL / 2, np.pi / 2)
        self.robot.reset(self.rng, np.array(start) + self.rng.normal(0.0, C.START_SIGMA))
        self.odom.reset(start)
        for _ in range(60):  # gyro bias calibration, standing still
            self.robot.drive((0.0, 0.0), C.CTRL_DT)
            self.odom.update(self.robot.read_encoders(), self.robot.read_gyro(C.CTRL_DT), C.CTRL_DT, calibrating=True)
        self.t, self.hits, self.contact = 0.0, 0, False
        self.tof = self.robot.read_tof_rects(self.rects)
        return self.tof

    def _collide(self):
        r = self.robot
        near = (np.abs(self.rect_c[:, 0] - r.x) < self.rect_h[:, 0] + 9.5) & (np.abs(self.rect_c[:, 1] - r.y) < self.rect_h[:, 1] + 9.5)
        hit = False
        for k in np.flatnonzero(near):
            x0, y0, x1, y1 = self.rects[k]
            mtv = sat_mtv(np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]]), r.body_corners())
            if mtv is not None:
                r.x += mtv[0]
                r.y += mtv[1]
                r.w *= 0.5
                hit = True
        return hit

    def step(self, u):
        target = np.clip(u, -1.0, 1.0) * C.V_FULL
        hit = False
        for _ in range(C.SUBSTEPS):
            self.robot.drive_to_target(target, C.PHYS_DT)
            hit |= self._collide()
        self.t += C.CTRL_DT
        self.hits += hit
        self.odom.update(self.robot.read_encoders(), self.robot.read_gyro(C.CTRL_DT), C.CTRL_DT)
        self.tof = self.robot.read_tof_rects(self.rects)
        return self.tof

    def true_cell(self):
        return int(np.clip(self.robot.x // CELL, 0, N - 1)), int(np.clip(self.robot.y // CELL, 0, N - 1))

