"""Scored run-2 environment: rules from "Luat thi xe do me cung" (autonomous run, 240 s)."""
from collections import deque

import numpy as np

from .. import config as C
from ..geometry import obb_corners, sat_mtv
from ..robot import FusedOdometry, Robot
from .arena import (
    N_SECTIONS, SEC_NORMAL, SEC_POINT, TRACK_LEN, block_section, inside_track, point_at, progress,
    sample_layout, section_progress, tangent_at,
)
from .localization import MapLocalizer

OBS_DIM, ACT_DIM = 12, 2
CALIBRATION_STEPS = 60  # car is locked on the start line: gyro bias is estimated here
STUCK_STEPS = int(4.0 / C.CTRL_DT)
LOOKAHEAD = (25.0, 60.0)
SEC_S = section_progress()  # centreline arc length of each dividing line


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


class MicromouseEnv:
    """Action: target wheel speed [-1, 1] (fraction of V_FULL) for (left, right), closed by the firmware's wheel
    speed loop (feed-forward + PID on the encoder-measured speed) before it becomes PWM. See `_observe` for the
    observation. `localize=False` by default: the firmware has only encoder+gyro odometry (lib/estimation), no
    map-matching localiser yet, so this is dead reckoning only, matching the car as it exists today."""

    def __init__(self, max_time=C.T_REF, max_repositions=None, with_random=True, localize=False):
        self.localize = localize
        self.max_time = max_time
        self.max_repositions = max_repositions
        self.with_random = with_random
        self.robot = Robot()
        self.odom = FusedOdometry()
        self.loc = MapLocalizer()

    # --- episode ----------------------------------------------------------
    def reset(self, seed=None):
        self.rng = np.random.default_rng(seed)
        lay = sample_layout(self.rng, self.with_random)
        self.centers, self.angles, self.is_random = lay.centers.copy(), lay.angles, lay.is_random
        self.centers0 = lay.centers.copy()
        corners = np.stack([obb_corners(*c, a, 5, 5) for c, a in zip(self.centers, self.angles)])
        s = np.array([progress(q)[0] for q in corners])
        self.s_far, self.s_near = s.max(1), s.min(1)
        n = len(self.centers)
        self.block_sec = block_section(self.centers)
        self.crossed = np.zeros(N_SECTIONS, bool)  # dividing lines the whole car has crossed (Dieu 4)
        self.passed = np.zeros(n, bool)
        self.disturbed = np.zeros(n, bool)

        x, y, th = C.START_POSE
        self.robot.reset(self.rng, (x, y, th) + self.rng.normal(0.0, C.START_SIGMA))
        self.odom.reset(C.START_POSE)
        for _ in range(CALIBRATION_STEPS):
            self.robot.drive((0.0, 0.0), C.CTRL_DT)
            self.odom.update(self.robot.read_encoders(), self.robot.read_gyro(C.CTRL_DT), C.CTRL_DT, calibrating=True)

        # run 1 maps the static blocks (with some mapping error); the random blocks are unknown
        static = ~self.is_random
        self.loc.reset(
            self.rng, C.START_POSE, self.centers0[static] + self.rng.normal(0, 0.5, (static.sum(), 2)),
            self.angles[static] + self.rng.normal(0, np.radians(2), static.sum()),
        )
        self.est = C.START_POSE
        self.err_sum = np.zeros(2)  # position error: [localised, dead-reckoning only]
        self.t, self.steps = 0.0, 0
        self.n_wheel_out = self.n_repos = 0
        self.wheel_out = [False, False]
        self.finished, self.finish_time = False, None
        self.prev_action = np.zeros(2)
        self.hist = deque(maxlen=STUCK_STEPS + 1)
        self.s_prev = progress(self.robot_xy())[0][0]
        self.tof = self.robot.read_tof(self.centers, self.angles)
        self._total_prev = self._raw_score()
        self.contact = False
        return self._observe()

    def robot_xy(self):
        return np.array([self.robot.x, self.robot.y])

    # --- step -------------------------------------------------------------
    def step(self, action):
        u = np.clip(np.asarray(action, float), -1.0, 1.0)
        target = u * C.V_FULL  # held fixed across all substeps, like wheelSetTargets() between main-loop calls
        self.contact = False
        for _ in range(C.SUBSTEPS):
            self.robot.drive_to_target(target, C.PHYS_DT)
            self.contact |= self._collide()
        self.t += C.CTRL_DT
        self.steps += 1
        self.odom.update(self.robot.read_encoders(), self.robot.read_gyro(C.CTRL_DT), C.CTRL_DT)
        self.tof = self.robot.read_tof(self.centers, self.angles)
        self._localize()
        self.prev_action = u

        self._check_wheels()
        self._check_passed()
        repositioned = self._check_out_or_stuck()
        self._check_finish()

        s_now = progress(self.robot_xy())[0][0]
        d_progress = 0.0 if repositioned else s_now - self.s_prev
        self.s_prev = s_now
        total = self._raw_score()
        reward = (total - self._total_prev) + 0.1 * d_progress - 0.005 - (0.03 if self.contact else 0.0)
        self._total_prev = total

        done = self.finished or self.t >= self.max_time
        if self.max_repositions is not None and self.n_repos > self.max_repositions:
            done = True
        return self._observe(), reward, done, self.info()

    def _localize(self):
        if self.localize:
            self.loc.predict(self.odom.ds, self.odom.dth)
            self.loc.update(self.tof)
            self.est = self.loc.estimate()
        else:
            self.est = (self.odom.x, self.odom.y, self.odom.th)
        r = self.robot
        self.err_sum += [np.hypot(self.est[0] - r.x, self.est[1] - r.y), np.hypot(self.odom.x - r.x, self.odom.y - r.y)]

    # --- physics ------------------------------------------------------------
    def _collide(self):
        """Blocks are light foam: the robot plows them and is only slightly slowed."""
        hit = False
        body = self.robot.body_corners()
        for i, c in enumerate(self.centers):
            if np.hypot(c[0] - self.robot.x, c[1] - self.robot.y) > 16.0:
                continue
            mtv = sat_mtv(body, obb_corners(c[0], c[1], self.angles[i], 5, 5))
            if mtv is None:
                continue
            hit = True
            self.centers[i] += mtv
            self.robot.w *= 0.6
            if not self.disturbed[i] and np.hypot(*(self.centers[i] - self.centers0[i])) > C.DISTURB_DIST:
                self.disturbed[i] = True
        return hit

    # --- rules --------------------------------------------------------------
    def _check_wheels(self):
        for side in (0, 1):
            out = not inside_track(self.robot.wheel_corners(side), C.TAPE_HALF).any()
            if out and not self.wheel_out[side]:
                self.n_wheel_out += 1
            self.wheel_out[side] = out

    def _check_passed(self):
        if not inside_track(self.robot_xy(), C.TAPE_HALF)[0]:
            return
        d = np.einsum("cld,ld->cl", self.robot.body_corners()[:, None, :] - SEC_POINT[None], SEC_NORMAL)
        past = (d > 0).all(0)
        for k in range(N_SECTIONS):  # lines are crossed in order; a line's half-plane also holds earlier on the track
            if past[k] and (k == 0 or self.crossed[k - 1]):
                self.crossed[k] = True
        self.passed = self.crossed[self.block_sec]

    def _check_out_or_stuck(self):
        self.hist.append((self.robot.x, self.robot.y, self.robot.th))
        out = not inside_track(self.robot.body_corners(), C.TAPE_HALF).any()
        stuck = False
        if len(self.hist) > STUCK_STEPS:
            x0, y0, t0 = self.hist[0]
            stuck = np.hypot(self.robot.x - x0, self.robot.y - y0) < 3.0 and abs(wrap(self.robot.th - t0)) < 0.35
        if out or stuck:
            self._reposition()
            return True
        return False

    def _reposition(self):
        """Dieu 5: -10, lift the car and put it within 20 cm of the nearest passed block, never beyond an unpassed one."""
        self.n_repos += 1
        self.hist.clear()
        pending = ~self.passed & ~self.disturbed
        limit = self.s_near[pending].min() if pending.any() else np.inf
        pose = C.START_POSE
        if self.crossed.any():
            s_ref = SEC_S[np.flatnonzero(self.crossed).max()]
            for ds in (6, 10, 2, 14, -4, -10, -16, -24):
                s = s_ref + ds
                th = tangent_at(s)
                normal = np.array([-np.sin(th), np.cos(th)])
                found = False
                for lat in (0, 8, -8, 14, -14):
                    p = point_at(s) + lat * normal
                    if (
                        inside_track(p, -5.0)[0]
                        and np.hypot(*(self.centers - p).T).min() > 15.5
                        and s + C.BODY_L / 2 + 1 < limit
                    ):
                        pose, found = (p[0], p[1], th), True
                        break
                if found:
                    break
        self.robot.place(*pose)
        jitter = self.rng.normal(0, [1.5, 1.5, np.radians(2)])
        seed = (pose[0] + jitter[0], pose[1] + jitter[1], pose[2] + jitter[2])
        self.odom.reset(seed, self.odom.bias)
        self.loc.reset(self.rng, seed, self.loc.map_centers, self.loc.map_angles, spread=(1.5, 1.5, np.radians(2)))
        self.est = seed
        self.s_prev = progress(self.robot_xy())[0][0]

    def _check_finish(self):
        c = self.robot.body_corners()
        if (
            (c[:, 1] < 0).all()
            and C.FINISH_X[0] <= self.robot.x <= C.FINISH_X[1]
            and progress(self.robot_xy())[0][0] > 0.75 * TRACK_LEN
        ):
            self.finished, self.finish_time = True, self.t

    # --- scoring ------------------------------------------------------------
    def score_parts(self):
        ok = self.passed & ~self.disturbed
        n_dist = int(self.disturbed.sum())
        parts = {
            "obstacles": float(np.where(self.is_random[ok], C.PTS_RANDOM, C.PTS_STATIC).sum()),
            "finish": C.PTS_FINISH if self.finished else 0.0,
            "speed": max(0.0, (C.T_REF - self.finish_time) / C.T_DIV) if self.finished else 0.0,
            "clean": C.PTS_CLEAN if self.finished and not (self.n_wheel_out or n_dist or self.n_repos) else 0.0,
            "wheel_out": -C.PEN_WHEEL_OUT * self.n_wheel_out,
            "disturbed": -C.PEN_DISTURB * n_dist,
            "reposition": -C.PEN_REPOSITION * self.n_repos,
        }
        return parts

    def _raw_score(self):
        return sum(self.score_parts().values())

    def score(self):
        return max(0.0, self._raw_score())

    def info(self):
        return {
            "t": self.t, "score": self.score(), "finished": self.finished, "finish_time": self.finish_time,
            "passed": int((self.passed & ~self.disturbed).sum()), "n_obstacles": len(self.centers),
            "wheel_out": self.n_wheel_out, "disturbed": int(self.disturbed.sum()), "repositions": self.n_repos,
            "parts": self.score_parts(),
            "pos_err": self.err_sum[0] / max(self.steps, 1), "pos_err_dr": self.err_sum[1] / max(self.steps, 1),
        }

    # --- observation ----------------------------------------------------------
    def _observe(self):
        """tof(4)/120, cross-track, heading err @25cm and @60cm, v, yaw rate, prev action(2), progress."""
        o = self.odom
        pos = np.array(self.est[:2])
        s, cross = progress(pos)
        s, cross = s[0], cross[0]
        herr = [wrap(np.arctan2(*(point_at(s + la) - pos)[::-1]) - self.est[2]) / np.pi for la in LOOKAHEAD]
        v_max = C.OMEGA_MAX * C.WHEEL_R
        return np.array(
            [*self.tof / C.TOF_MAX, np.clip(cross / 20, -1, 1), *herr, o.v / v_max,
             np.clip(o.omega / 10.0, -1, 1), *self.prev_action, s / TRACK_LEN]
        )

    def snapshot(self):
        r, o = self.robot, self.odom
        return {
            "pose": (r.x, r.y, r.th), "odom": self.est, "dead_reckoning": (o.x, o.y, o.th), "centers": self.centers.copy(), "angles": self.angles,
            "tof": self.tof.copy(), "t": self.t, "score": self.score(), "passed": (self.passed & ~self.disturbed).copy(),
            "disturbed": self.disturbed.copy(), "is_random": self.is_random,
        }
