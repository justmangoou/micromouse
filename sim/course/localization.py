"""Monte-Carlo localisation: corrects the fused odometry against the static blocks mapped in run 1."""
import numpy as np

from .. import config as C
from ..geometry import raycast
from .arena import inside_track


class MapLocalizer:
    """Particle filter. Motion = fused-odometry increments, measurement = the 4 ToF readings.

    Only the static blocks are in the map; the random block (and anything pushed) show up as
    readings shorter than expected, so the beam model keeps a flat outlier floor.
    """

    def __init__(self, n=150):
        self.n = n

    def reset(self, rng, pose, map_centers, map_angles, spread=(0.1, 0.1, np.radians(0.3))):
        self.rng = rng
        self.map_centers, self.map_angles = map_centers, map_angles
        self.p = np.asarray(pose) + rng.normal(0.0, spread, (self.n, 3))
        self.w = np.full(self.n, 1.0 / self.n)

    def predict(self, ds, dth):
        n = self.n
        ds_n = ds * (1 + self.rng.normal(0, 0.03, n)) + self.rng.normal(0, 0.05, n)
        dth_n = dth + self.rng.normal(0, np.radians(0.15) + 0.03 * abs(dth), n)
        mid = self.p[:, 2] + 0.5 * dth_n
        self.p[:, 0] += ds_n * np.cos(mid)
        self.p[:, 1] += ds_n * np.sin(mid)
        self.p[:, 2] += dth_n

    def update(self, tof):
        if len(self.map_centers) == 0 or np.all(tof >= C.TOF_MAX - 1e-6):
            return
        if tof.min() < C.CLOSE_RANGE:
            return  # pressed against a block: readings are unreliable and the block may have moved
        c, s = np.cos(self.p[:, 2]), np.sin(self.p[:, 2])
        # sensor origins and beam directions for every particle: (n, 4 sensors, 2)
        mx, my = C.TOF_MOUNT[:, 0], C.TOF_MOUNT[:, 1]
        ox = self.p[:, 0, None] + c[:, None] * mx - s[:, None] * my
        oy = self.p[:, 1, None] + s[:, None] * mx + c[:, None] * my
        a = (self.p[:, 2, None] + C.TOF_ANGLES)[..., None] + C.TOF_FOV  # (n, 4 sensors, rays per cone)
        k = len(C.TOF_FOV)
        origins = np.repeat(np.stack([ox, oy], -1).reshape(-1, 2), k, 0)
        dirs = np.stack([np.cos(a), np.sin(a)], -1).reshape(-1, 2)
        exp = raycast(origins, dirs, self.map_centers, self.map_angles).reshape(self.n, -1, k).min(2)
        sigma = 1.5 + 0.04 * exp
        like = 0.12 + np.exp(-0.5 * ((tof[None] - exp) / sigma) ** 2)  # flat floor = unmapped blocks / dropouts
        lw = np.log(like).sum(1)
        # the car stays on the tape: mild prior against particles that left the track
        off = ~inside_track(self.p[:, :2], C.TAPE_HALF)
        lw[off] += np.log(0.4)
        self.w *= np.exp(lw - lw.max())
        self.w /= self.w.sum()
        if 1.0 / np.sum(self.w**2) < self.n / 2:
            self._resample()

    def _resample(self):
        pos = (self.rng.random() + np.arange(self.n)) / self.n
        idx = np.minimum(np.searchsorted(np.cumsum(self.w), pos), self.n - 1)
        self.p = self.p[idx] + self.rng.normal(0, [0.15, 0.15, np.radians(0.2)], (self.n, 3))
        self.w[:] = 1.0 / self.n

    def estimate(self):
        x, y = self.w @ self.p[:, 0], self.w @ self.p[:, 1]
        th = np.arctan2(self.w @ np.sin(self.p[:, 2]), self.w @ np.cos(self.p[:, 2]))
        return x, y, th
