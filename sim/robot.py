"""Differential-drive robot: 2 DC motors + encoders, 4x VL53L0X, MPU6050 gyro, fused odometry."""
import numpy as np

from . import config as C
from .geometry import obb_corners, raycast


class Pid:
    """Direct port of nav::Pid (firmware lib/control/pid.h/.cpp)."""

    def __init__(self, kp, ki, kd, integral_limit, output_limit):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.integral_limit, self.output_limit = integral_limit, output_limit
        self.integral = 0.0
        self.prev_error = 0.0

    def reset(self):
        self.integral = self.prev_error = 0.0

    def update(self, setpoint, measured, dt):
        err = setpoint - measured
        self.integral += err * dt
        if self.integral_limit > 0:
            self.integral = np.clip(self.integral, -self.integral_limit, self.integral_limit)
        deriv = (err - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = err
        out = self.kp * err + self.ki * self.integral + self.kd * deriv
        if self.output_limit > 0:
            out = np.clip(out, -self.output_limit, self.output_limit)
        return out


class SpeedController:
    """Direct port of nav::SpeedController (firmware lib/control/speed_controller.h/.cpp)."""

    def __init__(self, v_full, use_speed_loop, pid):
        self.v_full, self.use_speed_loop, self.pid = v_full, use_speed_loop, pid

    def reset(self):
        self.pid.reset()

    def update(self, target, measured, dt):
        ff = target / self.v_full
        if not self.use_speed_loop:
            return ff
        if -C.SPEED_TARGET_DEADBAND < target < C.SPEED_TARGET_DEADBAND:
            self.pid.reset()
            return ff
        return ff + self.pid.update(target, measured, dt)


class Robot:
    """Ground truth + sensor models. Left wheel sits at body +y, right wheel at -y."""

    def reset(self, rng, pose):
        self.rng = rng
        self.x, self.y, self.th = pose
        self.w = np.zeros(2)  # wheel speed [L, R], rad/s
        self.phi = np.zeros(2)  # wheel shaft angle, rad
        self._counts = np.zeros(2, dtype=np.int64)
        # per-episode hardware spread: what the odometry cannot know about
        self.r_true = C.WHEEL_R * rng.normal(1.0, 0.01, 2)
        self.track_true = C.TRACK * rng.normal(1.0, 0.01)
        self.gain = rng.normal(1.0, 0.04, 2)
        self.battery = rng.uniform(0.85, 1.0)
        self.slip = rng.normal(1.0, 0.01, 2)
        self.tof_angles = C.TOF_ANGLES + rng.normal(0.0, C.TOF_MOUNT_ANGLE_SIGMA, len(C.TOF_ANGLES))
        self.tof_mount = C.TOF_MOUNT + rng.normal(0.0, C.TOF_MOUNT_POS_SIGMA, C.TOF_MOUNT.shape)
        self._tof_prev = None  # one-step sensor latency
        self.gyro_bias = rng.normal(0.0, C.GYRO_BIAS_SIGMA)
        self.gyro_scale = rng.normal(1.0, 0.01)
        self._yaw_acc = 0.0
        self.v = 0.0
        self.omega = 0.0
        # wheel speed loop (matches wheel.cpp): a separate encoder tally from read_encoders(), since the firmware's
        # 1 kHz tick feeds both the ~30 Hz main-loop count (acc_) and the 200 Hz speed estimate (block_) from the
        # same counter but with independent "last seen" pointers.
        self._sub_counts = np.zeros(2, dtype=np.int64)
        self.measured_speed = np.zeros(2)  # IIR-filtered encoder speed fed to the PID, cm/s
        self.speed_ctrl = [
            SpeedController(C.V_FULL, C.USE_SPEED_LOOP, Pid(C.SPEED_KP, C.SPEED_KI, C.SPEED_KD, C.SPEED_INTEGRAL_LIMIT, C.SPEED_OUTPUT_LIMIT))
            for _ in range(2)
        ]

    def place(self, x, y, th):
        """Used on the initial drop and on every reposition (Dieu 5). Resets the wheel speed loop's integral and
        measured-speed filter too: without this the PID can wind up to its clamp while the car is stuck against a
        block (repeatedly repositioned with the wheels forced to zero speed) and stay pinned there afterwards.
        NOTE: the firmware has the same latent gap -- main.cpp never calls wheelEnable(false) on a mid-run
        reposition, so Wheel's PID integral is never reset either. Worth fixing there too."""
        self.x, self.y, self.th = x, y, th
        self.w[:] = 0.0
        self.measured_speed[:] = 0.0
        for sc in self.speed_ctrl:
            sc.reset()

    def _substep_counts(self):
        counts = np.floor(self.phi / (2 * np.pi) * C.CPR).astype(np.int64)
        d = counts - self._sub_counts
        self._sub_counts = counts
        return d

    def drive_to_target(self, target_lr, dt):
        """One speed-loop tick: PID from the *previous* tick's measured speed picks a duty (one-tick feedback
        delay, as on the real car), the duty drives the motor physics, then the encoder delta from this tick
        updates the measured speed for the next call. Mirrors Wheel::tick() in wheel.cpp."""
        duty = np.array([self.speed_ctrl[i].update(target_lr[i], self.measured_speed[i], dt) for i in range(2)])
        duty = np.clip(duty, -1.0, 1.0)
        self.drive(duty, dt)
        inst = self._substep_counts() * C.CM_PER_COUNT / dt
        self.measured_speed += C.SPEED_LOOP_IIR_ALPHA * (inst - self.measured_speed)
        return duty

    def drive(self, u, dt):
        u = np.asarray(u, float)
        eff = np.sign(u) * np.clip((np.abs(u) - C.MOTOR_DEADBAND) / (1 - C.MOTOR_DEADBAND), 0, 1)
        target = C.OMEGA_MAX * self.battery * self.gain * eff
        w_old = self.w.copy()
        self.w += (target - self.w) * (1 - np.exp(-dt / C.MOTOR_TAU))
        self.phi += self.w * dt
        self.slip = np.clip(self.slip + self.rng.normal(0.0, C.SLIP_WALK * np.sqrt(dt), 2), 0.94, 1.06)
        accel = np.abs(self.w - w_old) / dt
        loss = np.clip((accel - C.ACCEL_SLIP_KNEE) / C.ACCEL_SLIP_SCALE, 0.0, C.ACCEL_SLIP_MAX)
        v_l, v_r = self.w * self.r_true * self.slip * (1.0 - loss)
        self.v = 0.5 * (v_l + v_r)
        self.omega = (v_r - v_l) / self.track_true
        mid = self.th + 0.5 * self.omega * dt
        self.x += self.v * np.cos(mid) * dt
        self.y += self.v * np.sin(mid) * dt
        self.th += self.omega * dt
        self._yaw_acc += self.omega * dt

    # --- geometry -------------------------------------------------------
    def body_corners(self):
        return obb_corners(self.x, self.y, self.th, C.BODY_L / 2, C.BODY_W / 2)

    def wheel_corners(self, side):
        """side 0 = left (+y), 1 = right (-y)."""
        off = (C.TRACK / 2) * (1 if side == 0 else -1)
        c, s = np.cos(self.th), np.sin(self.th)
        cx, cy = self.x - s * off, self.y + c * off
        return obb_corners(cx, cy, self.th, C.WHEEL_R, C.WHEEL_HALF_W)

    # --- sensors --------------------------------------------------------
    def read_encoders(self):
        counts = np.floor(self.phi / (2 * np.pi) * C.CPR).astype(np.int64)
        d = counts - self._counts
        self._counts = counts
        return d

    def read_gyro(self, dt):
        self.gyro_bias += self.rng.normal(0.0, C.GYRO_BIAS_WALK * np.sqrt(dt))
        yaw_rate = self._yaw_acc / dt
        self._yaw_acc = 0.0
        return self.gyro_scale * yaw_rate + self.gyro_bias + self.rng.normal(0.0, C.GYRO_NOISE)

    def read_tof(self, centers, angles):
        c, s = np.cos(self.th), np.sin(self.th)
        origins = self.tof_mount @ np.array([[c, s], [-s, c]]) + [self.x, self.y]
        ang = (self.th + self.tof_angles)[:, None] + C.TOF_FOV[None]
        dirs = np.stack([np.cos(ang), np.sin(ang)], -1).reshape(-1, 2)
        d = raycast(np.repeat(origins, len(C.TOF_FOV), 0), dirs, centers, angles)
        d = d.reshape(len(C.TOF_ANGLES), -1).min(1)
        d = d + self.rng.normal(0.0, np.maximum(0.5, 0.03 * d))
        d[self.rng.random(d.shape) < 0.01] = C.TOF_MAX  # occasional dropped reading
        d = np.clip(d, C.TOF_MIN, C.TOF_MAX)
        out = d if self._tof_prev is None else self._tof_prev  # reading is one step old
        self._tof_prev = d
        return out


class FusedOdometry:
    """Wheel-encoder + gyro complementary filter. Knows nominal geometry only."""

    ALPHA = 0.98  # weight of the gyro rate in the fused yaw rate
    SLIP_RATE = 1.0  # rad/s encoder/gyro disagreement above which the encoders are ignored

    def reset(self, pose, gyro_bias=0.0):
        self.x, self.y, self.th = pose
        self.bias = gyro_bias
        self.v = 0.0
        self.omega = 0.0
        self.ds = self.dth = 0.0

    def update(self, dcounts, gyro, dt, calibrating=False):
        dl, dr = dcounts * (2 * np.pi * C.WHEEL_R / C.CPR)
        if calibrating or (dcounts[0] == 0 and dcounts[1] == 0):
            self.bias += 0.05 * (gyro - self.bias)
        w_gyro = gyro - self.bias
        w_enc = (dr - dl) / C.TRACK / dt
        w = w_gyro if abs(w_enc - w_gyro) > self.SLIP_RATE else self.ALPHA * w_gyro + (1 - self.ALPHA) * w_enc
        ds = 0.5 * (dl + dr)
        mid = self.th + 0.5 * w * dt
        self.x += ds * np.cos(mid)
        self.y += ds * np.sin(mid)
        self.th += w * dt
        self.v = ds / dt
        self.omega = w
        self.ds, self.dth = ds, w * dt
