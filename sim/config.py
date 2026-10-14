"""Shared constants. Units: cm, s, rad. x right, y up, heading CCW from +x."""
import numpy as np

# --- Arena (rules, Dieu 1) -------------------------------------------------
ARENA_W, ARENA_H, CELL = 180.0, 120.0, 60.0
TAPE_HALF = 1.0  # black tape is ~2 cm wide, centred on the boundary line
PAD = 20.0  # floor below the start / finish line

# Inverted-U track as three rectangles (x0, y0, x1, y1): left column, top band, right column.
TRACK_RECTS = np.array(
    [
        [0.0, -PAD, CELL, ARENA_H],
        [0.0, CELL, ARENA_W, ARENA_H],
        [2 * CELL, -PAD, ARENA_W, ARENA_H],
    ]
)
START_POSE = (30.0, -8.0, np.pi / 2)
CENTERLINE = np.array([[30.0, -8.0], [30.0, 90.0], [150.0, 90.0], [150.0, -30.0]])
FINISH_X = (2 * CELL, ARENA_W)
# the car is placed on the start mark: true start pose error (x, y in cm, heading in rad), ~1 %
START_SIGMA = (0.3, 0.1, np.radians(0.9))

OBSTACLE_SIZE = 10.0
# Official map (rules update 12/10/2026): 17 static blocks at fixed positions, read off the diagram (block centres, cm).
STATIC_BLOCKS = np.array(
    [
        [45, 115], [115, 115], [85, 100], [35, 90], [125, 85], [5, 75], [65, 75], [5, 55], [75, 65],
        [95, 65], [55, 45], [145, 45], [30, 35], [55, 25], [175, 25], [5, 15], [145, 15],
    ],
    float,
)
# Run 2 only: one extra block on 1 of 4 marked spots in the right corner cell / finish cell. The diagram does not
# show the 4 marks, so these are placeholders -- replace them with the real ones.
RANDOM_SPOTS = np.array([[155.0, 105.0], [160.0, 72.0], [160.0, 40.0], [128.0, 28.0]])
DISTURB_DIST = 2.0  # a block pushed further than this from its mark counts as displaced

# --- Scoring (Dieu 3, 5) ---------------------------------------------------
PTS_STATIC, PTS_RANDOM = 10.0, 15.0
PTS_FINISH, PTS_CLEAN = 40.0, 10.0
PEN_WHEEL_OUT, PEN_DISTURB, PEN_REPOSITION = 5.0, 5.0, 10.0
T_REF, T_DIV = 240.0, 6.0  # speed bonus = (240 - t) / 6

# --- Robot (max 15 x 10 cm, Dieu 8) ---------------------------------------
BODY_L, BODY_W = 12.5, 10.0
WHEEL_R, WHEEL_HALF_W = 1.6, 0.5
TRACK = 9.0  # wheel separation
OMEGA_MAX = 42.0  # rad/s at the wheel, ~400 rpm N20 on 7.4 V -> ~67 cm/s
MOTOR_TAU = 0.06
MOTOR_DEADBAND = 0.06
CPR = 420  # encoder counts per wheel revolution (after gearbox)
CM_PER_COUNT = 2 * np.pi * WHEEL_R / CPR

CTRL_DT = 1 / 30  # one VL53L0X reading per control step, matches firmware CONTROL_PERIOD_MS = 33
SUBSTEPS = 4  # physics/speed-loop tick, ~8.3 ms (firmware's real wheel speed loop runs faster, at 5 ms / 200 Hz)
PHYS_DT = CTRL_DT / SUBSTEPS

# --- Wheel speed loop (firmware lib/control/speed_controller.*, config.h) --
# The policy's action is a *target wheel speed*, not raw PWM: duty = feed-forward (target / V_FULL) + PID on the
# speed error, measured from an IIR-filtered encoder speed. This mirrors wheel.cpp exactly (same gains).
V_FULL = OMEGA_MAX * WHEEL_R * 0.925  # cm/s at full duty, mid-life battery (matches config.h V_FULL_CM_S = 62.2)
USE_SPEED_LOOP = True  # firmware's cfg::USE_SPEED_LOOP
SPEED_KP, SPEED_KI, SPEED_KD = 0.004, 0.02, 0.0  # cfg::SPEED_KP / SPEED_KI, duty per cm/s
SPEED_INTEGRAL_LIMIT = 20.0  # nav::Pid's integralLimit, as constructed in wheel.cpp's makeController()
SPEED_OUTPUT_LIMIT = 0.3  # cfg::SPEED_CORR_LIMIT: PID correction clamp, on top of the feed-forward
SPEED_TARGET_DEADBAND = 1.0  # cm/s: below this the PID resets and only the feed-forward is used
SPEED_LOOP_IIR_ALPHA = 0.4  # wheel.cpp's speed_ += 0.4 * (inst - speed_), updated once per PHYS_DT

# 4x VL53L0X: right edge (60 deg out), middle-right (20 deg out), middle-left, left edge. Best compromise between
# the obstacle course (learned policy) and the 16x16 maze (sim/maze/), see sim/README.md. Angle from heading (CCW +), mounting point in the body
# frame. Same as the firmware's nav::TOF_ANGLE / TOF_MOUNT. 
TOF_ANGLES = np.radians([-60.0, -20.0, 20.0, 60.0])
TOF_MOUNT = np.array([[6.0, -4.5], [6.0, -2.0], [6.0, 2.0], [6.0, 4.5]])
TOF_FOV = np.radians([-12.0, -6.0, 0.0, 6.0, 12.0])  # each of the 4 sensors: a 25 deg cone (VL53L0X) approximated by 5 rays
TOF_MIN, TOF_MAX = 3.0, 120.0
CLOSE_RANGE = 4.5  # below this the localiser ignores the scan

# MPU6050 gyro-z
GYRO_NOISE = np.radians(0.3)
GYRO_BIAS_SIGMA = np.radians(0.4)

# --- Running errors the odometry / policy do not know about ---------------
GYRO_BIAS_WALK = np.radians(0.05)  # bias random walk, rad/s per sqrt(s) (temperature, vibration)
SLIP_WALK = 0.003  # wheel traction factor random walk, per sqrt(s)
ACCEL_SLIP_KNEE = 250.0  # wheel accel (rad/s^2) above which the tyres start to slip
ACCEL_SLIP_SCALE = 4000.0
ACCEL_SLIP_MAX = 0.08  # max fraction of encoder travel lost to slip while accelerating
TOF_MOUNT_ANGLE_SIGMA = np.radians(1.0)  # per-episode sensor mounting error
TOF_MOUNT_POS_SIGMA = 0.3  # cm
