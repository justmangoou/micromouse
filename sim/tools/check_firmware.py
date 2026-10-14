"""Check that the firmware's copies of the sim defaults have not drifted.

    python -m sim.tools.check_firmware

The sim keeps its own layout in sim/config.py (it can run other sensor layouts for experiments); the firmware defines each
value once (lib/nav/include/tof_layout.h, include/config.h, lib/nav/include/navigator.h). Exit status 1 if any differ.
"""
import re
import sys
from pathlib import Path

import numpy as np

from .. import config as C

FW = Path(__file__).resolve().parents[2] / "firmware"


def numbers(text):
    return [float(x) for x in re.findall(r"[-+]?\d+\.?\d*(?:[eE][-+]?\d+)?(?=f?\b)", text)]


def array(src, name):
    m = re.search(r"%s(?:\[[^\]]*\])+\s*=\s*\{(.*?)\};" % name, src, re.S)
    if not m:
        raise KeyError(name)
    return np.array(numbers(m.group(1)))


def scalar(src, name):
    m = re.search(r"\b%s\s*=\s*([-+]?[\d.]+(?:e[-+]?\d+)?)f?\s*;" % name, src)
    if not m:
        raise KeyError(name)
    return float(m.group(1))


def main():
    tof = (FW / "lib/nav/include/tof_layout.h").read_text()
    cfg = (FW / "include/config.h").read_text()
    nav = (FW / "lib/nav/include/navigator.h").read_text()
    checks = [
        ("tof angles (deg)", array(tof, "ANGLE_DEG"), np.degrees(C.TOF_ANGLES), 1e-3),
        ("tof mount (cm)", array(tof, "MOUNT_CM").reshape(-1, 2), C.TOF_MOUNT, 1e-3),
        ("tof cone rays (deg)", array(tof, "FOV_DEG"), np.degrees(C.TOF_FOV), 1e-3),
        ("tof range (cm)", np.array([scalar(tof, "MIN_CM"), scalar(tof, "MAX_CM")]), np.array([C.TOF_MIN, C.TOF_MAX]), 1e-3),
        ("wheel radius", scalar(cfg, "WHEEL_RADIUS_CM"), C.WHEEL_R, 1e-3),
        ("wheel track", scalar(cfg, "WHEEL_TRACK_CM"), C.TRACK, 1e-3),
        ("counts per rev", scalar(cfg, "COUNTS_PER_REV"), C.CPR, 1e-3),
        ("V_FULL (config.h)", scalar(cfg, "V_FULL_CM_S"), C.V_FULL, 0.1),
        ("V_FULL (navigator.h)", scalar(nav, "V_FULL"), C.V_FULL, 0.1),
        ("wheel track (navigator.h)", scalar(nav, "WHEEL_TRACK"), C.TRACK, 1e-3),
        ("speed loop Kp", scalar(cfg, "SPEED_KP"), C.SPEED_KP, 1e-6),
        ("speed loop Ki", scalar(cfg, "SPEED_KI"), C.SPEED_KI, 1e-6),
        ("speed loop correction limit", scalar(cfg, "SPEED_CORR_LIMIT"), C.SPEED_OUTPUT_LIMIT, 1e-6),
        ("start pose", np.array([scalar(cfg, "START_X"), scalar(cfg, "START_Y"), scalar(cfg, "START_TH")]), np.array(C.START_POSE), 1e-4),
    ]
    bad = 0
    for name, fw, sim, tol in checks:
        ok = np.shape(fw) == np.shape(sim) and np.allclose(fw, sim, atol=tol)
        bad += not ok
        print(f"{'ok      ' if ok else 'DIFFERS '}{name}" + ("" if ok else f": firmware {np.round(fw, 4).tolist()} vs sim {np.round(sim, 4).tolist()}"))
    print("\nfirmware matches the sim defaults" if not bad else f"\n{bad} value(s) differ")
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
