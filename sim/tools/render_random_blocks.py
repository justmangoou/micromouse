"""Render one gif per random-block placement into a folder, to see how the car deals with each.

    python -m sim.tools.render_random_blocks --out docs/media/random_block --count 12 [--astar]
"""
import argparse
from pathlib import Path

import numpy as np

from ..course.planner import AStarDriver
from ..course.policy import CHECKPOINT_DIR, MLPPolicy, rollout
from ..course.render import animate


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="docs/media/random_block")
    ap.add_argument("--count", type=int, default=12)
    ap.add_argument("--seed", type=int, default=2000)
    ap.add_argument("--astar", action="store_true")
    ap.add_argument("--policy", default=str(CHECKPOINT_DIR / "policy.npz"))
    args = ap.parse_args()

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    policy = AStarDriver() if args.astar else MLPPolicy.load(args.policy)
    for k in range(args.count):
        seed = args.seed + k
        _, info, frames = rollout(policy, seed, record=True)
        x, y = frames[0]["centers"][-1]
        name = f"r{k + 1:02d}_x{x:.0f}_y{y:.0f}_score{info['score']:.0f}.gif"
        animate(frames, str(out / name))
        print(f"{name}: finish {int(info['finished'])} t {info['t']:.1f}s wheel_out {info['wheel_out']} "
              f"disturbed {info['disturbed']} repos {info['repositions']}")


if __name__ == "__main__":
    main()
