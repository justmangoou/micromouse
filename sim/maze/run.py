"""Run maze episodes (search, then speed run) and print a summary.

    python -m sim.maze --seed 1 --episodes 12
    python -m sim.maze --seed 1 --gif docs/media/maze.gif
"""
import argparse

import numpy as np

from .driver import FloodFillDriver
from .world import MazeEnv

def run(seed, max_time=900.0, record=False):
    env = MazeEnv()
    env.reset(seed)
    drv = FloodFillDriver()
    frames, last_cell_t, last_cell = [], 0.0, None
    while env.t < max_time and drv.state != "DONE":
        env.step(drv(env))
        if record:
            frames.append({"pose": (env.robot.x, env.robot.y, env.robot.th), "tof": env.tof.copy(), "t": env.t,
                           "phase": drv.phase, "cell": drv.cell})
        if env.true_cell() != last_cell:
            last_cell, last_cell_t = env.true_cell(), env.t
        elif env.t - last_cell_t > 25.0:
            break  # not moving between cells for 25 s: stuck
    marks = dict((p, t) for p, t in drv.log)
    info = {
        "done": drv.state == "DONE", "t": env.t, "hits": env.hits, "rounds": drv.rounds,
        "visited": int(drv.visited.sum()), "t_found": marks.get("speed run"), "t_speed": (marks["done"] - marks["speed run"]) if "done" in marks and "speed run" in marks else None,
    }
    return env, drv, info, frames


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--episodes", type=int, default=1, help="run this many seeds and print a summary")
    ap.add_argument("--gif", default=None)
    args = ap.parse_args()
    results = []
    for k in range(args.episodes):
        env, drv, info, frames = run(args.seed + k, record=bool(args.gif) and k == 0)
        results.append(info)
        print(f"seed {args.seed + k}: done {info['done']} | search {info['t_found']} s | speed run {info['t_speed']} s | "
              f"wall bumps {info['hits']} | cells seen {info['visited']}/256 | rounds {info['rounds']}")
        if args.gif and k == 0:
            from .render import animate_maze

            animate_maze(env, frames, args.gif)
    if args.episodes > 1:
        ok = [r for r in results if r["done"]]
        print(f"\n{len(ok)}/{len(results)} solved | mean speed run {np.mean([r['t_speed'] for r in ok]):.1f} s")


if __name__ == "__main__":
    main()
