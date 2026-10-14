"""Score a policy under the competition rules and optionally render a run.

    python -m sim.course.evaluate --policy sim/checkpoints/policy.npz --episodes 20 --gif run.gif
    python -m sim.course.evaluate --baseline --episodes 20
"""
import argparse

import numpy as np

from .planner import AStarDriver
from .policy import CHECKPOINT_DIR, HeuristicPolicy, MLPPolicy, rollout


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--policy", default=str(CHECKPOINT_DIR / "policy.npz"))
    ap.add_argument("--baseline", action="store_true", help="evaluate the hand-written controller instead")
    ap.add_argument("--astar", action="store_true", help="evaluate the A* planner + pure pursuit driver")
    ap.add_argument("--episodes", type=int, default=20)
    ap.add_argument("--seed", type=int, default=1000)
    ap.add_argument("--max-time", type=float, default=240.0)
    ap.add_argument("--gif", default=None, help="render one episode to this gif")
    ap.add_argument("--show", action="store_true", help="show one episode in a window")
    ap.add_argument("--pick", choices=["best", "worst", "first"], default="best", help="which episode to render")
    args = ap.parse_args()

    policy = AStarDriver() if args.astar else HeuristicPolicy() if args.baseline else MLPPolicy.load(args.policy)
    rows, picked = [], None
    render = bool(args.gif or args.show)
    for k in range(args.episodes):
        _, info, frames = rollout(policy, args.seed + k, args.max_time, record=render)
        rows.append(info)
        better = {"best": lambda a, b: a > b, "worst": lambda a, b: a < b, "first": lambda a, b: False}[args.pick]
        if render and (picked is None or better(info["score"], picked[0])):
            picked = (info["score"], args.seed + k, frames)
        p = info["parts"]
        print(
            f"seed {args.seed + k}: score {info['score']:6.1f} | t {info['t']:6.1f}s | "
            f"obst {info['passed']:2d}/{info['n_obstacles']:2d} finish {int(info['finished'])} | "
            f"wheel_out {info['wheel_out']} disturbed {info['disturbed']} repos {info['repositions']} | "
            f"speed {p['speed']:4.1f} clean {p['clean']:2.0f} | pos err {info['pos_err']:.1f} cm (dead-reck {info['pos_err_dr']:.1f})"
        )

    if picked:
        from .render import animate

        print(f"rendering {args.pick} episode: seed {picked[1]} (score {picked[0]:.1f})")
        animate(picked[2], args.gif, show=args.show)

    fin = [r for r in rows if r["finished"]]
    print(
        f"\n{args.episodes} episodes: mean score {np.mean([r['score'] for r in rows]):.1f} | "
        f"finish rate {len(fin) / len(rows):.0%} | mean finish time "
        f"{np.mean([r['finish_time'] for r in fin]) if fin else float('nan'):.1f}s | "
        f"clean runs {sum(r['parts']['clean'] > 0 for r in rows)}/{len(rows)} | "
        f"mean pose error {np.mean([r['pos_err'] for r in rows]):.2f} cm vs {np.mean([r['pos_err_dr'] for r in rows]):.2f} cm without map correction"
    )


if __name__ == "__main__":
    main()
