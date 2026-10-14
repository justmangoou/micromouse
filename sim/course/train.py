"""Train the MLP driving policy with evolution strategies (antithetic, rank-shaped, Adam).

    python -m sim.course.train --generations 50 --out sim/checkpoints/policy.npz
"""
import argparse
import multiprocessing as mp
import time
from pathlib import Path

import numpy as np

from .policy import CHECKPOINT_DIR, MLPPolicy, rollout

MAX_REPOSITIONS = 3  # abort hopeless training episodes early


def _evaluate(args):
    flat, seeds, max_time = args
    policy = MLPPolicy().set_flat(flat)
    returns, scores, finished = [], [], []
    for s in seeds:
        ret, info, _ = rollout(policy, s, max_time, max_repositions=MAX_REPOSITIONS)
        returns.append(ret)
        scores.append(info["score"])
        finished.append(info["finished"])
    return np.mean(returns), np.mean(scores), np.mean(finished)


def _ranks(x):
    r = np.empty(len(x))
    r[np.argsort(x)] = np.linspace(-0.5, 0.5, len(x))
    return r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--generations", type=int, default=50)
    ap.add_argument("--pairs", type=int, default=24, help="antithetic pairs per generation")
    ap.add_argument("--layouts", type=int, default=2, help="obstacle layouts per generation (shared by all candidates)")
    ap.add_argument("--sigma", type=float, default=0.08)
    ap.add_argument("--lr", type=float, default=0.03)
    ap.add_argument("--max-time", type=float, default=45.0, help="seconds per training episode")
    ap.add_argument("--workers", type=int, default=mp.cpu_count())
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", default=str(CHECKPOINT_DIR / "policy.npz"))
    ap.add_argument("--resume", default=None)
    args = ap.parse_args()

    rng = np.random.default_rng(args.seed)
    policy = MLPPolicy()
    if args.resume:
        theta = MLPPolicy.load(args.resume).flat.copy()
    else:
        theta = policy.init_flat(rng)
        theta[-C_OUT:] = 0.3  # start by driving forward
    m, v = np.zeros_like(theta), np.zeros_like(theta)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    val_seeds = list(range(90_000, 90_008))
    best_val = -np.inf

    with mp.Pool(args.workers) as pool:
        for gen in range(1, args.generations + 1):
            t0 = time.time()
            seeds = [gen * 100 + k for k in range(args.layouts)]
            eps = rng.standard_normal((args.pairs, theta.size))
            cands = np.concatenate([theta + args.sigma * eps, theta - args.sigma * eps])
            jobs = [(c, seeds, args.max_time) for c in cands]
            res = np.array(pool.map(_evaluate, jobs, chunksize=2))
            ret = res[:, 0]
            u = _ranks(ret)
            grad = ((u[: args.pairs] - u[args.pairs :])[:, None] * eps).sum(0) / (args.pairs * args.sigma)
            grad -= 0.005 * theta  # weight decay
            m = 0.9 * m + 0.1 * grad
            v = 0.999 * v + 0.001 * grad**2
            theta += args.lr * (m / (1 - 0.9**gen)) / (np.sqrt(v / (1 - 0.999**gen)) + 1e-8)

            msg = f"gen {gen:4d} | pop ret mean {ret.mean():7.1f} max {ret.max():7.1f} | finish {res[:, 2].mean():4.2f}"
            if gen % 5 == 0 or gen == 1:
                val = _evaluate((theta, val_seeds, args.max_time))
                msg += f" | VAL ret {val[0]:7.1f} score {val[1]:6.1f} finish {val[2]:4.2f}"
                MLPPolicy().set_flat(theta).save(out.with_name(out.stem + "_last.npz"))
                if val[0] > best_val:
                    best_val = val[0]
                    MLPPolicy().set_flat(theta).save(out)
                    msg += " *"
            print(msg + f" | {time.time() - t0:5.1f}s", flush=True)


C_OUT = 2  # bias entries of the output layer

if __name__ == "__main__":
    main()
