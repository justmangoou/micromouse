"""Policies: small MLP (trained with evolution strategies) and a hand-written baseline."""
from pathlib import Path

import numpy as np

from .env import ACT_DIM, OBS_DIM, MicromouseEnv


CHECKPOINT_DIR = Path(__file__).resolve().parent.parent / "checkpoints"


class MLPPolicy:
    def __init__(self, sizes=(OBS_DIM, 32, 32, ACT_DIM)):
        self.sizes = tuple(sizes)
        self.n_params = sum(i * o + o for i, o in zip(sizes[:-1], sizes[1:]))
        self.set_flat(np.zeros(self.n_params))

    def init_flat(self, rng):
        parts = []
        for i, o in zip(self.sizes[:-1], self.sizes[1:]):
            parts += [(rng.standard_normal((i, o)) / np.sqrt(i)).ravel(), np.zeros(o)]
        return np.concatenate(parts)

    def set_flat(self, flat):
        self.flat = flat
        self.layers, k = [], 0
        for i, o in zip(self.sizes[:-1], self.sizes[1:]):
            w = flat[k : k + i * o].reshape(i, o)
            b = flat[k + i * o : k + i * o + o]
            self.layers.append((w, b))
            k += i * o + o
        return self

    def __call__(self, obs):
        x = obs
        for w, b in self.layers[:-1]:
            x = np.tanh(x @ w + b)
        w, b = self.layers[-1]
        return np.tanh(x @ w + b)

    def save(self, path):
        np.savez(path, flat=self.flat, sizes=np.array(self.sizes))

    @classmethod
    def load(cls, path):
        d = np.load(path)
        return cls(tuple(int(s) for s in d["sizes"])).set_flat(d["flat"])


class HeuristicPolicy:
    """Pure pursuit on the fused odometry plus ToF repulsion. Sanity baseline for the env."""

    def __init__(self, speed=0.7, k_head=1.6, k_avoid=1.4):
        self.speed, self.k_head, self.k_avoid = speed, k_head, k_avoid

    def __call__(self, obs):
        tof = obs[:4] * 120.0
        prox = np.clip((45.0 - tof) / 35.0, 0.0, 1.0)  # [-60 (right edge), -20, +20, +60 (left edge)]
        front = max(prox[0], prox[3])
        steer = self.k_head * obs[5] * np.pi + 0.35 * obs[4]
        steer += self.k_avoid * (prox[1] - prox[2]) + 0.5 * self.k_avoid * (prox[0] - prox[3])
        if front > 0.3 and abs(prox[0] - prox[3]) < 0.15:
            steer += 0.8 * front * (1.0 if obs[4] >= 0 else -1.0)
        v = self.speed * (1.0 - 0.75 * max(front, prox[1], prox[2]))
        steer = np.clip(steer, -1.0, 1.0)
        return np.clip([v - 0.5 * steer, v + 0.5 * steer], -1.0, 1.0)


def rollout(policy, seed, max_time=240.0, record=False, max_repositions=None, with_random=True):
    env = MicromouseEnv(max_time=max_time, max_repositions=max_repositions, with_random=with_random)
    obs = env.reset(seed)
    if hasattr(policy, "reset"):
        policy.reset(env)  # planners read the car's own map / pose estimate / ToF from env
    total, frames = 0.0, []
    done = False
    while not done:
        obs, r, done, info = env.step(policy(obs))
        total += r
        if record:
            frames.append(env.snapshot())
    return total, info, frames
