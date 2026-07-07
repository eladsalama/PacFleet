"""Dev utility: print the tracker's measured features for canonical motions.

Used to calibrate threat_classifier.synth_dataset against what the Kalman
pipeline actually measures (not hand-guessed numbers). Run:
    python3 -m test.calibrate_features   (from fleet_sim/, after sourcing ROS)
"""
import math
import random

import numpy as np

from fleet_sim import tracking


def measure(path_fn, ticks=80, seed=0):
    rng = random.Random(seed)
    tr = tracking.Tracker()
    for k in range(ticks):
        t = k * tracking.DT
        x, y = path_fn(t)
        zs = []
        if rng.random() >= 0.10:   # sensor dropout, like the sim
            zs.append((x + rng.gauss(0, 0.35), y + rng.gauss(0, 0.35)))
        tr.step(zs)
    best = max(tr.tracks, key=lambda t: t.hits)
    return best.features()


def sim_runner(t):
    """Mimics contacts.py hostile runner: 1.8 m/s with the same weave law."""
    # integrate the weave numerically
    x, y, phase = 0.0, 0.0, 0.0
    dt = 0.02
    steps = int(t / dt)
    for _ in range(steps):
        phase += dt * 0.9
        heading = 0.8 * math.sin(phase * math.pi)
        x += 1.8 * dt * math.cos(heading)
        y += 1.8 * dt * math.sin(heading)
    return x, y


if __name__ == '__main__':
    for seed in range(3):
        calm = measure(lambda t: (5 + 0.6 * t, 10.0), seed=seed)
        fast_straight = measure(lambda t: (5 + 1.8 * t, 10.0), seed=seed)
        zig = measure(lambda t: (5 + 1.7 * t, 10 + 1.2 * math.sin(2 * t)), seed=seed)
        runner = measure(sim_runner, seed=seed)
        print(f'seed {seed}')
        print(f'  calm walker   : {np.round(calm, 3)}')
        print(f'  fast straight : {np.round(fast_straight, 3)}')
        print(f'  test zigzag   : {np.round(zig, 3)}')
        print(f'  sim runner    : {np.round(runner, 3)}')
