"""Unit tests for the HUB's Kalman MOT fusion layer (pure logic, no ROS)."""
import math
import random

import numpy as np

from fleet_sim import tracking


def simulate(tracker, path_fn, ticks, sensors=1, p_miss=0.0, seed=1):
    rng = random.Random(seed)
    for k in range(ticks):
        t = k * tracking.DT
        x, y = path_fn(t)
        zs = []
        for _ in range(sensors):
            if rng.random() >= p_miss:
                zs.append((x + rng.gauss(0, 0.35), y + rng.gauss(0, 0.35)))
        tracker.step(zs)


def test_converges_on_constant_velocity_target():
    tr = tracking.Tracker()
    simulate(tr, lambda t: (5.0 + 1.0 * t, 10.0), ticks=50)
    assert len(tr.tracks) == 1
    t = tr.tracks[0]
    assert t.status == tracking.CONFIRMED
    tt = 49 * tracking.DT
    assert math.hypot(t.x[0] - (5.0 + tt), t.x[1] - 10.0) < 0.6
    assert abs(t.x[2] - 1.0) < 0.6          # vx estimate
    assert abs(t.x[3]) < 0.6                # vy estimate


def test_occlusion_coasts_then_reacquires():
    tr = tracking.Tracker()
    simulate(tr, lambda t: (5.0 + 1.0 * t, 10.0), ticks=40)
    t = tr.tracks[0]
    cov_before = t.p[0, 0] + t.p[1, 1]
    for _ in range(10):                     # target behind a building
        tr.step([])
    assert t.status == tracking.COASTING
    assert t.p[0, 0] + t.p[1, 1] > cov_before   # uncertainty visibly grows
    # reacquire near the predicted position
    px, py = t.x[0], t.x[1]
    for _ in range(5):
        tr.step([(px, py)])
    assert t.status == tracking.CONFIRMED


def test_track_drops_after_long_occlusion():
    tr = tracking.Tracker()
    simulate(tr, lambda t: (5.0, 10.0), ticks=10)
    for _ in range(tracking.DROP_CONFIRMED + 1):
        tr.step([])
    assert len(tr.tracks) == 0


def test_two_sensors_fuse_into_one_track():
    """Two robots seeing the same contact must NOT create duplicate tracks."""
    tr = tracking.Tracker()
    simulate(tr, lambda t: (20.0 + 0.5 * t, 20.0), ticks=50, sensors=2)
    assert len(tr.tracks) == 1


def test_two_separate_contacts_get_two_tracks():
    tr = tracking.Tracker()
    rng = random.Random(3)
    for k in range(50):
        zs = [(10.0 + rng.gauss(0, 0.35), 10.0 + rng.gauss(0, 0.35)),
              (30.0 + rng.gauss(0, 0.35), 30.0 + rng.gauss(0, 0.35))]
        tr.step(zs)
    confirmed = [t for t in tr.tracks if t.status == tracking.CONFIRMED]
    assert len(confirmed) == 2


def test_features_capture_erratic_motion():
    """Zigzagging fast mover must show higher cross-track deviation than a walker."""
    def run(path_fn):
        tr = tracking.Tracker()
        simulate(tr, path_fn, ticks=60)
        return tr.tracks[0].features()

    calm = run(lambda t: (5.0 + 0.6 * t, 10.0))
    erratic = run(lambda t: (5.0 + 1.7 * t, 10.0 + 1.2 * math.sin(2.0 * t)))
    assert calm is not None and erratic is not None
    assert erratic[0] > calm[0]        # faster
    assert erratic[1] > calm[1]        # more lateral weave (meters)
