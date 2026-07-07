"""Multi-object tracking for the HUB fusion layer.

Constant-velocity Kalman filter per track + Mahalanobis-gated Hungarian
data association. Detections from *different robots* observing the same
contact are fused sequentially into the same track in the same tick —
which is exactly how a Kalman filter combines multiple sensors.

Pure numpy/scipy (no ROS imports) so it is unit-testable.
"""
import math
from collections import deque

import numpy as np
from scipy.optimize import linear_sum_assignment

from . import worldmap

# state = [x, y, vx, vy]
DT = 0.1                      # fusion tick, seconds
ACCEL_STD = 2.5               # process noise (white accel), m/s^2 — sized for
                              # a sprinting, zigzagging human (maneuvering target)
GATE_CHI2 = 9.21              # chi-square 2 dof @ 99%
CONFIRM_HITS = 3              # tentative -> confirmed
COAST_AFTER = 4               # consecutive misses before status COASTING
DROP_CONFIRMED = 50           # consecutive misses before dropping (5 s) — long enough
                              # for the assigned chaser to close in and reacquire
DROP_TENTATIVE = 5

H = np.array([[1.0, 0, 0, 0],
              [0, 1.0, 0, 0]])
R = np.eye(2) * worldmap.SENSOR_NOISE_STD ** 2

TENTATIVE, CONFIRMED, COASTING = 0, 1, 2


def _f_q(dt: float):
    f = np.array([[1, 0, dt, 0],
                  [0, 1, 0, dt],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]], dtype=float)
    s = ACCEL_STD ** 2
    d2, d3, d4 = dt * dt, dt ** 3, dt ** 4
    q = s * np.array([[d4 / 4, 0, d3 / 2, 0],
                      [0, d4 / 4, 0, d3 / 2],
                      [d3 / 2, 0, d2, 0],
                      [0, d3 / 2, 0, d2]])
    return f, q


class Track:
    def __init__(self, track_id: int, z: np.ndarray):
        self.id = track_id
        self.x = np.array([z[0], z[1], 0.0, 0.0])
        self.p = np.diag([R[0, 0] * 2, R[1, 1] * 2, 4.0, 4.0])
        self.hits = 1
        self.misses = 0
        self.status = TENTATIVE
        self.threat_prob = 0.0
        # rolling state history for the threat classifier (~3 s @ 10 Hz)
        self.vel_history: deque[tuple[float, float]] = deque(maxlen=30)
        self.pos_history: deque[tuple[float, float]] = deque(maxlen=30)

    # -- Kalman ---------------------------------------------------------
    def predict(self, dt: float = DT) -> None:
        f, q = _f_q(dt)
        self.x = f @ self.x
        self.p = f @ self.p @ f.T + q

    def innovation(self, z: np.ndarray):
        s = H @ self.p @ H.T + R
        nu = z - H @ self.x
        return nu, s

    def mahalanobis2(self, z: np.ndarray) -> float:
        nu, s = self.innovation(z)
        return float(nu @ np.linalg.solve(s, nu))

    def update(self, z: np.ndarray) -> None:
        nu, s = self.innovation(z)
        k = self.p @ H.T @ np.linalg.inv(s)
        self.x = self.x + k @ nu
        self.p = (np.eye(4) - k @ H) @ self.p

    # -- lifecycle ------------------------------------------------------
    def mark_hit(self) -> None:
        self.hits += 1
        self.misses = 0
        if self.status != CONFIRMED and self.hits >= CONFIRM_HITS:
            self.status = CONFIRMED
        elif self.status == COASTING:
            self.status = CONFIRMED
    def mark_miss(self) -> None:
        self.misses += 1
        if self.status != TENTATIVE and self.misses >= COAST_AFTER:
            self.status = COASTING

    @property
    def dead(self) -> bool:
        limit = DROP_TENTATIVE if self.status == TENTATIVE else DROP_CONFIRMED
        return self.misses >= limit

    # -- features for the threat classifier ------------------------------
    def record_velocity(self) -> None:
        self.vel_history.append((self.x[2], self.x[3]))
        self.pos_history.append((self.x[0], self.x[1]))

    def features(self) -> np.ndarray | None:
        """[mean speed, cross-track deviation std (m), speed std] over ~3 s.

        Erraticism is measured as METERS of lateral deviation from the mean
        direction of travel, not as heading angles: angle noise scales with
        1/speed (slow walkers measure noisier headings than sprinters), while
        position noise contributes the same ~0.2 m to everyone — so a real
        0.5 m weave stands out and a calm walker doesn't.
        """
        if len(self.vel_history) < self.vel_history.maxlen:
            return None      # demand the full 3 s window: KF settling transients
                             # in the first seconds look "fast and erratic"
        v = np.asarray(self.vel_history)
        p = np.asarray(self.pos_history)
        speeds = np.hypot(v[:, 0], v[:, 1])
        disp = p[-1] - p[0]
        norm = math.hypot(disp[0], disp[1])
        if norm < 0.3:            # not really going anywhere: no cross-track axis
            return np.array([float(speeds.mean()), 0.0, float(speeds.std())])
        u = disp / norm           # mean direction of travel
        centered = p - p.mean(axis=0)
        cross_track = centered[:, 0] * (-u[1]) + centered[:, 1] * u[0]
        return np.array([float(speeds.mean()),
                         float(cross_track.std()),
                         float(speeds.std())])


class Tracker:
    """Gated-Hungarian multi-object tracker with sequential multi-sensor fusion."""

    def __init__(self):
        self.tracks: list[Track] = []
        self._next_id = 1

    def step(self, detections: list[tuple[float, float]], dt: float = DT) -> None:
        """One fusion tick: predict all tracks, associate + update, manage lifecycle.

        `detections` are world-frame (x, y) measurements collected since the
        last tick — possibly several per contact when multiple robots have eyes on.
        """
        for t in self.tracks:
            t.predict(dt)

        zs = [np.asarray(z, dtype=float) for z in detections]
        updated: set[int] = set()   # id() of tracks touched this tick
        used_z: set[int] = set()

        if self.tracks and zs:
            cost = np.full((len(self.tracks), len(zs)), 1e6)
            for i, t in enumerate(self.tracks):
                for j, z in enumerate(zs):
                    d2 = t.mahalanobis2(z)
                    if d2 < GATE_CHI2:
                        cost[i, j] = d2
            rows, cols = linear_sum_assignment(cost)
            for i, j in zip(rows, cols):
                if cost[i, j] < 1e6:
                    self.tracks[i].update(zs[j])
                    self.tracks[i].mark_hit()
                    updated.add(id(self.tracks[i]))
                    used_z.add(j)

        # Sequential fusion: leftover detections that still gate to an
        # (already-updated) track are a second sensor's view of the same
        # contact — fuse them instead of spawning a duplicate track.
        for j, z in enumerate(zs):
            if j in used_z:
                continue
            best, best_d2 = None, GATE_CHI2
            for t in self.tracks:
                d2 = t.mahalanobis2(z)
                if d2 < best_d2:
                    best, best_d2 = t, d2
            if best is not None:
                best.update(z)
                used_z.add(j)

        # Anything still unmatched starts a tentative track (counts as its hit) —
        # unless it gates to a track spawned earlier THIS tick (two robots first
        # seeing the same contact simultaneously must not create duplicates).
        for j, z in enumerate(zs):
            if j in used_z:
                continue
            twin = next((t for t in self.tracks
                         if id(t) in updated and t.mahalanobis2(z) < GATE_CHI2), None)
            if twin is not None:
                twin.update(z)
                continue
            t = Track(self._next_id, z)
            self._next_id += 1
            self.tracks.append(t)
            updated.add(id(t))

        # Everything not touched this tick took a miss; prune the dead.
        for t in self.tracks:
            if id(t) not in updated:
                t.mark_miss()
        self.tracks = [t for t in self.tracks if not t.dead]

        self._merge_overlapping()

        for t in self.tracks:
            t.record_velocity()

    def _merge_overlapping(self, radius: float = 1.2) -> None:
        """Safety net: absorb near-duplicate TENTATIVE tracks into established ones.

        Only tentative tracks are eligible for absorption: two *confirmed*
        tracks near each other are two real contacts crossing paths — merging
        them makes the survivor jump between both bodies, which reads as an
        erratic (threat-like) motion signature. Learned the hard way.
        """
        keep: list[Track] = []
        for t in sorted(self.tracks, key=lambda t: -t.hits):
            if t.status == TENTATIVE and any(
                    np.hypot(t.x[0] - k.x[0], t.x[1] - k.x[1]) < radius
                    for k in keep):
                continue
            keep.append(t)
        self.tracks = keep
