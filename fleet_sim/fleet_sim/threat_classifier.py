"""Neural threat classifier for fused tracks.

A small MLP (3 -> 16 -> 16 -> 1) maps kinematic track features
    [mean speed, cross-track deviation std (m), std of speed]
to P(threat). Trained on synthetic data whose distributions were CALIBRATED
against what the Kalman pipeline actually measures for walkers vs the
sprinting/weaving runner (see test/calibrate_features.py) — not guessed.

Training uses PyTorch when available and falls back to a hand-written
numpy trainer otherwise. Inference is ALWAYS pure numpy from an .npz of
weights — the ROS runtime never needs torch. (Talking point: train-time
and inference-time dependencies are different problems.)
"""
import os
from pathlib import Path

import numpy as np

MODEL_PATH = Path(os.path.expanduser('~/.pacfleet/model.npz'))
LAYERS = [3, 16, 16, 1]
RNG_SEED = 7

# Feature scaling keeps both trainers happy (features have very different ranges).
FEATURE_SCALE = np.array([2.0, 0.5, 0.3])


# --------------------------------------------------------------- synthetic data
def synth_dataset(n: int = 4000, seed: int = RNG_SEED):
    """Distributions match tracker-measured stats (test/calibrate_features.py)."""
    rng = np.random.default_rng(seed)
    half = n // 2
    # neutral: walking pace, lateral wobble is pure sensor/estimation noise
    walkers = np.column_stack([
        rng.normal(0.65, 0.15, 3 * half // 4),  # mean speed m/s
        np.abs(rng.normal(0.17, 0.07, 3 * half // 4)),   # cross-track std, m
        np.abs(rng.normal(0.21, 0.07, 3 * half // 4)),   # speed std
    ])
    # ...plus "junk" tracks: near-stationary or KF-settling artifacts (noisy
    # speed-std, no real displacement) — these must classify NEUTRAL
    junk = np.column_stack([
        np.abs(rng.normal(0.15, 0.12, half - len(walkers))),
        np.abs(rng.normal(0.10, 0.10, half - len(walkers))),
        np.abs(rng.normal(0.45, 0.30, half - len(walkers))),
    ])
    neutral = np.vstack([walkers, junk])
    # threat: sprinting with a real weave on top of the noise floor
    threat = np.column_stack([
        rng.normal(1.70, 0.30, half),
        0.10 + np.abs(rng.normal(0.55, 0.30, half)),
        np.abs(rng.normal(0.30, 0.10, half)),
    ])
    x = np.vstack([neutral, threat]) / FEATURE_SCALE
    y = np.concatenate([np.zeros(half), np.ones(half)])
    idx = rng.permutation(n)
    return x[idx], y[idx]


# --------------------------------------------------------------- numpy inference
class MLP:
    """Pure-numpy forward pass over weights loaded from .npz."""

    def __init__(self, weights: dict[str, np.ndarray]):
        self.ws = [weights[f'w{i}'] for i in range(len(LAYERS) - 1)]
        self.bs = [weights[f'b{i}'] for i in range(len(LAYERS) - 1)]

    def predict(self, features: np.ndarray) -> float:
        a = np.asarray(features, dtype=float) / FEATURE_SCALE
        for w, b in zip(self.ws[:-1], self.bs[:-1]):
            a = np.maximum(a @ w + b, 0.0)          # ReLU
        z = float(np.asarray(a @ self.ws[-1] + self.bs[-1]).ravel()[0])
        return 1.0 / (1.0 + np.exp(-z))            # sigmoid


def load_model(path: Path = MODEL_PATH) -> MLP | None:
    if not path.exists():
        return None
    return MLP(dict(np.load(path)))


# --------------------------------------------------------------- trainers
def _save(ws, bs, path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)
    out = {}
    for i, (w, b) in enumerate(zip(ws, bs)):
        out[f'w{i}'] = np.asarray(w, dtype=np.float64)
        out[f'b{i}'] = np.asarray(b, dtype=np.float64)
    np.savez(path, **out)


def train_torch(path: Path = MODEL_PATH, epochs: int = 300) -> float:
    import torch
    import torch.nn as nn
    torch.manual_seed(RNG_SEED)
    x_np, y_np = synth_dataset()
    x = torch.tensor(x_np, dtype=torch.float32)
    y = torch.tensor(y_np, dtype=torch.float32).unsqueeze(1)
    model = nn.Sequential(
        nn.Linear(3, 16), nn.ReLU(),
        nn.Linear(16, 16), nn.ReLU(),
        nn.Linear(16, 1),
    )
    opt = torch.optim.Adam(model.parameters(), lr=1e-2)
    loss_fn = nn.BCEWithLogitsLoss()
    for _ in range(epochs):
        opt.zero_grad()
        loss = loss_fn(model(x), y)
        loss.backward()
        opt.step()
    with torch.no_grad():
        acc = (((torch.sigmoid(model(x)) > 0.5).float() == y).float().mean().item())
    linears = [m for m in model if isinstance(m, nn.Linear)]
    _save([l.weight.T.detach().numpy() for l in linears],
          [l.bias.detach().numpy() for l in linears], path)
    return acc


def train_numpy(path: Path = MODEL_PATH, epochs: int = 800, lr: float = 0.05) -> float:
    """Fallback trainer: same MLP, hand-written backprop, full-batch Adam-less GD."""
    rng = np.random.default_rng(RNG_SEED)
    x, y = synth_dataset()
    y = y.reshape(-1, 1)
    ws = [rng.normal(0, np.sqrt(2.0 / LAYERS[i]), (LAYERS[i], LAYERS[i + 1]))
          for i in range(len(LAYERS) - 1)]
    bs = [np.zeros(LAYERS[i + 1]) for i in range(len(LAYERS) - 1)]
    n = len(x)
    for _ in range(epochs):
        # forward
        acts, zs = [x], []
        a = x
        for i, (w, b) in enumerate(zip(ws, bs)):
            z = a @ w + b
            zs.append(z)
            a = z if i == len(ws) - 1 else np.maximum(z, 0.0)
            acts.append(a)
        p = 1.0 / (1.0 + np.exp(-acts[-1]))
        # backward (BCE-with-logits gradient)
        delta = (p - y) / n
        for i in reversed(range(len(ws))):
            gw = acts[i].T @ delta
            gb = delta.sum(axis=0)
            if i > 0:
                delta = (delta @ ws[i].T) * (zs[i - 1] > 0)
            ws[i] -= lr * gw
            bs[i] -= lr * gb
    acc = float((((1.0 / (1.0 + np.exp(-(np.maximum(np.maximum(
        x @ ws[0] + bs[0], 0) @ ws[1] + bs[1], 0) @ ws[2] + bs[2])))) > 0.5) == y).mean())
    _save(ws, bs, path)
    return acc


def train(path: Path = MODEL_PATH) -> float:
    try:
        return train_torch(path)
    except ImportError:
        return train_numpy(path)


def main():
    acc = train()
    print(f'threat classifier trained, train accuracy = {acc:.3f}, saved to {MODEL_PATH}')


if __name__ == '__main__':
    main()
