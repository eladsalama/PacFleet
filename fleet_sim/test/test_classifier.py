"""Unit tests for the neural threat classifier."""
import numpy as np

from fleet_sim import threat_classifier as tc


def test_train_and_separate(tmp_path):
    model_path = tmp_path / 'model.npz'
    acc = tc.train(model_path)
    assert acc > 0.95
    model = tc.load_model(model_path)
    assert model is not None
    # canonical feature vectors, straight from calibrate_features.py output
    walker = np.array([0.65, 0.18, 0.20])
    sprinter = np.array([1.70, 0.55, 0.28])
    assert model.predict(walker) < 0.3
    assert model.predict(sprinter) > 0.7


def test_numpy_fallback_trainer(tmp_path):
    model_path = tmp_path / 'model_np.npz'
    acc = tc.train_numpy(model_path)
    assert acc > 0.93
    model = tc.load_model(model_path)
    assert model.predict(np.array([1.7, 0.55, 0.28])) > 0.6
    assert model.predict(np.array([0.6, 0.15, 0.20])) < 0.4
