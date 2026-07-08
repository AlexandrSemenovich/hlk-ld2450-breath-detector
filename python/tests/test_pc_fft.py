import os
os.environ.setdefault("MPLBACKEND", "Agg")

import numpy as np
from analysis import detect_breath


def _signal(f_hz, amp, fs=10.0, dur=30.0, noise=0.0, seed=0):
    rng = np.random.default_rng(seed)
    t = np.arange(0, dur, 1.0 / fs)
    ts = (t * 1000.0).astype(int).tolist()
    ac = amp * np.sin(2 * np.pi * f_hz * t)
    if noise:
        ac = ac + noise * rng.standard_normal(t.size)
    return ts, ac.tolist()


def test_15_bpm_detected():
    ts, ac = _signal(0.25, 8.0)
    bpm, detected, quality, snr = detect_breath(ts, ac)
    assert detected, "breathing should be detected"
    assert abs(bpm - 15.0) < 3.0, f"bpm={bpm}"
    assert quality > 0


def test_30_bpm_detected():
    ts, ac = _signal(0.5, 6.0)
    bpm, detected, quality, snr = detect_breath(ts, ac)
    assert detected
    assert abs(bpm - 30.0) < 4.0


def test_no_breath_flat():
    ts, ac = _signal(0.0, 0.0)
    ac = [0.0] * len(ac)
    bpm, detected, quality, snr = detect_breath(ts, ac)
    assert not detected
    assert bpm == 0.0


def test_low_snr_rejected():
    # Breathing present but drowned in noise -> should be rejected.
    ts, ac = _signal(0.25, 1.0, noise=20.0, seed=1)
    bpm, detected, quality, snr = detect_breath(ts, ac)
    assert not detected
