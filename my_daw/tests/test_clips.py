import numpy as np

from mydaw.clips import ToneClip


def test_toneclip_length_and_bounds():
    sr = 44100
    clip = ToneClip(frequency_hz=440.0, duration_seconds=1.0, amplitude=0.5)
    samples = clip.render(sr)
    assert samples.shape == (sr,)
    assert samples.dtype == np.float32
    assert np.max(samples) <= 0.5 + 1e-6
    assert np.min(samples) >= -0.5 - 1e-6


