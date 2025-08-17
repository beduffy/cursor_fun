import numpy as np

from mydaw.engine import OfflineEngine
from mydaw.tracks import Track, Mixer
from mydaw.clips import ToneClip


def test_two_tones_same_start_align():
    sr = 44100
    t1 = Track(gain=1.0)
    t2 = Track(gain=1.0)
    t1.add_clip(ToneClip(440.0, 0.2, 0.25), 0.5)
    t2.add_clip(ToneClip(660.0, 0.2, 0.25), 0.5)

    mixer = Mixer()
    mixer.add_track(t1)
    mixer.add_track(t2)

    eng = OfflineEngine(sample_rate=sr)
    out = eng.render([mixer], length_seconds=1.0)

    start = int(0.5 * sr)
    # Energy before start should be ~zero, after start should be non-zero
    assert np.allclose(out[: start - 10], 0.0)
    assert np.any(np.abs(out[start : start + 100]) > 1e-6)


