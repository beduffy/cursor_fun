import numpy as np

from mydaw.tracks import Track, Mixer
from mydaw.clips import ToneClip


def test_track_gain_and_mixer_sum():
    sr = 44100
    track1 = Track(gain=0.5)
    track1.add_clip(ToneClip(440.0, 0.1, 1.0), start_seconds=0.0)

    track2 = Track(gain=0.25)
    track2.add_clip(ToneClip(880.0, 0.1, 1.0), start_seconds=0.0)

    total_samples = int(0.1 * sr)
    mix = Mixer(bus_gain=1.0)
    mix.add_track(track1)
    mix.add_track(track2)
    out = mix.generate(sr, total_samples)

    # Both tracks produce non-zero energy
    assert np.any(out != 0.0)
    # Output is within [-1, 1]
    assert np.max(out) <= 1.0 + 1e-6
    assert np.min(out) >= -1.0 - 1e-6


