import numpy as np

from mydaw.clips import ToneClip
from mydaw.timeline import Timeline


def test_timeline_places_clip_at_start_time():
    sr = 44100
    timeline = Timeline()
    clip = ToneClip(frequency_hz=100.0, duration_seconds=0.1, amplitude=0.25)
    timeline.add_clip(clip, start_seconds=0.05)

    total_samples = int(0.2 * sr)
    mix = timeline.generate(sr, total_samples)

    start_sample = int(round(0.05 * sr))
    assert np.allclose(mix[: start_sample], 0.0)
    assert np.any(mix[start_sample : start_sample + 10] != 0.0)


