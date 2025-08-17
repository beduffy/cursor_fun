import numpy as np

from mydaw.timeline import Timeline
from mydaw.clips import ToneClip
from mydaw.stereo import render_tracks_stereo


def test_render_tracks_stereo_with_pans():
    sr = 44100
    total_samples = int(0.5 * sr)
    t1 = Timeline()
    t2 = Timeline()
    t1.add_clip(ToneClip(440.0, 0.5, 0.4), 0.0)
    t2.add_clip(ToneClip(440.0, 0.5, 0.4), 0.0)
    stereo = render_tracks_stereo([t1, t2], sr, total_samples, pans=[-1.0, 1.0])
    # Left-panned track should dominate left channel, right-panned dominates right
    assert stereo.shape == (2, total_samples)
    assert np.max(np.abs(stereo[0])) > np.max(np.abs(stereo[1])) * 0.9  # left louder overall
    assert np.max(np.abs(stereo[1])) > 0.1


