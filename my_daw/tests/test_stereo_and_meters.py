import numpy as np

from mydaw.stereo import StereoTrack, StereoMixer, constant_power_pan
from mydaw.clips import ToneClip
from mydaw.meters import peak_meter_stereo


def test_constant_power_pan_energy_distribution():
    mono = np.ones(100, dtype=np.float32) * 0.5
    stereo_mid = constant_power_pan(mono, 0.0)
    stereo_left = constant_power_pan(mono, -1.0)
    stereo_right = constant_power_pan(mono, 1.0)
    # Mid should split roughly evenly
    assert np.allclose(np.max(np.abs(stereo_mid[0])), np.max(np.abs(stereo_mid[1])), atol=1e-6)
    # Hard pan should put signal on one side
    assert np.max(np.abs(stereo_left[0])) > 0.1 and np.max(np.abs(stereo_left[1])) < 1e-3
    assert np.max(np.abs(stereo_right[1])) > 0.1 and np.max(np.abs(stereo_right[0])) < 1e-3


def test_stereo_mixer_and_peak_meter():
    sr = 44100
    t = StereoTrack(gain=1.0, pan=-0.5)
    t.add_clip(ToneClip(440.0, 0.1, 0.4), 0.0)
    mixer = StereoMixer()
    mixer.add_track(t)
    out = mixer.generate(sr, int(0.2 * sr))
    l, r = peak_meter_stereo(out)
    assert l > 0.1 and r > 0.0


