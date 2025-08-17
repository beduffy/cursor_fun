import numpy as np

from mydaw.tempo_transport import Transport
from mydaw.drums import DrumRack


def test_drum_rack_steps_render_noise():
    tr = Transport(bpm=120.0)
    rack = DrumRack(tr)
    # Four-on-the-floor kick
    for b in [0.0, 1.0, 2.0, 3.0]:
        rack.add_step(b, 0)
    sr = 44100
    samples = rack.render(sr)
    assert samples.ndim == 1 and samples.dtype == np.float32
    assert np.any(np.abs(samples) > 1e-6)


