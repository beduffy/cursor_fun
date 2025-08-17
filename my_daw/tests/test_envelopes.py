import numpy as np

from mydaw.envelopes import GainPoint, render_gain_envelope


def test_render_gain_envelope_linear():
    sr = 100
    total_samples = 100
    env = render_gain_envelope(
        [GainPoint(0.0, 0.0), GainPoint(1.0, 1.0)], sr, total_samples
    )
    assert env.shape == (total_samples,)
    # start near 0, mid near 0.5, end near 1.0
    assert env[0] == 0.0
    assert abs(env[50] - 0.5) < 0.02
    assert abs(env[-1] - 1.0) < 0.02


