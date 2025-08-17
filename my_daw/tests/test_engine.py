import numpy as np

from mydaw.engine import OfflineEngine
from mydaw.timeline import Timeline
from mydaw.clips import ToneClip


def test_engine_renders_and_clips():
    sr = 44100
    engine = OfflineEngine(sample_rate=sr)
    timeline = Timeline()
    # Intentionally large amplitude to test clip; timeline will clip at engine stage as well
    timeline.add_clip(ToneClip(frequency_hz=1000.0, duration_seconds=0.05, amplitude=2.0), start_seconds=0.0)
    out = engine.render([timeline], length_seconds=0.05)
    assert out.shape == (int(0.05 * sr),)
    assert np.max(out) <= 1.0 + 1e-6
    assert np.min(out) >= -1.0 - 1e-6


