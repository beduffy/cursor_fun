from mydaw.timeline import Timeline
from mydaw.clips import ToneClip


def test_tone_clip_resize_affects_duration_render():
    sr = 44100
    tl = Timeline()
    clip = ToneClip(440.0, 0.1, 0.2)
    tl.add_clip(clip, 0.0)
    out1 = tl.generate(sr, int(0.3 * sr))
    # Extend duration
    clip.duration_seconds = 0.2
    out2 = tl.generate(sr, int(0.3 * sr))
    assert out2.shape == out1.shape
    # More non-zero samples in extended version
    assert (out2 != 0.0).sum() > (out1 != 0.0).sum()


def test_duplicate_right_positioning():
    sr = 44100
    tl = Timeline()
    c = ToneClip(100.0, 0.2, 0.2)
    tl.add_clip(c, 0.5)
    # Simulate duplicate right by adding another clip at start+duration
    tl.add_clip(ToneClip(100.0, 0.2, 0.2), 0.5 + 0.2)
    out = tl.generate(sr, int(1.0 * sr))
    start1 = int(0.5 * sr)
    start2 = int((0.5 + 0.2) * sr)
    # Check a small window after the starts to avoid zero-crossing at exact start
    assert (out[start1 : start1 + 100] != 0.0).any()
    assert (out[start2 : start2 + 100] != 0.0).any()


