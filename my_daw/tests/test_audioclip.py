import os
import wave
import numpy as np

from mydaw.engine import write_wav_16bit
from mydaw.audioclip import AudioClip


def test_audioclip_load_and_render_resample(tmp_path):
    # Create a short 22.05kHz tone wav
    sr_src = 22050
    t = np.arange(int(0.1 * sr_src), dtype=np.float32) / sr_src
    samples = (0.5 * np.sin(2 * np.pi * 440.0 * t)).astype(np.float32)
    wav_path = tmp_path / "tone_22k.wav"
    write_wav_16bit(str(wav_path), samples, sample_rate=sr_src)

    clip = AudioClip(str(wav_path), amplitude=1.0)
    rendered = clip.render(44100)
    assert rendered.dtype == np.float32
    assert rendered.shape[0] > samples.shape[0]  # upsampled
    assert np.max(np.abs(rendered)) <= 1.0 + 1e-6


