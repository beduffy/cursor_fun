from __future__ import annotations

import wave
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np

from .timeline import Clip


def _read_wav_mono_16bit(path: str) -> tuple[np.ndarray, int]:
    with wave.open(path, "rb") as wf:
        channels = wf.getnchannels()
        sampwidth = wf.getsampwidth()
        fr = wf.getframerate()
        frames = wf.getnframes()
        data = wf.readframes(frames)
    if sampwidth != 2:
        raise ValueError("Only 16-bit PCM WAV supported")
    pcm = np.frombuffer(data, dtype=np.int16)
    if channels == 2:
        pcm = pcm.reshape(-1, 2).mean(axis=1).astype(np.int16)
    elif channels != 1:
        raise ValueError("Unsupported channel count: {channels}")
    # Normalize to [-1, 1]
    return (pcm.astype(np.float32) / 32767.0, fr)


@dataclass
class AudioClip(Clip):
    path: str
    amplitude: float = 1.0
    _cache: Optional[tuple[np.ndarray, int]] = None

    def _ensure(self) -> tuple[np.ndarray, int]:
        if self._cache is None:
            samples, sr = _read_wav_mono_16bit(self.path)
            self._cache = (samples, sr)
        return self._cache

    def render(self, sample_rate: int) -> np.ndarray:
        samples, sr = self._ensure()
        if sr != sample_rate:
            # simple nearest-neighbor resample for Milestone 0
            ratio = float(sample_rate) / float(sr)
            idx = (np.arange(int(round(samples.shape[0] * ratio))) / ratio).astype(np.int32)
            idx = np.clip(idx, 0, samples.shape[0] - 1)
            resampled = samples[idx]
        else:
            resampled = samples
        return (resampled * float(self.amplitude)).astype(np.float32)


