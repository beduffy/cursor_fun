from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, List

import numpy as np
import wave


DEFAULT_SAMPLE_RATE = 44100


def write_wav_16bit(path: str, samples: np.ndarray, sample_rate: int = DEFAULT_SAMPLE_RATE) -> None:
    """Write float32 samples to 16-bit PCM WAV.

    Accepts mono shape (N,) or stereo shape (2, N).
    """
    if samples.ndim == 1:
        channels = 1
        data = samples
    elif samples.ndim == 2 and samples.shape[0] == 2:
        channels = 2
        data = samples.T.reshape(-1)
    else:
        raise ValueError("Expected mono (N,) or stereo (2,N) array")
    clipped = np.clip(data, -1.0, 1.0)
    int16 = (clipped * 32767.0).astype(np.int16)
    with wave.open(path, "wb") as wf:
        wf.setnchannels(channels)
        wf.setsampwidth(2)  # 16-bit
        wf.setframerate(sample_rate)
        wf.writeframes(int16.tobytes())


@dataclass
class RenderRegion:
    start_sample: int
    num_samples: int


class OfflineEngine:
    """Very small offline rendering engine.

    Responsibilities:
    - Collect sample contributions from a set of producers (clips/tracks)
    - Sum into a mono buffer
    - Return float32 samples in [-1, 1]
    """

    def __init__(self, sample_rate: int = DEFAULT_SAMPLE_RATE):
        self.sample_rate = int(sample_rate)

    def render(self, generators: Iterable["SampleGenerator"], length_seconds: float) -> np.ndarray:
        total_samples = int(round(length_seconds * self.sample_rate))
        mix = np.zeros(total_samples, dtype=np.float32)
        for gen in generators:
            contribution = gen.generate(self.sample_rate, total_samples)
            if contribution.shape != mix.shape:
                raise ValueError("Generator returned wrong shape")
            mix += contribution
        # Simple hard clip for now
        mix = np.clip(mix, -1.0, 1.0)
        return mix


class SampleGenerator:
    """Interface for anything that can produce samples for a render window."""

    def generate(self, sample_rate: int, total_samples: int) -> np.ndarray:  # pragma: no cover - documented interface
        raise NotImplementedError


