from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, List

import numpy as np
import wave


DEFAULT_SAMPLE_RATE = 44100


def write_wav_16bit(path: str, samples: np.ndarray, sample_rate: int = DEFAULT_SAMPLE_RATE) -> None:
    """Write mono float32 samples in [-1, 1] to 16-bit PCM WAV.

    Clips to [-1, 1] and converts to int16. Accepts shape (num_samples,).
    """
    if samples.ndim != 1:
        raise ValueError("Expected mono 1D array for samples")
    clipped = np.clip(samples, -1.0, 1.0)
    int16 = (clipped * 32767.0).astype(np.int16)
    with wave.open(path, "wb") as wf:
        wf.setnchannels(1)
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


