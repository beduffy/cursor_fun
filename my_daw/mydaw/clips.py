from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .timeline import Clip


@dataclass
class ToneClip(Clip):
    """Sine tone clip of fixed duration.

    - frequency_hz: sine frequency
    - duration_seconds: length of clip
    - amplitude: linear amplitude (0..1)
    """

    frequency_hz: float
    duration_seconds: float
    amplitude: float = 0.2

    def render(self, sample_rate: int) -> np.ndarray:
        total_samples = int(round(self.duration_seconds * sample_rate))
        t = np.arange(total_samples, dtype=np.float32) / float(sample_rate)
        phase = 2.0 * math.pi * self.frequency_hz * t
        samples = np.sin(phase).astype(np.float32) * float(self.amplitude)
        return samples


