from __future__ import annotations

from dataclasses import dataclass
from typing import List

import numpy as np

from .engine import SampleGenerator


@dataclass
class PlacedClip:
    clip: "Clip"
    start_seconds: float


class Timeline(SampleGenerator):
    """A minimal timeline that mixes placed clips into a mono buffer."""

    def __init__(self) -> None:
        self._clips: List[PlacedClip] = []

    def add_clip(self, clip: "Clip", start_seconds: float) -> None:
        if start_seconds < 0:
            raise ValueError("start_seconds must be >= 0")
        self._clips.append(PlacedClip(clip=clip, start_seconds=float(start_seconds)))

    def generate(self, sample_rate: int, total_samples: int) -> np.ndarray:
        mix = np.zeros(total_samples, dtype=np.float32)
        for placed in self._clips:
            start_sample = int(round(placed.start_seconds * sample_rate))
            clip_samples = placed.clip.render(sample_rate)
            end_sample = min(total_samples, start_sample + clip_samples.shape[0])
            if end_sample > start_sample:
                span = end_sample - start_sample
                mix[start_sample:end_sample] += clip_samples[:span]
        return mix


class Clip:
    """Interface for clips that can be placed on a timeline."""

    def render(self, sample_rate: int) -> np.ndarray:  # pragma: no cover - documented interface
        raise NotImplementedError


