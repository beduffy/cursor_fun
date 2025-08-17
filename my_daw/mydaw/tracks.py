from __future__ import annotations

from dataclasses import dataclass
from typing import List

import numpy as np

from .engine import SampleGenerator
from .timeline import Clip


@dataclass
class PlacedClip:
    clip: Clip
    start_seconds: float


class Track(SampleGenerator):
    """A simple track that holds clips and applies a gain."""

    def __init__(self, gain: float = 1.0) -> None:
        self.gain = float(gain)
        self._clips: List[PlacedClip] = []

    def add_clip(self, clip: Clip, start_seconds: float):
        if start_seconds < 0:
            raise ValueError("start_seconds must be >= 0")
        placed = PlacedClip(clip=clip, start_seconds=float(start_seconds))
        self._clips.append(placed)
        return placed

    def generate(self, sample_rate: int, total_samples: int) -> np.ndarray:
        mix = np.zeros(total_samples, dtype=np.float32)
        for placed in self._clips:
            start_sample = int(round(placed.start_seconds * sample_rate))
            clip_samples = placed.clip.render(sample_rate)
            end_sample = min(total_samples, start_sample + clip_samples.shape[0])
            if end_sample > start_sample:
                span = end_sample - start_sample
                mix[start_sample:end_sample] += clip_samples[:span]
        return mix * self.gain


class Mixer(SampleGenerator):
    """A minimal mixer that sums tracks and applies a bus gain."""

    def __init__(self, bus_gain: float = 1.0) -> None:
        self.bus_gain = float(bus_gain)
        self._tracks: List[Track] = []

    def add_track(self, track: Track) -> None:
        self._tracks.append(track)

    def generate(self, sample_rate: int, total_samples: int) -> np.ndarray:
        mix = np.zeros(total_samples, dtype=np.float32)
        for track in self._tracks:
            mix += track.generate(sample_rate, total_samples)
        return mix * self.bus_gain


