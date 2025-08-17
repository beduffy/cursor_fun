from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List

import numpy as np

from .engine import SampleGenerator
from .timeline import Clip


@dataclass
class PlacedClip:
    clip: Clip
    start_seconds: float


def constant_power_pan(mono: np.ndarray, pan: float) -> np.ndarray:
    """Pan mono signal to stereo using constant power (-1 left .. +1 right)."""
    pan = float(max(-1.0, min(1.0, pan)))
    # Map pan [-1,1] to angle [0, pi/2]
    angle = (pan + 1.0) * (math.pi / 4.0)
    left = math.cos(angle)
    right = math.sin(angle)
    stereo = np.stack([mono * left, mono * right], axis=0)
    return stereo.astype(np.float32)


class StereoTrack(SampleGenerator):
    def __init__(self, gain: float = 1.0, pan: float = 0.0) -> None:
        self.gain = float(gain)
        self.pan = float(pan)
        self._clips: List[PlacedClip] = []

    def add_clip(self, clip: Clip, start_seconds: float):
        if start_seconds < 0:
            raise ValueError("start_seconds must be >= 0")
        placed = PlacedClip(clip=clip, start_seconds=float(start_seconds))
        self._clips.append(placed)
        return placed

    def generate(self, sample_rate: int, total_samples: int) -> np.ndarray:
        mono = np.zeros(total_samples, dtype=np.float32)
        for placed in self._clips:
            start_sample = int(round(placed.start_seconds * sample_rate))
            clip_samples = placed.clip.render(sample_rate)
            end_sample = min(total_samples, start_sample + clip_samples.shape[0])
            if end_sample > start_sample:
                span = end_sample - start_sample
                mono[start_sample:end_sample] += clip_samples[:span]
        mono *= self.gain
        return constant_power_pan(mono, self.pan)


class StereoMixer(SampleGenerator):
    def __init__(self) -> None:
        self._tracks: List[StereoTrack] = []

    def add_track(self, track: StereoTrack) -> None:
        self._tracks.append(track)

    def generate(self, sample_rate: int, total_samples: int) -> np.ndarray:
        # shape: (2, N)
        mix = np.zeros((2, total_samples), dtype=np.float32)
        for track in self._tracks:
            mix += track.generate(sample_rate, total_samples)
        # clip per channel
        np.clip(mix, -1.0, 1.0, out=mix)
        return mix


