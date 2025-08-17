from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Sequence

import numpy as np

from .timeline import Clip
from .tempo_transport import Transport


def _click(sample_rate: int, length_s: float = 0.05, amp: float = 0.8) -> np.ndarray:
    n = int(round(length_s * sample_rate))
    t = np.arange(n, dtype=np.float32) / float(sample_rate)
    env = np.exp(-t * 100.0).astype(np.float32)
    noise = (np.random.rand(n).astype(np.float32) * 2.0 - 1.0)
    return np.clip(noise * env * amp, -1.0, 1.0)


def _sine(sample_rate: int, freq: float, length_s: float = 0.08, amp: float = 0.8) -> np.ndarray:
    n = int(round(length_s * sample_rate))
    t = np.arange(n, dtype=np.float32) / float(sample_rate)
    env = np.exp(-t * 20.0).astype(np.float32)
    return (np.sin(2.0 * math.pi * freq * t) * env * amp).astype(np.float32)


@dataclass
class DrumStep:
    # For simplicity each step is on/off with instrument index
    beat: float
    instrument: int  # 0=kick,1=snare,2=hihat


class DrumRack(Clip):
    def __init__(self, transport: Transport) -> None:
        self.transport = transport
        self.steps: List[DrumStep] = []

    def add_step(self, beat: float, instrument: int) -> None:
        self.steps.append(DrumStep(beat=beat, instrument=instrument))

    def render(self, sample_rate: int) -> np.ndarray:
        if not self.steps:
            return np.zeros(0, dtype=np.float32)
        total_beats = max(s.beat for s in self.steps) + 1.0
        total_seconds = self.transport.beats_to_seconds(total_beats)
        total_samples = int(round(total_seconds * sample_rate))
        mix = np.zeros(total_samples, dtype=np.float32)
        for s in self.steps:
            start = int(round(self.transport.beats_to_seconds(s.beat) * sample_rate))
            if s.instrument == 0:
                sig = _sine(sample_rate, 60.0, 0.12, 0.95)
            elif s.instrument == 1:
                sig = _click(sample_rate, 0.08, 0.8)
            else:
                sig = _click(sample_rate, 0.03, 0.5)
            end = min(total_samples, start + sig.shape[0])
            mix[start:end] += sig[: end - start]
        return np.clip(mix, -1.0, 1.0)


