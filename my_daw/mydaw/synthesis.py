from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, List, Tuple

import numpy as np


@dataclass
class ADSR:
    attack: float = 0.01
    decay: float = 0.05
    sustain: float = 0.7
    release: float = 0.1

    def render(self, sample_rate: int, total_samples: int, note_on_samples: int, note_off_samples: int) -> np.ndarray:
        env = np.zeros(total_samples, dtype=np.float32)
        # Attack
        a = max(1, int(round(self.attack * sample_rate)))
        d = max(1, int(round(self.decay * sample_rate)))
        r = max(1, int(round(self.release * sample_rate)))

        on = note_on_samples
        off = note_off_samples
        # Build envelope around on/off
        # Attack
        if on < total_samples:
            a_end = min(total_samples, on + a)
            if a_end > on:
                env[on:a_end] = np.linspace(0.0, 1.0, a_end - on, endpoint=False, dtype=np.float32)
                level_at_a_end = 1.0
            else:
                level_at_a_end = 0.0
        else:
            level_at_a_end = 0.0

        # Decay
        d_start = min(total_samples, on + a)
        d_end = min(total_samples, d_start + d)
        if d_end > d_start:
            env[d_start:d_end] = np.linspace(level_at_a_end, self.sustain, d_end - d_start, endpoint=False, dtype=np.float32)

        # Sustain
        s_start = d_end
        s_end = min(total_samples, off)
        if s_end > s_start:
            env[s_start:s_end] = self.sustain

        # Release
        r_start = s_end
        r_end = min(total_samples, r_start + r)
        if r_end > r_start:
            env[r_start:r_end] = np.linspace(self.sustain, 0.0, r_end - r_start, endpoint=False, dtype=np.float32)
        return env


def render_poly_sine(sample_rate: int, notes: Iterable[Tuple[float, float, float]], adsr: ADSR, total_seconds: float) -> np.ndarray:
    """Render polyphonic sine given notes = [(start_s, freq_hz, dur_s), ...]."""
    total_samples = int(round(total_seconds * sample_rate))
    t = np.arange(total_samples, dtype=np.float32) / float(sample_rate)
    mix = np.zeros(total_samples, dtype=np.float32)
    for start_s, freq_hz, dur_s in notes:
        start_n = int(round(start_s * sample_rate))
        dur_n = int(round(dur_s * sample_rate))
        end_n = min(total_samples, start_n + dur_n)
        if end_n <= start_n:
            continue
        seg_t = (np.arange(end_n - start_n, dtype=np.float32) / float(sample_rate))
        phase = 2.0 * math.pi * float(freq_hz) * seg_t
        tone = np.sin(phase).astype(np.float32)
        env = adsr.render(sample_rate, end_n - start_n, 0, end_n - start_n)
        tone *= env
        mix[start_n:end_n] += tone
    return np.clip(mix, -1.0, 1.0)


