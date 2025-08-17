from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence, Tuple

import numpy as np


@dataclass(frozen=True)
class GainPoint:
    time_seconds: float
    linear_gain: float


def render_gain_envelope(points: Sequence[GainPoint], sample_rate: int, total_samples: int) -> np.ndarray:
    """Render a piecewise linear gain envelope.

    If no points, returns ones. Points outside [0, total_length] are ignored.
    The envelope is clamped to [0, +inf) but not clipped after multiplication.
    """
    if total_samples <= 0:
        return np.zeros(0, dtype=np.float32)
    if not points:
        return np.ones(total_samples, dtype=np.float32)

    # Filter and sort points
    total_length = total_samples / float(sample_rate)
    pts = [p for p in points if 0.0 <= p.time_seconds <= total_length]
    if not pts:
        return np.ones(total_samples, dtype=np.float32)
    pts.sort(key=lambda p: p.time_seconds)

    env = np.ones(total_samples, dtype=np.float32)
    # Fill before first point
    first_idx = int(round(pts[0].time_seconds * sample_rate))
    env[:first_idx] = float(max(0.0, pts[0].linear_gain))

    # Segments
    for a, b in zip(pts[:-1], pts[1:]):
        start = int(round(a.time_seconds * sample_rate))
        end = int(round(b.time_seconds * sample_rate))
        if end <= start:
            continue
        t = np.linspace(0.0, 1.0, end - start, endpoint=False, dtype=np.float32)
        segment = (1.0 - t) * float(max(0.0, a.linear_gain)) + t * float(max(0.0, b.linear_gain))
        env[start:end] = segment

    # After last point
    last_idx = int(round(pts[-1].time_seconds * sample_rate))
    env[last_idx:] = float(max(0.0, pts[-1].linear_gain))
    return env


