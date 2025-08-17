from __future__ import annotations

import numpy as np


def peak_meter_mono(samples: np.ndarray) -> float:
    if samples.size == 0:
        return 0.0
    return float(np.max(np.abs(samples)))


def peak_meter_stereo(samples_stereo: np.ndarray) -> tuple[float, float]:
    if samples_stereo.size == 0:
        return 0.0, 0.0
    return float(np.max(np.abs(samples_stereo[0]))), float(np.max(np.abs(samples_stereo[1])))


