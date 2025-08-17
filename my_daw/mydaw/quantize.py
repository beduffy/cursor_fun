from __future__ import annotations

from typing import Iterable, List


def quantize_beats(values: Iterable[float], grid: float) -> List[float]:
    """Quantize beat positions to nearest multiple of grid (beats)."""
    out: List[float] = []
    for v in values:
        q = round(v / grid) * grid
        out.append(q)
    return out


