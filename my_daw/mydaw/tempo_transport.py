from __future__ import annotations

from dataclasses import dataclass


@dataclass
class Transport:
    bpm: float = 120.0
    time_sig_numerator: int = 4
    time_sig_denominator: int = 4

    def seconds_per_beat(self) -> float:
        return 60.0 / float(self.bpm)

    def beats_to_seconds(self, beats: float) -> float:
        return beats * self.seconds_per_beat()

    def seconds_to_beats(self, seconds: float) -> float:
        return seconds / self.seconds_per_beat()


