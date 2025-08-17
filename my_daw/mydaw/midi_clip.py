from __future__ import annotations

from dataclasses import dataclass
from typing import List

import numpy as np

from .timeline import Clip
from .tempo_transport import Transport
from .synthesis import ADSR, render_poly_sine


@dataclass
class Note:
    start_beats: float
    duration_beats: float
    midi_note: int
    velocity: float = 1.0

    def frequency_hz(self) -> float:
        return 440.0 * (2.0 ** ((self.midi_note - 69) / 12.0))


class MidiNoteClip(Clip):
    def __init__(self, transport: Transport, adsr: ADSR | None = None) -> None:
        self.transport = transport
        self.adsr = adsr or ADSR()
        self.notes: List[Note] = []

    def add_note(self, start_beats: float, duration_beats: float, midi_note: int, velocity: float = 1.0) -> None:
        self.notes.append(Note(start_beats, duration_beats, midi_note, velocity))

    def render(self, sample_rate: int) -> np.ndarray:
        if not self.notes:
            return np.zeros(0, dtype=np.float32)
        # Determine total seconds from notes
        end_beats = max(n.start_beats + n.duration_beats for n in self.notes)
        total_seconds = self.transport.beats_to_seconds(end_beats)
        rendered = render_poly_sine(
            sample_rate,
            [
                (
                    self.transport.beats_to_seconds(n.start_beats),
                    n.frequency_hz(),
                    self.transport.beats_to_seconds(n.duration_beats),
                    n.velocity,
                )
                for n in self.notes
            ],
            self.adsr,
            total_seconds,
        )
        return rendered


