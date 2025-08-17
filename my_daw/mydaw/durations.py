from __future__ import annotations

from typing import Optional

from .clips import ToneClip
from .audioclip import AudioClip
from .midi_clip import MidiNoteClip


def get_clip_duration_seconds(clip) -> float:
    """Best-effort duration in seconds for known clip types.

    - ToneClip: duration_seconds
    - MidiNoteClip: max(end_beats) via transport
    - AudioClip: unknown (returns 0.0)
    """
    if isinstance(clip, ToneClip):
        return float(clip.duration_seconds)
    if isinstance(clip, MidiNoteClip):
        if not clip.notes:
            return 0.0
        end_beats = max(n.start_beats + n.duration_beats for n in clip.notes)
        return float(clip.transport.beats_to_seconds(end_beats))
    if isinstance(clip, AudioClip):
        # Unknown without reading file; leave 0.0 so callers can set a default
        return 0.0
    return 0.0


