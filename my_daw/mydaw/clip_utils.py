from __future__ import annotations

import copy

from .clips import ToneClip
from .audioclip import AudioClip
from .midi_clip import MidiNoteClip, Note


def clone_clip(clip):
    """Create a deep-ish copy of supported clip types."""
    if isinstance(clip, ToneClip):
        return ToneClip(clip.frequency_hz, clip.duration_seconds, clip.amplitude)
    if isinstance(clip, AudioClip):
        return AudioClip(clip.path, amplitude=clip.amplitude)
    if isinstance(clip, MidiNoteClip):
        new_clip = MidiNoteClip(clip.transport, adsr=copy.deepcopy(clip.adsr))
        for n in clip.notes:
            new_clip.add_note(n.start_beats, n.duration_beats, n.midi_note, n.velocity)
        return new_clip
    raise TypeError(f"Unsupported clip type for cloning: {type(clip)}")


