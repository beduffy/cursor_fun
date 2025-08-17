import numpy as np

from mydaw.tempo_transport import Transport
from mydaw.midi_clip import MidiNoteClip
from mydaw.synthesis import ADSR


def test_midi_note_clip_renders_with_transport():
    tr = Transport(bpm=120.0)
    midi = MidiNoteClip(tr, adsr=ADSR(attack=0.0, decay=0.0, sustain=1.0, release=0.0))
    midi.add_note(0.0, 1.0, 69)  # A4, 1 beat at 120bpm => 0.5s

    sr = 44100
    samples = midi.render(sr)
    assert samples.shape[0] == int(tr.beats_to_seconds(1.0) * sr)
    assert np.any(samples != 0.0)


