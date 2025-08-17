import numpy as np

from mydaw.quantize import quantize_beats
from mydaw.tempo_transport import Transport
from mydaw.midi_clip import MidiNoteClip
from mydaw.synthesis import ADSR


def test_quantize_beats_rounding():
    vals = [0.12, 0.26, 0.49, 0.51]
    out = quantize_beats(vals, 0.25)
    assert out == [0.0, 0.25, 0.5, 0.5]


def test_velocity_affects_amplitude():
    tr = Transport(bpm=120.0)
    midi = MidiNoteClip(tr, adsr=ADSR(attack=0.0, decay=0.0, sustain=1.0, release=0.0))
    midi.add_note(0.0, 1.0, 69, velocity=0.2)
    low = midi.render(44100)
    midi2 = MidiNoteClip(tr, adsr=ADSR(attack=0.0, decay=0.0, sustain=1.0, release=0.0))
    midi2.add_note(0.0, 1.0, 69, velocity=1.0)
    high = midi2.render(44100)
    assert low.shape == high.shape
    assert np.max(np.abs(high)) > np.max(np.abs(low)) * 2.5


