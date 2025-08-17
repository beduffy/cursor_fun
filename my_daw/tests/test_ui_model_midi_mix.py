import numpy as np

from mydaw.tempo_transport import Transport
from mydaw.midi_clip import MidiNoteClip
from mydaw.synthesis import ADSR
from mydaw.timeline import Timeline
from mydaw.stereo import render_tracks_stereo


def test_midi_clip_mixed_with_timeline_tracks_stereo():
    sr = 44100
    tr = Transport(bpm=120.0)
    midi = MidiNoteClip(tr, adsr=ADSR(attack=0.0, decay=0.0, sustain=1.0, release=0.0))
    # Quarter notes A4 over 1 bar
    for i in range(4):
        midi.add_note(i * 1.0, 0.5, 69)

    tl = Timeline()
    tl.add_clip(midi, 0.0)

    stereo = render_tracks_stereo([tl], sr, int(2.0 * sr), pans=[0.0])
    assert stereo.shape[0] == 2
    assert np.any(np.abs(stereo) > 1e-6)


