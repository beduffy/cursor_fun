from mydaw.tracks import Track
from mydaw.clips import ToneClip
from mydaw.midi_clip import MidiNoteClip
from mydaw.tempo_transport import Transport


def test_remove_placed_clip():
    tr = Track()
    placed = tr.add_clip(ToneClip(220.0, 0.1, 0.2), 0.0)
    assert len(tr._clips) == 1
    tr.remove_placed(placed)
    assert len(tr._clips) == 0


def test_midi_quantize():
    tr = Transport(bpm=120.0)
    clip = MidiNoteClip(tr)
    clip.add_note(0.12, 0.5, 60)
    clip.add_note(0.49, 0.5, 64)
    clip.quantize(0.25)
    starts = [n.start_beats for n in clip.notes]
    assert starts == [0.0, 0.5]


