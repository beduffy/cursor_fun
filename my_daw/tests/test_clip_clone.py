from mydaw.clip_utils import clone_clip
from mydaw.clips import ToneClip
from mydaw.audioclip import AudioClip
from mydaw.midi_clip import MidiNoteClip
from mydaw.tempo_transport import Transport


def test_clone_tone_and_audio():
    t = ToneClip(440.0, 1.0, 0.2)
    c = clone_clip(t)
    assert isinstance(c, ToneClip) and c.frequency_hz == 440.0

    a = AudioClip("/tmp/test.wav", amplitude=0.9)
    c2 = clone_clip(a)
    assert isinstance(c2, AudioClip) and c2.path == a.path and c2.amplitude == a.amplitude


def test_clone_midi():
    tr = Transport(bpm=120.0)
    m = MidiNoteClip(tr)
    m.add_note(0.0, 1.0, 60)
    m.add_note(1.0, 1.0, 64)
    c = clone_clip(m)
    assert isinstance(c, MidiNoteClip) and len(c.notes) == 2


