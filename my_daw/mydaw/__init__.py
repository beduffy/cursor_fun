"""my_daw minimal core package.

Milestone 0 exposes a tiny offline engine, a simple timeline, and a tone clip.
"""

from .engine import OfflineEngine, write_wav_16bit
from .timeline import Timeline
from .clips import ToneClip
from .tracks import Track, Mixer
from .envelopes import GainPoint, render_gain_envelope
from .audioclip import AudioClip
from .tempo_transport import Transport
from .midi_clip import MidiNoteClip
from .synthesis import ADSR
from .drums import DrumRack
from .stereo import StereoTrack, StereoMixer, constant_power_pan

__all__ = [
  "OfflineEngine",
  "write_wav_16bit",
  "Timeline",
  "ToneClip",
  "Track",
  "Mixer",
  "GainPoint",
  "render_gain_envelope",
  "AudioClip",
  "Transport",
  "MidiNoteClip",
  "ADSR",
  "DrumRack",
  "StereoTrack",
  "StereoMixer",
  "constant_power_pan",
]


