# my_daw — Rebuilding Ableton Live, bit by bit

This project is a clean-room, incremental rebuild of core Ableton Live workflows. We will progress in focused milestones, each with documentation and tests. The code is written to be small, readable, and testable.

## Guiding principles
- Minimal, orthogonal building blocks
- Tests first; stable APIs grow from tests
- Small, iterative milestones; shippable at each step
- Pure Python and standard libs where practical; light deps

## Milestone 0 — Minimal audio engine (this commit)
- Mono engine at 44.1 kHz
- `ToneClip` (sine) placed on a `Timeline`
- Offline render to WAV (16-bit PCM)
- Unit tests for clips, timeline assembly, and render

Run the example to generate an audio file:

```bash
python -m mydaw.examples.minimal_example
```

Expected output: `my_daw/output_minimal.wav` (~5 seconds long)

## Upcoming milestones
- Tracks, volume/pan, gain staging
- Basic `AudioClip` loader (16-bit PCM WAV)
- Fades, envelopes, crossfade
- Transport and simple scheduler
- Mixer: summing bus, peak meters, clipping indicators
- Tempo, bars/beats grid, quantization helpers
- MIDI note clip + simple synth (poly sine)
- Session vs Arrangement data models
- Non-destructive editing operations
- Realtime audio backend (optional)

## Project layout
```
my_daw/
  README.md
  requirements.txt
  mydaw/
    __init__.py
    engine.py
    timeline.py
    clips.py
    tracks.py
    envelopes.py
    audioclip.py
    tempo_transport.py
    midi_clip.py
    synthesis.py
    drums.py
    examples/
      minimal_example.py
      mixer_example.py
      stereo_example.py
    ui/
      app.py
      timeline_view.py
      step_sequencer.py
  tests/
    test_clips.py
    test_timeline.py
    test_engine.py
    test_tracks_and_mixer.py
    test_envelopes.py
    test_audioclip.py
```

## Development
- Install: `pip install -r requirements.txt`
- Test: `pytest -q my_daw/tests`
- Style: idiomatic Python, explicit, readable

## Visual timeline UI (experimental)
- Requires PyQt5: `pip install --user PyQt5`
- Run:
```bash
PYTHONPATH=/home/ben/all_projects/cursor_fun/my_daw python3 -m mydaw.ui.app
```
- Features:
  - Add tone clips with frequency and duration
  - Add audio clips by browsing a WAV file
  - Drag clips left/right to change start time
  - Export the current mix as WAV
  - Step sequencer dock (3 lanes: kick/snare/hihat) synced to 120 BPM transport
  - Piano Roll dock for MIDI notes: click to toggle notes after "Add MIDI Track"
  - Planned: MIDI keyboard input, meters and pan controls per track

## Roadmap (high-level)
- Transport/tempo and time grid (in progress)
- MIDI note clips with piano roll UI and poly synth (sine now, more later)
- Drum rack with sample pads and step sequencer (basic click/kick prototype done)
- Track mixer: volume, pan (constant power), mute/solo, meters
- Clip envelopes: gain, fade in/out, curve shaping
- Arranger vs Session view models; scene triggering
- Realtime audio backend and clickless scheduling
- File I/O: projects, clips, automation

## License
MIT


