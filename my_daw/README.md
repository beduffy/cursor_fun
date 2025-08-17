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
    examples/
      minimal_example.py
  tests/
    test_clips.py
    test_timeline.py
    test_engine.py
```

## Development
- Install: `pip install -r requirements.txt`
- Test: `pytest -q my_daw/tests`
- Style: idiomatic Python, explicit, readable

## License
MIT


