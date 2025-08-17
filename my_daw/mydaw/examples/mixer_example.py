from __future__ import annotations

from pathlib import Path

from mydaw import (
  OfflineEngine,
  write_wav_16bit,
  Track,
  Mixer,
  ToneClip,
  AudioClip,
)


def main() -> None:
    engine = OfflineEngine(sample_rate=44100)

    t1 = Track(gain=0.7)
    t1.add_clip(ToneClip(220.0, 1.0, 0.6), 0.0)
    t1.add_clip(ToneClip(330.0, 1.0, 0.6), 1.0)

    t2 = Track(gain=0.5)
    t2.add_clip(ToneClip(440.0, 2.0, 0.5), 0.5)

    mixer = Mixer(bus_gain=0.9)
    mixer.add_track(t1)
    mixer.add_track(t2)

    length_seconds = 3.0
    samples = engine.render([mixer], length_seconds)

    out_dir = Path(__file__).resolve().parents[2]
    out_path = out_dir / "output_mixer.wav"
    write_wav_16bit(str(out_path), samples, engine.sample_rate)
    print(f"Wrote {out_path}")


if __name__ == "__main__":
    main()


