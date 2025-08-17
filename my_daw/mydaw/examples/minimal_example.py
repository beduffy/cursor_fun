from __future__ import annotations

from pathlib import Path

from mydaw import OfflineEngine, Timeline, ToneClip, write_wav_16bit


def main() -> None:
    engine = OfflineEngine(sample_rate=44100)
    timeline = Timeline()

    # Place a few tones at different start times
    timeline.add_clip(ToneClip(frequency_hz=440.0, duration_seconds=1.5, amplitude=0.2), start_seconds=0.0)
    timeline.add_clip(ToneClip(frequency_hz=660.0, duration_seconds=1.5, amplitude=0.2), start_seconds=1.0)
    timeline.add_clip(ToneClip(frequency_hz=330.0, duration_seconds=1.5, amplitude=0.2), start_seconds=2.0)

    length_seconds = 5.0
    samples = engine.render([timeline], length_seconds=length_seconds)

    out_dir = Path(__file__).resolve().parents[2]
    out_path = out_dir / "output_minimal.wav"
    write_wav_16bit(str(out_path), samples, sample_rate=engine.sample_rate)
    print(f"Wrote {out_path}")


if __name__ == "__main__":
    main()


