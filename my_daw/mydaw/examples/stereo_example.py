from __future__ import annotations

from pathlib import Path

from mydaw import write_wav_16bit, ToneClip
from mydaw.stereo import StereoTrack, StereoMixer


def main() -> None:
    sr = 44100
    left = StereoTrack(gain=0.7, pan=-0.8)
    right = StereoTrack(gain=0.7, pan=0.8)
    left.add_clip(ToneClip(330.0, 2.0, 0.4), 0.0)
    right.add_clip(ToneClip(550.0, 2.0, 0.4), 0.5)

    mixer = StereoMixer()
    mixer.add_track(left)
    mixer.add_track(right)

    total_samples = int(3.0 * sr)
    stereo = mixer.generate(sr, total_samples)
    out = Path(__file__).resolve().parents[2] / "output_stereo.wav"
    write_wav_16bit(str(out), stereo, sr)
    print(f"Wrote {out}")


if __name__ == "__main__":
    main()


