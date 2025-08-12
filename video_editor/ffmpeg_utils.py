import os
import shlex
import subprocess
import tempfile
from dataclasses import dataclass
from typing import List, Tuple


@dataclass
class EditSegment:
    source_path: str
    start_seconds: float
    end_seconds: float



def run_command(command: str) -> Tuple[int, str, str]:
    process = subprocess.Popen(
        shlex.split(command), stdout=subprocess.PIPE, stderr=subprocess.PIPE
    )
    stdout, stderr = process.communicate()
    return process.returncode, stdout.decode("utf-8", errors="ignore"), stderr.decode("utf-8", errors="ignore")



def ensure_parent_dir(path: str) -> None:
    parent_dir = os.path.dirname(os.path.abspath(path))
    if parent_dir and not os.path.exists(parent_dir):
        os.makedirs(parent_dir, exist_ok=True)



def export_edit(segments: List[EditSegment], output_path: str) -> None:
    """Export the edit by cutting each segment then concatenating via ffmpeg.

    For robustness, we re-encode each segment to H.264/AAC, then concat via
    demuxer to ensure consistent codecs and containers.
    """
    if not segments:
        raise ValueError("No segments to export")

    ensure_parent_dir(output_path)

    temp_dir = tempfile.mkdtemp(prefix="video_editor_segments_")
    temp_segment_paths: List[str] = []

    try:
        # 1) Create a re-encoded file per segment
        for index, segment in enumerate(segments):
            seg_out = os.path.join(temp_dir, f"seg_{index:04d}.mp4")
            duration = max(0.0, segment.end_seconds - segment.start_seconds)
            if duration <= 0:
                continue

            # Use accurate seeking by placing -ss before input for keyframe seek
            # and re-encode to ensure stable concat.
            cmd = (
                f"ffmpeg -y -ss {segment.start_seconds:.3f} -i {shlex.quote(segment.source_path)} "
                f"-t {duration:.3f} -c:v libx264 -preset fast -crf 18 -c:a aac -b:a 192k "
                f"-movflags +faststart {shlex.quote(seg_out)}"
            )
            code, out, err = run_command(cmd)
            if code != 0 or not os.path.exists(seg_out):
                raise RuntimeError(f"ffmpeg segment export failed for {segment.source_path}: {err}")
            temp_segment_paths.append(seg_out)

        if not temp_segment_paths:
            raise ValueError("All segments were empty or invalid")

        # 2) Concat segments using demuxer
        concat_list_path = os.path.join(temp_dir, "concat_list.txt")
        with open(concat_list_path, "w", encoding="utf-8") as f:
            for p in temp_segment_paths:
                f.write(f"file {shlex.quote(p)}\n")

        cmd_concat = (
            f"ffmpeg -y -f concat -safe 0 -i {shlex.quote(concat_list_path)} "
            f"-c copy {shlex.quote(output_path)}"
        )
        code, out, err = run_command(cmd_concat)
        if code != 0 or not os.path.exists(output_path):
            # Fallback: re-encode concat if stream copy fails for any reason
            cmd_concat_reencode = (
                f"ffmpeg -y -f concat -safe 0 -i {shlex.quote(concat_list_path)} "
                f"-c:v libx264 -preset fast -crf 18 -c:a aac -b:a 192k -movflags +faststart "
                f"{shlex.quote(output_path)}"
            )
            code2, out2, err2 = run_command(cmd_concat_reencode)
            if code2 != 0 or not os.path.exists(output_path):
                raise RuntimeError(f"ffmpeg concat failed: {err}\nFallback err: {err2}")
    finally:
        # Best effort cleanup of temp_dir
        try:
            for p in temp_segment_paths:
                if os.path.exists(p):
                    os.remove(p)
            concat_list = os.path.join(temp_dir, "concat_list.txt")
            if os.path.exists(concat_list):
                os.remove(concat_list)
            if os.path.isdir(temp_dir):
                os.rmdir(temp_dir)
        except Exception:
            pass


