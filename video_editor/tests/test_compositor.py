import os

import pytest

from video_editor.compositor import project_to_segments, render_project
from video_editor.models import Project


def make_color_video(path: str, color: str, duration: float = 1.0, size: str = "160x120", fps: int = 25) -> None:
    # Use mpeg4 for broad compatibility; most ffmpeg builds provide it.
    cmd = (
        f"ffmpeg -y -f lavfi -i color=c={color}:s={size}:d={duration} -r {fps} -c:v mpeg4 -q:v 5 -pix_fmt yuv420p {path}"
    )
    code = os.system(cmd + " > /dev/null 2>&1")
    assert code == 0 and os.path.exists(path)


def test_project_to_segments_simple(tmp_path):
    red = tmp_path / "red.mp4"
    blue = tmp_path / "blue.mp4"
    make_color_video(str(red), "red", duration=2.0)
    make_color_video(str(blue), "blue", duration=2.0)

    p = Project()
    s_red = p.add_source(str(red))
    s_blue = p.add_source(str(blue))

    # Two clips with overlap; blue on higher track should win over red during overlap
    c1 = p.add_clip(s_red.id, 0.0, 2.0, 0.0, 0)
    c2 = p.add_clip(s_blue.id, 0.0, 2.0, 0.5, 1)

    segments = project_to_segments(p)
    total = sum(seg.end_seconds - seg.start_seconds for seg in segments)
    assert total == pytest.approx(p.get_timeline_duration(), abs=0.15)


def test_render_project(tmp_path):
    red = tmp_path / "red.mp4"
    green = tmp_path / "green.mp4"
    make_color_video(str(red), "red", duration=1.0)
    make_color_video(str(green), "green", duration=1.0)

    p = Project()
    s_red = p.add_source(str(red))
    s_green = p.add_source(str(green))
    p.add_clip(s_red.id, 0.0, s_red.duration, 0.0, 0)
    p.add_clip(s_green.id, 0.0, s_green.duration, 1.0, 0)

    out = tmp_path / "out.mp4"
    render_project(p, str(out))
    assert out.exists() and out.stat().st_size > 0


