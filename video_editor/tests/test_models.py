import os

import pytest

from video_editor.models import Project


def test_project_add_source_and_clip(tmp_path):
    # Make a tiny synthetic file using ffmpeg (1s color)
    out = tmp_path / "red.mp4"
    cmd = (
        f"ffmpeg -y -f lavfi -i color=c=red:s=160x120:d=1 -r 25 -c:v mpeg4 -q:v 5 -pix_fmt yuv420p {out}"
    )
    code = os.system(cmd + " > /dev/null 2>&1")
    assert code == 0 and out.exists()

    p = Project()
    src = p.add_source(str(out))
    assert src.id == 1
    assert src.duration > 0

    clip = p.add_clip(src.id, 0.2, 0.8, 0.0, 0)
    assert clip.id == 1
    assert p.get_timeline_duration() == pytest.approx(0.6, abs=0.05)


