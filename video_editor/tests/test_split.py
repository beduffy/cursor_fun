import os

from video_editor.models import Project


def test_split_clip(tmp_path):
    # make a tiny clip
    vid = tmp_path / "tiny.mp4"
    os.system(f"ffmpeg -y -f lavfi -i color=c=black:s=16x16:d=1 -r 25 -c:v mpeg4 -q:v 6 {vid} > /dev/null 2>&1")
    p = Project()
    s = p.add_source(str(vid))
    c = p.add_clip(s.id, 0.0, 1.0, 0.0, 0)
    new_c = p.split_clip(c.id, 0.4)
    assert new_c is not None
    assert c.source_in == 0.0
    assert c.source_out == 0.4
    assert new_c.source_in == 0.4
    assert new_c.source_out == 1.0
    assert new_c.timeline_start == 0.4


