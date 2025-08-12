import os
from video_editor.models import Project


def test_duplicate_clip(tmp_path):
    vid = tmp_path / "tiny.mp4"
    os.system(f"ffmpeg -y -f lavfi -i color=c=black:s=16x16:d=1 -r 25 -c:v mpeg4 -q:v 6 {vid} > /dev/null 2>&1")
    p = Project()
    s = p.add_source(str(vid))
    c = p.add_clip(s.id, 0.1, 0.6, 0.2, 0)
    d = p.duplicate_clip(c.id)
    assert d is not None
    assert d.id != c.id
    assert d.source_in == c.source_in and d.source_out == c.source_out
    assert d.timeline_start == c.timeline_start and d.track_index == c.track_index

