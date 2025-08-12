import os
from video_editor.models import Project


def test_bulk_delete(tmp_path):
    vid = tmp_path / "tiny.mp4"
    os.system(f"ffmpeg -y -f lavfi -i color=c=black:s=16x16:d=1 -r 25 -c:v mpeg4 -q:v 6 {vid} > /dev/null 2>&1")
    p = Project()
    s = p.add_source(str(vid))
    c1 = p.add_clip(s.id, 0.0, 0.2, 0.0, 0)
    c2 = p.add_clip(s.id, 0.2, 0.4, 0.3, 0)
    c3 = p.add_clip(s.id, 0.4, 0.6, 0.6, 0)
    p.remove_clips([c1.id, c3.id])
    assert c1.id not in p.clips and c3.id not in p.clips
    assert c2.id in p.clips


