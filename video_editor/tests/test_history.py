import os

from video_editor.models import Project, ProjectHistory


def test_undo_redo(tmp_path):
    vid = tmp_path / "tiny.mp4"
    os.system(f"ffmpeg -y -f lavfi -i color=c=black:s=16x16:d=1 -r 25 -c:v mpeg4 -q:v 6 {vid} > /dev/null 2>&1")
    p = Project()
    h = ProjectHistory()
    h.push(p)

    s = p.add_source(str(vid))
    c = p.add_clip(s.id, 0.0, 1.0, 0.0, 0)
    h.push(p)
    assert len(p.clips) == 1
    p.remove_clip(c.id)
    h.push(p)
    assert len(p.clips) == 0
    assert h.undo(p) is True
    assert len(p.clips) == 1
    assert h.redo(p) is True
    assert len(p.clips) == 0


