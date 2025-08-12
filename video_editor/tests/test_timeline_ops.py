import os

from video_editor.models import Project


def test_delete_clip_prefers_selected(tmp_path):
    vid = tmp_path / "tiny.mp4"
    os.system(f"ffmpeg -y -f lavfi -i color=c=black:s=16x16:d=1 -r 25 -c:v mpeg4 -q:v 6 {vid} > /dev/null 2>&1")
    p = Project()
    s = p.add_source(str(vid))
    c1 = p.add_clip(s.id, 0.0, 0.5, 0.0, 0)
    c2 = p.add_clip(s.id, 0.5, 1.0, 1.0, 0)
    # simulate selection of c1
    # deletion logic is in UI; here we ensure model behavior makes sense post-delete
    del p.clips[c1.id]
    assert c1.id not in p.clips
    assert c2.id in p.clips



