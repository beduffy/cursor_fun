import json
import os
from video_editor.models import Project


def test_project_save_and_load(tmp_path):
    # create a tiny real video
    vid = tmp_path / "tiny.mp4"
    os.system(f"ffmpeg -y -f lavfi -i color=c=black:s=16x16:d=1 -r 25 -c:v mpeg4 -q:v 6 {vid} > /dev/null 2>&1")
    p = Project()
    s = p.add_source(str(vid))
    c = p.add_clip(s.id, 0.2, 0.8, 0.0, 0)
    p.save_project(tmp_path / "proj.json")

    p2 = Project.load_project(tmp_path / "proj.json")
    assert p2.next_source_id == p.next_source_id
    assert p2.next_clip_id == p.next_clip_id
    assert len(p2.sources) == 1
    assert len(p2.clips) == 1
    loaded = next(iter(p2.clips.values()))
    assert loaded.source_in == 0.2
    assert loaded.source_out == 0.8

