from video_editor.models import Project


def test_track_locking():
    p = Project()
    p.set_track_count(4)
    assert p.is_track_locked(1) is False
    p.toggle_track_lock(1)
    assert p.is_track_locked(1) is True
    p.toggle_track_lock(1)
    assert p.is_track_locked(1) is False


