from video_editor.models import Project


def test_track_locking():
    p = Project()
    p.set_track_count(4)
    assert p.is_track_locked(1) is False
    p.toggle_track_lock(1)
    assert p.is_track_locked(1) is True
    p.toggle_track_lock(1)
    assert p.is_track_locked(1) is False


def test_track_mute_visible():
    p = Project()
    p.set_track_count(3)
    assert p.track_mute[2] is False and p.track_visible[2] is True
    p.toggle_track_mute(2)
    p.toggle_track_visible(2)
    assert p.track_mute[2] is True and p.track_visible[2] is False


