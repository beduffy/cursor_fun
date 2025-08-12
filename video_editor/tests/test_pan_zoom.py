from video_editor.timeline import TimelineView
from video_editor.models import Project


def test_pan_and_zoom_properties(qapp):
    p = Project()
    view = TimelineView(p)
    assert view.seconds_per_pixel > 0
    old = view.seconds_per_pixel
    view.seconds_per_pixel = old * 0.9
    assert view.seconds_per_pixel != old
    view.view_offset_seconds = 1.23
    assert view.view_offset_seconds == 1.23

