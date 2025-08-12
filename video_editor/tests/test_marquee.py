from video_editor.timeline import TimelineView
from video_editor.models import Project


def test_marquee_state_defaults(qapp):
    p = Project()
    view = TimelineView(p)
    assert getattr(view, '_is_marquee', False) is False
    assert getattr(view, '_marquee_rect', None) is None

