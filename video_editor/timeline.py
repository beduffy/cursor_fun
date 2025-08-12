from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QPainter, QPen
from PySide6.QtWidgets import QListWidget, QWidget

try:
    from .models import Clip, Project
except ImportError:  # Fallback for running as plain scripts
    from models import Clip, Project  # type: ignore


@dataclass
class HitTestResult:
    clip_id: Optional[int]
    edge: Optional[str]  # 'left', 'right', or None


class TimelineView(QWidget):
    def __init__(self, project: Project, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.project = project
        self.seconds_per_pixel = 0.02  # zoom level
        self.track_height = 44
        self.header_width = 60
        self.row_gap = 6
        self.setMouseTracking(True)
        self.drag_state = None  # ('move'|'trim_left'|'trim_right', clip_id, start_pos, start_time)

    
    def sizeHint(self):  # type: ignore[override]
        return super().sizeHint()

    
    def paintEvent(self, event):  # type: ignore[override]
        painter = QPainter(self)
        rect = self.rect()
        painter.fillRect(rect, QColor(30, 30, 30))

        # Tracks background
        for track in range(self.project.track_count):
            y = track * (self.track_height + self.row_gap)
            painter.fillRect(QRectF(0, y, rect.width(), self.track_height), QColor(40, 40, 40))

        # Grid lines
        pen = QPen(QColor(70, 70, 70))
        painter.setPen(pen)
        for x in range(0, rect.width(), 50):
            painter.drawLine(x, 0, x, rect.height())

        # Draw clips
        for clip in self.project.clips.values():
            x = int(clip.timeline_start / self.seconds_per_pixel)
            w = int((clip.source_out - clip.source_in) / self.seconds_per_pixel)
            y = clip.track_index * (self.track_height + self.row_gap)
            clip_rect = QRectF(x, y, max(6, w), self.track_height)
            painter.fillRect(clip_rect, QColor(80, 120, 200))
            painter.setPen(QPen(QColor(20, 20, 20)))
            painter.drawRect(clip_rect)

    
    def mousePressEvent(self, event):  # type: ignore[override]
        if event.button() != Qt.LeftButton:
            return
        res = self._hit_test(event.position())
        if res.clip_id is None:
            return
        if res.edge == 'left':
            self.drag_state = ('trim_left', res.clip_id, event.position(), None)
        elif res.edge == 'right':
            self.drag_state = ('trim_right', res.clip_id, event.position(), None)
        else:
            self.drag_state = ('move', res.clip_id, event.position(), None)
        self.update()

    
    def mouseMoveEvent(self, event):  # type: ignore[override]
        if not self.drag_state:
            return
        mode, clip_id, start_pos, _ = self.drag_state
        clip = self.project.clips.get(clip_id)
        if clip is None:
            return
        dx_pixels = event.position().x() - start_pos.x()
        dx_seconds = float(dx_pixels) * self.seconds_per_pixel
        if mode == 'move':
            clip.timeline_start = max(0.0, clip.timeline_start + dx_seconds)
        elif mode == 'trim_left':
            new_in = max(0.0, clip.source_in + dx_seconds)
            new_in = min(new_in, clip.source_out - 0.05)
            # Keep timeline start locked when trimming left
            clip.source_in = new_in
        elif mode == 'trim_right':
            new_out = max(clip.source_in + 0.05, clip.source_out + dx_seconds)
            clip.source_out = new_out
        self.drag_state = (mode, clip_id, event.position(), None)
        self.update()

    
    def mouseReleaseEvent(self, event):  # type: ignore[override]
        self.drag_state = None
        self.update()

    
    def _hit_test(self, pos: QPointF) -> HitTestResult:
        # Iterate topmost-first by natural id order is fine for now
        for clip in self.project.clips.values():
            x = int(clip.timeline_start / self.seconds_per_pixel)
            w = int((clip.source_out - clip.source_in) / self.seconds_per_pixel)
            y = clip.track_index * (self.track_height + self.row_gap)
            rect = QRectF(x, y, max(6, w), self.track_height)
            if rect.contains(pos):
                # Edge trim region = 6 px
                if abs(pos.x() - rect.left()) <= 6:
                    return HitTestResult(clip_id=clip.id, edge='left')
                if abs(pos.x() - rect.right()) <= 6:
                    return HitTestResult(clip_id=clip.id, edge='right')
                return HitTestResult(clip_id=clip.id, edge=None)
        return HitTestResult(clip_id=None, edge=None)


