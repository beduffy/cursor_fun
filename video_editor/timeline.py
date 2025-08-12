from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Set

from PySide6.QtCore import QPointF, QRectF, Qt, QMimeData, Signal
from PySide6.QtGui import QColor, QPainter, QPen, QCursor
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
    playheadChanged = Signal(float)
    def __init__(self, project: Project, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.project = project
        self.seconds_per_pixel = 0.02  # zoom level
        self.track_height = 44
        self.header_width = 60
        self.ruler_height = 22
        self.row_gap = 6
        self.setMouseTracking(True)
        self.setAcceptDrops(True)
        self.drag_state = None  # ('move'|'trim_left'|'trim_right', clip_id, start_pos, start_time)
        self.playhead_time = 0.0
        self.selected_clip_id = None
        self.selected_clip_ids: Set[int] = set()
        self.snapping_enabled = True
        self.view_offset_seconds = 0.0
        self.selected_clip_id = None

    
    def sizeHint(self):  # type: ignore[override]
        return super().sizeHint()

    
    def paintEvent(self, event):  # type: ignore[override]
        painter = QPainter(self)
        rect = self.rect()
        painter.fillRect(rect, QColor(30, 30, 30))

        # Time ruler
        ruler_rect = QRectF(0, 0, rect.width(), self.ruler_height)
        painter.fillRect(ruler_rect, QColor(35, 35, 35))
        painter.setPen(QPen(QColor(120, 120, 120)))
        seconds_per_tick = max(0.5, round(self.seconds_per_pixel * 100))  # adaptive
        # draw tick marks and labels approx every 1s at current zoom
        tick_step_px = max(50, int(seconds_per_tick / self.seconds_per_pixel))
        x0 = self.header_width
        for x in range(x0, rect.width(), tick_step_px):
            t = (x - x0) * self.seconds_per_pixel + self.view_offset_seconds
            painter.drawLine(x, 0, x, self.ruler_height)
            label = f"{t:0.1f}s"
            painter.drawText(x + 2, int(self.ruler_height - 6), label)

        # Tracks background
        for track in range(self.project.track_count):
            y = self.ruler_height + track * (self.track_height + self.row_gap)
            painter.fillRect(QRectF(0, y, rect.width(), self.track_height), QColor(40, 40, 40))
            # draw lock icon as simple text
            locked = False
            try:
                locked = self.project.is_track_locked(track)  # type: ignore[attr-defined]
            except Exception:
                pass
            painter.setPen(QPen(QColor(200, 200, 200)))
            painter.drawText(6, int(y + self.track_height / 2 + 5), "🔒" if locked else "🔓")
            painter.setPen(QPen(QColor(70, 70, 70)))

        # Grid lines
        pen = QPen(QColor(70, 70, 70))
        painter.setPen(pen)
        for x in range(self.header_width, rect.width(), 50):
            painter.drawLine(x, self.ruler_height, x, rect.height())

        # Draw clips
        painter.setPen(QPen(QColor(20, 20, 20)))
        for clip in self.project.clips.values():
            x = int((clip.timeline_start - self.view_offset_seconds) / self.seconds_per_pixel) + self.header_width
            w = int((clip.source_out - clip.source_in) / self.seconds_per_pixel)
            y = self.ruler_height + clip.track_index * (self.track_height + self.row_gap)
            clip_rect = QRectF(x, y, max(6, w), self.track_height)
            color = QColor(80, 120, 200)
            # highlight selection
            try:
                selected_id = self.selected_clip_id  # type: ignore[attr-defined]
            except Exception:
                selected_id = None
            if selected_id == clip.id:
                color = QColor(120, 160, 240)
            painter.fillRect(clip_rect, color)
            painter.drawRect(clip_rect)
            # label
            try:
                from .models import Project  # noqa: F401
            except Exception:
                pass
            # Draw a simple label
            painter.setPen(QPen(QColor(240, 240, 240)))
            try:
                src_path = self.project.sources[clip.source_id].path
                base = src_path.split('/')[-1]
            except Exception:
                base = f"Clip {clip.id}"
            painter.drawText(clip_rect.adjusted(6, 4, -6, -4), Qt.AlignLeft | Qt.AlignVCenter, base)
            painter.setPen(QPen(QColor(20, 20, 20)))

        # Playhead
        play_x = int((self.playhead_time - self.view_offset_seconds) / self.seconds_per_pixel) + self.header_width
        painter.setPen(QPen(QColor(200, 60, 60), 2))
        painter.drawLine(play_x, 0, play_x, rect.height())

    
    def mousePressEvent(self, event):  # type: ignore[override]
        if event.button() == Qt.RightButton:
            # start panning
            self._is_panning = True
            self._pan_start_x = float(event.position().x())
            self._pan_start_offset = self.view_offset_seconds
            self.setCursor(QCursor(Qt.ClosedHandCursor))
            return
        if event.button() != Qt.LeftButton:
            return
        # set playhead if clicked in empty area
        res = self._hit_test(event.position())
        if res.clip_id is None:
            # toggle lock if clicking header area
            if float(event.position().x()) < self.header_width:
                track = int(max(0, (float(event.position().y()) - self.ruler_height) // float(self.track_height + self.row_gap)))
                try:
                    self.project.toggle_track_lock(track)  # type: ignore[attr-defined]
                except Exception:
                    pass
                self.update()
                return
            self.playhead_time = max(0.0, (float(event.position().x()) - self.header_width) * self.seconds_per_pixel + self.view_offset_seconds)
            self.update()
            self.playheadChanged.emit(self.playhead_time)
            return
        if res.edge == 'left':
            self.drag_state = ('trim_left', res.clip_id, event.position(), None)
        elif res.edge == 'right':
            self.drag_state = ('trim_right', res.clip_id, event.position(), None)
        else:
            self.drag_state = ('move', res.clip_id, event.position(), None)
            self.setCursor(QCursor(Qt.ClosedHandCursor))
        # selection
        try:
            self.selected_clip_id = res.clip_id  # type: ignore[attr-defined]
        except Exception:
            pass
        self.update()

    
    def mouseMoveEvent(self, event):  # type: ignore[override]
        # cursor feedback
        if getattr(self, '_is_panning', False):
            dx = float(event.position().x()) - getattr(self, '_pan_start_x', 0.0)
            self.view_offset_seconds = max(0.0, getattr(self, '_pan_start_offset', 0.0) - dx * self.seconds_per_pixel)
            self.update()
            return
        res = self._hit_test(event.position())
        if res.clip_id is not None and res.edge in ('left', 'right'):
            self.setCursor(QCursor(Qt.SizeHorCursor))
        elif res.clip_id is not None:
            self.setCursor(QCursor(Qt.OpenHandCursor))
        else:
            self.setCursor(QCursor(Qt.ArrowCursor))

        if not self.drag_state:
            return
        mode, clip_id, start_pos, _ = self.drag_state
        clip = self.project.clips.get(clip_id)
        if clip is None:
            return
        dx_pixels = event.position().x() - start_pos.x()
        dx_seconds = float(dx_pixels) * self.seconds_per_pixel
        if mode == 'move':
            target_start = max(0.0, clip.timeline_start + dx_seconds)
            if self.snapping_enabled:
                target_start = self._apply_snapping_time(target_start, exclude_clip_id=clip.id)
            clip.timeline_start = target_start
            # vertical track change: snap to track under cursor
            cursor_y = float(event.position().y())
            track = int(max(0, (cursor_y - self.ruler_height) // float(self.track_height + self.row_gap)))
            track = min(self.project.track_count - 1, track)
            try:
                if not self.project.is_track_locked(track):  # type: ignore[attr-defined]
                    clip.track_index = track
            except Exception:
                clip.track_index = track
        elif mode == 'trim_left':
            new_left = clip.timeline_start + dx_seconds
            snapped_left = self._apply_snapping_time(new_left, exclude_clip_id=clip.id) if self.snapping_enabled else new_left
            delta_left = snapped_left - clip.timeline_start
            new_in = max(0.0, clip.source_in + delta_left)
            new_in = min(new_in, clip.source_out - 0.05)
            clip.source_in = new_in
        elif mode == 'trim_right':
            current_right = clip.timeline_start + (clip.source_out - clip.source_in)
            snapped_right = self._apply_snapping_time(current_right + dx_seconds, exclude_clip_id=clip.id) if self.snapping_enabled else (current_right + dx_seconds)
            delta = snapped_right - current_right
            new_out = max(clip.source_in + 0.05, clip.source_out + delta)
            clip.source_out = new_out
        self.drag_state = (mode, clip_id, event.position(), None)
        self.update()

    
    def mouseReleaseEvent(self, event):  # type: ignore[override]
        self.drag_state = None
        self.setCursor(QCursor(Qt.ArrowCursor))
        self.update()

    
    def _hit_test(self, pos: QPointF) -> HitTestResult:
        # Iterate topmost-first by natural id order is fine for now
        for clip in self.project.clips.values():
            x = int((clip.timeline_start - self.view_offset_seconds) / self.seconds_per_pixel)
            w = int((clip.source_out - clip.source_in) / self.seconds_per_pixel)
            y = self.ruler_height + clip.track_index * (self.track_height + self.row_gap)
            rect = QRectF(x, y, max(6, w), self.track_height)
            if rect.contains(pos):
                # Edge trim region = 6 px
                if abs(pos.x() - rect.left()) <= 6:
                    return HitTestResult(clip_id=clip.id, edge='left')
                if abs(pos.x() - rect.right()) <= 6:
                    return HitTestResult(clip_id=clip.id, edge='right')
                return HitTestResult(clip_id=clip.id, edge=None)
        return HitTestResult(clip_id=None, edge=None)

    
    def wheelEvent(self, event):  # type: ignore[override]
        # Zoom with Ctrl+Wheel, pan with Shift+Wheel, else scroll playhead
        if event.modifiers() & Qt.ControlModifier:
            delta = event.angleDelta().y()
            factor = 0.9 if delta > 0 else 1.1
            self.seconds_per_pixel = max(0.001, min(0.5, self.seconds_per_pixel * factor))
            self.update()
        elif event.modifiers() & Qt.ShiftModifier:
            delta = event.angleDelta().y()
            self.view_offset_seconds = max(0.0, self.view_offset_seconds - delta * 0.002 * (1.0 / max(self.seconds_per_pixel, 1e-6)))
            self.update()
        else:
            delta = event.angleDelta().y()
            self.playhead_time = max(0.0, self.playhead_time - delta * 0.001)
            self.update()
            self.playheadChanged.emit(self.playhead_time)

    
    def dragEnterEvent(self, event):  # type: ignore[override]
        if event.mimeData().hasFormat('application/x-videoeditor-source'):
            event.acceptProposedAction()
        else:
            event.ignore()

    
    def dragMoveEvent(self, event):  # type: ignore[override]
        if event.mimeData().hasFormat('application/x-videoeditor-source'):
            event.setDropAction(Qt.CopyAction)
            event.accept()
        else:
            event.ignore()

    
    def dropEvent(self, event):  # type: ignore[override]
        if not event.mimeData().hasFormat('application/x-videoeditor-source'):
            event.ignore()
            return
        data = bytes(event.mimeData().data('application/x-videoeditor-source')).decode('utf-8', errors='ignore')
        try:
            source_id = int(data)
        except Exception:
            event.ignore()
            return
        # compute drop time and track
        drop_x = float(event.position().x())
        drop_y = float(event.position().y())
        time_s = max(0.0, drop_x * self.seconds_per_pixel)
        track = int(max(0, (drop_y - self.ruler_height) // float(self.track_height + self.row_gap)))
        track = min(self.project.track_count - 1, track)
        src = self.project.sources.get(source_id)
        if src is None:
            event.ignore()
            return
        # default clip length: min(5s, src.duration)
        length = min(5.0, max(0.5, src.duration))
        clip = self.project.add_clip(source_id, 0.0, length, time_s, track)
        self.update()
        event.acceptProposedAction()

    
    def _apply_snapping_time(self, t: float, exclude_clip_id: Optional[int] = None) -> float:
        threshold = 8.0 * self.seconds_per_pixel
        candidates = [self.playhead_time]
        for c in self.project.clips.values():
            if exclude_clip_id is not None and c.id == exclude_clip_id:
                continue
            start = c.timeline_start
            end = c.timeline_start + (c.source_out - c.source_in)
            candidates.extend([start, end])
        best = t
        best_d = threshold + 1.0
        for s in candidates:
            d = abs(s - t)
            if d < best_d and d <= threshold:
                best = s
                best_d = d
        return best


