from __future__ import annotations

from typing import Dict, Optional

import cv2
import numpy as np
from PySide6.QtCore import Qt
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import QLabel, QVBoxLayout, QWidget

try:
    from .models import Clip, Project
except ImportError:
    from models import Clip, Project  # type: ignore


def _select_top_clip_at_time(project: Project, t: float) -> Optional[Clip]:
    candidates = [
        c for c in project.clips.values() if c.timeline_start <= t < c.timeline_end
    ]
    if not candidates:
        return None
    return sorted(candidates, key=lambda c: c.track_index, reverse=True)[0]


class PreviewWidget(QWidget):
    def __init__(self, project: Project, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.project = project
        self.label = QLabel("Preview")
        self.label.setAlignment(Qt.AlignCenter)
        layout = QVBoxLayout()
        layout.addWidget(self.label)
        self.setLayout(layout)
        self._captures: Dict[int, cv2.VideoCapture] = {}
        self._last_source_id: Optional[int] = None

    
    def _get_capture(self, source_id: int) -> Optional[cv2.VideoCapture]:
        cap = self._captures.get(source_id)
        if cap is not None and cap.isOpened():
            return cap
        src = self.project.sources.get(source_id)
        if src is None:
            return None
        cap = cv2.VideoCapture(src.path)
        if not cap.isOpened():
            return None
        self._captures[source_id] = cap
        return cap

    
    def update_preview(self, t: float) -> None:
        clip = _select_top_clip_at_time(self.project, t)
        if clip is None:
            self.label.setText("No clip at playhead")
            return
        # Map playhead time to source time
        local_offset = max(0.0, t - clip.timeline_start)
        source_time = clip.source_in + local_offset

        cap = self._get_capture(clip.source_id)
        if cap is None:
            self.label.setText("Failed to open source")
            return
        src = self.project.sources[clip.source_id]
        fps = max(1.0, float(src.fps) or 30.0)
        frame_index = int(round(source_time * fps))
        cap.set(cv2.CAP_PROP_POS_FRAMES, frame_index)
        ok, frame = cap.read()
        if not ok or frame is None:
            self.label.setText("End of clip")
            return
        self._show_frame(frame)

    
    def _show_frame(self, frame_bgr: np.ndarray) -> None:
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        h, w, ch = frame_rgb.shape
        bytes_per_line = ch * w
        image = QImage(frame_rgb.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
        pixmap = QPixmap.fromImage(image).scaled(
            self.label.width(),
            self.label.height(),
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation,
        )
        self.label.setPixmap(pixmap)


