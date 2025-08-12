import os
import sys
from typing import Optional

from PySide6.QtCore import Qt
from PySide6.QtGui import QAction, QKeySequence
from PySide6.QtCore import QMimeData, QTimer
from PySide6.QtWidgets import (
    QApplication,
    QFileDialog,
    QHBoxLayout,
    QLabel,
    QListWidget,
    QListWidgetItem,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSplitter,
    QVBoxLayout,
    QWidget,
)

try:
    from .compositor import render_project
    from .models import Project
    from .timeline import TimelineView
    from .preview import PreviewWidget
except ImportError:
    from compositor import render_project  # type: ignore
    from models import Project  # type: ignore
    from timeline import TimelineView  # type: ignore
    from preview import PreviewWidget  # type: ignore
from PySide6 import QtGui


class TimelineEditorWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Video Editor – Multi-track MVP")
        self.resize(1400, 900)

        self.project = Project()

        # Media bin
        self.media_list = QListWidget()
        self.media_list.setSelectionMode(QListWidget.SingleSelection)

        add_media_btn = QPushButton("+ Add Media")
        add_media_btn.clicked.connect(self.add_media)

        export_btn = QPushButton("Export…")
        export_btn.clicked.connect(self.export)

        left_layout = QVBoxLayout()
        left_layout.addWidget(QLabel("Media Bin"))
        left_layout.addWidget(self.media_list)
        left_layout.addWidget(add_media_btn)
        left_layout.addStretch(1)
        left_layout.addWidget(export_btn)

        left_widget = QWidget()
        left_widget.setLayout(left_layout)

        # Timeline
        self.timeline = TimelineView(self.project)
        self.timeline.playheadChanged.connect(self._on_playhead_changed)

        # Right side: preview above timeline
        right = QWidget()
        right_v = QVBoxLayout()
        controls = QWidget()
        controls_h = QHBoxLayout()
        self.play_btn = QPushButton("Play")
        self.play_btn.clicked.connect(self.toggle_play)
        self.time_label = QLabel("00:00:00.000")
        controls_h.addWidget(self.play_btn)
        controls_h.addStretch(1)
        controls_h.addWidget(self.time_label)
        controls.setLayout(controls_h)

        self.preview = PreviewWidget(self.project)
        right_v.addWidget(self.preview, 6)
        right_v.addWidget(controls, 1)
        right_v.addWidget(self.timeline, 3)
        right.setLayout(right_v)

        splitter = QSplitter()
        splitter.addWidget(left_widget)
        splitter.addWidget(right)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)

        container = QWidget()
        layout = QHBoxLayout()
        layout.addWidget(splitter)
        container.setLayout(layout)
        self.setCentralWidget(container)
        
        # Simple play timer (preview sync hook)
        self.play_timer = QTimer(self)
        self.play_timer.setInterval(33)
        self.play_timer.timeout.connect(self._advance_playhead)

        # Shortcuts
        add_clip_action = QAction(self)
        add_clip_action.setShortcut(Qt.Key_C)
        add_clip_action.triggered.connect(self.add_clip_from_selected)
        self.addAction(add_clip_action)

        # Enable dragging from media bin into timeline
        self.media_list.setDragEnabled(True)
        self.media_list.viewport().setAcceptDrops(False)
        self.media_list.setDefaultDropAction(Qt.IgnoreAction)
        self.media_list.startDrag = self._media_start_drag  # type: ignore[attr-defined]

        delete_clip_action = QAction(self)
        delete_clip_action.setShortcut(QKeySequence.Delete)
        delete_clip_action.triggered.connect(self.delete_clip_at_playhead)
        self.addAction(delete_clip_action)

    
    def add_media(self) -> None:
        paths, _ = QFileDialog.getOpenFileNames(
            self, "Add Media", os.path.expanduser("~"), "Videos (*.mp4 *.mov *.mkv *.avi *.webm);;All files (*)"
        )
        if not paths:
            return
        for p in paths:
            try:
                src = self.project.add_source(p)
                item = QListWidgetItem(os.path.basename(p))
                item.setData(Qt.UserRole, src.id)
                self.media_list.addItem(item)
            except Exception as exc:
                QMessageBox.critical(self, "Add Media Failed", str(exc))

    
    def add_clip_from_selected(self) -> None:
        item = self.media_list.currentItem()
        if item is None:
            QMessageBox.information(self, "Add Clip", "Select a media item first.")
            return
        source_id = item.data(Qt.UserRole)
        src = self.project.sources.get(source_id)
        if src is None:
            return
        # Add a full-length clip at t=0 on track 0 for now (MVP). User can drag/trim.
        clip = self.project.add_clip(source_id, 0.0, src.duration, 0.0, 0)
        self.timeline.update()

    
    def _media_start_drag(self, supportedActions):  # type: ignore[override]
        item = self.media_list.currentItem()
        if item is None:
            return
        source_id = item.data(Qt.UserRole)
        mime = QMimeData()
        mime.setData('application/x-videoeditor-source', str(source_id).encode('utf-8'))
        drag = QtGui.QDrag(self.media_list)
        drag.setMimeData(mime)
        drag.exec(Qt.CopyAction)

    
    def delete_clip_at_playhead(self) -> None:
        # MVP: delete the last added clip (no playhead implemented yet)
        if not self.project.clips:
            return
        last_id = max(self.project.clips.keys())
        self.project.remove_clip(last_id)
        self.timeline.update()

    
    def _on_playhead_changed(self, t: float) -> None:
        # Update preview on playhead change
        self.preview.update_preview(t)
        self._update_time_label(t)

    
    def _advance_playhead(self) -> None:
        self.timeline.playhead_time += 0.033
        self.timeline.update()
        t = self.timeline.playhead_time
        self.preview.update_preview(t)
        self._update_time_label(t)

    
    def toggle_play(self) -> None:
        if self.play_timer.isActive():
            self.play_timer.stop()
            self.play_btn.setText("Play")
        else:
            self.play_timer.start()
            self.play_btn.setText("Pause")

    
    def _update_time_label(self, t: float) -> None:
        h = int(t // 3600)
        m = int((t % 3600) // 60)
        s = t % 60
        self.time_label.setText(f"{h:02d}:{m:02d}:{s:06.3f}")

    
    def export(self) -> None:
        if not self.project.clips:
            QMessageBox.information(self, "Export", "No clips on the timeline.")
            return
        out_path, _ = QFileDialog.getSaveFileName(self, "Export", os.path.expanduser("~"), "MP4 Video (*.mp4)")
        if not out_path:
            return
        try:
            self.setEnabled(False)
            QApplication.setOverrideCursor(Qt.WaitCursor)
            render_project(self.project, out_path)
            QMessageBox.information(self, "Export", f"Exported to:\n{out_path}")
        except Exception as exc:
            QMessageBox.critical(self, "Export Failed", str(exc))
        finally:
            QApplication.restoreOverrideCursor()
            self.setEnabled(True)


def main() -> None:
    app = QApplication(sys.argv)
    w = TimelineEditorWindow()
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()


