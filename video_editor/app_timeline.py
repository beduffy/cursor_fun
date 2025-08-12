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
    from .models import Project, ProjectHistory
    from .timeline import TimelineView
    from .preview import PreviewWidget
except ImportError:
    from compositor import render_project  # type: ignore
    from models import Project, ProjectHistory  # type: ignore
    from timeline import TimelineView  # type: ignore
    from preview import PreviewWidget  # type: ignore
from PySide6 import QtGui


class MediaListWidget(QListWidget):
    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setAcceptDrops(True)
        self.setDragEnabled(True)
        self.setDefaultDropAction(Qt.IgnoreAction)

    def dragEnterEvent(self, event):  # type: ignore[override]
        if event.mimeData().hasUrls():
            event.acceptProposedAction()
        else:
            super().dragEnterEvent(event)

    def dragMoveEvent(self, event):  # type: ignore[override]
        if event.mimeData().hasUrls():
            event.accept()
        else:
            super().dragMoveEvent(event)

    def dropEvent(self, event):  # type: ignore[override]
        if not event.mimeData().hasUrls():
            super().dropEvent(event)
            return
        paths = []
        for url in event.mimeData().urls():
            local = url.toLocalFile()
            if local:
                paths.append(local)
        self.parent().handle_media_drop(paths)  # type: ignore[attr-defined]
        event.acceptProposedAction()


class TimelineEditorWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Video Editor – Multi-track MVP")
        self.resize(1400, 900)

        self.project = Project()
        self.history = ProjectHistory()
        self.history.push(self.project)

        # Media bin
        self.media_list = MediaListWidget(self)
        self.media_list.setSelectionMode(QListWidget.SingleSelection)

        add_media_btn = QPushButton("+ Add Media")
        add_media_btn.clicked.connect(self.add_media)

        export_btn = QPushButton("Export…")
        export_btn.clicked.connect(self.export)

        left_layout = QVBoxLayout()
        left_layout.addWidget(QLabel("Media Bin"))
        left_layout.addWidget(self.media_list)
        left_layout.addWidget(add_media_btn)
        # Zoom controls
        zoom_row = QHBoxLayout()
        z_in = QPushButton("+")
        z_out = QPushButton("-")
        z_in.clicked.connect(lambda: self._zoom(0.9))
        z_out.clicked.connect(lambda: self._zoom(1.1))
        zoom_row.addWidget(QLabel("Zoom"))
        zoom_row.addStretch(1)
        zoom_row.addWidget(z_out)
        zoom_row.addWidget(z_in)
        left_layout.addLayout(zoom_row)
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
        # Horizontal scroll control
        from PySide6.QtWidgets import QSlider
        self.hscroll = QSlider(Qt.Horizontal)
        self.hscroll.setRange(0, 1000)
        self.hscroll.setSingleStep(1)
        self.hscroll.valueChanged.connect(self._on_hscroll_changed)

        right_v.addWidget(self.timeline, 3)
        right_v.addWidget(self.hscroll)
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
        delete_clip_action.triggered.connect(self.delete_clip)
        self.addAction(delete_clip_action)

        # Toggle snapping (G)
        toggle_snap_action = QAction(self)
        toggle_snap_action.setShortcut(Qt.Key_G)
        toggle_snap_action.triggered.connect(self.toggle_snapping)
        self.addAction(toggle_snap_action)

        # Split at playhead (S)
        split_action = QAction(self)
        split_action.setShortcut(Qt.Key_S)
        split_action.triggered.connect(self.split_selected_at_playhead)
        self.addAction(split_action)

        # Toggle track mute (M) and visibility (V) on selected clip's track
        mute_action = QAction(self)
        mute_action.setShortcut(Qt.Key_M)
        mute_action.triggered.connect(self.toggle_selected_track_mute)
        self.addAction(mute_action)
        vis_action = QAction(self)
        vis_action.setShortcut(Qt.Key_V)
        vis_action.triggered.connect(self.toggle_selected_track_visibility)
        self.addAction(vis_action)

        # Duplicate selected (D)
        dup_action = QAction(self)
        dup_action.setShortcut(Qt.Key_D)
        dup_action.triggered.connect(self.duplicate_selected)
        self.addAction(dup_action)

        # Undo/Redo
        undo_action = QAction(self)
        undo_action.setShortcut(QKeySequence.Undo)
        undo_action.triggered.connect(self.undo)
        self.addAction(undo_action)
        redo_action = QAction(self)
        redo_action.setShortcut(QKeySequence.Redo)
        redo_action.triggered.connect(self.redo)
        self.addAction(redo_action)

        # Follow playhead (F)
        self.follow_enabled = False
        follow_action = QAction(self)
        follow_action.setShortcut(Qt.Key_F)
        follow_action.triggered.connect(self.toggle_follow)
        self.addAction(follow_action)

    
    def add_media(self) -> None:
        paths, _ = QFileDialog.getOpenFileNames(
            self, "Add Media", os.path.expanduser("~"), "Videos (*.mp4 *.mov *.mkv *.avi *.webm);;All files (*)"
        )
        if not paths:
            return
        self._add_media_paths(paths)

    
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
        self.history.push(self.project)
        self._sync_hscroll_to_view()

    
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

    
    def handle_media_drop(self, paths):  # called by MediaListWidget
        self._add_media_paths(paths)

    
    def _add_media_paths(self, paths):
        for p in paths:
            try:
                src = self.project.add_source(p)
                item = QListWidgetItem(os.path.basename(p))
                item.setData(Qt.UserRole, src.id)
                self.media_list.addItem(item)
            except Exception as exc:
                QMessageBox.critical(self, "Add Media Failed", str(exc))

    
    def delete_clip(self) -> None:
        # Delete selected clips if any; otherwise delete last added
        if not self.project.clips:
            return
        selected_ids = list(getattr(self.timeline, 'selected_clip_ids', set()))
        if selected_ids:
            self.project.remove_clips(selected_ids)
        else:
            last_id = max(self.project.clips.keys())
            self.project.remove_clip(last_id)
        self.timeline.update()
        self.history.push(self.project)
        self._sync_hscroll_to_view()

    
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
        if self.follow_enabled:
            self._auto_follow_view()
            self._sync_hscroll_to_view()

    
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

    
    def toggle_follow(self) -> None:
        self.follow_enabled = not self.follow_enabled
        status = "ON" if self.follow_enabled else "OFF"
        self.statusBar().showMessage(f"Follow playhead: {status}", 2000)

    
    def _visible_window_seconds(self) -> float:
        # Estimate visible window size in seconds based on timeline width
        try:
            width_px = max(1, self.timeline.width() - getattr(self.timeline, 'header_width', 60))
            return width_px * self.timeline.seconds_per_pixel
        except Exception:
            return 10.0

    
    def _auto_follow_view(self) -> None:
        win = self._visible_window_seconds()
        center = max(0.0, self.timeline.playhead_time - 0.5 * win)
        self.timeline.view_offset_seconds = max(0.0, center)
        self.timeline.update()

    
    def _sync_hscroll_to_view(self) -> None:
        total = max(0.0, self.project.get_timeline_duration())
        win = max(0.001, self._visible_window_seconds())
        max_offset = max(0.0, total - win)
        if max_offset <= 0:
            self.hscroll.setEnabled(False)
            self.hscroll.setValue(0)
            return
        self.hscroll.setEnabled(True)
        pos = int(1000.0 * min(1.0, max(0.0, self.timeline.view_offset_seconds / max_offset)))
        self.hscroll.blockSignals(True)
        self.hscroll.setValue(pos)
        self.hscroll.blockSignals(False)

    
    def _on_hscroll_changed(self, value: int) -> None:
        total = max(0.0, self.project.get_timeline_duration())
        win = max(0.001, self._visible_window_seconds())
        max_offset = max(0.0, total - win)
        target = (value / 1000.0) * max_offset
        self.timeline.view_offset_seconds = target
        self.timeline.update()

    
    def toggle_snapping(self) -> None:
        current = getattr(self.timeline, 'snapping_enabled', True)
        self.timeline.snapping_enabled = not current
        status = "ON" if self.timeline.snapping_enabled else "OFF"
        self.statusBar().showMessage(f"Snapping: {status}", 2000)

    
    def _zoom(self, factor: float) -> None:
        self.timeline.seconds_per_pixel = max(0.001, min(0.5, self.timeline.seconds_per_pixel * factor))
        self.timeline.update()

    
    def split_selected_at_playhead(self) -> None:
        clip_id = getattr(self.timeline, 'selected_clip_id', None)
        if clip_id is None:
            return
        self.project.split_clip(clip_id, self.timeline.playhead_time)
        self.timeline.update()
        self.history.push(self.project)

    
    def toggle_selected_track_mute(self) -> None:
        clip_id = getattr(self.timeline, 'selected_clip_id', None)
        if clip_id is None or clip_id not in self.project.clips:
            return
        track = self.project.clips[clip_id].track_index
        self.project.toggle_track_mute(track)
        self.timeline.update()

    
    def toggle_selected_track_visibility(self) -> None:
        clip_id = getattr(self.timeline, 'selected_clip_id', None)
        if clip_id is None or clip_id not in self.project.clips:
            return
        track = self.project.clips[clip_id].track_index
        self.project.toggle_track_visible(track)
        self.timeline.update()

    
    def duplicate_selected(self) -> None:
        clip_id = getattr(self.timeline, 'selected_clip_id', None)
        if clip_id is None:
            return
        new_clip = self.project.duplicate_clip(clip_id)
        if new_clip is not None:
            # nudge right by 0.5s to avoid overlap
            new_clip.timeline_start += 0.5
        self.timeline.update()
        self.history.push(self.project)

    
    def undo(self) -> None:
        if self.history.undo(self.project):
            self.timeline.update()

    
    def redo(self) -> None:
        if self.history.redo(self.project):
            self.timeline.update()

    
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


