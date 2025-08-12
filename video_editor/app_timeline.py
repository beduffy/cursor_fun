import os
import sys
from typing import Optional

from PySide6.QtCore import Qt
from PySide6.QtGui import QAction, QKeySequence
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
except ImportError:
    from compositor import render_project  # type: ignore
    from models import Project  # type: ignore
    from timeline import TimelineView  # type: ignore


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

        splitter = QSplitter()
        splitter.addWidget(left_widget)
        splitter.addWidget(self.timeline)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)

        container = QWidget()
        layout = QHBoxLayout()
        layout.addWidget(splitter)
        container.setLayout(layout)
        self.setCentralWidget(container)

        # Shortcuts
        add_clip_action = QAction(self)
        add_clip_action.setShortcut(Qt.Key_C)
        add_clip_action.triggered.connect(self.add_clip_from_selected)
        self.addAction(add_clip_action)

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

    
    def delete_clip_at_playhead(self) -> None:
        # MVP: delete the last added clip (no playhead implemented yet)
        if not self.project.clips:
            return
        last_id = max(self.project.clips.keys())
        self.project.remove_clip(last_id)
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


