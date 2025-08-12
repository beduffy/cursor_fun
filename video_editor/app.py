import os
import sys
import time
from dataclasses import dataclass
from typing import List, Optional

import cv2
import numpy as np
from PySide6.QtCore import QEvent, QObject, QPoint, QRect, Qt, QTimer
from PySide6.QtGui import QAction, QCloseEvent, QImage, QKeySequence, QPixmap
from PySide6.QtWidgets import (
    QApplication,
    QComboBox,
    QDialog,
    QFileDialog,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QListWidget,
    QListWidgetItem,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSlider,
    QSpinBox,
    QStyle,
    QToolBar,
    QVBoxLayout,
    QWidget,
)

from ffmpeg_utils import EditSegment, export_edit


def seconds_to_timecode(seconds: float) -> str:
    seconds = max(0.0, float(seconds))
    hours = int(seconds // 3600)
    minutes = int((seconds % 3600) // 60)
    secs = seconds % 60
    return f"{hours:02d}:{minutes:02d}:{secs:06.3f}"



class VideoSource:
    def __init__(self, path: str) -> None:
        self.path = path
        self.capture = cv2.VideoCapture(path)
        if not self.capture.isOpened():
            raise RuntimeError(f"Failed to open video: {path}")

        self.fps = float(self.capture.get(cv2.CAP_PROP_FPS) or 30.0)
        self.frame_count = int(self.capture.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
        self.width = int(self.capture.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
        self.height = int(self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)

    
    def get_duration_seconds(self) -> float:
        if self.fps <= 0 or self.frame_count <= 0:
            return 0.0
        return float(self.frame_count) / float(self.fps)

    
    def set_position_seconds(self, seconds: float) -> None:
        if self.fps <= 0:
            return
        frame_index = int(max(0, min(self.frame_count - 1, round(seconds * self.fps))))
        self.capture.set(cv2.CAP_PROP_POS_FRAMES, frame_index)

    
    def read_frame(self) -> Optional[np.ndarray]:
        ok, frame = self.capture.read()
        if not ok:
            return None
        return frame

    
    def get_position_seconds(self) -> float:
        if self.fps <= 0:
            return 0.0
        current_frame = int(self.capture.get(cv2.CAP_PROP_POS_FRAMES) or 0)
        return float(current_frame) / float(self.fps)

    
    def release(self) -> None:
        try:
            self.capture.release()
        except Exception:
            pass


class VideoPlayerWidget(QWidget):
    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.label = QLabel("No video loaded")
        self.label.setAlignment(Qt.AlignCenter)
        layout = QVBoxLayout()
        layout.addWidget(self.label)
        self.setLayout(layout)

    
    def show_frame(self, frame_bgr: np.ndarray) -> None:
        if frame_bgr is None:
            return
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



@dataclass
class InOut:
    mark_in: Optional[float] = None
    mark_out: Optional[float] = None



class EditorMainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Video Editor (MVP)")
        self.resize(1200, 800)

        self.player = VideoPlayerWidget()

        self.source_selector = QComboBox()
        self.source_selector.currentIndexChanged.connect(self._on_source_changed)

        self.play_button = QPushButton(self.style().standardIcon(QStyle.SP_MediaPlay), "")
        self.play_button.clicked.connect(self.toggle_play)

        self.current_time_label = QLabel("00:00:00.000")
        self.duration_time_label = QLabel("/ 00:00:00.000")

        self.timeline = QSlider(Qt.Horizontal)
        self.timeline.setRange(0, 1000)
        self.timeline.sliderPressed.connect(self._on_scrub_start)
        self.timeline.sliderReleased.connect(self._on_scrub_end)
        self.timeline.sliderMoved.connect(self._on_scrub_moved)

        self.set_in_button = QPushButton("I")
        self.set_in_button.clicked.connect(self.set_in)
        self.set_out_button = QPushButton("O")
        self.set_out_button.clicked.connect(self.set_out)
        self.add_segment_button = QPushButton("Add Segment (A)")
        self.add_segment_button.clicked.connect(self.add_segment)

        self.segment_list = QListWidget()
        self.segment_list.setSelectionMode(QListWidget.ExtendedSelection)
        self.segment_list.setDragDropMode(QListWidget.InternalMove)

        self.export_button = QPushButton("Export…")
        self.export_button.clicked.connect(self.export)

        control_bar = QHBoxLayout()
        control_bar.addWidget(self.play_button)
        control_bar.addWidget(self.current_time_label)
        control_bar.addWidget(self.duration_time_label)
        control_bar.addStretch(1)
        control_bar.addWidget(QLabel("Source:"))
        control_bar.addWidget(self.source_selector)

        marks_bar = QHBoxLayout()
        marks_bar.addWidget(self.set_in_button)
        marks_bar.addWidget(self.set_out_button)
        marks_bar.addWidget(self.add_segment_button)
        marks_bar.addStretch(1)
        marks_bar.addWidget(self.export_button)

        right_box = QVBoxLayout()
        right_box.addWidget(QLabel("Segments (drag to reorder):"))
        right_box.addWidget(self.segment_list)

        main_grid = QGridLayout()
        main_grid.addLayout(control_bar, 0, 0, 1, 2)
        main_grid.addWidget(self.player, 1, 0)
        main_grid.addLayout(right_box, 1, 1)
        main_grid.addWidget(self.timeline, 2, 0, 1, 2)
        main_grid.addLayout(marks_bar, 3, 0, 1, 2)

        container = QWidget()
        container.setLayout(main_grid)
        self.setCentralWidget(container)

        self._create_menu()
        self._create_shortcuts()

        self.sources: List[VideoSource] = []
        self.current_source: Optional[VideoSource] = None
        self.in_out = InOut()
        self.playing = False
        self.scrubbing = False
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.setInterval(30)

    
    def _create_menu(self) -> None:
        open_action = QAction("Open Video…", self)
        open_action.setShortcut(QKeySequence.Open)
        open_action.triggered.connect(self.open_video)

        remove_action = QAction("Remove Selected Segment(s)", self)
        remove_action.setShortcut(QKeySequence.Delete)
        remove_action.triggered.connect(self.remove_selected_segments)

        menubar = self.menuBar()
        file_menu = menubar.addMenu("File")
        file_menu.addAction(open_action)
        file_menu.addSeparator()
        file_menu.addAction(remove_action)

    
    def _create_shortcuts(self) -> None:
        # Space: Play/Pause
        toggle_action = QAction(self)
        toggle_action.setShortcut(Qt.Key_Space)
        toggle_action.triggered.connect(self.toggle_play)
        self.addAction(toggle_action)

        # I: Mark In
        in_action = QAction(self)
        in_action.setShortcut(Qt.Key_I)
        in_action.triggered.connect(self.set_in)
        self.addAction(in_action)

        # O: Mark Out
        out_action = QAction(self)
        out_action.setShortcut(Qt.Key_O)
        out_action.triggered.connect(self.set_out)
        self.addAction(out_action)

        # A: Add segment
        add_action = QAction(self)
        add_action.setShortcut(Qt.Key_A)
        add_action.triggered.connect(self.add_segment)
        self.addAction(add_action)

    
    def open_video(self) -> None:
        paths, _ = QFileDialog.getOpenFileNames(
            self, "Open Video(s)", os.path.expanduser("~"), "Videos (*.mp4 *.mov *.mkv *.avi *.webm);;All files (*)"
        )
        if not paths:
            return

        for path in paths:
            try:
                source = VideoSource(path)
                self.sources.append(source)
                self.source_selector.addItem(os.path.basename(path), path)
            except Exception as exc:
                QMessageBox.critical(self, "Open Video Failed", str(exc))
                continue

        if self.current_source is None and self.sources:
            self._select_source_by_index(0)

    
    def _select_source_by_index(self, index: int) -> None:
        if index < 0 or index >= len(self.sources):
            return
        self.current_source = self.sources[index]
        duration = self.current_source.get_duration_seconds()
        self.duration_time_label.setText(f"/ {seconds_to_timecode(duration)}")
        self.current_source.set_position_seconds(0.0)
        first = self.current_source.read_frame()
        if first is not None:
            self.player.show_frame(first)
        self._update_time_labels()

    
    def _on_source_changed(self, index: int) -> None:
        self._select_source_by_index(index)

    
    def toggle_play(self) -> None:
        if self.current_source is None:
            return
        self.playing = not self.playing
        icon = QStyle.SP_MediaPause if self.playing else QStyle.SP_MediaPlay
        self.play_button.setIcon(self.style().standardIcon(icon))
        if self.playing:
            interval_ms = int(max(1, round(1000.0 / max(1.0, self.current_source.fps))))
            self.timer.setInterval(interval_ms)
            self.timer.start()
        else:
            self.timer.stop()

    
    def set_in(self) -> None:
        if self.current_source is None:
            return
        self.in_out.mark_in = self.current_source.get_position_seconds()
        self._update_status_tip()

    
    def set_out(self) -> None:
        if self.current_source is None:
            return
        self.in_out.mark_out = self.current_source.get_position_seconds()
        self._update_status_tip()

    
    def add_segment(self) -> None:
        if self.current_source is None:
            return
        if self.in_out.mark_in is None or self.in_out.mark_out is None:
            QMessageBox.information(self, "Add Segment", "Set In and Out points first (I and O).")
            return
        start_s = min(self.in_out.mark_in, self.in_out.mark_out)
        end_s = max(self.in_out.mark_in, self.in_out.mark_out)
        if end_s - start_s <= 0.01:
            QMessageBox.information(self, "Add Segment", "Segment is too short.")
            return
        source_path = self.current_source.path
        display = f"{os.path.basename(source_path)}  [{seconds_to_timecode(start_s)} - {seconds_to_timecode(end_s)}]"
        item = QListWidgetItem(display)
        item.setData(Qt.UserRole, EditSegment(source_path, start_s, end_s))
        self.segment_list.addItem(item)

    
    def remove_selected_segments(self) -> None:
        for item in self.segment_list.selectedItems():
            row = self.segment_list.row(item)
            self.segment_list.takeItem(row)

    
    def export(self) -> None:
        if self.segment_list.count() == 0:
            QMessageBox.information(self, "Export", "No segments in the list.")
            return
        output_path, _ = QFileDialog.getSaveFileName(
            self, "Export Video", os.path.expanduser("~"), "MP4 Video (*.mp4)"
        )
        if not output_path:
            return

        segments: List[EditSegment] = []
        for i in range(self.segment_list.count()):
            item = self.segment_list.item(i)
            seg: EditSegment = item.data(Qt.UserRole)
            segments.append(seg)

        try:
            self.setEnabled(False)
            QApplication.setOverrideCursor(Qt.WaitCursor)
            export_edit(segments, output_path)
            QMessageBox.information(self, "Export", f"Exported to:\n{output_path}")
        except Exception as exc:
            QMessageBox.critical(self, "Export Failed", str(exc))
        finally:
            QApplication.restoreOverrideCursor()
            self.setEnabled(True)

    
    def _tick(self) -> None:
        if self.current_source is None or self.scrubbing:
            return
        frame = self.current_source.read_frame()
        if frame is None:
            # Reached end
            self.playing = False
            self.timer.stop()
            self.play_button.setIcon(self.style().standardIcon(QStyle.SP_MediaPlay))
            return
        self.player.show_frame(frame)
        self._sync_timeline_with_position()

    
    def _update_time_labels(self) -> None:
        if self.current_source is None:
            self.current_time_label.setText("00:00:00.000")
            self.duration_time_label.setText("/ 00:00:00.000")
            return
        pos = self.current_source.get_position_seconds()
        self.current_time_label.setText(seconds_to_timecode(pos))

    
    def _sync_timeline_with_position(self) -> None:
        if self.current_source is None:
            return
        pos = self.current_source.get_position_seconds()
        duration = max(0.001, self.current_source.get_duration_seconds())
        value = int(round((pos / duration) * 1000.0))
        value = max(0, min(1000, value))
        self.timeline.blockSignals(True)
        self.timeline.setValue(value)
        self.timeline.blockSignals(False)
        self._update_time_labels()

    
    def _on_scrub_start(self) -> None:
        self.scrubbing = True

    
    def _on_scrub_end(self) -> None:
        self.scrubbing = False
        self._apply_timeline_to_position()

    
    def _on_scrub_moved(self, value: int) -> None:
        self._apply_timeline_to_position()

    
    def _apply_timeline_to_position(self) -> None:
        if self.current_source is None:
            return
        duration = max(0.0, self.current_source.get_duration_seconds())
        value = float(self.timeline.value()) / 1000.0
        target = duration * value
        self.current_source.set_position_seconds(target)
        frame = self.current_source.read_frame()
        if frame is not None:
            self.player.show_frame(frame)
        self._update_time_labels()

    
    def _update_status_tip(self) -> None:
        in_tc = seconds_to_timecode(self.in_out.mark_in or 0.0)
        out_tc = seconds_to_timecode(self.in_out.mark_out or 0.0)
        self.statusBar().showMessage(f"In: {in_tc}   Out: {out_tc}")


 
def main() -> None:
    app = QApplication(sys.argv)
    window = EditorMainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()


