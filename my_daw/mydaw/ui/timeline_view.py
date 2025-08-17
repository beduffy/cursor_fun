from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

import numpy as np

from PyQt5 import QtCore, QtGui, QtWidgets

from mydaw import (
  OfflineEngine,
  write_wav_16bit,
  Track,
  Mixer,
  ToneClip,
  AudioClip,
)


@dataclass
class UiClip:
  track_index: int
  start_seconds: float
  duration_seconds: float
  kind: str  # 'tone' or 'audio'
  params: dict
  model_ref: object = None


class TimelineModel:
  def __init__(self) -> None:
    self.tracks: List[Track] = []
    self.ui_clips: List[UiClip] = []
    self.length_seconds: float = 8.0

  def ensure_tracks(self, count: int) -> None:
    while len(self.tracks) < count:
      self.tracks.append(Track(gain=1.0))

  def add_tone(self, track_index: int, start_seconds: float, freq: float, dur: float, amp: float = 0.2) -> None:
    self.ensure_tracks(track_index + 1)
    clip = ToneClip(frequency_hz=freq, duration_seconds=dur, amplitude=amp)
    placed = self.tracks[track_index].add_clip(clip, start_seconds)
    self.ui_clips.append(UiClip(track_index, start_seconds, dur, 'tone', dict(frequency=freq), model_ref=placed))
    self.length_seconds = max(self.length_seconds, start_seconds + dur + 1.0)

  def add_audio(self, track_index: int, start_seconds: float, path: str, amp: float = 1.0) -> None:
    self.ensure_tracks(track_index + 1)
    clip = AudioClip(path, amplitude=amp)
    placed = self.tracks[track_index].add_clip(clip, start_seconds)
    # Duration unknown until load; approximate from file length lazily
    self.ui_clips.append(UiClip(track_index, start_seconds, 0.0, 'audio', dict(path=path), model_ref=placed))
    self.length_seconds = max(self.length_seconds, start_seconds + 1.0)

  def render(self, sr: int, length_seconds: float) -> np.ndarray:
    mixer = Mixer()
    for tr in self.tracks:
      mixer.add_track(tr)
    eng = OfflineEngine(sample_rate=sr)
    return eng.render([mixer], length_seconds)


class TimelineCanvas(QtWidgets.QWidget):
  def __init__(self, model: TimelineModel, parent=None) -> None:
    super().__init__(parent)
    self.model = model
    self.pixels_per_second = 100.0
    self.track_height = 60
    self.setMouseTracking(True)
    self.drag_index: Optional[int] = None
    self.drag_offset: float = 0.0

  def sizeHint(self) -> QtCore.QSize:  # pragma: no cover - UI only
    return QtCore.QSize(1000, 300)

  def _approx_duration(self, uic: UiClip) -> float:
    if uic.kind == 'tone':
      return uic.duration_seconds
    if uic.kind == 'audio':
      # lazy approximate by reading header
      try:
        import wave
        with wave.open(uic.params['path'], 'rb') as wf:
          frames = wf.getnframes()
          sr = wf.getframerate()
        return max(0.01, frames / float(sr))
      except Exception:
        return 1.0
    return 1.0

  def paintEvent(self, ev):  # pragma: no cover - UI only
    painter = QtGui.QPainter(self)
    rect = self.rect()
    painter.fillRect(rect, QtGui.QColor(30, 30, 30))

    # grid/time ruler
    painter.setPen(QtGui.QPen(QtGui.QColor(80, 80, 80)))
    seconds = int(self.model.length_seconds) + 2
    for s in range(seconds):
      x = int(s * self.pixels_per_second)
      painter.drawLine(x, 0, x, rect.height())
      painter.drawText(x + 2, 12, f"{s}s")

    # tracks lanes
    for i in range(max(3, len(self.model.tracks))):
      y = i * self.track_height
      painter.setPen(QtGui.QPen(QtGui.QColor(50, 50, 50)))
      painter.drawLine(0, y + self.track_height - 1, rect.width(), y + self.track_height - 1)

    # clips
    for idx, uic in enumerate(self.model.ui_clips):
      y = uic.track_index * self.track_height
      x = int(uic.start_seconds * self.pixels_per_second)
      w = int(self._approx_duration(uic) * self.pixels_per_second)
      color = QtGui.QColor(80, 150, 255) if uic.kind == 'tone' else QtGui.QColor(120, 200, 120)
      painter.fillRect(QtCore.QRect(x, y + 5, max(10, w), self.track_height - 10), color)
      painter.setPen(QtGui.QPen(QtGui.QColor(20, 20, 20)))
      label = f"Tone {int(uic.params.get('frequency', 0))}Hz" if uic.kind == 'tone' else Path(uic.params['path']).name
      painter.drawText(x + 6, y + 22, label)

  def _hit_test(self, pos: QtCore.QPoint) -> Optional[int]:
    for i, uic in enumerate(self.model.ui_clips):
      y = uic.track_index * self.track_height
      x = int(uic.start_seconds * self.pixels_per_second)
      w = int(self._approx_duration(uic) * self.pixels_per_second)
      rect = QtCore.QRect(x, y + 5, max(10, w), self.track_height - 10)
      if rect.contains(pos):
        return i
    return None

  def mousePressEvent(self, ev):  # pragma: no cover - UI only
    idx = self._hit_test(ev.pos())
    if idx is not None:
      self.drag_index = idx
      uic = self.model.ui_clips[idx]
      clip_x = int(uic.start_seconds * self.pixels_per_second)
      self.drag_offset = ev.pos().x() - clip_x
      self.setCursor(QtCore.Qt.ClosedHandCursor)

  def mouseMoveEvent(self, ev):  # pragma: no cover - UI only
    if self.drag_index is None:
      idx = self._hit_test(ev.pos())
      self.setCursor(QtCore.Qt.OpenHandCursor if idx is not None else QtCore.Qt.ArrowCursor)
      return
    uic = self.model.ui_clips[self.drag_index]
    new_x = max(0, ev.pos().x() - int(self.drag_offset))
    uic.start_seconds = new_x / self.pixels_per_second
    # Update underlying model position so audio render reflects the drag
    if uic.model_ref is not None:
      try:
        uic.model_ref.start_seconds = float(uic.start_seconds)
      except Exception:
        pass
    self.model.length_seconds = max(self.model.length_seconds, uic.start_seconds + self._approx_duration(uic) + 1.0)
    self.update()

  def mouseReleaseEvent(self, ev):  # pragma: no cover - UI only
    self.drag_index = None
    self.setCursor(QtCore.Qt.ArrowCursor)


class TimelineWindow(QtWidgets.QMainWindow):  # pragma: no cover - UI only
  def __init__(self) -> None:
    super().__init__()
    self.setWindowTitle("my_daw — Timeline")
    self.model = TimelineModel()

    self.canvas = TimelineCanvas(self.model)
    self.setCentralWidget(self.canvas)

    self._build_toolbar()

  def _build_toolbar(self) -> None:
    tb = self.addToolBar("Main")
    add_tone = QtWidgets.QAction("Add Tone", self)
    add_tone.triggered.connect(self._on_add_tone)
    tb.addAction(add_tone)

    add_audio = QtWidgets.QAction("Add Audio", self)
    add_audio.triggered.connect(self._on_add_audio)
    tb.addAction(add_audio)

    render = QtWidgets.QAction("Export WAV", self)
    render.triggered.connect(self._on_export)
    tb.addAction(render)

  def _on_add_tone(self):
    freq, ok = QtWidgets.QInputDialog.getInt(self, "Tone Frequency", "Hz:", 440, 20, 20000, 1)
    if not ok:
      return
    dur, ok = QtWidgets.QInputDialog.getDouble(self, "Duration", "seconds:", 1.0, 0.05, 60.0, 2)
    if not ok:
      return
    track, ok = QtWidgets.QInputDialog.getInt(self, "Track", "index:", 0, 0, 64, 1)
    if not ok:
      return
    start, ok = QtWidgets.QInputDialog.getDouble(self, "Start", "seconds:", 0.0, 0.0, 600.0, 2)
    if not ok:
      return
    self.model.add_tone(track, start, float(freq), float(dur))
    self.canvas.update()

  def _on_add_audio(self):
    path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Open WAV", str(Path.home()), "WAV Files (*.wav)")
    if not path:
      return
    track, ok = QtWidgets.QInputDialog.getInt(self, "Track", "index:", 0, 0, 64, 1)
    if not ok:
      return
    start, ok = QtWidgets.QInputDialog.getDouble(self, "Start", "seconds:", 0.0, 0.0, 600.0, 2)
    if not ok:
      return
    self.model.add_audio(track, start, path)
    self.canvas.update()

  def _on_export(self):
    length, ok = QtWidgets.QInputDialog.getDouble(self, "Export Length", "seconds:", max(5.0, self.model.length_seconds), 0.1, 3600.0, 1)
    if not ok:
      return
    samples = self.model.render(44100, float(length))
    out_path = Path(__file__).resolve().parents[2] / "output_ui_export.wav"
    write_wav_16bit(str(out_path), samples, 44100)
    QtWidgets.QMessageBox.information(self, "Export", f"Wrote {out_path}")


