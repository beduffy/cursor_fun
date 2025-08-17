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
from mydaw.tempo_transport import Transport
from mydaw.drums import DrumRack
from .step_sequencer import StepSequencer
from .piano_roll import PianoRoll
from mydaw.midi_clip import MidiNoteClip
from mydaw.stereo import render_tracks_stereo
from mydaw.ui.mixer_dock import MixerDock
from mydaw.clip_utils import clone_clip


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
    self.pans: List[float] = []  # per-track pan (-1..+1)
    self.muted: List[bool] = []
    self.soloed: List[bool] = []
    self.track_colors: List[QtGui.QColor] = []
    self.loop_start: float = 0.0
    self.loop_length: float = 0.0  # 0 means no loop

  def ensure_tracks(self, count: int) -> None:
    while len(self.tracks) < count:
      self.tracks.append(Track(gain=1.0))
      self.pans.append(0.0)
      self.muted.append(False)
      self.soloed.append(False)
      self.track_colors.append(QtGui.QColor(80, 150, 255))

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
    total_samples = int(round(length_seconds * sr))
    # Apply mute/solo filtering
    tracks = []
    pans = []
    any_solo = any(self.soloed)
    for i, tr in enumerate(self.tracks):
      if any_solo and not self.soloed[i]:
        continue
      if not any_solo and self.muted[i]:
        continue
      tracks.append(tr)
      pans.append(self.pans[i])
    return render_tracks_stereo(tracks, sr, total_samples, pans)


class TimelineCanvas(QtWidgets.QWidget):
  def __init__(self, model: TimelineModel, parent=None) -> None:
    super().__init__(parent)
    self.model = model
    self.pixels_per_second = 100.0
    self.track_height = 60
    self.setMouseTracking(True)
    self.drag_index: Optional[int] = None
    self.drag_offset: float = 0.0
    self.selected_index: Optional[int] = None
    self.resizing: bool = False
    self.resize_edge_right_margin = 6
    self.scroll_x_seconds: float = 0.0

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
      x = int((s - self.scroll_x_seconds) * self.pixels_per_second)
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
      x = int((uic.start_seconds - self.scroll_x_seconds) * self.pixels_per_second)
      w = int(self._approx_duration(uic) * self.pixels_per_second)
      base_color = self.model.track_colors[uic.track_index] if uic.track_index < len(self.model.track_colors) else QtGui.QColor(80,150,255)
      # dim if muted when no solo, or when other track soloed
      any_solo = any(self.model.soloed) if self.model.soloed else False
      dim = (not any_solo and self.model.muted[uic.track_index]) or (any_solo and not self.model.soloed[uic.track_index]) if self.model.muted and self.model.soloed else False
      color = QtGui.QColor(base_color)
      if dim:
        color.setAlpha(120)
      painter.fillRect(QtCore.QRect(x, y + 5, max(10, w), self.track_height - 10), color)
      painter.setPen(QtGui.QPen(QtGui.QColor(20, 20, 20)))
      if uic.kind == 'tone':
        label = f"Tone {int(uic.params.get('frequency', 0))}Hz"
      elif uic.kind == 'audio':
        path = uic.params.get('path', '')
        label = Path(path).name if path else 'Audio'
      elif uic.kind == 'midi':
        label = 'MIDI Clip'
      else:
        label = 'Clip'
      painter.drawText(x + 6, y + 22, label)
      # draw selection outline and resize handle
      if self.selected_index == idx:
        painter.setPen(QtGui.QPen(QtGui.QColor(255, 255, 255)))
        painter.drawRect(QtCore.QRect(x, y + 5, max(10, w), self.track_height - 10))
      painter.setPen(QtGui.QPen(QtGui.QColor(10,10,10)))
      painter.drawLine(x + max(10, w) - self.resize_edge_right_margin, y + 5, x + max(10, w) - self.resize_edge_right_margin, y + self.track_height - 5)

  def _hit_test(self, pos: QtCore.QPoint) -> Optional[int]:
    for i, uic in enumerate(self.model.ui_clips):
      y = uic.track_index * self.track_height
      x = int((uic.start_seconds - self.scroll_x_seconds) * self.pixels_per_second)
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
      clip_x = int((uic.start_seconds - self.scroll_x_seconds) * self.pixels_per_second)
      self.drag_offset = ev.pos().x() - clip_x
      # detect resize near right edge
      w = int(self._approx_duration(uic) * self.pixels_per_second)
      if ev.pos().x() >= clip_x + max(10, w) - self.resize_edge_right_margin:
        self.resizing = True
      self.selected_index = idx
      self.setCursor(QtCore.Qt.ClosedHandCursor)
    else:
      # start panning if right button
      if ev.button() == QtCore.Qt.RightButton:
        self.setCursor(QtCore.Qt.SizeAllCursor)

  def mouseMoveEvent(self, ev):  # pragma: no cover - UI only
    if self.drag_index is None:
      idx = self._hit_test(ev.pos())
      self.setCursor(QtCore.Qt.OpenHandCursor if idx is not None else QtCore.Qt.ArrowCursor)
      return
    uic = self.model.ui_clips[self.drag_index]
    new_x = max(0, ev.pos().x() - int(self.drag_offset))
    if self.resizing:
      # adjust duration
      new_w = max(0.05, (new_x + self.resize_edge_right_margin) / self.pixels_per_second + self.scroll_x_seconds - uic.start_seconds)
      uic.duration_seconds = float(new_w)
      # propagate to underlying clip if ToneClip
      if uic.kind == 'tone' and hasattr(uic.model_ref.clip, 'duration_seconds'):
        uic.model_ref.clip.duration_seconds = float(new_w)
    else:
      # move clip
      uic.start_seconds = new_x / self.pixels_per_second + self.scroll_x_seconds
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
    self.resizing = False
    self.setCursor(QtCore.Qt.ArrowCursor)

  def wheelEvent(self, ev):  # pragma: no cover - UI only
    # zoom with Ctrl+wheel, scroll with wheel
    delta = ev.angleDelta().y()
    if ev.modifiers() & QtCore.Qt.ControlModifier:
      factor = 1.0 + (0.1 if delta > 0 else -0.1)
      self.pixels_per_second = max(20.0, min(800.0, self.pixels_per_second * factor))
    else:
      self.scroll_x_seconds = max(0.0, self.scroll_x_seconds - delta / 120.0 * (1.0 / 3.0))
    self.update()


class TimelineWindow(QtWidgets.QMainWindow):  # pragma: no cover - UI only
  def __init__(self) -> None:
    super().__init__()
    self.setWindowTitle("my_daw — Timeline")
    self.model = TimelineModel()
    self.transport = Transport(bpm=120.0)
    self.drum_rack = DrumRack(self.transport)

    self.canvas = TimelineCanvas(self.model)
    self.setCentralWidget(self.canvas)

    self._build_toolbar()
    self._build_dock_sequencer()
    self._build_dock_mixer()

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

    add_midi = QtWidgets.QAction("Add MIDI Track", self)
    add_midi.triggered.connect(self._on_add_midi_track)
    tb.addAction(add_midi)

    dup_track = QtWidgets.QAction("Duplicate Track", self)
    dup_track.triggered.connect(self._on_duplicate_track)
    tb.addAction(dup_track)

    set_tempo = QtWidgets.QAction("Set Tempo", self)
    set_tempo.triggered.connect(self._on_set_tempo)
    tb.addAction(set_tempo)

    set_loop = QtWidgets.QAction("Set Loop", self)
    set_loop.triggered.connect(self._on_set_loop)
    tb.addAction(set_loop)

    dup_clip = QtWidgets.QAction("Duplicate Clip Right", self)
    dup_clip.triggered.connect(self._on_duplicate_clip_right)
    tb.addAction(dup_clip)

    color_track = QtWidgets.QAction("Set Track Color", self)
    color_track.triggered.connect(self._on_set_track_color)
    tb.addAction(color_track)

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
    default_len = self.model.loop_length if self.model.loop_length > 0 else max(5.0, self.model.length_seconds)
    length, ok = QtWidgets.QInputDialog.getDouble(self, "Export Length", "seconds:", default_len, 0.1, 3600.0, 1)
    if not ok:
      return
    # include drum rack by mixing it on a new track on the fly
    temp_model = TimelineModel()
    # Copy existing tracks/clips by rendering through a mixer along with a temp track for drums
    samples_timeline = self.model.render(44100, float(length))  # now stereo (2,N)
    samples_drums = self.drum_rack.render(44100)  # mono
    # Mix drums mono into stereo
    n = max(samples_timeline.shape[1], samples_drums.shape[0])
    out = np.zeros((2, n), dtype=np.float32)
    out[:, :samples_timeline.shape[1]] += samples_timeline
    out[:, :samples_drums.shape[0]] += np.stack([samples_drums, samples_drums], axis=0)
    samples = np.clip(out, -1.0, 1.0)
    out_path = Path(__file__).resolve().parents[2] / "output_ui_export.wav"
    write_wav_16bit(str(out_path), samples, 44100)
    QtWidgets.QMessageBox.information(self, "Export", f"Wrote {out_path}")

  def _build_dock_sequencer(self):
    dock = QtWidgets.QDockWidget("Step Sequencer", self)
    dock.setAllowedAreas(QtCore.Qt.BottomDockWidgetArea)
    seq = StepSequencer(self.transport, self.drum_rack, steps=16, parent=dock)
    dock.setWidget(seq)
    self.addDockWidget(QtCore.Qt.BottomDockWidgetArea, dock)

    # Piano roll dock
    dock_pr = QtWidgets.QDockWidget("Piano Roll (selected MIDI)", self)
    dock_pr.setAllowedAreas(QtCore.Qt.BottomDockWidgetArea)
    self.piano_roll = PianoRoll(parent=dock_pr, steps=16, rows=24, lowest_midi=48)
    dock_pr.setWidget(self.piano_roll)
    self.addDockWidget(QtCore.Qt.BottomDockWidgetArea, dock_pr)

  def _build_dock_mixer(self):
    dock = MixerDock(self, self.model)
    self.addDockWidget(QtCore.Qt.RightDockWidgetArea, dock)

  def _on_add_midi_track(self):
    # Create a MIDI clip at track 0 by default and bind piano roll to it
    track_idx = 0
    self.model.ensure_tracks(track_idx + 1)
    midi_clip = MidiNoteClip(self.transport)
    placed = self.model.tracks[track_idx].add_clip(midi_clip, 0.0)
    self.model.ui_clips.append(UiClip(track_idx, 0.0, 2.0, 'midi', dict(), model_ref=placed))
    self.piano_roll.bind_clip(midi_clip)
    self.canvas.update()

  def _on_duplicate_track(self):
    if not self.model.tracks:
      return
    # duplicate last track as a simple UX
    src_idx = len(self.model.tracks) - 1
    self.model.ensure_tracks(src_idx + 2)
    dst_idx = src_idx + 1
    # copy pan
    self.model.pans[dst_idx] = self.model.pans[src_idx]
    # duplicate clips
    for uic in list(self.model.ui_clips):
      if uic.track_index == src_idx and uic.model_ref is not None:
        new_clip = clone_clip(uic.model_ref.clip)
        placed = self.model.tracks[dst_idx].add_clip(new_clip, uic.start_seconds)
        self.model.ui_clips.append(UiClip(dst_idx, uic.start_seconds, uic.duration_seconds, uic.kind, dict(uic.params), model_ref=placed))
    self.canvas.update()

  def _on_set_tempo(self):
    bpm, ok = QtWidgets.QInputDialog.getInt(self, "Set Tempo", "BPM:", int(self.transport.bpm), 20, 300, 1)
    if not ok:
      return
    self.transport.bpm = float(bpm)

  def _on_set_loop(self):
    start, ok = QtWidgets.QInputDialog.getDouble(self, "Loop Start", "seconds:", float(self.model.loop_start), 0.0, 3600.0, 2)
    if not ok:
      return
    length, ok = QtWidgets.QInputDialog.getDouble(self, "Loop Length", "seconds:", float(self.model.loop_length or 8.0), 0.0, 3600.0, 2)
    if not ok:
      return
    self.model.loop_start = float(start)
    self.model.loop_length = float(length)

  def _on_duplicate_clip_right(self):
    if self.canvas.selected_index is None:
      return
    uic = self.model.ui_clips[self.canvas.selected_index]
    # place new clip immediately to the right
    new_start = uic.start_seconds + self.canvas._approx_duration(uic)
    new_clip = clone_clip(uic.model_ref.clip) if uic.model_ref is not None else None
    if new_clip is None:
      return
    placed = self.model.tracks[uic.track_index].add_clip(new_clip, new_start)
    self.model.ui_clips.append(UiClip(uic.track_index, new_start, uic.duration_seconds, uic.kind, dict(uic.params), model_ref=placed))
    self.canvas.update()

  def _on_set_track_color(self):
    idx, ok = QtWidgets.QInputDialog.getInt(self, "Track Index", "index:", 0, 0, max(0, len(self.model.tracks)-1), 1)
    if not ok:
      return
    color = QtWidgets.QColorDialog.getColor(parent=self)
    if not color.isValid():
      return
    if idx >= len(self.model.track_colors):
      self.model.ensure_tracks(idx + 1)
    self.model.track_colors[idx] = color
    self.canvas.update()


