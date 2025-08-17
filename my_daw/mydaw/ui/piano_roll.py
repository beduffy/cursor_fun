from __future__ import annotations

from typing import List, Optional

from PyQt5 import QtCore, QtGui, QtWidgets

from mydaw.midi_clip import MidiNoteClip


class PianoRoll(QtWidgets.QWidget):  # pragma: no cover - UI only
  def __init__(self, parent=None, steps: int = 16, rows: int = 24, lowest_midi: int = 48):
    super().__init__(parent)
    self.steps = steps
    self.rows = rows
    self.lowest_midi = lowest_midi
    self.cell = 22
    self.setMinimumSize(self.steps * self.cell + 60, self.rows * self.cell + 10)
    self.clip: Optional[MidiNoteClip] = None
    # grid state: rows (pitch) x cols (step)
    self.pattern = [[False for _ in range(self.steps)] for _ in range(self.rows)]
    self.velocity = 1.0

    self._vel_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal, self)
    self._vel_slider.setGeometry(5, 5 + self.rows * self.cell + 5, self.steps * self.cell, 18)
    self._vel_slider.setRange(1, 127)
    self._vel_slider.setValue(100)
    self._vel_slider.valueChanged.connect(self._on_velocity)

  def bind_clip(self, clip: Optional[MidiNoteClip]):
    self.clip = clip
    # Rebuild pattern from clip
    for r in range(self.rows):
      for c in range(self.steps):
        self.pattern[r][c] = False
    if clip is not None:
      for n in clip.notes:
        step = int(round(n.start_beats * 4.0))  # 16th grid
        row = n.midi_note - self.lowest_midi
        if 0 <= row < self.rows and 0 <= step < self.steps:
          self.pattern[row][step] = True
    self.update()

  def paintEvent(self, ev):
    p = QtGui.QPainter(self)
    p.fillRect(self.rect(), QtGui.QColor(28, 28, 28))
    # grid
    for r in range(self.rows):
      y = 5 + r * self.cell
      shade = 36 if ((self.lowest_midi + r) % 12) in (1,3,6,8,10) else 32
      p.fillRect(QtCore.QRect(5, y, self.steps * self.cell, self.cell-2), QtGui.QColor(40,40,40,160))
    p.setPen(QtGui.QPen(QtGui.QColor(70,70,70)))
    for c in range(self.steps+1):
      x = 5 + c * self.cell
      p.drawLine(x, 5, x, 5 + self.rows * self.cell)
    # notes
    for r in range(self.rows):
      for c in range(self.steps):
        if self.pattern[r][c]:
          x = 5 + c * self.cell
          y = 5 + r * self.cell
          p.fillRect(QtCore.QRect(x+1, y+1, self.cell-4, self.cell-4), QtGui.QColor(120,180,255))

  def mousePressEvent(self, ev):
    c = (ev.x() - 5) // self.cell
    r = (ev.y() - 5) // self.cell
    if 0 <= r < self.rows and 0 <= c < self.steps:
      self.pattern[r][c] = not self.pattern[r][c]
      self._sync_to_clip()
      self.update()

  def _sync_to_clip(self):
    if self.clip is None:
      return
    # rebuild notes from pattern on a 16th grid across 4 beats
    self.clip.notes.clear()
    for r in range(self.rows):
      midi_note = self.lowest_midi + r
      for c in range(self.steps):
        if self.pattern[r][c]:
          start_beats = c / 4.0
          duration_beats = 0.25
          self.clip.add_note(start_beats, duration_beats, midi_note, velocity=self.velocity)

  def _on_velocity(self, val: int):
    self.velocity = max(0.0, min(1.0, val / 127.0))


