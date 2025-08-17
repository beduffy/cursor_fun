from __future__ import annotations

from dataclasses import dataclass
from typing import List

from PyQt5 import QtCore, QtGui, QtWidgets

from mydaw.tempo_transport import Transport
from mydaw.drums import DrumRack


class StepSequencer(QtWidgets.QWidget):  # pragma: no cover - UI
  def __init__(self, transport: Transport, rack: DrumRack, steps: int = 16, parent=None):
    super().__init__(parent)
    self.transport = transport
    self.rack = rack
    self.steps = steps
    self.cell = 28
    self.setMinimumHeight(4 * self.cell + 10)
    self.setMinimumWidth(self.steps * self.cell + 10)
    self.pattern = [[False for _ in range(steps)] for _ in range(3)]

  def paintEvent(self, ev):
    p = QtGui.QPainter(self)
    p.fillRect(self.rect(), QtGui.QColor(25, 25, 25))
    for row in range(3):
      for col in range(self.steps):
        x = 5 + col * self.cell
        y = 5 + row * self.cell
        r = QtCore.QRect(x, y, self.cell - 6, self.cell - 6)
        on = self.pattern[row][col]
        p.fillRect(r, QtGui.QColor(200, 120, 60) if on else QtGui.QColor(60, 60, 60))
    p.setPen(QtGui.QPen(QtGui.QColor(90, 90, 90)))
    for col in range(self.steps):
      x = 5 + col * self.cell
      p.drawLine(x, 5, x, 5 + 3 * self.cell)

  def mousePressEvent(self, ev):
    col = (ev.x() - 5) // self.cell
    row = (ev.y() - 5) // self.cell
    if 0 <= row < 3 and 0 <= col < self.steps:
      self.pattern[row][col] = not self.pattern[row][col]
      self._sync_to_rack()
      self.update()

  def _sync_to_rack(self):
    self.rack.steps.clear()
    for row in range(3):
      for col in range(self.steps):
        if self.pattern[row][col]:
          beat = col * (1.0 / 4.0)  # 16 steps -> quarter-beat steps
          self.rack.add_step(beat, row)


