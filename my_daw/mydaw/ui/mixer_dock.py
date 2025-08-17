from __future__ import annotations

from typing import List

import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets

from mydaw.meters import peak_meter_mono


class MixerDock(QtWidgets.QDockWidget):  # pragma: no cover - UI only
  def __init__(self, parent, timeline_model):
    super().__init__("Mixer", parent)
    self.model = timeline_model
    self.setAllowedAreas(QtCore.Qt.RightDockWidgetArea)
    self.panel = QtWidgets.QWidget(self)
    self.setWidget(self.panel)
    self.layout = QtWidgets.QVBoxLayout(self.panel)
    self.layout.setContentsMargins(6, 6, 6, 6)
    self._rows: List[tuple[QtWidgets.QSlider, QtWidgets.QSlider, QtWidgets.QCheckBox, QtWidgets.QCheckBox, QtWidgets.QProgressBar]] = []
    self.rebuild()

  def rebuild(self):
    # clear
    while self.layout.count():
      item = self.layout.takeAt(0)
      w = item.widget()
      if w:
        w.deleteLater()
    self._rows.clear()

    for idx, track in enumerate(self.model.tracks):
      row = QtWidgets.QHBoxLayout()
      label = QtWidgets.QLabel(f"Track {idx}")
      row.addWidget(label)

      gain = QtWidgets.QSlider(QtCore.Qt.Horizontal)
      gain.setRange(0, 200)
      gain.setValue(int(track.gain * 100))
      gain.valueChanged.connect(lambda v, i=idx: self._on_gain(i, v))
      row.addWidget(gain)

      pan = QtWidgets.QSlider(QtCore.Qt.Horizontal)
      pan.setRange(-100, 100)
      pan.setValue(int(self.model.pans[idx] * 100))
      pan.valueChanged.connect(lambda v, i=idx: self._on_pan(i, v))
      row.addWidget(pan)

      mute = QtWidgets.QCheckBox("M")
      mute.setChecked(self.model.muted[idx])
      mute.stateChanged.connect(lambda state, i=idx: self._on_mute(i, state))
      row.addWidget(mute)

      solo = QtWidgets.QCheckBox("S")
      solo.setChecked(self.model.soloed[idx])
      solo.stateChanged.connect(lambda state, i=idx: self._on_solo(i, state))
      row.addWidget(solo)

      meter = QtWidgets.QProgressBar()
      meter.setRange(0, 100)
      meter.setFormat("%p%")
      row.addWidget(meter)

      self.layout.addLayout(row)
      self._rows.append((gain, pan, mute, solo, meter))

    # meter refresh button
    btn = QtWidgets.QPushButton("Compute Meters")
    btn.clicked.connect(self._on_meters)
    self.layout.addWidget(btn)

    self.layout.addStretch(1)

  def _on_gain(self, idx: int, val: int):
    self.model.tracks[idx].gain = float(val) / 100.0

  def _on_pan(self, idx: int, val: int):
    self.model.pans[idx] = float(val) / 100.0

  def _on_meters(self):
    # crude: compute peak per track over current project length
    sr = 44100
    total_samples = int(self.model.length_seconds * sr)
    for idx, track in enumerate(self.model.tracks):
      mono = track.generate(sr, total_samples)
      peak = peak_meter_mono(mono)
      pct = int(min(100, max(0, round(peak * 100))))
      self._rows[idx][4].setValue(pct)

  def _on_mute(self, idx: int, state: int):
    self.model.muted[idx] = (state == QtCore.Qt.Checked)

  def _on_solo(self, idx: int, state: int):
    self.model.soloed[idx] = (state == QtCore.Qt.Checked)


