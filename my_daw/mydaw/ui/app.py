from __future__ import annotations

import sys
from pathlib import Path

try:
  from PyQt5 import QtWidgets
except Exception as exc:  # pragma: no cover - UI bootstrap
  print("PyQt5 is required. Install with: pip install --user PyQt5")
  raise

from .timeline_view import TimelineWindow


def main() -> None:
  app = QtWidgets.QApplication(sys.argv)
  window = TimelineWindow()
  window.resize(1000, 480)
  window.show()
  sys.exit(app.exec_())


if __name__ == "__main__":
  main()


