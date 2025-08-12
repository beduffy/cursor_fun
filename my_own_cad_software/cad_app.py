from __future__ import annotations

# Minimal optional GUI stub using PySide6. The core remains testable without GUI.
try:
  from PySide6.QtCore import Qt, QPointF
  from PySide6.QtGui import QPainter, QPen
  from PySide6.QtWidgets import QApplication, QWidget
except Exception as e:  # pragma: no cover - optional dependency
  Qt = None  # type: ignore

from .models.document import Document
from .models.entities import LineEntity
from .geometry import Point


class Canvas(QWidget):  # pragma: no cover - GUI smoke-tested manually
  def __init__(self, doc: Document):
    super().__init__()
    self.doc = doc
    self.setWindowTitle("My Own CAD (2D)")
    self.resize(800, 600)

  def paintEvent(self, event):
    painter = QPainter(self)
    painter.setRenderHint(QPainter.Antialiasing)
    pen = QPen()
    pen.setWidth(2)
    painter.setPen(pen)

    # Draw all line entities for demo
    for ent in self.doc.list_entities():
      if isinstance(ent, LineEntity):
        painter.drawLine(
          QPointF(ent.start.x, ent.start.y),
          QPointF(ent.end.x, ent.end.y)
        )


def main() -> int:  # pragma: no cover - GUI
  if Qt is None:
    print("PySide6 not installed. Install to run GUI: pip install PySide6")
    return 1

  app = QApplication([])
  doc = Document()
  # demo: add a couple of lines
  doc.add_entity(LineEntity(id=doc.create_id(), start=Point(50, 50), end=Point(300, 50)))
  doc.add_entity(LineEntity(id=doc.create_id(), start=Point(50, 100), end=Point(300, 300)))

  canvas = Canvas(doc)
  canvas.show()
  return app.exec()


if __name__ == "__main__":  # pragma: no cover
  raise SystemExit(main())
