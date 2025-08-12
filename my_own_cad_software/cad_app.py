from __future__ import annotations

# Minimal optional GUI stub using PySide6. The core remains testable without GUI.
try:
  from PySide6.QtCore import Qt, QPointF
from PySide6.QtGui import QPainter, QPen, QColor
  from PySide6.QtWidgets import QApplication, QWidget
except Exception as e:  # pragma: no cover - optional dependency
  Qt = None  # type: ignore

from .models.document import Document
from .models.entities import LineEntity, CircleEntity, RectEntity, PolylineEntity
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

    # Draw all visible entities for demo
    for ent in self.doc.iter_entities(visible_only=True):
      pen.setColor(QColor(ent.color))
      painter.setPen(pen)
      if isinstance(ent, LineEntity):
        painter.drawLine(QPointF(ent.start.x, ent.start.y), QPointF(ent.end.x, ent.end.y))
      elif isinstance(ent, CircleEntity):
        painter.drawEllipse(QPointF(ent.center.x, ent.center.y), ent.radius, ent.radius)
      elif isinstance(ent, RectEntity):
        min_pt, max_pt = ent.bbox()
        painter.drawRect(min_pt.x, min_pt.y, max_pt.x - min_pt.x, max_pt.y - min_pt.y)
      elif isinstance(ent, PolylineEntity):
        for a, b in zip(ent.points[:-1], ent.points[1:]):
          painter.drawLine(QPointF(a.x, a.y), QPointF(b.x, b.y))


def main() -> int:  # pragma: no cover - GUI
  if Qt is None:
    print("PySide6 not installed. Install to run GUI: pip install PySide6")
    return 1

  app = QApplication([])
  doc = Document()
  # demo: add a few shapes
  doc.add_entity(LineEntity(id=doc.create_id(), start=Point(50, 50), end=Point(300, 50), color="#cc0000"))
  doc.add_entity(LineEntity(id=doc.create_id(), start=Point(50, 100), end=Point(300, 300), color="#00aa00"))

  canvas = Canvas(doc)
  canvas.show()
  return app.exec()


if __name__ == "__main__":  # pragma: no cover
  raise SystemExit(main())
