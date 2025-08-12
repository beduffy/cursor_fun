from __future__ import annotations

# Minimal optional GUI stub using PySide6. The core remains testable without GUI.
from .models.document import Document
from .models.entities import LineEntity, CircleEntity, RectEntity, PolylineEntity
from .geometry import Point


def main() -> int:  # pragma: no cover - GUI
  try:
    from PySide6.QtCore import Qt, QPointF
    from PySide6.QtGui import QPainter, QPen, QColor
    from PySide6.QtWidgets import QApplication, QWidget
  except Exception:
    print("PySide6 not installed. Install to run GUI: pip install PySide6")
    return 1

  class Canvas(QWidget):  # type: ignore[misc]
    def __init__(self, doc: Document):
      super().__init__()
      self.doc = doc
      self.setWindowTitle("My Own CAD (2D)")
      self.resize(1000, 700)
      self._scale = 1.0
      self._origin = QPointF(0, 0)
      self._panning = False
      self._last_mouse = QPointF(0, 0)

    def paintEvent(self, event):  # type: ignore[override]
      painter = QPainter(self)
      painter.setRenderHint(QPainter.Antialiasing)
      pen = QPen()
      pen.setWidth(2)
      painter.setPen(pen)
      painter.translate(self._origin)
      painter.scale(self._scale, self._scale)

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

    def wheelEvent(self, event):  # type: ignore[override]
      delta = event.angleDelta().y()
      factor = 1.0 + (0.1 if delta > 0 else -0.1)
      self._scale = max(0.1, min(10.0, self._scale * factor))
      self.update()

    def mousePressEvent(self, event):  # type: ignore[override]
      if event.button() == Qt.MiddleButton:
        self._panning = True
        self._last_mouse = event.position()

    def mouseReleaseEvent(self, event):  # type: ignore[override]
      if event.button() == Qt.MiddleButton:
        self._panning = False

    def mouseMoveEvent(self, event):  # type: ignore[override]
      if self._panning:
        delta = event.position() - self._last_mouse
        self._origin += delta
        self._last_mouse = event.position()
        self.update()

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
