from __future__ import annotations

# Minimal optional GUI stub using PySide6. The core remains testable without GUI.
from .models.document import Document
from .models.entities import LineEntity, CircleEntity, RectEntity, PolylineEntity
from .models.command_stack import CommandStack
from .models.commands import AddEntityCommand, RemoveEntityCommand, MoveEntityCommand
from .selection import find_entity_at_point, snap_point
from .io.serializer import save_document_to_json_file, load_document_from_json_file
from .geometry import Point


def main() -> int:  # pragma: no cover - GUI
  try:
    from PySide6.QtCore import Qt, QPointF
    from PySide6.QtGui import QPainter, QPen, QColor, QAction
    from PySide6.QtWidgets import QApplication, QWidget, QMainWindow, QFileDialog, QToolBar, QMessageBox, QStatusBar
  except Exception:
    print("PySide6 not installed. Install to run GUI: pip install PySide6")
    return 1

  class Canvas(QWidget):  # type: ignore[misc]
    def __init__(self, doc: Document, stack: CommandStack):
      super().__init__()
      self.doc = doc
      self.stack = stack
      self.setWindowTitle("My Own CAD (2D)")
      self.resize(1000, 700)
      self._scale = 1.0
      self._origin = QPointF(0, 0)
      self._panning = False
      self._last_mouse = QPointF(0, 0)
      # UI/tool state
      self.active_tool = "select"  # select|line|circle|rect|polyline
      self.snap_endpoint = True
      self.snap_midpoint = True
      self.snap_center = True
      self.snap_grid = True
      self.grid_size = 25.0
      self.show_grid = True
      self.status_callback = None  # type: ignore
      # transient tool points
      self._points: list[QPointF] = []
      self._mouse_pos: QPointF = QPointF(0, 0)
      self._selected_id: int | None = None

    def paintEvent(self, event):  # type: ignore[override]
      painter = QPainter(self)
      painter.setRenderHint(QPainter.Antialiasing)
      pen = QPen()
      pen.setWidth(2)
      painter.setPen(pen)
      painter.translate(self._origin)
      painter.scale(self._scale, self._scale)

      # Grid
      if self.show_grid and self.grid_size > 0:
        grid_pen = QPen(QColor("#dddddd"))
        grid_pen.setWidth(1)
        painter.setPen(grid_pen)
        step = int(self.grid_size)
        w = int(self.width() / max(0.1, self._scale)) + 2 * step
        h = int(self.height() / max(0.1, self._scale)) + 2 * step
        for x in range(0, w, step):
          painter.drawLine(x, 0, x, h)
        for y in range(0, h, step):
          painter.drawLine(0, y, w, y)
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

      # Selection highlight
      if self._selected_id is not None:
        sel = self.doc.get_entity(self._selected_id)
        if sel is not None:
          sel_pen = QPen(QColor("#0077ff"))
          sel_pen.setWidth(3)
          painter.setPen(sel_pen)
          if isinstance(sel, LineEntity):
            painter.drawLine(QPointF(sel.start.x, sel.start.y), QPointF(sel.end.x, sel.end.y))
          elif isinstance(sel, CircleEntity):
            painter.drawEllipse(QPointF(sel.center.x, sel.center.y), sel.radius, sel.radius)
          elif isinstance(sel, RectEntity):
            min_pt, max_pt = sel.bbox()
            painter.drawRect(min_pt.x, min_pt.y, max_pt.x - min_pt.x, max_pt.y - min_pt.y)
          elif isinstance(sel, PolylineEntity):
            for a, b in zip(sel.points[:-1], sel.points[1:]):
              painter.drawLine(QPointF(a.x, a.y), QPointF(b.x, b.y))

      # Tool preview
      prev_pen = QPen(QColor("#555555"))
      prev_pen.setStyle(Qt.DashLine)
      painter.setPen(prev_pen)
      if self.active_tool == "line" and len(self._points) == 1:
        a = self._points[0]
        b = self._mouse_pos
        painter.drawLine(a, b)
      elif self.active_tool == "circle" and len(self._points) == 1:
        a = self._points[0]
        b = self._mouse_pos
        r = ((a.x() - b.x()) ** 2 + (a.y() - b.y()) ** 2) ** 0.5
        painter.drawEllipse(a, r, r)
      elif self.active_tool == "rect" and len(self._points) == 1:
        a = self._points[0]
        b = self._mouse_pos
        x = min(a.x(), b.x())
        y = min(a.y(), b.y())
        w = abs(a.x() - b.x())
        h = abs(a.y() - b.y())
        painter.drawRect(x, y, w, h)
      elif self.active_tool == "polyline" and len(self._points) >= 1:
        a = self._points[-1]
        b = self._mouse_pos
        painter.drawLine(a, b)

      # Snap marker
      snapped = self._snap_current()
      if snapped is not None:
        sm = QPen(QColor("#ff7700"))
        sm.setWidth(2)
        painter.setPen(sm)
        painter.drawEllipse(QPointF(snapped.x, snapped.y), 3, 3)

    def wheelEvent(self, event):  # type: ignore[override]
      delta = event.angleDelta().y()
      factor = 1.0 + (0.1 if delta > 0 else -0.1)
      self._scale = max(0.1, min(10.0, self._scale * factor))
      self.update()

    def mousePressEvent(self, event):  # type: ignore[override]
      if event.button() == Qt.MiddleButton:
        self._panning = True
        self._last_mouse = event.position()
        return
      if event.button() == Qt.LeftButton:
        world = self._to_world(event.position())
        self._handle_left_click(world)

    def mouseReleaseEvent(self, event):  # type: ignore[override]
      if event.button() == Qt.MiddleButton:
        self._panning = False

    def mouseMoveEvent(self, event):  # type: ignore[override]
      if self._panning:
        delta = event.position() - self._last_mouse
        self._origin += delta
        self._last_mouse = event.position()
        self.update()
        return
      self._mouse_pos = self._to_world(event.position())
      if self.status_callback:
        sp = self._snap_current()
        self.status_callback(self.active_tool, self._mouse_pos, sp)
      self.update()

    def keyPressEvent(self, event):  # type: ignore[override]
      if event.key() == Qt.Key_Escape:
        self._points.clear()
        self.update()
      elif event.key() == Qt.Key_Delete and self._selected_id is not None:
        try:
          self.stack.push_and_do(RemoveEntityCommand(self.doc, self._selected_id))
          self._selected_id = None
          self.update()
        except Exception as e:
          pass

    def _to_world(self, screen_pt: QPointF) -> QPointF:
      inv_scale = 1.0 / max(1e-6, self._scale)
      p = (screen_pt - self._origin) * inv_scale
      return QPointF(p.x(), p.y())

    def _snap_current(self):
      modes: list[str] = []
      if self.snap_endpoint: modes.append("endpoint")
      if self.snap_midpoint: modes.append("midpoint")
      if self.snap_center: modes.append("center")
      grid = self.grid_size if self.snap_grid else 0.0
      from .geometry import Point as GPoint
      res = snap_point(GPoint(self._mouse_pos.x(), self._mouse_pos.y()), self.doc, modes=modes, tolerance=10.0, grid_size=grid, visible_only=True)
      return res.point if res else None

    def _handle_left_click(self, world: QPointF) -> None:
      from .geometry import Point as GPoint
      # Selection if tool is select
      if self.active_tool == "select":
        ent = find_entity_at_point(self.doc, GPoint(world.x(), world.y()), tolerance=6.0, visible_only=True)
        self._selected_id = ent.id if ent else None
        self.update()
        return

      # Drawing tools
      snapped = self._snap_current()
      p = QPointF(snapped.x, snapped.y) if snapped is not None else world
      self._points.append(p)
      if self.active_tool == "line" and len(self._points) == 2:
        a, b = self._points
        ent = LineEntity(id=self.doc.create_id(), start=GPoint(a.x(), a.y()), end=GPoint(b.x(), b.y()))
        self.stack.push_and_do(AddEntityCommand(self.doc, ent))
        self._points.clear()
      elif self.active_tool == "circle" and len(self._points) == 2:
        a, b = self._points
        r = ((a.x() - b.x()) ** 2 + (a.y() - b.y()) ** 2) ** 0.5
        ent = CircleEntity(id=self.doc.create_id(), center=GPoint(a.x(), a.y()), radius=r)
        self.stack.push_and_do(AddEntityCommand(self.doc, ent))
        self._points.clear()
      elif self.active_tool == "rect" and len(self._points) == 2:
        a, b = self._points
        min_x, min_y = min(a.x(), b.x()), min(a.y(), b.y())
        max_x, max_y = max(a.x(), b.x()), max(a.y(), b.y())
        ent = RectEntity(id=self.doc.create_id(), min_pt=GPoint(min_x, min_y), max_pt=GPoint(max_x, max_y))
        self.stack.push_and_do(AddEntityCommand(self.doc, ent))
        self._points.clear()
      elif self.active_tool == "polyline" and len(self._points) >= 2:
        # Single-segment incremental: create on each click after the first
        a = self._points[-2]
        b = self._points[-1]
        ent = LineEntity(id=self.doc.create_id(), start=GPoint(a.x(), a.y()), end=GPoint(b.x(), b.y()))
        self.stack.push_and_do(AddEntityCommand(self.doc, ent))
      self.update()

  class MainWindow(QMainWindow):  # type: ignore[misc]
    def __init__(self):
      super().__init__()
      self.setWindowTitle("My Own CAD (2D)")
      self.doc = Document()
      self.stack = CommandStack()
      self.canvas = Canvas(self.doc, self.stack)
      self.canvas.status_callback = self._update_status
      self.setCentralWidget(self.canvas)

      tb = QToolBar("Tools")
      self.addToolBar(tb)

      def add_action(text, shortcut, handler, checkable=False):
        act = QAction(text, self)
        if shortcut:
          act.setShortcut(shortcut)
        act.triggered.connect(handler)
        act.setCheckable(checkable)
        tb.addAction(act)
        return act

      # Tools
      self.act_select = add_action("Select", "S", lambda: self._set_tool("select"), True)
      self.act_line = add_action("Line", "L", lambda: self._set_tool("line"), True)
      self.act_circle = add_action("Circle", "C", lambda: self._set_tool("circle"), True)
      self.act_rect = add_action("Rect", "R", lambda: self._set_tool("rect"), True)
      self.act_poly = add_action("Polyline", "P", lambda: self._set_tool("polyline"), True)
      self._set_tool("select")

      tb.addSeparator()
      # Edit
      add_action("Undo", "Ctrl+Z", self._undo)
      add_action("Redo", "Ctrl+Y", self._redo)
      add_action("Delete", "Del", self._delete_selected)

      tb.addSeparator()
      # File
      add_action("Open", "Ctrl+O", self._open)
      add_action("Save", "Ctrl+S", self._save)

      tb.addSeparator()
      # Snapping and grid
      self.act_snap_end = add_action("Snap End", None, self._toggle_snap_end, True)
      self.act_snap_mid = add_action("Snap Mid", None, self._toggle_snap_mid, True)
      self.act_snap_ctr = add_action("Snap Center", None, self._toggle_snap_ctr, True)
      self.act_snap_grid = add_action("Grid Snap", None, self._toggle_snap_grid, True)
      self.act_show_grid = add_action("Show Grid", None, self._toggle_show_grid, True)
      # defaults
      self.act_snap_end.setChecked(True)
      self.act_snap_mid.setChecked(True)
      self.act_snap_ctr.setChecked(True)
      self.act_snap_grid.setChecked(True)
      self.act_show_grid.setChecked(True)

      self.status = QStatusBar()
      self.setStatusBar(self.status)

      # demo content
      from .geometry import Point as GPoint
      self.doc.add_entity(LineEntity(id=self.doc.create_id(), start=GPoint(50, 50), end=GPoint(300, 50), color="#cc0000"))
      self.doc.add_entity(LineEntity(id=self.doc.create_id(), start=GPoint(50, 100), end=GPoint(300, 300), color="#00aa00"))

    def _set_tool(self, name: str) -> None:
      self.canvas.active_tool = name
      for act in (self.act_select, self.act_line, self.act_circle, self.act_rect, self.act_poly):
        act.setChecked(False)
      mapping = {
        "select": self.act_select,
        "line": self.act_line,
        "circle": self.act_circle,
        "rect": self.act_rect,
        "polyline": self.act_poly,
      }
      mapping[name].setChecked(True)

    def _undo(self):
      self.stack.undo()
      self.canvas.update()

    def _redo(self):
      self.stack.redo()
      self.canvas.update()

    def _delete_selected(self):
      if self.canvas._selected_id is not None:
        try:
          self.stack.push_and_do(RemoveEntityCommand(self.doc, self.canvas._selected_id))
          self.canvas._selected_id = None
          self.canvas.update()
        except Exception:
          pass

    def _open(self):
      path, _ = QFileDialog.getOpenFileName(self, "Open", "", "CAD JSON (*.json)")
      if not path:
        return
      try:
        self.doc.clear()
        loaded = load_document_from_json_file(path)
        # Move into our doc
        for ent in loaded.list_entities():
          self.doc.add_entity(ent)
        self.doc.layers = loaded.layers
        self.doc.next_id = loaded.next_id
        self.canvas.update()
      except Exception as e:
        QMessageBox.critical(self, "Open Failed", str(e))

    def _save(self):
      path, _ = QFileDialog.getSaveFileName(self, "Save", "", "CAD JSON (*.json)")
      if not path:
        return
      try:
        save_document_to_json_file(self.doc, path)
      except Exception as e:
        QMessageBox.critical(self, "Save Failed", str(e))

    def _toggle_snap_end(self):
      self.canvas.snap_endpoint = not self.canvas.snap_endpoint

    def _toggle_snap_mid(self):
      self.canvas.snap_midpoint = not self.canvas.snap_midpoint

    def _toggle_snap_ctr(self):
      self.canvas.snap_center = not self.canvas.snap_center

    def _toggle_snap_grid(self):
      self.canvas.snap_grid = not self.canvas.snap_grid

    def _toggle_show_grid(self):
      self.canvas.show_grid = not self.canvas.show_grid
      self.canvas.update()

    def _update_status(self, tool: str, pos: QPointF, snapped) -> None:
      if snapped is not None:
        self.status.showMessage(f"Tool: {tool} | Pos: ({pos.x():.1f}, {pos.y():.1f}) | Snap: ({snapped.x:.1f}, {snapped.y:.1f})")
      else:
        self.status.showMessage(f"Tool: {tool} | Pos: ({pos.x():.1f}, {pos.y():.1f})")

  app = QApplication([])
  win = MainWindow()
  win.show()
  return app.exec()


if __name__ == "__main__":  # pragma: no cover
  raise SystemExit(main())
