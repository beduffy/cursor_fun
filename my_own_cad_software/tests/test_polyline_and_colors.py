from my_own_cad_software.models.document import Document
from my_own_cad_software.models.entities import PolylineEntity
from my_own_cad_software.geometry import Point
from my_own_cad_software.io.svg_export import document_to_svg


def test_polyline_bbox_and_move_and_hit():
  doc = Document()
  pl = PolylineEntity(id=doc.create_id(), points=[Point(0,0), Point(10,0), Point(10,10)], color="#123456")
  doc.add_entity(pl)

  bb = pl.bbox()
  assert bb[0] == Point(0, 0) and bb[1] == Point(10, 10)

  pl.move(5, -5)
  bb2 = pl.bbox()
  assert bb2[0] == Point(5, -5) and bb2[1] == Point(15, 5)

  assert pl.hit_test(Point(10, -3), tolerance=3.1)


def test_svg_color_output_for_polyline():
  doc = Document()
  pl = PolylineEntity(id=doc.create_id(), points=[Point(0,0), Point(10,0), Point(10,10)], color="#ff00ff")
  doc.add_entity(pl)
  svg = document_to_svg(doc, width=100, height=100, visible_only=True)
  assert "polyline" in svg and "#ff00ff" in svg
