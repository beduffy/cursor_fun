from my_own_cad_software.models.document import Document
from my_own_cad_software.models.entities import LineEntity, CircleEntity, RectEntity, PolylineEntity
from my_own_cad_software.geometry import Point
from my_own_cad_software.selection import find_entity_at_point, snap_point, distance_to_entity_edge


def build_doc():
  doc = Document()
  l1 = LineEntity(id=doc.create_id(), start=Point(0,0), end=Point(10,0), layer="A")
  c1 = CircleEntity(id=doc.create_id(), center=Point(5,5), radius=3, layer="A")
  r1 = RectEntity(id=doc.create_id(), min_pt=Point(2,2), max_pt=Point(6,6), layer="B")
  p1 = PolylineEntity(id=doc.create_id(), points=[Point(0,10), Point(10,10)], layer="A")
  for e in (l1, c1, r1, p1):
    doc.add_entity(e)
  return doc


def test_find_entity_and_distance():
  doc = build_doc()
  # near l1
  ent = find_entity_at_point(doc, Point(5, 1.5), tolerance=2.0)
  assert ent is not None
  d = distance_to_entity_edge(Point(5,1.5), ent)
  assert d < 2.0


def test_snap_endpoint_midpoint_center_grid():
  doc = build_doc()
  # endpoint on polyline
  res = snap_point(Point(0.2, 10.1), doc, modes=("endpoint",), tolerance=1.0)
  assert res.kind == "endpoint"
  # midpoint on line
  res2 = snap_point(Point(5.2, 0.1), doc, modes=("midpoint",), tolerance=1.0)
  assert res2.kind == "midpoint"
  # center on circle
  res3 = snap_point(Point(5.3, 5.2), doc, modes=("center",), tolerance=0.5)
  assert res3.kind == "center"
  # grid fallback
  res4 = snap_point(Point(5.3, 5.2), doc, modes=(), tolerance=0.1, grid_size=1.0)
  assert res4.kind == "grid" and res4.point == Point(5,5)


def test_visibility_affects_selection_and_snap():
  doc = build_doc()
  doc.set_layer_visibility("A", False)
  # only rect on layer B remains visible
  ent = find_entity_at_point(doc, Point(4, 2.2), tolerance=1.0, visible_only=True)
  assert ent is not None and ent.layer == "B"
  # snap to A items should fail, so grid fallback kicks in
  res = snap_point(Point(0.2, 10.1), doc, modes=("endpoint",), tolerance=0.5, grid_size=1.0, visible_only=True)
  assert res.kind in ("grid", "none")
