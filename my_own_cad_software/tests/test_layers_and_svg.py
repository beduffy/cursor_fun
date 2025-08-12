from my_own_cad_software.models.document import Document
from my_own_cad_software.models.entities import LineEntity, CircleEntity, RectEntity
from my_own_cad_software.geometry import Point
from my_own_cad_software.io.serializer import document_to_dict, document_from_dict
from my_own_cad_software.io.svg_export import document_to_svg


def test_layers_visibility_and_iter():
  doc = Document()
  line = LineEntity(id=doc.create_id(), start=Point(0,0), end=Point(10,0), layer="A")
  circ = CircleEntity(id=doc.create_id(), center=Point(5,5), radius=2, layer="B")
  rect = RectEntity(id=doc.create_id(), min_pt=Point(1,1), max_pt=Point(4,4), layer="A")
  doc.add_entity(line)
  doc.add_entity(circ)
  doc.add_entity(rect)

  # default: layers are visible; should see all 3
  assert len(list(doc.iter_entities(visible_only=True))) == 3

  # hide layer B; circle should disappear
  doc.set_layer_visibility("B", False)
  visible = list(doc.iter_entities(visible_only=True))
  assert len(visible) == 2
  assert all(e.layer != "B" for e in visible)


def test_serializer_includes_layers():
  doc = Document()
  line = LineEntity(id=doc.create_id(), start=Point(0,0), end=Point(10,0), layer="A")
  doc.add_entity(line)
  doc.set_layer_visibility("A", False)

  data = document_to_dict(doc)
  assert data["layers"].get("A") is False

  doc2 = document_from_dict(data)
  assert doc2.is_layer_visible("A") is False
  assert len(list(doc2.list_entities())) == 1


def test_svg_export_contains_shapes_and_respects_visibility():
  doc = Document()
  doc.add_entity(LineEntity(id=doc.create_id(), start=Point(0,0), end=Point(10,0), layer="A"))
  doc.add_entity(CircleEntity(id=doc.create_id(), center=Point(5,5), radius=2, layer="B"))
  doc.add_entity(RectEntity(id=doc.create_id(), min_pt=Point(1,1), max_pt=Point(4,4), layer="A"))

  svg_all = document_to_svg(doc, width=100, height=100, visible_only=True)
  assert "<line" in svg_all and "<circle" in svg_all and "<rect" in svg_all

  # hide B and re-export
  doc.set_layer_visibility("B", False)
  svg_vis = document_to_svg(doc, width=100, height=100, visible_only=True)
  assert "<line" in svg_vis and "<rect" in svg_vis and "<circle" not in svg_vis
