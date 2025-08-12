from my_own_cad_software.models.document import Document
from my_own_cad_software.models.entities import LineEntity
from my_own_cad_software.geometry import Point


def test_document_add_get_remove():
  doc = Document()
  line = LineEntity(id=doc.create_id(), start=Point(0, 0), end=Point(10, 0))
  eid = doc.add_entity(line)
  assert doc.get_entity(eid) is line
  removed = doc.remove_entity(eid)
  assert removed is line
  assert doc.get_entity(eid) is None


def test_document_next_id_advances():
  doc = Document()
  e1 = LineEntity(id=doc.create_id(), start=Point(0, 0), end=Point(1, 0))
  doc.add_entity(e1)
  e2 = LineEntity(id=doc.create_id(), start=Point(0, 0), end=Point(2, 0))
  doc.add_entity(e2)
  assert e2.id == e1.id + 1
