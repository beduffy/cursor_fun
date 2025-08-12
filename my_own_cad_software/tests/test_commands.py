from my_own_cad_software.models.command_stack import CommandStack
from my_own_cad_software.models.document import Document
from my_own_cad_software.models.entities import LineEntity
from my_own_cad_software.geometry import Point


class AddEntityCommand:
  def __init__(self, doc: Document, entity: LineEntity):
    self.doc = doc
    self.entity = entity
    self.entity_id = None

  def do(self) -> None:
    self.entity_id = self.doc.add_entity(self.entity)

  def undo(self) -> None:
    assert self.entity_id is not None
    self.doc.remove_entity(self.entity_id)


def test_command_stack_add_undo_redo():
  doc = Document()
  stack = CommandStack()
  line = LineEntity(id=doc.create_id(), start=Point(0, 0), end=Point(5, 0))

  cmd = AddEntityCommand(doc, line)
  stack.push_and_do(cmd)
  assert len(list(doc.list_entities())) == 1

  stack.undo()
  assert len(list(doc.list_entities())) == 0

  stack.redo()
  assert len(list(doc.list_entities())) == 1
