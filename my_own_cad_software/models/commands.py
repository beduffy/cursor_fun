from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from .document import Document
from .entities import BaseEntity


class Command:
  def do(self) -> None:  # pragma: no cover - interface
    raise NotImplementedError

  def undo(self) -> None:  # pragma: no cover - interface
    raise NotImplementedError


@dataclass
class AddEntityCommand(Command):
  document: Document
  entity: BaseEntity
  _entity_id: Optional[int] = None

  def do(self) -> None:
    self._entity_id = self.document.add_entity(self.entity)

  def undo(self) -> None:
    assert self._entity_id is not None
    self.document.remove_entity(self._entity_id)


@dataclass
class RemoveEntityCommand(Command):
  document: Document
  entity_id: int
  _removed: Optional[BaseEntity] = None

  def do(self) -> None:
    self._removed = self.document.remove_entity(self.entity_id)

  def undo(self) -> None:
    assert self._removed is not None
    self.document.add_entity(self._removed)


@dataclass
class MoveEntityCommand(Command):
  document: Document
  entity_id: int
  dx: float
  dy: float

  def do(self) -> None:
    self.document.move_entity(self.entity_id, self.dx, self.dy)

  def undo(self) -> None:
    self.document.move_entity(self.entity_id, -self.dx, -self.dy)


@dataclass
class SetLayerVisibilityCommand(Command):
  document: Document
  layer: str
  visible: bool
  _prev: Optional[bool] = None

  def do(self) -> None:
    self._prev = self.document.is_layer_visible(self.layer)
    self.document.set_layer_visibility(self.layer, self.visible)

  def undo(self) -> None:
    assert self._prev is not None
    self.document.set_layer_visibility(self.layer, self._prev)
