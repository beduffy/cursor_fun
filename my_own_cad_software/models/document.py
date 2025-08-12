from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Optional, Iterable, Iterator

from .entities import BaseEntity


@dataclass
class Document:
  next_id: int = 1
  entities: Dict[int, BaseEntity] = field(default_factory=dict)
  layers: Dict[str, bool] = field(default_factory=lambda: {"0": True})

  def add_entity(self, entity: BaseEntity) -> int:
    if entity.id in self.entities:
      raise ValueError(f"Entity id already exists: {entity.id}")
    if entity.layer not in self.layers:
      self.layers[entity.layer] = True
    self.entities[entity.id] = entity
    self.next_id = max(self.next_id, entity.id + 1)
    return entity.id

  def create_id(self) -> int:
    fresh_id = self.next_id
    self.next_id += 1
    return fresh_id

  def get_entity(self, entity_id: int) -> Optional[BaseEntity]:
    return self.entities.get(entity_id)

  def remove_entity(self, entity_id: int) -> BaseEntity:
    if entity_id not in self.entities:
      raise KeyError(entity_id)
    return self.entities.pop(entity_id)

  def list_entities(self) -> Iterable[BaseEntity]:
    return list(self.entities.values())

  def iter_entities(self, visible_only: bool = False) -> Iterator[BaseEntity]:
    for ent in self.entities.values():
      if not visible_only or self.is_layer_visible(ent.layer):
        yield ent

  def list_visible_entities(self) -> Iterable[BaseEntity]:
    return list(self.iter_entities(visible_only=True))

  def is_layer_visible(self, layer: str) -> bool:
    return self.layers.get(layer, True)

  def set_layer_visibility(self, layer: str, visible: bool) -> None:
    self.layers[layer] = bool(visible)

  def move_entity(self, entity_id: int, dx: float, dy: float) -> None:
    ent = self.get_entity(entity_id)
    if ent is None:
      raise KeyError(entity_id)
    ent.move(dx, dy)

  def clear(self) -> None:
    self.entities.clear()
    self.next_id = 1
    self.layers = {"0": True}
