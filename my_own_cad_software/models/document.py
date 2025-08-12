from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Iterable

from .entities import BaseEntity


@dataclass
class Document:
  next_id: int = 1
  entities: Dict[int, BaseEntity] = field(default_factory=dict)

  def add_entity(self, entity: BaseEntity) -> int:
    if entity.id in self.entities:
      raise ValueError(f"Entity id already exists: {entity.id}")
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

  def clear(self) -> None:
    self.entities.clear()
    self.next_id = 1
