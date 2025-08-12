from __future__ import annotations

import json
from typing import Any, Dict

from ..models.document import Document
from ..models.entities import BaseEntity, entity_from_dict


def document_to_dict(doc: Document) -> Dict[str, Any]:
  return {
    "next_id": doc.next_id,
    "entities": [e.to_dict() for e in doc.list_entities()],
  }


def document_from_dict(data: Dict[str, Any]) -> Document:
  from ..models.document import Document  # local import to avoid cycles in type checkers
  doc = Document()
  doc.next_id = int(data.get("next_id", 1))
  for ent_data in data.get("entities", []):
    ent = entity_from_dict(ent_data)
    doc.add_entity(ent)
  return doc


def save_document_to_json_file(doc: Document, path: str) -> None:
  with open(path, "w", encoding="utf-8") as f:
    json.dump(document_to_dict(doc), f, indent=2)


def load_document_from_json_file(path: str) -> Document:
  with open(path, "r", encoding="utf-8") as f:
    data = json.load(f)
  return document_from_dict(data)
