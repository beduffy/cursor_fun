from __future__ import annotations

from typing import Iterable

from ..models.document import Document
from ..models.entities import LineEntity, CircleEntity, RectEntity, PolylineEntity


def document_to_svg(doc: Document, width: int = 800, height: int = 600, visible_only: bool = True) -> str:
  lines: list[str] = []
  lines.append(f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">')
  lines.append('<g stroke-width="2" fill="none">')

  for ent in (doc.iter_entities(visible_only=visible_only)):
    if isinstance(ent, LineEntity):
      lines.append(f'<line x1="{ent.start.x}" y1="{ent.start.y}" x2="{ent.end.x}" y2="{ent.end.y}" stroke="{ent.color}" />')
    elif isinstance(ent, CircleEntity):
      lines.append(f'<circle cx="{ent.center.x}" cy="{ent.center.y}" r="{ent.radius}" stroke="{ent.color}" />')
    elif isinstance(ent, RectEntity):
      min_pt, max_pt = ent.bbox()
      w = max_pt.x - min_pt.x
      h = max_pt.y - min_pt.y
      lines.append(f'<rect x="{min_pt.x}" y="{min_pt.y}" width="{w}" height="{h}" stroke="{ent.color}" />')
    elif isinstance(ent, PolylineEntity):
      if ent.points:
        pts = " ".join(f"{p.x},{p.y}" for p in ent.points)
        lines.append(f'<polyline points="{pts}" stroke="{ent.color}" />')

  lines.append('</g>')
  lines.append('</svg>')
  return "\n".join(lines)
