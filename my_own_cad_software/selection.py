from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, List, Optional, Set, Tuple

from .geometry import Point, distance_between_points, is_point_near_segment, nearest_point_on_segment
from .models.document import Document
from .models.entities import BaseEntity, LineEntity, CircleEntity, RectEntity, PolylineEntity


@dataclass(frozen=True)
class SnapResult:
  point: Point
  kind: str  # e.g., 'endpoint', 'midpoint', 'center', 'grid'
  entity_id: Optional[int] = None


def _polyline_segment_points(points: List[Point]) -> Iterable[Tuple[Point, Point]]:
  for a, b in zip(points[:-1], points[1:]):
    yield a, b


def find_entity_at_point(doc: Document, p: Point, tolerance: float = 3.0, visible_only: bool = True) -> Optional[BaseEntity]:
  candidates: List[BaseEntity] = []
  for ent in doc.iter_entities(visible_only=visible_only):
    if ent.hit_test(p, tolerance=tolerance):
      candidates.append(ent)
  # No Z-ordering defined; pick lowest id for determinism
  if not candidates:
    return None
  return sorted(candidates, key=lambda e: e.id)[0]


def distance_to_entity_edge(p: Point, ent: BaseEntity) -> float:
  if isinstance(ent, LineEntity):
    closest = nearest_point_on_segment(p, ent.start, ent.end)
    return distance_between_points(p, closest)
  if isinstance(ent, CircleEntity):
    return abs(distance_between_points(p, ent.center) - ent.radius)
  if isinstance(ent, RectEntity):
    a = ent.min_pt
    b = Point(ent.max_pt.x, ent.min_pt.y)
    c = ent.max_pt
    d = Point(ent.min_pt.x, ent.max_pt.y)
    return min(
      distance_between_points(p, nearest_point_on_segment(p, a, b)),
      distance_between_points(p, nearest_point_on_segment(p, b, c)),
      distance_between_points(p, nearest_point_on_segment(p, c, d)),
      distance_between_points(p, nearest_point_on_segment(p, d, a)),
    )
  if isinstance(ent, PolylineEntity):
    best = float("inf")
    for a, b in _polyline_segment_points(ent.points):
      d = distance_between_points(p, nearest_point_on_segment(p, a, b))
      if d < best:
        best = d
    return best
  # fallback
  bb_min, bb_max = ent.bbox()
  # distance to bbox center as rough measure
  cx = (bb_min.x + bb_max.x) / 2.0
  cy = (bb_min.y + bb_max.y) / 2.0
  return distance_between_points(p, Point(cx, cy))


def _collect_snap_points(ent: BaseEntity, modes: Set[str]) -> List[Tuple[Point, str]]:
  pts: List[Tuple[Point, str]] = []
  if isinstance(ent, LineEntity):
    if 'endpoint' in modes:
      pts.append((ent.start, 'endpoint'))
      pts.append((ent.end, 'endpoint'))
    if 'midpoint' in modes:
      mid = Point((ent.start.x + ent.end.x) / 2.0, (ent.start.y + ent.end.y) / 2.0)
      pts.append((mid, 'midpoint'))
  elif isinstance(ent, CircleEntity):
    if 'center' in modes:
      pts.append((ent.center, 'center'))
  elif isinstance(ent, RectEntity):
    a = ent.min_pt
    b = Point(ent.max_pt.x, ent.min_pt.y)
    c = ent.max_pt
    d = Point(ent.min_pt.x, ent.max_pt.y)
    if 'endpoint' in modes:
      pts.extend([(a, 'endpoint'), (b, 'endpoint'), (c, 'endpoint'), (d, 'endpoint')])
    if 'midpoint' in modes:
      pts.extend([
        (Point((a.x + b.x)/2, (a.y + b.y)/2), 'midpoint'),
        (Point((b.x + c.x)/2, (b.y + c.y)/2), 'midpoint'),
        (Point((c.x + d.x)/2, (c.y + d.y)/2), 'midpoint'),
        (Point((d.x + a.x)/2, (d.y + a.y)/2), 'midpoint'),
      ])
  elif isinstance(ent, PolylineEntity):
    if 'endpoint' in modes and ent.points:
      pts.append((ent.points[0], 'endpoint'))
      pts.append((ent.points[-1], 'endpoint'))
    if 'midpoint' in modes:
      for a, b in _polyline_segment_points(ent.points):
        pts.append((Point((a.x + b.x)/2, (a.y + b.y)/2), 'midpoint'))
  return pts


def snap_point(p: Point, doc: Document, modes: Iterable[str] = ("endpoint", "midpoint", "center"), tolerance: float = 8.0, grid_size: float = 0.0, visible_only: bool = True) -> SnapResult:
  modes_set = set(modes)
  best_dist = float('inf')
  best: Optional[SnapResult] = None

  for ent in doc.iter_entities(visible_only=visible_only):
    for candidate, kind in _collect_snap_points(ent, modes_set):
      d = distance_between_points(p, candidate)
      if d <= tolerance and d < best_dist:
        best_dist = d
        best = SnapResult(point=candidate, kind=kind, entity_id=ent.id)

  if best is not None:
    return best

  if grid_size > 0:
    from .geometry import snap_point_to_grid
    gp = snap_point_to_grid(p, grid_size)
    return SnapResult(point=gp, kind='grid', entity_id=None)

  return SnapResult(point=p, kind='none', entity_id=None)
