from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Any, Tuple

from ..geometry import Point, distance_between_points, is_point_near_segment, bounding_box_for_segment, bounding_box_for_circle


@dataclass
class BaseEntity:
  id: int
  layer: str = "0"
  color: str = "#000000"

  def bbox(self) -> Tuple[Point, Point]:  # pragma: no cover - abstract
    raise NotImplementedError

  def hit_test(self, p: Point, tolerance: float = 3.0) -> bool:  # pragma: no cover - abstract
    raise NotImplementedError

  def move(self, dx: float, dy: float) -> None:  # pragma: no cover - abstract
    raise NotImplementedError

  def to_dict(self) -> Dict[str, Any]:  # pragma: no cover - default override in subclasses
    return {"type": self.__class__.__name__, "id": self.id, "layer": self.layer, "color": self.color}


@dataclass
class LineEntity(BaseEntity):
  start: Point = field(default_factory=lambda: Point(0.0, 0.0))
  end: Point = field(default_factory=lambda: Point(0.0, 0.0))

  def bbox(self) -> Tuple[Point, Point]:
    return bounding_box_for_segment(self.start, self.end)

  def hit_test(self, p: Point, tolerance: float = 3.0) -> bool:
    return is_point_near_segment(p, self.start, self.end, tolerance)

  def move(self, dx: float, dy: float) -> None:
    self.start = Point(self.start.x + dx, self.start.y + dy)
    self.end = Point(self.end.x + dx, self.end.y + dy)

  def to_dict(self) -> Dict[str, Any]:
    base = super().to_dict()
    base.update({
      "start": {"x": self.start.x, "y": self.start.y},
      "end": {"x": self.end.x, "y": self.end.y},
    })
    return base


@dataclass
class CircleEntity(BaseEntity):
  center: Point = field(default_factory=lambda: Point(0.0, 0.0))
  radius: float = 1.0

  def bbox(self) -> Tuple[Point, Point]:
    return bounding_box_for_circle(self.center, self.radius)

  def hit_test(self, p: Point, tolerance: float = 3.0) -> bool:
    return abs(distance_between_points(p, self.center) - self.radius) <= tolerance

  def move(self, dx: float, dy: float) -> None:
    self.center = Point(self.center.x + dx, self.center.y + dy)

  def to_dict(self) -> Dict[str, Any]:
    base = super().to_dict()
    base.update({
      "center": {"x": self.center.x, "y": self.center.y},
      "radius": self.radius,
    })
    return base


@dataclass
class RectEntity(BaseEntity):
  min_pt: Point = field(default_factory=lambda: Point(0.0, 0.0))
  max_pt: Point = field(default_factory=lambda: Point(0.0, 0.0))

  def bbox(self) -> Tuple[Point, Point]:
    # Normalize just in case
    min_x = min(self.min_pt.x, self.max_pt.x)
    min_y = min(self.min_pt.y, self.max_pt.y)
    max_x = max(self.min_pt.x, self.max_pt.x)
    max_y = max(self.min_pt.y, self.max_pt.y)
    return Point(min_x, min_y), Point(max_x, max_y)

  def hit_test(self, p: Point, tolerance: float = 3.0) -> bool:
    # Hit test rectangle border: near any of the 4 edges
    a = Point(self.min_pt.x, self.min_pt.y)
    b = Point(self.max_pt.x, self.min_pt.y)
    c = Point(self.max_pt.x, self.max_pt.y)
    d = Point(self.min_pt.x, self.max_pt.y)
    return (
      is_point_near_segment(p, a, b, tolerance) or
      is_point_near_segment(p, b, c, tolerance) or
      is_point_near_segment(p, c, d, tolerance) or
      is_point_near_segment(p, d, a, tolerance)
    )

  def move(self, dx: float, dy: float) -> None:
    self.min_pt = Point(self.min_pt.x + dx, self.min_pt.y + dy)
    self.max_pt = Point(self.max_pt.x + dx, self.max_pt.y + dy)

  def to_dict(self) -> Dict[str, Any]:
    base = super().to_dict()
    base.update({
      "min_pt": {"x": self.min_pt.x, "y": self.min_pt.y},
      "max_pt": {"x": self.max_pt.x, "y": self.max_pt.y},
    })
    return base


def entity_from_dict(data: Dict[str, Any]) -> BaseEntity:
  etype = data.get("type")
  if etype == "LineEntity":
    return LineEntity(
      id=int(data["id"]),
      layer=data.get("layer", "0"),
      color=data.get("color", "#000000"),
      start=Point(float(data["start"]["x"]), float(data["start"]["y"])),
      end=Point(float(data["end"]["x"]), float(data["end"]["y"]))
    )
  if etype == "CircleEntity":
    return CircleEntity(
      id=int(data["id"]),
      layer=data.get("layer", "0"),
      color=data.get("color", "#000000"),
      center=Point(float(data["center"]["x"]), float(data["center"]["y"])),
      radius=float(data["radius"]) 
    )
  if etype == "RectEntity":
    return RectEntity(
      id=int(data["id"]),
      layer=data.get("layer", "0"),
      color=data.get("color", "#000000"),
      min_pt=Point(float(data["min_pt"]["x"]), float(data["min_pt"]["y"])),
      max_pt=Point(float(data["max_pt"]["x"]), float(data["max_pt"]["y"]))
    )
  raise ValueError(f"Unknown entity type: {etype}")
