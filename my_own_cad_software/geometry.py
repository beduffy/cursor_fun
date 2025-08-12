from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple
import math


@dataclass(frozen=True)
class Point:
  x: float
  y: float


def distance_between_points(a: Point, b: Point) -> float:
  dx = a.x - b.x
  dy = a.y - b.y
  return math.hypot(dx, dy)


def are_floats_close(a: float, b: float, tol: float = 1e-6) -> bool:
  return abs(a - b) <= tol


def is_point_near_segment(p: Point, a: Point, b: Point, tolerance: float = 3.0) -> bool:
  # Handle degenerate segment
  if are_floats_close(a.x, b.x) and are_floats_close(a.y, b.y):
    return distance_between_points(p, a) <= tolerance

  # Project point onto segment
  abx = b.x - a.x
  aby = b.y - a.y
  apx = p.x - a.x
  apy = p.y - a.y
  ab_len2 = abx * abx + aby * aby
  t = max(0.0, min(1.0, (apx * abx + apy * aby) / ab_len2))
  closest = Point(a.x + t * abx, a.y + t * aby)
  return distance_between_points(p, closest) <= tolerance


def bounding_box_for_segment(a: Point, b: Point) -> Tuple[Point, Point]:
  min_x = min(a.x, b.x)
  min_y = min(a.y, b.y)
  max_x = max(a.x, b.x)
  max_y = max(a.y, b.y)
  return Point(min_x, min_y), Point(max_x, max_y)


def bounding_box_for_circle(center: Point, radius: float) -> Tuple[Point, Point]:
  return Point(center.x - radius, center.y - radius), Point(center.x + radius, center.y + radius)


def is_point_in_rect(p: Point, min_pt: Point, max_pt: Point) -> bool:
  return (min_pt.x <= p.x <= max_pt.x) and (min_pt.y <= p.y <= max_pt.y)


def clamp_to_grid(value: float, grid_size: float) -> float:
  if grid_size <= 0:
    return value
  return round(value / grid_size) * grid_size


def snap_point_to_grid(p: Point, grid_size: float) -> Point:
  return Point(clamp_to_grid(p.x, grid_size), clamp_to_grid(p.y, grid_size))
