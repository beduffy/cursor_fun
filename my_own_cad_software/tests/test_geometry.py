from my_own_cad_software.geometry import Point, distance_between_points, is_point_near_segment, bounding_box_for_segment, snap_point_to_grid


def test_distance():
  a = Point(0, 0)
  b = Point(3, 4)
  assert distance_between_points(a, b) == 5


def test_segment_hit():
  a = Point(0, 0)
  b = Point(10, 0)
  p = Point(5, 2.5)
  assert is_point_near_segment(p, a, b, tolerance=3.0) is True


def test_bounding_box():
  a = Point(3, -2)
  b = Point(-1, 5)
  bb = bounding_box_for_segment(a, b)
  assert bb[0] == Point(-1, -2)
  assert bb[1] == Point(3, 5)


def test_snap_to_grid():
  p = Point(3.2, 4.7)
  g = snap_point_to_grid(p, 1.0)
  assert g == Point(3, 5)
