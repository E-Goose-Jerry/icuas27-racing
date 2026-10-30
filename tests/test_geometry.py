"""Geometry: gate frames and segment/plane crossings."""

from __future__ import annotations

import math
import unittest

from racing_core.geometry import (
    Vec3,
    closest_point_on_segment,
    crossing,
    gate_frame,
    polyline_length,
)


class Vec3Test(unittest.TestCase):
    def test_arithmetic(self) -> None:
        a = Vec3(1.0, 2.0, 3.0)
        b = Vec3(-1.0, 0.5, 2.0)
        self.assertEqual(a + b, Vec3(0.0, 2.5, 5.0))
        self.assertEqual(a - b, Vec3(2.0, 1.5, 1.0))
        self.assertEqual(a * 2.0, Vec3(2.0, 4.0, 6.0))
        self.assertEqual(2.0 * a, Vec3(2.0, 4.0, 6.0))
        self.assertEqual(-a, Vec3(-1.0, -2.0, -3.0))

    def test_products(self) -> None:
        a = Vec3(1.0, 0.0, 0.0)
        b = Vec3(0.0, 1.0, 0.0)
        self.assertEqual(a.dot(b), 0.0)
        self.assertEqual(a.cross(b), Vec3(0.0, 0.0, 1.0))
        self.assertAlmostEqual(Vec3(3.0, 4.0, 0.0).norm(), 5.0)
        self.assertEqual(Vec3(3.0, 4.0, 9.0).norm_xy(), 5.0)

    def test_from_any(self) -> None:
        self.assertEqual(Vec3.from_any([1, 2]), Vec3(1.0, 2.0, 0.0))
        self.assertEqual(Vec3.from_any([1, 2, 3]), Vec3(1.0, 2.0, 3.0))
        self.assertEqual(Vec3.from_any(Vec3(4, 5, 6)), Vec3(4.0, 5.0, 6.0))
        with self.assertRaises(ValueError):
            Vec3.from_any([1.0])

    def test_normalize_rejects_zero(self) -> None:
        with self.assertRaises(ValueError):
            Vec3(0.0, 0.0, 0.0).normalized()

    def test_finite(self) -> None:
        self.assertTrue(Vec3(1.0, 2.0, 3.0).is_finite())
        self.assertFalse(Vec3(float("nan"), 0.0, 0.0).is_finite())
        self.assertFalse(Vec3(0.0, float("inf"), 0.0).is_finite())

    def test_iteration_and_lists(self) -> None:
        self.assertEqual(list(Vec3(1.0, 2.0, 3.0)), [1.0, 2.0, 3.0])
        self.assertEqual(Vec3(1.0, 2.0, 3.0).to_list(), [1.0, 2.0, 3.0])


class GateFrameTest(unittest.TestCase):
    def test_axes_follow_yaw(self) -> None:
        frame = gate_frame(Vec3(5.0, 5.0, 2.0), 0.0)
        self.assertAlmostEqual(frame.normal.x, 1.0)
        self.assertAlmostEqual(frame.normal.y, 0.0)
        self.assertAlmostEqual(frame.lateral.x, 0.0)
        self.assertAlmostEqual(frame.lateral.y, 1.0)

        rotated = gate_frame(Vec3(0.0, 0.0, 0.0), math.pi / 2.0)
        self.assertAlmostEqual(rotated.normal.x, 0.0)
        self.assertAlmostEqual(rotated.normal.y, 1.0)
        self.assertAlmostEqual(rotated.lateral.x, -1.0)
        self.assertAlmostEqual(rotated.lateral.y, 0.0)

    def test_to_local(self) -> None:
        frame = gate_frame(Vec3(0.0, 0.0, 1.0), 0.0)
        along, lateral, vertical = frame.to_local(Vec3(2.0, 0.3, 1.4))
        self.assertAlmostEqual(along, 2.0)
        self.assertAlmostEqual(lateral, 0.3)
        self.assertAlmostEqual(vertical, 0.4)

    def test_contains_respects_opening_and_margin(self) -> None:
        frame = gate_frame(Vec3(), 0.0, width=1.0, height=1.0)
        self.assertTrue(frame.contains(0.0, 0.0))
        self.assertTrue(frame.contains(0.49, -0.49))
        self.assertFalse(frame.contains(0.51, 0.0))
        self.assertTrue(frame.contains(0.51, 0.0, margin=0.05))


class CrossingTest(unittest.TestCase):
    def setUp(self) -> None:
        self.frame = gate_frame(Vec3(0.0, 0.0, 1.0), 0.0)  # normal +x

    def test_clean_forward_pass(self) -> None:
        result = crossing(self.frame, Vec3(-1.0, 0.0, 1.0), Vec3(1.0, 0.0, 1.0))
        self.assertTrue(result.crossed)
        self.assertTrue(result.inside)
        self.assertEqual(result.direction, 1)
        self.assertTrue(result.valid_pass)
        self.assertAlmostEqual(result.fraction, 0.5)
        self.assertAlmostEqual(result.point.x, 0.0)

    def test_backwards_crossing_is_direction_minus_one(self) -> None:
        result = crossing(self.frame, Vec3(1.0, 0.0, 1.0), Vec3(-1.0, 0.0, 1.0))
        self.assertTrue(result.crossed)
        self.assertEqual(result.direction, -1)
        self.assertFalse(result.valid_pass)

    def test_same_side_does_not_cross(self) -> None:
        result = crossing(self.frame, Vec3(-2.0, 0.0, 1.0), Vec3(-0.5, 0.0, 1.0))
        self.assertFalse(result.crossed)

    def test_miss_outside_opening(self) -> None:
        result = crossing(self.frame, Vec3(-1.0, 0.9, 1.0), Vec3(1.0, 0.9, 1.0))
        self.assertTrue(result.crossed)
        self.assertFalse(result.inside)
        self.assertAlmostEqual(result.lateral, 0.9, places=6)
        self.assertFalse(result.valid_pass)

    def test_miss_above_opening(self) -> None:
        result = crossing(self.frame, Vec3(-1.0, 0.0, 1.8), Vec3(1.0, 0.0, 1.8))
        self.assertTrue(result.crossed)
        self.assertFalse(result.inside)
        self.assertAlmostEqual(result.vertical, 0.8, places=6)

    def test_exactly_on_plane_is_not_a_crossing(self) -> None:
        result = crossing(self.frame, Vec3(0.0, 0.0, 1.0), Vec3(1.0, 0.0, 1.0))
        self.assertFalse(result.crossed)

    def test_crossing_rotated_gate(self) -> None:
        frame = gate_frame(Vec3(3.0, -2.0, 1.5), math.pi / 2.0)  # normal +y
        result = crossing(frame, Vec3(3.0, -3.0, 1.5), Vec3(3.0, -1.0, 1.5))
        self.assertTrue(result.valid_pass)

    def test_diagonal_segment_hits_gate_centre(self) -> None:
        result = crossing(self.frame, Vec3(-1.0, -1.0, 1.0), Vec3(1.0, 1.0, 1.0))
        self.assertTrue(result.crossed)
        self.assertAlmostEqual(result.lateral, 0.0, places=6)
        self.assertTrue(result.inside)

    def test_start_on_plane_uses_the_remembered_side(self) -> None:
        # Our waypoint IS the gate centre, so a sample landing exactly on the
        # plane is the normal case. With start_side=-1 the pass must be counted
        # forward, not dropped or read as backwards.
        result = crossing(self.frame, Vec3(0.0, 0.0, 1.0), Vec3(1.0, 0.0, 1.0), start_side=-1)
        self.assertTrue(result.crossed)
        self.assertEqual(result.direction, 1)
        self.assertTrue(result.valid_pass)
        self.assertAlmostEqual(result.fraction, 0.0)

    def test_start_on_plane_without_history_is_no_crossing(self) -> None:
        # No side history: we cannot tell which way the vehicle came from.
        self.assertFalse(
            crossing(self.frame, Vec3(0.0, 0.0, 1.0), Vec3(1.0, 0.0, 1.0)).crossed
        )

    def test_distance_from_centre(self) -> None:
        result = crossing(self.frame, Vec3(-1.0, 0.3, 1.4), Vec3(1.0, 0.3, 1.4))
        self.assertAlmostEqual(result.distance_from_centre, 0.5, places=6)

    def test_zero_length_segment(self) -> None:
        point = Vec3(-1.0, 0.0, 1.0)
        self.assertFalse(crossing(self.frame, point, point).crossed)

    def test_result_serialises(self) -> None:
        payload = crossing(self.frame, Vec3(-1.0, 0.0, 1.0), Vec3(1.0, 0.0, 1.0)).to_dict()
        self.assertTrue(payload["valid_pass"])
        self.assertEqual(payload["point"]["x"], 0.0)


class SegmentTest(unittest.TestCase):
    def test_closest_point_midway(self) -> None:
        point, t = closest_point_on_segment(Vec3(0.0, 1.0, 0.0), Vec3(-1.0, 0.0, 0.0), Vec3(1.0, 0.0, 0.0))
        self.assertAlmostEqual(t, 0.5)
        self.assertEqual(point, Vec3(0.0, 0.0, 0.0))

    def test_closest_point_clamps(self) -> None:
        _, t = closest_point_on_segment(Vec3(5.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0), Vec3(1.0, 0.0, 0.0))
        self.assertAlmostEqual(t, 1.0)

    def test_closest_point_degenerate(self) -> None:
        point, t = closest_point_on_segment(Vec3(1.0, 1.0, 1.0), Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0))
        self.assertEqual(point, Vec3())
        self.assertEqual(t, 0.0)

    def test_polyline_length(self) -> None:
        self.assertAlmostEqual(
            polyline_length([Vec3(0, 0, 0), Vec3(3, 4, 0), Vec3(3, 4, 12)]), 17.0
        )


if __name__ == "__main__":
    unittest.main()
