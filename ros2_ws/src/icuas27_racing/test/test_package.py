"""Tests for course discovery and the detector maths (no ROS runtime needed).

Run from the ROS package directory::

    python3 -m pytest test -q
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

PACKAGE_DIR = Path(__file__).resolve().parent.parent      # .../src/icuas27_racing
PROJECT_ROOT = PACKAGE_DIR.parents[2]                     # .../racing
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PACKAGE_DIR))

from icuas27_racing.course_files import (  # noqa: E402
    COURSE_ENV,
    available_courses,
    load_default_course,
    resolve_course,
)
from icuas27_racing.detectors.common import (  # noqa: E402
    Intrinsics,
    PixelCandidate,
    bearing,
    centroid,
    order_quad,
    quad_area,
    select_best,
)
from racing_core.course import CourseError  # noqa: E402


class TestIntrinsics:
    def test_fov(self) -> None:
        # 100 px wide image at fx=100: half-angle atan(0.5) = 0.4636 rad, so the
        # full field of view is 0.9273 rad (~53.1 deg), not 106 deg.
        intrinsics = Intrinsics(fx=100.0, fy=100.0, cx=50.0, cy=50.0, width=100, height=100)
        assert abs(intrinsics.horizontal_fov() - 0.9272952) < 1e-6
        assert abs(intrinsics.horizontal_fov() - 2.0 * 0.4636476) < 1e-6
        assert intrinsics.vertical_fov() == intrinsics.horizontal_fov()

    def test_rejects_bad_focal_length(self) -> None:
        try:
            Intrinsics(fx=0.0, fy=1.0, cx=0.0, cy=0.0, width=10, height=10)
        except ValueError:
            return
        raise AssertionError("expected ValueError")

    def test_scaled_keeps_the_lens(self) -> None:
        intrinsics = Intrinsics(fx=200.0, fy=200.0, cx=100.0, cy=50.0, width=200, height=100)
        half = intrinsics.scaled(100, 50)
        assert half.fx == 100.0
        assert half.cx == 50.0
        # the same ray maps to the same angle before and after downsampling
        yaw_full, _ = bearing(intrinsics, 150.0, 50.0)
        yaw_half, _ = bearing(half, 75.0, 25.0)
        assert abs(yaw_full - yaw_half) < 1e-9


class TestBearing:
    def test_centre_pixel_is_zero_angle(self) -> None:
        intrinsics = Intrinsics(fx=100.0, fy=100.0, cx=50.0, cy=40.0, width=100, height=80)
        yaw, pitch = bearing(intrinsics, 50.0, 40.0)
        assert yaw == 0.0 and pitch == 0.0

    def test_right_and_down_are_positive(self) -> None:
        intrinsics = Intrinsics(fx=100.0, fy=100.0, cx=50.0, cy=40.0, width=100, height=80)
        yaw, pitch = bearing(intrinsics, 150.0, 140.0)
        assert yaw > 0.0 and pitch > 0.0

    def test_45_degrees(self) -> None:
        intrinsics = Intrinsics(fx=100.0, fy=100.0, cx=0.0, cy=0.0, width=200, height=200)
        yaw, _ = bearing(intrinsics, 100.0, 0.0)
        assert abs(yaw - 0.7853981) < 1e-6


class TestQuad:
    def test_area_of_square(self) -> None:
        assert quad_area([(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]) == 100.0

    def test_area_is_winding_independent(self) -> None:
        clockwise = [(0.0, 0.0), (0.0, 10.0), (10.0, 10.0), (10.0, 0.0)]
        assert quad_area(clockwise) == 100.0

    def test_area_needs_three_points(self) -> None:
        assert quad_area([(0.0, 0.0), (1.0, 1.0)]) == 0.0

    def test_order_quad(self) -> None:
        points = [(10.0, 10.0), (0.0, 0.0), (10.0, 0.0), (0.0, 10.0)]
        assert order_quad(points) == [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]

    def test_order_quad_passthrough(self) -> None:
        points = [(0.0, 0.0), (1.0, 1.0)]
        assert order_quad(points) == points

    def test_centroid(self) -> None:
        u, v = centroid([(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)])
        assert (u, v) == (5.0, 5.0)


class TestSelectBest:
    def test_empty(self) -> None:
        assert select_best([]) is None

    def test_biggest_wins(self) -> None:
        small = PixelCandidate(centre_u=10.0, centre_v=10.0, area_px=100.0)
        big = PixelCandidate(centre_u=90.0, centre_v=10.0, area_px=900.0)
        assert select_best([small, big]) is big

    def test_confidence_breaks_ties(self) -> None:
        weak = PixelCandidate(centre_u=0.0, centre_v=0.0, area_px=100.0, confidence=0.2)
        strong = PixelCandidate(centre_u=0.0, centre_v=0.0, area_px=100.0, confidence=0.9)
        assert select_best([weak, strong]) is strong

    def test_preferred_centre_pulls_the_choice(self) -> None:
        left = PixelCandidate(centre_u=100.0, centre_v=0.0, area_px=1000.0)
        middle = PixelCandidate(centre_u=160.0, centre_v=0.0, area_px=800.0)
        assert select_best([left, middle], preferred_u=160.0) is middle


class TestCourseFiles:
    def test_resolves_the_shipped_course(self) -> None:
        path = resolve_course()
        assert path.is_file()
        assert path.name == "provisional_sample.json"

    def test_loads_with_gates(self) -> None:
        course = load_default_course()
        assert course.gate_count == 7
        assert course.laps == 3

    def test_explicit_path_wins(self) -> None:
        path = resolve_course(str(PROJECT_ROOT / "course" / "arena_template.json"))
        assert path.name == "arena_template.json"

    def test_env_var_is_used(self) -> None:
        old = os.environ.get(COURSE_ENV)
        os.environ[COURSE_ENV] = str(PROJECT_ROOT / "course" / "arena_template.json")
        try:
            assert resolve_course().name == "arena_template.json"
        finally:
            if old is None:
                os.environ.pop(COURSE_ENV, None)
            else:
                os.environ[COURSE_ENV] = old

    def test_missing_explicit_path_raises(self) -> None:
        try:
            resolve_course("/definitely/not/here.json")
        except CourseError as exc:
            assert "does not exist" in str(exc)
            return
        raise AssertionError("expected CourseError")

    def test_available_courses_lists_both(self) -> None:
        names = {path.name for path in available_courses()}
        assert {"provisional_sample.json", "arena_template.json"} <= names
