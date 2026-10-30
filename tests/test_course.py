"""Course model, JSON loading, and validation."""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from racing_core.course import Course, CourseError, Gate, build_course, load_course
from racing_core.geometry import Vec3

PROJECT_ROOT = Path(__file__).resolve().parent.parent
COURSE_DIR = PROJECT_ROOT / "course"


class GateTest(unittest.TestCase):
    def test_roundtrip(self) -> None:
        gate = Gate(id=3, position=Vec3(1.0, 2.0, 3.0), yaw=0.5, name="gate3")
        again = Gate.from_dict(gate.to_dict(), fallback_id=99)
        self.assertEqual(again.id, 3)
        self.assertEqual(again.position, Vec3(1.0, 2.0, 3.0))
        self.assertEqual(again.yaw, 0.5)
        self.assertEqual(again.name, "gate3")

    def test_requires_position(self) -> None:
        with self.assertRaises(CourseError):
            Gate.from_dict({"id": 1}, fallback_id=1)

    def test_rejects_non_finite(self) -> None:
        with self.assertRaises(CourseError):
            Gate.from_dict({"id": 1, "position": {"x": float("nan"), "y": 0, "z": 0}}, 1)
        with self.assertRaises(CourseError):
            Gate.from_dict({"id": 1, "position": [0, 0, 0], "yaw": float("inf")}, 1)

    def test_frame_matches_yaw(self) -> None:
        gate = Gate(id=1, position=Vec3(0, 0, 1), yaw=0.0)
        self.assertAlmostEqual(gate.frame().normal.x, 1.0)


class CourseTest(unittest.TestCase):
    def _course(self, gates: list[Gate], laps: int = 1) -> Course:
        return Course(name="t", gates=tuple(gates), laps=laps)

    def test_sequence_repeats_per_lap(self) -> None:
        gates = [
            Gate(id=1, position=Vec3(0, 0, 1), yaw=0),
            Gate(id=2, position=Vec3(3, 0, 1), yaw=0),
        ]
        course = self._course(gates, laps=3)
        self.assertEqual([g.id for g in course.sequence()], [1, 2, 1, 2, 1, 2])
        self.assertEqual(course.gate_count, 2)

    def test_gate_lookup(self) -> None:
        course = self._course([Gate(id=7, position=Vec3(0, 0, 1), yaw=0)])
        self.assertEqual(course.gate(7).id, 7)
        with self.assertRaises(KeyError):
            course.gate(1)

    def test_validate_flags_duplicate_ids(self) -> None:
        course = self._course(
            [
                Gate(id=1, position=Vec3(0, 0, 1), yaw=0),
                Gate(id=1, position=Vec3(5, 0, 1), yaw=0),
            ]
        )
        self.assertIn("duplicate gate id 1", course.validate())

    def test_validate_flags_overlapping_gates(self) -> None:
        course = self._course(
            [
                Gate(id=1, position=Vec3(0, 0, 1), yaw=0),
                Gate(id=2, position=Vec3(0.5, 0, 1), yaw=0),
            ]
        )
        problems = course.validate()
        self.assertTrue(any("closer than one gate width" in p for p in problems), problems)

    def test_stacked_gates_are_fine_because_validation_is_3d(self) -> None:
        course = self._course(
            [
                Gate(id=1, position=Vec3(0, 0, 4.0), yaw=0),
                Gate(id=2, position=Vec3(0, 0, 6.0), yaw=0),
            ]
        )
        self.assertEqual(course.validate(), [])

    def test_validate_flags_bad_laps_and_height(self) -> None:
        course = Course(name="t", gates=(Gate(id=1, position=Vec3(0, 0, 1), yaw=0),), laps=0, takeoff_height=0.0)
        problems = course.validate()
        self.assertTrue(any("laps" in p for p in problems))
        self.assertTrue(any("takeoff_height" in p for p in problems))

    def test_lap_length(self) -> None:
        course = Course(
            name="t",
            gates=(Gate(id=1, position=Vec3(3, 4, 0), yaw=0),),
            start=Vec3(0, 0, 0),
        )
        self.assertAlmostEqual(course.lap_length(), 5.0)


class BuildCourseTest(unittest.TestCase):
    def test_estimates_yaw_from_chord(self) -> None:
        course = build_course(
            "line",
            [Vec3(0, 0, 1), Vec3(1, 0, 1), Vec3(2, 0, 1)],
            start=Vec3(0, 0, 1),
        )
        self.assertEqual(course.gate_count, 3)
        for gate in course.gates:
            self.assertAlmostEqual(gate.yaw, 0.0, places=6)
            self.assertEqual(gate.yaw_source, "estimated")

    def test_empty_waypoints_rejected(self) -> None:
        with self.assertRaises(CourseError):
            build_course("empty", [])

    def test_duplicate_waypoints_do_not_crash(self) -> None:
        course = build_course("dup", [Vec3(0, 0, 1), Vec3(0, 0, 1)])
        self.assertEqual(course.gate_count, 2)


class ShippedCourseTest(unittest.TestCase):
    """The two course files in ``course/`` must load and validate."""

    def test_provisional_sample_loads(self) -> None:
        course = load_course(COURSE_DIR / "provisional_sample.json")
        self.assertEqual(course.gate_count, 7)
        self.assertEqual(course.laps, 3)
        self.assertEqual(len(course.sequence()), 21)
        self.assertEqual(course.validate(), [])
        self.assertIn("laser_uav_system", course.source)
        # provisional gates all carry an estimated yaw
        self.assertTrue(all(g.yaw_source == "estimated" for g in course.gates))

    def test_arena_template_loads_and_fits_the_rulebook_field(self) -> None:
        course = load_course(COURSE_DIR / "arena_template.json")
        self.assertEqual(course.gate_count, 7)
        self.assertEqual(course.validate(), [])
        for gate in course.gates:
            self.assertLessEqual(abs(gate.position.x), 2.5)
            self.assertLessEqual(abs(gate.position.y), 2.5)
            # the whole 1 m opening has to stay under the 2 m ceiling
            self.assertLessEqual(gate.position.z + gate.height / 2.0, 2.0)
            self.assertGreaterEqual(gate.position.z - gate.height / 2.0, 0.0)

    def test_stacked_provisional_gates_are_really_3d_separated(self) -> None:
        course = load_course(COURSE_DIR / "provisional_sample.json")
        gate4 = course.gate(4)
        gate5 = course.gate(5)
        self.assertGreater((gate4.position - gate5.position).norm(), 3.0)
        self.assertLess(gate4.position.horizontal_distance(gate5.position), 1.0)


class LoadingTest(unittest.TestCase):
    def test_missing_file(self) -> None:
        with self.assertRaises(CourseError):
            load_course("/nonexistent/course.json")

    def test_invalid_json(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.json"
            path.write_text("{not json", encoding="utf-8")
            with self.assertRaises(CourseError):
                load_course(path)

    def test_non_object_json(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "list.json"
            path.write_text("[]", encoding="utf-8")
            with self.assertRaises(CourseError):
                load_course(path)

    def test_missing_gates(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "nogates.json"
            path.write_text(json.dumps({"name": "x"}), encoding="utf-8")
            with self.assertRaises(CourseError):
                load_course(path)

    def test_validation_failure_surfaces_as_course_error(self) -> None:
        payload = {
            "name": "bad",
            "gates": [
                {"id": 1, "position": [0, 0, 1], "yaw": 0},
                {"id": 2, "position": [0.2, 0, 1], "yaw": 0},
            ],
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.json"
            path.write_text(json.dumps(payload), encoding="utf-8")
            with self.assertRaises(CourseError) as context:
                load_course(path)
            self.assertIn("closer than one gate width", str(context.exception))


if __name__ == "__main__":
    unittest.main()
