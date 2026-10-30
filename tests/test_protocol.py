"""The ground-station contract: build, validate, persist."""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from racing_core.course import load_course
from racing_core.mission_fsm import RaceMission
from racing_core.protocol import (
    SCHEMA,
    build_document,
    read_document,
    validate_document,
    write_document,
)
from racing_core.safety import SafetyMonitor

PROJECT_ROOT = Path(__file__).resolve().parent.parent


def sample_document(seq: int = 1) -> dict:
    course = load_course(PROJECT_ROOT / "course" / "provisional_sample.json")
    mission = RaceMission(course=course)
    mission.start(0.0)
    mission.on_takeoff_complete(1.0)
    safety = SafetyMonitor(platform="sim")
    safety.check(2.0, pose_at=1.9, pose=__import__("racing_core.geometry", fromlist=["Vec3"]).Vec3(0, 0, 4.2))
    return build_document(
        seq=seq,
        stamp=2.0,
        race=mission.status(2.0),
        safety=safety.to_dict(),
        course=course,
        gates=[{"id": gate.id, "passed": False, "target": gate.id == 1} for gate in course.gates],
        detection={"visible": 1, "last_gate_id": 1, "source": "color"},
        notes=["test"],
    )


class BuildDocumentTest(unittest.TestCase):
    def test_shape(self) -> None:
        document = sample_document()
        self.assertEqual(document["schema"], SCHEMA)
        self.assertEqual(document["seq"], 1)
        self.assertEqual(document["course"]["gate_count"], 7)
        self.assertEqual(document["race"]["state"], "RACE")
        self.assertEqual(len(document["gates"]), 7)
        self.assertEqual(validate_document(document), [])

    def test_values_are_copied_not_referenced(self) -> None:
        race = {"state": "IDLE", "gate_index": 0, "total_gates": 1, "timer": {}}
        safety = {"level": "OK"}
        course = load_course(PROJECT_ROOT / "course" / "arena_template.json")
        document = build_document(seq=0, stamp=0.0, race=race, safety=safety, course=course)
        race["state"] = "MUTATED"
        safety["level"] = "MUTATED"
        self.assertEqual(document["race"]["state"], "IDLE")
        self.assertEqual(document["safety"]["level"], "OK")

    def test_notes_are_stringified(self) -> None:
        course = load_course(PROJECT_ROOT / "course" / "arena_template.json")
        document = build_document(
            seq=0,
            stamp=1.0,
            race={"state": "IDLE", "gate_index": 0, "total_gates": 0, "timer": {}},
            safety={},
            course=course,
            notes=[1, None, "x"],
        )
        self.assertEqual(document["notes"], ["1", "None", "x"])


class ValidateDocumentTest(unittest.TestCase):
    def test_missing_keys(self) -> None:
        problems = validate_document({"schema": SCHEMA})
        self.assertTrue(any("missing top-level key" in p for p in problems))

    def test_non_mapping(self) -> None:
        self.assertEqual(validate_document([]), ["document must be an object"])  # type: ignore[arg-type]

    def test_wrong_schema(self) -> None:
        document = sample_document()
        document["schema"] = "something.else"
        self.assertTrue(any("schema is" in p for p in validate_document(document)))

    def test_bad_seq_and_stamp(self) -> None:
        document = sample_document()
        document["seq"] = "one"
        document["stamp"] = float("nan")
        problems = validate_document(document)
        self.assertIn("seq must be an integer", problems)
        self.assertIn("stamp must be a finite number", problems)

    def test_missing_race_keys(self) -> None:
        document = sample_document()
        del document["race"]["state"]
        self.assertTrue(any("missing race key: state" in p for p in validate_document(document)))

    def test_missing_timer_keys(self) -> None:
        document = sample_document()
        document["race"]["timer"] = {"elapsed": None}
        problems = validate_document(document)
        self.assertTrue(any("missing timer key" in p for p in problems))

    def test_timer_not_an_object(self) -> None:
        document = sample_document()
        document["race"]["timer"] = 5
        self.assertIn("race.timer must be an object", validate_document(document))

    def test_gate_entries_need_ids(self) -> None:
        document = sample_document()
        document["gates"] = [{"passed": True}]
        self.assertTrue(any("must be an object with an id" in p for p in validate_document(document)))

    def test_course_gates_must_be_a_list(self) -> None:
        document = sample_document()
        document["course"]["gates"] = {}
        self.assertIn("course.gates must be a list", validate_document(document))


class PersistTest(unittest.TestCase):
    def test_write_then_read(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "nested" / "status.json"
            document = sample_document()
            write_document(path, document)
            self.assertTrue(path.is_file())
            self.assertEqual(read_document(path), document)

    def test_read_missing_and_corrupt(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            directory_path = Path(directory)
            self.assertIsNone(read_document(directory_path / "absent.json"))
            corrupt = directory_path / "corrupt.json"
            corrupt.write_text("{", encoding="utf-8")
            self.assertIsNone(read_document(corrupt))
            not_object = directory_path / "list.json"
            not_object.write_text("[]", encoding="utf-8")
            self.assertIsNone(read_document(not_object))

    def test_write_leaves_no_temp_files(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "status.json"
            write_document(path, sample_document())
            write_document(path, sample_document(seq=2))
            leftovers = [p.name for p in Path(directory).iterdir() if p.name.endswith(".tmp")]
            self.assertEqual(leftovers, [])
            self.assertEqual(json.loads(path.read_text())["seq"], 2)


if __name__ == "__main__":
    unittest.main()
