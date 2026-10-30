"""Ground station: mock playback, document serving and HTTP routing."""

from __future__ import annotations

import json
import threading
import time
import unittest
import urllib.error
import urllib.request
from pathlib import Path

from ground_station.mock import MockRace
from ground_station.server import DashboardState, build_server
from ground_station.state import StatusSource
from racing_core.mission_fsm import State
from racing_core.protocol import validate_document, write_document

PROJECT_ROOT = Path(__file__).resolve().parent.parent
ARENA = PROJECT_ROOT / "course" / "arena_template.json"


class MockRaceTest(unittest.TestCase):
    def test_starts_with_a_valid_document(self) -> None:
        race = MockRace(course_file=str(ARENA))
        document = race.tick(0.25)
        self.assertEqual(validate_document(document), [])
        self.assertEqual(document["course"]["gate_count"], 7)

    def test_reaches_race_state(self) -> None:
        race = MockRace(course_file=str(ARENA))
        for _ in range(40):
            race.tick(0.25)
            if race.mission.state is State.RACE:
                break
        self.assertIs(race.mission.state, State.RACE)
        self.assertEqual(race.mission.index, 0)

    def test_completes_sixteen_passes_then_lands(self) -> None:
        race = MockRace(course_file=str(ARENA), speed=6.0)
        for _ in range(4000):
            race.tick(0.5)
            if race.mission.state is State.DONE:
                break
        self.assertIs(race.mission.state, State.DONE)
        self.assertEqual(len(race.mission.timer.gate_times), 14)
        self.assertFalse(race.mission.incomplete)
        self.assertFalse(race.mission.timer.running)

    def test_completes_at_slow_speed_too(self) -> None:
        # The replay used to declare a waypoint reached before arriving (an
        # accumulator compared against a shrinking distance), which only showed
        # up at small step sizes and looked like a missed gate in the FSM.
        race = MockRace(course_file=str(PROJECT_ROOT / "course" / "provisional_sample.json"), speed=2.0)
        for _ in range(40000):
            race.tick(0.5)
            if race.mission.state is State.DONE:
                break
        self.assertIs(race.mission.state, State.DONE)
        self.assertEqual(len(race.mission.timer.gate_times), 21)
        self.assertFalse(race.mission.incomplete)
        self.assertEqual(race.mission.timer.penalties, [])

    def test_documents_carry_gate_flags(self) -> None:
        race = MockRace(course_file=str(ARENA))
        race.tick(2.0)
        document = race.tick(0.25)
        self.assertEqual(len(document["gates"]), 7)
        self.assertTrue(all("target" in gate and "passed" in gate for gate in document["gates"]))

    def test_sequence_numbers_increase(self) -> None:
        race = MockRace(course_file=str(ARENA))
        first = race.tick(0.25)["seq"]
        second = race.tick(0.25)["seq"]
        self.assertGreater(second, first)


class StatusSourceTest(unittest.TestCase):
    def _document(self) -> dict:
        race = MockRace(course_file=str(ARENA))
        return race.tick(1.0)

    def test_missing_file_returns_none(self) -> None:
        source = StatusSource("/nonexistent/status.json")
        self.assertIsNone(source.snapshot())
        self.assertFalse(source.health()["exists"])

    def test_reads_and_reports_age(self) -> None:
        import tempfile

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "status.json"
            document = self._document()
            write_document(path, document)
            source = StatusSource(path)
            snapshot = source.snapshot(now=time.time() + 5.0)
            self.assertIsNotNone(snapshot)
            self.assertEqual(snapshot.document["schema"], document["schema"])
            self.assertGreaterEqual(snapshot.age_sec, 5.0)
            self.assertTrue(snapshot.stale)
            payload = snapshot.to_dict()
            self.assertIn("_meta", payload)
            self.assertTrue(payload["_meta"]["stale"])

    def test_health_reports_contract_problems(self) -> None:
        import tempfile

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "status.json"
            path.write_text(json.dumps({"schema": "wrong"}), encoding="utf-8")
            source = StatusSource(path)
            health = source.health()
            self.assertFalse(health["ok"])
            self.assertTrue(health["problems"])

    def test_course_accessor(self) -> None:
        import tempfile

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "status.json"
            write_document(path, self._document())
            self.assertIsNotNone(StatusSource(path).course())


class DashboardStateTest(unittest.TestCase):
    def test_file_mode_without_document(self) -> None:
        state = DashboardState(status_file="/nonexistent/none.json")
        self.assertIsNone(state.document())
        self.assertFalse(state.health()["ok"])

    def test_mock_mode_produces_documents(self) -> None:
        state = DashboardState(mock=True, course_file=ARENA)
        document = state.document()
        self.assertIsNotNone(document)
        self.assertEqual(document["schema"], "icuas27.race.v1")
        self.assertTrue(state.health()["ok"])


class HttpServerTest(unittest.TestCase):
    """End-to-end over a real socket, on an ephemeral port."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.server = build_server(host="127.0.0.1", port=0, mock=True, course_file=ARENA)
        cls.port = cls.server.server_address[1]
        cls.thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.thread.start()
        cls.base = f"http://127.0.0.1:{cls.port}"

    @classmethod
    def tearDownClass(cls) -> None:
        cls.server.shutdown()
        cls.server.server_close()
        cls.thread.join(timeout=5.0)

    def _get(self, path: str):
        with urllib.request.urlopen(self.base + path, timeout=5.0) as response:
            return response.status, response.read()

    def test_index_is_served(self) -> None:
        status, body = self._get("/")
        self.assertEqual(status, 200)
        self.assertIn(b"ICUAS 2027", body)
        self.assertIn(b"<svg", body)

    def test_status_is_valid_json(self) -> None:
        status, body = self._get("/api/status")
        self.assertEqual(status, 200)
        document = json.loads(body)
        self.assertEqual(validate_document(document), [])

    def test_course_endpoint(self) -> None:
        status, body = self._get("/api/course")
        self.assertEqual(status, 200)
        self.assertEqual(json.loads(body)["gate_count"], 7)

    def test_health_endpoint(self) -> None:
        status, body = self._get("/api/health")
        self.assertEqual(status, 200)
        self.assertTrue(json.loads(body)["ok"])

    def test_unknown_route_is_404(self) -> None:
        with self.assertRaises(urllib.error.HTTPError) as context:
            self._get("/api/nope")
        self.assertEqual(context.exception.code, 404)

    def test_favicon_is_empty(self) -> None:
        status, body = self._get("/favicon.ico")
        self.assertEqual(status, 204)
        self.assertEqual(body, b"")


if __name__ == "__main__":
    unittest.main()
