"""Elapsed time, penalties and the adjusted score."""

from __future__ import annotations

import unittest

from racing_core.timing import PENALTY_SECONDS, Penalty, RaceTimer, format_duration


class FormatTest(unittest.TestCase):
    def test_formats(self) -> None:
        self.assertEqual(format_duration(0.0), "0:00.000")
        self.assertEqual(format_duration(1.5), "0:01.500")
        self.assertEqual(format_duration(83.456), "1:23.456")
        self.assertEqual(format_duration(None), "-")
        self.assertEqual(format_duration(-2.0), "-0:02.000")


class TimerTest(unittest.TestCase):
    def test_lifecycle(self) -> None:
        timer = RaceTimer()
        self.assertIsNone(timer.elapsed(0.0))
        self.assertFalse(timer.running)

        timer.start(100.0)
        self.assertTrue(timer.running)
        self.assertAlmostEqual(timer.elapsed(110.0), 10.0)

        timer.stop(120.0)
        self.assertFalse(timer.running)
        # stopped timers ignore later "now" values
        self.assertAlmostEqual(timer.elapsed(999.0), 20.0)

    def test_double_start_is_rejected(self) -> None:
        timer = RaceTimer()
        timer.start(0.0)
        with self.assertRaises(RuntimeError):
            timer.start(1.0)

    def test_stop_before_start_is_rejected(self) -> None:
        with self.assertRaises(RuntimeError):
            RaceTimer().stop(1.0)

    def test_stop_time_is_clamped_to_start(self) -> None:
        timer = RaceTimer()
        timer.start(10.0)
        timer.stop(5.0)
        self.assertEqual(timer.elapsed(), 0.0)

    def test_reset_clears_everything(self) -> None:
        timer = RaceTimer()
        timer.start(0.0)
        timer.add_penalty("collision", gate_id=2)
        timer.mark_gate(1, 5.0)
        timer.reset()
        self.assertIsNone(timer.elapsed())
        self.assertEqual(timer.penalties, [])
        self.assertEqual(timer.gate_times, [])

    def test_penalty_total_and_adjusted(self) -> None:
        timer = RaceTimer()
        timer.start(0.0)
        timer.stop(30.0)
        timer.add_penalty("collision", gate_id=1)
        timer.add_penalty("incomplete_gate", gate_id=4)
        self.assertAlmostEqual(timer.penalty_total, 2 * PENALTY_SECONDS)
        self.assertAlmostEqual(timer.adjusted(), 40.0)
        self.assertEqual(timer.snapshot()["adjusted_text"], "0:40.000")

    def test_penalty_seconds_override(self) -> None:
        timer = RaceTimer()
        timer.start(0.0)
        penalty = timer.add_penalty("manual", seconds=2.5)
        self.assertAlmostEqual(penalty.seconds, 2.5)
        self.assertAlmostEqual(timer.penalty_total, 2.5)

    def test_negative_penalty_rejected(self) -> None:
        with self.assertRaises(ValueError):
            RaceTimer().add_penalty("bad", seconds=-1.0)

    def test_adjusted_while_running(self) -> None:
        timer = RaceTimer()
        timer.start(1000.0)
        timer.add_penalty("collision", gate_id=3)
        self.assertAlmostEqual(timer.adjusted(1005.0), 10.0)

    def test_splits_record_elapsed_offsets(self) -> None:
        timer = RaceTimer()
        timer.start(50.0)
        timer.mark_gate(1, 52.5)
        timer.mark_gate(2, 60.0)
        splits = timer.snapshot(60.0)["splits"]
        self.assertEqual([s["gate_id"] for s in splits], [1, 2])
        self.assertAlmostEqual(splits[0]["at"], 2.5)
        self.assertEqual(splits[1]["at_text"], "0:10.000")

    def test_snapshot_shape(self) -> None:
        timer = RaceTimer()
        timer.start(0.0)
        snapshot = timer.snapshot(12.0)
        for key in ("running", "elapsed", "elapsed_text", "penalty_total", "adjusted", "splits"):
            self.assertIn(key, snapshot)
        self.assertEqual(snapshot["elapsed_text"], "0:12.000")

    def test_snapshot_before_start(self) -> None:
        snapshot = RaceTimer().snapshot(0.0)
        self.assertIsNone(snapshot["elapsed"])
        self.assertEqual(snapshot["elapsed_text"], "-")
        self.assertIsNone(snapshot["adjusted"])


class PenaltyTest(unittest.TestCase):
    def test_serialise(self) -> None:
        payload = Penalty("collision", gate_id=2, note="wall").to_dict()
        self.assertEqual(payload["reason"], "collision")
        self.assertEqual(payload["gate_id"], 2)
        self.assertEqual(payload["note"], "wall")
        self.assertAlmostEqual(payload["seconds"], PENALTY_SECONDS)


if __name__ == "__main__":
    unittest.main()
