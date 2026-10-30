"""The race mission state machine: gates, retries, penalties, landing."""

from __future__ import annotations

import math
import unittest

from racing_core.course import Course, Gate
from racing_core.geometry import Vec3
from racing_core.mission_fsm import ActionKind, MissionError, RaceMission, State
from racing_core.timing import RaceTimer


def straight_course(gate_count: int = 3, laps: int = 1, with_landing: bool = False) -> Course:
    """Gates along +X, one metre apart, all facing +X."""
    gates = tuple(
        Gate(id=index + 1, position=Vec3(float(index) * 2.0, 0.0, 1.0), yaw=0.0)
        for index in range(gate_count)
    )
    return Course(
        name="straight",
        gates=gates,
        laps=laps,
        start=Vec3(-2.0, 0.0, 0.5),
        takeoff_height=1.0,
        landing=Vec3(10.0, 0.0, 0.0) if with_landing else None,
    )


def before(gate: Gate, distance: float = 0.6) -> Vec3:
    normal = Vec3(math.cos(gate.yaw), math.sin(gate.yaw), 0.0)
    return gate.position - normal * distance


def after(gate: Gate, distance: float = 0.6) -> Vec3:
    normal = Vec3(math.cos(gate.yaw), math.sin(gate.yaw), 0.0)
    return gate.position + normal * distance


class MissionTest(unittest.TestCase):
    def setUp(self) -> None:
        self.mission = RaceMission(course=straight_course())

    def _start(self) -> None:
        self.mission.start(0.0)
        self.mission.on_takeoff_complete(0.5)

    def _pass_gate(self, gate: Gate, now: float = 1.0) -> None:
        self.mission.on_pose(now, before(gate))
        self.mission.on_pose(now + 0.1, after(gate))

    # -- startup ---------------------------------------------------------
    def test_starts_idle(self) -> None:
        self.assertIs(self.mission.state, State.IDLE)
        self.assertIsNone(self.mission.target_gate())

    def test_start_commands_takeoff(self) -> None:
        actions = self.mission.start(0.0)
        self.assertEqual(len(actions), 1)
        self.assertIs(actions[0].kind, ActionKind.START_TAKEOFF)
        self.assertIs(self.mission.state, State.TAKEOFF)

    def test_cannot_start_twice(self) -> None:
        self.mission.start(0.0)
        with self.assertRaises(MissionError):
            self.mission.start(1.0)

    def test_takeoff_complete_starts_clock_and_targets_first_gate(self) -> None:
        self.mission.start(0.0)
        actions = self.mission.on_takeoff_complete(0.5)
        self.assertEqual(len(actions), 1)
        self.assertIs(actions[0].kind, ActionKind.FLY_GATE)
        self.assertEqual(actions[0].gate_id, 1)
        self.assertIs(self.mission.state, State.RACE)
        self.assertTrue(self.mission.timer.running)

    def test_takeoff_complete_out_of_order_raises(self) -> None:
        with self.assertRaises(MissionError):
            self.mission.on_takeoff_complete(0.0)

    # -- waypoints -------------------------------------------------------
    def test_target_gate_is_none_before_the_race(self) -> None:
        self.mission.start(0.0)  # TAKEOFF, not RACE yet
        self.assertIs(self.mission.state, State.TAKEOFF)
        self.assertIsNone(self.mission.target_gate())

    def test_waypoint_geometry(self) -> None:
        mission = RaceMission(course=straight_course(), lead=1.0)
        mission.start(0.0)
        actions = mission.on_takeoff_complete(0.0)
        waypoints = actions[0].waypoints
        self.assertEqual(len(waypoints), 3)
        self.assertAlmostEqual(waypoints[1].x, 0.0)
        self.assertAlmostEqual(waypoints[0].x, -1.0)
        self.assertAlmostEqual(waypoints[2].x, 1.0)
        self.assertAlmostEqual(actions[0].heading, 0.0)

    # -- gate passing ----------------------------------------------------
    def test_passing_all_gates_finishes_and_lands(self) -> None:
        self._start()
        gates = self.mission.course.gates
        self._pass_gate(gates[0], now=1.0)
        self.assertEqual(self.mission.index, 1)
        self._pass_gate(gates[1], now=2.0)
        self._pass_gate(gates[2], now=3.0)
        self.assertIs(self.mission.state, State.LANDING)
        self.assertFalse(self.mission.timer.running)
        # clock started at takeoff-complete (0.5 s) and stopped at 3.1 s
        self.assertAlmostEqual(self.mission.timer.elapsed(), 2.6)
        self.assertEqual(self.mission.index, 3)
        self.assertEqual(len(self.mission.timer.gate_times), 3)

    def test_finish_with_landing_point_flies_there_first(self) -> None:
        mission = RaceMission(course=straight_course(gate_count=1, with_landing=True))
        mission.start(0.0)
        mission.on_takeoff_complete(0.0)
        actions = mission.on_pose(1.0, before(mission.course.gates[0]))
        actions = mission.on_pose(1.1, after(mission.course.gates[0]))
        self.assertEqual([a.kind for a in actions], [ActionKind.FLY_LANDING, ActionKind.LAND])
        self.assertEqual(actions[0].waypoints[0], Vec3(10.0, 0.0, 0.0))

    def test_landing_completes_the_run(self) -> None:
        mission = RaceMission(course=straight_course(gate_count=1))
        mission.start(0.0)
        mission.on_takeoff_complete(0.0)
        mission.on_pose(1.0, before(mission.course.gates[0]))
        mission.on_pose(1.1, after(mission.course.gates[0]))
        mission.on_landed(2.0)
        self.assertIs(mission.state, State.DONE)

    def test_landed_is_a_noop_outside_landing(self) -> None:
        self.assertEqual(self.mission.on_landed(1.0), ())
        self.assertIs(self.mission.state, State.IDLE)

    # -- laps ------------------------------------------------------------
    def test_two_laps_require_double_the_passes(self) -> None:
        mission = RaceMission(course=straight_course(gate_count=2, laps=2))
        mission.start(0.0)
        mission.on_takeoff_complete(0.0)
        gates = mission.course.gates
        for lap in range(2):
            for gate in gates:
                mission.on_pose(1.0 + lap, before(gate))
                mission.on_pose(1.1 + lap, after(gate))
        self.assertIs(mission.state, State.LANDING)
        self.assertEqual(mission.laps_done, 2)

    def test_laps_done_tracks_progress(self) -> None:
        mission = RaceMission(course=straight_course(gate_count=2, laps=3))
        self.assertEqual(mission.laps_done, 0)
        mission.start(0.0)
        mission.on_takeoff_complete(0.0)
        gates = mission.course.gates
        mission.on_pose(1.0, before(gates[0]))
        mission.on_pose(1.1, after(gates[0]))
        self.assertEqual(mission.laps_done, 0)
        mission.on_pose(2.0, before(gates[1]))
        mission.on_pose(2.1, after(gates[1]))
        self.assertEqual(mission.laps_done, 1)

    # -- misses ----------------------------------------------------------
    def test_miss_outside_opening_retries_without_penalty(self) -> None:
        mission = RaceMission(course=straight_course(), retry_limit=2)
        mission.start(0.0)
        mission.on_takeoff_complete(0.0)
        gate = mission.course.gates[0]
        lateral = 0.9  # outside the 1 m opening
        mission.on_pose(1.0, Vec3(-0.6, lateral, 1.0))
        actions = mission.on_pose(1.1, Vec3(0.6, lateral, 1.0))
        self.assertEqual(actions[0].kind, ActionKind.FLY_GATE)
        self.assertEqual(actions[0].gate_id, 1)
        self.assertEqual(mission.index, 0)
        self.assertEqual(mission.attempts, 1)
        self.assertEqual(mission.timer.penalty_total, 0.0)
        self.assertFalse(mission.incomplete)

    def test_default_keeps_retrying_the_same_gate(self) -> None:
        # Rulebook 3.2: a missed gate needs a legal retry, skipping does not
        # count as completion. So the default must never advance.
        mission = RaceMission(course=straight_course(), retry_limit=1)
        mission.start(0.0)
        mission.on_takeoff_complete(0.0)
        for attempt in range(4):
            now = 1.0 + attempt
            mission.on_pose(now, Vec3(-0.6, 2.0, 1.0))
            actions = mission.on_pose(now + 0.1, Vec3(0.6, 2.0, 1.0))
            self.assertEqual(mission.index, 0)
            self.assertEqual(actions[0].gate_id, 1)
        self.assertEqual(mission.timer.penalty_total, 0.0)
        self.assertFalse(mission.incomplete)
        self.assertTrue(mission.stuck)
        self.assertFalse(mission.allow_skip_with_penalty)

    def test_non_compliant_skip_must_be_opt_in(self) -> None:
        mission = RaceMission(
            course=straight_course(), retry_limit=1, allow_skip_with_penalty=True
        )
        mission.start(0.0)
        mission.on_takeoff_complete(0.0)
        gate = mission.course.gates[0]
        lateral = 2.0
        mission.on_pose(1.0, Vec3(-0.6, lateral, 1.0))
        mission.on_pose(1.1, Vec3(0.6, lateral, 1.0))  # attempt 1 -> retry
        now = 2.0
        mission.on_pose(now, Vec3(-0.6, lateral, 1.0))
        actions = mission.on_pose(now + 0.1, Vec3(0.6, lateral, 1.0))  # attempt 2 -> give up
        self.assertTrue(mission.incomplete)
        self.assertAlmostEqual(mission.timer.penalty_total, 5.0)
        self.assertEqual(mission.index, 1)
        self.assertEqual(mission.attempts, 0)
        self.assertEqual(actions[0].gate_id, 2)

    def test_non_compliant_skip_on_last_gate_still_finishes(self) -> None:
        mission = RaceMission(
            course=straight_course(gate_count=1),
            retry_limit=0,
            allow_skip_with_penalty=True,
        )
        mission.start(0.0)
        mission.on_takeoff_complete(0.0)
        mission.on_pose(1.0, Vec3(-0.6, 2.0, 1.0))
        actions = mission.on_pose(1.1, Vec3(0.6, 2.0, 1.0))
        self.assertEqual([a.kind for a in actions], [ActionKind.LAND])
        self.assertIs(mission.state, State.LANDING)
        self.assertAlmostEqual(mission.timer.penalty_total, 5.0)

    def test_fly_by_far_from_the_gate_is_ignored(self) -> None:
        # Crossing the gate's plane 4 m above it is not an attempt at the gate.
        mission = RaceMission(course=straight_course(), capture_radius=2.5)
        mission.start(0.0)
        mission.on_takeoff_complete(0.0)
        mission.on_pose(1.0, Vec3(-1.0, 0.0, 5.0))
        self.assertEqual(mission.on_pose(1.1, Vec3(1.0, 0.0, 5.0)), ())
        self.assertEqual(mission.attempts, 0)
        self.assertEqual(mission.index, 0)
        self.assertEqual(mission.timer.penalty_total, 0.0)

    def test_pass_survives_a_sample_landing_on_the_plane(self) -> None:
        # The waypoint we steer to is the gate centre, so a sample can land
        # exactly on the plane. That sample cannot decide the direction by
        # itself; the pass is credited one sample later and is never lost.
        mission = RaceMission(course=straight_course())
        mission.start(0.0)
        mission.on_takeoff_complete(0.0)
        gate = mission.course.gates[0]
        mission.on_pose(1.0, before(gate))
        mission.on_pose(1.1, gate.position)          # exactly on the plane
        self.assertEqual(mission.index, 0)           # deferred by one sample
        self.assertEqual(mission.attempts, 0)        # and not counted as a miss
        actions = mission.on_pose(1.2, after(gate))
        self.assertEqual(mission.index, 1)           # credited here
        self.assertEqual(mission.timer.penalty_total, 0.0)
        self.assertEqual(actions[0].gate_id, 2)

    def test_backwards_crossing_retries_exactly_the_same_gate(self) -> None:
        mission = RaceMission(course=straight_course())
        mission.start(0.0)
        mission.on_takeoff_complete(0.0)
        gate = mission.course.gates[0]
        mission.on_pose(1.0, after(gate))
        actions = mission.on_pose(1.1, before(gate))
        self.assertEqual(actions[0].gate_id, 1)
        self.assertEqual(mission.attempts, 0)
        self.assertEqual(mission.timer.penalty_total, 0.0)
        self.assertIn("crossed backwards", actions[0].detail)

    def test_non_crossing_poses_do_not_advance(self) -> None:
        mission = RaceMission(course=straight_course())
        mission.start(0.0)
        mission.on_takeoff_complete(0.0)
        for step in range(10):
            self.assertEqual(mission.on_pose(1.0 + step * 0.1, Vec3(-5.0, 0.0, 1.0)), ())
        self.assertEqual(mission.index, 0)

    def test_no_crossing_detected_before_race(self) -> None:
        gate = self.mission.course.gates[0]
        self.mission.on_pose(0.0, before(gate))
        self.assertEqual(self.mission.on_pose(0.1, after(gate)), ())
        self.assertEqual(self.mission.index, 0)

    # -- aborts ----------------------------------------------------------
    def test_abort_cancels_and_lands(self) -> None:
        self._start()
        actions = self.mission.abort("operator", now=1.0)
        self.assertEqual([a.kind for a in actions], [ActionKind.CANCEL_PLANNING, ActionKind.LAND])
        self.assertIs(self.mission.state, State.ABORTED)
        self.assertTrue(self.mission.incomplete)
        self.assertFalse(self.mission.timer.running)
        self.assertEqual(self.mission.abort_reason, "operator")

    def test_abort_is_idempotent(self) -> None:
        self._start()
        self.mission.abort("x", now=1.0)
        self.assertEqual(self.mission.abort("y", now=2.0), ())

    def test_safe_stop_holds_instead_of_landing(self) -> None:
        self._start()
        actions = self.mission.safe_stop("pose stale", now=1.0)
        self.assertEqual([a.kind for a in actions], [ActionKind.CANCEL_PLANNING, ActionKind.HOLD])
        self.assertIs(self.mission.state, State.SAFE_STOP)

    def test_safe_stop_is_idempotent(self) -> None:
        self._start()
        self.mission.safe_stop("x", now=1.0)
        self.assertEqual(self.mission.safe_stop("y", now=2.0), ())

    # -- reporting -------------------------------------------------------
    def test_status_snapshot(self) -> None:
        self._start()
        status = self.mission.status(1.0)
        self.assertEqual(status["state"], "RACE")
        self.assertEqual(status["course"], "straight")
        self.assertEqual(status["total_gates"], 3)
        self.assertEqual(status["gate_index"], 0)
        self.assertEqual(status["target_gate_id"], 1)
        self.assertFalse(status["incomplete"])
        self.assertIn("timer", status)
        self.assertIn("history", status)

    def test_history_records_events(self) -> None:
        self._start()
        self._pass_gate(self.mission.course.gates[0])
        self.assertTrue(any("gate 1 passed" in line for line in self.mission.history))

    def test_non_finite_pose_rejected(self) -> None:
        self._start()
        with self.assertRaises(MissionError):
            self.mission.on_pose(1.0, Vec3(float("nan"), 0.0, 1.0))

    def test_construction_rejects_bad_parameters(self) -> None:
        with self.assertRaises(ValueError):
            RaceMission(course=straight_course(), retry_limit=-1)
        with self.assertRaises(ValueError):
            RaceMission(course=straight_course(), lead=0.0)


class ScriptedRunTest(unittest.TestCase):
    """Drive the shipped provisional course end to end, as a sanity check."""

    def test_full_three_lap_run(self) -> None:
        from racing_core.course import load_course

        project_root = __import__("pathlib").Path(__file__).resolve().parent.parent
        course = load_course(project_root / "course" / "provisional_sample.json")
        mission = RaceMission(course=course, timer=RaceTimer())
        mission.start(0.0)
        mission.on_takeoff_complete(2.0)

        clock = 3.0
        for gate in mission.sequence:
            normal = Vec3(math.cos(gate.yaw), math.sin(gate.yaw), 0.0)
            mission.on_pose(clock, gate.position - normal * 1.5)
            mission.on_pose(clock + 0.2, gate.position + normal * 1.5)
            clock += 1.0

        self.assertIs(mission.state, State.LANDING)
        self.assertFalse(mission.incomplete)
        self.assertEqual(len(mission.timer.gate_times), 21)
        self.assertFalse(mission.timer.running)

    def test_shipped_arena_course_is_flyable_in_the_machine(self) -> None:
        from pathlib import Path

        from racing_core.course import load_course

        project_root = Path(__file__).resolve().parent.parent
        course = load_course(project_root / "course" / "arena_template.json")
        mission = RaceMission(course=course, timer=RaceTimer())
        mission.start(0.0)
        mission.on_takeoff_complete(1.0)
        clock = 2.0
        for gate in mission.sequence:
            normal = Vec3(math.cos(gate.yaw), math.sin(gate.yaw), 0.0)
            mission.on_pose(clock, gate.position - normal * 0.7)
            mission.on_pose(clock + 0.1, gate.position + normal * 0.7)
            clock += 0.6
        self.assertIs(mission.state, State.LANDING)
        self.assertFalse(mission.incomplete)
        self.assertEqual(len(mission.timer.gate_times), 14)


if __name__ == "__main__":
    unittest.main()
