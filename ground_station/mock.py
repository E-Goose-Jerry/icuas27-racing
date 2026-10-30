"""Synthetic race playback so the dashboard works without a simulator.

``python3 -m ground_station.server --mock`` runs this instead of reading the
status file. It drives a real :class:`racing_core.mission_fsm.RaceMission`
through the shipped course, so what you see on screen is produced by the same
code that flies, not by hand-written JSON.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Any

from racing_core.course import Course, load_course
from racing_core.geometry import Vec3
from racing_core.mission_fsm import RaceMission
from racing_core.protocol import build_document
from racing_core.safety import SafetyMonitor
from racing_core.timing import RaceTimer

__all__ = ["MockRace"]


@dataclass
class MockRace:
    """Advance a synthetic attempt one tick at a time."""

    course_file: str
    speed: float = 2.0          # metres per second along the synthetic path
    tick_sec: float = 0.25
    course: Course = field(init=False)
    mission: RaceMission = field(init=False)
    safety: SafetyMonitor = field(init=False)
    _clock: float = field(default=0.0, init=False)
    _seq: int = field(default=0, init=False)
    _started: bool = field(default=False, init=False)
    _pose: Vec3 = field(default=Vec3(0.0, 0.0, 0.0), init=False)
    _segment: int = field(default=0, init=False)
    _path: list[Vec3] = field(default_factory=list, init=False)

    def __post_init__(self) -> None:
        self.course = load_course(self.course_file)
        self.mission = RaceMission(course=self.course, timer=RaceTimer(), lead=1.0)
        self.safety = SafetyMonitor(platform="sim")
        self._pose = Vec3(self.course.start.x, self.course.start.y, 0.0)
        self._rebuild_path()

    # -- path ------------------------------------------------------------
    def _rebuild_path(self) -> None:
        """Waypoints of the current leg: from where we are to the next gate."""
        gate = self.mission.target_gate()
        if gate is None:
            self._path = []
            return
        normal = Vec3(math.cos(gate.yaw), math.sin(gate.yaw), 0.0)
        self._path = [
            gate.position - normal * 1.2,
            gate.position,
            gate.position + normal * 1.2,
        ]
        self._segment = 0

    def _advance(self, distance: float) -> None:
        """Move ``distance`` metres along the current leg, stopping at each waypoint.

        The arrival test compares the step just taken against the *remaining*
        distance of that step. An accumulator across ticks would be wrong: the
        remaining distance shrinks every tick while the accumulator grows, so a
        waypoint would be declared reached long before the vehicle got there.
        """
        remaining = distance
        guard = 0
        while remaining > 1e-9 and guard < 64:
            guard += 1
            if self._segment >= len(self._path):
                break
            target = self._path[self._segment]
            delta = target - self._pose
            length = delta.norm()
            if length <= 1e-9:
                self._segment += 1
                continue
            step = min(remaining, length)
            self._pose = self._pose + delta * (step / length)
            remaining -= step
            if step >= length - 1e-9:
                self._segment += 1
                # feed the pose at the waypoint so a gate crossing is never
                # skipped between two samples
                self.mission.on_pose(self._clock, self._pose)

    # -- public tick -----------------------------------------------------
    def tick(self, real_elapsed: float) -> dict[str, Any]:
        """Advance by ``real_elapsed`` seconds and return a contract document."""
        steps = max(1, int(round(real_elapsed / self.tick_sec)))
        for _ in range(steps):
            self._clock += self.tick_sec
            self._step()

        self._seq += 1
        document = build_document(
            seq=self._seq,
            stamp=self._clock,
            race=self.mission.status(self._clock),
            safety=self.safety.to_dict(),
            course=self.course,
            gates=self._gates(),
            detection={"source": "mock", "visible": 1},
            notes=["mock playback"],
        )
        return document

    def _gates(self) -> list[dict[str, Any]]:
        target = self.mission.target_gate()
        passed_ids = {gate_id for gate_id, _ in self.mission.timer.gate_times}
        return [
            {
                "id": gate.id,
                "target": target is not None and gate.id == target.id,
                "passed": gate.id in passed_ids,
            }
            for gate in self.course.gates
        ]

    def _step(self) -> None:
        state = self.mission.state

        if state.value == "IDLE":
            actions = self.mission.start(self._clock)
            if actions:
                self._started = True
            return

        if state.value == "TAKEOFF":
            # climb to the takeoff height, then report the takeoff as complete
            target_z = max(self.course.takeoff_height, 1.0)
            self._pose = Vec3(self._pose.x, self._pose.y, self._pose.z + 0.6)
            if self._pose.z >= target_z:
                self._pose = Vec3(self._pose.x, self._pose.y, target_z)
                self.mission.on_takeoff_complete(self._clock)
                self._rebuild_path()
            return

        if state.value == "RACE":
            if not self._path:
                self._rebuild_path()
            before = self.mission.index
            self._advance(self.speed * self.tick_sec)
            self.mission.on_pose(self._clock, self._pose)
            if self.mission.index != before:
                self._rebuild_path()
            return

        if state.value == "LANDING":
            # descend at 0.8 m/s
            self._pose = Vec3(self._pose.x, self._pose.y, max(0.0, self._pose.z - 0.8 * self.tick_sec))
            if self._pose.z <= 0.05:
                self._pose = Vec3(self._pose.x, self._pose.y, 0.0)
                self.mission.on_landed(self._clock)
            return

        # DONE / ABORTED / SAFE_STOP: hold still
        return


def run_mock(course_file: str, interval: float = 0.2) -> dict[str, Any]:
    """One-shot helper used by tests."""
    race = MockRace(course_file=course_file)
    return race.tick(interval)
