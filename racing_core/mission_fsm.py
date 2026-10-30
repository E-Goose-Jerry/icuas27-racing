"""The race mission state machine.

This module owns *competition* logic only: what to fly next, when a gate counts,
when the clock runs, when to abort. It does not import ROS, does not read
clocks, and does not publish anything. The ROS node in ``icuas27_racing`` turns
:class:`Action` objects into topics and service calls, which keeps the whole
decision path unit testable.

State graph::

    IDLE --start--> TAKEOFF --on_takeoff_complete--> RACE
    RACE --last gate passed--> LANDING --on_landed--> DONE
    any  --abort--> ABORTED
    any  --safe stop--> SAFE_STOP

Gate accounting follows rulebook section 10: a run that does not pass a gate is
penalised (5 s) and does not rank. The machine therefore retries a gate it did
not pass cleanly, and only after ``retry_limit`` attempts does it take the
penalty and move on -- with ``incomplete`` latched so scoring can see the run
will not rank.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Iterable

from .course import Course, Gate
from .geometry import CrossingResult, Vec3, crossing, side_of
from .timing import PENALTY_SECONDS, RaceTimer

__all__ = [
    "Action",
    "ActionKind",
    "MissionError",
    "RaceMission",
    "State",
]

# Default lead-in / lead-out distance used to build the three waypoints that
# carry the vehicle straight through a gate.
DEFAULT_LEAD = 1.5

# A plane crossing further than this from the gate centre is a fly-by, not an
# attempt: the vehicle merely crossed the gate's plane while heading elsewhere.
DEFAULT_CAPTURE_RADIUS = 2.5


class MissionError(RuntimeError):
    """Raised for calls that make no sense in the current state."""


class State(str, Enum):
    IDLE = "IDLE"
    TAKEOFF = "TAKEOFF"
    RACE = "RACE"
    LANDING = "LANDING"
    DONE = "DONE"
    ABORTED = "ABORTED"
    SAFE_STOP = "SAFE_STOP"


class ActionKind(str, Enum):
    START_TAKEOFF = "start_takeoff"
    FLY_GATE = "fly_gate"
    FLY_LANDING = "fly_landing"
    LAND = "land"
    CANCEL_PLANNING = "cancel_planning"
    HOLD = "hold"


@dataclass(frozen=True, slots=True)
class Action:
    """One instruction for the ROS layer to carry out."""

    kind: ActionKind
    gate_id: int | None = None
    waypoints: tuple[Vec3, ...] = ()
    heading: float | None = None
    speed: float | None = None
    detail: str = ""

    def to_dict(self) -> dict[str, Any]:
        return {
            "kind": self.kind.value,
            "gate_id": self.gate_id,
            "waypoints": [point.to_dict() for point in self.waypoints],
            "heading": self.heading,
            "speed": self.speed,
            "detail": self.detail,
        }


def _lead_waypoints(gate: Gate, lead: float) -> tuple[Vec3, ...]:
    """Entry / centre / exit waypoints that make the vehicle fly straight through."""
    normal = Vec3(math.cos(gate.yaw), math.sin(gate.yaw), 0.0)
    return (
        gate.position - normal * lead,
        gate.position,
        gate.position + normal * lead,
    )


@dataclass
class RaceMission:
    """Deterministic race mission driver.

    Parameters
    ----------
    course:
        Gates and lap count.
    timer:
        Timing ledger. A fresh one is created when omitted.
    retry_limit:
        How many times a gate is re-issued after an unclean crossing before the
        run takes the penalty and continues.
    lead:
        Lead-in/lead-out distance used for the per-gate waypoints.
    speed:
        Requested speed published with the waypoint path (``0`` means "let the
        planner decide").
    """

    course: Course
    timer: RaceTimer = field(default_factory=RaceTimer)
    retry_limit: int = 2
    lead: float = DEFAULT_LEAD
    speed: float = 0.0
    capture_radius: float = DEFAULT_CAPTURE_RADIUS
    allow_skip_with_penalty: bool = False

    state: State = State.IDLE
    index: int = 0
    attempts: int = 0
    stuck: bool = False
    incomplete: bool = False
    abort_reason: str = ""
    _sequence: tuple[Gate, ...] = field(default=(), repr=False)
    _previous_pose: Vec3 | None = field(default=None, repr=False)
    _current_pose: Vec3 | None = field(default=None, repr=False)
    _target_side: int = field(default=0, repr=False)
    _history: list[str] = field(default_factory=list, repr=False)

    def __post_init__(self) -> None:
        if self.retry_limit < 0:
            raise ValueError("retry_limit must not be negative")
        if self.lead <= 0.0:
            raise ValueError("lead must be positive")
        if self.capture_radius <= 0.0:
            raise ValueError("capture_radius must be positive")
        if self.course.laps < 1:
            raise ValueError("course must have at least one lap")
        self._sequence = self.course.sequence()

    # -- derived state ---------------------------------------------------
    @property
    def sequence(self) -> tuple[Gate, ...]:
        return self._sequence

    @property
    def total_gates(self) -> int:
        return len(self._sequence)

    @property
    def laps_done(self) -> int:
        return min(self.index // max(1, self.course.gate_count), self.course.laps)

    def target_gate(self) -> Gate | None:
        if self.state is not State.RACE:
            return None
        if self.index >= len(self._sequence):
            return None
        return self._sequence[self.index]

    # -- transitions -----------------------------------------------------
    def start(self, now: float) -> tuple[Action, ...]:
        """Begin the attempt: arm has succeeded and takeoff may be commanded."""
        if self.state is not State.IDLE:
            raise MissionError(f"cannot start from state {self.state.value}")
        self.state = State.TAKEOFF
        self._note("start")
        return (
            Action(
                kind=ActionKind.START_TAKEOFF,
                detail=f"takeoff to {self.course.takeoff_height:.2f} m",
            ),
        )

    def on_takeoff_complete(self, now: float) -> tuple[Action, ...]:
        """Takeoff finished: the clock starts and the first gate is issued."""
        if self.state is not State.TAKEOFF:
            raise MissionError(f"takeoff complete in state {self.state.value}")
        self.state = State.RACE
        self.timer.start(now)
        self._note("takeoff_complete")
        gate = self.target_gate()
        if gate is None:  # a course with no gates cannot get here; be explicit
            return self._finish(now)
        return (self._gate_action(gate),)

    def on_pose(self, now: float, position: Vec3, heading: float | None = None) -> tuple[Action, ...]:
        """Feed one pose sample; returns the actions it triggers.

        Crossing detection uses the segment between the previous and current
        pose, so the sample rate only has to keep the vehicle from crossing a
        gate and coming back within a single sample window.
        """
        position = Vec3.from_any(position)
        if not position.is_finite():
            raise MissionError("pose is not finite")
        previous = self._previous_pose
        self._previous_pose = position
        self._current_pose = position

        if self.state is not State.RACE or previous is None:
            return ()

        gate = self.target_gate()
        if gate is None:
            return ()

        frame = gate.frame()
        # Remember the last side we were sure about. The waypoint we steer to is
        # the gate centre, so a sample landing exactly on the plane is the normal
        # case; it has no side of its own and must not erase the crossing.
        # When the memory is empty (first segment after a target change) seed it
        # from the previous pose, otherwise a single on-plane sample would
        # disconnect the "behind the gate" side from the "in front" one.
        if self._target_side == 0:
            self._target_side = side_of(frame, previous)
        observed_side = side_of(frame, position)
        result = crossing(frame, previous, position, start_side=self._target_side)
        if observed_side != 0:
            self._target_side = observed_side
        if not result.crossed:
            return ()
        return self._handle_crossing(now, gate, result)

    def _handle_crossing(self, now: float, gate: Gate, result: CrossingResult) -> tuple[Action, ...]:
        if result.valid_pass:
            self.timer.mark_gate(gate.id, now)
            self._note(f"gate {gate.id} passed")
            self.index += 1
            self.attempts = 0
            self.stuck = False
            self._target_side = 0
            if self.index >= len(self._sequence):
                return self._finish(now)
            return (self._gate_action(self._sequence[self.index]),)

        if result.direction < 0:
            # Crossed the plane backwards: no progress, ask for the same gate.
            self._note(f"gate {gate.id} crossed backwards")
            return (self._gate_action(gate, detail="retry: crossed backwards"),)

        if result.distance_from_centre > self.capture_radius:
            # Crossing the plane metres away from the gate is not an attempt.
            self._note(
                f"ignored fly-by of gate {gate.id} "
                f"({result.distance_from_centre:.2f} m off centre)"
            )
            return ()

        # Forward crossing outside the opening: the gate does not count.
        self.attempts += 1
        self._note(
            f"gate {gate.id} missed (lateral {result.lateral:+.2f} m, "
            f"vertical {result.vertical:+.2f} m)"
        )

        if self.allow_skip_with_penalty and self.attempts > self.retry_limit:
            # NON-COMPLIANT shortcut, off by default. Rulebook 3.2: "Missing a
            # gate requires a legal retry of that checkpoint; skipping it does
            # not count as completion." Keeping it makes the runner usable for
            # pipeline tests, never for a scored run.
            self.timer.add_penalty(
                "incomplete_gate",
                gate_id=gate.id,
                note=f"skipped after {self.attempts} attempts (non-compliant)",
            )
            self.incomplete = True
            self.index += 1
            self.attempts = 0
            self.stuck = False
            self._target_side = 0
            if self.index >= len(self._sequence):
                return self._finish(now)
            return (self._gate_action(self._sequence[self.index]),)

        # Compliant path: go back and retry the same gate. The run simply cannot
        # be completed until this gate is passed, so the state machine keeps
        # asking for it and reports itself as stuck for the operator to see.
        self.stuck = self.attempts > self.retry_limit
        detail = "retry: outside opening"
        if self.stuck:
            detail += " (stuck: retrying until the gate is passed)"
        return (self._gate_action(gate, detail=detail),)

    def _finish(self, now: float) -> tuple[Action, ...]:
        self.state = State.LANDING
        if self.timer.running:
            self.timer.stop(now)
        self._note("course complete")
        landing = self.course.landing
        if landing is not None:
            return (
                Action(
                    kind=ActionKind.FLY_LANDING,
                    waypoints=(landing,),
                    detail="fly to landing point",
                ),
                Action(kind=ActionKind.LAND, detail="land"),
            )
        return (Action(kind=ActionKind.LAND, detail="land at current position"),)

    def on_landed(self, now: float) -> tuple[Action, ...]:
        if self.state is not State.LANDING:
            return ()
        if self.timer.running:
            self.timer.stop(now)
        self.state = State.DONE
        self._note("landed")
        return ()

    def abort(self, reason: str, now: float | None = None) -> tuple[Action, ...]:
        """Operator abort: cancel planning and land."""
        if self.state in (State.DONE, State.ABORTED, State.SAFE_STOP):
            return ()
        if self.timer.running and now is not None:
            self.timer.stop(now)
        self.state = State.ABORTED
        self.abort_reason = reason
        self.incomplete = True
        self._note(f"abort: {reason}")
        return (
            Action(kind=ActionKind.CANCEL_PLANNING, detail=reason),
            Action(kind=ActionKind.LAND, detail="abort landing"),
        )

    def safe_stop(self, reason: str, now: float | None = None) -> tuple[Action, ...]:
        """Hard stop: cancel everything and hold in place."""
        if self.state is State.SAFE_STOP:
            return ()
        if self.timer.running and now is not None:
            self.timer.stop(now)
        self.state = State.SAFE_STOP
        self.abort_reason = reason
        self.incomplete = True
        self._note(f"safe stop: {reason}")
        return (
            Action(kind=ActionKind.CANCEL_PLANNING, detail=reason),
            Action(kind=ActionKind.HOLD, detail="safety hold"),
        )

    # -- helpers ---------------------------------------------------------
    def _gate_action(self, gate: Gate, detail: str = "") -> Action:
        self._target_side = 0
        return Action(
            kind=ActionKind.FLY_GATE,
            gate_id=gate.id,
            waypoints=_lead_waypoints(gate, self.lead),
            heading=gate.yaw,
            speed=self.speed,
            detail=detail or f"gate {gate.id}",
        )

    def _note(self, message: str) -> None:
        self._history.append(f"[{self.state.value}] {message}")
        if len(self._history) > 200:
            del self._history[:100]

    @property
    def history(self) -> tuple[str, ...]:
        return tuple(self._history)

    def status(self, now: float | None = None) -> dict[str, Any]:
        """JSON-ready snapshot for the ground station."""
        gate = self.target_gate()
        return {
            "state": self.state.value,
            "course": self.course.name,
            "laps": self.course.laps,
            "laps_done": self.laps_done,
            "total_gates": self.total_gates,
            "gate_index": self.index,
            "target_gate_id": None if gate is None else gate.id,
            "attempts": self.attempts,
            "stuck": self.stuck,
            "skipping_allowed": self.allow_skip_with_penalty,
            "incomplete": self.incomplete,
            "abort_reason": self.abort_reason,
            "pose": None if self._current_pose is None else self._current_pose.to_dict(),
            "timer": self.timer.snapshot(now),
            "history": list(self._history[-12:]),
        }
