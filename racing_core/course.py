"""Course definition: an ordered list of gates the pilot must fly through.

A course is plain data (JSON) so it can be edited without touching code and so
both the simulator run and the real-flight run read the same file.

The shipped ``course/official_sample.json`` is derived from the waypoint set the
official LUS tmux session publishes (``tmux/one_drone_px4/session.yml``, the
"Race" pane). Those waypoints are gate centres, so the geometry is the
organiser's sample course, not something invented here.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable, Mapping

from .geometry import GateFrame, Vec3, gate_frame

__all__ = ["Gate", "Course", "CourseError", "load_course", "build_course"]

DEFAULT_GATE_SIZE = 1.0  # metres, rulebook section 3: 1 m x 1 m openings


class CourseError(ValueError):
    """Raised when a course file is structurally invalid."""


@dataclass(frozen=True, slots=True)
class Gate:
    """One gate: an opening of ``width`` x ``height`` centred on ``position``."""

    id: int
    position: Vec3
    yaw: float
    width: float = DEFAULT_GATE_SIZE
    height: float = DEFAULT_GATE_SIZE
    name: str = ""
    yaw_source: str = "estimated"

    def frame(self) -> GateFrame:
        return gate_frame(self.position, self.yaw, self.width, self.height)

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "name": self.name or f"gate{self.id}",
            "position": self.position.to_dict(),
            "yaw": self.yaw,
            "yaw_source": self.yaw_source,
            "width": self.width,
            "height": self.height,
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any], fallback_id: int) -> "Gate":
        if "position" not in data:
            raise CourseError(f"gate {data.get('id', fallback_id)} has no position")
        position = Vec3.from_any(data["position"])
        if not position.is_finite():
            raise CourseError(f"gate {data.get('id', fallback_id)} has a non-finite position")
        yaw = float(data.get("yaw", 0.0))
        if not math.isfinite(yaw):
            raise CourseError(f"gate {data.get('id', fallback_id)} has a non-finite yaw")
        return cls(
            id=int(data.get("id", fallback_id)),
            position=position,
            yaw=yaw,
            width=float(data.get("width", DEFAULT_GATE_SIZE)),
            height=float(data.get("height", DEFAULT_GATE_SIZE)),
            name=str(data.get("name", "")),
            yaw_source=str(data.get("yaw_source", "estimated")),
        )


@dataclass(frozen=True, slots=True)
class Course:
    """An ordered gate sequence flown ``laps`` times."""

    name: str
    gates: tuple[Gate, ...]
    laps: int = 1
    start: Vec3 = field(default_factory=Vec3)
    takeoff_height: float = 1.0
    landing: Vec3 | None = None
    source: str = ""

    # -- construction ----------------------------------------------------
    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "Course":
        raw_gates = data.get("gates")
        if not isinstance(raw_gates, list) or not raw_gates:
            raise CourseError("course needs a non-empty 'gates' list")
        gates = tuple(
            Gate.from_dict(item, fallback_id=index + 1)
            for index, item in enumerate(raw_gates)
        )
        landing_raw = data.get("landing")
        course = cls(
            name=str(data.get("name", "unnamed")),
            gates=gates,
            laps=int(data.get("laps", 1)),
            start=Vec3.from_any(data.get("start", [0.0, 0.0, 0.0])),
            takeoff_height=float(data.get("takeoff_height", 1.0)),
            landing=Vec3.from_any(landing_raw) if landing_raw is not None else None,
            source=str(data.get("source", "")),
        )
        problems = course.validate()
        if problems:
            raise CourseError("; ".join(problems))
        return course

    def to_dict(self) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "name": self.name,
            "laps": self.laps,
            "start": self.start.to_dict(),
            "takeoff_height": self.takeoff_height,
            "gates": [gate.to_dict() for gate in self.gates],
            "source": self.source,
        }
        if self.landing is not None:
            payload["landing"] = self.landing.to_dict()
        return payload

    # -- queries ---------------------------------------------------------
    def __len__(self) -> int:
        return len(self.gates)

    @property
    def gate_count(self) -> int:
        """Number of distinct gates in one lap."""
        return len(self.gates)

    def sequence(self) -> tuple[Gate, ...]:
        """Gate order for the whole run, repeated ``laps`` times."""
        return tuple(self.gates) * max(1, self.laps)

    def gate(self, gate_id: int) -> Gate:
        for candidate in self.gates:
            if candidate.id == gate_id:
                return candidate
        raise KeyError(f"no gate with id {gate_id}")

    def lap_length(self) -> float:
        """Approximate flown path length for one lap, in metres."""
        points = [self.start, *(gate.position for gate in self.gates)]
        return sum(
            (points[i + 1] - points[i]).norm() for i in range(len(points) - 1)
        )

    def validate(self) -> list[str]:
        """Return a list of human readable problems; empty means usable."""
        problems: list[str] = []
        if not self.gates:
            problems.append("course has no gates")
        if self.laps < 1:
            problems.append(f"laps must be >= 1, got {self.laps}")
        if self.takeoff_height <= 0.0:
            problems.append(f"takeoff_height must be positive, got {self.takeoff_height}")

        seen: dict[int, Gate] = {}
        for gate in self.gates:
            if gate.id in seen:
                problems.append(f"duplicate gate id {gate.id}")
            seen[gate.id] = gate
            if gate.width <= 0.0 or gate.height <= 0.0:
                problems.append(f"gate {gate.id} has a non-positive opening")
            if gate.position.z < 0.0:
                problems.append(f"gate {gate.id} is below ground (z={gate.position.z})")

        # Two gates closer than the opening width cannot be told apart, either by
        # a human looking at the map or by the detector.
        for i, first in enumerate(self.gates):
            for second in self.gates[i + 1 :]:
                gap = (first.position - second.position).norm()
                if gap < max(first.width, second.width):
                    problems.append(
                        f"gates {first.id} and {second.id} are {gap:.2f} m apart "
                        f"(closer than one gate width)"
                    )
        return problems


def load_course(path: str | Path) -> Course:
    """Load and validate a course from a JSON file."""
    resolved = Path(path)
    if not resolved.is_file():
        raise CourseError(f"course file not found: {resolved}")
    try:
        data = json.loads(resolved.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise CourseError(f"{resolved} is not valid JSON: {exc}") from exc
    if not isinstance(data, dict):
        raise CourseError(f"{resolved} must contain a JSON object")
    course = Course.from_dict(data)
    if not course.source:
        course = Course(
            name=course.name,
            gates=course.gates,
            laps=course.laps,
            start=course.start,
            takeoff_height=course.takeoff_height,
            landing=course.landing,
            source=str(resolved),
        )
    return course


def build_course(
    name: str,
    waypoints: Iterable[Vec3],
    *,
    laps: int = 1,
    start: Vec3 | None = None,
    takeoff_height: float = 1.0,
    width: float = DEFAULT_GATE_SIZE,
    height: float = DEFAULT_GATE_SIZE,
    source: str = "",
) -> Course:
    """Build a course from gate centres, estimating each gate's yaw.

    The yaw is the chord direction through the gate (previous point -> next
    point), which is the best guess available when only centres are known. The
    detector refines it at runtime; ``yaw_source`` records that this is an
    estimate.
    """
    points = [Vec3.from_any(point) for point in waypoints]
    if not points:
        raise CourseError("cannot build a course without waypoints")

    gates: list[Gate] = []
    for index, point in enumerate(points):
        previous = points[index - 1] if index > 0 else (start or point)
        following = points[index + 1] if index + 1 < len(points) else point
        chord = following - previous
        if chord.norm_xy() < 1e-6:
            chord = point - previous if (point - previous).norm_xy() > 1e-6 else Vec3(1.0, 0.0, 0.0)
        yaw = math.atan2(chord.y, chord.x)
        gates.append(
            Gate(
                id=index + 1,
                position=point,
                yaw=yaw,
                width=width,
                height=height,
                name=f"gate{index + 1}",
                yaw_source="estimated",
            )
        )

    return Course(
        name=name,
        gates=tuple(gates),
        laps=laps,
        start=start or Vec3(points[0].x, points[0].y, takeoff_height),
        takeoff_height=takeoff_height,
        source=source,
    )
