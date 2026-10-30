"""Vector and gate-frame geometry for the ICUAS 2027 racing task.

Pure standard library. No numpy, no ROS: this module is meant to be unit
tested on any machine that has Python 3.10+, including the Windows dual boot.

Frame conventions
-----------------
World frame is the simulator/odometry frame (ENU, metres, Z up).

A gate is a rectangle of ``width`` x ``height`` centred on ``position``. Its
yaw is the direction the pilot should fly through the gate, expressed as the
angle between the world X axis and the gate normal::

    normal  n = (cos yaw, sin yaw, 0)      direction of travel through the gate
    lateral l = (-sin yaw, cos yaw, 0)     gate "right" axis in the gate plane
    up      u = (0, 0, 1)

A point's gate-frame coordinates are ``(along, lateral, vertical)`` where
``along`` grows in the direction of travel. A crossing is therefore a sign
change of ``along`` between two consecutive poses.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Mapping, Sequence

__all__ = [
    "Vec3",
    "GateFrame",
    "CrossingResult",
    "gate_frame",
    "crossing",
    "side_of",
    "closest_point_on_segment",
]


@dataclass(frozen=True, slots=True)
class Vec3:
    """Immutable 3D vector with the handful of operations this project needs."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    # -- construction ----------------------------------------------------
    @classmethod
    def from_any(cls, value: "Vec3 | Sequence[float] | Mapping[str, float]") -> "Vec3":
        if isinstance(value, Vec3):
            return value
        if isinstance(value, Mapping):
            try:
                return cls(float(value["x"]), float(value["y"]), float(value.get("z", 0.0)))
            except KeyError as exc:
                raise ValueError(f"mapping needs x/y (and optional z): {exc}") from exc
        items = list(value)
        if len(items) == 2:
            return cls(float(items[0]), float(items[1]), 0.0)
        if len(items) != 3:
            raise ValueError(f"expected 2 or 3 components, got {len(items)}")
        return cls(float(items[0]), float(items[1]), float(items[2]))

    def to_list(self) -> list[float]:
        return [self.x, self.y, self.z]

    def to_dict(self) -> dict[str, float]:
        return {"x": self.x, "y": self.y, "z": self.z}

    # -- arithmetic ------------------------------------------------------
    def __add__(self, other: "Vec3") -> "Vec3":
        return Vec3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: "Vec3") -> "Vec3":
        return Vec3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, scalar: float) -> "Vec3":
        return Vec3(self.x * scalar, self.y * scalar, self.z * scalar)

    __rmul__ = __mul__

    def __truediv__(self, scalar: float) -> "Vec3":
        if scalar == 0.0:
            raise ZeroDivisionError("Vec3 division by zero")
        return Vec3(self.x / scalar, self.y / scalar, self.z / scalar)

    def __neg__(self) -> "Vec3":
        return Vec3(-self.x, -self.y, -self.z)

    def __iter__(self):
        yield self.x
        yield self.y
        yield self.z

    # -- products and norms ---------------------------------------------
    def dot(self, other: "Vec3") -> float:
        return self.x * other.x + self.y * other.y + self.z * other.z

    def cross(self, other: "Vec3") -> "Vec3":
        return Vec3(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x,
        )

    def norm(self) -> float:
        return math.sqrt(self.dot(self))

    def norm_xy(self) -> float:
        return math.hypot(self.x, self.y)

    def normalized(self) -> "Vec3":
        length = self.norm()
        if length == 0.0:
            raise ValueError("cannot normalize a zero vector")
        return self / length

    def with_z(self, z: float) -> "Vec3":
        return Vec3(self.x, self.y, z)

    def horizontal_distance(self, other: "Vec3") -> float:
        return math.hypot(self.x - other.x, self.y - other.y)

    def is_finite(self) -> bool:
        return all(math.isfinite(v) for v in (self.x, self.y, self.z))


@dataclass(frozen=True, slots=True)
class GateFrame:
    """World-frame axes of a gate, precomputed once per gate."""

    origin: Vec3
    normal: Vec3
    lateral: Vec3
    up: Vec3
    width: float
    height: float

    def to_local(self, point: Vec3) -> tuple[float, float, float]:
        """Return ``(along, lateral, vertical)`` of ``point`` in this frame."""
        delta = point - self.origin
        return (delta.dot(self.normal), delta.dot(self.lateral), delta.dot(self.up))

    def contains(self, lateral: float, vertical: float, margin: float = 0.0) -> bool:
        """True when a gate-plane point falls inside the opening."""
        limit_w = self.width / 2.0 + margin
        limit_h = self.height / 2.0 + margin
        return abs(lateral) <= limit_w and abs(vertical) <= limit_h


def gate_frame(
    position: Vec3,
    yaw: float,
    width: float = 1.0,
    height: float = 1.0,
) -> GateFrame:
    """Build the gate frame for a gate at ``position`` pointing along ``yaw``."""
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    normal = Vec3(cos_yaw, sin_yaw, 0.0)
    lateral = Vec3(-sin_yaw, cos_yaw, 0.0)
    return GateFrame(
        origin=position,
        normal=normal,
        lateral=lateral,
        up=Vec3(0.0, 0.0, 1.0),
        width=width,
        height=height,
    )


@dataclass(frozen=True, slots=True)
class CrossingResult:
    """Outcome of testing one motion segment against one gate."""

    crossed: bool
    inside: bool
    direction: int
    point: Vec3
    lateral: float
    vertical: float
    fraction: float

    @property
    def passed_forward(self) -> bool:
        """A valid pass in the intended direction of travel."""
        return self.crossed and self.direction > 0

    @property
    def valid_pass(self) -> bool:
        return self.crossed and self.inside and self.direction > 0

    @property
    def distance_from_centre(self) -> float:
        """In-plane distance between the crossing point and the gate centre."""
        if not self.crossed:
            return math.inf
        return math.hypot(self.lateral, self.vertical)

    def to_dict(self) -> dict[str, object]:
        return {
            "crossed": self.crossed,
            "inside": self.inside,
            "direction": self.direction,
            "point": self.point.to_dict(),
            "lateral": self.lateral,
            "vertical": self.vertical,
            "valid_pass": self.valid_pass,
            "distance_from_centre": None
            if self.distance_from_centre == math.inf
            else self.distance_from_centre,
        }


_NO_CROSSING = CrossingResult(
    crossed=False,
    inside=False,
    direction=0,
    point=Vec3(),
    lateral=math.inf,
    vertical=math.inf,
    fraction=-1.0,
)


def side_of(gate: GateFrame, point: Vec3, tolerance: float = 1e-9) -> int:
    """Which side of the gate plane a point is on: ``-1``, ``+1`` or ``0`` (on it)."""
    along, _, _ = gate.to_local(point)
    if along > tolerance:
        return 1
    if along < -tolerance:
        return -1
    return 0


def crossing(
    gate: GateFrame,
    start: Vec3,
    end: Vec3,
    tolerance: float = 1e-9,
    start_side: int = 0,
) -> CrossingResult:
    """Test whether the segment ``start -> end`` crosses the gate plane.

    ``tolerance`` is the dead band for "on the plane". A sample can land exactly
    on the plane -- the waypoint we steer to *is* the gate centre, so this is the
    normal case, not an edge case. Such a sample has no side of its own, so the
    caller passes ``start_side`` (the last side it observed) and the crossing is
    attributed to the moment the vehicle touched the plane instead of being lost.

    Without ``start_side`` a pass whose sample lands on the plane would be
    silently dropped, and the *next* plane crossing -- possibly metres away --
    would be recorded as a missed gate.
    """
    along_start, _, _ = gate.to_local(start)
    along_end, _, _ = gate.to_local(end)

    side_start = side_of(gate, start, tolerance) or int(start_side)
    side_end = side_of(gate, end, tolerance)

    # Unknown or zero-length in the normal direction: nothing to decide.
    if side_start == 0 or side_end == 0 or side_start == side_end:
        return _NO_CROSSING
    if abs(along_start - along_end) <= tolerance:
        return _NO_CROSSING

    fraction = along_start / (along_start - along_end)
    # A start sitting on the plane gives fraction 0 (the crossing is *here*).
    fraction = min(1.0, max(0.0, fraction))

    point = start + (end - start) * fraction
    lateral, vertical = _plane_offset(gate, point)
    # Use the remembered side, not the raw coordinate: a start on the plane has
    # along_start == 0 and would otherwise read as a backwards crossing.
    direction = 1 if side_start < 0 else -1

    return CrossingResult(
        crossed=True,
        inside=gate.contains(lateral, vertical),
        direction=direction,
        point=point,
        lateral=lateral,
        vertical=vertical,
        fraction=fraction,
    )


def _plane_offset(gate: GateFrame, point: Vec3) -> tuple[float, float]:
    delta = point - gate.origin
    return (delta.dot(gate.lateral), delta.dot(gate.up))


def closest_point_on_segment(point: Vec3, start: Vec3, end: Vec3) -> tuple[Vec3, float]:
    """Return the closest point on ``start -> end`` to ``point`` and its parameter."""
    segment = end - start
    length_sq = segment.dot(segment)
    if length_sq == 0.0:
        return start, 0.0
    t = max(0.0, min(1.0, (point - start).dot(segment) / length_sq))
    return start + segment * t, t


def polyline_length(points: Iterable[Vec3]) -> float:
    """Total length of a polyline; used to sanity check a course."""
    total = 0.0
    previous: Vec3 | None = None
    for point in points:
        if previous is not None:
            total += (point - previous).norm()
        previous = point
    return total
