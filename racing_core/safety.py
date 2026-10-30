"""Pre-flight and in-flight safety gates.

Two ideas are lifted from the team repositories and adapted:

* **Stale-data budgets that depend on the platform** (Xie Ning, ``freshness.py``).
  A SITL heartbeat reaches the wall clock at the simulation's real-time factor,
  so the same one-second budget that protects real hardware would fire
  constantly in simulation. The budget is an explicit platform choice, never an
  automatic switch.
* **A single verdict object with reason codes** (Max, ``safety_gates.py``). The
  caller does not get booleans sprinkled around; it gets one :class:`Verdict`
  whose ``level`` decides whether to refuse takeoff, warn, or abort.

The monitor is pure: feed it timestamps and poses, get a verdict. It never
imports ROS and never raises for ordinary flight data.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Any

from .geometry import Vec3

__all__ = [
    "Level",
    "SafetyLimits",
    "SafetyMonitor",
    "Verdict",
    "PLATFORM_ENV",
]

PLATFORM_ENV = "ICUAS27_PLATFORM"


class Level(str, Enum):
    OK = "OK"
    WARN = "WARN"
    ABORT = "ABORT"


# Reason codes are stable strings so the ground station and the logs can match on
# them without depending on message wording.
POSE_STALE = "POSE_STALE"
POSE_INVALID = "POSE_INVALID"
FCU_STALE = "FCU_STALE"
OUT_OF_BOUNDS = "OUT_OF_BOUNDS"
ALTITUDE_LOW = "ALTITUDE_LOW"
ALTITUDE_HIGH = "ALTITUDE_HIGH"


@dataclass(frozen=True, slots=True)
class SafetyLimits:
    """Everything the monitor needs to judge a sample."""

    min_altitude: float = 0.15
    max_altitude: float = 12.0
    # (x_min, x_max, y_min, y_max)
    bounds_xy: tuple[float, float, float, float] = (-12.0, 12.0, -12.0, 12.0)
    pose_timeout_sec_sim: float = 1.0
    pose_timeout_sec_real: float = 0.3
    fcu_timeout_sec_sim: float = 4.0
    fcu_timeout_sec_real: float = 1.0

    def pose_timeout(self, platform: str) -> float:
        return (
            self.pose_timeout_sec_sim
            if _normalise_platform(platform) == "sim"
            else self.pose_timeout_sec_real
        )

    def fcu_timeout(self, platform: str) -> float:
        return (
            self.fcu_timeout_sec_sim
            if _normalise_platform(platform) == "sim"
            else self.fcu_timeout_sec_real
        )

    def inside_bounds(self, position: Vec3) -> bool:
        x_min, x_max, y_min, y_max = self.bounds_xy
        return x_min <= position.x <= x_max and y_min <= position.y <= y_max

    def to_dict(self) -> dict[str, Any]:
        return {
            "min_altitude": self.min_altitude,
            "max_altitude": self.max_altitude,
            "bounds_xy": list(self.bounds_xy),
            "pose_timeout_sec_sim": self.pose_timeout_sec_sim,
            "pose_timeout_sec_real": self.pose_timeout_sec_real,
            "fcu_timeout_sec_sim": self.fcu_timeout_sec_sim,
            "fcu_timeout_sec_real": self.fcu_timeout_sec_real,
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "SafetyLimits":
        bounds = data.get("bounds_xy")
        kwargs: dict[str, Any] = {}
        for name in (
            "min_altitude",
            "max_altitude",
            "pose_timeout_sec_sim",
            "pose_timeout_sec_real",
            "fcu_timeout_sec_sim",
            "fcu_timeout_sec_real",
        ):
            if name in data:
                kwargs[name] = float(data[name])
        if bounds is not None:
            if len(bounds) != 4:
                raise ValueError("bounds_xy needs four numbers")
            kwargs["bounds_xy"] = tuple(float(v) for v in bounds)
        return cls(**kwargs)


@dataclass(frozen=True, slots=True)
class Verdict:
    """The single answer the flight layer acts on."""

    level: Level
    reasons: tuple[str, ...] = ()
    detail: str = ""
    pose_age: float | None = None
    fcu_age: float | None = None

    @property
    def ok(self) -> bool:
        return self.level is Level.OK

    @property
    def must_abort(self) -> bool:
        return self.level is Level.ABORT

    def to_dict(self) -> dict[str, Any]:
        return {
            "level": self.level.value,
            "reasons": list(self.reasons),
            "detail": self.detail,
            "pose_age": self.pose_age,
            "fcu_age": self.fcu_age,
        }


def _normalise_platform(platform: str | None) -> str:
    if platform is None:
        return "real"
    return "sim" if str(platform).strip().lower() == "sim" else "real"


@dataclass
class SafetyMonitor:
    """Judge the newest samples; remember the last verdict for the dashboard."""

    limits: SafetyLimits = field(default_factory=SafetyLimits)
    platform: str = "real"
    last: Verdict = field(default=Verdict(Level.OK), init=False)

    def __post_init__(self) -> None:
        self.platform = _normalise_platform(self.platform)

    @property
    def pose_timeout(self) -> float:
        return self.limits.pose_timeout(self.platform)

    @property
    def fcu_timeout(self) -> float:
        return self.limits.fcu_timeout(self.platform)

    def check(
        self,
        now: float,
        *,
        pose_at: float | None,
        pose: Vec3 | None,
        fcu_at: float | None = None,
        require_fcu: bool = False,
        airborne: bool = False,
    ) -> Verdict:
        """Judge the current samples.

        ``require_fcu`` is set once the hardware API is expected to talk to us;
        a missing heartbeat is fatal in that case and merely unknown before it.
        ``airborne`` turns a stale pose into an abort (you cannot hold a position
        you do not know); on the ground it is a warning.
        """
        reasons: list[str] = []
        level = Level.OK
        details: list[str] = []

        pose_age = None if pose_at is None else max(0.0, now - pose_at)
        fcu_age = None if fcu_at is None else max(0.0, now - fcu_at)

        if pose is None or pose_at is None:
            reasons.append(POSE_STALE)
            details.append("no pose received yet")
            level = self._raise(level, Level.ABORT if airborne else Level.WARN)
        else:
            if pose_age is not None and pose_age > self.pose_timeout:
                reasons.append(POSE_STALE)
                details.append(f"pose is {pose_age:.2f} s old")
                level = self._raise(level, Level.ABORT if airborne else Level.WARN)
            if not pose.is_finite():
                reasons.append(POSE_INVALID)
                details.append("pose contains a non-finite component")
                level = self._raise(level, Level.ABORT)
            else:
                # A stale pose still has to be a *plausible* pose: an old sample
                # outside the arena is just as wrong as a fresh one.
                if not self.limits.inside_bounds(pose):
                    reasons.append(OUT_OF_BOUNDS)
                    details.append(
                        f"position {pose.to_dict()} is outside {self.limits.bounds_xy}"
                    )
                    level = self._raise(level, Level.ABORT)
                if pose.z < self.limits.min_altitude:
                    reasons.append(ALTITUDE_LOW)
                    details.append(f"altitude {pose.z:.2f} m below minimum")
                    level = self._raise(level, Level.WARN)
                elif pose.z > self.limits.max_altitude:
                    reasons.append(ALTITUDE_HIGH)
                    details.append(f"altitude {pose.z:.2f} m above maximum")
                    level = self._raise(level, Level.ABORT)

        if require_fcu:
            if fcu_at is None:
                reasons.append(FCU_STALE)
                details.append("no flight-controller state received")
                level = self._raise(level, Level.ABORT if airborne else Level.WARN)
            elif fcu_age is not None and fcu_age > self.fcu_timeout:
                reasons.append(FCU_STALE)
                details.append(f"flight controller state is {fcu_age:.2f} s old")
                level = self._raise(level, Level.ABORT)

        verdict = Verdict(
            level=level,
            reasons=tuple(dict.fromkeys(reasons)),
            detail="; ".join(details),
            pose_age=pose_age,
            fcu_age=fcu_age,
        )
        self.last = verdict
        return verdict

    @staticmethod
    def _raise(current: Level, candidate: Level) -> Level:
        order = {Level.OK: 0, Level.WARN: 1, Level.ABORT: 2}
        return candidate if order[candidate] > order[current] else current

    def to_dict(self) -> dict[str, Any]:
        return {
            "platform": self.platform,
            "limits": self.limits.to_dict(),
            "last": self.last.to_dict(),
        }


def distance_ok(position: Vec3, target: Vec3, tolerance: float) -> bool:
    """Convenience predicate used by the arrival checks in the ROS layer."""
    if not math.isfinite(tolerance) or tolerance < 0.0:
        return False
    return (position - target).norm() <= tolerance
