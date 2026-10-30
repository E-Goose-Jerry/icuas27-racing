"""Race timing and penalty accounting.

Rulebook section 10::

    T_adj = T_elapsed + sum(n_j * dt_j),   dt = 5 s

Every incomplete run or collision counts once. The timer keeps the elapsed time
and the penalty ledger apart so the ground station can show both, and so a
penalty can be attributed to the gate where it happened.

All methods take the current time as a parameter (``now``) instead of reading a
clock internally: the ROS node passes ``node.get_clock().now()``, tests pass
plain floats. Nothing here needs wall-clock access.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

__all__ = [
    "PENALTY_SECONDS",
    "Penalty",
    "RaceTimer",
    "format_duration",
]

PENALTY_SECONDS = 5.0


def format_duration(seconds: float | None) -> str:
    """Format a duration as ``M:SS.mmm`` (or ``-`` when unknown)."""
    if seconds is None:
        return "-"
    sign = "-" if seconds < 0 else ""
    value = abs(float(seconds))
    minutes, rest = divmod(value, 60.0)
    return f"{sign}{int(minutes)}:{rest:06.3f}"


@dataclass(frozen=True, slots=True)
class Penalty:
    """One penalty entry in the ledger."""

    reason: str
    seconds: float = PENALTY_SECONDS
    gate_id: int | None = None
    note: str = ""

    def to_dict(self) -> dict[str, Any]:
        return {
            "reason": self.reason,
            "seconds": self.seconds,
            "gate_id": self.gate_id,
            "note": self.note,
        }


@dataclass
class RaceTimer:
    """Elapsed time plus a penalty ledger.

    The clock is an opaque float (seconds). The simulation can run at a
    different rate than real time, so the caller decides which clock to hand in.
    """

    penalty_seconds: float = PENALTY_SECONDS
    started_at: float | None = None
    stopped_at: float | None = None
    penalties: list[Penalty] = field(default_factory=list)
    gate_times: list[tuple[int, float]] = field(default_factory=list)

    # -- lifecycle -------------------------------------------------------
    @property
    def running(self) -> bool:
        return self.started_at is not None and self.stopped_at is None

    def start(self, now: float) -> None:
        if self.started_at is not None:
            raise RuntimeError("timer already started")
        self.started_at = float(now)
        self.stopped_at = None
        self.penalties.clear()
        self.gate_times.clear()

    def stop(self, now: float) -> None:
        if self.started_at is None:
            raise RuntimeError("cannot stop a timer that never started")
        if self.stopped_at is None:
            self.stopped_at = max(float(now), self.started_at)

    def reset(self) -> None:
        self.started_at = None
        self.stopped_at = None
        self.penalties.clear()
        self.gate_times.clear()

    # -- measurements ----------------------------------------------------
    def mark_gate(self, gate_id: int, now: float) -> None:
        """Record the split time at which a gate was passed."""
        self.gate_times.append((int(gate_id), self.elapsed(now) or 0.0))

    def add_penalty(
        self,
        reason: str,
        *,
        gate_id: int | None = None,
        seconds: float | None = None,
        note: str = "",
    ) -> Penalty:
        if seconds is not None and seconds < 0.0:
            raise ValueError("penalty seconds must not be negative")
        penalty = Penalty(
            reason=reason,
            seconds=self.penalty_seconds if seconds is None else float(seconds),
            gate_id=gate_id,
            note=note,
        )
        self.penalties.append(penalty)
        return penalty

    def elapsed(self, now: float | None = None) -> float | None:
        """Raw elapsed time; ``None`` before the first start."""
        if self.started_at is None:
            return None
        end = self.stopped_at if self.stopped_at is not None else now
        if end is None:
            return None
        return max(0.0, float(end) - self.started_at)

    @property
    def penalty_total(self) -> float:
        return float(sum(penalty.seconds for penalty in self.penalties))

    def adjusted(self, now: float | None = None) -> float | None:
        """``T_adj`` as defined by the rulebook."""
        elapsed = self.elapsed(now)
        if elapsed is None:
            return None
        return elapsed + self.penalty_total

    # -- reporting -------------------------------------------------------
    def snapshot(self, now: float | None = None) -> dict[str, Any]:
        elapsed = self.elapsed(now)
        adjusted = self.adjusted(now)
        return {
            "running": self.running,
            "elapsed": elapsed,
            "elapsed_text": format_duration(elapsed),
            "penalty_total": self.penalty_total,
            "penalty_count": len(self.penalties),
            "penalties": [penalty.to_dict() for penalty in self.penalties],
            "adjusted": adjusted,
            "adjusted_text": format_duration(adjusted),
            "splits": [
                {"gate_id": gate_id, "at": at, "at_text": format_duration(at)}
                for gate_id, at in self.gate_times
            ],
        }
