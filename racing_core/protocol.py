"""The ground-station contract.

One versioned JSON document describes everything the dashboard shows. The ROS
node writes it to stdout and to a file; the FastAPI app serves it. Keeping the
contract in one place (instead of letting each side invent keys) is what makes
the mock mode possible: a recorded document can be replayed without ROS.

Layout is deliberately flat and boring::

    {
      "schema": "icuas27.race.v1",
      "seq": 42,
      "stamp": 1730000000.123,
      "race": {...mission_fsm.status()...},
      "safety": {...safety.SafetyMonitor.to_dict()...},
      "course": {...course.to_dict()...},
      "gates": [{"id": 1, "passed": true, "target": false}, ...],
      "detection": {"visible": 2, "last_gate_id": 3, "source": "color"},
      "notes": ["free-form strings"]
    }
"""

from __future__ import annotations

import json
import os
import tempfile
from pathlib import Path
from typing import Any, Iterable, Mapping

from .course import Course

__all__ = [
    "SCHEMA",
    "build_document",
    "validate_document",
    "write_document",
    "read_document",
]

SCHEMA = "icuas27.race.v1"

_REQUIRED_TOP_LEVEL = ("schema", "seq", "stamp", "race", "safety", "course", "gates")
_REQUIRED_RACE = ("state", "gate_index", "total_gates", "timer")
_REQUIRED_TIMER = ("elapsed", "penalty_total", "adjusted")


def build_document(
    *,
    seq: int,
    stamp: float,
    race: Mapping[str, Any],
    safety: Mapping[str, Any],
    course: Course,
    gates: Iterable[Mapping[str, Any]] = (),
    detection: Mapping[str, Any] | None = None,
    notes: Iterable[str] = (),
) -> dict[str, Any]:
    """Assemble a contract document. Values are copied, not referenced."""
    return {
        "schema": SCHEMA,
        "seq": int(seq),
        "stamp": float(stamp),
        "race": dict(race),
        "safety": dict(safety),
        "course": {
            "name": course.name,
            "laps": course.laps,
            "gate_count": course.gate_count,
            "laps_done": race.get("laps_done", 0),
            "gates": [gate.to_dict() for gate in course.gates],
        },
        "gates": [dict(item) for item in gates],
        "detection": dict(detection) if detection is not None else {},
        "notes": [str(note) for note in notes],
    }


def validate_document(document: Mapping[str, Any]) -> list[str]:
    """Return contract violations; empty means the dashboard can render it."""
    problems: list[str] = []
    if not isinstance(document, Mapping):
        return ["document must be an object"]

    for key in _REQUIRED_TOP_LEVEL:
        if key not in document:
            problems.append(f"missing top-level key: {key}")
    if problems:
        return problems

    if document["schema"] != SCHEMA:
        problems.append(f"schema is {document['schema']!r}, expected {SCHEMA!r}")

    stamp = document["stamp"]
    if not isinstance(stamp, (int, float)) or stamp != stamp:  # NaN check
        problems.append("stamp must be a finite number")
    if not isinstance(document["seq"], int):
        problems.append("seq must be an integer")

    race = document["race"]
    if not isinstance(race, Mapping):
        problems.append("race must be an object")
    else:
        for key in _REQUIRED_RACE:
            if key not in race:
                problems.append(f"missing race key: {key}")
        timer = race.get("timer")
        if isinstance(timer, Mapping):
            for key in _REQUIRED_TIMER:
                if key not in timer:
                    problems.append(f"missing timer key: {key}")
            elapsed = timer.get("elapsed")
            if elapsed is not None and not isinstance(elapsed, (int, float)):
                problems.append("timer.elapsed must be a number or null")
        elif "timer" in race:
            problems.append("race.timer must be an object")

    course = document["course"]
    if not isinstance(course, Mapping):
        problems.append("course must be an object")
    elif not isinstance(course.get("gates"), list):
        problems.append("course.gates must be a list")

    gates = document["gates"]
    if not isinstance(gates, list):
        problems.append("gates must be a list")
    else:
        for index, gate in enumerate(gates):
            if not isinstance(gate, Mapping) or "id" not in gate:
                problems.append(f"gates[{index}] must be an object with an id")

    return problems


def write_document(path: str | Path, document: Mapping[str, Any]) -> None:
    """Write the document atomically so a reader never sees half a file."""
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    payload = json.dumps(document, ensure_ascii=False, indent=2, sort_keys=False)
    handle = tempfile.NamedTemporaryFile(
        "w",
        encoding="utf-8",
        dir=str(target.parent),
        prefix=target.name,
        suffix=".tmp",
        delete=False,
    )
    try:
        with handle as stream:
            stream.write(payload)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(handle.name, target)
    except BaseException:
        with_path = handle.name
        try:
            os.unlink(with_path)
        except OSError:
            pass
        raise


def read_document(path: str | Path) -> dict[str, Any] | None:
    """Read a document, returning ``None`` when it is absent or unreadable."""
    try:
        raw = Path(path).read_text(encoding="utf-8")
    except (OSError, UnicodeDecodeError):
        return None
    try:
        data = json.loads(raw)
    except json.JSONDecodeError:
        return None
    return data if isinstance(data, dict) else None
