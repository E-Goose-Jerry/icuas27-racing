"""Ground-station state holder.

Reads the JSON contract the race node writes and hands it to the HTTP layer.
Kept separate from the server so it can be tested without opening a socket.

The file is the transport on purpose: it survives a node restart, it can be
``tail``-ed, it works when ROS and the dashboard run in different containers
(shared volume), and it makes mock playback trivial.
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from racing_core.protocol import read_document, validate_document

__all__ = ["DEFAULT_STATUS_FILE", "StatusSource", "StaleDocument"]

DEFAULT_STATUS_FILE = "/tmp/icuas27_race/status.json"


@dataclass(frozen=True, slots=True)
class StaleDocument:
    """Wrapper that carries how old the served document is."""

    document: dict[str, Any]
    age_sec: float
    source: str

    @property
    def stale(self) -> bool:
        return self.age_sec > 3.0

    def to_dict(self) -> dict[str, Any]:
        payload = dict(self.document)
        payload["_meta"] = {
            "age_sec": round(self.age_sec, 3),
            "stale": self.stale,
            "source": self.source,
        }
        return payload


@dataclass
class StatusSource:
    """Latest document, with mtime-based caching so polling is cheap."""

    path: str | Path = DEFAULT_STATUS_FILE
    _cached: dict[str, Any] | None = field(default=None, init=False, repr=False)
    _cached_mtime: float = field(default=-1.0, init=False, repr=False)
    _problems: list[str] = field(default_factory=list, init=False, repr=False)

    def __post_init__(self) -> None:
        self.path = Path(self.path)

    # -- reads -----------------------------------------------------------
    def snapshot(self, now: float | None = None) -> StaleDocument | None:
        """Return the newest document, or ``None`` when there is nothing usable."""
        now = time.time() if now is None else now
        try:
            mtime = self.path.stat().st_mtime
        except OSError:
            return None

        if self._cached is None or mtime != self._cached_mtime:
            document = read_document(self.path)
            if document is None:
                return None
            self._problems = validate_document(document)
            self._cached = document
            self._cached_mtime = mtime

        if self._cached is None:
            return None
        # The age is measured against the file itself, so it stays meaningful
        # even when nobody polls for a while.
        return StaleDocument(
            document=self._cached,
            age_sec=max(0.0, now - mtime),
            source=str(self.path),
        )

    @property
    def problems(self) -> list[str]:
        """Contract violations in the last document read (empty when clean)."""
        return list(self._problems)

    def course(self) -> dict[str, Any] | None:
        snapshot = self.snapshot()
        if snapshot is None:
            return None
        course = snapshot.document.get("course")
        return dict(course) if isinstance(course, dict) else None

    def health(self) -> dict[str, Any]:
        snapshot = self.snapshot()
        return {
            "ok": snapshot is not None and not self.problems,
            "path": str(self.path),
            "exists": self.path.is_file(),
            "problems": self.problems,
            "age_sec": None if snapshot is None else round(snapshot.age_sec, 3),
            "schema": None if snapshot is None else snapshot.document.get("schema"),
        }


def load_fixture(path: str | Path) -> dict[str, Any] | None:
    """Load a recorded document for mock mode."""
    try:
        raw = Path(path).read_text(encoding="utf-8")
    except OSError:
        return None
    try:
        data = json.loads(raw)
    except json.JSONDecodeError:
        return None
    return data if isinstance(data, dict) else None
