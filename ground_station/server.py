"""Read-only ground station: one HTTP server, zero third-party dependencies.

Why stdlib: the dashboard has to run inside the competition container next to
ROS 2 Humble, on the Windows side while debugging, and on a phone hotspot at the
field. A single file with ``http.server`` does all three; FastAPI would need a
wheel, a venv and a compatible ``pydantic`` in every one of those places.

Endpoints
---------
``GET /``              the dashboard (static HTML, no build step)
``GET /api/status``    newest contract document, plus ``_meta`` age/staleness
``GET /api/course``    just the course, for map redraws
``GET /api/health``    is the status file there, is the schema valid
``GET /api/events``    Server-Sent Events stream of new documents

The dashboard is deliberately read-only. Commands go through the race node's
service, and the page shows the exact ``ros2 service call`` line to copy, so a
browser tab can never be the thing that arms a drone.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

try:
    from racing_core.protocol import SCHEMA
except ImportError:  # pragma: no cover - guidance for a bare checkout
    sys.exit(
        "racing_core is not importable. Run from the project root "
        "(python3 -m ground_station.server) or export PYTHONPATH=<project root>."
    )

from .mock import MockRace
from .state import DEFAULT_STATUS_FILE, StatusSource

STATIC_DIR = Path(__file__).resolve().parent / "static"
_SSE_POLL_SEC = 0.2
_SSE_KEEPALIVE_SEC = 10.0


class DashboardState:
    """Everything the HTTP handlers need, in one object."""

    def __init__(
        self,
        status_file: str | Path = DEFAULT_STATUS_FILE,
        mock: bool = False,
        course_file: str | Path | None = None,
    ) -> None:
        self.source = StatusSource(status_file)
        self.mock = mock
        self.race: MockRace | None = None
        self._mock_course = course_file
        self._last_tick = 0.0
        # Mock playback advances on read, so concurrent dashboard tabs must not
        # each push the synthetic race forward.
        self._mock_lock = threading.Lock()

    # -- documents -------------------------------------------------------
    def document(self) -> dict[str, Any] | None:
        if self.mock:
            return self._mock_document()
        snapshot = self.source.snapshot()
        return None if snapshot is None else snapshot.to_dict()

    def _mock_document(self) -> dict[str, Any] | None:
        with self._mock_lock:
            return self._mock_document_locked()

    def _mock_document_locked(self) -> dict[str, Any] | None:
        if self.race is None:
            if self._mock_course is None:
                return None
            self.race = MockRace(course_file=str(self._mock_course))
            self._last_tick = time.time()
            return self.race.tick(0.5)
        now = time.time()
        elapsed = max(0.0, now - self._last_tick)
        self._last_tick = now
        return self.race.tick(min(elapsed, 1.0))

    def course(self) -> dict[str, Any] | None:
        document = self.document()
        if document is None:
            return None
        course = document.get("course")
        return dict(course) if isinstance(course, dict) else None

    def health(self) -> dict[str, Any]:
        if self.mock:
            document = self.document()
            return {
                "ok": document is not None,
                "mode": "mock",
                "schema": SCHEMA,
                "problems": [],
            }
        payload = self.source.health()
        payload["mode"] = "file"
        return payload


def _json_bytes(payload: Any) -> bytes:
    return json.dumps(payload, ensure_ascii=False).encode("utf-8")


def make_handler(state: DashboardState):
    """Build a request handler bound to ``state``."""

    class Handler(BaseHTTPRequestHandler):
        server_version = "ICUAS27GroundStation/0.1"
        sys_version = ""

        # -- plumbing ---------------------------------------------------
        def log_message(self, fmt: str, *args: Any) -> None:  # noqa: D102 - quieter logs
            if os.environ.get("ICUAS27_GS_VERBOSE"):
                super().log_message(fmt, *args)

        def _send(self, status: HTTPStatus, body: bytes, content_type: str) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _send_json(self, status: HTTPStatus, payload: Any) -> None:
            self._send(status, _json_bytes(payload), "application/json; charset=utf-8")

        # -- routes -----------------------------------------------------
        def do_GET(self) -> None:  # noqa: N802 - stdlib naming
            path = urlparse(self.path).path
            if path in ("/", "/index.html"):
                return self._serve_index()
            if path == "/api/status":
                document = state.document()
                if document is None:
                    return self._send_json(
                        HTTPStatus.SERVICE_UNAVAILABLE,
                        {"error": "no status document yet", "schema": SCHEMA},
                    )
                return self._send_json(HTTPStatus.OK, document)
            if path == "/api/course":
                course = state.course()
                if course is None:
                    return self._send_json(
                        HTTPStatus.SERVICE_UNAVAILABLE, {"error": "no course yet"}
                    )
                return self._send_json(HTTPStatus.OK, course)
            if path == "/api/health":
                return self._send_json(HTTPStatus.OK, state.health())
            if path == "/api/events":
                return self._serve_events()
            if path == "/favicon.ico":
                return self._send(HTTPStatus.NO_CONTENT, b"", "image/x-icon")
            return self._send_json(HTTPStatus.NOT_FOUND, {"error": "not found"})

        # -- handlers ---------------------------------------------------
        def _serve_index(self) -> None:
            index = STATIC_DIR / "index.html"
            if not index.is_file():
                return self._send(
                    HTTPStatus.INTERNAL_SERVER_ERROR,
                    b"static/index.html is missing",
                    "text/plain; charset=utf-8",
                )
            self._send(HTTPStatus.OK, index.read_bytes(), "text/html; charset=utf-8")

        def _serve_events(self) -> None:
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "text/event-stream; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "keep-alive")
            self.end_headers()

            last_seq = -1
            last_keepalive = time.time()
            try:
                while True:
                    document = state.document()
                    if document is not None and document.get("seq") != last_seq:
                        last_seq = document.get("seq")
                        payload = json.dumps(document, ensure_ascii=False)
                        self.wfile.write(f"data: {payload}\n\n".encode("utf-8"))
                        self.wfile.flush()
                        last_keepalive = time.time()
                    elif time.time() - last_keepalive > _SSE_KEEPALIVE_SEC:
                        self.wfile.write(b": keepalive\n\n")
                        self.wfile.flush()
                        last_keepalive = time.time()
                    time.sleep(_SSE_POLL_SEC)
            except (BrokenPipeError, ConnectionResetError):
                return

    return Handler


def build_server(
    host: str = "127.0.0.1",
    port: int = 8080,
    status_file: str | Path = DEFAULT_STATUS_FILE,
    mock: bool = False,
    course_file: str | Path | None = None,
) -> ThreadingHTTPServer:
    state = DashboardState(status_file=status_file, mock=mock, course_file=course_file)
    server = ThreadingHTTPServer((host, port), make_handler(state))
    server.daemon_threads = True
    server.state = state  # type: ignore[attr-defined]
    return server


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="ICUAS 2027 race ground station")
    parser.add_argument("--host", default=os.environ.get("ICUAS27_GS_HOST", "127.0.0.1"))
    parser.add_argument(
        "--port", type=int, default=int(os.environ.get("ICUAS27_GS_PORT", "8080"))
    )
    parser.add_argument(
        "--status-file",
        default=os.environ.get("ICUAS27_STATUS_FILE", DEFAULT_STATUS_FILE),
        help="JSON contract written by the race node",
    )
    parser.add_argument(
        "--mock",
        action="store_true",
        help="replay a synthetic race instead of reading the status file",
    )
    parser.add_argument(
        "--course",
        default=os.environ.get("ICUAS27_COURSE", ""),
        help="course file for --mock (default: auto-discover)",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    course_file = args.course or None
    if args.mock and course_file is None:
        project_root = Path(__file__).resolve().parent.parent
        course_file = project_root / "course" / "provisional_sample.json"

    server = build_server(
        host=args.host,
        port=args.port,
        status_file=args.status_file,
        mock=args.mock,
        course_file=course_file,
    )
    mode = "MOCK replay" if args.mock else f"status file {args.status_file}"
    print(f"ICUAS 2027 ground station on http://{args.host}:{args.port}  ({mode})")
    print("Ctrl-C to stop.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nstopping")
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
