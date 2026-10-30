"""Locating course files inside the installed package.

The node defaults to the course shipped with this project so a bare
``ros2 run icuas27_racing race_mission`` does something sensible. Resolution
order, first hit wins:

1. an explicit ``course_file`` parameter (absolute, or relative to cwd),
2. ``$ICUAS27_COURSE``,
3. the ``course/`` directory inside the source tree (works with
   ``--symlink-install``),
4. the installed ``share/icuas27_racing/course`` directory,
5. ``course/provisional_sample.json`` next to the working directory.

Pure Python: this module is unit tested without ROS.
"""

from __future__ import annotations

import os
from pathlib import Path

from racing_core.course import Course, CourseError, load_course

__all__ = [
    "COURSE_ENV",
    "DEFAULT_COURSE",
    "available_courses",
    "resolve_course",
    "load_default_course",
]

COURSE_ENV = "ICUAS27_COURSE"
DEFAULT_COURSE = "provisional_sample.json"


def _candidate_directories() -> list[Path]:
    """Directories that may hold course JSON files, most specific first.

    Walks up from this file instead of assuming a fixed depth, so the same code
    works from the source tree (``racing/ros2_ws/src/...``), from an installed
    package (``install/.../site-packages/...``) and from a symlinked checkout.
    """
    candidates: list[Path] = []
    here = Path(__file__).resolve()
    for parent in list(here.parents)[:6]:
        candidates.append(parent / "course")
        candidates.append(parent / "share" / "icuas27_racing" / "course")
    candidates.append(Path.cwd() / "course")
    candidates.append(Path.cwd())

    seen: set[Path] = set()
    unique: list[Path] = []
    for candidate in candidates:
        key = candidate.resolve() if candidate.exists() else candidate
        if key in seen:
            continue
        seen.add(key)
        unique.append(candidate)
    return unique


def available_courses() -> list[Path]:
    """Every ``*.json`` course file that can be found; sorted for stable output."""
    found: list[Path] = []
    seen: set[Path] = set()
    for directory in _candidate_directories():
        if not directory.is_dir():
            continue
        for path in sorted(directory.glob("*.json")):
            resolved = path.resolve()
            if resolved in seen:
                continue
            seen.add(resolved)
            found.append(path)
    return found


def resolve_course(course_file: str | None = None) -> Path:
    """Return the path of the course to load, or raise :class:`CourseError`."""
    explicit = (course_file or "").strip() or os.environ.get(COURSE_ENV, "").strip()
    if explicit:
        path = Path(explicit).expanduser()
        if not path.is_absolute():
            path = Path.cwd() / path
        if not path.is_file():
            raise CourseError(f"course_file does not exist: {path}")
        return path

    for directory in _candidate_directories():
        candidate = directory / DEFAULT_COURSE
        if candidate.is_file():
            return candidate

    available = available_courses()
    if available:
        return available[0]

    searched = ", ".join(str(directory) for directory in _candidate_directories())
    raise CourseError(f"no course file found; searched: {searched}")


def load_default_course(course_file: str | None = None) -> Course:
    """Resolve and load the course, translating errors into :class:`CourseError`."""
    return load_course(resolve_course(course_file))
