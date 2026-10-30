"""Competition logic for the ICUAS 2027 autonomous drone racing entry.

Pure standard library, no ROS import, so the whole decision path can be tested
on a laptop and on the Windows side of the dual boot::

    python3 -m unittest discover -s tests -t .

Modules
-------
geometry
    Vectors, gate frames, segment/plane crossing.
course
    Course and gate data model, JSON loading, validation.
timing
    Elapsed time, penalty ledger, ``T_adj``.
mission_fsm
    Takeoff / race / landing state machine and waypoint issuing.
safety
    Stale-data budgets and the single flight verdict.
protocol
    Versioned JSON contract shared by the ROS node and the dashboard.
"""

from .course import Course, CourseError, Gate, build_course, load_course
from .geometry import CrossingResult, GateFrame, Vec3, crossing, gate_frame
from .mission_fsm import Action, ActionKind, MissionError, RaceMission, State
from .protocol import SCHEMA, build_document, read_document, validate_document, write_document
from .safety import Level, SafetyLimits, SafetyMonitor, Verdict
from .timing import PENALTY_SECONDS, Penalty, RaceTimer, format_duration

__version__ = "0.1.0"

__all__ = [
    "Action",
    "ActionKind",
    "Course",
    "CourseError",
    "CrossingResult",
    "Gate",
    "GateFrame",
    "Level",
    "MissionError",
    "PENALTY_SECONDS",
    "Penalty",
    "RaceMission",
    "RaceTimer",
    "SCHEMA",
    "SafetyLimits",
    "SafetyMonitor",
    "State",
    "Vec3",
    "Verdict",
    "build_course",
    "build_document",
    "crossing",
    "format_duration",
    "gate_frame",
    "load_course",
    "read_document",
    "validate_document",
    "write_document",
]
