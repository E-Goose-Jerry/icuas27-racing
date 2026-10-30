#!/usr/bin/env bash
# Pre-flight check: everything that has to be true before arming.
#
# Adapted from a pattern used in the team's other drone stack: check the boring
# things in one place, print a clear verdict, exit non-zero so a script or a
# human stops here before arming.
#
#   ./tools/preflight.sh            # checks the host (Docker, GPU, course files)
#   ./tools/preflight.sh --in-container   # checks ROS topics inside the sim image
set -uo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IN_CONTAINER=0
[ "${1:-}" = "--in-container" ] && IN_CONTAINER=1

FAILS=0
WARNS=0

ok()   { printf '  \033[1;32m[ok]\033[0m   %s\n' "$*"; }
warn() { printf '  \033[1;33m[warn]\033[0m %s\n' "$*"; WARNS=$((WARNS + 1)); }
bad()  { printf '  \033[1;31m[fail]\033[0m %s\n' "$*"; FAILS=$((FAILS + 1)); }
head_() { printf '\n\033[1;34m%s\033[0m\n' "$*"; }

head_ "1. project files"
for path in racing_core/__init__.py course/provisional_sample.json sim/configs/spawn.yaml; do
    if [ -e "${PROJECT_ROOT}/${path}" ]; then ok "${path}"; else bad "${path} missing"; fi
done

head_ "2. course validity"
if python3 - "${PROJECT_ROOT}" <<'PY'
import sys
from pathlib import Path
root = Path(sys.argv[1])
sys.path.insert(0, str(root))
from racing_core.course import load_course
failures = []
for path in sorted((root / "course").glob("*.json")):
    try:
        course = load_course(path)
    except Exception as exc:  # noqa: BLE001
        failures.append(f"{path.name}: {exc}")
        print(f"  [fail] {path.name}: {exc}")
        continue
    print(f"  [ok]   {path.name}: {course.gate_count} gates x {course.laps} laps "
          f"= {len(course.sequence())} passes, lap ~{course.lap_length():.1f} m")
sys.exit(1 if failures else 0)
PY
then :; else bad "a course file is invalid"; fi

head_ "3. unit tests"
if ( cd "${PROJECT_ROOT}" && python3 -m unittest discover -s tests -t . >/tmp/icuas27_tests.log 2>&1 ); then
    ok "$(grep -o 'Ran [0-9]* tests' /tmp/icuas27_tests.log | tail -1) passed"
else
    bad "unit tests failed (see /tmp/icuas27_tests.log)"
fi

head_ "4. status sink"
STATUS_DIR="$(dirname "${ICUAS27_STATUS_FILE:-/tmp/icuas27_race/status.json}")"
if mkdir -p "${STATUS_DIR}" 2>/dev/null && [ -w "${STATUS_DIR}" ]; then
    ok "${STATUS_DIR} writable"
else
    bad "${STATUS_DIR} is not writable"
fi

if [ "${IN_CONTAINER}" -eq 0 ]; then
    head_ "5. docker"
    if command -v docker >/dev/null 2>&1; then
        ok "docker present"
        if docker info >/dev/null 2>&1; then ok "daemon reachable"; else bad "daemon not reachable (user in docker group?)"; fi
        if docker run --rm --gpus all ubuntu:22.04 true >/dev/null 2>&1; then
            ok "GPU passthrough"
        else
            warn "no GPU passthrough: Gazebo will be slow"
        fi
    else
        bad "docker is not installed"
    fi

    head_ "6. display"
    if [ -n "${DISPLAY:-}" ] && [ -S /tmp/.X11-unix/X0 ]; then
        ok "X11 ${DISPLAY}"
    else
        warn "no X11 socket: Gazebo will run headless"
    fi
else
    head_ "5. ros runtime"
    if command -v ros2 >/dev/null 2>&1; then
        ok "ros2 on PATH"
        if ros2 pkg prefix laser_uav_managers >/dev/null 2>&1; then
            ok "laser_uav_managers found"
        else
            bad "laser_uav_managers not found (source /opt/laser_uav_system_ws/install/setup.bash)"
        fi
        if ros2 pkg prefix icuas27_racing >/dev/null 2>&1; then
            ok "icuas27_racing built"
        else
            bad "icuas27_racing not built (./sim/run_sim.sh build-race)"
        fi
        if ros2 topic list 2>/dev/null | grep -q "planning_manager"; then
            ok "planning_manager topics visible"
        else
            warn "no planning_manager topics: is the stack running?"
        fi
    else
        bad "ros2 not on PATH"
    fi
fi

head_ "verdict"
if [ "${FAILS}" -gt 0 ]; then
    printf '\033[1;31m  %d failure(s), %d warning(s) — do not arm.\033[0m\n' "${FAILS}" "${WARNS}"
    exit 1
elif [ "${WARNS}" -gt 0 ]; then
    printf '\033[1;33m  0 failures, %d warning(s) — usable, read the warnings.\033[0m\n' "${WARNS}"
    exit 0
else
    printf '\033[1;32m  all clear.\033[0m\n'
    exit 0
fi
