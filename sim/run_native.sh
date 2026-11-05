#!/usr/bin/env bash
# Reuse an existing Ubuntu 22.04/Humble LUS installation; no Docker bootstrap.
set -eo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
ENV_FILE=${ICUAS27_ENV_FILE:-$HOME/icuas_env.sh}
[[ -f "$ENV_FILE" ]] || { echo "Missing LUS environment: $ENV_FILE"; exit 1; }
source "$ENV_FILE"
[[ "${REAL_UAV:-}" == false ]] || { echo 'Native integration requires REAL_UAV=false.'; exit 1; }
export PYTHONPATH="$ROOT:${PYTHONPATH:-}"
export ICUAS27_PLATFORM=sim
export ICUAS27_LUS_BACKEND=${ICUAS27_LUS_BACKEND:-control_manager}
export ICUAS27_STATUS_FILE=${ICUAS27_STATUS_FILE:-/tmp/icuas27_race/status.json}
[[ ! -f "$ROOT/ros2_ws/install/local_setup.bash" ]] || source "$ROOT/ros2_ws/install/local_setup.bash"
cd "$ROOT"
case "${1:-check}" in
  build)
    cd ros2_ws
    colcon build --symlink-install --packages-up-to icuas27_racing --parallel-workers 2 --cmake-args -DBUILD_TESTING=OFF
    ;;
  check) bash tools/preflight.sh --in-container ;;
  test)
    python3 -m unittest discover -s tests -t .
    python3 -m pytest ros2_ws/src/icuas27_racing/test -q
    ;;
  race)
    exec ros2 launch icuas27_racing race.launch.py \
      course_file:="${ICUAS27_COURSE:-$ROOT/course/integration_smoke.json}" \
      lus_backend:="${ICUAS27_LUS_BACKEND:-control_manager}" \
      use_sim_time:=true auto_start:=false detector_backend:=grayscale \
      default_speed:=0.3 lead:=0.35
    ;;
  dashboard)
    exec python3 -m ground_station.server --host 127.0.0.1 --port 8080
    ;;
  *) echo 'Usage: sim/run_native.sh check|build|test|race|dashboard'; exit 2 ;;
esac
